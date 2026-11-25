/* cache: the release-timing trade-off, made measurable.
 *
 * A nav path cache holds resolved paths keyed by (start,goal). The question is
 * when to drop them: keep longer -> higher hit rate but more resident bytes;
 * drop promptly -> lower memory but more recomputation. This sweeps the two
 * knobs that decide release timing -- capacity and idle-TTL -- over a replay of
 * a path workload, and reports, per setting:
 *
 *     peak_bytes   high-water mark of cache payload
 *     hit_rate     queries served without recomputation
 *     recomputes   queries that had to re-run the search
 *
 * The replay is deterministic (fixed seed) so settings are comparable.
 *
 * build: cc -O2 -std=c11 -Ibench cache.c bench/allocwatch.c -o bench/cache -lm
 * run:   bench/cache
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "allocwatch.h"

#define W 16
#define H 12
#define MAXN (W * H)

typedef struct { unsigned char blocked[W * H]; } Map;

/* One cached entry: the path itself. Byte size is what the cache costs. */
typedef struct Entry {
    int key;            /* (sx,sy,gx,gy) packed */
    int touch;          /* last round used, for TTL / LRU */
    int n;              /* waypoint count */
    int pts[MAXN];      /* waypoints */
    struct Entry* next;
} Entry;

static Map g_map;

/* --- the real work: A* on the 16x12 grid, same algorithm as grid/ --- */
static int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
static int walkable(int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !g_map.blocked[y * W + x];
}
static int astar(int sx, int sy, int gx, int gy, int* out) {
    if (!walkable(sx, sy) || !walkable(gx, gy)) return -1;
    for (int i = 0; i < MAXN; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int open[MAXN], nopen = 0;
    int s = sy * W + sx;
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == gy * W + gx) {
            int n = 0, c = cur, rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(nx, ny)) continue;
            int ni = ny * W + nx;
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < nopen; i++) if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return -1;
}

/* --- cache policy under test --- */
typedef struct {
    Entry* head;
    int count;
    int cap;        /* max entries, 0 = unbounded */
    int ttl;        /* drop entries untouched for this many rounds, 0 = never */
} Cache;

static int cache_get(Cache* c, int key, int round, int* out) {
    for (Entry* e = c->head; e; e = e->next) {
        if (e->key == key) { e->touch = round; memcpy(out, e->pts, (size_t)e->n * sizeof(int)); return e->n; }
    }
    return -1;
}

static void cache_evict(Cache* c, int round) {
    /* TTL first */
    if (c->ttl > 0) {
        Entry** pp = &c->head;
        while (*pp) {
            Entry* e = *pp;
            if (round - e->touch > c->ttl) { *pp = e->next; free(e); c->count--; }
            else pp = &e->next;
        }
    }
    /* then LRU down to capacity */
    while (c->cap > 0 && c->count >= c->cap) {
        Entry** victim = NULL;
        Entry** pp = &c->head;
        int oldest = 1 << 29;
        while (*pp) {
            if ((*pp)->touch < oldest) { oldest = (*pp)->touch; victim = pp; }
            pp = &(*pp)->next;
        }
        if (!victim) break;
        Entry* e = *victim;
        *victim = e->next;
        free(e);
        c->count--;
    }
}

static void cache_put(Cache* c, int key, const int* pts, int n, int round) {
    Entry* e = malloc(sizeof(Entry));
    e->key = key; e->touch = round; e->n = n;
    memcpy(e->pts, pts, (size_t)n * sizeof(int));
    e->next = c->head;
    c->head = e;
    c->count++;
    cache_evict(c, round);
}

static void cache_free(Cache* c) {
    while (c->head) { Entry* e = c->head; c->head = e->next; free(e); }
    c->count = 0;
}

/* --- deterministic replay ---
 * A session has a finite set of "places" it keeps navigating between, drawn
 * with a skewed (hot/cold) distribution. Reuse is what makes a cache worth
 * anything, and how far apart the reuses are is what a TTL has to cover. */
static unsigned rng_state;
static unsigned rng(void) { rng_state = rng_state * 1103515245u + 12345u; return (rng_state >> 8) & 0xFFFFFF; }

#define HOT_PLACES 48
#define COLD_PLACES 320
static int hot_x[HOT_PLACES], hot_y[HOT_PLACES];
static int cold_x[COLD_PLACES], cold_y[COLD_PLACES];

static void build_places(void) {
    for (int i = 0; i < HOT_PLACES; i++) { hot_x[i] = (int)(rng() % W); hot_y[i] = (int)(rng() % H); }
    for (int i = 0; i < COLD_PLACES; i++) { cold_x[i] = (int)(rng() % W); cold_y[i] = (int)(rng() % H); }
}

static void pick_place(int* x, int* y, int hot_bias) {
    /* hot_bias: percent of queries drawn from the hot set */
    if ((int)(rng() % 100) < hot_bias) {
        int i = (int)(rng() % HOT_PLACES);
        *x = hot_x[i]; *y = hot_y[i];
    } else {
        int i = (int)(rng() % COLD_PLACES);
        *x = cold_x[i]; *y = cold_y[i];
    }
}

static void next_query(int* sx, int* sy, int* gx, int* gy, int hot_bias) {
    pick_place(sx, sy, hot_bias);
    pick_place(gx, gy, hot_bias);
    if (*sx == *gx && *sy == *gy) *gx = (*gx + 1) % W;
}

static int run_session(int rounds, int queries_per_round, Cache* c, int hot_bias,
                       int* hits, int* misses) {
    int path[MAXN], out[MAXN];
    *hits = *misses = 0;
    alloc_reset_peak();
    for (int r = 1; r <= rounds; r++) {
        for (int q = 0; q < queries_per_round; q++) {
            int sx, sy, gx, gy;
            next_query(&sx, &sy, &gx, &gy, hot_bias);
            int key = (sx << 24) | (sy << 16) | (gx << 8) | gy;
            if (cache_get(c, key, r, out) >= 0) { (*hits)++; continue; }
            (*misses)++;
            int n = astar(sx, sy, gx, gy, path);
            if (n > 0) cache_put(c, key, path, n, r);
        }
    }
    return (int)alloc_peak_bytes();
}

int main(void) {
    memset(&g_map, 0, sizeof g_map);
    /* a few walls, so paths are non-trivial and cache keys vary in value */
    for (int y = 0; y < 8; y++) g_map.blocked[y * W + 5] = 1;
    for (int x = 9; x < 15; x++) g_map.blocked[9 * W + x] = 1;

    const int rounds = 10, qpr = 200;
    struct { const char* name; int cap; int ttl; } policies[] = {
        {"unbounded ttl=0", 0, 0},
        {"cap2048 ttl0",  2048, 0},
        {"cap512 ttl0",   512, 0},
        {"cap128 ttl0",   128, 0},
        {"unbounded ttl1", 0, 1},
        {"unbounded ttl3", 0, 3},
        {"cap512 ttl3",   512, 3},
        {"cap128 ttl1",   128, 1},
    };
    const int n_pol = (int)(sizeof policies / sizeof policies[0]);

    build_places();
    printf("hot_bias,policy,cap,ttl,peak_bytes,hits,misses,queries,hit_rate,"
           "bytes_per_hit\n");
    const int biases[] = {90, 60};
    for (int bi = 0; bi < 2; bi++) {
        int loc = biases[bi];
        for (int i = 0; i < n_pol; i++) {
            Cache c = {0};
            c.cap = policies[i].cap;
            c.ttl = policies[i].ttl;
            rng_state = 12345u;            /* same workload for every policy */
            int hits = 0, misses = 0;
            int peak = run_session(rounds, qpr, &c, loc, &hits, &misses);
            int q = hits + misses;
            printf("%d,%s,%d,%d,%d,%d,%d,%d,%.4f,%.1f\n",
                   loc, policies[i].name, c.cap, c.ttl, peak, hits, misses, q,
                   q ? (double)hits / q : 0.0,
                   // bytes of peak per hit: the memory cost of each served query
                   hits ? (double)peak / hits : 0.0);
            cache_free(&c);
        }
    }
    return 0;
}
