/* bench: measure real A* latency and cooldown cache-hit latency on 16x12 maps.
 * Reuses the 4-neighbour A* from grid/src/grid.c verbatim in spirit.
 * The three request kinds are measured in random interleaved order so that
 * CPU warmup / branch-predictor state cannot bias one distribution.
 * Outputs one integer nanosecond per line, one CSV file per kind. */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define W 16
#define H 12
#define MAXN (W * H)
#define NMAPS 8
#define NREQ 4000                 /* requests per kind (full / miss / hit) */
#define HSHIFT 16
#define HSIZE (1 << HSHIFT)

typedef struct { unsigned char blocked[W * H]; int id; } Map;

static uint64_t rng_state[4];

static uint64_t splitmix64(uint64_t x) {
    x += 0x9e3779b97f4a7c15ULL;
    x = (x ^ (x >> 30)) * 0xbf58476d1ce4e5b9ULL;
    x = (x ^ (x >> 27)) * 0x94d049bb133111ebULL;
    return x ^ (x >> 31);
}

static uint64_t rotl(const uint64_t x, int k) { return (x << k) | (x >> (64 - k)); }

static uint64_t xoshiro(void) {
    const uint64_t res = rotl(rng_state[1] * 5, 7) * 9;
    const uint64_t t = rng_state[1] << 17;
    rng_state[2] ^= rng_state[0];
    rng_state[3] ^= rng_state[1];
    rng_state[1] ^= rng_state[2];
    rng_state[0] ^= rng_state[3];
    rng_state[2] ^= t;
    rng_state[3] = rotl(rng_state[3], 45);
    return res;
}

static double rng_uniform(void) { return (xoshiro() >> 11) * 0x1.0p-53; }

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* same A* as grid/src/grid.c; returns path length or -1 */
static int astar(const Map *m, int sx, int sy, int gx, int gy, int *out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == idx(gx, gy)) {
            int n = 0, c = cur;
            int rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
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

/* open-addressing cache: what a cooldown hit actually costs at runtime */
static uint64_t hkey[HSIZE];
static int hval[HSIZE];
static int hused[HSIZE];

static uint64_t make_key(int mapid, int sx, int sy, int gx, int gy) {
    return ((uint64_t)(mapid + 1) << 24) | ((uint64_t)sx << 18) |
           ((uint64_t)sy << 12) | ((uint64_t)gx << 6) | (uint64_t)gy;
}

static uint64_t hash64(uint64_t x) { return splitmix64(x); }

static void cache_insert(uint64_t key, int val) {
    uint32_t p = (uint32_t)(hash64(key) & (HSIZE - 1));
    while (hused[p]) { if (hkey[p] == key) { hval[p] = val; return; } p = (p + 1) & (HSIZE - 1); }
    hused[p] = 1; hkey[p] = key; hval[p] = val;
}

static int cache_lookup(uint64_t key) {
    uint32_t p = (uint32_t)(hash64(key) & (HSIZE - 1));
    while (hused[p]) { if (hkey[p] == key) return hval[p]; p = (p + 1) & (HSIZE - 1); }
    return -1; /* miss */
}

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ULL + (uint64_t)ts.tv_nsec;
}

static int cmp64(const void *a, const void *b) {
    uint64_t x = *(const uint64_t *)a, y = *(const uint64_t *)b;
    return (x > y) - (x < y);
}

enum kind { KIND_FULL = 0, KIND_MISS = 1, KIND_HIT = 2, NKIND = 3 };

typedef struct {
    int kind;
    int mapid, sx, sy, gx, gy;
    uint64_t key;
} Job;

int main(int argc, char **argv) {
    /* optional args: round id (selects seed + output dir), so independent
       measurement rounds can be pooled for stable heavy-tail moments */
    int round_id = 0;
    if (argc > 1) round_id = atoi(argv[1]);
    uint64_t seed = 20261126ULL + (uint64_t)round_id * 1000003ULL;
    for (int i = 0; i < 4; i++) rng_state[i] = splitmix64(seed + (uint64_t)i * 0x9e3779b97f4a7c15ULL);

    Map maps[NMAPS];
    for (int k = 0; k < NMAPS; k++) {
        memset(&maps[k], 0, sizeof(Map));
        maps[k].id = k;
        for (int y = 1; y < H - 1; y++)
            for (int x = 1; x < W - 1; x++)
                if (rng_uniform() < 0.12) maps[k].blocked[idx(x, y)] = 1;
    }

    /* clock_gettime measurement overhead (median of back-to-back deltas) */
    uint64_t overhead[10000];
    for (int i = 0; i < 10000; i++) { uint64_t a = now_ns(), b = now_ns(); overhead[i] = b - a; }
    qsort(overhead, 10000, sizeof(uint64_t), cmp64);
    uint64_t h = overhead[5000];

    int njobs = NREQ * NKIND;
    Job *jobs = malloc(njobs * sizeof(Job));
    int path[MAXN];
    int checksum = 0, j = 0;

    /* full jobs: bare timed A* (control group request) */
    for (int i = 0; i < NREQ; i++) {
        Map *m = &maps[(int)(xoshiro() % NMAPS)];
        int sx, sy, gx, gy;
        do { sx = (int)(xoshiro() % W); sy = (int)(xoshiro() % H); } while (!walkable(m, sx, sy));
        do { gx = (int)(xoshiro() % W); gy = (int)(xoshiro() % H); } while (!walkable(m, gx, gy) || (gx == sx && gy == sy));
        jobs[j++] = (Job){KIND_FULL, m->id, sx, sy, gx, gy, make_key(m->id, sx, sy, gx, gy)};
    }
    /* miss jobs: map id NMAPS is never inserted elsewhere; fold i into the
       key so every miss key is globally unique -> lookup always misses */
    for (int i = 0; i < NREQ; i++) {
        Map *m = &maps[(int)(xoshiro() % NMAPS)];
        int sx, sy, gx, gy;
        do { sx = (int)(xoshiro() % W); sy = (int)(xoshiro() % H); } while (!walkable(m, sx, sy));
        do { gx = (int)(xoshiro() % W); gy = (int)(xoshiro() % H); } while (!walkable(m, gx, gy) || (gx == sx && gy == sy));
        uint64_t key = make_key(NMAPS, i & 0xF, (i >> 4) & 0xF, (i >> 8) & 0xF, (i >> 12) & 0xF);
        jobs[j++] = (Job){KIND_MISS, m->id, sx, sy, gx, gy, key};
    }
    /* hit jobs: keys are inserted untimed first; timed part is lookup only */
    for (int i = 0; i < NREQ; i++) {
        Map *m = &maps[(int)(xoshiro() % NMAPS)];
        int sx, sy, gx, gy;
        do { sx = (int)(xoshiro() % W); sy = (int)(xoshiro() % H); } while (!walkable(m, sx, sy));
        do { gx = (int)(xoshiro() % W); gy = (int)(xoshiro() % H); } while (!walkable(m, gx, gy) || (gx == sx && gy == sy));
        uint64_t key = make_key(m->id, sx, sy, gx, gy);
        int n = astar(m, sx, sy, gx, gy, path, MAXN);
        cache_insert(key, n);
        checksum += n;
        jobs[j++] = (Job){KIND_HIT, m->id, sx, sy, gx, gy, key};
    }

    /* Fisher-Yates shuffle -> random interleaving of the three kinds */
    for (int i = njobs - 1; i > 0; i--) {
        int k = (int)(xoshiro() % (uint64_t)(i + 1));
        Job tmp = jobs[i]; jobs[i] = jobs[k]; jobs[k] = tmp;
    }

    uint64_t *times[NKIND];
    int counts[NKIND] = {0, 0, 0};
    for (int k = 0; k < NKIND; k++) times[k] = malloc(NREQ * sizeof(uint64_t));

    for (int i = 0; i < njobs; i++) {
        Job *jb = &jobs[i];
        Map *m = &maps[jb->mapid];
        uint64_t a = now_ns();
        int v = -1, n = 0;
        switch (jb->kind) {
        case KIND_FULL:
            n = astar(m, jb->sx, jb->sy, jb->gx, jb->gy, path, MAXN);
            break;
        case KIND_MISS:
            v = cache_lookup(jb->key);
            n = astar(m, jb->sx, jb->sy, jb->gx, jb->gy, path, MAXN);
            cache_insert(jb->key, n);
            break;
        case KIND_HIT:
            v = cache_lookup(jb->key);
            n = v;
            break;
        }
        uint64_t b = now_ns();
        uint64_t dt = (b - a > h) ? b - a - h : 1;
        times[jb->kind][counts[jb->kind]++] = dt;
        checksum += n + (v < 0 ? 0 : v) + path[0];
    }

    const char *names[NKIND] = {"full", "miss", "cache"};
    const char *outdir = argc > 2 ? argv[2] : "results";
    for (int k = 0; k < NKIND; k++) {
        char pathbuf[256];
        snprintf(pathbuf, sizeof(pathbuf), "%s/times_%s.csv", outdir, names[k]);
        FILE *fh = fopen(pathbuf, "w");
        if (!fh) { perror("open results"); return 2; }
        for (int i = 0; i < NREQ; i++) fprintf(fh, "%llu\n", (unsigned long long)times[k][i]);
        fclose(fh);
        free(times[k]);
    }

    printf("round=%d overhead_ns=%llu samples_per_kind=%d checksum=%d\n",
           round_id, (unsigned long long)h, NREQ, checksum);
    free(jobs);
    return 0;
}
