
/* demo: end-to-end preheat validation.
 * For one block it (1) times a cold query, (2) runs the full preheat
 * (all ordered s!=g pairs, one A* each) and times the wall clock,
 * (3) times a warm table lookup, then compares measured preheat time
 * against the cost-model prediction n(n-1)*t_q measured on this run.
 *
 * Usage: ./demo <W> <H> <open|choke> <query_pairs>
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

typedef struct { int w, h; unsigned char* blocked; } Map;
static int idx(const Map* m, int x, int y) { return y * m->w + x; }
static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= m->w || y >= m->h) return 0;
    return !m->blocked[idx(m, x, y)];
}
static unsigned long rng_state;
static unsigned long xr(void) {
    unsigned long x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x; return x;
}

static int astar_len(const Map* m, int sx, int sy, int gx, int gy) {
    int n = m->w * m->h;
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int* g = malloc(sizeof(int) * (size_t)n);
    int* f = malloc(sizeof(int) * (size_t)n);
    int* closed = malloc(sizeof(int) * (size_t)n);
    int* parent = malloc(sizeof(int) * (size_t)n);
    int* open = malloc(sizeof(int) * (size_t)n);
    if (!g || !f || !closed || !parent || !open) { fprintf(stderr, "oom\n"); exit(2); }
    for (int i = 0; i < n; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int nopen = 0, s = idx(m, sx, sy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    int found = -1;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == idx(m, gx, gy)) { found = cur; break; }
        closed[cur] = 1;
        int cx = cur % m->w, cy = cur / m->w;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(m, nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;
                int inopen = 0;
                for (int i = 0; i < nopen; i++) if (open[i] == ni) { inopen = 1; break; }
                if (!inopen) open[nopen++] = ni;
            }
        }
    }
    int len = -1;
    if (found >= 0) { len = 0; for (int c = found; c != -1; c = parent[c]) len++; }
    free(g); free(f); free(closed); free(parent); free(open);
    return len;
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

int main(int argc, char** argv) {
    if (argc != 5) { fprintf(stderr, "usage: %s W H open|choke query_pairs\n", argv[0]); return 2; }
    int w = atoi(argv[1]), h = atoi(argv[2]);
    int choke = !strcmp(argv[3], "choke");
    if (!choke && strcmp(argv[3], "open")) { fprintf(stderr, "bad profile\n"); return 2; }
    int qpairs = atoi(argv[4]);
    int n = w * h;

    Map m; m.w = w; m.h = h;
    m.blocked = calloc((size_t)n, 1);
    rng_state = 0xdeadbeefUL ^ ((unsigned long)n << 20) ^ (choke ? 0x55aaUL : 0);
    if (choke) {
        int stride = h >= 96 ? 8 : 6, wall = 0;
        for (int y = stride; y < h - 1; y += stride) {
            int gapx = (wall % 2 == 0) ? (w / 5) : (w - 1 - w / 5);
            for (int x = 0; x < w; x++) if (x != gapx) m.blocked[idx(&m, x, y)] = 1;
            wall++;
        }
    } else {
        int nblock = (int)(n * 0.12);
        for (int i = 0; i < nblock; i++)
            m.blocked[idx(&m, (int)(xr() % (unsigned)w), (int)(xr() % (unsigned)h))] = 1;
    }

    /* 1. cold latency over qpairs reachable queries */
    double cold_sum = 0; int got = 0, guard = 0;
    while (got < qpairs && guard < qpairs * 30) {
        guard++;
        int sx = (int)(xr() % (unsigned)w), sy = (int)(xr() % (unsigned)h);
        int gx = (int)(xr() % (unsigned)w), gy = (int)(xr() % (unsigned)h);
        if (sx == gx && sy == gy) continue;
        double t0 = now_ms();
        int len = astar_len(&m, sx, sy, gx, gy);
        double t1 = now_ms();
        if (len < 0) continue;
        cold_sum += t1 - t0; got++;
    }
    double t_q = cold_sum / got;

    /* 2. full preheat: every ordered s!=g pair; cache stores path length */
    int* cache = malloc(sizeof(int) * (size_t)n * (size_t)n);
    if (!cache) { fprintf(stderr, "cache oom\n"); return 2; }
    memset(cache, -1, sizeof(int) * (size_t)n * (size_t)n);
    double p0 = now_ms();
    long nqueries = 0;
    for (int s = 0; s < n; s++) {
        int sx = s % w, sy = s / w;
        if (!walkable(&m, sx, sy)) continue;
        for (int g = 0; g < n; g++) {
            if (s == g) continue;
            int gx = g % w, gy = g / w;
            if (!walkable(&m, gx, gy)) continue;
            cache[(long)s * n + g] = astar_len(&m, sx, sy, gx, gy);
            nqueries++;
        }
    }
    double preheat_ms = now_ms() - p0;
    double model_ms = (double)nqueries * t_q;

    /* 3. warm lookup latency */
    double w0 = now_ms();
    long sink = 0;
    for (int k = 0; k < 100000; k++) {
        int s = (int)(xr() % (unsigned)n), g = (int)(xr() % (unsigned)n);
        sink += cache[(long)s * n + g];
    }
    double t_hit = (now_ms() - w0) / 100000.0;
    (void)sink;

    printf("W=%d H=%d n=%d profile=%s\n", w, h, n, choke ? "choke" : "open");
    printf("pairs_timed=%d cold_tq_ms=%.6f preheat_queries=%ld\n", got, t_q, nqueries);
    printf("preheat_measured_ms=%.3f preheat_model_ms=%.3f model_over_measured=%.3f\n",
           preheat_ms, model_ms, model_ms / preheat_ms);
    printf("warm_thit_ms=%.6f speedup_cold_over_warm=%.1fx\n",
           t_hit, t_q / t_hit);
    free(cache);
    free(m.blocked);
    return 0;
}
