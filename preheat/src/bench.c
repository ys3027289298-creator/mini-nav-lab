
/* bench: cold single-query A* timing at multiple grid sizes.
 * Same algorithm as grid/src/grid.c: 4-neighbour, manhattan heuristic,
 * linear-scan open list (O(n) per pop, O(n^2) worst case).
 *
 * Two map profiles:
 *   open  - isolated obstacle cells, high connectivity (typical region)
 *   choke - staggered walls with one-cell gaps, path must snake and A*
 *           explores large dead-end areas (worst-case-shaped region)
 *
 * Usage:
 *   ./bench open   - cold A* latency, open terrain
 *   ./bench choke  - cold A* latency, choke terrain
 *   ./bench hit    - warm cache-hit latency (table keyed by s*n+g)
 * CSV stdout: profile,n,W,H,K,mean_ms,median_ms,p95_ms,max_ms
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

typedef struct {
    int w, h;
    unsigned char* blocked;
} Map;

static int idx(const Map* m, int x, int y) { return y * m->w + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= m->w || y >= m->h) return 0;
    return !m->blocked[idx(m, x, y)];
}

static unsigned long rng_state;
static unsigned long xr(void) {
    unsigned long x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x;
    return x;
}

/* returns path length or -1; identical search structure to grid.c */
static int astar(const Map* m, int sx, int sy, int gx, int gy) {
    int n = m->w * m->h;
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int* g = malloc(sizeof(int) * (size_t)n);
    int* f = malloc(sizeof(int) * (size_t)n);
    int* closed = malloc(sizeof(int) * (size_t)n);
    int* parent = malloc(sizeof(int) * (size_t)n);
    int* open = malloc(sizeof(int) * (size_t)n);
    if (!g || !f || !closed || !parent || !open) { fprintf(stderr, "oom\n"); exit(2); }
    for (int i = 0; i < n; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int nopen = 0;
    int s = idx(m, sx, sy);
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
    if (found >= 0) {
        len = 0;
        for (int c = found; c != -1; c = parent[c]) len++;
    }
    free(g); free(f); free(closed); free(parent); free(open);
    return len;
}

static int cmp_double(const void* a, const void* b) {
    double da = *(const double*)a, db = *(const double*)b;
    return (da > db) - (da < db);
}

static double now_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec * 1000.0 + ts.tv_nsec / 1e6;
}

static void build_open(Map* m, int n) {
    int nblock = (int)(n * 0.12);
    for (int i = 0; i < nblock; i++) {
        int x = (int)(xr() % (unsigned)m->w);
        int y = (int)(xr() % (unsigned)m->h);
        m->blocked[idx(m, x, y)] = 1;
    }
}

/* horizontal walls every `stride` rows, each with a single gap whose
 * x position alternates sides; forces long snake paths and wide search */
static void build_choke(Map* m) {
    int stride = m->h >= 96 ? 8 : 6;
    int wall = 0;
    for (int y = stride; y < m->h - 1; y += stride) {
        int gapx = (wall % 2 == 0) ? (m->w / 5) : (m->w - 1 - m->w / 5);
        for (int x = 0; x < m->w; x++) {
            if (x != gapx) m->blocked[idx(m, x, y)] = 1;
        }
        wall++;
    }
}

/* warm path: emulate the preheat cache as a dense n*n table of path
 * lengths and time a keyed lookup; measures t_hit, the post-preheat cost */
static void run_hit(const int sizes[][3], int nsizes) {
    printf("profile,n,W,H,K,mean_ms,median_ms,p95_ms,max_ms\n");
    for (int z = 0; z < nsizes; z++) {
        int w = sizes[z][0], h = sizes[z][1], K = sizes[z][2];
        int n = w * h;
        int* cache = malloc(sizeof(int) * (size_t)n * (size_t)n);
        if (!cache) { fprintf(stderr, "oom at n=%d\n", n); return; }
        for (long i = 0; i < (long)n * n; i++) cache[i] = (int)(i % 97);
        rng_state = 0x1234567UL ^ ((unsigned long)n << 16);
        double* samples = malloc(sizeof(double) * (size_t)K);
        long sink = 0;
        for (int k = 0; k < K; k++) {
            int s = (int)(xr() % (unsigned)n);
            int g = (int)(xr() % (unsigned)n);
            double t0 = now_ms();
            sink += cache[(long)s * n + g];
            double t1 = now_ms();
            samples[k] = t1 - t0;
        }
        qsort(samples, (size_t)K, sizeof(double), cmp_double);
        double sum = 0, mx = samples[K - 1];
        for (int i = 0; i < K; i++) sum += samples[i];
        printf("hit,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
               n, w, h, K, sum / K, samples[K / 2],
               samples[(int)(0.95 * (K - 1))], mx);
        (void)sink;
        free(samples);
        free(cache);
    }
}

int main(int argc, char** argv) {
    if (argc != 2) { fprintf(stderr, "usage: %s open|choke|hit\n", argv[0]); return 2; }
    const char* profile = argv[1];

    const int sizes[][3] = {
        {16,  12,  400},
        {32,  24,  400},
        {48,  36,  300},
        {64,  48,  250},
        {96,  72,  150},
        {128, 96,  100},
        {192, 144, 60},
        {256, 192, 40},
    };
    const int nsizes = (int)(sizeof(sizes) / sizeof(sizes[0]));

    if (!strcmp(profile, "hit")) { run_hit(sizes, nsizes); return 0; }
    int choke = !strcmp(profile, "choke");
    if (!choke && strcmp(profile, "open")) { fprintf(stderr, "bad profile\n"); return 2; }

    printf("profile,n,W,H,K,mean_ms,median_ms,p95_ms,max_ms\n");
    for (int z = 0; z < nsizes; z++) {
        int w = sizes[z][0], h = sizes[z][1], K = sizes[z][2];
        int n = w * h;
        Map m; m.w = w; m.h = h;
        m.blocked = calloc((size_t)n, 1);
        rng_state = 0x9e3779b97f4a7c15UL ^ ((unsigned long)n << 32) ^ (choke ? 0xabcdefUL : 0);
        if (choke) build_choke(&m); else build_open(&m, n);

        double* samples = malloc(sizeof(double) * (size_t)K);
        int got = 0, guard = 0;
        while (got < K && guard < K * 30) {
            guard++;
            int sx, sy, gx, gy;
            if (choke) {
                /* traverse every wall: bottom band -> top band */
                sx = (int)(xr() % (unsigned)w); sy = h - 1 - (int)(xr() % 3);
                gx = (int)(xr() % (unsigned)w); gy = (int)(xr() % 2);
            } else {
                sx = (int)(xr() % (unsigned)w); sy = (int)(xr() % (unsigned)h);
                gx = (int)(xr() % (unsigned)w); gy = (int)(xr() % (unsigned)h);
            }
            if (sx == gx && sy == gy) continue;
            double t0 = now_ms();
            int len = astar(&m, sx, sy, gx, gy);
            double t1 = now_ms();
            if (len < 0) continue;
            samples[got++] = t1 - t0;
        }
        if (got < K) { fprintf(stderr, "profile %s n=%d: only %d reachable pairs\n", profile, n, got); }
        qsort(samples, (size_t)got, sizeof(double), cmp_double);
        double sum = 0, mx = samples[got - 1];
        for (int i = 0; i < got; i++) sum += samples[i];
        printf("%s,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
               profile, n, w, h, got, sum / got, samples[got / 2],
               samples[(int)(0.95 * (got - 1))], mx);
        free(samples);
        free(m.blocked);
    }
    return 0;
}
