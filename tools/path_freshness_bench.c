/*
 * path_freshness_bench: cost of a last-frame feasibility recheck vs A*.
 *
 * The recheck is deliberately not another A*: it validates the stored path
 * against the current blocking map in O(path length).  It also checks LOS for
 * callers that consume a smoothed path.
 *
 * Output is key=value so tools/path_freshness_lab.py can consume it.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W * H)

typedef struct { unsigned char blocked[W * H]; } Map;

#if defined(__GNUC__) || defined(__clang__)
#define NOINLINE __attribute__((noinline))
#else
#define NOINLINE
#endif

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* Same 4-neighbour A* shape as grid/src/grid.c. */
static NOINLINE int astar(const Map *m, int sx, int sy, int gx, int gy,
                 int *out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;

    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29;
        f[i] = 1 << 29;
        closed[i] = 0;
        parent[i] = -1;
    }

    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0;
    f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;

    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) {
            if (f[open[i]] < f[open[best]]) best = i;
        }
        int cur = open[best];
        open[best] = open[--nopen];

        if (cur == idx(gx, gy)) {
            int n = 0, c = cur;
            int rev[MAXN];
            while (c != -1 && n < MAXN) {
                rev[n++] = c;
                c = parent[c];
            }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }

        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

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
                for (int i = 0; i < nopen; i++) {
                    if (open[i] == ni) { found = 1; break; }
                }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return -1;
}

/* O(n) final gate for a grid-cell path against the latest map. */
static NOINLINE int path_still_feasible(const Map *m, const int *path, int n) {
    if (n <= 0) return 0;

    for (int i = 0; i < n; i++) {
        int p = path[i];
        if (p < 0 || p >= W * H || m->blocked[p]) return 0;

        if (i > 0) {
            int px = path[i - 1] % W, py = path[i - 1] / W;
            int x = p % W, y = p / W;
            if (abs(x - px) + abs(y - py) != 1) return 0;
        }
    }
    return 1;
}

/* Supercover LOS, matching smooth/src/smooth.c. */
static NOINLINE int los(const Map *m, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;

    for (;;) {
        if (x < 0 || y < 0 || x >= W || y >= H) return 0;
        if (m->blocked[idx(x, y)]) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
}

/* Conservative validation for every segment of a smoothed path. */
static NOINLINE int smoothed_path_still_clear(const Map *m, const int *path, int n) {
    if (n <= 0) return 0;
    for (int i = 0; i + 1 < n; i++) {
        if (!los(m, path[i] % W, path[i] / W,
                 path[i + 1] % W, path[i + 1] / W)) {
            return 0;
        }
    }
    return path_still_feasible(m, path, n);
}

static double seconds_now(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + (double)ts.tv_nsec * 1e-9;
}

int main(void) {
    Map m;
    memset(&m, 0, sizeof(m));

    /* Partial wall: long enough to make A* do real work, with a gap. */
    for (int y = 0; y < H - 2; y++) m.blocked[idx(7, y)] = 1;

    int path[MAXN];
    int n = astar(&m, 0, 5, W - 1, 6, path, MAXN);
    if (n <= 0 || !path_still_feasible(&m, path, n)) {
        fprintf(stderr, "baseline path failed\n");
        return 1;
    }

    Map mutated = m;
    int middle = path[n / 2];
    mutated.blocked[middle] = 1;
    int detects_wall = !path_still_feasible(&mutated, path, n);
    int detects_smoothed_wall = !smoothed_path_still_clear(&mutated, path, n);
    if (!detects_wall || !detects_smoothed_wall) {
        fprintf(stderr, "freshness gate failed functional test\n");
        return 1;
    }

    const int PLAN_ITERS = 200000;
    const int CHECK_ITERS = 2000000;
    volatile long sink = 0;

    double t0 = seconds_now();
    for (int i = 0; i < PLAN_ITERS; i++) {
        int p[MAXN];
        int got = astar(&m, 0, 5, W - 1, 6, p, MAXN);
        sink += got > 0 ? p[got / 2] : -1;
    }
    double plan_seconds = seconds_now() - t0;

    t0 = seconds_now();
    for (int i = 0; i < CHECK_ITERS; i++) {
        sink += path_still_feasible(&m, path, n);
    }
    double check_seconds = seconds_now() - t0;

    t0 = seconds_now();
    for (int i = 0; i < CHECK_ITERS; i++) {
        sink += smoothed_path_still_clear(&m, path, n);
    }
    double los_seconds = seconds_now() - t0;

    double plan_ns = plan_seconds * 1e9 / PLAN_ITERS;
    double check_ns = check_seconds * 1e9 / CHECK_ITERS;
    double los_ns = los_seconds * 1e9 / CHECK_ITERS;
    double ratio = check_ns / plan_ns;
    double los_ratio = los_ns / plan_ns;

    /* Keep the volatile sink meaningful without printing it as a result. */
    if (sink == 0x7fffffff) printf("sink=%ld\n", (long)sink);

    printf("functional_ok=1\n");
    printf("path_nodes=%d\n", n);
    printf("plan_iterations=%d\n", PLAN_ITERS);
    printf("check_iterations=%d\n", CHECK_ITERS);
    printf("plan_ns_per_call=%.3f\n", plan_ns);
    printf("check_ns_per_call=%.3f\n", check_ns);
    printf("los_check_ns_per_call=%.3f\n", los_ns);
    printf("check_overhead_ratio=%.6f\n", ratio);
    printf("los_check_overhead_ratio=%.6f\n", los_ratio);
    printf("budget_ratio=0.200000\n");
    printf("check_pass_overhead=%d\n", ratio <= 0.20);
    printf("los_pass_overhead=%d\n", los_ratio <= 0.20);

    return ratio <= 0.20 && los_ratio <= 0.20 ? 0 : 2;
}
