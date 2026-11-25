/* astar_calib: instrumented 4-neighbour A* sampler.
 *
 * Reuses the search logic from grid/src/grid.c (same O(n) open list,
 * same 16x12 map) and counts node expansions per request so the
 * scheduler simulator can draw real per-request work sizes.
 *
 * Three request classes:
 *   P0 main-quest : sparse walls, short manhattan distance
 *   P1 normal     : medium walls, medium distance
 *   P2 background : dense walls, long distance, some unreachable
 *
 * Output CSV on stdout: class,expansions,reachable,distance
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 16
#define H 12
#define MAXN (W * H)

typedef struct { unsigned char blocked[W * H]; } Map;

static unsigned long rng_state;
static unsigned long xr(void) {
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 7;
    rng_state ^= rng_state << 17;
    return rng_state;
}
static int rnd(int n) { return (int)(xr() % (unsigned long)n); }

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* returns expansions used; *reachable=0 when goal unreachable */
static int astar_counted(const Map* m, int sx, int sy, int gx, int gy, int* reachable) {
    *reachable = 1;
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) {
        *reachable = 0;
        return 0;
    }
    int g[MAXN], f[MAXN], closed[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0;
    }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    int expansions = 0;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        expansions++;
        if (cur == idx(gx, gy)) return expansions;
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
                int found = 0;
                for (int i = 0; i < nopen; i++) if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    *reachable = 0;
    return expansions;
}

static void build_map(Map* m, double wall_p, int divide_wall) {
    memset(m, 0, sizeof(*m));
    for (int i = 0; i < MAXN; i++)
        if ((xr() % 1000) < (unsigned long)(wall_p * 1000.0)) m->blocked[i] = 1;
    if (divide_wall) {
        /* full vertical wall with no gap: splits map into two components */
        int wx = 4 + rnd(W - 8);
        for (int y = 0; y < H; y++) m->blocked[idx(wx, y)] = 1;
    }
}

static void random_endpoints(const Map* m, int dmin, int dmax,
                             int* sx, int* sy, int* gx, int* gy) {
    for (;;) {
        int x0 = rnd(W), y0 = rnd(H);
        if (!walkable(m, x0, y0)) continue;
        int x1 = rnd(W), y1 = rnd(H);
        if (!walkable(m, x1, y1)) continue;
        int d = abs(x1 - x0) + abs(y1 - y0);
        if (d < dmin || d > dmax) continue;
        *sx = x0; *sy = y0; *gx = x1; *gy = y1;
        return;
    }
}

int main(int argc, char** argv) {
    int per_class = (argc > 1) ? atoi(argv[1]) : 3000;
    rng_state = (argc > 2) ? strtoul(argv[2], NULL, 10) : 20261126UL;
    if (rng_state == 0) rng_state = 1;

    struct { const char* name; double wall; int dmin, dmax; int divide_pct; } cls[3] = {
        {"P0", 0.08, 2,  6,  0},
        {"P1", 0.15, 7,  14, 0},
        {"P2", 0.22, 15, 27, 15},
    };

    printf("class,expansions,reachable,distance\n");
    for (int c = 0; c < 3; c++) {
        for (int k = 0; k < per_class; k++) {
            Map m;
            int divide = rnd(100) < cls[c].divide_pct;
            build_map(&m, cls[c].wall, divide);
            int sx, sy, gx, gy;
            random_endpoints(&m, cls[c].dmin, cls[c].dmax, &sx, &sy, &gx, &gy);
            int reachable;
            int n = astar_counted(&m, sx, sy, gx, gy, &reachable);
            printf("%s,%d,%d,%d\n", cls[c].name, n, reachable,
                   abs(gx - sx) + abs(gy - sy));
        }
    }
    return 0;
}
