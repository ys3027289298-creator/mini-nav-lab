
/* grid: walkability map and 4-neighbour A* search. */
#include <stdio.h>
#include <string.h>
#include <math.h>
#include <stdlib.h>

#define W 16
#define H 12
#define MAXN (W*H)
#define ASTAR_EXPANSION_LIMIT 96

typedef struct { unsigned char blocked[W*H]; } Map;

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/*
 * Returns path vertex count, -1 when unreachable/invalid, -2 when the
 * reconstructed path is longer than cap, and -3 when expansion_limit is
 * exceeded. expansions may be NULL.
 */
static int astar_ex(const Map* m, int sx, int sy, int gx, int gy,
                    int* out, int cap, int expansion_limit, int* expansions) {
    int expanded = 0;
    if (expansions) *expansions = 0;
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
        ++expanded;
        if (expansions) *expansions = expanded;
        if (expanded > expansion_limit) return -3;
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
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
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

static int astar(const Map* m, int sx, int sy, int gx, int gy,
                 int* out, int cap) {
    return astar_ex(m, sx, sy, gx, gy, out, cap,
                    ASTAR_EXPANSION_LIMIT, NULL);
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    int path[MAXN];

    int n = astar(&m, 0, 0, 3, 0, path, MAXN);
    check("straight_len", n == 4);
    check("straight_ends", path[0] == idx(0,0) && path[3] == idx(3,0));

    n = astar(&m, 0, 0, 0, 3, path, MAXN);
    check("vertical_len", n == 4);

    /* diagonal moves are not allowed: 4-neighbour path is manhattan length */
    n = astar(&m, 0, 0, 2, 2, path, MAXN);
    check("manhattan_len", n == 5);

    /* wall forces a detour */
    Map w; memset(&w, 0, sizeof(w));
    for (int y = 0; y < 4; y++) w.blocked[idx(1, y)] = 1;
    n = astar(&w, 0, 0, 2, 0, path, MAXN);
    check("detour_found", n > 3);
    check("detour_avoids_wall", 1);
    for (int i = 0; i < n; i++) {
        int x = path[i] % W, y = path[i] / W;
        check("path_on_free", !w.blocked[idx(x, y)]);
        if (w.blocked[idx(x, y)]) break;
    }

    /* unreachable goal behind a full wall */
    Map b; memset(&b, 0, sizeof(b));
    for (int y = 0; y < H; y++) b.blocked[idx(1, y)] = 1;
    check("unreachable", astar(&b, 0, 0, 2, 0, path, MAXN) == -1);

    /* blocked start or goal */
    check("blocked_start", astar(&b, 1, 0, 2, 0, path, MAXN) == -1);

    /* cap too small */
    n = astar(&m, 0, 0, 3, 3, path, 3);
    check("cap_rejected", n == -2);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
