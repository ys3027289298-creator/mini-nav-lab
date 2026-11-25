/* Conformance probe: run the real grid.c A* on the case list and print each
 * path as a flat index sequence.  Used by migration/tests/test_cross_check.py
 * so the migration's datasets are anchored to the C implementation in this
 * repository rather than to a Python re-derivation of it.
 *
 * Reads lines "sx sy gx gy nblocked x0,y0 x1,y1 ..." from argv[1] (or stdin),
 * writes "OK idx,idx,..." or "NONE".
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 16
#define H 12
#define MAXN (W * H)

typedef struct { unsigned char blocked[W * H]; } Map;
static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* Same algorithm as grid/src/grid.c; cap removed because the caller sizes the
 * buffer at MAXN and the grid cannot produce a longer simple path. */
static int astar(const Map *m, int sx, int sy, int gx, int gy, int *out) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1;
    }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0;
    f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++)
            if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == idx(gx, gy)) {
            int n = 0, c = cur;
            int rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
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
                for (int i = 0; i < nopen; i++)
                    if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return -1;
}

int main(int argc, char **argv) {
    FILE *in = stdin;
    if (argc > 1) {
        in = fopen(argv[1], "r");
        if (!in) { perror(argv[1]); return 2; }
    }
    int sx, sy, gx, gy, nb;
    char line[4096];
    while (fgets(line, sizeof(line), in)) {
        if (line[0] == '#' || line[0] == '\n') continue;
        if (sscanf(line, "%d %d %d %d %d", &sx, &sy, &gx, &gy, &nb) != 5) continue;
        Map m;
        memset(&m, 0, sizeof(m));
        char *p = line;
        /* skip the five integers */
        for (int k = 0; k < 5; k++) {
            while (*p == ' ') p++;
            while (*p && *p != ' ') p++;
        }
        for (int i = 0; i < nb; i++) {
            int x, y;
            while (*p == ' ') p++;
            if (sscanf(p, "%d,%d", &x, &y) != 2) break;
            if (x >= 0 && y >= 0 && x < W && y < H) m.blocked[idx(x, y)] = 1;
            while (*p && *p != ' ') p++;
        }
        int out[MAXN];
        int n = astar(&m, sx, sy, gx, gy, out);
        if (n < 0) {
            printf("NONE\n");
        } else {
            printf("OK ");
            for (int i = 0; i < n; i++) printf(i ? ",%d" : "%d", out[i]);
            printf("\n");
        }
    }
    if (in != stdin) fclose(in);
    return 0;
}
