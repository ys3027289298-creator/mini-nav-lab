#include <stdlib.h>
#include <string.h>

#include "nav.h"

int nav_path_init(NavPath* p, int cells) {
    p->n = -1;
    p->cap = cells;
    p->x = (uint16_t*)malloc(sizeof(uint16_t) * (size_t)cells);
    p->y = (uint16_t*)malloc(sizeof(uint16_t) * (size_t)cells);
    if (!p->x || !p->y) { nav_path_free(p); return 0; }
    return 1;
}

void nav_path_free(NavPath* p) {
    free(p->x);
    free(p->y);
    p->x = NULL;
    p->y = NULL;
    p->n = -1;
}

static int walkable(const MapFile* m, int x, int y) {
    if (x < 0 || y < 0 || x >= m->w || y >= m->h) return 0;
    return !m->cells[(size_t)y * m->w + x];
}

/* Scratch arrays are per call, not hidden globals, so the benchmark can
 * compute two paths for different map sizes in one process. */
int nav_astar(const MapFile* m, int sx, int sy, int gx, int gy, NavPath* p) {
    if (p) p->n = -1;
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return 0;

    size_t cells = (size_t)m->w * m->h;
    int* g   = (int*)malloc(sizeof(int) * cells);
    int* f   = (int*)malloc(sizeof(int) * cells);
    int* par = (int*)malloc(sizeof(int) * cells);
    int* open = (int*)malloc(sizeof(int) * cells);
    unsigned char* closed = (unsigned char*)calloc(cells, 1);
    int* rev = (int*)malloc(sizeof(int) * cells);
    if (!g || !f || !par || !open || !closed || !rev) {
        free(g); free(f); free(par); free(open); free(closed); free(rev);
        return 0;
    }

    const int INF = 1 << 29;
    for (size_t i = 0; i < cells; i++) { g[i] = INF; f[i] = INF; par[i] = -1; }

    int start = sy * m->w + sx;
    int goal  = gy * m->w + gx;
    int nopen = 0;
    g[start] = 0;
    f[start] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = start;

    int found = 0;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++)
            if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == goal) { found = 1; break; }
        if (closed[cur]) continue;
        closed[cur] = 1;

        int cx = cur % m->w, cy = cur / m->w;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = ny * m->w + nx;
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                par[ni] = cur;
                int in_open = 0;
                for (int i = 0; i < nopen; i++)
                    if (open[i] == ni) { in_open = 1; break; }
                if (!in_open) open[nopen++] = ni;
            }
        }
    }

    int rc = 0;
    if (found && p) {
        int n = 0;
        for (int c = goal; c != -1 && n < (int)cells; c = par[c]) rev[n++] = c;
        if (n <= p->cap) {
            for (int i = 0; i < n; i++) {
                int c = rev[n - 1 - i];
                p->x[i] = (uint16_t)(c % m->w);
                p->y[i] = (uint16_t)(c / m->w);
            }
            p->n = n;
            rc = 1;
        }
    } else if (found) {
        rc = 1;
    }

    free(g); free(f); free(par); free(open); free(closed); free(rev);
    return rc;
}

int nav_compare(const NavPath* a, const NavPath* b, int* same_points,
                int* ratio_denom, int* first_diff) {
    int fd = -1;
    if (a->n < 0 || b->n < 0) {
        /* Both unreachable counts as identical; one reachable and one not is
         * the most serious difference there is. */
        int same = (a->n < 0 && b->n < 0);
        if (same_points) *same_points = 0;
        if (ratio_denom) *ratio_denom = 0;
        if (first_diff) *first_diff = same ? -1 : 0;
        return same;
    }

    int n1 = a->n, n2 = b->n;
    int lim = n1 < n2 ? n1 : n2;
    int same = 0;
    for (int i = 0; i < lim; i++) {
        if (a->x[i] == b->x[i] && a->y[i] == b->y[i]) same++;
        else if (fd < 0) fd = i;
    }
    if (n1 != n2 && fd < 0) fd = lim;

    if (same_points) *same_points = same;
    if (ratio_denom) *ratio_denom = n1 > n2 ? n1 : n2;
    if (first_diff) *first_diff = fd;
    return (n1 == n2 && fd < 0);
}
