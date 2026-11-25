/* turn: turn-minimising path under a length cap.
 *
 * Path P = p0..p_{n-1}, unit steps between 4-neighbour cells:
 *   L(P) = n - 1
 *   d_i  = p_{i+1} - p_i            i = 0..n-2
 *   T(P) = sum_{i=1}^{n-2} [d_i != d_{i-1}]
 *
 * Turn-budget shortest path (the constrained problem):
 *   L*(k) = min { L(P) : P: s->g, T(P) <= k }
 * solved exactly over (cell, direction, turns) states; L*(k) is the longest
 * path the turn budget k allows without paying extra cells.
 *
 * Turn-penalised shortest path:
 *   P_lam = argmin_P L(P) + lam * T(P)
 * solved with Dijkstra over (cell, direction) states.  lam is the price of one
 * turn in cells.  P_lam is the frontier vertex (k, L*(k)) minimising L + lam*k,
 * so it is optimal for lam in an interval bounded by the two adjacent frontier
 * slopes:
 *   lam in ( (L*(k) - L*(k+)) / (k+ - k),  (L*(k-) - L*(k)) / (k - k-) )
 * where k- and k+ are the neighbouring vertices of the frontier.
 *
 * Constraint inversion: acceptance needs turns < T0/2 and length <= 1.2*L0,
 * so the largest admissible vertex is
 *   k* = max { k on the frontier : 2k < T0 and L*(k) <= 6*L0/5 }
 * L*(k*) is the length actually paid, and the triangles below turn that into
 * one coefficient shared by all endpoint pairs.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 16
#define H 12
#define NCELL (W*H)
#define NDIR 5
#define TMAX 48
#define INF 0x3f3f3f3f

typedef struct { unsigned char blocked[NCELL]; } Map;
typedef struct { int sx, sy, gx, gy; } Pair;
typedef struct { int turns, len; } Stats;

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

static const int DX[4] = {1, -1, 0, 0};
static const int DY[4] = {0, 0, 1, -1};

static const int BLOCKED[][2] = {
    {1,4},{1,7},{1,8},{2,0},{2,3},{2,5},{2,7},{2,10},{3,4},{3,7},
    {4,9},{5,5},{5,7},{5,8},{6,5},{6,7},{6,10},{7,2},{7,6},{7,11},
    {8,0},{8,2},{8,6},{9,4},{9,8},{9,9},{9,10},{10,2},{10,4},{10,5},
    {10,6},{11,9},{11,10},{11,11},{12,0},{12,7},{12,9},{13,0},{13,1},
    {13,6},{14,2},{14,4},{15,3},{15,5}
};

static Map build_map(void) {
    Map m;
    memset(&m, 0, sizeof(m));
    for (unsigned i = 0; i < sizeof(BLOCKED) / sizeof(BLOCKED[0]); i++)
        m.blocked[idx(BLOCKED[i][0], BLOCKED[i][1])] = 1;
    return m;
}

/* baseline: plain 4-neighbour A*, minimises length only */
static int astar(const Map* m, int sx, int sy, int gx, int gy, int* out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[NCELL], f[NCELL], closed[NCELL], parent[NCELL];
    for (int i = 0; i < NCELL; i++) { g[i] = INF; f[i] = INF; closed[i] = 0; parent[i] = -1; }
    int open[NCELL], nopen = 0;
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
            int rev[NCELL];
            while (c != -1 && n < NCELL) { rev[n++] = c; c = parent[c]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + DX[d], ny = cy + DY[d];
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

static Stats stats(const int* path, int n) {
    Stats s = {0, 0};
    if (n < 2) return s;
    s.len = n - 1;
    int prev = -1;
    for (int i = 1; i < n; i++) {
        int dx = path[i] % W - path[i - 1] % W;
        int dy = path[i] / W - path[i - 1] / W;
        int d = dx == 1 ? 0 : dx == -1 ? 1 : dy == 1 ? 2 : 3;
        if (prev != -1 && d != prev) s.turns++;
        prev = d;
    }
    return s;
}

/* exact frontier L*(k): unweighted BFS over (cell, direction, turns) */
static void frontier(const Map* m, const Pair* p, int* Lmin) {
    static int dist[NCELL * NDIR * (TMAX + 1)];
    static int q[NCELL * NDIR * (TMAX + 1)];
    int cnt = NCELL * NDIR * (TMAX + 1);
    for (int i = 0; i < cnt; i++) dist[i] = -1;
    int s = ((idx(p->sx, p->sy) * NDIR + 4) * (TMAX + 1));
    dist[s] = 0;
    int head = 0, tail = 0;
    q[tail++] = s;
    while (head < tail) {
        int st = q[head++];
        int t = st % (TMAX + 1);
        int rest = st / (TMAX + 1);
        int d = rest % NDIR;
        int c = rest / NDIR;
        int cx = c % W, cy = c / W;
        for (int nd = 0; nd < 4; nd++) {
            if (d != 4 && nd == (d ^ 1)) continue;
            int nx = cx + DX[nd], ny = cy + DY[nd];
            if (!walkable(m, nx, ny)) continue;
            int nt = t + (d != 4 && nd != d ? 1 : 0);
            if (nt > TMAX) continue;
            int ns = ((idx(nx, ny) * NDIR + nd) * (TMAX + 1) + nt);
            if (dist[ns] != -1) continue;
            dist[ns] = dist[st] + 1;
            q[tail++] = ns;
        }
    }
    int goal = idx(p->gx, p->gy);
    int cur = INF;
    for (int k = 0; k <= TMAX; k++) {
        for (int d = 0; d < NDIR; d++) {
            int dd = dist[((goal * NDIR + d) * (TMAX + 1) + k)];
            if (dd >= 0 && dd < cur) cur = dd;
        }
        Lmin[k] = cur;
    }
}

/* turn-penalised Dijkstra: key = (L*1000 + lam_milli*T)*64 + T, so it
 * minimises L + lam*T first and turns second on exact ties. */
static int penalised(const Map* m, const Pair* p, int lam_milli, int* out, int cap) {
    enum { NSTATE = NCELL * NDIR, KSTEP = 64000 };
    static int key[NSTATE], parent[NSTATE];
    static unsigned char done[NSTATE];
    for (int i = 0; i < NSTATE; i++) { key[i] = INF; parent[i] = -1; done[i] = 0; }
    int s = idx(p->sx, p->sy) * NDIR + 4;
    int goalc = idx(p->gx, p->gy);
    key[s] = 0;
    for (;;) {
        int best = -1;
        for (int i = 0; i < NSTATE; i++) {
            if (done[i] || key[i] >= INF) continue;
            if (best < 0 || key[i] < key[best]) best = i;
        }
        if (best < 0) return -1;
        if (best / NDIR == goalc) {
            int rev[NCELL], n = 0, cur = best;
            while (cur != -1) { rev[n++] = cur / NDIR; cur = parent[cur]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        done[best] = 1;
        int c = best / NDIR, d = best % NDIR;
        int cx = c % W, cy = c / W;
        for (int nd = 0; nd < 4; nd++) {
            if (d != 4 && nd == (d ^ 1)) continue;
            int nx = cx + DX[nd], ny = cy + DY[nd];
            if (!walkable(m, nx, ny)) continue;
            int ns = idx(nx, ny) * NDIR + nd;
            if (done[ns]) continue;
            int turn = (d != 4 && nd != d);
            int w = KSTEP + (turn ? lam_milli * 64 + 1 : 0);
            if (key[best] + w < key[ns]) {
                key[ns] = key[best] + w;
                parent[ns] = best;
            }
        }
    }
}

static int valid_path(const Map* m, const Pair* p, const int* path, int n) {
    if (n < 1) return 0;
    if (path[0] != idx(p->sx, p->sy) || path[n - 1] != idx(p->gx, p->gy)) return 0;
    for (int i = 0; i < n; i++) {
        int x = path[i] % W, y = path[i] / W;
        if (!walkable(m, x, y)) return 0;
        if (i > 0) {
            int d = abs(path[i] % W - path[i - 1] % W) + abs(path[i] / W - path[i - 1] / W);
            if (d != 1) return 0;
        }
    }
    return 1;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map m = build_map();
    const Pair pairs[3] = {
        {0, 5, 15, 11},
        {0, 7, 15, 11},
        {0, 5, 15, 7},
    };
    const int LAMS[] = {250, 500, 750, 1000, 1250, 1500, 2000, 3000, 5000};
    const int NLAM = (int)(sizeof(LAMS) / sizeof(LAMS[0]));
    int T0[3], L0[3], Lmin[3][TMAX + 1];
    int vk[3][TMAX + 1], vl[3][TMAX + 1], nv[3];
    int kstar[3], Lstar[3];
    double lo[3], hi[3];
    int path[NCELL];

    for (int i = 0; i < 3; i++) {
        int n = astar(&m, pairs[i].sx, pairs[i].sy, pairs[i].gx, pairs[i].gy, path, NCELL);
        if (n < 2) { printf("baseline pair%d failed\n", i); return 1; }
        Stats st = stats(path, n);
        T0[i] = st.turns; L0[i] = st.len;
        frontier(&m, &pairs[i], Lmin[i]);
        nv[i] = 0;
        for (int k = 0; k <= TMAX; k++) {
            if (Lmin[i][k] >= INF) continue;
            if (k == 0 || Lmin[i][k] < Lmin[i][k - 1]) {
                vk[i][nv[i]] = k;
                vl[i][nv[i]] = Lmin[i][k];
                nv[i]++;
            }
        }
        printf("pair%d start=(%d,%d) goal=(%d,%d)\n",
               i, pairs[i].sx, pairs[i].sy, pairs[i].gx, pairs[i].gy);
        printf("baseline turns=%d length=%d\n", T0[i], L0[i]);
        printf("frontier");
        for (int v = 0; v < nv[i]; v++) printf(" k=%d:L=%d", vk[i][v], vl[i][v]);
        printf("\n");
    }

    /* invert the two acceptance bounds into the largest admissible vertex */
    for (int i = 0; i < 3; i++) {
        kstar[i] = -1; Lstar[i] = -1;
        for (int v = nv[i] - 1; v >= 0; v--) {
            int k = vk[i][v], L = vl[i][v];
            if (!(2 * k < T0[i])) continue;
            if (!(5 * L <= 6 * L0[i])) continue;
            double l = 0.0, h = 1e30;
            for (int w = 0; w < nv[i]; w++) {
                if (w == v) continue;
                int kw = vk[i][w], Lw = vl[i][w];
                double slope;
                if (kw > k) {
                    slope = (double)(L - Lw) / (double)(kw - k);
                    if (slope > l) l = slope;
                } else {
                    slope = (double)(Lw - L) / (double)(k - kw);
                    if (slope < h) h = slope;
                }
            }
            if (l < h) { kstar[i] = k; Lstar[i] = L; lo[i] = l; hi[i] = h; break; }
        }
        if (kstar[i] < 0) {
            printf("pair%d no_admissible_budget\n", i);
            failures++;
            continue;
        }
        printf("budget pair%d k*=%d allowed_length=%d length_cap=%d lambda_window=(%.3f,",
               i, kstar[i], Lstar[i], 6 * L0[i] / 5, lo[i]);
        if (hi[i] > 1e29) printf("inf)\n"); else printf("%.3f)\n", hi[i]);
    }

    double clo = 0.0, chi = 1e30;
    for (int i = 0; i < 3; i++) {
        if (kstar[i] < 0) continue;
        if (lo[i] > clo) clo = lo[i];
        if (hi[i] < chi) chi = hi[i];
    }
    int lam_milli = (chi > 1e29)
        ? (int)(clo * 2.0 * 1000.0 + 0.5)
        : (int)((clo + chi) * 0.5 * 1000.0 + 0.5);
    double lam = lam_milli / 1000.0;
    printf("common_lambda_window=(%.3f,", clo);
    if (chi > 1e29) printf("inf)\n"); else printf("%.3f)\n", chi);
    printf("lambda_star=%.3f\n", lam);

    printf("lambda_sweep\n");
    for (int l = 0; l < NLAM; l++) {
        printf("lam=%.3f", LAMS[l] / 1000.0);
        for (int i = 0; i < 3; i++) {
            int n = penalised(&m, &pairs[i], LAMS[l], path, NCELL);
            Stats st = stats(path, n);
            int ok = 2 * st.turns < T0[i] && 5 * st.len <= 6 * L0[i];
            printf(" pair%d=T%d,L%d%s", i, st.turns, st.len, ok ? "" : "!");
        }
        printf("\n");
    }

    for (int i = 0; i < 3; i++) {
        int n = penalised(&m, &pairs[i], lam_milli, path, NCELL);
        Stats st = stats(path, n);
        int turns_ok = 2 * st.turns < T0[i];
        int len_ok = 5 * st.len <= 6 * L0[i];
        printf("run pair%d turns=%d length=%d turns_reduction=%.1f%% length_growth=%.1f%% ok=%d\n",
               i, st.turns, st.len,
               100.0 * (T0[i] - st.turns) / T0[i],
               100.0 * (st.len - L0[i]) / L0[i],
               turns_ok && len_ok);
        char name[64];
        snprintf(name, sizeof(name), "pair%d_turns_under_half", i);
        check(name, turns_ok);
        snprintf(name, sizeof(name), "pair%d_length_within_20pct", i);
        check(name, len_ok);
        snprintf(name, sizeof(name), "pair%d_selected_budget_vertex", i);
        check(name, st.len == Lstar[i] && st.turns <= kstar[i]);
        snprintf(name, sizeof(name), "pair%d_path_valid", i);
        check(name, valid_path(&m, &pairs[i], path, n));
    }

    check("frontier_has_tension", nv[0] >= 3 && nv[1] >= 3 && nv[2] >= 3);
    check("lambda_in_common_window", lam > clo && lam < chi);
    check("lambda_sweep_toggles_turns",
          penalised(&m, &pairs[0], 250, path, NCELL) > 0 &&
          penalised(&m, &pairs[0], 5000, path, NCELL) > 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
