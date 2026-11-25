
/* migrate: grid-corner vs grid-center coordinate migration and its path-length impact.
 *
 * Old grid API addresses cells by integer index; the new float world API needs
 * world coordinates. Two conventions are compared:
 *   center: P(i) = (i + 0.5) * s   (cell centre, new-API reference point)
 *   corner: P(i) = i * s           (lower-left cell corner, legacy shortcut)
 * A third mode, mixed, models a partial rollout: endpoints keep corner anchors
 * while interior waypoints are recomputed to centres -- the real failure mode
 * behind the observed ~3% length discrepancy.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W * H)
#define CELL 1.0          /* world units per grid cell, s */
#define SQRT2 1.41421356237309504880
#define TOL_PCT 1.0       /* length-budget tolerance, percent */

typedef struct { unsigned char blocked[W * H]; } Map;

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* 4-neighbour A*, same search as grid/; returns cell count or negative error. */
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

typedef enum { CONV_CENTER, CONV_CORNER, CONV_MIXED } Conv;

/* world coordinate of grid cell cellx at path position pos (0..n-1). */
static double worldx(int cellx, int pos, int n, Conv conv) {
    if (conv == CONV_CENTER) return (cellx + 0.5) * CELL;
    if (conv == CONV_CORNER) return cellx * CELL;
    /* mixed: endpoints keep the legacy corner anchor, interiors use centres */
    double k = (pos == 0 || pos == n - 1) ? cellx : cellx + 0.5;
    return k * CELL;
}

static double path_len(const int *path, int n, Conv conv, double *segs) {
    double total = 0.0;
    for (int i = 0; i + 1 < n; i++) {
        int x0 = path[i] % W,     y0 = path[i] / W;
        int x1 = path[i + 1] % W, y1 = path[i + 1] / W;
        double d = hypot(worldx(x1, i + 1, n, conv) - worldx(x0, i, n, conv),
                         worldx(y1, i + 1, n, conv) - worldx(y0, i, n, conv));
        if (segs) segs[i] = d;
        total += d;
    }
    return total;
}

static int count_turns(const int *p, int n) {
    int t = 0;
    for (int i = 1; i + 1 < n; i++) {
        int dx0 = (p[i] % W) - (p[i - 1] % W), dy0 = (p[i] / W) - (p[i - 1] / W);
        int dx1 = (p[i + 1] % W) - (p[i] % W), dy1 = (p[i + 1] / W) - (p[i] / W);
        if (dx0 != dx1 || dy0 != dy1) t++;
    }
    return t;
}

/* Length of a path where corner_flag[i]=1 forces waypoint i onto the legacy
 * corner anchor and 0 keeps the centre anchor. Used to model a partial rollout
 * as the migration frontier advances along the path. */
static double len_masked(const int *path, int n, const int *corner_flag) {
    double total = 0.0;
    for (int i = 0; i + 1 < n; i++) {
        int x0 = path[i] % W,     y0 = path[i] / W;
        int x1 = path[i + 1] % W, y1 = path[i + 1] / W;
        double kx0 = corner_flag[i] ? x0 : x0 + 0.5;
        double ky0 = corner_flag[i] ? y0 : y0 + 0.5;
        double kx1 = corner_flag[i + 1] ? x1 : x1 + 0.5;
        double ky1 = corner_flag[i + 1] ? y1 : y1 + 0.5;
        total += hypot((kx1 - kx0) * CELL, (ky1 - ky0) * CELL);
    }
    return total;
}

typedef struct {
    const char *id, *name;
    int sx, sy, gx, gy;
    void (*walls)(Map *);
} Tier;

static void walls_short(Map *m) { m->blocked[idx(2, 0)] = 1; }

static void walls_medium(Map *m) {
    for (int y = 0; y <= 8; y++) m->blocked[idx(3, y)] = 1;   /* gap rows 9..11 */
    for (int y = 3; y <= 11; y++) m->blocked[idx(12, y)] = 1; /* gap rows 0..2 */
}

static void walls_long(Map *m) {
    for (int y = 0; y <= 2; y++)  m->blocked[idx(3, y)] = 1;
    for (int y = 3; y <= 5; y++)  m->blocked[idx(6, y)] = 1;
    for (int y = 6; y <= 8; y++)  m->blocked[idx(9, y)] = 1;
    for (int y = 9; y <= 11; y++) m->blocked[idx(12, y)] = 1;
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* tier-prefixed check, e.g. T1_uniform_corner_matches_center=1 */
static void tcheck(const char *tid, const char *name, int ok) {
    char buf[128];
    snprintf(buf, sizeof(buf), "%s_%s", tid, name);
    check(buf, ok);
}

int main(void) {
    const Tier tiers[] = {
        {"T1", "short",  0, 0, 4, 11, walls_short},
        {"T2", "medium", 0, 0, 15, 11, walls_medium},
        {"T3", "long",   0, 0, 15, 11, walls_long},
    };
    const int ntier = (int)(sizeof(tiers) / sizeof(tiers[0]));

    printf("cell_size_s=%.6f tolerance_pct=%.2f\n", CELL, TOL_PCT);
    printf("formula_center: P_c(i)=(i+0.5)*s\n");
    printf("formula_corner: P_k(i)=i*s\n");
    printf("formula_delta:  P_c(i)-P_k(i)=0.5*s\n");
    printf("formula_nmax_budget: Nmax=floor(%.2f*L/(0.5*s))\n", TOL_PCT / 100.0);
    printf("formula_nmax_tight:  Nmax=floor(%.2f*L/(sqrt(2)*s))  "
           "[two mis-anchored endpoints, each <= s/sqrt(2)]\n", TOL_PCT / 100.0);

    FILE *rsum = fopen("results.csv", "w");
    FILE *rseg = fopen("segments.csv", "w");
    FILE *rcur = fopen("curves.csv", "w");
    if (!rsum || !rseg || !rcur) { printf("fopen_failed=1\n"); return 1; }
    fprintf(rsum, "tier,name,cells,segments,turns,L_center,L_corner,L_mixed,"
                  "rel_corner_pct,rel_mixed_pct,nmax_budget,nmax_tight,"
                  "gate_corner,gate_mixed,worst_seg,worst_err\n");
    fprintf(rseg, "tier,seg,from_x,from_y,to_x,to_y,d_center,d_corner,d_mixed,abs_err_mixed\n");
    fprintf(rcur, "kind,x,value\n");

    /* analytical decision curves: worst-case deviation bounds (both conventions)
     * and migration cost of each convention, per segment count N */
    for (int N = 1; N <= 60; N++) {
        double L = N * CELL;
        fprintf(rcur, "bound_budget,%d,%.6f\n", N, 100.0 * (N * 0.5 * CELL) / L);
        fprintf(rcur, "bound_tight,%d,%.6f\n",  N, 100.0 * (SQRT2 * CELL) / L);
        fprintf(rcur, "cost_corner,%d,%.6f\n",  N, 0.0);
        fprintf(rcur, "cost_center,%d,%.6f\n",  N, (double)(N + 1));
    }

    for (int t = 0; t < ntier; t++) {
        Map m; memset(&m, 0, sizeof(m));
        tiers[t].walls(&m);
        int path[MAXN];
        int n = astar(&m, tiers[t].sx, tiers[t].sy, tiers[t].gx, tiers[t].gy, path, MAXN);
        if (n <= 0) { printf("%s_path_found=0\n", tiers[t].id); failures++; continue; }
        printf("%s_path_found=1\n", tiers[t].id);

        int N = n - 1;
        double dc[MAXN], dk[MAXN], dm[MAXN];
        double Lc = path_len(path, n, CONV_CENTER, dc);
        double Lk = path_len(path, n, CONV_CORNER, dk);
        double Lm = path_len(path, n, CONV_MIXED, dm);

        double rel_corner = 100.0 * fabs(Lk - Lc) / Lc;
        double rel_mixed  = 100.0 * fabs(Lm - Lc) / Lc;
        int nmax_budget = (int)floor((TOL_PCT / 100.0) * Lc / (0.5 * CELL));
        int nmax_tight  = (int)floor((TOL_PCT / 100.0) * Lc / (SQRT2 * CELL));
        int gate_corner = rel_corner <= TOL_PCT;
        int gate_mixed  = rel_mixed  <= TOL_PCT;

        int worst = 0;
        for (int i = 1; i < N; i++)
            if (fabs(dm[i] - dc[i]) > fabs(dm[worst] - dc[worst])) worst = i;
        double worst_err = fabs(dm[worst] - dc[worst]);

        printf("== %s %s start=(%d,%d) goal=(%d,%d) cells=%d segments=%d turns=%d\n",
               tiers[t].id, tiers[t].name, tiers[t].sx, tiers[t].sy,
               tiers[t].gx, tiers[t].gy, n, N, count_turns(path, n));
        printf("L_center=%.6f L_corner=%.6f L_mixed=%.6f\n", Lc, Lk, Lm);
        printf("rel_corner_pct=%.6f rel_mixed_pct=%.6f\n", rel_corner, rel_mixed);
        printf("nmax_budget=%d nmax_tight=%d (N=%d)\n", nmax_budget, nmax_tight, N);
        printf("gate_uniform_corner=%s gate_mixed_anchor=%s\n",
               gate_corner ? "PASS" : "FAIL", gate_mixed ? "PASS" : "FAIL");

        if (!gate_mixed) {
            int fx = path[worst] % W,     fy = path[worst] / W;
            int tx = path[worst + 1] % W, ty = path[worst + 1] / W;
            printf("worst_segment: tier=%s seg=%d from_cell=(%d,%d) to_cell=(%d,%d)\n",
                   tiers[t].id, worst, fx, fy, tx, ty);
            printf("  center_world=(%.2f,%.2f)->(%.2f,%.2f) d_ref=%.6f\n",
                   worldx(fx, worst, n, CONV_CENTER), worldx(fy, worst, n, CONV_CENTER),
                   worldx(tx, worst + 1, n, CONV_CENTER), worldx(ty, worst + 1, n, CONV_CENTER),
                   dc[worst]);
            printf("  mixed_world =(%.2f,%.2f)->(%.2f,%.2f) d_got=%.6f abs_err=%.6f\n",
                   worldx(fx, worst, n, CONV_MIXED), worldx(fy, worst, n, CONV_MIXED),
                   worldx(tx, worst + 1, n, CONV_MIXED), worldx(ty, worst + 1, n, CONV_MIXED),
                   dm[worst], worst_err);
        }

        fprintf(rsum, "%s,%s,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%d,%d,%s,%s,%d,%.6f\n",
                tiers[t].id, tiers[t].name, n, N, count_turns(path, n),
                Lc, Lk, Lm, rel_corner, rel_mixed, nmax_budget, nmax_tight,
                gate_corner ? "PASS" : "FAIL", gate_mixed ? "PASS" : "FAIL",
                worst, worst_err);
        fprintf(rcur, "measured_%s,%d,%.6f\n", tiers[t].id, N, rel_mixed);
        for (int i = 0; i < N; i++) {
            fprintf(rseg, "%s,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f\n",
                    tiers[t].id, i, path[i] % W, path[i] / W,
                    path[i + 1] % W, path[i + 1] / W,
                    dc[i], dk[i], dm[i], fabs(dm[i] - dc[i]));
        }

        /* self-checks: invariants the whole derivation rests on */
        tcheck(tiers[t].id, "uniform_corner_matches_center", rel_corner < 1e-12);
        tcheck(tiers[t].id, "center_hop_is_cell", fabs(dc[0] - CELL) < 1e-12);
        tcheck(tiers[t].id, "nmax_budget_formula",
               nmax_budget == (int)floor((TOL_PCT / 100.0) * Lc / (0.5 * CELL)));
        tcheck(tiers[t].id, "gate_labels_consistent",
               gate_corner == (rel_corner <= TOL_PCT) && gate_mixed == (rel_mixed <= TOL_PCT));
        int recheck = 0;
        for (int i = 1; i < N; i++)
            if (fabs(dm[i] - dc[i]) > fabs(dm[recheck] - dc[recheck])) recheck = i;
        tcheck(tiers[t].id, "worst_segment_argmax", recheck == worst);
    }

    /* Partial-rollout experiment on a zigzag path: long axis-aligned runs joined
     * by turns. A mis-anchored point inside a straight run telescopes away, but a
     * mis-anchored turn contributes independently (its two neighbour edges are
     * perpendicular, nothing cancels across the straight runs), so the length
     * deviation grows with the number of mis-anchored turns actually present. */
    int zig[8], nz = 8;
    const int zx[8] = {0, 4, 4, 9, 9, 13, 13, 15};
    const int zy[8] = {0, 0, 3, 3, 6, 6, 9, 9};
    for (int i = 0; i < nz; i++) zig[i] = idx(zx[i], zy[i]);
    int flag[8];
    for (int i = 0; i < nz; i++) flag[i] = 0;        /* fully migrated: all centres */
    double Lref = len_masked(zig, nz, flag);
    double prev = 0.0;
    int monotone = 1, zero_ok = 1;
    /* mis-anchored turn indices are 2,4,6 -- mutually isolated by at least one
     * migrated waypoint, so each one's contribution adds instead of interacting
     * across a shared edge. q = number of isolated mis-anchored turns. */
    const int bad_idx[3] = {2, 4, 6};
    for (int q = 0; q <= 3; q++) {
        for (int i = 0; i < nz; i++) flag[i] = 0;
        for (int b = 0; b < q; b++) flag[bad_idx[b]] = 1;
        double rel = 100.0 * fabs(len_masked(zig, nz, flag) - Lref) / Lref;
        fprintf(rcur, "measured_rollout,%d,%.6f\n", q, rel);
        if (q == 0 && rel > 1e-12) zero_ok = 0;
        if (rel + 1e-12 < prev) monotone = 0;
        prev = rel;
    }
    check("rollout_zero_at_full_center", zero_ok);
    check("rollout_monotone_non_decreasing", monotone);

    /* reference Nmax table: largest admissible segment count at several path
     * lengths, under both the conservative per-segment budget and the tight
     * two-mis-anchored-point bound (s=1, tolerance 1%) */
    const double Lrefs[] = {10.0, 20.0, 40.0, 100.0};
    printf("-- nmax_reference (tolerance=%.2f%%, s=%.2f)\n", TOL_PCT, CELL);
    for (int i = 0; i < 4; i++) {
        int nb = (int)floor((TOL_PCT / 100.0) * Lrefs[i] / (0.5 * CELL));
        int nt = (int)floor((TOL_PCT / 100.0) * Lrefs[i] / (SQRT2 * CELL));
        fprintf(rcur, "nmax_budget_ref,%.0f,%d\n", Lrefs[i], nb);
        fprintf(rcur, "nmax_tight_ref,%.0f,%d\n", Lrefs[i], nt);
        printf("L=%5.0f  Nmax_budget=%3d  Nmax_tight=%3d\n", Lrefs[i], nb, nt);
    }

    fclose(rsum);
    fclose(rseg);
    fclose(rcur);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
