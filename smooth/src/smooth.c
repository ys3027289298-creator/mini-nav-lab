
/* smooth: exact line-of-sight, clearance-aware string-pull smoothing.
 *
 * The previous version used a Bresenham supercover for LOS. Bresenham
 * walks cell centers, so on a steep/oblique segment it skips cells whose
 * square the segment actually crosses: e.g. the center line (0.5,0.5) to
 * (12.5,7.5) passes through cells (4,3) and (9,6), but Bresenham never
 * visits them. A wall there is invisible, LOS wrongly passes, and every
 * smoothing chord is accepted -- the 15-node/13-turn staircase never
 * collapses. The tests below include that case as a regression.
 *
 * Correct model (cells are unit squares [x,x+1]x[y,y+1]):
 *   LOS(p0,p1,r)  <=>  min over blocked cells b of dist(segment, b) >= r
 * i.e. the swept disk of character radius r never enters a blocked cell.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W * H)

typedef struct { unsigned char blocked[W * H]; } Map;
static int idx(int x, int y) { return y * W + x; }
static double min2(double a, double b) { return a < b ? a : b; }
static double max2(double a, double b) { return a > b ? a : b; }
static double clampd(double v, double lo, double hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* distance from point q to the unit square of cell (cx, cy) */
static double point_box_dist(double qx, double qy, int cx, int cy) {
    double dx = max2(cx - qx, max2(0.0, qx - (cx + 1)));
    double dy = max2(cy - qy, max2(0.0, qy - (cy + 1)));
    return hypot(dx, dy);
}

/* exact min distance between segment p0-p1 and a blocked unit square.
 * f(t) = dist(point(t), box)^2 is piecewise quadratic; its minimum is at
 * an endpoint, where the segment crosses a box face, or at the projection
 * of a box corner. Check all such candidate parameters. */
static double seg_box_dist(double x0, double y0, double x1, double y1,
                           int cx, int cy) {
    double vx = x1 - x0, vy = y1 - y0;
    double L2 = vx * vx + vy * vy;
    double ts[12];
    int nt = 0;
    ts[nt++] = 0.0;
    ts[nt++] = 1.0;
    double faces[4] = { (double)cx, (double)cx + 1, (double)cy,
                        (double)cy + 1 };
    for (int i = 0; i < 2; i++)
        if (fabs(i == 0 ? vx : vy) > 1e-12)
            ts[nt++] = (faces[i] - (i == 0 ? x0 : y0)) /
                       (i == 0 ? vx : vy);
    for (int i = 2; i < 4; i++)
        if (fabs(i == 2 ? vx : vy) > 1e-12)
            ts[nt++] = (faces[i] - (i == 2 ? x0 : y0)) /
                       (i == 2 ? vx : vy);
    for (int kx = 0; kx < 2; kx++)
        for (int ky = 0; ky < 2; ky++) {
            double px = cx + kx, py = cy + ky;
            if (L2 > 1e-12)
                ts[nt++] = ((px - x0) * vx + (py - y0) * vy) / L2;
        }
    double best = 1e30;
    for (int i = 0; i < nt; i++) {
        double t = clampd(ts[i], 0.0, 1.0);
        double qx = x0 + t * vx, qy = y0 + t * vy;
        best = min2(best, point_box_dist(qx, qy, cx, cy));
    }
    return best;
}

/* independent estimator: dense sampling, point-to-box distance per sample */
static double seg_clearance_sampled(const Map *m, int x0, int y0,
                                    int x1, int y1) {
    double p0x = x0 + 0.5, p0y = y0 + 0.5, p1x = x1 + 0.5, p1y = y1 + 0.5;
    double worst = 1e30;
    const int STEPS = 2048;
    for (int s = 0; s <= STEPS; s++) {
        double t = (double)s / STEPS;
        double qx = p0x + t * (p1x - p0x), qy = p0y + t * (p1y - p0y);
        for (int i = 0; i < W * H; i++) {
            if (!m->blocked[i]) continue;
            worst = min2(worst, point_box_dist(qx, qy, i % W, i / W));
        }
    }
    return worst;
}

/* exact minimum segment-to-wall clearance over every blocked cell.
 * endpoints outside the map are treated as fully blocked (out of bounds
 * never has LOS). */
static double seg_clearance(const Map *m, int x0, int y0,
                            int x1, int y1) {
    if (x0 < 0 || y0 < 0 || x0 >= W || y0 >= H ||
        x1 < 0 || y1 < 0 || x1 >= W || y1 >= H)
        return -1.0;
    double p0x = x0 + 0.5, p0y = y0 + 0.5, p1x = x1 + 0.5, p1y = y1 + 0.5;
    double worst = 1e30;
    for (int i = 0; i < W * H; i++) {
        if (!m->blocked[i]) continue;
        worst = min2(worst, seg_box_dist(p0x, p0y, p1x, p1y,
                                         i % W, i / W));
    }
    return worst;
}

/* radius-aware geometric LOS: the radius-r swept tube stays out of walls.
 * Tube/box intersection happens at clearance <= r, so LOS is the STRICT
 * inequality clearance > r -- grazing contact counts as blocked. */
static int los_r(const Map *m, int x0, int y0, int x1, int y1, double r) {
    return seg_clearance(m, x0, y0, x1, y1) - r > 1e-9;
}
static int los(const Map *m, int x0, int y0, int x1, int y1) {
    return los_r(m, x0, y0, x1, y1, 0.0);
}

/* legacy Bresenham supercover, kept only to demonstrate the missed-cell
 * failure mode in a regression test. Do not use for real LOS. */
static int los_bresenham(const Map *m, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy, x = x0, y = y0;
    for (;;) {
        if (x < 0 || y < 0 || x >= W || y >= H) return 0;
        if (m->blocked[idx(x, y)]) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx)  { err += dx; y += sy; }
    }
}

/* greedy string pull with radius-aware LOS; returns point count or <0 */
static int smooth_greedy(const Map *m, const int *path, int n,
                         int *out, int cap, double r) {
    if (n <= 0) return -1;
    if (n == 1) { if (cap < 1) return -2; out[0] = path[0]; return 1; }
    int w = 0;
    if (cap < 1) return -2;
    out[w++] = path[0];
    int i = 0;
    while (i < n - 1) {
        int j = n - 1;
        while (j > i + 1 &&
               !los_r(m, path[i] % W, path[i] / W,
                      path[j] % W, path[j] / W, r))
            j--;
        /* even the adjacent edge fails: corridor infeasible at radius r */
        if (!los_r(m, path[i] % W, path[i] / W,
                   path[j] % W, path[j] / W, r))
            return -3;
        if (w >= cap) return -2;
        out[w++] = path[j];
        i = j;
    }
    return w;
}

/* DP for the minimum-point feasible subsequence (the answer to "how few
 * points can this be pressed down to"). Intermediate vertices may be
 * unreachable at the radius (dp stays INF); only the goal matters.
 * Returns count, or <0 on infeasibility / capacity. */
static int smooth_min_points(const Map *m, const int *path, int n,
                             int *out, int cap, double r) {
    if (n <= 0) return -1;
    int dp[MAXN], prev[MAXN];
    for (int j = 0; j < n; j++) {
        dp[j] = j == 0 ? 1 : MAXN + 1;
        prev[j] = -1;
        for (int i = 0; i < j; i++) {
            if (dp[i] <= MAXN &&
                los_r(m, path[i] % W, path[i] / W,
                      path[j] % W, path[j] / W, r) &&
                dp[i] + 1 < dp[j]) {
                dp[j] = dp[i] + 1;
                prev[j] = i;
            }
        }
    }
    if (dp[n - 1] > MAXN) return -3; /* goal unreachable at this radius */
    int rev[MAXN], k = 0, c = n - 1;
    while (c != -1) { rev[k++] = c; c = prev[c]; }
    if (k > cap) return -2;
    for (int t = 0; t < k; t++) out[t] = path[rev[k - 1 - t]];
    return k;
}

static double poly_len(const int *p, int n) {
    double L = 0.0;
    for (int i = 0; i + 1 < n; i++)
        L += hypot((p[i + 1] % W) - (p[i] % W),
                   (p[i + 1] / W) - (p[i] / W));
    return L;
}

static double worst_clearance(const Map *m, const int *p, int n) {
    double c = 1e30;
    for (int i = 0; i + 1 < n; i++)
        c = min2(c, seg_clearance(m, p[i] % W, p[i] / W,
                                  p[i + 1] % W, p[i + 1] / W));
    return c;
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* the logged case: 14 moves, 13 turns, cost 14, (0,0) -> (12,7).
 * 7 E, 5 SE, 2 S, alternating direction on every move. */
static const int MOVES[14][2] = {
    {1,0},{1,1},{1,0},{1,1},{1,0},{0,1},{1,0},
    {1,1},{1,0},{1,1},{1,0},{0,1},{1,0},{1,1},
};

static int build_case(Map *m, int *path) {
    memset(m, 0, sizeof(*m));
    int x = 0, y = 0, n = 0;
    path[n++] = idx(x, y);
    int turns = 0;
    for (int s = 0; s < 14; s++) {
        if (s > 0 && (MOVES[s][0] != MOVES[s - 1][0] ||
                      MOVES[s][1] != MOVES[s - 1][1]))
            turns++;
        x += MOVES[s][0];
        y += MOVES[s][1];
        path[n++] = idx(x, y);
    }
    /* two pillars the (0,0)->(12,7) center chord crosses, each keeping
     * >= 0.5 cell clearance from every raw staircase segment */
    m->blocked[idx(4, 3)] = 1;
    m->blocked[idx(9, 6)] = 1;
    check("case_goal", x == 12 && y == 7);
    check("case_len14", n == 15);
    check("case_turns13", turns == 13);
    for (int i = 0; i < n; i++)
        if (m->blocked[path[i]]) check("case_path_walkable", 0);
    check("case_path_walkable", 1);
    return n;
}

int main(void) {
    /* ---------- basic geometric LOS ---------- */
    Map o; memset(&o, 0, sizeof(o));
    check("los_open", los(&o, 0, 0, 5, 0));
    check("los_diag", los(&o, 0, 0, 4, 4));

    Map w; memset(&w, 0, sizeof(w));
    w.blocked[idx(2, 0)] = 1;
    check("los_blocked", !los(&w, 0, 0, 4, 0));

    /* radius margin: passing within 0.5 of a wall fails r=0.6, passes r=0 */
    check("los_radius_ok", los_r(&w, 0, 1, 4, 1, 0.4));
    check("los_radius_cut", !los_r(&w, 0, 1, 4, 1, 0.6));
    check("los_oob", !los(&w, -1, 0, 4, 0));

    /* ---------- regression: Bresenham misses crossed cells ---------- */
    Map b; memset(&b, 0, sizeof(b));
    b.blocked[idx(4, 3)] = 1;
    b.blocked[idx(9, 6)] = 1;
    check("bresenham_misses_pillars",
          los_bresenham(&b, 0, 0, 12, 7) == 1);
    check("exact_los_catches_pillars", !los(&b, 0, 0, 12, 7));
    check("exact_zero_clearance",
          fabs(seg_clearance(&b, 0, 0, 12, 7)) < 1e-9);
    check("sampled_agrees_zero",
          fabs(seg_clearance_sampled(&b, 0, 0, 12, 7)) < 1e-6);

    /* ---------- the logged staircase path ---------- */
    Map m; int path[MAXN], out[MAXN], dp[MAXN];
    int n = build_case(&m, path);

    /* every raw segment must have clearance >= 0.5 */
    double raw_worst = worst_clearance(&m, path, n);
    check("raw_clearance_half", raw_worst >= 0.5 - 1e-9);

    /* r=0: the direct chord pierces the pillars, so the minimum is a
     * 3-point detour hugging them (clearance > 0 but tiny) */
    int k0 = smooth_min_points(&m, path, n, dp, MAXN, 0.0);
    int g0 = smooth_greedy(&m, path, n, out, MAXN, 0.0);
    check("r0_min_three", k0 == 3);
    check("r0_greedy_matches_dp", g0 == k0);
    check("r0_ends", dp[0] == path[0] && dp[k0 - 1] == path[n - 1]);
    double r0w = worst_clearance(&m, dp, k0);
    check("r0_tight_but_positive", r0w > 0.0 && r0w < 0.1);

    /* sweep: points and clearance vs smoothing strength (character radius) */
    printf("\nradius points poly_len worst_clear\n");
    double R = 0.0;
    int prev_k = -1;
    while (R <= 0.5001) {
        int k = smooth_min_points(&m, path, n, dp, MAXN, R);
        int g = smooth_greedy(&m, path, n, out, MAXN, R);
        if (k < 0) {
            printf("%.2f INFEASIBLE\n", R);
        } else {
            printf("%.2f %d %.4f %.4f%s\n", R, k, poly_len(dp, k),
                   worst_clearance(&m, dp, k),
                   g == k ? "" : " GREEDY!=DP");
            check("greedy_optimal", g == k);
        }
        if (prev_k >= 0 && k >= 0 && prev_k >= 0)
            check("points_non_decreasing", k >= prev_k);
        prev_k = k;
        R += 0.05;
    }

    /* ---------- acceptance at the chosen strength r = 0.35 ---------- */
    const double RCHOSEN = 0.35;
    int k = smooth_min_points(&m, path, n, dp, MAXN, RCHOSEN);
    int kg = smooth_greedy(&m, path, n, out, MAXN, RCHOSEN);
    check("chosen_greedy_matches_dp", kg == k);
    check("chosen_below_one_third", k < n / 3.0);          /* 4 < 5 */
    check("chosen_points_four", k == 4);

    /* every kept segment passes radius LOS; report any failing index */
    int bad_seg = -1;
    for (int i = 0; i + 1 < k; i++) {
        if (!los_r(&m, dp[i] % W, dp[i] / W,
                   dp[i + 1] % W, dp[i + 1] / W, RCHOSEN)) {
            bad_seg = i;
            break;
        }
    }
    check("chosen_all_segments_pass", bad_seg == -1);
    if (bad_seg >= 0) printf("failing_segment_index=%d\n", bad_seg);

    /* which of the 14 raw segments were absorbed by chords */
    int kept_edge[14] = {0};
    for (int t = 1; t < k; t++) {
        int cell_index = 0;
        for (int q = 0; q < n; q++) if (path[q] == dp[t]) { cell_index = q; break; }
        kept_edge[cell_index - 1] = 1;
    }
    int removed = 0;
    printf("removed_raw_segments=");
    for (int s = 0; s < 14; s++)
        if (!kept_edge[s]) { printf("%d ", s); removed++; }
    printf("(count=%d)\n", removed);
    check("removed_eleven", removed == 11);

    double cw = worst_clearance(&m, dp, k);
    check("chosen_clearance_above_radius", cw > RCHOSEN);
    double cws = 1e30;
    for (int i = 0; i + 1 < k; i++)
        cws = min2(cws, seg_clearance_sampled(&m, dp[i] % W, dp[i] / W,
                                              dp[i + 1] % W,
                                              dp[i + 1] / W));
    double measure_err = fabs(cw - cws);
    check("distance_error_under_half_cell", measure_err <= 0.5);
    printf("worst_clearance=%.4f sampled=%.4f error=%.6f\n",
           cw, cws, measure_err);

    /* ---------- per-segment LOS report ---------- */
    printf("\nraw 14 segments (r=0.35):\n");
    printf(" seg  from->to        clearance  LOS   status\n");
    for (int s = 0; s < 14; s++) {
        double d = seg_clearance(&m, path[s] % W, path[s] / W,
                                 path[s + 1] % W, path[s + 1] / W);
        int ok = los_r(&m, path[s] % W, path[s] / W,
                       path[s + 1] % W, path[s + 1] / W, RCHOSEN);
        printf(" %2d   (%2d,%2d)->(%2d,%2d)  %.4f     %s   %s\n",
               s, path[s] % W, path[s] / W,
               path[s + 1] % W, path[s + 1] / W, d,
               ok ? "pass" : "FAIL",
               kept_edge[s] ? "kept" : "skip->chord");
    }
    printf("\nsmoothed %d segments (r=0.35):\n", k - 1);
    printf(" seg  from->to        clearance  LOS   >=r\n");
    for (int t = 0; t + 1 < k; t++) {
        double d = seg_clearance(&m, dp[t] % W, dp[t] / W,
                                 dp[t + 1] % W, dp[t + 1] / W);
        int ok = los_r(&m, dp[t] % W, dp[t] / W,
                       dp[t + 1] % W, dp[t + 1] / W, RCHOSEN);
        printf(" %2d   (%2d,%2d)->(%2d,%2d)  %.4f     %s   %s\n",
               t, dp[t] % W, dp[t] / W,
               dp[t + 1] % W, dp[t + 1] / W, d,
               ok ? "pass" : "FAIL", d > RCHOSEN ? "yes" : "no");
        check("reported_segment_passes", ok);
    }

    /* ---------- original-style API checks ---------- */
    int small[1];
    check("empty_rejected",
          smooth_min_points(&o, path, 0, out, MAXN, 0.0) == -1);
    check("cap_rejected",
          smooth_min_points(&o, path, n, small, 1, 0.0) == -2);
    check("clear_map_collapses",
          smooth_min_points(&o, path, n, dp, MAXN, 0.0) == 2);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
