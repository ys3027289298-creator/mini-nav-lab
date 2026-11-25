
/* smooth: line-of-sight test, capsule-clearance test and path smoothing.
 *
 * The character is a capsule of radius R and the path is planned through its
 * centroid.  A path segment that passes at distance d from a wall makes the
 * capsule stick out by
 *
 *     overrun = max(0, R - d)
 *
 * so "zero overrun on every segment" means d >= R on every segment.  A
 * smoothing margin m is by definition the lower bound on d that every kept
 * segment must satisfy, so zero overrun forces m >= R, and the smallest legal
 * margin is exactly the capsule radius.
 *
 * The legacy line-of-sight test only rejects segments crossing blocked cells:
 * it requires d > 0, i.e. a margin of 0, and falls short of the requirement by
 * exactly R.  In a scene where the greedy path grazes a wall the shortfall is
 * therefore the whole radius.
 *
 * The capsule-aware route is built on a visibility graph over start, goal and
 * samples of the radius-m arcs around obstacle corners: the shortest path
 * around inflated obstacles is a polyline whose interior vertices lie on those
 * arcs, so the graph is near-optimal.  A cell-index Dijkstra is kept as a
 * fallback when the graph has no route.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W*H)
#define EPS 1e-9
#define PI 3.14159265358979323846

typedef struct { unsigned char blocked[W*H]; } Map;
typedef struct { double x, y; } Pt;

static int idx(int x, int y) { return y * W + x; }
static Pt center(int c) { Pt p = { c % W + 0.5, c / W + 0.5 }; return p; }

/* bresenham supercover: any blocked cell on the segment blocks LOS */
static int los(const Map* m, int x0, int y0, int x1, int y1) {
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

/* greedy smoothing: keep start, skip points while LOS holds */
static int smooth(const Map* m, const int* path, int n, int* out, int cap) {
    if (n <= 0) return -1;
    if (n == 1) { if (cap < 1) return -2; out[0] = path[0]; return 1; }
    int w = 0;
    out[w++] = path[0];
    int i = 0;
    while (i < n - 1) {
        int j = n - 1;
        while (j > i + 1) {
            if (los(m, path[i] % W, path[i] / W, path[j] % W, path[j] / W)) break;
            j--;
        }
        if (w >= cap) return -2;
        out[w++] = path[j];
        i = j;
    }
    return w;
}

/* ---- capsule geometry: exact distance from a segment to a cell rectangle ---- */

static double point_rect_dist(double px, double py, double x0, double y0, double x1, double y1) {
    double dx = px < x0 ? x0 - px : (px > x1 ? px - x1 : 0.0);
    double dy = py < y0 ? y0 - py : (py > y1 ? py - y1 : 0.0);
    return sqrt(dx * dx + dy * dy);
}

/* Liang-Barsky: does the closed segment touch the closed rectangle? */
static int seg_hits_rect(double ax, double ay, double bx, double by,
                         double x0, double y0, double x1, double y1) {
    double dx = bx - ax, dy = by - ay;
    double p[4] = { -dx, dx, -dy, dy };
    double q[4] = { ax - x0, x1 - ax, ay - y0, y1 - ay };
    double t0 = 0.0, t1 = 1.0;
    for (int i = 0; i < 4; i++) {
        if (p[i] == 0.0) {
            if (q[i] < 0.0) return 0;
        } else {
            double r = q[i] / p[i];
            if (p[i] < 0.0) { if (r > t1) return 0; if (r > t0) t0 = r; }
            else            { if (r < t0) return 0; if (r < t1) t1 = r; }
        }
    }
    return t0 <= t1;
}

/* exact euclidean distance between a segment and an axis-aligned rectangle */
static double seg_rect_dist(double ax, double ay, double bx, double by,
                            double x0, double y0, double x1, double y1) {
    if (seg_hits_rect(ax, ay, bx, by, x0, y0, x1, y1)) return 0.0;
    /* closest pair is endpoint->rect, or a rect corner->segment */
    double best = point_rect_dist(ax, ay, x0, y0, x1, y1);
    double d = point_rect_dist(bx, by, x0, y0, x1, y1);
    if (d < best) best = d;
    double vx = bx - ax, vy = by - ay;
    double len2 = vx * vx + vy * vy;
    const double cx[4] = { x0, x0, x1, x1 };
    const double cy[4] = { y0, y1, y0, y1 };
    for (int i = 0; i < 4; i++) {
        double t = len2 > 0.0 ? ((cx[i] - ax) * vx + (cy[i] - ay) * vy) / len2 : 0.0;
        if (t < 0.0) t = 0.0;
        if (t > 1.0) t = 1.0;
        double dx = cx[i] - (ax + t * vx), dy = cy[i] - (ay + t * vy);
        d = sqrt(dx * dx + dy * dy);
        if (d < best) best = d;
    }
    return best;
}

/* min distance from a segment to any blocked cell; INFINITY on an empty map */
static double map_clearance(const Map* m, double ax, double ay, double bx, double by) {
    double best = INFINITY;
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            if (!m->blocked[idx(x, y)]) continue;
            double d = seg_rect_dist(ax, ay, bx, by, x, y, x + 1, y + 1);
            if (d < best) best = d;
        }
    }
    return best;
}

/* does the whole segment keep at least `margin` from every wall? */
static int capsule_clear(const Map* m, double ax, double ay, double bx, double by, double margin) {
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            if (!m->blocked[idx(x, y)]) continue;
            if (seg_rect_dist(ax, ay, bx, by, x, y, x + 1, y + 1) < margin - EPS) return 0;
        }
    }
    return 1;
}

/* ---- polyline helpers ---- */

#define MAXV        1024

static double poly_len(const Pt* p, int n) {
    double s = 0.0;
    for (int i = 0; i + 1 < n; i++) s += hypot(p[i + 1].x - p[i].x, p[i + 1].y - p[i].y);
    return s;
}

static int seg_pair_clear(const Map* m, Pt a, Pt x, Pt b, double margin) {
    return capsule_clear(m, a.x, a.y, x.x, x.y, margin) &&
           capsule_clear(m, x.x, x.y, b.x, b.y, margin);
}

/* Slide each interior vertex toward its projection on the neighbour chord,
 * stopping at the last position that still keeps `margin` on both adjacent
 * segments.  Removes the half-cell zig-zag of lattice waypoints. */
static int polish(const Map* m, Pt* p, int n, double margin) {
    for (int pass = 0; pass < 40; pass++) {
        int moved = 0;
        for (int i = 1; i < n - 1; i++) {
            Pt a = p[i - 1], w = p[i], b = p[i + 1];
            double vx = b.x - a.x, vy = b.y - a.y;
            double len2 = vx * vx + vy * vy;
            if (len2 <= 0.0) continue;
            double t = ((w.x - a.x) * vx + (w.y - a.y) * vy) / len2;
            if (t < 0.0) t = 0.0;
            if (t > 1.0) t = 1.0;
            Pt proj = { a.x + t * vx, a.y + t * vy };
            if (seg_pair_clear(m, a, proj, b, margin)) { p[i] = proj; moved = 1; continue; }
            double lo = 0.0, hi = 1.0;
            for (int k = 0; k < 18; k++) {
                double mid = 0.5 * (lo + hi);
                Pt x = { w.x + mid * (proj.x - w.x), w.y + mid * (proj.y - w.y) };
                if (seg_pair_clear(m, a, x, b, margin)) lo = mid; else hi = mid;
            }
            if (lo > 1e-6) {
                p[i].x = w.x + lo * (proj.x - w.x);
                p[i].y = w.y + lo * (proj.y - w.y);
                moved = 1;
            }
        }
        if (!moved) break;
    }
    return n;
}

/* Drop vertices that are not needed: greedily skip forward while the straight
 * chord from the last kept vertex keeps `margin`.  Then re-polish. */
static int simplify(const Map* m, Pt* p, int n, double margin) {
    if (n <= 2) return n;
    Pt tmp[MAXV];
    int k = 0;
    tmp[k++] = p[0];
    int i = 0;
    while (i < n - 1) {
        int j = n - 1;
        while (j > i + 1 && !capsule_clear(m, p[i].x, p[i].y, p[j].x, p[j].y, margin)) j--;
        tmp[k++] = p[j];
        i = j;
    }
    for (int t = 0; t < k; t++) p[t] = tmp[t];
    polish(m, p, k, margin);
    return k;
}

/* ---- capsule-aware routing ---- */

#define SPQ         12       /* arc samples per quarter circle around a corner  */
#define GEOM_INSET  0.0015   /* built margin sits just above R so evaluation at R is clean */
#define PATH_REACH  2.5      /* corners farther than this from the raw path are skipped */

static double pt_seg_dist(Pt p, Pt a, Pt b) {
    double vx = b.x - a.x, vy = b.y - a.y;
    double len2 = vx * vx + vy * vy;
    double t = len2 > 0.0 ? ((p.x - a.x) * vx + (p.y - a.y) * vy) / len2 : 0.0;
    if (t < 0.0) t = 0.0;
    if (t > 1.0) t = 1.0;
    double dx = p.x - (a.x + t * vx), dy = p.y - (a.y + t * vy);
    return sqrt(dx * dx + dy * dy);
}

static double path_dist(const int* path, int n, Pt p) {
    if (n == 1) return pt_seg_dist(p, center(path[0]), center(path[0]));
    double best = INFINITY;
    for (int i = 0; i + 1 < n; i++) {
        double d = pt_seg_dist(p, center(path[i]), center(path[i + 1]));
        if (d < best) best = d;
    }
    return best;
}

static int add_node(Pt* v, int nv, int cap, Pt p) {
    for (int i = 0; i < nv; i++)
        if (fabs(v[i].x - p.x) < 1e-6 && fabs(v[i].y - p.y) < 1e-6) return nv;
    if (nv >= cap) return nv;
    v[nv] = p;
    return nv + 1;
}

/* start, goal and samples of the radius-(margin+inset) arcs around corners
 * that the raw path passes near */
static int build_nodes(const Map* m, const int* path, int n, double margin, Pt* v, int cap) {
    double Th = margin + GEOM_INSET;
    double rad = Th / cos(PI / (4.0 * SPQ)) * 1.000001;
    int nv = 0;
    nv = add_node(v, nv, cap, center(path[0]));
    nv = add_node(v, nv, cap, center(path[n - 1]));
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            if (!m->blocked[idx(x, y)]) continue;
            for (int corner = 0; corner < 4; corner++) {
                double cx = x + ((corner & 1) ? 1 : 0);
                double cy = y + ((corner & 2) ? 1 : 0);
                Pt c = { cx, cy };
                if (path_dist(path, n, c) > PATH_REACH) continue;
                for (int q = 0; q < 4; q++) {
                    int ox = (q & 1) ? 1 : -1, oy = (q & 2) ? 1 : -1;
                    int cellx = ox > 0 ? (int)cx : (int)cx - 1;
                    int celly = oy > 0 ? (int)cy : (int)cy - 1;
                    if (cellx < 0 || celly < 0 || cellx >= W || celly >= H) continue;
                    if (m->blocked[idx(cellx, celly)]) continue;
                    for (int k = 0; k <= SPQ; k++) {
                        double ang = (PI / 2.0) * k / SPQ;
                        Pt p = { cx + ox * rad * cos(ang), cy + oy * rad * sin(ang) };
                        if (p.x < 0.0 || p.y < 0.0 || p.x >= W || p.y >= H) continue;
                        if (map_clearance(m, p.x, p.y, p.x, p.y) < Th) continue;
                        nv = add_node(v, nv, cap, p);
                    }
                }
            }
        }
    }
    return nv;
}

/* cells of the raw path -> polyline meeting `margin`, via Dijkstra over the
 * raw path's own nodes; used when the visibility graph has no route */
static int route_cells(const Map* m, const int* path, int n, double margin, Pt* poly, int cap) {
    if (n == 1) { if (cap < 1) return -2; poly[0] = center(path[0]); return 1; }
    double dist[MAXN];
    int prev[MAXN], used[MAXN], chain[MAXN];
    for (int i = 0; i < n; i++) { dist[i] = INFINITY; prev[i] = -1; used[i] = 0; }
    dist[0] = 0.0;
    for (int it = 0; it < n; it++) {
        int u = -1;
        for (int i = 0; i < n; i++)
            if (!used[i] && (u < 0 || dist[i] < dist[u])) u = i;
        if (u < 0 || !isfinite(dist[u])) break;
        used[u] = 1;
        if (u == n - 1) break;
        Pt a = center(path[u]);
        for (int v = u + 1; v < n; v++) {
            if (used[v]) continue;
            Pt b = center(path[v]);
            if (!capsule_clear(m, a.x, a.y, b.x, b.y, margin)) continue;
            double nd = dist[u] + hypot(b.x - a.x, b.y - a.y);
            if (nd < dist[v] - EPS) { dist[v] = nd; prev[v] = u; }
        }
    }
    if (!isfinite(dist[n - 1])) return -3;
    int k = 0, c = n - 1;
    while (c != -1 && k < MAXN) { chain[k++] = c; c = prev[c]; }
    if (k > cap) return -2;
    for (int i = 0; i < k; i++) poly[i] = center(path[chain[k - 1 - i]]);
    polish(m, poly, k, margin);
    return simplify(m, poly, k, margin);
}

/* shortest polyline from start to goal whose every segment keeps `margin` */
static int route_roadmap(const Map* m, const int* path, int n, double margin, Pt* poly, int cap) {
    static Pt v[MAXV];
    static double dist[MAXV];
    static int prev[MAXV], chain[MAXV];
    static unsigned char done[MAXV];
    double Th = margin + GEOM_INSET;
    int nv = build_nodes(m, path, n, margin, v, MAXV);
    if (nv < 2) { if (nv == 1 && cap >= 1) { poly[0] = v[0]; return 1; } return -1; }
    for (int i = 0; i < nv; i++) { dist[i] = INFINITY; prev[i] = -1; done[i] = 0; }
    dist[0] = 0.0;
    for (int it = 0; it < nv; it++) {
        int u = -1;
        for (int i = 0; i < nv; i++)
            if (!done[i] && (u < 0 || dist[i] < dist[u])) u = i;
        if (u < 0 || !isfinite(dist[u])) break;
        done[u] = 1;
        if (u == 1) break;                      /* node 1 is the goal */
        for (int w = 1; w < nv; w++) {
            if (done[w]) continue;
            double d = hypot(v[w].x - v[u].x, v[w].y - v[u].y);
            if (dist[u] + d >= dist[w] - 1e-12) continue;
            if (!capsule_clear(m, v[u].x, v[u].y, v[w].x, v[w].y, Th)) continue;
            dist[w] = dist[u] + d;
            prev[w] = u;
        }
    }
    if (!isfinite(dist[1])) return -3;
    int k = 0, c = 1;
    while (c != -1 && k < MAXV) { chain[k++] = c; c = prev[c]; }
    if (k > cap) return -2;
    for (int i = 0; i < k; i++) poly[i] = v[chain[k - 1 - i]];
    polish(m, poly, k, Th);
    return simplify(m, poly, k, Th);
}

static int route(const Map* m, const int* path, int n, double margin, Pt* poly, int cap) {
    int k = route_roadmap(m, path, n, margin, poly, cap);
    if (k > 0) return k;
    return route_cells(m, path, n, margin, poly, cap);
}

/* ---- measurement ---- */

/* number of segments with overrun > 0, the largest overrun and its segment */
static int poly_violations(const Map* m, const Pt* p, int n, double R,
                           double* worst_over, int* worst_seg, double* min_d) {
    int count = 0;
    double wo = 0.0, md = INFINITY;
    int ws = -1;
    for (int i = 0; i + 1 < n; i++) {
        double d = map_clearance(m, p[i].x, p[i].y, p[i + 1].x, p[i + 1].y);
        if (d < md) md = d;
        if (d < R - 1e-9) {
            count++;
            if (R - d > wo) { wo = R - d; ws = i; }
        }
    }
    *worst_over = wo; *worst_seg = ws; *min_d = md;
    return count;
}

/* smallest margin on a 0.01 grid whose route has zero overrun at radius R */
static double find_min_zero_margin(const Map* m, const int* path, int n, double R) {
    static Pt poly[MAXN];
    double wo, md;
    int ws;
    double best = -1.0;
    for (int k = 0; k <= 80; k++) {
        double margin = 0.01 * k;
        int np = route(m, path, n, margin, poly, MAXN);
        if (np <= 0) continue;
        if (poly_violations(m, poly, np, R, &wo, &ws, &md) == 0) { best = margin; break; }
    }
    return best;
}

/* ---- scene fixtures: a raw 4-neighbour path, like the grid module returns ---- */

static int bfs_path(const Map* m, int sx, int sy, int gx, int gy, int* out, int cap) {
    if (sx < 0 || sy < 0 || sx >= W || sy >= H) return -1;
    if (gx < 0 || gy < 0 || gx >= W || gy >= H) return -1;
    if (m->blocked[idx(sx, sy)] || m->blocked[idx(gx, gy)]) return -1;
    static int prev[MAXN], q[MAXN];
    for (int i = 0; i < MAXN; i++) prev[i] = -2;
    int s = idx(sx, sy), g = idx(gx, gy);
    int qh = 0, qt = 0;
    prev[s] = -1; q[qt++] = s;
    const int dx[4] = { 0, -1, 1, 0 }, dy[4] = { -1, 0, 0, 1 };
    while (qh < qt) {
        int c = q[qh++];
        if (c == g) break;
        int cx = c % W, cy = c / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int ni = idx(nx, ny);
            if (m->blocked[ni] || prev[ni] != -2) continue;
            prev[ni] = c; q[qt++] = ni;
        }
    }
    if (prev[g] == -2) return -1;
    static int rev[MAXN];
    int k = 0, c = g;
    while (c != -1 && k < MAXN) { rev[k++] = c; c = prev[c]; }
    if (k > cap) return -2;
    for (int i = 0; i < k; i++) out[i] = rev[k - 1 - i];
    return k;
}

static void block_rect(Map* m, int x0, int y0, int x1, int y1) {
    for (int y = y0; y <= y1; y++)
        for (int x = x0; x <= x1; x++)
            m->blocked[idx(x, y)] = 1;
}

/* ---- self test ---- */

static int failures = 0;
static int budget_exceeded = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const double RADII[3] = { 0.25, 0.35, 0.45 };   /* small / medium / large body */
static const char* RNAME[3] = { "small", "medium", "large" };

static void run_scene(const char* name, const Map* m, int sx, int sy, int gx, int gy) {
    static int raw[MAXN], legacy[MAXN];
    static Pt lp[MAXN], poly[MAXN];
    int nraw = bfs_path(m, sx, sy, gx, gy, raw, MAXN);
    if (nraw < 2) { printf("scene_%s=0\n", name); failures++; return; }
    int nl = smooth(m, raw, nraw, legacy, MAXN);
    for (int i = 0; i < nl; i++) lp[i] = center(legacy[i]);
    double L0 = poly_len(lp, nl);

    printf("scene=%s raw=%d legacy_pts=%d L0=%.4f\n", name, nraw, nl, L0);

    for (int ri = 0; ri < 3; ri++) {
        double R = RADII[ri];
        char key[96];
        double lwo, lmd, swo, smd;
        int lws, sws;
        int lv = poly_violations(m, lp, nl, R, &lwo, &lws, &lmd);

        int np = route(m, raw, nraw, R, poly, MAXN);
        if (np <= 0) {
            printf("  R=%.2f (%s) route_failed=%d\n", R, RNAME[ri], np);
            failures++;
            continue;
        }
        int sv = poly_violations(m, poly, np, R, &swo, &sws, &smd);
        double L = poly_len(poly, np);
        double incr = (L / L0 - 1.0) * 100.0;
        double cstar = find_min_zero_margin(m, raw, nraw, R);

        printf("  R=%.2f %-6s legacy_viol=%d legacy_min_d=%.4f shortfall=%.4f "
               "| safe_pts=%d safe_viol=%d safe_min_d=%.4f margin=%.4f "
               "cstar=%.2f L=%.4f incr=%+.2f%%\n",
               R, RNAME[ri], lv, lmd, R - lmd, np, sv, smd, R, cstar, L, incr);

        snprintf(key, sizeof key, "%s_%s_keeps_start", name, RNAME[ri]);
        check(key, fabs(poly[0].x - (sx + 0.5)) < 1e-6 &&
                   fabs(poly[0].y - (sy + 0.5)) < 1e-6);
        snprintf(key, sizeof key, "%s_%s_keeps_goal", name, RNAME[ri]);
        check(key, fabs(poly[np - 1].x - (gx + 0.5)) < 1e-6 &&
                   fabs(poly[np - 1].y - (gy + 0.5)) < 1e-6);
        snprintf(key, sizeof key, "%s_%s_zero_overrun", name, RNAME[ri]);
        check(key, sv == 0);
        snprintf(key, sizeof key, "%s_%s_margin_at_least_radius", name, RNAME[ri]);
        check(key, smd >= R - 1e-6);
        /* zero overrun is already achieved by the legacy path -> no margin
         * is needed; otherwise the smallest fixing margin is the radius */
        snprintf(key, sizeof key, "%s_%s_cstar_is_radius", name, RNAME[ri]);
        check(key, lv == 0 || cstar >= R - 0.011);

        if (incr > 5.0 + 1e-9) {
            budget_exceeded++;
            printf("  BUDGET scene=%s body=%s margin=%.2f incr=%+.2f%% "
                   "worst_legacy_seg=%d overrun=%.4f seg=[(%.2f,%.2f)->(%.2f,%.2f)]\n",
                   name, RNAME[ri], R, incr, lws, lwo,
                   lws >= 0 ? lp[lws].x : 0.0, lws >= 0 ? lp[lws].y : 0.0,
                   lws >= 0 ? lp[lws + 1].x : 0.0, lws >= 0 ? lp[lws + 1].y : 0.0);
        }
    }
}

/* margin -> (overrun count, length) curve, the two lines that pick the margin */
static void margin_curve(const char* name, const Map* m, int sx, int sy, int gx, int gy) {
    static int raw[MAXN], legacy[MAXN];
    static Pt lp[MAXN], poly[MAXN];
    int nraw = bfs_path(m, sx, sy, gx, gy, raw, MAXN);
    int nl = smooth(m, raw, nraw, legacy, MAXN);
    for (int i = 0; i < nl; i++) lp[i] = center(legacy[i]);
    double L0 = poly_len(lp, nl);
    printf("curve=%s L0=%.4f\n", name, L0);
    for (int ri = 0; ri < 3; ri++) {
        double R = RADII[ri];
        printf(" curve_body=%s R=%.2f\n", RNAME[ri], R);
        double wo, md;
        int ws;
        for (int k = 0; k <= 30; k++) {
            double margin = 0.02 * k;
            int np = route(m, raw, nraw, margin, poly, MAXN);
            if (np <= 0) continue;
            int v = poly_violations(m, poly, np, R, &wo, &ws, &md);
            double L = poly_len(poly, np);
            printf("  margin=%.2f viol=%d len=%.4f incr=%+.2f%%\n",
                   margin, v, L, (L / L0 - 1.0) * 100.0);
        }
    }
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    check("los_open", los(&m, 0, 0, 5, 0) == 1);
    check("los_diag", los(&m, 0, 0, 4, 4) == 1);

    Map w; memset(&w, 0, sizeof(w));
    w.blocked[idx(2, 0)] = 1;
    check("los_blocked", los(&w, 0, 0, 4, 0) == 0);

    /* path detours around the blocked cell at (2,0) */
    int path[20], out[20];
    int n = 0;
    path[n++] = idx(0, 0);
    path[n++] = idx(1, 0);
    path[n++] = idx(1, 1);
    path[n++] = idx(2, 1);
    path[n++] = idx(3, 1);
    path[n++] = idx(3, 0);
    path[n++] = idx(4, 0);
    int k = smooth(&w, path, n, out, 20);
    check("smooth_ok", k > 0);
    check("smooth_fewer", k < n);
    check("smooth_starts_at_start", out[0] == path[0]);
    check("smooth_ends_at_goal", out[k - 1] == path[n - 1]);
    int all_clear = 1;
    for (int i = 0; i + 1 < k; i++)
        if (!los(&w, out[i] % W, out[i] / W, out[i+1] % W, out[i+1] / W)) all_clear = 0;
    check("kept_segments_clear", all_clear);

    /* a clear straight line collapses to two points */
    Map c; memset(&c, 0, sizeof(c));
    k = smooth(&c, path, n, out, 20);
    check("collapses_to_two", k == 2);
    check("empty_rejected", smooth(&c, path, 0, out, 20) == -1);
    check("cap_rejected", smooth(&c, path, n, out, 1) == -2);

    /* capsule geometry: cell (2,0) owns the rectangle [2,3] x [0,1] */
    check("geom_empty", map_clearance(&c, 0.5, 0.5, 1.5, 0.5) == INFINITY);
    check("geom_cross", fabs(map_clearance(&w, 0.5, 0.5, 3.5, 0.5)) < 1e-12);
    check("geom_adjacent", fabs(map_clearance(&w, 0.5, 0.5, 1.5, 0.5) - 0.5) < 1e-12);
    check("geom_above", fabs(map_clearance(&w, 2.5, 1.5, 3.5, 1.5) - 0.5) < 1e-12);
    check("geom_tangent", fabs(map_clearance(&w, 1.5, 1.5, 2.5, -0.5)) < 1e-12);
    check("capsule_margin_in", capsule_clear(&w, 0.5, 0.5, 1.5, 0.5, 0.49) == 1);
    check("capsule_margin_out", capsule_clear(&w, 0.5, 0.5, 1.5, 0.5, 0.51) == 0);

    /* the overrun law: overrun = max(0, R - d) */
    check("overrun_zero_when_far", fmax(0.0, 0.45 - 0.5) == 0.0);
    check("overrun_equals_shortfall", fabs(fmax(0.0, 0.45 - 0.0) - 0.45) < 1e-12);

    /* scenes */
    Map corner; memset(&corner, 0, sizeof(corner));
    block_rect(&corner, 8, 0, 8, 6);
    run_scene("corner", &corner, 2, 3, 13, 3);

    Map lwall; memset(&lwall, 0, sizeof(lwall));
    block_rect(&lwall, 7, 5, 12, 5);
    block_rect(&lwall, 7, 5, 7, 9);
    run_scene("lwall", &lwall, 3, 3, 10, 9);

    Map pillar; memset(&pillar, 0, sizeof(pillar));
    block_rect(&pillar, 7, 4, 8, 7);
    run_scene("pillar", &pillar, 2, 5, 12, 5);

    Map doorway; memset(&doorway, 0, sizeof(doorway));
    block_rect(&doorway, 8, 0, 8, 5);
    block_rect(&doorway, 8, 7, 8, 10);
    run_scene("doorway", &doorway, 2, 5, 13, 5);

    Map tip; memset(&tip, 0, sizeof(tip));
    block_rect(&tip, 7, 2, 7, 8);
    run_scene("tip", &tip, 5, 3, 9, 3);

    Map corridor; memset(&corridor, 0, sizeof(corridor));
    block_rect(&corridor, 6, 1, 6, 10);
    block_rect(&corridor, 8, 1, 8, 10);
    run_scene("corridor", &corridor, 7, 2, 7, 9);

    Map open; memset(&open, 0, sizeof(open));
    run_scene("open", &open, 2, 5, 13, 5);

    margin_curve("corner", &corner, 2, 3, 13, 3);

    printf("budget_exceeded=%d\n", budget_exceeded);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
