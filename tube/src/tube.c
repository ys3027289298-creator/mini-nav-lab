
/* tube: global polyline / local avoidance boundary layer.
 *
 * Responsibility split:
 *   global  -> a walkable multi-segment polyline Gamma;
 *   local   -> offsets the agent centre only inside the radius-R tube of Gamma.
 *
 * With agent radius r and radius-external offset budget eps:
 *
 *   R = r + eps
 *   T_R(Gamma) = { p : dist(p, Gamma) <= R }
 *
 * A crowd cross-section of raw occupied width B needs the agent centre to move
 * by B/2 (obstacle half-width) + r + clearance.  The interface is exceeded when
 * that inflated one-sided occupation is larger than the tube half-width R:
 *
 *   B/2 + r + clearance > R
 *
 * so the geometric lower bound is
 *
 *   R >= R_lb = B_max/2 + r + clearance.
 *
 * The fixture below is a deterministic slalom crowd snapshot.  It is not a
 * claim about a specific shipped level; replace crowd_fixture() with real
 * ped/crowd cross-sections and feed the same sweep.
 */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#define AGENT_R     0.30
#define CLEARANCE   0.18
#define PED_R       0.26
#define DS          0.08
#define COMMIT_R    0.95
#define MAXS        512
#define MAXPED      64
#define RS_FIRST    0.55
#define RS_STEP     0.05
#define RS_N        22
#define SMOOTH_ITERS 1200
#define EPS         1e-9

typedef struct { double x, y; } Vec;
typedef struct { Vec a, b; double len, s0; } Seg;
typedef struct { double s, y, rad; int side; Vec pos; } Ped;

static Vec v(double x, double y) { Vec r = {x, y}; return r; }
static Vec add(Vec a, Vec b) { return v(a.x + b.x, a.y + b.y); }
static Vec scale(Vec a, double k) { return v(a.x * k, a.y * k); }
static double dot(Vec a, Vec b) { return a.x * b.x + a.y * b.y; }
static double norm(Vec a) { return hypot(a.x, a.y); }
static double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static const Vec GLOBAL[] = {
    {0.0, 0.0}, {7.0, 0.8}, {14.0, -0.55}, {21.0, 0.65},
    {28.0, -0.25}, {35.0, 0.0}
};
#define GN ((int)(sizeof(GLOBAL) / sizeof(GLOBAL[0])))

static Seg segs[16];
static int nseg;
static double path_len;

static void build_segments(void) {
    path_len = 0.0;
    nseg = GN - 1;
    for (int i = 0; i < nseg; i++) {
        segs[i].a = GLOBAL[i];
        segs[i].b = GLOBAL[i + 1];
        segs[i].len = norm(v(segs[i].b.x - segs[i].a.x,
                            segs[i].b.y - segs[i].a.y));
        segs[i].s0 = path_len;
        path_len += segs[i].len;
    }
}

static double point_seg_dist(Vec p, Vec a, Vec b) {
    Vec ab = v(b.x - a.x, b.y - a.y);
    double l2 = dot(ab, ab);
    if (l2 <= EPS) return norm(v(p.x - a.x, p.y - a.y));
    double t = dot(v(p.x - a.x, p.y - a.y), ab) / l2;
    t = clampd(t, 0.0, 1.0);
    Vec q = add(a, scale(ab, t));
    return norm(v(p.x - q.x, p.y - q.y));
}

static double polyline_dist(Vec p) {
    double best = 1e100;
    for (int i = 0; i < nseg; i++) {
        double d = point_seg_dist(p, segs[i].a, segs[i].b);
        if (d < best) best = d;
    }
    return best;
}

/* Frenet frame at arc length s. */
static void frame_at(double s, Vec *center, Vec *normal) {
    if (s < 0.0) s = 0.0;
    if (s > path_len) s = path_len;
    int i = nseg - 1;
    for (int j = 0; j < nseg; j++) {
        if (s <= segs[j].s0 + segs[j].len + EPS) { i = j; break; }
    }
    double u = (s - segs[i].s0) / segs[i].len;
    u = clampd(u, 0.0, 1.0);
    Seg g = segs[i];
    Vec tangent = v((g.b.x - g.a.x) / g.len, (g.b.y - g.a.y) / g.len);
    *center = add(g.a, scale(tangent, u * g.len));
    *normal = v(-tangent.y, tangent.x);
}

static Vec frenet_point(double s, double y) {
    Vec c, n;
    frame_at(s, &c, &n);
    return add(c, scale(n, y));
}

/* Symmetric raw crowd width B gives this hard geometric tube lower bound. */
static double radius_lower_bound(double crowd_width_b, double agent_r,
                                 double clearance) {
    return 0.5 * crowd_width_b + agent_r + clearance;
}

static int half_width_exceeded(double crowd_width_b, double R,
                               double agent_r, double clearance) {
    return radius_lower_bound(crowd_width_b, agent_r, clearance) > R + EPS;
}

typedef struct {
    int nped;
    Ped peds[MAXPED];
} Crowd;

/* density: 0 low, 1 medium, 2 high. */
static void crowd_fixture(int density, Crowd *c) {
    static const int clusters[3] = {3, 5, 8};
    static const int behind[3] = {2, 3, 4};
    static const double intrusion[3] = {-0.10, 0.04, 0.18};
    int nc = clusters[density];
    int nb = behind[density];
    double bias = intrusion[density];
    c->nped = 0;
    for (int k = 0; k < nc; k++) {
        double s = 4.0 + (double)k * (path_len - 8.0) / (double)(nc - 1);
        int side = (k % 2 == 0) ? 1 : -1;
        double y0 = side * bias;
        for (int j = 0; j < nb; j++) {
            if (c->nped >= MAXPED) return;
            Ped *p = &c->peds[c->nped++];
            p->s = s;
            p->y = y0 - side * j * 0.50;
            p->rad = PED_R;
            p->side = side;
            p->pos = frenet_point(p->s, p->y);
        }
    }
}

typedef struct {
    double R;
    int density;
    int nclusters;
    int nped;
    int demand_violations; /* connected safe-corridor episodes beyond tube */
    int outside_samples;   /* executed samples literally outside tube */
    int wall_samples;      /* executed samples saturated at tube boundary */
    double base_length;
    double length;
    double detour_pct;
    double max_offset;
    double min_clearance;
    double raw_demand;
} Result;

static int cluster_count(int density) {
    static const int clusters[3] = {3, 5, 8};
    return clusters[density];
}

static int cmp_double(const void *a, const void *b) {
    double x = *(const double *)a, y = *(const double *)b;
    return (x > y) - (x < y);
}

static Result run_case(int density, double R) {
    Result res;
    memset(&res, 0, sizeof(res));
    res.R = R;
    res.density = density;
    res.nclusters = cluster_count(density);

    Crowd crowd;
    crowd_fixture(density, &crowd);
    res.nped = crowd.nped;

    double candidates[MAXS], s[MAXS];
    int nc = 0;
    for (double x = 0.0; x <= path_len + 1e-10; x += DS) {
        if (nc < MAXS) candidates[nc++] = fmin(x, path_len);
    }
    if (nc < MAXS) candidates[nc++] = path_len;
    for (int i = 1; i < nseg; i++) {
        if (nc < MAXS) candidates[nc++] = segs[i].s0;
    }
    qsort(candidates, (size_t)nc, sizeof(double), cmp_double);

    int ns = 0;
    for (int i = 0; i < nc; i++) {
        if (ns == 0 || candidates[i] - s[ns - 1] > 1e-7) {
            s[ns++] = candidates[i];
        }
    }
    if (ns > MAXS) { fprintf(stderr, "MAXS too small\n"); exit(2); }

    double lo[MAXS], hi[MAXS], lb[MAXS], ub[MAXS], y[MAXS];
    Vec center[MAXS], normal[MAXS], q[MAXS];
    for (int k = 0; k < ns; k++) {
        frame_at(s[k], &center[k], &normal[k]);
        lo[k] = -1e6;
        hi[k] = 1e6;
        y[k] = 0.0;
    }

    /* Inflated circular obstacles become one-sided interval constraints in
     * the Frenet normal coordinate. */
    for (int pi = 0; pi < crowd.nped; pi++) {
        Ped p = crowd.peds[pi];
        double safe = p.rad + AGENT_R + CLEARANCE;
        for (int k = 0; k < ns; k++) {
            double ds = s[k] - p.s;
            if (fabs(ds) >= safe) continue;
            double half = sqrt(fmax(0.0, safe * safe - ds * ds));
            if (p.side > 0) {
                double bound = p.y + half;
                if (bound > lo[k]) lo[k] = bound;
            } else {
                double bound = p.y - half;
                if (bound < hi[k]) hi[k] = bound;
            }
        }
    }

    res.raw_demand = 0.0;
    for (int k = 0; k < ns; k++) {
        if (lo[k] > -1e5) res.raw_demand = fmax(res.raw_demand, fabs(lo[k]));
        if (hi[k] <  1e5) res.raw_demand = fmax(res.raw_demand, fabs(hi[k]));
        lb[k] = fmax(lo[k], -R);
        ub[k] = fmin(hi[k], R);
    }

    /* Count connected stations where the collision-free interval cannot fit
     * in the tube.  Consecutive stations are one avoidance event. */
    int in_bad = 0;
    for (int k = 0; k < ns; k++) {
        int bad = lb[k] > ub[k] + 1e-8;
        if (bad && !in_bad) res.demand_violations++;
        in_bad = bad;

        if (bad) {
            if (lo[k] > R + EPS) {
                lb[k] = ub[k] = R;
            } else if (hi[k] < -R - EPS) {
                lb[k] = ub[k] = -R;
            } else {
                double mid = 0.5 * (lo[k] + hi[k]);
                mid = clampd(mid, -R, R);
                lb[k] = ub[k] = mid;
            }
        }
        y[k] = clampd(0.0, lb[k], ub[k]);
    }

    /* Constraint-preserving smoother: global polyline supplies topology,
     * these iterations only choose lateral offsets inside [lb, ub]. */
    for (int it = 0; it < SMOOTH_ITERS; it++) {
        y[0] = 0.0;
        y[ns - 1] = 0.0;
        for (int k = 1; k < ns - 1; k++) {
            double avg = 0.5 * (y[k - 1] + y[k + 1]);
            double z = 0.25 * y[k] + 0.75 * avg;
            y[k] = clampd(z, lb[k], ub[k]);
        }
    }
    y[0] = 0.0;
    y[ns - 1] = 0.0;

    res.min_clearance = 1e100;
    double base_len = 0.0, local_len = 0.0;
    q[0] = add(center[0], scale(normal[0], y[0]));
    for (int k = 1; k < ns; k++) {
        q[k] = add(center[k], scale(normal[k], y[k]));
        base_len += norm(v(center[k].x - center[k - 1].x,
                           center[k].y - center[k - 1].y));
        local_len += norm(v(q[k].x - q[k - 1].x, q[k].y - q[k - 1].y));
    }
    res.base_length = base_len;
    res.length = local_len;
    res.detour_pct = 100.0 * (local_len - base_len) / base_len;

    for (int k = 0; k < ns; k++) {
        double dgamma = polyline_dist(q[k]);
        res.max_offset = fmax(res.max_offset, dgamma);
        if (dgamma > R + 1e-7) res.outside_samples++;
        if (fabs(fabs(y[k]) - R) < 1e-4) res.wall_samples++;
        for (int pi = 0; pi < crowd.nped; pi++) {
            Ped p = crowd.peds[pi];
            double dc = norm(v(q[k].x - p.pos.x, q[k].y - p.pos.y))
                      - p.rad - AGENT_R;
            res.min_clearance = fmin(res.min_clearance, dc);
        }
    }
    return res;
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void print_csv_header(void) {
    printf("R,density,n_clusters,n_peds,demand_violations,outside_samples,"
           "wall_samples,base_length,length,detour_pct,max_offset,"
           "min_clearance,raw_demand\n");
}

static void print_csv_row(Result r) {
    printf("%.2f,%d,%d,%d,%d,%d,%d,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
           r.R, r.density, r.nclusters, r.nped, r.demand_violations,
           r.outside_samples, r.wall_samples, r.base_length, r.length,
           r.detour_pct, r.max_offset, r.min_clearance, r.raw_demand);
}

int main(int argc, char **argv) {
    build_segments();

    if (argc > 1 && strcmp(argv[1], "--csv") == 0) {
        print_csv_header();
        for (int di = 0; di < 3; di++) {
            for (int i = 0; i < RS_N; i++) {
                double R = RS_FIRST + i * RS_STEP;
                print_csv_row(run_case(di, R));
            }
        }
        return 0;
    }

    /* Geometry / expression checks. */
    Vec p = {3.0, 0.4};
    check("polyline_distance_straight",
          fabs(point_seg_dist(p, v(0, 0), v(10, 0)) - 0.4) < 1e-12);
    check("tube_contains", polyline_dist(p) <= 0.4 + 1e-12);

    double B = 0.88; /* high-density far half-span 0.44 -> effective width */
    double Rlb = radius_lower_bound(B, AGENT_R, CLEARANCE);
    check("lower_bound_formula", fabs(Rlb - 0.92) < 1e-12);
    check("half_width_condition_fires",
          half_width_exceeded(B, 0.90, AGENT_R, CLEARANCE) == 1);
    check("half_width_condition_clear",
          half_width_exceeded(B, 0.95, AGENT_R, CLEARANCE) == 0);
    check("radius_is_r_plus_offset_budget",
          fabs((AGENT_R + 0.62) - Rlb) < 1e-12);

    /* Curves have the expected failure signature on the narrow side. */
    Result high_narrow = run_case(2, 0.60);
    Result high_commit = run_case(2, COMMIT_R);
    check("narrow_tube_has_violations", high_narrow.demand_violations > 0);
    check("commit_radius_clears_formula_bound", COMMIT_R >= Rlb - 1e-12);
    check("commit_fixture_demand_fits",
          high_commit.raw_demand <= COMMIT_R + 1e-12);

    int all_zero = 1, all_detour = 1, no_outside = 1, no_wall = 1;
    for (int di = 0; di < 3; di++) {
        Result r = run_case(di, COMMIT_R);
        char name[96];
        snprintf(name, sizeof(name), "zero_boundary_violations_density_%d", di);
        check(name, r.demand_violations == 0);
        snprintf(name, sizeof(name), "zero_outside_samples_density_%d", di);
        check(name, r.outside_samples == 0);
        snprintf(name, sizeof(name), "detour_within_15pct_density_%d", di);
        check(name, r.detour_pct <= 15.0);
        if (r.demand_violations != 0) all_zero = 0;
        if (r.outside_samples != 0) no_outside = 0;
        if (r.detour_pct > 15.0) all_detour = 0;
        if (r.wall_samples != 0) no_wall = 0;

        printf("density=%d clusters=%d peds=%d R=%.2f "
               "boundary_violations=%d outside_samples=%d wall_samples=%d "
               "detour_pct=%.3f length=%.3f base=%.3f max_offset=%.3f "
               "min_clearance=%.3f raw_demand=%.3f\n",
               di, r.nclusters, r.nped, r.R, r.demand_violations,
               r.outside_samples, r.wall_samples, r.detour_pct, r.length,
               r.base_length, r.max_offset, r.min_clearance, r.raw_demand);
    }
    check("all_three_densities_zero_violations", all_zero);
    check("all_three_densities_no_outside_samples", no_outside);
    check("all_three_densities_detour_le_15pct", all_detour);
    check("commit_radius_has_no_wall_rub", no_wall);

    printf("R_lb_high=%.3f\n", Rlb);
    printf("commit_R=%.2f\n", COMMIT_R);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
