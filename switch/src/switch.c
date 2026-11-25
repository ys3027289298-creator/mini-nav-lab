
/* switch: fine/coarse navigation-grid origin and scale switching.
 *
 * Coordinate convention used by the demo:
 *   - coordinates inside the fitting math are grid-cell coordinates;
 *   - coarse cell pitch is 1 m, so a jump of 0.4 coarse cells is 0.4 m;
 *   - fine cell pitch is 0.25 m, hence the true scale is 0.25.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define N_SCENES 3
#define FINE_PITCH_M 0.25
#define COARSE_PITCH_M 1.0
#define CHARACTER_RADIUS_M 0.40
#define SCALE_ERROR 0.00035      /* coarse cells per fine cell */
#define SAFETY_FACTOR 0.90       /* design to 90% of the radius */

typedef struct { double x, y; } Vec2;
typedef struct { Vec2 mn, mx; } Box;
typedef struct { double s; Vec2 t; } Transform;

typedef struct {
    const char* name;
    double width_m, height_m;
    double fw, fh;               /* fine bbox size, fine cells */
    double cw, ch;               /* coarse bbox size, coarse cells */
    Box fine_box;
    Box coarse_box;
    Vec2 portal;                 /* actual indoor->outdoor switch point */
    double radius;               /* in coarse cells/metres */
    double ds;                   /* absolute scale error */
} Scene;

typedef struct {
    Vec2 anchor_f;
    Vec2 anchor_c;
    Vec2 q;
    Vec2 delta;
    Vec2 true_c;
    Vec2 hat_c;
    double rho;
    double jump;
    double rho_box;
    double jmax_box;
    double eps_allow;
    int portal_pass;
    int box_pass;
} Evaluation;

static Vec2 apply_transform(const Transform* tr, Vec2 p) {
    Vec2 out = { tr->s * p.x + tr->t.x, tr->s * p.y + tr->t.y };
    return out;
}

/* Recover s and t from matching fine/coarse bounding boxes.
 * A single uniform scale is assumed; anisotropic grids should use sx/sy.
 */
static int solve_transform(const Box* f, const Box* c, Transform* tr) {
    double fw = f->mx.x - f->mn.x;
    double fh = f->mx.y - f->mn.y;
    double cw = c->mx.x - c->mn.x;
    double ch = c->mx.y - c->mn.y;
    if (!(fw > 0.0 && fh > 0.0 && cw > 0.0 && ch > 0.0)) return -1;

    double sx = cw / fw;
    double sy = ch / fh;
    if (fabs(sx - sy) > 1e-12) return -2;

    tr->s = sx;
    tr->t.x = c->mn.x - sx * f->mn.x;
    tr->t.y = c->mn.y - sy * f->mn.y;
    return 0;
}

/* Largest distance from o to any corner of b: the worst switch error. */
static double max_radius(const Box* b, Vec2 o) {
    double dx = fmax(o.x - b->mn.x, b->mx.x - o.x);
    double dy = fmax(o.y - b->mn.y, b->mx.y - o.y);
    return hypot(dx, dy);
}

/* u is the equal fractional inset from the minimum corner:
 * u=0 is the bbox minimum corner, u=0.5 is the bbox centre.
 */
static Evaluation evaluate_at(const Scene* sc, const Transform* tr, double u) {
    Evaluation e;
    memset(&e, 0, sizeof(e));

    double fw = sc->fine_box.mx.x - sc->fine_box.mn.x;
    double fh = sc->fine_box.mx.y - sc->fine_box.mn.y;

    e.anchor_f.x = sc->fine_box.mn.x + u * fw;
    e.anchor_f.y = sc->fine_box.mn.y + u * fh;
    e.anchor_c = apply_transform(tr, e.anchor_f);

    e.q.x = sc->portal.x - e.anchor_f.x;
    e.q.y = sc->portal.y - e.anchor_f.y;
    e.rho = hypot(e.q.x, e.q.y);

    /* Re-anchor translation at O, then use s+ds. Translation error at O=0. */
    e.delta.x = sc->ds * e.q.x;
    e.delta.y = sc->ds * e.q.y;
    e.jump = fabs(sc->ds) * e.rho;

    e.true_c = apply_transform(tr, sc->portal);
    e.hat_c.x = e.anchor_c.x + (tr->s + sc->ds) * e.q.x;
    e.hat_c.y = e.anchor_c.y + (tr->s + sc->ds) * e.q.y;

    e.rho_box = max_radius(&sc->fine_box, e.anchor_f);
    e.jmax_box = fabs(sc->ds) * e.rho_box;
    e.eps_allow = sc->radius / e.rho_box;
    e.portal_pass = e.jump < sc->radius;
    e.box_pass = e.jmax_box < sc->radius;
    return e;
}

static int near_eq(double a, double b, double tol) {
    return fabs(a - b) <= tol;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void print_vec(Vec2 v) {
    printf("(%.3f, %.3f)", v.x, v.y);
}

static void print_violation(const Scene* sc, const Evaluation* e, const char* origin) {
    if (e->portal_pass) return;
    printf("VIOLATION tier=%s origin=%s jump=%.6fm >= radius=%.3fm",
           sc->name, origin, e->jump * COARSE_PITCH_M, sc->radius * COARSE_PITCH_M);
    printf(" fine_xy=");
    print_vec(sc->portal);
    printf(" coarse_true=");
    print_vec(e->true_c);
    printf(" coarse_estimated=");
    print_vec(e->hat_c);
    printf(" delta_m=");
    print_vec(e->delta);
    printf("\n");
}

static void init_scenes(Scene scenes[N_SCENES]) {
    const char* names[N_SCENES] = {
        "small_48x36m", "medium_160x120m", "large_400x300m"
    };
    const double widths[N_SCENES] = { 48.0, 160.0, 400.0 };
    const double heights[N_SCENES] = { 36.0, 120.0, 300.0 };

    for (int i = 0; i < N_SCENES; i++) {
        Scene* sc = &scenes[i];
        memset(sc, 0, sizeof(*sc));
        sc->name = names[i];
        sc->width_m = widths[i];
        sc->height_m = heights[i];
        sc->fw = sc->width_m / FINE_PITCH_M;
        sc->fh = sc->height_m / FINE_PITCH_M;
        sc->cw = sc->width_m / COARSE_PITCH_M;
        sc->ch = sc->height_m / COARSE_PITCH_M;

        /* Deliberately different fine/coarse origins. */
        sc->fine_box.mn = (Vec2){ 120.0, -80.0 };
        sc->fine_box.mx = (Vec2){ 120.0 + sc->fw, -80.0 + sc->fh };
        sc->coarse_box.mn = (Vec2){ -30.0, 50.0 };
        sc->coarse_box.mx = (Vec2){ -30.0 + sc->cw, 50.0 + sc->ch };

        /* Portal is 8 fine cells x and 6 fine cells before the far corner. */
        sc->portal = (Vec2){ sc->fine_box.mx.x - 8.0, sc->fine_box.mx.y - 6.0 };
        sc->radius = CHARACTER_RADIUS_M / COARSE_PITCH_M;
        sc->ds = SCALE_ERROR;
    }
}

static int print_csv(void) {
    Scene scenes[N_SCENES];
    init_scenes(scenes);

    printf("size,u,centralization,rho_box_fine_cells,jmax_m,radius_m,");
    printf("safety_budget_m,eps_allow,complexity_index\n");

    for (int i = 0; i < N_SCENES; i++) {
        Transform tr;
        if (solve_transform(&scenes[i].fine_box, &scenes[i].coarse_box, &tr) != 0) return 1;
        for (int k = 0; k <= 10; k++) {
            double u = 0.05 * k;
            double z = 2.0 * u;  /* 0=corner, 1=centre */
            Evaluation e = evaluate_at(&scenes[i], &tr, u);
            double complexity = 1.0 + 2.0 * z; /* normalized proxy: 1 corner -> 3 centre */
            printf("%s,%.2f,%.2f,%.3f,%.6f,%.2f,%.2f,%.9f,%.2f\n",
                   scenes[i].name, u, z, e.rho_box,
                   e.jmax_box * COARSE_PITCH_M,
                   scenes[i].radius * COARSE_PITCH_M,
                   SAFETY_FACTOR * scenes[i].radius * COARSE_PITCH_M,
                   e.eps_allow, complexity);
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc > 1 && strcmp(argv[1], "--csv") == 0) return print_csv();

    Scene scenes[N_SCENES];
    init_scenes(scenes);
    Transform transforms[N_SCENES];

    puts("# Fine/coarse navigation switch");
    printf("# fine_pitch=%.2fm coarse_pitch=%.2fm radius=%.2fm ds=%.6f (relative %.3f%%)\n\n",
           FINE_PITCH_M, COARSE_PITCH_M, CHARACTER_RADIUS_M,
           SCALE_ERROR, 100.0 * SCALE_ERROR / 0.25);

    int fit_ok = 1, scale_ok = 1, translation_ok = 1;
    int endpoint_ok = 1, center_half_ok = 1;
    int allowed_formula_ok = 1;
    int corner_expected_ok = 1, center_expected_ok = 1;
    int selected_all_pass = 1;

    const double expected_corner_rho[N_SCENES] = { 230.0, 790.0, 1990.0 };
    const double expected_center_rho[N_SCENES] = { 110.0, 390.0, 990.0 };
    const int expected_corner_pass[N_SCENES] = { 1, 1, 0 };

    for (int i = 0; i < N_SCENES; i++) {
        Scene* sc = &scenes[i];
        int rc = solve_transform(&sc->fine_box, &sc->coarse_box, &transforms[i]);
        if (rc != 0) { fit_ok = 0; continue; }
        Transform* tr = &transforms[i];

        if (!near_eq(tr->s, 0.25, 1e-12)) scale_ok = 0;
        if (!near_eq(tr->t.x, -60.0, 1e-12) || !near_eq(tr->t.y, 70.0, 1e-12)) translation_ok = 0;

        Vec2 fmin_mapped = apply_transform(tr, sc->fine_box.mn);
        Vec2 fmax_mapped = apply_transform(tr, sc->fine_box.mx);
        endpoint_ok &= near_eq(fmin_mapped.x, sc->coarse_box.mn.x, 1e-10)
                    && near_eq(fmin_mapped.y, sc->coarse_box.mn.y, 1e-10)
                    && near_eq(fmax_mapped.x, sc->coarse_box.mx.x, 1e-10)
                    && near_eq(fmax_mapped.y, sc->coarse_box.mx.y, 1e-10);

        Evaluation corner = evaluate_at(sc, tr, 0.0);
        Evaluation center = evaluate_at(sc, tr, 0.5);

        center_half_ok &= near_eq(center.rho_box, 0.5 * corner.rho_box, 1e-9);
        allowed_formula_ok &= near_eq(corner.eps_allow, sc->radius / corner.rho_box, 1e-12);

        corner_expected_ok &= near_eq(corner.rho, expected_corner_rho[i], 1e-9)
                           && (corner.portal_pass == expected_corner_pass[i]);
        center_expected_ok &= near_eq(center.rho, expected_center_rho[i], 1e-9)
                           && center.portal_pass && center.box_pass;
        selected_all_pass &= center.portal_pass && center.box_pass
                          && center.jmax_box * COARSE_PITCH_M < SAFETY_FACTOR * CHARACTER_RADIUS_M;

        printf("== %s ==\n", sc->name);
        printf("bbox fine min/max: ");
        print_vec(sc->fine_box.mn); printf(" / "); print_vec(sc->fine_box.mx); puts("");
        printf("bbox coarse min/max: ");
        print_vec(sc->coarse_box.mn); printf(" / "); print_vec(sc->coarse_box.mx); puts("");
        printf("fit: s=%.6f coarse-cell/fine-cell, t=", tr->s);
        print_vec(tr->t); puts("");
        printf("portal fine="); print_vec(sc->portal);
        printf(" true_coarse="); print_vec(corner.true_c); puts("");

        printf("corner origin: rho=%.1f jump=%.6fm worst_bbox=%.6fm eps_allow=%.9f -> %s\n",
               corner.rho, corner.jump * COARSE_PITCH_M,
               corner.jmax_box * COARSE_PITCH_M, corner.eps_allow,
               corner.portal_pass ? "PASS" : "FAIL");
        print_violation(sc, &corner, "corner");

        printf("center origin: rho=%.1f jump=%.6fm worst_bbox=%.6fm eps_allow=%.9f -> %s\n\n",
               center.rho, center.jump * COARSE_PITCH_M,
               center.jmax_box * COARSE_PITCH_M, center.eps_allow,
               center.portal_pass ? "PASS" : "FAIL");
    }

    /* Large-scene origin decision: Jmax(z)=ds*D0*(1-z/2), z in [0,1]. */
    Scene* large = &scenes[2];
    double d0 = hypot(large->fw, large->fh);
    double z_strict = 2.0 * (1.0 - large->radius / (SCALE_ERROR * d0));
    double z_safety = 2.0 * (1.0 - SAFETY_FACTOR * large->radius / (SCALE_ERROR * d0));
    printf("large origin threshold: z_strict>%.4f; with %.0f%% radius budget z>=%.4f -> choose centre z=1\n\n",
           z_strict, 100.0 * SAFETY_FACTOR, z_safety);

    check("bbox_fit_ok", fit_ok);
    check("scale_recovered", scale_ok);
    check("translation_recovered", translation_ok);
    check("bbox_endpoints_map", endpoint_ok);
    check("corner_values_and_expected_failure", corner_expected_ok);
    check("center_values_and_all_pass", center_expected_ok);
    check("center_halves_worst_distance", center_half_ok);
    check("allowed_error_formula", allowed_formula_ok);
    check("selected_center_meets_safety_budget", selected_all_pass);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
