/* corner: match polyline corners to a time-eased turn curve.
 *
 * Path coordinate s is a polyline. At a corner s_c its heading is a step:
 *
 *   psi_path(s) = psi0 + theta * H(s - s_c),       theta in radians.
 *
 * A linear time-interpolated animation turn over T seconds is
 *
 *   psi_anim(t) = psi0 + theta * clamp((t - t_c) / T, 0, 1).
 *
 * During the turn the heading mismatch is theta * (1 - u), u = (t-t_c)/T.
 * Its time integral is theta*T/2. In the small-angle locomotion model the
 * tangential foot-slip speed is v*|mismatch|, hence
 *
 *   d_slip = v * theta * T / 2.
 *
 * Requiring d_slip < b, where b is half a foot length, inverts to
 *
 *   theta_max(T) = 2*b / (v*T) = foot_length / (v*T).
 *
 * If "steering speed" is an angular rate omega rather than a blend window,
 * T = theta/omega for that corner. Substitution gives the rate form
 *
 *   d_slip = v*theta^2 / (2*omega),
 *   theta_max(omega) = sqrt(2*omega*b/v) = sqrt(omega*foot_length/v).
 *
 * Splitting theta into P+1 equal transition turns makes each sub-angle
 * alpha = theta/(P+1). This file uses the fixed blend-window form for its
 * three runs; the rate form is checked as an identity.
 *
 * Path length is the usual polyline length
 *
 *   L(P) = sum |p_{i+1}-p_i| = L0 + P*d_l,
 *
 * where d_l is the measured outward foot-placement detour added by one
 * transition point. The acceptance window is first P with slip < b and
 * P*d_l/L0 <= 0.05.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define CORNER_PI 3.14159265358979323846
#define CURVE_P_MAX 6

typedef struct { double x, y; } Vec2;

typedef struct {
    const char* name;
    double blend_time;     /* fixed interpolation window T, s */
    double ref_rate;       /* equivalent rate for a 90-degree turn, rad/s */
    double ref_rate_dps;   /* degrees/s */
} TurnProfile;

typedef struct {
    int added;
    int parts;
    double sub_angle;
    double sub_time;
    double slip_m;
    double length_m;
    double growth;
    int slip_ok;
    int length_ok;
    int admissible;
    int selected;
} CornerPlan;

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static double vdot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }

/* Unsigned heading change at b for the polyline a->b->c, radians in [0,pi]. */
static double turn_angle(Vec2 a, Vec2 b, Vec2 c) {
    Vec2 u = { b.x - a.x, b.y - a.y };
    Vec2 w = { c.x - b.x, c.y - b.y };
    double uu = vdot(u, u), ww = vdot(w, w);
    if (!(uu > 0.0) || !(ww > 0.0) || !isfinite(uu + ww)) return NAN;
    double cosine = vdot(u, w) / sqrt(uu * ww);
    if (cosine > 1.0) cosine = 1.0;
    if (cosine < -1.0) cosine = -1.0;
    return acos(cosine);
}

static double polyline_length(const Vec2* p, int n) {
    double total = 0.0;
    for (int i = 0; i + 1 < n; i++) {
        double dx = p[i + 1].x - p[i].x;
        double dy = p[i + 1].y - p[i].y;
        if (!isfinite(dx + dy)) return NAN;
        total += hypot(dx, dy);
    }
    return total;
}

/* Exact reduced-order slip formula: theta * v * T / 2. */
static double slip_linear(double theta, double speed, double turn_time) {
    if (!isfinite(theta) || !isfinite(speed) || !isfinite(turn_time)) return NAN;
    if (theta < 0.0 || speed < 0.0 || turn_time < 0.0) return NAN;
    return 0.5 * theta * speed * turn_time;
}

/* theta_max when T is a prescribed interpolation window. */
static double angle_cap_time(double half_foot, double speed, double turn_time) {
    if (!(half_foot > 0.0) || !(speed > 0.0) || !(turn_time > 0.0)) return NAN;
    return 2.0 * half_foot / (speed * turn_time);
}

/* Rate-form equivalents. */
static double slip_rate(double theta, double speed, double omega) {
    if (!(omega > 0.0)) return NAN;
    return slip_linear(theta, speed, theta / omega);
}

static double angle_cap_rate(double half_foot, double speed, double omega) {
    if (!(half_foot > 0.0) || !(speed > 0.0) || !(omega > 0.0)) return NAN;
    return sqrt(2.0 * omega * half_foot / speed);
}

/* Midpoint integration of v*theta*(1-t/T), used to test the closed form. */
static double integrated_slip(double theta, double speed, double T, int steps) {
    if (steps <= 0 || !(T > 0.0)) return NAN;
    double dt = T / (double)steps;
    double sum = 0.0;
    for (int i = 0; i < steps; i++) {
        double u = ((double)i + 0.5) / (double)steps;
        double mismatch = theta * (1.0 - u);
        sum += speed * mismatch * dt;
    }
    return sum;
}

static CornerPlan plan_corner(double theta, double base_length, double speed,
                              double half_foot, double detour_per_point,
                              double blend_time, int added) {
    CornerPlan p;
    memset(&p, 0, sizeof(p));
    p.added = added;
    p.parts = added + 1;
    p.sub_angle = theta / (double)p.parts;
    p.sub_time = blend_time;
    p.slip_m = slip_linear(p.sub_angle, speed, blend_time);
    p.length_m = base_length + (double)added * detour_per_point;
    p.growth = (p.length_m - base_length) / base_length;
    p.slip_ok = isfinite(p.slip_m) && p.slip_m < half_foot;
    p.length_ok = isfinite(p.growth) && p.growth <= 0.05 + 1e-12;
    p.admissible = p.slip_ok && p.length_ok;
    return p;
}

static int first_admissible(double theta, double L0, double v, double b,
                            double detour, double T, int max_added,
                            CornerPlan* chosen) {
    for (int p = 0; p <= max_added; p++) {
        CornerPlan plan = plan_corner(theta, L0, v, b, detour, T, p);
        if (plan.admissible) {
            plan.selected = 1;
            if (chosen) *chosen = plan;
            return p;
        }
    }
    return -1;
}

static void print_csv(const TurnProfile* profiles, int nprof,
                      double theta, double L0, double v, double b,
                      double detour) {
    printf("profile,speed_deg_s,blend_T_s,P,parts,subangle_deg,subtime_s,"
           "slip_m,half_foot_m,length_m,growth_pct,slip_ok,length_ok,"
           "admissible,selected\n");
    for (int j = 0; j < nprof; j++) {
        CornerPlan chosen;
        int pstar = first_admissible(theta, L0, v, b, detour,
                                     profiles[j].blend_time, CURVE_P_MAX,
                                     &chosen);
        for (int p = 0; p <= CURVE_P_MAX; p++) {
            CornerPlan q = plan_corner(theta, L0, v, b, detour,
                                       profiles[j].blend_time, p);
            int selected = (p == pstar);
            printf("%s,%.0f,%.3f,%d,%d,%.6f,%.3f,%.9f,%.3f,%.3f,%.4f,%d,%d,%d,%d\n",
                   profiles[j].name, profiles[j].ref_rate_dps,
                   profiles[j].blend_time, p, q.parts,
                   q.sub_angle * 180.0 / CORNER_PI, q.sub_time, q.slip_m, b,
                   q.length_m, 100.0 * q.growth, q.slip_ok, q.length_ok,
                   q.admissible, selected);
        }
    }
}

int main(int argc, char** argv) {
    /* Lab scale. Replace these with screenshot/asset measurements. */
    const double foot = 0.27;          /* m, EU ~43 boot/foot length */
    const double half_foot = foot / 2.0;
    const double speed = 1.4;          /* m/s, relaxed run/jog */
    const double theta = CORNER_PI / 2.0;
    const double base_length = 8.0;    /* two 4 m arms through the corner */
    const double detour_per_point = 0.10; /* m measured overhead per point */

    /* T is the animation blend window; equivalent deg/s is for a 90 turn. */
    TurnProfile profiles[3] = {
        { "slow",     0.45, (CORNER_PI / 2.0) / 0.45,  90.0 / 0.45 },
        { "standard", 0.30, (CORNER_PI / 2.0) / 0.30,  90.0 / 0.30 },
        { "fast",     0.18, (CORNER_PI / 2.0) / 0.18,  90.0 / 0.18 },
    };

    if (argc > 1 && strcmp(argv[1], "--csv") == 0) {
        print_csv(profiles, 3, theta, base_length, speed, half_foot,
                  detour_per_point);
        return 0;
    }

    printf("scale foot=%.3f half_foot=%.3f speed=%.3f theta_deg=%.1f "
           "L0=%.3f detour/P=%.3f length_cap=%.3f\n",
           foot, half_foot, speed, theta * 180.0 / CORNER_PI,
           base_length, detour_per_point, 0.05 * base_length);

    /* -- formula and geometry unit checks -- */
    Vec2 a = { -1.0, 0.0 }, o = { 0.0, 0.0 }, c = { 0.0, 1.0 };
    check("right_angle_detected",
          fabs(turn_angle(a, o, c) - CORNER_PI / 2.0) < 1e-12);
    Vec2 tri[3] = { {0,0}, {4,0}, {4,3} };
    check("polyline_length_3_4_5", fabs(polyline_length(tri, 3) - 7.0) < 1e-12);

    double d0 = slip_linear(theta, speed, 0.18);
    check("slip_formula_value", fabs(d0 - 0.197920337) < 1e-8);
    double integral = integrated_slip(theta, speed, 0.18, 1000);
    check("integral_matches_half_area", fabs(integral - d0) < 1e-12);
    check("angle_inversion_roundtrip",
          fabs(slip_linear(angle_cap_time(half_foot, speed, 0.30),
                           speed, 0.30) - half_foot) < 1e-12);
    check("rate_form_matches_time_form",
          fabs(slip_rate(0.73, speed, 2.0) -
               slip_linear(0.73, speed, 0.73 / 2.0)) < 1e-12);
    check("rate_cap_roundtrip",
          fabs(slip_rate(angle_cap_rate(half_foot, speed, 2.0),
                         speed, 2.0) - half_foot) < 1e-12);
    check("negative_speed_rejected", isnan(slip_linear(0.1, -1.0, 0.2)));
    check("zero_rate_cap_rejected", isnan(angle_cap_rate(half_foot, speed, 0.0)));

    /* -- three steering-speed runs -- */
    int expected_p[3] = { 3, 2, 1 };
    for (int j = 0; j < 3; j++) {
        CornerPlan chosen;
        int pstar = first_admissible(theta, base_length, speed, half_foot,
                                     detour_per_point,
                                     profiles[j].blend_time, CURVE_P_MAX,
                                     &chosen);
        double theta_max = angle_cap_time(half_foot, speed,
                                          profiles[j].blend_time);
        int length_pmax = (int)floor(0.05 * base_length / detour_per_point
                                     + 1e-12);
        printf("run=%s steer=%.0fdeg/s T=%.2fs theta_max=%.3frad(%.2fdeg) "
               "P*=%d parts=%d alpha=%.3frad(%.2fdeg) slip=%.6fm "
               "half_foot=%.3fm L=%.3fm growth=%.2f%% length_Pmax=%d ok=%d\n",
               profiles[j].name, profiles[j].ref_rate_dps,
               profiles[j].blend_time, theta_max,
               theta_max * 180.0 / CORNER_PI, pstar, chosen.parts,
               chosen.sub_angle, chosen.sub_angle * 180.0 / CORNER_PI,
               chosen.slip_m, half_foot, chosen.length_m,
               100.0 * chosen.growth, length_pmax, chosen.admissible);

        char name[96];
        snprintf(name, sizeof(name), "%s_transition_count", profiles[j].name);
        check(name, pstar == expected_p[j]);
        snprintf(name, sizeof(name), "%s_slip_under_half_foot", profiles[j].name);
        check(name, chosen.slip_m < half_foot);
        snprintf(name, sizeof(name), "%s_length_growth_under_5pct", profiles[j].name);
        check(name, chosen.growth <= 0.05 + 1e-12);
        snprintf(name, sizeof(name), "%s_minimal_transition_count", profiles[j].name);
        if (pstar > 0) {
            CornerPlan before = plan_corner(theta, base_length, speed, half_foot,
                                            detour_per_point,
                                            profiles[j].blend_time, pstar - 1);
            check(name, before.length_ok && !before.slip_ok);
        } else {
            check(name, 1);
        }
        snprintf(name, sizeof(name), "%s_within_length_curve_window", profiles[j].name);
        check(name, pstar >= 0 && pstar <= length_pmax);
    }

    /* The length curve alone says P=5 is already over the 5% cap here. */
    CornerPlan too_long = plan_corner(theta, base_length, speed, half_foot,
                                      detour_per_point, 0.45, 5);
    check("five_points_exceed_length_cap", !too_long.length_ok);

    /* If detour is too expensive, the slow slip solution is not feasible. */
    CornerPlan impossible;
    int bad = first_admissible(theta, base_length, speed, half_foot,
                               0.50, 0.45, CURVE_P_MAX, &impossible);
    check("expensive_detour_reported_infeasible", bad < 0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
