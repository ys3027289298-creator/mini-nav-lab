
/* pursuit: lead prediction for chasing a moving target.
 *
 * Review points being fixed:
 *  1. No prediction: the pursuer steered at the target's current position and
 *     always lagged behind a moving target.
 *  2. The look-ahead distance/time was a hardcoded constant.
 *  3. A target that reverses direction was never considered, so a long lead
 *     made the pursuer overshoot.
 *
 * Model (all comments refer to these symbols):
 *   p  pursuer position, speed u, radius r
 *   q  target position, velocity v, speed w = |v|
 *   D  distance from pursuer to target, e = (q-p)/D unit approach vector
 *   T  prediction horizon
 */
#include <stdio.h>
#include <math.h>

#define U_SPEED   10.0          /* pursuer speed u                         */
#define RADIUS    1.0           /* pursuer radius r                        */
#define DT        0.02          /* simulation frame step                   */
#define TAU       0.20          /* re-aim/commitment window                */
#define T_MIN     DT            /* shortest usable horizon                 */
#define T_MAX     2.0           /* longest usable horizon                  */
#define T_FIXED   1.0           /* the old hardcoded horizon, kept as a    */
                                /* regression baseline                     */

typedef struct { double x, y; } Vec2;

static Vec2 v2(double x, double y) { Vec2 r = {x, y}; return r; }
static Vec2 vadd(Vec2 a, Vec2 b) { return v2(a.x + b.x, a.y + b.y); }
static Vec2 vsub(Vec2 a, Vec2 b) { return v2(a.x - b.x, a.y - b.y); }
static Vec2 vscale(Vec2 a, double k) { return v2(a.x * k, a.y * k); }
static double vdot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
static double vlen(Vec2 a) { return hypot(a.x, a.y); }
static Vec2 vunit(Vec2 a) {
    double n = vlen(a);
    return n == 0.0 ? v2(0.0, 0.0) : v2(a.x / n, a.y / n);
}
static double clampd(double x, double lo, double hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

/*
 * Point 1 -- basic prediction expression.
 *
 *     q_pred = q + v * T
 *
 * The pursuer steers at where the target will be after horizon T instead of
 * at its current position. T = 0 recovers the old, always-lagging behaviour.
 */
static Vec2 predict_point(Vec2 q, Vec2 v, double T) {
    return vadd(q, vscale(v, T));
}

/*
 * Point 2 -- horizon vs. distance law (replaces the hardcoded value).
 *
 * A target at distance D reached at pursuer speed u takes time D/u to close,
 * so the natural lead is exactly that closing time:
 *
 *     T_lead = clamp(D / u, T_min, T_max)
 *
 * Far target  -> long horizon, near target -> short horizon. The clamp only
 * bounds the result; nothing is a magic constant distance anymore.
 */
static double lead_horizon(double D) {
    return clampd(D / U_SPEED, T_MIN, T_MAX);
}

/*
 * Point 3 -- reversal gate.
 *
 * Let theta be the angle between the target velocity v and the approach
 * vector e. The target is heading back toward the pursuer when
 *
 *     cos(theta) = v.e / |v| < 0      i.e. theta > 90 degrees
 *
 * Define the head-on closing speed
 *
 *     s = max(0, -v.e)                (0 while theta <= 90)
 *
 * Over horizon T the predicted point advances s*T toward the pursuer along
 * the approach axis. If it advances past the target-to-pursuer distance D,
 * the aim point lands behind the pursuer and the pursuer commits to
 * overshooting. The forward excursion is
 *
 *     overshoot = s*T - D
 *
 * Requiring it to stay inside the budget 2r gives
 *
 *     s*T - D <= 2r   <=>   T <= (D + 2r) / s = T_safe
 *
 * This is the condition that forces the horizon to be shortened. The final
 * horizon takes the smaller of the distance law and the reversal bound:
 *
 *     T = clamp(min(T_lead, T_safe), T_min, T_max)
 *
 * When theta <= 90, s = 0, T_safe is infinite and the distance law stands.
 */
static double approach_horizon(double D, Vec2 e, Vec2 v,
                               double *cos_theta, double *closing) {
    double w = vlen(v);
    double ct = w > 0.0 ? vdot(v, e) / w : 0.0;
    double vn = -vdot(v, e);
    double s = vn > 0.0 ? vn : 0.0;            /* s = max(0, -v.e) */
    double t_lead = lead_horizon(D);
    double t_safe = s > 0.0 ? (D + 2.0 * RADIUS) / s : 1e300;
    if (cos_theta) *cos_theta = ct;
    if (closing)   *closing = s;
    return clampd(t_lead < t_safe ? t_lead : t_safe, T_MIN, T_MAX);
}

/* ---- discrete simulator used by the self-tests -------------------------- */

typedef struct {
    Vec2 p, q, v, aim;
    double fixed_T;          /* <0 means use the adaptive horizon          */
    int window;              /* frames left in the current commitment      */
} Sim;

static void sim_init(Sim *s, Vec2 p, Vec2 q, Vec2 v, double fixed_T) {
    s->p = p; s->q = q; s->v = v; s->aim = p;
    s->fixed_T = fixed_T; s->window = 0;
}

static void sim_step(Sim *s) {
    if (s->window == 0) {
        Vec2 e = vunit(vsub(s->q, s->p));
        double D = vlen(vsub(s->q, s->p));
        double T = s->fixed_T < 0.0
                 ? approach_horizon(D, e, s->v, 0, 0)
                 : s->fixed_T;
        s->aim = predict_point(s->q, s->v, T);
        s->window = (int)(TAU / DT + 0.5);
    }
    /* move toward the committed aim point, arresting on arrival */
    Vec2 d = vsub(s->aim, s->p);
    double dist = vlen(d), step = U_SPEED * DT;
    if (dist <= step) s->p = s->aim;
    else              s->p = vadd(s->p, vscale(vunit(d), step));
    s->q = vadd(s->q, vscale(s->v, DT));
    s->window--;
}

/* straight receding target: contact frame and post-contact tracking envelope */
static void chase(double w, double fixed_T, int *contact_frame,
                  double *track_env, int frames) {
    Sim s;
    sim_init(&s, v2(0, 0), v2(12, 8), v2(w, 0), fixed_T);
    int contact = -1;
    double env = 0.0;
    for (int k = 0; k < frames; k++) {
        sim_step(&s);
        double D = vlen(vsub(s.q, s.p));
        if (D <= RADIUS && contact < 0) contact = k;
        if (contact >= 0 && D > env) env = D;
    }
    *contact_frame = contact;
    *track_env = env;
}

/* head-on target reverses to receding at the first contact-aligned re-aim */
static double reverse_overshoot(double w, double fixed_T, double *d_flip,
                                int frames) {
    Sim s;
    sim_init(&s, v2(0, 0), v2(8, 0), v2(-w, 0), fixed_T);
    int flipped = 0;
    double over = 0.0, dflip = 0.0;
    for (int k = 0; k < frames; k++) {
        if (!flipped && s.window == 0 && vlen(vsub(s.q, s.p)) <= RADIUS) {
            s.v = v2(w, 0);
            s.window = 0;
            flipped = 1;
            dflip = vlen(vsub(s.q, s.p));
        }
        sim_step(&s);
        if (flipped && s.p.x > s.q.x) {
            double ex = s.p.x - s.q.x;
            if (ex > over) over = ex;
        }
    }
    if (d_flip) *d_flip = dflip;
    return over;
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    const double speeds[3] = {3.0, 5.0, 7.0};
    const int frames = (int)(6.0 / DT);

    /* point 1: prediction moves the aim point along the velocity */
    Vec2 aim0 = predict_point(v2(10, 0), v2(2, 0), 0.0);
    Vec2 aim1 = predict_point(v2(10, 0), v2(2, 0), 1.0);
    check("predict_zero_horizon", fabs(aim0.x - 10.0) < 1e-9);
    check("predict_basic_form", fabs(aim1.x - 12.0) < 1e-9 &&
                                fabs(aim1.y - 0.0) < 1e-9);

    /* point 2: horizon grows with distance, never hardcoded; clamped */
    check("lead_far_longer", lead_horizon(10.0) > lead_horizon(2.0));
    check("lead_near_is_short", fabs(lead_horizon(1.0) - 0.1) < 1e-9);
    check("lead_clamped_low", lead_horizon(0.0) == T_MIN);
    check("lead_clamped_high", lead_horizon(100.0) == T_MAX);

    /* point 3: angle gate theta > 90 <=> cos < 0 */
    double ct = 0, s = 0;
    (void)approach_horizon(8.0, v2(1, 0), v2(5, 0), &ct, &s);       /* theta 0   */
    check("gate_closed_ahead", ct > 0 && s == 0.0);
    (void)approach_horizon(8.0, v2(1, 0), v2(0, 5), &ct, &s);       /* theta 90  */
    check("gate_ninety_boundary", fabs(ct) < 1e-9 && s == 0.0);
    double T_rev = approach_horizon(8.0, v2(1, 0), v2(-5, 0), &ct, &s);
    check("gate_open_reverse", ct < 0 && s == 5.0);
    /* moderate head-on speed: distance law is already inside the safe bound */
    check("gate_keeps_distance_law_when_safe", T_rev == lead_horizon(8.0));

    /* reversal bound keeps s*T - D inside the 2r budget */
    double T_fast = approach_horizon(10.0, v2(1, 0), v2(-20, 0), &ct, &s);
    check("gate_binds_fast_target", T_fast < lead_horizon(10.0));
    check("gate_overshoot_budget", s * T_fast - 10.0 <= 2.0 * RADIUS + 1e-9);

    /* trade-off curves at w=5: longer T reduces lag, increases overshoot */
    int c_short, c_long;
    double e_short, e_long;
    chase(5.0, 0.0, &c_short, &e_short, frames);
    chase(5.0, 0.4, &c_long, &e_long, frames);
    check("lag_curve_longer_lead_contacts_earlier", c_long < c_short);
    double o_short = reverse_overshoot(5.0, 0.1, 0, frames);
    double o_long  = reverse_overshoot(5.0, 0.4, 0, frames);
    check("overshoot_curve_rises_with_lead", o_long > o_short);

    /* fixed hardcoded horizon overshoots the tracking envelope */
    int cf; double ef;
    chase(5.0, T_FIXED, &cf, &ef, frames);
    check("fixed_horizon_tracks_poorly", ef > RADIUS);

    /* three-speed acceptance */
    int all_lag = 1, all_over = 1, beats_pure = 1;
    for (int i = 0; i < 3; i++) {
        double w = speeds[i];
        int ca, cp;
        double env_a, env_p;
        chase(w, -1.0, &ca, &env_a, frames);
        chase(w, 0.0, &cp, &env_p, frames);
        double dflip;
        double over_a = reverse_overshoot(w, -1.0, &dflip, frames);

        char name[64];
        snprintf(name, sizeof(name), "lag_within_radius_w%.0f", w);
        check(name, env_a <= RADIUS);
        snprintf(name, sizeof(name), "overshoot_within_2r_w%.0f", w);
        check(name, over_a <= 2.0 * RADIUS);
        snprintf(name, sizeof(name), "contacts_no_later_than_pure_w%.0f", w);
        check(name, ca >= 0 && ca <= cp);

        if (!(env_a <= RADIUS)) all_lag = 0;
        if (!(over_a <= 2.0 * RADIUS)) all_over = 0;
        if (!(ca >= 0 && ca <= cp)) beats_pure = 0;
    }
    check("all_three_speeds_lag_ok", all_lag);
    check("all_three_speeds_overshoot_ok", all_over);
    check("adaptive_beats_pure_pursuit", beats_pure);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
