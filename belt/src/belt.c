
/* belt: conveyor-belt vs path-following conflict.
 *
 * While the agent stands on a belt, each frame stacks two displacements:
 *   path advance : u * dt          (steered along the current path)
 *   belt push    : v_belt * dt     (forced, independent of steering)
 * The belt component parallel to the path is swallowed by path progress;
 * only the perpendicular component drives the agent off the path, so the
 * deviation against the active path accumulates linearly:
 *   d_{k+1} = d_k + v_perp * dt            ->  d(t) = v_perp * t
 * Inverting the radius budget d(t) <= r gives the longest dwell the active
 * path can tolerate before it must be re-anchored:
 *   t_max = r / v_perp                     (INF when the belt is parallel)
 * Past t_max two policies are compared:
 *   follow - keep steering; whenever d would cross r, repath from the
 *            current position so d re-anchors to 0. Deviation stays bounded
 *            but a long/fast belt triggers many recomputes.
 *   pause  - suspend path following in the frame the radius would be
 *            crossed; the belt carries the agent and one fresh path is
 *            issued at belt exit. One recompute, original route abandoned.
 * Corrections are applied in the same frame the radius would be crossed
 * (frame-atomic), so measured deviation against the active path never
 * exceeds the radius.
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define DT          (1.0 / 30.0)  /* 30 fps frame duration */
#define RADIUS      0.5           /* agent radius, deviation budget r */
#define AGENT_SPEED 2.0           /* path-following speed u */
#define BELT_W      4.0           /* belt extent along the path: x in [0, 4] */
#define BELT_HALF_D 2.0           /* belt extent across the path: |y| <= 2 */
#define EPS         1e-9
#define MAXTRACE    128

typedef struct { double vx, vy; } Belt;  /* path direction is +x */

typedef struct {
    int    frames;     /* frames spent inside the belt */
    int    repaths;    /* path recomputations */
    double max_dev;    /* max deviation vs the ACTIVE path */
    double carry;      /* total sideways offset vs the original route */
    int    suspended;  /* path following was ever paused */
} SimResult;

typedef struct {
    int n;
    int frame[MAXTRACE];
    double follow_d[MAXTRACE];   /* deviation under follow, -1 after exit */
    double pause_d[MAXTRACE];    /* active-path deviation under pause */
    double pause_carry[MAXTRACE];
} Trace;

/* ---- conflict expression ------------------------------------------------ */

static double perp_speed(const Belt* b) { return fabs(b->vy); }

/* extra off-path displacement produced by one belt frame */
static double dev_step(double vperp) { return vperp * DT; }

/* accumulated deviation after t seconds of uninterrupted belt push */
static double accumulated(double vperp, double t) { return vperp * t; }

/* longest dwell satisfying d(t) <= r; parallel belt has no limit */
static double max_dwell(double r, double vperp) {
    if (vperp <= EPS) return INFINITY;
    return r / vperp;
}

static int on_belt(double x, double y) {
    return x >= -EPS && x <= BELT_W + EPS && fabs(y) <= BELT_HALF_D + EPS;
}

/* ---- policy A: keep following, repath whenever the radius would break ---- */

static SimResult sim_follow(const Belt* b) {
    SimResult res; memset(&res, 0, sizeof(res));
    double x = 0.0, y = 0.0, d = 0.0;
    while (on_belt(x, y)) {
        x += (AGENT_SPEED + b->vx) * DT;   /* path advance, plus belt-aligned push */
        y += b->vy * DT;                   /* forced perpendicular belt push */
        double nd = d + fabs(b->vy) * DT;
        if (nd >= RADIUS) {
            res.repaths++;                 /* old path invalid, repath here */
            d = 0.0;                       /* fresh path starts at the agent */
        } else {
            d = nd;
            if (d > res.max_dev) res.max_dev = d;
        }
        res.frames++;
        if (fabs(y) > res.carry) res.carry = fabs(y);
    }
    return res;
}

/* ---- policy B: pause following at the radius, one repath at exit --------- */

static SimResult sim_pause(const Belt* b) {
    SimResult res; memset(&res, 0, sizeof(res));
    double x = 0.0, y = 0.0, d = 0.0;
    int paused = 0;
    while (on_belt(x, y)) {
        if (!paused) {
            double nd = d + fabs(b->vy) * DT;
            if (nd >= RADIUS) {
                paused = 1;                /* suspend path following */
                d = 0.0;                   /* no active path, no active error */
            } else {
                d = nd;
                if (d > res.max_dev) res.max_dev = d;
            }
            x += (AGENT_SPEED + b->vx) * DT;
        }
        y += b->vy * DT;                   /* belt keeps carrying regardless */
        res.frames++;
        if (fabs(y) > res.carry) res.carry = fabs(y);
    }
    res.suspended = paused;
    if (paused) res.repaths = 1;           /* single fresh path at belt exit */
    return res;
}

/* per-tier recompute budget: no more than one tenth of the belt frames */
static int budget(int frames) { return frames / 10; }

/* per-frame trace of both policies on the fast tier, for the curves */
static Trace trace_fast(void) {
    Trace t; memset(&t, 0, sizeof(t));
    Belt b = {0.0, 3.0};
    double fx = 0.0, fy = 0.0, fd = 0.0;
    double px = 0.0, py = 0.0, pd = 0.0;
    int paused = 0;
    while (t.n < MAXTRACE && (on_belt(fx, fy) || on_belt(px, py))) {
        if (on_belt(fx, fy)) {
            fx += AGENT_SPEED * DT;
            fy += b.vy * DT;
            double nd = fd + fabs(b.vy) * DT;
            fd = (nd >= RADIUS) ? 0.0 : nd;
        }
        if (on_belt(px, py)) {
            if (!paused) {
                double nd = pd + fabs(b.vy) * DT;
                if (nd >= RADIUS) { paused = 1; pd = 0.0; } else pd = nd;
                px += AGENT_SPEED * DT;
            }
            py += b.vy * DT;
        }
        t.frame[t.n] = t.n + 1;
        t.follow_d[t.n]    = on_belt(fx, fy) ? fd : -1.0;
        t.pause_d[t.n]     = on_belt(px, py) ? pd : -1.0;
        t.pause_carry[t.n] = fabs(py);
        t.n++;
    }
    return t;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void print_csv(const double* speeds, int ntiers) {
    puts("section,tier,v,policy,frames,max_dev,repaths,budget,selected");
    for (int i = 0; i < ntiers; i++) {
        Belt b = {0.0, speeds[i]};
        SimResult f = sim_follow(&b), p = sim_pause(&b);
        int pick_follow = f.repaths <= budget(f.frames);
        printf("summary,%d,%.2f,follow,%d,%.6f,%d,%d,%d\n",
               i, speeds[i], f.frames, f.max_dev, f.repaths,
               budget(f.frames), pick_follow);
        printf("summary,%d,%.2f,pause,%d,%.6f,%d,%d,%d\n",
               i, speeds[i], p.frames, p.max_dev, p.repaths,
               budget(p.frames), !pick_follow);
    }
    Trace t = trace_fast();
    puts("trace,frame,follow_d,pause_d,pause_carry");
    for (int i = 0; i < t.n; i++)
        printf("trace,%d,%.6f,%.6f,%.6f\n",
               t.frame[i], t.follow_d[i], t.pause_d[i], t.pause_carry[i]);
}

int main(int argc, char** argv) {
    const double speeds[3] = {0.2, 1.0, 3.0};   /* slow / medium / fast */
    const int ntiers = 3;

    if (argc == 2 && strcmp(argv[1], "csv") == 0) {
        print_csv(speeds, ntiers);
        return 0;
    }

    /* --- closed-form expressions ---------------------------------------- */
    check("step_equals_v_dt", fabs(dev_step(1.0) - DT) < EPS);
    check("accum_linear", fabs(accumulated(0.2, 2.0) - 0.4) < EPS);
    check("tmax_inversion", fabs(max_dwell(RADIUS, 1.0) - 0.5) < EPS);
    check("tmax_identity", fabs(1.0 * max_dwell(RADIUS, 1.0) - RADIUS) < EPS);
    check("parallel_belt_unbounded", isinf(max_dwell(RADIUS, 0.0)));

    /* --- three belt speed tiers, both policies, one run each ------------ */
    for (int i = 0; i < ntiers; i++) {
        Belt b = {0.0, speeds[i]};
        SimResult f = sim_follow(&b);
        SimResult p = sim_pause(&b);
        int pick_follow = f.repaths <= budget(f.frames);
        SimResult chosen = pick_follow ? f : p;
        char tag[32];

        printf("tier=%d v=%.2f follow{frames=%d maxdev=%.4f repaths=%d budget=%d} "
               "pause{frames=%d maxdev=%.4f repaths=%d budget=%d} -> %s\n",
               i, speeds[i], f.frames, f.max_dev, f.repaths, budget(f.frames),
               p.frames, p.max_dev, p.repaths, budget(p.frames),
               pick_follow ? "follow" : "pause");

        /* acceptance gates for the selected strategy */
        snprintf(tag, sizeof(tag), "tier%d_dev_within_r", i);
        check(tag, chosen.max_dev <= RADIUS + 1e-12);
        snprintf(tag, sizeof(tag), "tier%d_repath_budget", i);
        check(tag, chosen.repaths <= budget(chosen.frames));

        /* unused policy must also keep the radius; only its repath cost loses */
        snprintf(tag, sizeof(tag), "tier%d_follow_dev_within_r", i);
        check(tag, f.max_dev <= RADIUS + 1e-12);
        snprintf(tag, sizeof(tag), "tier%d_pause_dev_within_r", i);
        check(tag, p.max_dev <= RADIUS + 1e-12);
    }

    /* --- regime-level expectations from the closed forms ---------------- */
    Belt slow = {0.0, 0.2};
    SimResult sf = sim_follow(&slow);
    /* the boundary frame still receives belt push, so all frames
     * contribute; frame-sampled d(t) = v * frames * dt, no repath */
    check("slow_drift_matches_vt", fabs(sf.max_dev - 0.2 * sf.frames * DT) < 1e-9);
    check("slow_needs_no_repath", sf.repaths == 0);            /* t_max = 2.5 s > 2 s dwell */
    check("slow_pause_never_triggers", sim_pause(&slow).suspended == 0);

    Belt fast = {0.0, 3.0};
    SimResult ff = sim_follow(&fast), fp = sim_pause(&fast);
    check("fast_follow_blows_budget", ff.repaths > budget(ff.frames));
    check("fast_pause_single_repath", fp.repaths == 1);
    check("fast_pause_within_budget", fp.repaths <= budget(fp.frames));
    check("fast_pause_holds_radius", fp.max_dev < RADIUS);
    check("fast_selects_pause", ff.repaths > budget(ff.frames) &&
          fp.repaths <= budget(fp.frames));

    /* --- belt parallel to the path: no perpendicular deviation ---------- */
    Belt par = {2.0, 0.0};
    SimResult pf = sim_follow(&par);
    check("parallel_no_deviation", pf.max_dev == 0.0);
    check("parallel_no_repath", pf.repaths == 0);
    check("parallel_perp_zero", perp_speed(&par) == 0.0);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
