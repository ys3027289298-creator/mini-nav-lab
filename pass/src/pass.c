
/* pass: size-aware passability for three agent tiers.
 *
 * Three criteria, all must hold for a corridor to be passable:
 *   width: W* >= 2r                 W* = narrowest width along the corridor
 *   slope: alpha <= atan(mu)        friction-limited max climb angle
 *   turn:  Rb    >= v^2 / (mu g)    lateral grip limits the bend radius
 *
 * Stuck discriminant on the narrowest point:
 *   rho = W* / r ; rho <= 1 no entry, 1 < rho < 2 squeezed in then jammed,
 *   rho >= 2 width criterion satisfied.
 *
 * Wall-slide centrifugal effect (medium tier only):
 *   r_eff(v) = r0 + kc v^2,  W_min(v) = 2 r_eff(v),
 *   v_max(W) = sqrt((W/2 - r0) / kc).
 */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define G 9.81
#define EPS 1e-9
#define NTIER 3
#define NCH 10

enum { L_PASS = 0, L_STUCK, L_NOENTRY, L_SLOPE, L_TURN };

typedef struct { const char* name; double r, mu, v; } Tier;
typedef struct {
    const char* id;
    double widths[3]; int nseg;   /* segment widths; W* = min of them */
    double alpha;                 /* slope angle, degrees */
    double rbend;                 /* path bend radius, m (1e9 = straight) */
} Channel;

static double pi_val;

static const char* lname(int l) {
    static const char* s[] = {"PASS", "STUCK", "NO_ENTRY", "SLOPE_FAIL", "TURN_FAIL"};
    return s[l];
}

static int classify(const Tier* t, const Channel* c) {
    double w = c->widths[0];
    for (int i = 1; i < c->nseg; i++)
        if (c->widths[i] < w) w = c->widths[i];
    double rho = w / t->r;
    if (rho <= 1.0 + EPS) return L_NOENTRY;
    if (rho < 2.0 - EPS) return L_STUCK;
    double amax = atan(t->mu) * 180.0 / pi_val;
    if (c->alpha > amax + EPS) return L_SLOPE;
    double rmin = t->v * t->v / (t->mu * G);
    if (c->rbend < rmin - EPS) return L_TURN;
    return L_PASS;
}

static double narrowest(const Channel* c) {
    double w = c->widths[0];
    for (int i = 1; i < c->nseg; i++)
        if (c->widths[i] < w) w = c->widths[i];
    return w;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    pi_val = acos(-1.0);

    /* measured agent dimensions: r = collision radius, mu = ground friction,
       v = design travel speed. The old doc's r = 0.3 survives as the M tier. */
    Tier tiers[NTIER] = {
        /* name              r     mu    v   */
        { "S_small_pet",     0.15, 0.9, 4.0 },
        { "M_wall_slider",   0.30, 0.7, 6.0 },
        { "L_large_monster", 0.80, 0.5, 3.0 },
    };

    Channel ch[NCH] = {
        /* id   segment widths            n  alpha  rbend */
        { "C1",  {2.10, 2.00, 2.20},      3, 10.0, 8.0 },
        { "C2",  {1.10, 1.00, 1.20},      3, 20.0, 4.0 },
        { "C3",  {0.60, 0.50, 0.55},      3, 30.0, 2.5 },
        { "C4",  {0.45, 0.40, 0.50},      3, 15.0, 6.0 },
        { "C5",  {0.30, 0.25, 0.35},      3, 40.0, 1.5 },
        { "C6",  {1.60, 1.50, 1.70},      3, 28.0, 2.0 },
        { "C7",  {0.80, 0.70, 0.75},      3, 12.0, 1.0 },
        { "C8",  {0.95, 0.85, 0.90},      3,  5.0, 7.0 },
        { "C9",  {1.80, 1.70, 1.90},      3,  8.0, 6.0 },
        { "C10", {0.70, 0.62, 0.66},      3,  0.0, 1e9 },
    };

    /* ---- thresholds reverse-solved from the tier dimensions ---- */
    printf("== thresholds ==\n");
    printf("%-16s %8s %10s %12s\n", "tier", "Wmin(m)", "alpha_max", "Rmin(m)");
    double wmin[NTIER], amax[NTIER], rmin[NTIER];
    for (int t = 0; t < NTIER; t++) {
        wmin[t] = 2.0 * tiers[t].r;
        amax[t] = atan(tiers[t].mu) * 180.0 / pi_val;
        rmin[t] = tiers[t].v * tiers[t].v / (tiers[t].mu * G);
        printf("%-16s %8.3f %9.2f° %12.3f\n", tiers[t].name, wmin[t], amax[t], rmin[t]);
    }
    check("S_thresholds", fabs(wmin[0]-0.30) < 1e-9 &&
          fabs(amax[0]-41.99) < 0.01 && fabs(rmin[0]-1.812) < 0.001);
    check("M_thresholds", fabs(wmin[1]-0.60) < 1e-9 &&
          fabs(amax[1]-34.99) < 0.01 && fabs(rmin[1]-5.242) < 0.001);
    check("L_thresholds", fabs(wmin[2]-1.60) < 1e-9 &&
          fabs(amax[2]-26.57) < 0.01 && fabs(rmin[2]-1.835) < 0.001);

    /* ---- full classification grid; fixture = measured ground truth ---- */
    static const int expect[NTIER][NCH] = {
        /* S */ { L_PASS, L_PASS, L_PASS,  L_PASS,  L_STUCK, L_PASS,  L_TURN, L_PASS,  L_PASS, L_PASS  },
        /* M */ { L_PASS, L_TURN, L_STUCK, L_STUCK, L_NOENTRY, L_TURN, L_TURN, L_PASS, L_PASS, L_PASS  },
        /* L */ { L_PASS, L_STUCK, L_NOENTRY, L_NOENTRY, L_NOENTRY, L_STUCK, L_NOENTRY, L_STUCK, L_PASS, L_NOENTRY },
    };
    printf("\n== classification (rows = tiers, cols = corridors) ==\n");
    printf("%-16s", "tier");
    for (int i = 0; i < NCH; i++) printf("%10s", ch[i].id);
    printf("\n");
    int fixture_match = 1, stuck_count = 0, expect_stuck = 0;
    for (int t = 0; t < NTIER; t++) {
        printf("%-16s", tiers[t].name);
        for (int i = 0; i < NCH; i++) {
            int got = classify(&tiers[t], &ch[i]);
            printf("%10s", lname(got));
            if (got != expect[t][i]) fixture_match = 0;
            if (got == L_STUCK) stuck_count++;
            if (expect[t][i] == L_STUCK) expect_stuck++;
        }
        printf("\n");
    }
    check("stuck_matches_measured", fixture_match && stuck_count == expect_stuck);
    check("stuck_case_count", stuck_count == 6);

    /* ---- narrow corridors (W* < 1 m): misjudgment count per scheme ---- */
    int narrow[NCH], nnarrow = 0;
    for (int i = 0; i < NCH; i++)
        if (narrowest(&ch[i]) < 1.0) narrow[nnarrow++] = i;
    printf("\nnarrow corridors:");
    for (int i = 0; i < nnarrow; i++) printf(" %s", ch[narrow[i]].id);
    printf(" (%d cases per tier)\n", nnarrow);

    /* scheme 1: unified r = 0.3 rule for every agent (the obsolete doc) */
    Tier uni[NTIER];
    for (int t = 0; t < NTIER; t++) uni[t] = tiers[1];
    /* scheme 2: two tiers, small pet merged into the wall-slider params */
    Tier two[NTIER] = { tiers[1], tiers[1], tiers[2] };
    /* scheme 3: three dedicated tiers */
    const Tier* schemes[3][NTIER] = {
        { &uni[0], &uni[1], &uni[2] },
        { &two[0], &two[1], &two[2] },
        { &tiers[0], &tiers[1], &tiers[2] },
    };
    int mis[3] = {0, 0, 0}, total_mis[3] = {0, 0, 0};
    for (int s = 0; s < 3; s++) {
        for (int t = 0; t < NTIER; t++) {
            for (int i = 0; i < NCH; i++) {
                int got = classify(schemes[s][t], &ch[i]);
                if (got != expect[t][i]) {
                    total_mis[s]++;
                    for (int k = 0; k < nnarrow; k++)
                        if (narrow[k] == i) mis[s]++;
                }
            }
        }
    }
    printf("\n== scheme comparison (narrow set / full 30-case set) ==\n");
    printf("unified 1-tier: misjudge %d / %d, params maintained %d\n", mis[0], total_mis[0], 3);
    printf("two-tier      : misjudge %d / %d, params maintained %d\n", mis[1], total_mis[1], 6);
    printf("three-tier    : misjudge %d / %d, params maintained %d (+1 wall-slide kc)\n",
           mis[2], total_mis[2], 9);
    check("unified_narrow_misjudge_8", mis[0] == 8 && total_mis[0] == 12);
    check("twotier_narrow_misjudge_3", mis[1] == 3 && total_mis[1] == 5);
    check("threetier_narrow_misjudge_zero", mis[2] == 0 && total_mis[2] == 0);

    /* ---- wall-slider at speed: centrifugal widening of the envelope ---- */
    /* Calibration anchor: at v_max the effective radius grows by 50%,
       so kc = 0.5 r0 / v_max^2 = 1/240 s^2/m. */
    double r0 = tiers[1].r, vmax = tiers[1].v;
    double kc = 0.5 * r0 / (vmax * vmax);
    double reff = r0 + kc * vmax * vmax;
    printf("\n== M wall-slide speed effect, kc = %.6f s^2/m ==\n", kc);
    printf("r_eff(%.1f) = %.3f m, W_min(%.1f) = %.3f m\n", vmax, reff, vmax, 2.0 * reff);
    check("reff_growth_50pct", fabs(reff - 1.5 * r0) < 1e-12);

    printf("%-6s %8s %10s %12s %s\n", "corr", "W*(m)", "static", "v_max(m/s)", "high-speed verdict");
    int detour[NCH], ndet = 0;
    for (int i = 0; i < NCH; i++) {
        if (classify(&tiers[1], &ch[i]) != L_PASS) continue;  /* only static-passable ones */
        double w = narrowest(&ch[i]);
        double vpass = sqrt(fmax(0.0, (w / 2.0 - r0) / kc));
        int blocked_at_speed = 2.0 * (r0 + kc * vmax * vmax) > w + EPS;
        if (blocked_at_speed) detour[ndet++] = i;
        printf("%-6s %8.3f %10s %12.3f %s\n", ch[i].id, w, "PASS", vpass,
               blocked_at_speed ? "DETOUR" : "ok");
    }
    check("detour_C8_C10", ndet == 2 &&
          strcmp(ch[detour[0]].id, "C8") == 0 && strcmp(ch[detour[1]].id, "C10") == 0);
    /* spot-check the speed limits themselves: sqrt(30) and sqrt(2.4) */
    double v_c8 = sqrt(fmax(0.0, (narrowest(&ch[7]) / 2.0 - r0) / kc));
    double v_c10 = sqrt(fmax(0.0, (narrowest(&ch[9]) / 2.0 - r0) / kc));
    check("vmax_C8_sqrt30", fabs(v_c8 - sqrt(30.0)) < 1e-9);
    check("vmax_C10_sqrt2p4", fabs(v_c10 - sqrt(2.4)) < 1e-9);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
