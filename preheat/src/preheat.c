
/* preheat: dwell-weighted stutter metric and visibility-preheat region selection.
 *
 * Background: players spend most of their time in the main-city regions, but the
 * reported pathfinding cost is averaged uniformly over regions, so the idle
 * wilderness regions dilute the city stutter. This module replaces the plain
 * average with a dwell-weighted metric, works out how many regions must be
 * preheated to cover 80% of the dwell time, and checks that choice against the
 * pre-baked path memory budget. */
#include <stdio.h>
#include <string.h>

#define MAXN 16
#define TARGET 0.80   /* required preheat coverage */
#define RATIO  0.60   /* weighted stutter must drop below this fraction of the average */
#define MEM_CAP 700.0 /* pre-baked path memory budget, MB */
#define EPS 1e-9

/* N regions: the first 3 are main cities, the rest are wilderness. */
static const char* REGION[MAXN] = {
    "C1", "C2", "C3", "W1", "W2", "W3", "W4", "W5"
};
static const double LAG[MAXN] = {
    120.0, 110.0, 100.0, 35.0, 30.0, 25.0, 20.0, 15.0
}; /* per-call pathfinding stutter when not preheated, ms */
static const double MEM[MAXN] = {
    240.0, 220.0, 190.0, 90.0, 80.0, 70.0, 60.0, 50.0
}; /* pre-baked path memory per region, MB */

typedef struct {
    const char* name;
    double dwell[MAXN];
    int n;
} Scenario;

/* three dwell distributions: concentrated / balanced / dispersed */
static Scenario SCEN[] = {
    { "concentrated", {0.34, 0.28, 0.18, 0.07, 0.05, 0.04, 0.03, 0.01}, 8 },
    { "balanced",     {0.30, 0.27, 0.23, 0.07, 0.05, 0.04, 0.03, 0.01}, 8 },
    { "dispersed",    {0.30, 0.24, 0.18, 0.10, 0.07, 0.05, 0.04, 0.02}, 8 },
};
#define NSCEN (int)(sizeof(SCEN) / sizeof(SCEN[0]))

static int failures = 0;
static void check(const char* name, int ok) {
    printf("  check %s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* rank regions by dwell share, largest first */
static void rank_by_dwell(const Scenario* s, int* order) {
    for (int i = 0; i < s->n; i++) order[i] = i;
    for (int i = 1; i < s->n; i++) {
        int cur = order[i], j = i - 1;
        while (j >= 0 && s->dwell[order[j]] < s->dwell[cur]) {
            order[j + 1] = order[j];
            j--;
        }
        order[j + 1] = cur;
    }
}

/* plain per-region average: the metric that currently masks city stutter */
static double plain_average(const Scenario* s) {
    double sum = 0.0;
    for (int i = 0; i < s->n; i++) sum += LAG[i];
    return sum / s->n;
}

/* dwell-weighted stutter metric: L_w = sum_i p_i * l_i */
static double weighted_lag(const Scenario* s) {
    double w = 0.0;
    for (int i = 0; i < s->n; i++) w += s->dwell[i] * LAG[i];
    return w;
}

/* residual weighted stutter after preheating k regions (preheated lag -> 0) */
static double residual_lag(const Scenario* s, const int* order, int k) {
    int preheated[MAXN] = {0};
    for (int i = 0; i < k; i++) preheated[order[i]] = 1;
    double w = 0.0;
    for (int i = 0; i < s->n; i++)
        if (!preheated[i]) w += s->dwell[i] * LAG[i];
    return w;
}

static double coverage(const Scenario* s, const int* order, int k) {
    double c = 0.0;
    for (int i = 0; i < k; i++) c += s->dwell[order[i]];
    return c;
}

static double preheated_mem(const int* order, int k) {
    double m = 0.0;
    for (int i = 0; i < k; i++) m += MEM[order[i]];
    return m;
}

/* smallest k whose cumulative dwell reaches the target; -1 if impossible */
static int min_regions_for_target(const Scenario* s, const int* order) {
    double c = 0.0;
    for (int k = 1; k <= s->n; k++) {
        c += s->dwell[order[k - 1]];
        if (c + EPS >= TARGET) return k;
    }
    return -1;
}

static void run_scenario(const Scenario* s) {
    int order[MAXN];
    rank_by_dwell(s, order);

    double dwell_sum = 0.0;
    for (int i = 0; i < s->n; i++) dwell_sum += s->dwell[i];

    printf("scenario=%s n_regions=%d cities=3\n", s->name, s->n);
    check("dwell_sums_to_one", dwell_sum > 1.0 - EPS && dwell_sum < 1.0 + EPS);
    check("rank_is_dwell_sorted",
          s->dwell[order[0]] >= s->dwell[order[1]] &&
          s->dwell[order[s->n - 2]] >= s->dwell[order[s->n - 1]]);

    double avg = plain_average(s);
    double w0 = weighted_lag(s);
    printf("  plain_average_lag=%.2f ms  weighted_lag=%.2f ms  ratio=%.3f\n",
           avg, w0, w0 / avg);
    check("weighted_exceeds_average", w0 > avg);

    /* coverage of preheating only the three main cities */
    double city_cov = s->dwell[0] + s->dwell[1] + s->dwell[2];
    printf("  city_only_coverage(top3 dwell)=%.4f\n", city_cov);

    int kmin = min_regions_for_target(s, order);
    printf("  regions_for_%.0f%%_coverage k_min=%d of %d regions (%.1f%% of map)\n",
           TARGET * 100, kmin, s->n, 100.0 * kmin / s->n);

    /* the two decision curves, one row per preheat count */
    printf("  k | coverage | memory_MB | residual_lag_ms | lag_below_60pct_avg | mem_within_cap\n");
    for (int k = 1; k <= s->n; k++) {
        double cov = coverage(s, order, k);
        double mem = preheated_mem(order, k);
        double res = residual_lag(s, order, k);
        printf("  %d | %.4f   | %7.1f  | %8.2f      | %d                  | %d\n",
               k, cov, mem, res, res + EPS < RATIO * avg, mem <= MEM_CAP + EPS);
    }

    /* verdict at the smallest region count that meets the coverage target */
    double mem_at_k = preheated_mem(order, kmin);
    double res_at_k = residual_lag(s, order, kmin);
    int cov_ok = coverage(s, order, kmin) + EPS >= TARGET;
    int lag_ok = res_at_k + EPS < RATIO * avg;
    int mem_ok = mem_at_k <= MEM_CAP + EPS;
    int feasible = cov_ok && lag_ok && mem_ok;

    printf("  verdict k=%d: coverage_ok=%d lag_ok=%d(%.2f < %.2f) mem_ok=%d(%.1f <= %.0f) feasible=%d\n",
           kmin, cov_ok, lag_ok, res_at_k, RATIO * avg, mem_ok, mem_at_k, MEM_CAP, feasible);

    if (!feasible) {
        /* name the missing region: the kmin-th region is the one the coverage
         * target just pulled into the preheat set */
        int gap = order[kmin - 1];
        const char* why = !cov_ok ? "coverage" : !mem_ok ? "memory" : "lag";
        printf("  GAP: constraint=%s missing_region=%s (dwell=%.2f mem=%.0fMB)\n",
               why, REGION[gap], s->dwell[gap], MEM[gap]);
        if (!mem_ok)
            printf("  GAP: region %s closes coverage but preheating %d regions costs %.1f MB (cap %.0f MB, over by %.1f MB)\n",
                   REGION[gap], kmin, mem_at_k, MEM_CAP, mem_at_k - MEM_CAP);
    }
    printf("\n");
}

int main(void) {
    for (int i = 0; i < NSCEN; i++) run_scenario(&SCEN[i]);

    /* fixed expectations: the two 80/20 distributions hit the target with the 3
     * cities; the dispersed distribution needs a 4th region that blows the cap. */
    int order_c[MAXN], order_b[MAXN], order_d[MAXN];
    rank_by_dwell(&SCEN[0], order_c);
    rank_by_dwell(&SCEN[1], order_b);
    rank_by_dwell(&SCEN[2], order_d);
    check("concentrated_kmin_is_3", min_regions_for_target(&SCEN[0], order_c) == 3);
    check("balanced_kmin_is_3", min_regions_for_target(&SCEN[1], order_b) == 3);
    check("dispersed_kmin_is_4", min_regions_for_target(&SCEN[2], order_d) == 4);
    check("dispersed_k4_over_memory_cap",
          preheated_mem(order_d, 4) > MEM_CAP);
    check("dispersed_k3_under_memory_cap",
          preheated_mem(order_d, 3) <= MEM_CAP);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
