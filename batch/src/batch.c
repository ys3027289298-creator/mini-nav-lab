
/* batch: coalesce path requests that target the same destination region.
 *
 * Derivation used by the cost model:
 *   N requests, R distinct destination regions, one A* search costs C.
 *     direct total work : T_dir   = N * C
 *     merged total work : T_merge = R * C + G(N),  G(N) = g * log2(N)
 *   gross saving fraction = (N-R)/N = 1 - R/N   (speed-up ~ N/R)
 *   net merge condition   : (N-R)*C > g * log2(N)
 *
 * Latency is a separate curve.  We assume the job system can run the
 * region searches concurrently, which is the worst case for coalescing:
 * with no queueing delay for merging to remove, the only added critical
 * path is waiting for grouping to finish.
 *     L_dir   = C + H
 *     L_merge = g*log2(N) + C + H
 *   gate: L_merge / L_dir - 1 <= 10%
 *
 * `search_region' is the metred seam to the A* routine in grid/src/grid.c:
 * the run counter is what proves same-region duplicate work is gone.
 */
#include <stdio.h>
#include <math.h>
#include <string.h>

#define MAX_N 256
#define N_REGIONS 6      /* destination-region palette of the workload */
#define PLAN_REGIONS 6   /* conservative distinct-region count for sizing */

static const double C_SEARCH_US  = 800.0;  /* one path search */
static const double GROUP_G_US   = 12.0;   /* G(N) = g * log2(N) */
static const double HANDOFF_US   = 1.0;    /* per-request reply hand-off H */
static const double LATENCY_CAP  = 0.10;   /* average latency increase <= 10% */

/* ---------------------------------------------------------------- model */

/* stipulated grouping overhead: proportional to log of request count */
static double group_cost_us(int n) {
    return GROUP_G_US * log2((double)(n < 1 ? 1 : n));
}

/* conservative planning assumption: a batch touches at most PLAN_REGIONS
 * regions, and never more regions than requests */
static int plan_regions(int n) {
    return n < PLAN_REGIONS ? n : PLAN_REGIONS;
}

/* net compute saving in microseconds: (N-R)*C - g*log2(N) */
static double net_saving_us(int n) {
    int r = plan_regions(n);
    return (double)(n - r) * C_SEARCH_US - group_cost_us(n);
}

static double latency_inflation(int n) {
    return group_cost_us(n) / (C_SEARCH_US + HANDOFF_US);
}

/* smallest N for which compute saved beats grouping cost */
static int compute_threshold_min(void) {
    for (int n = 1; n <= MAX_N * 64; n++)
        if (net_saving_us(n) > 0.0) return n;
    return -1;
}

/* largest N for which the latency inflation gate (<=10%) still holds */
static int latency_threshold_max(void) {
    int best = -1;
    for (int n = 1; n <= 1000000; n++) {
        if (latency_inflation(n) <= LATENCY_CAP + 1e-12) best = n;
        else if (best > 0) break; /* monotone after n>=1 */
    }
    return best;
}

/* policy used by the frame loop: both curves must pass */
static int should_merge(int n) {
    return net_saving_us(n) > 0.0 && latency_inflation(n) <= LATENCY_CAP + 1e-12;
}

/* ------------------------------------------------------------- workload */

/* deterministic request stream: every even request targets region 0,
 * i.e. at N=30 exactly half share one destination region; odd requests
 * cycle through regions 1..N_REGIONS-1. */
static int destination_region(int i) {
    if ((i & 1) == 0) return 0;
    return 1 + ((i / 2) % (N_REGIONS - 1));
}

typedef struct {
    int n;
    int region[MAX_N];
    int msize[N_REGIONS];
    int members[N_REGIONS][MAX_N];
    int distinct;
} Batch;

static void group_by_region(Batch* b, int n) {
    memset(b, 0, sizeof(*b));
    b->n = n;
    for (int i = 0; i < n; i++) {
        int r = destination_region(i);
        b->region[i] = r;
        if (b->msize[r] == 0) b->distinct++;
        b->members[r][b->msize[r]++] = i;
    }
}

/* metred search seam */
typedef struct {
    int total_runs;
    int region_runs[N_REGIONS];
    double work_us;
} Meter;

static int search_region(Meter* m, int region) {
    m->total_runs++;
    m->region_runs[region]++;
    m->work_us += C_SEARCH_US;
    return m->total_runs; /* unique id of this search's result object */
}

/* independent path run for every request */
static void run_direct(const Batch* b, Meter* m, int* route_id, double* lat_us) {
    for (int i = 0; i < b->n; i++) {
        route_id[i] = search_region(m, b->region[i]);
        lat_us[i] = C_SEARCH_US + HANDOFF_US; /* concurrent job threads */
    }
}

/* one search per distinct region; every member shares that result */
static void run_merged(const Batch* b, Meter* m, int* route_id, double* lat_us) {
    double wait_group = group_cost_us(b->n);
    for (int r = 0; r < N_REGIONS; r++) {
        if (b->msize[r] == 0) continue;
        int id = search_region(m, r);
        for (int k = 0; k < b->msize[r]; k++) {
            int i = b->members[r][k];
            route_id[i] = id;
            lat_us[i] = wait_group + C_SEARCH_US + HANDOFF_US;
        }
    }
}

static double avg(const double* v, int n) {
    double s = 0.0;
    for (int i = 0; i < n; i++) s += v[i];
    return n > 0 ? s / n : 0.0;
}

/* ------------------------------------------------------------ reporting */

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int routes_shared_by_group(const Batch* b, const int* route_id) {
    for (int r = 0; r < N_REGIONS; r++) {
        if (b->msize[r] == 0) continue;
        int id = route_id[b->members[r][0]];
        if (id <= 0) return 0;
        for (int k = 1; k < b->msize[r]; k++)
            if (route_id[b->members[r][k]] != id) return 0;
    }
    return 1;
}

static int each_region_searched_once(const Batch* b, const Meter* m) {
    for (int r = 0; r < N_REGIONS; r++) {
        int want = b->msize[r] > 0 ? 1 : 0;
        if (m->region_runs[r] != want) return 0;
    }
    return 1;
}

static void write_curves_csv(int n_lo, int n_hi) {
    FILE* f = fopen("curves.csv", "w");
    if (!f) { perror("curves.csv"); return; }
    fprintf(f, "n,r_plan,t_dir_us,t_merge_us,net_saving_us,group_us,"
               "inflation_pct,worth_compute,latency_ok,merge_ok\n");
    for (int n = n_lo; n <= n_hi; n++) {
        int r = plan_regions(n);
        double g = group_cost_us(n);
        double tdir = (double)n * C_SEARCH_US;
        double tmerge = (double)r * C_SEARCH_US + g;
        double infl = latency_inflation(n) * 100.0;
        int worth = net_saving_us(n) > 0.0;
        int lok = latency_inflation(n) <= LATENCY_CAP + 1e-12;
        fprintf(f, "%d,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%d,%d,%d\n",
                n, r, tdir, tmerge, tdir - tmerge, g, infl,
                worth, lok, worth && lok);
    }
    fclose(f);
}

typedef struct {
    int n, k;
    int direct_runs, merged_runs;
    double work_before_us, work_after_us;
    double lat_direct_us, lat_merged_us, inflation_pct;
    int dup_ok, share_ok, once_ok, latency_ok, policy_merge;
} TierResult;

static TierResult run_tier(int n) {
    Batch b;
    group_by_region(&b, n);

    Meter md = {0};
    int route_d[MAX_N]; double lat_d[MAX_N];
    run_direct(&b, &md, route_d, lat_d);

    Meter mm = {0};
    int route_m[MAX_N]; double lat_m[MAX_N];
    run_merged(&b, &mm, route_m, lat_m);

    TierResult t;
    memset(&t, 0, sizeof(t));
    t.n = n;
    t.k = b.distinct;
    t.direct_runs = md.total_runs;
    t.merged_runs = mm.total_runs;
    t.work_before_us = md.work_us;
    t.work_after_us = mm.work_us + group_cost_us(n);
    t.lat_direct_us = avg(lat_d, n);
    t.lat_merged_us = avg(lat_m, n);
    t.inflation_pct = 100.0 * (t.lat_merged_us / t.lat_direct_us - 1.0);
    t.dup_ok = (mm.total_runs == b.distinct);
    t.share_ok = routes_shared_by_group(&b, route_m);
    t.once_ok = each_region_searched_once(&b, &mm);
    t.latency_ok = t.lat_merged_us <= t.lat_direct_us * (1.0 + LATENCY_CAP) + 1e-9;
    t.policy_merge = should_merge(n);

    int sizes[N_REGIONS], ns = 0;
    for (int r = 0; r < N_REGIONS; r++) if (b.msize[r] > 0) sizes[ns++] = b.msize[r];

    printf("\n[tier N=%d] distinct regions R=%d, group sizes=[", n, b.distinct);
    for (int i = 0; i < ns; i++) printf("%d%s", sizes[i], i + 1 < ns ? "," : "");
    printf("]\n");
    printf("  searches      : direct=%d  merged=%d  duplicate_runs_removed=%d\n",
           md.total_runs, mm.total_runs, n - mm.total_runs);
    printf("  compute work  : before=%.1f us  after=%.1f us  net_saved=%.1f us (%.1f%%)\n",
           t.work_before_us, t.work_after_us,
           t.work_before_us - t.work_after_us,
           100.0 * (1.0 - t.work_after_us / t.work_before_us));
    printf("  avg latency   : direct=%.2f us  merged=%.2f us  inflation=%.2f%% (cap 10%%)\n",
           t.lat_direct_us, t.lat_merged_us, t.inflation_pct);
    printf("  frame policy  : %s\n", t.policy_merge ? "MERGE" : "DIRECT (below threshold / over latency cap)");

    char name[64];
    snprintf(name, sizeof(name), "tier%d_direct_runs_n", n);
    check(name, md.total_runs == n);
    snprintf(name, sizeof(name), "tier%d_dup_eliminated", n);
    check(name, t.dup_ok);
    snprintf(name, sizeof(name), "tier%d_each_region_once", n);
    check(name, t.once_ok);
    snprintf(name, sizeof(name), "tier%d_routes_shared", n);
    check(name, t.share_ok);
    snprintf(name, sizeof(name), "tier%d_latency_le_10pct", n);
    check(name, t.latency_ok);
    snprintf(name, sizeof(name), "tier%d_policy_merge", n);
    check(name, t.policy_merge == 1);
    return t;
}

static void write_tiers_csv(const TierResult* ts, int count) {
    FILE* f = fopen("tiers.csv", "w");
    if (!f) { perror("tiers.csv"); return; }
    fprintf(f, "n,k,direct_runs,merged_runs,work_before_us,work_after_us,"
               "lat_direct_us,lat_merged_us,inflation_pct,latency_ok\n");
    for (int i = 0; i < count; i++) {
        const TierResult* t = &ts[i];
        fprintf(f, "%d,%d,%d,%d,%.3f,%.3f,%.3f,%.3f,%.4f,%d\n",
                t->n, t->k, t->direct_runs, t->merged_runs,
                t->work_before_us, t->work_after_us,
                t->lat_direct_us, t->lat_merged_us, t->inflation_pct,
                t->latency_ok);
    }
    fclose(f);
}

int main(void) {
    const int tiers[3] = {10, 30, 60}; /* low / typical (30 per frame) / high */
    TierResult results[3];

    int n_min = compute_threshold_min();
    int n_max = latency_threshold_max();

    printf("batch: coalesce path requests by destination region\n");
    printf("model: C=%.1f us/search, G(N)=%.1f*log2(N), H=%.1f us, latency cap=%.0f%%\n",
           C_SEARCH_US, GROUP_G_US, HANDOFF_US, 100.0 * LATENCY_CAP);
    printf("gross saving = 1-R/N (speed-up ~ N/R); planning R=min(N,%d)\n", PLAN_REGIONS);
    printf("compute threshold : (N-R)*C > g*log2(N) -> N_min=%d\n", n_min);
    printf("latency threshold : g*log2(N)/(C+H) <= 0.10 -> N_max=%d\n", n_max);
    printf("feasible window   : %d <= N <= %d\n", n_min, n_max);

    write_curves_csv(1, 128);

    for (int i = 0; i < 3; i++) results[i] = run_tier(tiers[i]);
    write_tiers_csv(results, 3);

    printf("\n-- threshold self-checks --\n");
    check("compute_n_min_is_7", n_min == 7);
    check("n6_compute_not_worth", net_saving_us(6) <= 0.0);
    check("n7_compute_worth", net_saving_us(7) > 0.0);
    check("below_threshold_policy_direct", should_merge(n_min - 1) == 0);
    check("at_threshold_policy_merge", should_merge(n_min) == 1);
    check("latency_n_max_is_102", n_max == 102);
    check("n102_latency_within_cap", latency_inflation(102) <= LATENCY_CAP + 1e-12);
    check("n103_latency_over_cap", latency_inflation(103) > LATENCY_CAP + 1e-12);
    check("three_tiers_10_30_60",
          results[0].n == 10 && results[1].n == 30 && results[2].n == 60);

    int all_latency = 1, all_dup = 1;
    for (int i = 0; i < 3; i++) {
        all_latency &= results[i].latency_ok;
        all_dup &= results[i].dup_ok && results[i].share_ok && results[i].once_ok;
    }
    check("all_tiers_duplicate_free", all_dup);
    check("all_tiers_latency_le_10pct", all_latency);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
