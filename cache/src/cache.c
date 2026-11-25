
/* cache: path-cache pressure test (prewarm hit rate vs capacity under concurrency).
 *
 * Model (see README.md for the derivation):
 *   active set   A(k) = HEAD_N + M_TAIL * k      (shared expensive head + per-worker tails)
 *   LRU hit rate h(C,k) = min(1, C / A(k))       (uniform access over the active set)
 *   required     h >= 0.60  =>  C >= 0.60 * A(k)
 *
 * Every config replays an identical deterministic request stream so the two
 * eviction policies and the capacity sweep are directly comparable.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>

#define MW 64
#define MH 48
#define NCELL (MW * MH)
#define POOL_MAX 1200
#define HEAD_N 200          /* prewarmed, most expensive paths */
#define M_TAIL 2            /* unique tail paths per concurrent worker */
#define TIERS 3
#define MEM_LIMIT_BYTES (96 * 1024)
#define HIT_TARGET 0.60
#define ENTRY_OVERHEAD 24   /* cache bookkeeping per entry, bytes */
#define PATH_ELEM 4         /* sizeof(int) per path cell */

typedef struct { unsigned char blocked[NCELL]; } Map;

static int idx(int x, int y) { return y * MW + x; }

/* ---------------- deterministic RNG (splitmix64) ---------------- */
static uint64_t rng_state = 1;
static void seed_rng(uint64_t s) { rng_state = s; }
static uint64_t rnd64(void) {
    uint64_t z = (rng_state += 0x9e3779b97f4a7c15ULL);
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}
static double rnd01(void) { return (rnd64() >> 11) * (1.0 / 9007199254740992.0); }

static Map g_map;

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= MW || y >= MH) return 0;
    return !m->blocked[idx(x, y)];
}

/* ---------------- A* with expansion counter ---------------- */
typedef struct { int f, node; } OpenNode;

static int astar(const Map* m, int sx, int sy, int gx, int gy, int* expand) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    static int g[NCELL], fval[NCELL], parent[NCELL];
    static unsigned char closed[NCELL];
    static OpenNode heap[NCELL * 8];
    int hn = 0;
    for (int i = 0; i < NCELL; i++) { g[i] = 1 << 29; fval[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int s = idx(sx, sy), goal = idx(gx, gy);
    g[s] = 0;
    fval[s] = abs(gx - sx) + abs(gy - sy);
    heap[hn++] = (OpenNode){ fval[s], s };
    int exp = 0;
    while (hn > 0) {
        OpenNode top = heap[0];
        heap[0] = heap[--hn];
        for (int i = 0; ; ) {
            int l = 2 * i + 1, r = l + 1, mi = i;
            if (l < hn && heap[l].f < heap[mi].f) mi = l;
            if (r < hn && heap[r].f < heap[mi].f) mi = r;
            if (mi == i) break;
            OpenNode t = heap[i]; heap[i] = heap[mi]; heap[mi] = t; i = mi;
        }
        int cur = top.node;
        if (closed[cur]) continue;
        closed[cur] = 1;
        exp++;
        if (cur == goal) {
            int n = 0, c = cur;
            while (c != -1) { n++; c = parent[c]; }
            *expand = exp;
            return n;
        }
        int cx = cur % MW, cy = cur / MW;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                fval[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;
                heap[hn++] = (OpenNode){ fval[ni], ni };
                for (int i = hn - 1; i > 0; ) {
                    int p = (i - 1) / 2;
                    if (heap[p].f <= heap[i].f) break;
                    OpenNode t = heap[i]; heap[i] = heap[p]; heap[p] = t; i = p;
                }
            }
        }
    }
    *expand = exp;
    return -1;
}

/* ---------------- path pool ---------------- */
typedef struct { int len, cost, sx, sy, gx, gy; } PInfo;
static PInfo pool[POOL_MAX];
static int pool_n = 0;
static int g_cost[POOL_MAX], g_len[POOL_MAX];

static void build_map(void) {
    seed_rng(0x1234);
    for (int i = 0; i < NCELL; i++) g_map.blocked[i] = rnd01() < 0.12;
    /* keep border cells open so every corner stays reachable */
    for (int x = 0; x < MW; x++) { g_map.blocked[idx(x, 0)] = 0; g_map.blocked[idx(x, MH - 1)] = 0; }
    for (int y = 0; y < MH; y++) { g_map.blocked[idx(0, y)] = 0; g_map.blocked[idx(MW - 1, y)] = 0; }
}

static void build_pool(void) {
    seed_rng(0x5678);
    int tries = 0;
    while (pool_n < POOL_MAX && tries < POOL_MAX * 60) {
        tries++;
        int sx = (int)(rnd64() % MW), sy = (int)(rnd64() % MH);
        int gx = (int)(rnd64() % MW), gy = (int)(rnd64() % MH);
        int exp = 0;
        int len = astar(&g_map, sx, sy, gx, gy, &exp);
        if (len <= 0) continue;
        pool[pool_n++] = (PInfo){ len, exp, sx, sy, gx, gy };
    }
}

static int cmp_cost_desc(const void* a, const void* b) {
    int ia = *(const int*)a, ib = *(const int*)b;
    if (pool[ia].cost != pool[ib].cost) return pool[ib].cost - pool[ia].cost;
    return ia - ib;
}

/* ids 0..HEAD_N-1: the prewarmed expensive head (descending cost).
 * ids HEAD_N..: tail, shuffled so the active tail mix is cost-diverse. */
static void order_pool(void) {
    static int ids[POOL_MAX];
    for (int i = 0; i < pool_n; i++) ids[i] = i;
    qsort(ids, pool_n, sizeof(int), cmp_cost_desc);

    static PInfo sorted[POOL_MAX];
    for (int i = 0; i < pool_n; i++) sorted[i] = pool[ids[i]];

    seed_rng(0x9abc);
    for (int i = HEAD_N; i < pool_n; i++) {
        int j = HEAD_N + (int)(rnd64() % (uint64_t)(pool_n - HEAD_N));
        PInfo t = sorted[i]; sorted[i] = sorted[j]; sorted[j] = t;
    }
    memcpy(pool, sorted, sizeof(PInfo) * pool_n);
    for (int i = 0; i < pool_n; i++) { g_cost[i] = pool[i].cost; g_len[i] = pool[i].len; }
}

/* ---------------- cache ---------------- */
enum { POL_LRU = 0, POL_COST = 1 };

typedef struct { double pri; int id, ver; } HEnt;

typedef struct {
    int cap, policy, count;
    unsigned char present[POOL_MAX];
    unsigned char is_prewarm[POOL_MAX];
    int last_use[POOL_MAX], freq[POOL_MAX], ver[POOL_MAX];
    long long bytes, peak;
    long long clock;
    HEnt* heap; int hn, heap_cap;
    long long prewarm_evicted;
} Cache;

static int entry_bytes(int id) { return ENTRY_OVERHEAD + PATH_ELEM * g_len[id]; }

static double prio_of(const Cache* c, int id) {
    double use = (double)c->last_use[id];
    if (c->policy == POL_LRU) return use;
    /* cost-aware: evict the cheapest recompute first, recency breaks ties */
    return (double)g_cost[id] * 1e7 + use;
}

static void heap_push(Cache* c, int id);

/* Lazy-deletion heaps accumulate stale records, so rebuild when full. */
static void heap_rebuild(Cache* c) {
    int n = c->hn;
    c->hn = 0;
    for (int id = 0; id < POOL_MAX; id++) {
        if (!c->present[id]) continue;
        HEnt e = { prio_of(c, id), id, c->ver[id] };
        int i = c->hn++;
        c->heap[i] = e;
        while (i > 0) {
            int p = (i - 1) / 2;
            if (c->heap[p].pri <= c->heap[i].pri) break;
            HEnt t = c->heap[p]; c->heap[p] = c->heap[i]; c->heap[i] = t;
            i = p;
        }
    }
    (void)n;
}

static void heap_push(Cache* c, int id) {
    if (c->hn == c->heap_cap) {
        heap_rebuild(c);
        if (c->hn == c->heap_cap) return; /* still full: cannot happen (cap<=POOL_MAX) */
    }
    HEnt e = { prio_of(c, id), id, c->ver[id] };
    int i = c->hn++;
    c->heap[i] = e;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (c->heap[p].pri <= c->heap[i].pri) break;
        HEnt t = c->heap[p]; c->heap[p] = c->heap[i]; c->heap[i] = t;
        i = p;
    }
}

static void heap_pop(Cache* c) {
    c->heap[0] = c->heap[--c->hn];
    int i = 0;
    for (;;) {
        int l = 2 * i + 1, r = l + 1, mi = i;
        if (l < c->hn && c->heap[l].pri < c->heap[mi].pri) mi = l;
        if (r < c->hn && c->heap[r].pri < c->heap[mi].pri) mi = r;
        if (mi == i) break;
        HEnt t = c->heap[i]; c->heap[i] = c->heap[mi]; c->heap[mi] = t;
        i = mi;
    }
}

static void cache_evict(Cache* c) {
    while (c->hn > 0) {
        HEnt top = c->heap[0];
        heap_pop(c);
        int id = top.id;
        if (!c->present[id] || c->ver[id] != top.ver) continue; /* stale heap record */
        c->present[id] = 0;
        c->count--;
        c->bytes -= entry_bytes(id);
        if (c->is_prewarm[id]) c->prewarm_evicted++;
        return;
    }
}

static void cache_reset(Cache* c, int cap, int policy, HEnt* heap, int heap_cap) {
    memset(c, 0, sizeof(*c));
    c->cap = cap; c->policy = policy;
    c->heap = heap; c->heap_cap = heap_cap;
}

static void cache_insert(Cache* c, int id) {
    if (c->count >= c->cap) cache_evict(c);
    c->present[id] = 1;
    c->last_use[id] = (int)c->clock;
    c->freq[id] = 1;
    c->ver[id]++;
    c->count++;
    c->bytes += entry_bytes(id);
    if (c->bytes > c->peak) c->peak = c->bytes;
    heap_push(c, id);
}

static void cache_touch(Cache* c, int id) {
    c->last_use[id] = (int)c->clock;
    c->freq[id]++;
    c->ver[id]++;
    heap_push(c, id);
}

/* ---------------- simulation ---------------- */
typedef struct {
    double prewarm_hit;   /* hits / requests for prewarmed (head) paths */
    double overall_hit;
    double cpu_per_req;   /* A* nodes re-expanded per request */
    long long peak_bytes;
    int entries, active, prewarm_evicted;
} Metrics;

static Metrics run(int cap, int policy, int workers, int reqs_per_worker, uint64_t seed) {
    int active = HEAD_N + M_TAIL * workers;
    int total = workers * reqs_per_worker;

    static HEnt heap_buf[POOL_MAX * 8];
    Cache c;
    cache_reset(&c, cap, policy, heap_buf, POOL_MAX * 8);

    /* prewarm the head; these entries start as the oldest */
    for (int i = 0; i < HEAD_N && i < cap; i++) {
        cache_insert(&c, i);
        c.is_prewarm[i] = 1;
    }
    for (int i = 0; i < pool_n; i++) if (c.present[i]) c.last_use[i] = 0;
    c.clock = 0;

    seed_rng(seed);
    long long hits = 0, preq = 0, phit = 0, miss_cost = 0;
    for (int round = 0; round < reqs_per_worker; round++) {
        for (int w = 0; w < workers; w++) {
            c.clock++;
            int id = (int)(rnd64() % (uint64_t)active);
            if (c.present[id]) {
                hits++;
                cache_touch(&c, id);
                if (id < HEAD_N) { preq++; phit++; }
            } else {
                miss_cost += g_cost[id];
                cache_insert(&c, id);
                if (id < HEAD_N) preq++;
            }
        }
    }

    Metrics m;
    m.prewarm_hit = preq ? (double)phit / (double)preq : 0.0;
    m.overall_hit = (double)hits / (double)total;
    m.cpu_per_req = (double)miss_cost / (double)total;
    m.peak_bytes = c.peak;
    m.entries = c.count;
    m.active = active;
    m.prewarm_evicted = (int)c.prewarm_evicted;
    return m;
}

/* ---------------- report ---------------- */
static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static const char* pol_name(int p) { return p == POL_LRU ? "lru" : "cost"; }

int main(void) {
    build_map();
    build_pool();
    order_pool();

    long long head_cost = 0, tail_cost = 0, head_len = 0, tail_len = 0;
    for (int i = 0; i < pool_n; i++) {
        if (i < HEAD_N) { head_cost += pool[i].cost; head_len += pool[i].len; }
        else { tail_cost += pool[i].cost; tail_len += pool[i].len; }
    }
    printf("pool=%d head=%d tail=%d avg_cost_head=%.0f avg_cost_tail=%.0f\n",
           pool_n, HEAD_N, pool_n - HEAD_N,
           (double)head_cost / HEAD_N, (double)tail_cost / (pool_n - HEAD_N));
    printf("entry_bytes: head=%.0f tail=%.0f  (cost policy keeps the fat head entries)\n",
           ENTRY_OVERHEAD + PATH_ELEM * (double)head_len / HEAD_N,
           ENTRY_OVERHEAD + PATH_ELEM * (double)tail_len / (pool_n - HEAD_N));

    /* ---- the three concurrency tiers at the current capacity ---- */
    const int tiers[TIERS] = {50, 100, 200};
    const int current_cap = 240;                 /* current cache capacity, entries */
    const int reqs_per_worker = 3000;
    int amax = HEAD_N + M_TAIL * tiers[TIERS - 1];
    int cap_required = (int)ceil(HIT_TARGET * amax);
    int gap = cap_required - current_cap;
    printf("active(k)=%d+%d*k  A_max=%d  required_cap=%d  current_cap=%d  gap=%d\n",
           HEAD_N, M_TAIL, amax, cap_required, current_cap, gap);
    printf("mem_limit=%dKiB  entry=%d+%d*len bytes\n",
           MEM_LIMIT_BYTES / 1024, ENTRY_OVERHEAD, PATH_ELEM);

    printf("\n-- current capacity=%d, three tiers --\n", current_cap);
    printf("%-5s %-4s %5s %6s %8s %8s %8s %6s\n",
           "tier", "pol", "A", "pre%", "over%", "cpu/req", "peakKiB", "evict");
    Metrics base_lru[TIERS], base_cost[TIERS];
    for (int t = 0; t < TIERS; t++) {
        for (int p = 0; p < 2; p++) {
            Metrics m = run(current_cap, p, tiers[t], reqs_per_worker, 0xabc);
            if (p == POL_LRU) base_lru[t] = m; else base_cost[t] = m;
            printf("%-5d %-4s %5d %6.1f %8.1f %8.1f %8lld %6d\n",
                   tiers[t], pol_name(p), m.active,
                   m.prewarm_hit * 100, m.overall_hit * 100, m.cpu_per_req,
                   m.peak_bytes / 1024, m.prewarm_evicted);
        }
    }

    /* ---- derivation, printed so the constants are auditable ---- */
    printf("\n-- derivation --\n");
    printf("h(C,k) = min(1, C / A(k)),  A(k) = HEAD + M*k\n");
    printf("reverse-solve from the reported incident (C=%d):\n", current_cap);
    printf("  h=%.0f%% at k=50  -> A=%d\n", 80.0, (int)(current_cap / 0.80));
    printf("  h=%.0f%% at k=200 -> A=%d\n", 40.0, (int)(current_cap / 0.40));
    printf("  slope=(A200-A50)/(200-50)=%.2f, intercept=%.0f\n",
           (current_cap / 0.40 - current_cap / 0.80) / 150.0,
           current_cap / 0.80 - 50.0 * (current_cap / 0.40 - current_cap / 0.80) / 150.0);
    printf("C_min(k) = %.2f * A(k)   (hit target %.0f%%)\n", HIT_TARGET, HIT_TARGET * 100);

    /* ---- per-tier gap at the current capacity ---- */
    printf("\n-- per-tier gap at hit>=%.0f%%, current cap=%d --\n", HIT_TARGET * 100, current_cap);
    printf("%-4s %5s %5s %6s %6s %9s %8s  %s\n",
           "k", "A(k)", "Cmin", "gap", "cap", "lru pre%", "lru over%", "verdict");
    int worst_need = 0;
    for (int t = 0; t < TIERS; t++) {
        int need = (int)ceil(HIT_TARGET * base_lru[t].active);
        int g = need > current_cap ? need - current_cap : 0;
        if (need > worst_need) worst_need = need;
        printf("%-4d %5d %5d %6d %6d %9.3f %8.3f  %s\n",
               tiers[t], base_lru[t].active, need, g, current_cap,
               base_lru[t].prewarm_hit * 100, base_lru[t].overall_hit * 100,
               (base_lru[t].overall_hit >= HIT_TARGET && base_lru[t].peak_bytes <= MEM_LIMIT_BYTES) ? "PASS" : "FAIL");
    }
    printf("C_req = max_k C_min(k) = %d entries,  current = %d,  gap = %d entries (%.0f%% short)\n",
           worst_need, current_cap, worst_need - current_cap,
           100.0 * (worst_need - current_cap) / current_cap);

    /* ---- capacity sweep at the hardest tier: hit curve and memory curve ---- */
    printf("\n-- capacity sweep, k=200 (cost policy) --\n");
    printf("%-5s %6s %8s %8s %8s %8s  %s\n", "cap", "pre%", "over%", "cpu/req", "peakKiB", "%mem", "note");
    const int sweep[] = {200, 240, 280, 320, 340, 360, 380, 400, 440, 480};
    int max_mem_cap = 0, min_hit_cap_cost = -1;
    for (int s = 0; s < (int)(sizeof(sweep) / sizeof(sweep[0])); s++) {
        Metrics m = run(sweep[s], POL_COST, tiers[TIERS - 1], reqs_per_worker, 0xabc);
        int over_mem = m.peak_bytes > MEM_LIMIT_BYTES;
        int over_hit = m.prewarm_hit < HIT_TARGET || m.overall_hit < HIT_TARGET;
        if (!over_mem && sweep[s] > max_mem_cap) max_mem_cap = sweep[s];
        if (!over_mem && !over_hit && min_hit_cap_cost < 0) min_hit_cap_cost = sweep[s];
        printf("%-5d %6.1f %8.3f %8.1f %8lld %8.1f  %s\n",
               sweep[s], m.prewarm_hit * 100, m.overall_hit * 100, m.cpu_per_req,
               m.peak_bytes / 1024, 100.0 * m.peak_bytes / MEM_LIMIT_BYTES,
               over_mem ? "MEM_OVER " : (over_hit ? "HIT_LOW " : "feasible "));
    }

    /* same sweep under LRU, to price the policy choice in entries and bytes */
    printf("\n-- capacity sweep, k=200 (lru policy) --\n");
    printf("%-5s %6s %8s %8s %8s %8s  %s\n", "cap", "pre%", "over%", "cpu/req", "peakKiB", "%mem", "note");
    int min_hit_cap_lru = -1, max_mem_cap_lru = 0;
    for (int s = 0; s < (int)(sizeof(sweep) / sizeof(sweep[0])); s++) {
        Metrics m = run(sweep[s], POL_LRU, tiers[TIERS - 1], reqs_per_worker, 0xabc);
        int over_mem = m.peak_bytes > MEM_LIMIT_BYTES;
        int over_hit = m.prewarm_hit < HIT_TARGET || m.overall_hit < HIT_TARGET;
        if (!over_mem && !over_hit && min_hit_cap_lru < 0) min_hit_cap_lru = sweep[s];
        if (!over_mem && sweep[s] > max_mem_cap_lru) max_mem_cap_lru = sweep[s];
        printf("%-5d %6.1f %8.3f %8.1f %8lld %8.1f  %s\n",
               sweep[s], m.prewarm_hit * 100, m.overall_hit * 100, m.cpu_per_req,
               m.peak_bytes / 1024, 100.0 * m.peak_bytes / MEM_LIMIT_BYTES,
               over_mem ? "MEM_OVER" : (over_hit ? "HIT_LOW" : "feasible"));
    }

    /* ---- equal-memory comparison at k=200 ----
     * Under uniform access both policies fill C slots, so total hit is C/A for
     * either one; what differs is which entries are resident, hence the
     * recompute cost per request and the memory each slot takes. */
    printf("\n-- equal-memory comparison, k=200 (memory is the binding constraint) --\n");
    printf("%-4s %5s %6s %8s %9s %8s\n", "pol", "cap", "pre%", "over%", "cpu/req", "peakKiB");
    Metrics mc = run(max_mem_cap, POL_COST, tiers[TIERS - 1], reqs_per_worker, 0xabc);
    Metrics ml = run(max_mem_cap_lru, POL_LRU, tiers[TIERS - 1], reqs_per_worker, 0xabc);
    printf("%-4s %5d %6.1f %8.3f %9.1f %8lld\n", "cost", max_mem_cap,
           mc.prewarm_hit * 100, mc.overall_hit * 100, mc.cpu_per_req, mc.peak_bytes / 1024);
    printf("%-4s %5d %6.1f %8.3f %9.1f %8lld\n", "lru", max_mem_cap_lru,
           ml.prewarm_hit * 100, ml.overall_hit * 100, ml.cpu_per_req, ml.peak_bytes / 1024);

    /* ---- decision ----
     * Two readings of "hit rate >= 60%":
     *   (a) prewarm-hit only: a policy that pins the prewarmed set reads 100%
     *       at the CURRENT capacity while every tail request still misses.
     *       Do not accept this; it satisfies the metric and fixes nothing.
     *   (b) total hit: under uniform access it is exactly min(1, C/A(k)) for
     *       ANY policy, so it pins down capacity and nothing else.
     * This harness enforces (b) as the SLO and reports (a) alongside. */
    int suggested_cap = worst_need;
    Metrics suggested = run(suggested_cap, POL_COST, tiers[TIERS - 1], reqs_per_worker, 0xabc);
    Metrics at_recommended = run(max_mem_cap, POL_COST, tiers[TIERS - 1], reqs_per_worker, 0xabc);
    printf("\n-- decision --\n");
    printf("1) capacity is the binding fix: h_total = min(1, C/A(k)) for every policy.\n");
    printf("   C_min(k) = 0.60*A(k); C_req = C_min(200) = %d.  current %d -> gap %d entries.\n",
           worst_need, current_cap, worst_need - current_cap);
    printf("2) policy does NOT change total hit (see equal-cap rows: both ~C/A).\n");
    printf("   cost-eviction raises PREWARM hit 40%%->%.0f%% and cuts cpu/req %.1f->%.1f,\n",
           base_cost[2].prewarm_hit * 100, base_lru[2].cpu_per_req, base_cost[2].cpu_per_req);
    printf("   but at k=200 with C=%d total hit stays %.1f%% - the cheap tail absorbs the evictions.\n",
           current_cap, base_cost[2].overall_hit * 100);
    printf("3) memory caps capacity too: cost policy keeps fatter head entries, so at equal\n");
    printf("   memory it fits fewer entries than lru (%.0f vs %.0f bytes/entry).\n",
           ENTRY_OVERHEAD + PATH_ELEM * (double)head_len / HEAD_N,
           ENTRY_OVERHEAD + PATH_ELEM * (double)tail_len / (pool_n - HEAD_N));
    printf("   feasible window under %dKiB: cost = [%d, %d], lru = [%d, %d] entries.\n",
           MEM_LIMIT_BYTES / 1024, min_hit_cap_cost, max_mem_cap, min_hit_cap_lru, max_mem_cap_lru);
    printf("4) recommend cap=%d, cost policy, numbers at k=200:\n", max_mem_cap);
    printf("   prewarm %.1f%%, total %.1f%% (target %.0f%%), cpu/req %.1f, peak %.1fKiB (%.1f%% of limit)\n",
           at_recommended.prewarm_hit * 100, at_recommended.overall_hit * 100, HIT_TARGET * 100,
           at_recommended.cpu_per_req, at_recommended.peak_bytes / 1024.0,
           100.0 * at_recommended.peak_bytes / MEM_LIMIT_BYTES);
    printf("   lower bound %d has zero hit margin (exactly %.1f%%); %d keeps %.1fpp.\n",
           suggested_cap, suggested.overall_hit * 100, max_mem_cap,
           (at_recommended.overall_hit - HIT_TARGET) * 100);
    printf("   alternative if only total-hit matters: lru at %d entries = %.1f%% total at the same\n",
           max_mem_cap_lru, ml.overall_hit * 100);
    printf("   memory, but %.1fx the recompute work (cpu/req %.1f vs %.1f).\n",
           ml.cpu_per_req / mc.cpu_per_req, ml.cpu_per_req, mc.cpu_per_req);

    /* ---- reproduces the incident and the fix ---- */
    check("lru_50_reproduces_80pct", base_lru[0].prewarm_hit >= 0.79 && base_lru[0].prewarm_hit <= 0.81);
    check("lru_200_reproduces_40pct", base_lru[2].prewarm_hit >= 0.39 && base_lru[2].prewarm_hit <= 0.41);
    check("formula_matches_lru_hit",
          fabs(base_lru[2].prewarm_hit - (double)current_cap / base_lru[2].active) < 0.01 &&
          fabs(base_lru[0].prewarm_hit - (double)current_cap / base_lru[0].active) < 0.01);
    check("lru_600_evicts_prewarm", base_lru[2].prewarm_evicted > 50000);
    check("cost_200_keeps_prewarm", base_cost[2].prewarm_hit >= 0.95);
    check("cost_200_few_prewarm_evictions", base_cost[2].prewarm_evicted < 100);
    check("cost_200_cheaper_cpu", base_cost[2].cpu_per_req < base_lru[2].cpu_per_req * 0.6);
    check("current_cap_fails_at_200", base_lru[2].overall_hit < HIT_TARGET);
    check("analytic_bound_is_360", worst_need == 360);
    check("gap_is_120", worst_need - current_cap == 120);
    check("suggested_hits_all_tiers", (run(suggested_cap, POL_COST, 50, reqs_per_worker, 0xabc).overall_hit >= HIT_TARGET) &&
                                       (run(suggested_cap, POL_COST, 100, reqs_per_worker, 0xabc).overall_hit >= HIT_TARGET) &&
                                       suggested.overall_hit >= HIT_TARGET);
    check("suggested_under_mem_all_tiers", suggested.peak_bytes <= MEM_LIMIT_BYTES);
    check("lru_needs_more_capacity", min_hit_cap_lru > min_hit_cap_cost);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
