
/* request: pathfinding request state machine with bounded chunked A*.
 *
 * Five states (queued, expanding, found, unreachable, canceled), one
 * explicit transition table instead of a bare enum, a totality/completeness
 * self-check, frame-level worst-response accounting and enter/exit hooks.
 *
 * Build: make test   (see README.md for the proof and the timing derivation).
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define W 16
#define H 12
#define N (W * H)                 /* 192: structural bound on node pops */

/* ---------------- states and triggers ---------------- */

enum {
    S_QUEUED = 0,
    S_EXPANDING = 1,
    S_FOUND = 2,
    S_UNREACHABLE = 3,
    S_CANCELED = 4,
    S_N = 5                       /* sentinel: not a real state */
};

enum {
    E_ADMIT = 0,                  /* scheduler grants the first frame      */
    E_BUDGET = 1,                 /* slice budget spent, open non-empty   */
    E_GOAL = 2,                   /* goal cell popped                     */
    E_EMPTY = 3,                  /* open set exhausted / bad endpoints  */
    E_CANCEL = 4,                 /* requester cancels                    */
    E_N = 5
};

enum {
    ED_NONE = -1,
    ED_ADMIT = 0,                 /* QUEUED     -> EXPANDING   */
    ED_BUDGET,                    /* EXPANDING  -> EXPANDING (self loop) */
    ED_GOAL,                      /* EXPANDING  -> FOUND       */
    ED_EMPTY,                     /* EXPANDING  -> UNREACHABLE */
    ED_CANCEL_Q,                  /* QUEUED     -> CANCELED    */
    ED_CANCEL_E,                  /* EXPANDING  -> CANCELED    */
    ED_N
};

/* ---------------- frame / work cost model ----------------
 *
 * Work unit: one hot-loop comparison counts 1; every popped node reserves
 * WORK_POP_FIXED units for its O(1) arithmetic/neighbour bookkeeping;
 * reconstruction reserves WORK_TAIL_FIXED.
 *
 * One pop scans the open list once to select the minimum (<= N iterations)
 * and performs at most four membership scans (<= 4N), so:
 *     work_per_pop <= 5N + WORK_POP_FIXED = 5*192 + 64 = 1024.
 * A B-pop slice therefore costs <= 1024B, and the terminal frame adds a
 * path reconstruction bounded by two N-length walks:
 *     H_wc(B) = B * 1024 + (2N + 64) = 1024B + 448.
 *
 * Every graph edge is serviced inside one scheduler frame, so its frame
 * budget is 1; the slice size is chosen so H_wc(B) <= FRAME_WORK_CAP.
 */
#define WORK_POP_FIXED   64
#define WORK_TAIL_FIXED  64
#define WORK_PER_POP_WC  (5 * N + WORK_POP_FIXED)        /* 1024 */
#define WORK_TAIL_WC     (2 * N + WORK_TAIL_FIXED)       /* 448  */
#define HITCH_WC(b)      ((uint64_t)(b) * WORK_PER_POP_WC + WORK_TAIL_WC)

#define FRAME_WORK_CAP   16384u   /* nav work units allowed in one frame */

/* longest Q -> terminal walk: admit + (ceil(N/B)-1) self loops + terminal */
#define SLICES_WC(b)     (((N) + (b) - 1) / (b))
#define WCRT_BOUND(b)    (1 + SLICES_WC(b))

static int choose_slice_budget(void) {
    int b = 1;
    while (b < N && HITCH_WC((uint64_t)b + 1) <= FRAME_WORK_CAP) b++;
    return b;                    /* evaluates to 15 with the constants above */
}

static const char *state_name(int s) {
    switch (s) {
        case S_QUEUED:     return "QUEUED";
        case S_EXPANDING:  return "EXPANDING";
        case S_FOUND:      return "FOUND";
        case S_UNREACHABLE:return "UNREACHABLE";
        case S_CANCELED:   return "CANCELED";
        default:           return "?";
    }
}

static const char *edge_name(int e) {
    switch (e) {
        case ED_ADMIT:    return "QUEUED->EXPANDING/ADMIT";
        case ED_BUDGET:   return "EXPANDING->EXPANDING/BUDGET";
        case ED_GOAL:     return "EXPANDING->FOUND/GOAL";
        case ED_EMPTY:    return "EXPANDING->UNREACHABLE/EMPTY";
        case ED_CANCEL_Q: return "QUEUED->CANCELED/CANCEL";
        case ED_CANCEL_E: return "EXPANDING->CANCELED/CANCEL";
        default:          return "NONE";
    }
}

/* ---------------- transition table delta: S x E -> S ---------------- */

typedef struct {
    unsigned char to;       /* destination state, or S_N for ignore */
    signed char   edge;     /* ED_* carried by a real edge, ED_NONE otherwise */
    unsigned char defined;  /* mechanical hole catcher for the self-check */
} Cell;

#define TR(to, edge) { (unsigned char)(to), (signed char)(edge), 1u }
#define IGNORE()     { (unsigned char)S_N,  (signed char)ED_NONE, 1u }

/* Every one of the 25 cells is explicit. Ignored triggers are totalized as
 * identity (state unchanged), so no input can leave the five-state set. */
static const Cell DELTA[S_N][E_N] = {
    [S_QUEUED] = {
        [E_ADMIT]  = TR(S_EXPANDING, ED_ADMIT),
        [E_BUDGET] = IGNORE(),
        [E_GOAL]   = IGNORE(),
        [E_EMPTY]  = IGNORE(),
        [E_CANCEL] = TR(S_CANCELED, ED_CANCEL_Q),
    },
    [S_EXPANDING] = {
        [E_ADMIT]  = IGNORE(),
        [E_BUDGET] = TR(S_EXPANDING, ED_BUDGET),
        [E_GOAL]   = TR(S_FOUND, ED_GOAL),
        [E_EMPTY]  = TR(S_UNREACHABLE, ED_EMPTY),
        [E_CANCEL] = TR(S_CANCELED, ED_CANCEL_E),
    },
    [S_FOUND] = {
        [E_ADMIT] = IGNORE(), [E_BUDGET] = IGNORE(), [E_GOAL] = IGNORE(),
        [E_EMPTY] = IGNORE(), [E_CANCEL] = IGNORE(),
    },
    [S_UNREACHABLE] = {
        [E_ADMIT] = IGNORE(), [E_BUDGET] = IGNORE(), [E_GOAL] = IGNORE(),
        [E_EMPTY] = IGNORE(), [E_CANCEL] = IGNORE(),
    },
    [S_CANCELED] = {
        [E_ADMIT] = IGNORE(), [E_BUDGET] = IGNORE(), [E_GOAL] = IGNORE(),
        [E_EMPTY] = IGNORE(), [E_CANCEL] = IGNORE(),
    },
};

/* Pure transition function. Out-of-range event ids are closed as identity. */
static int fsm_step(int state, int event, int *edge) {
    if (edge) *edge = ED_NONE;
    if (state < 0 || state >= S_N) return state;   /* caller invariant */
    if (event < 0 || event >= E_N) return state;
    const Cell *c = &DELTA[state][event];
    if (!c->defined || c->to == S_N) return state; /* explicit ignore */
    if (edge) *edge = c->edge;
    return c->to;
}

/* ---------------- grid map (same layout as grid/smooth/region) ---------------- */

typedef struct { unsigned char blocked[W * H]; } Map;
static int idx_xy(int x, int y) { return y * W + x; }
static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx_xy(x, y)];
}

/* ---------------- request record ---------------- */

typedef struct {
    Map map;
    int sx, sy, gx, gy, goal;
    int endpoints_ok;

    int g[N], f[N], parent[N];
    unsigned char closed[N];
    int open[N], nopen;
    int ready;

    int pops;                  /* total nodes popped so far */
    int slices;                /* expansion slices executed */
    int slice_pops;            /* pops in the current slice */
    int path[N], path_len;

    int state;
    int reaped;
    int pop_budget;            /* B */

    int enter_n[S_N];          /* entry-action counts */
    int leave_n[S_N];          /* exit-action counts  */
    int ignored;

    /* frame accounting */
    int frames;
    int edge_count[ED_N];
    int edge_frames[ED_N];     /* aggregate frames per edge kind */
    int edge_worst[ED_N];      /* worst single traversal */

    /* metered single-frame hitch */
    uint64_t work_cur;
    uint64_t hitch_worst;

    /* test hook: add stall frames to every traversal of one edge */
    int stall_edge;
    int stall_extra;

    uint64_t wall_ns;
} Req;

static uint64_t now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint64_t)ts.tv_sec * 1000000000ull + (uint64_t)ts.tv_nsec;
}

static void req_submit(Req *r, const Map *m, int sx, int sy, int gx, int gy,
                       int pop_budget) {
    memset(r, 0, sizeof(*r));
    r->map = *m;
    r->sx = sx; r->sy = sy; r->gx = gx; r->gy = gy;
    r->goal = (gx >= 0 && gy >= 0 && gx < W && gy < H) ? idx_xy(gx, gy) : -1;
    r->endpoints_ok = walkable(m, sx, sy) && walkable(m, gx, gy);
    r->pop_budget = pop_budget > 0 ? pop_budget : 1;
    r->stall_edge = ED_NONE;
    r->state = S_QUEUED;
    r->enter_n[S_QUEUED] = 1;  /* submit enters QUEUED once */
}

/* Apply one trigger: guarded transition, entry/exit actions, frame charge. */
static int req_apply(Req *r, int event) {
    int edge = ED_NONE;
    int next = fsm_step(r->state, event, &edge);
    if (edge == ED_NONE) { r->ignored++; return 0; }

    int cost = 1;              /* every edge has a 1-frame worst budget */
    if (r->stall_edge == edge) cost += r->stall_extra;

    r->edge_count[edge]++;
    r->edge_frames[edge] += cost;
    if (cost > r->edge_worst[edge]) r->edge_worst[edge] = cost;
    r->frames += cost;

    /* ED_BUDGET is an internal self-transition: entry/exit actions of
     * EXPANDING intentionally do not re-fire per slice. */
    if (edge != ED_BUDGET) r->leave_n[r->state]++;
    r->state = next;
    if (edge != ED_BUDGET) r->enter_n[next]++;
    return 1;
}

/* External entry point for arbitrary (even invalid) event sequences. */
static int req_dispatch(Req *r, int event) {
    if (r->reaped) return -1;
    if (event < 0 || event >= E_N) return -2;
    return req_apply(r, event);
}

/* Terminal records leave the five-state machine only by being reaped
 * (slot recycled); that is the one legal terminal exit action. */
static int req_reap(Req *r) {
    if (r->reaped) return 0;
    if (r->state != S_FOUND && r->state != S_UNREACHABLE &&
        r->state != S_CANCELED)
        return -1;
    r->leave_n[r->state]++;
    r->reaped = 1;
    return 1;
}

/* ---------------- chunked A* ---------------- */

static void admit_work(Req *r) {
    r->work_cur += 32;
    for (int i = 0; i < N; i++) {
        r->g[i] = 1 << 29;
        r->f[i] = 1 << 29;
        r->closed[i] = 0;
        r->parent[i] = -1;
        r->work_cur += 4;      /* four array initialisations */
    }
    r->nopen = 0;
    r->ready = 1;
    if (r->endpoints_ok) {
        int s = idx_xy(r->sx, r->sy);
        r->g[s] = 0;
        r->f[s] = abs(r->gx - r->sx) + abs(r->gy - r->sy);
        r->open[r->nopen++] = s;
    }
    /* bad/blocked endpoints leave open empty -> EMPTY on first slice */
}

static int reconstruct(Req *r, int cur) {
    int rev[N];
    int n = 0, c = cur;
    r->work_cur += WORK_TAIL_FIXED;
    while (c != -1 && n < N) { rev[n++] = c; c = r->parent[c]; r->work_cur++; }
    for (int i = 0; i < n; i++) {
        r->path[i] = rev[n - 1 - i];
        r->work_cur++;
    }
    r->path_len = n;
    return n;
}

/* Run at most pop_budget pops. Returns the trigger that ends the slice. */
static int run_slice(Req *r) {
    r->slices++;
    r->slice_pops = 0;

    while (r->slice_pops < r->pop_budget && r->nopen > 0) {
        int best = 0;
        for (int i = 1; i < r->nopen; i++) {
            r->work_cur++;     /* min-selection comparison */
            if (r->f[r->open[i]] < r->f[r->open[best]]) best = i;
        }
        int cur = r->open[best];
        r->open[best] = r->open[--r->nopen];

        r->pops++;
        r->slice_pops++;
        r->work_cur += WORK_POP_FIXED;

        if (r->endpoints_ok && cur == r->goal) {
            reconstruct(r, cur);
            return E_GOAL;
        }

        r->closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(&r->map, nx, ny)) continue;
            int ni = idx_xy(nx, ny);
            if (r->closed[ni]) continue;
            int ng = r->g[cur] + 1;
            if (ng < r->g[ni]) {
                r->g[ni] = ng;
                r->f[ni] = ng + abs(r->gx - nx) + abs(r->gy - ny);
                r->parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < r->nopen; i++) {
                    r->work_cur++; /* open membership comparison */
                    if (r->open[i] == ni) { found = 1; break; }
                }
                if (!found) r->open[r->nopen++] = ni;
            }
        }
    }

    if (r->nopen == 0) return E_EMPTY;
    return E_BUDGET;
}

/* One scheduler frame: at most one graph edge is taken. */
static void req_run_frame(Req *r) {
    if (r->reaped) return;
    r->work_cur = 0;

    if (r->state == S_QUEUED) {
        admit_work(r);
        req_apply(r, E_ADMIT);
    } else if (r->state == S_EXPANDING) {
        int trigger = run_slice(r);
        req_apply(r, trigger);
    } else {
        return;                /* terminal: no work, no edge */
    }

    if (r->work_cur > r->hitch_worst) r->hitch_worst = r->work_cur;
}

static void req_cancel(Req *r) {
    if (r->reaped) return;
    r->work_cur = 0;
    req_apply(r, E_CANCEL);
    if (r->work_cur > r->hitch_worst) r->hitch_worst = r->work_cur;
}

static void run_to_terminal(Req *r) {
    uint64_t t0 = now_ns();
    int guard = 0;
    while ((r->state == S_QUEUED || r->state == S_EXPANDING) &&
           guard++ < 100000)
        req_run_frame(r);
    r->wall_ns = now_ns() - t0;
}

/* ---------------- WCRT verdict and stuck-edge diagnosis ---------------- */

typedef struct {
    int contract_b;
    int bound_frames;
    int actual_frames;
    int violated;
    int stuck_edge;
    int edge_actual;
    int edge_allowance;
} WcrtReport;

/* Check against the published contract for slice size contract_b.
 * Aggregate per-edge allowances sum to the longest-path bound:
 *   ADMIT 1 + BUDGET (ceil(N/B)-1) + one terminal edge 1. */
static WcrtReport req_wcrt_check(const Req *r, int contract_b) {
    WcrtReport rep;
    memset(&rep, 0, sizeof(rep));
    int slices_wc = SLICES_WC(contract_b);
    int loop_allow = slices_wc - 1;
    rep.contract_b = contract_b;
    rep.bound_frames = 1 + slices_wc;
    rep.actual_frames = r->frames;
    rep.stuck_edge = ED_NONE;

    int allow[ED_N];
    for (int e = 0; e < ED_N; e++) allow[e] = 1;
    allow[ED_BUDGET] = loop_allow;

    int worst_excess = 0;
    for (int e = 0; e < ED_N; e++) {
        int excess = r->edge_frames[e] - allow[e];
        if (excess > worst_excess) {
            worst_excess = excess;
            rep.stuck_edge = e;
            rep.edge_actual = r->edge_frames[e];
            rep.edge_allowance = allow[e];
        }
    }

    if (r->frames > rep.bound_frames) {
        rep.violated = 1;
        if (rep.stuck_edge == ED_NONE) {
            /* fallback: busiest taken edge */
            int best = 0;
            for (int e = 1; e < ED_N; e++)
                if (r->edge_frames[e] > r->edge_frames[best]) best = e;
            rep.stuck_edge = best;
            rep.edge_actual = r->edge_frames[best];
            rep.edge_allowance = allow[best];
        }
    }
    return rep;
}

static void print_violation(const char *case_name, const WcrtReport *p,
                            const Req *r) {
    printf("WCRT_VIOLATION case=%s actual_frames=%d bound_frames=%d "
           "actual_wall_us=%.1f stuck_edge=%s edge_actual_frames=%d "
           "edge_allowance_frames=%d\n",
           case_name, p->actual_frames, p->bound_frames,
           r->wall_ns / 1000.0, edge_name(p->stuck_edge),
           p->edge_actual, p->edge_allowance);
}

/* ---------------- test helpers ---------------- */

static int failures = 0;
static void check(const char *name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static Map open_map(void) {
    Map m;
    memset(&m, 0, sizeof(m));
    return m;
}

/* Worst-case search map: goal (15,11) pocketed by two blocked corner
 * neighbours; A* pops all 189 cells of the start component before EMPTY. */
static Map pocket_map(void) {
    Map m = open_map();
    m.blocked[idx_xy(14, 11)] = 1;
    m.blocked[idx_xy(15, 10)] = 1;
    return m;
}

static int path_is_valid(const Req *r) {
    if (r->path_len <= 0) return 0;
    if (r->path[0] != idx_xy(r->sx, r->sy)) return 0;
    if (r->path[r->path_len - 1] != r->goal) return 0;
    for (int i = 0; i < r->path_len; i++) {
        int x = r->path[i] % W, y = r->path[i] / W;
        if (!walkable(&r->map, x, y)) return 0;
        if (i > 0) {
            int a = r->path[i - 1], b = r->path[i];
            int ax = a % W, ay = a / W, bx = b % W, by = b / W;
            if (abs(ax - bx) + abs(ay - by) != 1) return 0;
        }
    }
    return 1;
}

/* Run the pocket map at an arbitrary B and report measured curves. */
static void pocket_run(int b, int *frames, uint64_t *hitch, int *pops) {
    Req r;
    Map m = pocket_map();
    req_submit(&r, &m, 0, 0, W - 1, H - 1, b);
    run_to_terminal(&r);
    if (frames) *frames = r.frames;
    if (hitch) *hitch = r.hitch_worst;
    if (pops) *pops = r.pops;
}

/* Exhaustive sequence closure over the pure table, all starts, depth d. */
static int enumerate_closed(int state, int depth) {
    if (state < 0 || state >= S_N) return 0;
    if (depth == 0) return 1;
    for (int ev = 0; ev < E_N; ev++) {
        int edge;
        int ns = fsm_step(state, ev, &edge);
        if (ns < 0 || ns >= S_N) return 0;
        if (!enumerate_closed(ns, depth - 1)) return 0;
    }
    return 1;
}

static uint32_t rng = 0x9e3779b9u;
static uint32_t xr32(void) {
    rng ^= rng << 13; rng ^= rng >> 17; rng ^= rng << 5;
    return rng;
}

/* ---------------- CSV curve mode: B,U(B),H_wc(B),measured... ---------------- */

static int csv_main(void) {
    printf("B,bound_frames,wc_hitch_units,meas_frames,meas_hitch_units,"
           "meas_pops,chosen\n");
    int bstar = choose_slice_budget();
    for (int b = 1; b <= N; b++) {
        int fr, pops;
        uint64_t hitch;
        pocket_run(b, &fr, &hitch, &pops);
        printf("%d,%d,%llu,%d,%llu,%d,%d\n", b, WCRT_BOUND(b),
               (unsigned long long)HITCH_WC(b), fr,
               (unsigned long long)hitch, pops, b == bstar);
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc > 1 && strcmp(argv[1], "--csv") == 0) return csv_main();

    const int BSTAR = choose_slice_budget();
    const int USTAR = WCRT_BOUND(BSTAR);
    printf("# derived slice budget B*=%d frame_work_cap=%u "
           "wcrt_bound=%d frames hitch_wc(B*)=%llu units\n",
           BSTAR, FRAME_WORK_CAP, USTAR,
           (unsigned long long)HITCH_WC(BSTAR));

    /* ===== 1. transition table completeness ===== */
    int cells_defined = 1, real_edges = 0;
    for (int s = 0; s < S_N; s++)
        for (int e = 0; e < E_N; e++) {
            if (!DELTA[s][e].defined) cells_defined = 0;
            if (DELTA[s][e].edge != ED_NONE) real_edges++;
        }
    check("table_25_cells_defined", cells_defined);
    check("table_has_6_edges", real_edges == 6);

    int exact = 1;
    int expect[S_N][E_N] = {
        {ED_ADMIT,    ED_NONE, ED_NONE, ED_NONE, ED_CANCEL_Q},
        {ED_NONE,     ED_BUDGET, ED_GOAL, ED_EMPTY, ED_CANCEL_E},
        {ED_NONE, ED_NONE, ED_NONE, ED_NONE, ED_NONE},
        {ED_NONE, ED_NONE, ED_NONE, ED_NONE, ED_NONE},
        {ED_NONE, ED_NONE, ED_NONE, ED_NONE, ED_NONE},
    };
    for (int s = 0; s < S_N; s++)
        for (int e = 0; e < E_N; e++)
            if (DELTA[s][e].edge != expect[s][e]) exact = 0;
    check("delta_edges_exact", exact);

    int all_closed = 1;
    for (int s = 0; s < S_N; s++)
        if (!enumerate_closed(s, 6)) all_closed = 0;
    check("exhaustive_sequences_depth6_closed", all_closed);

    /* random input-sequence fuzz, including out-of-range event ids */
    int fuzz_ok = 1, sticky_ok = 1;
    for (int t = 0; t < 200000; t++) {
        Map m = open_map();
        Req r;
        req_submit(&r, &m, 0, 0, 5, 5, BSTAR);
        int len = xr32() % 40;
        for (int i = 0; i < len; i++) {
            int ev = (int)(xr32() % (E_N + 4)) - 2;  /* includes invalid ids */
            int before = r.state;
            int terminal = (before == S_FOUND || before == S_UNREACHABLE ||
                            before == S_CANCELED);
            req_dispatch(&r, ev);
            if (r.state < 0 || r.state >= S_N) fuzz_ok = 0;
            if (terminal && r.state != before) sticky_ok = 0;
        }
    }
    check("fuzz_200k_sequences_no_sixth_state", fuzz_ok);
    check("fuzz_terminal_states_absorbing", sticky_ok);

    /* ===== 2. one enter and one leave case per state ===== */

    /* QUEUED: entered at submit, left through ADMIT; stray GOAL is ignored. */
    {
        Req r; Map m = open_map();
        req_submit(&r, &m, 0, 0, 3, 0, BSTAR);
        check("enter_queued", r.state == S_QUEUED && r.enter_n[S_QUEUED] == 1);
        req_dispatch(&r, E_GOAL);
        int ignored_while_queued = (r.state == S_QUEUED && r.ignored == 1);
        req_run_frame(&r);       /* ADMIT */
        check("queued_stray_goal_ignored", ignored_while_queued);
        check("leave_queued", r.state == S_EXPANDING &&
                              r.leave_n[S_QUEUED] == 1 &&
                              r.enter_n[S_EXPANDING] == 1);
        run_to_terminal(&r);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_queued_lifecycle", !rep.violated);
        if (rep.violated) print_violation("queued", &rep, &r);
    }

    /* EXPANDING: observe the BUDGET self-loop, then leave via EMPTY. */
    {
        Req r; Map m = pocket_map();
        req_submit(&r, &m, 0, 0, W - 1, H - 1, BSTAR);
        req_run_frame(&r);       /* ADMIT */
        req_run_frame(&r);       /* first slice: B pops, still open */
        int self_looped = (r.state == S_EXPANDING &&
                           r.edge_count[ED_BUDGET] == 1);
        run_to_terminal(&r);
        check("expanding_budget_self_loop", self_looped);
        check("enter_expanding", r.enter_n[S_EXPANDING] == 1);
        check("leave_expanding", r.state == S_UNREACHABLE &&
                                 r.leave_n[S_EXPANDING] == 1);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_expanding_pocket", !rep.violated);
        if (rep.violated) print_violation("expanding", &rep, &r);
    }

    /* FOUND: enter via GOAL, refuse every later trigger, leave via reap. */
    {
        Req r; Map m = open_map();
        req_submit(&r, &m, 0, 0, 3, 0, BSTAR);
        run_to_terminal(&r);
        check("enter_found", r.state == S_FOUND &&
                             r.enter_n[S_FOUND] == 1);
        int sticky = 1;
        for (int ev = 0; ev < E_N; ev++) req_dispatch(&r, ev);
        if (r.state != S_FOUND) sticky = 0;
        check("found_refuses_all_triggers", sticky);
        check("found_path_correct", path_is_valid(&r) && r.path_len == 4);
        int reaped1 = req_reap(&r);
        check("leave_found", reaped1 == 1 && r.leave_n[S_FOUND] == 1);
        check("reap_fires_once", req_reap(&r) == 0 &&
                                 r.leave_n[S_FOUND] == 1);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_found", !rep.violated);
        if (rep.violated) print_violation("found", &rep, &r);
    }

    /* UNREACHABLE: pocketed goal, enter via EMPTY, leave via reap. */
    {
        Req r; Map m = pocket_map();
        req_submit(&r, &m, 0, 0, W - 1, H - 1, BSTAR);
        run_to_terminal(&r);
        check("enter_unreachable", r.state == S_UNREACHABLE &&
                                   r.enter_n[S_UNREACHABLE] == 1);
        int sticky = 1;
        for (int ev = 0; ev < E_N; ev++) req_dispatch(&r, ev);
        if (r.state != S_UNREACHABLE) sticky = 0;
        check("unreachable_refuses_all_triggers", sticky);
        int reaped1 = req_reap(&r);
        check("leave_unreachable", reaped1 == 1 &&
                                  r.leave_n[S_UNREACHABLE] == 1);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_unreachable", !rep.violated);
        if (rep.violated) print_violation("unreachable", &rep, &r);
        printf("timing case=unreachable state=%s actual_frames=%d bound_frames=%d "
               "actual_wall_us=%.1f hitch_max=%llu cap=%u pops=%d\n",
               state_name(r.state), r.frames, USTAR, r.wall_ns / 1000.0,
               (unsigned long long)r.hitch_worst, FRAME_WORK_CAP, r.pops);
    }

    /* CANCELED: canonical enter/leave from QUEUED, plus cancel in E. */
    {
        Req r; Map m = open_map();
        req_submit(&r, &m, 0, 0, 9, 9, BSTAR);
        req_cancel(&r);
        check("enter_canceled", r.state == S_CANCELED &&
                                r.enter_n[S_CANCELED] == 1 &&
                                r.edge_count[ED_CANCEL_Q] == 1);
        int sticky = 1;
        for (int ev = 0; ev < E_N; ev++) req_dispatch(&r, ev);
        if (r.state != S_CANCELED) sticky = 0;
        check("canceled_refuses_all_triggers", sticky);
        check("leave_canceled", req_reap(&r) == 1 &&
                                r.leave_n[S_CANCELED] == 1);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_cancel_queued", !rep.violated);
        if (rep.violated) print_violation("cancel_queued", &rep, &r);
    }
    {
        Req r; Map m = pocket_map();
        req_submit(&r, &m, 0, 0, W - 1, H - 1, BSTAR);
        req_run_frame(&r);       /* ADMIT */
        req_cancel(&r);          /* EXPANDING -> CANCELED */
        check("cancel_from_expanding_edge",
              r.state == S_CANCELED && r.edge_count[ED_CANCEL_E] == 1 &&
              r.leave_n[S_EXPANDING] == 1);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        check("wcrt_cancel_expanding", !rep.violated);
        if (rep.violated) print_violation("cancel_expanding", &rep, &r);
    }

    /* every graph edge has been traversed at least once */
    {
        int covered = 1;
        for (int e = 0; e < ED_N; e++) {
            /* edge coverage is accumulated in separate records, so rebuild
             * a lightweight coverage run here. */
            Req r;
            Map m;
            switch (e) {
                case ED_ADMIT: case ED_GOAL:
                    m = open_map();
                    req_submit(&r, &m, 0, 0, 2, 0, N);
                    run_to_terminal(&r);
                    break;
                case ED_BUDGET: case ED_EMPTY:
                    m = pocket_map();
                    req_submit(&r, &m, 0, 0, W - 1, H - 1, 1);
                    run_to_terminal(&r);
                    break;
                case ED_CANCEL_Q:
                    m = open_map();
                    req_submit(&r, &m, 0, 0, 2, 2, N);
                    req_cancel(&r);
                    break;
                default: /* ED_CANCEL_E */
                    m = pocket_map();
                    req_submit(&r, &m, 0, 0, W - 1, H - 1, N);
                    req_run_frame(&r);
                    req_cancel(&r);
                    break;
            }
            if (r.edge_count[e] != 1 &&
                !(e == ED_BUDGET && r.edge_count[e] >= 1))
                covered = 0;
        }
        check("all_six_edges_exercised", covered);
    }

    /* ===== 3. slice sweep: measured curves never exceed derived bounds ===== */
    int sweep_ok = 1, worst_pops = 0;
    for (int b = 1; b <= N; b++) {
        int fr, pops;
        uint64_t hitch;
        pocket_run(b, &fr, &hitch, &pops);
        if (pops > N) sweep_ok = 0;
        if (fr > WCRT_BOUND(b)) sweep_ok = 0;
        if (hitch > HITCH_WC(b)) sweep_ok = 0;
        if (pops > worst_pops) worst_pops = pops;
    }
    check("slice_sweep_192_budgets_within_bounds", sweep_ok);
    check("pocket_pops_match_structural_bound", worst_pops == N - 3); /*189*/

    /* B* is the largest B whose analytic hitch stays inside the cap */
    int decision_ok = (HITCH_WC(BSTAR) <= FRAME_WORK_CAP &&
                       (BSTAR == N || HITCH_WC(BSTAR + 1) > FRAME_WORK_CAP));
    check("slice_decision_is_max_b_under_cap", decision_ok);
    check("derived_bstar_15_bound_14", BSTAR == 15 && USTAR == 14);
    check("bstar_hitch_under_cap", HITCH_WC(BSTAR) <= FRAME_WORK_CAP);

    printf("curve sample (B: bound_frames / hitch_wc / measured_hitch):\n");
    const int show[] = {1, 2, 4, 8, 15, 16, 32, 64, 128, 192};
    for (unsigned i = 0; i < sizeof(show) / sizeof(show[0]); i++) {
        int b = show[i], fr;
        uint64_t hitch, mh;
        pocket_run(b, &fr, &mh, NULL);
        hitch = HITCH_WC(b);
        printf("  B=%3d wcrt=%3d frames hitch_wc=%6llu measured_hitch=%6llu%s\n",
               b, WCRT_BOUND(b), (unsigned long long)hitch,
               (unsigned long long)mh, b == BSTAR ? "  <- B*" : "");
    }

    /* ===== 4. random-map corpus validates the structural bound ===== */
    int corpus_ok = 1, corpus_n = 4000;
    for (int t = 0; t < corpus_n; t++) {
        Map m = open_map();
        int density = (int)(xr32() % 4);             /* 0/8/16/24 percent */
        for (int i = 0; i < N; i++)
            if ((int)(xr32() % 100) < density * 8) m.blocked[i] = 1;
        int sx = xr32() % W, sy = xr32() % H;
        int gx = xr32() % W, gy = xr32() % H;

        Req r;
        req_submit(&r, &m, sx, sy, gx, gy, BSTAR);
        run_to_terminal(&r);

        if (r.state != S_FOUND && r.state != S_UNREACHABLE) corpus_ok = 0;
        if (r.pops > N || r.frames > USTAR) corpus_ok = 0;
        if (r.hitch_worst > FRAME_WORK_CAP) corpus_ok = 0;
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        if (rep.violated) {
            corpus_ok = 0;
            print_violation("random_corpus", &rep, &r);
        }
        if (r.state == S_FOUND && !path_is_valid(&r)) corpus_ok = 0;
    }
    check("random_corpus_4000_within_wcrt_and_cap", corpus_ok);

    /* ===== 5. violation reporting: actual time + stuck edge ===== */

    /* 5a. one terminal edge is artificially stalled past its frame budget. */
    {
        Req r; Map m = open_map();
        req_submit(&r, &m, 0, 0, 9, 9, BSTAR);
        r.stall_edge = ED_GOAL;
        r.stall_extra = 40;
        run_to_terminal(&r);
        WcrtReport rep = req_wcrt_check(&r, BSTAR);
        int detects = rep.violated && rep.stuck_edge == ED_GOAL;
        check("overrun_stall_reports_stuck_goal_edge", detects);
        if (detects) print_violation("injected_goal_stall", &rep, &r);
        else if (rep.violated) print_violation("injected_goal_stall", &rep, &r);
    }

    /* 5b. B=1 deployed against the B*=15 contract: too many self loops. */
    {
        Req r; Map m = pocket_map();
        req_submit(&r, &m, 0, 0, W - 1, H - 1, 1);
        run_to_terminal(&r);
        WcrtReport bad = req_wcrt_check(&r, BSTAR);
        int detects = bad.violated && bad.stuck_edge == ED_BUDGET;
        check("overrun_tiny_b_reports_stuck_budget_loop", detects);
        if (detects) print_violation("b1_against_b15_contract", &bad, &r);

        /* against its own derived bound U(1)=193 it must still pass */
        WcrtReport own = req_wcrt_check(&r, 1);
        check("tiny_b_passes_its_own_bound", !own.violated);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
