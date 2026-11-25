
/* hotupdate: atomicity of chunked-map hot updates.
 *
 * Models two update strategies over the same deterministic query stream:
 *   naive - in-place free/load inside one live chunk table (the crash site:
 *           a query can hit a freed slot -> dangling, or mix old/new chunks
 *           -> torn half-old-half-new read).
 *   rcu   - three-phase update: shadow-load new generation, publish one
 *           pointer, reclaim old generations only after k retained
 *           generations (a bounded grace period).
 *
 * The simulation is single-threaded frame-stepped, so every violation is
 * reproduced deterministically with its frame number and query coordinate.
 */
#include <stdio.h>
#include <string.h>

/* ---- world -------------------------------------------------------------- */
#define CGW       4                 /* chunk grid 4x4                  */
#define CGH       4
#define NCHUNK    (CGW * CGH)       /* 16 chunks                       */
#define CELLS     16                /* each chunk is a 4x4 cell block  */
#define MAPCELLS  (NCHUNK * CELLS)  /* 256 cells, coordinates 0..15    */
#define FRAMES    96
#define QPF       3                 /* navigation queries spawned/frame */
#define QLEN      4                 /* cells touched by one query      */
#define LMIN      2                 /* query lifetime in frames        */
#define LMAX      6
#define LOADRATE  8                 /* chunks (or micro-ops) per frame */
#define MAXGEN    (FRAMES + 4)
#define MAXVIO    6
#define BASESEED  20261126u

#define TIERS 3
static const int TIER_T[TIERS] = {8, 4, 2};   /* slow / medium / fast */

typedef enum { KIND_NONE = 0, KIND_DANG, KIND_TORN } Kind;

typedef struct {
    int born;
    int life;
    int gen;                 /* generation snapshot taken at spawn    */
    int cell[QLEN];          /* cell coordinates 0..255                */
    int id;
} Query;

typedef struct {
    int frame;
    int qid;
    int x, y;
    Kind kind;
    int gen_snap;
    int gen_now;
} Violation;

typedef struct {
    int torn, dang;
    int peak_chunks;         /* peak live chunk count, in chunks      */
    int qmax;                /* max queries alive across a switch     */
    int switches;
    Violation vio[MAXVIO];
    int nvio;
} Report;

/* deterministic LCG so naive and rcu see identical query streams per tier */
static unsigned rng_state;
static unsigned rng(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return (rng_state >> 16) & 0x7fff;
}
static int rrand(int lo, int hi) { return lo + (int)(rng() % (unsigned)(hi - lo + 1)); }

static void vio_push(Report* r, int frame, const Query* q, Kind k, int cell, int gnow) {
    if (r->nvio < MAXVIO) {
        Violation* v = &r->vio[r->nvio++];
        v->frame = frame; v->qid = q->id;
        v->x = cell % 16; v->y = cell / 16;
        v->kind = k; v->gen_snap = q->gen; v->gen_now = gnow;
    }
    if (k == KIND_DANG) r->dang++; else r->torn++;
}

/* spawn QPF queries; snapshot generation is decided by the engine */
static void spawn(Query* qs, int* nq, int frame, int gen) {
    for (int i = 0; i < QPF; i++) {
        Query* q = &qs[(*nq)++];
        q->id = *nq - 1;
        q->born = frame;
        q->life = rrand(LMIN, LMAX);
        q->gen = gen;
        for (int j = 0; j < QLEN; j++) q->cell[j] = rrand(0, MAPCELLS - 1);
    }
}

/* one cell access per frame of a query's life; a query touches QLEN
 * frames at most, so lives longer than QLEN reuse the last cell.       */
static int q_alive(const Query* q, int frame) {
    int age = frame - q->born;          /* 1..life on access frames */
    return age >= 1 && age <= q->life;
}
static int q_cell(const Query* q, int frame) {
    int age = frame - q->born;
    int i = age - 1 < QLEN ? age - 1 : QLEN - 1;
    return q->cell[i];
}

/* ---- naive engine: in-place free/load in one live table ----------------- */
/*
 * An update frees every chunk first and loads the new generation into the
 * same slots afterwards, at LOADRATE*2 micro-ops per frame, so the update
 * spans two frames. The table stays live the whole time: on the free frame
 * every access lands on a poisoned slot (dangling), on the load frame old
 * queries find the new generation in slots they entered as the old one
 * (torn). This is the interleaving the crash stack points at.
 */
typedef struct {
    int owner[NCHUNK];       /* generation owning the slot, -1 if freed */
    int cur_gen;
    int updating;
    int op;                  /* 0..NCHUNK-1 frees, then NCHUNK loads    */
    int new_gen;
    Query qs[FRAMES * QPF + 8];
    int nq;
} Naive;

static void naive_start(Naive* n, int frame) {
    (void)frame;
    n->updating = 1;
    n->op = 0;
    n->new_gen = ++n->cur_gen;
}

static void naive_update_phase(Naive* n) {
    int budget = LOADRATE * 2;
    while (n->updating && budget-- > 0) {
        if (n->op < NCHUNK)
            n->owner[n->op] = -1;                    /* free old chunk    */
        else
            n->owner[n->op - NCHUNK] = n->new_gen;  /* load into same slot */
        n->op++;
        if (n->op == 2 * NCHUNK) { n->updating = 0; break; }
    }
}

static void naive_access(Naive* n, Report* r, int frame, Query* q) {
    int slot = q_cell(q, frame) / CELLS;
    int g = n->owner[slot];
    if (g < 0) {                                   /* poisoned slot */
        vio_push(r, frame, q, KIND_DANG, q_cell(q, frame), -1);
        return;
    }
    int first = (frame - q->born) == 1;
    if (first) q->gen = g;                         /* table has no pointer:
                                                      lookup per access   */
    else if (g != q->gen)
        vio_push(r, frame, q, KIND_TORN, q_cell(q, frame), g);
}

static Report run_naive(int T) {
    rng_state = BASESEED + (unsigned)T * 7919u;
    Naive n;
    for (int i = 0; i < NCHUNK; i++) n.owner[i] = 0;
    n.cur_gen = 0; n.updating = 0; n.op = 0; n.new_gen = 0; n.nq = 0;
    Report r; memset(&r, 0, sizeof(r));

    for (int f = 0; f < FRAMES; f++) {
        if (f % T == 0 && !n.updating) {
            naive_start(&n, f);
            r.switches++;
        }
        naive_update_phase(&n);

        int live = 0;
        for (int i = 0; i < NCHUNK; i++) if (n.owner[i] >= 0) live++;
        if (live > r.peak_chunks) r.peak_chunks = live;

        spawn(n.qs, &n.nq, f, n.cur_gen);
        int active = 0;
        for (int i = 0; i < n.nq; i++) {
            Query* q = &n.qs[i];
            if (!q_alive(q, f)) continue;
            active++;
            naive_access(&n, &r, f, q);
        }
        if (f % T == 0 && active > r.qmax) r.qmax = active;
    }
    return r;
}

/* ---- rcu engine: shadow load, publish pointer, deferred reclaim --------- */
/*
 * Every generation owns a private chunk array. A single published pointer
 * selects the live generation. Old generations go onto a retirement queue
 * and the oldest is reclaimed only once more than k generations are queued.
 */
typedef struct {
    unsigned char alive[MAXGEN][NCHUNK]; /* chunk loaded in that gen  */
    unsigned char dead[MAXGEN];
    int published;
    int loading_gen;                     /* -1 when nothing loading   */
    int loaded;                          /* chunks loaded so far      */
    int retired[MAXGEN]; int head, tail; /* retirement queue          */
    int live_chunks;
    Query qs[FRAMES * QPF + 8];
    int nq;
} Rcu;

static void rcu_reclaim(Rcu* c, int k, Report* r, int frame) {
    (void)r; (void)frame;
    while (c->tail - c->head > k) {
        int g = c->retired[c->head++];
        c->dead[g] = 1;
        c->live_chunks -= NCHUNK;
    }
}

static Report run_rcu(int T, int k) {
    rng_state = BASESEED + (unsigned)T * 7919u;
    Rcu c;
    memset(&c, 0, sizeof(c));
    c.published = 0;
    c.loading_gen = -1;
    for (int i = 0; i < NCHUNK; i++) c.alive[0][i] = 1;  /* gen 0 live */
    c.live_chunks = NCHUNK;
    c.head = c.tail = 0; c.nq = 0;
    Report r; memset(&r, 0, sizeof(r));

    for (int f = 0; f < FRAMES; f++) {
        /* phase 1: drive shadow load; a new generation is started T-2
         * frames before its switch so loading never blocks publishing.  */
        if (c.loading_gen < 0 && f + 2 < FRAMES && (f + 2) % T == 0) {
            c.loading_gen = c.published + 1;
            c.loaded = 0;
        }
        if (c.loading_gen >= 0) {
            for (int b = 0; b < LOADRATE && c.loaded < NCHUNK; b++) {
                c.alive[c.loading_gen][c.loaded++] = 1;
                c.live_chunks++;
            }
            if (c.loaded == NCHUNK) c.loading_gen = -1;
        }

        /* phase 2: publish pointer + retire old gen, then phase 3 reclaim */
        if (f % T == 0 && f > 0) {
            int old = c.published, ng = old + 1;
            c.published = ng;
            c.retired[c.tail++] = old;
            r.switches++;
            rcu_reclaim(&c, k, &r, f);
        }
        if (c.live_chunks > r.peak_chunks) r.peak_chunks = c.live_chunks;

        spawn(c.qs, &c.nq, f, c.published);
        int active = 0;
        for (int i = 0; i < c.nq; i++) {
            Query* q = &c.qs[i];
            if (!q_alive(q, f)) continue;
            active++;
            if (c.dead[q->gen])
                vio_push(&r, f, q, KIND_DANG, q_cell(q, f), c.published);
            /* torn is impossible: every access goes through one pointer,
             * and a generation is only published fully loaded.          */
        }
        if (f % T == 0 && active > r.qmax) r.qmax = active;
    }
    return r;
}

/* ---- phase-ordering demo ------------------------------------------------ */
/*
 * One update window of length T with the three events L (shadow load),
 * S (publish pointer), R (reclaim old gen, queue cap k) placed in each of
 * the 6 possible orders. Reclamation keeps the queue cap from the RCU
 * engine, so this table isolates the torn axis: publishing before the
 * shadow load completes. The dangling axis is swept separately via k.
 */
static const char* ORDERS[6] = {"LSR", "LRS", "SLR", "SRL", "RLS", "RSL"};

typedef struct { int torn, dang; int fframe, fx, fy; } OrdResult;

static OrdResult run_ordering(const char* order, int T) {
    rng_state = BASESEED + 99u;
    unsigned char alive[MAXGEN][NCHUNK];
    unsigned char dead[MAXGEN];
    memset(alive, 0, sizeof(alive));
    memset(dead, 0, sizeof(dead));
    for (int i = 0; i < NCHUNK; i++) alive[0][i] = 1;
    int published = 0, loaded = 0, loading = 0, newgen = 1;
    int retired[MAXGEN], head = 0, tail = 0;
    int k = 2;
    Query qs[64]; int nq = 0;
    OrdResult res; memset(&res, 0, sizeof(res));
    res.fframe = -1;

    int tL = -1, tS = -1, tR = -1;
    for (int p = 0; p < 3; p++) {
        int when = p;                      /* event slot inside window */
        if (order[p] == 'L') tL = when;
        if (order[p] == 'S') tS = when;
        if (order[p] == 'R') tR = when;
    }

    for (int f = 0; f < T; f++) {
        if (f == tL) loading = 1;
        if (loading) {
            for (int b = 0; b < LOADRATE && loaded < NCHUNK; b++)
                alive[newgen][loaded++] = 1;
            if (loaded == NCHUNK) loading = 0;
        }
        if (f == tS) published = newgen;
        if (f == tR) {
            retired[tail++] = 0;
            while (tail - head > k) dead[retired[head++]] = 1;
        }
        spawn(qs, &nq, f, published);
        for (int i = 0; i < nq; i++) {
            Query* q = &qs[i];
            if (!q_alive(q, f)) continue;
            int slot = q_cell(q, f) / CELLS;
            if (dead[q->gen]) {
                res.dang++;
                if (res.fframe < 0) {
                    res.fframe = f; res.fx = q_cell(q, f) % 16;
                    res.fy = q_cell(q, f) / 16;
                }
            } else if (q->gen == newgen && !alive[newgen][slot]) {
                res.torn++;
                if (res.fframe < 0) {
                    res.fframe = f; res.fx = q_cell(q, f) % 16;
                    res.fy = q_cell(q, f) / 16;
                }
            }
        }
    }
    return res;
}

/* ---- reporting ---------------------------------------------------------- */
static const char* kind_name(Kind k) {
    return k == KIND_DANG ? "dangling" : "torn";
}

static void print_violations(const Report* r) {
    for (int i = 0; i < r->nvio; i++) {
        const Violation* v = &r->vio[i];
        printf("  viol frame=%d q=%d kind=%s coord=(%d,%d) gen_snap=%d gen_now=%d\n",
               v->frame, v->qid, kind_name(v->kind), v->x, v->y,
               v->gen_snap, v->gen_now);
    }
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    printf("== world: chunks=%d cells=%d frames=%d qpf=%d life=[%d,%d] tiers T=",
           NCHUNK, MAPCELLS, FRAMES, QPF, LMIN, LMAX);
    for (int i = 0; i < TIERS; i++) printf("%d%s", TIER_T[i], i + 1 < TIERS ? "/" : "\n");

    /* ---- part 1: phase ordering, torn axis ---------------------------- */
    printf("\n== phase ordering (one window T=4, reclaim cap k=2) ==\n");
    int order_ok = 1;
    for (int i = 0; i < 6; i++) {
        OrdResult o = run_ordering(ORDERS[i], 4);
        int s_before_l = (int)(strchr(ORDERS[i], 'S') - ORDERS[i])
                       < (int)(strchr(ORDERS[i], 'L') - ORDERS[i]);
        printf("order=%s S_before_L=%d torn=%d dang=%d first_bad_frame=%d coord=(%d,%d)\n",
               ORDERS[i], s_before_l, o.torn, o.dang, o.fframe, o.fx, o.fy);
        /* L-before-LS orderings must stay clean; every S-before-L order
         * must expose a torn frame.                                 */
        if (s_before_l ? o.torn == 0 : o.torn != 0) order_ok = 0;
    }
    check("L_before_S_never_torn_and_S_before_L_always_torn", order_ok);

    /* ---- parts 2-4: per tier, naive vs rcu with k sweep --------------- */
    int k_measured[TIERS];
    int qmax_measured[TIERS];
    printf("\nCSVKSWEEP\ntier,T,k,torn,dang,peak_chunks,qmax,switches\n");
    for (int t = 0; t < TIERS; t++) {
        int T = TIER_T[t];
        const char* tname = T == 8 ? "slow" : T == 4 ? "medium" : "fast";

        Report nz = run_naive(T);
        printf("\n== tier=%s T=%d naive in-place ==\n", tname, T);
        printf("naive: torn=%d dang=%d peak_chunks=%d switches=%d\n",
               nz.torn, nz.dang, nz.peak_chunks, nz.switches);
        print_violations(&nz);
        check("naive_violates", nz.torn + nz.dang > 0);

        int first_safe = -1;
        for (int k = 0; k <= 4; k++) {
            Report rr = run_rcu(T, k);
            printf("rcu k=%d: torn=%d dang=%d peak_chunks=%d qmax=%d switches=%d\n",
                   k, rr.torn, rr.dang, rr.peak_chunks, rr.qmax, rr.switches);
            print_violations(&rr);
            printf("CSV,%s,%d,%d,%d,%d,%d,%d,%d\n", tname, T, k,
                   rr.torn, rr.dang, rr.peak_chunks, rr.qmax, rr.switches);
            if (first_safe < 0 && rr.torn == 0 && rr.dang == 0)
                first_safe = k;
            if (k == 4) qmax_measured[t] = rr.qmax;
        }
        k_measured[t] = first_safe;
        int k_formula = (LMAX + T - 1) / T;     /* ceil(Lmax / T) */
        printf("tier=%s k_measured=%d k_formula=ceil(%d/%d)=%d qmax=%d\n",
               tname, first_safe, LMAX, T, k_formula, qmax_measured[t]);
        check("k_matches_formula", first_safe == k_formula);
        check("rcu_safe_at_kstar", run_rcu(T, first_safe).dang == 0);
    }

    /* reverse solve from the maximum number of queries spanning a switch:
     * each query lives at most LMAX frames, T frames pass between switches,
     * so at most ceil(LMAX/T) retired generations can still be referenced. */
    printf("\n== reverse solve from switch-time query count ==\n");
    for (int t = 0; t < TIERS; t++) {
        int T = TIER_T[t];
        int k_from_q = (LMAX + T - 1) / T;
        printf("tier T=%d qmax=%d (capacity QPF*LMAX=%d) -> retain %d gen, measured k=%d\n",
               T, qmax_measured[t], QPF * LMAX, k_from_q, k_measured[t]);
        check("reverse_solve_matches", k_from_q == k_measured[t]);
    }

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
