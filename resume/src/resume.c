/* resume: interruptible A* — checkpoint state, exact resume, granularity sweep.
 *
 * Checkpoint holds everything an exact resume needs:
 *   g[], parent[], closed[], open[] (order preserved), nopen, expansions, work.
 * f[] is NOT saved: f = g + h is deterministic, recomputed on load.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define W 64
#define H 48
#define N (W*H)
#define INF (1<<29)

typedef struct { unsigned char blocked[N]; } Map;

static int idx(int x, int y) { return y * W + x; }
static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

static int GX, GY;
static int heur(int i) { return abs(GX - i % W) + abs(GY - i / W); }

/* live search state; work = deterministic cost units (open-scan steps + relaxations) */
typedef struct {
    int g[N], f[N], parent[N];
    unsigned char closed[N];
    int open[N];
    int nopen;
    int expansions;
    long work;
} State;

/* checkpoint: resume-sufficient subset of State (f dropped, recomputed on load) */
typedef struct {
    int g[N], parent[N];
    unsigned char closed[N];
    int open[N];
    int nopen;
    int expansions;
    long work;
} Ckpt;

static void state_init(State* s, int sx, int sy) {
    for (int i = 0; i < N; i++) { s->g[i] = INF; s->f[i] = INF; s->closed[i] = 0; s->parent[i] = -1; }
    s->nopen = 0; s->expansions = 0; s->work = 0;
    int st = idx(sx, sy);
    s->g[st] = 0; s->f[st] = heur(st);
    s->open[s->nopen++] = st;
}

/* returns words written */
static long ckpt_save(Ckpt* c, const State* s) {
    memcpy(c->g, s->g, sizeof c->g);
    memcpy(c->parent, s->parent, sizeof c->parent);
    memcpy(c->closed, s->closed, sizeof c->closed);
    memcpy(c->open, s->open, sizeof(int) * s->nopen);
    c->nopen = s->nopen;
    c->expansions = s->expansions;
    c->work = s->work;
    return 3L*N + s->nopen + 3;
}

/* returns words read (incl. f recompute touches) */
static long ckpt_load(State* s, const Ckpt* c) {
    memcpy(s->g, c->g, sizeof s->g);
    memcpy(s->parent, c->parent, sizeof s->parent);
    memcpy(s->closed, c->closed, sizeof s->closed);
    memcpy(s->open, c->open, sizeof(int) * c->nopen);
    s->nopen = c->nopen;
    s->expansions = c->expansions;
    s->work = c->work;
    for (int i = 0; i < N; i++) s->f[i] = (s->g[i] >= INF) ? INF : s->g[i] + heur(i);
    return 4L*N + c->nopen + 3;
}

/* A* identical in behaviour to grid.c (linear-scan open, swap-remove, strict-< tie
 * break). K > 0: checkpoint every K expansions into ck. interrupt_at > 0: stop and
 * return -3 once expansions == interrupt_at. Returns path length, -1 no path. */
static int search(const Map* m, State* s, int* out, int cap,
                  int K, int interrupt_at, Ckpt* ck, long* traffic) {
    if (K > 0 && ck) *traffic += ckpt_save(ck, s);   /* expansion-0 checkpoint */
    while (s->nopen > 0) {
        int best = 0;
        for (int i = 1; i < s->nopen; i++) {
            s->work++;
            if (s->f[s->open[i]] < s->f[s->open[best]]) best = i;
        }
        int cur = s->open[best];
        s->open[best] = s->open[--s->nopen];
        if (cur == idx(GX, GY)) {
            int n = 0, c = cur;
            int rev[N];
            while (c != -1 && n < N) { rev[n++] = c; c = s->parent[c]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        s->closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            s->work++;
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (s->closed[ni]) continue;
            int ng = s->g[cur] + 1;
            if (ng < s->g[ni]) {
                s->g[ni] = ng;
                s->f[ni] = ng + heur(ni);
                s->parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < s->nopen; i++) if (s->open[i] == ni) { found = 1; break; }
                if (!found) s->open[s->nopen++] = ni;
            }
        }
        s->expansions++;
        if (K > 0 && ck && s->expansions % K == 0) *traffic += ckpt_save(ck, s);
        if (interrupt_at > 0 && s->expansions == interrupt_at) return -3;
    }
    return -1;
}

typedef struct {
    int n;              /* final path length after resume */
    int path[N];
    long rec_work;      /* search work after resume + checkpoint load cost */
    long traffic;       /* checkpoint write traffic over the run, in words */
    int ckpt_exp;       /* expansion count of the checkpoint actually resumed from */
    int lost;           /* expansions done but not checkpointed (redone on resume) */
} Run;

static Run resume_run(const Map* m, int K, int interrupt_at, long* save_words_out) {
    Run r;
    State s; Ckpt ck; long tr = 0;
    state_init(&s, 0, 0);
    int rc = search(m, &s, r.path, N, K, interrupt_at, &ck, &tr);
    if (rc != -3) { fprintf(stderr, "expected interrupt, got %d\n", rc); exit(2); }
    State q;
    long load_words = ckpt_load(&q, &ck);
    long before = q.work;
    r.n = search(m, &q, r.path, N, 0, -1, NULL, &tr);
    r.rec_work = (q.work - before) + load_words;
    r.traffic = tr;
    r.ckpt_exp = ck.expansions;
    r.lost = interrupt_at - ck.expansions;
    if (save_words_out) *save_words_out = 3L*N + ck.nopen + 3;
    return r;
}

/* first divergence index, or -1 if identical */
static int path_first_diff(const int* a, int na, const int* b, int nb) {
    int m = na < nb ? na : nb;
    for (int i = 0; i < m; i++) if (a[i] != b[i]) return i;
    return na != nb ? m : -1;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static unsigned rs;
static unsigned xr(void) { rs ^= rs<<13; rs ^= rs>>17; rs ^= rs<<5; return rs; }

int main(void) {
    Map m;
    State base;
    int path0[N], n0 = -1;
    unsigned seed;
    GX = W - 1; GY = H - 1;
    for (seed = 1; seed < 100; seed++) {
        rs = seed;
        memset(&m, 0, sizeof m);
        for (int i = 0; i < N; i++) if (xr() % 1000 < 220) m.blocked[i] = 1;
        m.blocked[idx(0, 0)] = 0; m.blocked[idx(W-1, H-1)] = 0;
        state_init(&base, 0, 0);
        long tr = 0;
        n0 = search(&m, &base, path0, N, 0, -1, NULL, &tr);
        if (n0 > 0 && base.expansions >= 400) break;
    }
    if (n0 <= 0) { printf("no_solvable_map\n"); return 1; }

    const int E = base.expansions;
    const long Wfull = base.work;
    const int interrupt_at = 3 * E / 4;
    const double wbar = (double)Wfull / E;
    const long ckpt_words = (long)sizeof(Ckpt) / 4;
    printf("map_seed=%u E=%d W_full=%ld path_len=%d\n", seed, E, Wfull, n0);
    printf("interrupt_at=%d ckpt_bytes=%zu ckpt_words=%ld\n",
           interrupt_at, sizeof(Ckpt), ckpt_words);

    /* derived quantities */
    double kstar = sqrt(2.0 * E * ckpt_words / wbar);
    double e_min = 2.0 * ckpt_words / wbar;
    printf("wbar=%.1f Kstar=%.1f e_min_expansions=%.1f\n", wbar, kstar, e_min);

    /* granularity sweep: recovery-time curve vs write-traffic curve */
    printf("sweep: K ckpt_exp lost rec_ratio traffic_words total\n");
    double best_total = 1e30; int best_K = 1;
    int feas_K = 1;   /* largest K still inside the 60% recovery bar */
    for (int K = 1; K <= 2*E; K *= 2) {
        Run r = resume_run(&m, K, interrupt_at, NULL);
        double ratio = (double)r.rec_work / Wfull;
        double total = (double)r.rec_work + r.traffic;
        printf("sweep: %d %d %d %.4f %ld %.0f\n",
               K, r.ckpt_exp, r.lost, ratio, r.traffic, total);
        if (total < best_total) { best_total = total; best_K = K; }
        if (ratio <= 0.6) feas_K = K;
        if (K >= E) break;
    }
    printf("sweep_best_K=%d feasible_max_K=%d\n", best_K, feas_K);

    /* three checkpoint frequencies: fine / medium / coarse.
     * medium = coarsest K that still meets the 60% recovery bar. */
    int Ks[3] = { 1, feas_K, E };
    const char* tags[3] = { "fine", "mid", "coarse" };
    for (int t = 0; t < 3; t++) {
        Run r = resume_run(&m, Ks[t], interrupt_at, NULL);
        double ratio = (double)r.rec_work / Wfull;
        int d = path_first_diff(path0, n0, r.path, r.n);
        printf("%s: K=%d ckpt_exp=%d lost=%d rec_work=%ld ratio=%.4f "
               "traffic_words=%ld identical=%d\n",
               tags[t], Ks[t], r.ckpt_exp, r.lost, r.rec_work, ratio,
               r.traffic, d < 0);
        if (d >= 0)
            printf("%s: MISMATCH interrupt_at=%d first_diff=%d len_diff=%d\n",
                   tags[t], interrupt_at, d, r.n - n0);
        char nm[64];
        snprintf(nm, sizeof nm, "resume_identical_%s", tags[t]);
        check(nm, d < 0);
        if (t < 2) {   /* coarse is expected to break the bar; checked below */
            snprintf(nm, sizeof nm, "within_60pct_%s", tags[t]);
            check(nm, ratio <= 0.6);
        }
    }

    /* coarse granularity is expected to blow the 60% bar: the detector must fire */
    Run rc = resume_run(&m, E, interrupt_at, NULL);
    check("coarse_flagged_insufficient", (double)rc.rec_work / Wfull > 0.6);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
