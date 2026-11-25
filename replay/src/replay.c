#define _POSIX_C_SOURCE 199309L
/*
 * replay: deterministic replay harness for server-side path validation.
 *
 * Red-team question: if a cheater wants to beat server validation, the first
 * thing they exploit is non-determinism -- the server cannot reproduce the
 * reference path, so any dispute about a submitted move stream becomes
 * unresolvable. This harness runs the same scenario inputs through four
 * A* engines that differ in exactly one dimension each:
 *
 *   MODE_FIXED  canonical order, integer g, (f,id) total order  (baseline)
 *   MODE_HASH   open-set traversal order keyed by per-run hash salt
 *   MODE_FLOAT  float32 cost accumulation, strict float relax, id tie-break
 *   MODE_THREAD f-level parallel expansion, racy first-writer merge
 *   MODE_FTIE   float32 costs AND salted equal-f tie-break (the combination
 *               that actually carries rounding flips into the final path)
 *
 * For every scenario we replay R times with identical map/start/goal and only
 * the environment seed (hash salts / scheduling jitter) changing, then diff
 * the resulting paths point by point. Pass criterion: zero diff frames.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#define MAXW 128
#define MAXH 96
#define MAXN (MAXW * MAXH)
#define MAXHHEAP (MAXN * 8)
#define MAXCAND (MAXN * 4)
#define NSCEN 4
#define NREPLAY 40
#define ENV_BASE 0x51eedULL

enum { MODE_FIXED = 0, MODE_HASH = 1, MODE_FLOAT = 2, MODE_THREAD = 3,
       MODE_FTIE = 4, NMODES = 5 };
static const char* MODE_NAME[5] = { "fixed", "hash", "float", "thread",
                                    "float_tie" };

/* ---------------- deterministic PRNG / hashing ---------------- */

static uint64_t rng_state;
static uint64_t xs64(void) {
    uint64_t x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x;
    return x;
}
static void xs_seed(uint64_t s) { rng_state = s ? s : 0xdeadbeefULL; }

/* pure hash: same (a,b) always maps to the same value, no global state */
static uint64_t splitmix64(uint64_t a, uint64_t b) {
    uint64_t z = a + 0x9e3779b97f4a7c15ULL + b * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 30)) * 0xbf58476d1ce4e5b9ULL;
    z = (z ^ (z >> 27)) * 0x94d049bb133111ebULL;
    return z ^ (z >> 31);
}

/* terrain edge cost of entering cell ni, intrinsic to the map, identical in
 * every replay. Integer set {9,10,11,12}; float engine divides by 10. */
#define TERRAIN_SEED 0x7e57a11ULL
static int cell_cost(int ni) {
    static const int c[4] = { 10, 11, 12, 9 };
    return c[splitmix64((uint64_t)ni, TERRAIN_SEED) & 3u];
}

/* ---------------- scenario ---------------- */

typedef struct {
    int W, H, n;
    int sx, sy, gx, gy;
    unsigned char blocked[MAXN];
} Scen;

static int sidx(const Scen* s, int x, int y) { return y * s->W + x; }

static int bfs_reachable(const Scen* s) {
    static int vis[MAXN];
    int q[MAXN], qh = 0, qt = 0;
    for (int i = 0; i < s->n; i++) vis[i] = 0;
    int start = sidx(s, s->sx, s->sy), goal = sidx(s, s->gx, s->gy);
    vis[start] = 1; q[qt++] = start;
    const int dx[4] = { 1, -1, 0, 0 }, dy[4] = { 0, 0, 1, -1 };
    while (qh < qt) {
        int cur = q[qh++];
        if (cur == goal) return 1;
        int cx = cur % s->W, cy = cur / s->W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || ny < 0 || nx >= s->W || ny >= s->H) continue;
            int ni = sidx(s, nx, ny);
            if (s->blocked[ni] || vis[ni]) continue;
            vis[ni] = 1; q[qt++] = ni;
        }
    }
    return 0;
}

static void gen_scen(Scen* s, int W, int H, int blockPct, uint64_t seed) {
    s->W = W; s->H = H; s->n = W * H;
    s->sx = 1; s->sy = 1; s->gx = W - 2; s->gy = H - 2;
    uint64_t try_seed = seed;
    for (;;) {
        xs_seed(try_seed);
        for (int i = 0; i < s->n; i++)
            s->blocked[i] = (xs64() % 100) < (uint64_t)blockPct ? 1 : 0;
        s->blocked[sidx(s, s->sx, s->sy)] = 0;
        s->blocked[sidx(s, s->gx, s->gy)] = 0;
        if (bfs_reachable(s)) return;
        try_seed += 1000; /* regenerate, keeping the scenario deterministic */
    }
}

/* ---------------- A* engine ---------------- */

static int gi[MAXN];          /* exact integer g */
static float gf[MAXN];        /* float32 g (MODE_FLOAT) */
static int gti[MAXN];         /* exact integer terrain sum alongside gf */
static int par[MAXN], closed[MAXN], reached[MAXN];
static uint32_t salt[MAXN];

/* lazy binary heap */
static int hf_i[MAXHHEAP];
static float hf_f[MAXHHEAP];
static uint32_t h_tie[MAXHHEAP];
static int h_node[MAXHHEAP], h_entg[MAXHHEAP];
static float h_entgf[MAXHHEAP];
static int hsz;

static int h_less(int mode, int a, int b) {
    if (mode == MODE_FLOAT || mode == MODE_FTIE) {
        if (hf_f[a] < hf_f[b]) return 1;
        if (hf_f[a] > hf_f[b]) return 0;
        if (mode == MODE_FTIE)
            return h_tie[a] < h_tie[b];   /* float equal-f: salted tie-break */
        return h_node[a] < h_node[b];     /* float mode keeps id tie-break */
    }
    if (hf_i[a] != hf_i[b]) return hf_i[a] < hf_i[b];
    return h_tie[a] < h_tie[b];
}

static void h_swap(int a, int b) {
    int t; float ft; uint32_t u;
    t = hf_i[a]; hf_i[a] = hf_i[b]; hf_i[b] = t;
    ft = hf_f[a]; hf_f[a] = hf_f[b]; hf_f[b] = ft;
    u = h_tie[a]; h_tie[a] = h_tie[b]; h_tie[b] = u;
    t = h_node[a]; h_node[a] = h_node[b]; h_node[b] = t;
    t = h_entg[a]; h_entg[a] = h_entg[b]; h_entg[b] = t;
    ft = h_entgf[a]; h_entgf[a] = h_entgf[b]; h_entgf[b] = ft;
}

static void h_push(int mode, int fi, float ff, uint32_t tie,
                   int node, int entg, float entgf) {
    int c = hsz++;
    hf_i[c] = fi; hf_f[c] = ff; h_tie[c] = tie;
    h_node[c] = node; h_entg[c] = entg; h_entgf[c] = entgf;
    while (c > 0) {
        int p = (c - 1) / 2;
        if (!h_less(mode, c, p)) break;
        h_swap(c, p); c = p;
    }
}

static void h_pop(int mode) {
    hsz--;
    if (hsz > 0) {
        h_swap(0, hsz);
        int p = 0;
        for (;;) {
            int l = 2 * p + 1, r = l + 1, best = p;
            if (l < hsz && h_less(mode, l, best)) best = l;
            if (r < hsz && h_less(mode, r, best)) best = r;
            if (best == p) break;
            h_swap(p, best); p = best;
        }
    }
}

/* candidate buffer for the racy (thread) merge */
typedef struct { int ni, from; } Cand;
static Cand cand[MAXCAND];
static int ncand;

/* instrumentation: float-mode relax events across the most recent search */
static long cnt_relax, cnt_eqint_flip, cnt_flip_any;
/* last equal-integer-cost float flip: SAME node, two parents, two float g */
static int ev_node, ev_par_old, ev_par_new, ev_ti;
static float ev_gf_old, ev_gf_new;
static int ev_seen;

static void dir_perm(int node, uint64_t env_seed, int perm[4]) {
    for (int i = 0; i < 4; i++) perm[i] = i;
    for (int i = 3; i > 0; i--) {
        int j = (int)(splitmix64((uint64_t)node, env_seed + (uint64_t)i * 7u) % (uint64_t)(i + 1));
        int t = perm[i]; perm[i] = perm[j]; perm[j] = t;
    }
}

static const int DX[4] = { 1, -1, 0, 0 };
static const int DY[4] = { 0, 0, 1, -1 };

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* returns path point count, -1 unreachable, -2 cap exceeded */
static int astar(const Scen* sc, int mode, uint64_t env_seed,
                 int* out, int cap, double* usec) {
    const int W = sc->W, H = sc->H, n = sc->n;
    double t0 = now_sec();

    for (int i = 0; i < n; i++) {
        gi[i] = 1 << 29; gf[i] = 1e30f; gti[i] = 1 << 29;
        par[i] = -1; closed[i] = 0; reached[i] = 0;
        salt[i] = (uint32_t)splitmix64((uint64_t)i, env_seed);
    }
    hsz = 0; ncand = 0;
    cnt_relax = cnt_eqint_flip = cnt_flip_any = 0;
    ev_seen = 0;

    int s = sidx(sc, sc->sx, sc->sy), goal = sidx(sc, sc->gx, sc->gy);
    int h0 = (abs(sc->gx - sc->sx) + abs(sc->gy - sc->sy));
    gi[s] = 0; gf[s] = 0.0f; gti[s] = 0; reached[s] = 1;
    if (mode == MODE_FLOAT || mode == MODE_FTIE)
        h_push(mode, 0, (float)h0 * 0.9f,
               mode == MODE_FTIE ? salt[s] : 0, s, 0, 0.0f);
    else
        h_push(mode, h0 * 9, 0.0f, mode == MODE_HASH ? salt[s] : (uint32_t)s,
               s, 0, 0.0f);

    int batch[MAXN], nb;
    int found = 0;

    while (hsz > 0) {
        /* extract the whole min-f level: in MODE_THREAD this is the layer
         * expanded "in parallel" */
        nb = 0;
        int min_i = 0; float min_f = 0.0f; int have_min = 0;
        while (hsz > 0) {
            int node = h_node[0], entg = h_entg[0];
            float entgf = h_entgf[0];
            int fi = hf_i[0]; float ff = hf_f[0];
            uint32_t tie0 = h_tie[0];
            h_pop(mode);
            if (closed[node]) continue;                       /* lazy stale */
            if (mode == MODE_FLOAT || mode == MODE_FTIE) {
                if (entgf != gf[node]) continue;              /* relaxed again */
            } else {
                if (entg != gi[node]) continue;
            }
            if (!have_min) { min_i = fi; min_f = ff; have_min = 1; }
            int above = (mode == MODE_FLOAT || mode == MODE_FTIE)
                        ? (ff > min_f) : (fi > min_i);
            if (above) {
                /* belongs to the next f-level: put it back with its OWN tie */
                h_push(mode, fi, ff, tie0, node, entg, entgf);
                break;
            }
            closed[node] = 1;
            batch[nb++] = node;
            if (node == goal) { found = 1; break; }
        }
        if (!nb) break;
        if (found) break;

        ncand = 0;
        for (int bi = 0; bi < nb; bi++) {
            int cur = batch[bi];
            int cx = cur % W, cy = cur / W;
            int perm[4] = { 0, 1, 2, 3 };
            if (mode != MODE_FIXED) dir_perm(cur, env_seed, perm);
            for (int k = 0; k < 4; k++) {
                int d = perm[k];
                int nx = cx + DX[d], ny = cy + DY[d];
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                int ni = ny * W + nx;
                if (sc->blocked[ni] || closed[ni]) continue;

                if (mode == MODE_THREAD) {
                    cand[ncand].ni = ni; cand[ncand].from = cur;
                    if (ncand < MAXCAND - 1) ncand++;
                    continue;
                }
                if (mode == MODE_FLOAT || mode == MODE_FTIE) {
                    int c = cell_cost(ni);
                    float ngf = gf[cur] + (float)c * 0.1f;
                    int nti = gti[cur] + c;
                    cnt_relax++;
                    if (ngf < gf[ni]) {                 /* strict float relax */
                        cnt_flip_any++;
                        if (reached[ni] && nti == gti[ni]) {
                            cnt_eqint_flip++;
                            if (!ev_seen) {
                                /* same node ni: old parent vs new parent,
                                 * identical integer cost, different float g */
                                ev_seen = 1; ev_node = ni;
                                ev_par_old = par[ni]; ev_par_new = cur;
                                ev_gf_old = gf[ni]; ev_gf_new = ngf; ev_ti = nti;
                            }
                        }
                        gf[ni] = ngf;
                        gti[ni] = nti;
                        par[ni] = cur;
                        int hh = abs(sc->gx - nx) + abs(sc->gy - ny);
                        h_push(mode, 0, ngf + (float)hh * 0.9f,
                               mode == MODE_FTIE ? salt[ni] : (uint32_t)ni,
                               ni, 0, ngf);
                        reached[ni] = 1;
                    }
                } else {
                    int c = cell_cost(ni);
                    int ngi = gi[cur] + c;
                    if (ngi < gi[ni]) {                 /* strict int relax */
                        gi[ni] = ngi; par[ni] = cur; reached[ni] = 1;
                        int hh = (abs(sc->gx - nx) + abs(sc->gy - ny)) * 9;
                        uint32_t tie = mode == MODE_HASH ? salt[ni] : (uint32_t)ni;
                        h_push(mode, ngi + hh, 0.0f, tie, ni, ngi, 0.0f);
                    }
                }
            }
        }

        if (mode == MODE_THREAD) {
            /* simulate OS scheduling jitter over the parallel merge */
            for (int i = ncand - 1; i > 0; i--) {
                int j = (int)(xs64() % (uint64_t)(i + 1));
                Cand t = cand[i]; cand[i] = cand[j]; cand[j] = t;
            }
            for (int i = 0; i < ncand; i++) {
                int ni = cand[i].ni, from = cand[i].from;
                if (closed[ni] || reached[ni]) continue; /* first writer wins */
                reached[ni] = 1;
                gi[ni] = gi[from] + 1;
                par[ni] = from;
                int nx = ni % W, ny = ni / W;
                int hh = (abs(sc->gx - nx) + abs(sc->gy - ny)) * 9;
                h_push(mode, gi[ni] + hh, 0.0f, (uint32_t)ni, ni, gi[ni], 0.0f);
            }
        }
    }

    *usec = (now_sec() - t0) * 1e6;
    if (!found) return -1;

    int rev[MAXN], rn = 0, c = goal;
    while (c != -1 && rn < n) { rev[rn++] = c; c = par[c]; }
    if (rn > cap) return -2;
    for (int i = 0; i < rn; i++) out[i] = rev[rn - 1 - i];
    return rn;
}

/* ---------------- ordered array reference (sorting overhead study) -------- */

/* Same canonical specification as MODE_FIXED -- integer costs, f-level
 * expansion, node-id tie break, lazy stale-entry skipping -- but backed by an
 * unsorted array with a linear minimum scan instead of a binary heap. Any
 * output difference between the two is therefore a bug, not a spec gap. */
static int astar_ordered_array(const Scen* sc, int* out, int cap, double* usec) {
    const int W = sc->W, H = sc->H, n = sc->n;
    double t0 = now_sec();
    static int g2[MAXN], f2[MAXN], cl2[MAXN], pa2[MAXN], op2[MAXN];
    static int og[MAXN];                 /* g snapshot at insertion, for lazy */
    for (int i = 0; i < n; i++) {
        g2[i] = 1 << 29; f2[i] = 1 << 29; cl2[i] = 0; pa2[i] = -1; og[i] = -1;
    }
    int nopen = 0;
    int s = sidx(sc, sc->sx, sc->sy), goal = sidx(sc, sc->gx, sc->gy);
    g2[s] = 0; f2[s] = (abs(sc->gx - sc->sx) + abs(sc->gy - sc->sy)) * 9;
    og[s] = 0;
    op2[nopen++] = s;

    while (nopen > 0) {
        /* one f-level: extract every node tied at the minimum f, id order */
        int minf = 1 << 30;
        for (int i = 0; i < nopen; i++)
            if (f2[op2[i]] < minf) minf = f2[op2[i]];
        int layer[MAXN], nl = 0;
        for (int i = nopen - 1; i >= 0; i--) {
            int node = op2[i];
            if (f2[node] != minf) continue;
            if (cl2[node] || og[node] != g2[node]) {     /* lazy stale entry */
                op2[i] = op2[--nopen];
                continue;
            }
            layer[nl++] = node;
            op2[i] = op2[--nopen];
        }
        for (int a = 0; a < nl; a++)
            for (int b = a + 1; b < nl; b++)
                if (layer[b] < layer[a]) { int t = layer[a]; layer[a] = layer[b]; layer[b] = t; }

        for (int li = 0; li < nl; li++) {
            int cur = layer[li];
            cl2[cur] = 1;
            if (cur == goal) {
                int rev[MAXN], rn = 0, c = cur;
                *usec = (now_sec() - t0) * 1e6;
                while (c != -1 && rn < n) { rev[rn++] = c; c = pa2[c]; }
                if (rn > cap) return -2;
                for (int i = 0; i < rn; i++) out[i] = rev[rn - 1 - i];
                return rn;
            }
            int cx = cur % W, cy = cur / W;
            for (int d = 0; d < 4; d++) {
                int nx = cx + DX[d], ny = cy + DY[d];
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                int ni = ny * W + nx;
                if (sc->blocked[ni] || cl2[ni]) continue;
                int ng = g2[cur] + cell_cost(ni);
                if (ng < g2[ni]) {
                    g2[ni] = ng; pa2[ni] = cur;
                    f2[ni] = ng + (abs(sc->gx - nx) + abs(sc->gy - ny)) * 9;
                    og[ni] = ng;
                    int found = 0;
                    for (int i = 0; i < nopen; i++)
                        if (op2[i] == ni) { found = 1; break; }
                    if (!found) op2[nopen++] = ni;
                }
            }
        }
    }
    *usec = (now_sec() - t0) * 1e6;
    return -1;
}

/* ---------------- experiment driver ---------------- */

typedef struct {
    int cmp_total;        /* replay pair comparisons */
    int cmp_diverged;     /* comparisons with >=1 differing frame */
    long diff_frames;     /* total differing point slots over all comparisons */
    int first_scen;       /* first path-divergence diagnostics */
    int first_replay;
    int first_index;
    int first_node_a, first_node_b;
    int len_a, len_b;
    /* equal-int float-flip evidence (same node, two parents) */
    int flip_scen, flip_node, flip_par_old, flip_par_new, flip_ti, flip_seen;
    float flip_gf_old, flip_gf_new;
    double total_usec;
    int searches;
} ModeStat;

static void stat_init(ModeStat* m) { memset(m, 0, sizeof(*m)); m->first_index = -1; }

static int paths_equal(const int* a, int na, const int* b, int nb,
                       int* first_idx, int* node_a, int* node_b) {
    int common = na < nb ? na : nb;
    for (int i = 0; i < common; i++) {
        if (a[i] != b[i]) {
            *first_idx = i; *node_a = a[i]; *node_b = b[i];
            return 0;
        }
    }
    if (na != nb) {
        *first_idx = common;
        *node_a = na > nb ? a[common] : -1;
        *node_b = nb > na ? b[common] : -1;
        return 0;
    }
    return 1;
}

int main(int argc, char** argv) {
    const char* json_path = NULL;
    for (int i = 1; i < argc; i++)
        if (!strcmp(argv[i], "--json") && i + 1 < argc) json_path = argv[++i];

    const struct { int W, H, pct; const char* tag; } SIZES[3] = {
        { 32, 24, 15, "small_32x24" },
        { 64, 48, 18, "medium_64x48" },
        { 128, 96, 22, "large_128x96" },
    };

    Scen* scens = calloc(NSCEN, sizeof(Scen));
    ModeStat stats[3][NMODES];
    for (int z = 0; z < 3; z++)
        for (int m = 0; m < NMODES; m++) stat_init(&stats[z][m]);

    int *refp = malloc(MAXN * sizeof(int));
    int *rep = malloc(MAXN * sizeof(int));
    int *bp1 = malloc(MAXN * sizeof(int));
    int *bp2 = malloc(MAXN * sizeof(int));

    printf("== deterministic replay experiment ==\n");
    printf("scenarios/size=%d  replays/scenario=%d  (R-1 pair comparisons each)\n\n",
           NSCEN, NREPLAY);

    for (int z = 0; z < 3; z++) {
        for (int k = 0; k < NSCEN; k++)
            gen_scen(&scens[k], SIZES[z].W, SIZES[z].H, SIZES[z].pct,
                     splitmix64(0xabcdefULL, (uint64_t)(z * 100 + k)));

        printf("[%s] cells=%d blocked%%=%d\n", SIZES[z].tag,
               scens[0].n, SIZES[z].pct);
        for (int mode = 0; mode < NMODES; mode++) {
            ModeStat* st = &stats[z][mode];
            int isfloat = (mode == MODE_FLOAT || mode == MODE_FTIE);
            long relax_sum = 0, eqint_sum = 0, flip_sum = 0;
            for (int k = 0; k < NSCEN; k++) {
                double us;
                int nref = astar(&scens[k], mode, ENV_BASE, refp, MAXN, &us);
                st->total_usec += us; st->searches++;
                if (isfloat) {
                    relax_sum += cnt_relax; eqint_sum += cnt_eqint_flip;
                    flip_sum += cnt_flip_any;
                    if (ev_seen && !st->flip_seen) {
                        st->flip_seen = 1; st->flip_scen = k;
                        st->flip_node = ev_node;
                        st->flip_par_old = ev_par_old; st->flip_par_new = ev_par_new;
                        st->flip_gf_old = ev_gf_old; st->flip_gf_new = ev_gf_new;
                        st->flip_ti = ev_ti;
                    }
                }
                if (nref < 0) { fprintf(stderr, "reference search failed\n"); return 2; }
                for (int r = 1; r < NREPLAY; r++) {
                    int nr = astar(&scens[k], mode, ENV_BASE + (uint64_t)r,
                                   rep, MAXN, &us);
                    st->total_usec += us; st->searches++;
                    if (isfloat) {
                        relax_sum += cnt_relax; eqint_sum += cnt_eqint_flip;
                        flip_sum += cnt_flip_any;
                    }
                    if (nr < 0) { fprintf(stderr, "replay search failed\n"); return 2; }
                    st->cmp_total++;
                    int fi, na, nb;
                    if (!paths_equal(refp, nref, rep, nr, &fi, &na, &nb)) {
                        st->cmp_diverged++;
                        int common = nref < nr ? nref : nr;
                        st->diff_frames += (long)abs(nref - nr);
                        for (int i = fi; i < common; i++)
                            if (refp[i] != rep[i]) st->diff_frames++;
                        if (st->first_index < 0) {
                            st->first_scen = k; st->first_replay = r;
                            st->first_index = fi;
                            st->first_node_a = na; st->first_node_b = nb;
                            st->len_a = nref; st->len_b = nr;
                        }
                    }
                }
            }
            double rate = 100.0 * st->cmp_diverged / st->cmp_total;
            printf("  %-9s diverged_cmp=%3d/%-3d determinism=%6.2f%% "
                   "diff_frames=%-5ld avg_us=%7.2f\n",
                   MODE_NAME[mode], st->cmp_diverged, st->cmp_total,
                   100.0 - rate, st->diff_frames,
                   st->total_usec / st->searches);
            if (isfloat)
                printf("           [instrument] relax=%ld strict_flips=%ld "
                       "equal-int-cost flips=%ld\n",
                       relax_sum, flip_sum, eqint_sum);
        }
        printf("\n");
    }

    /* first-divergence evidence */
    static const char* CAUSE[5] = {
        "none: total-order tie break + fixed neighbour order",
        "hash: open-set traversal order follows per-run hash salt; equal-f nodes have no total-order tie break, so the parent flips",
        "float: strict float relax alone -- min() absorbs order-dependent rounding, every reachable node keeps the global minimum gf; path stays reproducible",
        "thread: f-level frontier merged with first-writer-wins race; no deterministic join, scheduling jitter picks the parent",
        "float_tie: float32 rounding flips the parent on integer-equal routes AND equal-f heap order follows a per-run salt, so the alternative equal-cost route is actually selected",
    };
    printf("== first divergence evidence ==\n");
    for (int z = 0; z < 3; z++) {
        for (int mode = 1; mode < NMODES; mode++) {
            ModeStat* st = &stats[z][mode];
            if (st->first_index < 0) {
                printf("%s/%s: no divergence (reproduced every replay)\n",
                       SIZES[z].tag, MODE_NAME[mode]);
                continue;
            }
            int ax = st->first_node_a % SIZES[z].W, ay = st->first_node_a / SIZES[z].W;
            int bx = st->first_node_b % SIZES[z].W, by = st->first_node_b / SIZES[z].W;
            printf("%s/%s: scen#%d replay#%d first_diff_point_index=%d "
                   "ref=(%d,%d) replay=(%d,%d) len %d vs %d\n  cause: %s\n",
                   SIZES[z].tag, MODE_NAME[mode], st->first_scen,
                   st->first_replay, st->first_index, ax, ay, bx, by,
                   st->len_a, st->len_b, CAUSE[mode]);
            if (mode == MODE_FLOAT || mode == MODE_FTIE) {
                if (st->flip_seen) {
                    int Wz = SIZES[z].W;
                    int nx = st->flip_node % Wz, ny = st->flip_node / Wz;
                    int ox = st->flip_par_old % Wz, oy = st->flip_par_old / Wz;
                    int qx = st->flip_par_new % Wz, qy = st->flip_par_new / Wz;
                    float fa = st->flip_gf_old, fb = st->flip_gf_new;
                    int expo; frexpf(fabs(fa) > fabs(fb) ? fa : fb, &expo);
                    double ulp1 = ldexp(1.0, expo - 24);
                    double gap_ulp = fabs((double)fa - (double)fb) / ulp1;
                    printf("  rounding evidence: scen#%d node=(%d,%d) old_parent=(%d,%d) "
                           "new_parent=(%d,%d)\n    SAME int terrain cost=%d both routes, "
                           "gf_old=%.9f gf_new=%.9f gap=%.3f ulp -> strict ngf<gf flips parent\n",
                           st->flip_scen, nx, ny, ox, oy, qx, qy,
                           st->flip_ti, fa, fb, gap_ulp);
                } else {
                    printf("  rounding evidence: no equal-int float flip observed\n");
                }
            }
        }
    }
    printf("\n");

    /* ordering overhead: binary heap vs linear-scan array, SAME canonical
     * specification (f-level extraction + id tie), outputs cross-checked */
    printf("== ordering overhead (same spec, heap vs linear-scan array) ==\n");
    double bench_us_heap[3], bench_us_scan[3];
    int bench_iter[3] = { 200, 100, 25 };
    for (int z = 0; z < 3; z++) {
        /* regenerate this size's scenarios for a fair, size-local bench */
        for (int k = 0; k < NSCEN; k++)
            gen_scen(&scens[k], SIZES[z].W, SIZES[z].H, SIZES[z].pct,
                     splitmix64(0xabcdefULL, (uint64_t)(z * 100 + k)));
        double th = 0, ts = 0;
        int mismatch = 0, iters = bench_iter[z];
        for (int it = 0; it < iters; it++) {
            for (int k = 0; k < NSCEN; k++) {
                double u1, u2;
                int n1 = astar(&scens[k], MODE_FIXED, ENV_BASE, bp1, MAXN, &u1);
                int n2 = astar_ordered_array(&scens[k], bp2, MAXN, &u2);
                th += u1; ts += u2;
                int fi, na, nb;
                if (n1 != n2 || !paths_equal(bp1, n1, bp2, n2, &fi, &na, &nb))
                    mismatch++;
            }
        }
        int searches = iters * NSCEN;
        bench_us_heap[z] = th / searches;
        bench_us_scan[z] = ts / searches;
        double overhead = 100.0 * (th - ts) / ts;
        printf("%s: heap=%.2f us/search  linear_scan=%.2f us/search  "
               "delta=%+.2f%%  output_mismatches=%d (iters=%d)\n",
               SIZES[z].tag, bench_us_heap[z], bench_us_scan[z],
               overhead, mismatch, iters);
    }
    printf("\n");

    /* verdict */
    int pass = 1;
    for (int z = 0; z < 3; z++)
        if (stats[z][MODE_FIXED].diff_frames != 0) pass = 0;
    printf("== verdict: fixed-engine zero diff frames on all sizes? %s ==\n",
           pass ? "PASS" : "FAIL");

    if (json_path) {
        FILE* f = fopen(json_path, "w");
        if (!f) { perror("json"); return 2; }
        fprintf(f, "{\n  \"replays\": %d, \"scenarios_per_size\": %d,\n",
                NREPLAY, NSCEN);
        fprintf(f, "  \"sizes\": [\n");
        for (int z = 0; z < 3; z++) {
            fprintf(f, "    {\"tag\": \"%s\", \"W\": %d, \"H\": %d, \"cells\": %d, "
                    "\"block_pct\": %d, \"modes\": {\n",
                    SIZES[z].tag, SIZES[z].W, SIZES[z].H,
                    SIZES[z].W * SIZES[z].H, SIZES[z].pct);
            for (int m = 0; m < NMODES; m++) {
                ModeStat* st = &stats[z][m];
                fprintf(f, "      \"%s\": {\"cmp_total\": %d, \"cmp_diverged\": %d, "
                        "\"determinism_pct\": %.4f, \"diff_frames\": %ld, "
                        "\"avg_us\": %.4f, \"first_scen\": %d, "
                        "\"first_replay\": %d, \"first_index\": %d, "
                        "\"first_node_ref\": %d, \"first_node_replay\": %d, "
                        "\"len_ref\": %d, \"len_replay\": %d, "
                        "\"flip_node\": %d, \"flip_par_old\": %d, "
                        "\"flip_par_new\": %d, \"flip_int_cost\": %d, "
                        "\"flip_gf_old\": %.9f, \"flip_gf_new\": %.9f}%s\n",
                        MODE_NAME[m], st->cmp_total, st->cmp_diverged,
                        100.0 * (st->cmp_total - st->cmp_diverged) / st->cmp_total,
                        st->diff_frames, st->total_usec / st->searches,
                        st->first_scen, st->first_replay, st->first_index,
                        st->first_node_a, st->first_node_b,
                        st->len_a, st->len_b,
                        st->flip_seen ? st->flip_node : -1,
                        st->flip_seen ? st->flip_par_old : -1,
                        st->flip_seen ? st->flip_par_new : -1,
                        st->flip_seen ? st->flip_ti : -1,
                        st->flip_gf_old, st->flip_gf_new,
                        m == NMODES - 1 ? "" : ",");
            }
            fprintf(f, "    }, \"bench\": {\"heap_us\": %.4f, \"scan_us\": %.4f, "
                    "\"overhead_pct\": %.4f}}%s\n",
                    bench_us_heap[z], bench_us_scan[z],
                    100.0 * (bench_us_heap[z] - bench_us_scan[z]) / bench_us_scan[z],
                    z == 2 ? "" : ",");
        }
        fprintf(f, "  ],\n  \"verdict_pass\": %s\n}\n", pass ? "true" : "false");
        fclose(f);
        printf("json written: %s\n", json_path);
    }

    free(scens); free(refp); free(rep); free(bp1); free(bp2);
    return pass ? 0 : 1;
}
