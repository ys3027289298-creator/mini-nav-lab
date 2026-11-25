
/* symmetry: mirror-region criterion and path reuse verification.
 *
 * 判据: D(A,B) = sum_{x,y} |walk_A[x,y] - walk_B[W-1-x,y]|
 *       Sym(A,B) <=> D == 0
 * 比例: eta = P/R (R 个区域, P 个两两确认的镜像对, 复用后只算 R-P 次)
 *
 * 程序读取 quadrants.navmap (资源清单的地图部分), 并把区域数扩展到
 * 4 / 8 / 16 三档, 逐格比较“镜像复用路径”和“逐区域单算路径”。
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W * H)
#define MAXR 16
#define MAXQ 12

typedef struct { unsigned char blocked[MAXN]; } Map;

static int idx(int x, int y) { return y * W + x; }
static int in_b(int x, int y) { return 0 <= x && x < W && 0 <= y && y < H; }

/* ---------------- A* (4-neighbour), returns status, fills counters ------- */
enum { ASTAR_OK = 0, ASTAR_BAD_START = -1, ASTAR_BAD_GOAL = -2, ASTAR_UNREACH = -3 };

static int astar(const Map* m, int sx, int sy, int gx, int gy,
                 int* out, int* out_n, long* expanded, long* work) {
    *expanded = 0;
    *work = 0;
    if (!in_b(sx, sy) || m->blocked[idx(sx, sy)]) return ASTAR_BAD_START;
    if (!in_b(gx, gy) || m->blocked[idx(gx, gy)]) return ASTAR_BAD_GOAL;

    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1;
    }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy), t = idx(gx, gy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;

    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++)
            if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        (*work)++;                       /* settled pop */
        if (cur == t) {
            int n = 0, c = cur, rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            *out_n = n;
            return ASTAR_OK;
        }
        if (closed[cur]) continue;
        closed[cur] = 1;
        (*expanded)++;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            (*work)++;                   /* neighbour check */
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!in_b(nx, ny) || m->blocked[idx(nx, ny)]) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < nopen; i++)
                    if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return ASTAR_UNREACH;
}

/* ---------------- symmetry criterion ------------------------------------ */

static int mismatch_count(const Map* a, const Map* b) {
    int d = 0;
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            d += abs((int)a->blocked[idx(x, y)] -
                     (int)b->blocked[idx(W - 1 - x, y)]);
    return d;
}

/* first cell where walk_A[x,y] != walk_B[W-1-x,y]; returns 0 if symmetric */
static int first_mismatch(const Map* a, const Map* b, int* ax, int* ay,
                          int* bx, int* by) {
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++) {
            int mx = W - 1 - x;
            if (a->blocked[idx(x, y)] != b->blocked[idx(mx, y)]) {
                *ax = x; *ay = y; *bx = mx; *by = y;
                return 1;
            }
        }
    return 0;
}

static void mirror_path(int* p, int n) {
    for (int i = 0; i < n; i++) {
        int x = p[i] % W, y = p[i] / W;
        p[i] = idx(W - 1 - x, y);
    }
}

/* first positional difference; index may be == max len (length mismatch) */
static int first_path_diff(const int* a, int na, const int* b, int nb,
                           int* at_index, int* ca, int* cb) {
    int n = na > nb ? na : nb;
    for (int i = 0; i < n; i++) {
        int x = i < na ? a[i] : -1;
        int y = i < nb ? b[i] : -1;
        if (x != y) {
            *at_index = i;
            *ca = x; *cb = y;
            return 1;
        }
    }
    return 0;
}

/* ---------------- regions / manifest parsing ---------------------------- */

typedef struct {
    char id[16], pair[16];
    int ox, oy, rep;
    Map m;
    int qn;
    int q[MAXQ][4];
} Region;

static Region R[MAXR];
static int NR = 0;

static Region* find_region(const char* id) {
    for (int i = 0; i < NR; i++)
        if (strcmp(R[i].id, id) == 0) return &R[i];
    return NULL;
}

static void add_query(Region* r, int sx, int sy, int gx, int gy) {
    if (r->qn < MAXQ) {
        r->q[r->qn][0] = sx; r->q[r->qn][1] = sy;
        r->q[r->qn][2] = gx; r->q[r->qn][3] = gy;
        r->qn++;
    }
}

/* canonical 12 queries: endpoints sit on odd corridor rows, always free in
 * every snake layout used here. */
static const int CANON_Q[MAXQ][4] = {
    {1,1,14,1}, {1,1,1,3}, {2,5,13,7}, {14,9,1,9},
    {1,1,14,9}, {7,3,8,7}, {1,5,14,5}, {14,3,1,7},
    {3,1,3,9}, {12,9,2,1}, {7,1,7,9}, {1,7,14,7}
};

static void snake_layout(Map* m, int xl, int xr, int right_first) {
    memset(m, 0, sizeof(*m));
    for (int x = 0; x < W; x++) { m->blocked[idx(x,0)] = 1; m->blocked[idx(x,H-1)] = 1; m->blocked[idx(x,H-2)] = 1; }
    for (int y = 0; y < H; y++) { m->blocked[idx(0,y)] = 1; m->blocked[idx(W-1,y)] = 1; }
    int seps[4] = {2, 4, 6, 8};
    for (int i = 0; i < 4; i++) {
        int gap = (((i % 2 == 0) == right_first) ? xr : xl);
        for (int x = 1; x < W - 1; x++)
            if (x != gap) m->blocked[idx(x, seps[i])] = 1;
    }
}

static Map mirrored(const Map* a) {
    Map o;
    memset(&o, 0, sizeof(o));
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            o.blocked[idx(W - 1 - x, y)] = a->blocked[idx(x, y)];
    return o;
}

static int load_navmap(const char* path) {
    FILE* f = fopen(path, "r");
    if (!f) return 0;
    char line[128];
    int pending_rows = 0;
    Region* cur = NULL;
    while (fgets(line, sizeof(line), f)) {
        if (pending_rows > 0) {
            char row[32];
            if (sscanf(line, "%16s", row) == 1 && (row[0] == '#' || row[0] == '.')) {
                int y = H - pending_rows;
                for (int x = 0; x < W; x++)
                    if (row[x] == '#') cur->m.blocked[idx(x, y)] = 1;
                pending_rows--;
                continue;
            }
        }
        char tag[32];
        if (sscanf(line, "%31s", tag) != 1) continue;
        if (strcmp(tag, "REGION") == 0) {
            char role[16];
            int ox, oy;
            if (sscanf(line, "REGION %15s %d %d PAIR %15s %15s",
                       R[NR].id, &ox, &oy, R[NR].pair, role) == 5) {
                R[NR].ox = ox; R[NR].oy = oy;
                R[NR].rep = (strcmp(role, "REP") == 0);
                memset(&R[NR].m, 0, sizeof(Map));
                R[NR].qn = 0;
                cur = &R[NR];
                pending_rows = H;
                NR++;
            }
        } else if (strcmp(tag, "Q") == 0) {
            char rid[16];
            int sx, sy, gx, gy;
            if (sscanf(line, "Q %15s %d %d %d %d", rid, &sx, &sy, &gx, &gy) == 5) {
                Region* q = find_region(rid);
                if (q) {
                    add_query(q, sx, sy, gx, gy);
                    Region* p = find_region(q->pair);
                    if (p) add_query(p, W - 1 - sx, sy, W - 1 - gx, gy);
                }
            }
        }
    }
    fclose(f);
    return NR == 4;
}

static void add_generated_pair(int k) {
    int p = 2 + k;
    int xl = 1 + (3 * k) % 4;
    int xr = 14 - (2 * k) % 4;
    int ox = (p % 2) * 16;
    int oy = 24 + (p / 2) * 12;

    Region* a = &R[NR++];
    snprintf(a->id, sizeof(a->id), "W%d", k + 2);
    snprintf(a->pair, sizeof(a->pair), "E%d", k + 2);
    a->ox = ox; a->oy = oy; a->rep = 1; a->qn = MAXQ;
    snake_layout(&a->m, xl, xr, (k % 2 == 0));
    memcpy(a->q, CANON_Q, sizeof(CANON_Q));

    Region* b = &R[NR++];
    snprintf(b->id, sizeof(b->id), "E%d", k + 2);
    snprintf(b->pair, sizeof(b->pair), "W%d", k + 2);
    b->ox = ox == 0 ? 16 : ox - 16; b->oy = oy; b->rep = 0; b->qn = MAXQ;
    b->m = mirrored(&a->m);
    for (int i = 0; i < MAXQ; i++) {
        b->q[i][0] = W - 1 - CANON_Q[i][0];
        b->q[i][1] = CANON_Q[i][1];
        b->q[i][2] = W - 1 - CANON_Q[i][2];
        b->q[i][3] = CANON_Q[i][3];
    }
}

/* ---------------- tier verification ------------------------------------- */

static int Bpath[MAXR][MAXQ][MAXN];
static int Bn[MAXR][MAXQ];
static int Bstatus[MAXR][MAXQ];

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void run_tier(int total_regions) {
    /* NR currently 4; extend deterministically up to requested size */
    while (NR < total_regions) add_generated_pair((NR - 4) / 2);
    int nreg = total_regions;
    int pairs = nreg / 2;

    long base_work = 0, base_exp = 0, reuse_work = 0, reuse_exp = 0;
    int total_q = 0, agree = 0;
    int diff_set = 0;
    char diff_region[16] = "";
    int diff_qi = -1, diff_idx = -1, diff_ca = -1, diff_cb = -1;

    /* baseline: independent A* for every region/query */
    for (int ri = 0; ri < nreg; ri++) {
        Region* r = &R[ri];
        for (int qi = 0; qi < r->qn; qi++) {
            long e, w;
            Bstatus[ri][qi] = astar(&r->m, r->q[qi][0], r->q[qi][1],
                                    r->q[qi][2], r->q[qi][3],
                                    Bpath[ri][qi], &Bn[ri][qi], &e, &w);
            base_exp += e; base_work += w;
            total_q++;
        }
    }

    /* reuse: one A* per pair (on REP), mirror-transform into partner */
    for (int ri = 0; ri < nreg; ri++) {
        Region* r = &R[ri];
        if (!r->rep) continue;
        Region* p = find_region(r->pair);
        if (!p) continue;
        int pi = (int)(p - R);
        reuse_work += W * H;             /* criterion scan, once per pair */
        for (int qi = 0; qi < r->qn; qi++) {
            long e, w;
            int pp[MAXN], pn = 0;
            int st = astar(&r->m, r->q[qi][0], r->q[qi][1],
                           r->q[qi][2], r->q[qi][3], pp, &pn, &e, &w);
            reuse_exp += e; reuse_work += w;

            int served[MAXN], sn = 0, sstatus = st;
            if (st == ASTAR_OK) {
                memcpy(served, pp, sizeof(int) * pn);
                mirror_path(served, pn);
                sn = pn;
                reuse_work += pn;
            }

            /* representative compares to its own baseline */
            if (Bstatus[ri][qi] == st) {
                int di, ca, cb;
                if (st != ASTAR_OK ||
                    !first_path_diff(pp, pn, Bpath[ri][qi], Bn[ri][qi],
                                     &di, &ca, &cb))
                    agree++;
            }
            /* partner: transformed path vs its independent baseline.
             * total_q already counts every region/query from baseline. */
            if (qi < p->qn) {
                int ok = 0, di = -1, ca = -1, cb = -1;
                if (Bstatus[pi][qi] == sstatus) {
                    if (sstatus != ASTAR_OK) ok = 1;
                    else ok = !first_path_diff(served, sn,
                                               Bpath[pi][qi], Bn[pi][qi],
                                               &di, &ca, &cb);
                }
                if (ok) {
                    agree++;
                } else if (!diff_set) {
                    diff_set = 1;
                    snprintf(diff_region, sizeof(diff_region), "%s", p->id);
                    diff_qi = qi; diff_idx = di; diff_ca = ca; diff_cb = cb;
                }
            }
        }
    }

    double eta_formula = (double)pairs / nreg;
    double eta_calls = 0.5; /* every tier here is fully paired */
    double save_exp = 1.0 - (double)reuse_exp / base_exp;
    double save_work = 1.0 - (double)reuse_work / base_work;
    int pmin = (int)ceil(0.30 * nreg);

    printf("[tier R=%2d] P=%d eta=P/R=%.1f%% P_min(30%%)=%d unpaired=%d\n",
           nreg, pairs, 100.0 * eta_formula, pmin, nreg - 2 * pairs);
    printf("          A* calls %d -> %d (%.1f%% saved)\n",
           total_q, total_q / 2, 100.0 * eta_calls);
    printf("          expanded %ld -> %ld (%.2f%% saved)\n",
           base_exp, reuse_exp, 100.0 * save_exp);
    printf("          work ops %ld -> %ld incl criterion+transform "
           "(%.2f%% saved)\n", base_work, reuse_work, 100.0 * save_work);
    printf("          agreement %d/%d = %.2f%%\n",
           agree, total_q, 100.0 * agree / total_q);
    if (diff_set) {
        Region* d = find_region(diff_region);
        int cax = diff_ca >= 0 ? diff_ca % W : -1;
        int cay = diff_ca >= 0 ? diff_ca / W : -1;
        int cbx = diff_cb >= 0 ? diff_cb % W : -1;
        int cby = diff_cb >= 0 ? diff_cb / W : -1;
        printf("          FIRST_DIFF region=%s query=%d index=%d "
               "local_reused=(%d,%d) local_baseline=(%d,%d) "
               "world_reused=(%d,%d) world_baseline=(%d,%d)\n",
               diff_region, diff_qi, diff_idx, cax, cay, cbx, cby,
               d ? d->ox + cax : -1, d ? d->oy + cay : -1,
               d ? d->ox + cbx : -1, d ? d->oy + cby : -1);
    } else {
        printf("          first_diff=null\n");
    }

    char nm[64];
    snprintf(nm, sizeof(nm), "tier%d_agreement100", nreg);
    check(nm, agree == total_q);
    snprintf(nm, sizeof(nm), "tier%d_savings30", nreg);
    check(nm, save_exp >= 0.30 && save_work >= 0.30);
    printf("\n");
}

/* ---------------- negative control -------------------------------------- */

static void negative_control(void) {
    Region* ne = find_region("NE");
    Region* nw = find_region("NW");
    Map bad = ne->m;
    int mx = -1, my = -1, got = 0;
    int fdi = -1, fca = -1, fcb = -1, fqi = -1;

    /* same deterministic search as repro_symmetry.py: open first blocked
     * interior cell that yields a real coordinate divergence */
    for (int y = 1; y < H - 1 && !got; y++) {
        for (int x = 1; x < W - 1 && !got; x++) {
            if (!bad.blocked[idx(x, y)]) continue;
            Map trial = bad;
            trial.blocked[idx(x, y)] = 0;
            for (int qi = 0; qi < nw->qn && !got; qi++) {
                long e, w;
                int ap[MAXN], an = 0;
                int stA = astar(&nw->m, nw->q[qi][0], nw->q[qi][1],
                                nw->q[qi][2], nw->q[qi][3],
                                ap, &an, &e, &w);
                int msx = W - 1 - nw->q[qi][0];
                int msy = nw->q[qi][1];
                int mgx = W - 1 - nw->q[qi][2];
                int mgy = nw->q[qi][3];
                int bp[MAXN], bn = 0;
                int stB = astar(&trial, msx, msy, mgx, mgy,
                                bp, &bn, &e, &w);
                if (stA != ASTAR_OK || stB != ASTAR_OK) continue;
                int served[MAXN];
                memcpy(served, ap, sizeof(int) * an);
                mirror_path(served, an);
                int di, ca, cb;
                if (first_path_diff(served, an, bp, bn, &di, &ca, &cb)) {
                    mx = x; my = y; got = 1;
                    fqi = qi; fdi = di; fca = ca; fcb = cb;
                }
            }
        }
    }

    Map trial = ne->m;
    if (got) trial.blocked[idx(mx, my)] = 0;
    int d = mismatch_count(&nw->m, &trial);
    int ax = -1, ay = -1, bx = -1, by = -1;
    first_mismatch(&nw->m, &trial, &ax, &ay, &bx, &by);

    printf("[negative-control] mutation region=NE local=(%d,%d) world=(%d,%d)\n",
           mx, my, 16 + mx, my);
    printf("  criterion D=%d strict_gate_rejects=%d\n", d, d != 0);
    printf("  first symmetry mismatch pair=NW~NE local_NW=(%d,%d) "
           "local_NE=(%d,%d) world_NE=(%d,%d)\n",
           ax, ay, bx, by, 16 + bx, by);
    if (got) {
        int cax = fca % W, cay = fca / W;
        int cbx = fcb % W, cby = fcb / W;
        printf("  FORCED_PATH_DIFF region=NE query=%d index=%d "
               "local_reused=(%d,%d) local_baseline=(%d,%d) "
               "world_reused=(%d,%d) world_baseline=(%d,%d)\n",
               fqi, fdi, cax, cay, cbx, cby,
               16 + cax, cay, 16 + cbx, cby);
    }
    check("neg_gate_rejects_D1", d == 1);
    check("neg_path_diff_found", got);
    printf("\n");
}

int main(int argc, char** argv) {
    char exe_dir[300] = ".";
    if (argc > 0 && argv[0]) {
        strncpy(exe_dir, argv[0], sizeof(exe_dir) - 1);
        char* slash = strrchr(exe_dir, '/');
        if (slash) *slash = '\0';
        else strcpy(exe_dir, ".");
    }
    char p_exe[360], p_exe_sym[380];
    snprintf(p_exe, sizeof(p_exe), "%s/quadrants.navmap", exe_dir);
    snprintf(p_exe_sym, sizeof(p_exe_sym),
             "%s/../symmetry/quadrants.navmap", exe_dir);

    const char* paths[6];
    int npath = 0;
    if (argc > 1) paths[npath++] = argv[1];
    paths[npath++] = "quadrants.navmap";
    paths[npath++] = "symmetry/quadrants.navmap";
    paths[npath++] = p_exe;
    paths[npath++] = p_exe_sym;
    int loaded = 0;
    for (int i = 0; i < npath && !loaded; i++)
        loaded = load_navmap(paths[i]);
    if (!loaded) {
        fprintf(stderr, "cannot load quadrants.navmap\n");
        return 2;
    }

    printf("symmetry self-test: %dx%d cells/region, loaded %d regions\n\n",
           W, H, NR);

    check("criterion_NW_NE_D0", mismatch_count(&R[0].m, &R[1].m) == 0);
    check("criterion_SW_SE_D0", mismatch_count(&R[2].m, &R[3].m) == 0);
    check("criterion_NW_SW_nonzero", mismatch_count(&R[0].m, &R[2].m) > 0);

    run_tier(4);
    run_tier(8);
    run_tier(16);
    negative_control();

    /* reverse formula table for R=4 and the 30% floor */
    check("formula_4_2_is_half",
          fabs((double)2 / 4 - 0.5) < 1e-9 && (int)ceil(0.30 * 4) == 2);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
