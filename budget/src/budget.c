
/* budget: A* node-expansion budget on a 512x512 grid and degradation policy.
 *
 * Three obstacle tiers (sparse random / dense random / perfect maze) are
 * searched with a binary-heap 4-neighbour A*.  For every query we record the
 * full-A* expansion count and time, a windowed (radius-limited) A* and a
 * coarse block-graph A* as degradation candidates, plus path error against
 * the full-A* reference.  A per-node cost measured on this machine is turned
 * into a 1 ms expansion cap; queries over the cap are re-run through the
 * chosen degradation and must finish inside 1 ms.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <limits.h>
#include <math.h>
#include <sys/stat.h>

#define L 512
#define N (L * L)
#define K 200                 /* queries per tier                                    */
#define BUDGET_NS 1000000L    /* 1 ms per frame                                      */
#define RESERVE_NUM 8         /* keep 20% of the frame for game logic                */
#define RESERVE_DEN 10
#define RADII_N 4
static const int RADII[RADII_N] = {48, 96, 192, 256};
#define BLOCKS 2
static const int BS[BLOCKS] = {8, 16};

/* ---------------- RNG (xorshift32, fixed seeds => reproducible) ---------------- */
static uint32_t rng_state;
static uint32_t xrnd(void) {
    uint32_t x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}
static int pick_r(int n) { return (int)(xrnd() % (uint32_t)n); }

/* ------------------------------- map generation -------------------------------- */
static int id(int x, int y) { return y * L + x; }

static void gen_random(unsigned char* blk, double rho, uint32_t seed) {
    rng_state = seed;
    uint32_t thresh = (uint32_t)(rho * 4294967296.0);
    for (int i = 0; i < N; i++) blk[i] = xrnd() < thresh ? 1 : 0;
}

/* iterative recursive-backtracker maze; free cells on even coordinates */
static void gen_maze(unsigned char* blk, uint32_t seed) {
    rng_state = seed;
    memset(blk, 1, N);
    enum { MC = L / 2 };              /* 256 maze cells per axis */
    static unsigned char vis[MC * MC];
    static int stk[MC * MC];
    memset(vis, 0, sizeof(vis));
    int sp = 0;
    stk[sp++] = 0;
    vis[0] = 1;
    blk[0] = 0;
    while (sp > 0) {
        int cur = stk[sp - 1];
        int cx = cur % MC, cy = cur / MC;
        int opts[4], no = 0;
        const int ox[4] = {1, -1, 0, 0}, oy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + ox[d], ny = cy + oy[d];
            if (nx >= 0 && ny >= 0 && nx < MC && ny < MC && !vis[ny * MC + nx])
                opts[no++] = d;
        }
        if (no == 0) { sp--; continue; }
        int d = opts[pick_r(no)];
        int nx = cx + ox[d], ny = cy + oy[d];
        vis[ny * MC + nx] = 1;
        blk[id(2 * cx + ox[d], 2 * cy + oy[d])] = 0;   /* knock down the wall */
        blk[id(2 * nx, 2 * ny)] = 0;                    /* carve the new cell  */
        stk[sp++] = ny * MC + nx;
    }
}

/* ------------------------- flood fill / largest component ----------------------- */
static int largest_component(const unsigned char* blk, int* comp, int* pool) {
    memset(comp, -1, N * sizeof(int));
    int best = -1, best_size = 0, cid = 0;
    int* q = malloc(N * sizeof(int));
    for (int s = 0; s < N; s++) {
        if (blk[s] || comp[s] >= 0) continue;
        int head = 0, tail = 0;
        q[tail++] = s; comp[s] = cid;
        while (head < tail) {
            int cur = q[head++];
            int cx = cur % L, cy = cur / L;
            const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
            for (int d = 0; d < 4; d++) {
                int nx = cx + dx[d], ny = cy + dy[d];
                if (nx < 0 || ny < 0 || nx >= L || ny >= L) continue;
                int ni = id(nx, ny);
                if (blk[ni] || comp[ni] >= 0) continue;
                comp[ni] = cid; q[tail++] = ni;
            }
        }
        if (tail > best_size) { best_size = tail; best = cid; }
        cid++;
    }
    int np = 0;
    for (int i = 0; i < N && np < N; i++) if (comp[i] == best) pool[np++] = i;
    free(q);
    return best_size;
}

/* ------------------------------- generic A* ------------------------------------ */
typedef struct {
    int status;      /* 0 found, 1 exhausted/cap, 2 out cap, -1 bad endpoint */
    int expanded;
    int pathlen;
    long ns;
} ARes;

typedef struct {
    int *g, *f, *parent, *pos, *heap, *closed;
    unsigned *gen;
    unsigned run;
} AWork;

static long now_ns(void) {
    struct timespec t;
    clock_gettime(CLOCK_MONOTONIC, &t);
    return (long)t.tv_sec * 1000000000L + t.tv_nsec;
}

static void heap_swap(AWork* z, int a, int b) {
    int t = z->heap[a]; z->heap[a] = z->heap[b]; z->heap[b] = t;
    z->pos[z->heap[a]] = a; z->pos[z->heap[b]] = b;
}
/* min f, break ties towards larger g (closer to goal): keeps the open-map
 * expansion band narrow instead of fanning out across the whole f contour */
static int better(AWork* z, int a, int b) {
    int na = z->heap[a], nb = z->heap[b];
    if (z->f[na] != z->f[nb]) return z->f[na] < z->f[nb];
    return z->g[na] > z->g[nb];
}
static void sift_up(AWork* z, int i) {
    while (i > 0) {
        int p = (i - 1) / 2;
        if (!better(z, i, p)) break;
        heap_swap(z, i, p); i = p;
    }
}
static void sift_down(AWork* z, int i, int nh) {
    for (;;) {
        int l = 2 * i + 1, r = l + 1, b = i;
        if (l < nh && better(z, l, b)) b = l;
        if (r < nh && better(z, r, b)) b = r;
        if (b == i) break;
        heap_swap(z, i, b); i = b;
    }
}

static ARes astar(const unsigned char* blk, int w, int h,
                  int sx, int sy, int gx, int gy,
                  int win, int cap, int* out, int outcap, AWork* z) {
    ARes r; r.status = 1; r.expanded = 0; r.pathlen = 0; r.ns = 0;
    if (sx < 0 || sy < 0 || gx < 0 || gy < 0 || sx >= w || sy >= h || gx >= w || gy >= h)
        { r.status = -1; return r; }
    int start = sy * w + sx, goal = gy * w + gx;
    if (blk[start] || blk[goal]) { r.status = -1; return r; }

    long t0 = now_ns();
    if (z->run == UINT_MAX) { memset(z->gen, 0, N * sizeof(unsigned)); z->run = 0; }
    z->run++;
    unsigned run = z->run;
    int nh = 0;
    z->gen[start] = run; z->g[start] = 0; z->closed[start] = 0;
    z->parent[start] = -1; z->f[start] = abs(gx - sx) + abs(gy - sy);
    z->heap[nh] = start; z->pos[start] = nh; nh++;

    while (nh > 0) {
        int cur = z->heap[0];
        nh--;
        if (nh > 0) { z->heap[0] = z->heap[nh]; z->pos[z->heap[0]] = 0; sift_down(z, 0, nh); }
        if (cur == goal) { r.status = 0; break; }
        z->closed[cur] = 1;
        r.expanded++;
        if (r.expanded >= cap) { r.status = 1; break; }

        int cx = cur % w, cy = cur / w;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            if (win != INT_MAX && (abs(nx - sx) > win || abs(ny - sy) > win)) continue;
            int ni = ny * w + nx;
            if (blk[ni] || (z->closed[ni] == 1 && z->gen[ni] == run)) continue;
            int ng = z->g[cur] + 1;
            if (z->gen[ni] != run) {
                z->gen[ni] = run; z->g[ni] = INT_MAX; z->closed[ni] = 0; z->parent[ni] = -1;
            }
            if (ng < z->g[ni]) {
                z->g[ni] = ng;
                z->f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                z->parent[ni] = cur;
                int existing = (z->gen[ni] == run && z->pos[ni] >= 0 &&
                                z->pos[ni] < nh && z->heap[z->pos[ni]] == ni);
                if (existing) {
                    sift_up(z, z->pos[ni]);
                } else {
                    z->pos[ni] = nh; z->heap[nh++] = ni; sift_up(z, nh - 1);
                }
            }
        }
    }
    if (r.status == 0) {
        static int rev[N];
        int n = 0, c = goal;
        while (c != -1 && n < N) { rev[n++] = c; c = z->parent[c]; }
        if (n > outcap) { r.status = 2; r.ns = now_ns() - t0; return r; }
        for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
        r.pathlen = n;
    }
    r.ns = now_ns() - t0;
    return r;
}

/* ---------------------- multi-source BFS distance field ------------------------- */
static void bfs_field(const unsigned char* blk, const int* src, int nsrc,
                      int* dist, int* q) {
    memset(dist, -1, N * sizeof(int));
    int head = 0, tail = 0;
    for (int i = 0; i < nsrc; i++) { dist[src[i]] = 0; q[tail++] = src[i]; }
    while (head < tail) {
        int cur = q[head++];
        int cx = cur % L, cy = cur / L;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || ny < 0 || nx >= L || ny >= L) continue;
            int ni = id(nx, ny);
            if (blk[ni] || dist[ni] >= 0) continue;
            dist[ni] = dist[cur] + 1; q[tail++] = ni;
        }
    }
}

/* Bresenham LOS, mirrors the smooth module */
static int los(const unsigned char* blk, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy, x = x0, y = y0;
    for (;;) {
        if (x < 0 || y < 0 || x >= L || y >= L) return 0;
        if (blk[id(x, y)]) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
}

/* ------------------------------- per-query record ------------------------------- */
typedef struct {
    int sx, sy, gx, gy, d, optlen, exp, full_ns;
    int rok[RADII_N];
    int cok[BLOCKS], cdev[BLOCKS], cover[BLOCKS], clos[BLOCKS], clen[BLOCKS];
    int over, method, dns, dexp, ddev;   /* degradation run (phase 2) */
} QRec;

static int cmp_int(const void* a, const void* b) {
    int x = *(const int*)a, y = *(const int*)b;
    return (x > y) - (x < y);
}
static int cmp_double(const void* a, const void* b) {
    double x = *(const double*)a, y = *(const double*)b;
    return (x > y) - (x < y);
}
static int pctile(int* v, int n, double p) {
    qsort(v, n, sizeof(int), cmp_int);
    int i = (int)(p * (n - 1) + 0.5);
    return v[i];
}
static double mean_int(const int* v, int n) {
    long s = 0; for (int i = 0; i < n; i++) s += v[i];
    return n ? (double)s / n : 0.0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ------------------------------- coarse helpers -------------------------------- */
/* A block is blocked only when a strict majority of its cells are blocked.
 * The all-blocked rule makes the coarse graph nearly empty at rho=0.25
 * (P(free 8x8 block) ~ 1e-8), so majority keeps the graph connected while
 * half-blocked blocks are still excluded; deviance is measured afterwards. */
static void build_coarse(const unsigned char* fine, unsigned char* coarse, int B) {
    int M = L / B;
    for (int by = 0; by < M; by++) {
        for (int bx = 0; bx < M; bx++) {
            int bl = 0, total = 0;
            for (int y = by * B; y < (by + 1) * B; y++)
                for (int x = bx * B; x < (bx + 1) * B; x++) {
                    bl += fine[id(x, y)]; total++;
                }
            coarse[by * M + bx] = (unsigned char)(bl > total / 2);
        }
    }
}

int main(void) {
    mkdir("results", 0755);
    /* static so the multi-MB arrays stay off the stack */
    static unsigned char maps[3][N], coarse[BLOCKS][N];
    static int comp[N], pool[N], ref[N], test[N];
    static int zbuf_g[N], zbuf_f[N], zbuf_par[N], zbuf_pos[N], zbuf_heap[N],
               zbuf_closed[N], dist[N], bfsq[N];
    static unsigned zbuf_gen[N];
    static AWork zw;
    zw.g = zbuf_g; zw.f = zbuf_f; zw.parent = zbuf_par; zw.pos = zbuf_pos;
    zw.heap = zbuf_heap; zw.closed = zbuf_closed; zw.gen = zbuf_gen;
    for (int i = 0; i < N; i++) zbuf_pos[i] = -1;
    static QRec recs[3][K];

    static const char* tname[3] = {"sparse_rho0.10", "dense_rho0.25", "maze"};
    static const double rho_target[3] = {0.10, 0.25, -1.0};
    const uint32_t seeds[3] = {11u, 22u, 33u};

    FILE* fq = fopen("results/queries.csv", "w");
    FILE* fs = fopen("results/summary.csv", "w");
    FILE* fo = fopen("results/over_budget.csv", "w");
    check("csv_open", fq && fs && fo);
    fprintf(fq, "tier,qid,dist,opt_len,full_exp,full_ns,"
            "r48_ok,r96_ok,r192_ok,r256_ok,"
            "c8_ok,c8_dev,c8_over_pct,c8_los_pct,"
            "c16_ok,c16_dev,c16_over_pct,c16_los_pct\n");
    fprintf(fs, "tier,rho,comp_frac,queries,exp_mean,exp_p50,exp_p95,exp_max,"
            "ns_per_node,full_p50_ns,full_p95_ns,full_max_ns,over_1ms_measured\n");
    fprintf(fo, "tier,qid,dist,full_exp,full_ns,method,degrade_exp,degrade_ns,degrade_dev\n");

    long sum_ns = 0, sum_exp = 0;
    /* per-node cost pool: only large enough searches, where clock overhead is
     * negligible; p95 of the pool is the conservative per-node cost */
    double cpool[3 * K];
    int ncpool = 0;
    int tier_rho[3];

    for (int t = 0; t < 3; t++) {
        unsigned char* blk = maps[t];
        if (rho_target[t] < 0) gen_maze(blk, seeds[t]);
        else gen_random(blk, rho_target[t], seeds[t]);
        int blocked_n = 0;
        for (int i = 0; i < N; i++) blocked_n += blk[i];
        tier_rho[t] = blocked_n;
        int comp_size = largest_component(blk, comp, pool);

        /* coarse maps, endpoint blocks force-opened per query later */
        for (int bi = 0; bi < BLOCKS; bi++) build_coarse(blk, coarse[bi], BS[bi]);

        /* stratified start/goal samples by Manhattan distance */
        static const int blo[5] = {1, 150, 400, 700, 1024};
        int per = K / 4, got = 0;
        for (int bk = 0; bk < 4 && got < K; bk++) {
            int want = per, tries = 0;
            while (want > 0 && tries++ < 200000 && got < K) {
                int a = pool[pick_r(comp_size)], b = pool[pick_r(comp_size)];
                int d = abs(a % L - b % L) + abs(a / L - b / L);
                if (d < blo[bk] || d >= blo[bk + 1]) continue;
                QRec* q = &recs[t][got];
                q->sx = a % L; q->sy = a / L; q->gx = b % L; q->gy = b / L; q->d = d;
                want--; got++;
            }
        }
        int nq = got;

        int exps[K], ftimes[K];
        for (int qi = 0; qi < nq; qi++) {
            QRec* q = &recs[t][qi];
            ARes fr = astar(blk, L, L, q->sx, q->sy, q->gx, q->gy,
                            INT_MAX, INT_MAX, ref, N, &zw);
            if (fr.status != 0) { check("full_astar_found", 0); continue; }
            q->optlen = fr.pathlen - 1;
            q->exp = fr.expanded; q->full_ns = fr.ns;
            exps[qi] = fr.expanded; ftimes[qi] = fr.ns;
            sum_ns += fr.ns; sum_exp += fr.expanded;
            if (fr.expanded >= 200) cpool[ncpool++] = (double)fr.ns / fr.expanded;

            bfs_field(blk, ref, fr.pathlen, dist, bfsq);

            /* radius-limited A*: exact while the goal lies inside the window */
            for (int ri = 0; ri < RADII_N; ri++) {
                ARes rr = astar(blk, L, L, q->sx, q->sy, q->gx, q->gy,
                                RADII[ri], INT_MAX, test, N, &zw);
                q->rok[ri] = (rr.status == 0);
            }

            /* coarse block-graph A* */
            for (int bi = 0; bi < BLOCKS; bi++) {
                int B = BS[bi], M = L / B;
                unsigned char save_s, save_g;
                int sb = (q->sy / B) * M + q->sx / B;
                int gb = (q->gy / B) * M + q->gx / B;
                save_s = coarse[bi][sb]; save_g = coarse[bi][gb];
                coarse[bi][sb] = 0; coarse[bi][gb] = 0;
                ARes cr = astar(coarse[bi], M, M,
                                q->sx / B, q->sy / B, q->gx / B, q->gy / B,
                                INT_MAX, INT_MAX, test, N, &zw);
                coarse[bi][sb] = save_s; coarse[bi][gb] = save_g;
                q->cok[bi] = (cr.status == 0);
                if (cr.status == 0) {
                    static int way[N / 4];
                    int nw = 0;
                    for (int i = 0; i < cr.pathlen; i++) {
                        int bx = test[i] % M, by = test[i] / M;
                        int wx = bx * B + B / 2, wy = by * B + B / 2;
                        if (i == 0) { wx = q->sx; wy = q->sy; }
                        if (i == cr.pathlen - 1) { wx = q->gx; wy = q->gy; }
                        way[nw++] = id(wx, wy);
                    }
                    long dev = 0, len = 0, los_ok = 0;
                    for (int i = 0; i < nw; i++) dev += dist[way[i]];
                    for (int i = 0; i + 1 < nw; i++) {
                        len += abs(way[i] % L - way[i + 1] % L) +
                               abs(way[i] / L - way[i + 1] / L);
                        if (los(blk, way[i] % L, way[i] / L,
                                way[i + 1] % L, way[i + 1] / L)) los_ok++;
                    }
                    q->cdev[bi] = (int)(dev / nw);
                    q->clen[bi] = (int)len;
                    q->cover[bi] = q->optlen > 0
                        ? (int)(((long)q->clen[bi] - q->optlen) * 100 / q->optlen) : 0;
                    q->clos[bi] = (int)(los_ok * 100 / (nw - 1));
                }
            }

            fprintf(fq, "%s,%d,%d,%d,%d,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                    tname[t], qi, q->d, q->optlen, q->exp, (long)q->full_ns,
                    q->rok[0], q->rok[1], q->rok[2], q->rok[3],
                    q->cok[0], q->cdev[0], q->cover[0], q->clos[0],
                    q->cok[1], q->cdev[1], q->cover[1], q->clos[1]);
        }

        int over1ms = 0;
        for (int i = 0; i < nq; i++) if (ftimes[i] > BUDGET_NS) over1ms++;
        double ns_node = sum_exp ? (double)sum_ns / sum_exp : 0.0;
        int emean = (int)mean_int(exps, nq);
        int ep50 = pctile(exps, nq, 0.50), ep95 = pctile(exps, nq, 0.95),
            emax = pctile(exps, nq, 1.0);
        int tp50 = pctile(ftimes, nq, 0.50), tp95 = pctile(ftimes, nq, 0.95),
            tmax = pctile(ftimes, nq, 1.0);
        printf("\n[tier %s] rho=%.4f comp_frac=%.4f queries=%d\n", tname[t],
               (double)tier_rho[t] / N, (double)comp_size / N, nq);
        printf("exp_mean=%d exp_p50=%d exp_p95=%d exp_max=%d\n", emean, ep50, ep95, emax);
        printf("full_time_ns p50=%d p95=%d max=%d over_1ms=%d\n", tp50, tp95, tmax, over1ms);
        printf("cumulative_ns_per_expansion=%.1f\n", ns_node);
        for (int ri = 0; ri < RADII_N; ri++) {
            int ok = 0;
            for (int i = 0; i < nq; i++) ok += recs[t][i].rok[ri];
            printf("radius_%d_success=%.3f\n", RADII[ri], (double)ok / nq);
        }
        for (int bi = 0; bi < BLOCKS; bi++) {
            int ok = 0; long dv = 0;
            static int devs[K];
            for (int i = 0; i < nq; i++)
                if (recs[t][i].cok[bi]) { ok++; dv += recs[t][i].cdev[bi]; devs[ok - 1] = recs[t][i].cdev[bi]; }
            int med = ok ? pctile(devs, ok, 0.50) : 0;
            printf("coarse_B%d_success=%.3f mean_dev_cells=%.2f median_dev_cells=%d\n",
                   BS[bi], (double)ok / nq, ok ? (double)dv / ok : 0.0, med);
        }
        /* E vs d: corridor width E/d per distance bucket */
        printf("expansion_vs_distance: bucket d_mean exp_mean E_over_d\n");
        for (int bk = 0; bk < 4; bk++) {
            long sd = 0, se = 0; int nb = 0;
            for (int i = 0; i < nq; i++) {
                QRec* q = &recs[t][i];
                if (q->d >= blo[bk] && q->d < blo[bk + 1]) { sd += q->d; se += q->exp; nb++; }
            }
            if (nb) printf("  [%d,%d) %.1f %.1f %.2f\n", blo[bk], blo[bk + 1],
                           (double)sd / nb, (double)se / nb, (double)se / sd);
        }
        fprintf(fs, "%s,%.4f,%.4f,%d,%d,%d,%d,%d,%.1f,%d,%d,%d,%d\n",
                tname[t], (double)tier_rho[t] / N, (double)comp_size / N, nq,
                emean, ep50, ep95, emax, ns_node, tp50, tp95, tmax, over1ms);
    }
    fclose(fq); fclose(fs);

    /* ---------- phase 2: derive the 1 ms cap and apply the policy ---------- */
    /* The pooled mean ns/node underestimates the dense tier; invert with the
     * p95 per-node cost measured on sizeable searches so a cap-bounded search
     * fits even on the slowest map. */
    double c_mean = sum_exp ? (double)sum_ns / sum_exp : 1.0;
    qsort(cpool, ncpool, sizeof(double), cmp_double);
    double c_safe = cpool[(int)(0.95 * (ncpool - 1) + 0.5)];
    double budget_us = (double)BUDGET_NS * RESERVE_NUM / RESERVE_DEN / 1000.0;
    int cap_mean = (int)((double)BUDGET_NS * RESERVE_NUM / RESERVE_DEN / c_mean);
    int cap = (int)((double)BUDGET_NS * RESERVE_NUM / RESERVE_DEN / c_safe);
    int rsel = (int)((sqrt((double)cap) - 1.0) / 2.0);  /* (2r+1)^2 <= cap */
    printf("\n[budget] ns_per_node mean=%.1f p95=%.1f (n=%d) cap_mean_based=%d "
           "cap_safe=%d (%.0f us reserved), policy_radius=%d\n",
           c_mean, c_safe, ncpool, cap_mean, cap, budget_us, rsel);

    int over_total = 0, n_window = 0, n_coarse = 0, n_partial = 0, late = 0;
    for (int t = 0; t < 3; t++) {
        unsigned char* blk = maps[t];
        int shown = 0, over_tier = 0;
        for (int qi = 0; qi < K; qi++) {
            QRec* q = &recs[t][qi];
            if (q->exp == 0) continue;
            if (q->exp <= cap) continue;
            over_tier++; over_total++;
            int cheb = abs(q->gx - q->sx) > abs(q->gy - q->sy)
                       ? abs(q->gx - q->sx) : abs(q->gy - q->sy);
            ARes dr;
            const char* mname;
            if (cheb <= rsel) {
                /* windowed fine A*: exact, zero error, goal inside the window */
                q->method = 1; mname = "window";
                dr = astar(blk, L, L, q->sx, q->sy, q->gx, q->gy,
                           rsel, cap, test, N, &zw);
                q->ddev = 0;
            } else {
                /* coarse block graph; B=16 only if B=8's whole graph exceeds
                 * the cap, otherwise the 4096-node graph always fits */
                int bi = (L / BS[0]) * (L / BS[0]) > cap ? 1 : 0;
                int B = BS[bi], Mc = L / B;
                int sb = (q->sy / B) * Mc + q->sx / B;
                int gb = (q->gy / B) * Mc + q->gx / B;
                unsigned char ss = coarse[bi][sb], sg = coarse[bi][gb];
                coarse[bi][sb] = 0; coarse[bi][gb] = 0;
                dr = astar(coarse[bi], Mc, Mc,
                           q->sx / B, q->sy / B, q->gx / B, q->gy / B,
                           INT_MAX, cap, test, N, &zw);
                coarse[bi][sb] = ss; coarse[bi][gb] = sg;
                if (dr.status == 0) {
                    q->method = 20 + B; mname = "coarse";
                    q->ddev = q->cdev[bi];
                } else {
                    /* coarse graph disconnects (maze corridors): emit a
                     * cap-bounded partial fine path towards the goal */
                    q->method = 3; mname = "partial";
                    dr = astar(blk, L, L, q->sx, q->sy, q->gx, q->gy,
                               rsel, cap, test, N, &zw);
                    q->ddev = dr.status == 0 ? 0 : -1;
                }
            }
            if (q->method == 1) n_window++;
            else if (q->method == 3) n_partial++;
            else n_coarse++;
            q->dns = (int)dr.ns; q->dexp = dr.expanded;
            if (dr.ns >= BUDGET_NS) late++;
            if (shown++ < 10)
                printf("over_budget tier=%s q=%d exp=%d full_us=%.1f -> method=%s "
                       "dexp=%d d_us=%.2f dev=%d%s\n",
                       tname[t], qi, q->exp, q->full_ns / 1000.0, mname,
                       q->dexp, dr.ns / 1000.0, q->ddev,
                       dr.ns >= BUDGET_NS ? " LATE" : "");
            fprintf(fo, "%s,%d,%d,%d,%ld,%s,%d,%ld,%d\n", tname[t], qi, q->d,
                    q->exp, (long)q->full_ns, mname,
                    q->dexp, (long)dr.ns, q->ddev);
        }
        printf("tier=%s over_budget_queries=%d\n", tname[t], over_tier);
    }
    fclose(fo);
    printf("over_budget_total=%d window=%d coarse=%d partial=%d degraded_over_1ms=%d\n",
           over_total, n_window, n_coarse, n_partial, late);
    check("all_degraded_under_1ms", late == 0);
    check("cap_positive", cap > 0);

    /* ------------------------------- self-tests ------------------------------- */
    unsigned char tiny[64];
    memset(tiny, 0, sizeof(tiny));
    static AWork tw;
    static int tg[64], tf[64], tp[64], tpos[64], th[64], tc[64], tpath[64];
    static unsigned tgen[64];
    tw.g = tg; tw.f = tf; tw.parent = tp; tw.pos = tpos; tw.heap = th;
    tw.closed = tc; tw.gen = tgen; tw.run = 0;
    for (int i = 0; i < 64; i++) tpos[i] = -1;
    ARes a = astar(tiny, 8, 8, 0, 0, 7, 7, INT_MAX, INT_MAX, tpath, 64, &tw);
    check("empty_optimal_len", a.status == 0 && a.pathlen == 15);
    check("empty_expansions_band", a.expanded <= 50);   /* narrow tie band, not 64 */
    for (int y = 0; y < 8; y++) tiny[y * 8 + 3] = 1;
    a = astar(tiny, 8, 8, 0, 0, 7, 0, INT_MAX, INT_MAX, tpath, 64, &tw);
    check("unreachable_status", a.status == 1);
    memset(tiny, 0, sizeof(tiny));
    a = astar(tiny, 8, 8, 0, 0, 7, 7, INT_MAX, 5, tpath, 64, &tw);
    check("cap_honored", a.status == 1 && a.expanded <= 5);
    a = astar(tiny, 8, 8, 0, 0, 7, 7, 3, INT_MAX, tpath, 64, &tw);
    check("window_blocks_far_goal", a.status == 1);
    a = astar(tiny, 8, 8, 0, 0, 2, 2, 3, INT_MAX, tpath, 64, &tw);
    check("window_allows_near_goal", a.status == 0 && a.pathlen == 5);

    /* BFS field: reference cells have distance 0, a neighbour has distance 1 */
    int src[2] = {id(1, 1), id(2, 1)};
    static int dd[N], dq[N];
    static unsigned char fmap[N];
    bfs_field(fmap, src, 2, dd, dq);
    check("bfs_source_zero", dd[id(1, 1)] == 0 && dd[id(2, 1)] == 0);
    check("bfs_neighbour_one", dd[id(1, 0)] == 1 && dd[id(3, 1)] == 1);

    /* coarse endpoints map back onto the requested start/goal cells;
     * the 64x64 coarse graph needs the full-size work buffers */
    QRec* q0 = &recs[0][0];
    int B = 8, M = L / B;
    int sb = (q0->sy / B) * M + q0->sx / B, gb = (q0->gy / B) * M + q0->gx / B;
    unsigned char ss = coarse[0][sb], sg = coarse[0][gb];
    coarse[0][sb] = 0; coarse[0][gb] = 0;
    a = astar(coarse[0], M, M, q0->sx / B, q0->sy / B, q0->gx / B, q0->gy / B,
              INT_MAX, INT_MAX, test, N, &zw);
    coarse[0][sb] = ss; coarse[0][gb] = sg;
    check("coarse_endpoints", a.status == 0 &&
          test[0] == sb && test[a.pathlen - 1] == gb);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
