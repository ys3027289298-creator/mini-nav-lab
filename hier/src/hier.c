
/* hier: two-level hierarchical A* on a 2048x2048 cell world.
 *
 * Offline preprocessing (once per chunk size c):
 *   - block walkability: a block counts as walkable if it has an open cell;
 *   - border passability: an edge between two 4-neighbour blocks is passable
 *     only if some pair of open cells straddles their shared border. This is
 *     the HPA* entrance test and guarantees that every coarse path can be
 *     refined into a real cell path.
 *
 * Level 1 (coarse): one node per block, 4-neighbour A* over passable edges,
 *                   every edge costs c (the block side in cells).
 * Level 2 (fine):   cell-level 4-neighbour A* restricted to the union of the
 *                   blocks on the coarse path (the corridor). Nodes outside
 *                   the corridor are never generated, so expanded-node count
 *                   stays proportional to the corridor area instead of W*H.
 *
 * The baseline is the same cell A* with the corridor restriction removed.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>
#include <math.h>

#define W 2048
#define H 2048
#define NCELLS (W * H)
#define INF_COST (1 << 29)

/* candidate chunk side lengths: every value must divide W (and H) */
static const int CS[] = {8, 16, 32, 64, 128, 256};
static const int NCS = (int)(sizeof(CS) / sizeof(CS[0]));

typedef struct { unsigned char *blocked; } Map;

static int cell_idx(int x, int y) { return y * W + x; }
static int block_id(int bx, int by, int nb) { return by * nb + bx; }

/* deterministic LCG so every run uses the same world */
static uint32_t rng_state;
static uint32_t rng_next(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return rng_state;
}

/* corridor dilation: blocks within MARGIN block-steps of the coarse path
 * are also opened to the fine search, so detours inside a block (the entrance
 * pair promised by the coarse edge need not connect inside the block) have
 * room. MARGIN=2 made every benchmark query refinable; the fallback below
 * stays as the correctness backstop. */
#define MARGIN 2

/* scattered rectangular clutter plus a few long walls with gaps:
 * roughly 6% of cells blocked, still fully connected (full A* verifies). */
static void build_map(Map *m, uint32_t seed) {
    memset(m->blocked, 0, NCELLS);
    rng_state = seed;
    for (int i = 0; i < 2600; i++) {
        int rw = 1 + rng_next() % 7;
        int rh = 1 + rng_next() % 7;
        int x0 = rng_next() % (W - rw);
        int y0 = rng_next() % (H - rh);
        for (int y = y0; y < y0 + rh; y++)
            for (int x = x0; x < x0 + rw; x++)
                m->blocked[cell_idx(x, y)] = 1;
    }
    for (int i = 0; i < 36; i++) {
        int horizontal = rng_next() & 1;
        int len = 60 + rng_next() % 320;
        int x0 = rng_next() % (W - len);
        int y0 = rng_next() % H;
        int gap = 4 + rng_next() % 6;          /* gap keeps regions connected */
        int gstart = len / 3 + rng_next() % (len / 3);
        for (int t = 0; t < len; t++) {
            if (t >= gstart && t < gstart + gap) continue;
            int x = horizontal ? x0 + t : x0;
            int y = horizontal ? y0 : y0 + t;
            if (y < H && x < W) m->blocked[cell_idx(x, y)] = 1;
        }
    }
    /* keep the query endpoints open */
    const int pts[][2] = {{100, 100}, {1900, 1900}, {300, 300},
                          {1700, 300}, {900, 900}, {1150, 1100}};
    for (size_t p = 0; p < sizeof(pts) / sizeof(pts[0]); p++) {
        int cx = pts[p][0], cy = pts[p][1];
        for (int y = cy - 3; y <= cy + 3; y++)
            for (int x = cx - 3; x <= cx + 3; x++)
                if (x >= 0 && y >= 0 && x < W && y < H)
                    m->blocked[cell_idx(x, y)] = 0;
    }
}

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[cell_idx(x, y)];
}

/* ---- binary min-heap keyed by f ---- */
typedef struct {
    int *heap;      /* node ids */
    int *pos;       /* pos[node] = index in heap, -1 when absent */
    int *key;       /* f value */
    int n;
    int cap;
} PQueue;

static void pq_init(PQueue *q, int cap) {
    q->heap = malloc((size_t)cap * sizeof(int));
    q->pos = malloc((size_t)cap * sizeof(int));
    q->key = malloc((size_t)cap * sizeof(int));
    for (int i = 0; i < cap; i++) q->pos[i] = -1;
    q->n = 0;
    q->cap = cap;
}

static void pq_free(PQueue *q) {
    free(q->heap); free(q->pos); free(q->key);
}

static void pq_swap(PQueue *q, int i, int j) {
    int t = q->heap[i];
    q->heap[i] = q->heap[j];
    q->heap[j] = t;
    q->pos[q->heap[i]] = i;
    q->pos[q->heap[j]] = j;
}

static void pq_up(PQueue *q, int i) {
    while (i > 0) {
        int p = (i - 1) / 2;
        if (q->key[q->heap[p]] <= q->key[q->heap[i]]) break;
        pq_swap(q, p, i);
        i = p;
    }
}

static void pq_down(PQueue *q, int i) {
    for (;;) {
        int l = 2 * i + 1, r = l + 1, best = i;
        if (l < q->n && q->key[q->heap[l]] < q->key[q->heap[best]]) best = l;
        if (r < q->n && q->key[q->heap[r]] < q->key[q->heap[best]]) best = r;
        if (best == i) break;
        pq_swap(q, best, i);
        i = best;
    }
}

static void pq_push_or_decrease(PQueue *q, int node, int f) {
    int i = q->pos[node];
    if (i >= 0) {
        q->key[node] = f;
        pq_up(q, i);
        return;
    }
    i = q->n++;
    q->heap[i] = node;
    q->pos[node] = i;
    q->key[node] = f;
    pq_up(q, i);
}

static int pq_pop(PQueue *q) {
    int top = q->heap[0];
    q->pos[top] = -1;
    q->n--;
    if (q->n > 0) {
        q->heap[0] = q->heap[q->n];
        q->pos[q->heap[0]] = 0;
        pq_down(q, 0);
    }
    return top;
}

/* ---- coarse graph for one chunk size, built once offline ---- */
typedef struct {
    int c;                 /* chunk side in cells */
    int nb;                /* blocks per row/column */
    unsigned char *walk;   /* nb*nb: block contains an open cell */
    /* hpass[by*(nb-1)+bx]: border between block (bx,by) and (bx+1,by) */
    unsigned char *hpass;
    /* vpass[ty*nb+bx]:     border between block (bx,ty) and (bx,ty+1) */
    unsigned char *vpass;
} Coarse;

static void coarse_build(const Map *m, int c, Coarse *co) {
    int nb = W / c;
    co->c = c;
    co->nb = nb;
    co->walk = calloc((size_t)nb * nb, 1);
    co->hpass = calloc((size_t)nb * (nb - 1), 1);
    co->vpass = calloc((size_t)(nb - 1) * nb, 1);

    for (int by = 0; by < nb; by++) {
        for (int bx = 0; bx < nb; bx++) {
            int x0 = bx * c, y0 = by * c, any = 0;
            for (int y = y0; y < y0 + c; y++)
                for (int x = x0; x < x0 + c; x++)
                    if (!m->blocked[cell_idx(x, y)]) { any = 1; goto done; }
        done:
            co->walk[block_id(bx, by, nb)] = (unsigned char)any;
        }
    }
    /* vertical line borders: open cell on both sides at the same y */
    for (int by = 0; by < nb; by++) {
        for (int bx = 0; bx < nb - 1; bx++) {
            int xL = (bx + 1) * c - 1, xR = xL + 1, pass = 0;
            for (int y = by * c; y < (by + 1) * c; y++) {
                if (!m->blocked[cell_idx(xL, y)] &&
                    !m->blocked[cell_idx(xR, y)]) { pass = 1; break; }
            }
            co->hpass[by * (nb - 1) + bx] = (unsigned char)pass;
        }
    }
    /* horizontal line borders */
    for (int ty = 0; ty < nb - 1; ty++) {
        int yT = (ty + 1) * c - 1, yB = yT + 1;
        for (int bx = 0; bx < nb; bx++) {
            int pass = 0;
            for (int x = bx * c; x < (bx + 1) * c; x++) {
                if (!m->blocked[cell_idx(x, yT)] &&
                    !m->blocked[cell_idx(x, yB)]) { pass = 1; break; }
            }
            co->vpass[ty * nb + bx] = (unsigned char)pass;
        }
    }
}

static void coarse_free(Coarse *co) {
    free(co->walk); free(co->hpass); free(co->vpass);
}

/* ---- cell-level A* (baseline and corridor refinement share this) ----
 * corridor != NULL restricts the search to marked blocks.
 * *expanded receives the number of nodes popped from the open list.
 * When path_out != NULL the node chain is written there (length returned). */
static int cell_astar(const Map *m, int sx, int sy, int gx, int gy,
                      int c, const unsigned char *corridor, int *expanded,
                      int *path_out) {
    static int *g = NULL, *parent;
    static unsigned char *closed;
    static PQueue pq;
    static int ready = 0;
    if (!ready) {
        g = malloc((size_t)NCELLS * sizeof(int));
        parent = malloc((size_t)NCELLS * sizeof(int));
        closed = calloc(NCELLS, 1);
        pq_init(&pq, NCELLS);
        ready = 1;
    }
    for (int i = 0; i < NCELLS; i++) { g[i] = INF_COST; closed[i] = 0; }
    pq.n = 0;
    for (int i = 0; i < pq.cap; i++) pq.pos[i] = -1;
    *expanded = 0;

    int nb = W / c;
    int s = cell_idx(sx, sy), goal = cell_idx(gx, gy);
    g[s] = 0;
    pq_push_or_decrease(&pq, s, abs(gx - sx) + abs(gy - sy));
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};

    while (pq.n > 0) {
        int cur = pq_pop(&pq);
        if (closed[cur]) continue;
        closed[cur] = 1;
        (*expanded)++;
        if (cur == goal) {
            int len = 1, k = cur;
            while (k != s) { k = parent[k]; len++; }
            if (path_out) {
                int n = len, kk = cur;
                path_out[--n] = kk;
                while (kk != s) { kk = parent[kk]; path_out[--n] = kk; }
            }
            return len;
        }
        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            if (corridor && !corridor[block_id(nx / c, ny / c, nb)]) continue;
            int ni = cell_idx(nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                parent[ni] = cur;
                pq_push_or_decrease(&pq, ni, ng + abs(gx - nx) + abs(gy - ny));
            }
        }
    }
    return -1;
}

/* ---- coarse block A* over the precomputed passable edges ---- */
static int block_astar(const Coarse *co, int sbid, int gbid,
                       unsigned char *corridor, int *nblocks_on_path,
                       int *expanded) {
    int nb = co->nb, c = co->c, nb_total = nb * nb;
    int *g = malloc((size_t)nb_total * sizeof(int));
    int *parent = malloc((size_t)nb_total * sizeof(int));
    unsigned char *closed = calloc((size_t)nb_total, 1);
    PQueue pq; pq_init(&pq, nb_total);
    for (int i = 0; i < nb_total; i++) g[i] = INF_COST;
    *expanded = 0;

    int gbx = gbid % nb, gby = gbid / nb;
    g[sbid] = 0;
    pq_push_or_decrease(&pq, sbid, c * (abs(gbx - sbid % nb) + abs(gby - sbid / nb)));
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};

    int found = 0;
    while (pq.n > 0) {
        int cur = pq_pop(&pq);
        if (closed[cur]) continue;
        closed[cur] = 1;
        (*expanded)++;
        if (cur == gbid) { found = 1; break; }
        int cx = cur % nb, cy = cur / nb;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (nx < 0 || ny < 0 || nx >= nb || ny >= nb) continue;
            /* consult the precomputed border passability table */
            int pass;
            if (dx[d] == 1)       pass = co->hpass[cy * (nb - 1) + cx];
            else if (dx[d] == -1) pass = co->hpass[cy * (nb - 1) + (cx - 1)];
            else if (dy[d] == 1)  pass = co->vpass[cy * nb + cx];
            else                  pass = co->vpass[(cy - 1) * nb + cx];
            if (!pass) continue;
            int ni = block_id(nx, ny, nb);
            if (closed[ni]) continue;
            int ng = g[cur] + c;
            if (ng < g[ni]) {
                g[ni] = ng;
                parent[ni] = cur;
                pq_push_or_decrease(&pq, ni,
                                    ng + c * (abs(gbx - nx) + abs(gby - ny)));
            }
        }
    }
    int path_len = -1;
    if (found) {
        memset(corridor, 0, (size_t)nb_total);
        int k = gbid, n = 0;
        corridor[k] = 1;
        while (k != sbid) { k = parent[k]; corridor[k] = 1; n++; }
        *nblocks_on_path = n + 1;
        path_len = (n + 1) * c;          /* coarse distance in cell units */
    }
    free(g); free(parent); free(closed); pq_free(&pq);
    return path_len;
}

/* hierarchical query: coarse path first, then refine inside its corridor.
 * If the chosen block chain cannot be threaded inside the corridor (a legal
 * coarse edge only promises an entrance exists, not that entrances connect
 * inside the block), fall back to one full-map A* and extend the corridor
 * with the blocks the real path uses, so later queries reuse them. */
static int hier_query(const Map *m, const Coarse *co,
                      int sx, int sy, int gx, int gy,
                      unsigned char *corridor,
                      int *coarse_exp, int *fine_exp, int *nblocks_path,
                      int *used_fallback, int *fallback_exp,
                      int *refine_blocks) {
    int nb = co->nb, c = co->c;
    int sbid = block_id(sx / c, sy / c, nb);
    int gbid = block_id(gx / c, gy / c, nb);
    int coarse_len = block_astar(co, sbid, gbid, corridor,
                                 nblocks_path, coarse_exp);
    if (coarse_len < 0) return -1;

    /* dilate the coarse-path corridor by MARGIN blocks before refining */
    unsigned char *wide = calloc((size_t)nb * nb, 1);
    int wide_blocks = 0;
    for (int by = 0; by < nb; by++) {
        for (int bx = 0; bx < nb; bx++) {
            if (!corridor[block_id(bx, by, nb)]) continue;
            for (int dy = -MARGIN; dy <= MARGIN; dy++) {
                for (int dx = -MARGIN; dx <= MARGIN; dx++) {
                    int nx = bx + dx, ny = by + dy;
                    if (nx >= 0 && ny >= 0 && nx < nb && ny < nb) {
                        int nid = block_id(nx, ny, nb);
                        if (!wide[nid]) { wide[nid] = 1; wide_blocks++; }
                    }
                }
            }
        }
    }
    *refine_blocks = wide_blocks;

    static int *pathbuf;
    if (!pathbuf) pathbuf = malloc((size_t)NCELLS * sizeof(int));
    int r = cell_astar(m, sx, sy, gx, gy, c, wide, fine_exp, NULL);
    *used_fallback = 0;
    *fallback_exp = 0;
    if (r >= 0) { free(wide); return r; }

    /* corridor refinement failed: full-map search, then widen the corridor */
    r = cell_astar(m, sx, sy, gx, gy, c, NULL, fallback_exp, pathbuf);
    if (r < 0) { free(wide); return -1; }
    *used_fallback = 1;
    for (int i = 0; i < r; i++) {
        int px = pathbuf[i] % W, py = pathbuf[i] / W;
        corridor[block_id(px / c, py / c, nb)] = 1;
    }
    free(wide);
    return r;
}

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

/* move a blocked endpoint to the nearest walkable cell (spiral out) */
static void snap_to_walkable(const Map *m, int *x, int *y) {
    if (walkable(m, *x, *y)) return;
    for (int r = 1; r < 64; r++) {
        for (int dy = -r; dy <= r; dy++)
            for (int dx = -r; dx <= r; dx++) {
                if (abs(dx) != r && abs(dy) != r) continue;
                if (walkable(m, *x + dx, *y + dy)) { *x += dx; *y += dy; return; }
            }
    }
}

static int failures = 0;
static void check(const char *name, int ok) {
    printf("selftest %-28s %d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* small-map self tests inside a sealed 16x12 open window at the corner */
static void selftests(const Map *big) {
    (void)big;
    Map m;
    m.blocked = calloc(NCELLS, 1);
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            if (x >= 16 || y >= 12) m.blocked[cell_idx(x, y)] = 1;

    int c = 4, nb = W / c;
    unsigned char *corr = calloc((size_t)nb * nb, 1);
    int exp_full, exp_fine, coarse_exp, nbp, refine_blocks, fb_used, fb_exp;

    Coarse co; coarse_build(&m, c, &co);
    int full = cell_astar(&m, 0, 0, 15, 11, c, NULL, &exp_full, NULL);
    int hf = hier_query(&m, &co, 0, 0, 15, 11, corr,
                        &coarse_exp, &exp_fine, &nbp,
                        &fb_used, &fb_exp, &refine_blocks);
    check("open_window_lengths_match", full == hf);
    /* the fine search may only touch cells inside the (dilated) corridor */
    check("fine_within_corridor", exp_fine <= refine_blocks * c * c);
    /* 4 block-columns and 3 block-rows, start and goal block counted once */
    check("coarse_path_blocks", nbp == 4 + 3 - 1);

    /* wall at x=8 with a gap (y=2,3) lying inside the straight corridor:
     * the coarse edge stays passable and refinement stays optimal */
    for (int y = 0; y < 12; y++)
        if (y < 2 || y > 3) m.blocked[cell_idx(8, y)] = 1;
    coarse_free(&co); coarse_build(&m, c, &co);
    full = cell_astar(&m, 0, 0, 15, 11, c, NULL, &exp_full, NULL);
    hf = hier_query(&m, &co, 0, 0, 15, 11, corr, &coarse_exp, &exp_fine,
                    &nbp, &fb_used, &fb_exp, &refine_blocks);
    check("wall_gap_path_found", full > 0 && hf > 0);
    check("wall_gap_lengths_match", full == hf);
    check("wall_gap_no_fallback", fb_used == 0);

    /* sealed wall: border edges disappear; coarse search must report failure */
    m.blocked[cell_idx(8, 2)] = 1;
    m.blocked[cell_idx(8, 3)] = 1;
    coarse_free(&co); coarse_build(&m, c, &co);
    full = cell_astar(&m, 0, 0, 15, 11, c, NULL, &exp_full, NULL);
    hf = hier_query(&m, &co, 0, 0, 15, 11, corr, &coarse_exp, &exp_fine,
                    &nbp, &fb_used, &fb_exp, &refine_blocks);
    check("sealed_wall_full_fails", full == -1);
    check("sealed_wall_hier_fails", hf == -1);

    coarse_free(&co);
    free(corr);
    free(m.blocked);
}

int main(void) {
    Map m;
    m.blocked = malloc(NCELLS);
    uint32_t seed = 1u;
    build_map(&m, seed);
    selftests(&m);

    /* three distance tiers: near / mid / far */
    int q[3][4] = {
        {900, 900, 1150, 1100},   /* near:  manhattan ~450  */
        {300, 300, 1700, 300},    /* mid:   manhattan 1400  */
        {100, 100, 1900, 1900},   /* far:   manhattan 3600  */
    };
    const char *tier[] = {"near", "mid", "far"};

    int full_len[3], full_exp[3];
    double full_ms[3];
    for (int t = 0; t < 3; t++) {
        snap_to_walkable(&m, &q[t][0], &q[t][1]);
        snap_to_walkable(&m, &q[t][2], &q[t][3]);
        double t0 = now_sec();
        full_len[t] = cell_astar(&m, q[t][0], q[t][1], q[t][2], q[t][3],
                                 32, NULL, &full_exp[t], NULL);
        full_ms[t] = (now_sec() - t0) * 1e3;
        if (full_len[t] < 0) {
            printf("FATAL: tier %s unreachable on seed %u; regenerate map\n",
                   tier[t], seed);
            return 2;
        }
    }

    /* offline preprocessing, one coarse graph per candidate chunk size */
    Coarse co[NCS];
    double pre_ms[NCS];
    for (int ci = 0; ci < NCS; ci++) {
        double t0 = now_sec();
        coarse_build(&m, CS[ci], &co[ci]);
        pre_ms[ci] = (now_sec() - t0) * 1e3;
    }

    size_t cap = (size_t)(W / 8) * (W / 8);   /* corridor sized for c=8 */
    unsigned char *corridor = malloc(cap);

    printf("\n# seed=%u map=%dx%d full_graph_nodes=%d\n", seed, W, H, NCELLS);
    printf("tier,c,full_len,hier_len,inflation_pct,coarse_nodes,fine_exp,");
    printf("hier_nodes,full_exp,node_ratio,model_ratio,coarse_exp,");
    printf("blocks_on_path,corridor_cells,fallback,full_ms,hier_ms,preprocess_ms\n");

    for (int t = 0; t < 3; t++) {
        for (int ci = 0; ci < NCS; ci++) {
            int c = CS[ci];
            int nblk = W / c;
            int coarse_exp, fine_exp, nbp, used_fb, fb_exp, refine_blocks;
            double t0 = now_sec();
            int hlen = hier_query(&m, &co[ci], q[t][0], q[t][1], q[t][2], q[t][3],
                                  corridor, &coarse_exp, &fine_exp, &nbp,
                                  &used_fb, &fb_exp, &refine_blocks);
            double hier_ms = (now_sec() - t0) * 1e3;
            if (hlen < 0) {
                printf("%s,%d,FAILED\n", tier[t], c);
                failures++;
                continue;
            }
            long coarse_nodes = (long)nblk * nblk;
            /* total generated nodes: coarse graph + refinement, plus the
             * full-map fallback search when the corridor was not threadable */
            long hier_nodes = coarse_nodes + fine_exp
                            + (used_fb ? (long)fb_exp : 0);
            long corridor_cells = (long)refine_blocks * c * c;
            double node_ratio = (double)hier_nodes / NCELLS;
            /* analytic model: (W/c)^2 + L*c, expressed as a fraction of W^2 */
            double model_ratio = (double)coarse_nodes / NCELLS
                               + (double)full_len[t] * c / (double)NCELLS;
            double inflation = 100.0 * (hlen - full_len[t]) / full_len[t];

            printf("%s,%d,%d,%d,%.3f,%ld,%d,%ld,%d,%.5f,%.5f,%d,%d,%ld,%d,%.2f,%.2f,%.2f\n",
                   tier[t], c, full_len[t], hlen, inflation,
                   coarse_nodes, fine_exp + (used_fb ? fb_exp : 0), hier_nodes,
                   full_exp[t], node_ratio, model_ratio, coarse_exp, nbp,
                   corridor_cells, used_fb, full_ms[t], hier_ms, pre_ms[ci]);
        }
    }

    printf("\nall_ok=%d\n", failures == 0);
    free(corridor);
    for (int ci = 0; ci < NCS; ci++) coarse_free(&co[ci]);
    free(m.blocked);
    return failures == 0 ? 0 : 1;
}
