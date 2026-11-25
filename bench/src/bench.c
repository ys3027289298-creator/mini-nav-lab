/*
 * bench: A* open-list structure benchmark.
 *
 * Three open-list implementations share identical search logic, map,
 * heuristic and tie rules; only the open table changes:
 *   ARRAY  - unsorted array: find-min O(n), membership O(n)   (cache hot)
 *   BITMAP - unsorted array + in-open byte: find-min O(n), membership O(1)
 *   HEAP   - binary heap + pos[] index: extract-min O(log n), membership O(1)
 *
 * Two map families are swept:
 *   scatter - isolated random blocked cells; the open set is a wavefront of
 *             perimeter size O(sqrt A), so the scan term grows as A^1.5.
 *   maze    - DFS backtracker maze with a few loops knocked out; the open set
 *             fills a fraction of the explored area O(A), reproducing the
 *             clean linear -> quadratic knee the capacity formula predicts.
 *
 * Per search the program counts expansions, insertions and scan entries so
 * the theoretical knee formula can be checked against measured data.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <time.h>

#define INF (1 << 29)
#define BLOCK_DENSITY 12       /* percent, scatter family */
#define MAZE_LOOP_KNOCKOUT 6   /* percent of extra walls removed, maze family */

typedef struct {
    int w, h;
    int sx, sy, gx, gy;
    unsigned char *block;
} Map;

typedef struct {
    long exp;       /* nodes popped / expanded */
    long ins;       /* nodes newly inserted into open */
    long scan;      /* entries touched by linear scans (array variants) */
    int  maxopen;   /* peak open-list size */
    int  pathlen;
} Stats;

static uint64_t rng_next(uint64_t *s) {
    uint64_t x = *s;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    *s = x;
    return x;
}

static Map *map_alloc(int w, int h) {
    Map *m = malloc(sizeof *m);
    m->w = w; m->h = h;
    m->sx = m->sy = 0; m->gx = w - 1; m->gy = h - 1;
    m->block = calloc((size_t)w * h, 1);
    return m;
}

/* family 1: scattered single-cell obstacles */
static Map *map_scatter(int L) {
    Map *m = map_alloc(L, L);
    uint64_t st = 0x9e3779b97f4a7c15ULL ^ ((uint64_t)L * 2654435761u);
    for (int y = 0; y < L; y++)
        for (int x = 0; x < L; x++)
            m->block[y * L + x] = (rng_next(&st) % 100) < BLOCK_DENSITY;
    const int keep[][2] = {
        {0,0},{1,0},{0,1},
        {L-1,L-1},{L-2,L-1},{L-1,L-2}
    };
    for (unsigned i = 0; i < sizeof keep / sizeof keep[0]; i++) {
        int x = keep[i][0], y = keep[i][1];
        if (x >= 0 && y >= 0 && x < L && y < L) m->block[y * L + x] = 0;
    }
    return m;
}

/* family 2: DFS backtracker maze on an odd-sized grid, plus a few loops */
static Map *map_maze(int L) {
    Map *m = map_alloc(L, L);
    memset(m->block, 1, (size_t)L * L);
    int *stack = malloc((size_t)L * L * sizeof *stack);
    uint64_t st = 0xD1B54A32D192ED03ULL ^ ((uint64_t)L * 0x9E3779B1u);
    int top = 0;
    m->block[0] = 0;
    stack[top++] = 0;
    static const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
    while (top > 0) {
        int cell = stack[top - 1];
        int cx = cell % L, cy = cell / L;
        int order[4] = {0, 1, 2, 3};
        for (int i = 3; i > 0; i--) {
            int j = (int)(rng_next(&st) % (unsigned)(i + 1));
            int tmp = order[i]; order[i] = order[j]; order[j] = tmp;
        }
        int moved = 0;
        for (int d = 0; d < 4; d++) {
            int nx = cx + 2 * dx[order[d]], ny = cy + 2 * dy[order[d]];
            if (nx < 0 || ny < 0 || nx >= L || ny >= L) continue;
            int ni = ny * L + nx;
            if (!m->block[ni]) continue;
            int wx = cx + dx[order[d]], wy = cy + dy[order[d]];
            m->block[wy * L + wx] = 0;
            m->block[ni] = 0;
            stack[top++] = ni;
            moved = 1;
            break;
        }
        if (!moved) top--;
    }
    /* knock out a few interior walls so the maze has loops (not a pure tree) */
    for (int y = 1; y < L - 1; y++)
        for (int x = 1; x < L - 1; x++)
            if (m->block[y * L + x] && (rng_next(&st) % 100) < MAZE_LOOP_KNOCKOUT)
                m->block[y * L + x] = 0;
    free(stack);
    m->block[0] = 0;
    m->block[(L - 1) * L + (L - 1)] = 0;
    return m;
}

static void map_free(Map *m) { free(m->block); free(m); }

typedef struct {
    int n;
    int *g, *f, *parent, *closed;
    int *open;          /* array open list / heap storage (1-indexed for heap) */
    int *pos;           /* heap: node -> heap index; bitmap: in-open flag */
} Ctx;

static Ctx ctx_make(int n) {
    Ctx c;
    c.n = n;
    c.g = malloc((size_t)n * sizeof *c.g);
    c.f = malloc((size_t)n * sizeof *c.f);
    c.parent = malloc((size_t)n * sizeof *c.parent);
    c.closed = malloc((size_t)n * sizeof *c.closed);
    c.open = malloc((size_t)(n + 1) * sizeof *c.open);
    c.pos = malloc((size_t)n * sizeof *c.pos);
    return c;
}

static void ctx_free(Ctx *c) {
    free(c->g); free(c->f); free(c->parent); free(c->closed);
    free(c->open); free(c->pos);
}

static void ctx_reset(Ctx *c) {
    for (int i = 0; i < c->n; i++) {
        c->g[i] = INF; c->f[i] = INF;
        c->parent[i] = -1; c->closed[i] = 0; c->pos[i] = -1;
    }
}

static int walkable(const Map *m, int x, int y) {
    if (x < 0 || y < 0 || x >= m->w || y >= m->h) return 0;
    return !m->block[y * m->w + x];
}

/* deterministic total order so every variant pops the same node on f ties */
static int key_less(const Ctx *c, int a, int b) {
    return c->f[a] < c->f[b] || (c->f[a] == c->f[b] && a < b);
}

/* variant 0: plain unsorted array, both scans linear */
static int astar_array(const Map *m, Ctx *c, Stats *st) {
    const int W = m->w, goal = m->gy * W + m->gx;
    ctx_reset(c);
    int nopen = 0;
    int s = m->sy * W + m->sx;
    c->g[s] = 0; c->f[s] = (m->gx - m->sx) + (m->gy - m->sy);
    c->open[nopen++] = s;
    static const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};

    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) {
            st->scan++;
            if (key_less(c, c->open[i], c->open[best])) best = i;
        }
        int cur = c->open[best];
        c->open[best] = c->open[--nopen];
        if (cur == goal) { st->pathlen = c->g[cur] + 1; return 0; }
        c->closed[cur] = 1;
        st->exp++;

        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = ny * W + nx;
            if (c->closed[ni]) continue;
            int ng = c->g[cur] + 1;
            if (ng < c->g[ni]) {
                c->g[ni] = ng;
                c->f[ni] = ng + (m->gx > nx ? m->gx - nx : nx - m->gx)
                               + (m->gy > ny ? m->gy - ny : ny - m->gy);
                c->parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < nopen; i++) {
                    st->scan++;
                    if (c->open[i] == ni) { found = 1; break; }
                }
                if (!found) {
                    c->open[nopen++] = ni; st->ins++;
                    if (nopen > st->maxopen) st->maxopen = nopen;
                }
            }
        }
    }
    st->pathlen = -1;
    return -1;
}

/* variant 1: unsorted array with O(1) membership byte; find-min still linear */
static int astar_bitmap(const Map *m, Ctx *c, Stats *st) {
    const int W = m->w, goal = m->gy * W + m->gx;
    ctx_reset(c); /* pos[] reused as in-open flag: -1 absent, 0 present */
    int nopen = 0;
    int s = m->sy * W + m->sx;
    c->g[s] = 0; c->f[s] = (m->gx - m->sx) + (m->gy - m->sy);
    c->open[nopen++] = s; c->pos[s] = 0;
    static const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};

    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) {
            st->scan++;
            if (key_less(c, c->open[i], c->open[best])) best = i;
        }
        int cur = c->open[best];
        c->open[best] = c->open[--nopen];
        c->pos[cur] = -1;
        if (cur == goal) { st->pathlen = c->g[cur] + 1; return 0; }
        c->closed[cur] = 1;
        st->exp++;

        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = ny * W + nx;
            if (c->closed[ni]) continue;
            int ng = c->g[cur] + 1;
            if (ng < c->g[ni]) {
                c->g[ni] = ng;
                c->f[ni] = ng + (m->gx > nx ? m->gx - nx : nx - m->gx)
                               + (m->gy > ny ? m->gy - ny : ny - m->gy);
                c->parent[ni] = cur;
                if (c->pos[ni] == -1) {
                    c->pos[ni] = 0;
                    c->open[nopen++] = ni;
                    st->ins++;
                    if (nopen > st->maxopen) st->maxopen = nopen;
                }
            }
        }
    }
    st->pathlen = -1;
    return -1;
}

/* variant 2: binary min-heap on f[] with pos[] index */
static void heap_siftup(Ctx *c, int i) {
    int node = c->open[i];
    while (i > 1) {
        int p = i >> 1;
        if (!key_less(c, node, c->open[p])) break;
        c->open[i] = c->open[p];
        c->pos[c->open[i]] = i;
        i = p;
    }
    c->open[i] = node;
    c->pos[node] = i;
}

static void heap_siftdown(Ctx *c, int i, int sz) {
    int node = c->open[i];
    for (;;) {
        int l = i << 1;
        if (l > sz) break;
        int r = l + 1, child = l;
        /* pick the smaller child, then compare IT against the saved node:
           open[i] here may be a stale duplicate lifted on the previous hop */
        if (r <= sz && key_less(c, c->open[r], c->open[l])) child = r;
        if (!key_less(c, c->open[child], node)) break;
        c->open[i] = c->open[child];
        c->pos[c->open[i]] = i;
        i = child;
    }
    c->open[i] = node;
    c->pos[node] = i;
}

static int astar_heap(const Map *m, Ctx *c, Stats *st) {
    const int W = m->w, goal = m->gy * W + m->gx;
    ctx_reset(c);
    int sz = 0;
    int s = m->sy * W + m->sx;
    c->g[s] = 0; c->f[s] = (m->gx - m->sx) + (m->gy - m->sy);
    c->open[++sz] = s; c->pos[s] = sz;
    static const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};

    while (sz > 0) {
        int cur = c->open[1];
        c->open[1] = c->open[sz];
        if (--sz >= 1) { c->pos[c->open[1]] = 1; heap_siftdown(c, 1, sz); }
        c->pos[cur] = -1;
        if (cur == goal) { st->pathlen = c->g[cur] + 1; return 0; }
        c->closed[cur] = 1;
        st->exp++;

        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = ny * W + nx;
            if (c->closed[ni]) continue;
            int ng = c->g[cur] + 1;
            if (ng < c->g[ni]) {
                c->g[ni] = ng;
                c->f[ni] = ng + (m->gx > nx ? m->gx - nx : nx - m->gx)
                               + (m->gy > ny ? m->gy - ny : ny - m->gy);
                c->parent[ni] = cur;
                if (c->pos[ni] == -1) {
                    c->open[++sz] = ni; c->pos[ni] = sz;
                    heap_siftup(c, sz);
                    st->ins++;
                    if (sz > st->maxopen) st->maxopen = sz;
                } else {
                    heap_siftup(c, c->pos[ni]); /* f only decreases */
                }
            }
        }
    }
    st->pathlen = -1;
    return -1;
}

typedef int (*search_fn)(const Map *, Ctx *, Stats *);

static double now_sec(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec / 1e9;
}

static int cmp_i64(const void *a, const void *b) {
    int64_t x = *(const int64_t *)a, y = *(const int64_t *)b;
    return (x > y) - (x < y);
}

int main(void) {
    /* odd side lengths so the maze generator has well-defined walls/cells.
     * Dense sampling below side 49 brackets the measured knee region. */
    const int sides_scatter[] = {9, 11, 13, 15, 17, 19, 23, 27, 33, 41, 49,
                                 65, 97, 129, 193, 257, 385};
    const int sides_maze_arr[] = {9, 11, 13, 15, 17, 19, 23, 27, 33, 41, 49,
                                  65, 97, 129, 193, 257};
    const int sides_maze_heap[] = {9, 13, 17, 21, 25, 33, 49, 65, 97, 129,
                                   193, 257, 385, 513, 769, 1025, 1449};
    struct { const char *family; const char *name; search_fn fn;
             const int *sides; int nsides; } impls[] = {
        {"scatter","array",  astar_array,  sides_scatter,
         sizeof sides_scatter / sizeof *sides_scatter},
        {"scatter","bitmap", astar_bitmap, sides_scatter,
         sizeof sides_scatter / sizeof *sides_scatter},
        {"scatter","heap",   astar_heap,   sides_maze_heap,
         sizeof sides_maze_heap / sizeof *sides_maze_heap},
        {"maze","array",     astar_array,  sides_maze_arr,
         sizeof sides_maze_arr / sizeof *sides_maze_arr},
        {"maze","bitmap",    astar_bitmap, sides_maze_arr,
         sizeof sides_maze_arr / sizeof *sides_maze_arr},
        {"maze","heap",      astar_heap,   sides_maze_heap,
         sizeof sides_maze_heap / sizeof *sides_maze_heap},
    };

    puts("family,variant,side,area,ns_median,ns_min,reps,exp,ins,scan,k,alpha,omax,pathlen");
    for (unsigned im = 0; im < sizeof impls / sizeof *impls; im++) {
        for (int si = 0; si < impls[im].nsides; si++) {
            int L = impls[im].sides[si];
            Map *map = impls[im].family[0] == 's' ? map_scatter(L) : map_maze(L);
            Ctx ctx = ctx_make(L * L);
            Stats probe = {0};
            impls[im].fn(map, &ctx, &probe);
            if (probe.pathlen < 0) {
                fprintf(stderr, "WARN: no path on %s L=%d\n",
                        impls[im].family, L);
            }

            /* adaptive repetitions: aim for ~0.8 s total per point */
            double t0 = now_sec();
            Stats tmp = {0};
            impls[im].fn(map, &ctx, &tmp);
            double first = now_sec() - t0;
            int reps = (int)(0.8 / first);
            if (reps < 3) reps = 3;
            if (reps > 31) reps = 31;

            int64_t *samples = malloc((size_t)reps * sizeof *samples);
            Stats last = {0};
            for (int r = 0; r < reps; r++) {
                Stats run = {0};
                double a = now_sec();
                impls[im].fn(map, &ctx, &run);
                samples[r] = (int64_t)((now_sec() - a) * 1e9);
                last = run;
            }
            qsort(samples, (size_t)reps, sizeof *samples, cmp_i64);
            int64_t med = samples[reps / 2], mn = samples[0];
            free(samples);

            long area = (long)L * L;
            double k = last.exp ? (double)last.ins / last.exp : 0.0;
            double alpha = (double)last.exp / area;
            printf("%s,%s,%d,%ld,%lld,%lld,%d,%ld,%ld,%ld,%.4f,%.4f,%d,%d\n",
                   impls[im].family, impls[im].name, L, area,
                   (long long)med, (long long)mn, reps,
                   last.exp, last.ins, last.scan, k, alpha,
                   last.maxopen, last.pathlen);
            fflush(stdout);
            ctx_free(&ctx);
            map_free(map);
        }
    }
    return 0;
}
