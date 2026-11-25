/* bench: compare forward (A) vs backward (B) greedy LOS smoothing.
 *
 * A: for (j = i+1; j <= n-1; j++) if (!los(i,j)) break;   // keep j-1
 * B: for (j = n-1; j > i+1;   j--) if ( los(i,j)) break;   // keep j
 *
 * Self-contained: random grid maps, 4-neighbour A* (binary heap),
 * Bresenham supercover LOS with call counter, multi-source Dijkstra
 * clearance field, windowed-B horizon sweep.
 */
#define _POSIX_C_SOURCE 199309L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

typedef struct { int w, h; unsigned char *blocked; } Map;

static int midx(const Map *m, int x, int y) { return y * m->w + x; }
static int inb(const Map *m, int x, int y) {
    return x >= 0 && y >= 0 && x < m->w && y < m->h;
}

/* ---------- deterministic RNG (xorshift32) ---------- */
static unsigned int rng_state;
static unsigned int rng(void) {
    unsigned int x = rng_state;
    x ^= x << 13; x ^= x >> 17; x ^= x << 5;
    rng_state = x;
    return x;
}

/* ---------- LOS: Bresenham supercover as in smooth/src/smooth.c.
 * strict mode (no corner cutting): a diagonal step additionally requires
 * both orthogonally adjacent cells to be free. */
static long los_calls;
static int los_strict;
static int los(const Map *m, int x0, int y0, int x1, int y1) {
    los_calls++;
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy, x = x0, y = y0;
    for (;;) {
        if (!inb(m, x, y)) return 0;
        if (m->blocked[midx(m, x, y)]) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        int gx = (e2 > -dy), gy = (e2 < dx);
        if (los_strict && gx && gy) {
            /* diagonal move: both side cells must be walkable */
            if (!inb(m, x+sx, y) || m->blocked[midx(m, x+sx, y)]) return 0;
            if (!inb(m, x, y+sy) || m->blocked[midx(m, x, y+sy)]) return 0;
        }
        if (gx) { err -= dy; x += sx; }
        if (gy) { err += dx; y += sy; }
    }
}
static int losp(const Map *m, int a, int b) {
    return los(m, a % m->w, a / m->w, b % m->w, b / m->w);
}

/* ---------- binary heap A*, 4-neighbour ---------- */
typedef struct { int *h, n, cap, *f; } Heap;
static void hpush(Heap *q, int v) {
    int i = q->n++;
    q->h[i] = v;
    while (i > 0) {
        int p = (i - 1) / 2;
        if (q->f[q->h[p]] <= q->f[q->h[i]]) break;
        int t = q->h[p]; q->h[p] = q->h[i]; q->h[i] = t;
        i = p;
    }
}
static int hpop(Heap *q) {
    int top = q->h[0], last = q->h[--q->n];
    if (q->n > 0) {
        int i = 0; q->h[0] = last;
        for (;;) {
            int l = 2*i+1, r = l+1, s = i;
            if (l < q->n && q->f[q->h[l]] < q->f[q->h[s]]) s = l;
            if (r < q->n && q->f[q->h[r]] < q->f[q->h[s]]) s = r;
            if (s == i) break;
            int t = q->h[s]; q->h[s] = q->h[i]; q->h[i] = t;
            i = s;
        }
    }
    return top;
}
static int astar(const Map *m, int sx, int sy, int gx, int gy, int *out, int cap) {
    int N = m->w * m->h, INF = 1 << 29;
    int *g = malloc(N * sizeof(int)), *f = malloc(N * sizeof(int));
    int *par = malloc(N * sizeof(int)), *closed = calloc(N, sizeof(int));
    int *heap = malloc(N * sizeof(int));
    for (int i = 0; i < N; i++) { g[i] = INF; f[i] = INF; par[i] = -1; }
    Heap q = { heap, 0, N, f };
    int s = midx(m, sx, sy), goal = midx(m, gx, gy);
    g[s] = 0; f[s] = abs(gx-sx) + abs(gy-sy); hpush(&q, s);
    int found = 0;
    while (q.n > 0) {
        int cur = hpop(&q);
        if (closed[cur]) continue;
        closed[cur] = 1;
        if (cur == goal) { found = 1; break; }
        int cx = cur % m->w, cy = cur / m->w;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx+dx[d], ny = cy+dy[d];
            if (!inb(m, nx, ny) || m->blocked[midx(m,nx,ny)]) continue;
            int ni = midx(m, nx, ny);
            if (closed[ni]) continue;
            if (g[cur] + 1 < g[ni]) {
                g[ni] = g[cur] + 1;
                f[ni] = g[ni] + abs(gx-nx) + abs(gy-ny);
                par[ni] = cur;
                hpush(&q, ni);
            }
        }
    }
    int k = -1;
    if (found) {
        int rev[N > 0 ? N : 1];
        int c = goal, n = 0;
        while (c != -1) { rev[n++] = c; c = par[c]; }
        if (n > cap) { k = -2; }
        else { k = n; for (int i = 0; i < n; i++) out[i] = rev[n-1-i]; }
    }
    free(g); free(f); free(par); free(closed); free(heap);
    return k;
}

/* ---------- map generators ---------- */
static long gen_random(Map *m, double density, int sx, int sy, int gx, int gy) {
    int N = m->w * m->h;
    long nb = 0;
    for (int i = 0; i < N; i++) {
        int x = i % m->w, y = i / m->w;
        int endpoint = (x==sx&&y==sy) || (x==gx&&y==gy);
        int blk = !endpoint && (rng() % 1000) < (unsigned int)(density*1000);
        m->blocked[i] = (unsigned char)blk;
        nb += blk;
    }
    return nb;
}

/* pillar hall: b*b blocks on a jittered lattice, spacing g; dense enough that
 * A* hugs pillar corners (local LOS dies at i+2), open gaps still allow long
 * diagonal sightlines for the backward variant.
 * If band_h > 0, a horizontal obstacle-free band of that height is kept
 * around mid-height, giving the backward variant a long-sightline chamber. */
static long gen_pillars(Map *m, int g, int b, int pp_milli, int band_h) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int band_lo = band_h > 0 ? m->h / 2 - band_h / 2 : -1;
    int band_hi = band_h > 0 ? m->h / 2 + band_h / 2 : -1;
    for (int gy2 = g; gy2 + b < m->h - 1; gy2 += g) {
        for (int gx2 = g; gx2 + b < m->w - 1; gx2 += g) {
            if ((int)(rng() % 1000) >= pp_milli) continue;
            int jx = (int)(rng() % 3) - 1, jy = (int)(rng() % 3) - 1;
            for (int oy = 0; oy < b; oy++) for (int ox = 0; ox < b; ox++) {
                int x = gx2 + ox + jx, y = gy2 + oy + jy;
                if (band_h > 0 && y >= band_lo && y < band_hi) continue;
                if (!inb(m, x, y) || m->blocked[midx(m,x,y)]) continue;
                m->blocked[midx(m,x,y)] = 1; nb++;
            }
        }
    }
    return nb;
}

/* staggered short walls: horizontal segments on every g-th row with a single
 * doorway; jitter=1 randomizes each door column so the path must zigzag
 * through door jambs, jitter=0 alternates left/right. */
static long gen_walls(Map *m, int g, int gap, int jitter) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int row = 0;
    for (int y = g; y < m->h - 1; y += g, row++) {
        int gapx;
        if (jitter) {
            gapx = 2 + (int)(rng() % (unsigned)(m->w - 4));
        } else {
            gapx = (row % 2) ? m->w - 1 - gap : gap;
        }
        for (int x = 1; x < m->w - 1; x++) {
            if (abs(x - gapx) <= gap / 2) continue;
            m->blocked[midx(m, x, y)] = 1; nb++;
        }
    }
    return nb;
}

/* room grid: solid wall bands every `cell` rows/cols, each band pierced by a
 * narrow doorway (width `door`); door jambs are solid wall corners, so under
 * no-corner-cutting LOS every local bend is occluded while room interiors
 * admit long sightlines to far doorways. */
static long gen_rooms(Map *m, int cell, int door) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int band = 0;
    for (int y = cell; y < m->h - 1; y += cell, band++) {
        int doorx = (band % 2) ? m->w - 1 - cell / 2 : cell / 2 + 1;
        for (int x = 1; x < m->w - 1; x++) {
            if (abs(x - doorx) <= door / 2) continue;
            m->blocked[midx(m, x, y)] = 1; nb++;
        }
    }
    band = 0;
    for (int x = cell; x < m->w - 1; x += cell, band++) {
        int doory = (band % 2) ? m->h - 1 - cell / 2 : cell / 2 + 1;
        for (int y = 1; y < m->h - 1; y++) {
            if (abs(y - doory) <= door / 2) continue;
            m->blocked[midx(m, x, y)] = 1; nb++;
        }
    }
    return nb;
}

/* snake walls: full-width horizontal walls every g rows, one cell-thick,
 * each with a single-cell doorway alternating at the left/right margin.
 * A* must cross every room end to end; at each door jamb it takes a
 * corner-hugging diagonal step, which strict (no-corner-cut) LOS rejects
 * for any segment ending at the next bend, while open room interiors keep
 * long sightlines alive for the backward probe. */
static long gen_snake(Map *m, int g, int margin) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int row = 0;
    for (int y = g; y < m->h - 1; y += g, row++) {
        int doorx = (row % 2) ? m->w - 1 - margin : margin;
        for (int x = 1; x < m->w - 1; x++) {
            if (x == doorx) continue;
            m->blocked[midx(m, x, y)] = 1; nb++;
        }
    }
    return nb;
}

/* zigzag: 2x2 blocks placed on a diagonal checker lattice so the free space
 * is a one-cell-wide zigzag corridor; every corner step touches a block
 * diagonally, so strict LOS fails at i+2 everywhere. A horizontal open band
 * (band_h rows at mid-height) breaks the maze into two zigzag halves joined
 * by a chamber, where a long sightline can survive for the backward probe. */
static long gen_zigzag(Map *m, int g, int band_h) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int band_lo = band_h > 0 ? m->h / 2 - band_h / 2 : -1;
    int band_hi = band_h > 0 ? m->h / 2 + band_h / 2 : -1;
    int row = 0;
    for (int by = 1; by + 1 < m->h - 1; by += g, row++) {
        int shift = (row % 2) ? g / 2 : 0;
        for (int bx = 1 + shift; bx + 1 < m->w - 1; bx += g) {
            for (int oy = 0; oy < 2; oy++) for (int ox = 0; ox < 2; ox++) {
                int x = bx + ox, y = by + oy;
                if (band_h > 0 && y >= band_lo && y < band_hi) continue;
                if (!inb(m, x, y) || m->blocked[midx(m,x,y)]) continue;
                m->blocked[midx(m,x,y)] = 1; nb++;
            }
        }
    }
    return nb;
}

/* lanes: vertical one-cell corridors separated by solid walls, connected at
 * alternating ends by one-cell horizontal slots (a classic switchback maze).
 * Inside a corridor the path runs straight alongside a wall: strict LOS from
 * corridor cell t to t+2 crosses the wall-adjacent diagonal side cell and
 * fails, so the forward probe cannot skip two steps; the connector slots let
 * a long backward sightline jump across many corridor cells at once. */
static long gen_lanes(Map *m, int period, int band_h) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int wall_step = period + 1;
    int lane = 0;
    for (int wx = wall_step; wx < m->w - 1; wx += wall_step, lane++) {
        int open_top = (lane % 2) == 0;
        for (int y = 1; y < m->h - 1; y++) {
            int near_top = y < 1 + band_h, near_bot = y > m->h - 2 - band_h;
            int slot = open_top ? near_top : near_bot;
            if (slot) continue;
            m->blocked[midx(m, wx, y)] = 1; nb++;
        }
    }
    return nb;
}

/* corridor: width-w snake corridor carved through a solid block field.
 * Horizontal runs of width w stacked with alternating flow direction; the
 * connector between two runs is a w-wide vertical slot at the end they share,
 * so the free cells form one connected switchback. Every turn is a 90-degree
 * bend whose inner corner stays solid: under strict no-corner-cut LOS,
 * los(t, t+2) fails at each bend, while long straight runs keep far
 * sightlines alive for the backward probe. */
static long gen_corridor(Map *m, int w, int pitch) {
    int N = m->w * m->h;
    memset(m->blocked, 1, N);
    long nb = N;
    #define CARVE(X, Y) do { int _x=(X), _y=(Y); \
        if (inb(m,_x,_y) && m->blocked[midx(m,_x,_y)]) { m->blocked[midx(m,_x,_y)] = 0; nb--; } } while(0)
    int y0 = 1, seg = 0;
    while (y0 + w <= m->h - 1) {
        int ltr = (seg % 2 == 0);
        int lo = 1, hi = m->w - 2;
        for (int y = y0; y < y0 + w; y++)
            for (int x = lo; x <= hi; x++) CARVE(x, y);
        int next_y = y0 + w + pitch;
        if (next_y + w > m->h - 1) break;
        int endx = ltr ? m->w - 2 : 1;          /* shared end column */
        int cxlo = endx - w + 1, cxhi = endx;
        if (cxlo < 1) cxlo = 1;
        for (int y = y0 + w; y < next_y + w && y <= m->h - 2; y++)
            for (int x = cxlo; x <= cxhi; x++) CARVE(x, y);
        y0 = next_y;
        seg++;
    }
    #undef CARVE
    return nb;
}

/* staircase: a one-cell-thick monotone staircase (alternating R/U steps)
 * carved through solid wall, so every other step is an L-bend whose inner
 * corner is solid: strict LOS fails at los(i, i+2) for every anchor on the
 * stairs, giving the forward variant reach ~ 1. Periodic full-width open
 * bands break the stairs; inside a band, long straight sightlines survive,
 * which only the backward probe can reach for. */
static long gen_staircase(Map *m, int nbands, int band_h) {
    int N = m->w * m->h;
    memset(m->blocked, 1, N);
    long nb = N;
    #define CARVE2(X, Y) do { int _x=(X), _y=(Y); \
        if (inb(m,_x,_y) && m->blocked[midx(m,_x,_y)]) { m->blocked[midx(m,_x,_y)] = 0; nb--; } } while(0)
    /* open bands at evenly spaced rows */
    int span = (m->h - 2) / (nbands + 1);
    for (int b = 1; b <= nbands; b++) {
        int yc = 1 + b * span;
        for (int y = yc - band_h/2; y <= yc + band_h/2; y++)
            for (int x = 1; x < m->w - 1; x++) CARVE2(x, y);
    }
    /* monotone staircase polyline from (1,1) to (w-2,h-2) */
    int x = 1, y = 1, parity = 0;
    CARVE2(x, y);
    while (x < m->w - 2 || y < m->h - 2) {
        int go_r = (parity % 2 == 0) ? (x < m->w - 2) : (y == m->h - 2);
        if (go_r) x++; else y++;
        parity++;
        CARVE2(x, y);
    }
    #undef CARVE2
    return nb;
}

/* comb: vertical solid walls spaced `spacing` columns apart, each pierced by
 * a one-cell seam on the same row. A* zigzags wall to wall through the seams;
 * strict LOS rejects los(i, i+2) at every wall-hugging bend, while any two
 * seam cells see each other horizontally through every intervening seam —
 * long-range visibility that only the backward probe can reach for. */
static long gen_comb(Map *m, int spacing) {
    int N = m->w * m->h;
    memset(m->blocked, 0, N);
    long nb = 0;
    int seam_y = m->h / 2;
    for (int wx = spacing; wx < m->w - 1; wx += spacing) {
        for (int y = 1; y < m->h - 1; y++) {
            if (y == seam_y) continue;
            m->blocked[midx(m, wx, y)] = 1; nb++;
        }
    }
    return nb;
}

static void clear_endpoints(Map *m, int sx, int sy, int gx, int gy) {
    for (int oy = -1; oy <= 1; oy++) for (int ox = -1; ox <= 1; ox++) {
        int p1x = sx+ox, p1y = sy+oy, p2x = gx+ox, p2y = gy+oy;
        if (inb(m,p1x,p1y)) m->blocked[midx(m,p1x,p1y)] = 0;
        if (inb(m,p2x,p2y)) m->blocked[midx(m,p2x,p2y)] = 0;
    }
}

/* ---------- BFS reachability, used to reject disconnected maps ---------- */
static int reachable(const Map *m, int s, int t) {
    int N = m->w * m->h;
    unsigned char *seen = calloc(N, 1);
    int *st = malloc(N * sizeof(int));
    int n = 0; st[n++] = s; seen[s] = 1;
    while (n > 0) {
        int cur = st[--n];
        if (cur == t) { free(seen); free(st); return 1; }
        int cx = cur % m->w, cy = cur / m->w;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx+dx[d], ny = cy+dy[d];
            if (!inb(m, nx, ny)) continue;
            int ni = midx(m, nx, ny);
            if (m->blocked[ni] || seen[ni]) continue;
            seen[ni] = 1; st[n++] = ni;
        }
    }
    free(seen); free(st);
    return 0;
}

/* ---------- smoothing A: forward, first LOS failure stops the probe ---------- */
static int smooth_a(const Map *m, const int *path, int n, int *out) {
    int w = 0;
    out[w++] = path[0];
    int i = 0;
    while (i < n - 1) {
        int j = i + 1;
        while (j <= n - 1 && losp(m, path[i], path[j])) j++;
        int keep = (j > n - 1) ? n - 1 : j - 1;   /* all clear -> goal */
        out[w++] = path[keep];
        i = keep;
    }
    return w;
}

/* ---------- smoothing B: backward, first LOS success stops the probe ----------
 * horizon S limits how far ahead a probe may reach (S >= n-1 = full B). */
static int smooth_b(const Map *m, const int *path, int n, int *out, int S) {
    int w = 0;
    out[w++] = path[0];
    int i = 0;
    while (i < n - 1) {
        int top = i + S; if (top > n - 1) top = n - 1;
        int j = top;
        while (j > i + 1 && !losp(m, path[i], path[j])) j--;
        out[w++] = path[j];
        i = j;
    }
    return w;
}

/* ---------- clearance field: euclidean-ish distance to nearest blocked cell,
 * multi-source Dijkstra on the 8-neighbour graph ---------- */
static double *clearance_field(const Map *m) {
    int N = m->w * m->h;
    double *d = malloc(N * sizeof(double));
    int *done = calloc(N, sizeof(int));
    int *inheap = calloc(N, sizeof(int));
    int *heap = malloc(N * sizeof(int));
    int hn = 0;
    for (int i = 0; i < N; i++) d[i] = 1e18;
    /* index heap by sifting on d[] */
    #define UP(I) do { int i=(I); while (i>0){int p=(i-1)/2; \
        if (d[heap[p]]<=d[heap[i]]) break; \
        int t=heap[p]; heap[p]=heap[i]; heap[i]=t; i=p;} } while(0)
    #define DN() do { int i=0; for(;;){int l=2*i+1,r=l+1,s=i; \
        if (l<hn && d[heap[l]]<d[heap[s]]) s=l; \
        if (r<hn && d[heap[r]]<d[heap[s]]) s=r; \
        if (s==i) break; \
        int t=heap[s]; heap[s]=heap[i]; heap[i]=t; i=s;} } while(0)
    #define PUSH(V) do { if (!inheap[V]) { inheap[V] = 1; heap[hn++] = (V); UP(hn-1); } } while(0)
    for (int i = 0; i < N; i++) if (m->blocked[i]) { d[i] = 0; PUSH(i); }
    while (hn > 0) {
        int cur = heap[0]; heap[0] = heap[--hn]; if (hn) DN();
        if (done[cur]) continue;
        done[cur] = 1;
        int cx = cur % m->w, cy = cur / m->w;
        for (int oy = -1; oy <= 1; oy++) for (int ox = -1; ox <= 1; ox++) {
            if (!ox && !oy) continue;
            int nx = cx+ox, ny = cy+oy;
            if (!inb(m, nx, ny) || m->blocked[midx(m,nx,ny)]) continue;
            int ni = midx(m, nx, ny);
            double wgt = (ox && oy) ? 1.4142135623730951 : 1.0;
            if (d[cur] + wgt < d[ni]) { d[ni] = d[cur] + wgt; PUSH(ni); }
        }
    }
    #undef UP
    #undef DN
    #undef PUSH
    free(done); free(inheap); free(heap);
    return d;
}

/* minimum clearance over every supercover cell of every kept segment */
static double path_clearance(const Map *m, const int *p, int k, const double *d) {
    double best = 1e18;
    for (int t = 0; t + 1 < k; t++) {
        int x0 = p[t] % m->w, y0 = p[t] / m->w;
        int x1 = p[t+1] % m->w, y1 = p[t+1] / m->w;
        int dx = abs(x1-x0), dy = abs(y1-y0);
        int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
        int err = dx-dy, x = x0, y = y0;
        for (;;) {
            double v = d[midx(m, x, y)];
            if (v < best) best = v;
            if (x == x1 && y == y1) break;
            int e2 = 2*err;
            if (e2 > -dy) { err -= dy; x += sx; }
            if (e2 <  dx) { err += dx; y += sy; }
        }
    }
    return best;
}

static int count_bends(const Map *m, const int *p, int n) {
    int b = 0;
    for (int i = 1; i + 1 < n; i++) {
        int dx1 = p[i]%m->w - p[i-1]%m->w, dy1 = p[i]/m->w - p[i-1]/m->w;
        int dx2 = p[i+1]%m->w - p[i]%m->w, dy2 = p[i+1]/m->w - p[i]/m->w;
        if (dx1 != dx2 || dy1 != dy2) b++;
    }
    return b;
}

static double now_sec(void) {
    struct timespec ts; clock_gettime(CLOCK_MONOTONIC, &ts);
    return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static int segments_clear(const Map *m, const int *p, int k) {
    for (int i = 0; i + 1 < k; i++) if (!losp(m, p[i], p[i+1])) return 0;
    return 1;
}

/* mean index distance between consecutive kept points; 1.0 == identity path.
 * Needs the raw path to map kept points back to their indices. */
static double avg_reach(const Map *m, const int *raw, int n, const int *out, int k) {
    (void)m;
    long total = 0;
    int prev = 0;
    for (int t = 1; t < k; t++) {
        int cur = prev + 1;
        while (cur < n && raw[cur] != out[t]) cur++;
        total += cur - prev;
        prev = cur;
    }
    return (double)total / (k - 1);
}

int main(int argc, char **argv) {
    const char *outdir = argc > 1 ? argv[1] : ".";
    const char *mode   = argc > 2 ? argv[2] : "rooms";    /* random|pillars|walls|rooms */
    const char *losm   = argc > 3 ? argv[3] : "strict";   /* strict|plain */
    los_strict = !strcmp(losm, "strict");
    char path[256];
    char tag[64]; snprintf(tag, sizeof(tag), "%s_%s", mode, losm);
    FILE *fb = NULL, *fs = NULL;
    snprintf(path, sizeof(path), "%s/smooth_bench_%s.csv", outdir, tag);
    fb = fopen(path, "w");
    snprintf(path, sizeof(path), "%s/smooth_sweep_%s.csv", outdir, tag);
    fs = fopen(path, "w");
    if (!fb || !fs) { perror("open output"); return 2; }
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(fb, NULL, _IOLBF, 0);
    setvbuf(fs, NULL, _IOLBF, 0);
    fprintf(fb, "size,W,H,obstacle_pct,raw_n,bends,version,out_m,reduction,los_calls,min_clearance,usec,avg_reach\n");
    fprintf(fs, "size,W,H,horizon_S,out_m,reduction,los_calls,min_clearance\n");

    /* ---- sanity fixture from smooth/src/smooth.c: 16x12, blocked (2,0) ---- */
    {
        Map f; f.w = 16; f.h = 12;
        f.blocked = calloc(f.w*f.h, 1);
        f.blocked[midx(&f, 2, 0)] = 1;
        int p[20], oa[20], ob[20], n = 0;
        int pts[7][2] = {{0,0},{1,0},{1,1},{2,1},{3,1},{3,0},{4,0}};
        for (int i = 0; i < 7; i++) p[n++] = midx(&f, pts[i][0], pts[i][1]);
        los_strict = 0;   /* fixture exercises the plain repo LOS semantics */
        los_calls = 0; int ma = smooth_a(&f, p, n, oa); long pa = los_calls;
        los_calls = 0; int mb = smooth_b(&f, p, n, ob, n); long pb = los_calls;
        los_strict = !strcmp(losm, "strict");
        int same = (ma == mb);
        for (int i = 0; same && i < ma; i++) if (oa[i] != ob[i]) same = 0;
        printf("fixture_7pt: A_m=%d B_m=%d identical=%d (A_probes=%ld B_probes=%ld)\n",
               ma, mb, same, pa, pb);
        free(f.blocked);
    }

    const int sizes_all[3][2] = {{32,24},{128,96},{512,384}};
    int zlo = 0, zhi = 2;
    if (argc > 4) { zlo = atoi(argv[4]); zhi = argc > 5 ? atoi(argv[5]) : zlo; }
    const int (*sizes)[2] = sizes_all;
    const int sweep[] = {2,3,4,6,8,12,16,24,32,48,64,96,128,192,256,384,512,768,1024,2048,4096};
    const int nsw = (int)(sizeof(sweep)/sizeof(sweep[0]));

    for (int z = zlo; z <= zhi; z++) {
        int W = sizes[z][0], H = sizes[z][1], N = W*H;
        Map m; m.w = W; m.h = H; m.blocked = malloc(N);
        int sx=1, sy=1, gx=W-2, gy=H-2, attempts = 0;
        unsigned int base_seed = 20261126u + (unsigned int)W*7u + (unsigned int)H;
        long blocked_cnt;
        do {
            rng_state = base_seed + (unsigned int)attempts;
            attempts++;
            if (!strcmp(mode, "pillars")) {
                blocked_cnt = gen_pillars(&m, 6, 2, 850, 0);
            } else if (!strcmp(mode, "dense")) {
                blocked_cnt = gen_pillars(&m, 3, 1, 950, 0);
            } else if (!strcmp(mode, "denseband")) {
                blocked_cnt = gen_pillars(&m, 3, 1, 950, 12);
            } else if (!strcmp(mode, "walls")) {
                blocked_cnt = gen_walls(&m, 5, 3, 0);
            } else if (!strcmp(mode, "pinball")) {
                blocked_cnt = gen_walls(&m, 4, 1, 1);
            } else if (!strcmp(mode, "snake")) {
                blocked_cnt = gen_snake(&m, 6, 2);
            } else if (!strcmp(mode, "zigzag")) {
                blocked_cnt = gen_zigzag(&m, 4, 10);
            } else if (!strcmp(mode, "lanes")) {
                blocked_cnt = gen_lanes(&m, 2, 3);
            } else if (!strcmp(mode, "corridor")) {
                blocked_cnt = gen_corridor(&m, 1, 2);
            } else if (!strcmp(mode, "staircase")) {
                blocked_cnt = gen_staircase(&m, 3, 3);
            } else if (!strcmp(mode, "stair2")) {
                blocked_cnt = gen_staircase(&m, 2, 12);
            } else if (!strcmp(mode, "comb")) {
                blocked_cnt = gen_comb(&m, 6);
            } else if (!strcmp(mode, "rooms")) {
                blocked_cnt = gen_rooms(&m, 10, 3);
            } else if (!strcmp(mode, "forest")) {
                blocked_cnt = gen_pillars(&m, 4, 1, 1000, 0);
            } else if (!strcmp(mode, "random42")) {
                blocked_cnt = gen_random(&m, 0.42, sx, sy, gx, gy);
            } else if (!strcmp(mode, "random35") || !strcmp(mode, "random40") ||
                       !strcmp(mode, "random45")) {
                double dens = atof(mode + 6) / 100.0;
                blocked_cnt = gen_random(&m, dens, sx, sy, gx, gy);
            } else {
                blocked_cnt = gen_random(&m, 0.25, sx, sy, gx, gy);
            }
            clear_endpoints(&m, sx, sy, gx, gy);
            int stochastic = (strncmp(mode, "random", 6) == 0) ||
                             !strcmp(mode, "pillars") || !strcmp(mode, "dense") ||
                             !strcmp(mode, "denseband") || !strcmp(mode, "pinball") ||
                             !strcmp(mode, "zigzag");
            if (reachable(&m, midx(&m,sx,sy), midx(&m,gx,gy))) break;
            if (!stochastic || attempts >= 200) {
                fprintf(stderr, "generator %s produced a disconnected map at %dx%d; pick another mode/params\n", mode, W, H);
                free(m.blocked);
                blocked_cnt = -1;
                break;
            }
        } while (1);
        if (blocked_cnt < 0) continue;

        int *raw = malloc(N * sizeof(int)), *oa = malloc(N * sizeof(int)), *ob = malloc(N * sizeof(int));
        int n = astar(&m, sx, sy, gx, gy, raw, N);
        if (n <= 0) { fprintf(stderr, "astar failed at %dx%d\n", W, H); return 3; }
        int bends = count_bends(&m, raw, n);
        double *dfield = clearance_field(&m);
        double t0, t1;

        /* local occlusion diagnostic: fraction of bend anchors whose i+2 is
         * not visible — this is the rate at which the forward probe dies. */
        int local_total = 0, local_fail = 0;
        for (int t = 0; t + 2 < n; t++) {
            local_total++;
            los_calls = 0;
            if (!losp(&m, raw[t], raw[t+2])) local_fail++;
        }
        double local_rate = local_total ? (double)local_fail / local_total : 0.0;

        los_calls = 0; t0 = now_sec();
        int ma = smooth_a(&m, raw, n, oa);
        t1 = now_sec();
        long pa = los_calls;
        double us_a = (t1 - t0) * 1e6;
        double ca = path_clearance(&m, oa, ma, dfield);
        int ok_a = segments_clear(&m, oa, ma) && oa[0]==raw[0] && oa[ma-1]==raw[n-1];

        double cr = path_clearance(&m, raw, n, dfield);
        double reach_a = avg_reach(&m, raw, n, oa, ma);

        printf("\n=== %dx%d mode=%s los=%s (attempts=%d, blocked=%.1f%%) raw_n=%d bends=%d (%.1f%%) raw_clear=%.3f\n",
               W, H, mode, losm, attempts, 100.0*blocked_cnt/N, n, bends, 100.0*bends/n, cr);
        printf("  local los(i,i+2) fail rate = %.3f (%d/%d)\n", local_rate, local_fail, local_total);
        printf("  A: m=%d r=%.4f probes=%ld  [n+m-3=%d] reach=%.3f clear=%.3f valid=%d\n",
               ma, (double)(n-ma)/n, pa, n+ma-3, reach_a, ca, ok_a);
        printf("  check: P_A == n+m-3 ? %ld == %d -> %s\n",
               pa, n+ma-3, pa==n+ma-3 ? "yes":"no");

        fprintf(fb, "%dx%d,%d,%d,%.2f,%d,%d,A,%d,%.4f,%ld,%.4f,%.1f,%.4f\n",
                W,H,W,H,100.0*blocked_cnt/N,n,bends,ma,(double)(n-ma)/n,pa,ca,us_a,reach_a);
        int mb; long pb; double cb; int ok_b;
        los_calls = 0; t0 = now_sec();
        mb = smooth_b(&m, raw, n, ob, n);
        t1 = now_sec();
        pb = los_calls;
        cb = path_clearance(&m, ob, mb, dfield);
        ok_b = segments_clear(&m, ob, mb) && ob[0]==raw[0] && ob[mb-1]==raw[n-1];
        double reach_b = avg_reach(&m, raw, n, ob, mb);
        printf("  B: m=%d r=%.4f probes=%ld reach=%.3f clear=%.3f valid=%d usec=%.1f\n",
               mb, (double)(n-mb)/n, pb, reach_b, cb, ok_b, (t1-t0)*1e6);
        fprintf(fb, "%dx%d,%d,%d,%.2f,%d,%d,B,%d,%.4f,%ld,%.4f,%.1f,%.4f\n",
                W,H,W,H,100.0*blocked_cnt/N,n,bends,mb,(double)(n-mb)/n,pb,cb,(t1-t0)*1e6,reach_b);

        /* horizon sweep: probe-start knob for the backward variant */
        int *os = malloc(N * sizeof(int));
        for (int s = 0; s < nsw; s++) {
            int S = sweep[s];
            if (S < 2 || S > n - 1) continue;
            los_calls = 0;
            int ms = smooth_b(&m, raw, n, os, S);
            double cs = path_clearance(&m, os, ms, dfield);
            fprintf(fs, "%dx%d,%d,%d,%d,%d,%.4f,%ld,%.4f\n",
                    W,H,W,H,S,ms,(double)(n-ms)/n,los_calls,cs);
        }
        /* full-B endpoint for the curve */
        fprintf(fs, "%dx%d,%d,%d,%d,%d,%.4f,%ld,%.4f\n",
                W,H,W,H,n-1,mb,(double)(n-mb)/n,pb,cb);
        free(os);

        free(raw); free(oa); free(ob); free(dfield); free(m.blocked);
    }

    fclose(fb); fclose(fs);
    printf("\ncsv written to %s/smooth_bench_%s.csv and %s/smooth_sweep_%s.csv\n", outdir, tag, outdir, tag);
    return 0;
}
