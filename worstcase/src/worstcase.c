/* worstcase: maps that inflate the A* open list, and a perturbation cure.
 *
 * Bloat condition: A* must expand every node with f = g + h < C*, and a
 * depth-blind tie rule also expands the whole f = C* plateau.  When many
 * nodes share one estimate and that estimate is below the optimal cost,
 * expansions grow with the size of the tied region, not the path length.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static int W, H, N;
static unsigned char *blocked;
static int SX, SY, GX, GY;

static int idx(int x, int y) { return y * W + x; }

static int walkable(int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !blocked[idx(x, y)];
}

static int manh(int x, int y) { return abs(GX - x) + abs(GY - y); }

typedef struct { long expansions; int cost; } Result;

/* A* with inflated heuristic f = g + (1+eps)*h.  Ties break toward
 * smaller g (shallow first): the depth-blind order that maximises
 * plateau expansion at eps = 0. */
static Result astar(double eps) {
    Result r = {0, -1};
    if (!walkable(SX, SY) || !walkable(GX, GY)) return r;
    int *g = malloc((size_t)N * sizeof(int));
    double *f = malloc((size_t)N * sizeof(double));
    unsigned char *closed = calloc((size_t)N, 1);
    int cap = 4 * N + 64, nopen = 0;
    int *open = malloc((size_t)cap * sizeof(int));
    for (int i = 0; i < N; i++) g[i] = 1 << 29;
    int s = idx(SX, SY), t = idx(GX, GY);
    g[s] = 0;
    f[s] = (1.0 + eps) * manh(SX, SY);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) {
            int a = open[i], b = open[best];
            if (f[a] < f[b] - 1e-9) best = i;
            else if (f[a] < f[b] + 1e-9 && g[a] < g[b]) best = i;
        }
        int cur = open[best];
        open[best] = open[--nopen];
        if (closed[cur]) continue;
        if (cur == t) { r.cost = g[cur]; break; }
        closed[cur] = 1;
        r.expansions++;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(nx, ny)) continue;
            int ni = idx(nx, ny);
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + (1.0 + eps) * manh(nx, ny);
                closed[ni] = 0; /* allow re-open: (1+eps)*h is not consistent */
                if (nopen == cap) {
                    cap *= 2;
                    open = realloc(open, (size_t)cap * sizeof(int));
                }
                open[nopen++] = ni;
            }
        }
    }
    free(g); free(f); free(closed); free(open);
    return r;
}

static void clear_map(int w, int h) {
    W = w; H = h; N = W * H;
    free(blocked);
    blocked = malloc((size_t)N);
    memset(blocked, 1, N); /* all wall, then carve */
}

static void carve(int x, int y) { blocked[idx(x, y)] = 0; }

static int count_free(void) {
    int n = 0;
    for (int i = 0; i < N; i++) n += !blocked[i];
    return n;
}

/* comb parameters, kept for the model check */
static int cL, cD, cS, cNB, cFree;

/* Map A "comb_trap": corridor of length L with dead-end branches of
 * depth d every s columns on both sides.  The goal hides behind a wall
 * at the corridor end; the only route detours over the top, so
 * C* = L + 2d + 6 exceeds every comb cell's f and the whole comb is
 * expanded before the goal is reached. */
static void build_comb(void) {
    cL = 60; cD = 6; cS = 2;
    clear_map(cL + 8, 2 * cD + 8);
    int cy = cD + 4;
    for (int x = 0; x < cL; x++) carve(x, cy);
    cNB = 0;
    for (int x = 1; x < cL - 1; x += cS) {
        for (int j = 1; j <= cD; j++) { carve(x, cy - j); carve(x, cy + j); }
        cNB++;
    }
    /* detour over the top: up at x=L-1, across, down at x=L+2 */
    for (int y = cy - cD - 2; y < cy; y++) carve(cL - 1, y);
    for (int x = cL - 1; x <= cL + 2; x++) carve(x, cy - cD - 2);
    for (int y = cy - cD - 2; y <= cy; y++) carve(cL + 2, y);
    SX = 0; SY = cy; GX = cL + 2; GY = cy;
    cFree = count_free();
}

/* Map B "plateau": open field, start and goal at opposite corners.
 * Every node's f equals D = W+H-2: the whole grid is one tie plateau. */
static void build_plateau(void) {
    clear_map(64, 64);
    memset(blocked, 0, N);
    SX = 0; SY = 0; GX = W - 1; GY = H - 1;
}

/* Map C "folded_tunnel": a solid block crossed by a serpentine tunnel.
 * The short route (around the block) must climb in h; the long route
 * (through the tunnel) stays h-smooth.  The goal sits in a pocket whose
 * only entrance is on the short route's side, so the long route cannot
 * improve the goal's g from behind: once inflation lets the tunnel
 * frontier deliver the goal before the short route's h-bottleneck is
 * expanded, the returned path is suboptimal. */
static void build_tunnel(void) {
    clear_map(51, 61);
    for (int x = 5; x <= 45; x++)
        for (int y = 38; y <= 55; y++) carve(x, y);
    for (int x = 20; x <= 36; x++)
        for (int y = 42; y <= 50; y++) blocked[idx(x, y)] = 1;
    /* serpentine: rows y=48,46,44 joined by columns x=35 and x=20 */
    for (int x = 20; x <= 35; x++) carve(x, 48);
    for (int y = 46; y <= 48; y++) carve(35, y);
    for (int x = 20; x <= 35; x++) carve(x, 46);
    for (int y = 44; y <= 46; y++) carve(20, y);
    for (int x = 20; x <= 36; x++) carve(x, 44);
    /* goal pocket: column x=40 sealed by walls at x=39 and x=41,
     * open only at the bottom (y=55) */
    for (int y = 38; y <= 54; y++) { blocked[idx(39, y)] = 1; blocked[idx(41, y)] = 1; }
    blocked[idx(40, 38)] = 1;
    SX = 10; SY = 48; GX = 40; GY = 40;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

typedef void (*Builder)(void);
typedef struct { const char *name; const char *features; Builder build; int cstar; } MapDef;

static const double EPS[] = {0, 0.01, 0.05, 0.1, 0.2, 0.5, 1, 1.5, 2, 3, 4, 5};
#define NE ((int)(sizeof(EPS) / sizeof(EPS[0])))
#define NM 3

int main(void) {
    MapDef maps[NM] = {
        {"comb_trap", "L=60 depth=6 spacing=2 corridor+dead-ends", build_comb, 78},
        {"plateau", "64x64 open field corner-to-corner", build_plateau, 126},
        {"folded_tunnel", "serpentine tunnel vs short detour, goal in pocket", build_tunnel, 52},
    };
    static long exps[NM][16];
    static int costs[NM][16];
    static long half[NM];
    int base[NM];

    for (int m = 0; m < NM; m++) {
        maps[m].build();
        int free_cells = count_free();
        half[m] = (long)W * H / 2;
        printf("map=%s size=%dx%d free=%d half=%ld features=%s\n",
               maps[m].name, W, H, free_cells, half[m], maps[m].features);

        if (m == 0) {
            /* every dead-end entrance carries the same estimate */
            int cy = cD + 4, f0 = -1, equal = 1;
            printf("comb_entrance_f:");
            for (int x = 1; x < cL - 1; x += cS) {
                int fe = (x + 1) + manh(x, cy - 1);
                if (f0 < 0) f0 = fe;
                if (fe != f0) equal = 0;
                if (x == 1 || x == 15 || x == 29 || x == 43 || x == 57)
                    printf(" %d", fe);
            }
            printf("\n");
            check("comb_entrance_f_equal", equal);
            /* E = L + 2d*nb + (2d+7) ~ L*(1+2d/s) = L + N_de*d */
            printf("comb_model E=L(1+2d/s)=%.0f exact_free=%d\n",
                   cL * (1.0 + 2.0 * cD / cS), cFree);
            /* aspect bound from E <= W*H/2 */
            double lmax = (W * H / 2.0 - (2 * cD + 7)) / (1.0 + 2.0 * cD / cS);
            printf("comb_aspect Lmax=%.0f L=%d\n", lmax, cL);
            check("comb_within_aspect", cL <= lmax);
        }

        for (int i = 0; i < NE; i++) {
            Result r = astar(EPS[i]);
            exps[m][i] = r.expansions;
            costs[m][i] = r.cost;
            if (i == 0) base[m] = r.cost;
            printf("  eps=%.2f expansions=%ld cost=%d ratio=%.3f\n",
                   EPS[i], r.expansions, r.cost,
                   r.cost > 0 ? (double)r.cost / base[m] : 0.0);
        }
        check(m == 0 ? "comb_cost_optimal" : m == 1 ? "plateau_cost_optimal"
                                                    : "tunnel_cost_optimal",
              base[m] == maps[m].cstar);
    }

    /* comb: at eps=0 every free cell but the goal is expanded */
    check("comb_E0_eq_free", exps[0][0] == cFree - 1);
    /* plateau: at eps=0 the tie rule expands the whole grid */
    check("plateau_E0_full", exps[1][0] == (long)64 * 64 - 1);

    /* pick the largest eps that keeps the optimal cost on all maps */
    int star = 0;
    for (int i = NE - 1; i >= 0; i--) {
        int ok = 1;
        for (int m = 0; m < NM; m++) if (costs[m][i] != base[m]) ok = 0;
        if (ok) { star = i; break; }
    }
    printf("eps_star=%.2f\n", EPS[star]);
    check("eps_star_positive", EPS[star] > 0);

    /* acceptance: one run per map at eps_star, expansions < half the grid */
    for (int m = 0; m < NM; m++) {
        int pass = exps[m][star] < half[m] && costs[m][star] == base[m];
        printf("final map=%s eps=%.2f expansions=%ld half=%ld cost=%d pass=%d features=%s\n",
               maps[m].name, EPS[star], exps[m][star], half[m],
               costs[m][star], pass, maps[m].features);
        if (!pass) failures++;
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
