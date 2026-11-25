
/* degrade: fallback chain for the five pathfinding error codes.
 *
 * Error codes (the table's pathfinding rows):
 *   E_TIMEOUT(1)     -> partial path to the best node reached in budget
 *   E_UNREACHABLE(2) -> path to the reachable cell nearest the goal
 *   E_BAD_START(3)   -> push start to nearest walkable cell, then search
 *   E_BAD_GOAL(4)    -> push goal to nearest walkable cell, then search
 *   E_TOO_LONG(5)    -> keep the full path's prefix up to the cap (truncate)
 *   E_FAIL(9)        -> no walkable cell exists, degradation impossible
 *
 * Every usable result carries a path and a closeness ratio:
 *   q   = L_d / D0   walk fraction: degraded path length over target distance
 *   rho = 1 - d(end, G) / D0   geometric fraction of the target distance closed
 * where D0 = Manhattan(S, G). For a monotone shortest-path prefix rho == q.
 * The acceptance gate is rho >= 0.5 ("the player gets at least halfway").
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W * H)
#define GATE 0.5          /* minimum closeness required to accept a fallback */
#define MISLEAD 0.5       /* a delivered path below this is counted misdirected */

#define NAV_OK 0
#define E_TIMEOUT 1
#define E_UNREACHABLE 2
#define E_BAD_START 3
#define E_BAD_GOAL 4
#define E_TOO_LONG 5
#define E_FAIL 9

typedef struct { unsigned char blocked[MAXN]; } Map;

static int idx(int x, int y) { return y * W + x; }
static int manh(int ax, int ay, int bx, int by) {
    return abs(ax - bx) + abs(ay - by);
}

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* BFS to the nearest walkable cell from (x,y); distance 0 if itself walkable.
 * Ties resolve to the smallest cell index so results are deterministic.
 * Returns 1 on success, 0 when the map has no walkable cell at all. */
static int nearest_walkable(const Map* m, int x, int y, int* ox, int* oy) {
    if (walkable(m, x, y)) { *ox = x; *oy = y; return 1; }
    int dist[MAXN];
    for (int i = 0; i < MAXN; i++) dist[i] = -1;
    int q[MAXN], head = 0, tail = 0;
    /* seed from every walkable 4-neighbour of (x,y) */
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    for (int d = 0; d < 4; d++) {
        int nx = x + dx[d], ny = y + dy[d];
        if (walkable(m, nx, ny) && dist[idx(nx, ny)] < 0) {
            dist[idx(nx, ny)] = 1;
            q[tail++] = idx(nx, ny);
        }
    }
    while (head < tail) {
        int cur = q[head++];
        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (dist[ni] >= 0) continue;
            dist[ni] = dist[cur] + 1;
            q[tail++] = ni;
        }
    }
    int best = -1;
    for (int i = 0; i < MAXN; i++) {
        if (dist[i] >= 0 && (best < 0 || dist[i] < dist[best])) best = i;
    }
    if (best < 0) return 0;
    *ox = best % W;
    *oy = best / W;
    return 1;
}

/* BFS reachability from (sx,sy): dist per cell, -1 when unreachable. */
static void reachable(const Map* m, int sx, int sy, int* dist) {
    for (int i = 0; i < MAXN; i++) dist[i] = -1;
    int q[MAXN], head = 0, tail = 0;
    dist[idx(sx, sy)] = 0;
    q[tail++] = idx(sx, sy);
    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    while (head < tail) {
        int cur = q[head++];
        int cx = cur % W, cy = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (dist[ni] >= 0) continue;
            dist[ni] = dist[cur] + 1;
            q[tail++] = ni;
        }
    }
}

/* A* with a node-expansion budget and an output cap (cell count).
 * Returns the path length in cells and writes a status:
 *   NAV_OK        goal reached, length <= cap
 *   E_TOO_LONG    goal reached, length > cap (full path still written)
 *   E_TIMEOUT     budget exhausted before the goal (best-so-far path written)
 *   E_UNREACHABLE open set exhausted without reaching the goal
 * On E_TIMEOUT the returned path ends at the expanded node with the smallest
 * heuristic distance to the goal, i.e. the deepest useful prefix. */
static int astar(const Map* m, int sx, int sy, int gx, int gy,
                 int budget, int cap, int* out, int* status) {
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1;
    }
    int open[MAXN], nopen = 0, expanded = 0;
    int s = idx(sx, sy), goal = idx(gx, gy);
    g[s] = 0;
    f[s] = manh(sx, sy, gx, gy);
    open[nopen++] = s;
    int best = s, besth = manh(sx, sy, gx, gy);

    while (nopen > 0) {
        int bi = 0;
        for (int i = 1; i < nopen; i++)
            if (f[open[i]] < f[open[bi]]) bi = i;
        int cur = open[bi];
        open[bi] = open[--nopen];
        int cx = cur % W, cy = cur / W;
        int hcur = manh(cx, cy, gx, gy);
        if (hcur < besth) { besth = hcur; best = cur; }

        if (cur == goal) {
            int rev[MAXN], n = 0, c = cur;
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            *status = (n > cap) ? E_TOO_LONG : NAV_OK;
            return n;
        }
        if (expanded >= budget) {
            int rev[MAXN], n = 0, c = best;
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            *status = E_TIMEOUT;
            return n;
        }
        expanded++;
        closed[cur] = 1;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + manh(nx, ny, gx, gy);
                parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < nopen; i++)
                    if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    *status = E_UNREACHABLE;
    return 0;
}

typedef struct {
    int code;         /* error that triggered (NAV_OK on a clean search) */
    int n;            /* returned path length in cells; 0 on hard failure */
    int moved_start;  /* start was pushed to a neighbouring walkable cell */
    int moved_goal;   /* goal was pushed likewise */
    int partial;      /* timeout prefix */
    int truncated;    /* too-long prefix cut at the cap */
} NavResult;

static const char* code_name(int code) {
    switch (code) {
        case NAV_OK: return "OK";
        case E_TIMEOUT: return "E_TIMEOUT";
        case E_UNREACHABLE: return "E_UNREACHABLE";
        case E_BAD_START: return "E_BAD_START";
        case E_BAD_GOAL: return "E_BAD_GOAL";
        case E_TOO_LONG: return "E_TOO_LONG";
        default: return "E_FAIL";
    }
}

/* Full fallback chain. Always writes a usable path when it returns 1. */
static int navigate(const Map* m, int sx, int sy, int gx, int gy,
                    int budget, int cap, int* out, NavResult* r) {
    memset(r, 0, sizeof(*r));
    r->code = NAV_OK;

    /* E_BAD_START: push start to the nearest walkable cell. */
    int esx = sx, esy = sy, egx = gx, egy = gy;
    if (!walkable(m, sx, sy)) {
        if (!nearest_walkable(m, sx, sy, &esx, &esy)) { r->code = E_FAIL; return 0; }
        r->moved_start = 1;
        r->code = E_BAD_START;
    }
    /* E_BAD_GOAL: push goal to the nearest walkable cell. */
    if (!walkable(m, gx, gy)) {
        if (!nearest_walkable(m, gx, gy, &egx, &egy)) { r->code = E_FAIL; return 0; }
        r->moved_goal = 1;
        if (r->code == NAV_OK) r->code = E_BAD_GOAL;
    }

    int status;
    int n = astar(m, esx, esy, egx, egy, budget, cap, out, &status);

    if (status == E_TIMEOUT) {
        r->partial = 1;
        if (r->code == NAV_OK) r->code = E_TIMEOUT;
    } else if (status == E_TOO_LONG) {
        /* truncate: keep the first cap cells of the full shortest path */
        r->truncated = 1;
        n = cap;
        if (r->code == NAV_OK) r->code = E_TOO_LONG;
    } else if (status == E_UNREACHABLE) {
        /* nearest reachable cell to the *original* goal */
        int dist[MAXN];
        reachable(m, esx, esy, dist);
        int tgt = -1;
        for (int i = 0; i < MAXN; i++) {
            if (dist[i] < 0) continue;
            if (tgt < 0 ||
                manh(i % W, i / W, gx, gy) < manh(tgt % W, tgt / W, gx, gy) ||
                (manh(i % W, i / W, gx, gy) == manh(tgt % W, tgt / W, gx, gy) &&
                 i < tgt))
                tgt = i;
        }
        if (tgt < 0) { r->code = E_FAIL; return 0; }
        n = astar(m, esx, esy, tgt % W, tgt / W, MAXN, MAXN, out, &status);
        if (status != NAV_OK || n <= 0) { r->code = E_FAIL; return 0; }
        if (r->code == NAV_OK) r->code = E_UNREACHABLE;
    }

    r->n = n;
    return n > 0;
}

/* Geometric closeness: fraction of the initial target distance actually closed.
 * Clipped to [0,1] so a detour prefix that heads away reads 0, not negative. */
static double closeness(int ex, int ey, int sx, int sy, int gx, int gy) {
    double d0 = manh(sx, sy, gx, gy);
    if (d0 == 0) return 1.0;
    double rho = 1.0 - (double)manh(ex, ey, gx, gy) / d0;
    if (rho < 0) rho = 0;
    if (rho > 1) rho = 1;
    return rho;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("  %s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void checkf(const char* tag, const char* suffix, int ok) {
    char name[96];
    snprintf(name, sizeof(name), "%s_%s", tag, suffix);
    check(name, ok);
}

/* Run one trigger case and evaluate the acceptance gate.
 * Returns 1 when the degraded path is usable and rho >= GATE. */
static int run_case(const char* tag, const Map* m,
                    int sx, int sy, int gx, int gy, int budget, int cap,
                    int expect_code) {
    int path[MAXN];
    NavResult r;
    int ok = navigate(m, sx, sy, gx, gy, budget, cap, path, &r);
    printf("[%s] code=%s(expected %s) usable=%d n=%d\n",
           tag, code_name(r.code), code_name(expect_code), ok, r.n);
    int code_ok = r.code == expect_code;
    if (!ok) {
        printf("  FAIL: %s degradation did not produce a usable path\n", tag);
        check(tag, 0);
        return 0;
    }
    int end = path[r.n - 1];
    int ex = end % W, ey = end / W;
    double d0 = manh(sx, sy, gx, gy);
    double q = d0 == 0 ? 1.0 : (r.n - 1) / d0;
    double rho = closeness(ex, ey, sx, sy, gx, gy);
    int gate_ok = rho >= GATE;
    printf("  endpoint=(%d,%d) walk_fraction=%.3f closeness=%.3f gate=%.2f\n",
           ex, ey, q, rho, GATE);
    checkf(tag, "code", code_ok);
    checkf(tag, "usable", ok && r.n >= 1);
    /* every cell of a degraded path must itself be walkable */
    int cells_ok = 1;
    for (int i = 0; i < r.n; i++)
        if (m->blocked[path[i]]) cells_ok = 0;
    checkf(tag, "walkable_cells", cells_ok);
    if (r.moved_start) checkf(tag, "start_pushed", 1);
    if (r.moved_goal) checkf(tag, "goal_pushed", 1);
    if (r.partial) checkf(tag, "partial", 1);
    if (r.truncated) checkf(tag, "truncated", 1);
    if (!gate_ok)
        printf("  FAIL: %s degradation took effect but closeness %.3f < %.2f\n",
               tag, rho, GATE);
    checkf(tag, "gate", gate_ok);
    return code_ok && ok && cells_ok && gate_ok;
}

/* A case expected to fall short of the gate: the harness must name it. */
static void run_diagnostic(const char* tag, const Map* m,
                           int sx, int sy, int gx, int gy, int budget, int cap) {
    int path[MAXN];
    NavResult r;
    int ok = navigate(m, sx, sy, gx, gy, budget, cap, path, &r);
    if (!ok) {
        printf("[diagnostic %s] hard failure, no path: degradation %s ineffective\n",
               tag, code_name(r.code));
        checkf(tag, "diagnosed", 1);
        return;
    }
    int end = path[r.n - 1];
    double rho = closeness(end % W, end / W, sx, sy, gx, gy);
    int flagged = rho < GATE;
    printf("[diagnostic %s] code=%s n=%d closeness=%.3f -> %s\n",
           tag, code_name(r.code), r.n, rho,
           flagged ? "below gate, flagged correctly" : "unexpectedly passes");
    if (!flagged)
        printf("  FAIL: %s should have been reported below the gate but was not\n",
               tag);
    checkf(tag, "diagnosed", flagged);
}

/* ---- deterministic PRNG for the strategy sweep ---- */
static unsigned long rng_state;
static unsigned long xrnd(void) {
    unsigned long x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x;
    return x;
}

int main(void) {
    /* ---------- the five error triggers, one construction each ---------- */

    /* E_BAD_START: start sits on a blocked cell */
    Map m1; memset(&m1, 0, sizeof(m1));
    m1.blocked[idx(2, 2)] = 1;
    run_case("bad_start", &m1, 2, 2, 10, 2, MAXN, MAXN, E_BAD_START);

    /* E_BAD_GOAL: goal sits on a blocked cell */
    Map m2; memset(&m2, 0, sizeof(m2));
    m2.blocked[idx(12, 5)] = 1;
    run_case("bad_goal", &m2, 2, 5, 12, 5, MAXN, MAXN, E_BAD_GOAL);

    /* E_UNREACHABLE: full-height wall at x=9 splits the map in two */
    Map m3; memset(&m3, 0, sizeof(m3));
    for (int y = 0; y < H; y++) m3.blocked[idx(9, y)] = 1;
    run_case("unreachable", &m3, 2, 6, 12, 6, MAXN, MAXN, E_UNREACHABLE);

    /* E_TIMEOUT: open map, budget 26 stops the search one expansion short of
     * the goal; the best-so-far prefix reaches (2,11), 13 of the 23 manhattan
     * steps (closeness 0.565, a margin above the 0.5 gate). */
    Map m4; memset(&m4, 0, sizeof(m4));
    run_case("timeout", &m4, 0, 0, 12, 11, 26, MAXN, E_TIMEOUT);

    /* E_TOO_LONG: wall column x=6 with a gap at y=0 forces a long detour.
     * Full path is 21 cells; cap 19 keeps an 18-step prefix (cut 2/20=10%),
     * ending at (7,0) with closeness 0.625, a margin above the gate. */
    Map m5; memset(&m5, 0, sizeof(m5));
    for (int y = 1; y < H; y++) m5.blocked[idx(6, y)] = 1;
    run_case("too_long", &m5, 2, 6, 10, 6, MAXN, 19, E_TOO_LONG);

    /* ---------- diagnostics: degradations that cannot meet the gate ---------- */

    /* starved timeout budget: only a handful of expansions, nowhere near half */
    run_diagnostic("timeout_starved", &m4, 0, 0, 15, 11, 3, MAXN);
    /* aggressive cap cuts during the detour, before the goalward leg */
    run_diagnostic("too_long_aggressive", &m5, 2, 6, 10, 6, MAXN, 12);
    /* wall close to the start: the nearest reachable cell stays far away */
    Map m6; memset(&m6, 0, sizeof(m6));
    for (int y = 0; y < H; y++) m6.blocked[idx(5, y)] = 1;
    run_diagnostic("unreachable_far", &m6, 1, 6, 15, 6, MAXN, MAXN);

    /* ---------- strategy sweep: effective-rate vs mislead-rate curves ------ */
    printf("\nstrategy sweep (400 random requests, fixed seed):\n");
    rng_state = 0x12345678UL;
    enum { TRIALS = 400 };
    double rho_buf[TRIALS];
    int degraded_buf[TRIALS];
    int hard_fails = 0, raw_errors = 0;
    for (int t = 0; t < TRIALS; t++) {
        Map rm; memset(&rm, 0, sizeof(rm));
        int density = 15 + (int)(xrnd() % 11);          /* 15%..25% walls */
        for (int i = 0; i < MAXN; i++)
            if ((int)(xrnd() % 100) < density) rm.blocked[i] = 1;
        int sx = (int)(xrnd() % W), sy = (int)(xrnd() % H);
        int gx = (int)(xrnd() % W), gy = (int)(xrnd() % H);
        int path[MAXN];
        NavResult r;
        int ok = navigate(&rm, sx, sy, gx, gy, 120, 60, path, &r);
        if (!ok) { hard_fails++; rho_buf[t] = -1.0; degraded_buf[t] = 0; continue; }
        if (r.code != NAV_OK) raw_errors++;
        int end = path[r.n - 1];
        rho_buf[t] = closeness(end % W, end / W, sx, sy, gx, gy);
        degraded_buf[t] = r.code != NAV_OK;
    }
    const double theta[] = {0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.8, 1.0};
    int nth = (int)(sizeof(theta) / sizeof(theta[0]));
    printf("theta  effective  mislead  mean_rho_accepted\n");
    double prev_eff = 2.0, prev_mis = 2.0;
    int mono_e = 1, mono_m = 1;
    for (int k = 0; k < nth; k++) {
        int accepted = 0, mislead = 0, acc_deg = 0;
        double sum = 0.0;
        for (int t = 0; t < TRIALS; t++) {
            if (rho_buf[t] < 0 || rho_buf[t] < theta[k]) continue;
            accepted++;
            sum += rho_buf[t];
            if (degraded_buf[t]) {
                acc_deg++;
                if (rho_buf[t] < MISLEAD) mislead++;
            }
        }
        double eff = (double)accepted / TRIALS;
        double mis = acc_deg ? (double)mislead / acc_deg : 0.0;
        double mean = accepted ? sum / accepted : 0.0;
        printf("%.1f     %.3f      %.3f    %.3f\n", theta[k], eff, mis, mean);
        if (eff > prev_eff + 1e-9) mono_e = 0;
        if (mis > prev_mis + 1e-9) mono_m = 0;
        prev_eff = eff;
        prev_mis = mis;
        if (theta[k] == GATE) {
            /* at the chosen operating point no misdirected path survives */
            check("sweep_gate_kills_mislead", mis == 0.0);
            check("sweep_gate_effective_majority", eff >= 0.5);
        }
    }
    check("sweep_effective_monotonic", mono_e);
    check("sweep_mislead_monotonic", mono_m);
    printf("raw_error_rate=%.3f hard_fail_rate=%.3f (without fallback these all fail)\n",
           (double)raw_errors / TRIALS, (double)hard_fails / TRIALS);

    printf("\nall_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
