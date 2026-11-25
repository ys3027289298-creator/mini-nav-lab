
/* grid: walkability map, 4-neighbour A*, and explicit edge-case behaviour. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define W 16
#define H 12
#define NODE_CAP (W*H)
#define INF (1 << 29)

/* Larger than any simple path on this grid, so one blocked crossing dominates
   ordinary walking steps in the reverse projection distance. */
#define BLOCK_CROSS_COST 1024
#define FALLBACK_FAR_STEPS 18

typedef struct { unsigned char blocked[W*H]; } Map;

enum find_status {
    /* Non-negative statuses mean that out[] contains usable cells. */
    FIND_OK = 0,
    FIND_NEAREST = 1 << 0,     /* goal was disconnected; out[] ends at fallback */
    FIND_TRUNCATED = 1 << 1,   /* out[] is only the prefix that cap can hold */
    FIND_MISDIRECTED = 1 << 2,  /* fallback endpoint is behind start vs. goal */
    FIND_FAR = 1 << 3,         /* reverse path-distance cost exceeds policy budget */

    /* Negative statuses are errors and no path cell is promised. */
    FIND_E_ARG = -1,
    FIND_E_START_BLOCKED = -2,
    FIND_E_GOAL_BLOCKED = -3
};

typedef struct {
    int status;
    int length;            /* number of valid cells written to out[] */
    int required_length;   /* complete start-to-endpoint vertex count */
    int start;
    int goal;
    int nearest;           /* selected path-distance projection of goal */
    int endpoint;          /* last cell actually written (may be a truncation prefix) */
    int reverse_weighted;  /* lexicographic projection cost encoded as cross*1024+steps */
    int blocked_crossings;
    int reverse_steps;
} FindResult;

static int idx(int x, int y) { return y * W + x; }

static int in_bounds(int x, int y) {
    return 0 <= x && x < W && 0 <= y && y < H;
}

static int walkable(const Map* m, int x, int y) {
    if (!in_bounds(x, y)) return 0;
    return !m->blocked[idx(x, y)];
}

/* A* with unit 4-neighbour edges. On success rev[] is goal-to-start and n is
   its vertex count. Returns 0 when the reachable component is exhausted. */
static int astar_reverse(const Map* m, int sx, int sy, int gx, int gy,
                         int* rev, int* nout) {
    int g[NODE_CAP], f[NODE_CAP], closed[NODE_CAP], parent[NODE_CAP];
    for (int i = 0; i < NODE_CAP; i++) {
        g[i] = INF;
        f[i] = INF;
        closed[i] = 0;
        parent[i] = -1;
    }

    int open[NODE_CAP], nopen = 0;
    int s = idx(sx, sy), goal = idx(gx, gy);
    g[s] = 0;
    f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;

    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) {
            if (f[open[i]] < f[open[best]]) best = i;
        }
        int cur = open[best];
        open[best] = open[--nopen];

        if (cur == goal) {
            int n = 0, c = cur;
            while (c != -1 && n < NODE_CAP) {
                rev[n++] = c;
                c = parent[c];
            }
            *nout = n;
            return 1;
        }

        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;

            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;

                int found = 0;
                for (int i = 0; i < nopen; i++) {
                    if (open[i] == ni) { found = 1; break; }
                }
                if (!found) open[nopen++] = ni;
            }
        }
    }

    return 0;
}

static void bfs_component(const Map* m, int start, int* dist, int* parent) {
    int queue[NODE_CAP], head = 0, tail = 0;
    for (int i = 0; i < NODE_CAP; i++) {
        dist[i] = INF;
        parent[i] = -1;
    }

    dist[start] = 0;
    queue[tail++] = start;

    while (head < tail) {
        int cur = queue[head++];
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};

        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (dist[ni] != INF) continue;
            dist[ni] = dist[cur] + 1;
            parent[ni] = cur;
            queue[tail++] = ni;
        }
    }
}

/* Reverse projection distance. Ordinary legal path distance from the goal to
   another component is infinite, so this Dijkstra makes a declared extension:
   entering a blocked cell costs BLOCK_CROSS_COST. The resulting label is
   lexicographic in (blocked crossings, ordinary steps), giving an obstacle-aware
   "path-distance nearest reachable point" rather than a Euclidean guess. */
static void reverse_projection(const Map* m, int goal, int* rd) {
    int used[NODE_CAP] = {0};
    for (int i = 0; i < NODE_CAP; i++) rd[i] = INF;
    rd[goal] = 0;

    for (int iter = 0; iter < NODE_CAP; iter++) {
        int cur = -1;
        for (int i = 0; i < NODE_CAP; i++) {
            if (!used[i] && (cur < 0 || rd[i] < rd[cur])) cur = i;
        }
        if (cur < 0 || rd[cur] == INF) break;
        used[cur] = 1;

        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!in_bounds(nx, ny)) continue;
            int ni = idx(nx, ny);
            int step_cost = m->blocked[ni] ? BLOCK_CROSS_COST : 1;
            if (rd[cur] + step_cost < rd[ni]) rd[ni] = rd[cur] + step_cost;
        }
    }
}

static FindResult make_error(int status, int sx, int sy, int gx, int gy) {
    FindResult r;
    memset(&r, 0, sizeof(r));
    r.status = status;
    r.length = 0;
    r.required_length = 0;
    r.start = in_bounds(sx, sy) ? idx(sx, sy) : -1;
    r.goal = in_bounds(gx, gy) ? idx(gx, gy) : -1;
    r.nearest = -1;
    r.endpoint = -1;
    return r;
}

/* Documented find_path contract; see grid/README.md. */
static FindResult find_path(const Map* m, int sx, int sy, int gx, int gy,
                           int* out, int cap) {
    if (!m || !out || cap <= 0) {
        return make_error(FIND_E_ARG, sx, sy, gx, gy);
    }
    if (!walkable(m, sx, sy)) {
        return make_error(FIND_E_START_BLOCKED, sx, sy, gx, gy);
    }
    if (!walkable(m, gx, gy)) {
        return make_error(FIND_E_GOAL_BLOCKED, sx, sy, gx, gy);
    }

    FindResult result;
    memset(&result, 0, sizeof(result));
    result.start = idx(sx, sy);
    result.goal = idx(gx, gy);
    result.nearest = -1;
    result.endpoint = -1;

    int rev[NODE_CAP], n = 0;
    if (astar_reverse(m, sx, sy, gx, gy, rev, &n)) {
        int take = n < cap ? n : cap;
        for (int i = 0; i < take; i++) out[i] = rev[n - 1 - i];

        result.required_length = n;
        result.length = take;
        result.endpoint = out[take - 1];
        result.nearest = result.goal;
        result.status = (n <= cap) ? FIND_OK : FIND_TRUNCATED;
        return result;
    }

    /* Legal endpoints in different connected components. */
    int dist[NODE_CAP], parent[NODE_CAP], rd[NODE_CAP];
    bfs_component(m, result.start, dist, parent);
    reverse_projection(m, result.goal, rd);

    int best = -1;
    for (int cell = 0; cell < NODE_CAP; cell++) {
        if (dist[cell] == INF) continue;
        if (best < 0 || rd[cell] < rd[best]) best = cell;
    }

    /* start itself is always in this component, so best cannot remain -1. */
    result.nearest = best;
    result.reverse_weighted = rd[best];
    result.blocked_crossings = rd[best] / BLOCK_CROSS_COST;
    result.reverse_steps = rd[best] % BLOCK_CROSS_COST;

    int vx = best % W, vy = best / W;
    int dot = (vx - sx) * (gx - sx) + (vy - sy) * (gy - sy);
    int status = FIND_NEAREST;
    if (dot < 0) status |= FIND_MISDIRECTED;
    if (result.reverse_steps > FALLBACK_FAR_STEPS) status |= FIND_FAR;

    int fallback_rev[NODE_CAP], mlen = 0, c = best;
    while (c != -1 && mlen < NODE_CAP) {
        fallback_rev[mlen++] = c;
        c = parent[c];
    }

    int take = mlen < cap ? mlen : cap;
    for (int i = 0; i < take; i++) {
        out[i] = fallback_rev[mlen - 1 - i];
    }

    result.required_length = mlen;
    result.length = take;
    result.endpoint = out[take - 1];
    if (mlen > cap) status |= FIND_TRUNCATED;
    result.status = status;
    return result;
}

static int valid_path(const Map* m, const int* p, int n) {
    if (n <= 0) return 0;
    for (int i = 0; i < n; i++) {
        int x = p[i] % W, y = p[i] / W;
        if (!walkable(m, x, y)) return 0;
        if (i > 0) {
            int px = p[i - 1] % W, py = p[i - 1] / W;
            if (abs(x - px) + abs(y - py) != 1) return 0;
        }
    }
    return 1;
}

/* Test oracle: Euclidean-nearest cell in the start connected component. */
static int euclidean_nearest_in_start_component(const Map* m,
                                                int sx, int sy,
                                                int gx, int gy) {
    int dist[NODE_CAP], parent[NODE_CAP];
    int s = idx(sx, sy);
    bfs_component(m, s, dist, parent);

    int best = -1, best_d2 = INF;
    for (int cell = 0; cell < NODE_CAP; cell++) {
        if (dist[cell] == INF) continue;
        int x = cell % W, y = cell / W;
        int d2 = (x - gx) * (x - gx) + (y - gy) * (y - gy);
        if (d2 < best_d2) {
            best_d2 = d2;
            best = cell;
        }
    }
    return best;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map open;
    memset(&open, 0, sizeof(open));
    int path[NODE_CAP];

    /* Case 1/4 leaf: legal, connected, and complete path fits cache/capacity. */
    FindResult ok_case = find_path(&open, 0, 0, 2, 2, path, NODE_CAP);
    int reachable_fit =
        ok_case.status == FIND_OK &&
        ok_case.length == 5 &&
        ok_case.required_length == 5 &&
        path[0] == idx(0, 0) &&
        path[ok_case.length - 1] == idx(2, 2) &&
        valid_path(&open, path, ok_case.length);
    check("case_reachable_fit", reachable_fit);

    /* Disconnected map: full wall x=8, with a two-cell-thick patch at y=4..6.
       Euclidean chooses (7,5); reverse path-distance chooses thin crossing (7,3). */
    Map split;
    memset(&split, 0, sizeof(split));
    for (int y = 0; y < H; y++) split.blocked[idx(8, y)] = 1;
    split.blocked[idx(9, 4)] = 1;
    split.blocked[idx(9, 5)] = 1;
    split.blocked[idx(9, 6)] = 1;

    int euclid = euclidean_nearest_in_start_component(&split, 0, 5, 10, 5);
    FindResult near_case = find_path(&split, 0, 5, 10, 5, path, NODE_CAP);
    int unreachable_fallback =
        euclid == idx(7, 5) &&
        near_case.status == FIND_NEAREST &&
        near_case.nearest == idx(7, 3) &&
        near_case.endpoint == idx(7, 3) &&
        near_case.length == 10 &&
        near_case.required_length == 10 &&
        near_case.blocked_crossings == 1 &&
        near_case.reverse_steps == 4 &&
        path[0] == idx(0, 5) &&
        path[near_case.length - 1] == idx(7, 3) &&
        valid_path(&split, path, near_case.length);
    check("case_unreachable_path_distance_nearest", unreachable_fallback);
    printf("euclidean_projection=%d path_projection=%d\n",
           euclid, near_case.nearest);

    /* Capacity also applies to a fallback path, not only an exact path. */
    int fallback_prefix[3];
    FindResult near_capped = find_path(&split, 0, 5, 10, 5,
                                       fallback_prefix, 3);
    check("bonus_fallback_cap_combination",
          near_capped.status == (FIND_NEAREST | FIND_TRUNCATED) &&
          near_capped.length == 3 &&
          near_capped.required_length == near_case.length &&
          memcmp(fallback_prefix, path, 3 * sizeof(int)) == 0 &&
          valid_path(&split, fallback_prefix, 3));

    /* Boundary 2: start itself is blocked/out of bounds. */
    int sentinel = -777;
    path[0] = sentinel;
    FindResult start_case = find_path(&split, 8, 0, 10, 5, path, NODE_CAP);
    int blocked_start =
        start_case.status == FIND_E_START_BLOCKED &&
        start_case.length == 0 &&
        path[0] == sentinel;
    check("case_blocked_start", blocked_start);

    /* Boundary 3: complete path has seven vertices, cap holds only three.
       The result is an explicit prefix, never an apparent complete FIND_OK. */
    int full_reference[NODE_CAP];
    FindResult full = find_path(&open, 0, 0, 3, 3,
                                full_reference, NODE_CAP);
    FindResult capped = find_path(&open, 0, 0, 3, 3, path, 3);
    int over_cap =
        full.status == FIND_OK && full.length == 7 &&
        capped.status == FIND_TRUNCATED &&
        capped.length == 3 &&
        capped.required_length == 7 &&
        capped.endpoint == path[2] &&
        memcmp(path, full_reference, 3 * sizeof(int)) == 0 &&
        valid_path(&open, path, 3);
    check("case_over_limit_explicitly_truncated", over_cap);

    /* Omitted branch from the four-leaf table: legal start but invalid goal. */
    Map goal_blocked;
    memset(&goal_blocked, 0, sizeof(goal_blocked));
    goal_blocked.blocked[idx(4, 0)] = 1;
    FindResult goal_case = find_path(&goal_blocked, 0, 0, 4, 0,
                                     path, NODE_CAP);
    FindResult oob_goal = find_path(&open, 0, 0, -1, 0,
                                    path, NODE_CAP);
    int invalid_goal =
        goal_case.status == FIND_E_GOAL_BLOCKED &&
        oob_goal.status == FIND_E_GOAL_BLOCKED;
    check("uncovered_goal_blocked_or_out_of_bounds", invalid_goal);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
