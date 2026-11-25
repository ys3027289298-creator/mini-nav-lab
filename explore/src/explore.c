/* explore: fog-aware frontier eligibility and target-reselection hysteresis.
 *
 * The failure mode fixed here is admitting cells that the actor has never
 * explored.  A target is selectable only when BOTH conditions hold:
 *
 *   selectable(t) = explored[t] && reachable_through_known_fog(actor, t)
 *
 * The reachability flood itself traverses only explored, unblocked cells, so an
 * unknown cell can never be used as a shortcut around the first condition.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#define W 16
#define H 12
#define N (W * H)

#define STEP_CELLS_PER_FRAME 1.0
/* docs/summary.json: first common curve zero is 1.25; half-cell margin is 1.5. */
#define RESELECT_MARGIN_CELLS 1.5

typedef struct {
    unsigned char blocked[N];
    unsigned char explored[N];
} FogMap;

static int idx(int x, int y) { return y * W + x; }

static int inside(int x, int y) {
    return 0 <= x && x < W && 0 <= y && y < H;
}

static int explored_cell(const FogMap* m, int t) {
    return 0 <= t && t < N && m->explored[t];
}

static int known_free(const FogMap* m, int x, int y) {
    return inside(x, y) && m->explored[idx(x, y)] && !m->blocked[idx(x, y)];
}

/* BFS over known free cells only.  Unknown fog is not traversable. */
static int reachable_known(const FogMap* m, int sx, int sy, int tx, int ty) {
    if (!known_free(m, sx, sy) || !known_free(m, tx, ty)) return 0;

    int start = idx(sx, sy), goal = idx(tx, ty);
    int queue[N], head = 0, tail = 0;
    unsigned char seen[N];
    memset(seen, 0, sizeof(seen));

    queue[tail++] = start;
    seen[start] = 1;

    static const int dx[4] = {1, -1, 0, 0};
    static const int dy[4] = {0, 0, 1, -1};

    while (head < tail) {
        int cur = queue[head++];
        if (cur == goal) return 1;

        int x = cur % W, y = cur / W;
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (!known_free(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (seen[ni]) continue;
            seen[ni] = 1;
            queue[tail++] = ni;
        }
    }
    return 0;
}

/* The requested two-clause predicate.  The reachability clause includes the
 * blocked check because a blocked target cannot be reached. */
static int target_selectable(const FogMap* m, int ax, int ay, int tx, int ty) {
    int t = idx(tx, ty);
    if (!explored_cell(m, t)) return 0;                  /* clause 1 */
    return reachable_known(m, ax, ay, tx, ty);           /* clause 2 */
}

/* Explored free cell touching still-unexplored terrain. */
static int explored_frontier(const FogMap* m, int tx, int ty) {
    if (!known_free(m, tx, ty)) return 0;
    static const int dx[4] = {1, -1, 0, 0};
    static const int dy[4] = {0, 0, 1, -1};
    for (int d = 0; d < 4; d++) {
        int nx = tx + dx[d], ny = ty + dy[d];
        if (inside(nx, ny) && !m->explored[idx(nx, ny)]) return 1;
    }
    return 0;
}

/* Full candidate-set predicate: boundary AND explored AND reachable. */
static int frontier_candidate(const FogMap* m, int ax, int ay, int tx, int ty) {
    return explored_frontier(m, tx, ty) &&
           target_selectable(m, ax, ay, tx, ty);
}

/* Cost hysteresis.  Lower cost is better.  A challenger must improve by a
 * strict margin greater than theta before replacing the incumbent. */
static int cost_reselect_required(int current_selectable,
                                  double current_cost,
                                  int have_challenger,
                                  double challenger_cost,
                                  double theta) {
    if (!current_selectable) return 1;
    if (!have_challenger) return 0;
    return challenger_cost + theta < current_cost;
}

/* Positional deadband form, for code whose threshold is a target displacement
 * rather than a score margin.  Switch only when the new target is farther than
 * rho from the held target. */
static int position_reselect_required(double old_x, double old_y,
                                      double new_x, double new_y,
                                      double rho) {
    double dx = new_x - old_x, dy = new_y - old_y;
    return hypot(dx, dy) > rho;
}

static int count_reversals(const int* moves, int n) {
    int reversals = 0;
    for (int i = 1; i < n; i++) {
        if (moves[i - 1] != 0 && moves[i] == -moves[i - 1]) reversals++;
    }
    return reversals;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    FogMap m;
    memset(&m, 0, sizeof(m));

    /* A fully open true map, but only the left chamber is explored. */
    for (int y = 0; y < H; y++) {
        for (int x = 0; x <= 5; x++) m.explored[idx(x, y)] = 1;
    }

    check("explored_same_region_selectable",
          target_selectable(&m, 1, 6, 4, 6) == 1);
    check("unknown_open_cell_rejected",
          target_selectable(&m, 1, 6, 9, 6) == 0);
    check("unknown_frontier_candidate_rejected",
          frontier_candidate(&m, 1, 6, 9, 6) == 0);

    /* (5,6) is explored and touches unexplored (6,6): valid boundary. */
    check("explored_edge_is_frontier", explored_frontier(&m, 5, 6) == 1);
    check("explored_edge_candidate",
          frontier_candidate(&m, 1, 6, 5, 6) == 1);

    /* Explore the right chamber too, but leave an unexplored wall-column gap.
     * True cells in column 8 are open; fog must still make (10,6) unreachable. */
    for (int y = 0; y < H; y++) {
        for (int x = 9; x < W; x++) m.explored[idx(x, y)] = 1;
    }
    check("explored_but_fog_separated_rejected",
          target_selectable(&m, 1, 6, 10, 6) == 0);

    /* Once the connecting columns are explored, reachability succeeds. */
    for (int y = 0; y < H; y++) {
        for (int x = 6; x <= 8; x++) m.explored[idx(x, y)] = 1;
    }
    check("bridge_explored_reachable",
          target_selectable(&m, 1, 6, 10, 6) == 1);

    /* A blocked explored cell fails the reachability clause. */
    FogMap wall;
    memset(&wall, 0, sizeof(wall));
    for (int i = 0; i < N; i++) wall.explored[i] = 1;
    for (int y = 0; y < H; y++) wall.blocked[idx(8, y)] = 1;
    check("blocked_target_rejected",
          target_selectable(&wall, 1, 6, 8, 6) == 0);
    check("full_wall_unreachable",
          target_selectable(&wall, 1, 6, 10, 6) == 0);

    /* Threshold must exceed one frame of movement. */
    check("margin_above_step", RESELECT_MARGIN_CELLS > STEP_CELLS_PER_FRAME);

    /* A one-cell cost improvement must not break a 1.5-cell hysteresis margin;
     * a three-cell improvement does. */
    check("small_gap_keeps_incumbent",
          cost_reselect_required(1, 10.0, 1, 9.0, RESELECT_MARGIN_CELLS) == 0);
    check("large_gap_reselects",
          cost_reselect_required(1, 10.0, 1, 7.0, RESELECT_MARGIN_CELLS) == 1);
    check("invalid_incumbent_forces_reselect",
          cost_reselect_required(0, 10.0, 1, 9.9, RESELECT_MARGIN_CELLS) == 1);

    /* Positional deadband: adjacent one-cell target jitter is ignored by a
     * 1.5-cell radius; a three-cell jump is accepted. */
    check("adjacent_jitter_ignored",
          position_reselect_required(5, 6, 6, 6, RESELECT_MARGIN_CELLS) == 0);
    check("far_target_change_accepted",
          position_reselect_required(5, 6, 8, 6, RESELECT_MARGIN_CELLS) == 1);

    /* The replayed two-forward/one-back signature is detected by the metric. */
    const int noisy[] = {1, 1, -1, 1, 1, -1};
    check("replay_detects_reversals", count_reversals(noisy, 6) == 3);
    const int stable[] = {1, 1, 1, 1, 1, 1};
    check("stable_trace_zero_reversals", count_reversals(stable, 6) == 0);

    printf("threshold_cells=%.2f step_cells=%.1f\n",
           RESELECT_MARGIN_CELLS, STEP_CELLS_PER_FRAME);
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
