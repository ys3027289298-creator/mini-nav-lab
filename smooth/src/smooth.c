
/* smooth: line-of-sight test and path smoothing. */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>

#define W 16
#define H 12

typedef struct { unsigned char blocked[W*H]; } Map;
static int idx(int x, int y) { return y * W + x; }

/* Name the polarity; a bool makes the two directions interchangeable at
   every call site and gives the compiler nothing to check. */
typedef enum {
    LOS_A_TO_B = 0,
    LOS_B_TO_A = 1
} LosDirection;

/* bresenham supercover: any blocked cell on the segment blocks LOS */
static int los_trace(const Map* m, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    for (;;) {
        if (x < 0 || y < 0 || x >= W || y >= H) return 0;
        if (m->blocked[idx(x, y)]) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
}

/* Direction branch expression:
   D-F: direction == LOS_A_TO_B -> trace (ax,ay) -> (bx,by)
   D-R: otherwise               -> trace (bx,by) -> (ax,ay) */
static int los(const Map* m, int ax, int ay, int bx, int by,
               LosDirection direction) {
    return direction == LOS_A_TO_B
        ? los_trace(m, ax, ay, bx, by)
        : los_trace(m, bx, by, ax, ay);
}

/* greedy smoothing: keep start, skip points while LOS holds */
static int smooth(const Map* m, const int* path, int n, int* out, int cap) {
    if (n <= 0) return -1;
    if (n == 1) { if (cap < 1) return -2; out[0] = path[0]; return 1; }
    int w = 0;
    out[w++] = path[0];
    int i = 0;
    while (i < n - 1) {
        int j = n - 1;
        while (j > i + 1) {
            if (los(m, path[i] % W, path[i] / W, path[j] % W, path[j] / W, LOS_A_TO_B)) break;
            j--;
        }
        if (w >= cap) return -2;
        out[w++] = path[j];
        i = j;
    }
    return w;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok != 0);
    failures += (ok == 0);
}

static int all_segments_clear(const Map* m, const int* out, int k) {
    for (int i = 0; i + 1 < k; i++) {
        if (!los(m, out[i] % W, out[i] / W,
                    out[i + 1] % W, out[i + 1] / W, LOS_A_TO_B))
            return 0;
    }
    return 1;
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    check("los_open", los(&m, 0, 0, 5, 0, LOS_A_TO_B) == 1);
    check("los_diag", los(&m, 0, 0, 4, 4, LOS_A_TO_B) == 1);

    Map w; memset(&w, 0, sizeof(w));
    w.blocked[idx(2, 0)] = 1;
    check("los_blocked", los(&w, 0, 0, 4, 0, LOS_A_TO_B) == 0);

    /* Direction pair: exercise both arms of the direction expression on
       the same wall. A polarity reversal cannot leave one arm untested. */
    Map d; memset(&d, 0, sizeof(d));
    d.blocked[idx(2, 3)] = 1;
    check("los_dir_a_to_b_blocked",
          los(&d, 1, 3, 4, 3, LOS_A_TO_B) == 0);
    check("los_dir_b_to_a_blocked",
          los(&d, 1, 3, 4, 3, LOS_B_TO_A) == 0);
    check("los_dir_b_to_a_open",
          los(&m, 1, 3, 4, 3, LOS_B_TO_A) == 1);

    /* Defensive boundary branches in los_trace. */
    check("los_outside_left", los(&m, -1, 0, 2, 0, LOS_A_TO_B) == 0);
    check("los_outside_top", los(&m, 0, -1, 2, 0, LOS_A_TO_B) == 0);
    check("los_outside_right", los(&m, 0, 0, W, 0, LOS_A_TO_B) == 0);
    check("los_outside_bottom", los(&m, 0, 0, 0, H, LOS_A_TO_B) == 0);
    check("los_vertical", los(&m, 2, 0, 2, 4, LOS_A_TO_B) == 1);

    /* path detours around the blocked cell at (2,0) */
    int path[20], out[20];
    int n = 0;
    path[n++] = idx(0, 0);
    path[n++] = idx(1, 0);
    path[n++] = idx(1, 1);
    path[n++] = idx(2, 1);
    path[n++] = idx(3, 1);
    path[n++] = idx(3, 0);
    path[n++] = idx(4, 0);
    int k = smooth(&w, path, n, out, 20);
    check("smooth_ok", k > 0);
    check("smooth_fewer", k < n);
    check("smooth_starts_at_start", out[0] == path[0]);
    check("smooth_ends_at_goal", out[k - 1] == path[n - 1]);
    check("smooth_exact_len", k == 3);
    check("smooth_exact_midpoint", out[1] == idx(3, 1));
    check("smooth_exact_goal_approach", out[2] == idx(4, 0));
    check("kept_segments_clear", all_segments_clear(&w, out, k));

    /* The segment oracle must fail when handed a line through (2,0). */
    int blocked_segment[2] = { idx(0, 0), idx(4, 0) };
    check("segment_oracle_detects_wall",
          !all_segments_clear(&w, blocked_segment, 2));

    /* a clear straight line collapses to two points */
    Map c; memset(&c, 0, sizeof(c));
    k = smooth(&c, path, n, out, 20);
    check("collapses_to_two", k == 2);

    check("empty_rejected", smooth(&c, path, 0, out, 20) == -1);

    int one[1] = { idx(2, 2) };
    k = smooth(&c, one, 1, out, 20);
    check("singleton_ok", k == 1);
    check("singleton_point_preserved", out[0] == one[0]);
    check("singleton_cap_rejected", smooth(&c, one, 1, out, 0) == -2);

    int two[2] = { idx(0, 0), idx(1, 0) };
    k = smooth(&c, two, 2, out, 20);
    check("two_point_kept", k == 2);

    k = smooth(&c, path, n, out, 1);
    check("cap_rejected", k == -2);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
