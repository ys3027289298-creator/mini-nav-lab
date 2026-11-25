
/* smooth: line-of-sight test and path smoothing. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12

typedef struct {
    unsigned char blocked[W*H];
    unsigned char terrain[W*H]; /* 新增：有限权重档（普通/草/泥），不参与 LOS */
} Map;
static int idx(int x, int y) { return y * W + x; }

/* 加权后视线判据仍然成立：只有代价 ∞ 的 blocked 端点遮挡视线；
   草=2、泥=4 都只是有限移动代价，不构成遮挡。
   因此 LOS / 贪心平滑的几何结论与 bool 时代完全相同。 */
static int sight_blocked(const Map* m, int x, int y) {
    return m->blocked[idx(x, y)];
}

/* bresenham supercover: any blocked cell on the segment blocks LOS */
static int los(const Map* m, int x0, int y0, int x1, int y1) {
    int dx = abs(x1 - x0), dy = abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int x = x0, y = y0;
    for (;;) {
        if (x < 0 || y < 0 || x >= W || y >= H) return 0;
        if (sight_blocked(m, x, y)) return 0;
        if (x == x1 && y == y1) return 1;
        int e2 = 2 * err;
        if (e2 > -dy) { err -= dy; x += sx; }
        if (e2 < dx) { err += dx; y += sy; }
    }
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
            if (los(m, path[i] % W, path[i] / W, path[j] % W, path[j] / W)) break;
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
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    check("los_open", los(&m, 0, 0, 5, 0) == 1);
    check("los_diag", los(&m, 0, 0, 4, 4) == 1);

    Map w; memset(&w, 0, sizeof(w));
    w.blocked[idx(2, 0)] = 1;
    check("los_blocked", los(&w, 0, 0, 4, 0) == 0);

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
    /* every kept segment must actually have line of sight */
    int all_clear = 1;
    for (int i = 0; i + 1 < k; i++) {
        if (!los(&w, out[i] % W, out[i] / W, out[i+1] % W, out[i+1] / W)) all_clear = 0;
    }
    check("kept_segments_clear", all_clear);

    /* a clear straight line collapses to two points */
    Map c; memset(&c, 0, sizeof(c));
    k = smooth(&c, path, n, out, 20);
    check("collapses_to_two", k == 2);

    check("empty_rejected", smooth(&c, path, 0, out, 20) == -1);
    k = smooth(&c, path, n, out, 1);
    check("cap_rejected", k == -2);

    /* finite terrain weights must not change LOS or smoothing geometry */
    Map tw; memset(&tw, 0, sizeof(tw));
    tw.terrain[idx(2, 0)] = 1; /* grass */
    check("los_ignores_grass", los(&tw, 0, 0, 4, 0) == 1);
    tw.terrain[idx(2, 0)] = 2; /* mud, heavier but still finite */
    check("los_ignores_mud", los(&tw, 0, 0, 4, 0) == 1);
    k = smooth(&tw, path, n, out, 20);
    check("smooth_weights_collapse_two", k == 2);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
