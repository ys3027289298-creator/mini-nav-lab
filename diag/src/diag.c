
/*
 * diag: 「卡住」与「穿墙」两类移动缺陷的判据、分量反解与加粗取舍。
 *
 * 仓库只提供三个纯函数库，没有场景/角色/地图加载器，因此这里用三个库的
 * 原语（逐字复制语义，仅补 <stdlib.h> 与缩进）构造最小复现：
 *   grid/src/grid.c       walkable / astar
 *   region/src/region.c   components
 *   smooth/src/smooth.c   los（普通 Bresenham）/ smooth
 *
 * 新增：los_supercover（超覆盖真值视线，角点接触时两侧格都算擦到）、
 *       dilate（切比雪夫半径膨胀）、场景 RUN1/2/3、加粗扫描。
 *
 * 判据：
 *   stuck(s,g) := astar(s,g) == -1
 *   clip(s,g)  := astar(s,g) != -1 且 smooth 后某段 los_supercover 为假
 *
 * 反解：
 *   stuck -> comp(s) != comp(g) 且无解
 *   clip  -> comp(s) == comp(g) 且可解，但真值地图上该段穿墙
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define W 16
#define H 12
#define MAXN (W*H)

typedef struct { unsigned char blocked[W*H]; } Map;

static int idx(int x, int y) { return y * W + x; }

/* ---- grid/src/grid.c ---- */
static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

static int astar(const Map* m, int sx, int sy, int gx, int gy, int* out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) {
        g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1;
    }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == idx(gx, gy)) {
            int n = 0, c = cur;
            int rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
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
                for (int i = 0; i < nopen; i++) if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return -1;
}

/* ---- region/src/region.c ---- */
static int components(const Map* m, int* comp) {
    for (int i = 0; i < W*H; i++) comp[i] = -1;
    int next = 0;
    int stack[W*H];
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int s = idx(x, y);
            if (m->blocked[s] || comp[s] >= 0) continue;
            int sp = 0;
            stack[sp++] = s;
            comp[s] = next;
            while (sp > 0) {
                int cur = stack[--sp];
                int cx = cur % W, cy = cur / W;
                const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
                for (int d = 0; d < 4; d++) {
                    int nx = cx + dx[d], ny = cy + dy[d];
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    int ni = idx(nx, ny);
                    if (m->blocked[ni] || comp[ni] >= 0) continue;
                    comp[ni] = next;
                    stack[sp++] = ni;
                }
            }
            next++;
        }
    }
    return next;
}

/* ---- smooth/src/smooth.c : 普通 Bresenham。注释写 supercover，实际不是。 ---- */
static int los_plain(const Map* m, int x0, int y0, int x1, int y1) {
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

/* ---- 真值视线：超覆盖。dx==dy 的精确对角在角点接触两侧格，普通 Bresenham
 *      会直接跳过去，因此漏判。 ---- */
static int los_supercover(const Map* m, int x0, int y0, int x1, int y1) {
    int dx = x1 - x0, dy = y1 - y0;
    int nx = abs(dx), ny = abs(dy);
    int sx = dx > 0 ? 1 : -1, sy = dy > 0 ? 1 : -1;
    int x = x0, y = y0, ix = 0, iy = 0;
    if (!walkable(m, x, y)) return 0;
    while (ix < nx || iy < ny) {
        int dec = (1 + 2 * ix) * ny - (1 + 2 * iy) * nx;
        if (dec == 0 && ix < nx && iy < ny) {
            if (!walkable(m, x + sx, y)) return 0;
            if (!walkable(m, x, y + sy)) return 0;
            x += sx; y += sy; ix++; iy++;
        } else if (dec < 0 || iy == ny) {
            x += sx; ix++;
        } else {
            y += sy; iy++;
        }
        if (!walkable(m, x, y)) return 0;
    }
    return 1;
}

typedef int (*LosFn)(const Map*, int, int, int, int);

static int smooth_with(const Map* m, const int* path, int n, int* out, int cap,
                       LosFn los) {
    if (n <= 0) return -1;
    if (n == 1) {
        if (cap < 1) return -2;
        out[0] = path[0];
        return 1;
    }
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

/* 以切比雪夫半径 t 膨胀阻挡格；t=0 原样复制 */
static void dilate(const Map* src, Map* dst, int t) {
    memset(dst, 0, sizeof(*dst));
    for (int y = 0; y < H; y++) {
        for (int x = 0; x < W; x++) {
            int hit = 0;
            for (int dy = -t; dy <= t && !hit; dy++) {
                for (int dx = -t; dx <= t; dx++) {
                    int nx = x + dx, ny = y + dy;
                    if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                    if (src->blocked[idx(nx, ny)]) { hit = 1; break; }
                }
            }
            dst->blocked[idx(x, y)] = (unsigned char)hit;
        }
    }
}

static void show(const Map* m, const char* title) {
    printf("%s\n", title);
    for (int y = 0; y < H; y++) {
        printf("  y%2d ", y);
        for (int x = 0; x < W; x++) printf("%c", m->blocked[idx(x, y)] ? '#' : '.');
        printf("\n");
    }
    printf("       ");
    for (int x = 0; x < W; x++) printf("%d", x % 10);
    printf("\n\n");
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ============================================================ */

/* RUN 1 — 卡住（规划级）：楼梯形墙，仅对角接触，四邻域断开。
 * 判据 stuck 成立，起终点落在不同分量。 */
static void run_stuck_planner(void) {
    Map st;
    memset(&st, 0, sizeof st);
    for (int y = 0; y <= 5; y++) st.blocked[idx(8, y)] = 1;
    for (int y = 6; y < H; y++) st.blocked[idx(9, y)] = 1;

    int comp[W*H];
    int nc = components(&st, comp);
    int cs = comp[idx(7, 6)], cg = comp[idx(10, 5)];
    int n = astar(&st, 7, 6, 10, 5, (int[MAXN]){0}, MAXN);

    printf("=== RUN 1: 卡住（规划级，起终点不同分量） ===\n");
    show(&st, "地图：x=8 封 y<=5，x=9 封 y>=6，仅 (8,6)/(9,5) 对角接触");
    printf("分量数=%d  start(7,6) comp=%d  goal(10,5) comp=%d  astar=%d\n",
           nc, cs, cg, n);
    check("run1_diff_components", cs != cg);
    check("run1_no_path", n == -1);
    printf("\n");
}

/* RUN 2 — 卡住（执行级）：A* 有解、同分量，但 smooth 把路径塌缩成贴角对角线，
 * 角色的下一格是墙。这是「路径的下一格不可走」的字面情形，却与「不同分量」矛盾。 */
static void run_stuck_executor(void) {
    Map cc;
    memset(&cc, 0, sizeof cc);
    cc.blocked[idx(3, 2)] = 1;
    cc.blocked[idx(2, 3)] = 1;

    int comp[W*H];
    int nc = components(&cc, comp);
    int path[MAXN], out[MAXN];
    int n = astar(&cc, 2, 2, 3, 3, path, MAXN);
    int k = smooth_with(&cc, path, n, out, MAXN, los_plain);

    printf("=== RUN 2: 卡住（执行级，同分量但平滑切角） ===\n");
    show(&cc, "地图：阻挡格 (3,2) 与 (2,3) 在角点相接");
    printf("分量数=%d  start(2,2) comp=%d  goal(3,3) comp=%d\n",
           nc, comp[idx(2,2)], comp[idx(3,3)]);
    printf("astar 长度=%d  平滑后点数=%d:", n, k);
    for (int i = 0; i < k; i++) printf(" (%d,%d)", out[i] % W, out[i] / W);
    printf("\n");
    int lp = los_plain(&cc, 2, 2, 3, 3);
    int ls = los_supercover(&cc, 2, 2, 3, 3);
    printf("los_plain(2,2->3,3)=%d  los_supercover=%d  下一格 (3,2)/(2,3) 均不可走\n", lp, ls);

    check("run2_same_component", comp[idx(2,2)] == comp[idx(3,3)]);
    check("run2_has_path", n > 0);
    check("run2_collapsed_to_diag", k == 2);
    check("run2_plain_los_blind", lp == 1 && ls == 0);
    check("run2_next_cells_blocked",
          !walkable(&cc, 3, 2) && !walkable(&cc, 2, 3));
    printf("\n");
}

/* RUN 3 — 穿墙（数据级）：真值有整墙，可走性地图漏标 x=8,y=4..7。
 * 地图上同分量（误判可过），真值上不同分量；A* 直接穿过漏标格。 */
static void run_clip_map(void) {
    Map truth, mm;
    memset(&truth, 0, sizeof truth);
    for (int y = 0; y < H; y++) truth.blocked[idx(8, y)] = 1;
    mm = truth;
    for (int y = 4; y <= 7; y++) mm.blocked[idx(8, y)] = 0;

    int cT[W*H], cM[W*H];
    int nT = components(&truth, cT), nM = components(&mm, cM);
    int path[MAXN];
    int n = astar(&mm, 7, 5, 9, 5, path, MAXN);

    printf("=== RUN 3: 穿墙（数据级，薄墙漏标） ===\n");
    show(&mm, "可走性地图 M：x=8,y=4..7 漏标为可走");
    printf("M   分量数=%d  comp(7,5)=%d comp(9,5)=%d  -> 同一分量（误判可过）\n",
           nM, cM[idx(7,5)], cM[idx(9,5)]);
    printf("真值 分量数=%d  comp(7,5)=%d comp(9,5)=%d  -> 不同分量（应不可过）\n",
           nT, cT[idx(7,5)], cT[idx(9,5)]);
    printf("astar(7,5 -> 9,5) 长度=%d  路径穿过漏标格:", n);
    for (int i = 0; i < n; i++)
        if (truth.blocked[path[i]]) printf(" (%d,%d)", path[i] % W, path[i] / W);
    printf("\n漏标格坐标:");
    int missed = 0;
    for (int y = 0; y < H; y++) {
        if (truth.blocked[idx(8, y)] && !mm.blocked[idx(8, y)]) {
            printf(" (%d,%d)", 8, y);
            missed++;
        }
    }
    printf("\n");

    int crossed = 0;
    for (int i = 0; i < n; i++) if (truth.blocked[path[i]]) crossed = 1;
    check("run3_map_one_component", cM[idx(7,5)] == cM[idx(9,5)]);
    check("run3_truth_split", cT[idx(7,5)] != cT[idx(9,5)]);
    check("run3_missed_four_cells", missed == 4);
    check("run3_path_crosses_truth", crossed);
    printf("\n");
}

/* 加粗取舍：真值地图 x=8 整墙 + 一条门。加粗同时缩窄门。
 * 两条曲线：窄缝通过率（门的各种宽度）、穿墙段率（普通 vs 超覆盖视线）。 */
static void run_sweep(void) {
    const int widths[4] = {1, 2, 3, 5};

    printf("=== 加粗扫描：t = 切比雪夫膨胀半径 ===\n");
    printf("窄缝通过率（左 x<=5 到右 x>=10，按行可达比例）\n");
    printf("t | w=1 | w=2 | w=3 | w=5\n");
    int gap_pass1_t0 = -1, gap_pass1_t1 = -1;
    for (int t = 0; t <= 3; t++) {
        printf("%d |", t);
        for (int wi = 0; wi < 4; wi++) {
            Map truth, mt;
            memset(&truth, 0, sizeof truth);
            for (int y = 0; y < H; y++) truth.blocked[idx(8, y)] = 1;
            int y0 = 5 - (widths[wi] - 1) / 2;
            for (int y = y0; y < y0 + widths[wi]; y++)
                if (y >= 0 && y < H) truth.blocked[idx(8, y)] = 0;
            dilate(&truth, &mt, t);
            int ok = 0, tot = 0;
            for (int y = 0; y < H; y++) {
                if (!walkable(&mt, 6, y) || !walkable(&mt, 10, y)) continue;
                tot++;
                if (astar(&mt, 6, y, 10, y, (int[MAXN]){0}, MAXN) > 0) ok++;
            }
            printf(" %2d%% |", tot ? 100 * ok / tot : 0);
            if (wi == 0) {
                if (t == 0) gap_pass1_t0 = tot ? 100 * ok / tot : 0;
                if (t == 1) gap_pass1_t1 = tot ? 100 * ok / tot : 0;
            }
        }
        printf("\n");
    }

    printf("\n穿墙段率（门宽 3，左区到右区全部点对；smooth 后任一段被真值"
           " supercover 判为穿墙即计一次）\n");
    printf("t | 普通 Bresenham | 超覆盖 supercover\n");
    int clip_plain_t0 = -1, clip_sc_t0 = -1;
    for (int t = 0; t <= 2; t++) {
        Map truth, mt;
        memset(&truth, 0, sizeof truth);
        for (int y = 0; y < H; y++) truth.blocked[idx(8, y)] = 1;
        for (int y = 4; y <= 6; y++) truth.blocked[idx(8, y)] = 0;
        dilate(&truth, &mt, t);

        int pairs = 0, bad_plain = 0, bad_sc = 0;
        for (int sx = 0; sx <= 5; sx++) for (int sy = 0; sy < H; sy++) {
            for (int gx = 10; gx < W; gx++) for (int gy = 0; gy < H; gy++) {
                int path[MAXN], out[MAXN];
                int n = astar(&mt, sx, sy, gx, gy, path, MAXN);
                if (n <= 0) continue;
                pairs++;
                for (int mode = 0; mode < 2; mode++) {
                    LosFn sf = mode == 0 ? los_plain : los_supercover;
                    int k = smooth_with(&mt, path, n, out, MAXN, sf);
                    int bad = 0;
                    for (int i = 0; i + 1 < k && !bad; i++)
                        if (!los_supercover(&truth, out[i] % W, out[i] / W,
                                            out[i+1] % W, out[i+1] / W))
                            bad = 1;
                    if (bad) { if (mode == 0) bad_plain++; else bad_sc++; }
                }
            }
        }
        printf("%d |   %3d%% (%d/%d)   |   %3d%% (%d/%d)\n", t,
               pairs ? 100 * bad_plain / pairs : 0, bad_plain, pairs,
               pairs ? 100 * bad_sc / pairs : 0, bad_sc, pairs);
        if (t == 0) {
            clip_plain_t0 = pairs ? 100 * bad_plain / pairs : 0;
            clip_sc_t0 = pairs ? 100 * bad_sc / pairs : 0;
        }
    }

    printf("\n取舍：加粗 t=1 已把 w=1 窄缝通过率打到 0%%，是用窄缝换防穿；\n");
    printf("      不改视线、只加粗 t=0->1 穿墙率降为 0，但代价是窄缝全灭。\n");
    printf("      换 supercover 视线、t=0：穿墙率归零，窄缝通过率不变。\n");

    check("sweep_gap1_dies_with_dilation", gap_pass1_t0 > 0 && gap_pass1_t1 == 0);
    check("sweep_plain_los_clips_at_t0", clip_plain_t0 > 0);
    check("sweep_supercover_no_clip_at_t0", clip_sc_t0 == 0);
}

int main(void) {
    run_stuck_planner();
    run_stuck_executor();
    run_clip_map();
    run_sweep();
    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
