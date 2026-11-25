
/* grid: walkability map and 4-neighbour A* search. */
#include <stdio.h>
#include <string.h>
#include <math.h>

#define W 16
#define H 12
#define MAXN (W*H)

/* 可走性由 bool 迁移为权重：blocked=1 仍是代价 ∞ 的端点；
   terrain 是地形档位，经 kCost 查表得到有限步代价。
   0 档=普通，地图零初始化即全普通，行为与 bool 时代逐位一致。 */
#define T_FLAT 0
#define T_GRASS 1
#define T_MUD 2
#define T_KINDS 3
static const int kCost[T_KINDS] = { 1, 2, 4 }; /* 普通 / 草地(慢) / 泥地(更慢) */

typedef struct {
    unsigned char blocked[W*H];
    unsigned char terrain[W*H]; /* 取值 0..T_KINDS-1 */
} Map;

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* 进入格 (x,y) 的代价：一次 O(1) 查表。最小权重为 1，
   所以 Manhattan 启发依旧可采纳（h <= 任意剩余路径代价）。 */
static int step_cost(const Map* m, int x, int y) {
    unsigned char t = m->terrain[idx(x, y)];
    return kCost[t < T_KINDS ? t : T_FLAT];
}

/* returns path length or -1 */
static int astar(const Map* m, int sx, int sy, int gx, int gy, int* out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
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
            int ng = g[cur] + step_cost(m, nx, ny); /* 原: +1 */
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

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    int path[MAXN];

    int n = astar(&m, 0, 0, 3, 0, path, MAXN);
    check("straight_len", n == 4);
    check("straight_ends", path[0] == idx(0,0) && path[3] == idx(3,0));

    n = astar(&m, 0, 0, 0, 3, path, MAXN);
    check("vertical_len", n == 4);

    /* diagonal moves are not allowed: 4-neighbour path is manhattan length */
    n = astar(&m, 0, 0, 2, 2, path, MAXN);
    check("manhattan_len", n == 5);

    /* wall forces a detour */
    Map w; memset(&w, 0, sizeof(w));
    for (int y = 0; y < 4; y++) w.blocked[idx(1, y)] = 1;
    n = astar(&w, 0, 0, 2, 0, path, MAXN);
    check("detour_found", n > 3);
    check("detour_avoids_wall", 1);
    for (int i = 0; i < n; i++) {
        int x = path[i] % W, y = path[i] / W;
        check("path_on_free", !w.blocked[idx(x, y)]);
        if (w.blocked[idx(x, y)]) break;
    }

    /* unreachable goal behind a full wall */
    Map b; memset(&b, 0, sizeof(b));
    for (int y = 0; y < H; y++) b.blocked[idx(1, y)] = 1;
    check("unreachable", astar(&b, 0, 0, 2, 0, path, MAXN) == -1);

    /* blocked start or goal */
    check("blocked_start", astar(&b, 1, 0, 2, 0, path, MAXN) == -1);

    /* cap too small */
    n = astar(&m, 0, 0, 3, 3, path, 3);
    check("cap_rejected", n == -2);

    /* weighted terrain: mud on the direct cell (cost 4) makes the
       4-step flat detour (cost 4) win despite having more nodes. */
    Map q; memset(&q, 0, sizeof(q));
    q.terrain[idx(1, 0)] = T_MUD;
    n = astar(&q, 0, 0, 2, 0, path, MAXN);
    check("weight_prefers_cheap_detour", n == 5);
    int hit_mud = 0, entered_cost = 0;
    for (int i = 1; i < n; i++) {
        int x = path[i] % W, y = path[i] / W;
        if (path[i] == idx(1, 0)) hit_mud = 1;
        entered_cost += step_cost(&q, x, y);
    }
    check("weight_avoids_mud", !hit_mud);
    check("weight_detour_cost4", entered_cost == 4);

    /* uniform scaling changes g but not the optimal geometry */
    Map u; memset(&u, 0, sizeof(u));
    memset(u.terrain, T_GRASS, W*H);
    n = astar(&u, 0, 0, 3, 0, path, MAXN);
    check("uniform_weight_same_len", n == 4);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
