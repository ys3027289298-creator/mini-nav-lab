
/* region: connected components over the walkable grid. */
#include <stdio.h>
#include <string.h>

#define W 16
#define H 12

typedef struct { unsigned char blocked[W*H]; } Map;
static int idx(int x, int y) { return y * W + x; }

/* flood fill, returns component id per cell (-1 for blocked), and count */
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

static int same_region(const int* comp, int ax, int ay, int bx, int by) {
    int a = comp[idx(ax, ay)], b = comp[idx(bx, by)];
    return (a >= 0 && a == b) ? 1 : 0;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

int main(void) {
    Map m; memset(&m, 0, sizeof(m));
    int comp[W*H];
    int n = components(&m, comp);
    check("one_component", n == 1);
    check("all_same", same_region(comp, 0, 0, 15, 11) == 1);

    /* split the map in two with a full-height wall */
    Map s; memset(&s, 0, sizeof(s));
    for (int y = 0; y < H; y++) s.blocked[idx(8, y)] = 1;
    n = components(&s, comp);
    check("two_components", n == 2);
    check("left_not_right", same_region(comp, 0, 0, 9, 0) == 0);
    check("left_to_left", same_region(comp, 0, 0, 7, 11) == 1);

    /* a pillar in the middle does not split the space */
    Map p; memset(&p, 0, sizeof(p));
    p.blocked[idx(8, 5)] = 1;
    n = components(&p, comp);
    check("pillar_keeps_one", n == 1);
    check("around_pillar", same_region(comp, 0, 0, 15, 11) == 1);

    check("blocked_is_minus_one", comp[idx(8, 5)] == -1);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
