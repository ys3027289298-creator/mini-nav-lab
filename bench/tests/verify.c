/* BFS oracle: every A* variant must return the true shortest path length,
 * on both map families, with identical search traces across variants. */
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#define main bench_discarded_main
#include "../src/bench.c"
#undef main

static int bfs_dist(const Map *m) {
    int n = m->w * m->h;
    int *dist = malloc((size_t)n * sizeof *dist);
    int *q = malloc((size_t)n * sizeof *q);
    for (int i = 0; i < n; i++) dist[i] = -1;
    int head = 0, tail = 0;
    int s = m->sy * m->w + m->sx;
    dist[s] = 0; q[tail++] = s;
    static const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
    while (head < tail) {
        int cur = q[head++];
        if (cur == m->gy * m->w + m->gx) break;
        int cx = cur % m->w, cy = cur / m->w;
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = ny * m->w + nx;
            if (dist[ni] != -1) continue;
            dist[ni] = dist[cur] + 1;
            q[tail++] = ni;
        }
    }
    int r = dist[m->gy * m->w + m->gx];
    free(dist); free(q);
    return r < 0 ? -1 : r + 1; /* node count, matching pathlen convention */
}

int main(void) {
    search_fn fns[] = {astar_array, astar_bitmap, astar_heap};
    const char *names[] = {"array", "bitmap", "heap"};
    int sides[] = {9, 11, 15, 19, 23, 27, 33, 41, 49, 65, 97, 129, 193, 257};
    const char *fams[] = {"scatter", "maze"};
    int fails = 0, trace_mismatch = 0;
    for (unsigned fi = 0; fi < 2; fi++) {
        for (unsigned li = 0; li < sizeof sides / sizeof *sides; li++) {
            int L = sides[li];
            Map *map = fams[fi][0] == 's' ? map_scatter(L) : map_maze(L);
            Ctx ctx = ctx_make(L * L);
            int truth = bfs_dist(map);
            long base_exp = -1, base_ins = -1;
            for (unsigned vi = 0; vi < 3; vi++) {
                Stats st = {0};
                fns[vi](map, &ctx, &st);
                if (st.pathlen != truth) {
                    printf("FAIL %s L=%d %s: astar=%d bfs=%d exp=%ld omax=%d\n",
                           fams[fi], L, names[vi], st.pathlen, truth,
                           st.exp, st.maxopen);
                    fails++;
                }
                if (base_exp < 0) { base_exp = st.exp; base_ins = st.ins; }
                else if (st.exp != base_exp || st.ins != base_ins) {
                    printf("TRACE %s L=%d %s: exp=%ld ins=%ld (array exp=%ld ins=%ld)\n",
                           fams[fi], L, names[vi], st.exp, st.ins,
                           base_exp, base_ins);
                    trace_mismatch++;
                }
            }
            ctx_free(&ctx);
            map_free(map);
        }
    }
    printf(fails || trace_mismatch
               ? "ORACLE: %d optimality failures, %d trace mismatches\n"
               : "ORACLE: all variants optimal with identical traces on all maps\n",
           fails, trace_mismatch);
    return fails ? 1 : 0;
}
