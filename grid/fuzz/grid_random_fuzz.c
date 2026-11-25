/* Deterministic random failure-path driver for grid A*.
 *
 * Build through run_experiment.py, or manually:
 *   gcc -O0 -g --coverage -std=c11 -Wall -Wextra \
 *       grid_random_fuzz.c -o grid_random_fuzz -lm
 */
#define _POSIX_C_SOURCE 200809L

#define main grid_reference_selftest_main
#include "../src/grid.c"
#undef main

extern int grid_reference_selftest_main(void);

#include <stdint.h>
#include <stdlib.h>
#include <time.h>

typedef struct { uint64_t state; } Rng;

static uint64_t splitmix64(Rng* r) {
    uint64_t z = (r->state += UINT64_C(0x9e3779b97f4a7c15));
    z = (z ^ (z >> 30)) * UINT64_C(0xbf58476d1ce4e5b9);
    z = (z ^ (z >> 27)) * UINT64_C(0x94d049bb133111eb);
    return z ^ (z >> 31);
}

static double rng_unit(Rng* r) {
    return (splitmix64(r) >> 11) * 0x1.0p-53;
}

static long long now_ns(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (long long)ts.tv_sec * 1000000000LL + ts.tv_nsec;
}

static int flood_components(const Map* m, int comp[MAXN]) {
    static int stack[MAXN];
    int next = 0;
    for (int i = 0; i < MAXN; i++) comp[i] = -1;

    for (int start = 0; start < MAXN; start++) {
        if (m->blocked[start] || comp[start] >= 0) continue;
        int sp = 0;
        stack[sp++] = start;
        comp[start] = next;
        while (sp > 0) {
            int u = stack[--sp];
            int x = u % W, y = u / W;
            const int dx[4] = {1, -1, 0, 0};
            const int dy[4] = {0, 0, 1, -1};
            for (int d = 0; d < 4; d++) {
                int nx = x + dx[d], ny = y + dy[d];
                if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                int v = idx(nx, ny);
                if (m->blocked[v] || comp[v] >= 0) continue;
                comp[v] = next;
                stack[sp++] = v;
            }
        }
        next++;
    }
    return next;
}

/* Returns vertex count of the shortest path, or 0 when unreachable. */
static int shortest_path_len(const Map* m, int s, int g) {
    int dist[MAXN], queue[MAXN];
    for (int i = 0; i < MAXN; i++) dist[i] = -1;
    int head = 0, tail = 0;
    dist[s] = 0;
    queue[tail++] = s;

    while (head < tail) {
        int u = queue[head++];
        if (u == g) return dist[u] + 1;
        int x = u % W, y = u / W;
        const int dx[4] = {1, -1, 0, 0};
        const int dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
            int v = idx(nx, ny);
            if (m->blocked[v] || dist[v] >= 0) continue;
            dist[v] = dist[u] + 1;
            queue[tail++] = v;
        }
    }
    return 0;
}

int main(int argc, char** argv) {
    if (argc != 8) {
        fprintf(stderr,
                "usage: %s density seed rounds cap expansion_limit "
                "csv_path stop_after_all\n",
                argv[0]);
        return 2;
    }

    double density = atof(argv[1]);
    uint64_t seed = strtoull(argv[2], NULL, 10);
    int rounds = atoi(argv[3]);
    int cap = atoi(argv[4]);
    int expansion_limit = atoi(argv[5]);
    const char* csv_path = argv[6];
    int stop_after_all = atoi(argv[7]);

    /* Fold the checked-in deterministic self-test into the same gcov run. */
    (void)grid_reference_selftest_main();

    FILE* csv = fopen(csv_path, "w");
    if (!csv) {
        perror("fopen csv");
        return 2;
    }
    fprintf(csv,
            "round,density,blocked,free,start,goal,components,same_component,"
            "shortest_len,expansions,rc,timeout_branch,unreachable_branch,"
            "long_branch,elapsed_ns\n");

    Rng rng = {seed};
    int path[MAXN];
    int comp[MAXN];
    int free_cells[MAXN];

    long long wall_start = now_ns();
    int hits_t = 0, hits_u = 0, hits_l = 0;
    int first_t = 0, first_u = 0, first_l = 0;
    int completed_rounds = 0;

    for (int round = 1; round <= rounds; round++) {
        Map m;
        memset(&m, 0, sizeof(m));
        int blocked = 0;
        for (int i = 0; i < MAXN; i++) {
            if (rng_unit(&rng) < density) {
                m.blocked[i] = 1;
                blocked++;
            }
        }

        int nfree = 0;
        for (int i = 0; i < MAXN; i++) {
            if (!m.blocked[i]) free_cells[nfree++] = i;
        }

        int s = -1, g = -1, components = -1, same = 0, shortest = 0;
        int expansions = 0, rc = -99;
        int hit_t = 0, hit_u = 0, hit_l = 0;

        if (nfree >= 2) {
            int si = (int)(splitmix64(&rng) % (uint64_t)nfree);
            int gi = (int)(splitmix64(&rng) % (uint64_t)(nfree - 1));
            if (gi >= si) gi++;
            s = free_cells[si];
            g = free_cells[gi];

            components = flood_components(&m, comp);
            same = comp[s] == comp[g];
            shortest = shortest_path_len(&m, s, g);
            rc = astar_ex(&m, s % W, s / W, g % W, g / W,
                           path, cap, expansion_limit, &expansions);

            hit_t = (rc == -3);
            hit_u = (rc == -1);
            hit_l = (rc == -2);
            hits_t += hit_t;
            hits_u += hit_u;
            hits_l += hit_l;
            if (hit_t && !first_t) first_t = round;
            if (hit_u && !first_u) first_u = round;
            if (hit_l && !first_l) first_l = round;
        }

        completed_rounds = round;
        fprintf(csv,
                "%d,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%lld\n",
                round, density, blocked, nfree, s, g, components, same,
                shortest, expansions, rc, hit_t, hit_u, hit_l,
                now_ns() - wall_start);
        fflush(csv);

        if (stop_after_all && first_t && first_u && first_l) break;
    }

    fclose(csv);
    long long elapsed_ns = now_ns() - wall_start;
    printf("{"
           "\"density\":%.2f,\"seed\":%llu,\"rounds_requested\":%d,"
           "\"rounds_attempted\":%d,\"cap\":%d,\"expansion_limit\":%d,"
           "\"hits_timeout\":%d,\"hits_unreachable\":%d,\"hits_long\":%d,"
           "\"first_timeout\":%d,\"first_unreachable\":%d,"
           "\"first_long\":%d,\"elapsed_ns\":%lld"
           "}\n",
           density, (unsigned long long)seed, rounds, completed_rounds,
           cap, expansion_limit, hits_t, hits_u, hits_l,
           first_t, first_u, first_l, elapsed_ns);
    return 0;
}
