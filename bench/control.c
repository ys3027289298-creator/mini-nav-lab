/* control: a synthetic nav-cache workload with a known ground truth.
 *
 * Purpose: validate that bench/analyze.py recovers the right answer before we
 * trust it on real measurements. The model is the one the task asks about --
 * each round resolves `entries` paths, each `entry_bytes` large, then a fixed
 * fraction `retire` of everything currently held is dropped:
 *
 *     held[n] = (held[n-1] + entries) * (1 - retire),   held[0] = 0
 *
 *   retire = 0    -> linear growth, slope = entries*entry_bytes  (leak)
 *   retire = 1    -> flat after each round                       (no leak)
 *   0 < retire < 1 -> geometric convergence to entries*(1-retire)/retire
 *                    (partial reclaim). Distinguishable from a straight line
 *                    only when the run covers several 1/retire rounds.
 *
 * The slot bookkeeping arrays are allocated before the first measurement, so
 * the counters and RSS see path entries only, never the harness's own growth.
 *
 * build: cc -O2 -std=c11 -Ibench control.c bench/allocwatch.c -o bench/control
 * run:   bench/control leak --rounds 12 --entries 40 --entry-bytes 72 --retire 0
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "allocwatch.h"

/* A cache slot stands in for one resolved path: a handful of waypoints plus
 * bookkeeping. Only its size matters here, so it is a plain byte blob. */
typedef struct { void* mem; } Slot;

static Slot* pool;          /* preallocated before measurement */
static int held_head = 0;   /* oldest still-held slot */
static int held_tail = 0;   /* one past newest still-held slot */

static long rss_kb(void) {
    FILE* f = fopen("/proc/self/statm", "r");
    long total = 0, resident = 0;
    if (!f) return -1;
    if (fscanf(f, "%ld %ld", &total, &resident) != 2) resident = -1;
    fclose(f);
    return resident * (long)(sysconf(_SC_PAGESIZE) / 1024);
}

int main(int argc, char** argv) {
    const char* name = argc > 1 ? argv[1] : "control";
    int rounds = 12, entries = 40;
    size_t entry_bytes = 72;
    double retire_frac = 0.0;
    const char* run = "0";
    for (int i = 1; i < argc; i++) {
        if (!strcmp(argv[i], "--rounds") && i + 1 < argc) rounds = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--entries") && i + 1 < argc) entries = atoi(argv[++i]);
        else if (!strcmp(argv[i], "--entry-bytes") && i + 1 < argc) entry_bytes = (size_t)atol(argv[++i]);
        else if (!strcmp(argv[i], "--retire") && i + 1 < argc) retire_frac = atof(argv[++i]);
        else if (!strcmp(argv[i], "--run") && i + 1 < argc) run = argv[++i];
    }
    if (retire_frac < 0) retire_frac = 0;
    if (retire_frac > 1) retire_frac = 1;

    /* slot pool: one slot per entry ever created, allocated up front */
    pool = malloc((size_t)rounds * (size_t)entries * sizeof *pool);
    if (!pool) { perror("malloc"); return 1; }

    fprintf(stderr, "truth %s: entries/round=%d entry_bytes=%zu retire=%.2f "
            "-> net %.2f entries/round, %.1f B/round\n",
            name, entries, entry_bytes, retire_frac,
            entries * (1.0 - retire_frac), entries * entry_bytes * (1.0 - retire_frac));

    printf("module,run,rounds_planned,round,rss_kb,live_bytes,live_blocks,"
           "peak_bytes,allocs,frees,selftest_rc\n");
    printf("%s,%s,%d,0,%ld,0,0,0,0,0,-1\n", name, run, rounds, rss_kb());
    fflush(stdout);

    for (int r = 1; r <= rounds; r++) {
        alloc_reset_peak();
        for (int i = 0; i < entries; i++) {
            void* p = malloc(entry_bytes);
            memset(p, 1, entry_bytes); /* touch it: RSS must reflect it */
            pool[held_tail].mem = p;   /* one pool slot per entry ever created */
            held_tail++;
        }
        if (retire_frac > 0) {
            int held = held_tail - held_head;
            int to_drop = (int)(held * retire_frac + 0.5);
            if (to_drop > held) to_drop = held;
            for (int i = 0; i < to_drop; i++) free(pool[held_head + i].mem);
            held_head += to_drop;
        }
        printf("%s,%s,%d,%d,%ld,%llu,%llu,%llu,%llu,%llu,0\n",
               name, run, rounds, r, rss_kb(),
               alloc_live_bytes(), alloc_live_blocks(), alloc_peak_bytes(),
               alloc_total_allocs(), alloc_total_frees());
        fflush(stdout);
    }
    free(pool);
    return 0;
}
