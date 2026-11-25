/* navbench: multi-round memory harness around a nav module.
 *
 * A "round" is one complete load-and-query cycle of the module under test:
 * the module's own self-test is re-run inside the same process, which is the
 * closest thing this repo has to "one game session". Between rounds we record
 * live heap (from allocwatch) and process RSS so the two can be compared.
 *
 * build: cc -O2 -std=c11 -Ibench -DMOD_SRC='"../grid/src/grid.c"' \
 *            bench/navbench.c bench/allocwatch.c -o bench/navbench_grid -lm
 * run:   bench/navbench_grid grid 10
 *
 * CSV columns: module,run,rounds_planned,round,rss_kb,live_bytes,live_blocks,
 *              peak_bytes,allocs,frees,selftest_rc
 * Round 0 is the pre-run baseline; it is a cold-start transient, not a data
 * point about per-round behaviour, so the analyzer drops it from the fits.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "allocwatch.h"

#define main mod_selftest
#include MOD_SRC
#undef main

#ifndef RUN_ID
#define RUN_ID "0"
#endif

static long rss_kb(void) {
    FILE* f = fopen("/proc/self/statm", "r");
    long total = 0, resident = 0;
    if (!f) return -1;
    if (fscanf(f, "%ld %ld", &total, &resident) != 2) resident = -1;
    fclose(f);
    return resident * (long)(sysconf(_SC_PAGESIZE) / 1024);
}

int main(int argc, char** argv) {
    const char* name = argc > 1 ? argv[1] : "mod";
    int rounds = argc > 2 ? atoi(argv[2]) : 10;
    if (rounds < 1) rounds = 1;

    printf("module,run,rounds_planned,round,rss_kb,live_bytes,live_blocks,"
           "peak_bytes,allocs,frees,selftest_rc\n");
    printf("%s,%s,%d,0,%ld,0,0,0,0,0,-1\n", name, RUN_ID, rounds, rss_kb());
    fflush(stdout);

    int rc_sum = 0;
    for (int r = 1; r <= rounds; r++) {
        alloc_reset_peak();
        /* module self-test prints to stdout; keep the CSV clean */
        FILE* saved = stdout;
        stdout = fopen("/dev/null", "w");
        int rc = mod_selftest();
        fflush(stdout);
        fclose(stdout);
        stdout = saved;

        rc_sum += rc;
        printf("%s,%s,%d,%d,%ld,%llu,%llu,%llu,%llu,%llu,%d\n",
               name, RUN_ID, rounds, r, rss_kb(),
               alloc_live_bytes(), alloc_live_blocks(), alloc_peak_bytes(),
               alloc_total_allocs(), alloc_total_frees(), rc);
        fflush(stdout);
    }
    fprintf(stderr, "%s run %s: %d rounds, self-test failures across rounds = %d\n",
            name, RUN_ID, rounds, rc_sum);
    return rc_sum == 0 ? 0 : 1;
}
