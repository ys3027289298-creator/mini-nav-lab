#include <stdlib.h>
#include <string.h>

#include "retention.h"

unsigned map_min_kept_mask(const long files_per_gen[MAP_GEN_COUNT]) {
    unsigned mask = 0;
    /* The current generation is always read; only legacy ones are optional. */
    for (int g = 0; g < MAP_GEN_CURRENT; g++)
        if (files_per_gen[g] > 0) mask |= 1u << g;
    return mask;
}

static int by_score(const void* a, const void* b) {
    const RetentionOption* x = (const RetentionOption*)a;
    const RetentionOption* y = (const RetentionOption*)b;
    /* Ties go to fewer readers: same cost, less code to keep alive. */
    if (x->score < y->score) return -1;
    if (x->score > y->score) return 1;
    if (x->kept_count != y->kept_count) return x->kept_count - y->kept_count;
    return (int)x->kept_mask - (int)y->kept_mask;
}

int map_plan_retention(const long files_per_gen[MAP_GEN_COUNT],
                       const long loc_per_gen[MAP_GEN_COUNT],
                       long dispatch_loc, double reexport_unit,
                       RetentionOption* out, int out_cap) {
    int legacy = MAP_GEN_CURRENT;             /* generations 0..legacy-1 */
    int subsets = 1 << legacy;
    if (!out || out_cap < subsets) return 0;

    for (int s = 0; s < subsets; s++) {
        RetentionOption o;
        memset(&o, 0, sizeof o);
        o.kept_mask = (unsigned)s;
        o.lines = dispatch_loc;
        for (int g = 0; g < legacy; g++) {
            if (s & (1 << g)) {
                o.kept_count++;
                o.lines += loc_per_gen[g];
                o.readable += files_per_gen[g];
            } else {
                o.reexport += files_per_gen[g];
            }
        }
        o.score = (double)o.lines + reexport_unit * (double)o.reexport;
        out[s] = o;
    }

    qsort(out, (size_t)subsets, sizeof *out, by_score);
    return subsets;
}
