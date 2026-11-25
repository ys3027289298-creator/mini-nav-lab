#ifndef RETENTION_H
#define RETENTION_H

#include "mapfile.h"

/*
 * How many legacy generations to keep a reader for.
 *
 * The input is a census of old map files by generation: how many exist and
 * have not been re-exported yet. From that count the answers are:
 *
 *   map_min_kept_mask()   the smallest set of readers that reads every one of
 *                         those files directly. With a 100% read-success
 *                         requirement this is forced: a generation with any
 *                         file at all needs a reader. That is the inversion
 *                         the note asks for, and it is one line of logic
 *                         because the requirement is absolute.
 *
 *   map_plan_retention()  the same question when the requirement is allowed
 *                         to be met partly by one-time re-export. Every
 *                         subset of legacy generations is scored as
 *
 *                             lines(mask) + reexport_unit * files(mask)
 *
 *                         i.e. the reader code you carry forever plus the
 *                         one-time cost of re-exporting the files you chose
 *                         not to support. Returns the subsets cheapest-first,
 *                         so the caller can see the whole curve rather than
 *                         only the minimum, and judge how flat it is near the
 *                         bottom.
 *
 * `loc_per_gen` is measured from the sources (`make loc`), not assumed, so the
 * answer moves when the code does. `reexport_unit` is in the same unit as
 * lines: roughly "how many lines of reader is one re-export worth". It is the
 * one judgement call in the model, and it is passed in rather than hidden.
 */

typedef struct {
    unsigned kept_mask;   /* bit g set: a reader is kept for generation g */
    int      kept_count;  /* legacy readers kept */
    long     readable;    /* files read directly with this choice */
    long     reexport;    /* files that need a one-time re-export instead */
    long     lines;       /* dispatch + kept legacy readers, measured */
    double   score;       /* lines + reexport_unit * reexport */
} RetentionOption;

/* Smallest reader set that reads every file directly: one bit per legacy
 * generation that has at least one un-re-exported file. Zero when no legacy
 * files remain. */
unsigned map_min_kept_mask(const long files_per_gen[MAP_GEN_COUNT]);

/*
 * Enumerate all 2^(MAP_GEN_CURRENT-1) subsets of legacy generations, score
 * them, sort cheapest-first into out[0..], and return how many were written.
 * out_cap must be at least 1 << (MAP_GEN_CURRENT-1).
 */
int map_plan_retention(const long files_per_gen[MAP_GEN_COUNT],
                       const long loc_per_gen[MAP_GEN_COUNT],
                       long dispatch_loc, double reexport_unit,
                       RetentionOption* out, int out_cap);

#endif
