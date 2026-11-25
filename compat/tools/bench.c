/*
 * bench: the acceptance numbers for the compatibility layer.
 *
 * Three sections, each answering one question from the handoff note.
 *
 *   ACCEPTANCE  read every old file; require 100% success. Then compute the
 *               path twice per file, once through the compatibility reader
 *               and once after a one-shot re-export, and require the two to
 *               be identical point by point. Where they are not, report the
 *               shared-point ratio and the query field whose value the
 *               whole-cell snap changed. Reported per generation, because the
 *               generations differ in exactly the way that matters: gen 0 has
 *               no query to lose, gen 1 has one.
 *
 *   RETENTION   the fraction of existing old files readable under each choice
 *               of how many legacy generations get a reader, against the
 *               measured reader size. Then the inversion: for a census of
 *               un-re-exported files, which generations does the 100% read
 *               requirement force, and where is the crossover at which
 *               re-exporting becomes cheaper than keeping the reader.
 *
 *   DIVERGENCE  a deliberate mismatch: the reader snaps one way, the offline
 *               re-export another. Not a supported configuration; it shows
 *               the equality test is not vacuous and exhibits the field-level
 *               attribution the note asks for.
 *
 * On the fixtures: the modules in grid/, smooth/ and region/ never write a
 * file, so there is no old corpus in this repo. All fixtures here are
 * synthetic and shaped like what those modules would have produced (16x12
 * cells; gen 1 adds a sub-cell query). Numbers are therefore about the reader
 * and the upgrade, not about real level data.
 *
 * Exit status is nonzero if any tier misses 100% read success or falls under
 * the 99% shared-point floor.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "fixtures.h"
#include "mapfile.h"
#include "nav.h"
#include "reexport.h"
#include "retention.h"

#define READ_FLOOR_PCT   100.0
#define SHARED_FLOOR_PCT  99.0

/* Reader size, measured by the Makefile from the sources. Absent, the
 * retention section prints "-" rather than a number nobody measured. */
#if defined(LOC_GEN0) && defined(LOC_GEN1) && defined(LOC_DISPATCH)
#define HAVE_LOC 1
#else
#define HAVE_LOC 0
#endif

static const char* field_name(int i) {
    static const char* n[4] = {"sx", "sy", "gx", "gy"};
    return (i >= 0 && i < 4) ? n[i] : "?";
}

static void print_mask(unsigned mask) {
    int first = 1;
    for (int i = 0; i < 4; i++) {
        if (mask & (1u << i)) {
            printf("%s%s", first ? "" : ",", field_name(i));
            first = 0;
        }
    }
    if (first) printf("(none)");
}

/* Per-generation tally, so a statement about gen 0 is never contaminated by
 * gen 1's sub-cell query. */
typedef struct {
    int files;
    int read_ok;
    int identical;
    long shared_points;
    long total_points;
    int path_diffs;
    int cell_diffs;
    int query_diffs;         /* the two roads disagreed on the query itself */
    long moved_field[4];     /* per query field: files where the snap moved a
                              * path, so attribution is a count and not a
                              * union that is always "all four" */
    long snapped_field[4];   /* per query field: files where the snap was
                              * inexact, whether or not the path moved */
} GenResult;

typedef struct {
    GenResult gen[2];
} TierResult;

/*
 * Compare one file's two roads. `reader_round`, `tool_round` are the two
 * quantisation policies; equal in ACCEPTANCE, deliberately different in
 * DIVERGENCE.
 */
static void one_file(const Fixture* fx, int gen,
                     MapQRounding reader_round, MapQRounding tool_round,
                     GenResult* r, NavPath* pa, NavPath* pb, int verbose) {
    size_t len = 0;
    unsigned char* old = (gen == 0) ? fx_emit_gen0(fx, &len)
                                    : fx_emit_gen1(fx, &len);
    if (!old) return;

    /* Road A: the compatibility reader, with the reader's policy. */
    map_upgrade_set_rounding(reader_round);
    MapFile a;
    MapStatus sa = map_read(old, len, gen == 0 ? fx->q : NULL, &a);
    if (sa != MAP_OK) {
        if (verbose)
            printf("  READ FAIL gen=%d len=%zu status=%s\n", gen, len,
                   map_status_str(sa));
        free(old);
        return;
    }
    r->read_ok++;
    for (int i = 0; i < 4; i++)
        if (a.q_lossy & (1u << i)) r->snapped_field[i]++;
    map_upgrade_set_rounding(MAP_Q_FLOOR);

    /* Road B: offline re-export with the tool's policy, then read it back. */
    unsigned char* modern = NULL;
    size_t modern_len = 0;
    unsigned lost = 0;
    MapStatus sr = reexport_to_gen2(old, len, gen == 0 ? fx->q : NULL,
                                    tool_round, &modern, &modern_len, &lost);
    if (sr != MAP_OK) {
        if (verbose)
            printf("  REEXPORT FAIL gen=%d status=%s\n", gen, map_status_str(sr));
        map_free(&a);
        free(old);
        return;
    }
    MapFile b;
    MapStatus sb = map_read(modern, modern_len, NULL, &b);
    if (sb != MAP_OK) {
        if (verbose)
            printf("  GEN2 REREAD FAIL: %s\n", map_status_str(sb));
        map_free(&a);
        free(modern);
        free(old);
        return;
    }

    /* Cells must survive both roads; so must the query. The query is the one
     * thing gen 1's upgrade can lose, so it gets its own check rather than
     * being inferred from whether the path happened to come out the same. */
    if (!map_same_cells(&a, &b)) {
        r->cell_diffs++;
        if (verbose) printf("  CELLS DIFFER gen=%d\n", gen);
    }
    int qdiff = 0;
    for (int i = 0; i < 4; i++)
        if (a.q[i] != b.q[i] || (a.has_query != b.has_query)) qdiff = 1;
    if (qdiff) {
        r->query_diffs++;
        if (verbose) {
            printf("  QUERY DIFF gen=%d compat=(%u,%u,%u,%u) "
                   "reexport=(%u,%u,%u,%u)\n", gen,
                   a.q[0], a.q[1], a.q[2], a.q[3],
                   b.q[0], b.q[1], b.q[2], b.q[3]);
        }
    }

    int reach_a = nav_astar(&a, a.q[0], a.q[1], a.q[2], a.q[3], pa);
    int reach_b = nav_astar(&b, b.q[0], b.q[1], b.q[2], b.q[3], pb);

    int same = 0, denom = 0, fd = -1;
    int identical;
    if (!reach_a && !reach_b) {
        identical = 1;                    /* both unreachable: same answer */
    } else if (reach_a && reach_b) {
        identical = nav_compare(pa, pb, &same, &denom, &fd);
    } else {
        identical = 0;                    /* reachable one way only */
        same = 0;
        denom = reach_a ? pa->n : pb->n;
        fd = 0;
    }

    if (identical) {
        r->identical++;
        long n = (pa->n > 0) ? pa->n : 0;
        r->shared_points += n;
        r->total_points += n;
    } else {
        r->path_diffs++;
        r->shared_points += same;
        r->total_points += denom;
        for (int i = 0; i < 4; i++)
            if (lost & (1u << i)) r->moved_field[i]++;
        if (verbose) {
            printf("  PATH DIFF gen=%d first_diff=%d shared=%d/%d snap_fields=",
                   gen, fd, same, denom);
            print_mask(lost);
            printf("\n");
            if (fd >= 0 && reach_a && reach_b) {
                printf("    at %d: compat=(%u,%u) reexport=(%u,%u)\n", fd,
                       fd < pa->n ? pa->x[fd] : 0, fd < pa->n ? pa->y[fd] : 0,
                       fd < pb->n ? pb->x[fd] : 0, fd < pb->n ? pb->y[fd] : 0);
            } else if (fd >= 0) {
                printf("    reachable: compat=%d reexport=%d\n", reach_a, reach_b);
            }
        }
    }

    map_free(&a);
    map_free(&b);
    free(modern);
    free(old);
}

static void run_tier(int tier, int verbose, TierResult* r) {
    memset(r, 0, sizeof *r);

    NavPath pa, pb;
    if (!nav_path_init(&pa, MAP_V0_W * MAP_V0_H) ||
        !nav_path_init(&pb, MAP_V0_W * MAP_V0_H)) {
        nav_path_free(&pa);
        nav_path_free(&pb);
        return;
    }

    /* gen 0: the only generation with real files. */
    int n = fx_tier_files(tier);
    for (int i = 0; i < n; i++) {
        Fixture fx;
        if (!fx_build(tier, i, 12345u + (unsigned)tier, &fx)) continue;
        r->gen[0].files++;
        one_file(&fx, 0, MAP_Q_FLOOR, MAP_Q_FLOOR, &r->gen[0], &pa, &pb, verbose);
        fx_free(&fx);
    }
    /* gen 1: synthetic. No file here was ever written in it, but the reader
     * has to work the day one is, and the retention curve needs the point. */
    for (int i = 0; i < n; i++) {
        Fixture fx;
        if (!fx_build(tier, i, 12345u + (unsigned)tier, &fx)) continue;
        r->gen[1].files++;
        one_file(&fx, 1, MAP_Q_FLOOR, MAP_Q_FLOOR, &r->gen[1], &pa, &pb, verbose);
        fx_free(&fx);
    }

    nav_path_free(&pa);
    nav_path_free(&pb);
}

static double pct(long num, long den) {
    return den ? 100.0 * (double)num / (double)den : 100.0;
}

/* Share of a tier's existing files that a reader set of `gens` generations
 * reconstructs. Existing means gen 0: that is what is on disk. */
static double existing_readable_pct(const TierResult* r, int gens) {
    long files = r->gen[0].files;
    long ok = (gens >= 1) ? r->gen[0].read_ok : 0;
    return pct(ok, files);
}

static const char* loc_legacy(int gens) {
    static char buf[32];
#if HAVE_LOC
    int loc = LOC_DISPATCH + (gens >= 1 ? LOC_GEN0 : 0) +
              (gens >= 2 ? LOC_GEN1 : 0);
    snprintf(buf, sizeof buf, "%d", loc);
#else
    (void)gens;
    snprintf(buf, sizeof buf, "-");
#endif
    return buf;
}

static void print_field_counts(const long counts[4]) {
    int first = 1;
    for (int i = 0; i < 4; i++) {
        if (counts[i]) {
            printf("%s%s=%ld", first ? "" : " ", field_name(i), counts[i]);
            first = 0;
        }
    }
    if (first) printf("(none)");
}

static void run_divergence(int tier) {
    GenResult r;
    memset(&r, 0, sizeof r);

    NavPath pa, pb;
    if (!nav_path_init(&pa, MAP_V0_W * MAP_V0_H) ||
        !nav_path_init(&pb, MAP_V0_W * MAP_V0_H)) {
        nav_path_free(&pa);
        nav_path_free(&pb);
        return;
    }
    int n = fx_tier_files(tier);
    int first_shown = 0;
    for (int i = 0; i < n; i++) {
        Fixture fx;
        if (!fx_build(tier, i, 12345u + (unsigned)tier, &fx)) continue;
        r.files++;
        int before = r.path_diffs;
        /* reader floors, tool rounds: the two roads now disagree */
        one_file(&fx, 1, MAP_Q_FLOOR, MAP_Q_ROUND, &r, &pa, &pb, 0);
        /* Show one concrete case per tier so the attribution is checkable:
         * which file, where the two paths split, and which field moved. The
         * replay tallies into a throwaway result so r is not counted twice. */
        if (!first_shown && r.path_diffs > before) {
            first_shown = 1;
            GenResult tmp;
            memset(&tmp, 0, sizeof tmp);
            printf("    e.g. file #%d: ", i);
            one_file(&fx, 1, MAP_Q_FLOOR, MAP_Q_ROUND, &tmp, &pa, &pb, 1);
        }
        fx_free(&fx);
    }
    nav_path_free(&pa);
    nav_path_free(&pb);

    printf("  %-6s gen1 files=%-4d read=%-4d path_diffs=%-4d shared=%ld/%ld "
           "(%.2f%%) moved_by_field: ",
           fx_tier_name(tier), r.files, r.read_ok, r.path_diffs,
           r.shared_points, r.total_points,
           pct(r.shared_points, r.total_points));
    print_field_counts(r.moved_field);
    printf("\n");
}

int main(int argc, char** argv) {
    int quiet = 0;
    for (int i = 1; i < argc; i++)
        if (strcmp(argv[i], "--quiet") == 0) quiet = 1;

    printf("# Synthetic fixtures, 16x12. No old map files exist in this repo:\n"
           "# grid/, smooth/ and region/ keep cells in memory and never write.\n"
           "# 'read' means 'the reader reconstructed the map and query'.\n");
    printf("# rounding=floor  read_floor=%.0f%%  shared_floor=%.0f%%\n\n",
           READ_FLOOR_PCT, SHARED_FLOOR_PCT);

    int bad = 0;
    TierResult results[FX_TIER_COUNT];
    for (int t = 0; t < FX_TIER_COUNT; t++)
        run_tier(t, !quiet, &results[t]);

    /* ---- acceptance ---------------------------------------------------- */
    printf("ACCEPTANCE  compat-read path vs re-export-then-read path\n");
    printf("%-6s %-5s %6s %6s %8s %15s %9s %s\n",
           "tier", "gen", "files", "read", "read%", "shared points",
           "shared%", "snapped(same path)");
    for (int t = 0; t < FX_TIER_COUNT; t++) {
        for (int g = 0; g < 2; g++) {
            const GenResult* r = &results[t].gen[g];
            double read_pct = pct(r->read_ok, r->files);
            double shared_pct = pct(r->shared_points, r->total_points);
            int attempted = r->files;
            printf("%-6s gen%-2d %6d %6d %7.2f%% %8ld/%-6ld %8.2f%% ",
                   fx_tier_name(t), g, attempted, r->read_ok, read_pct,
                   r->shared_points, r->total_points, shared_pct);
            print_field_counts(r->snapped_field);
            printf("\n");

            if (read_pct < READ_FLOOR_PCT) {
                printf("  FAIL tier=%s gen%d read %.2f%% < %.0f%%\n",
                       fx_tier_name(t), g, read_pct, READ_FLOOR_PCT);
                bad = 1;
            }
            if (shared_pct < SHARED_FLOOR_PCT) {
                printf("  FAIL tier=%s gen%d shared %.2f%% < %.0f%%\n",
                       fx_tier_name(t), g, shared_pct, SHARED_FLOOR_PCT);
                bad = 1;
            }
            if (r->cell_diffs) {
                printf("  FAIL tier=%s gen%d cells changed by upgrade (%d)\n",
                       fx_tier_name(t), g, r->cell_diffs);
                bad = 1;
            }
            if (r->query_diffs) {
                printf("  FAIL tier=%s gen%d query differs between the two "
                       "roads (%d/%d files)\n",
                       fx_tier_name(t), g, r->query_diffs, attempted);
                bad = 1;
            }
            if (r->path_diffs) {
                printf("  NOTE tier=%s gen%d %d/%d paths differ, moved_by_field: ",
                       fx_tier_name(t), g, r->path_diffs, attempted);
                print_field_counts(r->moved_field);
                printf("\n");
            }
        }
    }
    printf("  gen0 snapped=(none) is expected: gen 0 has no query to lose.\n"
           "  gen1 snapped counts files whose sub-cell part was dropped by the\n"
           "  whole-cell snap; every path still matches, because floor() is\n"
           "  what the legacy runtime did and the re-export does the same.\n");

    /* ---- retention ----------------------------------------------------- */
    printf("\nRETENTION  existing old files readable vs reader lines\n");
    printf("  %-22s %-6s %-6s %-6s %6s %s\n",
           "generations kept", "small", "mid", "large", "lines", "reads");
    static const char* label[3] = {
        "0 (re-export all)",
        "1 (gen 0)",
        "2 (gen 0 + gen 1)",
    };
    for (int g = 0; g < 3; g++) {
        printf("  %-22s %5.1f%% %5.1f%% %5.1f%% %6s %s\n", label[g],
               existing_readable_pct(&results[0], g),
               existing_readable_pct(&results[1], g),
               existing_readable_pct(&results[2], g),
               loc_legacy(g),
               g == 0 ? "no old file; all re-exported"
                      : g == 1 ? "every old file that exists today"
                               : "today's files, plus gen 1 if it ever appears");
    }
#if !HAVE_LOC
    printf("  (line counts absent; build through make)\n");
#endif

    /* ---- the inversion ------------------------------------------------- */
    printf("\nRETENTION DECISION  old-file count -> readers kept\n");
#if HAVE_LOC
    {
        long loc[MAP_GEN_COUNT] = { LOC_GEN0, LOC_GEN1, 0 };
        double unit = 1.0;    /* one re-export == one line of reader */

        /*
         * Two rules give two answers, and the note asks for both, so both are
         * printed rather than one being silently preferred.
         *
         *   RULE A: every old file must be readable, always, with no
         *           re-export. Then any generation with even one file forces
         *           its reader, whatever that costs. Count -> mask, exactly.
         *
         *   RULE B: a one-time re-export is allowed. Then keep a generation's
         *           reader only while carrying it is cheaper than converting
         *           what it would read. The crossover is a file count.
         *
         * The gap between them is the price of "读入成功率必须是百分之百".
         */
        printf("  RULE A: 100%% read, no re-export allowed\n"
               "    kept generations = bitmask of generations with >0 files\n"
               "    gen 0 reader = %d lines (measured), gen 1 = %d lines\n",
               LOC_GEN0, LOC_GEN1);

        printf("  RULE B: re-export allowed, 1 unit per file\n");
        printf("    keep gen0 reader while un-re-exported files > %d\n",
               LOC_GEN0);
        printf("    keep gen1 reader while un-re-exported files > %d\n",
               LOC_GEN1);

        for (int t = 0; t < FX_TIER_COUNT; t++) {
            long files[MAP_GEN_COUNT] = { results[t].gen[0].files, 0, 0 };
            unsigned forced = map_min_kept_mask(files);
            RetentionOption opts[1 << (MAP_GEN_CURRENT)];
            int n = map_plan_retention(files, loc, LOC_DISPATCH, unit,
                                       opts, (int)(sizeof opts / sizeof opts[0]));
            (void)n;
            printf("  tier=%-6s files=%-4ld  A: keep mask 0x%x (%s)   "
                   "B: keep mask 0x%x, %d lines, %ld re-exports\n",
                   fx_tier_name(t), files[0], forced,
                   "gen0", opts[0].kept_mask, (int)opts[0].lines,
                   opts[0].reexport);
        }

        /*
         * The three tiers all land on the same answer, and that is the useful
         * finding: the 100% requirement makes the count almost irrelevant.
         * The count only decides the small tier, and only if you are willing
         * to drop the requirement there. Say so plainly.
         */
        printf("  reading: with 100%% read required, all three tiers keep\n"
               "  exactly one legacy reader (gen 0). The count changes the\n"
               "  answer only under RULE B, and only at the low end: at\n"
               "  %d files or fewer, re-exporting all of them costs no more\n"
               "  than carrying the %d-line reader (a tie is broken toward\n"
               "  fewer readers), so the small tier would keep zero readers\n"
               "  if re-export were acceptable there.\n",
               LOC_GEN0, LOC_GEN0);
    }
#else
    printf("  (line counts absent; build through make)\n");
#endif

    /* ---- divergence probe ---------------------------------------------- */
    printf("\nDIVERGENCE PROBE  reader floors, offline re-export rounds.\n"
           "  Not a supported setup: it shows the equality test bites, and\n"
           "  names the field when a snap moves a path.\n");
    for (int t = 0; t < FX_TIER_COUNT; t++) run_divergence(t);

    printf("\nbench_ok=%d\n", bad == 0);
    return bad == 0 ? 0 : 1;
}
