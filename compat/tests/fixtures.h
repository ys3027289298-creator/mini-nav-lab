#ifndef FIXTURES_H
#define FIXTURES_H

#include <stddef.h>
#include <stdint.h>

#include "mapfile.h"

/*
 * Synthetic old maps for the compatibility bench.
 *
 * These are the only maps this module has ever seen. The three modules in
 * grid/, smooth/ and region/ keep their cells in a `blocked[16*12]` array in
 * memory and never write a file, so "old maps" here means "what an archive
 * would hold if it had ever written them". Every fixture is fabricated and
 * the report says so; what is being measured is the reader, not a corpus.
 *
 * Tiers are counts of old files, not map sizes: gen 0 is fixed at 16x12 by
 * definition, so size cannot vary across it. The count is the quantity the
 * retention decision is a function of.
 */

typedef struct {
    uint16_t w, h;
    unsigned char* cells;
    uint16_t q[4];        /* whole-cell query: sx, sy, gx, gy */
    float q_sub[4];       /* gen-1 f32 query: cells plus a sub-cell jitter */
} Fixture;

enum {
    FX_TIER_SMALL,        /* 8 old files: the state today */
    FX_TIER_MID,          /* 80: a few releases of drift */
    FX_TIER_LARGE,        /* 800: legacy debt */
    FX_TIER_COUNT
};

int         fx_tier_files(int tier);
const char* fx_tier_name(int tier);

/* Deterministic from (tier, index, seed). Fills fx; returns 1 on success. */
int  fx_build(int tier, int index, unsigned seed, Fixture* fx);
void fx_free(Fixture* fx);

/* Old-generation serialisers. Both allocate; caller frees. */
unsigned char* fx_emit_gen0(const Fixture* fx, size_t* len);
unsigned char* fx_emit_gen1(const Fixture* fx, size_t* len);

/* The same cells as gen 2, through the shared writer, with an explicit query.
 * Callers pass fx->q for gen 0 (whole cells already) and the shared
 * quantiser's output over fx->q_sub for gen 1. Returns malloc'd bytes. */
unsigned char* fx_emit_gen2(const Fixture* fx, const uint16_t query[4],
                            size_t* len);

#endif
