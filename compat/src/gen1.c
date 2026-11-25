#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "internal.h"

/* ==========================================================================
 * gen 1: headerless w,h + cells + f32 query.
 *
 * This is the generation the repo never wrote, but it is the shape a
 * "we added w/h but never a version field" exporter produces, and it is the
 * one that costs a reader. It exists here so the retention decision can be
 * evaluated for N = 2 generations, not only for the single raw array.
 * ========================================================================== */

int v1_shape_ok(const unsigned char* buf, size_t len, uint16_t* w, uint16_t* h) {
    if (len < 4 + 16) return 0;
    uint16_t ww = rd16(buf), hh = rd16(buf + 2);
    if (!dims_ok(ww, hh)) return 0;
    size_t cells = (size_t)ww * (size_t)hh;
    if (len != 4 + cells + 16) return 0;
    if (!cells_payload_ok(buf + 4, ww, hh)) return 0;

    /* The query is four f32 cell coordinates. Anything outside the grid, or
     * not finite, means this is not a gen-1 file; refuse rather than clamp,
     * because a clamped query would silently move a path. */
    for (int i = 0; i < 4; i++) {
        float q = rdf32(buf + 4 + cells + 4 * i);
        if (!isfinite(q) || q < 0.0f || q >= 65536.0f) return 0;
        if (q >= (float)((i % 2 == 0) ? ww : hh)) return 0;
    }

    *w = ww;
    *h = hh;
    return 1;
}

MapStatus v1_parse(const unsigned char* buf, size_t len, MapFile* out) {
    uint16_t w, h;
    if (!v1_shape_ok(buf, len, &w, &h)) return MAP_ERR_FORMAT;

    size_t n = (size_t)w * h;
    MapStatus err;
    unsigned char* cells = alloc_cells(w, h, &err);
    if (!cells) return err;
    memcpy(cells, buf + 4, n);

    unsigned lossy = 0;
    for (int i = 0; i < 4; i++) {
        double exact = (double)rdf32(buf + 4 + n + 4 * i);
        unsigned field = 0;
        out->q_exact[i] = exact;
        out->q[i] = map_q_quantise(exact, &field);
        if (field) lossy |= 1u << i;   /* q_lossy is one bit per field */
    }
    out->q_lossy = lossy;

    out->version = 1;
    out->upgraded = 1;
    out->has_query = 1;
    out->w = w;
    out->h = h;
    out->cells = cells;
    return MAP_OK;
}
