#include <stdlib.h>
#include <string.h>

#include "internal.h"

/* ==========================================================================
 * gen 0: the bare 16x12 cell array.
 *
 * No dimensions, no query, no flag byte: 192 cells in row-major order, which
 * is exactly `unsigned char blocked[16*12]` in grid/, smooth/ and region/.
 * This is the only generation with real files today, and the only one whose
 * upgrade is lossless: everything it holds is cells, and gen 2 holds cells.
 * ========================================================================== */

int v0_shape_ok(const unsigned char* buf, size_t len, uint16_t* w, uint16_t* h) {
    if (len != MAP_V0_BYTES) return 0;
    if (!cells_payload_ok(buf, MAP_V0_W, MAP_V0_H)) return 0;
    *w = MAP_V0_W;
    *h = MAP_V0_H;
    return 1;
}

MapStatus v0_parse(const unsigned char* buf, size_t len,
                   const uint16_t* fallback_q, MapFile* out) {
    uint16_t w, h;
    if (!v0_shape_ok(buf, len, &w, &h)) return MAP_ERR_FORMAT;

    MapStatus err;
    unsigned char* cells = alloc_cells(w, h, &err);
    if (!cells) return err;
    memcpy(cells, buf, MAP_V0_BYTES);

    out->version = 0;
    out->upgraded = 1;
    out->w = w;
    out->h = h;
    out->cells = cells;

    /* The file has no query, so the caller's level data supplies one. Whole
     * cells already, so it passes through with no snap and no loss. */
    if (fallback_q) {
        if (fallback_q[0] >= w || fallback_q[2] >= w ||
            fallback_q[1] >= h || fallback_q[3] >= h) {
            free(cells);
            return MAP_ERR_PAYLOAD;
        }
        for (int i = 0; i < 4; i++) {
            out->q[i] = fallback_q[i];
            out->q_exact[i] = (double)fallback_q[i];
        }
        out->q_lossy = 0;
        out->has_query = 1;
    }
    return MAP_OK;
}
