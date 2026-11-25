#include <stdlib.h>
#include <string.h>
#include <math.h>

#include "internal.h"

/* ---- status / names ---------------------------------------------------- */

const char* map_status_str(MapStatus s) {
    switch (s) {
    case MAP_OK:            return "ok";
    case MAP_ERR_IO:        return "io error";
    case MAP_ERR_TRUNCATED: return "truncated";
    case MAP_ERR_MAGIC:     return "bad magic";
    case MAP_ERR_VERSION:   return "unsupported version";
    case MAP_ERR_DIM:       return "bad dimensions";
    case MAP_ERR_PAYLOAD:   return "bad payload";
    case MAP_ERR_NO_MEM:    return "out of memory";
    case MAP_ERR_FORMAT:    return "unrecognised format";
    }
    return "unknown";
}

const char* map_gen_name(int gen) {
    switch (gen) {
    case 0: return "v0-raw";
    case 1: return "v1-fixedpoint";
    case 2: return "v2-header";
    }
    return "unknown";
}

int map_gen_supported(int gen) {
    switch (gen) {
    case 0: return MAP_KEEP_V0 ? 1 : 0;
    case 1: return MAP_KEEP_V1 ? 1 : 0;
    case 2: return 1;
    }
    return 0;
}

/* ---- quantisation ------------------------------------------------------ */

static MapQRounding g_rounding = MAP_Q_FLOOR;

void map_upgrade_set_rounding(MapQRounding mode) { g_rounding = mode; }
MapQRounding map_upgrade_get_rounding(void)      { return g_rounding; }

uint16_t map_q_quantise(double exact, unsigned* lossy) {
    double snapped = (g_rounding == MAP_Q_ROUND) ? floor(exact + 0.5)
                                                 : floor(exact);
    if (snapped < 0.0) snapped = 0.0;
    if (snapped > 65535.0) snapped = 65535.0;
    if (snapped != exact && lossy) *lossy = 1;
    return (uint16_t)snapped;
}

/* ---- helpers ----------------------------------------------------------- */

int cells_payload_ok(const unsigned char* cells, uint16_t w, uint16_t h) {
    size_t n = (size_t)w * (size_t)h;
    for (size_t i = 0; i < n; i++)
        if (cells[i] > 1) return 0;
    return 1;
}

unsigned char* alloc_cells(uint16_t w, uint16_t h, MapStatus* err) {
    size_t n = (size_t)w * (size_t)h;
    unsigned char* p = (unsigned char*)malloc(n);
    if (!p) { *err = MAP_ERR_NO_MEM; return NULL; }
    *err = MAP_OK;
    return p;
}

/* ==========================================================================
 * dispatch
 * ========================================================================== */

int map_detect_version(const unsigned char* buf, size_t buf_len) {
    if (!buf) return MAP_DET_UNKNOWN;

    /* Header first. If the current structure's header parses, the answer is
     * "current" and no legacy shape is consulted. Magic wins: a gen-2 file
     * with a corrupt payload is reported as a gen-2 error and never demoted
     * to a legacy guess. */
    if (buf_len >= 4 && buf[0] == MAP_MAGIC0 && buf[1] == MAP_MAGIC1 &&
        buf[2] == MAP_MAGIC2 && buf[3] == '\0') {
        if (buf_len < 12) return MAP_DET_UNKNOWN;
        if (rd16(buf + 4) != MAP_GEN_CURRENT) return MAP_DET_FUTURE;
        return v2_header_ok(buf, buf_len) ? MAP_GEN_CURRENT : MAP_DET_UNKNOWN;
    }

    /* Otherwise the old structures. gen 1 is checked before gen 0; the two
     * length equations cannot both hold, and the cell-value check is what
     * keeps a coincidental length from counting as a match. */
    if (buf_len == MAP_V0_BYTES) {
        uint16_t w, h;
        if (v0_shape_ok(buf, buf_len, &w, &h)) return 0;
    }
    if (buf_len >= 4) {
        uint16_t w, h;
        if (v1_shape_ok(buf, buf_len, &w, &h)) return 1;
    }

    return MAP_DET_UNKNOWN;
}

MapStatus map_read(const unsigned char* buf, size_t buf_len,
                   const uint16_t* fallback_q, MapFile* out) {
    if (!buf || !out) return MAP_ERR_IO;
    memset(out, 0, sizeof *out);
#if !MAP_KEEP_V0
    /* Only gen 0 consumes this; keep the parameter (the API is one function
     * for every build) but say so, rather than warning in the end-state
     * build where gen 0 is the generation being retired. */
    (void)fallback_q;
#endif

    switch (map_detect_version(buf, buf_len)) {
    case MAP_GEN_CURRENT:
        return v2_parse(buf, buf_len, out);
    case 1:
#if MAP_KEEP_V1
        return v1_parse(buf, buf_len, out);
#else
        /* The shape is recognised, the reader is not in this build. That is
         * a different answer from "not a map at all", and the retention
         * report depends on being able to tell them apart. */
        return MAP_ERR_VERSION;
#endif
    case 0:
#if MAP_KEEP_V0
        return v0_parse(buf, buf_len, fallback_q, out);
#else
        return MAP_ERR_VERSION;
#endif
    case MAP_DET_FUTURE:
        return MAP_ERR_VERSION;
    default:
        return MAP_ERR_FORMAT;
    }
}

void map_free(MapFile* m) {
    if (!m) return;
    free(m->cells);
    m->cells = NULL;
}

int map_same_cells(const MapFile* a, const MapFile* b) {
    if (!a || !b || !a->cells || !b->cells) return 0;
    if (a->w != b->w || a->h != b->h) return 0;
    return memcmp(a->cells, b->cells, (size_t)a->w * a->h) == 0;
}
