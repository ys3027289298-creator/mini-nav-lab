#ifndef MAP_INTERNAL_H
#define MAP_INTERNAL_H

#include <stddef.h>
#include <stdint.h>

#include "mapfile.h"

/* ---- little-endian accessors, shared by every generation ---------------- */

static inline uint16_t rd16(const unsigned char* p) {
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t rd32(const unsigned char* p) {
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) |
           ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline float rdf32(const unsigned char* p) {
    uint32_t bits = rd32(p);
    float f;
    __builtin_memcpy(&f, &bits, sizeof f); /* not a cast: aliasing, alignment */
    return f;
}

static inline void wr16(unsigned char* p, uint16_t v) {
    p[0] = (unsigned char)(v & 0xff);
    p[1] = (unsigned char)(v >> 8);
}

static inline void wr32(unsigned char* p, uint32_t v) {
    p[0] = (unsigned char)(v & 0xff);
    p[1] = (unsigned char)(v >> 8);
    p[2] = (unsigned char)(v >> 16);
    p[3] = (unsigned char)(v >> 24);
}

static inline void wrf32(unsigned char* p, float f) {
    uint32_t bits;
    __builtin_memcpy(&bits, &f, sizeof bits);
    wr32(p, bits);
}

/* ---- payload helpers --------------------------------------------------- */

/* Every payload byte must be 0 or 1. A stray value means the file is not the
 * generation it claims; clamping it would hide a real format mismatch. */
int cells_payload_ok(const unsigned char* cells, uint16_t w, uint16_t h);

unsigned char* alloc_cells(uint16_t w, uint16_t h, MapStatus* err);

/* A width/height pair we are willing to allocate for. */
static inline int dims_ok(uint16_t w, uint16_t h) {
    return w >= 1 && h >= 1 && w <= MAP_MAX_W && h <= MAP_MAX_H;
}

/* ---- gen 2 (current) --------------------------------------------------- */

int       v2_header_ok(const unsigned char* buf, size_t len);
MapStatus v2_parse(const unsigned char* buf, size_t len, MapFile* out);

/* ---- gen 1 (legacy: w,h + cells + f32 query in cell units) ------------- */

int       v1_shape_ok(const unsigned char* buf, size_t len,
                      uint16_t* w, uint16_t* h);
MapStatus v1_parse(const unsigned char* buf, size_t len, MapFile* out);

/* ---- gen 0 (legacy: bare 16x12 cell array) ----------------------------- */

int       v0_shape_ok(const unsigned char* buf, size_t len,
                      uint16_t* w, uint16_t* h);
MapStatus v0_parse(const unsigned char* buf, size_t len,
                   const uint16_t* fallback_q, MapFile* out);

#endif
