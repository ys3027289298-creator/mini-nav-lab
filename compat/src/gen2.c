#include <stdlib.h>
#include <string.h>

#include "internal.h"

/* ==========================================================================
 * gen 2: the current, self-describing format. Not a compatibility shim; it
 * is the format every other generation is upgraded into.
 * ========================================================================== */

int v2_header_ok(const unsigned char* buf, size_t len) {
    if (len < 12) return 0;
    if (buf[0] != MAP_MAGIC0 || buf[1] != MAP_MAGIC1 ||
        buf[2] != MAP_MAGIC2 || buf[3] != '\0') return 0;
    if (rd16(buf + 4) != MAP_GEN_CURRENT) return 0;
    if (buf[11] != 0) return 0;                          /* reserved */
    if (buf[10] & (unsigned char)~MAP_FLAG_KNOWN) return 0;
    return dims_ok(rd16(buf + 6), rd16(buf + 8));
}

/* Decode `h` RLE rows starting at p, and report how many bytes they used.
 * Run lengths are u8, so a row wider than 255 is split into several runs;
 * that is the encoder's job, the decoder just refuses runs crossing the row
 * edge or leaving it short. */
static MapStatus rle_decode_rows(const unsigned char* p, size_t avail,
                                 unsigned char* cells, uint16_t w, uint16_t h,
                                 size_t* used) {
    size_t off = 0;
    for (uint16_t y = 0; y < h; y++) {
        if (avail - off < 2) return MAP_ERR_TRUNCATED;
        uint16_t runs = rd16(p + off);
        off += 2;
        size_t x = 0;
        for (uint16_t r = 0; r < runs; r++) {
            if (avail - off < 2) return MAP_ERR_TRUNCATED;
            unsigned char v = p[off], n = p[off + 1];
            off += 2;
            if (v > 1 || n == 0) return MAP_ERR_PAYLOAD;
            if (x + n > w) return MAP_ERR_PAYLOAD;   /* run crosses the edge */
            memset(cells + (size_t)y * w + x, v, n);
            x += n;
        }
        if (x != w) return MAP_ERR_PAYLOAD;          /* short row */
    }
    *used = off;
    return MAP_OK;
}

MapStatus v2_parse(const unsigned char* buf, size_t len, MapFile* out) {
    uint16_t w = rd16(buf + 6), h = rd16(buf + 8);
    unsigned flags = buf[10];
    size_t off = 12;
    MapStatus err;

    unsigned char* cells = alloc_cells(w, h, &err);
    if (!cells) return err;

    if (flags & MAP_FLAG_RLE) {
        size_t used = 0;
        err = rle_decode_rows(buf + off, len - off, cells, w, h, &used);
        if (err != MAP_OK) { free(cells); return err; }
        out->from_rle = 1;
        off += used;
    } else {
        size_t n = (size_t)w * h;
        if (len - off < n) { free(cells); return MAP_ERR_TRUNCATED; }
        memcpy(cells, buf + off, n);
        if (!cells_payload_ok(cells, w, h)) { free(cells); return MAP_ERR_PAYLOAD; }
        off += n;
    }

    if (flags & MAP_FLAG_QUERY) {
        if (len - off < 8) { free(cells); return MAP_ERR_TRUNCATED; }
        for (int i = 0; i < 4; i++) out->q[i] = rd16(buf + off + 2 * i);
        if (out->q[0] >= w || out->q[2] >= w ||
            out->q[1] >= h || out->q[3] >= h) { free(cells); return MAP_ERR_PAYLOAD; }
        for (int i = 0; i < 4; i++) out->q_exact[i] = (double)out->q[i];
        out->has_query = 1;
        off += 8;
    }
    if (off != len) { free(cells); return MAP_ERR_PAYLOAD; }

    out->version = MAP_GEN_CURRENT;
    out->upgraded = 0;
    out->w = w;
    out->h = h;
    out->cells = cells;
    return MAP_OK;
}

/* ---- writer ------------------------------------------------------------ */

static MapStatus rle_encode_rows(const unsigned char* cells, uint16_t w,
                                 uint16_t h, unsigned char* dst, size_t cap,
                                 size_t* used) {
    size_t off = 0;
    for (uint16_t y = 0; y < h; y++) {
        const unsigned char* row = cells + (size_t)y * w;
        /* count runs first: the u16 run count precedes them */
        size_t runs = 0;
        for (uint16_t x = 0; x < w; ) {
            unsigned char v = row[x];
            size_t n = 1;
            while (x + n < w && row[x + n] == v && n < 255) n++;
            runs++;
            x = (uint16_t)(x + n);
        }
        if (runs > 65535) return MAP_ERR_PAYLOAD;
        if (cap - off < 2 + runs * 2) return MAP_ERR_NO_MEM;
        wr16(dst + off, (uint16_t)runs);
        off += 2;
        for (uint16_t x = 0; x < w; ) {
            unsigned char v = row[x];
            unsigned char n = 1;
            while (x + n < w && row[x + n] == v && n < 255) n++;
            dst[off] = v;
            dst[off + 1] = n;
            off += 2;
            x = (uint16_t)(x + n);
        }
    }
    *used = off;
    return MAP_OK;
}

MapStatus map_write(const unsigned char* cells, uint16_t w, uint16_t h,
                    const uint16_t query[4], unsigned char** out_buf,
                    size_t* out_len) {
    if (!cells || !out_buf || !out_len || !dims_ok(w, h)) return MAP_ERR_DIM;
    *out_buf = NULL;
    *out_len = 0;

    size_t n = (size_t)w * h;
    if (!cells_payload_ok(cells, w, h)) return MAP_ERR_PAYLOAD;

    /* RLE is an optimisation, never a semantic choice: pick it only when the
     * encoded payload is actually shorter, so a round trip is byte-lossless
     * whichever layout wins. */
    /* Worst case is every cell its own run: 2 bytes per cell plus the u16 run
     * count per row. RLE only wins when it is shorter than raw, so this is an
     * upper bound we allocate once rather than a size we expect. */
    size_t cap = 2 + 2 * n + 2 * (size_t)h + 16;
    unsigned char* rle = (unsigned char*)malloc(cap);
    if (!rle) return MAP_ERR_NO_MEM;
    size_t rle_len = 0;
    MapStatus st = rle_encode_rows(cells, w, h, rle, cap, &rle_len);
    if (st != MAP_OK) { free(rle); return st; }

    int use_rle = rle_len < n;
    size_t payload = use_rle ? rle_len : n;
    size_t total = 12 + payload + (query ? 8 : 0);
    unsigned char* out = (unsigned char*)malloc(total);
    if (!out) { free(rle); return MAP_ERR_NO_MEM; }

    out[0] = MAP_MAGIC0;
    out[1] = MAP_MAGIC1;
    out[2] = MAP_MAGIC2;
    out[3] = '\0';
    wr16(out + 4, MAP_GEN_CURRENT);
    wr16(out + 6, w);
    wr16(out + 8, h);
    out[10] = (unsigned char)((use_rle ? MAP_FLAG_RLE : 0) |
                              (query ? MAP_FLAG_QUERY : 0));
    out[11] = 0;

    size_t off = 12;
    if (use_rle) {
        memcpy(out + off, rle, rle_len);
        off += rle_len;
    } else {
        memcpy(out + off, cells, n);
        off += n;
    }
    free(rle);

    if (query) {
        for (int i = 0; i < 4; i++) wr16(out + off + 2 * i, query[i]);
        off += 8;
    }

    *out_buf = out;
    *out_len = off;
    return MAP_OK;
}
