#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "mapfile.h"
#include "nav.h"
#include "fixtures.h"
#include "reexport.h"

static int failures = 0;

static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

/* ---- small hand-built files ------------------------------------------- */

static void put16(unsigned char* p, uint16_t v) {
    p[0] = (unsigned char)(v & 0xff);
    p[1] = (unsigned char)(v >> 8);
}

static void putf32(unsigned char* p, float f) {
    uint32_t b;
    memcpy(&b, &f, sizeof b);
    p[0] = (unsigned char)(b & 0xff);
    p[1] = (unsigned char)((b >> 8) & 0xff);
    p[2] = (unsigned char)((b >> 16) & 0xff);
    p[3] = (unsigned char)((b >> 24) & 0xff);
}

static void test_gen0(void) {
    Fixture fx;
    if (!fx_build(FX_TIER_SMALL, 0, 1, &fx)) { check("gen0_build", 0); return; }
    size_t len = 0;
    unsigned char* raw = fx_emit_gen0(&fx, &len);
    check("gen0_len", len == 192);
    check("gen0_detect", map_detect_version(raw, len) == 0);

    /* Shape detection is independent of whether this build keeps the reader,
     * so a dropped generation is reported as "unsupported", not "not a map". */
    if (!map_gen_supported(0)) {
        MapFile m;
        check("gen0_dropped_read_refused",
              map_read(raw, len, fx.q, &m) == MAP_ERR_VERSION);
        check("gen0_dropped_not_format_err",
              map_read(raw, len, fx.q, &m) != MAP_ERR_FORMAT);
        free(raw);
        fx_free(&fx);
        return;
    }

    MapFile m;
    MapStatus st = map_read(raw, len, fx.q, &m);
    check("gen0_read_ok", st == MAP_OK);
    check("gen0_dims", m.w == 16 && m.h == 12);
    check("gen0_version", m.version == 0 && m.upgraded == 1);
    check("gen0_cells", memcmp(m.cells, fx.cells, 192) == 0);
    check("gen0_query", m.has_query && m.q[0] == fx.q[0] && m.q[3] == fx.q[3]);
    check("gen0_lossless", m.q_lossy == 0);
    map_free(&m);

    /* no fallback query: cells still read, query simply absent */
    st = map_read(raw, len, NULL, &m);
    check("gen0_noquery_ok", st == MAP_OK && !m.has_query);
    map_free(&m);

    /* a byte that is not 0/1 is not this generation */
    raw[100] = 7;
    check("gen0_dirty_cell_rejected", map_detect_version(raw, len) == MAP_DET_UNKNOWN);
    free(raw);
    fx_free(&fx);
}

static void test_gen1(void) {
    Fixture fx;
    if (!fx_build(FX_TIER_SMALL, 1, 2, &fx)) { check("gen1_build", 0); return; }
    size_t len = 0;
    unsigned char* raw = fx_emit_gen1(&fx, &len);
    check("gen1_len", len == 4 + 192 + 16);
    /* The shape is detected whether or not this build keeps the reader: that
     * is what lets a dropped generation answer "not supported" instead of
     * "not a map". */
    check("gen1_detect", map_detect_version(raw, len) == 1);

    if (!map_gen_supported(1)) {
        MapFile m;
        check("gen1_dropped_read_refused",
              map_read(raw, len, NULL, &m) == MAP_ERR_VERSION);
        check("gen1_dropped_not_format_err",
              map_read(raw, len, NULL, &m) != MAP_ERR_FORMAT);
        free(raw);
        fx_free(&fx);
        return;
    }

    MapFile m;
    MapStatus st = map_read(raw, len, NULL, &m);
    check("gen1_read_ok", st == MAP_OK);
    check("gen1_version", m.version == 1 && m.upgraded == 1);
    check("gen1_cells", memcmp(m.cells, fx.cells, 192) == 0);
    /* floor of q+fraction is q, so the whole-cell query is recovered exactly */
    check("gen1_query_floor", m.q[0] == fx.q[0] && m.q[1] == fx.q[1] &&
                              m.q[2] == fx.q[2] && m.q[3] == fx.q[3]);
    /* every fixture's gen-1 query has a fraction, so the snap is lossy by
     * construction; the reader must say so, per field */
    check("gen1_lossy_flagged", m.q_lossy != 0);
    int expect_lossy = 0;
    for (int i = 0; i < 4; i++)
        if ((double)fx.q_sub[i] != (double)m.q[i]) expect_lossy |= 1 << i;
    check("gen1_lossy_fields", (int)m.q_lossy == expect_lossy);
    map_free(&m);

    /* a query outside the grid is not a gen-1 file */
    putf32(raw + 4 + 192 + 8, 900.0f);
    check("gen1_out_of_range_rejected",
          map_detect_version(raw, len) == MAP_DET_UNKNOWN);
    free(raw);
    fx_free(&fx);
}

static void test_gen2_roundtrip(void) {
    Fixture fx;
    if (!fx_build(FX_TIER_SMALL, 2, 3, &fx)) { check("gen2_build", 0); return; }
    size_t len = 0;
    unsigned char* raw = fx_emit_gen2(&fx, fx.q, &len);
    check("gen2_detect", map_detect_version(raw, len) == MAP_GEN_CURRENT);
    check("gen2_magic", raw[0] == 'M' && raw[1] == 'N' && raw[2] == 'L' && raw[3] == 0);

    MapFile m;
    check("gen2_read_ok", map_read(raw, len, NULL, &m) == MAP_OK);
    check("gen2_cells", memcmp(m.cells, fx.cells, 192) == 0);
    check("gen2_query", m.has_query && m.q[2] == fx.q[2]);
    check("gen2_not_upgraded", m.version == 2 && m.upgraded == 0);
    map_free(&m);

    /* rewriting what we read must reproduce the same bytes: RLE choice is
     * deterministic, so anything else means the encoder is not stable */
    size_t len2 = 0;
    unsigned char* raw2 = fx_emit_gen2(&fx, fx.q, &len2);
    check("gen2_stable_bytes", len == len2 && memcmp(raw, raw2, len) == 0);
    free(raw2);
    free(raw);
    fx_free(&fx);
}

static void test_rejections(void) {
    unsigned char junk[512];
    memset(junk, 0x41, sizeof junk);
    MapFile m;
    check("junk_rejected", map_read(junk, sizeof junk, NULL, &m) == MAP_ERR_FORMAT);

    /* magic says gen 2 but the version word says later: not ours, and we do
     * not guess it is a legacy file just because the payload won't parse */
    unsigned char future[13] = {'M','N','L',0, 9,0, 1,0, 1,0, 0,0, 0};
    check("future_version_detected",
          map_detect_version(future, sizeof future) == MAP_DET_FUTURE);
    check("future_version_refused",
          map_read(future, sizeof future, NULL, &m) == MAP_ERR_VERSION);

    /* truncated gen 2 header */
    unsigned char short_hdr[8] = {'M','N','L',0, 2,0, 1,0};
    check("short_header_refused", map_read(short_hdr, sizeof short_hdr, NULL, &m)
                                   != MAP_OK);

    /* gen-2 header, raw payload one byte short */
    unsigned char buf[12 + 16];
    memcpy(buf, "MNL", 3); buf[3] = 0;
    put16(buf + 4, 2); put16(buf + 6, 4); put16(buf + 8, 4);
    buf[10] = 0; buf[11] = 0;
    memset(buf + 12, 0, 16);
    check("gen2_ok_len", map_read(buf, 12 + 16, NULL, &m) == MAP_OK);
    map_free(&m);
    check("gen2_short_payload", map_read(buf, 12 + 15, NULL, &m) == MAP_ERR_TRUNCATED);

    /* reserved byte and unknown flag bits are refused */
    buf[11] = 1;
    check("gen2_reserved", map_read(buf, 12 + 16, NULL, &m) == MAP_ERR_FORMAT);
    buf[11] = 0;
    buf[10] = 0x80;
    check("gen2_unknown_flag", map_read(buf, 12 + 16, NULL, &m) == MAP_ERR_FORMAT);
}

static void test_rle(void) {
    /* a mostly-empty 64x64 map: RLE must win and decode exactly */
    enum { W = 64, H = 64 };
    unsigned char cells[W * H];
    memset(cells, 0, sizeof cells);
    for (int y = 0; y < H; y++) cells[y * W + (y % W)] = 1;

    unsigned char* buf = NULL;
    size_t len = 0;
    uint16_t q[4] = {1, 2, 3, 4};
    check("rle_write_ok", map_write(cells, W, H, q, &buf, &len) == MAP_OK);
    check("rle_chosen", len < 12 + W * H);
    check("rle_flag", (buf[10] & MAP_FLAG_RLE) != 0);

    MapFile m;
    check("rle_read_ok", map_read(buf, len, NULL, &m) == MAP_OK);
    check("rle_from_rle", m.from_rle == 1);
    check("rle_cells", memcmp(m.cells, cells, sizeof cells) == 0);
    map_free(&m);
    free(buf);
}

static void test_quantisers(void) {
    unsigned lossy = 0;
    map_upgrade_set_rounding(MAP_Q_FLOOR);
    check("floor_2_9", map_q_quantise(2.9, &lossy) == 2);
    lossy = 0;
    map_upgrade_set_rounding(MAP_Q_ROUND);
    check("round_2_9", map_q_quantise(2.9, &lossy) == 3);
    check("round_2_9_lossy", lossy == 1);
    lossy = 0;
    check("round_2_0_exact", map_q_quantise(2.0, &lossy) == 2 && lossy == 0);
    map_upgrade_set_rounding(MAP_Q_FLOOR);
}

static void test_nav(void) {
    Fixture fx;
    if (!fx_build(FX_TIER_SMALL, 3, 4, &fx)) { check("nav_build", 0); return; }
    MapFile m;
    memset(&m, 0, sizeof m);
    m.w = fx.w; m.h = fx.h; m.cells = fx.cells;

    NavPath a, b;
    check("nav_init", nav_path_init(&a, fx.w * fx.h) && nav_path_init(&b, fx.w * fx.h));
    int ok = nav_astar(&m, fx.q[0], fx.q[1], fx.q[2], fx.q[3], &a);
    check("nav_path_found", ok && a.n > 0);
    check("nav_path_ends", a.n > 0 && a.x[0] == fx.q[0] && a.y[0] == fx.q[1] &&
                            a.x[a.n - 1] == fx.q[2] && a.y[a.n - 1] == fx.q[3]);
    nav_astar(&m, fx.q[0], fx.q[1], fx.q[2], fx.q[3], &b);
    int same = 0, denom = 0, fd = -1;
    check("nav_identical", nav_compare(&a, &b, &same, &denom, &fd) &&
                            same == a.n && same == denom && fd == -1);

    /* one step of the goal shifted: the paths must differ, and the first
     * differing point must be reported */
    NavPath c;
    nav_path_init(&c, fx.w * fx.h);
    int gx = fx.q[2] > 0 ? fx.q[2] - 1 : fx.q[2] + 1;
    nav_astar(&m, fx.q[0], fx.q[1], gx, fx.q[3], &c);
    int diff = nav_compare(&a, &c, &same, &denom, &fd);
    check("nav_diff_detected", !diff || fd >= 0);
    check("nav_ratio_bounded", denom == 0 || (same <= denom && same >= 0));

    nav_path_free(&a);
    nav_path_free(&b);
    nav_path_free(&c);
    fx_free(&fx);
}

static void test_reexport(void) {
    Fixture fx;
    if (!fx_build(FX_TIER_SMALL, 4, 5, &fx)) { check("reexport_build", 0); return; }

    size_t len = 0;
    unsigned char* raw = NULL;
    unsigned char* out = NULL;
    size_t out_len = 0;
    unsigned lost = 0;
    MapFile m;

    if (map_gen_supported(0)) {
        /* gen 0 -> gen 2 -> read: same cells, same query, and no precision
         * loss reported, because the query was already whole cells. */
        raw = fx_emit_gen0(&fx, &len);
        check("reexport_gen0_ok",
              reexport_to_gen2(raw, len, fx.q, MAP_Q_FLOOR, &out, &out_len,
                               &lost) == MAP_OK);
        check("reexport_gen0_is_gen2", map_detect_version(out, out_len) == 2);
        check("reexport_gen0_lossless", lost == 0);
        check("reexport_gen0_read", map_read(out, out_len, NULL, &m) == MAP_OK);
        check("reexport_gen0_cells", memcmp(m.cells, fx.cells, 192) == 0);
        check("reexport_gen0_query", m.has_query && m.q[2] == fx.q[2]);
        map_free(&m);
        free(out);
        free(raw);
    }

    /* The load-bearing property: whichever road a gen-1 map takes, the query
     * in the result is the same, because both call map_q_quantise(). Skipped
     * when this build has no gen-1 reader; there is then no second road. */
    if (map_gen_supported(1)) {
        raw = fx_emit_gen1(&fx, &len);
        MapFile direct;
        check("reexport_gen1_direct_read",
              map_read(raw, len, NULL, &direct) == MAP_OK);
        out = NULL;
        check("reexport_gen1_ok",
              reexport_to_gen2(raw, len, NULL, MAP_Q_FLOOR, &out, &out_len,
                               &lost) == MAP_OK);
        check("reexport_gen1_reports_loss", lost != 0);
        check("reexport_gen1_read", map_read(out, out_len, NULL, &m) == MAP_OK);
        check("reexport_gen1_same_query",
              m.has_query && memcmp(m.q, direct.q, sizeof m.q) == 0);
        check("reexport_gen1_same_cells",
              memcmp(m.cells, direct.cells, 192) == 0);
        map_free(&m);
        map_free(&direct);
        free(out);
        free(raw);
    } else {
        /* A dropped generation must be refused by the tool, not silently
         * converted into something else. */
        raw = fx_emit_gen1(&fx, &len);
        out = NULL;
        check("reexport_dropped_gen1_refused",
              reexport_to_gen2(raw, len, NULL, MAP_Q_FLOOR, &out, &out_len,
                               &lost) == MAP_ERR_VERSION);
        check("reexport_dropped_gen1_no_output", out == NULL && out_len == 0);
        free(raw);
    }

    /* The tool's behaviour on a dropped generation mirrors the reader's:
     * refuse, and produce no output. */
    if (!map_gen_supported(0)) {
        raw = fx_emit_gen0(&fx, &len);
        out = NULL;
        check("reexport_dropped_gen0_refused",
              reexport_to_gen2(raw, len, fx.q, MAP_Q_FLOOR, &out, &out_len,
                               &lost) == MAP_ERR_VERSION);
        check("reexport_dropped_gen0_no_output", out == NULL && out_len == 0);
        free(raw);
    }

    /* Input that matches no generation at all is a format error, not a
     * version error, and must not produce output. */
    check("reexport_bad_input", reexport_to_gen2((const unsigned char*)"x", 1,
                                                 NULL, MAP_Q_FLOOR,
                                                 &out, &out_len, &lost)
                                 == MAP_ERR_FORMAT);
    check("reexport_bad_input_no_output", out == NULL && out_len == 0);
    fx_free(&fx);
}

int main(int argc, char** argv) {
    /* Built with a generation compiled out, assert that the shape is still
     * recognised and the refusal is MAP_ERR_VERSION, not MAP_ERR_FORMAT. */
    int expect_no_gen1 = 0, expect_no_gen0 = 0;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--expect-no-gen1") == 0) expect_no_gen1 = 1;
        if (strcmp(argv[i], "--expect-no-gen0") == 0) expect_no_gen0 = 1;
    }

    test_gen0();
    test_gen1();
    test_gen2_roundtrip();
    test_rejections();
    test_rle();
    test_quantisers();
    test_nav();
    test_reexport();

    if (expect_no_gen1) {
        Fixture fx;
        if (fx_build(FX_TIER_SMALL, 1, 2, &fx)) {
            size_t len = 0;
            unsigned char* raw = fx_emit_gen1(&fx, &len);
            check("dropped_gen1_shape_still_detected",
                  map_detect_version(raw, len) == 1);
            MapFile m;
            check("dropped_gen1_read_refused",
                  map_read(raw, len, NULL, &m) == MAP_ERR_VERSION);
            check("dropped_gen1_not_a_format_error",
                  map_read(raw, len, NULL, &m) != MAP_ERR_FORMAT);
            free(raw);
            fx_free(&fx);
        }
        check("dropped_gen1_supported_flag", map_gen_supported(1) == 0);
    }

    if (expect_no_gen0) {
        Fixture fx;
        if (fx_build(FX_TIER_SMALL, 0, 1, &fx)) {
            size_t len = 0;
            unsigned char* raw = fx_emit_gen0(&fx, &len);
            check("dropped_gen0_shape_still_detected",
                  map_detect_version(raw, len) == 0);
            MapFile m;
            check("dropped_gen0_read_refused",
                  map_read(raw, len, fx.q, &m) == MAP_ERR_VERSION);
            check("dropped_gen0_not_a_format_error",
                  map_read(raw, len, fx.q, &m) != MAP_ERR_FORMAT);
            free(raw);
            fx_free(&fx);
        }
        check("dropped_gen0_supported_flag", map_gen_supported(0) == 0);
    }

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
