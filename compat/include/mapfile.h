#ifndef MAPFILE_H
#define MAPFILE_H

#include <stddef.h>
#include <stdint.h>

/*
 * Map file generations. All multi-byte fields are little-endian, in every
 * generation, including the ones that predate the version field.
 *
 * ---- gen 2, current: self-describing -----------------------------------
 *   off  size  field
 *   0    4     magic "MNL\0"
 *   4    2     version (= 2)
 *   6    2     width
 *   8    2     height
 *   10   1     flags    bit0 payload is run-length encoded
 *                       bit1 a query block follows the payload
 *   11   1     reserved, must be 0
 *   12   ...   payload, then the query block if bit1
 *
 *   payload, bit0 clear: height rows of `width` bytes, 0 free / 1 blocked.
 *   payload, bit0 set  : per row, u16 runcount, then runcount (u8 value,
 *                        u8 length) pairs; lengths sum to width.
 *   query block        : u16 sx, sy, gx, gy in whole cells.
 *
 * ---- gen 1, legacy: no magic, no version -------------------------------
 *   u16 w, u16 h, w*h cell bytes, then sx, sy, gx, gy as f32 in cell units
 *   (1.0 == one cell). Exact length 4 + w*h + 16. The sub-cell part of those
 *   f32 coordinates has nowhere to go in gen 2's whole-cell query block, so
 *   upgrading snaps them; see MAP_Q_FLOOR / MAP_Q_ROUND below.
 *
 * ---- gen 0, legacy: no magic, no version, no dimensions ----------------
 *   The bare `unsigned char blocked[16*12]` array that grid/, smooth/ and
 *   region/ use today. Exactly 192 bytes, fixed 16x12, cells only, no query.
 *   This is the generation that actually exists on disk right now. The query
 *   is supplied by the caller (the level data it came from), so nothing is
 *   lost on upgrade: there is nothing in the file but cells.
 *
 * gen 1 has no files in this repository's history. It is kept because the
 * retention decision is a function of how many generations still have files,
 * and the machinery has to be exercised for N = 2 as well as N = 1. The bench
 * synthesises gen-1 files for that case; see compat/README.md.
 *
 * Detection is header-first, parse-second: if the header parses as the
 * current structure, read it directly; otherwise try the legacy structures
 * and upgrade. See map_detect_version().
 */

#define MAP_MAGIC0 'M'
#define MAP_MAGIC1 'N'
#define MAP_MAGIC2 'L'

#define MAP_GEN_CURRENT 2
#define MAP_GEN_COUNT   3

/* Compile-time retention switches. Turning one off drops its reader from the
 * binary and makes map_read() answer MAP_ERR_VERSION for that generation.
 * These are the knobs the retention decision in compat/README.md sets; the
 * per-generation line counts in that file are measured from the sources. */
#ifndef MAP_KEEP_V0
#define MAP_KEEP_V0 1
#endif
#ifndef MAP_KEEP_V1
#define MAP_KEEP_V1 1
#endif

#define MAP_FLAG_RLE   0x01
#define MAP_FLAG_QUERY 0x02
#define MAP_FLAG_KNOWN (MAP_FLAG_RLE | MAP_FLAG_QUERY)

#define MAP_MAX_W 1024
#define MAP_MAX_H 1024
#define MAP_MAX_CELLS ((size_t)MAP_MAX_W * (size_t)MAP_MAX_H)

/* gen 0 has no dimensions in the file; they are part of the generation. */
#define MAP_V0_W 16
#define MAP_V0_H 12
#define MAP_V0_BYTES ((size_t)MAP_V0_W * (size_t)MAP_V0_H)

/* detect() results. Generations are returned as themselves (0, 1, 2). */
#define MAP_DET_UNKNOWN (-1)
#define MAP_DET_FUTURE  (-2)   /* magic present, version this build can't read */

typedef struct {
    int version;          /* generation the bytes were read as */
    int upgraded;         /* nonzero if version < MAP_GEN_CURRENT */
    int from_rle;         /* nonzero if the gen-2 payload was RLE */
    int has_query;        /* nonzero if a query is present */
    uint16_t w, h;
    uint16_t q[4];        /* sx, sy, gx, gy in whole cells */
    double   q_exact[4];  /* value before the whole-cell snap */
    unsigned q_lossy;     /* bit i set: q_exact[i] != (double)q[i] */
    unsigned char* cells; /* w*h, row-major, 0 free / 1 blocked */
} MapFile;

typedef enum {
    MAP_OK = 0,
    MAP_ERR_IO = -1,
    MAP_ERR_TRUNCATED = -2,
    MAP_ERR_MAGIC = -3,
    MAP_ERR_VERSION = -4,
    MAP_ERR_DIM = -5,
    MAP_ERR_PAYLOAD = -6,
    MAP_ERR_NO_MEM = -7,
    MAP_ERR_FORMAT = -8
} MapStatus;

const char* map_status_str(MapStatus s);
const char* map_gen_name(int gen);
int         map_gen_supported(int gen);

/*
 * How a sub-cell query coordinate becomes a whole cell on upgrade. This snap
 * is the only lossy step in the whole pipeline, and the compatibility reader
 * and the re-export tool both call map_q_quantise(), so the two roads into
 * gen 2 cannot disagree about where a point lands.
 *
 * MAP_Q_FLOOR reproduces the legacy runtime, whose cell index was
 * (int)coordinate; it is the default. MAP_Q_ROUND is the tempting alternative
 * and exists so the bench can measure what it costs.
 */
typedef enum { MAP_Q_FLOOR = 0, MAP_Q_ROUND = 1 } MapQRounding;

void         map_upgrade_set_rounding(MapQRounding mode);
MapQRounding map_upgrade_get_rounding(void);

/* `exact` is a cell coordinate. Sets *lossy when the snap is not exact. */
uint16_t map_q_quantise(double exact, unsigned* lossy);

/*
 * The compatibility predicate, written once:
 *
 *   detect(b,n) = 2                  if v2_header_ok(b,n)
 *               = 1                  if n == 4 + w*h + 16, w,h = le16(b),
 *                                    cells all 0/1, every |q| < 65536
 *               = 0                  if n == MAP_V0_BYTES and cells all 0/1
 *               = MAP_DET_FUTURE     if b[0..3] == "MNL\0", version != 2
 *               = MAP_DET_UNKNOWN    otherwise
 *
 *   read(b,n,q) = parse_v2(b,n)                when detect == 2
 *               = upgrade_1(parse_1(b,n))      when detect == 1, kept
 *               = upgrade_0(parse_0(b,n), q)   when detect == 0, kept
 *               = MAP_ERR_VERSION              when that gen was dropped
 *               = MAP_ERR_FORMAT               otherwise
 *
 * v2_header_ok: magic, version == 2, reserved byte 0, no unknown flag bits,
 * 1 <= w,h <= MAP_MAX_W/H. Payload validity is v2_parse's job. A file with
 * valid magic but a bad payload is an error and is never re-tried as a
 * legacy file: magic wins.
 *
 * The raw 192-byte array cannot be confused with gen 1: its first four bytes
 * are cell values, so w,h in {0,1,256,257}, and 4 + w*h + 16 == 192 would
 * need w*h == 172, which no such pair gives.
 */
int map_detect_version(const unsigned char* buf, size_t buf_len);

/*
 * Parse into `out`. On success out->cells is malloc'd; free with map_free().
 * Never reads past buf_len. `fallback_q` is used only when the generation
 * carries no query of its own (gen 0); pass NULL otherwise. It is taken as
 * whole cells, so it introduces no loss.
 */
MapStatus map_read(const unsigned char* buf, size_t buf_len,
                   const uint16_t* fallback_q, MapFile* out);
void      map_free(MapFile* m);

/*
 * Serialise cells (w*h, row-major) as gen 2. `query` may be NULL. RLE is used
 * only when it actually makes the file smaller, so writing is lossless.
 */
MapStatus map_write(const unsigned char* cells, uint16_t w, uint16_t h,
                    const uint16_t query[4], unsigned char** out_buf,
                    size_t* out_len);

/* Dimensions equal and every cell equal. */
int map_same_cells(const MapFile* a, const MapFile* b);

#endif
