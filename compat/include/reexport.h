#ifndef REEXPORT_H
#define REEXPORT_H

#include <stddef.h>
#include <stdint.h>

#include "mapfile.h"

/*
 * One-shot re-export: read a file of any supported generation and write it
 * back out as gen 2. This is the tool a level archive runs once per old map;
 * the compatibility reader is what runs forever afterwards. The two share
 * map_q_quantise(), so a map that took either road lands on the same query.
 *
 * `rounding` is explicit rather than inherited from the global because the
 * tool is run offline by hand and its choice must not depend on whatever the
 * process happened to set. `fallback_q` is the query for gen-0 files, which
 * carry none; pass NULL to write a queryless gen-2 file.
 *
 * *lost_fields receives the MapFile q_lossy mask: bit 0 = sx, 1 = sy,
 * 2 = gx, 3 = gy, set when the whole-cell snap changed that field.
 */
MapStatus reexport_to_gen2(const unsigned char* in, size_t in_len,
                           const uint16_t* fallback_q, MapQRounding rounding,
                           unsigned char** out, size_t* out_len,
                           unsigned* lost_fields);

#endif
