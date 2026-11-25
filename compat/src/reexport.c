#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "reexport.h"

MapStatus reexport_to_gen2(const unsigned char* in, size_t in_len,
                           const uint16_t* fallback_q, MapQRounding rounding,
                           unsigned char** out, size_t* out_len,
                           unsigned* lost_fields) {
    *out = NULL;
    *out_len = 0;
    if (lost_fields) *lost_fields = 0;

    /* The quantiser is process-global, so the tool pins it for the duration of
     * this call and restores it. Offline tool, single call, no threads. */
    MapQRounding prev = map_upgrade_get_rounding();
    map_upgrade_set_rounding(rounding);

    MapFile m;
    MapStatus st = map_read(in, in_len, fallback_q, &m);
    if (st == MAP_OK) {
        st = map_write(m.cells, m.w, m.h, m.has_query ? m.q : NULL,
                       out, out_len);
        if (st == MAP_OK && lost_fields) *lost_fields = m.q_lossy;
    }
    map_free(&m);

    map_upgrade_set_rounding(prev);
    return st;
}
