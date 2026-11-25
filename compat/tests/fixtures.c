#include <stdlib.h>
#include <string.h>

#include "fixtures.h"

/*
 * All fixtures are gen-0 shaped: 16x12, because that is the generation whose
 * files exist. A "tier" is a file count, not a size, since the generation
 * fixes the size. The maze is built with a simple LCG (not rand(), which is
 * implementation-defined and would make a failure unreproducible) and is
 * then flood-filled so the start and goal are guaranteed connected: the bench
 * needs reachable queries to compare paths, and an unreachable pair would
 * compare as "both none" without exercising anything.
 */

static unsigned lcg(unsigned* s) {
    *s = *s * 1664525u + 1013904223u;
    return *s;
}

static unsigned rng_range(unsigned* s, unsigned n) {
    return n ? lcg(s) % n : 0;
}

int fx_tier_files(int tier) {
    switch (tier) {
    case FX_TIER_SMALL: return 8;
    case FX_TIER_MID:   return 80;
    case FX_TIER_LARGE: return 800;
    }
    return 0;
}

const char* fx_tier_name(int tier) {
    switch (tier) {
    case FX_TIER_SMALL: return "small";
    case FX_TIER_MID:   return "mid";
    case FX_TIER_LARGE: return "large";
    }
    return "?";
}

/* Flood fill from (sx,sy) over free cells; returns 1 if (gx,gy) is reached. */
static int connected(const unsigned char* cells, int w, int h,
                     int sx, int sy, int gx, int gy) {
    unsigned char* seen = (unsigned char*)calloc((size_t)w * h, 1);
    int* stack = (int*)malloc(sizeof(int) * (size_t)w * h);
    if (!seen || !stack) { free(seen); free(stack); return 0; }
    int sp = 0, ok = 0;
    stack[sp++] = sy * w + sx;
    seen[sy * w + sx] = 1;
    while (sp > 0) {
        int c = stack[--sp];
        if (c == gy * w + gx) { ok = 1; break; }
        int x = c % w, y = c / w;
        const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
        for (int d = 0; d < 4; d++) {
            int nx = x + dx[d], ny = y + dy[d];
            if (nx < 0 || ny < 0 || nx >= w || ny >= h) continue;
            int ni = ny * w + nx;
            if (seen[ni] || cells[ni]) continue;
            seen[ni] = 1;
            stack[sp++] = ni;
        }
    }
    free(seen);
    free(stack);
    return ok;
}

int fx_build(int tier, int index, unsigned seed, Fixture* fx) {
    (void)tier;
    int w = MAP_V0_W, h = MAP_V0_H;
    unsigned s = seed * 2654435761u + (unsigned)index * 40503u + 17u;

    memset(fx, 0, sizeof *fx);
    fx->w = (uint16_t)w;
    fx->h = (uint16_t)h;
    fx->cells = (unsigned char*)calloc((size_t)w * h, 1);
    if (!fx->cells) return 0;

    /* About one cell in six blocked, but never the border, so a path that
     * exists can be compared without the boundary dominating the route. */
    for (int y = 1; y < h - 1; y++)
        for (int x = 1; x < w - 1; x++)
            if (rng_range(&s, 6) == 0) fx->cells[y * w + x] = 1;

    /* Pick a start on the left third and a goal on the right third, clearing
     * the endpoints themselves, then retry until they connect. The retry is
     * bounded; the cell density above makes a connection overwhelmingly
     * likely, but a bounded loop keeps the generator honest. */
    int sx = 0, sy = 0, gx = 0, gy = 0;
    int tries = 0;
    do {
        sx = (int)rng_range(&s, (unsigned)w / 3);
        sy = (int)rng_range(&s, (unsigned)h);
        gx = w - 1 - (int)rng_range(&s, (unsigned)w / 3);
        gy = (int)rng_range(&s, (unsigned)h);
        fx->cells[sy * w + sx] = 0;
        fx->cells[gy * w + gx] = 0;
        tries++;
    } while (!connected(fx->cells, w, h, sx, sy, gx, gy) && tries < 200);

    if (!connected(fx->cells, w, h, sx, sy, gx, gy)) { fx_free(fx); return 0; }

    fx->q[0] = (uint16_t)sx;
    fx->q[1] = (uint16_t)sy;
    fx->q[2] = (uint16_t)gx;
    fx->q[3] = (uint16_t)gy;

    /* The gen-1 query carries a fraction inside the cell. Both floor and round
     * can land on the same whole cell; the point is that the snap is applied,
     * and the bench reports any map where it changed the point. */
    for (int i = 0; i < 4; i++) {
        float frac = (float)(rng_range(&s, 1000)) / 1000.0f;
        if (frac > 0.999f) frac = 0.999f;
        fx->q_sub[i] = (float)fx->q[i] + frac;
    }
    return 1;
}

void fx_free(Fixture* fx) {
    free(fx->cells);
    fx->cells = NULL;
}

/* gen 0: exactly the 192-byte array the modules hold in memory. */
unsigned char* fx_emit_gen0(const Fixture* fx, size_t* len) {
    if (fx->w != MAP_V0_W || fx->h != MAP_V0_H) return NULL;
    unsigned char* buf = (unsigned char*)malloc(MAP_V0_BYTES);
    if (!buf) return NULL;
    memcpy(buf, fx->cells, MAP_V0_BYTES);
    *len = MAP_V0_BYTES;
    return buf;
}

/* gen 1: u16 w, u16 h, cells, f32 sx, sy, gx, gy. */
unsigned char* fx_emit_gen1(const Fixture* fx, size_t* len) {
    size_t n = (size_t)fx->w * fx->h;
    size_t total = 4 + n + 16;
    unsigned char* buf = (unsigned char*)malloc(total);
    if (!buf) return NULL;
    buf[0] = (unsigned char)(fx->w & 0xff);
    buf[1] = (unsigned char)(fx->w >> 8);
    buf[2] = (unsigned char)(fx->h & 0xff);
    buf[3] = (unsigned char)(fx->h >> 8);
    memcpy(buf + 4, fx->cells, n);
    for (int i = 0; i < 4; i++) {
        uint32_t bits;
        float f = fx->q_sub[i];
        memcpy(&bits, &f, sizeof bits);
        buf[4 + n + 4 * i + 0] = (unsigned char)(bits & 0xff);
        buf[4 + n + 4 * i + 1] = (unsigned char)((bits >> 8) & 0xff);
        buf[4 + n + 4 * i + 2] = (unsigned char)((bits >> 16) & 0xff);
        buf[4 + n + 4 * i + 3] = (unsigned char)((bits >> 24) & 0xff);
    }
    *len = total;
    return buf;
}

unsigned char* fx_emit_gen2(const Fixture* fx, const uint16_t query[4],
                            size_t* len) {
    unsigned char* buf = NULL;
    size_t n = 0;
    if (map_write(fx->cells, fx->w, fx->h, query, &buf, &n) != MAP_OK)
        return NULL;
    *len = n;
    return buf;
}
