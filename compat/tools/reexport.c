/*
 * reexport: convert old map files to the current generation.
 *
 *   reexport [--round] [--query sx,sy,gx,gy] in.map out.map
 *
 * gen-0 files carry no query, so pass --query when the level data has one;
 * without it the output has no query block and any path comparison will have
 * nothing to compare. gen-1 files carry their own, and it wins.
 *
 * Exits 0 on success, 1 on a read/write error, 2 on bad arguments.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "reexport.h"

static int read_file(const char* path, unsigned char** buf, size_t* len) {
    FILE* f = fopen(path, "rb");
    if (!f) return 0;
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return 0; }
    long n = ftell(f);
    if (n < 0) { fclose(f); return 0; }
    rewind(f);
    unsigned char* p = (unsigned char*)malloc(n > 0 ? (size_t)n : 1);
    if (!p) { fclose(f); return 0; }
    if (n > 0 && fread(p, 1, (size_t)n, f) != (size_t)n) {
        free(p);
        fclose(f);
        return 0;
    }
    fclose(f);
    *buf = p;
    *len = (size_t)n;
    return 1;
}

static int write_file(const char* path, const unsigned char* buf, size_t len) {
    FILE* f = fopen(path, "wb");
    if (!f) return 0;
    int ok = fwrite(buf, 1, len, f) == len;
    if (fclose(f) != 0) ok = 0;
    return ok;
}

static int parse_query(const char* s, uint16_t q[4]) {
    unsigned v[4];
    if (sscanf(s, "%u,%u,%u,%u", &v[0], &v[1], &v[2], &v[3]) != 4) return 0;
    for (int i = 0; i < 4; i++) {
        if (v[i] > 65535u) return 0;
        q[i] = (uint16_t)v[i];
    }
    return 1;
}

int main(int argc, char** argv) {
    MapQRounding rounding = MAP_Q_FLOOR;
    uint16_t query[4];
    const uint16_t* fallback = NULL;
    const char* pos[2] = {NULL, NULL};
    int npos = 0;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--round") == 0) {
            rounding = MAP_Q_ROUND;
        } else if (strcmp(argv[i], "--floor") == 0) {
            rounding = MAP_Q_FLOOR;
        } else if (strcmp(argv[i], "--query") == 0 && i + 1 < argc) {
            if (!parse_query(argv[++i], query)) {
                fprintf(stderr, "reexport: bad --query, want sx,sy,gx,gy\n");
                return 2;
            }
            fallback = query;
        } else if (argv[i][0] == '-') {
            fprintf(stderr, "reexport: unknown option %s\n", argv[i]);
            return 2;
        } else if (npos < 2) {
            pos[npos++] = argv[i];
        } else {
            fprintf(stderr, "reexport: unexpected argument %s\n", argv[i]);
            return 2;
        }
    }
    if (npos != 2) {
        fprintf(stderr, "usage: reexport [--round|--floor] "
                        "[--query sx,sy,gx,gy] in.map out.map\n");
        return 2;
    }

    unsigned char* in = NULL;
    size_t in_len = 0;
    if (!read_file(pos[0], &in, &in_len)) {
        fprintf(stderr, "reexport: cannot read %s\n", pos[0]);
        return 1;
    }

    int gen = map_detect_version(in, in_len);
    unsigned char* out = NULL;
    size_t out_len = 0;
    unsigned lost = 0;
    MapStatus st = reexport_to_gen2(in, in_len, fallback, rounding,
                                    &out, &out_len, &lost);
    if (st != MAP_OK) {
        fprintf(stderr, "reexport: %s: %s\n", pos[0], map_status_str(st));
        free(in);
        return 1;
    }
    if (!write_file(pos[1], out, out_len)) {
        fprintf(stderr, "reexport: cannot write %s\n", pos[1]);
        free(in);
        free(out);
        return 1;
    }

    printf("in=%s gen=%d bytes=%zu out=%s gen=%d bytes=%zu rounding=%s\n",
           pos[0], gen, in_len, pos[1], MAP_GEN_CURRENT, out_len,
           rounding == MAP_Q_ROUND ? "round" : "floor");
    if (lost) {
        const char* names[4] = {"sx", "sy", "gx", "gy"};
        printf("lossy_fields=");
        int first = 1;
        for (int i = 0; i < 4; i++) {
            if (lost & (1u << i)) {
                printf("%s%s", first ? "" : ",", names[i]);
                first = 0;
            }
        }
        printf("\n");
    }

    free(in);
    free(out);
    return 0;
}
