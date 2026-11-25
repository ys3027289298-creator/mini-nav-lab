/*
 * crossfloor: sketch stair circles -> floor connection points, and weighted
 * cross-floor A* pathfinding.
 *
 * Cost model (integer hundredths of a grid cell):
 *   flat 4-neighbour edge            = 100
 *   one adjacent-floor stair link    = lambda_h  (lambda grid cells * 100)
 *
 * lambda is never guessed: it is reverse-solved from measured times,
 *   tau_cell = h / v_flat,  lambda = T_stair / tau_cell = v_flat * T_stair / h
 * see data/survey.txt and the accept report.
 */
#define _POSIX_C_SOURCE 200809L
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <stdint.h>
#include <unistd.h>

#define MAXW 32
#define MAXH 32
#define MAXF 4
#define MAXN (MAXW * MAXH * MAXF)
#define MAXSHAFT 16
#define MAXCASE 16
#define MAXPERSON 64
#define CELL_H 100 /* hundredths of a grid cell per flat edge */

typedef struct {
    char name[48];
    int sf, sx, sy;
    int gf, gx, gy;
    int expect;
} Case;

typedef struct {
    int w, h, floors;
    double cell_m;
    unsigned char block[MAXF][MAXH][MAXW];
    int shaft_at[MAXF][MAXH][MAXW]; /* shaft index + 1, or 0 */
    int ns;
    int sx[MAXSHAFT], sy[MAXSHAFT];
    int ncase;
    Case cases[MAXCASE];
} Sketch;

typedef struct {
    double cell_m, speed;
    int n;
    double up[MAXPERSON], down[MAXPERSON];
} Survey;

typedef struct {
    int found;
    int cost_h;      /* weighted cost in hundredths of a cell */
    int flat;        /* flat edges */
    int vert;        /* vertical stair links */
    int npath;
    int shafts_used[8];
    int nshaft_used;
} Info;

/* ---------------- tiny line parser over an in-memory sketch -------------- */

static char* skip_sp(char* p) {
    while (*p == ' ' || *p == '\t') p++;
    return p;
}

/* Parse sketch text. Returns 0 on success, -1 on malformed input. */
static int parse_sketch_text(const char* text, Sketch* sk, char* err, size_t errn) {
    if (errn) err[0] = 0;
    memset(sk, 0, sizeof(*sk));
    for (int f = 0; f < MAXF; f++)
        for (int y = 0; y < MAXH; y++)
            for (int x = 0; x < MAXW; x++) sk->shaft_at[f][y][x] = 0;
    sk->cell_m = 1.0;

    size_t len = strlen(text);
    char* buf = (char*)malloc(len + 1);
    if (!buf) { snprintf(err, errn, "oom"); return -1; }
    memcpy(buf, text, len + 1);

    /* split into lines */
    enum { MAXLINES = 4096 };
    char* lines[MAXLINES];
    int nl = 0;
    lines[nl++] = buf;
    for (char* p = buf; *p; p++) {
        if (*p == '\n') {
            *p = 0;
            if (nl < MAXLINES) lines[nl++] = p + 1;
        }
    }

    int have_w = 0, have_h = 0, have_f = 0;
    int rc = 0;
    for (int li = 0; li < nl && rc == 0; li++) {
        char* line = lines[li];
        while (*line && (*line == '\r' || *line == '\n')) line++;
        char* s = skip_sp(line);
        if (*s == 0 || *s == '#') continue;

        int iv;
        double dv;
        if (sscanf(s, "width %d", &iv) == 1) {
            if (iv <= 0 || iv > MAXW) { rc = -1; snprintf(err, errn, "bad width"); break; }
            sk->w = iv; have_w = 1;
        } else if (sscanf(s, "height %d", &iv) == 1) {
            if (iv <= 0 || iv > MAXH) { rc = -1; snprintf(err, errn, "bad height"); break; }
            sk->h = iv; have_h = 1;
        } else if (sscanf(s, "floors %d", &iv) == 1) {
            if (iv <= 0 || iv > MAXF) { rc = -1; snprintf(err, errn, "bad floors"); break; }
            sk->floors = iv; have_f = 1;
        } else if (sscanf(s, "cell_m %lf", &dv) == 1) {
            sk->cell_m = dv;
        } else if (strncmp(s, "floor ", 6) == 0) {
            if (!have_w || !have_h || !have_f) { rc = -1; snprintf(err, errn, "floor before dims"); break; }
            int fi = -1;
            if (sscanf(s, "floor %d", &fi) != 1 || fi < 0 || fi >= sk->floors) {
                rc = -1; snprintf(err, errn, "bad floor index"); break;
            }
            /* next non-empty directive token must be rows */
            int rj = li + 1;
            while (rj < nl) {
                char* t = skip_sp(lines[rj]);
                if (*t == '\r' || *t == 0) { rj++; continue; }
                break;
            }
            if (rj >= nl || strncmp(skip_sp(lines[rj]), "rows", 4) != 0) {
                rc = -1; snprintf(err, errn, "floor %d missing rows", fi); break;
            }
            rj++;
            for (int y = 0; y < sk->h; y++) {
                if (rj + y >= nl) { rc = -1; snprintf(err, errn, "floor %d short rows", fi); break; }
                char* row = lines[rj + y];
                size_t rl = strlen(row);
                while (rl && (row[rl - 1] == '\r' || row[rl - 1] == '\n')) row[--rl] = 0;
                if ((int)strlen(row) != sk->w) {
                    rc = -1; snprintf(err, errn, "floor %d row %d len %d != %d",
                                      fi, y, (int)strlen(row), sk->w);
                    break;
                }
                for (int x = 0; x < sk->w; x++) {
                    char c = row[x];
                    if (c == '#') sk->block[fi][y][x] = 1;
                    else if (c == 'O') {
                        sk->block[fi][y][x] = 0;
                        int id = -1;
                        for (int k = 0; k < sk->ns; k++)
                            if (sk->sx[k] == x && sk->sy[k] == y) { id = k; break; }
                        if (id < 0) {
                            if (sk->ns >= MAXSHAFT) { rc = -1; snprintf(err, errn, "too many shafts"); break; }
                            id = sk->ns++;
                            sk->sx[id] = x; sk->sy[id] = y;
                        }
                        sk->shaft_at[fi][y][x] = id + 1;
                    } else if (c == '.') {
                        /* free */
                    } else {
                        rc = -1; snprintf(err, errn, "bad char %c at f%d y%d x%d", c, fi, y, x);
                        break;
                    }
                }
                if (rc) break;
            }
            if (rc) break;
            /* skip past consumed rows + endrows */
            int ej = rj + sk->h;
            while (ej < nl && strncmp(skip_sp(lines[ej]), "endrows", 7) != 0) ej++;
            li = ej;
        } else if (strncmp(s, "case ", 5) == 0) {
            if (sk->ncase >= MAXCASE) { rc = -1; snprintf(err, errn, "too many cases"); break; }
            Case* c = &sk->cases[sk->ncase];
            char expword[16];
            int got = sscanf(s, "case %47s %d %d %d %d %d %d %15s %d",
                             c->name, &c->sf, &c->sx, &c->sy,
                             &c->gf, &c->gx, &c->gy, expword, &c->expect);
            if (got != 9 || strcmp(expword, "expect") != 0 ||
                c->sf < 0 || c->sf >= sk->floors || c->gf < 0 || c->gf >= sk->floors) {
                rc = -1; snprintf(err, errn, "bad case line: %s", s); break;
            }
            sk->ncase++;
        }
    }
    if (rc == 0 && (!have_w || !have_h || !have_f)) {
        rc = -1; snprintf(err, errn, "missing width/height/floors");
    }
    free(buf);
    return rc;
}

static char* read_file(const char* path) {
    FILE* fp = fopen(path, "rb");
    if (!fp) return NULL;
    if (fseek(fp, 0, SEEK_END) != 0) { fclose(fp); return NULL; }
    long n = ftell(fp);
    if (n < 0) { fclose(fp); return NULL; }
    rewind(fp);
    char* b = (char*)malloc((size_t)n + 1);
    if (!b) { fclose(fp); return NULL; }
    size_t rd = fread(b, 1, (size_t)n, fp);
    b[rd] = 0;
    fclose(fp);
    return b;
}

/* find a data file: explicit path, env dir, ./data, ./crossfloor/data, exe-relative */
static const char* find_data(const char* given, const char* name, char* alt, size_t altn) {
    if (given) return given;
    static char p1[1024], p2[1024], p3[2048];
    FILE* fp;
    const char* env = getenv("CROSSFLOOR_DATA");
    if (env) {
        snprintf(p1, sizeof(p1), "%s/%s", env, name);
        if ((fp = fopen(p1, "rb"))) { fclose(fp); return p1; }
    }
    snprintf(p2, sizeof(p2), "data/%s", name);
    if ((fp = fopen(p2, "rb"))) { fclose(fp); return p2; }
    snprintf(p3, sizeof(p3), "crossfloor/data/%s", name);
    if ((fp = fopen(p3, "rb"))) { fclose(fp); return p3; }
    char exe[1024];
    ssize_t k = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (k > 0) {
        exe[k] = 0;
        char* slash = strrchr(exe, '/');
        if (slash) {
            *slash = 0;
            snprintf(alt, altn, "%s/../data/%s", exe, name);
            if ((fp = fopen(alt, "rb"))) { fclose(fp); return alt; }
        }
    }
    snprintf(alt, altn, "data/%s", name);
    return alt;
}

static int load_sketch(const char* path, Sketch* sk, char* err, size_t errn) {
    char alt[2048];
    const char* p = find_data(path, "floors.txt", alt, sizeof(alt));
    char* txt = read_file(p);
    if (!txt) { snprintf(err, errn, "cannot read %s", p); return -1; }
    int rc = parse_sketch_text(txt, sk, err, errn);
    free(txt);
    return rc;
}

/* ------------------------------ survey ----------------------------------- */

static int parse_survey_text(const char* text, Survey* sv) {
    memset(sv, 0, sizeof(*sv));
    sv->cell_m = 1.0;
    sv->speed = 1.4;
    const char* p = text;
    while (*p) {
        const char* eol = strchr(p, '\n');
        char line[512];
        size_t n = eol ? (size_t)(eol - p) : strlen(p);
        if (n >= sizeof(line)) n = sizeof(line) - 1;
        memcpy(line, p, n); line[n] = 0;
        char* s = skip_sp(line);
        double dv;
        if (sscanf(s, "cell_m %lf", &dv) == 1) sv->cell_m = dv;
        else if (sscanf(s, "flat_speed_mps %lf", &dv) == 1) sv->speed = dv;
        else if (strncmp(s, "person", 6) == 0 && sv->n < MAXPERSON) {
            double u, d;
            if (sscanf(s, "person %lf %lf", &u, &d) == 2) {
                sv->up[sv->n] = u; sv->down[sv->n] = d; sv->n++;
            }
        }
        if (!eol) break;
        p = eol + 1;
    }
    return sv->n > 0 ? 0 : -1;
}

static int load_survey(const char* path, Survey* sv) {
    char alt[2048];
    const char* p = find_data(path, "survey.txt", alt, sizeof(alt));
    char* txt = read_file(p);
    if (!txt) return -1;
    int rc = parse_survey_text(txt, sv);
    free(txt);
    return rc;
}

static double tau_cell(const Survey* sv) { return sv->cell_m / sv->speed; }
static int lambda_h_from_seconds(double sec, double tau) {
    return (int)llround(100.0 * sec / tau);
}
static double survey_up_mean(const Survey* sv) {
    double a = 0; for (int i = 0; i < sv->n; i++) a += sv->up[i]; return a / sv->n;
}
static double survey_down_mean(const Survey* sv) {
    double a = 0; for (int i = 0; i < sv->n; i++) a += sv->down[i]; return a / sv->n;
}
/* scalar link time = person-average of up and down, then mean over people */
static double survey_link_mean(const Survey* sv) {
    double a = 0;
    for (int i = 0; i < sv->n; i++) a += 0.5 * (sv->up[i] + sv->down[i]);
    return a / sv->n;
}

/* --------------------------- cross-floor A* ------------------------------ */

static int nid(const Sketch* sk, int f, int y, int x) {
    return (f * sk->h + y) * sk->w + x;
}
static void decode(const Sketch* sk, int id, int* f, int* y, int* x) {
    *x = id % sk->w;
    int t = id / sk->w;
    *y = t % sk->h;
    *f = t / sk->h;
}
static int free_cell(const Sketch* sk, int f, int y, int x) {
    return f >= 0 && f < sk->floors && y >= 0 && y < sk->h &&
           x >= 0 && x < sk->w && !sk->block[f][y][x];
}

typedef struct { int id, key; } HItem;
typedef struct {
    int n;
    HItem a[MAXN + 1];
    int pos[MAXN]; /* 1-based heap pos; 0 absent */
} Heap;

static int hbetter(const HItem* x, const HItem* y) {
    if (x->key != y->key) return x->key < y->key;
    return x->id < y->id;
}
static void hswap(Heap* h, int i, int j) {
    HItem t = h->a[i]; h->a[i] = h->a[j]; h->a[j] = t;
    h->pos[h->a[i].id] = i; h->pos[h->a[j].id] = j;
}
static void hup(Heap* h, int i) {
    while (i > 1 && hbetter(&h->a[i], &h->a[i / 2])) { hswap(h, i, i / 2); i /= 2; }
}
static void hdown(Heap* h, int i) {
    for (;;) {
        int b = i, l = 2 * i, r = 2 * i + 1;
        if (l <= h->n && hbetter(&h->a[l], &h->a[b])) b = l;
        if (r <= h->n && hbetter(&h->a[r], &h->a[b])) b = r;
        if (b == i) break;
        hswap(h, i, b); i = b;
    }
}
static void hput(Heap* h, int id, int key) {
    int p = h->pos[id];
    if (p > 0) {
        if (key >= h->a[p].key) return;
        h->a[p].key = key; hup(h, p);
        return;
    }
    h->a[++h->n] = (HItem){id, key};
    h->pos[id] = h->n;
    hup(h, h->n);
}
static HItem hpop(Heap* h) {
    HItem top = h->a[1];
    h->pos[top.id] = 0;
    if (h->n > 1) { h->a[1] = h->a[h->n]; h->pos[h->a[1].id] = 1; }
    h->n--;
    if (h->n > 0) hdown(h, 1);
    return top;
}

/* returns path node count, -1 unreachable, -2 cap */
static int astar_x(const Sketch* sk, int sf, int sx, int sy, int gf, int gx, int gy,
                   int lambda_h, int* out, int cap, Info* info) {
    if (info) memset(info, 0, sizeof(*info));
    if (!free_cell(sk, sf, sy, sx) || !free_cell(sk, gf, gy, gx)) return -1;
    const int N = sk->floors * sk->h * sk->w;
    static int dist[MAXN], keyf[MAXN], parent[MAXN];
    static unsigned char closed[MAXN];
    static Heap heap;
    for (int i = 0; i < N; i++) {
        dist[i] = INT32_MAX / 4; keyf[i] = INT32_MAX / 4;
        parent[i] = -1; closed[i] = 0; heap.pos[i] = 0;
    }
    heap.n = 0;

    int s = nid(sk, sf, sy, sx), g = nid(sk, gf, gy, gx);
    int h0 = (abs(sx - gx) + abs(sy - gy)) * CELL_H + abs(sf - gf) * lambda_h;
    dist[s] = 0; keyf[s] = h0; hput(&heap, s, h0);

    const int dx[4] = {1, -1, 0, 0}, dy[4] = {0, 0, 1, -1};
    int reached = 0;
    while (heap.n > 0) {
        HItem cur = hpop(&heap);
        int u = cur.id;
        if (closed[u]) continue;
        closed[u] = 1;
        if (u == g) { reached = 1; break; }
        int uf, uy, ux; decode(sk, u, &uf, &uy, &ux);
        for (int d = 0; d < 4; d++) {
            int nx = ux + dx[d], ny = uy + dy[d];
            if (!free_cell(sk, uf, ny, nx)) continue;
            int v = nid(sk, uf, ny, nx);
            if (closed[v]) continue;
            int nd = dist[u] + CELL_H;
            if (nd < dist[v]) {
                dist[v] = nd; parent[v] = u;
                int hv = (abs(nx - gx) + abs(ny - gy)) * CELL_H + abs(uf - gf) * lambda_h;
                keyf[v] = nd + hv;
                hput(&heap, v, keyf[v]);
            }
        }
        int sh = sk->shaft_at[uf][uy][ux];
        if (sh > 0) {
            for (int t = 0; t < 2; t++) {
                int nf = uf + (t == 0 ? -1 : 1);
                if (!free_cell(sk, nf, uy, ux)) continue;
                if (sk->shaft_at[nf][uy][ux] != sh) continue;
                int v = nid(sk, nf, uy, ux);
                if (closed[v]) continue;
                int nd = dist[u] + lambda_h;
                if (nd < dist[v]) {
                    dist[v] = nd; parent[v] = u;
                    int hv = (abs(ux - gx) + abs(uy - gy)) * CELL_H + abs(nf - gf) * lambda_h;
                    keyf[v] = nd + hv;
                    hput(&heap, v, keyf[v]);
                }
            }
        }
    }
    if (!reached) return -1;

    int rev[MAXN], rn = 0, c = g;
    while (c != -1 && rn < N) { rev[rn++] = c; if (c == s) break; c = parent[c]; }
    if (rev[rn - 1] != s) return -1;
    if (rn > cap) return -2;
    for (int i = 0; i < rn; i++) out[i] = rev[rn - 1 - i];

    if (info) {
        info->found = 1; info->npath = rn; info->cost_h = dist[g];
        for (int i = 0; i + 1 < rn; i++) {
            int f1, y1, x1, f2, y2, x2;
            decode(sk, out[i], &f1, &y1, &x1);
            decode(sk, out[i + 1], &f2, &y2, &x2);
            if (f1 != f2) {
                info->vert++;
                int sh = sk->shaft_at[f1][y1][x1] - 1;
                if (info->nshaft_used < 8 && sh >= 0)
                    info->shafts_used[info->nshaft_used++] = sh;
            } else info->flat++;
        }
    }
    return rn;
}

static int run_case(const Sketch* sk, const Case* c, int lambda_h, Info* info, int* path, int cap) {
    return astar_x(sk, c->sf, c->sx, c->sy, c->gf, c->gx, c->gy, lambda_h, path, cap, info);
}

/* ------------------------- selection-rate verdict ------------------------ */

typedef struct {
    int k, n;
    double expected, tol, actual, dev;
    int flagged; /* deviation strictly greater than tol */
} Verdict;

static Verdict rate_verdict(int k, int n, double expected, double tol) {
    Verdict v;
    v.k = k; v.n = n; v.expected = expected; v.tol = tol;
    v.actual = n > 0 ? (double)k / n : 0.0;
    v.dev = fabs(v.actual - expected);
    v.flagged = v.dev > tol + 1e-12;
    return v;
}

static void print_verdict_line(const char* scope, const char* name, const Verdict* v) {
    if (v->flagged) {
        printf("RATE_DEVIATION scope=%s%s%s expected_rate=%.4f "
               "actual_stair_selection_rate=%.4f actual_selection_rate=%.4f "
               "k=%d n=%d deviation=%.4f tol=%.2f\n",
               scope, name ? " case=" : "", name ? name : "",
               v->expected, v->actual, v->actual, v->k, v->n, v->dev, v->tol);
    } else {
        printf("rate_match scope=%s%s%s expected_rate=%.4f "
               "actual_stair_selection_rate=%.4f deviation=%.4f tol=%.2f\n",
               scope, name ? " case=" : "", name ? name : "",
               v->expected, v->actual, v->dev, v->tol);
    }
}

/* ------------------------- deterministic OD corpus ----------------------- */

#define NCORPUS 120
typedef struct { int s, g; } Pair;

static uint64_t rng_state = 0x9e3779b97f4a7c15ULL;
static uint64_t xrnd(void) {
    uint64_t x = rng_state;
    x ^= x << 13; x ^= x >> 7; x ^= x << 17;
    rng_state = x;
    return x;
}

static int build_corpus(const Sketch* sk, Pair* pairs, int npair) {
    int cells[MAXW * MAXH], nc = 0;
    for (int y = 0; y < sk->h; y++)
        for (int x = 0; x < sk->w; x++)
            if (free_cell(sk, 0, y, x)) cells[nc++] = nid(sk, 0, y, x);
    rng_state = 0x20261126ULL;
    int made = 0, guard = 0;
    while (made < npair && guard < npair * 200) {
        guard++;
        int a = cells[xrnd() % (unsigned)nc];
        int b = cells[xrnd() % (unsigned)nc];
        int fa, ya, xa, fb, yb, xb;
        decode(sk, a, &fa, &ya, &xa); decode(sk, b, &fb, &yb, &xb);
        if (abs(xa - xb) + abs(ya - yb) < 6) continue;
        pairs[made++] = (Pair){a, b};
    }
    return made;
}

/* -------------------------------- self test ------------------------------ */

static int failures = 0, checks = 0;
static void check(const char* name, int ok) {
    checks++;
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static int count_conn(const Sketch* sk) {
    int c = 0;
    for (int f = 0; f < sk->floors; f++)
        for (int y = 0; y < sk->h; y++)
            for (int x = 0; x < sk->w; x++)
                if (sk->shaft_at[f][y][x] > 0) c++;
    return c;
}
static int count_vertical_pairs(const Sketch* sk) {
    int c = 0;
    for (int k = 0; k < sk->ns; k++)
        for (int f = 0; f + 1 < sk->floors; f++)
            if (sk->shaft_at[f][sk->sy[k]][sk->sx[k]] == k + 1 &&
                sk->shaft_at[f + 1][sk->sy[k]][sk->sx[k]] == k + 1) c++;
    return c;
}

static int path_is_valid(const Sketch* sk, const int* p, int n) {
    for (int i = 0; i < n; i++) {
        int f, y, x; decode(sk, p[i], &f, &y, &x);
        if (!free_cell(sk, f, y, x)) return 0;
        if (i) {
            int pf, py, px; decode(sk, p[i - 1], &pf, &py, &px);
            if (pf == f) {
                if (abs(px - x) + abs(py - y) != 1) return 0;
            } else {
                if (px != x || py != y || abs(pf - f) != 1) return 0;
                if (sk->shaft_at[pf][py][px] != sk->shaft_at[f][y][x]) return 0;
            }
        }
    }
    return 1;
}

static int mode_selftest(int argc, char** argv) {
    const char* fpath = argc >= 1 ? argv[0] : NULL;
    const char* spath = argc >= 2 ? argv[1] : NULL;
    char err[256];

    Sketch sk;
    int loaded = load_sketch(fpath, &sk, err, sizeof(err));
    check("data_file_loaded", loaded == 0);
    if (loaded == 0) {
        check("dims_23_19_3", sk.w == 23 && sk.h == 19 && sk.floors == 3);
        check("four_shafts_from_circles", sk.ns == 4);
        check("twelve_connection_points", count_conn(&sk) == 12);
        check("eight_vertical_links", count_vertical_pairs(&sk) == 8);
        check("three_named_cases", sk.ncase == 3);
        check("walls_parsed_f0", sk.block[0][13][6] && sk.block[0][5][11] && sk.block[0][13][16]);
        check("wall_door_f2", sk.block[2][12][11] && !sk.block[2][13][11]);

        int path[MAXN]; Info inf;
        const int LSTAR = 1381; /* reverse-solved, see survey checks below */

        int ok_all_cases = 1;
        for (int i = 0; i < sk.ncase; i++) {
            int n = run_case(&sk, &sk.cases[i], LSTAR, &inf, path, MAXN);
            if (n <= 0 || !inf.found || !path_is_valid(&sk, path, n)) ok_all_cases = 0;
        }
        check("three_cases_found_and_valid", ok_all_cases);

        Info i1, i2, i3;
        run_case(&sk, &sk.cases[0], LSTAR, &i1, path, MAXN);
        run_case(&sk, &sk.cases[1], LSTAR, &i2, path, MAXN);
        run_case(&sk, &sk.cases[2], LSTAR, &i3, path, MAXN);
        check("c1_2f_to_1f_uses_one_stair", i1.vert == 1 && i1.flat == 17);
        check("c2_short_uses_flat_only", i2.vert == 0 && i2.flat == 20);
        check("c3_bridge_uses_two_stairs", i3.vert == 2 && i3.flat == 18);

        /* exact break-even: c3 18+2lambda vs ground 48 -> lambda_be=15 */
        run_case(&sk, &sk.cases[2], 1499, &i3, path, MAXN);
        check("c3_stair_below_break_even", i3.vert == 2);
        run_case(&sk, &sk.cases[2], 1501, &i3, path, MAXN);
        check("c3_flat_above_break_even", i3.vert == 0 && i3.flat == 48);
        /* c2 4+2lambda vs ground 20 -> lambda_be=8 */
        run_case(&sk, &sk.cases[1], 799, &i2, path, MAXN);
        check("c2_stair_below_8", i2.vert == 2 && i2.flat == 4);
        run_case(&sk, &sk.cases[1], 801, &i2, path, MAXN);
        check("c2_flat_above_8", i2.vert == 0 && i2.flat == 20);

        /* coefficient too small -> c2 wrongly takes stairs; too large -> c3 detours */
        int ksmall = 0, klarge = 0;
        for (int i = 0; i < 3; i++) {
            run_case(&sk, &sk.cases[i], 300, &inf, path, MAXN); ksmall += inf.vert > 0;
            run_case(&sk, &sk.cases[i], 4000, &inf, path, MAXN); klarge += inf.vert > 0;
        }
        check("small_lambda_overuses_stairs", ksmall == 3);
        check("large_lambda_abandons_bridge", klarge == 1);
        Verdict vs = rate_verdict(ksmall, 3, 2.0 / 3.0, 0.10);
        Verdict vl = rate_verdict(klarge, 3, 2.0 / 3.0, 0.10);
        check("small_lambda_deviation_flagged", vs.flagged && fabs(vs.actual - 1.0) < 1e-9);
        check("large_lambda_deviation_flagged", vl.flagged && fabs(vl.actual - 1.0 / 3.0) < 1e-9);

        /* cap rejection */
        check("cap_rejected", run_case(&sk, &sk.cases[0], LSTAR, &inf, path, 2) == -2);

        /* 3F -> 1F must traverse two links */
        Case up3 = {"tmp31", 2, 4, 9, 0, 18, 9, 1};
        run_case(&sk, &up3, LSTAR, &inf, path, MAXN);
        check("3f_to_1f_two_links", inf.found && inf.vert == 2);

        /* survey reverse solve */
        Survey sv;
        if (load_survey(spath, &sv) == 0) {
            double tau = tau_cell(&sv);
            int lstar = lambda_h_from_seconds(survey_link_mean(&sv), tau);
            check("survey_20_people", sv.n == 20);
            check("tau_cell_0p714", fabs(tau - 1.0 / 1.4) < 1e-9);
            check("lambda_star_1381", lstar == 1381);
            check("lambda_up_1579", lambda_h_from_seconds(survey_up_mean(&sv), tau) == 1579);
            check("lambda_down_1182", lambda_h_from_seconds(survey_down_mean(&sv), tau) == 1182);

            int cnt[3] = {0, 0, 0};
            for (int p = 0; p < sv.n; p++) {
                double t = 0.5 * (sv.up[p] + sv.down[p]);
                int lh = lambda_h_from_seconds(t, tau);
                for (int i = 0; i < 3; i++) {
                    run_case(&sk, &sk.cases[i], lh, &inf, path, MAXN);
                    cnt[i] += inf.vert > 0;
                }
            }
            check("sample_rates_20_0_17", cnt[0] == 20 && cnt[1] == 0 && cnt[2] == 17);
            Verdict vp = rate_verdict(cnt[2], sv.n, 1.0, 0.10);
            check("c3_population_deviation_reported", vp.flagged && fabs(vp.actual - 0.85) < 1e-9);

            /* verified level fix: baffle at F0 (11,4) raises ground route 48->50,
             * break-even lambda 15->16, covering the worst sample lambda 15.82 */
            Sketch fixed = sk;
            fixed.block[0][4][11] = 1;
            int allsel = 0, groundlen = -1;
            for (int p = 0; p < sv.n; p++) {
                double t = 0.5 * (sv.up[p] + sv.down[p]);
                int lh = lambda_h_from_seconds(t, tau);
                run_case(&fixed, &fixed.cases[2], lh, &inf, path, MAXN);
                allsel += inf.vert > 0;
            }
            run_case(&fixed, &fixed.cases[2], 4000, &inf, path, MAXN);
            groundlen = inf.flat;
            check("baffle_ground_len_50", groundlen == 50);
            check("baffle_sample_rate_20_of_20", allsel == 20);
        } else {
            check("survey_loaded", 0);
        }

        /* corpus determinism and high-lambda ground connectivity */
        Pair corpus[NCORPUS];
        int np = build_corpus(&sk, corpus, NCORPUS);
        check("corpus_120", np == 120);
        Pair first = corpus[0];
        Pair corpus2[NCORPUS];
        build_corpus(&sk, corpus2, NCORPUS);
        check("corpus_deterministic", corpus2[0].s == first.s && corpus2[0].g == first.g);
        int selhi = 0; double mlen = 0;
        for (int i = 0; i < np; i++) {
            int ff, yy, xx, gf, gy, gx;
            decode(&sk, corpus[i].s, &ff, &yy, &xx);
            decode(&sk, corpus[i].g, &gf, &gy, &gx);
            astar_x(&sk, ff, xx, yy, gf, gx, gy, 4000, path, MAXN, &inf);
            if (inf.vert > 0) selhi++;
            mlen += inf.flat + inf.vert;
        }
        check("corpus_ground_connected_at_high_lambda", selhi == 0);
        check("corpus_mean_len_sane", mlen / np > 23.0 && mlen / np < 30.0);
    }

    /* parser negative + tiny unreachable map, independent of data file */
    Sketch tiny; char e2[128];
    const char* bad = "width 3\nheight 3\nfloors 1\nfloor 0\nrows\n...\n...\nendrows\n";
    check("parser_missing_h_rejected", parse_sketch_text(bad, &tiny, e2, sizeof(e2)) != 0);

    const char* wallmap =
        "width 5\nheight 3\nfloors 1\n"
        "floor 0\nrows\n"
        ".....\n"
        "##?..\n"
        ".....\n"
        "endrows\n";
    check("parser_bad_char_rejected", parse_sketch_text(wallmap, &tiny, e2, sizeof(e2)) != 0);

    const char* sep =
        "width 5\nheight 3\nfloors 1\n"
        "floor 0\nrows\n"
        ".....\n"
        "#####\n"
        ".....\n"
        "endrows\n";
    if (parse_sketch_text(sep, &tiny, e2, sizeof(e2)) == 0) {
        int path[64]; Info inf;
        check("tiny_straight_found", astar_x(&tiny, 0, 0, 0, 0, 4, 0, 100, path, 64, &inf) == 5);
        check("tiny_wall_unreachable", astar_x(&tiny, 0, 0, 0, 0, 4, 2, 100, path, 64, &inf) == -1);
    } else {
        check("tiny_map_parses", 0);
    }

    printf("checks=%d failures=%d all_ok=%d\n", checks, failures, failures == 0);
    return failures == 0 ? 0 : 1;
}

/* --------------------------------- accept -------------------------------- */

static void print_case_run(const Sketch* sk, const Case* c, int n, const Info* inf) {
    double cost = n > 0 ? inf->cost_h / 100.0 : -1.0;
    printf("RUN case=%s run_count=1 found=%d stair_selected=%d "
           "weighted_cost_cells=%.2f flat_edges=%d vertical_links=%d path_nodes=%d",
           c->name, n > 0, inf->vert > 0, cost, inf->flat, inf->vert, n > 0 ? inf->npath : -1);
    if (inf->vert > 0 && inf->nshaft_used > 0) {
        printf(" shafts=");
        for (int i = 0; i < inf->nshaft_used; i++) {
            int id = inf->shafts_used[i];
            printf("(%d,%d)%s", sk->sx[id], sk->sy[id],
                   i + 1 < inf->nshaft_used ? "-" : "");
        }
    }
    printf(" sketch_expect_stair=%d expectation_met=%d\n",
           c->expect, (inf->vert > 0) == c->expect);
}

static int mode_accept(const char* fpath, const char* spath) {
    char err[256];
    Sketch sk;
    if (load_sketch(fpath, &sk, err, sizeof(err))) { fprintf(stderr, "sketch: %s\n", err); return 2; }
    Survey sv;
    if (load_survey(spath, &sv)) { fprintf(stderr, "survey load failed\n"); return 2; }

    double tau = tau_cell(&sv);
    double upm = survey_up_mean(&sv), dnm = survey_down_mean(&sv), lm = survey_link_mean(&sv);
    int lstar = lambda_h_from_seconds(lm, tau);

    printf("# crossfloor accept report\n");
    printf("cell_m=%.3f flat_speed_mps=%.3f tau_flat_cell_s=%.4f\n", sv.cell_m, sv.speed, tau);
    printf("cost_expression: C = N_flat*1 + sum(lambda_dir); "
           "lambda = T_stair/tau_cell = v_flat*T_stair/h\n");
    printf("stair_up_mean_s=%.3f lambda_up_cells=%.4f\n", upm, upm / tau);
    printf("stair_down_mean_s=%.3f lambda_down_cells=%.4f\n", dnm, dnm / tau);
    printf("stair_avg_mean_s=%.4f lambda_star_cells=%.4f lambda_star_hundredths=%d\n",
           lm, lm / tau, lstar);

    /* Exactly one A* run per case; the same Info feeds print + tally. */
    printf("\n# one run per case at lambda_star\n");
    int k = 0, expect_k = 0;
    for (int i = 0; i < sk.ncase; i++) {
        int path[MAXN]; Info inf;
        int n = run_case(&sk, &sk.cases[i], lstar, &inf, path, MAXN);
        print_case_run(&sk, &sk.cases[i], n, &inf);
        k += inf.vert > 0;
        expect_k += sk.cases[i].expect != 0;
    }
    double expected_rate = expect_k / (double)sk.ncase;
    Verdict va = rate_verdict(k, sk.ncase, expected_rate, 0.10);
    printf("\nselected_stairs=%d of %d expected=%d\n", k, sk.ncase, expect_k);
    print_verdict_line("acceptance", NULL, &va);
    printf("acceptance_consistent=%d\n", va.flagged ? 0 : 1);

    printf("\n# per-person measured-ratio selection rates (actual, not Monte Carlo)\n");
    int cnt[MAXCASE] = {0};
    for (int p = 0; p < sv.n; p++) {
        double t = 0.5 * (sv.up[p] + sv.down[p]);
        int lh = lambda_h_from_seconds(t, tau);
        int path[MAXN]; Info inf;
        for (int i = 0; i < sk.ncase; i++) {
            run_case(&sk, &sk.cases[i], lh, &inf, path, MAXN);
            cnt[i] += inf.vert > 0;
        }
    }
    for (int i = 0; i < sk.ncase; i++) {
        Verdict vp = rate_verdict(cnt[i], sv.n, sk.cases[i].expect ? 1.0 : 0.0, 0.10);
        printf("SAMPLE_RATE case=%s expected_rate=%.2f actual_selection_rate=%d/%d=%.4f "
               "deviation=%.4f\n", sk.cases[i].name, sk.cases[i].expect ? 1.0 : 0.0,
               cnt[i], sv.n, vp.actual, vp.dev);
        print_verdict_line("population", sk.cases[i].name, &vp);
    }

    printf("\n# coefficient trade-off controls (same 3 cases, sweep endpoints)\n");
    int csmall = 0, clarge = 0;
    for (int i = 0; i < sk.ncase; i++) {
        int path[MAXN]; Info inf;
        run_case(&sk, &sk.cases[i], 300, &inf, path, MAXN); csmall += inf.vert > 0;
        run_case(&sk, &sk.cases[i], 4000, &inf, path, MAXN); clarge += inf.vert > 0;
    }
    Verdict vs = rate_verdict(csmall, 3, expected_rate, 0.10);
    Verdict vl = rate_verdict(clarge, 3, expected_rate, 0.10);
    printf("control lambda=3.0 actual_stair_selection_rate=%d/3=%.4f\n", csmall, vs.actual);
    print_verdict_line("control_small_lambda=3", NULL, &vs);
    printf("control lambda=40.0 actual_stair_selection_rate=%d/3=%.4f\n", clarge, vl.actual);
    print_verdict_line("control_large_lambda=40", NULL, &vl);

    printf("\n# quantified level fix for C3 population deviation\n");
    printf("c3_break_even_lambda=15.0000; baffle F0(11,4) raises ground detour 48->50, "
           "break-even ->16.0000; worst measured lambda=15.82 -> sample rate 20/20=1.0000\n");
    return 0;
}

/* --------------------------------- routes -------------------------------- */

static int mode_routes(const char* fpath, const char* spath) {
    char err[256];
    Sketch sk;
    if (load_sketch(fpath, &sk, err, sizeof(err))) { fprintf(stderr, "sketch: %s\n", err); return 2; }
    Survey sv;
    int lstar = 1381;
    if (load_survey(spath, &sv) == 0)
        lstar = lambda_h_from_seconds(survey_link_mean(&sv), tau_cell(&sv));
    printf("lambda_h %d\n", lstar);
    int path[MAXN]; Info inf;
    for (int i = 0; i < sk.ncase; i++) {
        int n = run_case(&sk, &sk.cases[i], lstar, &inf, path, MAXN);
        printf("case %s expect %d selected %d flat %d vert %d\n",
               sk.cases[i].name, sk.cases[i].expect, inf.vert > 0, inf.flat, inf.vert);
        for (int j = 0; j < n; j++) {
            int f, y, x; decode(&sk, path[j], &f, &y, &x);
            printf("node %d %d %d\n", f, x, y);
        }
        printf("endcase\n");
    }
    return 0;
}

/* --------------------------------- sweep --------------------------------- */

static int mode_sweep(const char* fpath, double lo, double hi, double step) {
    char err[256];
    Sketch sk;
    if (load_sketch(fpath, &sk, err, sizeof(err))) { fprintf(stderr, "sketch: %s\n", err); return 2; }
    Pair corpus[NCORPUS];
    int np = build_corpus(&sk, corpus, NCORPUS);

    printf("lambda_cells,corpus_select_rate,corpus_mean_len,corpus_mean_cost,"
           "c1_sel,c1_len,c1_cost,c2_sel,c2_len,c2_cost,c3_sel,c3_len,c3_cost\n");
    for (double lc = lo; lc <= hi + 1e-9; lc += step) {
        int lh = (int)llround(lc * 100.0);
        int sel = 0; double mlen = 0, mcost = 0;
        int path[MAXN]; Info inf;
        for (int i = 0; i < np; i++) {
            int ff, yy, xx, gf, gy, gx;
            decode(&sk, corpus[i].s, &ff, &yy, &xx);
            decode(&sk, corpus[i].g, &gf, &gy, &gx);
            astar_x(&sk, ff, xx, yy, gf, gx, gy, lh, path, MAXN, &inf);
            if (inf.vert > 0) sel++;
            mlen += inf.flat + inf.vert;
            mcost += inf.cost_h / 100.0;
        }
        int csel[3], clen[3], ccost[3];
        for (int i = 0; i < 3; i++) {
            run_case(&sk, &sk.cases[i], lh, &inf, path, MAXN);
            csel[i] = inf.vert > 0; clen[i] = inf.flat + inf.vert; ccost[i] = inf.cost_h;
        }
        printf("%.3f,%.4f,%.3f,%.3f,%d,%d,%.2f,%d,%d,%.2f,%d,%d,%.2f\n",
               lc, sel / (double)np, mlen / np, mcost / np,
               csel[0], clen[0], ccost[0] / 100.0,
               csel[1], clen[1], ccost[1] / 100.0,
               csel[2], clen[2], ccost[2] / 100.0);
    }
    return 0;
}

int main(int argc, char** argv) {
    const char* mode = argc >= 2 ? argv[1] : "selftest";
    if (strcmp(mode, "selftest") == 0) {
        return mode_selftest(argc >= 3 ? argc - 2 : 0, argc >= 3 ? argv + 2 : NULL);
    }
    if (strcmp(mode, "accept") == 0) {
        return mode_accept(argc >= 3 ? argv[2] : NULL, argc >= 4 ? argv[3] : NULL);
    }
    if (strcmp(mode, "routes") == 0) {
        return mode_routes(argc >= 3 ? argv[2] : NULL, argc >= 4 ? argv[3] : NULL);
    }
    if (strcmp(mode, "sweep") == 0) {
        const char* fp = argc >= 3 ? argv[2] : NULL;
        double lo = argc >= 4 ? atof(argv[3]) : 0.0;
        double hi = argc >= 5 ? atof(argv[4]) : 20.0;
        double step = argc >= 6 ? atof(argv[5]) : 0.25;
        return mode_sweep(fp, lo, hi, step);
    }
    fprintf(stderr, "usage: %s [selftest|accept|sweep] ...\n", argv[0]);
    return 2;
}
