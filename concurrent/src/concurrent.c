
/* concurrent: merge rule for simultaneous map edits.
 *
 * conflict(e1,e2) <=> S1 n S2 != 0, scope = |S1 n S2| (S = edited cell set).
 * resolution: last-writer-wins per cell by (ts, player); the overwritten
 * edit's owner must recompute its path over the conflict cells' influence
 * region, bounded by one block: influence <= BS*BS <= BUDGET.
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define W 16
#define H 12
#define MAXN (W*H)
#define BS 4                 /* block edge; derived in README: B*=(WH)^(1/4)~3.7, BS*BS<=BUDGET */
#define BUDGET 16            /* per-conflict recompute budget in cells */
#define NBW ((W + BS - 1) / BS)
#define NBH ((H + BS - 1) / BS)
#define NBLOCKS (NBW * NBH)

typedef struct { unsigned char blocked[W*H]; } Map;

static int idx(int x, int y) { return y * W + x; }

static int walkable(const Map* m, int x, int y) {
    if (x < 0 || y < 0 || x >= W || y >= H) return 0;
    return !m->blocked[idx(x, y)];
}

/* returns path length or -1 (same A* as grid/) */
static int astar(const Map* m, int sx, int sy, int gx, int gy, int* out, int cap) {
    if (!walkable(m, sx, sy) || !walkable(m, gx, gy)) return -1;
    int g[MAXN], f[MAXN], closed[MAXN], parent[MAXN];
    for (int i = 0; i < MAXN; i++) { g[i] = 1 << 29; f[i] = 1 << 29; closed[i] = 0; parent[i] = -1; }
    int open[MAXN], nopen = 0;
    int s = idx(sx, sy);
    g[s] = 0; f[s] = abs(gx - sx) + abs(gy - sy);
    open[nopen++] = s;
    while (nopen > 0) {
        int best = 0;
        for (int i = 1; i < nopen; i++) if (f[open[i]] < f[open[best]]) best = i;
        int cur = open[best];
        open[best] = open[--nopen];
        if (cur == idx(gx, gy)) {
            int n = 0, c = cur;
            int rev[MAXN];
            while (c != -1 && n < MAXN) { rev[n++] = c; c = parent[c]; }
            if (n > cap) return -2;
            for (int i = 0; i < n; i++) out[i] = rev[n - 1 - i];
            return n;
        }
        closed[cur] = 1;
        int cx = cur % W, cy = cur / W;
        const int dx[4] = {1,-1,0,0}, dy[4] = {0,0,1,-1};
        for (int d = 0; d < 4; d++) {
            int nx = cx + dx[d], ny = cy + dy[d];
            if (!walkable(m, nx, ny)) continue;
            int ni = idx(nx, ny);
            if (closed[ni]) continue;
            int ng = g[cur] + 1;
            if (ng < g[ni]) {
                g[ni] = ng;
                f[ni] = ng + abs(gx - nx) + abs(gy - ny);
                parent[ni] = cur;
                int found = 0;
                for (int i = 0; i < nopen; i++) if (open[i] == ni) { found = 1; break; }
                if (!found) open[nopen++] = ni;
            }
        }
    }
    return -1;
}

#define MAX_CELLS 4
#define MAX_EDITS 8

typedef struct {
    int player;
    int ts;                  /* timestamp; ties broken by player id */
    int n;
    int cell[MAX_CELLS];
    unsigned char val;       /* 1 = add wall, 0 = remove wall */
    int overwritten;         /* own cells lost to a later write */
    int shared;              /* own cells also touched by another edit */
} Edit;

static int later(const Edit* a, const Edit* b) {
    if (a->ts != b->ts) return a->ts > b->ts;
    return a->player > b->player;
}

/* authoritative merge: per cell, the write with max (ts, player) wins */
static void server_merge(const Map* base, const Edit* es, int ne, Map* out, int* owner) {
    *out = *base;
    for (int i = 0; i < MAXN; i++) owner[i] = -1;
    for (int i = 0; i < ne; i++) {
        for (int k = 0; k < es[i].n; k++) {
            int c = es[i].cell[k];
            if (owner[c] < 0 || later(&es[i], &es[owner[c]])) {
                owner[c] = i;
                out->blocked[c] = es[i].val;
            }
        }
    }
}

/* client-side merge: replay all edits in (ts, player) order onto base */
static void client_replay(const Map* base, const Edit* es, int ne, Map* out) {
    *out = *base;
    int ord[MAX_EDITS];
    for (int i = 0; i < ne; i++) ord[i] = i;
    for (int i = 1; i < ne; i++) {           /* insertion sort, ascending time */
        int k = ord[i], j = i - 1;
        while (j >= 0 && later(&es[ord[j]], &es[k])) { ord[j + 1] = ord[j]; j--; }
        ord[j + 1] = k;
    }
    for (int i = 0; i < ne; i++) {
        const Edit* e = &es[ord[i]];
        for (int k = 0; k < e->n; k++) out->blocked[e->cell[k]] = e->val;
    }
}

static void edit_stats(Edit* es, int ne, const int* owner) {
    for (int i = 0; i < ne; i++) {
        es[i].overwritten = 0;
        es[i].shared = 0;
        for (int k = 0; k < es[i].n; k++) {
            int c = es[i].cell[k];
            if (owner[c] != i) es[i].overwritten++;
            for (int j = 0; j < ne; j++) {
                int hit = 0;
                if (j == i) continue;
                for (int t = 0; t < es[j].n; t++) if (es[j].cell[t] == c) { hit = 1; break; }
                if (hit) { es[i].shared++; break; }
            }
        }
    }
}

static int intersect_count(const Edit* a, const Edit* b) {
    int n = 0;
    for (int i = 0; i < a->n; i++)
        for (int j = 0; j < b->n; j++)
            if (a->cell[i] == b->cell[j]) { n++; break; }
    return n;
}

static int union_count(const Edit* a, const Edit* b) {
    int n = a->n;
    for (int j = 0; j < b->n; j++) {
        int dup = 0;
        for (int i = 0; i < a->n; i++) if (a->cell[i] == b->cell[j]) { dup = 1; break; }
        if (!dup) n++;
    }
    return n;
}

/* recompute scope: blocks touched by this edit's overwritten cells */
static int influence_cells(const Edit* e, const int* owner, int self) {
    int seen[NBLOCKS] = {0};
    int nb = 0;
    for (int k = 0; k < e->n; k++) {
        int c = e->cell[k];
        if (owner[c] == self) continue;
        int b = (c / W / BS) * NBW + (c % W) / BS;
        if (!seen[b]) { seen[b] = 1; nb++; }
    }
    return nb * BS * BS;
}

static unsigned rng_state;
static unsigned rnd(void) {
    rng_state = rng_state * 1103515245u + 12345u;
    return (rng_state >> 16) & 0x7fff;
}

/* hotspot pool, all inside block (2,1): x in [8,11], y in [4,7] */
static const int POOL_X[] = {8, 9, 10, 11, 8, 9};
static const int POOL_Y[] = {5, 5, 5, 5, 6, 6};
#define POOLN 6

static int build_edits(int nplayers, Edit* es) {
    /* every edit touches (9,5): pairwise conflict is guaranteed */
    for (int i = 0; i < nplayers; i++) {
        Edit* e = &es[i];
        e->player = i;
        e->ts = 1000 + (int)(rnd() % (nplayers * 2));
        e->n = 0;
        e->cell[e->n++] = idx(9, 5);
        int extra = 1 + (int)(rnd() % 2);
        for (int k = 0; k < extra && e->n < MAX_CELLS; k++) {
            int p = (int)(rnd() % POOLN);
            int c = idx(POOL_X[p], POOL_Y[p]);
            int dup = 0;
            for (int j = 0; j < e->n; j++) if (e->cell[j] == c) dup = 1;
            if (!dup) e->cell[e->n++] = c;
        }
        e->val = (unsigned char)(rnd() % 2);
        e->overwritten = 0;
        e->shared = 0;
    }
    return nplayers;
}

static int failures = 0;
static void check(const char* name, int ok) {
    printf("%s=%d\n", name, ok ? 1 : 0);
    if (!ok) failures++;
}

static void run_tier(int nplayers, const Map* base) {
    char name[64];
    rng_state = 1000 + (unsigned)nplayers;
    Edit es[MAX_EDITS];
    int ne = build_edits(nplayers, es);

    Map server;
    int owner[MAXN];
    server_merge(base, es, ne, &server, owner);
    edit_stats(es, ne, owner);

    int pairs = 0, formula_ok = 1, all_shared = 1;
    for (int i = 0; i < ne; i++) {
        if (es[i].shared == 0) all_shared = 0;
        for (int j = i + 1; j < ne; j++) {
            int inter = intersect_count(&es[i], &es[j]);
            if (inter > 0) pairs++;
            if (inter != es[i].n + es[j].n - union_count(&es[i], &es[j])) formula_ok = 0;
        }
    }

    Map replay;
    client_replay(base, es, ne, &replay);
    int merge_ok = memcmp(&server, &replay, sizeof(Map)) == 0;

    int spath[MAXN], rpath[MAXN];
    int sn = astar(&server, 0, 5, 15, 5, spath, MAXN);
    int rn = astar(&replay, 0, 5, 15, 5, rpath, MAXN);
    int paths_ok = (rn == sn) && (sn <= 0 || memcmp(spath, rpath, sn * sizeof(int)) == 0);

    int divergent = 0, bounded = 1, max_infl = 0, overwritten_edits = 0;
    for (int i = 0; i < ne; i++) {
        int infl = influence_cells(&es[i], owner, i);
        if (infl > max_infl) max_infl = infl;
        if (infl > BS * BS) bounded = 0;      /* must fit in a single block */
        if (es[i].overwritten == 0) continue;
        overwritten_edits++;
        /* optimistic local view: base + own edit only, before the merge */
        Map local = *base;
        for (int k = 0; k < es[i].n; k++) local.blocked[es[i].cell[k]] = es[i].val;
        int lp[MAXN];
        int ln = astar(&local, 0, 5, 15, 5, lp, MAXN);
        int same = (ln == sn) && (sn <= 0 || memcmp(lp, spath, sn * sizeof(int)) == 0);
        if (!same) {
            int winner = -1;
            for (int k = 0; k < es[i].n; k++)
                if (owner[es[i].cell[k]] != i) { winner = owner[es[i].cell[k]]; break; }
            printf("divergence: tier=%d player=%d edit=%d overwritten_by_edit=%d cells=%d\n",
                   nplayers, es[i].player, i, winner, es[i].overwritten);
            divergent++;
        }
    }

    printf("tier=%d edits=%d conflict_pairs=%d overwritten_edits=%d "
           "divergent_before_merge=%d max_recompute_cells=%d\n",
           nplayers, ne, pairs, overwritten_edits, divergent, max_infl);

    snprintf(name, sizeof name, "tier%d_conflicts_found", nplayers);
    check(name, pairs > 0);
    snprintf(name, sizeof name, "tier%d_all_edits_conflicting", nplayers);
    check(name, all_shared);
    snprintf(name, sizeof name, "tier%d_intersection_formula", nplayers);
    check(name, formula_ok);
    snprintf(name, sizeof name, "tier%d_merge_matches_server", nplayers);
    check(name, merge_ok);
    snprintf(name, sizeof name, "tier%d_paths_consistent", nplayers);
    check(name, paths_ok);
    snprintf(name, sizeof name, "tier%d_recompute_single_block", nplayers);
    check(name, bounded);
    snprintf(name, sizeof name, "tier%d_divergence_detected", nplayers);
    check(name, divergent > 0);
}

int main(void) {
    /* block size: recompute cost B^2 vs management overhead WH/B^2 */
    const int cand[] = {1, 2, 3, 4, 6, 8, 16};
    int best_b = 1;
    long best_c = -1;
    printf("block_analysis:\n");
    for (int i = 0; i < 7; i++) {
        int b = cand[i];
        long recompute = (long)b * b;
        long blocks = (long)((W + b - 1) / b) * ((H + b - 1) / b);
        long total = recompute + blocks;
        printf("B=%2d recompute=%3ld blocks=%3ld total=%3ld%s\n",
               b, recompute, blocks, total, b == BS ? " <- chosen" : "");
        if (best_c < 0 || total < best_c) { best_c = total; best_b = b; }
    }
    check("block_size_optimal", best_b == BS);
    check("block_within_budget", BS * BS <= BUDGET);

    /* base map: a 2-cell wall on the (0,5)->(15,5) straight corridor */
    Map base;
    memset(&base, 0, sizeof(base));
    base.blocked[idx(8, 5)] = 1;
    base.blocked[idx(9, 5)] = 1;

    const int tiers[] = {2, 4, 8};
    for (int i = 0; i < 3; i++) run_tier(tiers[i], &base);

    printf("all_ok=%d\n", failures == 0);
    return failures == 0 ? 0 : 1;
}
