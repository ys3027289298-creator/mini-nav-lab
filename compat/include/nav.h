#ifndef NAV_H
#define NAV_H

#include "mapfile.h"

/*
 * Navigation used to compare the two roads into gen 2: read-and-upgrade vs
 * re-export-then-read. Both sides run exactly this code on the cells and
 * query they ended up with, so a difference in the resulting path is a
 * difference the upgrade produced, never a difference in how A* breaks ties.
 *
 * The algorithm mirrors grid/src/grid.c: four-neighbour A*, the open set
 * scanned for the smallest f, ties going to the earlier insertion, Manhattan
 * heuristic. Keeping the tie-break identical is load-bearing for a
 * point-by-point comparison; a "better" A* here would bury the signal.
 */

typedef struct {
    int n;              /* points in the path, -1 if none */
    int cap;            /* capacity of x/y */
    uint16_t* x;
    uint16_t* y;
} NavPath;

/* Allocate a path able to hold every cell of an w*h map. */
int  nav_path_init(NavPath* p, int cells);
void nav_path_free(NavPath* p);

/* Returns 1 if a path was found, 0 if not. Coordinates outside the map are
 * unreachable. Leaves p->n == -1 when there is no path. */
int nav_astar(const MapFile* m, int sx, int sy, int gx, int gy, NavPath* p);

/*
 * Compare two paths point by point.
 *   *same_points  = positions where both paths hold the same cell
 *   *ratio_denom  = max(n1, n2), so a path that is a strict prefix of the
 *                   other still counts its shared points
 *   *first_diff   = index of the first differing point, -1 if none
 * Returns 1 when the paths are identical: same length, every point equal.
 */
int nav_compare(const NavPath* a, const NavPath* b, int* same_points,
                int* ratio_denom, int* first_diff);

#endif
