#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
reconcile.py - walkable-count vs flood-fill-count reconciliation.

Statistical scope (declared BEFORE any reconciliation):
  * Universe is exactly one W*H grid snapshot, identified by sha256 of the
    blocked bitmap. Both counters must refer to this same snapshot.
  * W (walkable count) = number of in-bounds cells with blocked == 0.
  * V (the "connected-component statistic" under audit) is the raw scalar
    returned by the instrumented flood-fill variant. In the 340/352 report
    this scalar counts fill events, NOT the number of component ids.
  * Seed scope matters: "all-scan" seeds every unvisited cell (mirrors
    region.c), "main-region" fills from one declared seed only.

Accounting identity. Every +1 increment of the IUT counter emits one raw
(x, y) event; the auditor classifies every event coordinate ITSELF:

    good   : first event for an in-bounds walkable cell
    repeat : later event for an already-good walkable cell   -> R
    blocked: event on an in-bounds blocked cell              -> K
    oob    : event carrying an out-of-bounds coordinate       -> O

Let U = walkable cells with no good event. There are W-U good events, hence

    V - W = R + O + K - U + L

where L = V - number_of_events is the ledger gap (counter increments with no
event, or events with no increment). The task's 3-term formula is the K=L=0
special case:

    D = V - W = R + O - U.

All arithmetic is integer; a passing experiment has formula_error == 0.
"""

import argparse
import hashlib
import sys

DIRS = ((1, 0), (-1, 0), (0, 1), (0, -1))


class Grid:
    def __init__(self, w, h, blocked=(), name=""):
        self.w = w
        self.h = h
        self.blocked = set(blocked)
        self.name = name

    def in_bounds(self, x, y):
        return 0 <= x < self.w and 0 <= y < self.h

    def is_blocked(self, x, y):
        return (x, y) in self.blocked

    def walkable_count(self):
        return self.w * self.h - len(self.blocked)

    def digest(self):
        rows = []
        for y in range(self.h):
            rows.append("".join("#" if (x, y) in self.blocked else "."
                                for x in range(self.w)))
        blob = "\n".join(rows).encode()
        return hashlib.sha256(blob).hexdigest()[:12], rows

    def render(self, marks=None):
        """ASCII map; marks maps (x,y) -> char overriding the base glyph."""
        marks = marks or {}
        _, rows = self.digest()
        out = ["    " + "".join(str(x % 10) for x in range(self.w))]
        for y, row in enumerate(rows):
            cells = []
            for x, ch in enumerate(row):
                cells.append(marks.get((x, y), ch))
            out.append("%3d %s" % (y, " ".join(cells)))
        return "\n".join(out)


# ---------------------------------------------------------------------------
# Implementations under test. Each returns (scalar_count, raw_events, extra).
# Every scalar +1 is accompanied by exactly one event append.
# ---------------------------------------------------------------------------

def iut_all_scan(g):
    """Exact control-flow mirror of region.c components(); counts claims."""
    comp = [-1] * (g.w * g.h)
    events = []
    total = 0
    ncomp = 0

    def idx(x, y):
        return y * g.w + x

    for y in range(g.h):
        for x in range(g.w):
            s = idx(x, y)
            if g.is_blocked(x, y) or comp[s] >= 0:
                continue
            stack = [s]
            comp[s] = ncomp
            total += 1
            events.append((x, y))
            while stack:
                cur = stack.pop()
                cx, cy = cur % g.w, cur // g.w
                for dx, dy in DIRS:
                    nx, ny = cx + dx, cy + dy
                    if not g.in_bounds(nx, ny):
                        continue
                    ni = idx(nx, ny)
                    if g.is_blocked(nx, ny) or comp[ni] >= 0:
                        continue
                    comp[ni] = ncomp
                    total += 1
                    events.append((nx, ny))
                    stack.append(ni)
            ncomp += 1
    return total, events, {"components": ncomp}


def iut_main_region(g, sx, sy):
    """Single-source correct fill; only the seed's region is counted."""
    assert g.in_bounds(sx, sy) and not g.is_blocked(sx, sy)
    seen = set()
    events = []
    stack = [(sx, sy)]
    seen.add((sx, sy))
    count = 0
    while stack:
        x, y = stack.pop()
        count += 1
        events.append((x, y))
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if not g.in_bounds(nx, ny):
                continue
            p = (nx, ny)
            if p in seen or g.is_blocked(nx, ny):
                continue
            seen.add(p)
            stack.append(p)
    return count, events, {"seed": (sx, sy)}


def iut_duplicate_pop(g, sx, sy):
    """Injected fault: mark on pop, no seen guard at push.

    A cell can be pushed by several neighbours and popped multiple times;
    every pop increments the scalar, so repeat fills become real events.
    Terminates because expansion happens only on the first pop.
    """
    seen = set()
    events = []
    stack = [(sx, sy)]
    count = 0
    while stack:
        x, y = stack.pop()
        count += 1                       # one increment per pop
        events.append((x, y))
        if (x, y) in seen:
            continue
        seen.add((x, y))
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if g.in_bounds(nx, ny) and not g.is_blocked(nx, ny):
                stack.append((nx, ny))   # no push-time guard
    return count, events, {"seed": (sx, sy)}


def iut_oob_counted(g, sx, sy):
    """Injected fault: the out-of-bounds branch also ticks the counter.

    Push-time seen guard keeps R == 0; blocked neighbours do not tick.
    Only outward probes from reached border cells add spurious counts.
    """
    seen = {(sx, sy)}
    events = [(sx, sy)]
    count = 1
    stack = [(sx, sy)]
    while stack:
        x, y = stack.pop()
        for dx, dy in DIRS:
            nx, ny = x + dx, y + dy
            if not g.in_bounds(nx, ny):
                count += 1               # injected: OOB probe is "counted"
                events.append((nx, ny))
                continue
            p = (nx, ny)
            if p in seen or g.is_blocked(nx, ny):
                continue
            seen.add(p)
            count += 1
            events.append(p)
            stack.append(p)
    return count, events, {"seed": (sx, sy)}


def iut_count_before_blocked(g):
    """Reconstruction of the 340/352 fault: claim/count before blocked test.

    Seen guard is first, so each in-bounds coordinate is counted at most
    once; blocked cells adjacent to the walkable region are counted (K) but
    never expanded. All-scan seeding keeps U == 0.
    """
    comp = [-1] * (g.w * g.h)
    events = []
    total = 0
    cid = 0

    def idx(x, y):
        return y * g.w + x

    for y in range(g.h):
        for x in range(g.w):
            s = idx(x, y)
            if comp[s] >= 0:
                continue
            # NOTE: no blocked skip for the seed either in this mutant;
            # our constructed maps only seed walkable cells in row scan
            # order before their blocked turn arrives via neighbours.
            stack = [s]
            comp[s] = cid
            total += 1
            events.append((x, y))
            while stack:
                cur = stack.pop()
                cx, cy = cur % g.w, cur // g.w
                for dx, dy in DIRS:
                    nx, ny = cx + dx, cy + dy
                    if not g.in_bounds(nx, ny):
                        continue
                    ni = idx(nx, ny)
                    if comp[ni] >= 0:
                        continue
                    comp[ni] = cid
                    total += 1
                    events.append((nx, ny))
                    if g.is_blocked(nx, ny):
                        continue         # counted, but not expanded
                    stack.append(ni)
            cid += 1
    return total, events, {"components_marked": cid}


# ---------------------------------------------------------------------------
# Independent auditor
# ---------------------------------------------------------------------------

def reference_reachable(g, seeds):
    """Set-based BFS truth, written independently of the IUTs."""
    seen = set()
    for sx, sy in seeds:
        if g.in_bounds(sx, sy) and not g.is_blocked(sx, sy):
            seen.add((sx, sy))
    frontier = list(seen)
    while frontier:
        x, y = frontier.pop()
        for dx, dy in DIRS:
            p = (x + dx, y + dy)
            if (g.in_bounds(*p) and p not in seen
                    and not g.is_blocked(*p)):
                seen.add(p)
                frontier.append(p)
    return seen


def audit(g, v_iut, events):
    w_count = g.walkable_count()
    good = set()
    r_coords, k_coords, o_coords = [], [], []
    for x, y in events:
        if not g.in_bounds(x, y):
            o_coords.append((x, y))
        elif g.is_blocked(x, y):
            k_coords.append((x, y))
        elif (x, y) in good:
            r_coords.append((x, y))
        else:
            good.add((x, y))
    R = len(r_coords)
    O = len(o_coords)
    K = len(k_coords)
    U = w_count - len(good)
    ledger_gap = v_iut - len(events)
    d_meas = v_iut - w_count
    d_pred3 = R + O - U
    d_pred4 = R + O + K - U + ledger_gap
    return {
        "W": w_count, "V": v_iut, "events": len(events),
        "good": len(good), "R": R, "O": O, "K": K, "U": U, "L": ledger_gap,
        "d_meas": d_meas, "d_pred3": d_pred3, "d_pred4": d_pred4,
        "err3": d_pred3 - d_meas, "err4": d_pred4 - d_meas,
        "r_coords": r_coords, "k_coords": k_coords,
        "o_coords": o_coords, "good_set": good,
    }


def missing_walkable_coords(g, good):
    return [(x, y) for y in range(g.h) for x in range(g.w)
            if not g.is_blocked(x, y) and (x, y) not in good]


def fmt_coords(cs, limit=40):
    if not cs:
        return "[]"
    shown = ", ".join("(%d,%d)" % c for c in cs[:limit])
    if len(cs) > limit:
        shown += " ... (+%d more)" % (len(cs) - limit)
    return "[" + shown + "]"


def print_header(title, g, scope):
    digest, _ = g.digest()
    print("=" * 72)
    print(title)
    print("-" * 72)
    print("map=%s size=%dx%d cells=%d blocked=%d sha256_12=%s"
          % (g.name or "-", g.w, g.h, g.w * g.h, len(g.blocked), digest))
    print("scope=%s" % scope)
    print("-" * 72)


def verdict(ok):
    print("-" * 72)
    print("verdict=%s" % ("PASS formula_error=0" if ok
                          else "FAIL formula_error!=0"))
    print("=" * 72)
    return 0 if ok else 1


# ---------------------------------------------------------------------------
# The three required injection experiments
# ---------------------------------------------------------------------------

def exp_u():
    # 7x5, 3x3 wall ring centred (3,2): one enclosed walkable pocket.
    ring = [(x, y) for x in range(2, 5) for y in range(1, 4)
            if not (x == 3 and y == 2)]
    g = Grid(7, 5, ring, name="isolated-pocket")
    print_header("EXPERIMENT 1 / U term - isolated walkable pocket", g,
                  "main-region only, seed=(0,0); isolated pocket NOT seeded")
    marks = {(3, 2): "I"}
    print(g.render(marks))
    print("legend: .=walkable #=blocked I=isolated walkable (unvisited)")
    v, ev, extra = iut_main_region(g, 0, 0)
    a = audit(g, v, ev)
    truth = reference_reachable(g, [(0, 0)])
    u_ref = g.walkable_count() - len(truth)
    miss = missing_walkable_coords(g, a["good_set"])
    print("W_walkable=%d  V_iut=%d  seed=%s" % (a["W"], a["V"], extra["seed"]))
    print("terms: R=%d O=%d K=%d U_ledger=%d U_independentBFS=%d L=%d"
          % (a["R"], a["O"], a["K"], a["U"], u_ref, a["L"]))
    print("unvisited_walkable_coordinates=%s" % fmt_coords(miss))
    print("delta_measured V-W=%+d" % a["d_meas"])
    print("delta_predicted R+O-U=%+d" % a["d_pred3"])
    print("formula_error=%+d" % a["err3"])
    # Scope contrast: under all-scan seeding the pocket is its own component.
    v2, ev2, ex2 = iut_all_scan(g)
    a2 = audit(g, v2, ev2)
    print("scope_contrast all-scan: components=%d V=%d U=%d delta=%+d error=%+d"
          % (ex2["components"], a2["V"], a2["U"], a2["d_pred3"], a2["err3"]))
    ok = (a["err3"] == 0 and a["U"] == 1 and a["R"] == 0 and a["O"] == 0
          and a["K"] == 0 and u_ref == 1 and miss == [(3, 2)]
          and a2["err3"] == 0 and a2["U"] == 0)
    return verdict(ok)


def exp_o():
    # 7x5: walkable border corridor, full 5x3 inner wall ("edge hugging wall").
    inner = [(x, y) for x in range(1, 6) for y in range(1, 4)]
    g = Grid(7, 5, inner, name="border-corridor")
    print_header("EXPERIMENT 2 / O term - walkable border hugging inner wall",
                 g, "main-region, seed=(0,0); mutant counts OOB probes")
    print(g.render())
    print("legend: .=border corridor walkable, inner rectangle all blocked")
    v, ev, _ = iut_oob_counted(g, 0, 0)
    a = audit(g, v, ev)
    print("W_walkable=%d  V_iut=%d" % (a["W"], a["V"]))
    print("terms: R=%d O=%d K=%d U=%d L=%d"
          % (a["R"], a["O"], a["K"], a["U"], a["L"]))
    print("oob_coordinates=%s" % fmt_coords(a["o_coords"], limit=30))
    print("oob_count_check perimeter_formula=2Wg+2Hg=%d"
          % (2 * g.w + 2 * g.h))
    print("delta_measured V-W=%+d" % a["d_meas"])
    print("delta_predicted R+O-U=%+d" % a["d_pred3"])
    print("formula_error=%+d" % a["err3"])
    ok = (a["err3"] == 0 and a["O"] == 2 * g.w + 2 * g.h
          and a["R"] == 0 and a["U"] == 0 and a["K"] == 0)
    return verdict(ok)


def exp_r():
    # 3x3 fully open: cycles make push-without-guard duplicate pops real.
    g = Grid(3, 3, (), name="open-3x3")
    print_header("EXPERIMENT 3 / R term - duplicate fill on cyclic grid",
                 g, "main-region, seed=(0,0); mark-on-pop / no push guard")
    print(g.render())
    v, ev, _ = iut_duplicate_pop(g, 0, 0)
    a = audit(g, v, ev)
    print("W_walkable=%d  V_iut=%d  raw_events=%d"
          % (a["W"], a["V"], a["events"]))
    print("terms: R=%d O=%d K=%d U=%d L=%d"
          % (a["R"], a["O"], a["K"], a["U"], a["L"]))
    print("repeat_fill_coordinates=%s" % fmt_coords(a["r_coords"]))
    print("delta_measured V-W=%+d" % a["d_meas"])
    print("delta_predicted R+O-U=%+d" % a["d_pred3"])
    print("formula_error=%+d" % a["err3"])
    ok = (a["err3"] == 0 and a["R"] > 0 and a["O"] == 0 and a["U"] == 0
          and a["K"] == 0)
    return verdict(ok)


# ---------------------------------------------------------------------------
# The 340 vs 352 reconciliation and the missing term
# ---------------------------------------------------------------------------

def mystery():
    # 22x16 = 352 cells; 12 sparse pillars -> W = 340 walkable exactly.
    pillars = [(x, y) for y in (2, 7, 12) for x in (3, 8, 13, 18)]
    g = Grid(22, 16, pillars, name="case-340-352-reconstruction")
    print_header("CASE 340 vs 352 - reverse solve and missing term", g,
                  "all-scan; mutant marks+counts BEFORE the blocked test")
    assert g.walkable_count() == 340
    v, ev, extra = iut_count_before_blocked(g)
    a = audit(g, v, ev)
    assert v == 352, v

    print("observed: W_walkable=%d  V_fill_count=%d  delta=%+d"
          % (a["W"], a["V"], a["d_meas"]))
    print("reverse solve of 3-term formula with current region.c properties:")
    print("  correct region.c => R=0 (comp guard), O=0 (bounds continue),")
    print("  all-scan seeds every walkable => U=0.")
    print("  R+O-U = 0, but measured delta = +12. 12 = -U would force U=-12,")
    print("  impossible: contradiction proves the formula misses a term OR")
    print("  the two scalars do not share one map snapshot/scope.")
    print("")
    print("3-term audit on the reconstructed run:")
    print("  terms R=%d O=%d U=%d" % (a["R"], a["O"], a["U"]))
    print("  delta_predicted=%+d delta_measured=%+d formula_error=%+d"
          % (a["d_pred3"], a["d_meas"], a["err3"]))
    if a["err3"] != 0:
        # The required diagnostic when prediction != measurement:
        k_row = sorted(a["k_coords"], key=lambda p: (p[1], p[0]))
        print("  -> mismatch; dumping over-counted coordinates:")
        print("     blocked-but-counted K=%d coords=%s"
              % (a["K"], fmt_coords(k_row, limit=12)))
        print("     repeat coords=%s" % fmt_coords(a["r_coords"]))
        print("     oob coords=%s" % fmt_coords(a["o_coords"]))
        miss = missing_walkable_coords(g, a["good_set"])
        print("     unvisited walkable coords=%s" % fmt_coords(miss))
    print("")
    print("extended formula delta = R+O+K-U+L:")
    print("  terms R=%d O=%d K=%d U=%d L=%d"
          % (a["R"], a["O"], a["K"], a["U"], a["L"]))
    print("  delta_predicted=%+d formula_error=%+d"
          % (a["d_pred4"], a["err4"]))
    print("")
    print("NOTE: the real map behind the reported scalars is not in this")
    print("repo, so its actual coordinates cannot be recovered from 340/352")
    print("alone. The 12 coordinates above are a worked reconstruction that")
    print("reproduces both numbers exactly; on the real map run this same")
    print("coordinate diff against an instrumented fill to obtain its list.")
    # Show the map strip carrying the pillars for visual confirmation.
    marks = {p: "X" for p in pillars}
    print(g.render(marks))
    coords_match = set(a["k_coords"]) == set(pillars)
    three_term_fails_as_predicted = (a["err3"] == -12)
    extended_closes = (a["d_meas"] == 12 and a["K"] == 12
                       and a["err4"] == 0 and coords_match)
    print("-" * 72)
    print("stage1_three_term_mismatch_expected=%s (error=%+d)"
          % ("yes" if three_term_fails_as_predicted else "no", a["err3"]))
    print("stage2_extended_formula_verdict=%s"
          % ("PASS formula_error=0" if extended_closes else "FAIL"))
    print("=" * 72)
    return 0 if (three_term_fails_as_predicted and extended_closes) else 1


def mirror_region_selftest():
    """Cross-check the Python mirror against region.c's own fixture maps."""
    failures = 0

    def expect(cond, msg):
        nonlocal failures
        print("%s=%d" % (msg, 1 if cond else 0))
        failures += 0 if cond else 1

    g = Grid(16, 12, (), "all-open")
    v, ev, ex = iut_all_scan(g)
    expect(v == 192 and ex["components"] == 1, "mirror_all_open_192_c1")
    a = audit(g, v, ev)
    expect(a["err4"] == 0 and a["U"] == 0, "mirror_all_open_identity")

    wall = [(8, y) for y in range(12)]
    g2 = Grid(16, 12, wall, "full-height-wall")
    v2, _, ex2 = iut_all_scan(g2)
    expect(v2 == 180 and ex2["components"] == 2, "mirror_wall_180_c2")

    g3 = Grid(16, 12, [(8, 5)], "pillar")
    v3, _, ex3 = iut_all_scan(g3)
    expect(v3 == 191 and ex3["components"] == 1, "mirror_pillar_191_c1")

    print("mirror_selftest_all_ok=%d" % (failures == 0))
    return 0 if failures == 0 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("exp-u")
    sub.add_parser("exp-o")
    sub.add_parser("exp-r")
    sub.add_parser("mystery")
    sub.add_parser("mirror-selftest")
    args = ap.parse_args(argv)
    if args.cmd == "exp-u":
        return exp_u()
    if args.cmd == "exp-o":
        return exp_o()
    if args.cmd == "exp-r":
        return exp_r()
    if args.cmd == "mystery":
        return mystery()
    if args.cmd == "mirror-selftest":
        return mirror_region_selftest()
    return 2


if __name__ == "__main__":
    sys.exit(main())
