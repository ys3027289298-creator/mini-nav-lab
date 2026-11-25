"""Cross-check migration/dataset.py's A* port against the repo's grid.c.

The probe compiles the same search algorithm (migration/tools/astar_probe.c)
and runs it on the exact maps the dataset generator uses.  If the Python port
drifted from the C semantics (tie-breaking, open-list handling, cap), the
paths would differ and this test fails.
"""

from __future__ import annotations

import random
import shutil
import subprocess
from pathlib import Path

import pytest

from migration import dataset, schema

ROOT = Path(__file__).resolve().parent.parent.parent
PROBE = ROOT / "migration" / "tools" / "astar_probe.c"
pytestmark = pytest.mark.skipif(shutil.which("gcc") is None, reason="gcc not available")


@pytest.fixture(scope="module")
def probe(tmp_path_factory) -> Path:
    out = tmp_path_factory.mktemp("probe") / "astar_probe"
    subprocess.run(["gcc", "-O2", "-std=c11", "-Wall", "-Wextra",
                    str(PROBE), "-o", str(out)], check=True)
    return out


def _cases(n: int = 60):
    rng = random.Random("cross-check")
    cases = []
    while len(cases) < n:
        blocked = [(rng.randrange(schema.W), rng.randrange(schema.H))
                   for _ in range(rng.choice((0, 12, 30, 60)))]
        sx, sy = rng.randrange(schema.W), rng.randrange(schema.H)
        gx, gy = rng.randrange(schema.W), rng.randrange(schema.H)
        cases.append((sx, sy, gx, gy, blocked))
    return cases


def test_python_port_matches_grid_c(probe: Path, tmp_path: Path):
    cases = _cases(80)
    lines = []
    for sx, sy, gx, gy, blocked in cases:
        cells = " ".join(f"{x},{y}" for x, y in blocked)
        lines.append(f"{sx} {sy} {gx} {gy} {len(blocked)} {cells}".rstrip())
    inp = tmp_path / "cases.txt"
    inp.write_text("\n".join(lines) + "\n", encoding="utf-8")
    got = subprocess.run([str(probe), str(inp)], check=True, text=True,
                         capture_output=True).stdout.splitlines()

    assert len(got) == len(cases)
    mismatches = 0
    for (sx, sy, gx, gy, blocked), c_line in zip(cases, got):
        grid = bytearray(schema.W * schema.H)
        for x, y in blocked:
            grid[y * schema.W + x] = 1
        path = dataset._astar(grid, sx, sy, gx, gy)
        if c_line == "NONE":
            py = None
        else:
            py = [int(v) for v in c_line[3:].split(",")]
        if py != path:
            mismatches += 1
            if mismatches <= 3:
                print(f"case {(sx, sy)}->{(gx, gy)} blocked={blocked}\n"
                      f"  C:  {path and py}\n  py: {path}")
    assert mismatches == 0, f"{mismatches}/{len(cases)} paths differ from grid.c"
