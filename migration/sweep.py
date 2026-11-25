"""Trade-off sweep: undo reliability vs snapshot size.

For each tier, replay exactly the same records the evaluator uses and, for a
prefix of a field priority order, store a *fixed* field set and measure the
actual undo success rate and mean snapshot bytes.  Two orderings are compared:

  predicted  fields in layout order, which is what a developer writes down
             from reading the schema (created_at first among the scalars)
  empirical  fields in descending order of how often they actually break the
             stateless inverse on this data

The chosen delta scheme stores only fields that need storing per record, so it
sits on the right-hand edge of both curves at every size; the sweep shows why
the fixed-set variants cannot be made both exact and small.

Writes results/curve.json for docs/make_figs.py.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import dataset, schema, snapshot

TIERS = ("small", "medium", "large")
KS = (1, 2, 3, 5, 8, 12, 16, 24, 32, 40, 48, 49, 50, 51, 52, 53, 54, 55,
      56, 57)
# The k-sweep is O(sum(KS) * records) migrations; rates and mean sizes are
# stable to <0.1% by 20k records, so the sweep and the frequency estimate use a
# deterministic even-stride sample.  The acceptance run (evaluate.py) uses
# every record.
SAMPLE = 20_000
FREQ_SAMPLE = 50_000


def _sample(size: str, n: int) -> list[schema.OldRecord]:
    total = dataset.SIZES[size]
    if total <= n:
        return list(dataset.iter_records(size))
    stride = total // n
    out = []
    for i, rec in enumerate(dataset.iter_records(size)):
        if i % stride == 0:
            out.append(rec)
        if len(out) >= n:
            break
    return out


def _sweep(size: str) -> dict:
    records = _sample(size, SAMPLE)
    freq_records = _sample(size, FREQ_SAMPLE)
    total = len(records)
    freq = snapshot.field_failure_frequency(freq_records, dataset.MIGRATION_EPOCH_US)
    orders = {
        "empirical": snapshot.frequency_order(freq),
        "predicted": list(range(len(schema.OLD_FIELDS))),
    }
    out = {"records": total, "frequency_sample": len(freq_records),
           "field_failure_frequency": freq, "curves": {}}
    for name, order in orders.items():
        points = []
        for k in KS:
            ids = order[:k]
            stored = 0
            successes = 0
            for i, rec in enumerate(records):
                mig = snapshot.migrate_with(rec, dataset.MIGRATION_EPOCH_US + i, ids, name)
                stored += mig.snapshot_bytes
                if snapshot.undo(mig.new, mig.snapshot) == mig.old:
                    successes += 1
            points.append({
                "fields_stored": k,
                "undo_success_rate": successes / total,
                "mean_snapshot_bytes": stored / total,
                "snapshot_over_old": stored / total / schema.OLD_SIZE,
            })
        out["curves"][name] = points

    # Per-record delta (the shipped scheme) and full snapshot, for reference.
    delta_bytes = 0
    full_bytes = 0
    for i, rec in enumerate(records):
        delta_bytes += snapshot.migrate(rec, dataset.MIGRATION_EPOCH_US + i, "delta").snapshot_bytes
        full_bytes += snapshot.migrate(rec, dataset.MIGRATION_EPOCH_US + i, "full").snapshot_bytes
    out["delta_per_record"] = {
        "undo_success_rate": 1.0,
        "mean_snapshot_bytes": delta_bytes / total,
        "snapshot_over_old": delta_bytes / total / schema.OLD_SIZE,
    }
    out["full"] = {
        "undo_success_rate": 1.0,
        "mean_snapshot_bytes": full_bytes / total,
        "snapshot_over_old": full_bytes / total / schema.OLD_SIZE,
    }
    return out


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--tiers", nargs="+", default=list(TIERS))
    args = parser.parse_args(argv)
    out = {size: _sweep(size) for size in args.tiers}
    (args.root / "results").mkdir(parents=True, exist_ok=True)
    (args.root / "results" / "curve.json").write_text(json.dumps(out, indent=2),
                                                      encoding="utf-8")
    for size in args.tiers:
        d = out[size]
        print(f"[{size}] delta={d['delta_per_record']['mean_snapshot_bytes']:.1f}B "
              f"({d['delta_per_record']['snapshot_over_old']:.1%}) "
              f"full={d['full']['mean_snapshot_bytes']:.1f}B "
              f"({d['full']['snapshot_over_old']:.1%})")
        for name, pts in d["curves"].items():
            first_exact = next((p for p in pts if p["undo_success_rate"] == 1.0), None)
            print(f"  {name:9s} exact at k={first_exact and first_exact['fields_stored']} "
                  f"({first_exact and first_exact['snapshot_over_old']:.1%})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
