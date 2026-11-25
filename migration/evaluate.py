"""Evaluate every snapshot scheme on every data tier.

Produces results/metrics.json, results/metrics_per_size.json and
results/inconsistencies.txt.  Exits non-zero if the acceptance gate fails:
  * per-tier undo success rate must be 1.0 at the chosen scheme (delta)
  * per-tier inconsistency rate must be 0.0 at the chosen scheme
  * per-tier mean snapshot bytes must be <= 0.30 * old record bytes
"""

from __future__ import annotations

import argparse
import hashlib
import json
import statistics
import sys
import time
from pathlib import Path

from . import dataset, schema, snapshot

CHOSEN = snapshot.DEFAULT_SCHEME
SIZE_BUDGET = 0.30
TIERS = ("small", "medium", "large")


def _run_tier(size: str, schemes: list[str]) -> dict:
    n_records = dataset.SIZES[size]
    per_scheme = {
        s: {"attempts": 0, "successes": 0, "snap_bytes": 0, "max_snap": 0,
            "stored_fields": 0, "failures": [], "field_counts": {},
            "decoder_mismatches": 0}
        for s in schemes
    }
    old_bytes = 0
    noncanonical = 0
    started = time.perf_counter()
    for i, rec in enumerate(dataset.iter_records(size)):
        old = schema.encode_old(rec)
        old_bytes += schema.OLD_SIZE
        if not schema.is_canonical(old):
            noncanonical += 1
        for s in schemes:
            mig = snapshot.migrate(rec, dataset.MIGRATION_EPOCH_US + i, s)
            got = snapshot.undo(mig.new, mig.snapshot)
            slot = per_scheme[s]
            slot["attempts"] += 1
            slot["snap_bytes"] += mig.snapshot_bytes
            slot["max_snap"] = max(slot["max_snap"], mig.snapshot_bytes)
            slot["stored_fields"] += len(mig.stored_fields)
            if got == old:
                slot["successes"] += 1
                try:
                    independent_ok = snapshot.undo_independent(mig.new, mig.snapshot) == old
                except ValueError:
                    independent_ok = False
                if not independent_ok:
                    slot["decoder_mismatches"] += 1
            else:
                diffs = schema.diff_report(old, got, limit=5)
                for f in diffs:
                    key = f["field"].split("[")[0]
                    slot["field_counts"][key] = slot["field_counts"].get(key, 0) + 1
                if len(slot["failures"]) < 5:
                    slot["failures"].append({
                        "record": i,
                        "scheme": s,
                        "fields": diffs,
                    })
    elapsed = time.perf_counter() - started

    report = {
        "size": size,
        "records": n_records,
        "old_bytes": old_bytes,
        "noncanonical_records": noncanonical,
        "noncanonical_rate": noncanonical / n_records,
        "elapsed_s": round(elapsed, 3),
        "schemes": {},
    }
    for s, slot in per_scheme.items():
        attempts = slot["attempts"]
        mean_snap = slot["snap_bytes"] / attempts
        report["schemes"][s] = {
            "records": attempts,
            "undo_success_rate": slot["successes"] / attempts,
            "inconsistency_rate": 1.0 - slot["successes"] / attempts,
            "mean_snapshot_bytes": mean_snap,
            "max_snapshot_bytes": slot["max_snap"],
            "snapshot_over_old": mean_snap / schema.OLD_SIZE,
            "mean_stored_fields": slot["stored_fields"] / attempts,
            "failure_field_counts": slot["field_counts"],
            "independent_decoder_mismatches": slot["decoder_mismatches"],
            "sample_failures": slot["failures"],
        }
    return report


def _gate(reports: dict[str, dict], require_all: bool = True) -> tuple[bool, list[str]]:
    problems: list[str] = []
    if require_all:
        missing = [t for t in TIERS if t not in reports]
        if missing:
            problems.append(f"missing tiers: {missing}")
    for size in reports:
        chosen = reports[size]["schemes"][CHOSEN]
        if chosen["undo_success_rate"] != 1.0:
            problems.append(
                f"{size}: {CHOSEN} undo_success_rate={chosen['undo_success_rate']:.6f} != 1")
        if chosen["inconsistency_rate"] != 0.0:
            problems.append(
                f"{size}: {CHOSEN} inconsistency_rate={chosen['inconsistency_rate']:.6f} != 0")
        if chosen["snapshot_over_old"] > SIZE_BUDGET:
            problems.append(
                f"{size}: {CHOSEN} snapshot_over_old={chosen['snapshot_over_old']:.4f} "
                f"> {SIZE_BUDGET}")
        if chosen["max_snapshot_bytes"] > SIZE_BUDGET * schema.OLD_SIZE:
            problems.append(
                f"{size}: {CHOSEN} worst-case snapshot {chosen['max_snapshot_bytes']}B "
                f"> {SIZE_BUDGET:.0%} of {schema.OLD_SIZE}B")
        if chosen["independent_decoder_mismatches"] != 0:
            problems.append(
                f"{size}: {CHOSEN} independent decoder disagreed on "
                f"{chosen['independent_decoder_mismatches']} records")
        stateless = reports[size]["schemes"]["stateless"]
        if stateless["undo_success_rate"] == 1.0:
            problems.append(f"{size}: stateless scheme passed; timestamps must vary")
    return (not problems), problems


def _write_inconsistencies(reports: dict[str, dict], path: Path) -> tuple[int, int]:
    """Write every field-level mismatch for the sampled failing records.

    Returns (sampled mismatching records, mismatch groups).
    """
    lines: list[str] = []
    records = 0
    groups = 0
    for size in reports:
        for scheme, data in reports[size]["schemes"].items():
            if data["sample_failures"]:
                lines.append(
                    f"# {size}/{scheme}: {len(data['sample_failures'])} sampled "
                    f"failing records, field_counts={data['failure_field_counts']}")
            for failure in data["sample_failures"]:
                records += 1
                lines.append(f"tier={size} scheme={scheme} record={failure['record']}")
                for f in failure["fields"]:
                    groups += 1
                    lines.append(
                        f"  {f['field']} off={f['offset']} size={f['size']} "
                        f"recovery={f['recovery']} "
                        f"old=0x{f['old_hex'].upper()} ({f['old_value']}) "
                        f"got=0x{f['new_hex'].upper()} ({f['new_value']})")
    path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return records, groups


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root", type=Path,
                        default=Path(__file__).resolve().parent.parent)
    parser.add_argument("--tiers", nargs="+", default=list(TIERS))
    parser.add_argument("--allow-partial", action="store_true",
                        help="gate only the tiers that were run (smoke tests)")
    args = parser.parse_args(argv)
    results = args.root / "results"
    results.mkdir(parents=True, exist_ok=True)

    reports = {}
    for size in args.tiers:
        reports[size] = _run_tier(size, list(snapshot.SCHEMES))
        r = reports[size]
        print(f"[{size}] records={r['records']} "
              f"noncanonical={r['noncanonical_rate']:.3f} "
              f"elapsed={r['elapsed_s']}s")
        for scheme, d in r["schemes"].items():
            print(f"  {scheme:10s} undo={d['undo_success_rate']:.6f} "
                  f"incons={d['inconsistency_rate']:.6f} "
                  f"snap={d['mean_snapshot_bytes']:.1f}B "
                  f"({d['snapshot_over_old']:.1%}) "
                  f"fields={d['mean_stored_fields']:.2f}")

    (results / "metrics_per_size.json").write_text(
        json.dumps(reports, indent=2), encoding="utf-8")
    n_incons, n_groups = _write_inconsistencies(reports, results / "inconsistencies.txt")

    full_gate = not args.allow_partial and tuple(args.tiers) == TIERS
    ok, problems = _gate(reports, require_all=full_gate)
    summary = {
        "chosen_scheme": CHOSEN,
        "size_budget": SIZE_BUDGET,
        "old_record_bytes": schema.OLD_SIZE,
        "total_fields": schema.TOTAL_FIELDS,
        "snapshot_wire_header_bytes": snapshot.SNAP_HEADER,
        "gate_scoped_to": list(reports),
        "gate_passed": ok,
        "gate_problems": problems,
        "inconsistency_sampled_records": n_incons,
        "inconsistency_sampled_field_groups": n_groups,
        "tiers": {
            size: {
                "records": reports[size]["records"],
                "chosen": reports[size]["schemes"][CHOSEN],
                "schemes": {s: {k: v for k, v in d.items() if k != "sample_failures"}
                            for s, d in reports[size]["schemes"].items() if s != CHOSEN},
            }
            for size in args.tiers
        },
        "digest_sha256": hashlib.sha256(
            json.dumps(reports, sort_keys=True).encode()).hexdigest(),
    }
    (results / "metrics.json").write_text(json.dumps(summary, indent=2), encoding="utf-8")
    if problems:
        print("GATE FAILED")
        for p in problems:
            print("  -", p)
        return 1
    print(f"GATE PASSED (scheme={CHOSEN}, budget={SIZE_BUDGET:.0%})")
    return 0


if __name__ == "__main__":
    sys.exit(main())
