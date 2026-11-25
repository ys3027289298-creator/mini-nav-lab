#!/usr/bin/env python3
"""
Run the event-binding experiment and materialise artifacts/.

- builds ./event via make
- one canonical run drives every correctness/acceptance number
  (three smoothing strengths, miss lists, 100% interval consistency, GATE)
- repeats the binary a few times and takes the median for the noisy
  query-cost scaling curve only

Outputs (under <repo>/artifacts):
  event_lab_output.txt      raw canonical stdout
  experiment_results.json   structured summary
  loss_curve.csv            cap sweep: sigma vs curated/dense loss
  official_levels.csv       the three acceptance strengths
  overhead_curve.csv        median ns/query by E and lookup strategy
"""
from __future__ import annotations

import csv
import json
import re
import statistics
import subprocess
import sys
from pathlib import Path

EVENT_DIR = Path(__file__).resolve().parent
ROOT = EVENT_DIR.parent
ART = ROOT / "artifacts"
BIN = EVENT_DIR / "event"
COST_REPEATS = 5

KV_RE = re.compile(r"^([A-Z0-9_]+)\s+(.*)$")
CHECK_RE = re.compile(r"^([A-Za-z0-9_]+)=([01])$")


def build() -> None:
    subprocess.run(["make", "-C", str(EVENT_DIR), "clean", "all"],
                   check=True, capture_output=True, text=True)


def run_once() -> str:
    p = subprocess.run([str(BIN)], check=True, capture_output=True, text=True)
    return p.stdout


def parse(stdout: str) -> dict:
    lines = stdout.splitlines()
    out: dict = {"checks": {}, "info": [], "trig": [], "miss": [],
                 "csv_loss": [], "csv_fix": [], "csv_law": [],
                 "csv_cost": [], "gate": None, "all_ok": None}
    for ln in lines:
        if ln.startswith("all_ok="):
            out["all_ok"] = int(ln.split("=", 1)[1])
            continue
        if m := CHECK_RE.match(ln):
            out["checks"][m.group(1)] = int(m.group(2))
            continue
        if ln.startswith("INFO "):
            d = {"_kind": "INFO"}
            for tok in ln.split()[1:]:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    d[k] = _num(v)
            out["info"].append(d)
        elif ln.startswith("TRIG "):
            d = {"_kind": "TRIG"}
            for tok in ln.split()[1:]:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    d[k] = _num(v)
            out["trig"].append(d)
        elif ln.startswith("MISS "):
            d = {"_kind": "MISS"}
            for tok in ln.split()[1:]:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    d[k] = _num(v) if k in ("anchor", "x", "y", "cell") else v
            out["miss"].append(d)
        elif ln.startswith("GATE "):
            d = {}
            for tok in ln.split()[1:]:
                if "=" in tok:
                    k, v = tok.split("=", 1)
                    d[k] = _num(v)
            out["gate"] = d
        elif ln.startswith("CSV_LOSS,"):
            _, level, sigma, rho, fired, total = ln.split(",")
            out["csv_loss"].append(dict(level=level, sigma=float(sigma),
                                        loss_rate=float(rho),
                                        fired=int(fired), total=int(total)))
        elif ln.startswith("CSV_FIX,"):
            _, level, fired, total, consist = ln.split(",")
            out["csv_fix"].append(dict(level=level, fired=int(fired),
                                       total=int(total),
                                       consistency=float(consist)))
        elif ln.startswith("CSV_LAW,"):
            _, cap, kept, sigma, rho8 = ln.split(",")
            out["csv_law"].append(dict(cap=int(cap), kept=int(kept),
                                       sigma=float(sigma),
                                       curated_loss=float(rho8)))
        elif ln.startswith("CSV_COST,"):
            _, e, method, ns = ln.split(",")
            out["csv_cost"].append(dict(E=int(e), method=method,
                                        ns=float(ns)))
    return out


def _num(v: str):
    try:
        f = float(v)
        return int(f) if f.is_integer() else f
    except ValueError:
        return v


def write_csv(path: Path, rows: list[dict], fields: list[str]) -> None:
    with path.open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=fields)
        w.writeheader()
        for r in rows:
            w.writerow(r)


def main() -> int:
    ART.mkdir(exist_ok=True)
    build()

    canonical = run_once()
    (ART / "event_lab_output.txt").write_text(canonical)
    result = parse(canonical)

    # repeat for a stable query-cost curve (median per E/method)
    cost_runs: list[list[dict]] = [result["csv_cost"]]
    gate_overheads = [result["gate"]["median_overhead_pct"]]
    for _ in range(COST_REPEATS - 1):
        r = parse(run_once())
        cost_runs.append(r["csv_cost"])
        if r["gate"]:
            gate_overheads.append(r["gate"]["median_overhead_pct"])

    bucket: dict[tuple[int, str], list[float]] = {}
    for rows in cost_runs:
        for row in rows:
            bucket.setdefault((row["E"], row["method"]), []).append(row["ns"])
    cost_agg = []
    for (e, method), vals in sorted(bucket.items()):
        cost_agg.append(dict(E=e, method=method,
                             ns_median=round(statistics.median(vals), 4),
                             ns_min=round(min(vals), 4),
                             repeats=len(vals)))
    write_csv(ART / "overhead_curve.csv", cost_agg,
              ["E", "method", "ns_median", "ns_min", "repeats"])

    write_csv(ART / "loss_curve.csv", result["csv_law"],
              ["cap", "kept", "sigma", "curated_loss"])
    write_csv(ART / "official_levels.csv", result["csv_loss"],
              ["level", "sigma", "loss_rate", "fired", "total"])

    gate_ovh_median = round(statistics.median(gate_overheads), 3)
    summary = {
        "all_ok": result["all_ok"],
        "checks_total": len(result["checks"]),
        "checks_failed": [k for k, v in result["checks"].items() if v != 1],
        "official_levels": result["csv_loss"],
        "interval_fix": result["csv_fix"],
        "misses": result["miss"],
        "gate_canonical": result["gate"],
        "gate_median_overhead_pct_across_runs": gate_ovh_median,
        "gate_overhead_runs": gate_overheads,
    }
    (ART / "experiment_results.json").write_text(
        json.dumps(summary, ensure_ascii=False, indent=2))

    try:
        subprocess.run([sys.executable, str(EVENT_DIR / "make_charts.py")],
                       check=True, capture_output=True, text=True)
        charts = "charts=ok"
    except Exception as exc:  # charting is optional for the numeric verdict
        charts = f"charts=skipped ({exc})"

    print(f"all_ok={result['all_ok']} checks={len(result['checks'])} "
          f"failed={summary['checks_failed']} {charts}")
    for row in result["csv_loss"]:
        print(f"  {row['level']}: sigma={row['sigma']:.3f} "
              f"index_fired={row['fired']}/{row['total']} "
              f"loss={row['loss_rate']:.3f}")
    for row in result["csv_fix"]:
        print(f"  {row['level']}: interval_fired={row['fired']}/"
              f"{row['total']} consistency={row['consistency']:.3f}")
    g = result["gate"]
    print(f"  GATE best_ratio={g['best_ratio']:.4f} "
          f"median_overhead={g['median_overhead_pct']:.2f}% "
          f"pass={g['pass']}; across-run median={gate_ovh_median}%")
    return 0 if result["all_ok"] == 1 and not summary["checks_failed"] else 1


if __name__ == "__main__":
    sys.exit(main())
