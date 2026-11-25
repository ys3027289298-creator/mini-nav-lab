#!/usr/bin/env bash
# Reproduce every result in docs/REVERSIBILITY.md from a clean checkout.
set -euo pipefail
cd "$(dirname "$0")/.."

export PYTHONPATH="$PWD${PYTHONPATH:+:$PYTHONPATH}"

echo "== 1/5 unit tests =="
python3 -m pytest migration/tests -q

echo "== 2/5 acceptance run (three tiers, gate) =="
python3 -m migration.evaluate

echo "== 3/5 trade-off sweep =="
python3 -m migration.sweep

echo "== 4/5 figure =="
python3 docs/make_figs.py

echo "== 5/5 criterion via CLI =="
python3 -m migration.cli check --tier large --limit 200000

echo
echo "artefacts: results/metrics.json results/metrics_per_size.json"
echo "           results/curve.json results/inconsistencies.txt"
echo "           results/fig/tradeoff.png"
