# migration

Reversible migration of the `grid/` path record from the flat-index layout
(NAV0) to the coordinate layout (NAV1), with incremental snapshots.

Design, the undo criterion, the field-by-field recovery table, and the
trade-off analysis: **[`../docs/REVERSIBILITY.md`](../docs/REVERSIBILITY.md)**.

```bash
python3 -m migration.evaluate                 # three tiers + acceptance gate
python3 -m migration.sweep                    # reliability/size curves
python3 -m migration.cli check --tier large --limit 200000
python3 -m pytest migration/tests -q
../docs/reproduce.sh                          # all of the above, in order
```

| file | role |
|---|---|
| `schema.py` | 228-byte field table, encode/decode, forward + stateless inverse |
| `snapshot.py` | four snapshot schemes, undo, independent cross-decoder |
| `dataset.py` | deterministic records at 5k / 50k / 500k |
| `evaluate.py` | acceptance gate over all three tiers |
| `sweep.py` | fixed-field-set reliability vs size curves |
| `cli.py` | `check` / `migrate` / `undo` |
| `tools/astar_probe.c` | runs the repo's A* to anchor the Python port |

`results/` holds the generated artefacts; `results/metrics.json` is the
machine-readable summary.
