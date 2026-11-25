# Reversible migration: NAV0 → NAV1

`migration/` migrates the path record produced by `grid/src/grid.c` from the
flat-index layout (**NAV0**) to the coordinate layout (**NAV1**), keeps an
incremental snapshot, and undoes the migration byte-for-byte.

Everything here is validated by a run that can be repeated:

```bash
bash docs/reproduce.sh                 # the whole sequence below, in order
python3 -m migration.evaluate          # three tiers, acceptance gate
python3 -m migration.sweep             # reliability vs size curves
python3 docs/make_figs.py              # results/fig/tradeoff.png
python3 -m pytest migration/tests -q   # 154 tests
```

## 1. What is being migrated

Both layouts are **228 bytes**; the migration is a re-interpretation of the
same bytes, not a resize. The full field table lives in
`migration/schema.py`; in short:

| NAV0 (old) | NAV1 (new) |
|---|---|
| `magic`/`version` | `NAV0`/1 → `NAV1`/2 |
| `flags` (u16) | same flags, `COORD_FORMAT` bit set |
| `path_len` (u32) | `point_count` (u32) |
| `path[48]` flat index `y*16+x` | `points[48]` of `(u16 x, u16 y)` |
| `start_idx`, `goal_idx` (u32) | `start_x/y`, `goal_x/y` (u16 each) |
| `cost` (u32) | `cost` (u32) |
| `created_at` (u64 µs) | **no counterpart**; `updated_at` is written by the migration |
| `checksum` (crc32 over `[0,224)`) | same |

**57 fields** in total: 9 scalars + 48 path slots. The path slots are counted
individually because that is the granularity at which the snapshot decides what
to store.

Because every logical old field except `created_at` has a documented inverse,
`old_from_new` (the stateless inverse) needs no snapshot to rebuild the record
*except* for that one field. This is the whole basis of the size saving.

## 2. The property, as a testable criterion

> **Undo criterion.** For every old record `o` and migration timestamp `t`,
> with `n = migrate(o, t)` and `s = snapshot(o, t)`:
>
> `undo(n.new, n.snapshot) == o` — byte-for-byte, all 228 bytes.

Written as an assertion (`migration/tests/test_migration.py`):

```python
for rec in records:                       # every tier, every record
    mig = migrate(rec, t, "delta")
    assert undo(mig.new, mig.snapshot) == mig.old
```

Three things make this evidence rather than a restatement:

1. **A second decoder.** `snapshot.undo_independent()` rebuilds NAV0 from its
   own offset table, reading each point as one little-endian `u32` rather than
   two `u16`s. It is compared against `undo()` on every record in the
   acceptance run; **0 disagreements**. Two independent transcriptions of the
   layout that agree rule out "both share a misreading".
2. **A negative control.** `stateless` scores 0% and `predicted` ~91% on the
   same data (below), so the test can fail.
3. **A per-field diff on failure.** When the bytes differ the harness emits the
   field name, offset, and both values (`results/inconsistencies.txt`), so a
   failure names what broke.

### Which fields cannot satisfy the stateless form of it

| Recovery | Fields | Why |
|---|---|---|
| `const` | `magic`, `version` | Both sides are known constants. |
| `inverse` | `flags`, `path_len`, 48 × `path[i]`, `start_idx`, `goal_idx` | Index → coordinate conversion, documented and total. |
| `recompute` | `checksum` | Deterministic function of `[0,224)`; recomputed after patching. |
| **`store`** | **`created_at`** | **NAV1 has no slot for it.** The migration writes `updated_at` instead. This is the single field the snapshot must carry. |

Note the two ways a field can fail to satisfy the property:

- **No image.** `created_at` has no representation in NAV1 at all. No amount of
  clever decoding recovers it; it *must* be stored.
- **Lossy image.** A non-canonical old record loses information on the way in:
  garbage in a path slot past `path_len` (NAV1 stores by `point_count`), the
  `COORD_FORMAT` flag that the migration normalises, an index ≥ 256 (y
  truncates to u16), or `path_len > 48`. These are not schema bugs — they are
  inputs outside the format's contract, and the delta scheme copes with them
  without a special case.

The generated data deliberately contains **10.0%** such records
(`noncanonical_rate = 0.10016` at the large tier; composition: flags 2.05%,
padding garbage 5.72%, `path_len` 1.01%, out-of-range start/goal 0.68%, path
slot out of range 0.32%; one record can carry more than one). Without them the
"incremental snapshot is not always enough" result would be untested.

## 3. Snapshot schemes

| Scheme | Stores | Exact when |
|---|---|---|
| `stateless` | nothing (9-byte header) | never in practice — `created_at` is lost |
| `predicted` | the fields tagged `store` (`created_at`) | the stateless inverse misses nothing else |
| **`delta`** | **only the fields whose old bytes the inverse actually misses** (`created_at` always, `checksum` always, plus any field corrupted out of the format) | **always, by construction** |
| `full` | the whole 228-byte record | always |

`delta` measures `changed_fields(o, old_from_new(new_from_old(o)))` at
migration time and stores exactly those old field bytes. Its snapshot is
`9 + 1 + Σ(1 + field_size)` bytes; the mean is **24.4 B ≈ 10.7% of the
record**, and the worst case observed is **29 B**.

Why `predicted` is not exact: the schema reads *"store `created_at`"* and a
developer stops there. It also fails on the 10% of records the inverse cannot
round-trip, and — more subtly — an over-long `path_len` round-trips through the
u32 `point_count`, so the exactness condition is *"the inverse misses only
`created_at`/`checksum`"*, which is a strict superset of canonical. The test
`test_predicted_exact_iff_only_timestamp_unrecoverable` pins that boundary.

## 4. Trade-off: reliability vs volume

`results/fig/tradeoff.png`, from `results/curve.json`. Per-record migration is
exact at ~2.09 stored fields (10.7% volume). A **fixed** field set is exact only
near the top of the field list: **51–53 fields** in empirical order, **56** in
layout order — 117–127% volume, larger than the record it protects.

The two orderings show why "store the fields you predict" does not work. The
sweep's `predicted` curve is the **struct layout order** — the natural guess if
you read the record definition top to bottom.

- **layout order** — fields in the order they appear in the struct (magic,
  version, flags, path_len, path slots, …, checksum). It sits flat at **0.00%**
  through 55 fields, then jumps to 100% at **56**. The jump is structural: with
  56 fields stored the only payload field left out is `checksum` itself, which
  the snapshot recomputes after patching and which therefore lands correct. A
  developer who lists fields top-to-bottom from the header lands here — 0% on
  every partial set, then one exact point above the record's own size.
- **empirical order** — most-frequently-breaking field first (`flags`, then
  the path slots, then `start_idx`/`goal_idx`, checksum last). It rises
  smoothly to 99.97% by 52 fields and first reaches 100% somewhere between
  **51 and 53** depending on sample size — a threshold set by the rarest
  damage class present, not by the data volume. Its climb plateaus because the
  damage is spread over 48 path slots; no partial set catches the tail.

Measured means, all three tiers (they agree within **0.73 pp**, drawn once as
the `large` tier):

| scheme | undo success | mean snapshot | % of record | exact? |
|---|---|---|---|---|
| stateless | 0.000000 | 10.0 B | 4.4% | no |
| predicted | 0.909886 | 19.0 B | 8.3% | no |
| **delta (chosen)** | **1.000000** | **24.4 B** | **10.7%** | **yes** |
| full | 1.000000 | 237.0 B | 103.9% | yes |

`full` is 103.9% because of the 9-byte wire header on a 228-byte body.

**Decision.** Take `delta`. It is the only scheme that is both exact on 100% of
the data and under the 30% budget (max observed 29 B = 12.7%). `full` is exact
but over budget; `predicted` is small but not exact; `stateless` is neither. The
trade-off curve's shape is the argument: for a fixed field set reliability is a
**cliff near the end of the field list**, but for a per-record set it is a
**point at ~2 fields**, because the set only has to cover what *this* record
actually lost.

## 5. Acceptance results

`python3 -m migration.evaluate` — `results/metrics.json`,
`results/metrics_per_size.json`, `results/inconsistencies.txt`.

| tier | records | undo success | inconsistency rate | mean snapshot | max snapshot | over record |
|---|---|---|---|---|---|---|
| small | 5,000 | 1.000000 | 0.0 | 24.40 B | 29 B | 10.70% |
| medium | 50,000 | 1.000000 | 0.0 | 24.42 B | 29 B | 10.71% |
| large | 500,000 | 1.000000 | 0.0 | 24.41 B | 29 B | 10.71% |

Gate checks (all enforced by `evaluate.py`, exit code 1 on failure):

- undo success = 1.000000 on every tier ✔
- inconsistency rate = 0.0 on every tier ✔
- mean **and** max snapshot ≤ 30% of 228 B (68.4 B) ✔ (worst 29 B)
- independent decoder agrees on every record ✔ (0 mismatches)
- `stateless` must *not* score 1.0, or the test is vacuous ✔

If a byte comparison ever fails, `inconsistencies.txt` lists field name,
offset, size, recovery class, and both hex/decimal values for the first five
failing records of each scheme — e.g. from a `stateless` failure:

```
tier=small scheme=stateless record=0
  created_at off=216 size=8 recovery=store old=0x0020C10A03650600 (1799913600000000) got=0x0080982817650600 (1800000000000000)
  checksum   off=224 size=4 recovery=recompute old=0x8B6BB6B7 (3082185611) got=0x765DE6BF (3219545462)
```

## 6. Limits worth stating

- **Timestamps are unrecoverable by decoding alone.** `created_at` is the one
  field with no image; any scheme that does not store it fails. That is a
  property of NAV1, not of the harness.
- **Delta assumes the checksum is trustworthy at undo time.** It is recomputed
  from the patched payload unless it was itself stored; both paths are tested.
- **The delta's size depends on the damage rate, not the data volume.** 10.7%
  here tracks the 10% non-canonical rate; a fully canonical dataset would store
  only `created_at` + `checksum` (≈ 19 B, 8.3%) and still be exact.
- **`path_len > 48` and indices ≥ 256 are lossy in NAV1 by width, not by
  design.** They cost one stored path field each in the delta. If NAV1 were
  ever widened these would leave the delta too — the criterion, not this field
  list, is the contract.
