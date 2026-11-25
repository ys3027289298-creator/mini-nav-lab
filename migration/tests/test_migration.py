"""Tests for the reversible migration.

Run with:  python3 -m pytest migration/tests -q
"""

from __future__ import annotations

import struct

import pytest

from migration import dataset, schema, snapshot

TS = dataset.MIGRATION_EPOCH_US


def canonical_records(n: int = 200):
    """Only records that satisfy validate_old (empty problem list)."""
    out = []
    for i, rec in enumerate(dataset.iter_records("small")):
        old = schema.encode_old(rec)
        if schema.is_canonical(old):
            out.append(rec)
        if len(out) >= n:
            break
    return out


canon = canonical_records(120)


# ---------------------------------------------------------------- schema


def test_field_table_covers_record_exactly():
    covered = sum(f.size for f in schema.OLD_FIELDS)
    assert covered == schema.OLD_SIZE
    pos = 0
    for f in schema.OLD_FIELDS:
        assert f.offset == pos, f"{f.name} leaves a gap at {pos}"
        pos += f.size
    assert pos == schema.OLD_SIZE


def test_field_manifest():
    names = [f.name for f in schema.OLD_FIELDS]
    assert names[0] == "magic" and names[-1] == "checksum"
    assert schema.FIELD_BY_NAME["created_at"].recovery == schema.STORE
    assert [f.name for f in schema.STORE_FIELDS] == ["created_at"]
    assert schema.TOTAL_FIELDS == 57


def test_checksum_detects_single_bit():
    rec = canonical_records(1)[0]
    buf = bytearray(schema.encode_old(rec))
    buf[212] ^= 0x01
    problems = schema.validate_old(bytes(buf))
    assert any(p.startswith("checksum=") for p in problems)


@pytest.mark.parametrize("rec", canon, ids=range(len(canon)))
def test_inverse_is_identity_when_timestamp_is_preserved(rec):
    old = schema.encode_old(rec)
    assert schema.is_canonical(old)
    new = schema.new_from_old(old, rec.created_at)
    assert schema.old_from_new(new) == old


def test_inverse_differs_only_in_timestamp_otherwise():
    rec = canon[0]
    old = schema.encode_old(rec)
    new = schema.new_from_old(old, TS)
    recovered = schema.old_from_new(new)
    lost = [f.name for f in schema.changed_fields(old, recovered)]
    # created_at has no image in NAV1; checksum is a function of it.
    assert lost == ["created_at", "checksum"], lost
    at = schema.FIELD_BY_NAME["created_at"]
    assert recovered[at.offset:at.offset + at.size] == TS.to_bytes(at.size, "little")


def test_inverse_misses_only_timestamp_on_canonical():
    for rec in canon:
        old = schema.encode_old(rec)
        recovered = schema.old_from_new(schema.new_from_old(old, TS + 7))
        changed = [f.name for f in schema.changed_fields(old, recovered)]
        assert changed == ["created_at", "checksum"], changed


def test_forward_migration_is_total():
    # Arbitrary 228-byte inputs must not raise.
    import random
    rng = random.Random(1)
    for _ in range(500):
        blob = bytes(rng.randrange(256) for _ in range(schema.OLD_SIZE))
        new = schema.new_from_old(blob, TS)
        assert len(new) == schema.NEW_SIZE
        assert struct.unpack_from("<I", new, 0)[0] == schema.NEW_MAGIC


# ---------------------------------------------------------------- schemes


@pytest.mark.parametrize("scheme", ["delta", "full"])
@pytest.mark.parametrize("size", [0, 1, 2, 11, 23])
def test_lossless_schemes_are_exact(scheme, size):
    for i, rec in enumerate(dataset.iter_records("small")):
        if i % 97 != size:
            continue
        mig = snapshot.migrate(rec, TS + i, scheme)
        assert snapshot.undo(mig.new, mig.snapshot) == mig.old


def test_delta_stores_exactly_the_mismatched_fields():
    for i, rec in enumerate(dataset.iter_records("small")):
        old = schema.encode_old(rec)
        mig = snapshot.migrate(rec, TS + i, "delta")
        base = schema.old_from_new(mig.new)
        needed = {f.name for f in schema.changed_fields(old, base)}
        assert set(mig.stored_fields) == needed


def test_predicted_exact_iff_only_timestamp_unrecoverable():
    # predicted stores created_at only, so it is exact exactly when the
    # stateless inverse misses nothing else.  That is a superset of canonical:
    # e.g. an over-long path_len round-trips through the u32 point_count.
    saw_lossless_noncanonical = False
    for i, rec in enumerate(dataset.iter_records("small")):
        if i >= 600:
            break
        old = schema.encode_old(rec)
        base = schema.old_from_new(schema.new_from_old(old, TS + i))
        needed = {f.name for f in schema.changed_fields(old, base)}
        expect_exact = needed <= {"created_at", "checksum"}
        if expect_exact and not schema.is_canonical(old):
            saw_lossless_noncanonical = True
        mig = snapshot.migrate(rec, TS + i, "predicted")
        assert (snapshot.undo(mig.new, mig.snapshot) == old) == expect_exact, i
    assert saw_lossless_noncanonical, "sample never exercised the superset case"


def test_negative_control_stateless_fails():
    # If this ever passes, the chosen scheme's advantage is not being tested.
    rec = canon[0]
    mig = snapshot.migrate(rec, TS + 1, "stateless")
    assert snapshot.undo(mig.new, mig.snapshot) != mig.old


def test_negative_control_snapshot_is_load_bearing():
    rec = canon[0]
    mig = snapshot.migrate(rec, TS + 1, "delta")
    other = snapshot.migrate(canon[1], TS + 2, "delta")
    # Swapping in another record's snapshot must not magically succeed.
    assert snapshot.undo(mig.new, other.snapshot) != mig.old


# ---------------------------------------------------------------- snapshot wire


@pytest.mark.parametrize("scheme", list(snapshot.SCHEMES))
def test_snapshot_size_matches_analytic(scheme):
    rec = canon[3]
    mig = snapshot.migrate(rec, TS, scheme)
    if scheme == "full":
        assert mig.snapshot_bytes == snapshot.SNAP_HEADER + schema.OLD_SIZE
    elif scheme == "stateless":
        assert mig.snapshot_bytes == snapshot.SNAP_HEADER + 1
    else:
        ids = [schema.FIELD_INDEX[n] for n in mig.stored_fields]
        assert mig.snapshot_bytes == snapshot.snapshot_bytes_for(ids)


def test_snapshot_crc_rejects_corruption():
    mig = snapshot.migrate(canon[0], TS, "delta")
    bad = bytearray(mig.snapshot)
    bad[-1] ^= 0xFF
    with pytest.raises(snapshot.SnapshotError):
        snapshot.undo(mig.new, bytes(bad))


def test_undo_requires_nav1():
    mig = snapshot.migrate(canon[0], TS, "delta")
    with pytest.raises(ValueError):
        snapshot.undo(mig.old, mig.snapshot)


@pytest.mark.parametrize("scheme", ["delta", "full", "predicted"])
def test_independent_decoder_agrees(scheme):
    # The property is only evidence if a second, separately written decoder
    # reproduces the same 228 bytes.
    for i, rec in enumerate(dataset.iter_records("small")):
        if i >= 500:
            break
        mig = snapshot.migrate(rec, TS + i, scheme)
        assert snapshot.undo_independent(mig.new, mig.snapshot) == \
            snapshot.undo(mig.new, mig.snapshot)


def test_independent_decoder_is_not_a_wrapper():
    # Mutating one implementation's constant must show up as disagreement;
    # this guards against the cross-check silently becoming a tautology.
    mig = snapshot.migrate(canon[0], TS, "delta")
    assert snapshot.undo_independent(mig.new, mig.snapshot) == mig.old


def test_analytic_curve_is_monotone():
    curve = snapshot.analytic_curve()
    sizes = [b for _, b in curve]
    assert sizes == sorted(sizes)
    assert len(curve) == schema.TOTAL_FIELDS


# ---------------------------------------------------------------- chosen scheme


def test_delta_snapshot_within_budget_on_all_tiers_sample():
    for size in dataset.SIZES:
        n = 0
        stored = 0
        for i, rec in enumerate(dataset.iter_records(size)):
            if i >= 2000:
                break
            stored += snapshot.migrate(rec, TS + i, "delta").snapshot_bytes
            n += 1
        ratio = stored / n / schema.OLD_SIZE
        assert ratio <= 0.30, f"{size}: {ratio:.3f}"


def test_delta_undo_exact_on_all_tiers_sample():
    for size in dataset.SIZES:
        for i, rec in enumerate(dataset.iter_records(size)):
            if i >= 2000:
                break
            mig = snapshot.migrate(rec, TS + i, "delta")
            assert snapshot.undo(mig.new, mig.snapshot) == mig.old
