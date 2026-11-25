"""Reversible migration NAV0 -> NAV1 with incremental snapshots.

Snapshot wire format (all integers little endian)::

    magic  4  b"SNP1"
    scheme 1  0 = delta, 1 = full
    crc32  4  crc32 of everything after this header
    body      delta: u8 count, then count * (u8 field_id, value[f.size])
              full:  the 228 old bytes

A delta body stores, for a chosen field set, the *old* bytes.  Undo applies
``old_from_new`` first, then overwrites those fields.  The checksum field is
special: it is patched last when stored, otherwise recomputed after patching,
because it is a function of the payload.

Schemes differ only in which fields they choose to store:

``stateless``  no snapshot at all.
``predicted``  fields whose documented recovery tag is STORE (created_at).
``delta``      fields whose old bytes the stateless inverse actually misses,
               measured against the record that was produced.  This includes
               the checksum when, and only when, it too differs.
``full``       the whole record.

``delta`` is exact for every 228-byte input by construction; ``predicted`` is
exact for canonical records only; ``stateless`` is never exact when the
timestamps differ.  See docs/REVERSIBILITY.md.
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass

from . import schema

SNAP_MAGIC = b"SNP1"
SNAP_HEADER = 9
SNAP_SCHEME_DELTA = 0
SNAP_SCHEME_FULL = 1
SCHEMES = ("stateless", "predicted", "delta", "full")
DEFAULT_SCHEME = "delta"


class SnapshotError(ValueError):
    pass


@dataclass(frozen=True)
class Migration:
    old: bytes
    new: bytes
    snapshot: bytes
    stored_fields: tuple[str, ...]
    scheme: str

    @property
    def snapshot_bytes(self) -> int:
        return len(self.snapshot)


def _snapshot_body(fields: dict[int, bytes]) -> bytes:
    if len(fields) > 0xFF:
        raise SnapshotError(f"too many stored fields: {len(fields)}")
    parts = [bytes([len(fields)])]
    for fid in sorted(fields):
        size = schema.OLD_FIELDS[fid].size
        value = fields[fid]
        if len(value) != size:
            raise SnapshotError(f"{schema.OLD_FIELDS[fid].name}: {len(value)} != {size}")
        parts.append(bytes([fid]))
        parts.append(value)
    return b"".join(parts)


def _pack(scheme_id: int, body: bytes) -> bytes:
    head = SNAP_MAGIC + bytes([scheme_id])
    return head + struct.pack("<I", zlib.crc32(body) & 0xFFFFFFFF) + body


def _unpack(snapshot: bytes) -> tuple[int, bytes]:
    if len(snapshot) < SNAP_HEADER:
        raise SnapshotError(f"snapshot too short: {len(snapshot)}")
    if snapshot[:4] != SNAP_MAGIC:
        raise SnapshotError(f"bad snapshot magic: {snapshot[:4]!r}")
    scheme_id = snapshot[4]
    (want,) = struct.unpack_from("<I", snapshot, 5)
    body = snapshot[SNAP_HEADER:]
    got = zlib.crc32(body) & 0xFFFFFFFF
    if want != got:
        raise SnapshotError(f"snapshot crc mismatch: 0x{want:08X} != 0x{got:08X}")
    return scheme_id, body


def _parse_delta(body: bytes) -> dict[int, bytes]:
    if not body:
        raise SnapshotError("empty delta body")
    count = body[0]
    pos = 1
    out: dict[int, bytes] = {}
    for _ in range(count):
        if pos >= len(body):
            raise SnapshotError("truncated delta entry")
        fid = body[pos]
        pos += 1
        if fid >= len(schema.OLD_FIELDS):
            raise SnapshotError(f"delta field id {fid} out of range")
        size = schema.OLD_FIELDS[fid].size
        if pos + size > len(body):
            raise SnapshotError("truncated delta value")
        out[fid] = body[pos:pos + size]
        pos += size
    if pos != len(body):
        raise SnapshotError("trailing bytes in delta body")
    return out


def stored_field_ids(rec: schema.OldRecord, recovered: bytes, scheme: str) -> dict[int, bytes]:
    """The field set a scheme chooses to store, with the old bytes to store."""
    old = schema.encode_old(rec)
    if scheme == "stateless":
        return {}
    if scheme == "full":
        # Full snapshot is expressed as every field; the writer collapses it.
        return {i: schema.field_bytes(old, f) for i, f in enumerate(schema.OLD_FIELDS)}
    if scheme == "predicted":
        ids = {schema.FIELD_INDEX[f.name] for f in schema.STORE_FIELDS}
        return {i: schema.field_bytes(old, schema.OLD_FIELDS[i]) for i in ids}
    if scheme == "delta":
        diff = schema.changed_fields(old, recovered)
        return {schema.FIELD_INDEX[f.name]: schema.field_bytes(old, f) for f in diff}
    raise SnapshotError(f"unknown scheme: {scheme}")


def migrate(rec: schema.OldRecord, updated_at: int, scheme: str = DEFAULT_SCHEME) -> Migration:
    """Forward migration plus the snapshot needed to undo it exactly."""
    old = schema.encode_old(rec)
    new = schema.new_from_old(old, updated_at)
    recovered = schema.old_from_new(new)
    fields = stored_field_ids(rec, recovered, scheme)
    if scheme == "full":
        snapshot = _pack(SNAP_SCHEME_FULL, old)
        stored = tuple(f.name for f in schema.OLD_FIELDS)
    else:
        snapshot = _pack(SNAP_SCHEME_DELTA, _snapshot_body(fields))
        stored = tuple(schema.OLD_FIELDS[i].name for i in sorted(fields))
    return Migration(old, new, snapshot, stored, scheme)


def migrate_with(rec: schema.OldRecord, updated_at: int, field_ids, label: str) -> Migration:
    """Migration with an explicit stored-field set, for the size/success sweep."""
    old = schema.encode_old(rec)
    new = schema.new_from_old(old, updated_at)
    ids = sorted(set(int(i) for i in field_ids))
    fields = {i: schema.field_bytes(old, schema.OLD_FIELDS[i]) for i in ids}
    snapshot = _pack(SNAP_SCHEME_DELTA, _snapshot_body(fields))
    return Migration(old, new, snapshot,
                     tuple(schema.OLD_FIELDS[i].name for i in ids), label)


def field_failure_frequency(records, updated_at_base: int) -> dict[str, int]:
    """How often the stateless inverse misses each field, over a record set."""
    counts = {f.name: 0 for f in schema.OLD_FIELDS}
    for i, rec in enumerate(records):
        old = schema.encode_old(rec)
        new = schema.new_from_old(old, updated_at_base + i)
        for f in schema.changed_fields(old, schema.old_from_new(new)):
            counts[f.name] += 1
    return counts


def frequency_order(counts: dict[str, int]) -> list[int]:
    """Field ids ordered by descending failure frequency, then by layout order."""
    return sorted(range(len(schema.OLD_FIELDS)),
                  key=lambda i: (-counts[schema.OLD_FIELDS[i].name], i))


def undo(new: bytes, snapshot: bytes) -> bytes:
    """Inverse migration. Returns the old bytes, patched from the snapshot."""
    base = schema.old_from_new(new)
    scheme_id, body = _unpack(snapshot)
    if scheme_id == SNAP_SCHEME_FULL:
        if len(body) != schema.OLD_SIZE:
            raise SnapshotError(f"full snapshot body is {len(body)} bytes")
        return body
    if scheme_id != SNAP_SCHEME_DELTA:
        raise SnapshotError(f"unknown snapshot scheme id {scheme_id}")
    fields = _parse_delta(body)
    buf = bytearray(base)
    for fid, value in fields.items():
        f = schema.OLD_FIELDS[fid]
        if f.name == "checksum":
            continue
        buf[f.offset:f.offset + f.size] = value
    if schema.FIELD_INDEX["checksum"] in fields:
        f = schema.OLD_FIELDS[schema.FIELD_INDEX["checksum"]]
        buf[f.offset:f.offset + f.size] = fields[schema.FIELD_INDEX["checksum"]]
    else:
        struct.pack_into("<I", buf, schema.CHECKSUM_OFF, schema.checksum(buf))
    return bytes(buf)


def undo_independent(new: bytes, snapshot: bytes) -> bytes:
    """Undo written from the binary layout alone, not via ``schema`` helpers.

    This is the cross-check for the round-trip property.  It decodes NAV1 with
    its own offset table (reading each point as one little-endian u32 rather
    than two u16s), reassembles NAV0 with its own field slicing, and calls
    ``zlib.crc32`` directly.  Keeping it separate is what makes byte equality
    evidence rather than a tautology: if both implementations shared a
    misreading of the layout, they would agree with each other and be wrong
    together.
    """
    nav0 = [("magic", 0, 4), ("version", 4, 2), ("flags", 6, 2), ("path_len", 8, 4)]
    nav0 += [(f"path{i}", 12 + 4 * i, 4) for i in range(48)]
    nav0 += [("start_idx", 204, 4), ("goal_idx", 208, 4), ("cost", 212, 4),
             ("created_at", 216, 8), ("checksum", 224, 4)]

    if len(new) != 228:
        raise ValueError("independent decoder wants exactly 228 bytes")
    if new[0:4] != b"1VAN" or int.from_bytes(new[4:6], "little") != 2:
        raise ValueError("independent decoder rejects this magic/version")

    point_count = int.from_bytes(new[8:12], "little")
    flags = int.from_bytes(new[6:8], "little")
    values = {
        "magic": 0x4E415630,
        "version": 1,
        "flags": flags & 0xFFFE,
        "path_len": point_count,
        "cost": int.from_bytes(new[212:216], "little"),
        "created_at": int.from_bytes(new[216:224], "little"),
        "start_idx": int.from_bytes(new[206:208], "little") * 16
                     + int.from_bytes(new[204:206], "little"),
        "goal_idx": int.from_bytes(new[210:212], "little") * 16
                    + int.from_bytes(new[208:210], "little"),
    }
    for i in range(48):
        word = int.from_bytes(new[12 + 4 * i:16 + 4 * i], "little")
        values[f"path{i}"] = (word >> 16) * 16 + (word & 0xFFFF) if i < point_count else 0

    if len(snapshot) < 9 or snapshot[0:4] != b"SNP1":
        raise ValueError("independent decoder rejects this snapshot")
    body = snapshot[9:]
    if zlib.crc32(body) & 0xFFFFFFFF != int.from_bytes(snapshot[5:9], "little"):
        raise ValueError("independent decoder: snapshot crc")
    scheme_id = snapshot[4]
    if scheme_id == 1:
        if len(body) != 228:
            raise ValueError("independent decoder: full body size")
        return body
    if scheme_id != 0:
        raise ValueError("independent decoder: unknown scheme")

    sizes = [size for _, _, size in nav0]
    names = [name for name, _, _ in nav0]
    pos = 1
    for _ in range(body[0]):
        fid = body[pos]
        pos += 1
        size = sizes[fid]
        chunk = body[pos:pos + size]
        if len(chunk) != size:
            raise ValueError("independent decoder: truncated delta")
        values[names[fid]] = int.from_bytes(chunk, "little")
        pos += size
    if pos != len(body):
        raise ValueError("independent decoder: trailing delta bytes")

    out = bytearray(228)
    for name, off, size in nav0:
        if name == "checksum":
            continue
        out[off:off + size] = values[name].to_bytes(size, "little")
    return _finish_independent(out, values)


def _finish_independent(out: bytearray, values: dict[str, int]) -> bytes:
    """Write the checksum: the stored value if the delta carried one, else crc32."""
    if "checksum" in values:
        stored = values["checksum"] & 0xFFFFFFFF
    else:
        stored = zlib.crc32(bytes(out[:224])) & 0xFFFFFFFF
    out[224:228] = stored.to_bytes(4, "little")
    return bytes(out)


def snapshot_bytes_for(field_ids: list[int]) -> int:
    """Analytic snapshot size for a delta with the given stored fields."""
    return SNAP_HEADER + 1 + sum(1 + schema.OLD_FIELDS[i].size for i in field_ids)


def analytic_curve() -> list[tuple[int, int]]:
    """(stored field count, delta snapshot bytes) with most-likely-first ordering.

    Fields are added cheapest first, which is also the order in which padding
    corruption appears in practice (4-byte path slots, then scalars).
    """
    order = sorted(range(len(schema.OLD_FIELDS)),
                   key=lambda i: (schema.OLD_FIELDS[i].size, i))
    curve = []
    total = 0
    for k, fid in enumerate(order, start=1):
        total += 1 + schema.OLD_FIELDS[fid].size
        curve.append((k, SNAP_HEADER + 1 + total))
    return curve
