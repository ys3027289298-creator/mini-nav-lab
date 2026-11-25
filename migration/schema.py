"""Byte-level schema for the mini-nav path record.

Old structure (NAV0), 228 bytes, little endian::

    off  size  field
      0     4  magic      = 0x4E415630 ("NAV0")
      4     2  version    = 1
      6     2  flags      (bit 0 must be 0)
      8     4  path_len   number of used path slots, 0..48
     12   192  path[48]   flat cell index, y*16 + x
    204     4  start_idx  flat cell index
    208     4  goal_idx   flat cell index
    212     4  cost
    216     8  created_at microseconds since epoch
    224     4  checksum   crc32 over bytes [0, 224)

New structure (NAV1), also 228 bytes::

    off  size  field
      0     4  magic      = 0x4E415631 ("NAV1")
      4     2  version    = 2
      6     2  flags      = old flags | COORD_FORMAT
      8     4  point_count
     12   192  points[48] {u16 x, u16 y}
    204     2  start_x
    206     2  start_y
    208     2  goal_x
    210     2  goal_y
    212     4  cost
    216     8  updated_at  assigned by the migration, NOT the old value
    224     4  checksum     crc32 over bytes [0, 224)

The two layouts are deliberately the same size: the migration is a
re-interpretation of the same 228 bytes, not a resize.  ``old_from_new``
implements the documented inverse of every field; only ``created_at`` has no
representation in NAV1 at all, which is why it is the single field a snapshot
must store (see docs/REVERSIBILITY.md).
"""

from __future__ import annotations

import struct
import zlib
from dataclasses import dataclass, field

W, H = 16, 12
CELLS = W * H
MAX_POINTS = 48

OLD_SIZE = 228
NEW_SIZE = 228
OLD_MAGIC = 0x4E415630
NEW_MAGIC = 0x4E415631
OLD_VERSION = 1
NEW_VERSION = 2
FLAG_COORD = 0x0001

CHECKSUM_OFF = 224
CHECKSUM_LEN = 4
PAYLOAD_LEN = CHECKSUM_OFF  # bytes covered by the checksum
PATH_OFF = 12

CONST = "const"
INVERSE = "inverse"
RECOMPUTE = "recompute"
STORE = "store"


@dataclass(frozen=True)
class Field:
    name: str
    offset: int
    size: int
    recovery: str
    note: str = ""


def _build_fields() -> tuple[Field, ...]:
    fields = [
        Field("magic", 0, 4, CONST, "NAV0 -> NAV1; both are known constants"),
        Field("version", 4, 2, CONST, "1 -> 2; both are known constants"),
        Field(
            "flags",
            6,
            2,
            INVERSE,
            "new = old | COORD_FORMAT; inverse clears the bit, valid only "
            "when the old bit was already clear",
        ),
        Field("path_len", 8, 4, INVERSE, "identical to new.point_count"),
    ]
    for i in range(MAX_POINTS):
        fields.append(
            Field(
                f"path[{i}]",
                PATH_OFF + 4 * i,
                4,
                INVERSE,
                "old index = y*W + x; new stores the (x, y) pair",
            )
        )
    fields += [
        Field("start_idx", 204, 4, INVERSE, "old index = y*W + x"),
        Field("goal_idx", 208, 4, INVERSE, "old index = y*W + x"),
        Field("cost", 212, 4, INVERSE, "copied verbatim"),
        Field(
            "created_at",
            216,
            8,
            STORE,
            "NAV1 only carries updated_at, assigned at migration time; the "
            "old value has no image and must be snapshotted",
        ),
        Field(
            "checksum",
            224,
            4,
            RECOMPUTE,
            "crc32 over bytes [0, 224); deterministic, recomputed on undo",
        ),
    ]
    return tuple(fields)


OLD_FIELDS: tuple[Field, ...] = _build_fields()
TOTAL_FIELDS = len(OLD_FIELDS)  # 57: 9 scalars + 48 path slots
FIELD_INDEX = {f.name: i for i, f in enumerate(OLD_FIELDS)}
FIELD_BY_NAME = {f.name: f for f in OLD_FIELDS}
STORE_FIELDS = tuple(f for f in OLD_FIELDS if f.recovery == STORE)


@dataclass
class OldRecord:
    """Logical old record. ``path`` is always 48 slots, zero padded."""

    flags: int = 0
    path_len: int = 0
    path: list[int] = field(default_factory=lambda: [0] * MAX_POINTS)
    start_idx: int = 0
    goal_idx: int = 0
    cost: int = 0
    created_at: int = 0


def checksum(buf: bytes | bytearray | memoryview) -> int:
    return zlib.crc32(bytes(buf[:PAYLOAD_LEN])) & 0xFFFFFFFF


def encode_old(rec: OldRecord) -> bytes:
    if len(rec.path) != MAX_POINTS:
        raise ValueError(f"path must have {MAX_POINTS} slots")
    buf = bytearray(OLD_SIZE)
    struct.pack_into("<IHH", buf, 0, OLD_MAGIC, OLD_VERSION, rec.flags & 0xFFFF)
    struct.pack_into("<I", buf, 8, rec.path_len & 0xFFFFFFFF)
    for i, value in enumerate(rec.path):
        struct.pack_into("<I", buf, PATH_OFF + 4 * i, value & 0xFFFFFFFF)
    struct.pack_into("<III", buf, 204, rec.start_idx & 0xFFFFFFFF,
                     rec.goal_idx & 0xFFFFFFFF, rec.cost & 0xFFFFFFFF)
    struct.pack_into("<Q", buf, 216, rec.created_at & 0xFFFFFFFFFFFFFFFF)
    struct.pack_into("<I", buf, CHECKSUM_OFF, checksum(buf))
    return bytes(buf)


def decode_old(buf: bytes) -> tuple[int, int, OldRecord]:
    """Decode without validating. Raises only on a wrong buffer size."""
    if len(buf) != OLD_SIZE:
        raise ValueError(f"old record must be {OLD_SIZE} bytes, got {len(buf)}")
    magic, version, flags = struct.unpack_from("<IHH", buf, 0)
    (path_len,) = struct.unpack_from("<I", buf, 8)
    path = list(struct.unpack_from("<48I", buf, PATH_OFF))
    start_idx, goal_idx, cost = struct.unpack_from("<III", buf, 204)
    (created_at,) = struct.unpack_from("<Q", buf, 216)
    return magic, version, OldRecord(flags, path_len, path, start_idx,
                                     goal_idx, cost, created_at)


def validate_old(buf: bytes) -> list[str]:
    """Canonical-form checks. An empty list means the stateless inverse is exact."""
    problems: list[str] = []
    if len(buf) != OLD_SIZE:
        return [f"size={len(buf)} (want {OLD_SIZE})"]
    magic, version, rec = decode_old(buf)
    if magic != OLD_MAGIC:
        problems.append(f"magic=0x{magic:08X} (want 0x{OLD_MAGIC:08X})")
    if version != OLD_VERSION:
        problems.append(f"version={version} (want {OLD_VERSION})")
    if rec.flags & FLAG_COORD:
        problems.append(f"flags=0x{rec.flags:04X} has COORD_FORMAT set")
    if rec.path_len > MAX_POINTS:
        problems.append(f"path_len={rec.path_len} > {MAX_POINTS}")
    for i in range(min(rec.path_len, MAX_POINTS)):
        if rec.path[i] >= CELLS:
            problems.append(f"path[{i}]={rec.path[i]} outside grid")
            break
    for i in range(rec.path_len, MAX_POINTS):
        if rec.path[i] != 0:
            problems.append(f"path[{i}]={rec.path[i]} beyond path_len is not zero")
            break
    if rec.start_idx >= CELLS:
        problems.append(f"start_idx={rec.start_idx} outside grid")
    if rec.goal_idx >= CELLS:
        problems.append(f"goal_idx={rec.goal_idx} outside grid")
    stored = struct.unpack_from("<I", buf, CHECKSUM_OFF)[0]
    actual = checksum(buf)
    if stored != actual:
        problems.append(f"checksum=0x{stored:08X} (want 0x{actual:08X})")
    return problems


def is_canonical(buf: bytes) -> bool:
    return not validate_old(buf)


def new_from_old(old: bytes, updated_at: int) -> bytes:
    """Forward migration. Total on any 228-byte input: values that do not fit
    the NAV1 coordinate fields are truncated (the safe snapshot scheme detects
    this and falls back to a full copy)."""
    _, _, rec = decode_old(old)
    buf = bytearray(NEW_SIZE)
    flags = (rec.flags & ~FLAG_COORD) | FLAG_COORD
    struct.pack_into("<IHH", buf, 0, NEW_MAGIC, NEW_VERSION, flags & 0xFFFF)
    struct.pack_into("<I", buf, 8, rec.path_len & 0xFFFFFFFF)
    for i in range(min(rec.path_len, MAX_POINTS)):
        value = rec.path[i] & 0xFFFFFFFF
        x = value % W
        y = (value // W) & 0xFFFF
        struct.pack_into("<HH", buf, PATH_OFF + 4 * i, x, y)

    def split(value: int) -> tuple[int, int]:
        value &= 0xFFFFFFFF
        return value % W, ((value // W) & 0xFFFF)

    sx, sy = split(rec.start_idx)
    gx, gy = split(rec.goal_idx)
    struct.pack_into("<HHHH", buf, 204, sx, sy, gx, gy)
    struct.pack_into("<I", buf, 212, rec.cost & 0xFFFFFFFF)
    struct.pack_into("<Q", buf, 216, updated_at & 0xFFFFFFFFFFFFFFFF)
    struct.pack_into("<I", buf, CHECKSUM_OFF, checksum(buf))
    return bytes(buf)


def old_from_new(new: bytes) -> bytes:
    """Documented inverse of NAV1 -> NAV0, without any snapshot.

    Exact for canonical old records. It is also the baseline every delta
    snapshot is measured against: whatever this function gets wrong is exactly
    what a delta has to store.
    """
    if len(new) != NEW_SIZE:
        raise ValueError(f"new record must be {NEW_SIZE} bytes, got {len(new)}")
    magic, version = struct.unpack_from("<IH", new, 0)
    if magic != NEW_MAGIC or version != NEW_VERSION:
        raise ValueError(f"not a NAV1 record: magic=0x{magic:08X} version={version}")
    (flags,) = struct.unpack_from("<H", new, 6)
    (point_count,) = struct.unpack_from("<I", new, 8)
    path = [0] * MAX_POINTS
    for i in range(min(point_count, MAX_POINTS)):
        x, y = struct.unpack_from("<HH", new, PATH_OFF + 4 * i)
        path[i] = y * W + x
    sx, sy, gx, gy = struct.unpack_from("<HHHH", new, 204)
    (cost,) = struct.unpack_from("<I", new, 212)
    (updated_at,) = struct.unpack_from("<Q", new, 216)
    buf = bytearray(OLD_SIZE)
    struct.pack_into("<IHH", buf, 0, OLD_MAGIC, OLD_VERSION,
                     (flags & ~FLAG_COORD) & 0xFFFF)
    struct.pack_into("<I", buf, 8, point_count & 0xFFFFFFFF)
    for i, value in enumerate(path):
        struct.pack_into("<I", buf, PATH_OFF + 4 * i, value & 0xFFFFFFFF)
    struct.pack_into("<III", buf, 204, (sy * W + sx) & 0xFFFFFFFF,
                     (gy * W + gx) & 0xFFFFFFFF, cost & 0xFFFFFFFF)
    # Best effort for the one field with no image in NAV1.
    struct.pack_into("<Q", buf, 216, updated_at & 0xFFFFFFFFFFFFFFFF)
    struct.pack_into("<I", buf, CHECKSUM_OFF, checksum(buf))
    return bytes(buf)


def field_bytes(buf: bytes, f: Field) -> bytes:
    return bytes(buf[f.offset:f.offset + f.size])


def changed_fields(old: bytes, recovered: bytes) -> list[Field]:
    """Fields whose old bytes the stateless inverse does not reproduce."""
    return [f for f in OLD_FIELDS if field_bytes(old, f) != field_bytes(recovered, f)]


def field_value(buf: bytes, f: Field) -> int:
    raw = buf[f.offset:f.offset + f.size]
    return int.from_bytes(raw, "little")


def diff_report(old: bytes, other: bytes, limit: int = 10) -> list[dict]:
    """Human-readable field-level diff, used when the byte-equality gate fails."""
    out = []
    for f in OLD_FIELDS:
        a = field_bytes(old, f)
        b = field_bytes(other, f)
        if a == b:
            continue
        out.append({
            "field": f.name,
            "offset": f.offset,
            "size": f.size,
            "old_hex": a.hex(),
            "new_hex": b.hex(),
            "old_value": int.from_bytes(a, "little"),
            "new_value": int.from_bytes(b, "little"),
            "recovery": f.recovery,
        })
        if len(out) >= limit:
            break
    return out
