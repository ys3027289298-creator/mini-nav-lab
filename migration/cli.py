"""Command-line entry point.

    python3 -m migration.cli check     # run the undo criterion over a tier
    python3 -m migration.cli migrate --tier small --out /tmp/nav1
    python3 -m migration.cli undo --new /tmp/nav1/000000.nav --snap /tmp/nav1/000000.snp

``check`` prints a field-level report for the first mismatches and returns
non-zero if any record fails, so it can be used as a gate in its own right.
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from . import dataset, schema, snapshot


def cmd_check(args: argparse.Namespace) -> int:
    total = 0
    failures = 0
    stored = 0
    for i, rec in enumerate(dataset.iter_records(args.tier)):
        if args.limit is not None and i >= args.limit:
            break
        mig = snapshot.migrate(rec, dataset.MIGRATION_EPOCH_US + i, args.scheme)
        got = snapshot.undo(mig.new, mig.snapshot)
        total += 1
        stored += mig.snapshot_bytes
        if got != mig.old:
            failures += 1
            if failures <= args.show:
                report = schema.diff_report(mig.old, got, limit=args.fields)
                print(f"record {i}: {len(report)} field(s) differ")
                for f in report:
                    print(f"  {f['field']:<10} off={f['offset']:<3} "
                          f"old=0x{f['old_hex']} got=0x{f['new_hex']} "
                          f"({f['old_value']} vs {f['new_value']})")
    rate = failures / total if total else 0.0
    print(f"tier={args.tier} scheme={args.scheme} records={total} "
          f"failures={failures} inconsistency_rate={rate:.6f} "
          f"mean_snapshot={stored / max(total, 1):.2f}B")
    return 0 if failures == 0 else 1


def cmd_migrate(args: argparse.Namespace) -> int:
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    manifest = []
    for i, rec in enumerate(dataset.iter_records(args.tier)):
        if args.limit is not None and i >= args.limit:
            break
        mig = snapshot.migrate(rec, dataset.MIGRATION_EPOCH_US + i, args.scheme)
        stem = f"{i:06d}"
        (out / f"{stem}.old").write_bytes(mig.old)
        (out / f"{stem}.nav").write_bytes(mig.new)
        (out / f"{stem}.snp").write_bytes(mig.snapshot)
        manifest.append({"record": i, "old_sha256": _sha(mig.old),
                         "new_sha256": _sha(mig.new),
                         "snapshot_bytes": mig.snapshot_bytes,
                         "stored_fields": list(mig.stored_fields)})
    (out / "manifest.json").write_text(json.dumps(manifest, indent=2), encoding="utf-8")
    print(f"wrote {len(manifest)} records to {out}")
    return 0


def cmd_undo(args: argparse.Namespace) -> int:
    new = Path(args.new).read_bytes()
    snap = Path(args.snap).read_bytes()
    old = snapshot.undo(new, snap)
    if args.out:
        Path(args.out).write_bytes(old)
        print(f"wrote {args.out} ({len(old)} bytes)")
    else:
        print(old.hex())
    return 0


def _sha(buf: bytes) -> str:
    import hashlib
    return hashlib.sha256(buf).hexdigest()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(prog="migration.cli")
    sub = parser.add_subparsers(dest="cmd", required=True)

    p = sub.add_parser("check", help="verify the undo criterion over a tier")
    p.add_argument("--tier", default="small", choices=list(dataset.SIZES))
    p.add_argument("--scheme", default=snapshot.DEFAULT_SCHEME,
                   choices=list(snapshot.SCHEMES))
    p.add_argument("--limit", type=int, default=None)
    p.add_argument("--show", type=int, default=5)
    p.add_argument("--fields", type=int, default=8)
    p.set_defaults(func=cmd_check)

    p = sub.add_parser("migrate", help="write migrated records and snapshots")
    p.add_argument("--tier", default="small", choices=list(dataset.SIZES))
    p.add_argument("--scheme", default=snapshot.DEFAULT_SCHEME,
                   choices=list(snapshot.SCHEMES))
    p.add_argument("--limit", type=int, default=100)
    p.add_argument("--out", required=True)
    p.set_defaults(func=cmd_migrate)

    p = sub.add_parser("undo", help="undo a single migrated record")
    p.add_argument("--new", required=True)
    p.add_argument("--snap", required=True)
    p.add_argument("--out")
    p.set_defaults(func=cmd_undo)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
