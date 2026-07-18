#!/usr/bin/env python3
"""Validate the parity manifest and generate its human-readable status page."""

from __future__ import annotations

import argparse
from collections import Counter
import json
from pathlib import Path
import sys

REQUIRED = {
    "id", "upstream_path", "scope", "status", "strategy", "rust_crates",
    "owner_packages", "evidence", "candidate_test_roots",
}


def validate(manifest: dict, repo: Path) -> None:
    allowed = set(manifest["status_values"])
    seen: set[str] = set()
    for index, row in enumerate(manifest["rows"]):
        absent = REQUIRED - row.keys()
        if absent:
            raise ValueError(f"row {index} lacks fields: {sorted(absent)}")
        if row["id"] in seen:
            raise ValueError(f"duplicate row id: {row['id']}")
        seen.add(row["id"])
        if row["status"] not in allowed:
            raise ValueError(f"invalid status for {row['id']}: {row['status']}")
        if not row["owner_packages"] or not row["evidence"]:
            raise ValueError(f"row {row['id']} must have owner_packages and evidence")
        for evidence in row["evidence"]:
            if evidence.startswith(("http://", "https://")):
                continue
            if not (repo / evidence).exists():
                raise ValueError(f"missing evidence for {row['id']}: {evidence}")


def render(manifest: dict) -> str:
    rows = manifest["rows"]
    counts = Counter((row["scope"], row["status"]) for row in rows)
    mandatory = [row for row in rows if row["scope"] == "mandatory"]
    closed = sum(row["status"] in {"done", "native replacement"} for row in mandatory)
    lines = [
        "# OpenBSW Rust parity status",
        "",
        "> Generated from `parity-manifest.json` by `tools/port/generate_status.py`; do not edit by hand.",
        "",
        f"Pinned upstream: `{manifest['baseline']['commit']}`",
        "",
        f"Mandatory rows closed: **{closed}/{len(mandatory)}**. "
        "A row closes only as `done` or `native replacement`; `partial` never counts.",
        "",
        "| Scope | Done | Native replacement | Partial | Missing | Excluded |",
        "|---|---:|---:|---:|---:|---:|",
    ]
    for scope in sorted({row["scope"] for row in rows}):
        lines.append(
            f"| {scope} | {counts[scope, 'done']} | {counts[scope, 'native replacement']} | "
            f"{counts[scope, 'partial']} | {counts[scope, 'missing']} | {counts[scope, 'excluded']} |"
        )
    lines.extend([
        "",
        "| Row | Status | Strategy | Rust crates | Owner packages |",
        "|---|---|---|---|---|",
    ])
    for row in rows:
        crates = ", ".join(f"`{name}`" for name in row["rust_crates"]) or "—"
        owners = ", ".join(row["owner_packages"])
        lines.append(f"| `{row['id']}` | {row['status']} | {row['strategy']} | {crates} | {owners} |")
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path("docs/port/parity-manifest.json"))
    parser.add_argument("--check", type=Path)
    parser.add_argument("--write", type=Path)
    args = parser.parse_args()
    manifest_path = args.manifest.resolve()
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    repo = manifest_path.parents[2]
    validate(manifest, repo)
    output = render(manifest)
    if args.check and args.write:
        parser.error("--check and --write are mutually exclusive")
    if args.check:
        actual = args.check.read_text(encoding="utf-8").replace("\r\n", "\n")
        if actual != output:
            print(f"stale generated status: {args.check}", file=sys.stderr)
            return 1
    elif args.write:
        args.write.write_text(output, encoding="utf-8", newline="\n")
        print(f"wrote generated status: {args.write}")
    else:
        print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
