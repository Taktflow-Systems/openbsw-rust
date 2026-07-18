#!/usr/bin/env python3
"""Inventory pinned OpenBSW test candidates and verify manifest coverage."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

SOURCE_SUFFIXES = {".c", ".cc", ".cpp", ".cxx", ".h", ".hh", ".hpp", ".hxx"}


def module_paths(root: Path, relative: str) -> set[str]:
    base = root / relative
    if not base.is_dir():
        return set()
    return {f"{relative}/{entry.name}" for entry in base.iterdir() if entry.is_dir()}


def count_candidates(root: Path, roots: list[str]) -> int:
    files: set[Path] = set()
    for relative in roots:
        candidate = root / relative
        if candidate.is_file() and candidate.suffix.lower() in SOURCE_SUFFIXES:
            files.add(candidate)
        elif candidate.is_dir():
            files.update(
                path for path in candidate.rglob("*")
                if path.is_file() and path.suffix.lower() in SOURCE_SUFFIXES
            )
    return len(files)


def render(manifest: dict, upstream: Path) -> str:
    rows = manifest["rows"]
    tracked = {row["upstream_path"] for row in rows if row["upstream_path"]}
    expected = (
        module_paths(upstream, "libs/bsw")
        | module_paths(upstream, "libs/bsp")
        | module_paths(upstream, "libs/safety")
        | {"platforms/posix", "platforms/s32k1xx", "executables/referenceApp"}
    )
    missing = sorted(expected - tracked)
    stale = sorted(
        path for path in tracked
        if path.startswith(("libs/bsw/", "libs/bsp/", "libs/safety/"))
        and not (upstream / path).is_dir()
    )
    if missing or stale:
        raise ValueError(f"manifest coverage mismatch; missing={missing}, stale={stale}")

    lines = [
        "# Pinned upstream test inventory",
        "",
        f"OpenBSW commit: `{manifest['baseline']['commit']}`",
        "",
        "Counts include C/C++ source and header files below each candidate root. "
        "A zero means the behavior needs implementation-derived or integration evidence; "
        "it does not mean the module is out of scope.",
        "",
        "| Parity row | Status | Upstream path | Candidate files | Candidate roots |",
        "|---|---|---|---:|---|",
    ]
    total = 0
    for row in rows:
        if not row["upstream_path"]:
            continue
        roots = row["candidate_test_roots"]
        count = count_candidates(upstream, roots)
        total += count
        roots_text = "<br>".join(f"`{root}`" for root in roots) or "—"
        lines.append(
            f"| `{row['id']}` | {row['status']} | `{row['upstream_path']}` | "
            f"{count} | {roots_text} |"
        )
    lines.extend(["", f"Total candidate files (roots may intentionally overlap): **{total}**.", ""])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--manifest", type=Path, default=Path("docs/port/parity-manifest.json"))
    parser.add_argument("--upstream", type=Path, default=Path("target/oracle/openbsw"))
    parser.add_argument("--check", type=Path)
    args = parser.parse_args()
    manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
    output = render(manifest, args.upstream.resolve())
    if args.check:
        actual = args.check.read_text(encoding="utf-8").replace("\r\n", "\n")
        if actual != output:
            print(f"stale generated inventory: {args.check}", file=sys.stderr)
            return 1
    else:
        print(output, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
