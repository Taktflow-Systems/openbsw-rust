#!/usr/bin/env python3
"""Compare normalized OpenBSW and Rust oracle records."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def normalized(path: Path) -> list[dict]:
    document = json.loads(path.read_text(encoding="utf-8"))
    records = document["records"]
    return [
        {key: value for key, value in record.items() if key not in {"timestamp_ns", "source"}}
        for record in records
    ]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("expected", type=Path)
    parser.add_argument("observed", type=Path)
    args = parser.parse_args()
    expected = normalized(args.expected)
    observed = normalized(args.observed)
    if expected != observed:
        print(json.dumps({"expected": expected, "observed": observed}, indent=2))
        return 1
    print(f"oracle comparison passed: {len(expected)} normalized record(s)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
