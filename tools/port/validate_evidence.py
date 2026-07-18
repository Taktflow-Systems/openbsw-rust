#!/usr/bin/env python3
"""Dependency-free validation for the repository's evidence contract."""

from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
import re
import sys

REQUIRED = {"schema_version", "evidence_id", "package_ids", "started_at", "source", "environment", "commands", "results", "artifacts"}
SHA40 = re.compile(r"^[0-9a-f]{40}$")
SHA256 = re.compile(r"^[0-9a-f]{64}$")


def validate(path: Path) -> list[str]:
    doc = json.loads(path.read_text(encoding="utf-8"))
    errors = [f"missing {key}" for key in sorted(REQUIRED - doc.keys())]
    if doc.get("schema_version") != 1:
        errors.append("schema_version must be 1")
    try:
        datetime.fromisoformat(doc.get("started_at", "").replace("Z", "+00:00"))
    except ValueError:
        errors.append("started_at must be an ISO-8601 date-time")
    if not SHA40.fullmatch(doc.get("source", {}).get("openbsw_revision", "")):
        errors.append("source.openbsw_revision must be a 40-character lowercase SHA")
    if not doc.get("package_ids") or not doc.get("commands"):
        errors.append("package_ids and commands must not be empty")
    result = doc.get("results", {})
    if result.get("outcome") not in {"passed", "failed", "incomplete"}:
        errors.append("results.outcome is invalid")
    for field in ("passed", "failed", "skipped", "errors"):
        if not isinstance(result.get(field), int) or result.get(field, -1) < 0:
            errors.append(f"results.{field} must be a non-negative integer")
    for artifact in doc.get("artifacts", []):
        if not artifact.get("name") or not SHA256.fullmatch(artifact.get("sha256", "")):
            errors.append("each artifact needs a name and lowercase SHA-256")
    serialized = json.dumps(doc).lower()
    for forbidden in ("c:\\users\\", "/home/", "password", "access_token", "secret_key"):
        if forbidden in serialized:
            errors.append(f"private-data marker found: {forbidden}")
    return errors


def main() -> int:
    paths = [Path(arg) for arg in sys.argv[1:]] or sorted(Path("docs/test-evidence/samples").glob("*.json"))
    failed = False
    for path in paths:
        errors = validate(path)
        if errors:
            failed = True
            for error in errors:
                print(f"{path}: {error}", file=sys.stderr)
        else:
            print(f"valid evidence: {path}")
    return int(failed)


if __name__ == "__main__":
    raise SystemExit(main())
