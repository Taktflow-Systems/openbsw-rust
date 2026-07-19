#!/usr/bin/env python3
"""Fail when publishable repository content contains private bench data."""

from __future__ import annotations

import pathlib
import re
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]
FORBIDDEN_SUFFIXES = {".pyc", ".pyo", ".pyd"}
CACHE_DIRECTORIES = {"__pycache__", ".pytest_cache"}
SKIP_CONTENT = {
    "Cargo.lock",
    "fuzz/Cargo.lock",
    "tools/resource-probe/Cargo.lock",
    "docs/port/sbom.cdx.json",
    "LICENSE",
    "NOTICE",
    "tools/port/check_privacy.py",
}
LEGACY_TRACKED_BYTECODE = {
    "hil/__pycache__/conftest.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_architecture.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_dem_dtc.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_e2e.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_isotp_framing.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_negative_paths.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_nvm_persistence.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_session_security.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_stress.cpython-314-pytest-9.0.2.pyc",
    "hil/__pycache__/test_uds_services.cpython-314-pytest-9.0.2.pyc",
    "hil/helpers/__pycache__/__init__.cpython-314.pyc",
    "hil/helpers/__pycache__/can_transport.cpython-314.pyc",
    "hil/helpers/__pycache__/isotp.cpython-314.pyc",
    "hil/helpers/__pycache__/uds.cpython-314.pyc",
}
PATTERNS = {
    "windows-user-path": re.compile(rb"(?i)[a-z]:[\\/]+users[\\/]+[^\\/\s<]+[\\/]"),
    "unix-user-home": re.compile(rb"/home/[^/\s<]+/"),
    "private-ipv4": re.compile(
        rb"(?<![0-9])(?:10(?:\.[0-9]{1,3}){3}|192\.168(?:\.[0-9]{1,3}){2}|172\.(?:1[6-9]|2[0-9]|3[01])(?:\.[0-9]{1,3}){2})(?![0-9])"
    ),
    "email-address": re.compile(rb"(?i)\b[a-z0-9._%+-]+@[a-z0-9.-]+\.[a-z]{2,}\b"),
    "hardware-serial": re.compile(rb"(?i)(?<![0-9a-f])[0-9a-f]{24}(?![0-9a-f])"),
    "embedded-credential": re.compile(
        rb"(?i)(?:password|access[_-]?token|secret[_-]?key)\s*[:=]\s*[^\s<][^\r\n]*"
    ),
    "private-build-digest": re.compile(
        rb"(?i)(?:sha-?256|firmware[_ -]?digest|artifact[_ -]?digest)[^\r\n]{0,40}\b[0-9a-f]{64}\b"
    ),
}


def candidate_paths() -> list[pathlib.Path]:
    completed = subprocess.run(
        ["git", "ls-files", "--cached", "--others", "--exclude-standard", "-z"],
        cwd=ROOT,
        check=True,
        capture_output=True,
    )
    return [
        ROOT / pathlib.Path(raw.decode("utf-8"))
        for raw in completed.stdout.split(b"\0")
        if raw
    ]


def main() -> int:
    findings: list[tuple[str, str]] = []
    paths = candidate_paths()
    for path in paths:
        relative = path.relative_to(ROOT).as_posix()
        if (
            path.suffix.lower() in FORBIDDEN_SUFFIXES
            or any(part in CACHE_DIRECTORIES for part in path.parts)
        ) and relative not in LEGACY_TRACKED_BYTECODE:
            findings.append((relative, "python-bytecode"))
            continue
        if relative in SKIP_CONTENT or not path.is_file():
            continue
        try:
            data = path.read_bytes()
        except OSError:
            findings.append((relative, "unreadable"))
            continue
        if b"\0" in data:
            continue
        for name, pattern in PATTERNS.items():
            if pattern.search(data):
                findings.append((relative, name))
        if relative.startswith("docs/test-evidence/samples/") and re.search(
            rb'"sha256"\s*:', data
        ):
            findings.append((relative, "tracked-private-artifact-digest"))
    if findings:
        for path, kind in sorted(set(findings)):
            print(f"privacy finding: {path}: {kind}", file=sys.stderr)
        return 1
    print(f"privacy gate passed: {len(paths)} publishable repository files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
