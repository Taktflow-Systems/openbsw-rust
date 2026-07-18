#!/usr/bin/env python3
"""Enforce source and license policy for every Rust dependency graph."""

from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path


ROOT = Path(__file__).resolve().parents[2]
POLICY_PATH = ROOT / "docs" / "port" / "dependency-policy.json"


def metadata(manifest: str) -> dict:
    command = [
        "cargo",
        "metadata",
        "--format-version",
        "1",
        "--locked",
        "--manifest-path",
        str(ROOT / manifest),
    ]
    return json.loads(subprocess.check_output(command, cwd=ROOT, text=True))


def main() -> int:
    policy = json.loads(POLICY_PATH.read_text(encoding="utf-8"))
    allowed = set(policy["allowed_license_expressions"])
    errors: list[str] = []
    seen: set[tuple[str, str, str | None]] = set()

    for manifest in policy["manifests"]:
        graph = metadata(manifest)
        workspace_root = Path(graph["workspace_root"]).resolve()
        for package in graph["packages"]:
            key = (package["name"], package["version"], package["source"])
            if key in seen:
                continue
            seen.add(key)
            source = package["source"]
            if source is None:
                package_path = Path(package["manifest_path"]).resolve()
                if workspace_root in package_path.parents:
                    if package.get("license") != policy["workspace_license"]:
                        errors.append(
                            f"{package['name']}: workspace package license must be "
                            f"{policy['workspace_license']}"
                        )
                elif ROOT not in package_path.parents:
                    errors.append(f"{package['name']}: path dependency escapes repository")
                continue
            if policy["forbid_git_dependencies"] and source.startswith("git+"):
                errors.append(f"{package['name']}: git dependency is forbidden: {source}")
            license_expression = package.get("license")
            if license_expression not in allowed:
                errors.append(
                    f"{package['name']} {package['version']}: unapproved license expression "
                    f"{license_expression!r}"
                )

    if errors:
        print("Dependency policy failures:", file=sys.stderr)
        for error in errors:
            print(f"- {error}", file=sys.stderr)
        return 1
    print(f"dependency policy passed for {len(seen)} unique packages")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
