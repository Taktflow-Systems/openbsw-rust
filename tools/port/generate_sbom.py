"""Generate a deterministic CycloneDX 1.5 dependency SBOM from Cargo.lock."""

from __future__ import annotations

import argparse
import json
import pathlib
import subprocess
import sys


ROOT = pathlib.Path(__file__).resolve().parents[2]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--check", action="store_true")
    args = parser.parse_args()
    metadata = json.loads(
        subprocess.check_output(
            ["cargo", "metadata", "--locked", "--format-version", "1"],
            cwd=ROOT,
            text=True,
        )
    )
    components = []
    for package in sorted(metadata["packages"], key=lambda item: (item["name"], item["version"])):
        component = {
            "type": "library",
            "bom-ref": f"pkg:cargo/{package['name']}@{package['version']}",
            "name": package["name"],
            "version": package["version"],
            "purl": f"pkg:cargo/{package['name']}@{package['version']}",
        }
        if package.get("license"):
            component["licenses"] = [{"expression": package["license"]}]
        components.append(component)
    document = {
        "bomFormat": "CycloneDX",
        "specVersion": "1.5",
        "serialNumber": "urn:uuid:00000000-0000-4000-8000-000000000000",
        "version": 1,
        "metadata": {"component": {"type": "application", "name": "openbsw-rust", "version": "0.1.0"}},
        "components": components,
    }
    output = (ROOT / args.output).resolve()
    if ROOT not in output.parents:
        raise SystemExit("output must remain inside the workspace")
    rendered = json.dumps(document, indent=2, sort_keys=True) + "\n"
    if args.check:
        if not output.is_file() or output.read_text(encoding="utf-8").replace("\r\n", "\n") != rendered:
            print(f"stale generated SBOM: {args.output}", file=sys.stderr)
            return 1
        return 0
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(rendered, encoding="utf-8", newline="\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
