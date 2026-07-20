#!/usr/bin/env python3
"""Prove portable crates compile no_std and contain no heap allocation APIs."""

from __future__ import annotations

import os
from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
HOST_ONLY_CRATES = {
    "bsw-bsp-stm32",  # checked per MCU feature/target by check_features.py
    "bsw-bsp-tms570",  # checked on the tier-3 BE32 target by check_features.py
    "bsw-platform-posix",
    "openbsw-reference-app",
}
FORBIDDEN = re.compile(
    r"extern\s+crate\s+alloc|\balloc::|\bstd::(?:vec|boxed|string|collections)::|"
    r"\b(?:Vec|Box|String|HashMap|BTreeMap)\s*(?:::|<)"
)

# Module declarations gated on a std-implying cfg mark host adapters:
# per the architecture decisions, `std` enables host adapters without
# changing protocol behavior, so heap use inside those modules is allowed.
STD_GATED_MOD = re.compile(
    r"#\[cfg\([^\]]*feature\s*=\s*\"(?:std|socketcan)\"[^\]]*\)\]\s*(?:pub\s+)?mod\s+(\w+)\s*;",
    re.S,
)


def std_gated_paths(src: Path) -> set[Path]:
    """Files belonging to modules declared behind a std-implying cfg."""
    gated: set[Path] = set()
    for source in src.rglob("*.rs"):
        text = source.read_text(encoding="utf-8")
        for name in STD_GATED_MOD.findall(text):
            if source.name in ("lib.rs", "mod.rs"):
                base = source.parent
            else:
                base = source.parent / source.stem
            for candidate in (base / f"{name}.rs", base / name):
                if candidate.is_file():
                    gated.add(candidate)
                elif candidate.is_dir():
                    gated.update(candidate.rglob("*.rs"))
    return gated


def allocation_findings() -> list[str]:
    findings: list[str] = []
    for crate in sorted((ROOT / "crates").iterdir()):
        if not crate.is_dir() or crate.name in HOST_ONLY_CRATES:
            continue
        src = crate / "src"
        gated = std_gated_paths(src)
        for source in sorted(src.rglob("*.rs")):
            if source in gated:
                continue
            production = source.read_text(encoding="utf-8").split("#[cfg(test)]", 1)[0]
            for line_number, line in enumerate(production.splitlines(), 1):
                code = line.split("//", 1)[0]
                if FORBIDDEN.search(code):
                    findings.append(f"{source.relative_to(ROOT)}:{line_number}: {code.strip()}")
    return findings


def main() -> int:
    findings = allocation_findings()
    if findings:
        print("heap allocation APIs found in portable production code:", file=sys.stderr)
        print("\n".join(findings), file=sys.stderr)
        return 1
    print("allocation scan passed for portable production crates")

    env = os.environ.copy()
    env["RUSTFLAGS"] = "-D warnings"
    command = [
        "cargo", "check", "--workspace",
        "--exclude", "bsw-bsp-stm32",
        "--exclude", "bsw-bsp-tms570",
        "--exclude", "bsw-platform-posix",
        "--exclude", "openbsw-reference-app",
        "--no-default-features", "--target", "thumbv7em-none-eabihf",
    ]
    result = subprocess.run(command, cwd=ROOT, env=env, check=False)
    if result.returncode:
        print("no_std workspace check failed", file=sys.stderr)
        return result.returncode
    print("no_std workspace check passed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
