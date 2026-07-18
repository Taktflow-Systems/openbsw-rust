#!/usr/bin/env python3
"""Reject board examples that bypass the typed shared BSP composition."""

from pathlib import Path
import re
import sys

ROOT = Path(__file__).resolve().parents[2]
EXAMPLES = ROOT / "crates" / "bsw-bsp-stm32" / "examples"
FORBIDDEN = re.compile(
    r"read_volatile|write_volatile|(?:0x400|0x480|0x500|0xE000)[0-9A-Fa-f_]*|unsafe\s+fn"
)


def main() -> int:
    failures: list[str] = []
    for source in sorted(EXAMPLES.glob("*.rs")):
        text = source.read_text(encoding="utf-8")
        if FORBIDDEN.search(text):
            failures.append(f"{source.relative_to(ROOT).as_posix()}: raw MMIO")
        if "board_apps::run(DemoRole::" not in text:
            failures.append(f"{source.relative_to(ROOT).as_posix()}: bypasses board_apps")
    if failures:
        print("\n".join(failures), file=sys.stderr)
        return 1
    print(f"STM32 example policy current: {len(list(EXAMPLES.glob('*.rs')))} entrypoints")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
