#!/usr/bin/env python3
"""Assert that the physical LC4357 RAM marker is three register-only A32 instructions."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/ram_marker"


def tool_output(tool: str, *arguments: str) -> str:
    result = subprocess.run(
        [tool, *arguments],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(f"{tool} failed: {result.stderr.strip()}")
    return result.stdout


def require(text: str, pattern: str, description: str) -> None:
    if re.search(pattern, text, re.MULTILINE) is None:
        raise ValueError(f"missing {description}")


def main() -> int:
    if not ELF.is_file():
        print("missing RAM marker; run its release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-h", "-S", "-l", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-d", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x8078000", "SRAM marker entry"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.ram_marker\s+PROGBITS\s+08078000\s+\S+\s+00000c", "12-byte SRAM section"),
            (r"LOAD\s+0x\S+\s+0x08078000\s+0x08078000\s+0x0000c\s+0x0000c\s+R E", "single executable SRAM segment"),
            (r"08078000\s+12\s+FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+_tms570_ram_marker", "entry symbol"),
        ]:
            require(readelf, pattern, description)
        marker = disassembly.split("<_tms570_ram_marker>:", 1)[1]
        instructions = [
            line for line in marker.splitlines() if re.match(r"\s*807800[0-8]:", line)
        ]
        if len(instructions) != 3:
            raise ValueError("RAM marker is not exactly three A32 instructions")
        for pattern, description in [
            (r"8078000:\s+e305c332\s+movw\s+(?:r12|ip), #21298", "R12 marker low half"),
            (r"8078004:\s+e345c44d\s+movt\s+(?:r12|ip), #21581", "R12 marker high half"),
            (r"8078008:\s+eafffffe\s+b\s+8078008(?:\s|$)", "self branch"),
        ]:
            require(marker, pattern, description)
        if re.search(r"\b(sp|lr|ldr|str|push|pop|bl|bx)\b", "\n".join(instructions)):
            raise ValueError("RAM marker gained stack, memory, or call behavior")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 RAM marker inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 RAM marker current: 12-byte BE32 A32 register-only SRAM loop")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
