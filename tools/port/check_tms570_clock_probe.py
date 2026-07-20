#!/usr/bin/env python3
"""Inspect the bounded RAM-only LC4357 clock qualification role."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/clock_probe"


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
        print("missing clock probe; run its release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-W", "-h", "-S", "-l", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-d", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x8077000", "SRAM clock-probe entry"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.clock_probe\s+PROGBITS\s+08077000\s+\S+\s+000[0-9a-f]{3}", "bounded SRAM section"),
            (
                r"LOAD\s+0x\S+\s+0x08077000\s+0x08077000\s+0x00[0-9a-f]{3}\s+0x00[0-9a-f]{3}\s+R E",
                "single executable SRAM segment",
            ),
            (r"08077000\s+\d+\s+FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+_tms570_clock_probe", "entry symbol"),
            (r"_tms570_clock_probe_pass", "stable pass loop symbol"),
        ]:
            require(readelf, pattern, description)

        body = disassembly.split("<_tms570_clock_probe>:", 1)[1]
        for pattern, description in [
            (r"movw\s+r4, #44301", "revision-B device-ID low half"),
            (r"movt\s+r4, #32836", "revision-B device-ID high half"),
            (r"movw\s+r3, #38144", "300-MHz PLL multiplier field"),
            (r"movt\s+r3, #16135", "PLL lock configuration"),
            (r"movt\s+r3, #8199", "PLL final-divider configuration"),
            (r"movw\s+r3, #1600", "DCC reference seed"),
            (r"movw\s+r3, #30150", "DCC measured-clock seed"),
            (r"cmp\s+(?:r10|sl), #5", "five-attempt bound"),
            (r"movw\s+(?:r12|ip), #19280", "physical pass marker low half"),
            (r"movt\s+(?:r12|ip), #17228", "physical pass marker high half"),
        ]:
            require(body, pattern, description)

        instructions = "\n".join(
            line for line in body.splitlines() if re.match(r"\s*8077[0-9a-f]{3}:", line)
        )
        if re.search(r"\b(sp|lr|push|pop|bl|blx|bx|ldm|stm)\b", instructions):
            raise ValueError("clock probe gained stack, call, return, or multi-register access")
        if re.search(r"\bldr\s+\w+,\s*\[pc", instructions):
            raise ValueError("clock probe gained a literal-pool load")

        allowed_store_bases = {
            "r0": {0x34, 0x38, 0x48, 0x70, 0x74, 0xD0, 0xEC},
            "r1": {0x18},
            "r2": {0x00, 0x08, 0x0C, 0x10, 0x14, 0x24, 0x28},
            "r3": {0x3C, 0x54},
        }
        stores = re.findall(
            r"\bstr\s+\w+,\s*\[(r[0-3])(?:,\s*#(\d+))?\]", instructions
        )
        if not stores:
            raise ValueError("clock probe has no reviewed MMIO stores")
        for base, offset_text in stores:
            offset = int(offset_text) if offset_text else 0
            if offset not in allowed_store_bases[base]:
                raise ValueError(f"unreviewed MMIO store via {base}+0x{offset:x}")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 clock-probe inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 clock probe current: bounded BE32 A32 SRAM-only PLL/DCC role")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
