#!/usr/bin/env python3
"""Inspect the RAM-only LC4357 RTI counter-0 qualification role."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/rti_probe"


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
        print("missing RTI probe; run its release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-W", "-h", "-S", "-l", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-d", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x8076000", "SRAM RTI-probe entry"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.rti_probe\s+PROGBITS\s+08076000\s+\S+\s+0000[0-9a-f]{2}", "bounded SRAM section"),
            (
                r"LOAD\s+0x\S+\s+0x08076000\s+0x08076000\s+0x0{2,4}[0-9a-f]{2}\s+0x0{2,4}[0-9a-f]{2}\s+R E",
                "single executable SRAM segment",
            ),
            (r"08076000\s+\d+\s+FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+_tms570_rti_probe", "entry symbol"),
            (r"_tms570_rti_probe_pass", "stable pass loop symbol"),
        ]:
            require(readelf, pattern, description)

        body = disassembly.split("<_tms570_rti_probe>:", 1)[1]
        for pattern, description in [
            (r"movw\s+r0, #64512", "RTI register base low half"),
            (r"movt\s+r0, #65535", "RTI register base high half"),
            (r"mov\s+r1, #74", "75-to-1-MHz prescaler"),
            (r"movw\s+r1, #34464", "100-ms threshold low half"),
            (r"movt\s+r1, #1", "100-ms threshold high half"),
            (r"mrs\s+r4, CPSR", "critical-section entry-mask observation"),
            (r"cpsid\s+if", "IRQ/FIQ mask operation"),
            (r"cpsie\s+i", "conditional IRQ restore"),
            (r"cpsie\s+f", "conditional FIQ restore"),
            (r"movw\s+(?:r12|ip), #18768", "physical pass marker low half"),
            (r"movt\s+(?:r12|ip), #21076", "physical pass marker high half"),
        ]:
            require(body, pattern, description)

        instructions = "\n".join(
            line for line in body.splitlines() if re.match(r"\s*80760[0-9a-f]{2}:", line)
        )
        if re.search(r"\b(sp|lr|push|pop|bl|blx|bx|ldm|stm)\b", instructions):
            raise ValueError("RTI probe gained stack, call, return, or multi-register access")
        if re.search(r"\bldr\s+\w+,\s*\[pc", instructions):
            raise ValueError("RTI probe gained a literal-pool load")

        stores = re.findall(r"\bstr\s+\w+,\s*\[(r\d+)(?:,\s*#(\d+))?\]", instructions)
        expected = {("r0", 0x00), ("r0", 0x04), ("r0", 0x10), ("r0", 0x14), ("r0", 0x18)}
        observed = {(base, int(offset) if offset else 0) for base, offset in stores}
        if not stores or not observed.issubset(expected) or observed != expected:
            raise ValueError(f"unreviewed or missing RTI counter-0 stores: {sorted(observed)}")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 RTI-probe inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 RTI probe current: bounded BE32 A32 SRAM-only 1-MHz counter-0 role")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
