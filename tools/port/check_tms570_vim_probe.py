#!/usr/bin/env python3
"""Inspect the RAM-only LC4357 VIM IRQ/FIQ qualification role."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/vim_probe"


def tool_output(tool: str, *arguments: str) -> str:
    result = subprocess.run(
        [tool, *arguments], cwd=ROOT, text=True, capture_output=True, check=False
    )
    if result.returncode != 0:
        raise RuntimeError(f"{tool} failed: {result.stderr.strip()}")
    return result.stdout


def require(text: str, pattern: str, description: str) -> None:
    if re.search(pattern, text, re.MULTILINE | re.IGNORECASE) is None:
        raise ValueError(f"missing {description}")


def main() -> int:
    if not ELF.is_file():
        print("missing VIM probe; run its release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-W", "-h", "-S", "-l", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-d", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x8075000", "SRAM VIM-probe entry"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.vim_probe\s+PROGBITS\s+08075000\s+\S+\s+000[0-9a-f]{3}", "bounded code section"),
            (
                r"LOAD\s+0x\S+\s+0x08075000\s+0x08075000\s+0x00[0-9a-f]{3}\s+0x00[0-9a-f]{3}\s+R E",
                "single executable SRAM segment",
            ),
            (r"08075000\s+\d+\s+FUNC\s+GLOBAL\s+DEFAULT\s+\d+\s+_tms570_vim_probe", "entry symbol"),
            (r"_tms570_vim_probe_pass", "stable pass loop symbol"),
            (r"_tms570_irq_vector_entry", "IRQ entry symbol"),
            (r"_tms570_fiq_vector_entry", "FIQ entry symbol"),
        ]:
            require(readelf, pattern, description)

        for pattern, description in [
            (r"cps\s+#18.*\n.*movw\s+r0, #24064.*\n.*movt\s+r0, #2055.*\n.*mov\s+sp, r0", "IRQ banked stack"),
            (r"cps\s+#17.*\n.*movw\s+r0, #24320.*\n.*movt\s+r0, #2055.*\n.*mov\s+sp, r0", "FIQ banked stack"),
            (r"str\s+r4, \[r3, #12\]", "channel-2 vector word registration"),
            (r"bic\s+r4, r4, #4", "channel-2 IRQ classification"),
            (r"orr\s+r4, r4, #4", "channel-2 FIQ classification"),
            (r"movw\s+r4, #10000", "10-ms RTI stimulus"),
            (r"cpsie\s+i", "IRQ enable"),
            (r"cpsie\s+f", "FIQ enable"),
            (r"str\s+r5, \[r6, #28\]", "DEVICE#56 live Group-2 acknowledgement"),
            (r"str\s+r5, \[r6, #60\]", "DEVICE#56 shadow Group-2 acknowledgement"),
            (r"mov\s+r5, #10", "DEVICE#60 diagnostic-mode key"),
            (r"str\s+r5, \[r6, #56\]", "DEVICE#60 error-key write"),
            (r"push\s+\{r0, r1, r2, r3, ip, lr\}", "IRQ context save"),
            (r"pop\s+\{r0, r1, r2, r3, ip, lr\}", "IRQ context restore"),
            (r"movw\s+(?:r12|ip), #19792", "physical pass marker low half"),
            (r"movt\s+(?:r12|ip), #22089", "physical pass marker high half"),
        ]:
            require(disassembly, pattern, description)
        if len(re.findall(r"subs\s+pc, lr, #4", disassembly, re.IGNORECASE)) != 2:
            raise ValueError("missing exact IRQ/FIQ architectural exception returns")
        if len(re.findall(r"ldr\s+r[45], \[r2, #260\]", disassembly, re.IGNORECASE)) != 2:
            raise ValueError("missing bounded DEVICE#56 FIQINDEX refresh")

        instructions = "\n".join(
            line for line in disassembly.splitlines() if re.match(r"\s*8075[0-9a-b][0-9a-f]{2}:", line)
        )
        if re.search(r"\bldr\s+\w+,\s*\[pc", instructions):
            raise ValueError("VIM probe gained a literal-pool load")

        aliases = {"ip": "r12", "sl": "r10", "fp": "r11"}
        allowed_store_bases = {
            "r0": {0x00, 0x04, 0x0C, 0x10, 0x14, 0x18, 0x50, 0x54, 0x80, 0x84, 0x88},
            "r1": {0x5C, 0x60},
            "r2": {0xF8, 0x110, 0x130, 0x140},
            "r3": {0x0C},
            "r6": {0x1C, 0x38, 0x3C},
            "r7": {0x00, 0x04, 0x08, 0x0C, 0x10, 0x14, 0x18, 0x1C, 0x20, 0x24, 0x28, 0x2C, 0x30, 0x34},
            "r10": {0x88},
            "r12": {0x04, 0x0C, 0x18, 0x1C},
        }
        stores = re.findall(
            r"\bstr\s+\w+,\s*\[((?:r\d+)|ip|sl|fp)(?:,\s*#(\d+))?\]", instructions
        )
        if not stores:
            raise ValueError("VIM probe has no reviewed stores")
        for raw_base, offset_text in stores:
            base = aliases.get(raw_base, raw_base)
            offset = int(offset_text) if offset_text else 0
            if base not in allowed_store_bases or offset not in allowed_store_bases[base]:
                raise ValueError(f"unreviewed store via {base}+0x{offset:x}")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 VIM-probe inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 VIM probe current: bounded BE32 A32 SRAM-only RTI channel-2 IRQ/FIQ role")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
