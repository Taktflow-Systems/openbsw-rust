#!/usr/bin/env python3
"""Assert the compile-only LC4357 startup ELF layout and A32 ordering."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/startup_probe"


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
        print("missing startup probe; run the release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-h", "-S", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-D", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x20", "reset entry at 0x20"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.vectors\s+PROGBITS\s+00000000\s+\S+\s+000020", "32-byte vectors at zero"),
            (r"\.stacks\s+NOBITS\s+0807de00\s+\S+\s+002200", "8,704-byte top-of-SRAM stacks"),
            (r"\.retained\s+NOBITS\s+08070000\s+\S+\s+0000a0", "160-byte warm-reset record"),
            (r"08080000\s+0\s+NOTYPE\s+GLOBAL\s+DEFAULT\s+\d+\s+__stack_system_top", "system SP at SRAM end"),
        ]:
            require(readelf, pattern, description)
        for pattern, description in [
            (r"00000000 <__tms570_vector_table>", "vector symbol"),
            (r"4:\s+\S+\s+b\s+\S+ <_tms570_undefined>", "direct undefined vector"),
            (r"8:\s+\S+\s+b\s+\S+ <_tms570_svc>", "direct SVC vector"),
            (r"c:\s+\S+\s+b\s+\S+ <_tms570_prefetch_abort>", "direct prefetch-abort vector"),
            (r"10:\s+\S+\s+b\s+\S+ <_tms570_data_abort>", "direct data-abort vector"),
            (r"18:\s+e51ff1b0\s+ldr\s+pc, \[pc, #-432\]", "VIM IRQ vector"),
            (r"1c:\s+e51ff1b0\s+ldr\s+pc, \[pc, #-432\]", "VIM FIQ vector"),
            (r"cpsid\s+aif", "abort/interrupt masking at reset"),
            (r"mov(?:eq)?\s+r1, #255", "cold reset selects all SRAM banks"),
            (r"mov(?:ne)?\s+r1, #127", "warm reset excludes retained SRAM bank"),
            (r"strd\s+r2, \[r0\], #8", "warm-reset bank-7 ECC regeneration"),
            (r"cps\s+#19", "supervisor stack mode"),
            (r"cps\s+#23", "abort stack mode"),
            (r"cps\s+#27", "undefined stack mode"),
            (r"cps\s+#18", "IRQ stack mode"),
            (r"cps\s+#17", "FIQ stack mode"),
            (r"cps\s+#31", "system stack mode"),
            (r"vmsr\s+fpexc", "VFP enable before Rust"),
            (r"mov\s+r2, #128", "full VIM table initialization"),
            (r"mcr\s+15, 0, r0, cr7, cr10, \{1\}", "retained-record cache-line clean"),
            (r"bl\s+\S+ <tms570_rust_entry>", "Rust entry call"),
        ]:
            require(disassembly, pattern, description)
        reset = disassembly.split("<_tms570_reset>:", 1)[1].split(
            "<_tms570_reserved>:", 1
        )[0]
        completion_poll = reset.find("tst\tr1, #256")
        first_stack_use = reset.find("sp")
        if completion_poll < 0 or first_stack_use < completion_poll:
            raise ValueError("stack use appears before the SRAM/VIM completion poll")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 startup inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 startup inspection current: BE32 vectors, ECC ordering, mode stacks, and VIM")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
