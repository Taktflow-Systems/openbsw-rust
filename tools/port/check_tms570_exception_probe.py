#!/usr/bin/env python3
"""Inspect the RAM-only LC4357 retained-exception qualification role."""

from __future__ import annotations

from pathlib import Path
import re
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
ELF = ROOT / "target/armebv7r-none-eabihf/debug/examples/exception_probe"


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


def function(disassembly: str, symbol: str, next_symbol: str | None = None) -> str:
    start = disassembly.split(f"<{symbol}>:", 1)[1]
    return start if next_symbol is None else start.split(f"<{next_symbol}>:", 1)[0]


def main() -> int:
    if not ELF.is_file():
        print("missing exception probe; run its release-build command first", file=sys.stderr)
        return 1
    try:
        readelf = tool_output("arm-none-eabi-readelf", "-W", "-h", "-S", "-l", "-s", str(ELF))
        disassembly = tool_output("arm-none-eabi-objdump", "-d", str(ELF))
        for pattern, description in [
            (r"Class:\s+ELF32", "ELF32 class"),
            (r"Data:\s+2's complement, big endian", "big-endian encoding"),
            (r"Machine:\s+ARM", "Arm machine"),
            (r"Entry point address:\s+0x8073000", "SRAM exception entry"),
            (r"Version5 EABI, hard-float ABI", "EABI5 hard-float flags"),
            (r"\.exception_probe\s+PROGBITS\s+08073000\s+\S+\s+00072c", "bounded code section"),
            (r"\.retained\s+NOBITS\s+08074000\s+\S+\s+0000a0", "160-byte retained record"),
            (r"LOAD\s+\S+\s+0x08073000\s+0x08073000\s+0x0*72c\s+0x0*72c\s+R E", "single executable SRAM segment"),
            (r"08074000\s+160\s+OBJECT\s+GLOBAL\s+DEFAULT\s+\d+\s+__tms570_exception_record", "private retained symbol"),
        ]:
            require(readelf, pattern, description)

        for symbol in [
            "_tms570_exception_probe",
            "_tms570_exception_inject_undefined",
            "_tms570_exception_inject_svc",
            "_tms570_exception_inject_prefetch_abort",
            "_tms570_exception_inject_data_abort",
            "_tms570_exception_fault_undefined",
            "_tms570_exception_fault_svc",
            "_tms570_exception_fault_data_abort",
            "_tms570_undefined",
            "_tms570_svc",
            "_tms570_prefetch_abort",
            "_tms570_data_abort",
            "_tms570_exception_capture_common",
            "_tms570_exception_request_reset",
        ]:
            require(disassembly, rf"<{symbol}>", f"{symbol} symbol")

        for pattern, description in [
            (r"cpsid\s+aif", "asynchronous/IRQ/FIQ masking"),
            (r"cps\s+#27.*\n.*movw\s+sp, #16896.*\n.*movt\s+sp, #2055", "undefined stack"),
            (r"cps\s+#23.*\n.*movw\s+sp, #17408.*\n.*movt\s+sp, #2055", "abort stack"),
            (r"cps\s+#19.*\n.*movw\s+sp, #17920.*\n.*movt\s+sp, #2055", "supervisor stack"),
            (r"cps\s+#31.*\n.*movw\s+sp, #18432.*\n.*movt\s+sp, #2055", "system stack"),
            (r"udf\s+#19", "bounded undefined instruction"),
            (r"svc\s+0x00000013", "bounded supervisor call"),
            (r"movt\s+lr, #1024.*\n.*bx\s+lr", "read-only unimplemented instruction fetch"),
            (r"ldr\s+lr, \[lr\]", "read-only unimplemented data access"),
            (r"subeq\s+r0, r5, #8", "data-abort LR correction"),
            (r"subne\s+r0, r5, #4", "undefined/prefetch/SVC instruction correction"),
            (r"moveq\s+r0, r5", "SVC next-instruction handoff"),
            (r"stmia\s+r0, \{sp, lr\}\^", "User/System banked SP/LR capture"),
            (r"mrc\s+15, 0, r0, cr5, cr0, \{0\}", "DFSR capture"),
            (r"mrc\s+15, 0, r1, cr6, cr0, \{0\}", "DFAR capture"),
            (r"mrc\s+15, 0, r1, cr5, cr0, \{1\}", "IFSR capture"),
            (r"mrc\s+15, 0, r1, cr6, cr0, \{2\}", "IFAR capture"),
            (r"mrc\s+15, 0, r1, cr5, cr1, \{0\}", "ADFSR capture"),
            (r"mrc\s+15, 0, r1, cr5, cr1, \{1\}", "AIFSR capture"),
            (r"ldr[b]?\s+r3, \[r0\], #1", "byte-ordered retained CRC"),
            (r"mcr\s+15, 0, r0, cr7, cr10, \{1\}", "record cache-line clean to point of coherency"),
            (r"movw\s+r0, #65504.*\n.*movt\s+r0, #65535.*\n.*movw\s+r1, #32768.*\n.*str\s+r1, \[r0\]", "documented software-reset write"),
        ]:
            require(disassembly, pattern, description)

        entries = [
            ("_tms570_undefined", "_tms570_svc", 1),
            ("_tms570_svc", "_tms570_prefetch_abort", 2),
            ("_tms570_prefetch_abort", "_tms570_data_abort", 3),
            ("_tms570_data_abort", "_tms570_unknown_exception", 4),
        ]
        for current, following, class_value in entries:
            entry = function(disassembly, current, following)
            require(entry, r"push\s+\{r0, r1, r2, r3, r4, r5, r6, r7, r8, r9, sl, fp, ip, lr\}", f"{current} full entry frame")
            require(entry, rf"mov\s+r0, #{class_value}", f"{current} distinct class")

        request_reset = function(disassembly, "_tms570_exception_request_reset", "_tms570_exception_reset_failed")
        if len(re.findall(r"\bstr\b", request_reset)) != 1:
            raise ValueError("software-reset function gained an additional store")
        if "0xffffffe0" not in disassembly.lower() and "#65504" not in request_reset:
            raise ValueError("software-reset address construction changed")

        probe = function(disassembly, "_tms570_exception_probe", "_tms570_exception_inject_undefined")
        if re.search(r"\bstr\w*\s+\w+, \[(?:r3|r4|r5|r6|r7|r8|r9|ip|fp)", probe, re.IGNORECASE):
            raise ValueError("probe arming gained an unreviewed store base")
        if re.search(r"\b(?:str|stm)\w*\b", function(disassembly, "_tms570_exception_inject_undefined", "_tms570_exception_probe_fail"), re.IGNORECASE):
            raise ValueError("fault-injection blocks gained a write")
    except (RuntimeError, ValueError, IndexError) as error:
        print(f"TMS570 exception-probe inspection failed: {error}", file=sys.stderr)
        return 1
    print("TMS570 exception probe current: bounded BE32 A32 retained capture and read-only fault injection")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
