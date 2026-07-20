#!/usr/bin/env python3
"""Compile the supported feature matrix and assert MCU selection failures."""

from __future__ import annotations

import os
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[2]
TARGET = "thumbv7em-none-eabihf"
TMS570_TARGET = "armebv7r-none-eabihf"
TMS570_TOOLCHAIN = "+nightly-2026-03-14"


def cargo(
    arguments: list[str],
    should_pass: bool = True,
    expected_error: str = "exactly one MCU feature",
) -> bool:
    env = os.environ.copy()
    env["RUSTFLAGS"] = "-D warnings"
    command = ["cargo", *arguments]
    result = subprocess.run(command, cwd=ROOT, env=env, text=True, capture_output=True, check=False)
    display = " ".join(command)
    if should_pass and result.returncode != 0:
        print(f"FAILED: {display}\n{result.stdout}\n{result.stderr}", file=sys.stderr)
        return False
    if not should_pass:
        if result.returncode == 0:
            print(f"unexpected success: {display}", file=sys.stderr)
            return False
        if expected_error not in result.stderr:
            print(f"wrong failure for: {display}\n{result.stderr}", file=sys.stderr)
            return False
    print(f"feature check passed: {display}")
    return True


def main() -> int:
    checks = [
        cargo(["check", "-p", "bsw-can", "--no-default-features"]),
        cargo(["check", "-p", "bsw-can", "--no-default-features", "--features", "std"]),
        cargo(["check", "-p", "bsw-can", "--no-default-features", "--features", "can-fd"]),
        cargo(["check", "-p", "bsw-can", "--no-default-features", "--features", "std,can-fd"]),
        # socketcan implies std; the module itself only exists on Linux.
        cargo(["check", "-p", "bsw-can", "--no-default-features", "--features", "socketcan"]),
        cargo(["check", "-p", "bsw-async", "--no-default-features"]),
        cargo(["check", "-p", "bsw-async", "--no-default-features", "--features", "std"]),
        cargo(["check", "-p", "bsw-console", "--no-default-features"]),
        cargo(["check", "-p", "bsw-console", "--no-default-features", "--features", "std"]),
        cargo(["check", "-p", "bsw-logger", "--no-default-features"]),
        cargo(["check", "-p", "bsw-platform", "--no-default-features"]),
        cargo(["check", "-p", "bsw-storage", "--no-default-features"]),
        cargo(["check", "-p", "bsw-storage", "--no-default-features", "--features", "std"]),
        cargo(["check", "-p", "bsw-ethernet", "--no-default-features", "--features", "ipv6"]),
        cargo(["check", "-p", "bsw-bsp-stm32", "--no-default-features", "--features", "stm32f413", "--target", TARGET]),
        cargo(["check", "-p", "bsw-bsp-stm32", "--no-default-features", "--features", "stm32g474", "--target", TARGET]),
        cargo(["check", "-p", "bsw-bsp-stm32", "--no-default-features", "--features", "stm32f413,stm32g474", "--target", TARGET], should_pass=False),
        cargo(["check", "-p", "bsw-bsp-stm32", "--no-default-features", "--target", TARGET], should_pass=False),
        cargo([
            TMS570_TOOLCHAIN,
            "build",
            "-Z",
            "build-std=core",
            "--target",
            TMS570_TARGET,
            "-p",
            "bsw-bsp-tms570",
            "--features",
            "tms570lc4357-revb",
            "--lib",
            "--locked",
        ], should_pass=False, expected_error="requires board feature launchxl2-570lc43"),
        cargo([
            TMS570_TOOLCHAIN,
            "build",
            "-Z",
            "build-std=core",
            "--target",
            TMS570_TARGET,
            "-p",
            "bsw-bsp-tms570",
            "--features",
            "launchxl2-570lc43",
            "--lib",
            "--locked",
        ]),
        cargo([
            TMS570_TOOLCHAIN,
            "rustc",
            "-Z",
            "build-std=core",
            "--target",
            TMS570_TARGET,
            "-p",
            "bsw-bsp-tms570",
            "--features",
            "launchxl2-570lc43",
            "--example",
            "startup_probe",
            "--locked",
            "--",
            "-C",
            "linker=arm-none-eabi-gcc",
            "-C",
            "link-arg=-mcpu=cortex-r5",
            "-C",
            "link-arg=-marm",
            "-C",
            "link-arg=-mfloat-abi=hard",
            "-C",
            "link-arg=-mfpu=vfpv3-d16",
            "-C",
            "link-arg=-mbig-endian",
            "-C",
            "link-arg=-mbe32",
            "-C",
            "link-arg=-nostartfiles",
            "-C",
            "link-arg=-nostdlib",
            "-C",
            "link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-startup-probe.ld",
        ]),
        cargo([
            TMS570_TOOLCHAIN,
            "rustc",
            "-Z",
            "build-std=core",
            "--target",
            TMS570_TARGET,
            "-p",
            "bsw-bsp-tms570",
            "--features",
            "launchxl2-570lc43",
            "--example",
            "exception_probe",
            "--locked",
            "--",
            "-C",
            "linker=arm-none-eabi-gcc",
            "-C",
            "link-arg=-mcpu=cortex-r5",
            "-C",
            "link-arg=-marm",
            "-C",
            "link-arg=-mfloat-abi=hard",
            "-C",
            "link-arg=-mfpu=vfpv3-d16",
            "-C",
            "link-arg=-mbig-endian",
            "-C",
            "link-arg=-mbe32",
            "-C",
            "link-arg=-nostartfiles",
            "-C",
            "link-arg=-nostdlib",
            "-C",
            "link-arg=-Tcrates/bsw-bsp-tms570/linker/tms570lc4357-exception-probe.ld",
        ]),
    ]
    return 0 if all(checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
