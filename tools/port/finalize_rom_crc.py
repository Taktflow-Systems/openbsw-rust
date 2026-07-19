#!/usr/bin/env python3
"""Embed the executable-region CRC into an independently linked release ELF."""

from __future__ import annotations

import argparse
import pathlib
import shutil
import struct
import subprocess
import tempfile
import zlib


SECTION = ".openbsw_rom_crc"


def run(command: list[str]) -> None:
    subprocess.run(command, check=True, capture_output=True)


def resolve_objcopy(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in ("llvm-objcopy", "rust-objcopy", "arm-none-eabi-objcopy"):
        if shutil.which(candidate):
            return candidate
    raise SystemExit("an LLVM or Arm GNU objcopy executable is required")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--input", type=pathlib.Path, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--objcopy")
    args = parser.parse_args()
    source = args.input.resolve()
    output = args.output.resolve()
    objcopy = resolve_objcopy(args.objcopy)
    if source == output:
        raise SystemExit("input and output must be distinct exact artifacts")
    if not source.is_file():
        raise SystemExit("input ELF does not exist")
    output.parent.mkdir(parents=True, exist_ok=True)

    with tempfile.TemporaryDirectory(prefix="openbsw-rom-crc-") as directory:
        temporary = pathlib.Path(directory)
        text = temporary / "text.bin"
        expected = temporary / "expected.bin"
        verified = temporary / "verified.bin"
        run([objcopy, f"--dump-section=.text={text}", str(source)])
        checksum = zlib.crc32(text.read_bytes()) & 0xFFFF_FFFF
        expected.write_bytes(struct.pack("<I", checksum))
        run(
            [
                objcopy,
                f"--update-section={SECTION}={expected}",
                str(source),
                str(output),
            ]
        )
        run([objcopy, f"--dump-section={SECTION}={verified}", str(output)])
        if verified.read_bytes() != expected.read_bytes():
            raise SystemExit("embedded ROM CRC verification failed")
    shutil.copystat(source, output)
    print("release ROM CRC embedded and verified")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
