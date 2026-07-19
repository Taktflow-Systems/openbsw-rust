"""Capture one privacy-safe HIL UART stream until a required marker appears."""

from __future__ import annotations

import argparse
import os
import pathlib
import subprocess
import sys
import time


ROOT = pathlib.Path(__file__).resolve().parents[1]
BOARDS = {
    "stm32f413": ("STM32F413ZH", "OPENBSW_HIL_F413_PROBE"),
    "stm32g474": ("STM32G474RETx", "OPENBSW_HIL_G474_PROBE"),
}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--needle", required=True)
    parser.add_argument("--timeout", type=float, required=True)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    parser.add_argument("--board", choices=BOARDS)
    parser.add_argument("--artifact", type=pathlib.Path)
    args = parser.parse_args()
    if (args.board is None) != (args.artifact is None):
        parser.error("--board and --artifact must be used together")
    port_name = os.environ.get("OPENBSW_HIL_SERIAL")
    if not port_name:
        raise SystemExit("OPENBSW_HIL_SERIAL is required")
    output = (ROOT / args.output).resolve()
    target = (ROOT / "target").resolve()
    if target not in output.parents:
        raise SystemExit("capture output must be below target/")

    import serial

    captured = bytearray()
    with serial.Serial(port_name, 115200, timeout=0.05) as stream:
        stream.reset_input_buffer()
        if args.board is not None:
            chip, probe_env = BOARDS[args.board]
            selector = os.environ.get(probe_env)
            if not selector:
                raise SystemExit(f"{probe_env} is required")
            artifact = (ROOT / args.artifact).resolve()
            if ROOT not in artifact.parents or not artifact.is_file():
                raise SystemExit("artifact must be an existing workspace file")
            subprocess.run(
                [
                    "probe-rs",
                    "download",
                    "--chip",
                    chip,
                    "--probe",
                    selector,
                    "--verify",
                    "--disable-progressbars",
                    str(artifact),
                ],
                cwd=ROOT,
                check=True,
                timeout=60,
                capture_output=True,
            )
            subprocess.run(
                ["probe-rs", "reset", "--chip", chip, "--probe", selector],
                cwd=ROOT,
                check=True,
                timeout=30,
                capture_output=True,
            )
        deadline = time.monotonic() + args.timeout
        needle = args.needle.encode()
        while time.monotonic() < deadline:
            captured.extend(stream.read(512))
            if needle in captured:
                time.sleep(0.2)
                captured.extend(stream.read(2048))
                break
    text = captured.decode("utf-8", errors="replace").replace("\r", "")
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(text, encoding="utf-8")
    if args.needle not in text:
        print("required marker not observed", file=sys.stderr)
        return 1
    print("required marker observed")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
