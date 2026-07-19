"""Run the reset-spanning H01-H08 production safety campaign."""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import tempfile
import time

from deterministic_fixture import (
    BOARDS,
    FIXTURE_RE,
    FIXTURE_REVISION,
    PRIVATE_EVIDENCE,
    fixture_lock,
    probe,
    verify_peer_isolation,
)
from storage_campaign import BOUNDS, resolve_nm, symbols


ROOT = pathlib.Path(__file__).resolve().parents[1]
NOINIT = {
    "stm32f413": (0x2004FC00, 1024),
    "stm32g474": (0x2001FC00, 1024),
}


def resolve_objcopy(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in ("llvm-objcopy", "rust-objcopy", "arm-none-eabi-objcopy"):
        if shutil.which(candidate):
            return candidate
    raise SystemExit("an LLVM or Arm GNU objcopy executable is required")


def finalized_crc(artifact: pathlib.Path, objcopy: str) -> bool:
    with tempfile.TemporaryDirectory(prefix="openbsw-safety-crc-") as directory:
        dumped = pathlib.Path(directory) / "expected.bin"
        subprocess.run(
            [objcopy, f"--dump-section=.openbsw_rom_crc={dumped}", str(artifact)],
            cwd=ROOT,
            check=True,
            capture_output=True,
            timeout=30,
        )
        value = dumped.read_bytes()
        return len(value) == 4 and value != b"\xff\xff\xff\xff"


def symbol_value(artifact: pathlib.Path, nm: str, name: str) -> int:
    completed = subprocess.run(
        [nm, str(artifact)],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
        timeout=30,
    )
    match = re.search(rf"^([0-9a-fA-F]+)\s+\w\s+{re.escape(name)}$", completed.stdout, re.MULTILINE)
    if not match:
        raise RuntimeError(f"exact artifact lacks {name}")
    return int(match.group(1), 16)


def clear_noinit(board, selector: str, artifact: pathlib.Path, nm: str) -> None:
    start, size = NOINIT[board.name]
    if symbol_value(artifact, nm, "_noinit_start") != start or size % 4 != 0:
        raise RuntimeError("exact artifact NOINIT symbol does not match the board contract")
    words_per_chunk = 64
    for word in range(0, size // 4, words_per_chunk):
        count = min(words_per_chunk, size // 4 - word)
        subprocess.run(
            [
                "probe-rs",
                "write",
                "b32",
                hex(start + word * 4),
                *(["0"] * count),
                "--chip",
                board.chip,
                "--probe",
                selector,
                "--non-interactive",
            ],
            cwd=ROOT,
            check=True,
            capture_output=True,
            timeout=30,
        )


def capture(
    board,
    selector: str,
    serial_port: str,
    artifact: pathlib.Path,
    nm: str,
    timeout: float,
) -> str:
    import serial

    probe(board, selector, "download", artifact)
    clear_noinit(board, selector, artifact, nm)
    with serial.Serial(serial_port, 115200, timeout=0.05) as stream:
        stream.reset_input_buffer()
        time.sleep(0.2)
        probe(board, selector, "reset")
        captured = bytearray()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            captured.extend(stream.read(1024))
            if b"safety conformance=passed" in captured or b"safety conformance=failed" in captured:
                time.sleep(0.2)
                captured.extend(stream.read(1024))
                break
    return captured.decode("utf-8", errors="replace").replace("\r", "")


def case(name: str, passed: bool, detail: str) -> dict[str, object]:
    return {"name": name, "passed": passed, "detail": detail}


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--artifact", type=pathlib.Path, required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument("--nm")
    parser.add_argument("--objcopy")
    parser.add_argument("--timeout", type=float, default=90)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if not FIXTURE_RE.fullmatch(args.fixture_id):
        raise SystemExit("--fixture-id must be a generic fixture-<token> identifier")
    artifact = args.artifact.resolve()
    output = args.output.resolve()
    if not artifact.is_file() or ROOT not in artifact.parents:
        raise SystemExit("--artifact must identify an existing exact workspace artifact")
    if PRIVATE_EVIDENCE not in output.parents:
        raise SystemExit("--output must be below target/private-evidence")
    board = BOARDS[args.board]
    if f"safety_fault_{'f413' if board.name == 'stm32f413' else 'g474'}" not in artifact.name:
        raise SystemExit("artifact name does not match the selected safety campaign")
    nm = resolve_nm(args.nm)
    objcopy = resolve_objcopy(args.objcopy)
    if symbols(artifact, nm) != BOUNDS[board.name]:
        raise SystemExit("resolved storage symbols exceed the authorized region")
    if not finalized_crc(artifact, objcopy):
        raise SystemExit("safety artifact does not contain a finalized ROM CRC")
    selector = os.environ.get(board.probe_env)
    serial_port = os.environ.get(board.serial_env)
    if not selector or not serial_port:
        raise SystemExit(f"set {board.probe_env} and {board.serial_env}")

    with fixture_lock(args.fixture_id):
        verify_peer_isolation()
        if symbols(artifact, nm) != BOUNDS[board.name]:
            raise SystemExit("immediate pre-flash storage-bound recheck failed")
        text = capture(board, selector, serial_port, artifact, nm, args.timeout)

    expected_ecc = board.name == "stm32g474"
    common = "safety conformance=passed" in text
    watchdog_failure = re.search(r"watchdog-state=([A-Za-z]+)", text)
    rom_failure = re.search(r"rom-crc=([A-Za-z]+)", text)
    if watchdog_failure:
        terminal_stage = f"watchdog-{watchdog_failure.group(1)}"
    elif rom_failure:
        terminal_stage = f"rom-{rom_failure.group(1)}"
    elif "monitor-routing=false" in text:
        terminal_stage = "monitor-routing"
    elif "storage-isolation=false" in text:
        terminal_stage = "storage-isolation"
    elif "inherited-watchdog=false" in text:
        terminal_stage = "inherited-watchdog"
    elif "h04=false" in text:
        terminal_stage = "retained-handoff"
    elif "safety critical-reset=passed" in text:
        terminal_stage = "mpu-or-hardfault"
    elif "safety watchdog=passed" in text:
        terminal_stage = "critical-reset"
    elif "safety watchdog-fast-test=" in text:
        terminal_stage = "watchdog-reset"
    elif "safety isolation=passed" in text:
        terminal_stage = "watchdog-arm"
    elif "safety isolation=armed" in text:
        terminal_stage = "storage-erase"
    else:
        terminal_stage = "startup"
    diagnostic = (
        f"terminal-stage={terminal_stage}; "
        f"boots={text.count('openbsw role=SafetyConformance')}; "
        f"lines={len(text.splitlines())}; chars={len(text)}; "
        f"safety-tokens={text.count('safety')}; watchdog-tokens={text.count('watchdog')}"
        f"; rom-tokens={text.lower().count('rom')}; crc-tokens={text.lower().count('crc')}"
        f"; monitor-tokens={text.lower().count('monitor')}"
        f"; panic-tokens={text.lower().count('panic')}"
        f"; assertion-tokens={text.lower().count('assert')}"
        f"; hardfault-tokens={text.lower().count('hardfault')}"
        f"; failed-tokens={text.lower().count('failed')}"
    )
    cases = [
        case("H01-monitors", common and "h01=true" in text, "monitor fault routed by composed task"),
        case("H02-supervisor-policy", common and "h02=true" in text, "degraded and critical reactions executed"),
        case("H03-watchdog-fast-test", common and "h03=true" in text, "one retained watchdog reset completed"),
        case("H04-retained-fault", common and "h04=true" in text, "HardFault record survived reset and was durably handed off"),
        case("H05-rom-crc", common and "h05=true" in text, "finalized executable-region CRC passed"),
        case("H06-mpu-isr", common and "h06=true" in text, "negative executable-ROM write caused retained HardFault"),
        case(
            "H07-capabilities",
            common and "h07=true" in text and f"ecc_flash={str(expected_ecc).lower()}" in text,
            "board capability and live ECC observation matched the declared MCU limit",
        ),
        case(
            "H08-production-lifecycle",
            common and "h08=true" in text,
            f"production safety task/supervisor/sink completed the campaign; {diagnostic}",
        ),
    ]
    document = {
        "schema_version": 1,
        "fixture_id": args.fixture_id,
        "fixture_revision": FIXTURE_REVISION,
        "board": board.name,
        "probe_class": "st-link",
        "reset_method": "watchdog-controlled-hardfault",
        "artifact": {
            "name": artifact.name,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
        },
        "destructive_scope": "linker-reserved-storage-only",
        "isolation": {
            "fixture_lock": True,
            "peer_isolated": True,
            "serial_flushed": True,
            "noinit_cleared": True,
        },
        "cases": cases,
        "outcome": "passed" if all(item["passed"] for item in cases) else "failed",
    }
    from jsonschema import validate

    schema = json.loads((ROOT / "hil" / "safety-result.schema.json").read_text(encoding="utf-8"))
    validate(instance=document, schema=schema)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"board": board.name, "cases": len(cases), "outcome": document["outcome"]}, sort_keys=True))
    return 0 if document["outcome"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
