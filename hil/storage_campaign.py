"""Run reset-safe destructive storage conformance on the exact input ELF."""

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


ROOT = pathlib.Path(__file__).resolve().parents[1]
BOUNDS = {
    "stm32f413": (0x08140000, 0x08180000),
    "stm32g474": (0x0807E000, 0x08080000),
}


def resolve_nm(explicit: str | None) -> str:
    if explicit:
        return explicit
    for candidate in ("llvm-nm", "rust-nm", "arm-none-eabi-nm"):
        if shutil.which(candidate):
            return candidate
    raise SystemExit("an LLVM or Arm GNU nm executable is required")


def symbols(artifact: pathlib.Path, nm: str) -> tuple[int, int]:
    completed = subprocess.run(
        [nm, str(artifact)],
        cwd=ROOT,
        check=True,
        text=True,
        capture_output=True,
        timeout=30,
    )
    values: dict[str, int] = {}
    for line in completed.stdout.splitlines():
        match = re.match(r"^([0-9a-fA-F]+)\s+\w\s+(_storage_start|_storage_end)$", line)
        if match:
            values[match.group(2)] = int(match.group(1), 16)
    if set(values) != {"_storage_start", "_storage_end"}:
        raise RuntimeError("exact artifact lacks resolved storage symbols")
    return values["_storage_start"], values["_storage_end"]


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--artifact", type=pathlib.Path, required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument("--nm")
    parser.add_argument("--timeout", type=float, default=240)
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
    if board.storage not in artifact.name:
        raise SystemExit("artifact is not the selected board's storage conformance image")
    nm = resolve_nm(args.nm)
    resolved = symbols(artifact, nm)
    if resolved != BOUNDS[board.name]:
        raise SystemExit("resolved storage symbols exceed the authorized region")
    selector = os.environ.get(board.probe_env)
    serial_port = os.environ.get(board.serial_env)
    if not selector or not serial_port:
        raise SystemExit(f"set {board.probe_env} and {board.serial_env}")

    import serial

    with fixture_lock(args.fixture_id):
        verify_peer_isolation()
        resolved = symbols(artifact, nm)
        if resolved != BOUNDS[board.name]:
            raise SystemExit("immediate pre-flash storage-bound recheck failed")
        probe(board, selector, "download", artifact)
        captured = bytearray()
        started = time.monotonic()
        with serial.Serial(serial_port, 115200, timeout=0.05) as stream:
            stream.reset_input_buffer()
            probe(board, selector, "reset")
            while time.monotonic() - started < args.timeout:
                captured.extend(stream.read(1024))
                if b"storage conformance=" in captured:
                    time.sleep(0.2)
                    captured.extend(stream.read(1024))
                    break

    text = captured.decode("utf-8", errors="replace").replace("\r", "")
    boot_count = text.count("openbsw role=StorageConformance")
    resumed_cuts = [
        int(value)
        for value in re.findall(r"storage campaign resume cut=(\d+)", text)
    ]
    last_cut = resumed_cuts[-1] if resumed_cuts else -1
    terminal_error = re.search(r"storage conformance=Err\(([A-Za-z]+)\)", text)
    terminal_state = terminal_error.group(1) if terminal_error else "none"
    cases = [
        {
            "name": "linker-bounds",
            "passed": resolved == BOUNDS[board.name],
            "detail": "exact artifact matched the authorized linker-reserved region",
        },
        {
            "name": "backend-conformance",
            "passed": "storage conformance=Ok(TargetStorageReport" in text
            and "checks: 9" in text,
            "detail": (
                "geometry, programming, remount, and wear checks completed; "
                f"terminal-error={terminal_state}"
            ),
        },
        {
            "name": "reset-safe-journal-transitions",
            "passed": "power_cut_replays: 7" in text,
            "detail": (
                "seven ordered reset/reboot transition cuts recovered old or new data; "
                f"boots={boot_count}; last-resumed-cut={last_cut}"
            ),
        },
    ]
    document = {
        "schema_version": 1,
        "fixture_id": args.fixture_id,
        "fixture_revision": FIXTURE_REVISION,
        "board": board.name,
        "probe_class": "st-link",
        "artifact": {
            "name": artifact.name,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
        },
        "destructive_scope": "linker-reserved-storage-only",
        "reset_safe": True,
        "cases": cases,
        "outcome": "passed" if all(case["passed"] for case in cases) else "failed",
    }
    from jsonschema import validate

    schema = json.loads(
        (ROOT / "hil" / "storage-result.schema.json").read_text(encoding="utf-8")
    )
    validate(instance=document, schema=schema)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"board": board.name, "outcome": document["outcome"]}))
    return 0 if document["outcome"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
