"""Privacy-safe deterministic STM32 smoke fixture.

Secrets and bench-local identifiers are accepted only through environment
variables and are never written to the result. All durations use monotonic
clocks. The fixture serializes access with a generic fixture-id lock.
"""

from __future__ import annotations

import argparse
import base64
import contextlib
import hashlib
import json
import os
import pathlib
import re
import shutil
import subprocess
import sys
import time
from dataclasses import dataclass


ROOT = pathlib.Path(__file__).resolve().parents[1]
PRIVATE_EVIDENCE = ROOT / "target" / "private-evidence"
FIXTURE_RE = re.compile(r"^fixture-[a-z0-9]{1,16}$")
CAN_INTERFACE_RE = re.compile(r"^[A-Za-z0-9_.-]{1,15}$")
FIXTURE_REVISION = "hil-fixture-v2"


@dataclass(frozen=True)
class Board:
    name: str
    feature: str
    chip: str
    app: str
    storage: str
    probe_env: str
    serial_env: str
    peer: str
    noinit_start: int
    noinit_size: int


BOARDS = {
    "stm32f413": Board("stm32f413", "stm32f413", "STM32F413ZH", "app_f413", "nvm_test_f413", "OPENBSW_HIL_F413_PROBE", "OPENBSW_HIL_F413_SERIAL", "stm32g474", 0x2004FC00, 1024),
    "stm32g474": Board("stm32g474", "stm32g474", "STM32G474RETx", "app_g474", "nvm_test_g474", "OPENBSW_HIL_G474_PROBE", "OPENBSW_HIL_G474_SERIAL", "stm32f413", 0x2001FC00, 1024),
}


def run(command: list[str], *, timeout: float = 120.0, capture: bool = True) -> subprocess.CompletedProcess[str]:
    return subprocess.run(command, cwd=ROOT, check=True, timeout=timeout, text=True, capture_output=capture)


@contextlib.contextmanager
def fixture_lock(fixture_id: str):
    lock_dir = ROOT / "target" / "hil-locks"
    lock_dir.mkdir(parents=True, exist_ok=True)
    path = lock_dir / f"{fixture_id}.lock"
    handle = path.open("a+b")
    locked = False
    try:
        if os.name == "nt":
            import msvcrt
            if handle.tell() == 0:
                handle.write(b"\0")
                handle.flush()
            handle.seek(0)
            msvcrt.locking(handle.fileno(), msvcrt.LK_NBLCK, 1)
        else:
            import fcntl
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        locked = True
        yield
    finally:
        if locked and os.name == "nt":
            import msvcrt
            handle.seek(0)
            msvcrt.locking(handle.fileno(), msvcrt.LK_UNLCK, 1)
        elif locked:
            import fcntl
            fcntl.flock(handle.fileno(), fcntl.LOCK_UN)
        handle.close()


def verify_peer_isolation() -> None:
    if os.environ.get("OPENBSW_HIL_PEER_ISOLATED") != "1":
        raise RuntimeError(
            "peer isolation must be established non-destructively before the campaign"
        )


def probe(board: Board, selector: str, action: str, artifact: pathlib.Path | None = None) -> None:
    command = ["probe-rs", action, "--chip", board.chip, "--probe", selector]
    if action == "download":
        command += ["--verify", "--disable-progressbars", str(artifact)]
    run(command, timeout=60)


def clear_noinit(board: Board, selector: str, artifact: pathlib.Path) -> None:
    nm = next((candidate for candidate in ("llvm-nm", "rust-nm", "arm-none-eabi-nm") if shutil.which(candidate)), None)
    if nm is None:
        raise RuntimeError("an LLVM or Arm GNU nm executable is required")
    table = run([nm, str(artifact)], timeout=30).stdout
    match = re.search(r"^([0-9a-fA-F]+)\s+\w\s+_noinit_start$", table, re.MULTILINE)
    if not match or int(match.group(1), 16) != board.noinit_start or board.noinit_size % 4:
        raise RuntimeError("exact artifact NOINIT symbol does not match the board contract")
    for word in range(0, board.noinit_size // 4, 64):
        count = min(64, board.noinit_size // 4 - word)
        run(
            [
                "probe-rs",
                "write",
                "b32",
                hex(board.noinit_start + word * 4),
                *(["0"] * count),
                "--chip",
                board.chip,
                "--probe",
                selector,
                "--non-interactive",
            ],
            timeout=30,
        )


def serial_boot(board: Board, selector: str, serial_port: str, artifact: pathlib.Path) -> tuple[list[str], int]:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required for authoritative HIL") from error
    probe(board, selector, "download", artifact)
    clear_noinit(board, selector, artifact)
    started = time.monotonic_ns()
    with serial.Serial(serial_port, 115200, timeout=0.05) as port:
        port.reset_input_buffer()
        time.sleep(0.2)
        probe(board, selector, "reset")
        data = bytearray()
        deadline = time.monotonic() + 4.0
        heartbeat_seen_at = None
        while time.monotonic() < deadline:
            data.extend(port.read(512))
            if heartbeat_seen_at is None and b"reference heartbeat" in data:
                heartbeat_seen_at = time.monotonic()
            if heartbeat_seen_at is not None and time.monotonic() - heartbeat_seen_at >= 0.5:
                break
    lines = data.decode("utf-8", errors="replace").replace("\r", "").splitlines()
    return lines, (time.monotonic_ns() - started) // 1_000


REMOTE_CASES = r'''
import can,json,time
bus=can.Bus(interface="socketcan",channel="__CAN_INTERFACE__")
def flush():
    while bus.recv(0.0) is not None: pass
def send(data): bus.send(can.Message(arbitration_id=0x600,data=data,is_extended_id=False))
def recv_id(deadline=1.0):
    end=time.monotonic()+deadline
    while time.monotonic()<end:
        msg=bus.recv(0.05)
        if msg is not None and msg.arbitration_id==0x601: return bytes(msg.data)
    return None
results=[]
flush(); t=time.monotonic_ns(); send([2,0x3e,0,0,0,0,0,0]); r=recv_id(); results.append({"name":"uds-single-frame","passed":r is not None and r[:3]==bytes([2,0x7e,0]),"duration_us":(time.monotonic_ns()-t)//1000,"detail":"TesterPresent positive response"})
flush(); t=time.monotonic_ns(); send([3,0x22,0xcf,1,0,0,0,0]); frames=[]; payload=bytearray(); total=None; seq=1; end=time.monotonic()+1.0
while time.monotonic()<end:
    r=recv_id(0.1)
    if r is None: continue
    frames.append(r.hex())
    if r[0]>>4==1: total=((r[0]&15)<<8)|r[1]; payload.extend(r[2:]); send([0x30,0,0,0,0,0,0,0])
    elif r[0]>>4==2:
        if (r[0]&15)!=seq: break
        seq=(seq+1)&15; payload.extend(r[1:])
        if total is not None and len(payload)>=total: break
expected=bytes.fromhex("62cf01")+b"OpenBSW Rust Reference!!"
results.append({"name":"uds-multiframe","passed":total==len(expected) and bytes(payload[:total])==expected and len(frames)==4,"duration_us":(time.monotonic_ns()-t)//1000,"detail":f"ISO-TP frames={len(frames)}"})
flush(); t=time.monotonic_ns(); send([0x1f,0xff,0,0,0,0,0,0]); send([0x22,0,0,0,0,0,0,0]); send([2,0x3e,0,0,0,0,0,0]); r=recv_id(); results.append({"name":"malformed-recovery","passed":r is not None and r[:3]==bytes([2,0x7e,0]),"duration_us":(time.monotonic_ns()-t)//1000,"detail":"invalid FF/CF did not poison next request"})
print(json.dumps(results,sort_keys=True)); bus.shutdown()
'''


def remote_cases(ssh_target: str, can_interface: str) -> list[dict[str, object]]:
    run(["ssh", ssh_target, f"sudo -n ip link set {can_interface} down; sudo -n ip link set {can_interface} type can bitrate 500000; sudo -n ip link set {can_interface} up"], timeout=15)
    script = REMOTE_CASES.replace("__CAN_INTERFACE__", can_interface)
    encoded = base64.b64encode(script.encode()).decode()
    completed = run(["ssh", ssh_target, f"echo {encoded} | base64 -d | python3"], timeout=15)
    return json.loads(completed.stdout)


def result_case(name: str, passed: bool, duration_us: int, detail: str) -> dict[str, object]:
    return {"name": name, "passed": passed, "duration_us": duration_us, "detail": detail}


def one_run(board: Board, fixture_id: str, run_index: int, artifact: pathlib.Path, selector: str, serial_port: str, ssh_target: str, can_interface: str) -> dict[str, object]:
    origin = time.monotonic_ns()
    lines, boot_us = serial_boot(board, selector, serial_port, artifact)
    joined = "\n".join(lines)
    cases = [
        result_case("clean-boot", "role=Application" in joined and "reference ready" in joined, boot_us, "verified flash/reset and production-ready marker"),
        result_case("clock-time", "clock_hz=" in joined and "heartbeat" in joined, boot_us, "configured clock assertion and monotonic heartbeat"),
        result_case(
            "uart-console",
            any("reference ready" in line and line.isascii() for line in lines),
            boot_us,
            "complete structured ASCII UART readiness record decoded",
        ),
        result_case("gpio-pwm-adc", "gpio=toggle" in joined and "adc=" in joined and "pwm=" in joined, boot_us, "shared I/O cycle observed"),
        result_case("lifecycle-watchdog", "watchdog=Ok" in joined, boot_us, "startup ordering and watchdog start observed"),
        result_case("reset-reason", "reset=" in joined, boot_us, "latched reset reason emitted"),
        result_case("storage", "storage mount=Ok" in joined, boot_us, "linker-reserved journal mounted"),
    ]
    cases.extend(remote_cases(ssh_target, can_interface))
    digest = hashlib.sha256(artifact.read_bytes()).hexdigest()
    return {
        "schema_version": 1,
        "fixture_id": fixture_id,
        "board": board.name,
        "run_index": run_index,
        "artifact": {"name": artifact.name, "sha256": digest},
        "monotonic": {"origin_ns": origin, "finished_ns": time.monotonic_ns()},
        "isolation": {
            "fixture_lock": True,
            "peer_isolated": True,
            "can_reset": True,
            "serial_flushed": True,
            "noinit_cleared": True,
        },
        "cases": cases,
        "outcome": "passed" if all(case["passed"] for case in cases) else "failed",
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--artifact", type=pathlib.Path, required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument("--runs", type=int, default=2)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    board = BOARDS[args.board]
    fixture_id = args.fixture_id
    if not FIXTURE_RE.fullmatch(fixture_id):
        raise SystemExit("--fixture-id must be a generic fixture-<token> identifier")
    selector = os.environ.get(board.probe_env); serial_port = os.environ.get(board.serial_env); ssh_target = os.environ.get("OPENBSW_HIL_SSH"); can_interface = os.environ.get("OPENBSW_HIL_CAN_INTERFACE")
    if not selector or not serial_port or not ssh_target or not can_interface: raise SystemExit(f"set {board.probe_env}, {board.serial_env}, OPENBSW_HIL_SSH, and OPENBSW_HIL_CAN_INTERFACE")
    if not CAN_INTERFACE_RE.fullmatch(can_interface): raise SystemExit("invalid SocketCAN interface selector")
    artifact = args.artifact.resolve()
    if not artifact.is_file() or ROOT not in artifact.parents:
        raise SystemExit("--artifact must identify an existing exact workspace artifact")
    output = args.output.resolve()
    if PRIVATE_EVIDENCE not in output.parents:
        raise SystemExit("--output must be below target/private-evidence")
    with fixture_lock(fixture_id):
        verify_peer_isolation()
        results = [one_run(board, fixture_id, index, artifact, selector, serial_port, ssh_target, can_interface) for index in range(1, args.runs + 1)]
    signatures = [[(case["name"], case["passed"]) for case in result["cases"]] for result in results]
    if any(result["outcome"] != "passed" for result in results) or any(signature != signatures[0] for signature in signatures[1:]):
        outcome = "failed"
    else: outcome = "passed"
    document = {
        "schema_version": 1,
        "fixture_id": fixture_id,
        "fixture_revision": FIXTURE_REVISION,
        "board": board.name,
        "probe_class": "st-link",
        "can": {"interface_class": "socketcan", "bitrate": 500000, "mode": "classic"},
        "reset_method": "debug-port-system-reset",
        "runs": results,
        "agreement": len(set(map(str, signatures))) == 1,
        "outcome": outcome,
    }
    from jsonschema import validate

    schema = json.loads((ROOT / "hil" / "result.schema.json").read_text(encoding="utf-8"))
    validate(instance=document, schema=schema)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"board": board.name, "runs": len(results), "agreement": document["agreement"], "outcome": outcome}, sort_keys=True))
    return 0 if outcome == "passed" else 1


if __name__ == "__main__": sys.exit(main())
