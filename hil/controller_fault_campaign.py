"""Run privacy-safe physical CAN overflow or bus-off fault injection."""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import pathlib
import re
import subprocess
import sys
import time

from deterministic_fixture import (
    BOARDS,
    CAN_INTERFACE_RE,
    FIXTURE_RE,
    FIXTURE_REVISION,
    PRIVATE_EVIDENCE,
    clear_noinit,
    fixture_lock,
    probe,
    run,
    verify_peer_isolation,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]
REMOTE_BUSOFF = r'''
import can,json,time
bus=can.Bus(interface="socketcan",channel="__CAN_INTERFACE__")
seen=0; started=time.monotonic_ns(); deadline=time.monotonic()+12
while time.monotonic()<deadline:
    msg=bus.recv(0.1)
    if msg is not None and msg.arbitration_id==0x600: seen+=1
print(json.dumps({"valid_frames":seen,"duration_us":(time.monotonic_ns()-started)//1000}))
bus.shutdown()
'''
REMOTE_OVERFLOW = r'''
import can,json,time
bus=can.Bus(interface="socketcan",channel="__CAN_INTERFACE__")
sent=0; send_errors=0
for _ in range(256):
    try:
        bus.send(can.Message(arbitration_id=0x600,data=[2,0x3e,0,0,0,0,0,0],is_extended_id=False),timeout=0.1)
        sent+=1
    except can.CanError:
        send_errors+=1
time.sleep(11)
try: bus.send(can.Message(arbitration_id=0x600,data=[2,0x3e,0,0,0,0,0,0],is_extended_id=False),timeout=1.0)
except can.CanError: send_errors+=1
positive=False; deadline=time.monotonic()+2
while time.monotonic()<deadline:
    msg=bus.recv(0.05)
    if msg is not None and msg.arbitration_id==0x601 and bytes(msg.data)[:3]==bytes([2,0x7e,0]):
        positive=True; break
print(json.dumps({"burst_frames":sent,"send_errors":send_errors,"valid_response":positive}))
bus.shutdown()
'''


def reset_can(ssh_target: str, can_interface: str) -> None:
    run(
        [
            "ssh",
            ssh_target,
            (
                f"sudo -n ip link set {can_interface} down; "
                f"sudo -n ip link set {can_interface} type can bitrate 500000; "
                f"sudo -n ip link set {can_interface} up"
            ),
        ],
        timeout=15,
    )


def remote_process(ssh_target: str, can_interface: str, mode: str) -> subprocess.Popen[str]:
    script = REMOTE_BUSOFF if mode == "busoff" else REMOTE_OVERFLOW
    script = script.replace("__CAN_INTERFACE__", can_interface)
    encoded = base64.b64encode(script.encode()).decode()
    return subprocess.Popen(
        ["ssh", ssh_target, f"echo {encoded} | base64 -d | python3"],
        cwd=ROOT,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
    )


def capture(
    board, selector: str, serial_port: str, artifact: pathlib.Path, mode: str, ssh_target: str, can_interface: str
) -> tuple[str, dict[str, object], int]:
    import serial

    probe(board, selector, "download", artifact)
    clear_noinit(board, selector, artifact)
    reset_can(ssh_target, can_interface)
    started = time.monotonic_ns()
    with serial.Serial(serial_port, 115200, timeout=0.05) as stream:
        stream.reset_input_buffer()
        remote = remote_process(ssh_target, can_interface, mode)
        probe(board, selector, "reset")
        captured = bytearray()
        needle = (
            b"CAN busoff recovered_us="
            if mode == "busoff"
            else b"CAN stress valid-traffic=accepted"
        )
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and needle not in captured:
            captured.extend(stream.read(512))
        try:
            stdout, _stderr = remote.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            remote.kill()
            remote.communicate()
            stdout = ""
    return (
        captured.decode("utf-8", errors="replace").replace("\r", ""),
        json.loads(stdout) if stdout.strip() else {"remote_output": "missing"},
        (time.monotonic_ns() - started) // 1_000,
    )


def case(name: str, passed: bool, duration_us: int, detail: str, threshold: str):
    return {
        "name": name,
        "passed": passed,
        "duration_us": duration_us,
        "detail": detail,
        "threshold": threshold,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--mode", choices=("busoff", "overflow"), required=True)
    parser.add_argument("--artifact", type=pathlib.Path, required=True)
    parser.add_argument("--fixture-id", required=True)
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
    expected_role = f"can_{args.mode}_" if args.mode == "busoff" else "can_stress_"
    if expected_role not in artifact.name:
        raise SystemExit("artifact name does not match the selected fault mode")
    board = BOARDS[args.board]
    selector = os.environ.get(board.probe_env)
    serial_port = os.environ.get(board.serial_env)
    ssh_target = os.environ.get("OPENBSW_HIL_SSH")
    can_interface = os.environ.get("OPENBSW_HIL_CAN_INTERFACE")
    if not selector or not serial_port or not ssh_target or not can_interface:
        raise SystemExit(f"set {board.probe_env}, {board.serial_env}, OPENBSW_HIL_SSH, and OPENBSW_HIL_CAN_INTERFACE")
    if not CAN_INTERFACE_RE.fullmatch(can_interface):
        raise SystemExit("invalid SocketCAN interface selector")

    with fixture_lock(args.fixture_id):
        verify_peer_isolation()
        text, remote, duration_us = capture(
            board, selector, serial_port, artifact, args.mode, ssh_target, can_interface
        )

    if args.mode == "busoff":
        match = re.search(r"recovered_us=(\d+)", text)
        recovery_us = int(match.group(1)) if match else 0
        cases = [
            case(
                "error-passive",
                "CAN error-passive observed_us=" in text,
                duration_us,
                "real controller entered error-passive",
                "error-passive marker required before bus-off",
            ),
            case(
                "bus-off",
                "CAN busoff observed" in text,
                duration_us,
                "real controller entered bus-off",
                "bus-off marker required",
            ),
            case(
                "bus-off-recovery-timing",
                0 < recovery_us <= 1_100_000,
                recovery_us,
                f"recovery_us={recovery_us}",
                "recovery <= 1100000 us",
            ),
            case(
                "valid-traffic-after-bus-off",
                "valid-tx=true" in text and int(remote.get("valid_frames", 0)) >= 1,
                duration_us,
                f"fixture_valid_frames={int(remote.get('valid_frames', 0))}",
                "controller accepts TX and fixture observes at least one frame",
            ),
        ]
    else:
        match = re.search(r"CAN stress overflow=(\d+)", text)
        dropped = int(match.group(1)) if match else 0
        cases = [
            case(
                "hardware-overflow-accounting",
                dropped > 0 and int(remote.get("burst_frames", 0)) >= 64,
                duration_us,
                f"overflow_count={dropped}; fixture_burst={int(remote.get('burst_frames', 0))}",
                "overflow count > 0 after at least 64 transmitted burst frames",
            ),
            case(
                "valid-traffic-after-overflow",
                "CAN stress valid-traffic=accepted" in text,
                duration_us,
                "target accepted post-overflow diagnostic traffic",
                "target acceptance marker required",
            ),
            case(
                "fixture-response-after-overflow",
                bool(remote.get("valid_response")),
                duration_us,
                "fixture received a valid post-overflow response",
                "positive TesterPresent response required",
            ),
        ]

    document = {
        "schema_version": 1,
        "fixture_id": args.fixture_id,
        "fixture_revision": FIXTURE_REVISION,
        "board": board.name,
        "mode": args.mode,
        "probe_class": "st-link",
        "can": {"interface_class": "socketcan", "bitrate": 500000, "mode": "classic"},
        "artifact": {
            "name": artifact.name,
            "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest(),
        },
        "isolation": {
            "fixture_lock": True,
            "peer_isolated": True,
            "can_reset": True,
            "serial_flushed": True,
            "noinit_cleared": True,
        },
        "cases": cases,
        "outcome": "passed" if all(item["passed"] for item in cases) else "failed",
    }
    from jsonschema import validate

    schema = json.loads(
        (ROOT / "hil" / "controller-fault.schema.json").read_text(encoding="utf-8")
    )
    validate(instance=document, schema=schema)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"board": board.name, "mode": args.mode, "outcome": document["outcome"]}))
    return 0 if document["outcome"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
