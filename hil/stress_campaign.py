"""Run the privacy-safe G20 production-image stress campaign.

Bench selectors stay in environment variables. The JSON result contains only
generic board/fixture names, monotonic durations, thresholds, and outcomes.
"""

from __future__ import annotations

import argparse
import base64
import hashlib
import json
import os
import pathlib
import subprocess
import sys
import time

from deterministic_fixture import (
    BOARDS,
    CAN_INTERFACE_RE,
    FIXTURE_RE,
    PRIVATE_EVIDENCE,
    FIXTURE_REVISION,
    clear_noinit,
    fixture_lock,
    probe,
    run,
    verify_peer_isolation,
)


ROOT = pathlib.Path(__file__).resolve().parents[1]

REMOTE_CAMPAIGN = r'''
import can,json,time

SOAK_SECONDS=__SOAK_SECONDS__
BURST_REQUESTS=256
BURST_MIN_RESPONSES=64
RESOURCE_FRAMES=64
SOAK_HZ=10
SOAK_MIN_RATIO=0.999

bus=can.Bus(interface="socketcan",channel="__CAN_INTERFACE__")
def flush():
    while bus.recv(0.0) is not None: pass
def send(data):
    try:
        bus.send(can.Message(arbitration_id=0x600,data=data,is_extended_id=False),timeout=0.1)
        return True
    except can.CanError:
        return False
def positive(deadline=0.1):
    end=time.monotonic()+deadline
    while time.monotonic()<end:
        msg=bus.recv(0.01)
        if msg is not None and msg.arbitration_id==0x601 and bytes(msg.data)[:3]==bytes([2,0x7e,0]):
            return True
    return False
def case(name,passed,started,detail,threshold):
    return {"name":name,"passed":passed,"duration_us":(time.monotonic_ns()-started)//1000,"detail":detail,"threshold":threshold}

results=[]
flush(); started=time.monotonic_ns(); burst_sent=0
for _ in range(BURST_REQUESTS): burst_sent+=int(send([2,0x3e,0,0,0,0,0,0]))
time.sleep(11.0); flush(); responses=0; recovery_sent=0
for _ in range(BURST_MIN_RESPONSES):
    if send([2,0x3e,0,0,0,0,0,0]):
        recovery_sent+=1
        if positive(0.2): responses+=1
final_sent=send([2,0x3e,0,0,0,0,0,0]); recovered=final_sent and positive(1.0)
burst_passed=burst_sent>=64 and recovery_sent==BURST_MIN_RESPONSES and responses>=BURST_MIN_RESPONSES and recovered
results.append(case("burst-recovery",burst_passed,started,f"burst_sent={burst_sent}/{BURST_REQUESTS}; recovery_responses={responses}/{recovery_sent}; final_recovery={recovered}",f"burst_sent>=64, recovery responses>={BURST_MIN_RESPONSES}, and final recovery"))

flush(); started=time.monotonic_ns()
send([0x10,0x20,0x3e,0,1,2,3,4])
send([0x10,0x20,0x3e,0,5,6,7,8])
send([2,0x3e,0,0,0,0,0,0])
occupied=not positive(0.2)
time.sleep(1.1)
recovered=send([2,0x3e,0,0,0,0,0,0]) and positive(1.0)
results.append(case("resource-exhaustion-recovery",occupied and recovered,started,f"static_session_occupied={occupied}; final_recovery={recovered}","competing requests rejected while occupied and valid response after 1-second timeout"))

flush(); started=time.monotonic_ns(); sent=0; responses=0
deadline=time.monotonic()+SOAK_SECONDS; next_request=time.monotonic()
while time.monotonic()<deadline:
    if send([2,0x3e,0,0,0,0,0,0]):
        sent+=1
        if positive(0.08): responses+=1
    next_request+=1.0/SOAK_HZ
    delay=next_request-time.monotonic()
    if delay>0: time.sleep(delay)
ratio=(responses/sent) if sent else 0.0
recovered=send([2,0x3e,0,0,0,0,0,0]) and positive(1.0)
passed=sent>=int(SOAK_SECONDS*SOAK_HZ*0.99) and ratio>=SOAK_MIN_RATIO and recovered
results.append(case("bounded-soak",passed,started,f"responses={responses}/{sent}; ratio={ratio:.6f}; final_recovery={recovered}",f"duration={SOAK_SECONDS}s, rate={SOAK_HZ}Hz, ratio>={SOAK_MIN_RATIO}, final recovery"))
print(json.dumps(results,sort_keys=True)); bus.shutdown()
'''


def reset_can(ssh_target: str, can_interface: str) -> None:
    run(
        [
            "ssh",
            ssh_target,
            f"sudo -n ip link set {can_interface} down; sudo -n ip link set {can_interface} type can bitrate 500000; sudo -n ip link set {can_interface} up",
        ],
        timeout=15,
    )


def repeated_resets(board, selector: str, serial_port: str, count: int) -> dict[str, object]:
    try:
        import serial
    except ImportError as error:
        raise RuntimeError("pyserial is required for authoritative HIL") from error
    durations: list[int] = []
    ready_boots = 0
    ready_deadline_seconds = 8.0
    production_ready = b"reference ready UDS/ISO-TP"
    for _ in range(count):
        with serial.Serial(serial_port, 115200, timeout=0.05) as stream:
            stream.reset_input_buffer()
            started = time.monotonic_ns()
            probe(board, selector, "reset")
            captured = bytearray()
            deadline = time.monotonic() + ready_deadline_seconds
            while time.monotonic() < deadline and production_ready not in captured:
                captured.extend(stream.read(512))
            durations.append((time.monotonic_ns() - started) // 1_000)
            if production_ready in captured:
                ready_boots += 1
    deadline_us = int(ready_deadline_seconds * 1_000_000)
    passed = ready_boots == count and all(duration <= deadline_us for duration in durations)
    return {
        "name": "repeated-reset",
        "passed": passed,
        "duration_us": sum(durations),
        "detail": f"ready_boots={ready_boots}/{count}; max_boot_us={max(durations, default=0)}",
        "threshold": f"{count} consecutive ready markers within {int(ready_deadline_seconds)} seconds each",
    }


def remote_campaign(ssh_target: str, can_interface: str, soak_seconds: int) -> list[dict[str, object]]:
    script = REMOTE_CAMPAIGN.replace("__SOAK_SECONDS__", str(soak_seconds)).replace("__CAN_INTERFACE__", can_interface)
    encoded = base64.b64encode(script.encode()).decode()
    try:
        completed = subprocess.run(
            ["ssh", ssh_target, f"echo {encoded} | base64 -d | python3"],
            cwd=ROOT,
            check=False,
            text=True,
            capture_output=True,
            timeout=soak_seconds + 60,
        )
    except subprocess.TimeoutExpired as error:
        raise RuntimeError("remote stress campaign exceeded its bounded timeout") from error
    if completed.returncode != 0 or not completed.stdout.strip():
        raise RuntimeError("remote stress campaign failed without publishable output")
    return json.loads(completed.stdout)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--board", choices=BOARDS, required=True)
    parser.add_argument("--artifact", type=pathlib.Path, required=True)
    parser.add_argument("--fixture-id", required=True)
    parser.add_argument("--soak-seconds", type=int, default=600)
    parser.add_argument("--reset-count", type=int, default=10)
    parser.add_argument("--output", type=pathlib.Path, required=True)
    args = parser.parse_args()
    if args.soak_seconds < 600:
        raise SystemExit("release soak must run for at least 600 seconds")
    if args.reset_count < 10:
        raise SystemExit("release reset campaign requires at least 10 resets")
    artifact = (ROOT / args.artifact).resolve()
    if ROOT not in artifact.parents or not artifact.is_file():
        raise SystemExit("artifact must be an existing workspace file")
    output = (ROOT / args.output).resolve()
    if PRIVATE_EVIDENCE not in output.parents:
        raise SystemExit("output must be below target/private-evidence")

    board = BOARDS[args.board]
    fixture_id = args.fixture_id
    if not FIXTURE_RE.fullmatch(fixture_id):
        raise SystemExit("--fixture-id must be a generic fixture-<token> identifier")
    selector = os.environ.get(board.probe_env)
    serial_port = os.environ.get(board.serial_env)
    ssh_target = os.environ.get("OPENBSW_HIL_SSH")
    can_interface = os.environ.get("OPENBSW_HIL_CAN_INTERFACE")
    if not selector or not serial_port or not ssh_target or not can_interface:
        raise SystemExit(f"set {board.probe_env}, {board.serial_env}, OPENBSW_HIL_SSH, and OPENBSW_HIL_CAN_INTERFACE")
    if not CAN_INTERFACE_RE.fullmatch(can_interface):
        raise SystemExit("invalid SocketCAN interface selector")

    with fixture_lock(fixture_id):
        verify_peer_isolation()
        probe(board, selector, "download", artifact)
        clear_noinit(board, selector, artifact)
        reset_case = repeated_resets(board, selector, serial_port, args.reset_count)
        reset_can(ssh_target, can_interface)
        cases = [reset_case, *remote_campaign(ssh_target, can_interface, args.soak_seconds)]

    document = {
        "schema_version": 1,
        "fixture_id": fixture_id,
        "fixture_revision": FIXTURE_REVISION,
        "board": board.name,
        "probe_class": "st-link",
        "can": {"interface_class": "socketcan", "bitrate": 500000, "mode": "classic"},
        "reset_method": "debug-port-system-reset",
        "artifact": {"name": artifact.name, "sha256": hashlib.sha256(artifact.read_bytes()).hexdigest()},
        "soak_seconds": args.soak_seconds,
        "reset_count": args.reset_count,
        "isolation": {
            "fixture_lock": True,
            "peer_isolated": True,
            "can_reset": True,
            "noinit_cleared": True,
        },
        "cases": cases,
        "outcome": "passed" if all(case["passed"] for case in cases) else "failed",
    }
    from jsonschema import validate

    schema = json.loads(
        (ROOT / "hil" / "stress-result.schema.json").read_text(encoding="utf-8")
    )
    validate(instance=document, schema=schema)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(document, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"board": board.name, "cases": len(cases), "outcome": document["outcome"]}, sort_keys=True))
    return 0 if document["outcome"] == "passed" else 1


if __name__ == "__main__":
    sys.exit(main())
