#!/usr/bin/env python3
"""Compute conservative Cortex-M stack paths from compiler stack records."""

from __future__ import annotations

import argparse
import json
import re
import subprocess
from collections.abc import Iterable


def output(command: list[str]) -> str:
    return subprocess.run(command, check=True, capture_output=True, text=True).stdout


def stack_sizes(readobj: str, elf: str) -> dict[str, int]:
    result: dict[str, int] = {}
    pending: list[str] = []
    for line in output([readobj, "--stack-sizes", elf]).splitlines():
        match = re.search(r"Functions: \[(.*)]", line)
        if match:
            pending = [item.strip() for item in match.group(1).split(",")]
            continue
        match = re.search(r"Size: 0x([0-9A-Fa-f]+)", line)
        if match and pending:
            size = int(match.group(1), 16)
            for symbol in pending:
                result[symbol] = max(result.get(symbol, 0), size)
            pending = []
    if not result:
        raise RuntimeError("no compiler stack-size records found")
    return result


def call_graph(objdump: str, elf: str) -> tuple[dict[str, set[str]], set[str]]:
    graph: dict[str, set[str]] = {}
    indirect: set[str] = set()
    current: str | None = None
    for line in output([objdump, "-d", elf]).splitlines():
        header = re.match(r"^[0-9A-Fa-f]+ <([^>]+)>:$", line.strip())
        if header:
            current = header.group(1)
            graph.setdefault(current, set())
            continue
        if current is None:
            continue
        direct = re.search(r"\bblx?\s+[0-9A-Fa-f]+\s+<([^>]+)>", line)
        if direct:
            graph[current].add(direct.group(1).split("+", 1)[0])
        elif re.search(r"\bblx?\s+r(?:[0-9]+|1[0-5])\b", line):
            indirect.add(current)
    return graph, indirect


def longest_path(
    roots: Iterable[str], frames: dict[str, int], graph: dict[str, set[str]]
) -> tuple[int, int]:
    cycles = 0

    def visit(symbol: str, active: frozenset[str]) -> int:
        nonlocal cycles
        if symbol in active:
            cycles += 1
            return 0
        nested = max(
            (visit(child, active | {symbol}) for child in graph.get(symbol, set())),
            default=0,
        )
        return frames.get(symbol, 0) + nested

    return max((visit(root, frozenset()) for root in roots), default=0), cycles


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--readobj", required=True)
    parser.add_argument("--objdump", required=True)
    parser.add_argument("--elf", required=True)
    parser.add_argument("--indirect-reserve", type=int, default=1024)
    parser.add_argument("--exception-frame", type=int, default=104)
    args = parser.parse_args()

    frames = stack_sizes(args.readobj, args.elf)
    graph, indirect = call_graph(args.objdump, args.elf)
    task_direct, task_cycles = longest_path(["main"], frames, graph)
    isr_roots = [
        symbol
        for symbol in frames
        if any(token in symbol.lower() for token in ("hardfault", "defaulthandler", "isr"))
    ]
    isr_direct, isr_cycles = longest_path(isr_roots, frames, graph)
    task_required = task_direct + args.indirect_reserve
    isr_increment = args.exception_frame + isr_direct
    print(
        json.dumps(
            {
                "largest_frame_bytes": max(frames.values()),
                "stack_record_count": len(frames),
                "task_direct_path_bytes": task_direct,
                "indirect_call_reserve_bytes": args.indirect_reserve,
                "task_stack_required_bytes": task_required,
                "isr_direct_path_bytes": isr_direct,
                "exception_frame_bytes": args.exception_frame,
                "isr_stack_increment_bytes": isr_increment,
                "combined_msp_required_bytes": task_required + isr_increment,
                "indirect_call_sites": len(indirect),
                "cycle_edges_observed": task_cycles + isr_cycles,
            },
            sort_keys=True,
        )
    )


if __name__ == "__main__":
    main()
