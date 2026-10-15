#!/usr/bin/env python3
"""Verify deterministic Phase 11 frames in a captured USB-CDC transcript."""

import argparse
import re
import sys
from pathlib import Path


LINES_PER_WORKER = 64
FRAME = re.compile(
    r"P11\|W([01])\|P([12])\|L([0-9]{3})\|T([0-9a-f]{8})\|abcdefghijklmnop"
)


def expected_token(worker: int, line: int) -> int:
    return (0x11C0DC00 ^ (worker * 0x01010101) ^ (line * 0x9E3779B9)) & 0xFFFFFFFF


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("log", type=Path, help="captured USB-CDC text log")
    args = parser.parse_args()

    seen: set[tuple[int, int]] = set()
    errors: list[str] = []
    for number, raw in enumerate(args.log.read_text(errors="replace").splitlines(), 1):
        if "P11|" not in raw:
            continue
        match = FRAME.fullmatch(raw.rstrip("\r"))
        if match is None:
            errors.append(f"line {number}: malformed/interleaved frame: {raw!r}")
            continue
        worker, processor, line = map(int, match.group(1, 2, 3))
        token = int(match.group(4), 16)
        key = (worker, line)
        if processor != worker + 1:
            errors.append(f"line {number}: worker {worker} ran on processor {processor}")
        if line >= LINES_PER_WORKER:
            errors.append(f"line {number}: out-of-range frame {key}")
        if token != expected_token(worker, line):
            errors.append(f"line {number}: bad token for frame {key}: {token:08x}")
        if key in seen:
            errors.append(f"line {number}: duplicate frame {key}")
        seen.add(key)

    expected = {(worker, line) for worker in range(2) for line in range(LINES_PER_WORKER)}
    missing = sorted(expected - seen)
    if missing:
        errors.append(f"missing {len(missing)} frame(s): {missing[:8]}")

    if errors:
        for error in errors:
            print(f"FAIL: {error}", file=sys.stderr)
        return 1
    print(f"PASS: {len(seen)} complete Phase 11 frames; 64 from each processor")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
