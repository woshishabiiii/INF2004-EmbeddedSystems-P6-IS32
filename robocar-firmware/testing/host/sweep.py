#!/usr/bin/env python3
"""Robustness sweep: run every simulator scenario on N random cars.

Each car's real motor and geometry differ from the values the firmware
assumes (motion_cfg.h), within what a real car may plausibly have:
  deadband 170-310 permille, feed-forward 1.4-2.1 permille per mm/s,
  track 122-142 mm, 1.88-2.12 encoder counts per mm, right wheel 85-100 %.

Usage (after `make`):   python3 sweep.py [seed] [cars]
Prints one line per car, then every failed check under it.
"""
import os
import random
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
random.seed(int(sys.argv[1]) if len(sys.argv) > 1 else 7)
n = int(sys.argv[2]) if len(sys.argv) > 2 else 12
bad = 0
for k in range(n):
    db = random.uniform(170, 310)
    kff = random.uniform(1.4, 2.1)
    tr = random.uniform(122, 142)
    cm = random.uniform(1.88, 2.12)
    re = random.uniform(0.85, 1.0)
    args = ['./sim', '-p', str(100 + k), f'{db:.0f}', f'{kff:.2f}', f'{tr:.0f}', f'{cm:.2f}', f'{re:.2f}']
    out = subprocess.run(args, capture_output=True, text=True, cwd=HERE).stdout
    fails = [l for l in out.splitlines() if 'FAIL' in l]
    summ = [l for l in out.splitlines() if 'SIMULATION' in l]
    print(' '.join(args[2:]), summ[0] if summ else '?', flush=True)
    for f in fails:
        print('    ', f.strip(), flush=True)
    bad += 1 if fails or not summ else 0
print(f'TOTAL: {n} cars, {n - bad} with 0 failed checks')
sys.exit(1 if bad else 0)
