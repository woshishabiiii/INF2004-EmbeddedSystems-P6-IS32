# Subsystem 5 — test log

**Subsystem:** Obstacle Scan + Vehicle Controller (Buddy 5)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.
Three kinds of test are listed:

- **auto**: a `TEST_CHECK` in `subsystem5_test.c`, run on the Pico by the test image (`make TEST=1`);
- **host**: a check in `testing/host/unit_tests.c`, run on a laptop with `make` in `testing/host/` (the logic files only, no hardware);
- **manual**: a person measures or observes the result on the car.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | box found 300 mm ahead | FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U02 | box edges either side of the centre line | FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U03 | missing echoes counted as unknown, not free | FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U04 | sin / cos table | FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U05 | Vehicle Controller waits for START | FR6 | auto | `subsystem5_test.c` | PASS |
| S5-U06 | no pinging until START | FR5 | auto | `subsystem5_test.c` | PASS |
| S5-U07 | TRIG GP0 and ECHO GP1 are plain GPIO on this kernel build (UART0 not on them) | A6 | auto | `subsystem5_test.c` | PASS |
| S5-H01 | profile: width, closest point, wall beside the path, missing echoes: OP-01 … OP-05 | FR5 | host | `unit_tests.c` | all PASS |
| S5-H02 | scan: 70 ms trigger gap, single ALERT, coarse + fine angles, time limit, sensor fault, the stop flag stops the car with the ALERT and not without it, the ALERT point moves out with the measured speed and in with a harder learned stop (SC-10 … SC-12): SC-01 … SC-12 | FR5, NFR3, NFR6 | host | `unit_tests.c` | all PASS |
| S5-H03 | statechart: START, barcode at junction, stale DONE, STOP, faults, Continue / Bypass / Reverse decisions, HALTED for mission failures, rescan, START after HALTED, the WATCH stop flag, latency timestamp, step watchdogs from each move's own distance and speed (VC-25 … VC-27), on a hump the line is still followed unless both sensors read black (VC-28, VC-29): VC-01 … VC-29 | FR1–FR7 | host | `unit_tests.c` | all PASS |
| S5-M01 | ECHO high level at 3.3 V | NFR5, A5 | manual | Before wiring GP1: power the module from 3V3, meter or scope on ECHO while pinging. | ≤ 3.3 V |
| S5-M02 | range accuracy | FR5, A5 | manual | Board at 100, 200, 400, 800 mm straight ahead; read `rng`. | within ±10 mm |
| S5-M03 | servo calibration | FR5 | manual | Find the pulse widths that point the sensor 30° right and 30° left of straight ahead; set `SERVO_US_AT_30DEG` / `_150DEG`. | 30° = right, 150° = left |
| S5-M04 | profile of a known box | FR5 | manual | 100 mm box 250 mm ahead, centred, then 60 mm to the right; read the `profile` event. | edges within ±30 mm (wider, never narrower) |
| S5-M05 | bypass and rejoin | FR5 | manual | Box on a straight line; START. | stops, scans, passes on the shorter side, rejoins the line |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S5-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
| 2026-10-15 | S5-H01 … S5-H03 | working tree | host | PASS | `testing/host/logs/2026-10-15_unit.txt` | after the IS32 changes |
| 2026-10-23 | S5-H01 … S5-H03 (SC-01 … SC-12, VC-01 … VC-27) | working tree | host | PASS | `testing/host/logs/2026-10-23_unit.txt`, `2026-10-23_sim.txt`, `2026-10-23_robustness.txt` | speed-aware obstacle stop (simulated car at rest 145–152 mm from the box at 80, 120, 300 mm/s), step watchdogs instead of fixed 6/8/20 s limits, every speed capped at base_speed |
| 2026-10-23 | S5-H03 (VC-28, VC-29) | working tree | host | PASS | `testing/host/logs/2026-10-23_unit.txt`, `2026-10-23_sim.txt` | on a hump the line is still followed unless both sensors read black; `oval_tilt_run` failed on the old rule (car drove straight off the curves) |
