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
| S5-H02 | scan: 70 ms trigger gap, single ALERT, coarse + fine angles, time limit, sensor fault, the stop flag stops the car with the ALERT and not without it: SC-01 … SC-09 | FR5, NFR3, NFR6 | host | `unit_tests.c` | all PASS |
| S5-H03 | statechart: START, barcode at junction, stale DONE, STOP, faults, Continue / Bypass / Reverse decisions, HALTED for mission failures, rescan, START after HALTED, the WATCH stop flag, latency timestamp: VC-01 … VC-24 | FR1–FR7 | host | `unit_tests.c` | all PASS |
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
