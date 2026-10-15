# Subsystem 2 — test log

**Subsystem:** Motion Control (Buddy 2)  ·  **Student ID:** 2500518  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.
Three kinds of test are listed:

- **auto**: a `TEST_CHECK` in `subsystem2_test.c`, run on the Pico by the test image (`make TEST=1`);
- **host**: a check in `testing/host/unit_tests.c`, run on a laptop with `make` in `testing/host/` (the logic files only, no hardware);
- **manual**: a person measures or observes the result on the car.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | zero distance refused | FR3 | auto | `subsystem2_test.c` | PASS |
| S2-U02 | speed above the limit refused | FR3 | auto | `subsystem2_test.c` | PASS |
| S2-U03 | turn over 360 degrees refused | FR3 | auto | `subsystem2_test.c` | PASS |
| S2-U04 | speed command accepted | FR3 | auto | `subsystem2_test.c` | PASS |
| S2-U05 | CMD_STOP is set at start-up (READY) | NFR3 | auto | `subsystem2_test.c` | PASS |
| S2-U06 | control loop runs every 20 ms | NFR2 | auto | `subsystem2_test.c` | PASS |
| S2-U07 | both motors braked while stopped | NFR3 | auto | `subsystem2_test.c` | PASS |
| S2-U08 | counts per 90 degrees is positive | FR3 | auto | `subsystem2_test.c` | PASS |
| S2-H01 | moves, turns, stop latch, stall, encoder direction, tuning: MC-01 … MC-15 | FR3, NFR3 | host | `unit_tests.c` | all PASS |
| S2-M01 | each wheel turns forward for a positive duty | FR3 | manual | Car on blocks; START with base_speed 100; watch both wheels. | both roll forward; else set `MOTOR_DIR_*` = -1 in `drv_motor.c` |
| S2-M02 | encoder counts rise rolling forward | FR3 | manual | Push the car forward 200 mm by hand; read `enc` in telemetry. | both counts increase; else set `ENC_SIGN_*` = -1 |
| S2-M03 | counts per metre | FR3, A2 | manual | Roll the car exactly 1 m along a tape; read the counts. | set `MC_COUNTS_PER_M` to the mean of 3 runs |
| S2-M04 | distance accuracy | FR3 | manual | Command 1000 mm 5 times (`motion_move_forward`); measure. | within ±10 mm, else `SET dist_cal` |
| S2-M05 | turn accuracy | FR3 | manual | Command 90° left and right 5 times each; measure with a protractor or the IMU heading. | within ±3°, else `SET turn_cal` |
| S2-M06 | speed control step response | FR3 | manual | `motion_hold_speed(200,200)` from rest; log `v` at 20 ms (console). | reaches 200 ±20 mm/s within 0.6 s, no oscillation; tune `MC_KP/KI` |
| S2-M07 | stop latency | NFR3, A7 | manual | Press STOP (GP21) while driving; read the `[MC] CMD_STOP: duty 0 after … us` line. | under 100 ms (expected well under 1 ms) |
| S2-M08 | stall fault | FR7 | manual | Hold both wheels while driving. | `[MC] FAULT 1`, Safe Stop within 0.6 s |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S2-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
