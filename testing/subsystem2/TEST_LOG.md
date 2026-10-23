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
| S2-U09 | encoder pins GP2, GP3, GP16, GP17 are plain GPIO on this kernel build (feedback IS32: nothing reconfigures GP16) | A6 | auto | `subsystem2_test.c` (also prints GP25's function) | PASS |
| S2-H01 | moves, turns, stop latch, stall, encoder direction, tuning, a hold between ticks applied at once, stop latency and distance logged, smooth start (MC-18) and coasting stop (MC-19), a stall caught while the speed hold is refreshed (MC-20), stopping rate learned from the car's own stops (MC-21), coasting that does not slow the car turns into braking (MC-22): MC-01 … MC-22 | FR3, FR7, NFR3 | host | `unit_tests.c` | all PASS |
| S2-M01 | each wheel turns forward for a positive duty | FR3 | manual | Car on blocks; START with base_speed 100; watch both wheels. | both roll forward; else set `MOTOR_DIR_*` = -1 in `drv_motor.c` |
| S2-M02 | encoder counts rise rolling forward | FR3 | manual | Push the car forward 200 mm by hand; read `enc16` in telemetry (`python3 tools/mqtt_decode.py --broker <ip>`). | both counts increase; else set `ENC_SIGN_*` = -1 |
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
| 2026-10-15 | S2-H01 (MC-01 … MC-17) | working tree | host | PASS | `testing/host/logs/2026-10-15_unit.txt` | after the IS32 changes: holds applied between ticks, steering and stop latency logged |
| 2026-10-23 | S2-H01 (MC-01 … MC-22) | working tree | host | PASS | `testing/host/logs/2026-10-23_unit.txt`, `2026-10-23_sim.txt`, `2026-10-23_robustness.txt` | no fixed times or speeds in the behaviour: stall caught while line following (MC-20 failed on the old code, fixed), learned stopping rate, coast bound; 12 random cars, all 140 simulator checks |
| 2026-10-23 | S2-H01 (MC-01 … MC-22; MC-16 now: a hold between ticks starts at once as a short ramp) | working tree | host | PASS | `testing/host/logs/2026-10-23_unit.txt`, `2026-10-23_sim.txt` | V12 smooth steering: travel speed (faster wheel) ramped, each wheel changes by at most 8 % of the travel speed per mm; simulator at 140 mm/s: largest sudden wheel-target change 31 mm/s (V11: 52), outside wheel never above base (V11: +19 mm/s); robustness 5 of 6 random cars pass everything, car 100 fails only `line_gap_fast` at 300 mm/s (see `2026-10-23_robustness.txt`) |
