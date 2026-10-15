# Subsystem 4 — test log

**Subsystem:** IMU and Terrain + Safety Monitor (Buddy 4)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem4/`

See `testing/README.md` for how to run tests and record results.
Three kinds of test are listed:

- **auto**: a `TEST_CHECK` in `subsystem4_test.c`, run on the Pico by the test image (`make TEST=1`);
- **host**: a check in `testing/host/unit_tests.c`, run on a laptop with `make` in `testing/host/` (the logic files only, no hardware);
- **manual**: a person measures or observes the result on the car.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | atan2 within 0.02 degree | FR4 | auto | `subsystem4_test.c` | PASS |
| S4-U02 | sine table within 0.0003 | FR4 | auto | `subsystem4_test.c` | PASS |
| S4-U03 | a fault latches FAULT_STOP; then only RESTART is waited for | FR7 | auto | `subsystem4_test.c` | PASS |
| S4-U04 | RESTART clears the flags, latch still on | FR7 | auto | `subsystem4_test.c` | PASS |
| S4-U05 | a fault raised again cancels the restart | FR7 | auto | `subsystem4_test.c` | PASS |
| S4-U06 | faults clear for 2 polls: latch released | FR7 | auto | `subsystem4_test.c` | PASS |
| S4-U07 | IMU answers and has calibrated | FR4, A4 | auto | `subsystem4_test.c` (car still and level) | PASS |
| S4-U08 | |g| at rest is 900..1100 mg | FR4 | auto | `subsystem4_test.c` | PASS |
| S4-H01 | maths, calibration, braking is not a slope, hump height, collision vs bump, motion classes, read failures: IMU-01 … IMU-14 | FR4, FR7 | host | `unit_tests.c` | all PASS |
| S4-H02 | buttons, health, START-as-RESTART: SM-01 … SM-05 | FR6, FR7 | host | `unit_tests.c` | all PASS |
| S4-M01 | axes and signs | FR4 | manual | Car level: `up` ≈ +1000 mg. Nose up 20°: `forward` positive. Set `IMU_AXIS_*` / `IMU_SIGN_*`. | pitch `p` in telemetry ≈ +200 (tenths of a degree) |
| S4-M02 | hump height | FR4, A4 | manual | Drive over humps of known height (10, 20, 30 mm) 5 times each. | reported within ±30 %; record the error; tune `IMU_WHEELBASE_MM` |
| S4-M03 | impact threshold | FR4, FR7 | manual | Drive slowly into a soft obstacle; drive over each hump. | collision gives Safe Stop; humps never do |
| S4-M04 | buttons | FR6 | manual | Press GP20 in READY, GP21 while running, GP20 in Safe Stop. | START, STOP, RESTART |
| S4-M05 | task health | FR7 | manual | Unplug the IMU while READY. | `FLT_IMU` (0x04) in `status`, Safe Stop; replug + RESTART clears it |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S4-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
