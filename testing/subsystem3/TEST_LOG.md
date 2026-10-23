# Subsystem 3 — test log

**Subsystem:** Line and Barcode (Buddy 3)  ·  **Student ID:** 2500337  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.
Three kinds of test are listed:

- **auto**: a `TEST_CHECK` in `subsystem3_test.c`, run on the Pico by the test image (`make TEST=1`);
- **host**: a check in `testing/host/unit_tests.c`, run on a laptop with `make` in `testing/host/` (the logic files only, no hardware);
- **manual**: a person measures or observes the result on the car.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | *A* decodes forwards (15 ms narrow bars) | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U02 | *A* decodes backwards | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U03 | *Z* decodes, no command | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U04 | command table A-D | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U05 | both black for 20 ms and 10 mm = junction | FR1 | auto | `subsystem3_test.c` | PASS |
| S3-U06 | no black for T_lost = line lost | FR1 | auto | `subsystem3_test.c` | PASS |
| S3-U07 | no barcode read while standing still | FR2 | auto | `subsystem3_test.c` | PASS |
| S3-U08 | GP6, GP7, GP27 are plain GPIO on this kernel build (I2C1 not started) | A6 | auto | `subsystem3_test.c` | PASS |
| S3-H01 | Code 39: all 43 characters both ways, 30 % speed change, misreads rejected: C39-01 … C39-07 | FR2 | host | `unit_tests.c` | all PASS |
| S3-H02 | capture: chatter, merged edges, line crossings, too many edges: BC-01 … BC-06 | FR2, NFR1 | host | `unit_tests.c` | all PASS |
| S3-H03 | line status, junction, line lost after T_lost of travel (never by standing still), steering rule as fractions of the speed and mm of travel (same path at any speed, LS-09): LC-01 … LC-05, LS-01 … LS-09 | FR1 | host | `unit_tests.c` | all PASS |
| S3-M01 | DO level over black | FR1 | manual | Hold each module 3 mm over the black line, then white; read `line` in telemetry (`python3 tools/mqtt_decode.py --broker <ip>`) or the start-up print. | black reads `B`; else change `LB_BLACK_LEVEL` / `LB_BAR_LEVEL` |
| S3-M02 | sensor thresholds at 3.3 V | FR1, A3 | manual | Turn each potentiometer until the module LED switches cleanly between the line and the floor. | clean switching 2–5 mm above the floor |
| S3-M03 | barcode read at speed | FR2, A3 | manual | Drive past the *A* and *Z* samples 10 times each at 150, 200, 250 mm/s. | ≥ 9/10 correct reads, never a wrong character |
| S3-M04 | junction and turn | FR1, FR2 | manual | Barcode A before a + junction. | car turns left and follows the branch |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S3-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
| 2026-10-15 | S3-H01 … S3-H03 | working tree | host | PASS | `testing/host/logs/2026-10-15_unit.txt` | after the IS32 changes (logic unchanged; overflow and re-send are in task_line.c) |
| 2026-10-23 | S3-H01 … S3-H03 (LC-01 … LC-05, LS-01 … LS-09) | working tree | host | PASS | `testing/host/logs/2026-10-23_unit.txt`, `2026-10-23_sim.txt` | steering as fractions of the speed and mm of travel, T_lost as travel; `line_gap` failed at 120 mm/s with the old fixed mm/s rule, passes at 80, 120 and 300 now |
