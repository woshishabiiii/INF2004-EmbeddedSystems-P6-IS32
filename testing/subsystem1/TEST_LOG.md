# Subsystem 1 — test log

**Subsystem:** Telemetry and Command (Buddy 1)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem1/`

See `testing/README.md` for how to run tests and record results.
Three kinds of test are listed:

- **auto**: a `TEST_CHECK` in `subsystem1_test.c`, run on the Pico by the test image (`make TEST=1`);
- **host**: a check in `testing/host/unit_tests.c`, run on a laptop with `make` in `testing/host/` (the logic files only, no hardware);
- **manual**: a person measures or observes the result on the car.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | a 4-byte START command (seq 1) accepted | FR6 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | repeated sequence number refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U03 | unknown verb refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U04 | SET outside its range refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | SET refused while running | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U06 | SET stop_mm 300 accepted | FR6, IF-08 | auto | `subsystem1_test.c` | PASS |
| S1-U07 | parameters start unset (owners keep their defaults) | IF-08 | auto | `subsystem1_test.c` | PASS |
| S1-U08 | telemetry is a 23-byte binary record | NFR9, CR1 | auto | `subsystem1_test.c` | PASS |
| S1-U09 | status is an 18-byte binary record | NFR9, CR1 | auto | `subsystem1_test.c` | PASS |
| S1-U10 | bridge buffers and telemetry_q exist | IF-07, IF-11 | auto | `subsystem1_test.c` | PASS |
| S1-H01 | command parser, 4-byte binary commands: CMD-01 … CMD-13 (repeats and the 8-bit sequence wrap, unknown verbs and parameters, ranges, SET only while stopped, wrong length, spare bits) | FR6 | host | `unit_tests.c` | all PASS |
| S1-H02 | binary MQTT records: TEL-01 … TEL-09 (every record's exact size and fields, saturation outside a range, no spare byte, refusal when the buffer is too small) | NFR9, CR1 | host | `unit_tests.c` | all PASS |
| S1-H03 | queue policy: EVT-01 (which events are safety events and must arrive) | IF-01 | host | `unit_tests.c` | PASS |
| S1-M01 | MQTT connects and publishes | FR6 | manual | 1. Put SSID, password, broker IP in `config/wifi_secrets.h`. 2. `make NET=lwip MQTT=1`. 3. Run `python3 tools/mqtt_decode.py --broker <ip>` (it subscribes to `car/g32/#` and decodes every record). | `status` every 1 s, `telemetry` every 0.5 s |
| S1-M02 | command round trip | FR6, IF-11 | manual | With the car in READY on blocks, type `1 START`, then `2 STOP` into `python3 tools/mqtt_decode.py --broker <ip>` (it sends each as a 4-byte command to `car/g32/cmd`). | an `ack` with `seq` 1 and `ok` 1, then `seq` 2; wheels start, then stop |
| S1-M03 | WiFi loss does not stop the run | FR6, NFR4 | manual | Car running on blocks; switch the access point off for 20 s, then on. | car keeps running; `connects` in `status` goes up by 1 after reconnecting |
| S1-M04 | message rate under load | A1 | manual | Run 5 min with telemetry; read `tx_drops`, `rx_drops`, `net_errors` in `status`. | all stay 0 (or record the count) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S1-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
| 2026-10-15 | S1-H01 … S1-H03 | working tree | host | PASS | `testing/host/logs/2026-10-15_unit.txt` | after the IS32 changes (timing message, queue policy, SET also in HALTED) |
