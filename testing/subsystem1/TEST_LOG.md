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
| S1-U01 | "1 START" accepted | FR6 | auto | `subsystem1_test.c` | `[TEST] S1-U01 PASS` |
| S1-U02 | repeated sequence number refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U03 | unknown verb refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U04 | SET outside its range refused | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U05 | SET refused while running | FR6 | auto | `subsystem1_test.c` | PASS |
| S1-U06 | SET stop_mm 300 accepted | FR6, IF-08 | auto | `subsystem1_test.c` | PASS |
| S1-U07 | parameters start unset (owners keep their defaults) | IF-08 | auto | `subsystem1_test.c` | PASS |
| S1-U08 | telemetry fits in 200 bytes with the largest values | NFR9 | auto | `subsystem1_test.c` | PASS |
| S1-U09 | heartbeat fits in 200 bytes | NFR9 | auto | `subsystem1_test.c` | PASS |
| S1-U10 | bridge buffers and telemetry_q exist | IF-07, IF-11 | auto | `subsystem1_test.c` | PASS |
| S1-H01 | command parser: CMD-01 … CMD-12 (case, CR LF, repeats, trailing words, ranges, length) | FR6 | host | `unit_tests.c` | all PASS |
| S1-H02 | telemetry formats: TEL-01 … TEL-05 (≤ 200 B worst case, escaping, refusal not truncation) | NFR9 | host | `unit_tests.c` | all PASS |
| S1-M01 | MQTT connects and publishes | FR6 | manual | 1. Put SSID, password, broker IP in `config/wifi_secrets.h`. 2. `make NET=lwip MQTT=1`. 3. Subscribe to `car/g07/#` in MQTT Explorer. | `status` every 1 s, `telemetry` every 0.5 s |
| S1-M02 | command round trip | FR6, IF-11 | manual | Publish `1 START`, `2 STOP` to `car/g07/cmd` with the car in READY on blocks. | `event` shows `{"ev":"cmd","seq":1,"ok":1…}`; wheels start, then stop |
| S1-M03 | WiFi loss does not stop the run | FR6, NFR4 | manual | Car running on blocks; switch the access point off for 20 s, then on. | car keeps running; `conn` in `status` goes up by 1 after reconnecting |
| S1-M04 | message rate under load | A1 | manual | Run 5 min with telemetry; read `txd`, `rxd`, `nerr` in `status`. | all stay 0 (or record the count) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**host** = laptop unit test, result printed by `testing/host/unit_tests`;
**manual** = a person observes the result (meter, scope, ruler, phone app…).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | S1-H01 … | working tree | host | PASS | `testing/host/logs/2026-10-14_unit.txt` | laptop run of the logic modules; on-target rows to be added after flashing |
