# Integration — test log

Tests that two or more subsystems work together: at least one test for every
interface in TEAM.md section 5.  **sim** tests run the real logic modules of
all five subsystems together in the closed-loop car simulator
(`testing/host/sim.c`, `make` in `testing/host/`); **manual** tests run on the car.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | line status → VC steering → motion commands → wheels (ovals R 200/300 mm, left/right, 120/200/300 mm/s) | IF-01, IF-02, IF-10 | sim | `oval_*` scenarios | never lost, axle within 25 mm of the line |
| INT-02 | barcode → pending command → junction → move + turn → follow the branch (A, B, C, D, Z, A read backwards) | IF-01, IF-02, FR2 | sim | `barcode_*` scenarios | correct branch, no Safe Stop or HALTED |
| INT-03 | ALERT → CMD_STOP → STOP_ACK → profile → bypass → find line → rejoin | IF-01, IF-02, IF-03, IF-06, FR5 | sim | `obstacle_*` scenarios | no collision, rejoins, detour < 450 mm |
| INT-04 | IMU terrain events → VC hump mode; hump heights reported | IF-01, IF-05, IF-07, FR4 | sim | `hump` scenario | 2 humps, heights within tolerance, line kept |
| INT-05 | fault → latch → Safe Stop → RESTART → READY → START | IF-03, IF-04, FR7 | sim | `dead_end`, `imu_fault` | motors off at once; restart only when the fault is gone |
| INT-06 | collision → impact → Safe Stop | IF-04, FR4, FR7 | sim | `impact` | Safe Stop within 300 ms of the crash |
| INT-07 | MQTT commands and SET values → VC / MC; STOP latency | IF-08, IF-01, IF-03 | sim | `stop_start` | duty 0 within 1 ms, SET used at the next START |
| INT-08 | every MQTT record fits its fixed binary size (≤ 34 B since CR1; was 200 B of JSON), no queue overflow in any scenario | IF-07, IF-11, NFR9 | sim | all scenarios | 0 oversize, 0 overflow |
| INT-09 | dist_snapshot read by line, IMU, VC and telemetry | IF-05 | manual | Run 2 min; watch `odo_mm` in telemetry and the `[IMU]` hump lines. | `odo_mm` grows smoothly; no `ERROR` lines |
| INT-10 | GPIO interrupt shared by encoders, barcode, echo | IF-12, A6 | manual | Drive at 300 mm/s over a barcode with the ultrasonic pinging. | barcode decodes, `range_mm` valid, `enc16` counts on both wheels |
| INT-11 | MQTT bridge end to end | IF-11 | manual | S1-M01 and S1-M02 together while the car runs | commands accepted while telemetry flows |
| INT-12 | robustness: the same scenarios with the car's physics changed (motor strength, deadband, track width, counts/mm, left/right mismatch) | all | sim | `sim -p SEED DB KFF TRACK COUNTS RIGHT` (see `testing/host/README.md`) | all scenarios still pass |
| INT-13 | obstacle stop by Obstacle Scan (feedback IS32): ALERT → CMD_STOP set by Scan → duty 0 → STOP_ACK | IF-03, IF-06, NFR3 | manual | Drive toward a box 10 times at `SET base_speed` 80, 120 and 200; read `stop_lat_max_us` and `stop_dist_mm` in the timing record and measure the gap | no contact; `stop_lat_max_us` under 100 ms (expected well under 1 ms); the car comes to rest about 150 mm (`stop_mm`) from the box at every speed, ± 40 mm (the stop starts earlier the faster it goes, and the car learns its own stopping distance from the first stop) |
| INT-14 | task timing under full load (feedback IS32: WCETs and deadlines) | IF-14, A7, NFR2 | manual | Full run with MQTT on; read the timing records (every 5 s) | `max_us` motion ≪ 20 000 µs, line ≪ 5 000 µs; `overruns` 0; `irq_per_s` × `irq_max_us` under 5 % of a second; `vq_high` < 32; `dropped` 0 |
| INT-15 | steering latency (feedback IS32) | IF-02, IF-10 | manual | Follow a curved loop; read `steer_lat_max_ms` in the timing record | a few ms at most (was up to 20 ms before holds were applied at once) |
| INT-16 | mission failure is not a fault (feedback IS32): lift the car off the line while following | IF-04 | manual | wait for the search to fail | HALTED with `why` search_failed (6), no fault bit; START carries on |
| INT-17 | minimum-bit change (Change Requirement 1): task timing before and after | IF-14, NFR2, CR1 | manual | Flash V12 (`cf0176a`, `make NET=lwip MQTT=1`), follow the course loop for 2 min with MQTT on and keep three of its JSON `tim` events; repeat with the min-bit build and keep three timing records decoded by `tools/mqtt_decode.py` | every task well inside its period; each task's max no more than about 20 µs above V12 (the emulator predicts +17 µs Motion Control, +11 µs IMU, +10 µs Obstacle Scan, `MIN_BIT_EVIDENCE.md` 4.3); overruns 0 in both |
| INT-18 | 16-bit and 8-bit wraps on the car (CR1, Version C wrap test) | IF-01, IF-05, IF-08, IF-09 | manual | In READY, send 300 SETs so the 8-bit command seq wraps: `for i in $(seq 1 300); do echo "$i SET base_speed 220"; sleep 0.2; done \| python3 tools/mqtt_decode.py --broker <broker>`; then follow the loop for 3 min (two wraps of the 16-bit ms times) | all 300 acks `ok` (none refused as a repeat after seq 255 → 0); no `HEALTH` fault; `sl` stays a few ms |
| INT-19 | binary MQTT end to end (CR1, Version C Test 4) | IF-11, CR1 | manual | `mosquitto_sub -t 'car/g32/#' -F '%t %x' \| python3 tools/mqtt_decode.py`; send START, STOP, SET and one out-of-range SET | every record decodes (telemetry 23 B, status 18 B, event 9 B); every command acknowledged; the bad SET refused with `range` |
| INT-20 | fault bits on the car (CR1, Version C Test 2) | IF-04, CR1 | manual | Hold a wheel (stall); swap one encoder's A and B wires (encoder); unplug the IMU; point the ultrasonic at open space; switch the access point off | each fault appears as its own name in `status.faults`; STALL, ENCODER and IMU latch (Safe Stop, RESTART needed); NO_ECHO and WIFI_DOWN show but never latch |
| INT-21 | interrupts with packed ISR data (CR1, Version C Test 1) | IF-12, CR1 | manual | Drive at 300 mm/s over a barcode with the ultrasonic pinging; read `irq_per_s` and `irq_max_us` in the timing record; STOP, then read the console line `[MC] CMD_STOP: duty 0 after … us, encoder errors L/R` | both encoder error counts 0; `irq_max_us` a few µs; the barcode decodes |

*Method:* **auto** = `TEST_CHECK` in code; **sim** = host simulator; **manual** = on the car.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | INT-01 … INT-08 | working tree | host | PASS | `testing/host/logs/2026-10-14_sim.txt` | 99 checks |
| 2026-10-14 | INT-12 | working tree | host | PASS | `testing/host/logs/2026-10-14_robustness.txt` | 48 random cars × 99 checks, 0 failed |
| 2026-10-15 | INT-01 … INT-08 | working tree | host | PASS | `testing/host/logs/2026-10-15_sim.txt` | 98 checks, after the IS32 changes (dead end now ends in HALTED, START carries on) |
| 2026-10-15 | INT-12 | working tree | host | PASS | `testing/host/logs/2026-10-15_robustness.txt` | 48 random cars × 98 checks, 0 failed (seeds 11 and 23), after the IS32 changes |
| 2026-10-23 | INT-01 … INT-08 | `1be16d5` | host | PASS | `testing/host/logs/2026-10-23_min-bit_sim.txt` | 162 checks, minimum-bit build (CR1); INT-08 now checks every MQTT record is at most 34 B (was 200 B of JSON) |
| 2026-10-23 | INT-12 | `1be16d5` | host | PASS (same as V12) | `testing/host/logs/2026-10-23_min-bit_robustness.txt` | the 6 cars of V12's run 3 × 162 checks: the same single known failure as V12 (car 100 at 2× speed, `line_gap_fast`) |
| 2026-10-23 | INT-01 … INT-08 | `762f29f` | host | PASS | `testing/host/logs/2026-10-23_min-bit_sim.txt` | 162 checks, after removing the JSON debug build and the status record's duplicate MQTT-up bit |
| 2026-10-23 | INT-12 | `762f29f` | host | PASS (same as V12) | `testing/host/logs/2026-10-23_min-bit_robustness.txt` | the same 6 cars × 162 checks: the same single known failure as V12 and as at `1be16d5` |
