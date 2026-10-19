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
| INT-08 | every message ≤ 200 B, no queue overflow in any scenario | IF-07, IF-11, NFR9 | sim | all scenarios | 0 oversize, 0 overflow |
| INT-09 | dist_snapshot read by line, IMU, VC and telemetry | IF-05 | manual | Run 2 min; watch `d` in telemetry and the `[IMU]` hump lines. | `d` grows smoothly; no `ERROR` lines |
| INT-10 | GPIO interrupt shared by encoders, barcode, echo | IF-12, A6 | manual | Drive at 300 mm/s over a barcode with the ultrasonic pinging. | barcode decodes, `rng` valid, encoder `enc` counts both wheels |
| INT-11 | MQTT bridge end to end | IF-11 | manual | S1-M01 and S1-M02 together while the car runs | commands accepted while telemetry flows |
| INT-12 | robustness: the same scenarios with the car's physics changed (motor strength, deadband, track width, counts/mm, left/right mismatch) | all | sim | `sim -p SEED DB KFF TRACK COUNTS RIGHT` (see `testing/host/README.md`) | all scenarios still pass |
| INT-13 | obstacle stop by Obstacle Scan (feedback IS32): ALERT → CMD_STOP set by Scan → duty 0 → STOP_ACK | IF-03, IF-06, NFR3 | manual | Drive at 200 mm/s toward a box 10 times; read `st` and `sd` in the `tim` event | no contact; `st` under 100 ms (expected well under 1 ms); stops at least 150 mm from the box |
| INT-14 | task timing under full load (feedback IS32: WCETs and deadlines) | IF-14, A7, NFR2 | manual | Full run with MQTT on; read the `tim` events | `mc` ≪ 20 000 µs, `ln` ≪ 5 000 µs, `ov` = 0, `irq` × `irqm` under 5 % of a second, `vq` < 32, `dr` = 0 |
| INT-15 | steering latency (feedback IS32) | IF-02, IF-10 | manual | Follow a curved loop; read `sl` | `sl` of a few ms at most (was up to 20 ms before holds were applied at once) |
| INT-16 | mission failure is not a fault (feedback IS32): lift the car off the line while following | IF-04 | manual | wait for the search to fail | HALTED with `why` 6, no fault bit; START carries on |

*Method:* **auto** = `TEST_CHECK` in code; **sim** = host simulator; **manual** = on the car.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-14 | INT-01 … INT-08 | working tree | host | PASS | `testing/host/logs/2026-10-14_sim.txt` | 99 checks |
| 2026-10-14 | INT-12 | working tree | host | PASS | `testing/host/logs/2026-10-14_robustness.txt` | 48 random cars × 99 checks, 0 failed |
| 2026-10-15 | INT-01 … INT-08 | working tree | host | PASS | `testing/host/logs/2026-10-15_sim.txt` | 98 checks, after the IS32 changes (dead end now ends in HALTED, START carries on) |
| 2026-10-15 | INT-12 | working tree | host | PASS | `testing/host/logs/2026-10-15_robustness.txt` | 48 random cars × 98 checks, 0 failed (seeds 11 and 23), after the IS32 changes |
