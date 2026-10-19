# Team and subsystems

Autonomous line-following car: micro T-Kernel 3.0 on a Raspberry Pi Pico W
with a Cytron Robo Pico board. Five subsystems, seven application tasks.

It is the single place that says **who owns what**, so that five people can
work in one firmware image without clashing over pins, priorities, or kernel
objects. Keep it up to date.

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Telemetry and Command: MQTT topics, telemetry, heartbeat, command validation, SET parameters | | | `app_program/subsystem1/` | `testing/subsystem1/` |
| 2 | Motion Control: motors, encoders, per-wheel PI speed control, distance and turn moves, odometry, stop path | 2500518 | | `app_program/subsystem2/` | `testing/subsystem2/` |
| 3 | Line and Barcode: line sensing, junctions, line-following rule, Code 39 barcode capture and decoding, command table | 2500337 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | IMU and Terrain + Safety Monitor: pitch, humps and height, motion classes, impacts; fault latch, buttons, task health | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | Obstacle Scan + Vehicle Controller: forward watch, coarse and fine scans, obstacle profile; mission statechart, bypass, line recovery | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files, changed only by agreement of the team: `app_program/app_main.c`,
`app.h`, `app_common.c`, `app_evt.h/.c`, `app_msg.h`, `gpio_irq.c/.h`
(maintained by Buddy 2), the configuration changes listed in section 3, and
this file.

Each subsystem keeps its **logic** in pure C files (`*_core.c`, `code39.c`,
`os_profile.c`, `imu_math.c`, `cmd_parse.c`, `tel_json.c`: no kernel calls)
and its **RTOS side** in `task_*.c` and `drv_*.c`. The logic files are also
compiled on a laptop by `testing/host/` (unit tests and a car simulator).

## 2. Pins

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | S5 Obstacle Scan | HC-SR04 TRIG (Grove 1). Unplug for `CRASH_REPORT=1` builds |
| GP1 | S5 Obstacle Scan | HC-SR04 ECHO, edge interrupt (Grove 1). Unplug for `CRASH_REPORT=1` builds |
| GP2 | S2 Motion Control | left encoder A, edge interrupt (Grove 2) |
| GP3 | S2 Motion Control | left encoder B, edge interrupt (Grove 2) |
| GP4 | S4 IMU and Terrain | I2C0 SDA, GY-511 IMU (Grove 3) |
| GP5 | S4 IMU and Terrain | I2C0 SCL, GY-511 IMU (Grove 3) |
| GP6 | S3 Line and Barcode | left line sensor DO (Grove 5) |
| GP7 | S3 Line and Barcode | right line sensor DO (Grove 7) |
| GP8 | S2 Motion Control | M1A, right motor, PWM 4A |
| GP9 | S2 Motion Control | M1B, right motor, PWM 4B |
| GP10 | S2 Motion Control | M2A, left motor, PWM 5A |
| GP11 | S2 Motion Control | M2B, left motor, PWM 5B |
| GP12 | S5 Obstacle Scan | SG90 scan servo, PWM 6A (servo port 1) |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S2 Motion Control | right encoder A, edge interrupt (Grove 4) |
| GP17 | S2 Motion Control | right encoder B, edge interrupt (Grove 4) |
| GP18 | | (on-board NeoPixel, unused) |
| GP19 | | |
| GP20 | S4 Safety Monitor | START / RESTART button (on board, active low, pull-up) |
| GP21 | S4 Safety Monitor | STOP button (on board, active low, pull-up) |
| GP22 | | (on-board buzzer, unused) |
| GP26 | S3 Line and Barcode | barcode sensor AO, ADC0 (Grove 6). The GP26 wire of Grove 5 stays insulated |
| GP27 | S3 Line and Barcode | barcode sensor DO, edge interrupt (Grove 6) |
| GP28 | | (Grove 7 second wire, unconnected) |

## 3. Peripherals

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART1 | | | |
| I2C0 (kernel driver `iica`, 400 kHz) | S4 | IMU and Terrain | GP4, GP5 |
| I2C1 | | | (not started: only unit 0 is initialised) |
| SPI0 | | | |
| SPI1 | | | |
| ADC (kernel driver `adca`) | S3 | Line and Barcode | GP26 (ADC0) |
| PWM slices 4, 5 | S2 | Motion Control | GP8–GP11, 16 kHz |
| PWM slice 6 | S5 | Obstacle Scan | GP12, 50 Hz |
| GPIO bank interrupt (IO_IRQ_BANK0) | shared `gpio_irq.c` | enabled by Motion Control on core 1 | GP1, GP2, GP3, GP16, GP17, GP27 |
| TIMER (1 µs) | shared, read only | started in `usermain()` (`app_timer_init`) | — |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

Shared configuration changed for this project (team agreement):

| File | Change | Why |
|---|---|---|
| `config/config.h` | `CNF_TIMER_PERIOD` 10 → **1** ms | 5 ms and 10 ms tasks cannot be released on a 10 ms tick |
| `config/config_device.h` | `DEVCNF_USE_IIC` 0 → **1** | IMU on I2C0 |
| `device/i2c/sysdepend/rp2040/i2c_cnf_sysdep.h` | I2C0 SDA/SCL 8/9 → **4/5**, timeout 1000 → **20** ms | GP8/GP9 are Motor 1; a stuck bus must not block for 1 s |
| `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h` | `DEVCONF_ADC_PIN_INIT_0` → **TRUE** | barcode AO on GP26 |
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c` | telemetry bridge (IF-11) | to be agreed with the instructor (template rule for shared library code); still runs only inside `cyw43_utk_task` |
| `app_program/app.h` | `TASK_STACK_SIZE` 2048 → **4096** | JSON buffers and scan arrays on the stack |

### WiFi and MQTT

No application task calls `cyw43_...`, lwIP or MQTT functions. Telemetry and
Command reads the link state with `cyw43_utk_get_status()` (always safe) and
exchanges bytes with the kernel's MQTT code through two message buffers that
the MQTT code reads and writes with `TMO_POL` (IF-11). Build with
`make NET=lwip MQTT=1` and a `config/wifi_secrets.h`.

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4, both on
core 0). These values replace the template's default bands by team
agreement (design review, section 3).

| Subsystem | Tasks (name – priority – core – release) |
|---|---|
| 2 | `motion_task` – **6** – core 1 – 20 ms cyclic + immediate wake-up on a stop |
| 4 | `safety_task` – **7** – core 0 – waits on `fault_flags`, 20 ms timeout |
| 3 | `line_task` – **8** – core 1 – 5 ms cyclic |
| 5 | `vc_task` – **9** – core 1 – waits on `vc_event_q`, 20 ms tick |
| 5 | `scan_task` – **10** – core 0 – 10 ms cyclic |
| 4 | `imu_task` – **11** – core 0 – 10 ms cyclic |
| 1 | `telemetry_task` – **14** – core 0 – waits on `mqtt_rx_mbf`, 100 ms timeout |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1. In the single-core build
(`SMP=0`) every task runs on core 0 with the same priorities.

## 5. Interfaces between subsystems

The **provider** creates the kernel object and publishes its ID in its own
`subsystemN.h`; the **user** only sends to it or waits on it.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | all → S5 Vehicle Controller | message buffer, 32 × 16 B (owned by S5) | `vc_event_q` | `APP_MSG` (`app_evt.h`), sent with `TMO_POL` through `app_send_event(s)`. If it is full: a **safety** event (`evt_is_safety()`: FAULT, FAULT_CLEAR, IMPACT, ALERT, STOP_ACK, BUTTON, COMMAND) sets `FLT_QUEUE_OVF`; any other event is dropped and counted, and a dropped line status or terrain change is sent again next cycle. Peak backlog logged (`vq` in the `tim` event) | team |
| IF-02 | S5 VC → S2 | message buffer, 8 × 16 B, wrapped by the motion API (owned by S2) | `motion_cmd_q` | `motion_move_forward/backward(mm, speed, id)`, `motion_turn_left/right(deg, id)`, `motion_hold_speed(l, r, src_ms)`; a hold wakes Motion Control and is applied at once (`src_ms` = time of the line reading, for the latency log); `EV_DONE.p2` = id; `EV_STOP_ACK` = [latency µs, stopping distance mm] | team |
| IF-03 | S5 VC, S5 Scan, S4, S2 → S2 | event flag + `tk_wup_tsk` (owned by S2) | `stop_flags` | `MC_CMD_STOP`: set by VC, or by Obstacle Scan itself on a confirmed close range (`SCAN_STOP_FLAG`); cleared only by VC. `MC_FAULT_STOP`: Safety Monitor, Motion Control; cleared only by Safety Monitor. `motion_stop()`, `motion_fault_stop()` | team |
| IF-04 | all → S4 Safety Monitor | event flag (owned by S4) | `fault_flags` | one `FLT_...` bit per fault, `FLT_RESTART_REQ` (`safety_core.h`) | team |
| IF-05 | S2 → S1, S3, S4, S5 | shared record + mutex `TA_INHERIT` (owned by S2) | `mtx_snapshot` | `MC_SNAPSHOT` via `motion_get_snapshot()`, 2 ms lock timeout | team |
| IF-06 | S5 VC → S5 Obstacle Scan | message buffer, 4 × 12 B (owned by S5) | `scan_request_q` | `SCAN_REQ` {mode (+ `SCAN_STOP_FLAG` for WATCH: Scan sets CMD_STOP on a confirmed ALERT), stop distance, id} | S5 |
| IF-07 | S5, S4 → S1 | message buffer, 16 × 16 B (owned by S1) | `telemetry_q` | `APP_MSG` records; full = dropped (telemetry never delays control) | team |
| IF-08 | S1 → S2, S5 | shared array, single writer (owned by S1) | `param_get()` | SET values, accepted only in READY / FINISHED / HALTED, copied by owners at START | team |
| IF-09 | S2, S3, S4, S5 → S4 | shared array, one word per task (owned by S4) | `health_stamp()` | time of each task's last cycle | team |
| IF-10 | S3 → S5 VC | function interface (pure C) | `ls_update/ls_output`, `lb_steer_params` | the line-following rule: line status → wheel speeds | S3, S5 |
| IF-11 | S1 ↔ kernel `cyw43_utk_task` | 2 message buffers, `TMO_POL` on the kernel side (owned by S1) | `mqtt_tx_mbf`, `mqtt_rx_mbf` | 1-byte topic + JSON ≤ 200 B out; command text ≤ 64 B in (`net_bridge.c` + `mqtt_utk.c`) | team; instructor to confirm |
| IF-12 | S2, S3, S5 → shared | GPIO bank interrupt dispatcher | `gpio_irq_attach()` | per-pin edge callback with a 1 µs timestamp, runs on core 1 | team |
| IF-13 | S2, S3, S4, S5 → S1 | status getters, single writer each | `lb_get_status`, `imu_get_status`, `vc_get_status`, `scan_get_status`, `scan_profile_get`, `safety_latched_bits` | latest values for telemetry | team |
| IF-14 | every task → S1 | shared table, one writer per slot (`app_common.c`) | `app_task_time()`, `app_task_overrun()`, `app_timing_get()` | each task's longest run (µs) and missed releases, published every 5 s in the `tim` event | team |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

### Faults and stops (fault table, design-review feedback IS32)

| Kind | What | Result | Leave with |
|---|---|---|---|
| Safety fault (bit in `fault_flags`) | motor stall, move timeout, wheel turning against its duty (`FLT_MOTION`); collision (`FLT_IMPACT`); IMU not answering (`FLT_IMU`); ultrasonic not answering (`FLT_SCAN`); a safety event or motion command lost to a full queue (`FLT_QUEUE_OVF`); a task stopped running (`FLT_HEALTH`); wheels not stopping after CMD_STOP (`FLT_NAV`); barcode interrupt set-up failed at start-up (`FLT_LINE`) | Safety Monitor latches `FAULT_STOP`; VC `SAFE_STOP` | RESTART (bits clear for 100 ms), then START |
| Mission failure (no fault bit) | line not found, no way round after 2 retries, no profile after a rescan, obstacle during a bypass, barcode command or bypass too slow | VC stops with `CMD_STOP`; `HALTED` with the reason | START (the run carries on) |
| Counted only | barcode capture lost to an `edge_buf` overflow, non-safety event dropped from `vc_event_q`, telemetry dropped | the car carries on | — |

## 6. Kernel object budget

Limits are set in `config/config.h`.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 1 | 1 | 1 | 2 | 2 |
| Semaphore | 16 | | | | | |
| Event flag | 16 | | 1 | | 1 | |
| Mailbox | 12 | | | | | |
| Mutex | 12 | | 1 | | | |
| Message buffer | 12 | 3 | 1 | | | 2 |
| Fixed memory pool | 12 | | | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | | 1 | 1 | 1 | 1 |
| Alarm handler | 12 | | | | | |

Total: 7 tasks, 2 event flags, 1 mutex, 6 message buffers, 4 cyclic
handlers. (The ultrasonic echo is collected by polling every 10 ms, so the
design's `echo_done` flag is not needed. The kernel's USB and WiFi service
tasks and its I2C and ADC drivers use a few objects of their own.)
