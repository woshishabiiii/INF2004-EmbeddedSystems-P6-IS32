# Autonomous car firmware: build, bring-up and use

This repository is the course template with the car's firmware in
`app_program/`. The template's own guide (`README.md`) still applies:
toolchain, flashing, build options. This page covers the car.

## 1. What runs

| Core | Task | Pri | Buddy | Folder | Released |
|---|---|---|---|---|---|
| 1 | Motion Control | 6 | 2 | `subsystem2/` | every 20 ms, and at once on a stop |
| 0 | Safety Monitor | 7 | 4 | `subsystem4/` | on any fault bit, else every 20 ms |
| 1 | Line and Barcode | 8 | 3 | `subsystem3/` | every 5 ms |
| 1 | Vehicle Controller | 9 | 5 | `subsystem5/` | on every event, plus a 20 ms tick |
| 0 | Obstacle Scan | 10 | 5 | `subsystem5/` | every 10 ms |
| 0 | IMU and Terrain | 11 | 4 | `subsystem4/` | every 10 ms |
| 0 | Telemetry and Command | 14 | 1 | `subsystem1/` | on an MQTT command, else every 100 ms |

Each subsystem has two kinds of file:

- `*_core.c` (and `code39.c`, `os_profile.c`, `imu_math.c`, `cmd_parse.c`,
  `tel_json.c`): the **logic**, plain C with no kernel calls. These are the
  files you tune and reason about, and they are tested on a laptop
  (`testing/host/`).
- `task_*.c` and `drv_*.c`: the **RTOS side** (wait, read the hardware, call
  the logic, send the results) and the **drivers** (registers and pins).

Tuning values live in one header per subsystem, each with its unit:
`motion_cfg.h`, `line_cfg.h`, `imu_cfg.h`, `scan_cfg.h`, `vc_cfg.h`.
`MEASURE` marks a value you must measure on your own car; `TUNE` one you
adjust on the track.

## 2. Build

```sh
cd build_make
make -j8                          # the car; telemetry printed on the USB console
make TEST=1 -j8                   # + every subsystem's self-tests at power-up
make NET=lwip MQTT=1 -j8          # + WiFi and MQTT telemetry and commands
```

For MQTT, copy `config/wifi_secrets.example.h` to `config/wifi_secrets.h`
and fill in the SSID, password and broker IP. Change the topic prefix
`car/g07/` in `app_program/subsystem1/net_bridge.c` to your group number.

The single-core build (`make SMP=0`) also works: every task runs on core 0.

## 3. First power-up: what the console shows

Open the USB serial port (README section 2.4). Place the car **still and
level, centred on a straight line**, then power it up. Expect:

```
=== Autonomous car on micro T-Kernel, Pico W ... ===
[MC] ready: 2000 counts/m, 204 counts per 90 deg
[LB] barcode AO: mean ..., range ...
[LB] line sensors at start: left white, right white
[IMU] LSM303DLHC found; keep the car still and level for 1 s
[VC] STARTUP: waiting for the sensors, then READY (press START)
all subsystems started
[IMU] calibrated: forward ... mg, up ~1000 mg, |g| ~1000 mg, mounting tilt ...
[VC] -> READY
```

The motors stay braked (CMD_STOP set) until START. Without MQTT, one
`[TEL] car/g07/telemetry {...}` line appears every second.

If a sensor is not wired yet, set `IMU_REQUIRED` (imu_cfg.h) or
`OS_REQUIRED` (scan_cfg.h) to 0 so its failures are printed but do not stop
the car.

## 4. Bring-up order (do these once, in order)

Each step is a manual test in `testing/subsystemN/TEST_LOG.md`.

1. **Wheels on blocks.** Motor directions (S2-M01) and encoder signs
   (S2-M02): `MOTOR_DIR_*` in `drv_motor.c`, `ENC_SIGN_*` in `drv_encoder.c`.
   A wrong encoder sign is caught as fault `mcf 3` within 0.3 s.
2. **Counts per metre and track width** (S2-M03): `MC_COUNTS_PER_M`,
   `MC_TRACK_MM` in `motion_cfg.h`. Then distance and turn accuracy
   (S2-M04/M05): correct them without reflashing with
   `SET dist_cal` / `SET turn_cal` (1000 × commanded / measured).
3. **Speed loop** (S2-M06): `MC_KFF_X1000`, `MC_DEADBAND`, then `MC_KP_X1000`,
   `MC_KI_X1000` (or `SET kp_x1000` / `SET ki_x1000`).
4. **Line sensors** (S3-M01/M02): `LB_BLACK_LEVEL`, `LB_BAR_LEVEL`,
   potentiometers. Then follow a loop (SYS-01) and tune the `LS_*` steering
   values in `line_cfg.h`.
5. **Barcodes** (S3-M03): read *A* and *Z* both ways at three speeds.
6. **IMU axes** (S4-M01): `IMU_AXIS_*`, `IMU_SIGN_*`, then hump heights
   (S4-M02, `IMU_WHEELBASE_MM`) and the impact threshold (S4-M03).
7. **Ultrasonic and servo** (S5-M01…M03): measure ECHO's voltage before
   wiring GP1; calibrate `SERVO_US_AT_30DEG/150DEG` so that 30° points
   **right**; check ranges. Then measure the car for `vc_cfg.h`
   (`VC_CAR_WIDTH_MM`, `VC_US_TO_AXLE_MM`, `VC_LINE_TO_AXLE_MM`,
   `VC_REAR_CLEAR_MM`).
8. **MQTT** (S1-M01…M04).

## 5. Driving it

| Action | Button | MQTT (`car/g07/cmd`) |
|---|---|---|
| Start (from READY or FINISHED), or carry on after HALTED | GP20 | `<seq> START` |
| Stop (back to READY, motors held) | GP21 | `<seq> STOP` |
| Restart after a fault (Safe Stop → READY) | GP20 | `<seq> RESTART` |
| End the run, publish the summary | — | `<seq> FINISH` |
| Change a value (only in READY / FINISHED / HALTED; used from the next START) | — | `<seq> SET <name> <value>` |

`<seq>` is 1, 2, 3, … and must grow with every command (a repeat is
ignored); it starts again from 1 after the car restarts. `SET` names and
limits: `base_speed` 80–400 mm/s, `stop_mm` 150–600, `kp_x1000` 0–20000,
`ki_x1000` 0–50000, `dist_cal` 500–1500, `turn_cal` 500–1500.

### Topics (MQTT build)

| Topic | When | Example |
|---|---|---|
| `car/g07/telemetry` | every 0.5 s | `{"t":81520,"st":"FOLLOW","v":[212,208],"enc":[1520,1498],"d":1834,"ln":"WB","bc":"A","cmd":1,"p":-12,"mo":"MOVING","hmax":12,"rng":350,"yaw":-5,"flt":0}` |
| `car/g07/status` | every 1 s | `{"up":81,"st":"FOLLOW","why":0,"flt":0,"mcf":0,"imu":1,"hdg":-34,"pings":1204,"wifi":3,"mq":1,"conn":1,"txd":0,"rxd":0,"nerr":0}` |
| `car/g07/event` | on change | `{"t":…,"ev":"barcode","c":"A","cmd":1}`, `"state"`, `"nav"`, `"junction"`, `"hump"`, `"terrain"`, `"motion"`, `"profile"`, `"impact"`, `"fault"`, `"cmd"` (acknowledgement), `"summary"` |
| `car/g07/event` | every 5 s | `{"t":…,"ev":"tim","mc":85,"ln":160,"vc":40,"sc":30,"im":260,"sm":12,"tl":900,"ov":0,"irq":820,"irqm":4,"vq":2,"dr":0,"sl":1,"st":35,"sd":14,"rc":7400}` (illustrative values) |

Field meanings: `st` state, `v` wheel speeds mm/s, `enc` encoder counts,
`d` distance travelled mm, `ln` line sensors (left, right; B = black),
`bc` last barcode, `cmd` waiting barcode command (1 left, 2 right,
3 straight, 4 U-turn), `p` pitch in tenths of a degree, `mo` motion class,
`hmax` highest hump mm, `rng` forward range mm (-1 = none), `yaw` turn rate
deg/s, `flt` latched fault bits, `why` reason for the last SAFE_STOP or
HALTED, `mcf` Motion Control fault (1 stall, 2 timeout, 3 encoder
direction).

`tim` (timing and queue health, design-review feedback IS32): `mc ln vc sc
im sm tl` longest run of each task in µs (Motion, Line, Vehicle Controller,
Scan, IMU, Safety Monitor, Telemetry), `ov` releases missed, `irq` GPIO
interrupts per second, `irqm` longest interrupt µs, `vq` most events
waiting in Vehicle Controller's inbox, `dr` events dropped, `sl` steering
latency ms (line reading → new wheel targets), `st` stop latency µs
(request → duty 0), `sd` last stopping distance mm, `rc` last MQTT
reconnect time ms. Compare `mc` and `ln` with their 20 ms and 5 ms periods.

Fault bits (safety faults: SAFE_STOP, leave with RESTART then START):
0x01 motion, 0x02 barcode interrupt set-up, 0x04 IMU, 0x08 impact,
0x10 ultrasonic not answering, 0x20 a safety event lost to a full queue,
0x40 task stopped, 0x80 wheels did not stop (`why` 3).

Mission failures are not faults: the car stops (HALTED) and `why` says why:
4 no scan result (after a rescan), 5 no way round, 6 line not found,
7 blocked during a bypass, 8/9 command or bypass too slow. Put the car back
on the line and press START to carry on.

## 6. Behaviour in one paragraph

After START the car follows the line with the two straddling sensors,
steering by slowing the inside wheel. A barcode beside the line is decoded
(either direction) and its command waits for the next junction (both
sensors black). There the car drives its axle onto the junction and turns
90° left (A) or right (B), drives straight across (C), or, for a U-turn
(D), spins 180° where it detected the junction, so it lands back on the
line it arrived on; with no junction within `VC_D_EXEC_MM` it acts where it
is. After any turn or bypass the line must show up within `VC_ACQUIRE_MM`,
otherwise the car searches at once. A line search (line lost for
`LB_T_LOST_MS`, or not reacquired) spins 120° toward the side the line was
last seen, then 240° the other way, then 120° back; the first black ends
it, and nothing found stops the car (HALTED, `why` 6). On a hump the car slows, drives straight on the encoders and ignores the
line until the IMU reports level again; the hump's height is reported when
the car is past it. When the ultrasonic sees something within `stop_mm`
twice in a row, the Obstacle Scan task stops the car itself, then the
controller scans 30°–150° (coarse, then 5° steps around the obstacle), and
either continues (obstacle beside the path), drives round it on the side
with room and the smaller detour, then finds and lines up with the line
again, or backs off and scans again (twice), then stops (HALTED). A safety
fault stops the motors at once (Safe Stop) until RESTART with the fault
gone.

## 7. Laptop tests and simulator

```sh
cd testing/host
make            # unit tests of every logic module + the closed-loop simulator
```

See `testing/host/README.md`. The simulator runs the real logic of all five
subsystems on a simulated car (oval tracks, barcodes and junctions,
obstacles, humps, faults, MQTT commands). It checks the logic, not your
hardware: the values marked MEASURE still have to be measured.
