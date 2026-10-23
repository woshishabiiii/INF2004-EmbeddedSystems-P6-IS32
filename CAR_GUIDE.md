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
  `tel_bin.c`): the **logic**, plain C with no kernel calls. These are the
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
and fill in the SSID, password and broker IP. The topic prefix is
`car/g32/` (`app_program/subsystem1/net_bridge.c`).

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

The motors stay braked (CMD_STOP set) until START. **Before every START,
leave the car standing still on the floor for 2 seconds**: in READY the
IMU re-learns "level" from the floor it stands on (the decoded telemetry
`pitch_cdeg` goes to about 0), so a car that was tilted at power-up is fine.
Without MQTT, one `[TEL] car/g32/telemetry <hex bytes>` line appears every
second; paste the bytes into `python3 tools/mqtt_decode.py --hex "..."`.

If a sensor is not wired yet, set `IMU_REQUIRED` (imu_cfg.h) or
`OS_REQUIRED` (scan_cfg.h) to 0 so its failures are printed but do not stop
the car.

## 4. Bring-up order (do these once, in order)

Each step is a manual test in `testing/subsystemN/TEST_LOG.md`.

1. **Wheels on blocks.** Motor directions (S2-M01) and encoder signs
   (S2-M02): `MOTOR_DIR_*` in `drv_motor.c`, `ENC_SIGN_*` in `drv_encoder.c`.
   A wrong encoder sign is caught as fault 0x100 (encoder) within 0.3 s.
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

| Action | Button | MQTT (`car/g32/cmd`, typed into `tools/mqtt_decode.py`) |
|---|---|---|
| Start (from READY or FINISHED), or carry on after HALTED | GP20 | `<seq> START` |
| Stop (back to READY, motors held) | GP21 | `<seq> STOP` |
| Restart after a fault (Safe Stop → READY) | GP20 | `<seq> RESTART` |
| End the run, publish the summary | — | `<seq> FINISH` |
| Change a value (only in READY / FINISHED / HALTED; used from the next START) | — | `<seq> SET <name> <value>` |

MQTT messages are **binary** (minimum-bit data): the car takes 4-byte
commands and publishes compact records, so use the laptop tool rather than
typing text into an MQTT client:

```sh
python3 tools/mqtt_decode.py --broker 192.168.1.10   # live: decodes every record, sends typed commands
python3 tools/mqtt_decode.py --cmd "16 SET base_speed 220"   # prints the 4 bytes: 1005dc00
python3 tools/mqtt_decode.py --hex "<bytes from the console>"   # decode one record
```

The car sends binary only, as Change Requirement 1 asks. Without Python,
`mosquitto_sub -h <broker> -t 'car/g32/#' -F '%t %x'` still shows every
record in hex; the bit layouts are at the top of
`app_program/subsystem1/tel_bin.h`.

A command is 32 bits, little-endian: `seq` 8 | verb 3 (1 START, 2 STOP,
3 RESTART, 4 FINISH, 5 SET) | name 3 | 2 spare bits (must be 0) | value 16.
`<seq>` is 1, 2, 3, … and wraps after 255: a command is new if its `seq`
is 1–127 ahead of the last accepted one (mod 256), so a repeat or a delayed
message is ignored. It starts again from 1 after the car restarts. `SET`
names and limits: `base_speed` 80–400 mm/s, `stop_mm` 150–600, `kp_x1000`
0–20000, `ki_x1000` 0–50000, `dist_cal` 500–1500, `turn_cal` 500–1500.

`base_speed` is the car's **speed limit**: hump, junction, bypass and
line-finding speeds are all capped at it, so `SET base_speed 80` slows the
whole car. `stop_mm` is where the car should come to **rest** in front of
an obstacle; the stop starts earlier the faster the car is going (see
section 6).

### Topics (MQTT build)

Every record starts with a 32-bit header: version 4 bits, record type
4 bits, uptime ms 24 bits (wraps every 4.66 h). The exact layouts are in
`app_program/subsystem1/tel_bin.h`; without the MQTT build the same bytes
are printed on the console as hex.

| Topic | When | Record (bytes) | Content |
|---|---|---|---|
| `car/g32/telemetry` | every 0.5 s | telemetry (23) | state, wheel speeds, encoder positions (low 16 bits), distance, line sensors, last barcode, pending command, pitch, motion class, highest hump, range, turn rate, faults |
| `car/g32/status` | every 1 s | status (18) | state, `why`, faults, Motion Control fault bits, IMU ok, heading, pings, WiFi link status, MQTT counters, last reconnect time (MQTT up or down is the `WIFI_DOWN` fault bit) |
| `car/g32/event` | on change | event (9) | state, nav, barcode, junction, hump, terrain, motion, impact, fault: event type and its 32-bit payload |
| `car/g32/event` | per command | ack (6) | `seq`, accepted, error code |
| `car/g32/event` | after a scan | profile (17) | the obstacle profile |
| `car/g32/event` | at FINISH | summary (16) | run time, distance, highest hump, humps, commands, obstacles, barcode reads ok/failed |
| `car/g32/event` | every 5 s | timing (34) | see below |

The decoder prints each record with named fields. Notes: `range_mm` 4095
means no echo; `pending` is the waiting barcode command (turn left/right,
straight, U-turn); `why` is the reason for the last SAFE_STOP or HALTED;
the Motion Control fault field has one bit each for stall, move timeout and
encoder direction.

Timing (design-review feedback IS32): the longest run of each task in µs
(Motion, Line, Vehicle Controller, Scan, IMU, Safety Monitor, Telemetry),
releases missed, GPIO interrupts per second, longest interrupt µs, most
events waiting in Vehicle Controller's inbox, events dropped, steering
latency ms (line reading → new wheel targets), stop latency µs (request →
duty 0), last stopping distance mm, last MQTT reconnect time ms. Compare
Motion and Line with their 20 ms and 5 ms periods.

Fault bits (safety faults: SAFE_STOP, leave with RESTART then START):
0x001 motor stall, 0x002 barcode interrupt set-up, 0x004 IMU, 0x008 impact,
0x010 ultrasonic not answering, 0x020 a safety event lost to a full queue,
0x040 task stopped, 0x080 wheels did not stop (`why` 3), 0x100 wheel
turning against its duty (encoder or motor backwards), 0x200 move timeout.
On the wire the 12-bit fault field adds two status bits that never latch
and never stop the car: 0x400 no echo on the last ping, 0x800 MQTT link down.

Mission failures are not faults: the car stops (HALTED) and `why` says why:
4 no scan result (after a rescan), 5 no way round, 6 line not found,
7 blocked during a bypass, 8/9 a move or turn of a barcode command or
bypass never finished (its watchdog: 2 × its own expected time + a margin). Put the car back
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
`LB_LOST_MM` of travel, or not reacquired) spins on the spot: 180° toward the
side the line was last seen, then all the way back round to 180° the
other side (the whole circle; `VC_SEARCH_STEP_DEG`,
`VC_SEARCH_MAX_DEG` in vc_cfg.h); the first black ends it, and nothing found stops the car (HALTED, `why` 6). On a hump the car slows and keeps following the line; only while both
line sensors read black (lifted off the floor) does it drive straight on.
The hump's height is reported when the car is past it. When the ultrasonic sees something inside `stop_mm` plus the
distance the car still needs to stop at its measured speed (twice in a
row), the Obstacle Scan task stops the car itself, so it comes to rest
about `stop_mm` away at any speed; then the
controller scans 30°–150° (coarse, then 5° steps around the obstacle), and
either continues (obstacle beside the path), drives round it on the side
with room and the smaller detour, then finds and lines up with the line
again, or backs off and scans again (twice), then stops (HALTED). A safety
fault stops the motors at once (Safe Stop) until RESTART with the fault
gone.

### Nothing in the behaviour is a fixed time or speed

Every decision comes from what the car measures:

- **Steering**: corrections are a fraction of the speed and "how long" is
  millimetres of travel, so the car follows the same path at any speed.
- **Line lost**: after `LB_LOST_MM` of travel without the line, not after
  a time.
- **Barcode commands, line search, bypass**: no overall time limit; each
  move or turn has a watchdog of 2 × its own expected time (its distance or
  angle ÷ its speed) + a margin.
- **Obstacle stop**: starts at `stop_mm` + the distance covered while the
  pings confirm + the car's own stopping distance, learned from its real
  stops.
- **Impact**: a jolt plus the measured speed collapsing, at any driving
  speed.
- **Hump**: the line is still followed on a hump whenever a sensor sees
  white (a lifted sensor reads black). Line loss is ignored for
  `VC_HUMP_GRACE_MM` of travel after a hump.
- **Level**: re-learned from the floor while the car stands still in
  READY, so a tilted power-up calibration does not matter. Slopes and
  humps are only measured while the car drives itself.

What is left are **calibration values** (`MEASURE`: counts per metre,
track, feed-forward, sensor positions) and **fault watchdogs** (a task or
sensor not answering).

### If the car is too fast or lurches

1. Calibrate `MC_COUNTS_PER_M` first (bring-up step 2): push the car
   exactly 1 m along a ruler and read the change of `enc` in the telemetry.
   If this value is too high, the car thinks it is slower than it is and
   drives faster than `base_speed`.
2. Then lower `base_speed` with `SET base_speed 80` (no reflash), or
   `VC_BASE_SPEED` in `vc_cfg.h`.
3. Starts and speed changes: `MC_ACCEL` (mm/s²), steering smoothness
   `MC_STEER_PM_MM` (how fast a wheel may change speed per mm travelled;
   lower = smoother, higher = sharper reaction), and `MC_DUTY_RISE`
   (largest increase of a wheel's drive per 20 ms) in `motion_cfg.h`.

## 7. Laptop tests and simulator

```sh
cd testing/host
make            # unit tests of every logic module + the closed-loop simulator
```

See `testing/host/README.md`. The simulator runs the real logic of all five
subsystems on a simulated car (oval tracks, barcodes and junctions,
obstacles, humps, faults, MQTT commands). It checks the logic, not your
hardware: the values marked MEASURE still have to be measured.
