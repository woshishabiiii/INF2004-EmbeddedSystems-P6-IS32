# How the car's code works

One section per buddy. Read **Part 0** first (everyone), then your own
section, then the sections of the buddies you exchange data with (the
"Talks to" line at the top of each section). File names are under
`app_program/` unless a path says otherwise. Numbers quoted here are the
defaults in the `*_cfg.h` headers; change them there, not in the code.

Companion pages: `CAR_GUIDE.md` (build, first power-up, bring-up order,
MQTT commands and topics) and `TEAM.md` (pins, priorities, interfaces).

---

## Part 0 — The whole picture (everyone)

### 0.1 Three kinds of file

Every subsystem is split the same way:

| Kind | Files | What is in it | Kernel calls? |
|---|---|---|---|
| **Logic** | `*_core.c`, `code39.c`, `os_profile.c`, `imu_math.c`, `cmd_parse.c`, `tel_json.c` | the algorithms: a step function that takes inputs and returns outputs | **none** — only `<stdint.h>` / `<stdbool.h>` |
| **Task** | `task_*.c` | the RTOS side: wait for the period or a message, read the inputs, call the logic, send the outputs | yes |
| **Driver** | `drv_*.c` | registers and pins of one device | yes |

Why: the logic files compile on a laptop too (`testing/host/`), so every
algorithm is unit-tested and run in a car simulator before it touches the
car. When the car misbehaves, you can usually reproduce it on the laptop.
The logic never "does" anything itself — it returns a list (duties, events,
actions) and the task carries it out. That also makes the logic easy to read.

All arithmetic is integer (the RP2040 has no floating-point unit and the
template does not link `libm`): speeds in mm/s, angles in degrees or
centidegrees (1/100°), accelerations in mg, duty in permille (1000 = 100 %).
Sines and arctangents come from small tables.

### 0.2 Start-up

`app_main.c` → `usermain()` runs once, in the kernel's initial task:

1. `app_timer_init()` starts the RP2040's 1 µs TIMER (encoder, barcode and
   echo edges are all timestamped with it).
2. **Phase 1:** `subsystem1_init()` … `subsystem5_init()` create every
   kernel object (message buffers, event flags, mutex, cyclic handlers).
3. **Phase 2:** `subsystemN_start()` create and start the tasks — Motion
   Control first, because it brakes the motors and enables the shared GPIO
   interrupt.
4. In a test build (`make TEST=1`) the on-target self-tests run here.
5. `usermain()` sleeps forever (returning would shut the kernel down).

Two phases matter on the dual-core build: a task started on the other core
may run immediately, so every object it might use must already exist.

### 0.3 The seven tasks

| Core | Task | Priority | Released | Buddy |
|---|---|---|---|---|
| 1 | Motion Control | 6 | every 20 ms + at once on a stop | 2 |
| 0 | Safety Monitor | 7 | any fault bit, else every 20 ms | 4 |
| 1 | Line and Barcode | 8 | every 5 ms | 3 |
| 1 | Vehicle Controller | 9 | every event + 20 ms tick | 5 |
| 0 | Obstacle Scan | 10 | every 10 ms | 5 |
| 0 | IMU and Terrain | 11 | every 10 ms | 4 |
| 0 | Telemetry and Command | 14 | MQTT command, else every 100 ms | 1 |

Lower number = more urgent. The kernel's own USB console (3) and WiFi/MQTT
(4) tasks run on core 0 above all of ours. Core 1 holds the control loop
(motors, line, decisions) so nothing network-related can delay it; core 0
holds the watchdog-like Safety Monitor, so a task stuck on core 1 cannot
silence it.

Periodic tasks are woken by **cyclic handlers** (`tk_cre_cyc`) that call
`tk_wup_tsk()`; the task sleeps in `tk_slp_tsk()`. The 5 ms and 10 ms tasks
then cancel any extra queued wake-ups (`tk_can_wup()`), so after a delay
they never "catch up" with a burst of runs.

### 0.4 How the tasks talk

| Object | Kind | From → to | Carries |
|---|---|---|---|
| `vc_event_q` | message buffer, 32 × 16 B | every task → Vehicle Controller | `APP_MSG` events |
| `motion_cmd_q` | message buffer, 8 × 16 B | Vehicle Controller → Motion Control | speed / distance / turn commands |
| `scan_request_q` | message buffer, 4 × 12 B | Vehicle Controller → Obstacle Scan | IDLE / WATCH / PROFILE requests |
| `telemetry_q` | message buffer, 16 × 16 B | VC, IMU, Scan → Telemetry | `APP_MSG` records to publish |
| `mqtt_tx_mbf`, `mqtt_rx_mbf` | message buffers | Telemetry ↔ kernel MQTT code | JSON out, command text in |
| `stop_flags` | event flag | VC, Safety → Motion Control | `CMD_STOP`, `FAULT_STOP` |
| `fault_flags` | event flag | any task → Safety Monitor | `FLT_...` bits, `FLT_RESTART_REQ` |
| `mtx_snapshot` | mutex (priority inheritance) | Motion Control → readers | the odometry snapshot |

**Every send uses `TMO_POL`** (never waits). A full queue to Vehicle
Controller or Motion Control is a fault (`FLT_QUEUE_OVF` → Safe Stop); a full
telemetry queue just drops the record — telemetry must never slow control.

The shared event record (`app_evt.h`) is 16 bytes:
`type, source, seq, t_ms, p1, p2`. The meaning of `p1`/`p2` per event type
is listed in `app_evt.h`; for example `EV_BARCODE` is `[character, command]`.

Small status structs (`LB_STATUS`, `IMU_STATUS`, `VC_STATUS`,
`SCAN_STATUS`) are read with getter functions. Each has exactly one writer,
and readers use them for telemetry and simple word-sized checks (is the IMU
calibrated? is the car in READY?), so no lock is needed. The motion
snapshot is different: several tasks use it for control and need all its
fields from the same moment, so it is copied under `mtx_snapshot`.

### 0.5 One run, end to end

1. **Power-up.** The car is braked (`CMD_STOP` starts set). The IMU
   calibrates for 1 s; Vehicle Controller goes STARTUP → READY.
2. **START** (GP20 button or `"<seq> START"` over MQTT). Telemetry (or Safety
   Monitor, for the button) sends `EV_COMMAND`/`EV_BUTTON` to Vehicle
   Controller. VC clears `CMD_STOP`, asks Obstacle Scan to WATCH, enters
   FOLLOW.
3. **Following.** Line and Barcode sends `EV_LINE_STATUS` whenever a
   sensor changes. VC runs the steering rule (Buddy 3's `ls_output`) and
   sends wheel speeds with `motion_hold_speed()`. Motion Control's PI loops
   hold those speeds.
4. **Barcode.** The barcode sensor's edges are timestamped by the GPIO
   interrupt; the Line task decodes the Code 39 character and sends
   `EV_BARCODE ['A', TURN_LEFT]`. VC stores it as *pending* and slows down.
5. **Junction.** Both line sensors black → `EV_JUNCTION`. VC drives the
   axle onto the junction (`motion_move_forward`), waits for `EV_DONE`,
   turns 90° (`motion_turn_left`), waits for `EV_DONE`, follows the branch.
6. **Obstacle.** Obstacle Scan sees something within 250 mm twice →
   `EV_ALERT`. VC stops (`motion_stop`), waits for `EV_STOP_ACK`, asks for a
   PROFILE scan, gets `EV_PROFILE`, decides Continue / Bypass / Reverse, then
   drives the bypass as a list of moves and turns, finds the line again and
   lines up with it.
7. **Hump.** The IMU sees the nose rise → `EV_TERRAIN CLIMBING`. VC drives
   straight on the encoders until `EV_TERRAIN LEVEL`; the IMU reports
   `EV_HUMP [height, highest]` when the car is past it.
8. **Fault.** Anything wrong → a task sets its bit in `fault_flags` →
   Safety Monitor latches `FAULT_STOP` (motors off within microseconds) and
   sends `EV_FAULT` → VC enters SAFE_STOP. RESTART clears it.
9. **FINISH** over MQTT → VC stops and sends `NAV_FINISH`; Telemetry
   publishes the run summary (time, distance, highest hump …).

Throughout, Telemetry publishes the state twice a second and every event as
it happens.

### 0.6 Shared files (team agreement)

`app.h` (task helpers, stack size), `app_evt.h/.c` (event record, event
numbers, the small `EVT_LIST` the logic modules fill), `app_msg.h` +
`app_common.c` (`app_send()`, `app_ms()`, `app_us()`, `app_timer_init()`),
`app_main.c`, and `gpio_irq.c/.h` (maintained by Buddy 2, see 2.6). Outside
`app_program/`, the team changed a few template files (listed in `TEAM.md`
section 3), most importantly `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`
(Buddy 1, see 1.5).

---

## Buddy 1 — Telemetry and Command (`subsystem1/`)

**Talks to:** everyone (reads their status), Vehicle Controller (sends it
commands), the kernel's MQTT code (through two buffers).

### 1.1 Files

| File | Kind | What it does |
|---|---|---|
| `task_telemetry.c` | task | the loop: commands in, records out, periodic messages |
| `cmd_parse.c/.h` | logic | parses and validates one command line |
| `tel_json.c/.h` | logic | builds every JSON message, never longer than 200 bytes |
| `net_bridge.c/.h` | glue | the functions the kernel's MQTT code calls: take a message to publish, put a received command |
| `subsystem1.c/.h` | setup | creates `telemetry_q`, `mqtt_tx_mbf`, `mqtt_rx_mbf`; starts the task; `param_get()` |
| `subsystem1_test.c` | test | on-target checks S1-U01…U10 |

### 1.2 The task loop (core 0, priority 14 — the least urgent)

```
forever:
  1. wait up to 100 ms on mqtt_rx_mbf      -> a command? handle_command()
  2. empty telemetry_q (no waiting)        -> one "event" message per record
  3. every 500 ms: gather() + telemetry message
     every 1 s:    gather() + status (heartbeat) message
```

Waiting on the command buffer (instead of sleeping 100 ms) means a remote
STOP is handled the moment it arrives. Being the lowest priority means
telemetry only ever uses spare CPU time.

`gather()` copies everyone's status into one `TEL_DATA`: the motion
snapshot (wheel speeds, encoder counts, distance, turn rate), line sensors
and last barcode, IMU pitch / motion class / highest hump, VC state and
pending command, forward range, latched fault bits, and the WiFi/MQTT
counters.

### 1.3 Commands (`cmd_parse.c`)

Format: `<seq> <VERB> [<name> <value>]`, for example `12 START`,
`16 SET base_speed 220`. Verbs: START, STOP, RESTART, FINISH, SET (any case).

Checks, in order, each with its own error code (`cmd_err_t`): length ≤ 64;
`seq` is a number; the verb is known; for SET the name is in the whitelist
and the value within its limits, and the car is not running (READY or
FINISHED only); no extra words; and finally **`seq` is larger than the last
accepted one** — so a repeated or delayed MQTT message is ignored. The
sequence check comes last so that a rejected command never "uses up" a
number.

No `sscanf`/`atoi`: plain loops, every index checked against the length
(Barr-C, and safe against any input).

Accepted START/STOP/RESTART/FINISH go to Vehicle Controller as
`EV_COMMAND [verb, value]`. Accepted SET values go into a small parameter
store (`s_params[]`); their owners read them with `param_get()` **at the
next START**, never in the middle of a run:

| Name | Limits | Owner |
|---|---|---|
| `base_speed` | 80–400 mm/s | Vehicle Controller |
| `stop_mm` | 150–600 mm | Vehicle Controller |
| `kp_x1000`, `ki_x1000` | 0–20000, 0–50000 | Motion Control |
| `dist_cal`, `turn_cal` | 500–1500 (1000 = no change) | Motion Control |

Every command, accepted or not, is answered on the event topic:
`{"ev":"cmd","seq":16,"ok":1,"err":0}`.

### 1.4 Messages (`tel_json.c`)

A tiny JSON builder (`tj_begin`, `tj_int`, `tj_str`, `tj_pair`, `tj_end`)
writes into a fixed buffer and remembers if anything did not fit — then the
message is dropped and counted, never truncated into broken JSON. No
`sprintf`. Numbers are clamped to a sensible width (`lim()`), so the
longest possible telemetry message is 198 bytes, under the 200-byte limit
(lwIP's MQTT output buffer is 256 bytes, shared with the topic).

Topics (prefix `car/g07/`, set in `net_bridge.c`):

| Topic | When | Content |
|---|---|---|
| `telemetry` | every 0.5 s | state, wheel speeds, encoder counts, distance, line sensors, last barcode, pending command, pitch, motion class, highest hump, range, turn rate, faults |
| `status` | every 1 s | uptime, state, last Safe Stop reason, faults, motion fault code, IMU ok, heading, pings, WiFi/MQTT state and counters |
| `event` | on change | state changes, navigation decisions, barcodes, junctions, humps, terrain, motion class, obstacle profile, impacts, faults, command acknowledgements, run summary |

Field meanings are in `CAR_GUIDE.md` section 5. When `NAV_FINISH` arrives,
the task also builds the **run summary** (time, distance, highest hump,
humps, commands done, obstacles passed, barcode reads ok/failed).

### 1.5 The network bridge — why it looks like this

The template has one rule for networking: **only the kernel's
`cyw43_utk_task` may call the CYW43 radio, lwIP or MQTT functions.** So
this subsystem never calls them. Instead:

- **Out:** `publish()` puts `[topic number][JSON]` into `mqtt_tx_mbf` with
  `TMO_POL`. In `mqtt_utk.c` (template file, changed by the team),
  `bridge_timeout()` runs every 20 ms inside the network task, takes up to 4
  messages via `net_bridge_take()` and calls `mqtt_publish()`. If lwIP's
  buffer is full (`ERR_MEM`) the message is kept and retried next time.
- **In:** on connect, `mqtt_utk.c` also subscribes to `car/g07/cmd`. A
  message on that topic goes to `net_bridge_put()`, which puts it in
  `mqtt_rx_mbf` (`TMO_POL`; too long or buffer full is counted, not
  blocked).
- `net_bridge_link()` keeps `net_up` / `net_connects` for the status topic.

`net_bridge.c` uses only plain C types in these functions because
`mqtt_utk.c` must not include the kernel headers (template rule).

In a build without `MQTT=1` nobody calls the bridge, and `publish()` prints
each message on the USB console instead (`[TEL] car/g07/telemetry {...}`),
less often so the console stays readable.

### 1.6 What to set and check

- Topic prefix `car/g07/` → your group (`net_bridge.c`).
- `config/wifi_secrets.h` (copy the example): SSID, password, broker IP.
- Status counters: `txd` (messages dropped because the network was slow),
  `rxd` (commands dropped), `nerr` (too long / refused). All should stay 0.

### 1.7 Tests

Host: `CMD-01…12` (case, CR LF, old sequence numbers, extra or missing
words, limits, exact names, SET while running, over-long and absurd input)
and `TEL-01…05` (every periodic message fits in 200 bytes with the largest
numbers, quotes escaped, a message that does not fit is refused, never cut
short). On target: S1-U01…U10. Manual: S1-M01…M04 in
`testing/subsystem1/TEST_LOG.md` (connect and publish, command round
trip, WiFi loss does not stop the run, message rate under load).

---

## Buddy 2 — Motion Control (`subsystem2/`)

**Talks to:** Vehicle Controller (receives commands, sends `EV_DONE` and
`EV_STOP_ACK`), Safety Monitor (fault stop), and everyone who reads the
odometry snapshot (Line, IMU, VC, Telemetry).

### 2.1 Files

| File | Kind | What it does |
|---|---|---|
| `mc_core.c/.h` | logic | speed PI loops, moves, stop latch, fault checks, snapshot |
| `motion_cfg.h` | config | calibration and tuning values |
| `task_motion.c` | task | reads encoders/stop bits, calls `mc_core_step()`, writes the motors |
| `motion_api.c` | API | the functions VC and Safety call |
| `drv_motor.c/.h` | driver | PWM for the two motors |
| `drv_encoder.c/.h` | driver | quadrature decoding of the two encoders |
| `subsystem2.c/.h` | setup | `stop_flags`, `motion_cmd_q`, `mtx_snapshot`, 20 ms cyclic handler, task on core 1 |
| `../gpio_irq.c/.h` | shared | the one GPIO interrupt, shared with Buddy 3 and 5 (you maintain it) |

### 2.2 The motion API (`motion_api.c`)

```c
motion_move_forward(mm, speed, id)   motion_move_backward(mm, speed, id)
motion_turn_left(deg, id)            motion_turn_right(deg, id)
motion_hold_speed(left, right)       motion_stop() / motion_clear_stop()
motion_fault_stop() / motion_clear_fault_stop()   motion_get_snapshot(&snap)
```

Each call only **validates and queues** a 16-byte `MC_CMD` on
`motion_cmd_q` (or sets a stop bit) and returns at once: `E_OK`, `E_PAR`
(bad argument) or `E_TMOUT` (queue full). A new command **replaces** the
move in progress. Moves and turns carry an `id` that comes back in
`EV_DONE [type, id]`, so VC can tell which move finished and ignore stale
ones. Only VC sends moves; only Safety Monitor uses the fault-stop pair.

`motion_stop()` / `motion_fault_stop()` set their bit in `stop_flags`,
record the time, and **wake the Motion task immediately** (`tk_wup_tsk`) —
a stop does not wait for the next 20 ms tick.

### 2.3 The task (core 1, priority 6 — the most urgent)

```
on every wake-up:
  read stop_flags, both encoder counts, the time
  if it was the 20 ms tick: take up to 8 queued commands
  if the stop bits just cleared (START): load SET values (gains, calibration)
  mc_core_step()
  write both motor duties                    <- always first
  if stopped: throw away queued commands     (never run stale moves after a stop)
  own fault: motion_fault_stop() + FLT_MOTION
  send EV_DONE / EV_STOP_ACK to VC
  on the tick: copy the snapshot under the mutex, stamp health
```

### 2.4 The logic, one step (`mc_core_step`)

1. **Stop bits first, on every call.** `FAULT_STOP` → mode FAULT;
   `CMD_STOP` → mode STOPPED. Both brake at once (duty 0 = both driver
   inputs low = brake), flush the queue and record the latency. Both bits
   clear → back to IDLE. A stop always wins over any command.
2. **Measure** (tick only). Speed per wheel = counts over the last 2
   periods (40 ms) → mm/s. Odometer (`odo_mm`, always grows) and net
   distance (`dist_mm`, reversing subtracts) are kept with remainders, so no
   millimetre is lost over a long run and nothing overflows. Acceleration is
   measured over 5 periods (100 ms) — one encoder count is a big step over
   one period, so a shorter window would be noise.
3. **Commands.** Refused while STOPPED/FAULT or invalid; otherwise start
   the move (speed hold, distance, or turn).
4. **The active move.**
   - *Speed hold* (line following): the common speed ramps at
     600 mm/s²; the steering difference is scaled with it, so while the car
     accelerates the wheels keep their ratio and neither wheel reverses.
   - *Distance / turn*: a **trapezoid** — speed rises by `accel × 20 ms` each
     tick, is capped at the move speed, and is capped again at
     `v_brake = √(2·a·d_remaining)` so it reaches zero exactly at the goal,
     but never below 60 mm/s while running. The goal is in encoder counts
     (mean of both wheels). **Straight-line trim:** if one wheel is ahead,
     it is slowed and the other sped up by 4 mm/s per mm of difference. A
     turn runs the wheels in opposite directions (+ = left).
   - When the goal is reached: brake → SETTLE → when both wheels are below
     20 mm/s (or after 0.5 s) → `EV_DONE`.
5. **PI per wheel** (`wheel_pi`):
   `duty = Kff·target + deadband + Kp·error + ∫Ki·error`.
   The feed-forward (1.6 ‰ per mm/s) plus the deadband (250 ‰, the duty
   below which the wheel does not turn at all) gets the wheel close to the
   right duty straight away; Kp and Ki remove what is left. **Anti-windup:**
   the integral only grows while the output is not saturated (or while the
   error is pulling it back). Target 0 → duty 0 (brake) and the integral is
   cleared.
6. **Checks.** *Stall*: duty above 60 % but the wheel below 10 mm/s for
   0.5 s. *Encoder direction*: the wheel turns the wrong way (>80 mm/s
   against a duty ≥ 30 %) for 0.3 s — a motor or encoder wired backwards;
   this catches it on the first run. *Timeout*: a move takes more than
   2 × its expected time + 1 s. Each one brakes, enters FAULT and raises
   `FLT_MOTION`; the snapshot's `fault_code` says which (1, 2, 3 = `mcf` in
   telemetry). Motion Control then stays in FAULT until Safety Monitor
   has latched and later released `FAULT_STOP`.
7. **Snapshot**: counts, speeds, targets, duties, mean speed,
   acceleration, distances, turn rate `(v_right − v_left) / track`, mode,
   fault code.

`EV_STOP_ACK [latency µs]` is sent once both wheels are below 20 mm/s after
a `CMD_STOP` — VC waits for it before it lets the servo scan.

### 2.5 Drivers

**Motors (`drv_motor.c`).** Robo Pico truth table: A low/B low = brake,
A PWM/B low = one way, A low/B PWM = the other way. PWM at 16 kHz
(TOP 7811, below the board's 20 kHz limit). Both channels of a slice are in
one 32-bit compare register and are written in **one store**, so A and B
are never high together. Duty is in permille. `MOTOR_DIR_LEFT/RIGHT` flip a
wheel in software instead of swapping wires. The PWM block is taken out of
reset by writing the RESETS clear alias (never put back into reset), so the
servo driver can do the same safely.

**Encoders (`drv_encoder.c`).** Software "×4" quadrature: an interrupt on
every edge of A and B; the pair (A,B) is looked up in a 16-entry table
indexed by (old state, new state) → +1, −1 or 0. A jump of both bits at
once (an edge was missed) counts as an error instead of a wrong count.
`ENC_SIGN_LEFT/RIGHT` fix a wheel that counts backwards.

### 2.6 The shared GPIO interrupt (`gpio_irq.c`, you maintain it)

The RP2040 has one interrupt for all GPIO pins. `gpio_irq.c` owns it: each
subsystem attaches a callback for its own pins (encoders GP2/3/16/17,
barcode GP27, echo GP1), and the single handler reads the 1 µs timer
**first**, then calls each pin's callback with that timestamp. Enabling is
done once, from the Motion task on core 1 (the NVIC is per core). Callback
rules: count, timestamp, store; never wait or print. `gpio_irq_max_us`
records the longest handler run, for the timing tests.

### 2.7 What to measure and tune (`motion_cfg.h`)

Motor and encoder signs first (`drv_motor.c`, `drv_encoder.c`). Then
`MC_COUNTS_PER_M` (push the car 1 m) and `MC_TRACK_MM` (wheel centre to
centre), fine-tuned with `SET dist_cal` / `SET turn_cal` (1000 × commanded
÷ measured). `MC_KFF_X1000` and `MC_DEADBAND` from a duty-vs-speed test on
blocks, then `MC_KP_X1000`, `MC_KI_X1000`. Lower `MC_ACCEL` if the wheels
slip.

### 2.8 Tests

Host: `MC-01…15` (a 500 mm move within 10 mm with its id, both wheels
equal, odometer vs net distance, a 90° turn, the speed ramp, CMD_STOP acted
on in the wake-up itself with the queue flushed, STOP_ACK, FAULT_STOP wins,
stall, own fault held until Safety Monitor has seen it, encoder backwards,
invalid commands, SET tuning). Simulator: every
scenario drives the real `mc_core.c` on a simulated motor with a deadband,
a 60 ms lag and a weaker right wheel. On target: S2-U01…U08. Manual: S2-M01…M08
in `testing/subsystem2/TEST_LOG.md` (directions, encoder signs, counts per
metre, distance and turn accuracy, speed step response, stop latency,
stall fault).

---

## Buddy 3 — Line and Barcode (`subsystem3/`)

**Talks to:** Vehicle Controller (sends line, junction and barcode events;
VC calls your steering rule), Motion Control (reads the odometer).

### 3.1 Files

| File | Kind | What it does |
|---|---|---|
| `line_core.c/.h` | logic | Region A (line status, junction, line lost) and the **steering rule** used by VC |
| `barcode_core.c/.h` | logic | turns timestamped edges into one barcode capture; the command table |
| `code39.c/.h` | logic | Code 39 decoding (forwards and backwards) |
| `line_cfg.h` | config | pins, levels, timing, steering values |
| `task_line.c` | task | every 5 ms: read the sensors, run Region A and B, send events |
| `drv_ir.c/.h` | driver | line sensor inputs, barcode analogue level (ADC) |
| `drv_barcode.c/.h` | driver | barcode DO edge interrupt → ring buffer of (time, level) |
| `subsystem3.c/.h` | setup | 5 ms cyclic handler, task on core 1, `lb_steer_params` |

### 3.2 The task (core 1, priority 8, every 5 ms)

```
Region A: read both line sensors (one register read), the odometer
          lc_step() -> LINE_STATUS on change, JUNCTION, LINE_LOST -> VC
Region B: move edges from the interrupt's ring buffer into the capture
          bc_poll() -> when the barcode is over: decode -> BARCODE or BARCODE_BAD -> VC
          ring buffer overflowed? -> FLT_LINE
stamp health
```

At start-up `calibrate()` prints the barcode sensor's analogue level and the
two line sensors (they should read white with the car centred on a line).

### 3.3 Region A — line status (`lc_step`)

The two sensors straddle the 18 mm line: both white = centred, left black =
the car drifted right, right black = drifted left, both black = junction.

- `EV_LINE_STATUS [left, right]` only when something changes (VC steers on
  these, so it reacts within 5 ms without being flooded).
- **Junction**: both black for at least **10 ms and 4 mm of travel**. The
  travel condition matters: a car standing still on a black band is not a
  junction. One event per crossing.
- **Line lost**: no black on either sensor for 2 s → `EV_LINE_LOST [side
  last seen]` (for the record; the steering rule makes the decision).

### 3.4 The steering rule (`ls_update` / `ls_output`)

Vehicle Controller is the only task that sends wheel speeds, so it calls
this rule; the rule lives here because it is about the line sensors.
Steering by `delta` **slows the inside wheel by 2 × delta**; the outside
wheel keeps the base speed. No wheel is ever asked for more than the base
speed, so there is always room to steer and the car slows in curves by
itself.

| Sensors | What it means | What the rule does |
|---|---|---|
| one black | drifting off that side | steer toward it: `delta` = 35 mm/s, +30 every 100 ms it stays black, up to 90 |
| both black | junction / crossing band | straight on |
| both white, last sensor was black **≥ 250 ms** | the line slipped **outside** past that sensor (a curve) | slow to 60 % and turn firmly (60) toward it |
| both white, last sensor left quickly | the line is **between** the sensors | lean gently (12) toward the side last seen, so a sensor touches the edge again; after 250 ms with no touch the lean grows by 12 every 250 ms (up to 60) |
| both white for 2 s | lost | `lost = true` → VC starts a search |

The "how long was it black" test (`t_escape`) is what tells the two
both-white cases apart; without it the car loses tight curves. The lean is
held for 250 ms before it grows so that crossing a short gap in the line
does not swing the car off its heading.

### 3.5 Region B — barcode capture (`barcode_core.c`)

The barcode sensor's digital output (GP27) interrupts on both edges;
`drv_barcode.c` stores `(1 µs time, level)` in a 64-entry ring buffer (one
writer — the interrupt; one reader — the task; no lock needed). Every 5 ms
the task feeds the edges to `bc_edge()`:

- a capture starts at the first edge **into a bar**;
- **chatter**: an edge closer than 300 µs to the previous one is removed
  *together with* that previous edge (a glitch pair), so bars and spaces
  still alternate;
- more than 40 edges → not a barcode;
- the capture ends when the sensor has been quiet for
  `max(60 ms, 4 × widest element)` — long enough at any speed.

Then the 29 widths (3 characters × 9 elements + 2 gaps) go to the decoder.
Captures of 1–2 elements (crossing a junction line) are ignored silently.

### 3.6 Code 39 (`code39.c`)

Each character is 9 elements (5 bars, 4 spaces), **exactly 3 of them
wide**. The decoder, per character: rank the 9 widths and call the 3
widest "wide" (so a change of speed across the barcode does not matter —
only elements of the same character, about 16 mm of barcode, are
compared); reject if the narrowest wide element is not at least 1.3 × the
widest narrow one; look up the 9-bit pattern. A valid barcode is
`*X*` — start `*`, data, stop `*`. If the forward read fails, the widths are
reversed and decoded again (the car may cross from the other end); a
reversed `*` reads as `P`, so a backwards read can never produce a wrong
character silently.

Commands (`bc_command_for`): **A** turn left, **B** turn right, **C**
straight, **D** U-turn; any other valid character (e.g. the sample **Z**)
is reported but has no command.

### 3.7 What to measure and tune (`line_cfg.h`)

`LB_BLACK_LEVEL`, `LB_BAR_LEVEL` (DO level over black — check with the
console), the modules' potentiometers, then the steering values
`LS_D_MIN … LS_START_SIDE` on a loop of the real course. `LB_T_MIN_US` if
barcodes fail with "bad count" (chatter), `LB_JUNCTION_MS/MM` if junctions
are missed or false.

### 3.8 Tests

Host: `LC-01…04` (status only on change, no junction while barely
moving, one junction per band, line lost on the side last seen), `LS-01…08`
(every row of the steering table), `BC-01…06` (clean capture, chatter
pair, merged edges, a single bar ignored, too many edges, command table),
`C39-01…09` (all 43 characters forwards and backwards, 30 % and 3× speed
changes, wide/narrow ratio 2.25, a misread element rejected — never a wrong
character, bad count, equal widths). Simulator: ovals (left, right, tight, fast,
slow), every barcode at speed, a barcode laid backwards, a 60 mm gap.
On target: S3-U01…U07. Manual: S3-M01…M04 (sensor levels, thresholds,
barcode reads at three speeds, junction and turn).

---

## Buddy 4 — IMU and Terrain + Safety Monitor (`subsystem4/`)

**Talks to:** Vehicle Controller (terrain, humps, impacts, faults,
buttons), Telemetry (all IMU events), Motion Control (fault stop; reads the
snapshot), every task (health stamps, fault bits).

### 4.1 Files

| File | Kind | What it does |
|---|---|---|
| `imu_core.c/.h` | logic | calibration, pitch, terrain, humps and height, motion class, impacts |
| `imu_math.c/.h` | logic | integer `atan2`, `sin` (tables) and square root |
| `safety_core.c/.h` | logic | fault latch, restart, buttons, task health; the `FLT_` bits |
| `imu_cfg.h` | config | axes, thresholds, Safety Monitor limits |
| `task_imu.c` | task | every 10 ms: read the accelerometer, call `imu_core_step()`, route events |
| `task_safety.c` | task | waits on `fault_flags`; every 20 ms buttons and health |
| `drv_lsm303.c/.h` | driver | LSM303DLHC over the kernel's I2C driver |
| `subsystem4.c/.h` | setup | `fault_flags`, 10 ms cyclic handler, both tasks on core 0 |

### 4.2 IMU driver (`drv_lsm303.c`)

I2C0 on GP4/GP5 through the kernel's `iica` device. `lsm_open()` checks
the magnetometer's identity register (0x48). The accelerometer is set to
100 Hz, ±2 g, high-resolution, block data update, and the settings are read
back. Each 10 ms one 6-byte burst read gets X, Y, Z (12-bit values,
1 mg per count). The magnetometer is read 10 times a second for an
uncalibrated heading (a cross-check only).

### 4.3 The IMU logic, one sample (`imu_core_step`)

1. **Calibration**: the first 100 samples (1 s, car still and level) give
   the gravity vector. Its angle is the board's **mounting tilt**, which
   is subtracted from every later pitch — the board does not need to be
   mounted perfectly level.
2. **Impact**: a sudden jump (> 1500 mg) of the forward or sideways
   reading against the filtered value, while moving at ≥ 80 mm/s, starts a
   150 ms window. If the wheels' speed falls to half or less within it →
   **collision**: `EV_IMPACT` and `FLT_IMPACT` (Safe Stop). If not, it was
   a bump. Ignored on a hump (its edges jolt the car too).
3. **Filter** each axis (y += (x − y)/8, about 80 ms) and **remove the
   car's own acceleration**: speeding up tilts the measured gravity vector
   just like going uphill, so the encoder acceleration (from the motion
   snapshot, converted to mg) is subtracted from the forward axis.
4. **Pitch** = `atan2(forward, up)` − mounting tilt, in centidegrees,
   nose up positive.
5. **Terrain**: CLIMBING above +3°, DESCENDING below −3°, LEVEL again
   inside ±1.5° (hysteresis). A slope only counts after **20 mm of travel**
   while tilted — braking, a steering kick or a knock tilts the reading
   for a moment without the car going anywhere. Each change → `EV_TERRAIN`.
6. **Hump height**. The LSM303DLHC has no gyro, so two estimates are kept
   and the larger is reported:
   - *integral*: `h += Δs × sin(pitch)` with Δs from the encoders — right
     for a hump longer than the car (the body follows the slope);
   - *wheelbase*: `L × sin(largest pitch)` — right for a hump shorter than
     the car (the body tilts by asin(H/L) while one end is on top).
   The last 200 ms of rise before the 3° threshold is added (the hump
   started before it was detected). The hump ends when the car has been
   level for wheelbase + 40 mm of travel (the rear wheels must be over it
   too), or 2 s if stopped → `EV_HUMP [height, highest so far]`.
7. **Motion class** (for telemetry): IMPACT, CLIMBING, DESCENDING,
   TURNING (> 30°/s), ACCELERATING / DECELERATING (> 250 mm/s²),
   STATIONARY, MOVING — reported after it has held for 50 ms.

Read failures: 5 in a row → `FLT_IMU` (if `IMU_REQUIRED` = 1).

Routing (`task_imu.c`): VC gets terrain, humps and impacts (not the motion
class); Telemetry gets everything.

### 4.4 Safety Monitor (`safety_core.c`, `task_safety.c`)

**Fault bits** (`safety_core.h`): `FLT_MOTION 0x01`, `FLT_LINE 0x02`,
`FLT_IMU 0x04`, `FLT_IMPACT 0x08`, `FLT_SCAN 0x10`, `FLT_QUEUE_OVF 0x20`,
`FLT_HEALTH 0x40`, `FLT_NAV 0x80`. Any task reports a fault with
`tk_set_flg(fault_flags, FLT_...)` — it never stops the motors itself.

The task waits on `fault_flags` with a 20 ms timeout:

- **A fault bit** → latch at once: `motion_fault_stop()` (motors off,
  queue flushed), `EV_FAULT [bits]` to VC.
- **RESTART** (`FLT_RESTART_REQ`, set by VC for an MQTT RESTART, or by the
  START button while faulted) → clear every bit, then the bits must stay
  clear for **5 polls (100 ms)**. Any fault in that time cancels the
  restart. Then `motion_clear_fault_stop()` and `EV_FAULT_CLEAR` → VC goes
  to READY. (`CMD_STOP` stays set: the car still needs START.)
- **The 20 ms timeout**: read the buttons (debounced over two polls;
  GP20 = START, GP21 = STOP → `EV_BUTTON`), and check **task health**:
  every periodic task stamps its slot each cycle (`health_stamp()`); a stamp
  older than its limit (Motion 100 ms, Line 50 ms, IMU 250 ms, Scan
  250 ms, VC 200 ms) raises `FLT_HEALTH`.

One subtle point (`sm_wait_pattern`): while latched, the fault bits stay
set until the restart clears them, so waiting for them would return
immediately forever. While latched (and no restart counting down) the task
therefore waits only for `FLT_RESTART_REQ`.

### 4.5 What to measure and tune (`imu_cfg.h`)

`IMU_AXIS_*` / `IMU_SIGN_*` first (level: "up" ≈ +1000 mg; nose up:
"forward" positive). `IMU_WHEELBASE_MM`. Then humps of known height to
check the estimate, and `IMU_IMPACT_MG` (drive slowly into a soft box). `SM_HEALTH_*` only if a task legitimately runs late.

### 4.6 Tests

Host: `IMU-01…14` (atan2, sine and square root accuracy, calibration
finds a 1.5° mounting tilt, level after calibration, a braking jolt is not
a slope, climbing / level / descending, a 20 mm hump within ±5 mm, highest
hump kept, collision vs bump, turning, stationary, fault after 5 failed
reads) and `SM-01…05` (button debounce, START, a silent task raises
`FLT_HEALTH` once, START while faulted = RESTART). The latch and restart
rules are checked on the target (S4-U03…U06) and in the simulator, which
also runs two humps of 12 and 25 mm, an IMU that goes silent and a
hidden-box collision. On target: S4-U01…U08. Manual: S4-M01…M05 (axes and signs, hump heights,
impact threshold, buttons, task health).

---

## Buddy 5 — Obstacle Scan + Vehicle Controller (`subsystem5/`)

**Talks to:** everyone. Vehicle Controller (VC) receives every event and
is the **only** sender of motion commands and scan requests.

### 5.1 Files

| File | Kind | What it does |
|---|---|---|
| `vc_core.c/.h` | logic | the mission statechart: follow, barcode commands, search, obstacle handling, bypass, Safe Stop |
| `vc_cfg.h` | config | speeds, car geometry, distances, time limits |
| `task_vc.c` | task | waits on `vc_event_q` (at most until the next 20 ms tick), runs the actions |
| `scan_core.c/.h` | logic | forward watch, coarse and fine scans |
| `os_profile.c/.h` | logic | turns a scan into an obstacle profile |
| `scan_cfg.h` | config | ranges, timing, scan angles |
| `task_scan.c` | task | every 10 ms: one step of the scan logic, servo, trigger |
| `drv_hcsr04.c/.h` | driver | ultrasonic trigger and echo timing |
| `drv_servo.c/.h` | driver | SG90 PWM |
| `subsystem5.c/.h` | setup | `vc_event_q`, `scan_request_q`, 10 ms cyclic handler, both tasks |

### 5.2 Ultrasonic and servo drivers

**HC-SR04 (`drv_hcsr04.c`)**: `hcsr04_trigger()` makes a 10 µs TRIG pulse
(the only busy-wait in the whole system; at least 60 ms between pings).
ECHO edges are timestamped by the shared GPIO interrupt; range =
echo width × 0.1715 mm/µs. `hcsr04_poll()` returns DONE (range valid),
NO_ECHO (ECHO rose but nothing came back: **unknown, not free**) or
NO_RESPONSE (ECHO never rose: sensor missing). Check ECHO is 3.3 V-safe
before wiring it to GP1.

**Servo (`drv_servo.c`)**: 50 Hz with 1 µs PWM counts; the angle is
converted to a pulse width with `SERVO_US_AT_30DEG` / `_AT_150DEG`
(measure them so 30° points **right**, 150° left, 90° ahead).

### 5.3 Obstacle Scan logic (`scan_core.c`, every 10 ms)

Nothing ever waits: servo settling (40 ms + 3 ms per degree), the 70 ms
gap between pings and the echo are each a state checked again next step.
The newest request from VC wins; every request carries an **id** that is
copied into the events it causes, so VC can ignore events from an old
request.

- **IDLE**: servo centred, no pings.
- **WATCH** (while following): servo at 90°, a ping every 70 ms.
  `EV_RANGE_ZONE` near/far when the range crosses 450 mm (back to far above
  500 mm) — VC slows down; `EV_ALERT [range]` after **2 readings in a
  row** closer than `stop_mm` (250 mm; 120 mm during a bypass). One ALERT
  per request.
- **PROFILE** (car stopped):
  1. *coarse*: 30°, 60°, 90°, 120°, 150°;
  2. *fine*: find the nearest coarse hit **inside the car's path**
     (|x| ≤ half the car width + margin = 130 mm) — a wall beside the path
     must not pull the scan away — take all coarse hits within 150 mm of
     that range, and scan from 25° beyond one side to 25° beyond the other
     in 5° steps (skipping angles already measured);
  3. each angle gets up to 3 pings; no echo after that is stored as
     **unknown**;
  4. build the profile (below), send `EV_PROFILE [valid, id]`, centre the
     servo, go IDLE. Over 8 s → `FLT_SCAN` and an invalid profile.

Ten pings in a row with no response at all → `FLT_SCAN` (sensor dead).

### 5.4 The profile (`os_profile.c`)

Each reading (angle, range) becomes a point `x = r·cos(angle)` (sideways,
right +), `y = r·sin(angle)` (ahead). Ranges beyond 800 mm are background.

1. Start from the **nearest point inside the path corridor** (or the
   nearest overall if nothing is in the path).
2. Grow left and right while neighbouring ranges agree within 80 mm; one
   missing echo between two matching points does not split the object.
3. The object's left/right edges (x), centre, width, closest distance (y)
   and its angle.
4. **Clearance** on each side: the gap to the nearest *other* point beyond
   that edge, counting only points up to 300 mm further away than the
   obstacle; 1000 mm if nothing. Missing echoes beyond each edge are
   counted as `unknown_left/right`.

The sonar beam is about 15° wide, so objects look a little wider than they
are — the error is on the safe side.

### 5.5 Vehicle Controller — how it is built

`vc_core.c` is a statechart that receives three kinds of input —
`vc_event()` (any event), `vc_profile()` (a profile arrived) and
`vc_tick()` (every 20 ms: timers and steering) — and returns a short list
of **actions**: `HOLD` (wheel speeds), `MOVE`, `TURN`, `STOP`,
`CLEAR_STOP`, `SCAN` (request), `FAULT`, `RESTART`, `TEL` (telemetry
record). `task_vc.c` carries them out in order with the motion API,
`scan_request()`, `tk_set_flg()` and `app_send(telemetry_q, …)`.

Two habits keep it robust:

- **Ids.** Every move/turn gets a new id and VC only accepts the `EV_DONE`
  with that id; every scan request gets a new id and VC only accepts the
  ALERT / RANGE_ZONE / PROFILE with that id. A late event from an
  interrupted move or an old watch can never trigger the wrong step.
- **Speed holds are re-sent** whenever a wheel speed changes by 3 mm/s or
  more, and at least every 200 ms anyway, so a hold lost to a full queue is
  replaced quickly without flooding Motion Control.

States: `STARTUP, READY, FOLLOW, EXEC, SEARCH, WAIT_ACK, SCANNING, BYPASS,
REVERSE, FIND_LINE, ALIGN, SAFE_STOP, FINISHED`.

### 5.6 Vehicle Controller — what it does

**Start-up and commands.** STARTUP → READY when the IMU has calibrated
(or after 3 s). START (from READY/FINISHED, no fault): clear `CMD_STOP`,
copy the SET values, WATCH, FOLLOW. STOP while running → READY (motors
held). RESTART → asks Safety Monitor to clear the faults; READY when
`EV_FAULT_CLEAR` arrives. FINISH → FINISHED with `NAV_FINISH [distance]`
(Telemetry adds the summary).

**FOLLOW.** On every line-status change and every tick, run Buddy 3's
steering rule and send the wheel speeds. Speed: 200 mm/s; 120 if the
ultrasonic says something is near (< 450 mm); 150 while a barcode command
is waiting for its junction. **On a hump** (terrain not LEVEL, and 500 ms
after it): drive straight at 150 mm/s and ignore the line, junctions and
barcodes (the sensors lift off the floor); when level again the line-lost
clock starts over.

**Barcode → junction (EXEC).** A barcode with a command is stored as
*pending*. At the next junction (junctions within 80 mm of the last one,
or of a turn, are ignored):

| Command | What the car does |
|---|---|
| A / B | drive 69 mm (line sensors-to-axle 60 + half the line 9) so the **axle** is over the junction, then spin 90° left / right |
| C | drive 40 mm across the junction band, then follow on |
| D | spin 180° **right where the junction was detected** — the sensors come back down on the line it arrived on, never on a side branch |

If no junction comes within 1000 mm of the barcode, the command is carried
out where the car is. After the turn the car follows again, leaning toward
the direction it turned (spins slip, so a car usually turns slightly
short and the line is further round).

**Acquire.** After a turn or a bypass, the line must reach a sensor within
60 mm; otherwise the car is pointing past it and searches at once, instead
of driving on for 2 s first.

**SEARCH** (line lost for 2 s, or not acquired): spin 120° toward the side
the line was last seen, then 240° the other way, then 120° back. The first
black on either sensor ends the search and following resumes. Nothing →
Safe Stop (`why` 6). Whole search limit 8 s.

**Obstacle (Figure 7 of the design).**

1. `EV_ALERT` in FOLLOW or EXEC → `motion_stop()`, scan IDLE, **WAIT_ACK**
   (1.5 s limit). A barcode command still waiting for its junction is
   dropped: after a detour the car cannot be sure which junction it meant.
2. `EV_STOP_ACK` (wheels stopped) → PROFILE request, **SCANNING** (10 s
   limit).
3. `EV_PROFILE` → **decide** (corridor = 80 + 50 = 130 mm each side of the
   line; needed clearance = car width + margin = 210 mm):
   - no valid profile → **Reverse and Retry**;
   - the object lies entirely outside the corridor → **Continue**
     (ALERTs are ignored until the car is past it);
   - otherwise compute the sideways shift needed to pass on each side
     (left: 130 − x_left; right: 130 + x_right). Pass on a side only if its
     clearance ≥ 210 mm. Both possible: the **smaller shift** (less
     deviation from the route); within 20 mm of each other: the side with
     fewer unknown readings, then the preferred side (left);
   - no side possible → **Reverse and Retry**: back 150 mm, stop, scan
     again; after 2 retries → Safe Stop (`why` 5).
4. **BYPASS** — a plan of five steps, each waiting for its `EV_DONE`:
   turn 45° out → drive `d1 = shift / sin 45°` → turn 45° back (parallel to
   the line) → drive `d2` = enough for the axle to clear the far side of the
   obstacle (closest distance + sensor-to-axle + assumed depth 150 + rear
   clearance − what leg 1 already gained) → turn 45° back toward the line.
   The ultrasonic watches at 120 mm during every drive; an ALERT there →
   Safe Stop (`why` 7).
5. **FIND_LINE**: drive toward the line (`d1` + 150 mm) slowly. Black →
   **ALIGN**: drive 70 mm more (sensors to axle + 10) so the axle is on the
   line, turn 45° back to the line's heading, follow again (`NAV_REJOINED`).
   Nothing by the end → a line search, turning first toward the side the
   line should be on.
   The whole obstacle episode is limited to 20 s.

**Safe Stop.** On `EV_FAULT`, an impact, or any of the limits above: stop,
scan IDLE, raise `FLT_NAV` (for its own reasons, so Safety Monitor latches
it too), record `why`: 1 fault, 2 impact, 3 stop not acknowledged, 4 no
scan result, 5 no way round, 6 line not found, 7 blocked during a bypass,
8 command too slow, 9 bypass too slow. Leave with RESTART, then START.

Every state change (`EV_STATE [state, why]`) and decision (`EV_NAV`:
pending, exec, continue, bypass ± shift, reverse, rejoined, search, found,
finish, refused) goes to Telemetry and is printed on the console.

### 5.7 What to measure and tune

`scan_cfg.h`: `OS_STOP_MM`, `OS_SLOW_MM`, servo settling. `vc_cfg.h`
**MEASURE**: `VC_CAR_WIDTH_MM`, `VC_US_TO_AXLE_MM`, `VC_LINE_TO_AXLE_MM`,
`VC_REAR_CLEAR_MM`, `VC_OBST_DEPTH_MM` (the deepest obstacle you expect).
**TUNE**: speeds, `VC_MARGIN_MM`, `VC_D_EXEC_MM`, `VC_ACQUIRE_MM`, and
`VC_PREF_SIDE`. Servo pulse widths in `drv_servo.h`.

### 5.8 Tests

Host: `OP-01…05` (a box is never narrower than it is, missing echoes
are unknown, a wall beside the path, a missing echo inside the box, no
readings = no profile), `SC-01…07` (pings 70 ms apart, one ALERT, the box
found, coarse then fine angles, time limit, servo back to the front, sensor
fault), `VC-01…21` (START refused before the sensors are ready, START,
barcode then junction → axle onto the junction → turn → follow, a DONE with
the wrong id ignored, STOP, faults, RESTART, every Continue / Bypass /
Reverse decision, scan timeout). Simulator: obstacle in the middle, sticking
out right, with a wall on the left; dead end (search fails → Safe Stop →
RESTART → START). On target: S5-U01…U06. Manual: S5-M01…M05 (ECHO
voltage, range accuracy, servo calibration, profile of a known box, bypass
and rejoin).

---

## Part 6 — How the code is checked

| Check | Command | What it proves |
|---|---|---|
| Firmware build, every variant | `cd build_make; make`, `make TEST=1`, `make NET=lwip MQTT=1`, `make SMP=0`, `make CONSOLE=uart WIFI=none`, `make CRASH_REPORT=1` | compiles with zero warnings for the RP2040 |
| Host unit tests | `cd testing/host; make` → `./unit_tests` | 111 checks of every logic module, built with strict warnings and the address / undefined-behaviour sanitizers |
| Closed-loop simulator | `./sim` (or `./sim -v <scenario>`) | the real logic of all five subsystems driving a simulated car through 20 scenarios (99 checks) |
| Robustness | `python3 sweep.py 11 24` (or one car: `./sim -p <seed> <deadband> <kff> <track> <counts/mm> <right wheel>`) | the same scenarios on cars whose real motor, track width and encoder scale differ from the configured values; last run: 48 random cars, all 99 checks passed on every one |
| On target | `make TEST=1`, flash, read the console | the `S*-U*` checks on the real board, while the system runs |

Saved results: `testing/host/logs/`. The manual tests for the real car
(measurements, timings, track runs) are defined in
`testing/subsystemN/TEST_LOG.md`, `testing/integration/TEST_LOG.md` and
`testing/system/TEST_LOG.md`; add a row for every run.

The simulator checks the logic, not your hardware. Every value marked
**MEASURE** still has to be measured on your car, in the order given in
`CAR_GUIDE.md` section 4.
