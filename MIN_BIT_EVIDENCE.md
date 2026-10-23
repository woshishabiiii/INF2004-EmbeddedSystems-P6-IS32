# Minimum-bit data representation: Week 13 evidence

Change Requirement 1 asks for four pieces of evidence in the repository: a
data representation table, a RAM comparison, a message size comparison, and
evidence that nothing was lost in function or timing. This file gives all
four. Every number comes from a script in `testing/evidence/` that anyone can
rerun (section 6); the raw output is in `testing/evidence/results.txt`.

- **Before:** V12, commit `cf0176a`: JSON MQTT, text commands, 32-bit fields
  in every shared record and message.
- **After:** branch `min-bit-v12`, measured at commit `762f29f`: binary MQTT,
  packed records, bit-sized fields.
- Both firmware images are the `make NET=lwip MQTT=1` build that goes on the
  car.

## Summary

| | V12 (before) | min-bit (after) | Change |
|---|---|---|---|
| Static RAM, whole image (`.data` + `.bss`) | 85,640 B | 85,096 B | −544 B |
| Message buffers (allocated by the kernel at start-up) | 2,276 B | 896 B | −1,380 B (−61 %) |
| **RAM used, total of the two** | **87,916 B** | **85,992 B** | **−1,924 B** |
| Event message in `vc_event_q` and `telemetry_q` | 16 B | 8 B | −50 % |
| Motion snapshot shared under `mtx_snapshot` | 88 B | 36 B | −59 % |
| MQTT telemetry payload (every 0.5 s) | 153 B (up to 180) | 23 B | −85 % |
| MQTT traffic in steady running | 530 B/s | 134 B/s | −75 % |
| Host tests | 140 unit + 153 simulator | 172 unit + 162 simulator + 13 decoder | all pass |
| CPU time on data handling | 0.05 % per core | 0.16 % (core 1), 0.24 % (core 0) | +0.11 and +0.19 points |
| Time added to a task release | | at most +17 µs (Motion Control, every 20 ms) | at most 0.11 % of a period (IMU, every 10 ms) |
| Flash (`text` + `data`) | 374,736 B | 385,236 B | +10,500 B (+2.8 %) |

The car's behaviour is unchanged: 29 of the 30 V12 simulator scenarios end in
exactly the same state, position, heading and odometer reading, and the 30th
is the line search, which we changed on purpose (section 4.1).

---

## 1. Data representation table

### 1.1 How the fields are packed

- **Masks and shifts, not C bit-fields.** The requirement allows either. We
  use the helpers in `app_program/bitpack.h`, because the order and padding of
  C bit-fields are up to the compiler, and our MQTT records are read by a
  Python tool (`tools/mqtt_decode.py`). Every record is a byte or word array
  written field by field, least significant bit first.
- **Signed values are stored as `value − min`**, so every stored field is
  unsigned.
- **A value outside its range saturates; it never wraps.** For example, 5,000 mm
  in a 12-bit range field becomes 4,095 (no echo), not 904 mm. Every record has
  a unit test for this.
- **Counters and times that wrap are only subtracted** (`bp_age16()`,
  `bp_dt31()`), never compared with "greater than".
- **Every packed record has a bit-by-bit layout comment** in its owner's
  header, next to a compile-time size check (`BP_STATIC_ASSERT`), and a unit
  test that round-trips every field at its minimum and maximum.
- **We pack at the boundary.** Data that a task stores for other tasks, puts
  in a queue, or sends over MQTT is packed. A task's own working variables
  stay 32-bit (section 1.12).

"Bits needed" below is the smallest number of bits that holds every value in
the valid range. Where we use more, section 1.11 gives the reason.

### 1.2 Fault conditions: one shared bitmask

`fault_flags` is a kernel event flag (`safety_core.h`). Each condition is one
bit, never an integer code or a separate variable.

| Bit | Name | Condition | Latches |
|---|---|---|---|
| 0 | `FLT_STALL` | high duty but the wheel is not turning | yes |
| 1 | `FLT_LINE` | the barcode interrupt could not be set up | yes |
| 2 | `FLT_IMU` | the IMU is not answering | yes |
| 3 | `FLT_IMPACT` | collision | yes |
| 4 | `FLT_SCAN` | the ultrasonic sensor is not answering | yes |
| 5 | `FLT_QUEUE_OVF` | a safety event or a motion command could not be queued | yes |
| 6 | `FLT_HEALTH` | a task stopped stamping its health slot | yes |
| 7 | `FLT_NAV` | the wheels did not stop after `CMD_STOP` | yes |
| 8 | `FLT_ENCODER` | a wheel turns against its duty | yes |
| 9 | `FLT_MOVE_TIMEOUT` | a move took far longer than expected | yes |
| 13 | `FLT_NO_ECHO` | the last ping heard no echo | no (status bit) |
| 14 | `FLT_WIFI_DOWN` | the MQTT link is down | no (status bit) |
| 15 | `FLT_RESTART_REQ` | a RESTART was asked for (not a fault) | no |

- Any task sets **its own** bit with `tk_set_flg()`. The kernel does the
  read-modify-write, so two cores setting bits at the same time cannot lose one.
- V12 also kept the Wi-Fi state in a separate `BOOL net_up`, and sent it
  again as an "MQTT up" field in the status record. Both are gone: the
  link state is only the `FLT_WIFI_DOWN` bit.
- Over MQTT the faults travel as one 12-bit field: bits 0–9 latched, bit 10
  `NO_ECHO`, bit 11 `WIFI_DOWN`.
- Motion Control's own faults (stall, move timeout, encoder) are a 3-bit mask
  in the snapshot and the status record.

### 1.3 Motion control and encoder

**Encoder: GPIO interrupt → Motion Control** (`drv_encoder.h`, `mc_core.h`)

| Field | Valid range | Bits needed | Bits used | Representation |
|---|---|---|---|---|
| Wheel count, each wheel | change between two 20 ms reads: ±16 counts at 400 mm/s | 6 | 16 | One 16-bit half of a 32-bit word per wheel. The task adds each 16-bit difference to its own 32-bit position (1.11). |
| Quadrature state, each wheel | 4 states | 2 | 2 | bits 0–1 of a status byte per wheel |
| Skipped-state errors, each wheel | 0–63, stops at 63 | 6 | 6 | bits 2–7 of the same byte |

**Motion command: `motion_cmd_q`, `MC_CMD_PK`** (8 B, was 16 B)

| Field | Valid range | Bits needed | Bits used | Representation |
|---|---|---|---|---|
| Command type | SPEED, DISTANCE, TURN | 2 | 2 | bits 0–1 |
| SPEED: left and right wheel speed | ±400 mm/s | 10 | 10 each | value + 512 |
| SPEED: line-reading time present | yes / no | 1 | 1 | |
| SPEED: line-reading time | age up to 65.5 s | 16 | 16 | low 16 bits of `app_ms()`, used only for the latency log |
| DISTANCE: distance | ±10,000 mm | 15 | 15 | value + 16,384 |
| DISTANCE or TURN: speed | 0–400 mm/s | 9 | 9 | |
| DISTANCE or TURN: move id | 1–255 | 8 | 8 | |
| TURN: angle | ±360° | 10 | 10 | value + 512 |

The largest layout (SPEED) is 39 bits.

**Motion snapshot: shared under `mtx_snapshot`, `MC_SNAP_PK`** (36 B, was 88 B)

| Field | Valid range | Bits needed | Bits used |
|---|---|---|---|
| Sequence number | compared only for "changed?" | 8 | 8, wraps |
| Time | age up to 65.5 s | 16 | 16, wraps |
| Encoder count, left and right | low 16 bits (telemetry) | 16 | 16 each |
| Measured speed, left and right | ±469 mm/s (full duty) | 10 | 10 each |
| Target speed, left and right | ±400 mm/s | 10 | 10 each |
| Duty, left and right | ±1,000 ‰ | 11 | 11 each |
| Mean speed | ±469 mm/s | 10 | 10 |
| Acceleration | ±9,380 mm/s² (±938 mm/s within 100 ms) | 15 | 15 |
| Net distance (reversing subtracts) | ±16.7 km | 25 | 25 |
| Odometer (always grows) | 0–16.7 km | 24 | 24 |
| Turn rate | ±413 °/s, in centidegrees | 17 | 17 |
| Mode | 7 modes | 3 | 3 |
| Fault mask | 3 faults | 3 | 3 |
| Steering latency, last and largest | 0–1,023 ms (a few ms expected) | 10 | 10 each |
| Largest stop latency | 0–131,071 µs | 17 | 17 |
| Last stopping distance | 0–1,023 mm | 10 | 10 |
| Learned stopping rate | 100–20,000 mm/s² (0 = not yet) | 15 | 15 |
| **Total** | | | **277 bits = 35 B**, kept in 9 words (36 B) |

### 1.4 Line and barcode

| Field | Valid range | Bits needed | Bits used | Where |
|---|---|---|---|---|
| Line sensor, left and right | black / white | 1 | 1 each | line status word bits 0–1; `EV_LINE_STATUS` |
| Side the line was last seen | −1, 0, +1 | 2 | 2 | `EV_LINE_LOST` |
| Junction position | odometer, 0–16.7 km | 24 | 24 | `EV_JUNCTION` |
| Decoded character | `' '`–`'_'` (64 codes; Code 39's 43 characters fall inside) | 6 | 6 | character − 32; `'-'` = none |
| Barcode command | 0–4 | 3 | 3 | `EV_BARCODE`; pending command |
| Bad-read reason | Code 39 error codes, 100 = buffer overflow | 7 | 7 | `EV_BARCODE_BAD` |
| Elements in a bad read | 0–40 | 6 | 6 | `EV_BARCODE_BAD` |
| Good reads, failed reads, junctions | 0–255, stop at 255 | 8 | 8 each | line status word |
| Barcode edge time | µs between edges of one capture, at most the quiet time that ends it (60 ms or more; a 9 mm bar at 120 mm/s lasts 75 ms) | 17 | 31 | bits 0–30 of the 4 B edge (1.11) |
| Barcode edge level | 0 / 1 | 1 | 1 | bit 31 of the edge |
| Edge ring head and tail | 0–63 (64 entries) | 6 | 8 each | separate bytes, one writer each (1.11) |
| Edge overflow count | wraps; only "changed?" is asked | 8 | 8 | written only by the interrupt |
| Barcode sensor analog level | 0–4,095 (12-bit ADC) | 12 | 12 | masked with `IR_ADC_MASK`, calibration only |

### 1.5 Obstacle detection

| Field | Valid range | Bits needed | Bits used | Where |
|---|---|---|---|---|
| Echo width | 0–32,767 µs (4 m is 23,324 µs) | 15 | 15 | echo word bits 0–14, written only by the interrupt |
| Completed-echo count | wraps; only "changed?" is asked | 8 | 8 | echo word bits 15–22 |
| Range | 0–4,000 mm, 4,095 = no echo | 12 | 12 | scan status, `EV_ALERT`, telemetry |
| Scan mode | IDLE, WATCH, PROFILE | 2 | 2 | scan request, scan status |
| "Stop the car yourself" flag | yes / no | 1 | 1 | scan request |
| Stop distance | 120–630 mm plus one code for "default" (SET allows 150–600) | 9 | 9 | value − 120 |
| Request or watch id | 1–255 | 8 | 8 | scan request; `EV_ALERT`, `EV_RANGE_ZONE`, `EV_PROFILE` |
| Ping count | low 16 bits; only subtracted | 16 | 16 | scan status, status record |
| Near or far zone | 0 / 1 | 1 | 1 | `EV_RANGE_ZONE` |

**Obstacle profile: shared under `mtx_profile`, `OS_PROFILE_PK`** (13 B, was 48 B)

| Field | Valid range | Bits needed | Bits used |
|---|---|---|---|
| Valid | yes / no | 1 | 1 |
| Left edge, right edge, centre (x) | ±800 mm | 11 | 11 each |
| Width | 0–1,600 mm | 11 | 11 |
| Closest point | 0–800 mm | 10 | 10 |
| Angle of the closest point | 30–150° (servo), stored − 30 | 7 | 7 |
| Clearance left and right | 0–1,000 mm | 10 | 10 each |
| No-echo readings left and right; points used | 0–25 (a scan has 25 slots) | 5 | 5 each |
| **Total** | | | **97 bits = 13 B** |

### 1.6 Hump estimation (IMU)

**IMU status: 3 words, `IC_STATUS_PK`**

| Field | Valid range | Bits needed | Bits used |
|---|---|---|---|
| Calibrated, last read OK | yes / no | 1 | 1 each |
| Terrain | level, climbing, descending | 2 | 2 |
| Motion class | 8 classes | 3 | 3 |
| Pitch | ±81.9° in centidegrees (steeper saturates) | 14 | 14 |
| Last hump height, highest hump | 0–255 mm, stop at 255 | 8 | 8 each |
| Humps | 0–255, stops at 255 | 8 | 8 |
| Gravity measured at calibration | 0–3,464 mg (±2 g on three axes) | 12 | 12 |
| Heading | ±180.00° in centidegrees | 16 | 16 |

**IMU events:** `EV_TERRAIN` terrain 2 + pitch 14; `EV_HUMP` height 8 +
highest 8; `EV_MOTION` class 3; `EV_IMPACT` jump 0–4,095 mg 12 + speed
0–511 mm/s 9.

### 1.7 Vehicle and controller states

| Field | Valid range | Bits needed | Bits used | Where |
|---|---|---|---|---|
| Vehicle Controller state | 14 states | 4 | 4 | VC status word 0, `EV_STATE`, telemetry |
| Reason for SAFE_STOP or HALTED | 10 reasons | 4 | 4 | VC status, `EV_STATE`, status record |
| Pending barcode command | 0–4 | 3 | 3 | VC status, telemetry |
| Most events waiting in `vc_event_q` | 0–32 | 6 | 6 | VC status, timing record |
| Commands done | 0–255, stops | 8 | 8 | VC status, summary |
| Obstacles passed | 0–127, stops | 7 | 7 | VC status, summary |
| Run start time | full ms clock | 32 | 32 | VC status word 1 (1.11) |
| Run start odometer | 0–16.7 km | 24 | 24 | VC status word 2 |
| Motion Control mode | 7 modes | 3 | 3 | snapshot |
| Obstacle Scan mode | 3 modes | 2 | 2 | scan request and status |
| Navigation event kind | 11 kinds | 4 | 4 | `EV_NAV` |
| Navigation value | ±16.7 km | 25 | 25 | `EV_NAV` |

### 1.8 Inter-task queue messages

**`APP_MSG`: every event in `vc_event_q` (32 slots) and `telemetry_q` (16
slots)** (8 B, was 16 B)

| Field | Valid range | Bits needed | Bits used |
|---|---|---|---|
| Event type | 21 types | 5 | 5 |
| Sender | 7 tasks | 3 | 3 |
| Sequence number | per sender, orders consecutive messages; wraps | 8 | 8 |
| Time | age up to 65.5 s | 16 | 16 |
| Payload p1 and p2 | each type's own widths (below) | up to 29 | 32, shared by p1 and p2 |

| Event | p1: range → bits | p2: range → bits |
|---|---|---|
| `EV_LINE_STATUS` | left on black → 1 | right on black → 1 |
| `EV_LINE_LOST` | side −1..1 → 2 | |
| `EV_JUNCTION` | odometer 0–16.7 km → 24 | |
| `EV_BARCODE` | character → 6 | command 0–4 → 3 |
| `EV_BARCODE_BAD` | reason 0–100 → 7 | elements 0–40 → 6 |
| `EV_DONE` | move type 1–2 → 2 | move id 1–255 → 8 |
| `EV_STOP_ACK` | latency 0–131,071 µs → 17 | stopping distance 0–1,023 mm → 10 |
| `EV_FAULT` | the 10 latching fault bits → 10 | |
| `EV_FAULT_CLEAR` | | |
| `EV_ALERT` | range 0–4,095 mm → 12 | watch id → 8 |
| `EV_RANGE_ZONE` | near or far → 1 | watch id → 8 |
| `EV_PROFILE` | valid → 1 | request id → 8 |
| `EV_TERRAIN` | terrain 0–2 → 2 | pitch ±81.9° → 14 |
| `EV_HUMP` | height 0–255 mm → 8 | highest of the run → 8 |
| `EV_MOTION` | motion class 0–7 → 3 | |
| `EV_IMPACT` | jump 0–4,095 mg → 12 | speed 0–511 mm/s → 9 |
| `EV_BUTTON` | START or STOP → 1 | |
| `EV_COMMAND` | START, STOP, RESTART, FINISH → 3 | |
| `EV_STATE` | state → 4 | reason → 4 |
| `EV_NAV` | kind → 4 | value ±16.7 km → 25 |
| `EV_PROFILE_DATA` | | |

The widest payload is `EV_NAV` (29 bits). The table is `s_layout[]` in
`app_evt.c`; `tools/test_mqtt_decode.py` checks that the laptop decoder uses
the same widths.

**The other queues**

| Queue | Message | Was |
|---|---|---|
| `motion_cmd_q` (8 slots) | `MC_CMD_PK`, 8 B (1.3) | 16 B |
| `scan_request_q` (4 slots) | `SCAN_REQ_PK`, 20 bits in 4 B (1.5) | 12 B |
| `mqtt_tx` (4 slots) | 1 B topic + binary record, at most 34 B | 1 B + JSON, up to 200 B |
| `mqtt_rx` (4 slots) | 4 B binary command | text, up to 64 B |

### 1.9 MQTT telemetry and commands

Every record starts with a 32-bit header: version 4 bits, record type 4 bits
(7 types), time 24 bits (ms, wraps every 4.66 h). Layouts are in
`app_program/subsystem1/tel_bin.h`; `tools/mqtt_decode.py` decodes them.
Each record is the fewest whole bytes that hold its fields, and every packer
checks this each time it runs (`finish()` in `tel_bin.c`).

**Telemetry, every 0.5 s: 182 bits = 23 B** (V12: 153 B of JSON, up to 180 B)

| Field | Valid range | Bits |
|---|---|---|
| Header | | 32 |
| State | 14 states | 4 |
| Wheel speed, left and right | ±469 mm/s | 10 each |
| Encoder count, left and right | low 16 bits | 16 each |
| Odometer | 0–16.7 km | 24 |
| Line sensors | 2 × black / white | 2 |
| Last barcode character | 64 codes | 6 |
| Pending command | 0–4 | 3 |
| Pitch | ±81.9° | 14 |
| Motion class | 8 classes | 3 |
| Highest hump | 0–255 mm | 8 |
| Range | 0–4,000 mm, 4,095 = no echo | 12 |
| Turn rate | ±413 °/s | 10 |
| Faults | 12 bits (1.2) | 12 |

**Status, every 1 s: 142 bits = 18 B** (V12: 126 B, up to 181 B)

| Field | Valid range | Bits |
|---|---|---|
| Header | | 32 |
| State, reason | 14 states, 10 reasons | 4 + 4 |
| Faults | 12 bits | 12 |
| Motion Control faults | 3 bits | 3 |
| IMU working | yes / no | 1 |
| Heading | ±180° | 9 |
| Pings | low 16 bits | 16 |
| Wi-Fi link status | −3 to +3 | 3 |
| Connections made, commands lost, network errors | 0–255, stop at 255 | 8 each |
| Records not sent | 0–65,535, stops | 16 |
| Length of the last outage | 0–262 s in ms | 18 |

**The other records**

| Record | Fields (bits) | Size | V12 JSON |
|---|---|---|---|
| Event | header 32, event type 5, payload 32 exactly as in `APP_MSG` | 69 bits = 9 B | 40 B on average, up to 48 B |
| Command ack | header 32, seq 8, accepted 1, error 3 (7 codes) | 44 bits = 6 B | 46 B |
| Obstacle profile | header 32, the 13 B profile (1.5) | 17 B | 108 B |
| Run summary | header 32, run time 24, distance 25, highest hump 8, humps 8, commands 8, obstacles 7, barcodes read 8, failed 8 | 128 bits = 16 B | 107 B, up to 132 B |
| Timing, every 5 s | header 32, longest run of each of 7 tasks 7 × 16, overruns 16, interrupts per second 16, longest interrupt 16, queue peak 6, dropped events 16, steering latency 10, stop latency 17, stopping distance 10, reconnect time 18 | 269 bits = 34 B | 155 B, up to 199 B |
| Heartbeat (kernel template) | 16-bit count | 2 B | text `"hb 5"` |

**Command, 4 B** (V12: text such as `"16 SET base_speed 220"`, up to 64 B)

| Field | Valid range | Bits |
|---|---|---|
| Sequence number | 0–255; new if 1–127 ahead of the last accepted, so it can wrap | 8 |
| Verb | START, STOP, RESTART, FINISH, SET | 3 |
| Parameter (SET only) | 6 tunable values | 3 |
| Spare | must be 0, so an unknown layout is refused rather than misread | 2 |
| Value (SET only) | 0–50,000 (the largest is `ki_x1000`) | 16 |

### 1.10 Shared data and how it is accessed safely

No packing code disables interrupts, and no shared word has two writers that
each do a read-modify-write.

| Shared data | Written by | Read by | Representation | Why it is safe |
|---|---|---|---|---|
| `fault_flags`, `stop_flags` | several tasks | Safety Monitor, Motion Control, Telemetry | kernel event flags, one bit per condition | `tk_set_flg` / `tk_clr_flg`: the kernel does the read-modify-write, safe across both cores |
| Encoder counts | GPIO interrupt (core 1) | Motion Control (core 1) | two 16-bit halves of one word | the interrupt writes each half with one halfword store; the task reads both with one 32-bit load, so both counts are from the same moment |
| Encoder status | GPIO interrupt | Motion Control | one byte per wheel | byte stores by the interrupt only |
| Barcode edge ring | interrupt (entries and head), Line task (tail) | Line task | 64 × 4 B; head and tail in separate bytes | one producer and one consumer; each byte has one writer |
| Echo word | echo interrupt (core 1) | Obstacle Scan (core 0) | width 15 + count 8 bits in one word | one store per echo, one load by the task. The task never writes or clears it; it waits for the count to change. (V12's "done" flag was written from both cores.) |
| Motion snapshot | Motion Control | Line, IMU, VC, Obstacle Scan, Telemetry | 36 B packed | copied in and out under `mtx_snapshot` (priority inheritance) as whole words; packing and unpacking happen outside the lock |
| Obstacle profile | Obstacle Scan | VC, Telemetry | 13 B packed | copied under `mtx_profile` |
| Line status, scan status | their own task | Telemetry | one packed word each | one writer, one store, one load |
| IMU status, VC status | their own task | VC / IMU, Telemetry | three packed words each | one store and one load per word; fields in different words are independent |
| Health stamps | each periodic task, its own slot | Safety Monitor | 5 halfwords (16-bit ms) | a halfword store does not touch the stamp next to it |
| Task timing, dropped-event counts | each task, its own slot | Telemetry | halfwords | one writer per slot |
| Network counters | network task | Telemetry | 3 bytes + 1 word | one writer each |
| Interrupt statistics | GPIO interrupt | Telemetry | 1 word + 1 halfword | interrupt only |
| SET parameters | Telemetry (commands) | Motion Control, VC | array of values | written only while the car is not running; read at START |

### 1.11 Fields kept wider than the minimum, and why

| Field | Needed | Used | Why |
|---|---|---|---|
| Encoder count, each wheel | 6 bits between two reads | 16 | A halfword is the smallest store that keeps both wheels in one word without a read-modify-write, so the task reads both counts from the same moment. 16 bits also survive a read that is late by up to 32,767 counts (16 m), and the low 16 bits go to telemetry. |
| Barcode edge time | about 17 bits | 31 | Version C promised to assess a 23-bit time in a 3 B entry. We keep 31 bits in a 4 B entry because (1) the interrupt writes an edge with one 32-bit store instead of three byte stores, and risk R1 says to keep interrupt work small; (2) with 31 bits no interval can alias before 35.8 min, so a car standing on a barcode cannot produce a false short bar; (3) 3 B entries would save only 64 B. |
| Edge ring head and tail | 6 bits each | 8 each | They have different writers (interrupt and task). One byte each is the smallest unit that is stored without a read-modify-write. |
| Odometer, net distance | 22 bits for 4 km | 24, 25 | The odometer counts all travel since power-up, and 4 km is about 2.5 hours at full speed. We kept 2 bits of margin so it can never wrap during a session. The same 24 bits carry it in `EV_JUNCTION`, the snapshot and telemetry, so it is never rescaled. |
| 16-bit times: event time, snapshot time, health stamps | a few seconds | 16 | 16 bits wrap every 65.5 s and are only subtracted. Safety Monitor keeps a private 32-bit "first seen" time for each stamp, so a task that is silent for more than 65.5 s is never mistaken for a fresh one. |
| Longest task run time | 15 bits (20 ms periods) | 16 | Telemetry's period is 100 ms. 16 bits saturate at 65.5 ms. |
| VC run start time | 32 | 32 | A run can last longer than 65.5 s, and the run time is `app_ms() − run_start_ms`. |
| `net_reconnect_ms` | 18 bits (sent as 18) | 32 stored | One 32-bit store by the network task; a halfword cannot hold 18 bits. |
| `gpio_irq_calls` | more than 16 | 32 | Over 65,535 interrupts in the 5 s timing window; only subtracted. |
| `mqtt_tx` topic index | 2 bits | 8 | The bridge hands the bytes after it straight to lwIP, so the record must start on a byte boundary. |
| Motion command | 39 bits | 8 B | The kernel rounds every message up to 4 B, so 5 to 8 B use the same 12 B slot. |
| Motion snapshot | 277 bits (35 B) | 36 B | Whole words, so the copy under the mutex is a word copy. |

### 1.12 A task's own working values stay 32-bit

These values are never stored for another task, queued or sent:

- **Control arithmetic.** The PI integrator (‰ × 1000, more than 16 bits),
  wheel speed history, ramped speeds, 32-bit wheel positions and the odometer
  remainder live in each task's own state (`s_core` in Motion Control, 420 B).
- **Each task's own state and tuning values.** `s_vc` 444 B, `s_scan` 416 B,
  `s_imu` 392 B, `s_bc` 200 B, `s_sm` 88 B, `s_lc` 52 B: about 2 kB in total,
  including the parameters copied in at START.
- **Unpacked copies.** The logic modules work on unpacked structures
  (`APP_EVT`, `MC_SNAPSHOT`, `OS_PROFILE`, …) on the stack, right after
  `..._unpack()`.

The Cortex-M0+ does all its arithmetic in 32-bit registers. A narrower private
field would need a mask, shift or sign extension every time the 5–20 ms
control code uses it. These values total about 2 kB, so narrowing all of them
would save at most about 1 kB of the RP2040's 264 kB. This is the Version C
rule we defended at the tribunal: the requirement covers application data
that is stored, shared or sent.

---

## 2. RAM before and after

### 2.1 Static RAM in the firmware image

Measured with `arm-none-eabi-size -A` on both images:

| | `.data` | `.bss` | Total |
|---|---|---|---|
| V12 | 204 B | 85,436 B | 85,640 B |
| min-bit | 148 B | 84,948 B | 85,096 B |
| Change | | | **−544 B** |

Most of `.bss` is the kernel, lwIP and the Wi-Fi driver, which this change
does not touch. The application's own variables went from 3,242 B to 2,844 B
(−398 B, −12 %).

### 2.2 The variables that changed

From `arm-none-eabi-nm` on both images. The image's debug information gives
each variable's source file.

| Variable (file) | V12 | min-bit | What changed |
|---|---|---|---|
| `s_buf` (drv_barcode.c) | 512 | 256 | barcode edge 8 B → 4 B, 64 entries |
| `payload` (mqtt_utk.c) | 200 | 34 | MQTT bridge buffer: exactly the largest record (34 B) instead of 200 B of JSON |
| `s_snap` (task_motion.c) | 88 | 36 | packed motion snapshot |
| `s_profile` (task_scan.c) | 48 | 13 | packed obstacle profile |
| `s_enc` (drv_encoder.c) → `s_counts`, `s_stat` + `s_enc` (task_motion.c) | 48 | 4 + 2 + 12 | encoder shared as one word and two bytes; positions moved into the task |
| `s_timing` (app_common.c) | 56 | 28 | task run times and overruns: 16 bits each |
| `s_dropped` (app_common.c) | 32 | 16 | dropped-event counters: 16 bits each |
| `s_health_ms` (task_safety.c) | 20 | 10 | health stamps: 16 bits each |
| `s_status` + `s_status_w` (task_line.c) | 24 | 16 + 4 | private copy + one packed shared word |
| `s_status` + `s_status_w` (task_imu.c) | 40 | 36 + 12 | private copy + three packed shared words |
| `s_status_w` (task_vc.c) | 0 | 12 | three packed shared words |
| `s_status_w` (task_scan.c) | 0 | 4 | one packed shared word |
| `s_sm` (task_safety.c) | 56 | 88 | private "first seen" times that make 16-bit health stamps safe after a wrap (1.11) |
| `s_vc` (task_vc.c) | 432 | 444 | the 8-bit scan request id counter, plus the two line-search settings added this week |
| echo variables (drv_hcsr04.c) | 12 | 8 | `s_armed`, `s_done`, `s_t_fall` replaced by one echo word and the count at trigger |
| `net_up`, network counters (net_bridge.c) | 16 | 3 | `net_up` became the `FLT_WIFI_DOWN` bit; counters 8 bits each |
| sequence numbers (9 variables in 7 files) | 18 | 9 | 8-bit sequence numbers |
| `mtx_profile` (subsystem5.c) | 0 | 4 | the new mutex id for the profile |
| others: `gpio_irq_max_us`, `s_head`, `s_tail`, `s_overflows`, `s_tx_drops`, heartbeat `count` | 24 | 9 | 16-bit and 8-bit counters |
| **Total of every change** | | | **−566 B** (−544 B after alignment) |

### 2.3 Message buffers

The kernel allocates each message buffer once, at start-up. Its size is set by
`MBF_BYTES(slots, size)` in the code: the kernel stores a 4 B header with each
message and rounds the message up to a multiple of 4 B.

| Buffer | Slots | V12 message | V12 bytes | min-bit message | min-bit bytes |
|---|---|---|---|---|---|
| `vc_event_q` | 32 | 16 B | 640 | 8 B | 384 |
| `telemetry_q` | 16 | 16 B | 320 | 8 B | 192 |
| `motion_cmd_q` | 8 | 16 B | 160 | 8 B | 96 |
| `scan_request_q` | 4 | 12 B | 64 | 4 B | 32 |
| `mqtt_tx` | 4 | 201 B | 820 | 35 B | 160 |
| `mqtt_rx` | 4 | 64 B | 272 | 4 B | 32 |
| **Total** | | | **2,276 B** | | **896 B** |

V12's `MBF_BYTES` did not round up as the kernel does, so `mqtt_tx` really held
only 3 of its 4 worst-case messages. The min-bit macro rounds correctly.

### 2.4 Total and flash

- **RAM: −1,924 B** (static −544 B, message buffers −1,380 B). Task stacks
  are the same size in both versions.
- **Flash: +10,500 B** (374,736 → 385,236 B, 0.5 % of the 2 MB flash). The
  pack and unpack code is inlined for speed: `tel_bin.c` is 6.7 kB where
  `tel_json.c` was 3.7 kB, and `mc_core.c` grew by 4.8 kB.

### 2.5 Not measured

The stack each task actually uses was not measured on the car in either
version. Every task has a 4,096 B stack in both. A snapshot reader now holds
the 36 B packed copy as well as the 88 B unpacked structure during the call,
so its stack can be up to 36 B deeper for that moment.

---

## 3. Message size comparison

Measured by `testing/evidence/msg_sizes.c`, which calls each version's own
code with the same values.

### 3.1 Inter-task messages and shared records

| Message | V12 | min-bit | Change |
|---|---|---|---|
| Event (`APP_MSG`) in `vc_event_q`, `telemetry_q` | 16 B | 8 B | −50 % |
| Motion command in `motion_cmd_q` | 16 B | 8 B | −50 % |
| Scan request in `scan_request_q` | 12 B | 4 B | −67 % |
| Motion snapshot under `mtx_snapshot` | 88 B | 36 B | −59 % |
| Obstacle profile | 48 B | 13 B | −73 % |
| Line status | 24 B | 4 B | −83 % |
| IMU status | 40 B | 12 B | −70 % |
| VC status | 32 B | 12 B | −63 % |
| Scan status | 12 B | 4 B | −67 % |

### 3.2 MQTT payloads and packets

A packet is the payload plus the MQTT header and topic name (`car/g32/...`).
TCP/IP adds the same 40 B to each packet in both versions.

| Message | V12 payload | min-bit payload | V12 packet | min-bit packet |
|---|---|---|---|---|
| Telemetry (every 0.5 s) | 153 B (up to 180) | 23 B | 175 B (up to 202) | 44 B |
| Status (every 1 s) | 126 B (up to 181) | 18 B | 145 B (up to 200) | 36 B |
| Event, mean of 12 types | 40 B (up to 48) | 9 B | 57 B (up to 65) | 26 B |
| Command ack | 46 B | 6 B | 63 B | 23 B |
| Obstacle profile | 108 B | 17 B | 125 B | 34 B |
| Run summary | 107 B (up to 132) | 16 B | 124 B (up to 150) | 33 B |
| Timing (every 5 s) | 155 B (up to 199) | 34 B | 173 B (up to 217) | 51 B |
| Command (laptop → car) | 21 B (up to 64) | 4 B | 36 B (up to 79) | 19 B |

The binary records have a fixed size, so the "up to" sizes are gone: the
largest MQTT record is 34 B.

### 3.3 Bytes per second

In steady running the car sends 2 telemetry, 1 status and 0.2 timing messages
a second. Events come on top and depend on the course, so they are left out:

| | V12 | min-bit |
|---|---|---|
| Payload | 463 B/s | 71 B/s |
| MQTT packets | 530 B/s | 134 B/s (−75 %) |

---

## 4. Correctness and performance

### 4.1 No loss of function

All host tests run the firmware's own logic modules on a laptop
(`testing/host/`, built with AddressSanitizer and UndefinedBehaviorSanitizer).
The logs are in `testing/host/logs/` (V12: `2026-10-23_*.txt`; min-bit:
`2026-10-23_min-bit_*.txt`).

| Test | V12 | min-bit |
|---|---|---|
| Unit tests | 140, all pass | 172, all pass |
| Simulator scenarios | 30 scenarios, 153 checks, all pass | 32 scenarios, 162 checks, all pass |
| Robustness: the same 6 simulated cars with different motors and geometry | 5 of 6 pass every check | the same 5 of 6 |
| Laptop MQTT decoder against the firmware's packer | | 13 checks, all pass |

- **Every V12 unit test still runs and passes.** The command and telemetry
  tests (CMD-01 to CMD-12, TEL-01 to TEL-06) now check the same rules on the
  binary formats: refused repeats, ranges, the SET whitelist, SET only while
  stopped, and a record that does not fit is refused. Text-only rules such as
  lower-case verbs and JSON escaping no longer apply.
- **32 new unit tests** cover the packed formats: every record round-trips
  at its limits and saturates outside them (PK-M01–09, PK-S01–07, PK-I01–02,
  EVT-02–04, TEL-07–09). The others cover the command sequence window
  (CMD-13), the 31-bit barcode times and the line status word (BC-07–10),
  the 16-bit health stamps (SM-06–07) and the line search (VC-30).
- **The car behaves exactly as before.** 29 of the 30 V12 scenarios end in
  the same state, position, heading, odometer reading, line deviation, hump
  count and barcode count, to the millimetre. The 30th, `dead_end`, is the line
  search, which now sweeps 180° each way (commits `58f5799` and `8447afc`, a
  behaviour change we asked for). It still ends HALTED and START carries on.
- **Two new scenarios run across the wrap of the 16-bit millisecond times**
  at 65.5 s (`wrap_oval` starts at 60 s, `wrap_barcode_B` reads its barcode
  at 62 s). Neither loses the line or raises a false health timeout, and
  `wrap_barcode_B` ends exactly like `barcode_B`. Unit tests cross the other
  wraps: encoder halves from 65,500 counts (PK-M08), the 8-bit command
  sequence from 250 to 10 (CMD-12), health stamps from 65,000 ms (SM-06,
  SM-07) and the 31-bit barcode time (BC-07).
- **Robustness sweep:** the same single known failure as V12 (car 100 at twice
  the default speed, `line_gap_fast`). Nothing new fails.
- **The firmware builds** in all four variants (with and without MQTT, each
  with and without `TEST=1`) with no warnings. No JSON code is left: the
  `JSON=1` debug build was removed, so every build sends binary only.

### 4.2 CPU cost of each operation

`testing/evidence/bench.c` runs each operation on an emulated Cortex-M0+
(unicorn), compiled exactly as the firmware is (`-O2 -mcpu=cortex-m0plus`).
`count_instr.py` counts instructions and clock cycles with the M0+ timing:
one cycle per instruction, plus one per memory access and one or two per taken
branch, with code in the flash cache. We count 20 cycles for each divide,
about the cost of the Pico SDK's call to the RP2040 hardware divider (the
emulator itself divides in software). Times are at 125 MHz.

| Operation | V12 | min-bit |
|---|---|---|
| Format a telemetry message | 7,422 cycles (59.4 µs) | 1,833 cycles (14.7 µs) |
| Format a status message | 5,081 (40.7 µs) | 1,597 (12.8 µs) |
| Publish one event (min-bit includes the `APP_MSG` pack and unpack) | 1,764 (14.1 µs) | 1,080 (8.6 µs) |
| Read one command | 1,390 (11.1 µs) | 126 (1.0 µs) |
| Copy the snapshot while holding `mtx_snapshot` | 191 (1.53 µs) | 35 (0.28 µs) |
| Pack the snapshot (Motion Control, every 20 ms) | | 2,280 (18.2 µs) |
| Unpack the whole snapshot (IMU, Obstacle Scan, Telemetry) | | 1,311 (10.5 µs) |
| Read only the odometer (Line, VC) | | 28 (0.22 µs) |
| Pack and unpack a motion command | | 803 (6.4 µs) |
| Pack and unpack the obstacle profile | | 1,865 (14.9 µs) |
| Write the line / scan / IMU / VC status | | 77 / 40 / 144 / 84 |
| Read the line / scan / IMU / VC status | 26 / 19 / 38 / 33 (struct copy) | 49 / 34 / 95 / 75 |

Formatting telemetry and status is 3 to 4 times cheaper, reading a command
11 times, and publishing an event 1.6 times even with the extra pack and
unpack. Packing adds work where V12 only copied a struct, so section 4.3
checks that the added time is harmless.

### 4.3 Time added to each task release

Computed by `testing/evidence/cpu_load.py` from the cycle counts above:

| Task | Period | V12 | min-bit | Added | Share of the period |
|---|---|---|---|---|---|
| Motion Control | 20 ms | 1.53 µs | 18.52 µs | +16.99 µs | 0.085 % |
| Line and Barcode | 5 ms | 1.53 µs | 1.12 µs | −0.41 µs | −0.008 % |
| Vehicle Controller | 20 ms | 1.83 µs | 1.94 µs | +0.10 µs | 0.001 % |
| IMU | 10 ms | 1.79 µs | 12.52 µs | +10.73 µs | 0.107 % |
| Obstacle Scan | 10 ms | 1.53 µs | 11.09 µs | +9.56 µs | 0.096 % |

Every task keeps its deadline with a large margin: the largest addition is
17 µs in a 20 ms period, and the largest share of a period is IMU's 0.11 %.
Line and Barcode, the 5 ms task, is slightly faster than in V12 because it
now reads only the odometer from the snapshot.

### 4.4 Load on each core

| | V12 | min-bit | Change |
|---|---|---|---|
| Core 1 (Motion Control, Line, VC) | 59,200 cycles/s = 0.047 % | 196,000 cycles/s = 0.157 % | +0.11 points |
| Core 0 (IMU, Obstacle Scan, Telemetry) | 62,346 cycles/s = 0.050 % | 305,160 cycles/s = 0.244 % | +0.19 points |

The core 1 figure counts a wheel-speed command every 20 ms, which is more than
the car sends. The largest new cost is IMU and Obstacle Scan unpacking the
whole snapshot 100 times a second. They need four and two of its fields, so a
field reader like the odometer one could remove most of it, but 0.2 % of core
0 does not justify more code.

### 4.5 Safe access to shared packed data

- **No interrupt is disabled** for packing. Interrupts only write words or
  halfwords that nothing else writes (1.10).
- **The mutex is held for less time.** `mtx_snapshot` is held for a
  0.28 µs copy instead of 1.53 µs, because the record is smaller and packing
  happens outside the lock.
- **The on-car test image** (`make TEST=1`) checks the binary command format
  and record sizes on the Pico itself (S1-U01 to S1-U09), creates the smaller
  message buffers (S1-U10) and reads the packed snapshot from the running
  control loop (S2-U06, S2-U07).

### 4.6 Conclusion

The packed representation removes 1.9 kB of RAM and 75 % of the MQTT
traffic. It keeps every tested behaviour identical, and costs at most 17 µs
per task release. That is no unacceptable loss of function or timing. The
on-car timing record (section 5) will confirm the emulator figures.

---

## 5. Still to do on the car

These tests are defined in `testing/integration/TEST_LOG.md`. Record each
result there with a saved log, as `testing/README.md` asks.

| Test | What it shows | Version C |
|---|---|---|
| INT-17 | Task timing before and after: run V12 and min-bit on the same loop with MQTT on, and compare the timing records (each task's longest run, overruns). Expect each task at most about 20 µs above V12 (4.3) and 0 overruns. | timing evidence |
| INT-18 | Wraps on the car: 300 commands across the 8-bit sequence wrap, then 3 minutes of driving across two 16-bit time wraps. Expect no command refused and no false health timeout. | wrap test |
| INT-19 | Binary MQTT end to end with the laptop decoder: every record decodes, every command is acknowledged. | Test 4 |
| INT-20 | Fault injection: each fault shows as its own bit; NO_ECHO and WIFI_DOWN never latch. | Test 2 |
| INT-21 | Interrupt load at 300 mm/s over a barcode: encoder error count 0, longest interrupt a few µs. | Test 1 |
| `make TEST=1` | The on-car test image: every check of subsystems 1 to 5 passes on the Pico. S1-U01 to S1-U10, S2-U06 and S2-U07 cover the packed formats and buffers (4.5). Record them in each subsystem's `TEST_LOG.md`. | |

The free RAM on the board was not measured. Section 2 measures what changed:
the image's static RAM and the message buffers, which are the only RAM the
change touches.

---

## 6. How to reproduce

```sh
# 1. The V12 tree, next to this one
git worktree add ../v12 cf0176a

# 2. Build both images as they go on the car (PICO_SDK_PATH set, see README.md)
make -C build_make NET=lwip MQTT=1 -j8
make -C ../v12/robocar-firmware/build_make NET=lwip MQTT=1 -j8

# 3. Sizes, CPU cost, RAM and the per-core table (needs gcc, arm-none-eabi-gcc,
#    and Python 3 with: pip install unicorn pyelftools)
sh testing/evidence/measure.sh ../v12/robocar-firmware
#    -> testing/evidence/results.txt

# 4. Tests, before and after
make -C testing/host all
make -C ../v12/robocar-firmware/testing/host all
(cd testing/host && python3 sweep.py 7 6)     # the robustness sweep, 6 cars
```

| File in `testing/evidence/` | What it does |
|---|---|
| `msg_sizes.c` | message and record sizes, built once against each version |
| `bench.c`, `bench.ld` | one function per operation, built for the Cortex-M0+ |
| `count_instr.py` | runs each one in the emulator; instructions, cycles, µs |
| `cpu_load.py` | per-task and per-core time from the cycle counts |
| `ram_compare.py` | static RAM and every variable whose size changed, from both images |
| `measure.sh` | runs all of the above and writes `results.txt` |
