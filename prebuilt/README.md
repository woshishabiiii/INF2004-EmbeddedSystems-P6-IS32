# Prebuilt images

Built from this tree on 2026-10-23 (branch `min-bit-v12`), dual-core.

Minimum-bit version (Change Requirement 1): every record shared between
tasks, queued or sent uses only the bits its range needs, and **MQTT is
binary only**. The car publishes compact binary records and takes 4-byte
commands, so read and drive it with `python3 tools/mqtt_decode.py --broker
<broker IP>` (CAR_GUIDE.md section 5). The evidence for the change is in
`MIN_BIT_EVIDENCE.md`.

Wiring after swapping encoder A and B on both motors: `MOTOR_DIR_LEFT` 1,
`MOTOR_DIR_RIGHT` -1, `ENC_SIGN_LEFT` 1, `ENC_SIGN_RIGHT` -1 (both encoder
signs flipped on 2026-10-23: rolling forward counted backwards, which drove
the wheels to full power and ended in FAULT 3 after about a second). MQTT topics
use `car/g32/`.

V12 (23 Oct, evening): a little faster and smoother. Line following at
140 mm/s (was 120). Steering corrections are now short ramps instead of
jumps: a wheel's speed changes by at most 8 % of the travel speed per mm
travelled (`MC_STEER_PM_MM`), and the outside wheel is never pushed above
the set speed. In the simulator at 140 mm/s the largest sudden change of
a wheel's speed went from 52 to 31 mm/s, with the same line accuracy.

V11 (23 Oct, afternoon): the car no longer drives straight off curves.
The IMU read 4.4° at rest (its power-up calibration was done tilted) and
kept reporting "climbing", and on a hump the car ignored the line. Now:
on a hump the line is still followed whenever a sensor sees white; level
is re-learned while the car stands still in READY; slopes and humps are
only measured during a run. **Leave the car still on the floor for 2 s
before START.**

What changed since the 2026-10-19 images: no driving behaviour is a fixed
time or a fixed speed any more (CAR_GUIDE.md, end of section 6).

- Gentler by default: `base_speed` 140 mm/s since V12 (120 in V11) and every other speed
  capped at it (`SET base_speed 80` slows the whole car), turns at 100 mm/s,
  `MC_ACCEL` 250 mm/s², `MC_DUTY_RISE` 40 per 20 ms.
- Steering corrections are a fraction of the speed and use millimetres of
  travel, so the car follows the same path at any speed. The line counts as
  lost after 400 mm of travel, not after 2 s.
- Obstacles: the car comes to rest about `stop_mm` (150 mm) from the
  obstacle at any speed. The stop starts earlier the faster it goes, using a
  stopping distance it learns from its own stops.
- Barcode commands, line search and bypass have no fixed time limit. Each
  move or turn has a watchdog of 2 × its own expected time + a margin.
- A stall is now caught while line following (before, the steering refresh
  reset the stall count). An impact is detected at low speeds too. Coasting
  that does not slow the car turns into braking.

| File | Same as | Use |
|---|---|---|
| `car_usb_console.uf2` | `cd build_make; make` | the car; each telemetry record printed in hex on the USB console (`tools/mqtt_decode.py --hex` decodes a line) |
| `car_usb_console_selftest.uf2` | `make TEST=1` | the same, plus every subsystem's `[TEST]` checks at power-up |
| `car_mqtt.uf2` | `make NET=lwip MQTT=1` with your `config/wifi_secrets.h` | WiFi + binary MQTT telemetry and 4-byte commands on `car/g32/` (your hotspot and broker IP are built in); this is the build for the tribunal |

Flash: hold BOOTSEL while plugging in the Pico W, copy the file to the
RPI-RP2 drive. Open the USB serial port at 115200 (CAR_GUIDE.md section 3).

`car_mqtt.uf2` contains your WiFi name, password and broker IP from
`config/wifi_secrets.h` (included in this folder at your request). If the
broker's IP address changes (a new hotspot connection can give the laptop a
new address), edit `config/wifi_secrets.h` and run `make NET=lwip MQTT=1`
again. `.gitignore` keeps `wifi_secrets.h` out of git, so pushing this
folder does not upload the password; keep the zip itself private.
After you change any `MEASURE` value, rebuild instead of using these.

**If the car still drives too fast:** calibrate `MC_COUNTS_PER_M` first
(CAR_GUIDE.md, "If the car is too fast or lurches"). The value 2000 is a
placeholder. If your encoders give fewer counts per metre, the car thinks
it is slower than it really is and drives faster than `base_speed`.
