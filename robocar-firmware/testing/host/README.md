# Host tests and car simulator

The firmware keeps every subsystem's **logic** in plain C files with no
kernel calls (`*_core.c`, `code39.c`, `os_profile.c`, `imu_math.c`,
`cmd_parse.c`, `tel_json.c`). This folder compiles exactly those files for
your laptop and tests them. Nothing here is flashed, and nothing here is
compiled into the firmware (the firmware build only takes `app_program/`).

```sh
cd testing/host
make                 # builds and runs both programs below
./unit_tests         # one line per check, then "UNIT: n checks, m failed"
./sim                # every scenario, then "SIMULATION: n checks, m failed"
./sim -v obstacle_mid      # one scenario with a trace of states and commands
SIM_LINE=1 SIM_HOLD=1 ./sim -v oval_tight   # also every line-sensor change and speed command
SIM_TERRAIN=1 ./sim -v hump    # also every terrain change from the IMU logic
./sim -p 7 200 1.6 135 2.0 0.9   # one car: seed, deadband, kff, track mm, counts/mm, right-motor factor
python3 sweep.py 11 24           # robustness: every scenario on 24 random cars (about 8 min)
```

Saved results of the last full run are in `logs/` (`*_unit.txt`, `*_sim.txt`,
`*_robustness.txt`).

Needs `gcc` and `make` (Linux, macOS, or WSL on Windows). The build uses
`-Wall -Wextra -Werror` plus AddressSanitizer and UndefinedBehaviorSanitizer,
so an out-of-bounds access or an integer overflow in the logic stops the test.

## unit_tests.c

Checks each module on its own: Code 39 (all 43 characters, both directions,
speed changes, misreads), barcode capture (chatter, merged edges), line
sensing and the steering rule, motion control against a small wheel model
(moves, turns, stop latch, stall, encoder wired backwards), IMU maths,
terrain, humps, collisions and motion classes, the Safety Monitor, obstacle
profiles, the scan state machine, the Vehicle Controller's decisions,
command parsing and the telemetry formats.

## sim.c and sim_world.c

A closed-loop simulation of the whole car at 0.1 ms steps: differential
drive with first-order motors (a different deadband, gain and track width
from the firmware's assumptions, and a weaker right motor), quantised
encoders, IR sensors over an 18 mm line and Code 39 bars, a lifted sensor
reading black on a hump, an HC-SR04 with a 15 degree beam and specular
misses, a servo that slews, and an accelerometer with noise and a 1.5 degree
mounting tilt. The glue in `sim.c` mirrors each `task_*.c` file: the same
queues with the same sizes, stop and fault flags, periodic releases, the
Safety Monitor's latch.

| Scenario | What it shows |
|---|---|
| `oval_left`, `oval_right`, `oval_tight`, `oval_fast`, `oval_slow` | line following on R 300 / 200 mm loops at 120–300 mm/s |
| `barcode_A`, `_B`, `_C`, `_D`, `_Z`, `barcode_A_back` | decode at speed, run the command at the junction, follow the new branch |
| `obstacle_mid`, `obstacle_right`, `obstacle_wall` | stop, profile, choose a side, bypass, find and rejoin the line |
| `hump` | a short and a long hump: heights and the highest hump |
| `dead_end` | line search fails: HALTED (a mission failure, nothing latched), START carries on |
| `stop_start` | STOP latency, repeated command, SET, STOP button, FINISH |
| `imu_fault` | sensor failure, restart refused until it recovers |
| `impact` | an obstacle the sonar cannot see: collision detected, Safe Stop |
| `line_gap` | a 60 mm gap in the line |

The simulator checks the **logic** and how the parts work together. It does
not replace the measurements on the real car: the values marked MEASURE in
the configuration headers still have to be measured.
