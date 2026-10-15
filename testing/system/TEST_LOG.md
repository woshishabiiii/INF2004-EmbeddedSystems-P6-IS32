# System — test log

Tests of the complete car against the project requirements (design review
section 1), run on the final hardware on a test course.  Run the full set
before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | follows a loop with curves for 3 laps | FR1 | manual | Course loop with the tightest curve of the real course; START. | 3 laps, never leaves the line (no SEARCH in `event`) |
| SYS-02 | barcode commands A, B, C, D at junctions | FR2 | manual | One barcode before each junction type. | the right action every time, 10 runs |
| SYS-03 | distance and turn accuracy | FR3 | manual | S2-M04 and S2-M05 on the course surface | ±10 mm, ±3° |
| SYS-04 | hump detected and highest hump reported | FR4 | manual | Two humps of known height; FINISH at the end. | `hump` events; `summary.hmax` within ±30 % |
| SYS-05 | obstacle on the line: stop, profile, bypass, rejoin | FR5 | manual | Box on a straight and on a gentle curve. | no contact; rejoins; detour shown in a video |
| SYS-06 | no clear side: reverse and retry, then Safe Stop | FR5, FR7 | manual | Box between two walls. | 2 reverses, then Safe Stop with reason 5 |
| SYS-07 | telemetry, commands, recovery over WiFi | FR6 | manual | Full run with MQTT Explorer; drop WiFi for 20 s mid-run. | telemetry throughout; the run continues |
| SYS-08 | fault handling | FR7, NFR3 | manual | Unplug the ultrasonic mid-run; RESTART; START. | Safe Stop within 0.2 s; restart only after replugging |
| SYS-09 | full course | all | manual | The real course, timed, 3 runs. | completes; record time and summary |

*Method:* **manual** = a person observes the result (video, MQTT log, stopwatch, ruler).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
