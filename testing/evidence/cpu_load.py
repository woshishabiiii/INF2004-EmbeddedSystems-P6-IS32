#!/usr/bin/env python3
"""cpu_load.py [results.txt]: the CPU time each core spends on the data
representation, before (V12) and after, from the cycle counts measure.sh
wrote.  Each line is (work, how often per second, V12 bench, min-bit bench):
a bench is a bench_...() name from bench.c, "a+b" adds two, and None means
that version does no such work (V12 stored the fields directly)."""

import re
import sys

MHZ = 125

CORE1 = [
    ("Motion Control: snapshot out", 50, "snapshot_copy_in_lock",
     "snapshot_pack+snapshot_copy_in_lock"),
    ("Line: odometer in", 200, "snapshot_copy_in_lock", "snapshot_copy_in_lock+snapshot_odo"),
    ("Line: status out", 200, None, "status_write_line"),
    ("VC: odometer in (each 20 ms tick)", 50, "snapshot_copy_in_lock",
     "snapshot_copy_in_lock+snapshot_odo"),
    ("VC: status out", 50, None, "status_write_vc"),
    ("VC: IMU status in", 50, "status_read_imu", "status_read_imu"),
    ("VC -> Motion: speed command (at most)", 50, None, "motion_cmd"),
]
CORE0 = [
    ("IMU: snapshot in", 100, "snapshot_copy_in_lock", "snapshot_copy_in_lock+snapshot_unpack"),
    ("IMU: VC status in", 100, "status_read_vc", "status_read_vc"),
    ("IMU: status out", 100, None, "status_write_imu"),
    ("Obstacle Scan: snapshot in", 100, "snapshot_copy_in_lock",
     "snapshot_copy_in_lock+snapshot_unpack"),
    ("Obstacle Scan: status out", 100, None, "status_write_scan"),
    ("Telemetry: telemetry record", 2, "mqtt_telemetry", "mqtt_telemetry"),
    ("Telemetry: status record", 1, "mqtt_status", "mqtt_status"),
    ("Telemetry: snapshot in", 3, "snapshot_copy_in_lock", "snapshot_copy_in_lock+snapshot_unpack"),
    ("Telemetry: line, IMU, VC, scan status in", 3,
     "status_read_line+status_read_imu+status_read_vc+status_read_scan",
     "status_read_line+status_read_imu+status_read_vc+status_read_scan"),
]


def load(path):
    """{"V12": {bench: cycles}, "min-bit": {...}} from results.txt"""
    out, cur = {}, None
    for line in open(path):
        if line.rstrip() in ("V12:", "min-bit:"):
            cur = out.setdefault(line.strip()[:-1], {})
            continue
        m = re.match(r"(\w+)\s+\d+ instructions\s+(\d+) cycles", line)
        if m and cur is not None:
            cur[m.group(1)] = int(m.group(2))
    return out


def cost(table, names):
    return 0 if names is None else sum(table[n] for n in names.split("+"))


def core(title, rows, v12, now):
    print("%s%s %10s %10s" % (title, " " * (52 - len(title)), "V12", "min-bit"))
    ta = tb = 0
    for what, per_s, a, b in rows:
        ca, cb = cost(v12, a) * per_s, cost(now, b) * per_s
        ta += ca
        tb += cb
        print("  %-44s x%-4d %10d %10d" % (what, per_s, ca, cb))
    print("  %-50s %10d %10d cycles/s" % ("total", ta, tb))
    print("  %-50s %9.3f%% %9.3f%% of the core (%+.3f%%)\n"
          % ("", 100.0 * ta / (MHZ * 1e6), 100.0 * tb / (MHZ * 1e6), 100.0 * (tb - ta) / (MHZ * 1e6)))


def per_release(v12, now):
    """the periodic control tasks: added time in each release, against the period"""
    print("time added to each release of a control task (us)")
    tasks = {}
    for what, per_s, a, b in CORE1 + CORE0:
        task = what.split(":")[0]
        if task in ("Motion Control", "Line", "VC", "IMU", "Obstacle Scan"):
            t = tasks.setdefault(task, [per_s, 0, 0])
            t[1] += cost(v12, a)
            t[2] += cost(now, b)
    for task, (per_s, ca, cb) in tasks.items():
        period_us = 1e6 / per_s
        print("  %-16s every %2d ms %7.2f -> %6.2f us  (%+.2f us, %+.3f%% of the period)"
              % (task, round(period_us / 1000), ca / MHZ, cb / MHZ, (cb - ca) / MHZ,
                 100.0 * (cb - ca) / MHZ / period_us))
    print()


def main(path):
    r = load(path)
    v12, now = r["V12"], r["min-bit"]
    print("CPU cycles per second spent on the data representation (%d MHz)\n" % MHZ)
    core("core 1", CORE1, v12, now)
    core("core 0", CORE0, v12, now)
    per_release(v12, now)
    ev_a, ev_b = v12["mqtt_events_x12"] / 12.0, now["mqtt_events_x12"] / 12.0
    print("each event published on MQTT: %.0f -> %.0f cycles (min-bit includes the 8 B"
          " APP_MSG pack and unpack)" % (ev_a, ev_b))
    print("mtx_snapshot held for each copy: %d -> %d cycles (%.2f -> %.2f us)"
          % (v12["snapshot_copy_in_lock"], now["snapshot_copy_in_lock"],
             v12["snapshot_copy_in_lock"] / MHZ, now["snapshot_copy_in_lock"] / MHZ))


if __name__ == "__main__":
    main(sys.argv[1] if len(sys.argv) > 1 else "results.txt")
