#!/usr/bin/env python3
"""test_mqtt_decode.py -- checks tools/mqtt_decode.py against the firmware.

The host unit tests pack records with the firmware's own code
(tel_bin.c, cmd_parse.c) and write them out:
    testing/host/unit_tests --vectors vectors.txt
This script decodes every one and compares it with the values the unit
test packed (test_tel() in testing/host/unit_tests.c), checks the event
layout table against app_evt.c, and checks that unknown versions, types
and sizes are rejected.  Run by: make -C testing/host decoder
"""

import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import mqtt_decode as md  # noqa: E402

EXPECT = {
    "telemetry": {"type": "telemetry", "t_ms": 0x345678, "state": "FIND_LINE", "v": [-469, 212],
                  "enc16": [(65500 + 100) & 0xFFFF, 0xFFFD], "odo_mm": 123456, "line": "BW",
                  "barcode": "B", "pending": "turn_right", "pitch_cdeg": -1234,
                  "motion": "decelerating", "hmax_mm": 25, "range_mm": "no echo",
                  "yaw_deg_s": -413, "faults": ["STALL", "MOVE_TIMEOUT", "NO_ECHO", "WIFI_DOWN"]},
    "status": {"type": "status", "t_ms": 1000, "state": "SAFE_STOP", "why": "impact",
               "faults": ["IMPACT", "WIFI_DOWN"], "mc_faults": ["encoder"], "imu_ok": 1,
               "heading_deg": -179, "pings": 65535, "wifi": -3, "connects": 3,
               "tx_drops": 5, "rx_drops": 255, "net_errors": 1, "reconnect_ms": 9500},
    "event": {"type": "event", "t_ms": 70000, "ev": "terrain", "terrain": "climbing",
              "pitch_deg": -8.12},
    "ack": {"type": "ack", "t_ms": 5, "seq": 200, "ok": 0, "err": "repeat"},
    "profile": {"type": "profile", "t_ms": 77, "valid": 1, "x_left_mm": -120, "x_right_mm": 60,
                "width_mm": 180, "x_center_mm": -30, "closest_mm": 300, "closest_deg": 95,
                "left_clear_mm": 1000, "right_clear_mm": 240, "unknown_left": 2,
                "unknown_right": 0, "npoints": 9},
    "summary": {"type": "summary", "t_ms": 99000, "time_ms": 95000, "dist_mm": -250, "hmax_mm": 21,
                "humps": 2, "commands": 4, "obstacles": 1, "barcodes_ok": 4, "barcodes_failed": 255},
    "timing": {"type": "timing", "t_ms": 0xFFFFFF,
               "max_us": {"motion": 100, "line": 200, "vc": 300, "scan": 400, "imu": 500,
                          "safety": 600, "telemetry": 65535},
               "overruns": 3, "irq_per_s": 3300, "irq_max_us": 12, "vq_high": 4, "dropped": 0,
               "steer_lat_max_ms": 2, "stop_lat_max_us": 450, "stop_dist_mm": 31,
               "reconnect_ms": 262143},
}


def main(path):
    fails = 0
    checks = 0

    def check(ok, what):
        nonlocal fails, checks
        checks += 1
        if not ok:
            fails += 1
        print("[%s] %s" % ("PASS" if ok else "FAIL", what))

    records = {}
    layouts = {}
    with open(path) as f:
        for line in f:
            w = line.split()
            if w[0] == "layout":
                layouts[int(w[1])] = (int(w[2]), int(w[3]), int(w[4]), int(w[5]))
            else:
                records[w[0]] = bytes.fromhex(w[1])

    for kind, want in EXPECT.items():
        got = md.decode(records[kind])
        diff = {k: (got.get(k), v) for k, v in want.items() if got.get(k) != v}
        check(not diff, "%s: decoded values match what the firmware packed %s" % (kind, diff or ""))

    check(records["cmd_set"] == md.cmd_from_text("201 SET ki_x1000 50000"),
          "command: the decoder encodes the same 4 bytes as cmd_encode() (%s)" % records["cmd_set"].hex())
    table = {t: (e[1], e[2] if e[1] else 0, e[3], e[4] if e[3] else 0) for t, e in md.EVENTS.items()}
    check(table == layouts, "event payload widths and offsets match s_layout[] in app_evt.c")

    bad = bytearray(records["ack"])
    bad[0] = (bad[0] & 0xF0) | 2             # version 2
    try:
        md.decode(bytes(bad))
        check(False, "an unknown version is rejected")
    except md.DecodeError:
        check(True, "an unknown version is rejected")
    try:
        md.decode(records["ack"] + b"\x00")
        check(False, "a record of the wrong size is rejected")
    except md.DecodeError:
        check(True, "a record of the wrong size is rejected")
    try:
        md.cmd_from_text("3 SET base_speed 999")
        check(False, "an out-of-range SET is refused before sending")
    except ValueError:
        check(True, "an out-of-range SET is refused before sending")
    check(md.heartbeat(bytes([0x34, 0x12])) == "heartbeat 4660",
          "the heartbeat is a 2-byte binary count (0x1234 = 4660)")

    print("\nDECODER: %d checks, %d failed" % (checks, fails))
    return 1 if fails else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1] if len(sys.argv) > 1 else "vectors.txt"))
