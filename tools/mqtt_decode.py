#!/usr/bin/env python3
"""mqtt_decode.py -- the laptop side of the car's binary MQTT messages.

The car publishes compact binary records (app_program/subsystem1/tel_bin.h)
and takes 4-byte binary commands (app_program/subsystem1/cmd_parse.h).
This tool decodes the records and encodes the commands, with exactly the
same layouts.  tools/test_mqtt_decode.py checks it against the firmware's
own packer (the host unit tests write the reference records).

  Decode one record (from a console line or an MQTT client):
      python3 tools/mqtt_decode.py --hex "11 78 56 34 b9 02 ..."
  Encode a command (the bytes to publish on car/<group>/cmd):
      python3 tools/mqtt_decode.py --cmd "7 SET base_speed 300"
  From a subscriber, without paho-mqtt:
      mosquitto_sub -h 192.168.1.10 -t 'car/g32/#' -F '%t %x' | python3 tools/mqtt_decode.py
  Live, with paho-mqtt installed (pip install paho-mqtt):
      python3 tools/mqtt_decode.py --broker 192.168.1.10 [--group g32]
      then type commands such as "1 START" on stdin.

Every field is unsigned on the wire; signed values are sent as value - min.
Records are read LSB first: bit 0 of the record is bit 0 of byte 0.
"""

import argparse
import sys

VERSION = 1

T_TELEMETRY, T_STATUS, T_EVENT, T_ACK, T_PROFILE, T_SUMMARY, T_TIMING = range(1, 8)
SIZES = {T_TELEMETRY: 23, T_STATUS: 18, T_EVENT: 9, T_ACK: 6,
         T_PROFILE: 17, T_SUMMARY: 16, T_TIMING: 34}

STATES = ["STARTUP", "READY", "FOLLOW", "EXEC", "SEARCH", "WAIT_ACK", "SCANNING",
          "BYPASS", "REVERSE", "FIND_LINE", "ALIGN", "SAFE_STOP", "FINISHED", "HALTED"]
WHY = ["none", "fault", "impact", "ack_timeout", "scan_failed", "no_route",
       "search_failed", "blocked", "exec_timeout", "bypass_timeout"]
MOTION = ["stationary", "moving", "accelerating", "decelerating", "turning",
          "climbing", "descending", "impact"]
TERRAIN = ["level", "climbing", "descending"]
NAV = ["?", "pending", "exec", "continue", "bypass", "reverse", "rejoined",
       "search", "found", "finish", "refused"]
LB_CMD = ["none", "turn_left", "turn_right", "straight", "u_turn"]
CMD_ERR = ["ok", "format", "verb", "repeat", "name", "range", "state"]
TASKS = ["motion", "line", "vc", "scan", "imu", "safety", "telemetry"]

# The 12-bit fault field: bits 0-9 latched faults, 10-11 status bits
FAULTS = ["STALL", "LINE", "IMU", "IMPACT", "SCAN", "QUEUE_OVF", "HEALTH", "NAV",
          "ENCODER", "MOVE_TIMEOUT", "NO_ECHO", "WIFI_DOWN"]
MC_FAULTS = ["stall", "timeout", "encoder"]

VERBS = {"START": 1, "STOP": 2, "RESTART": 3, "FINISH": 4, "SET": 5}
PARAMS = [("base_speed", 80, 400), ("stop_mm", 150, 600), ("kp_x1000", 0, 20000),
          ("ki_x1000", 0, 50000), ("dist_cal", 500, 1500), ("turn_cal", 500, 1500)]

# APP_MSG payload per event type: (name, p1 bits, p1 min, p2 bits, p2 min, p1 raw)
# Same table as s_layout[] in app_program/app_evt.c.
EVENTS = {
    1: ("line_status", 1, 0, 1, 0, False),
    2: ("line_lost", 2, -1, 0, 0, False),
    3: ("junction", 24, 0, 0, 0, False),
    4: ("barcode", 6, 32, 3, 0, False),
    5: ("barcode_bad", 7, 0, 6, 0, False),
    6: ("done", 2, 0, 8, 0, False),
    7: ("stop_ack", 17, 0, 10, 0, False),
    8: ("fault", 10, 0, 0, 0, True),
    9: ("fault_clear", 0, 0, 0, 0, False),
    10: ("alert", 12, 0, 8, 0, False),
    11: ("range_zone", 1, 0, 8, 0, False),
    12: ("profile", 1, 0, 8, 0, False),
    13: ("terrain", 2, 0, 14, -8192, False),
    14: ("hump", 8, 0, 8, 0, False),
    15: ("motion", 3, 0, 0, 0, False),
    16: ("impact", 12, 0, 9, 0, False),
    17: ("button", 1, 20, 0, 0, False),
    18: ("command", 3, 0, 0, 0, False),
    19: ("state", 4, 0, 4, 0, False),
    20: ("nav", 4, 0, 25, -16777216, False),
    21: ("profile_data", 0, 0, 0, 0, False),
}


class DecodeError(Exception):
    pass


class Reader:
    """Fields LSB first, as bitpack.h writes them."""

    def __init__(self, data):
        self.v = int.from_bytes(bytes(data), "little")
        self.pos = 0

    def u(self, n):
        x = (self.v >> self.pos) & ((1 << n) - 1)
        self.pos += n
        return x

    def s(self, n, lo):
        return self.u(n) + lo


def name(table, i):
    return table[i] if 0 <= i < len(table) else str(i)


def fault_names(bits):
    return [FAULTS[i] for i in range(12) if bits & (1 << i)]


def range_text(mm):
    return "no echo" if mm == 4095 else mm


def char6(v):
    return chr(v + 32)


def decode_event(etype, pl):
    if etype not in EVENTS:
        raise DecodeError("unknown event type %d" % etype)
    nm, n1, lo1, n2, lo2, raw1 = EVENTS[etype]
    p1 = (pl & ((1 << n1) - 1)) + (0 if raw1 else lo1) if n1 else 0
    p2 = ((pl >> n1) & ((1 << n2) - 1)) + lo2 if n2 else 0
    out = {"ev": nm, "p1": p1, "p2": p2}
    if nm == "state":
        out.update(state=name(STATES, p1), why=name(WHY, p2))
    elif nm == "nav":
        out.update(what=name(NAV, p1), value=p2)
    elif nm == "barcode":
        out.update(c=char6(p1 - 32), cmd=name(LB_CMD, p2))
    elif nm == "terrain":
        out.update(terrain=name(TERRAIN, p1), pitch_deg=p2 / 100.0)
    elif nm == "motion":
        out.update(motion=name(MOTION, p1))
    elif nm == "fault":
        out.update(faults=fault_names(p1))
    elif nm == "line_lost":
        out.update(side={-1: "left", 1: "right"}.get(p1, "none"))
    return out


def decode(data):
    """One record -> dict.  Raises DecodeError for anything it does not know."""
    data = bytes(data)
    if len(data) < 4:
        raise DecodeError("too short (%d bytes)" % len(data))
    r = Reader(data)
    ver, typ, t_ms = r.u(4), r.u(4), r.u(24)
    if ver != VERSION:
        raise DecodeError("unknown version %d (this decoder knows %d)" % (ver, VERSION))
    if typ not in SIZES:
        raise DecodeError("unknown record type %d" % typ)
    if len(data) != SIZES[typ]:
        raise DecodeError("type %d must be %d bytes, got %d" % (typ, SIZES[typ], len(data)))
    out = {"t_ms": t_ms}
    if typ == T_TELEMETRY:
        out["type"] = "telemetry"
        out["state"] = name(STATES, r.u(4))
        out["v"] = [r.s(10, -512), r.s(10, -512)]
        out["enc16"] = [r.u(16), r.u(16)]
        out["odo_mm"] = r.u(24)
        line = r.u(2)
        out["line"] = ("B" if line & 1 else "W") + ("B" if line & 2 else "W")
        out["barcode"] = char6(r.u(6))
        out["pending"] = name(LB_CMD, r.u(3))
        out["pitch_cdeg"] = r.s(14, -8192)
        out["motion"] = name(MOTION, r.u(3))
        out["hmax_mm"] = r.u(8)
        out["range_mm"] = range_text(r.u(12))
        out["yaw_deg_s"] = r.s(10, -512)
        out["faults"] = fault_names(r.u(12))
    elif typ == T_STATUS:
        out["type"] = "status"
        out["state"] = name(STATES, r.u(4))
        out["why"] = name(WHY, r.u(4))
        out["faults"] = fault_names(r.u(12))
        mcf = r.u(3)
        out["mc_faults"] = [MC_FAULTS[i] for i in range(3) if mcf & (1 << i)]
        out["imu_ok"] = r.u(1)
        out["heading_deg"] = r.s(9, -256)
        out["pings"] = r.u(16)
        out["wifi"] = r.s(3, -4)
        out["connects"] = r.u(8)
        out["tx_drops"] = r.u(16)
        out["rx_drops"] = r.u(8)
        out["net_errors"] = r.u(8)
        out["reconnect_ms"] = r.u(18)
    elif typ == T_EVENT:
        out["type"] = "event"
        etype = r.u(5)
        out.update(decode_event(etype, r.u(32)))
    elif typ == T_ACK:
        out["type"] = "ack"
        out["seq"] = r.u(8)
        out["ok"] = r.u(1)
        out["err"] = name(CMD_ERR, r.u(3))
    elif typ == T_PROFILE:
        out["type"] = "profile"
        out["valid"] = r.u(1)
        out["x_left_mm"] = r.s(11, -1024)
        out["x_right_mm"] = r.s(11, -1024)
        out["width_mm"] = r.u(11)
        out["x_center_mm"] = r.s(11, -1024)
        out["closest_mm"] = r.u(10)
        out["closest_deg"] = r.s(7, 30)
        out["left_clear_mm"] = r.u(10)
        out["right_clear_mm"] = r.u(10)
        out["unknown_left"] = r.u(5)
        out["unknown_right"] = r.u(5)
        out["npoints"] = r.u(5)
    elif typ == T_SUMMARY:
        out["type"] = "summary"
        out["time_ms"] = r.u(24)
        out["dist_mm"] = r.s(25, -16777216)
        out["hmax_mm"] = r.u(8)
        out["humps"] = r.u(8)
        out["commands"] = r.u(8)
        out["obstacles"] = r.u(7)
        out["barcodes_ok"] = r.u(8)
        out["barcodes_failed"] = r.u(8)
    elif typ == T_TIMING:
        out["type"] = "timing"
        out["max_us"] = {t: r.u(16) for t in TASKS}
        out["overruns"] = r.u(16)
        out["irq_per_s"] = r.u(16)
        out["irq_max_us"] = r.u(16)
        out["vq_high"] = r.u(6)
        out["dropped"] = r.u(16)
        out["steer_lat_max_ms"] = r.u(10)
        out["stop_lat_max_us"] = r.u(17)
        out["stop_dist_mm"] = r.u(10)
        out["reconnect_ms"] = r.u(18)
    return out


def encode_cmd(seq, verb, param=0, value=0):
    """The 4 command bytes: seq 8 | verb 3 | param 3 | spare 2 | value 16."""
    w = (seq & 0xFF) | ((verb & 7) << 8)
    if verb == VERBS["SET"]:
        w |= ((param & 7) << 11) | ((value & 0xFFFF) << 16)
    return w.to_bytes(4, "little")


def cmd_from_text(text):
    """'<seq> <VERB> [<name> <value>]' -> 4 bytes, checked like the car does."""
    words = text.split()
    if len(words) < 2 or words[1].upper() not in VERBS:
        raise ValueError("expected '<seq> START|STOP|RESTART|FINISH|SET [name value]'")
    seq, verb = int(words[0]), VERBS[words[1].upper()]
    if verb != VERBS["SET"]:
        if len(words) != 2:
            raise ValueError("only SET takes a name and a value")
        return encode_cmd(seq, verb)
    if len(words) != 4:
        raise ValueError("SET needs a name and a value")
    names = [p[0] for p in PARAMS]
    if words[2] not in names:
        raise ValueError("unknown parameter; one of: " + ", ".join(names))
    idx = names.index(words[2])
    value = int(words[3])
    lo, hi = PARAMS[idx][1], PARAMS[idx][2]
    if not lo <= value <= hi:
        raise ValueError("%s must be %d..%d" % (words[2], lo, hi))
    return encode_cmd(seq, verb, idx, value)


def heartbeat(b):
    """the kernel template's heartbeat: a 16-bit count, little-endian"""
    if len(b) != 2:
        raise DecodeError("heartbeat of %d B (expected 2)" % len(b))
    return "heartbeat %d" % (b[0] | (b[1] << 8))


def parse_hex(text):
    """'[TEL] telemetry 11 78 ...' or '117856...' -> bytes"""
    words = [w for w in text.replace(",", " ").split()
             if all(c in "0123456789abcdefABCDEF" for c in w)]
    return bytes.fromhex("".join(words))


def live(broker, group):
    try:
        import paho.mqtt.client as mqtt
    except ImportError:
        sys.exit("live mode needs paho-mqtt: pip install paho-mqtt")
    prefix = "car/%s/" % group

    def on_message(_client, _userdata, msg):
        try:
            print(msg.topic, decode(msg.payload))
        except DecodeError as e:
            print(msg.topic, "REJECTED:", e, msg.payload.hex())

    client = mqtt.Client()
    client.on_message = on_message
    client.connect(broker)
    for topic in ("status", "telemetry", "event"):
        client.subscribe(prefix + topic)
    client.loop_start()
    print("listening on %s#; type commands like '1 START' (Ctrl-D to quit)" % prefix)
    for line in sys.stdin:
        line = line.strip()
        if not line:
            continue
        try:
            payload = cmd_from_text(line)
        except ValueError as e:
            print("not sent:", e)
            continue
        client.publish(prefix + "cmd", payload)
        print("sent", payload.hex())
    client.loop_stop()


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--hex", help="decode one record given as hex")
    ap.add_argument("--cmd", help="encode a command, e.g. '7 SET base_speed 300'")
    ap.add_argument("--broker", help="MQTT broker address for live mode")
    ap.add_argument("--group", default="g32", help="topic group (car/<group>/...)")
    a = ap.parse_args()
    if a.hex:
        try:
            print(decode(parse_hex(a.hex)))
        except DecodeError as e:
            sys.exit("rejected: %s" % e)
    elif a.cmd:
        print(cmd_from_text(a.cmd).hex())
    elif a.broker:
        live(a.broker, a.group)
    else:
        # stdin: the car's console ("[TEL] <topic> <hex bytes>") or a
        # subscriber printing hex, e.g. mosquitto_sub -v -F "%t %x"
        for line in sys.stdin:
            if "[TEL]" in line:
                line = line.split("[TEL]", 1)[1]
            elif not line.startswith("car/"):
                continue
            try:
                if "/heartbeat" in line:
                    print(heartbeat(parse_hex(line)))
                    continue
                print(decode(parse_hex(line)))
            except (DecodeError, ValueError) as e:
                print("REJECTED:", e)


if __name__ == "__main__":
    main()
