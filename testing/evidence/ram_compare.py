#!/usr/bin/env python3
"""ram_compare.py V12_FOLDER NOW_FOLDER: RAM before and after, from the
firmware images.  Build both first with "make NET=lwip MQTT=1" in their
build_make folders.  Prints the whole image's static RAM (.data + .bss,
arm-none-eabi-size), the application's variables per source file, and every
RAM variable whose size changed (arm-none-eabi-nm -l: the image's debug
information gives each variable's source file, so only that image is read)."""

import collections
import os
import subprocess
import sys

IMAGE = "build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf"


def static_ram(root):
    out = subprocess.run(["arm-none-eabi-size", "-A", os.path.join(root, IMAGE)],
                         capture_output=True, text=True, check=True).stdout
    size = {f[0]: int(f[1]) for f in (l.split() for l in out.splitlines())
            if len(f) == 3 and f[1].isdigit()}
    return size[".data"], size[".bss"]


def ram_vars(root):
    """{(source file, variable): bytes} for every RAM variable of the image"""
    out = subprocess.run(["arm-none-eabi-nm", "-S", "-l", "--defined-only",
                          os.path.join(root, IMAGE)],
                         capture_output=True, text=True, check=True).stdout
    top = os.path.realpath(root)
    found = {}
    for line in out.splitlines():
        sym, _, where = line.partition("\t")
        f = sym.split()
        if len(f) == 4 and f[2] in "BbDd":
            src = os.path.realpath(where.rsplit(":", 1)[0]) if where else "?"
            src = os.path.relpath(src, top) if src.startswith(top) else src
            found[(src, f[3])] = found.get((src, f[3]), 0) + int(f[1], 16)
    return found


def main(v12, now):
    for name, root in (("V12", v12), ("min-bit", now)):
        d, b = static_ram(root)
        print("%-8s whole image: .data %5d + .bss %6d = %6d B static RAM" % (name, d, b, d + b))
    a, b = ram_vars(v12), ram_vars(now)
    per_a, per_b = collections.Counter(), collections.Counter()
    for (src, _), n in a.items():
        if src.startswith("app_program/"):
            per_a[src[len("app_program/"):]] += n
    for (src, _), n in b.items():
        if src.startswith("app_program/"):
            per_b[src[len("app_program/"):]] += n
    print("\napplication variables (app_program), per source file: V12 -> min-bit")
    for src in sorted(set(per_a) | set(per_b)):
        print("  %-30s %5d -> %5d  (%+d)" % (src, per_a[src], per_b[src], per_b[src] - per_a[src]))
    print("  %-30s %5d -> %5d  (%+d)" % ("TOTAL", sum(per_a.values()), sum(per_b.values()),
                                          sum(per_b.values()) - sum(per_a.values())))
    print("\nvariables whose size changed (whole image):")
    total = 0
    for key in sorted(set(a) | set(b)):
        if a.get(key, 0) != b.get(key, 0):
            total += b.get(key, 0) - a.get(key, 0)
            print("  %-40s %-18s %5d -> %5d" % (key[0], key[1], a.get(key, 0), b.get(key, 0)))
    print("  %-40s %-18s %+d B" % ("TOTAL", "", total))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
