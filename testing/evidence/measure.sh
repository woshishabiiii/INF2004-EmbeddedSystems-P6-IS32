#!/bin/sh
# Week 13 evidence: message sizes, CPU cost and RAM, before (V12) and after.
#   sh measure.sh <V12 robocar-firmware folder>     (for example a
#   "git worktree add ../v12 cf0176a" of the V12 import)
# Needs gcc, arm-none-eabi-gcc and Python 3 with unicorn and pyelftools
# (pip install unicorn pyelftools).  The RAM part runs only when both trees
# were last built with "make NET=lwip MQTT=1" in build_make.
# Prints every table; results.txt keeps them.
set -e
V12=${1:?usage: sh measure.sh <V12 robocar-firmware folder>}
HERE=$(cd "$(dirname "$0")" && pwd)
NOW=$(cd "$HERE/../.." && pwd)
OUT=$(mktemp -d)
core() {
	A=$1/app_program
	echo $A/app_evt.c $A/subsystem1/cmd_parse.c $A/subsystem2/mc_core.c \
	     $A/subsystem3/line_core.c $A/subsystem3/barcode_core.c $A/subsystem3/code39.c \
	     $A/subsystem4/imu_core.c $A/subsystem4/imu_math.c $A/subsystem4/safety_core.c \
	     $A/subsystem5/os_profile.c $A/subsystem5/scan_core.c $A/subsystem5/vc_core.c
}
ARM="arm-none-eabi-gcc -mcpu=cortex-m0plus -mthumb -O2 -ffreestanding -std=c99 --specs=nano.specs -nostartfiles -T $HERE/bench.ld"

gcc -std=c99 -O1 -DV12 -I"$V12/app_program" -o "$OUT/sizes_v12" "$HERE/msg_sizes.c" $(core "$V12") "$V12/app_program/subsystem1/tel_json.c" -lm
gcc -std=c99 -O1 -I"$NOW/app_program" -o "$OUT/sizes_now" "$HERE/msg_sizes.c" $(core "$NOW") "$NOW/app_program/subsystem1/tel_bin.c" -lm
$ARM -DV12 -I"$V12/app_program" -o "$OUT/bench_v12.elf" "$HERE/bench.c" $(core "$V12") "$V12/app_program/subsystem1/tel_json.c" -lm
$ARM -I"$NOW/app_program" -o "$OUT/bench_now.elf" "$HERE/bench.c" $(core "$NOW") "$NOW/app_program/subsystem1/tel_bin.c" -lm

{
	echo "== message sizes =="
	"$OUT/sizes_v12"; echo; "$OUT/sizes_now"
	echo; echo "== CPU cost on the Cortex-M0+ at 125 MHz (emulator) =="
	echo "V12:"; python3 "$HERE/count_instr.py" "$OUT/bench_v12.elf"
	echo "min-bit:"; python3 "$HERE/count_instr.py" "$OUT/bench_now.elf"
	ELF=build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt.elf
	if [ -f "$V12/$ELF" ] && [ -f "$NOW/$ELF" ]; then
		echo; echo "== RAM (firmware images, NET=lwip MQTT=1) =="
		python3 "$HERE/ram_compare.py" "$V12" "$NOW"
	fi
} > "$OUT/results.txt"
{ echo; python3 "$HERE/cpu_load.py" "$OUT/results.txt"; } >> "$OUT/results.txt"
cp "$OUT/results.txt" "$HERE/results.txt"
cat "$HERE/results.txt"
rm -rf "$OUT"
