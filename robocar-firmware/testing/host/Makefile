# Host tests: the firmware's pure logic modules, compiled for this PC.
#   make            build and run the unit tests and the car simulator
#   make unit       unit tests only
#   make sim        simulator only;  ./sim -v obstacle_mid  for a trace
# Needs gcc (Linux, macOS, or WSL on Windows).  Nothing here is flashed.

APP      = ../../app_program
CC      ?= gcc
SAN      = -fsanitize=address,undefined -fno-omit-frame-pointer
CFLAGS   = -std=c99 -g -O1 -Wall -Wextra -Werror $(SAN)
STRICT   = -Wconversion -Wno-sign-conversion -Wshadow -Wmissing-prototypes -Wstrict-prototypes

CORE_SRC = $(APP)/app_evt.c \
           $(APP)/subsystem1/cmd_parse.c $(APP)/subsystem1/tel_json.c \
           $(APP)/subsystem2/mc_core.c \
           $(APP)/subsystem3/line_core.c $(APP)/subsystem3/barcode_core.c $(APP)/subsystem3/code39.c \
           $(APP)/subsystem4/imu_core.c $(APP)/subsystem4/imu_math.c $(APP)/subsystem4/safety_core.c \
           $(APP)/subsystem5/os_profile.c $(APP)/subsystem5/scan_core.c $(APP)/subsystem5/vc_core.c

CORE_OBJ = $(patsubst $(APP)/%.c,obj/%.o,$(CORE_SRC))

all: unit sim
	./unit_tests
	./sim

obj/%.o: $(APP)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(STRICT) -MMD -MP -c -o $@ $<

-include $(CORE_OBJ:.o=.d)

unit: unit_tests
unit_tests: unit_tests.c $(CORE_OBJ) $(wildcard $(APP)/*/*.h) $(wildcard $(APP)/*.h)
	$(CC) $(CFLAGS) -o $@ unit_tests.c $(CORE_OBJ) -lm

sim: sim_bin
sim_bin: sim.c sim_world.c sim_world.h $(CORE_OBJ) $(wildcard $(APP)/*/*.h) $(wildcard $(APP)/*.h)
	$(CC) $(CFLAGS) -o sim sim.c sim_world.c $(CORE_OBJ) -lm

clean:
	rm -rf obj unit_tests sim

.PHONY: all unit sim sim_bin clean
