#!/usr/bin/env python3
"""count_instr.py ELF: run every bench_...() of bench.c in an ARM Cortex-M0+
emulator (unicorn) and print the instructions and clock cycles each one takes.

Cycles follow the Cortex-M0+ timing (Technical Reference Manual, table 3-1):
one per instruction, plus one per memory access (LDR/STR 2, LDM/STM/PUSH/POP
1 + N), plus one for a taken branch (B, BX, POP {pc}) or two for BL.  Code is
assumed to be in the XIP cache (no flash wait states) and MULS is one cycle
(the RP2040 has the fast multiplier).

The firmware divides with the RP2040's hardware divider (Pico SDK); this
emulator links the library's software divide instead.  Each call into
__aeabi_*div* is replaced by 12 instructions and 20 cycles (about the SDK's
hardware-divider call), so the totals match the car."""

import sys
from elftools.elf.elffile import ELFFile
from unicorn import (Uc, UC_ARCH_ARM, UC_MODE_THUMB, UC_MODE_MCLASS, UC_HOOK_CODE,
                     UC_HOOK_MEM_READ, UC_HOOK_MEM_WRITE)
from unicorn.arm_const import UC_ARM_REG_SP, UC_ARM_REG_LR

HW_DIV_INSTR = 12
HW_DIV_CYCLES = 20
CPU_MHZ = 125				# SYSCLK of the car (app_main.c)
STOP = 0x10F00000			# return address: the run ends here


def main(path):
    with open(path, "rb") as f:
        elf = ELFFile(f)
        uc = Uc(UC_ARCH_ARM, UC_MODE_THUMB | UC_MODE_MCLASS)
        uc.mem_map(0x10000000, 0x01000000)
        uc.mem_map(0x20000000, 0x00040000)
        for seg in elf.iter_segments():
            if seg["p_type"] == "PT_LOAD" and seg["p_filesz"] > 0:
                uc.mem_write(seg["p_vaddr"], seg.data())
        syms = {}
        for s in elf.get_section_by_name(".symtab").iter_symbols():
            if s["st_info"]["type"] == "STT_FUNC" and s["st_size"] > 0:
                syms[s.name] = (s["st_value"] & ~1, s["st_size"])
        stack = elf.get_section_by_name(".symtab").get_symbol_by_name("_stack_top")[0]["st_value"]
    ranges = [(a, a + n) for k, (a, n) in syms.items()
              if (k.startswith("__aeabi_") and "div" in k) or k in ("__udivsi3", "__divsi3")]
    st = {}

    def code(_uc, addr, size, _data):
        for lo, hi in ranges:
            if lo <= addr < hi:
                if addr == lo:
                    st["calls"] += 1
                st["in_div"] = True
                st["next"] = None		# the call is charged as a whole
                return
        st["in_div"] = False
        if st["next"] is not None and addr != st["next"]:
            st["cyc"] += 2 if st["long"] else 1	# taken branch
        st["n"] += 1
        st["cyc"] += 1
        st["next"] = addr + size
        op = int.from_bytes(_uc.mem_read(addr, 2), "little")
        st["long"] = (size == 4) or ((op & 0xFF00) == 0xBD00)	# BL, or POP {..., pc}

    def mem(_uc, _access, _addr, _size, _value, _data):
        if not st["in_div"]:
            st["cyc"] += 1

    uc.hook_add(UC_HOOK_CODE, code)
    uc.hook_add(UC_HOOK_MEM_READ | UC_HOOK_MEM_WRITE, mem)

    def run(name):
        a, _ = syms[name]
        st.update(n=0, cyc=0, calls=0, in_div=False, next=None, long=False)
        uc.reg_write(UC_ARM_REG_SP, stack)
        uc.reg_write(UC_ARM_REG_LR, STOP | 1)
        uc.emu_start(a | 1, STOP, count=10_000_000)
        return (st["n"] + st["calls"] * HW_DIV_INSTR, st["cyc"] + st["calls"] * HW_DIV_CYCLES,
                st["calls"])

    run("bench_setup")
    for name in sorted(k for k in syms if k.startswith("bench_") and k != "bench_setup"):
        n, cyc, calls = run(name)
        print("%-24s %6d instructions %6d cycles %7.2f us (%d divides)"
              % (name[len("bench_"):], n, cyc, cyc / CPU_MHZ, calls))


if __name__ == "__main__":
    main(sys.argv[1])
