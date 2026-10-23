/* bench.ld: a flat memory map for the emulator (count_instr.py) */
MEMORY
{
	FLASH (rx)  : ORIGIN = 0x10000000, LENGTH = 2M
	RAM   (rwx) : ORIGIN = 0x20000000, LENGTH = 256K
}
SECTIONS
{
	.text : { *(.text*) *(.rodata*) } > FLASH
	.ARM.exidx : { *(.ARM.exidx*) } > FLASH
	.data : { *(.data*) } > RAM
	.bss : { *(.bss*) *(COMMON) } > RAM
	_stack_top = ORIGIN(RAM) + LENGTH(RAM);
}
