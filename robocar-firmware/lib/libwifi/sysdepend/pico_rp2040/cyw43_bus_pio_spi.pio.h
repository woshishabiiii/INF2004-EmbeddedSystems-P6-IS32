/* Generated from pico-sdk 2.2.0 cyw43_bus_pio_spi.pio.
 * Only the program selected by the RP2040 Pico W driver is retained. */
#pragma once

#include "hardware/pio.h"

#define spi_gap01_sample0_wrap_target 0
#define spi_gap01_sample0_wrap 5
#define spi_gap01_sample0_pio_version 0
#define spi_gap01_sample0_offset_lp1_end 2u
#define spi_gap01_sample0_offset_end 6u

static const uint16_t spi_gap01_sample0_program_instructions[] = {
    0x6001, /* out pins, 1; side 0 */
    0x1040, /* jmp x--, 0; side 1 */
    0xe080, /* set pindirs, 0; side 0 */
    0xb042, /* nop; side 1 */
    0x4001, /* in pins, 1; side 0 */
    0x1084, /* jmp y--, 4; side 1 */
};

static const struct pio_program spi_gap01_sample0_program = {
    .instructions = spi_gap01_sample0_program_instructions,
    .length = 6,
    .origin = -1,
    .pio_version = spi_gap01_sample0_pio_version,
#if PICO_PIO_VERSION > 0
    .used_gpio_ranges = 0x0
#endif
};

static inline pio_sm_config
spi_gap01_sample0_program_get_default_config(uint offset)
{
    pio_sm_config config = pio_get_default_sm_config();
    sm_config_set_wrap(&config, offset + spi_gap01_sample0_wrap_target,
                       offset + spi_gap01_sample0_wrap);
    sm_config_set_sideset(&config, 1, false, false);
    return config;
}
