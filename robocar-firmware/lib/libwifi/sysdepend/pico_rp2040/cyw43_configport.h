/* Minimal cyw43-driver environment for the micro T-Kernel polling port. */
#ifndef CYW43_CONFIGPORT_UTK_H
#define CYW43_CONFIGPORT_UTK_H

#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico.h"
#include "hardware/gpio.h"

#define CYW43_HOST_NAME "utk3-picow"
#define CYW43_GPIO 1
#define CYW43_LOGIC_DEBUG 0
#define CYW43_NO_NETUTILS 1
#define CYW43_USE_STATS 0
#define CYW43_PRINTF(...) ((void)0)
#define CYW43_HAL_MAC_WLAN0 0
#define CYW43_USE_SPI 1
#define CYW43_SPI_PIO 1
#define CYW43_WL_GPIO_COUNT 3
#define CYW43_NUM_GPIOS CYW43_WL_GPIO_COUNT
#define CYW43_ARRAY_SIZE(a) count_of(a)

#define CYW43_CHIPSET_FIRMWARE_INCLUDE_FILE "w43439A0_7_95_49_00_combined.h"
#define CYW43_WIFI_NVRAM_INCLUDE_FILE "wifi_nvram_43439.h"

#define CYW43_EPERM     (-PICO_ERROR_NOT_PERMITTED)
#define CYW43_EIO       (-PICO_ERROR_IO)
#define CYW43_EINVAL    (-PICO_ERROR_INVALID_ARG)
#define CYW43_ETIMEDOUT (-PICO_ERROR_TIMEOUT)

#define CYW43_PIN_WL_REG_ON    CYW43_DEFAULT_PIN_WL_REG_ON
#define CYW43_PIN_WL_DATA_OUT  CYW43_DEFAULT_PIN_WL_DATA_OUT
#define CYW43_PIN_WL_DATA_IN   CYW43_DEFAULT_PIN_WL_DATA_IN
#define CYW43_PIN_WL_HOST_WAKE CYW43_DEFAULT_PIN_WL_HOST_WAKE
#define CYW43_PIN_WL_CLOCK     CYW43_DEFAULT_PIN_WL_CLOCK
#define CYW43_PIN_WL_CS        CYW43_DEFAULT_PIN_WL_CS

typedef uint cyw43_hal_pin_obj_t;
extern uint64_t time_us_64(void);

static inline uint32_t cyw43_hal_ticks_us(void) { return (uint32_t)time_us_64(); }
static inline uint32_t cyw43_hal_ticks_ms(void) { return (uint32_t)(time_us_64() / 1000u); }
static inline int cyw43_hal_pin_read(cyw43_hal_pin_obj_t pin) { return gpio_get(pin); }
static inline void cyw43_hal_pin_low(cyw43_hal_pin_obj_t pin) { gpio_put(pin, false); }
static inline void cyw43_hal_pin_high(cyw43_hal_pin_obj_t pin) { gpio_put(pin, true); }

#define CYW43_HAL_PIN_MODE_INPUT GPIO_IN
#define CYW43_HAL_PIN_MODE_OUTPUT GPIO_OUT
#define CYW43_HAL_PIN_PULL_NONE 0
#define CYW43_HAL_PIN_PULL_UP 1
#define CYW43_HAL_PIN_PULL_DOWN 2

static inline void cyw43_hal_pin_config(cyw43_hal_pin_obj_t pin,
        uint32_t mode, uint32_t pull, __unused uint32_t alt)
{
    assert(alt == 0);
    gpio_set_dir(pin, mode);
    gpio_set_pulls(pin, pull == CYW43_HAL_PIN_PULL_UP,
                   pull == CYW43_HAL_PIN_PULL_DOWN);
}

void cyw43_hal_get_mac(int index, uint8_t mac[6]);
void cyw43_hal_generate_laa_mac(int index, uint8_t mac[6]);
void cyw43_thread_enter(void);
void cyw43_thread_exit(void);
void cyw43_await_background_or_timeout_us(uint32_t timeout_us);
void cyw43_delay_ms(uint32_t ms);
void cyw43_delay_us(uint32_t us);
void cyw43_schedule_internal_poll_dispatch(void (*func)(void));
void cyw43_post_poll_hook(void);

#define CYW43_THREAD_ENTER cyw43_thread_enter();
#define CYW43_THREAD_EXIT cyw43_thread_exit();
#define CYW43_THREAD_LOCK_CHECK
#define CYW43_SDPCM_SEND_COMMON_WAIT cyw43_await_background_or_timeout_us(1000);
#define CYW43_DO_IOCTL_WAIT cyw43_await_background_or_timeout_us(1000);
#define CYW43_POST_POLL_HOOK cyw43_post_poll_hook();

#endif
