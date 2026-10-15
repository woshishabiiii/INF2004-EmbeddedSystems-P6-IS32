/* Pico SDK and cyw43-driver hooks supplied by micro T-Kernel.
 * The radio is polled only by a TP_PRC1 service task, so no SDK background
 * interrupt or second scheduler is introduced. */

#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "hardware/claim.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/structs/timer.h"
#include "cyw43.h"

extern int32_t tk_dly_tsk(uint32_t delay);

volatile uint32_t cyw43_utk_poll_requested;

/*
 * Keep the claim bitmaps safe in all supported configurations, the same split
 * usb_console.c's tx_lock_enter() already makes: interrupt masking is
 * sufficient on a single-core build, SMP additionally needs a cross-core lock.
 *
 * CNF_SMP is tested rather than TK_SUPPORT_SMP because it arrives as a -D flag
 * and needs no header; this file lives in the pico-sdk namespace and must not
 * pull in tk/syslib.h, whose size_t typedef collides here (the same reason
 * cyw43_utk_kernel.c exists as a separate translation unit). That is also why
 * the UP path open-codes PRIMASK save/restore instead of using DI()/EI().
 */
#if CNF_SMP
typedef volatile uint32_t T_SPLOCK;
extern int32_t ISpinLock(T_SPLOCK *lock, uint32_t *intsts);
extern int32_t ISpinUnlock(T_SPLOCK *lock, uint32_t intsts);

static T_SPLOCK claim_lock;
#endif

uint32_t hw_claim_lock(void)
{
    uint32_t saved;
#if CNF_SMP
    (void)ISpinLock(&claim_lock, &saved);
#else
    __asm__ volatile("mrs %0, primask" : "=r"(saved));
    __asm__ volatile("cpsid i" ::: "memory");
#endif
    return saved;
}

void hw_claim_unlock(uint32_t saved)
{
#if CNF_SMP
    (void)ISpinUnlock(&claim_lock, saved);
#else
    __asm__ volatile("" ::: "memory");
    __asm__ volatile("msr primask, %0" :: "r"(saved) : "memory");
#endif
}

bool hw_is_claimed(const uint8_t *bits, uint bit_index)
{
    return (bits[bit_index >> 3u] & (1u << (bit_index & 7u))) != 0;
}

void hw_claim_or_assert(uint8_t *bits, uint bit_index, const char *message)
{
    uint32_t saved = hw_claim_lock();
    if(hw_is_claimed(bits, bit_index)) {
        hw_claim_unlock(saved);
        panic(message, bit_index);
    }
    bits[bit_index >> 3u] |= (uint8_t)(1u << (bit_index & 7u));
    hw_claim_unlock(saved);
}

int hw_claim_unused_from_range(uint8_t *bits, bool required, uint bit_lsb,
                               uint bit_msb, const char *message)
{
    uint32_t saved = hw_claim_lock();
    int found = -1;
    uint bit;

    for(bit = bit_lsb; bit <= bit_msb; bit++) {
        if(!hw_is_claimed(bits, bit)) {
            bits[bit >> 3u] |= (uint8_t)(1u << (bit & 7u));
            found = (int)bit;
            break;
        }
    }
    hw_claim_unlock(saved);
    if(found < 0 && required) panic(message);
    return found;
}

void hw_claim_clear(uint8_t *bits, uint bit_index)
{
    uint32_t saved = hw_claim_lock();
    bits[bit_index >> 3u] &= (uint8_t)~(1u << (bit_index & 7u));
    hw_claim_unlock(saved);
}

uint32_t clock_get_hz(clock_handle_t clock)
{
    (void)clock;
    return 125000000u;
}

uint64_t time_us_64(void)
{
    uint32_t hi0, lo, hi1;
    do {
        hi0 = timer_hw->timerawh;
        lo = timer_hw->timerawl;
        hi1 = timer_hw->timerawh;
    } while(hi0 != hi1);
    return ((uint64_t)hi0 << 32) | lo;
}

void cyw43_delay_ms(uint32_t ms)
{
    (void)tk_dly_tsk((ms == 0)? 1: ms);
}

void cyw43_delay_us(uint32_t us)
{
    uint64_t end = time_us_64() + us;
    while((int64_t)(end - time_us_64()) > 0) __asm__ volatile("nop");
}

void cyw43_await_background_or_timeout_us(uint32_t timeout_us)
{
    if(timeout_us >= 1000) (void)tk_dly_tsk(timeout_us / 1000);
    else cyw43_delay_us(timeout_us);
}

void cyw43_schedule_internal_poll_dispatch(void (*func)(void))
{
    (void)func;
    cyw43_utk_poll_requested = 1;
}

void cyw43_post_poll_hook(void) { }
void cyw43_thread_enter(void) { }
void cyw43_thread_exit(void) { }

void cyw43_hal_generate_laa_mac(int index, uint8_t mac[6])
{
    (void)index;
    mac[0] = 0x02;
    mac[1] = 0x55;
    mac[2] = 0x54;
    mac[3] = 0x4b;
    mac[4] = 0x33;
    mac[5] = 0x01;
}

void cyw43_hal_get_mac(int index, uint8_t mac[6])
{
    (void)index;
    memcpy(mac, cyw43_state.mac, 6);
}

#if !TM_NET_LWIP
/* With NET=lwip these five are the real implementations from
 * lib/cyw43-driver/src/cyw43_lwip.c (unmodified upstream) instead --
 * defining both would be a duplicate symbol at link time. */
void cyw43_cb_tcpip_init(cyw43_t *self, int itf) { (void)self; (void)itf; }
void cyw43_cb_tcpip_deinit(cyw43_t *self, int itf) { (void)self; (void)itf; }
void cyw43_cb_tcpip_set_link_up(cyw43_t *self, int itf) { (void)self; (void)itf; }
void cyw43_cb_tcpip_set_link_down(cyw43_t *self, int itf) { (void)self; (void)itf; }
void cyw43_cb_process_ethernet(void *arg, int itf, size_t len,
                              const uint8_t *buf)
{
    (void)arg; (void)itf; (void)len; (void)buf;
}
#endif

void __attribute__((weak)) hard_assertion_failure(void)
{
    __asm__ volatile("cpsid i" ::: "memory");
    for(;;) __asm__ volatile("wfi");
}

void __attribute__((weak, noreturn)) panic(const char *format, ...)
{
    (void)format;
    hard_assertion_failure();
    __builtin_unreachable();
}

/* gpio.c contains optional SDK IRQ registration entry points.  The CYW43
 * polling port never calls them; weak stubs keep the GPIO implementation
 * independent from the SDK's vector-table machinery. */
void __attribute__((weak)) irq_add_shared_handler(uint num,
        irq_handler_t handler, uint8_t order_priority)
{
    (void)num; (void)handler; (void)order_priority;
}
void __attribute__((weak)) irq_remove_handler(uint num, irq_handler_t handler)
{
    (void)num; (void)handler;
}
void __attribute__((weak)) irq_set_enabled(uint num, bool enabled)
{
    (void)num; (void)enabled;
}
