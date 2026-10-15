/* Pico SDK hooks needed by TinyUSB, implemented on micro T-Kernel. */

#include <stdarg.h>
#include <stdbool.h>
#include <stdint.h>
#include "pico.h"
#include "hardware/irq.h"

typedef void (*mtk_fp_t)();
typedef struct {
    uint32_t intatr;
    mtk_fp_t inthdr;
} mtk_dint_t;

extern int32_t tk_def_int(uint32_t intno, const mtk_dint_t *definition);
extern int32_t tk_wup_tsk(int32_t tskid);

/*
 * Service task to wake when the controller raises an interrupt.  Enumeration
 * is a chain of control transfers in which each stage is gated on the device
 * responding to the previous one, so servicing on a timer tick puts the tick
 * period into every stage and the host gives up and resets. Waking the task
 * directly from the interrupt removes that latency entirely.
 */
volatile int32_t tm_usb_service_tskid;

#define MTK_TA_HLNG 1U
#define NVIC_ISER0 (*(volatile uint32_t *)0xe000e100u)
#define NVIC_ICER0 (*(volatile uint32_t *)0xe000e180u)

static irq_handler_t usb_irq_handler;

/* Diagnostics: the console cannot report on the console. */
volatile uint32_t tm_usb_irq_count;      /* USB interrupts actually taken */
volatile int32_t  tm_usb_def_int_ercd = 1;  /* tk_def_int() result; 1 = not called */
volatile uint32_t tm_usb_nvic_enabled;   /* NVIC ISER bit for USBCTRL_IRQ */

static void usb_irq_dispatch(uint32_t intno)
{
    (void)intno;
    tm_usb_irq_count++;
    if (usb_irq_handler != NULL) usb_irq_handler();

    /* Hand the queued events to the service task now, not on the next tick.
       tk_wup_tsk() is legal from the task-independent part, and micro
       T-Kernel counts wakeup requests, so one arriving while the task is
       running is not lost -- its next tk_slp_tsk() returns immediately. */
    if (tm_usb_service_tskid > 0) {
        (void)tk_wup_tsk(tm_usb_service_tskid);
    }
}

void irq_add_shared_handler(uint num, irq_handler_t handler, uint8_t order_priority)
{
    mtk_dint_t dint;
    (void)order_priority;
    usb_irq_handler = handler;
    dint.intatr = MTK_TA_HLNG;
    dint.inthdr = (mtk_fp_t)usb_irq_dispatch;
    tm_usb_def_int_ercd = tk_def_int(num, &dint);
}

void irq_remove_handler(uint num, irq_handler_t handler)
{
    (void)handler;
    NVIC_ICER0 = 1UL << num;
    (void)tk_def_int(num, NULL);
    usb_irq_handler = NULL;
}

void irq_set_enabled(uint num, bool enabled)
{
    if (enabled) NVIC_ISER0 = 1UL << num;
    else NVIC_ICER0 = 1UL << num;
    tm_usb_nvic_enabled = (NVIC_ISER0 >> num) & 1UL;
}

uint8_t rp2040_chip_version(void)
{
    volatile uint32_t const *chip_id = (volatile uint32_t const *)0x40000000u;
    return (uint8_t)(*chip_id >> 28);
}

void hard_assertion_failure(void)
{
    NVIC_ICER0 = 1UL << 5;
    for (;;) __asm__ volatile ("wfi");
}

void __attribute__((noreturn)) panic(const char *format, ...)
{
    (void)format;
    hard_assertion_failure();
    __builtin_unreachable();
}
