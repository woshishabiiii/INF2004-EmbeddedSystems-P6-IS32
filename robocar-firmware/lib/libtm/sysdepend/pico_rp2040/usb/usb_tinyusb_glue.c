/* Keep Pico SDK types out of translation units that include µT-Kernel. */

#include "tusb.h"

uint32_t tusb_time_millis_api(void)
{
    volatile uint32_t const *timer_raw_low = (volatile uint32_t const *)0x40054028u;
    return *timer_raw_low / 1000U;
}

int tm_tusb_init(void)
{
    return tusb_init() ? 1 : 0;
}

void tm_tusb_service(void)
{
    tud_task();
}

/* True while the device stack still has queued events to process.  Used to
   drain a whole burst without returning to the scheduler between events. */
int tm_tusb_event_ready(void)
{
    return tud_task_event_ready() ? 1 : 0;
}

/*
 * Enumerated and configured by the host.  This, not tud_cdc_connected(), is
 * the correct gate for draining output: tud_cdc_connected() reports the CDC
 * line state (DTR), which many terminal programs never assert, so gating on
 * it silently discards all output on an otherwise working link.
 */
int tm_tusb_mounted(void)
{
    return tud_mounted() ? 1 : 0;
}

/* DTR from the host.  Diagnostic only -- do not gate output on this. */
int tm_tusb_connected(void)
{
    return tud_cdc_connected() ? 1 : 0;
}

unsigned tm_tusb_write_available(void)
{
    return tud_cdc_write_available();
}

void tm_tusb_write_char(char value)
{
    (void)tud_cdc_write_char(value);
}

void tm_tusb_write_flush(void)
{
    (void)tud_cdc_write_flush();
}

void tm_tusb_discard_input(void)
{
    while (tud_cdc_available()) (void)tud_cdc_read_char();
}

/*
 * Diagnostics.  The console cannot report on the console, so these are read
 * out over GPIO when USB fails to enumerate.
 */
#include "hardware/structs/usb.h"
#include "hardware/regs/usb.h"

uint32_t tm_tusb_diag_controller_en(void)
{
    return (usb_hw->main_ctrl & USB_MAIN_CTRL_CONTROLLER_EN_BITS) ? 1u : 0u;
}

uint32_t tm_tusb_diag_pullup_en(void)
{
    return (usb_hw->sie_ctrl & USB_SIE_CTRL_PULLUP_EN_BITS) ? 1u : 0u;
}

uint32_t tm_tusb_diag_bus_seen(void)
{
    /* Either the host has driven a bus reset, or the SIE reports a
       connection.  Both indicate the host is actually talking to us. */
    return (usb_hw->sie_status &
            (USB_SIE_STATUS_BUS_RESET_BITS | USB_SIE_STATUS_CONNECTED_BITS))
           ? 1u : 0u;
}

uint32_t tm_tusb_diag_inte(void)
{
    return usb_hw->inte;
}
