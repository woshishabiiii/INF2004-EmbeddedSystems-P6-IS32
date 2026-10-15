/* Core-0-owned CYW43439 polling service for micro T-Kernel. */

#include "cyw43.h"
#include "cyw43_country.h"
#include "cyw43_utk.h"

#if TM_NET_LWIP
/* lwip_init() must run before cyw43_wifi_set_up() -- cyw43_cb_tcpip_init()
 * (lib/cyw43-driver/src/cyw43_lwip.c, unmodified upstream) calls netif_add()
 * as soon as the interface comes up, from inside that same call. sys_check_
 * timeouts() then needs polling from here for the same reason cyw43_poll()
 * does: NO_SYS=1 means nothing else ever calls it. Both run on this task,
 * which is the single processor-1 owner of the whole radio+network stack
 * (docs/RELEASE.md safety rule 6) -- see lib/liblwip/sysdepend/pico_rp2040/. */
#include "lwip/init.h"
#include "lwip/timeouts.h"
#include "lwip/netif.h"
#include "wifi_secrets.h"
#if TM_MQTT
#include "mqtt_utk.h"
#endif
#endif

#include <string.h>

/* tm/tmonitor.h only pulls in tk/typedef.h (plain B/H/W/D-style typedefs),
 * never tk/tkernel.h/tk/syslib.h -- so it is safe in this pico-sdk/cyw43
 * namespace file, unlike tk/tkernel.h itself (see cc.h's comment on why
 * that one must stay out of here). Needed so join/link failures are
 * actually visible on the console instead of only sitting in radio_status. */
#include <tm/tmonitor.h>

#define CYW43_POLL_MS        2

/* An association can fail transiently -- a busy channel, a weak beacon, an AP
 * that drops the handshake under load. Without a retry the radio would sit in
 * CYW43_LINK_FAIL until the next power cycle, so re-issue the join on a fixed
 * backoff for as long as the link is down. */
#define WIFI_REJOIN_DELAY_US (5 * 1000 * 1000ull)

/* Overridable from the makefile if an AP needs something else. MIXED accepts
 * both TKIP and AES; the driver treats it the same as WPA2_AES_PSK for a
 * pure-AES AP, so it is a strictly wider match than AES-only. */
#ifndef WIFI_AUTH_MODE
#define WIFI_AUTH_MODE       CYW43_AUTH_WPA2_MIXED_PSK
#endif

extern volatile uint32_t cyw43_utk_poll_requested;
extern int32_t tk_dly_tsk(uint32_t delay);
extern uint64_t time_us_64(void);

#if CNF_SMP
extern int32_t tk_get_prc(void);
extern void mp_memory_barrir(void);
#else
/* Neither primitive is built in a single-core kernel -- both live behind
 * TK_SUPPORT_SMP. There is only ever processor 1, and no second core for the
 * publish to become visible to; the barrier still has to order the radio state
 * this task hands to DMA, so it stays as a real one rather than a no-op.
 * CNF_SMP is used here for the same header-avoidance reason as in
 * cyw43_utk_compat.c. */
#define tk_get_prc()        1
#define mp_memory_barrir()  __asm__ volatile("dmb" ::: "memory")
#endif

static T_CYW43_UTK_STATUS radio_status = {
    .init_result = -5,
    .strongest_rssi = -32768,
};

#if TM_NET_LWIP
static const char *link_status_str(int32_t status)
{
    switch(status) {
    case CYW43_LINK_DOWN:   return "down";
    case CYW43_LINK_JOIN:   return "joined AP, waiting for IP";
    case CYW43_LINK_NOIP:   return "joined AP, no IP yet";
    case CYW43_LINK_UP:     return "up";
    case CYW43_LINK_FAIL:   return "FAILED (association failure)";
    case CYW43_LINK_NONET:  return "FAILED: SSID not found (out of range, wrong name, or 5GHz-only network -- this chip is 2.4GHz only)";
    case CYW43_LINK_BADAUTH: return "FAILED: authentication rejected (wrong password, or AP requires an auth mode this chip doesn't support, e.g. WPA3-only)";
    default:                return "unknown";
    }
}
#endif

/* Off by default: it cost ~10s of every boot, flooded the console right before
 * the join, and leaves the radio channel-hopping immediately before the
 * association it is about to attempt. Build with -DWIFI_DIAGNOSTIC_SCAN=1 when
 * a network is not being found and you need to see what the chip can hear. */
#ifndef WIFI_DIAGNOSTIC_SCAN
#define WIFI_DIAGNOSTIC_SCAN 0
#endif

#if TM_NET_LWIP && WIFI_DIAGNOSTIC_SCAN
/* Diagnostic only: prints every beacon the chip actually hears, before the
 * join is attempted. This is what separates "the radio and driver are fine,
 * the AP just rejected us" from "the chip cannot even see that SSID" (wrong
 * name, out of range, or -- the common one for phone hotspots -- a 5GHz-only
 * network, which this 2.4GHz-only chip is physically unable to detect). */
static int scan_print_result(void *env, const cyw43_ev_scan_result_t *result)
{
    UB  ssid[33];
    int i;

    (void)env;
    if(result == NULL) return 0;

    for(i = 0; i < result->ssid_len && i < 32; i++) {
        UB c = result->ssid[i];
        ssid[i] = (c >= 0x20 && c < 0x7f)? c: '?';
    }
    ssid[i] = '\0';

    radio_status.scan_count++;
    if(result->rssi > radio_status.strongest_rssi) {
        radio_status.strongest_rssi = result->rssi;
    }

    tm_printf((UB*)"wifi:   \"%s\"  rssi=%d  ch=%d  auth=%d\n",
              ssid, (int)result->rssi, (int)result->channel,
              (int)result->auth_mode);
    return 0;
}

/* Drives the poll loop by hand until the scan finishes -- the main poll loop
 * further down has not been entered yet at this point, and scan results only
 * arrive as async events processed by cyw43_poll(). */
static void run_diagnostic_scan(void)
{
    cyw43_wifi_scan_options_t opts = {0};
    int32_t                   err;
    int                       i;

    tm_printf((UB*)"wifi: scanning for visible 2.4GHz networks ...\n");

    err = cyw43_wifi_scan(&cyw43_state, &opts, NULL, scan_print_result);
    if(err != 0) {
        tm_printf((UB*)"wifi: scan request failed, err=%d\n", (int)err);
        return;
    }
    cyw43_thread_exit();

    /* ~10s ceiling; a scan normally settles in 2-4s. */
    for(i = 0; i < 1000; i++) {
        bool active;

        cyw43_thread_enter();
        if(cyw43_poll != NULL) cyw43_poll();
        active = cyw43_wifi_scan_active(&cyw43_state);
        cyw43_thread_exit();

        if(!active && i > 30) break;
        (void)tk_dly_tsk(10);
    }

    cyw43_thread_enter();
    tm_printf((UB*)"wifi: scan done -- %d network(s) seen, strongest rssi=%d\n",
              (int)radio_status.scan_count, (int)radio_status.strongest_rssi);
}
#endif

#if TM_NET_LWIP
/* Caller must hold cyw43_thread_enter(). Returns the driver's acceptance of
 * the join *request*; the actual outcome arrives later as a link status.
 *
 * The BSSID argument is NULL, so the chip picks among all APs advertising this
 * SSID. Where two of them are in range (a mesh or a repeater), that choice can
 * change between the association and the handshake and the join fails. Pass a
 * six-byte BSSID here instead of NULL to pin it to one radio. */
static int32_t wifi_try_join(void)
{
    int32_t err;

    tm_printf((UB*)"wifi: joining \"%s\" ...\n", WIFI_SSID);

    err = cyw43_wifi_join(&cyw43_state,
                          strlen(WIFI_SSID), (const uint8_t*)WIFI_SSID,
                          strlen(WIFI_PASSWORD), (const uint8_t*)WIFI_PASSWORD,
                          WIFI_AUTH_MODE, NULL, CYW43_CHANNEL_NONE);
    if(err != 0) {
        tm_printf((UB*)"wifi: cyw43_wifi_join() request rejected, err=%d\n",
                  (int)err);
    }
    return err;
}
#endif

#if !TM_NET_LWIP
static int scan_result(void *env, const cyw43_ev_scan_result_t *result)
{
    T_CYW43_UTK_STATUS *status = env;

    if(result == NULL) return 0;
    status->scan_count++;
    if(result->rssi > status->strongest_rssi) {
        status->strongest_rssi = result->rssi;
    }
    return 0;
}
#endif

void cyw43_utk_task(int32_t stacd, void *exinf)
{
#if !TM_NET_LWIP
    cyw43_wifi_scan_options_t options = {0};
    bool was_scanning;
#endif
    int32_t result;

    (void)stacd;
    (void)exinf;
    radio_status.owner_prc = (uint32_t)tk_get_prc();

    tm_printf((UB*)"wifi: cyw43_utk_task starting (proc %d)\n",
              (int)radio_status.owner_prc);

#if TM_NET_LWIP
    lwip_init();
#endif

    tm_printf((UB*)"wifi: cyw43_init() ...\n");
    cyw43_init(&cyw43_state);
    tm_printf((UB*)"wifi: cyw43_init() returned, bringing interface up ...\n");
    cyw43_thread_enter();
    cyw43_wifi_set_up(&cyw43_state, CYW43_ITF_STA, true,
                      CYW43_COUNTRY_WORLDWIDE);
    if((cyw43_state.itf_state & (1u << CYW43_ITF_STA)) == 0) {
        /* cyw43_wifi_set_up() has a void API and otherwise hides its bus-init
         * error.  Report the underlying I/O class instead of letting scan()
         * replace it with the secondary "interface not up" (-4) result. */
        tm_printf((UB*)"wifi: FAILED -- interface never came up (SPI bus / "
                  "firmware load to the CYW43439 did not succeed)\n");
        result = -CYW43_EIO;
    } else {
        (void)cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA,
                                 radio_status.mac);
#if TM_NET_LWIP
        /* Join instead of scan: with lwIP wired in, the point of coming up
         * is a working network, not a diagnostic scan. cyw43_cb_tcpip_init()
         * already ran (from inside cyw43_wifi_set_up() above) and started
         * DHCP; cyw43_wifi_join() below is what actually gets a beacon to
         * respond to it. */
#if WIFI_DIAGNOSTIC_SCAN
        run_diagnostic_scan();
#endif

        /* Only the *request* outcome. The AP's verdict shows up later as
         * radio_status.link_status, polled below, which is also where a
         * failed association gets retried. */
        result = wifi_try_join();
#else
        result = cyw43_wifi_scan(&cyw43_state, &options, &radio_status,
                                 scan_result);
#endif
    }
    cyw43_thread_exit();

    radio_status.init_result = result;
    radio_status.ready = (result == 0);
#if TM_NET_LWIP
    radio_status.link_status = CYW43_LINK_DOWN;
#else
    radio_status.scan_active = (result == 0);
#endif
    mp_memory_barrir();
    radio_status.init_complete = 1;

    /* A failed bus has nothing useful to poll.  More importantly, repeatedly
     * waking a failed high-rate service must not interfere with USB or the
     * rest of the kernel qualification. */
    if(result != 0) {
        for(;;) (void)tk_dly_tsk(1000);
    }

#if TM_NET_LWIP
    int32_t  prev_link_status = radio_status.link_status;
    uint64_t rejoin_at = 0;
#endif

    for(;;) {
#if TM_NET_LWIP
        cyw43_thread_enter();
        if(cyw43_poll != NULL) cyw43_poll();
        radio_status.poll_count++;
        radio_status.link_status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
        if(radio_status.link_status == CYW43_LINK_UP) {
            radio_status.ip_addr = ip4_addr_get_u32(netif_ip4_addr(&cyw43_state.netif[CYW43_ITF_STA]));
        }
        if(radio_status.link_status != prev_link_status) {
            prev_link_status = radio_status.link_status;
            if(radio_status.link_status == CYW43_LINK_UP) {
                uint32_t ip = radio_status.ip_addr;
                tm_printf((UB*)"wifi: link up, ip=%d.%d.%d.%d\n",
                          (int)(ip & 0xff), (int)((ip >> 8) & 0xff),
                          (int)((ip >> 16) & 0xff), (int)((ip >> 24) & 0xff));
            } else {
                tm_printf((UB*)"wifi: link status -> %s (%d)\n",
                          link_status_str(radio_status.link_status),
                          (int)radio_status.link_status);
            }

            /* Negative == the association attempt is over and lost. Arm a
             * retry; a positive-but-not-UP status (JOIN/NOIP) is still making
             * progress, so leave it alone and let DHCP finish. */
            rejoin_at = (radio_status.link_status < 0)?
                        time_us_64() + WIFI_REJOIN_DELAY_US: 0;
        }

        if(rejoin_at != 0 && time_us_64() >= rejoin_at) {
            rejoin_at = 0;
            if(radio_status.link_status < 0) {
                (void)wifi_try_join();
                /* Force the next iteration to treat whatever comes back as a
                 * change, so the new outcome is always reported. Must be a
                 * value cyw43_tcpip_link_status() can never return (it yields
                 * -3..3), or a genuine repeat failure would be swallowed. */
                prev_link_status = 0x7fffffff;
            }
        }
#else
        was_scanning = radio_status.scan_active != 0;
        cyw43_thread_enter();
        if(cyw43_poll != NULL) cyw43_poll();
        radio_status.poll_count++;
        radio_status.scan_active = cyw43_wifi_scan_active(&cyw43_state);
        if(was_scanning && !radio_status.scan_active) {
            mp_memory_barrir();
            radio_status.scan_complete = 1;
        }
#endif
        cyw43_utk_poll_requested = 0;
        cyw43_thread_exit();
#if TM_NET_LWIP
        sys_check_timeouts();
#if TM_MQTT
        /* mqtt_utk_start() is idempotent (see its own connecting/is_connected
         * guard), so calling it on every poll iteration once the link is up
         * is fine -- it only actually does anything on the first call, or
         * again after connection_cb() schedules a retry. */
        if(radio_status.link_status == CYW43_LINK_UP) {
            mqtt_utk_start();
        }
#endif
#endif
        (void)tk_dly_tsk(CYW43_POLL_MS);
    }
}

void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status)
{
    if(status == NULL) return;
    mp_memory_barrir();
    *status = radio_status;
    mp_memory_barrir();
}
