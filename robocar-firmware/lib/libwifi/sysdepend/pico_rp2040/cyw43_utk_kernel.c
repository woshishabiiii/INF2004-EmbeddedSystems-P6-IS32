/* Kernel-facing half of the CYW43 service.  Kept separate so the pico-sdk's
 * ISO C size_t does not collide with the inherited T-Kernel syslib typedef. */

#include <tk/tkernel.h>

IMPORT void cyw43_utk_task(INT stacd, void *exinf);

/* USB CDC drains at priority 3 on the same physical core.  Keep radio polling
 * below it so firmware download and the 2 ms poll cadence cannot overflow the
 * console ring, while remaining well above the qualification task. */
#define CYW43_TASK_PRIORITY  4
#if TM_NET_LWIP
/* lwIP's own call chains (tcp_input/dhcp/etharp) run on this same task and
 * go deeper than the bare radio poll loop did. */
#define CYW43_TASK_STACK     (8 * 1024)
#else
#define CYW43_TASK_STACK     (6 * 1024)
#endif

EXPORT ER cyw43_utk_start(void)
{
    T_CTSK ctsk = {0};
    ID task;

    /* Pinned to processor 1 (core 0) for the same reason usb_console.c pins
     * its service task: the CYW43439's PIO/DMA and pin ownership, and the
     * lwIP state serviced alongside it, must not migrate between cores. On a
     * single-core build there is only one processor, so the affinity request
     * is dropped rather than being passed an ID the kernel does not define. */
#if TK_SUPPORT_SMP
    ctsk.tskatr = TA_HLNG | TA_RNG1 | TA_ASSPRC;
    ctsk.assprc = TP_PRC1;
#else
    ctsk.tskatr = TA_HLNG | TA_RNG1;
    ctsk.assprc = 0;
#endif
	ctsk.task = (FP)cyw43_utk_task;
    ctsk.itskpri = CYW43_TASK_PRIORITY;
    ctsk.stksz = CYW43_TASK_STACK;
#if USE_OBJECT_NAME
    ctsk.dsname = (UB*)"cyw43";
#endif

    task = tk_cre_tsk(&ctsk);
    if(task < E_OK) return task;
    return tk_sta_tsk(task, 0);
}
