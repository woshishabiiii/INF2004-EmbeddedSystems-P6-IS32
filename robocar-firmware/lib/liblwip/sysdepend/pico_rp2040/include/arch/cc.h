/*
 *	cc.h
 *	lwIP compiler/platform abstraction for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	This is deliberately the smallest cc.h that works: everything lwIP can
 *	default sanely (integer widths, format macros) is left alone, and only
 *	what genuinely differs -- struct packing, the diagnostic/assert sinks,
 *	and the PRNG -- is overridden. Modelled on pico-sdk's own
 *	src/rp2_common/pico_lwip/include/arch/cc.h, which is the proven
 *	reference for this exact chip and lwIP version.
 */

#ifndef LWIP_ARCH_CC_H
#define LWIP_ARCH_CC_H

/*
 * Deliberately does NOT include tk/tkernel.h. This header is pulled in by
 * lwip/init.h et al, and those are included from cyw43_utk.c -- which lives
 * in the pico-sdk/cyw43-driver namespace and must never see tk/tkernel.h in
 * the same translation unit (its own comment: "the pico-sdk's ISO C size_t
 * does not collide with the inherited T-Kernel syslib typedef"). Follow the
 * same split the project already uses for that: only forward-declare the
 * three plain-C functions this file needs, implemented where tk/tkernel.h
 * and tm/tmonitor.h are safe to include -- lib/liblwip/sysdepend/
 * pico_rp2040/sys_arch.c, the same way cyw43_utk_kernel.c forward-declares
 * cyw43_utk_task() instead of including the kernel headers itself.
 */

#ifdef __cplusplus
extern "C" {
#endif

#if NO_SYS
/* NO_SYS builds don't use SYS_ARCH_PROTECT (see lwipopts.h:
 * SYS_LIGHTWEIGHT_PROT=0 -- everything that touches lwIP runs on the single
 * radio-owning task, so there is nothing to protect against), but lwip/sys.h
 * still requires the type to exist. */
typedef int sys_prot_t;
#endif

/* GCC struct packing */
#define PACK_STRUCT_BEGIN
#define PACK_STRUCT_STRUCT __attribute__((__packed__))
#define PACK_STRUCT_END
#define PACK_STRUCT_FIELD(x) x

/* lwIP's own debug/stats output is compiled out entirely (LWIP_DEBUG=0,
 * LWIP_STATS_DISPLAY=0 in lwipopts.h), same reasoning as this project's own
 * CYW43_PRINTF(...) ((void)0) a few directories over. */
#define LWIP_PLATFORM_DIAG(x) ((void)0)

/* An lwIP invariant violation is not safe to run past. lwip_utk_assert()
 * (sys_arch.c) reports it on the T-Monitor console and halts. */
void lwip_utk_assert(const char *msg);
#define LWIP_PLATFORM_ASSERT(x) lwip_utk_assert(x)

/* TCP initial sequence numbers etc. Not cryptographic-quality entropy --
 * fine for transport-layer randomization, not a TLS substitute. */
unsigned int lwip_utk_rand(void);
#define LWIP_RAND() ((u32_t)lwip_utk_rand())

#ifdef __cplusplus
}
#endif

#endif /* LWIP_ARCH_CC_H */
