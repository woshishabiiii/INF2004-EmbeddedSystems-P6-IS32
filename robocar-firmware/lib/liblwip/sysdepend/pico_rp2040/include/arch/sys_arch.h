/*
 *	sys_arch.h
 *	lwIP OS abstraction for micro T-Kernel (RP2040, NO_SYS=1)
 *
 *	NO_SYS=1 means lwIP never creates its own thread, semaphore, or mailbox
 *	-- sys.h supplies stub types and no-op macros for all of that on its
 *	own. Everything that ever calls into lwIP (the radio's poll loop and
 *	any application code) runs on the single processor-1-owned task that
 *	also owns the CYW43439, by the same single-owner-resource rule the
 *	rest of this port already applies to UART/I2C/ADC (see
 *	docs/RELEASE.md, safety rule 6) -- so there is no cross-task or
 *	cross-core race for SYS_ARCH_PROTECT to guard against, which is why
 *	lwipopts.h leaves SYS_LIGHTWEIGHT_PROT at 0 and this file has nothing
 *	to add.
 */

#ifndef LWIP_ARCH_SYS_ARCH_H
#define LWIP_ARCH_SYS_ARCH_H

#endif /* LWIP_ARCH_SYS_ARCH_H */
