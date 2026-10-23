/*
 *	subsystem4.h
 *	Subsystem 4 -- IMU and Terrain + Safety Monitor (Buddy 4).
 *	PUBLIC interface.
 *
 *	Owns the GY-511 (LSM303DLHC) IMU on I2C0 (GP4 = SDA, GP5 = SCL) and
 *	the Robo Pico user buttons GP20 (START / RESTART) and GP21 (STOP).
 */

#ifndef SUBSYSTEM4_H
#define SUBSYSTEM4_H

#include "../app.h"
#include "../app_msg.h"
#include "safety_core.h"	/* FLT_..., HEALTH_... */
#include "imu_core.h"		/* TERRAIN_..., MOTION_... */

#define S4_PRI_SAFETY		7	/* core 0 */
#define S4_PRI_IMU		11	/* core 0 */

/* Any task reports a fault by setting ITS OWN bit:
       tk_set_flg(fault_flags, FLT_...);
   Safety Monitor then latches FAULT_STOP and tells Vehicle Controller. */
IMPORT ID	fault_flags;

IMPORT void	health_stamp(UINT slot);	/* each periodic task, every cycle */
IMPORT BOOL	safety_faulted(void);		/* TRUE while FAULT_STOP is latched */
IMPORT UW	safety_latched_bits(void);	/* the faults behind the latch */

/* Latest IMU results, for telemetry and Vehicle Controller: shared as
   packed words (IC_STATUS in imu_core.h), unpacked by imu_get_status(). */
typedef IC_STATUS IMU_STATUS;

IMPORT void	imu_get_status(IMU_STATUS *out);

/* ---- private to subsystem 4 ---- */
IMPORT ID	tid_safety, tid_imu, cyc_imu;
IMPORT void	safety_task(INT stacd, void *exinf);
IMPORT void	imu_task(INT stacd, void *exinf);
IMPORT void	imu_cyclic_handler(void *exinf);

#endif /* SUBSYSTEM4_H */
