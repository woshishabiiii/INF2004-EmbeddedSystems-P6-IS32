/*
 *	drv_lsm303.h
 *	LSM303DLHC (GY-511 board) on I2C0, GP4 = SDA, GP5 = SCL, through the
 *	kernel's I2C driver "iica".  Called only from the IMU and Terrain task.
 */

#ifndef DRV_LSM303_H
#define DRV_LSM303_H

#include "../app.h"

#define LSM_ACC_ADDR		0x19U	/* 7-bit accelerometer address */
#define LSM_MAG_ADDR		0x1EU	/* 7-bit magnetometer address */

/* Open "iica" and check the magnetometer identification register
   (IRA_REG_M, 0x0A, reads 0x48 = 'H').  Returns E_OK or an error. */
IMPORT ER	lsm_open(void);

/* Accelerometer: 100 Hz, all axes, +/-2 g, block data update, high
   resolution (1 mg per LSB).  Reads the two control registers back. */
IMPORT ER	lsm_accel_init(void);

/* One 6-byte burst read; results in mg (1 mg per LSB at +/-2 g). */
IMPORT ER	lsm_accel_read(W *ax, W *ay, W *az);

/* Magnetometer: continuous mode, raw counts (for interference tests). */
IMPORT ER	lsm_mag_init(void);
IMPORT ER	lsm_mag_read(W *mx, W *my, W *mz);

/* Read / write any single register (used by the labs) */
IMPORT ER	lsm_read_reg(UW addr, UW reg, UB *val);
IMPORT ER	lsm_write_reg(UW addr, UW reg, UB val);

#endif /* DRV_LSM303_H */
