/*
 *	drv_lsm303.c  (see drv_lsm303.h)
 *	Register numbers and values are from the LSM303DLHC datasheet
 *	(DocID018771), section 7.
 */

#include "drv_lsm303.h"
#include "../../device/include/dev_i2c.h"	/* T_I2C_EXEC, i2c_read_reg() */

#define CTRL_REG1_A	0x20U
#define CTRL_REG4_A	0x23U
#define OUT_X_L_A	0x28U
#define AUTO_INCREMENT	0x80U	/* MSB of the sub-address: read several registers */
#define CRA_REG_M	0x00U
#define MR_REG_M	0x02U
#define OUT_X_H_M	0x03U	/* order: X_H X_L Z_H Z_L Y_H Y_L (big-endian) */
#define IRA_REG_M	0x0AU
#define IRA_VALUE	0x48U

#define CTRL1_100HZ_XYZ	0x57U	/* ODR = 0101 (100 Hz), normal power, Z Y X on */
#define CTRL4_BDU_HR	0x88U	/* BDU = 1, +/-2 g, HR = 1 */
#define CRA_15HZ	0x10U	/* magnetometer 15 Hz (the default rate) */
#define MR_CONTINUOUS	0x00U

LOCAL ID	s_dd = 0;

EXPORT ER lsm_read_reg(UW addr, UW reg, UB *val)
{
	return i2c_read_reg(s_dd, addr, reg, val);
}

EXPORT ER lsm_write_reg(UW addr, UW reg, UB val)
{
	return i2c_write_reg(s_dd, addr, reg, val);
}

EXPORT ER lsm_open(void)
{
	UB	id = 0U;
	ER	er;

	if (s_dd <= 0) {
		s_dd = tk_opn_dev((UB*)"iica", TD_UPDATE);
		if (s_dd < E_OK) {
			return s_dd;
		}
	}
	er = lsm_read_reg(LSM_MAG_ADDR, IRA_REG_M, &id);
	if (er < E_OK) {
		return er;
	}
	return (id == IRA_VALUE) ? E_OK : E_IO;
}

EXPORT ER lsm_accel_init(void)
{
	UB	r1 = 0U, r4 = 0U;
	ER	er;

	er = lsm_write_reg(LSM_ACC_ADDR, CTRL_REG1_A, CTRL1_100HZ_XYZ);
	if (er >= E_OK) {
		er = lsm_write_reg(LSM_ACC_ADDR, CTRL_REG4_A, CTRL4_BDU_HR);
	}
	if (er >= E_OK) {
		er = lsm_read_reg(LSM_ACC_ADDR, CTRL_REG1_A, &r1);
	}
	if (er >= E_OK) {
		er = lsm_read_reg(LSM_ACC_ADDR, CTRL_REG4_A, &r4);
	}
	if (er < E_OK) {
		return er;
	}
	return ((r1 == CTRL1_100HZ_XYZ) && (r4 == CTRL4_BDU_HR)) ? E_OK : E_IO;
}

/* 16-bit two's complement, low byte first -> mg.  The data is 12 bits,
   left-justified, so divide by 16 (1 mg per LSB at +/-2 g). */
LOCAL W acc_mg(UB lo, UB hi)
{
	H raw = (H)(UH)(((UH)hi << 8) | (UH)lo);

	return (W)raw / 16;
}

EXPORT ER lsm_accel_read(W *ax, W *ay, W *az)
{
	T_I2C_EXEC	ex;
	UB		reg = (UB)(OUT_X_L_A | AUTO_INCREMENT);
	UB		b[6];
	SZ		asz;
	ER		er;

	ex.sadr = LSM_ACC_ADDR;
	ex.snd_size = 1;
	ex.snd_data = &reg;
	ex.rcv_size = 6;
	ex.rcv_data = b;
	er = tk_swri_dev(s_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &asz);
	if (er < E_OK) {
		return er;
	}
	*ax = acc_mg(b[0], b[1]);
	*ay = acc_mg(b[2], b[3]);
	*az = acc_mg(b[4], b[5]);
	return E_OK;
}

EXPORT ER lsm_mag_init(void)
{
	ER er = lsm_write_reg(LSM_MAG_ADDR, CRA_REG_M, CRA_15HZ);

	if (er >= E_OK) {
		er = lsm_write_reg(LSM_MAG_ADDR, MR_REG_M, MR_CONTINUOUS);
	}
	return (er < E_OK) ? er : E_OK;
}

EXPORT ER lsm_mag_read(W *mx, W *my, W *mz)
{
	T_I2C_EXEC	ex;
	UB		reg = (UB)OUT_X_H_M;
	UB		b[6];
	SZ		asz;
	ER		er;

	ex.sadr = LSM_MAG_ADDR;
	ex.snd_size = 1;
	ex.snd_data = &reg;
	ex.rcv_size = 6;
	ex.rcv_data = b;
	er = tk_swri_dev(s_dd, TDN_I2C_EXEC, &ex, (SZ)sizeof(ex), &asz);
	if (er < E_OK) {
		return er;
	}
	*mx = (W)(H)(UH)(((UH)b[0] << 8) | b[1]);
	*mz = (W)(H)(UH)(((UH)b[2] << 8) | b[3]);
	*my = (W)(H)(UH)(((UH)b[4] << 8) | b[5]);
	return E_OK;
}
