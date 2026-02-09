/*
 * lsm6dsrx_reg.h
 *
 *  Created on: Jul 4, 2025
 *      Author: sato1
 */

#ifndef INC_LSM6DSRX_REG_H_
#define INC_LSM6DSRX_REG_H_

#ifdef __cplusplus
extern "C" {
#endif


/* ==============================
 * Device ID
 * ============================== */
#define LSM6DSRX_WHO_AM_I        0x0F
#define LSM6DSRX_ID              0x6C


/* ==============================
 * Output registers
 * ============================== */
/* Gyroscope */
#define LSM6DSRX_OUTX_L_G 0x22
#define LSM6DSRX_OUTX_H_G 0x23
#define LSM6DSRX_OUTY_L_G 0x24
#define LSM6DSRX_OUTY_H_G 0x25
#define LSM6DSRX_OUTZ_L_G 0x26
#define LSM6DSRX_OUTZ_H_G 0x27

/* Accelerometer */
#define LSM6DSRX_OUTX_L_A 0x28
#define LSM6DSRX_OUTX_H_A 0x29
#define LSM6DSRX_OUTY_L_A 0x2A
#define LSM6DSRX_OUTY_H_A 0x2B
#define LSM6DSRX_OUTZ_L_A 0x2C
#define LSM6DSRX_OUTZ_H_A 0x2D


/* ==============================
 * Control registers
 * ============================== */
#define LSM6DSRX_CTRL1_XL       0x10
#define LSM6DSRX_CTRL2_G        0x11
#define LSM6DSRX_CTRL3_C        0x12
#define LSM6DSRX_CTRL4_C        0x13
#define LSM6DSRX_CTRL5_C        0x14
#define LSM6DSRX_CTRL6_C        0x15
#define LSM6DSRX_CTRL7_G        0x16
#define LSM6DSRX_CTRL8_XL       0x17

/* ==============================
 * Accelerometer settings
 * ============================== */
/* ODR [7:4] */
#define LSM6DSRX_XL_ODR_OFF     (0x00 << 4)
#define LSM6DSRX_XL_ODR_26HZ    (0x02 << 4)
#define LSM6DSRX_XL_ODR_104HZ   (0x04 << 4)
#define LSM6DSRX_XL_ODR_208HZ   (0x05 << 4)
#define LSM6DSRX_XL_ODR_1KHZ    (0x08 << 4)

/* FS [3:2] */
#define LSM6DSRX_XL_FS_2G       (0x00 << 2)
#define LSM6DSRX_XL_FS_4G       (0x02 << 2)
#define LSM6DSRX_XL_FS_8G       (0x03 << 2)
#define LSM6DSRX_XL_FS_16G      (0x01 << 2)

/* LPF2_XL_EN [1] */
#define LSM6DSRX_LPF2_XL_EN      (0x01 << 1)

/* ==============================
 * Gyroscope settings
 * ============================== */
/* ODR [7:4] */
#define LSM6DSRX_GY_ODR_OFF     (0x00 << 4)
#define LSM6DSRX_GY_ODR_26HZ    (0x02 << 4)
#define LSM6DSRX_GY_ODR_104HZ   (0x04 << 4)
#define LSM6DSRX_GY_ODR_208HZ   (0x05 << 4)
#define LSM6DSRX_GY_ODR_1KHZ    (0x08 << 4)

/* FS [3:2] : normal range */
#define LSM6DSRX_GY_FS_250DPS   (0x00 << 2)
#define LSM6DSRX_GY_FS_500DPS   (0x01 << 2)
#define LSM6DSRX_GY_FS_1000DPS  (0x02 << 2)
#define LSM6DSRX_GY_FS_2000DPS  (0x03 << 2)

/* FS [0] : normal range */
#define LSM6DSRX_GY_FS_4000DPS     (0x01 << 0)
/* FS [1] : normal range */
#define LSM6DSRX_GY_FS_125DPS      (0x01 << 1)



/* ==============================
 * CTRL3_C bits
 * ============================== */
#define LSM6DSRX_BDU_ENABLE     (1 << 6)
#define LSM6DSRX_IF_INC_ENABLE (1 << 2)
#define LSM6DSRX_SW_RESET      (1 << 0)

/* ==============================
 * CTRL6_C (0x15) – Gyro LPF1 control
 * ============================== */

/* Accelerometer high-performance disable */
#define LSM6DSRX_XL_HM_MODE_DIS  (1 << 4)

/* Gyroscope LPF1 bandwidth (FTYPE[2:0]) */

/* 208 Hz @ ODR = 1kHz (recommended) */
#define LSM6DSRX_GY_LPF1_FTYPE_0   (0x00 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_1   (0x01 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_2   (0x02 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_3   (0x03 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_4   (0x04 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_5   (0x05 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_6   (0x06 << 0)
#define LSM6DSRX_GY_LPF1_FTYPE_7   (0x07 << 0)



/* ==============================
 * CTRL8_XL bits
 * ============================== */
/* High-pass cutoff frequency selection [7:5] */
#define LSM6DSRX_XL_HPCF_ODR_4     (0x00 << 5)
#define LSM6DSRX_XL_HPCF_ODR_10    (0x01 << 5)
#define LSM6DSRX_XL_HPCF_ODR_20    (0x02 << 5)
#define LSM6DSRX_XL_HPCF_ODR_45    (0x03 << 5)
#define LSM6DSRX_XL_HPCF_ODR_100   (0x04 << 5)
#define LSM6DSRX_XL_HPCF_ODR_200   (0x05 << 5)
#define LSM6DSRX_XL_HPCF_ODR_400   (0x06 << 5)
#define LSM6DSRX_XL_HPCF_ODR_800   (0x07 << 5)

/* Reference mode enable */
#define LSM6DSRX_XL_HP_REF_MODE   (1 << 4)

/* Fast settling enable */
#define LSM6DSRX_XL_FAST_SETTLE   (1 << 3)

/* Slope / high-pass enable */
#define LSM6DSRX_XL_HP_SLOPE_EN    (1 << 2)
#define LSM6DSRX_XL_HP_SLOPE_DIS   (0 << 2)

/* Low-pass filter on 6D function */
#define LSM6DSRX_XL_LPF_ON_6D     (1 << 0)



#ifdef __cplusplus
}
#endif



#endif /* INC_LSM6DSRX_REG_H_ */


