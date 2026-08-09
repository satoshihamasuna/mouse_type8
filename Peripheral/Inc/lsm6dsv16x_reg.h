/*
 * lsm6dsv16x_reg.h
 *
 *  Created on: Feb 6, 2026
 *      Author: sato1
 */

#ifndef INC_LSM6DSV16X_REG_H_
#define INC_LSM6DSV16X_REG_H_


#ifdef __cplusplus
extern "C" {
#endif

/* ==============================
 * Device ID
 * ============================== */
#define LSM6DSV16X_WHO_AM_I        0x0F
#define LSM6DSV16X_ID              0x70


/* ==============================
 * UI output registers
 * ============================== */
/* Gyroscope */
#define LSM6DSV16X_OUTX_L_G        0x22
#define LSM6DSV16X_OUTX_H_G        0x23
#define LSM6DSV16X_OUTY_L_G        0x24
#define LSM6DSV16X_OUTY_H_G        0x25
#define LSM6DSV16X_OUTZ_L_G        0x26
#define LSM6DSV16X_OUTZ_H_G        0x27

/* Accelerometer */
#define LSM6DSV16X_OUTX_L_A        0x28
#define LSM6DSV16X_OUTX_H_A        0x29
#define LSM6DSV16X_OUTY_L_A        0x2A
#define LSM6DSV16X_OUTY_H_A        0x2B
#define LSM6DSV16X_OUTZ_L_A        0x2C
#define LSM6DSV16X_OUTZ_H_A        0x2D


/* ==============================
 * control registers
 * ============================== */
#define LSM6DSV16X_CTRL1_XL     0x10
#define LSM6DSV16X_CTRL2_G      0x11
#define LSM6DSV16X_CTRL3_C      0x12
#define LSM6DSV16X_CTRL4_C      0x13
#define LSM6DSV16X_CTRL5_C      0x14
#define LSM6DSV16X_CTRL6_G      0x15
#define LSM6DSV16X_CTRL7_C      0x16
#define LSM6DSV16X_CTRL8_XL     0x17
#define LSM6DSV16X_CTRL9_C      0x18
#define LSM6DSV16X_CTRL10_C     0x19

#define LSM6DSV16X_HAODR_CFG   0x62


/* ==============================
 * Accelerometer settings LSM6DSV16X_CTRL1
 * ============================== */
/* OP_MODE_XL [6:4] */
#define LSM6DSV16X_XL_HP_MODE    (0x00 << 4)  // high-performance (default)
#define LSM6DSV16X_XL_HA_MODE    (0x01 << 4)  // high-accuracy
#define LSM6DSV16X_XL_ODR_TRIG   (0x03 << 4)
#define LSM6DSV16X_XL_LP1        (0x04 << 4)
#define LSM6DSV16X_XL_LP2        (0x05 << 4)
#define LSM6DSV16X_XL_LP3        (0x06 << 4)
#define LSM6DSV16X_XL_NORMAL     (0x07 << 4)

/* ODR_XL [3:0] */
#define LSM6DSV16X_XL_ODR_OFF     	(0x00 << 0)
#define LSM6DSV16X_XL_ODR_2HZ    	(0x01 << 0)
#define LSM6DSV16X_XL_ODR_8HZ   	(0x02 << 0)
#define LSM6DSV16X_XL_ODR_15HZ   	(0x03 << 0)
#define LSM6DSV16X_XL_ODR_30HZ    	(0x04 << 0)
#define LSM6DSV16X_XL_ODR_60HZ   	(0x05 << 0)
#define LSM6DSV16X_XL_ODR_120HZ   	(0x06 << 0)
#define LSM6DSV16X_XL_ODR_240HZ   	(0x07 << 0)
#define LSM6DSV16X_XL_ODR_480HZ   	(0x08 << 0)
#define LSM6DSV16X_XL_ODR_960HZ   	(0x09 << 0)
#define LSM6DSV16X_XL_ODR_2KHZ   	(0x0A << 0)
#define LSM6DSV16X_XL_ODR_4KHZ   	(0x0B << 0)
#define LSM6DSV16X_XL_ODR_8KHZ   	(0x0C << 0)


/* ==============================
 * Gyroscope settings LSM6DSV16X_CTRL2
 * ============================== */
/* OP_MODE_G [6:4] */
#define LSM6DSV16X_G_HP_MODE    (0x00 << 4)  // high-performance (default)
#define LSM6DSV16X_G_HA_MODE    (0x01 << 4)  // high-accuracy
#define LSM6DSV16X_G_ODR_TRIG   (0x03 << 4)
#define LSM6DSV16X_G_SLEEP_MODE (0x04 << 4)
#define LSM6DSV16X_G_LP_MODE    (0x05 << 4)

/* ODR_G [3:0] */
#define LSM6DSV16X_G_ODR_OFF     	(0x00 << 0)
#define LSM6DSV16X_G_ODR_8HZ	   	(0x02 << 0)
#define LSM6DSV16X_G_ODR_15HZ   	(0x03 << 0)
#define LSM6DSV16X_G_ODR_30HZ    	(0x04 << 0)
#define LSM6DSV16X_G_ODR_60HZ   	(0x05 << 0)
#define LSM6DSV16X_G_ODR_120HZ   	(0x06 << 0)
#define LSM6DSV16X_G_ODR_240HZ   	(0x07 << 0)
#define LSM6DSV16X_G_ODR_480HZ   	(0x08 << 0)
#define LSM6DSV16X_G_ODR_960HZ   	(0x09 << 0)
#define LSM6DSV16X_G_ODR_2KHZ   	(0x0A << 0)
#define LSM6DSV16X_G_ODR_4KHZ   	(0x0B << 0)
#define LSM6DSV16X_G_ODR_8KHZ   	(0x0C << 0)

/* ==============================
 * CTRL3_C bits
 * ============================== */
//Reboots memory content. This bit is automatically cleared. Default value: 0
#define LSM6DSV16X_BOOT_NORMAL     	(0 << 7)
#define LSM6DSV16X_BOOT_REBOOT     	(1 << 7) // (0: normal mode; 1: reboot memory content)

// output registers are not updated until LSB and MSB have been read)
#define LSM6DSV16X_BDU_EN	      	(1 << 6)	// output registers are not updated until LSB and MSB have been read
#define LSM6DSV16X_BDU_DIS	      	(0 << 6)	// continuous update;

//Register address automatically incremented during a multiple byte access with a serial interface (I²C, MIPI I3C, or SPI). Default value: 1
#define LSM6DSV16X_IF_INC_EN 		(1 << 2)
#define LSM6DSV16X_IF_INC_DIS		(0 << 2)

//Software reset, resets all control registers to their default value. This bit is automatically cleared. Default value: 0
#define LSM6DSV16X_SW_RESET      (1 << 0)

/* ==============================
 * CTRL6_C bits
 * ============================== */
/*LPF1_G_BW_[6:4]*/
#define LSM6DSV16X_GY_LPF1_FTYPE_0   (0x00 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_1   (0x01 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_2   (0x02 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_3   (0x03 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_4   (0x04 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_5   (0x05 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_6   (0x06 << 4)
#define LSM6DSV16X_GY_LPF1_FTYPE_7   (0x07 << 4)

/* FS [3:0] */
#define LSM6DSV16X_GY_FS_125PS   	(0x00 << 0)
#define LSM6DSV16X_GY_FS_250DPS   	(0x01 << 0)
#define LSM6DSV16X_GY_FS_500DPS   	(0x02 << 0)
#define LSM6DSV16X_GY_FS_1000DPS  	(0x03 << 0)
#define LSM6DSV16X_GY_FS_2000DPS  	(0x04 << 0)
#define LSM6DSV16X_GY_FS_4000DPS  	(0x0C << 0)  /* dedicated mode */

/* ===============================
 *  CTRL7 (0x16)
 * =============================== */
/* Bit 7: AH_QVAR_EN */
#define LSM6DSV16X_AH_QVAR_DISABLE    (0x00 << 7)
#define LSM6DSV16X_AH_QVAR_ENABLE     (0x01 << 7)

/* Bit 6: INT2_DRDY_AH_QVAR */
#define LSM6DSV16X_INT2_AH_QVAR_OFF   (0x00 << 6)
#define LSM6DSV16X_INT2_AH_QVAR_ON    (0x01 << 6)

/* Bit [5:4] AH_QVAR_C_ZIN1 */
#define LSM6DSV16X_AH_QVAR_ZIN1_2G4    (0x00 << 4)   /* 2.4 GΩ (default) */
#define LSM6DSV16X_AH_QVAR_ZIN1_730M   (0x01 << 4)   /* 730 MΩ */
#define LSM6DSV16X_AH_QVAR_ZIN0_300M   (0x02 << 4)   /* 300 MΩ */
#define LSM6DSV16X_AH_QVAR_ZIN0_235M   (0x03 << 4)   /* 235 MΩ */

/* Bit 0: LPF1_G_EN */
#define LSM6DSV16X_GY_LPF1_DISABLE    (0x00 << 0)
#define LSM6DSV16X_GY_LPF1_ENABLE     (0x01 << 0)

/* ==============================
 * CTRL8_C bits
 * ============================== */
//Accelerometer LPF2 and HP filter configuration and cutoff setting.
/*HP_LPF2_XL_BW_[7:5]*/
#define LSM6DSV16X_XL_CF_ODR_4     (0x00 << 5)
#define LSM6DSV16X_XL_CF_ODR_10    (0x01 << 5)
#define LSM6DSV16X_XL_CF_ODR_20    (0x02 << 5)
#define LSM6DSV16X_XL_CF_ODR_45    (0x03 << 5)
#define LSM6DSV16X_XL_CF_ODR_100   (0x04 << 5)
#define LSM6DSV16X_XL_CF_ODR_200   (0x05 << 5)
#define LSM6DSV16X_XL_CF_ODR_400   (0x06 << 5)
#define LSM6DSV16X_XL_CF_ODR_800   (0x07 << 5)

//Enables dual-channel mode. When this bit is set to 1, data with the maximum full scale are sent to the output registers at addresses 34h to 39h. The UI processing chain is used. Default value: 0
#define LSM6DSV16X_XL_DualIC_EN		(1 << 3)
#define LSM6DSV16X_XL_DualIC_DIS	(0 << 3)

/* FS [1:0] */
#define LSM6DSV16X_XL_FS_2G       	(0x00 << 0)
#define LSM6DSV16X_XL_FS_4G       	(0x01 << 0)
#define LSM6DSV16X_XL_FS_8G       	(0x02 << 0)
#define LSM6DSV16X_XL_FS_16G      	(0x03 << 0)

/* ==============================
 * CTRL9_C bits
 * ============================== */
//Enables accelerometer high-pass filter reference mode (valid for high-pass path HP_SLOPE_XL_EN bit must be 1). Default value: 0
#define LSM6DSV16X_XL_HP_REF_MODE_XL_EN		(1 << 6)
#define LSM6DSV16X_XL_HP_REF_MODE_XL_DIS	(0 << 6)

#define LSM6DSV16X_XL_FASTSETTL_MODE_EN 	(1 << 5)
#define LSM6DSV16X_XL_FASTSETTL_MODE_DIS 	(0 << 5)

#define LSM6DSV16X_XL_HP_SLOPE_XL_EN  		(1 << 4)
#define LSM6DSV16X_XL_HP_SLOPE_XL_DIS 		(0 << 4)

#define LSM6DSV16X_XL_LPF2_XL_EN			(1 << 3)
#define LSM6DSV16X_XL_LPF2_XL_DIS 			(0 << 3)

#define LSM6DSV16X_XL_USR_OFF_W				(1 << 1)
#define LSM6DSV16X_XL_USR_OFF_ON_OUT		(1 << 0)

/* FS [1:0] */
#define LSM6DSV16X_XL_FS_2G       	(0x00 << 0)
#define LSM6DSV16X_XL_FS_4G       	(0x01 << 0)
#define LSM6DSV16X_XL_FS_8G       	(0x02 << 0)
#define LSM6DSV16X_XL_FS_16G      	(0x03 << 0)

/*HAODR_SEL [1:0]*/
#define LSM6DSV16X_HAODR_HAODR_SEL0 (0x00 << 0)
#define LSM6DSV16X_HAODR_HAODR_SEL1 (0x01 << 0)
#define LSM6DSV16X_HAODR_HAODR_SEL2 (0x02 << 0)

#ifdef __cplusplus
}
#endif



#endif /* INC_LSM6DSV16X_REG_H_ */
