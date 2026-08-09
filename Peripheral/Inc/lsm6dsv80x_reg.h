/*
 * lsm6dsv80x_reg.h
 *
 * Register definitions for the STMicroelectronics LSM6DSV80X.
 * Based on datasheet DS14764 Rev 2.
 */

#ifndef INC_LSM6DSV80X_REG_H_
#define INC_LSM6DSV80X_REG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* ==============================
 * Device ID
 * ============================== */
#define LSM6DSV80X_WHO_AM_I             0x0FU
#define LSM6DSV80X_ID                   0x73U

/* ==============================
 * Output registers
 * ============================== */
/* Temperature */
#define LSM6DSV80X_OUT_TEMP_L            0x20U
#define LSM6DSV80X_OUT_TEMP_H            0x21U

/* Gyroscope */
#define LSM6DSV80X_OUTX_L_G              0x22U
#define LSM6DSV80X_OUTX_H_G              0x23U
#define LSM6DSV80X_OUTY_L_G              0x24U
#define LSM6DSV80X_OUTY_H_G              0x25U
#define LSM6DSV80X_OUTZ_L_G              0x26U
#define LSM6DSV80X_OUTZ_H_G              0x27U

/* Low-g accelerometer */
#define LSM6DSV80X_OUTX_L_A              0x28U
#define LSM6DSV80X_OUTX_H_A              0x29U
#define LSM6DSV80X_OUTY_L_A              0x2AU
#define LSM6DSV80X_OUTY_H_A              0x2BU
#define LSM6DSV80X_OUTZ_L_A              0x2CU
#define LSM6DSV80X_OUTZ_H_A              0x2DU

/* High-g accelerometer */
#define LSM6DSV80X_UI_OUTX_L_A_HG        0x34U
#define LSM6DSV80X_UI_OUTX_H_A_HG        0x35U
#define LSM6DSV80X_UI_OUTY_L_A_HG        0x36U
#define LSM6DSV80X_UI_OUTY_H_A_HG        0x37U
#define LSM6DSV80X_UI_OUTZ_L_A_HG        0x38U
#define LSM6DSV80X_UI_OUTZ_H_A_HG        0x39U

/* ==============================
 * Main-page registers
 * ============================== */
#define LSM6DSV80X_CTRL1                 0x10U
#define LSM6DSV80X_CTRL2                 0x11U
#define LSM6DSV80X_CTRL3                 0x12U
#define LSM6DSV80X_CTRL4                 0x13U
#define LSM6DSV80X_CTRL5                 0x14U
#define LSM6DSV80X_CTRL6                 0x15U
#define LSM6DSV80X_CTRL7                 0x16U
#define LSM6DSV80X_CTRL8                 0x17U
#define LSM6DSV80X_CTRL9                 0x18U
#define LSM6DSV80X_CTRL10                0x19U
#define LSM6DSV80X_STATUS_REG            0x1EU
#define LSM6DSV80X_UI_STATUS_REG         0x44U
#define LSM6DSV80X_CTRL2_XL_HG           0x4DU
#define LSM6DSV80X_CTRL1_XL_HG           0x4EU
#define LSM6DSV80X_HAODR_CFG             0x62U

/* Compatibility names used by the other IMU register headers. */
#define LSM6DSV80X_CTRL1_XL              LSM6DSV80X_CTRL1
#define LSM6DSV80X_CTRL2_G               LSM6DSV80X_CTRL2
#define LSM6DSV80X_CTRL3_C               LSM6DSV80X_CTRL3
#define LSM6DSV80X_CTRL4_C               LSM6DSV80X_CTRL4
#define LSM6DSV80X_CTRL5_C               LSM6DSV80X_CTRL5
#define LSM6DSV80X_CTRL6_G               LSM6DSV80X_CTRL6
#define LSM6DSV80X_CTRL7_C               LSM6DSV80X_CTRL7
#define LSM6DSV80X_CTRL8_XL              LSM6DSV80X_CTRL8
#define LSM6DSV80X_CTRL9_C               LSM6DSV80X_CTRL9

/* ==============================
 * CTRL1: low-g accelerometer
 * ============================== */
/* OP_MODE_XL [6:4] */
#define LSM6DSV80X_XL_OP_MODE_MASK       (0x07U << 4)
#define LSM6DSV80X_XL_HP_MODE            (0x00U << 4)
#define LSM6DSV80X_XL_HA_MODE            (0x01U << 4)
#define LSM6DSV80X_XL_ODR_TRIG           (0x03U << 4)
#define LSM6DSV80X_XL_LP1_MODE           (0x04U << 4)
#define LSM6DSV80X_XL_LP2_MODE           (0x05U << 4)
#define LSM6DSV80X_XL_LP3_MODE           (0x06U << 4)
#define LSM6DSV80X_XL_NORMAL_MODE        (0x07U << 4)

/* ODR_XL [3:0] */
#define LSM6DSV80X_XL_ODR_MASK           (0x0FU << 0)
#define LSM6DSV80X_XL_ODR_OFF            (0x00U << 0)
#define LSM6DSV80X_XL_ODR_1HZ875         (0x01U << 0)
#define LSM6DSV80X_XL_ODR_7HZ5           (0x02U << 0)
#define LSM6DSV80X_XL_ODR_15HZ           (0x03U << 0)
#define LSM6DSV80X_XL_ODR_30HZ           (0x04U << 0)
#define LSM6DSV80X_XL_ODR_60HZ           (0x05U << 0)
#define LSM6DSV80X_XL_ODR_120HZ          (0x06U << 0)
#define LSM6DSV80X_XL_ODR_240HZ          (0x07U << 0)
#define LSM6DSV80X_XL_ODR_480HZ          (0x08U << 0)
#define LSM6DSV80X_XL_ODR_960HZ          (0x09U << 0)
#define LSM6DSV80X_XL_ODR_1KHZ92         (0x0AU << 0)
#define LSM6DSV80X_XL_ODR_3KHZ84         (0x0BU << 0)
#define LSM6DSV80X_XL_ODR_7KHZ68         (0x0CU << 0)

/* Rounded aliases retained for consistency with lsm6dsv16x_reg.h. */
#define LSM6DSV80X_XL_ODR_2KHZ           LSM6DSV80X_XL_ODR_1KHZ92
#define LSM6DSV80X_XL_ODR_4KHZ           LSM6DSV80X_XL_ODR_3KHZ84
#define LSM6DSV80X_XL_ODR_8KHZ           LSM6DSV80X_XL_ODR_7KHZ68

/* ==============================
 * CTRL2: gyroscope
 * ============================== */
/* OP_MODE_G [6:4] */
#define LSM6DSV80X_G_OP_MODE_MASK        (0x07U << 4)
#define LSM6DSV80X_G_HP_MODE             (0x00U << 4)
#define LSM6DSV80X_G_HA_MODE             (0x01U << 4)
#define LSM6DSV80X_G_ODR_TRIG            (0x03U << 4)
#define LSM6DSV80X_G_SLEEP_MODE          (0x04U << 4)
#define LSM6DSV80X_G_LP_MODE             (0x05U << 4)

/* ODR_G [3:0] */
#define LSM6DSV80X_G_ODR_MASK            (0x0FU << 0)
#define LSM6DSV80X_G_ODR_OFF             (0x00U << 0)
#define LSM6DSV80X_G_ODR_7HZ5            (0x02U << 0)
#define LSM6DSV80X_G_ODR_15HZ            (0x03U << 0)
#define LSM6DSV80X_G_ODR_30HZ            (0x04U << 0)
#define LSM6DSV80X_G_ODR_60HZ            (0x05U << 0)
#define LSM6DSV80X_G_ODR_120HZ           (0x06U << 0)
#define LSM6DSV80X_G_ODR_240HZ           (0x07U << 0)
#define LSM6DSV80X_G_ODR_480HZ           (0x08U << 0)
#define LSM6DSV80X_G_ODR_960HZ           (0x09U << 0)
#define LSM6DSV80X_G_ODR_1KHZ92          (0x0AU << 0)
#define LSM6DSV80X_G_ODR_3KHZ84          (0x0BU << 0)
#define LSM6DSV80X_G_ODR_7KHZ68          (0x0CU << 0)

/* Rounded aliases retained for consistency with lsm6dsv16x_reg.h. */
#define LSM6DSV80X_G_ODR_2KHZ            LSM6DSV80X_G_ODR_1KHZ92
#define LSM6DSV80X_G_ODR_4KHZ            LSM6DSV80X_G_ODR_3KHZ84
#define LSM6DSV80X_G_ODR_8KHZ            LSM6DSV80X_G_ODR_7KHZ68

/* ==============================
 * CTRL3
 * ============================== */
#define LSM6DSV80X_BOOT_NORMAL           (0x00U << 7)
#define LSM6DSV80X_BOOT_REBOOT           (0x01U << 7)
#define LSM6DSV80X_BDU_DIS               (0x00U << 6)
#define LSM6DSV80X_BDU_EN                (0x01U << 6)
#define LSM6DSV80X_IF_INC_DIS            (0x00U << 2)
#define LSM6DSV80X_IF_INC_EN             (0x01U << 2)
#define LSM6DSV80X_SW_RESET              (0x01U << 0)

/* ==============================
 * CTRL6: gyroscope LPF1 and full scale
 * ============================== */
/* LPF1_G_BW [6:4] */
#define LSM6DSV80X_G_LPF1_BW_MASK        (0x07U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_0        (0x00U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_1        (0x01U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_2        (0x02U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_3        (0x03U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_4        (0x04U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_5        (0x05U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_6        (0x06U << 4)
#define LSM6DSV80X_G_LPF1_FTYPE_7        (0x07U << 4)

/* FS_G [2:0]; bit 3 must always be set to 1. */
#define LSM6DSV80X_G_FS_MASK             ((0x01U << 3) | (0x07U << 0))
#define LSM6DSV80X_G_FS_250DPS           ((0x01U << 3) | (0x01U << 0))
#define LSM6DSV80X_G_FS_500DPS           ((0x01U << 3) | (0x02U << 0))
#define LSM6DSV80X_G_FS_1000DPS          ((0x01U << 3) | (0x03U << 0))
#define LSM6DSV80X_G_FS_2000DPS          ((0x01U << 3) | (0x04U << 0))
#define LSM6DSV80X_G_FS_4000DPS          ((0x01U << 3) | (0x05U << 0))

/* Compatibility aliases used by the existing IMU initialization code. */
#define LSM6DSV80X_GY_LPF1_FTYPE_0       LSM6DSV80X_G_LPF1_FTYPE_0
#define LSM6DSV80X_GY_LPF1_FTYPE_1       LSM6DSV80X_G_LPF1_FTYPE_1
#define LSM6DSV80X_GY_LPF1_FTYPE_2       LSM6DSV80X_G_LPF1_FTYPE_2
#define LSM6DSV80X_GY_LPF1_FTYPE_3       LSM6DSV80X_G_LPF1_FTYPE_3
#define LSM6DSV80X_GY_LPF1_FTYPE_4       LSM6DSV80X_G_LPF1_FTYPE_4
#define LSM6DSV80X_GY_LPF1_FTYPE_5       LSM6DSV80X_G_LPF1_FTYPE_5
#define LSM6DSV80X_GY_LPF1_FTYPE_6       LSM6DSV80X_G_LPF1_FTYPE_6
#define LSM6DSV80X_GY_LPF1_FTYPE_7       LSM6DSV80X_G_LPF1_FTYPE_7
#define LSM6DSV80X_GY_FS_250DPS          LSM6DSV80X_G_FS_250DPS
#define LSM6DSV80X_GY_FS_500DPS          LSM6DSV80X_G_FS_500DPS
#define LSM6DSV80X_GY_FS_1000DPS         LSM6DSV80X_G_FS_1000DPS
#define LSM6DSV80X_GY_FS_2000DPS         LSM6DSV80X_G_FS_2000DPS
#define LSM6DSV80X_GY_FS_4000DPS         LSM6DSV80X_G_FS_4000DPS

/* ==============================
 * CTRL7
 * ============================== */
#define LSM6DSV80X_INT1_DRDY_XL_HG_DIS   (0x00U << 7)
#define LSM6DSV80X_INT1_DRDY_XL_HG_EN    (0x01U << 7)
#define LSM6DSV80X_INT2_DRDY_XL_HG_DIS   (0x00U << 6)
#define LSM6DSV80X_INT2_DRDY_XL_HG_EN    (0x01U << 6)
#define LSM6DSV80X_G_LPF1_DISABLE        (0x00U << 0)
#define LSM6DSV80X_G_LPF1_ENABLE         (0x01U << 0)

/* ==============================
 * CTRL8: low-g accelerometer filter and full scale
 * ============================== */
/* HP_LPF2_XL_BW [7:5] */
#define LSM6DSV80X_XL_FILTER_BW_MASK     (0x07U << 5)
#define LSM6DSV80X_XL_CF_ODR_4           (0x00U << 5)
#define LSM6DSV80X_XL_CF_ODR_10          (0x01U << 5)
#define LSM6DSV80X_XL_CF_ODR_20          (0x02U << 5)
#define LSM6DSV80X_XL_CF_ODR_45          (0x03U << 5)
#define LSM6DSV80X_XL_CF_ODR_100         (0x04U << 5)
#define LSM6DSV80X_XL_CF_ODR_200         (0x05U << 5)
#define LSM6DSV80X_XL_CF_ODR_400         (0x06U << 5)
#define LSM6DSV80X_XL_CF_ODR_800         (0x07U << 5)

/* FS_XL [1:0] */
#define LSM6DSV80X_XL_FS_MASK            (0x03U << 0)
#define LSM6DSV80X_XL_FS_2G              (0x00U << 0)
#define LSM6DSV80X_XL_FS_4G              (0x01U << 0)
#define LSM6DSV80X_XL_FS_8G              (0x02U << 0)
#define LSM6DSV80X_XL_FS_16G             (0x03U << 0)

/* ==============================
 * CTRL9: low-g accelerometer filter path
 * ============================== */
#define LSM6DSV80X_XL_HP_REF_MODE_DIS    (0x00U << 6)
#define LSM6DSV80X_XL_HP_REF_MODE_EN     (0x01U << 6)
#define LSM6DSV80X_XL_FASTSETTL_MODE_DIS (0x00U << 5)
#define LSM6DSV80X_XL_FASTSETTL_MODE_EN  (0x01U << 5)
#define LSM6DSV80X_XL_HP_SLOPE_DIS       (0x00U << 4)
#define LSM6DSV80X_XL_HP_SLOPE_EN        (0x01U << 4)
#define LSM6DSV80X_XL_LPF2_DIS           (0x00U << 3)
#define LSM6DSV80X_XL_LPF2_EN            (0x01U << 3)
#define LSM6DSV80X_XL_USR_OFF_W_2POW10   (0x00U << 1)
#define LSM6DSV80X_XL_USR_OFF_W_2POW6    (0x01U << 1)
#define LSM6DSV80X_XL_USR_OFF_OUT_DIS    (0x00U << 0)
#define LSM6DSV80X_XL_USR_OFF_OUT_EN     (0x01U << 0)

/* Compatibility aliases used by lsm6dsv16x_reg.h. */
#define LSM6DSV80X_XL_HP_REF_MODE_XL_DIS LSM6DSV80X_XL_HP_REF_MODE_DIS
#define LSM6DSV80X_XL_HP_REF_MODE_XL_EN  LSM6DSV80X_XL_HP_REF_MODE_EN
#define LSM6DSV80X_XL_LPF2_XL_DIS        LSM6DSV80X_XL_LPF2_DIS
#define LSM6DSV80X_XL_LPF2_XL_EN         LSM6DSV80X_XL_LPF2_EN

/* ==============================
 * CTRL1_XL_HG: high-g accelerometer
 * ============================== */
#define LSM6DSV80X_XL_HG_REGOUT_DIS      (0x00U << 7)
#define LSM6DSV80X_XL_HG_REGOUT_EN       (0x01U << 7)
#define LSM6DSV80X_XL_HG_USR_OFF_DIS     (0x00U << 6)
#define LSM6DSV80X_XL_HG_USR_OFF_EN      (0x01U << 6)

/* ODR_XL_HG [5:3] */
#define LSM6DSV80X_XL_HG_ODR_MASK        (0x07U << 3)
#define LSM6DSV80X_XL_HG_ODR_OFF         (0x00U << 3)
#define LSM6DSV80X_XL_HG_ODR_480HZ       (0x03U << 3)
#define LSM6DSV80X_XL_HG_ODR_960HZ       (0x04U << 3)
#define LSM6DSV80X_XL_HG_ODR_1KHZ92      (0x05U << 3)
#define LSM6DSV80X_XL_HG_ODR_3KHZ84      (0x06U << 3)
#define LSM6DSV80X_XL_HG_ODR_7KHZ68      (0x07U << 3)

/* FS_XL_HG [2:0] */
#define LSM6DSV80X_XL_HG_FS_MASK         (0x07U << 0)
#define LSM6DSV80X_XL_HG_FS_32G          (0x00U << 0)
#define LSM6DSV80X_XL_HG_FS_64G          (0x01U << 0)
#define LSM6DSV80X_XL_HG_FS_80G          (0x02U << 0)

/* ==============================
 * HAODR_CFG
 * ============================== */
#define LSM6DSV80X_HAODR_SEL_MASK        (0x03U << 0)
#define LSM6DSV80X_HAODR_SEL_0           (0x00U << 0)
#define LSM6DSV80X_HAODR_SEL_1           (0x01U << 0)
#define LSM6DSV80X_HAODR_SEL_2           (0x02U << 0)
#define LSM6DSV80X_HAODR_SEL_3           (0x03U << 0)

#define LSM6DSV80X_HAODR_HAODR_SEL0      LSM6DSV80X_HAODR_SEL_0
#define LSM6DSV80X_HAODR_HAODR_SEL1      LSM6DSV80X_HAODR_SEL_1
#define LSM6DSV80X_HAODR_HAODR_SEL2      LSM6DSV80X_HAODR_SEL_2
#define LSM6DSV80X_HAODR_HAODR_SEL3      LSM6DSV80X_HAODR_SEL_3

#ifdef __cplusplus
}
#endif

#endif /* INC_LSM6DSV80X_REG_H_ */
