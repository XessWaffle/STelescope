/*
 * lsm6dsox.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Vatsal
 *
 *  Macros generated via AI
 */

#ifndef SRC_LSM6DSOX_H_
#define SRC_LSM6DSOX_H_

#define LSM6DSOX_WHO_AM_I 		   0x0F

/* Temperature sensor output registers (16-bit, two's complement) */
#define LSM6DSOX_OUT_TEMP_L        0x20  /* Low byte of temperature data */
#define LSM6DSOX_OUT_TEMP_H        0x21  /* High byte of temperature data */

/* Gyroscope output registers (16-bit per axis, two's complement) */
#define LSM6DSOX_OUTX_L_G          0x22  /* Gyro X-axis low byte */
#define LSM6DSOX_OUTX_H_G          0x23  /* Gyro X-axis high byte */
#define LSM6DSOX_OUTY_L_G          0x24  /* Gyro Y-axis low byte */
#define LSM6DSOX_OUTY_H_G          0x25  /* Gyro Y-axis high byte */
#define LSM6DSOX_OUTZ_L_G          0x26  /* Gyro Z-axis low byte */
#define LSM6DSOX_OUTZ_H_G          0x27  /* Gyro Z-axis high byte */

/* Accelerometer output registers (16-bit per axis, two's complement) */
#define LSM6DSOX_OUTX_L_A          0x28  /* Accel X-axis low byte */
#define LSM6DSOX_OUTX_H_A          0x29  /* Accel X-axis high byte */
#define LSM6DSOX_OUTY_L_A          0x2A  /* Accel Y-axis low byte */
#define LSM6DSOX_OUTY_H_A          0x2B  /* Accel Y-axis high byte */
#define LSM6DSOX_OUTZ_L_A          0x2C  /* Accel Z-axis low byte */
#define LSM6DSOX_OUTZ_H_A          0x2D  /* Accel Z-axis high byte */

/* FIFO data output register */
#define LSM6DSOX_FIFO_DATA_OUT_L   0x3E  /* Low byte of FIFO data output */
#define LSM6DSOX_FIFO_DATA_OUT_H   0x3F  /* High byte of FIFO data output */

/* Additional output-related registers */
#define LSM6DSOX_TIMESTAMP0        0x40  /* Timestamp register 0 (LSB) */
#define LSM6DSOX_TIMESTAMP1        0x41  /* Timestamp register 1 */
#define LSM6DSOX_TIMESTAMP2        0x42  /* Timestamp register 2 */
#define LSM6DSOX_TIMESTAMP3        0x43  /* Timestamp register 3 (MSB) */

/* Machine Learning Core (MLC) output registers */
#define LSM6DSOX_MLC1_SRC          0x70  /* MLC1 source register */
#define LSM6DSOX_MLC2_SRC          0x71  /* MLC2 source register */
#define LSM6DSOX_MLC3_SRC          0x72  /* MLC3 source register */
#define LSM6DSOX_MLC4_SRC          0x73  /* MLC4 source register */

/* Step counter output registers (16-bit) */
#define LSM6DSOX_STEP_COUNTER_L    0x4B  /* Step counter low byte */
#define LSM6DSOX_STEP_COUNTER_H    0x4C  /* Step counter high byte */


/* LSM6DSOX Control Register Addresses */
#define LSM6DSOX_CTRL1_XL       0x10  // Accelerometer control register 1
#define LSM6DSOX_CTRL2_G        0x11  // Gyroscope control register 2
#define LSM6DSOX_CTRL3_C        0x12  // Control register 3 (general settings)
#define LSM6DSOX_CTRL4_C        0x13  // Control register 4 (additional settings)
#define LSM6DSOX_CTRL5_C        0x14  // Control register 5 (self-test & rounding)
#define LSM6DSOX_CTRL6_C        0x15  // Control register 6 (gyro settings)
#define LSM6DSOX_CTRL7_G        0x16  // Control register 7 (gyro additional settings)
#define LSM6DSOX_CTRL8_XL       0x17  // Control register 8 (accel filtering)
#define LSM6DSOX_CTRL9_XL       0x18  // Control register 9 (accel axis enable)
#define LSM6DSOX_CTRL10_C       0x19  // Control register 10 (embedded functions)

/* CTRL1_XL (0x10) - Accelerometer Control Register 1 */
/* ODR_XL[3:0] - Output Data Rate (bits 7-4) */
#define LSM6DSOX_XL_ODR_OFF     (0x0 << 4)  // Power-down
#define LSM6DSOX_XL_ODR_1_6HZ   (0xB << 4)  // 1.6 Hz (low power)
#define LSM6DSOX_XL_ODR_12_5HZ  (0x1 << 4)  // 12.5 Hz
#define LSM6DSOX_XL_ODR_26HZ    (0x2 << 4)  // 26 Hz
#define LSM6DSOX_XL_ODR_52HZ    (0x3 << 4)  // 52 Hz
#define LSM6DSOX_XL_ODR_104HZ   (0x4 << 4)  // 104 Hz
#define LSM6DSOX_XL_ODR_208HZ   (0x5 << 4)  // 208 Hz
#define LSM6DSOX_XL_ODR_416HZ   (0x6 << 4)  // 416 Hz
#define LSM6DSOX_XL_ODR_833HZ   (0x7 << 4)  // 833 Hz
#define LSM6DSOX_XL_ODR_1666HZ  (0x8 << 4)  // 1.66 kHz
#define LSM6DSOX_XL_ODR_3332HZ  (0x9 << 4)  // 3.33 kHz
#define LSM6DSOX_XL_ODR_6664HZ  (0xA << 4)  // 6.66 kHz
/* FS_XL[1:0] - Full Scale Selection (bits 3-2) */
#define LSM6DSOX_XL_FS_2G       (0x0 << 2)  // ±2g
#define LSM6DSOX_XL_FS_16G      (0x1 << 2)  // ±16g
#define LSM6DSOX_XL_FS_4G       (0x2 << 2)  // ±4g
#define LSM6DSOX_XL_FS_8G       (0x3 << 2)  // ±8g
/* LPF1_BW_SEL - Low-pass filter bandwidth (bit 1) */
#define LSM6DSOX_XL_LPF1_BW_SEL (1 << 1)    // LPF1 bandwidth selection

/* CTRL2_G (0x11) - Gyroscope Control Register 2 */
/* ODR_G[3:0] - Output Data Rate (bits 7-4) */
#define LSM6DSOX_G_ODR_OFF      (0x0 << 4)  // Power-down
#define LSM6DSOX_G_ODR_12_5HZ   (0x1 << 4)  // 12.5 Hz
#define LSM6DSOX_G_ODR_26HZ     (0x2 << 4)  // 26 Hz
#define LSM6DSOX_G_ODR_52HZ     (0x3 << 4)  // 52 Hz
#define LSM6DSOX_G_ODR_104HZ    (0x4 << 4)  // 104 Hz
#define LSM6DSOX_G_ODR_208HZ    (0x5 << 4)  // 208 Hz
#define LSM6DSOX_G_ODR_416HZ    (0x6 << 4)  // 416 Hz
#define LSM6DSOX_G_ODR_833HZ    (0x7 << 4)  // 833 Hz
#define LSM6DSOX_G_ODR_1666HZ   (0x8 << 4)  // 1.66 kHz
#define LSM6DSOX_G_ODR_3332HZ   (0x9 << 4)  // 3.33 kHz
#define LSM6DSOX_G_ODR_6664HZ   (0xA << 4)  // 6.66 kHz
/* FS_G[1:0] - Full Scale Selection (bits 3-2) */
#define LSM6DSOX_G_FS_250DPS    (0x0 << 2)  // ±250 dps
#define LSM6DSOX_G_FS_500DPS    (0x1 << 2)  // ±500 dps
#define LSM6DSOX_G_FS_1000DPS   (0x2 << 2)  // ±1000 dps
#define LSM6DSOX_G_FS_2000DPS   (0x3 << 2)  // ±2000 dps
/* FS_125 - ±125 dps selection (bit 1) */
#define LSM6DSOX_G_FS_125       (1 << 1)    // ±125 dps

/* CTRL3_C (0x12) - Control Register 3 */
#define LSM6DSOX_BOOT           (1 << 7)    // Reboot memory content
#define LSM6DSOX_BDU            (1 << 6)    // Block Data Update
#define LSM6DSOX_H_LACTIVE      (1 << 5)    // High/low interrupt active level
#define LSM6DSOX_PP_OD          (1 << 4)    // Push-pull/open-drain selection
#define LSM6DSOX_SIM            (1 << 3)    // SPI mode selection
#define LSM6DSOX_IF_INC         (1 << 2)    // Register address auto-increment
#define LSM6DSOX_BLE            (1 << 1)    // Big/little endian selection
#define LSM6DSOX_SW_RESET       (1 << 0)    // Software reset

/* CTRL4_C (0x13) - Control Register 4 */
#define LSM6DSOX_LPF1_SEL_G     (1 << 4)    // Gyro LPF1 selection
#define LSM6DSOX_I2C_DISABLE    (1 << 2)    // I2C interface disable
#define LSM6DSOX_DRDY_MASK      (1 << 0)    // Data-ready on INT1

/* CTRL5_C (0x14) - Control Register 5 */
/* ROUNDING[1:0] - Rounding function (bits 6-5) */
#define LSM6DSOX_ROUNDING_NONE  (0x0 << 5)  // No rounding
#define LSM6DSOX_ROUNDING_XL    (0x1 << 5)  // Accel rounding
#define LSM6DSOX_ROUNDING_G     (0x2 << 5)  // Gyro rounding
#define LSM6DSOX_ROUNDING_BOTH  (0x3 << 5)  // Accel + Gyro rounding
/* ST_G[1:0] - Gyro self-test (bits 3-2) */
#define LSM6DSOX_ST_G_OFF       (0x0 << 2)  // Gyro self-test disabled
#define LSM6DSOX_ST_G_POS       (0x1 << 2)  // Positive sign self-test
#define LSM6DSOX_ST_G_NEG       (0x2 << 2)  // Negative sign self-test
/* ST_XL[1:0] - Accel self-test (bits 1-0) */
#define LSM6DSOX_ST_XL_OFF      (0x0 << 0)  // Accel self-test disabled
#define LSM6DSOX_ST_XL_POS      (0x1 << 0)  // Positive sign self-test
#define LSM6DSOX_ST_XL_NEG      (0x2 << 0)  // Negative sign self-test

/* CTRL6_C (0x15) - Control Register 6 (Gyro) */
#define LSM6DSOX_TRIG_EN        (1 << 7)    // Trigger enable
#define LSM6DSOX_LVL_EN         (1 << 6)    // Level-sensitive latch enable
#define LSM6DSOX_LVL2_EN        (1 << 5)    // Level-sensitive latch 2 enable
#define LSM6DSOX_XL_HM_MODE     (1 << 4)    // Accel high-performance mode disable
#define LSM6DSOX_USR_OFF_W      (1 << 3)    // User offset weight
/* FTYPE[1:0] - Gyro LPF1 filter type (bits 1-0) */
#define LSM6DSOX_FTYPE_0        (0x0 << 0)  // Filter type 0
#define LSM6DSOX_FTYPE_1        (0x1 << 0)  // Filter type 1
#define LSM6DSOX_FTYPE_2        (0x2 << 0)  // Filter type 2
#define LSM6DSOX_FTYPE_3        (0x3 << 0)  // Filter type 3

/* CTRL7_G (0x16) - Control Register 7 (Gyro) */
#define LSM6DSOX_G_HM_MODE      (1 << 7)    // Gyro high-performance mode disable
#define LSM6DSOX_HP_EN_G        (1 << 6)    // Gyro high-pass filter enable
/* HPM_G[1:0] - HP filter cutoff (bits 5-4) */
#define LSM6DSOX_HPM_G_16MHZ    (0x0 << 4)  // 16 mHz
#define LSM6DSOX_HPM_G_65MHZ    (0x1 << 4)  // 65 mHz
#define LSM6DSOX_HPM_G_260MHZ   (0x2 << 4)  // 260 mHz
#define LSM6DSOX_HPM_G_1_04HZ   (0x3 << 4)  // 1.04 Hz
#define LSM6DSOX_HP_G_RST       (1 << 3)    // Gyro high-pass filter reset

/* CTRL8_XL (0x17) - Control Register 8 (Accel Filtering) */
#define LSM6DSOX_HPM_XL_EN     (1 << 7)    // Accel HPM enable
/* HP_XL[1:0] - HP filter cutoff (bits 5-4) */
#define LSM6DSOX_HPM_XL_16MHZ   (0x0 << 4)  // 16 mHz
#define LSM6DSOX_HPM_XL_65MHZ   (0x1 << 4)  // 65 mHz
#define LSM6DSOX_HPM_XL_260MHZ  (0x2 << 4)  // 260 mHz
#define LSM6DSOX_HPM_XL_1_04HZ  (0x3 << 4)  // 1.04 Hz
#define LSM6DSOX_HP_SLOPE_XL_EN (1 << 2)    // Accel high-pass/slope filter enable
#define LSM6DSOX_HP_REF_MODE_XL (1 << 1)    // HP reference mode
#define LSM6DSOX_FASTSETTL_MODE_XL (1 << 0) // Fast settling mode

/* CTRL9_XL (0x18) - Control Register 9 (Accel Axis Enable) */
#define LSM6DSOX_DEN_XL_EN      (1 << 7)    // DEN function enable
#define LSM6DSOX_DEN_X          (1 << 6)    // DEN X-axis
#define LSM6DSOX_DEN_Y          (1 << 5)    // DEN Y-axis
#define LSM6DSOX_DEN_Z          (1 << 4)    // DEN Z-axis
#define LSM6DSOX_Z_EN           (1 << 5)    // Z-axis enable
#define LSM6DSOX_Y_EN           (1 << 4)    // Y-axis enable
#define LSM6DSOX_X_EN           (1 << 3)    // X-axis enable

/* CTRL10_C (0x19) - Control Register 10 (Embedded Functions) */
#define LSM6DSOX_TIMESTAMP_EN   (1 << 5)    // Timestamp enable
#define LSM6DSOX_EMB_FUNC_EN    (1 << 3)    // Embedded functions enable


#endif /* SRC_LSM6DSOX_H_ */
