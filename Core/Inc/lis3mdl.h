/*
 * lis3mdl.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Vatsal
 *
 * 	Macros generated via AI
 */

#ifndef SRC_LIS3MDL_H_
#define SRC_LIS3MDL_H_

#define LIS3MDL_MULTI 		 0x1 << 7


/* LIS3MDL Output Registers */
#define LIS3MDL_OUT_X_L      0x28    // X-axis low byte
#define LIS3MDL_OUT_X_H      0x29    // X-axis high byte
#define LIS3MDL_OUT_Y_L      0x2A    // Y-axis low byte
#define LIS3MDL_OUT_Y_H      0x2B    // Y-axis high byte
#define LIS3MDL_OUT_Z_L      0x2C    // Z-axis low byte
#define LIS3MDL_OUT_Z_H      0x2D    // Z-axis high byte

/* Commonly used control registers for reference */
#define LIS3MDL_WHO_AM_I     0x0F    // Device ID (0x3D)
#define LIS3MDL_CTRL_REG1    0x20    // Control register 1
#define LIS3MDL_CTRL_REG2    0x21    // Control register 2
#define LIS3MDL_CTRL_REG3    0x22    // Control register 3
#define LIS3MDL_CTRL_REG4    0x23    // Control register 4
#define LIS3MDL_CTRL_REG5    0x24    // Control register 5
#define LIS3MDL_STATUS_REG   0x27    // Status register

/* Bit masks for status register */
#define LIS3MDL_STATUS_ZYXOR 0x80    // X, Y, Z axis data overrun
#define LIS3MDL_STATUS_ZOR   0x40    // Z axis data overrun
#define LIS3MDL_STATUS_YOR   0x20    // Y axis data overrun
#define LIS3MDL_STATUS_XOR   0x10    // X axis data overrun
#define LIS3MDL_STATUS_ZYXDA 0x08    // X, Y, Z axis data available
#define LIS3MDL_STATUS_ZDA   0x04    // Z axis data available
#define LIS3MDL_STATUS_YDA   0x02    // Y axis data available
#define LIS3MDL_STATUS_XDA   0x01    // X axis data available


// Register Addresses
#define LIS3MDL_CTRL_REG1       0x20  // Control register 1 (general config)
#define LIS3MDL_CTRL_REG2       0x21  // Control register 2 (full-scale config)
#define LIS3MDL_CTRL_REG3       0x22  // Control register 3 (operating mode)
#define LIS3MDL_CTRL_REG4       0x23  // Control register 4 (Z-axis mode)
#define LIS3MDL_CTRL_REG5       0x24  // Control register 5 (data update config)

// CTRL_REG1 (0x20) - Control Register 1
// TEMP_EN - Temperature Sensor Enable (Bit 7)
#define LIS3MDL_TEMP_DISABLE    (0x00 << 7)  // Temperature sensor disabled
#define LIS3MDL_TEMP_ENABLE     (0x01 << 7)  // Temperature sensor enabled

// OM[1:0] - X/Y Axes Operating Mode (Bits 6:5)
#define LIS3MDL_OM_LOW_POWER    (0x00 << 5)  // Low-power mode
#define LIS3MDL_OM_MEDIUM       (0x01 << 5)  // Medium-performance mode
#define LIS3MDL_OM_HIGH         (0x02 << 5)  // High-performance mode
#define LIS3MDL_OM_ULTRA_HIGH   (0x03 << 5)  // Ultra-high-performance mode

// DO[2:0] - Output Data Rate (Bits 4:2)
#define LIS3MDL_DO_0_625HZ      (0x00 << 2)  // 0.625 Hz
#define LIS3MDL_DO_1_25HZ       (0x01 << 2)  // 1.25 Hz
#define LIS3MDL_DO_2_5HZ        (0x02 << 2)  // 2.5 Hz
#define LIS3MDL_DO_5HZ          (0x03 << 2)  // 5 Hz
#define LIS3MDL_DO_10HZ         (0x04 << 2)  // 10 Hz
#define LIS3MDL_DO_20HZ         (0x05 << 2)  // 20 Hz
#define LIS3MDL_DO_40HZ         (0x06 << 2)  // 40 Hz
#define LIS3MDL_DO_80HZ         (0x07 << 2)  // 80 Hz

// ST - Self-Test Enable (Bit 0)
#define LIS3MDL_ST_DISABLE      (0x00 << 0)  // Self-test disabled
#define LIS3MDL_ST_ENABLE       (0x01 << 0)  // Self-test enabled

// CTRL_REG2 (0x21) - Control Register 2
// FS[1:0] - Full-Scale Selection (Bits 6:5)
#define LIS3MDL_FS_4GAUSS       (0x00 << 5)  // ±4 gauss
#define LIS3MDL_FS_8GAUSS       (0x01 << 5)  // ±8 gauss
#define LIS3MDL_FS_12GAUSS      (0x02 << 5)  // ±12 gauss
#define LIS3MDL_FS_16GAUSS      (0x03 << 5)  // ±16 gauss

// REBOOT - Reboot Memory Content (Bit 3)
#define LIS3MDL_REBOOT_NORMAL   (0x00 << 3)  // Normal mode
#define LIS3MDL_REBOOT_TRIGGER  (0x01 << 3)  // Reboot memory content

// SOFT_RST - Software Reset (Bit 2)
#define LIS3MDL_SOFT_RST_NORMAL (0x00 << 2)  // Normal operation
#define LIS3MDL_SOFT_RST_RESET  (0x01 << 2)  // Reset configuration registers

// CTRL_REG3 (0x22) - Control Register 3
// I2C_DISABLE - I2C Interface Disable (Bit 7)
#define LIS3MDL_I2C_ENABLE      (0x00 << 7)  // I2C enabled (SPI only if disabled)
#define LIS3MDL_I2C_DISABLE     (0x01 << 7)  // I2C disabled (SPI only)

// LP - Low-Power Mode Enable (Bit 5)
#define LIS3MDL_LP_DISABLE      (0x00 << 5)  // Low-power mode disabled
#define LIS3MDL_LP_ENABLE       (0x01 << 5)  // Low-power mode enabled

// SIM - SPI Mode Selection (Bit 2)
#define LIS3MDL_SIM_4WIRE       (0x00 << 2)  // 4-wire SPI
#define LIS3MDL_SIM_3WIRE       (0x01 << 2)  // 3-wire SPI

// MD[1:0] - Operating Mode (Bits 1:0)
#define LIS3MDL_MD_CONTINUOUS   (0x00 << 0)  // Continuous-conversion mode
#define LIS3MDL_MD_SINGLE       (0x01 << 0)  // Single-conversion mode
#define LIS3MDL_MD_POWER_DOWN   (0x02 << 0)  // Power-down mode (default)

// CTRL_REG4 (0x23) - Control Register 4
// OMZ[1:0] - Z-Axis Operating Mode (Bits 3:2)
#define LIS3MDL_OMZ_LOW_POWER   (0x00 << 2)  // Low-power mode
#define LIS3MDL_OMZ_MEDIUM      (0x01 << 2)  // Medium-performance mode
#define LIS3MDL_OMZ_HIGH        (0x02 << 2)  // High-performance mode
#define LIS3MDL_OMZ_ULTRA_HIGH  (0x03 << 2)  // Ultra-high-performance mode

// CTRL_REG5 (0x24) - Control Register 5
// BDU - Block Data Update (Bit 6)
#define LIS3MDL_BDU_CONTINUOUS  (0x00 << 6)  // Continuous update
#define LIS3MDL_BDU_BLOCK       (0x01 << 6)  // Block update until MSB/LSB read

#endif /* SRC_LIS3MDL_H_ */
