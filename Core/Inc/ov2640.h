/*
 * ov2640.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 * 
 *  Macros generated with AI
 */

#ifndef INC_OV2640_H_
#define INC_OV2640_H_

// Default SCCB (I2C) Slave Address for OV2640
#define OV2640_I2C_ADDR            0x30  // 7-bit address, write mode (shifts to 0x60 in some implementations)

// Common OV2640 SCCB Register Addresses (I2C Commands)
#define OV2640_REG_SENSOR_RESET    0xFF  // Bank select register
#define OV2640_REG_COM7            0x12  // Common control 7 (reset, resolution)
#define OV2640_REG_COM8            0x13  // Common control 8 (AGC, AEC, AWB)
#define OV2640_REG_COM10           0x15  // Common control 10 (polarity, etc.)
#define OV2640_REG_PIDH            0x0A  // Product ID high byte (read-only)
#define OV2640_REG_PIDL            0x0B  // Product ID low byte (read-only)
#define OV2640_REG_HSTART          0x17  // Horizontal start high bits
#define OV2640_REG_HSIZE           0x19  // Horizontal size
#define OV2640_REG_VSTART          0x1C  // Vertical start high bits
#define OV2640_REG_VSIZE           0x1E  // Vertical size
#define OV2640_REG_JPEG_CTRL       0xE0  // JPEG control register

// SCCB Command Values (I2C Configuration Macros)
#define OV2640_RESET_ALL           0x80  // Reset all registers (COM7)
#define OV2640_SET_QVGA            0x40  // Set QVGA resolution (COM7)
#define OV2640_SET_VGA             0x00  // Set VGA resolution (COM7)
#define OV2640_SET_RGB565          0x04  // Set RGB565 output (COM7)
#define OV2640_SET_JPEG            0x08  // Set JPEG output (COM7)
#define OV2640_ENABLE_AEC          0x04  // Enable Auto Exposure Control (COM8)
#define OV2640_ENABLE_AWB          0x02  // Enable Auto White Balance (COM8)
#define OV2640_ENABLE_AGC          0x01  // Enable Auto Gain Control (COM8)

// Arducam SPI Command Macros (for Arducam shield interfacing with OV2640)
#define ARDUCAM_SPI_WRITE_REG(a)   0x80 | a // Write to a register (generic SPI command)
#define ARDUCAM_SPI_READ_REG(a)    0x7F & a // Read from a register (generic SPI command)


#define ARDUCAM_SPI_FIFO_CLEAR     0x01  // Clear FIFO buffer
#define ARDUCAM_SPI_FIFO_START     0x02  // Start capture to FIFO
#define ARDUCAM_SPI_SINGLE_READ    0x3D  // Read FIFO data
#define ARDUCAM_SPI_BURST_READ     0x3C  // Burst read from FIFO
#define ARDUCAM_SPI_SET_FIFO_SIZE  0x07  // Set FIFO size

// Arducam Register Addresses (accessed via SPI)
#define ARDUCAM_REG_FIFO_CTRL      0x04  // FIFO control register
#define ARDUCAM_REG_CAPTURE_CTRL   0x01  // Capture control register
#define ARDUCAM_REG_STATUS         0x03  // Status register
#define ARDUCAM_REG_FIFO_SIZE1     0x42  // FIFO size byte 1
#define ARDUCAM_REG_FIFO_SIZE2     0x43  // FIFO size byte 2
#define ARDUCAM_REG_FIFO_SIZE3     0x44  // FIFO size byte 3
#define ARDUCHIP_TEST1 			   0x00 // Test register 1

#define ARDUCHIP_TRIG      		   0x41  //Trigger source
#define VSYNC_MASK         		   0x01
#define SHUTTER_MASK       		   0x02
#define CAP_DONE_MASK      		   0x08

#endif /* INC_OV2640_H_ */