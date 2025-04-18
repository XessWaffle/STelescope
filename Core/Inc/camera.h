/*
 * camera.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_CAMERA_H_
#define INC_CAMERA_H_

#include "peripheral.h"
#include "ov2640.h"

#define CAMERA_BUFFER_LENGTH 8

typedef enum
{
	CAPTURE_PREP,
	CAPTURE_IN_PROGRESS,
	CAPTURE_METADATA,
	CAPTURE_DATA,
	CAPTURE_END,
	CAPTURE_NUM_STEPS,
	CAPTURE_INVALID = CAPTURE_NUM_STEPS
} capture_step_e;

typedef struct
{
	spi_per_s cam_spi;
	i2c_per_s cam_i2c;

	/*
	 * Peripheral data
	 */

	uint8_t pid, vid;
	uint32_t fifo_length;
	uint16_t last_fifo_read_length;

} camera_s;

extern camera_s camera;

uint8_t init_camera();

uint8_t capture_and_pipe(uint8_t new_capture, uint8_t *buff, uint32_t buff_len);

uint8_t is_camera_spi_data_ready();

uint8_t *get_camera_rx_buffer();
uint16_t get_camera_read_fifo_length();
#endif /* INC_CAMERA_H_ */
