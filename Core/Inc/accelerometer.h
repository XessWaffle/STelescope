/*
 * accelerometer.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_ACCELEROMETER_H_
#define INC_ACCELEROMETER_H_

#include <peripheral.h>
#include <lsm6dsox.h>


#define ACCELEROMETER_BUFFER_LENGTH 12
#define ACCELEROMETER_WHO_AM_I 0x0F

/*
 * #define critical registers
 */

typedef struct
{
	i2c_per_s accel_i2c;

	/*
	 * Peripheral data
	 */
	int16_t out_x_g_raw;
	int16_t out_y_g_raw;
	int16_t out_z_g_raw;
	int16_t out_x_a_raw;
	int16_t out_y_a_raw;
	int16_t out_z_a_raw;

} accelerometer_s;

extern accelerometer_s accelerometer;

void set_accelerometer_read(uint8_t read, uint8_t tx_buffer_length, uint8_t rx_buffer_length);

uint8_t init_accelerometer();
void dacq_accelerometer();
void drdy_accelerometer_cb();

accelerometer_s* get_accelerometer_data();

#endif /* INC_ACCELEROMETER_H_ */
