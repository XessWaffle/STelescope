/*
 * compass.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_COMPASS_H_
#define INC_COMPASS_H_

#include "peripheral.h"
#include "lis3mdl.h"

#define COMPASS_BUFFER_LENGTH 8
#define COMPASS_DIMENSIONS 3

typedef enum
{
	OFFBOARD = 0,
	ONBOARD = 1,
	NUM_COMPASSES = 2
} compass_e;

typedef struct
{
	i2c_per_s comp_i2c;

	/*
	 * Peripheral data
	 */
	int16_t out_x_raw;
	int16_t out_y_raw;
	int16_t out_z_raw;
	
	int32_t out_x_cal;
	int32_t out_y_cal;
	int32_t out_z_cal;

	int16_t sampled_x;
	int16_t sampled_y;
	int16_t sampled_z;

} compass_s;


extern compass_s compass[NUM_COMPASSES];

void set_compass_read(uint8_t read, uint8_t tx_buffer_length, uint8_t rx_buffer_length, compass_e compass_type);

uint8_t init_compass();
void dacq_compass(compass_e compass_type);
void sample_compass(uint8_t samples, compass_e compass_type);
void drdy_onboard_compass_cb();
void drdy_offboard_compass_cb();

compass_s* get_compass_data(compass_e compass_type);

#endif /* INC_COMPASS_H_ */
