/*
 * compass.h
 *
 *  Created on: Mar 21, 2025
 *      Author: Vatsal
 */
#include "compass.h"

// Soft and hard iron calibration parameters
const int16_t soft_cal_num[COMPASS_DIMENSIONS][COMPASS_DIMENSIONS] = {{15175, -17615, 0}, {18142, 19281, 0}, {0, 0, 1}};
const int16_t soft_cal_den[COMPASS_DIMENSIONS][COMPASS_DIMENSIONS] = {{19652, 28186, 0}, {26077, 22430, 0}, {0, 0, 1}};
const int16_t hard_cal[COMPASS_DIMENSIONS] = {746, 673, -3661};

compass_s compass;

uint8_t compass_rx_buff[COMPASS_BUFFER_LENGTH];
uint8_t compass_tx_buff[COMPASS_BUFFER_LENGTH];

uint8_t populate_compass_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= COMPASS_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	compass_tx_buff[buff_pos++] = byte;
	return buff_pos;
}


uint8_t init_compass()
{
	/*
	 * Constants
	 */
	compass.comp_i2c.address = 0x1C;
	compass.comp_i2c.data.rx_buff = compass_rx_buff;
	compass.comp_i2c.data.tx_buff = compass_tx_buff;
	compass.comp_i2c.data.handle = I2C_1;
	compass.comp_i2c.i2c_cb = drdy_compass_cb;

	/*
	 * Validate that I2C communication is established
	 */
	uint8_t length = populate_compass_tx_buff(TRUE, LIS3MDL_WHO_AM_I);
	set_compass_read(TRUE, length, 1);
	while(rtx_i2c(&(compass.comp_i2c)) == HAL_BUSY);

	HAL_Delay(100);

	/*
	 * Expected response
	 */
	if(compass_rx_buff[0] != 0x3D)
		return FALSE;

	/*
	 * Transmit
	 */
	populate_compass_tx_buff(TRUE, LIS3MDL_CTRL_REG1 | LIS3MDL_MULTI);
	populate_compass_tx_buff(FALSE, LIS3MDL_TEMP_DISABLE | LIS3MDL_OM_HIGH | LIS3MDL_DO_20HZ | LIS3MDL_ST_DISABLE);
	populate_compass_tx_buff(FALSE, LIS3MDL_FS_4GAUSS | LIS3MDL_REBOOT_NORMAL | LIS3MDL_SOFT_RST_NORMAL);
	populate_compass_tx_buff(FALSE, LIS3MDL_I2C_ENABLE | LIS3MDL_LP_DISABLE | LIS3MDL_SIM_4WIRE | LIS3MDL_MD_CONTINUOUS);
	populate_compass_tx_buff(FALSE, LIS3MDL_OMZ_HIGH);
	length = populate_compass_tx_buff(FALSE, LIS3MDL_BDU_CONTINUOUS);
	set_compass_read(FALSE, length, 0);
	while(rtx_i2c(&(compass.comp_i2c)) == HAL_BUSY);

	return TRUE;
}


inline void dacq_compass()
{
	uint8_t length = populate_compass_tx_buff(TRUE, LIS3MDL_OUT_X_L | LIS3MDL_MULTI);
	set_compass_read(TRUE, length, 6);
	while(rtx_i2c(&(compass.comp_i2c)) == HAL_BUSY);
}

inline void sample_compass(uint8_t samples)
{
	compass.sampled_x = 0;
	compass.sampled_y = 0;
	compass.sampled_z = 0;

	for(uint8_t i = 0; i < samples; i++)
		dacq_compass();

	HAL_Delay(100); // Wait for data to be ready

	compass.sampled_x /= samples;
	compass.sampled_y /= samples;
	compass.sampled_z /= samples;
}

inline void set_compass_read(uint8_t read, uint8_t tx_buffer_length, uint8_t rx_buffer_length)
{
	compass.comp_i2c.data.read = read;
	compass.comp_i2c.data.tx_buff_length = tx_buffer_length;
	compass.comp_i2c.data.rx_buff_length = rx_buffer_length;
}

inline void drdy_compass_cb()
{
	compass.out_x_raw = compass_rx_buff[0] | (compass_rx_buff[1] << 8);
	compass.out_y_raw = compass_rx_buff[2] | (compass_rx_buff[3] << 8);
	compass.out_z_raw = compass_rx_buff[4] | (compass_rx_buff[5] << 8);

	int16_t out_x_hard = compass.out_x_raw - hard_cal[0];
	int16_t out_y_hard = compass.out_y_raw - hard_cal[1];
	int16_t out_z_hard = compass.out_z_raw - hard_cal[2];

	compass.out_x_cal = (int32_t) (out_x_hard * soft_cal_num[0][0]) / soft_cal_den[0][0] 
					+ (int32_t) (out_y_hard * soft_cal_num[0][1]) / soft_cal_den[0][1]
					+ (int32_t) (out_z_hard * soft_cal_num[0][2]) / soft_cal_den[0][2];
	compass.out_y_cal = (int32_t) (out_x_hard * soft_cal_num[1][0]) / soft_cal_den[1][0] 
					+ (int32_t) (out_y_hard * soft_cal_num[1][1]) / soft_cal_den[1][1]
					+ (int32_t) (out_z_hard * soft_cal_num[1][2]) / soft_cal_den[1][2];
	compass.out_z_cal = (int32_t) (out_x_hard * soft_cal_num[2][0]) / soft_cal_den[2][0] 
					+ (int32_t) (out_y_hard * soft_cal_num[2][1]) / soft_cal_den[2][1]
					+ (int32_t) (out_z_hard * soft_cal_num[2][2]) / soft_cal_den[2][2];

	compass.sampled_x += compass.out_x_cal;
	compass.sampled_y += compass.out_y_cal;
	compass.sampled_z += compass.out_z_cal;
}

inline compass_s* get_compass_data()
{
	return &compass;
}
