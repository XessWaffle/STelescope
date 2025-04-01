/*
 * accelerometer.c
 *
 *  Created on: Mar 21, 2025
 *      Author: Vatsal
 */
#include <accelerometer.h>

accelerometer_s accelerometer;

uint8_t accelerometer_rx_buff[ACCELEROMETER_BUFFER_LENGTH];
uint8_t accelerometer_tx_buff[ACCELEROMETER_BUFFER_LENGTH];

uint8_t populate_accelerometer_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= ACCELEROMETER_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	accelerometer_tx_buff[buff_pos++] = byte;
	return buff_pos;
}

uint8_t init_accelerometer()
{
	/*
	 * Constants
	 */
	accelerometer.accel_i2c.address = 0x6A;
	accelerometer.accel_i2c.data.rx_buff = accelerometer_rx_buff;
	accelerometer.accel_i2c.data.tx_buff = accelerometer_tx_buff;
	accelerometer.accel_i2c.data.handle = I2C_1;
	accelerometer.accel_i2c.i2c_cb = drdy_accelerometer_cb;

	/*
	 * Validate that I2C communication is established
	 */
	uint8_t length = populate_accelerometer_tx_buff(TRUE, LSM6DSOX_WHO_AM_I);
	set_accelerometer_read(TRUE, length, 1);
	while(rtx_i2c(&(accelerometer.accel_i2c)) == HAL_BUSY);

	HAL_Delay(100);

	/*
	 * Expected response
	 */
	if(accelerometer_rx_buff[0] != 0x6C)
		return FALSE;

	/*
	 * Transmit
	 */
	populate_accelerometer_tx_buff(TRUE, LSM6DSOX_CTRL1_XL);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_XL_ODR_52HZ | LSM6DSOX_XL_FS_2G | LSM6DSOX_XL_LPF1_BW_SEL);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_G_ODR_52HZ | LSM6DSOX_G_FS_250DPS | LSM6DSOX_G_FS_125);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_IF_INC);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_LPF1_SEL_G);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_ROUNDING_NONE | LSM6DSOX_ST_G_OFF | LSM6DSOX_ST_XL_OFF);
	populate_accelerometer_tx_buff(FALSE, LSM6DSOX_FTYPE_0);
	populate_accelerometer_tx_buff(FALSE, 0x0);
	populate_accelerometer_tx_buff(FALSE, 0x0);
	populate_accelerometer_tx_buff(FALSE, 0x0);
	length = populate_accelerometer_tx_buff(FALSE, 0x0);
	set_accelerometer_read(FALSE, length, 0);
	while(rtx_i2c(&(accelerometer.accel_i2c)) == HAL_BUSY);

	return TRUE;

}

inline void dacq_accelerometer()
{
	uint8_t length = populate_accelerometer_tx_buff(TRUE, LSM6DSOX_OUTX_L_G);
	set_accelerometer_read(TRUE, length, 12);
	while(rtx_i2c(&(accelerometer.accel_i2c)) == HAL_BUSY);
}

inline void set_accelerometer_read(uint8_t read, uint8_t tx_buffer_length, uint8_t rx_buffer_length)
{
	accelerometer.accel_i2c.data.read = read;
	accelerometer.accel_i2c.data.tx_buff_length = tx_buffer_length;
	accelerometer.accel_i2c.data.rx_buff_length = rx_buffer_length;
}

inline void drdy_accelerometer_cb()
{
	accelerometer.out_x_g_raw = accelerometer_rx_buff[0] | (accelerometer_rx_buff[1] << 8);
	accelerometer.out_y_g_raw = accelerometer_rx_buff[2] | (accelerometer_rx_buff[3] << 8);
	accelerometer.out_z_g_raw = accelerometer_rx_buff[4] | (accelerometer_rx_buff[5] << 8);

	accelerometer.out_x_a_raw = accelerometer_rx_buff[6] | (accelerometer_rx_buff[7] << 8);
	accelerometer.out_y_a_raw = accelerometer_rx_buff[8] | (accelerometer_rx_buff[9] << 8);
	accelerometer.out_z_a_raw = accelerometer_rx_buff[10] | (accelerometer_rx_buff[11] << 8);
}

inline accelerometer_s* get_accelerometer_data()
{
	return &accelerometer;
}
