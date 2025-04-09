/*
 * camera.h
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 */

#include "main.h"
#include "camera.h"

camera_s camera;

uint8_t camera_rx_buff[CAMERA_BUFFER_LENGTH];
uint8_t camera_tx_buff[CAMERA_BUFFER_LENGTH];

uint8_t populate_camera_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= CAMERA_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	camera_tx_buff[buff_pos++] = byte;
	return buff_pos;
}

void set_camera_spi_read(uint8_t read, uint16_t buffer_length)
{
	camera.cam_spi.data.read = read;
	camera.cam_spi.data.tx_buff_length = buffer_length;
	camera.cam_spi.data.rx_buff_length = buffer_length;
}


void set_camera_i2c_read(uint8_t read, uint16_t rx_buffer_length, uint16_t tx_buffer_length)
{
	camera.cam_i2c.data.read = read;
	camera.cam_i2c.data.tx_buff_length = tx_buffer_length;
	camera.cam_i2c.data.rx_buff_length = rx_buffer_length;
}


uint8_t init_camera()
{
    /*
     * Initialize SPI
     */
    camera.cam_spi.cs = CAM_CS_Pin;
	camera.cam_spi.port = CAM_CS_GPIO_Port;
    camera.cam_spi.spi_cb = NULL;

	camera.cam_spi.data.handle = SPI_1;
	camera.cam_spi.data.tx_buff = camera_tx_buff;
	camera.cam_spi.data.rx_buff = camera_rx_buff;
    
    /*
     * Initialize I2C
     */
    camera.cam_i2c.address = OV2640_I2C_ADDR;
    camera.cam_i2c.i2c_cb = NULL;

    camera.cam_i2c.data.handle = I2C_2;
    camera.cam_i2c.data.tx_buff = camera_tx_buff;
    camera.cam_i2c.data.rx_buff = camera_rx_buff;

    HAL_GPIO_WritePin(camera.cam_spi.port, camera.cam_spi.cs, GPIO_PIN_SET);
    
    HAL_StatusTypeDef status;
    populate_camera_tx_buff(TRUE, 0x07 | 0x80);
    uint16_t length = populate_camera_tx_buff(FALSE, 0x80);
    set_camera_spi_read(FALSE, length);
    status = rtx_spi(&(camera.cam_spi), TRUE);
    HAL_Delay(100);

    populate_camera_tx_buff(TRUE, 0x07 | 0x7f);
    length = populate_camera_tx_buff(FALSE, 0x00);
    set_camera_spi_read(FALSE, length);
    status = rtx_spi(&(camera.cam_spi), TRUE);
    HAL_Delay(100);

    populate_camera_tx_buff(TRUE, ARDUCHIP_TEST1 | 0x80);
    length = populate_camera_tx_buff(FALSE, 0x55);
    set_camera_spi_read(FALSE, length);
    status = rtx_spi(&(camera.cam_spi), TRUE);
    HAL_Delay(100);

    populate_camera_tx_buff(TRUE, ARDUCHIP_TEST1);
    length = populate_camera_tx_buff(FALSE, 0x00);
    set_camera_spi_read(TRUE, length);
    status = rtx_spi(&(camera.cam_spi), TRUE);
    HAL_Delay(100);



    /*
     * Validate that I2C communication is established
     */
    populate_camera_tx_buff(TRUE, OV2640_REG_SENSOR_RESET);
    length = populate_camera_tx_buff(FALSE, 0x01);
    set_camera_i2c_read(FALSE, 0, length);
    while(rtx_i2c(&(camera.cam_i2c)) == HAL_BUSY);
    HAL_Delay(100);


    length = populate_camera_tx_buff(TRUE, OV2640_REG_PIDH);
    set_camera_i2c_read(TRUE, 1, length);
    while(rtx_i2c(&(camera.cam_i2c)) == HAL_BUSY);
    HAL_Delay(100);

    /*
     * Expected response
     */
    
    if(camera_rx_buff[0] != 0x26 && camera_rx_buff[1] != 0x42)
        Error_Handler();

    return TRUE;
}