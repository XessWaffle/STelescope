/*
 * display.c
 *
 *  Created on: Mar 22, 2025
 *      Author: Vatsal
 */

#include "main.h"
#include "display.h"

display_s display;

uint8_t display_tx_buff[DISPLAY_BUFFER_LENGTH];

uint8_t populate_display_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= DISPLAY_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	display_tx_buff[buff_pos++] = byte;
	return buff_pos;
}

void set_display_read(uint8_t read, uint16_t buffer_length)
{
	display.disp_spi.data.read = read;
	display.disp_spi.data.tx_buff_length = buffer_length;
	display.disp_spi.data.rx_buff_length = buffer_length;
}

void reset_display()
{
	HAL_GPIO_WritePin(display.rst_port, display.rst, GPIO_PIN_SET);
	HAL_Delay(5);
	HAL_GPIO_WritePin(display.rst_port, display.rst, GPIO_PIN_RESET);
	HAL_Delay(10);
	HAL_GPIO_WritePin(display.rst_port, display.rst, GPIO_PIN_SET);
	HAL_Delay(120);
}

void write_command(uint8_t cmd)
{
	HAL_GPIO_WritePin(display.dc_port, display.dc, GPIO_PIN_RESET);

	uint8_t length = populate_display_tx_buff(TRUE, cmd);
	set_display_read(FALSE, length);

	rtx_spi(&(display.disp_spi), TRUE);
}

void write_data(uint8_t data)
{
	HAL_GPIO_WritePin(display.dc_port, display.dc, GPIO_PIN_SET);

	uint8_t length = populate_display_tx_buff(TRUE, data);
	set_display_read(FALSE, length);

	rtx_spi(&(display.disp_spi), TRUE);
}

void write_data16(uint16_t data)
{
	HAL_GPIO_WritePin(display.dc_port, display.dc, GPIO_PIN_SET);

	populate_display_tx_buff(TRUE, data >> 8);
	uint8_t length = populate_display_tx_buff(FALSE, data & 0xFF);
	set_display_read(FALSE, length);

	rtx_spi(&(display.disp_spi), TRUE);
}

void draw_post_init()
{
	// Set column address (240x240 resolution)
	write_command(GC9A01A_CASET);
	write_data16(0);    // Start column
	write_data16(239);  // End column

	// Set row address
	write_command(GC9A01A_RASET);
	write_data16(0);    // Start row
	write_data16(239);  // End row

	// Start writing to RAM
	write_command(GC9A01A_RAMWR);

	// Write color data for all pixels (240 * 240 = 57600 pixels)
	HAL_GPIO_WritePin(display.dc_port, display.dc, GPIO_PIN_SET);  // Data mode

	// Optimize by keeping CS low for the entire transfer
	for(uint32_t it = 0; it < 240; it++)
	{
		for (uint32_t i = 0; i < 240; i++) {
			display_tx_buff[2*i] = (it * 240 + i) >> 8;
			display_tx_buff[2*i + 1] = (it * 240 + i) & 0xFF;
		}

		set_display_read(FALSE, DISPLAY_BUFFER_LENGTH);
		rtx_spi(&(display.disp_spi), TRUE);
	}

	HAL_Delay(100);

}


uint8_t init_display()
{
	display.disp_spi.cs = DISP_CS1_Pin;
	display.disp_spi.port = DISP_CS1_GPIO_Port;

	display.dc = DISP_DC_Pin;
	display.dc_port = DISP_DC_GPIO_Port;

	display.rst = DISP_RST_Pin;
	display.rst_port = DISP_RST_GPIO_Port;

	display.disp_spi.data.handle = SPI_2;
	display.disp_spi.data.tx_buff = display_tx_buff;
	display.disp_spi.data.rx_buff = NULL;


	reset_display();

	// Software reset
	write_command(GC9A01A_SWRESET);
	HAL_Delay(120);

	/*
	 * Thanks Adafruit
	 */
	uint8_t cmd, x, numArgs;
	uint16_t addr = 0;
	while ((cmd = initcmd[addr++]) > 0) {
		x = initcmd[addr++];
		numArgs = x & 0x7F;
		write_command(cmd);

		for(int i = 0; i < numArgs; i++)
		{
			write_data(initcmd[addr + i]);
		}

		addr += numArgs;
		if (x & 0x80)
		  HAL_Delay(150);
	}

	draw_post_init();

	return TRUE;

}
