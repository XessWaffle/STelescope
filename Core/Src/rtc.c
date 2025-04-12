#include "rtc.h"

rtc_s rtc;

uint8_t rtc_rx_buff[RTC_BUFFER_LENGTH];
uint8_t rtc_tx_buff[RTC_BUFFER_LENGTH];

uint8_t populate_rtc_tx_buff(uint8_t reset, uint8_t byte)
{
	static int buff_pos = 0;

	if(reset == TRUE)
		buff_pos = 0;

	if(buff_pos >= RTC_BUFFER_LENGTH)
	{
		buff_pos = 0;
		return buff_pos;
	}

	rtc_tx_buff[buff_pos++] = byte;
	return buff_pos;
}

void set_rtc_i2c_read(uint8_t read, uint16_t rx_buffer_length, uint16_t tx_buffer_length)
{
	rtc.rtc_i2c.data.read = read;
	rtc.rtc_i2c.data.tx_buff_length = tx_buffer_length;
	rtc.rtc_i2c.data.rx_buff_length = rx_buffer_length;
}

uint8_t init_rtc()
{
    rtc.rtc_i2c.address = DS3231_I2C_ADDRESS;
    rtc.rtc_i2c.data.handle = I2C_1;
    rtc.rtc_i2c.i2c_cb = drdy_rtc_cb;
    rtc.rtc_i2c.data.rx_buff = rtc_rx_buff;
    rtc.rtc_i2c.data.tx_buff = rtc_tx_buff;
    
    uint8_t length = populate_rtc_tx_buff(TRUE, DS3231_REG_SECONDS);
    set_rtc_i2c_read(TRUE, length, 1);
    while(rtx_i2c(&(rtc.rtc_i2c)) == HAL_BUSY);

    if(rtc_rx_buff[0] >= 60)
        return FALSE;

    populate_rtc_tx_buff(TRUE, DS3231_REG_CONTROL);
    length = populate_rtc_tx_buff(FALSE, DS3231_CONTROL_EOSC | DS3231_SQW_1HZ);
    set_rtc_i2c_read(FALSE, 1, length);
    while(rtx_i2c(&(rtc.rtc_i2c)) == HAL_BUSY);

    return TRUE;
}

void dacq_rtc()
{
    uint8_t length = populate_rtc_tx_buff(TRUE, DS3231_REG_SECONDS);
    set_rtc_i2c_read(TRUE, length, 7);
    while(rtx_i2c(&(rtc.rtc_i2c)) == HAL_BUSY);
}

void drdy_rtc_cb()
{
    uint8_t *rtc_rx_buff = rtc.rtc_i2c.data.rx_buff;
    
    rtc.seconds = rtc_rx_buff[0];
    rtc.minutes = rtc_rx_buff[1];
    rtc.hours = rtc_rx_buff[2];
    rtc.day = rtc_rx_buff[3];
    
    rtc.date = rtc_rx_buff[4];
    rtc.month = rtc_rx_buff[5];
    rtc.year = (rtc_rx_buff[6] + 2000);
}