/*
 * Created on 04/09/2025
 * Author: Vatsal
 */

#ifndef INC_RTC_H_
#define INC_RTC_H_

#include "peripheral.h"
#include "ds3231.h"

#define RTC_BUFFER_LENGTH 8

typedef struct
{
    i2c_per_s rtc_i2c;

    uint8_t seconds;
    uint8_t minutes;
    uint8_t hours;
    uint8_t day;

    uint8_t date;
    uint8_t month;
    uint16_t year;
} rtc_s;

extern rtc_s rtc;

uint8_t init_rtc();
void dacq_rtc();
void drdy_rtc_cb();

#endif /* INC_RTC_H_ */