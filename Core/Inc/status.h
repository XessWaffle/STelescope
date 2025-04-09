/*
 * Created on 04/09/2025
 * Author: Vatsal
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

#define UPDATE_REQUIRED_SHFT 0

typedef enum
{
    STATUS_SENS,
    STATUS_PWR,
    STATUS_STAT,
    STATUS_MODE,
    STATUS_LED_NUM
} status_led_e;


typedef struct
{
    status_led_e led;
    uint32_t grb_color : 24;
    uint32_t _flags : 8;
} status_led_s;

extern status_led_s status_leds[STATUS_LED_NUM];

uint8_t init_status_leds();

void set_status_led(status_led_e led, uint32_t grb_color);

void update();

#endif /* INC_STATUS_H_ */