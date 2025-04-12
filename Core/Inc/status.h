/*
 * Created on 04/09/2025
 * Author: Vatsal
 */

#ifndef INC_STATUS_H_
#define INC_STATUS_H_

#include "main.h"

#define WS2812B_WRITE_BITS 24
#define GET_BUFF_POS 0xFFFF00
#define CCR_VAL_MASK 0xFF

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
    union
    {
        uint8_t grb_color[3];
        uint32_t grb_color_hex : 24;
    };
} status_led_s;

typedef struct
{
    uint16_t buff_pos;
    uint8_t ccr_val;
} status_led_iterator_s;


uint8_t init_status_leds();

void set_status_led(status_led_e led, uint32_t grb_color);

status_led_iterator_s get_next_pulse_cc(uint8_t reset);
void update_leds();

#endif /* INC_STATUS_H_ */