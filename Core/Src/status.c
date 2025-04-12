#include "status.h"

extern TIM_HandleTypeDef htim3;
extern uint32_t tim3_counter_max;

status_led_s status_leds[STATUS_LED_NUM];

uint8_t init_status_leds()
{
    for (int i = 0; i < STATUS_LED_NUM; i++)
    {
        status_leds[i].led = i;
        set_status_led(i, 0x000000);
    }

    update_leds();

    return TRUE;   
}

void set_status_led(status_led_e led, uint32_t grb_color)
{
    if (led < STATUS_LED_NUM)
        status_leds[led].grb_color_hex = grb_color;
}

status_led_iterator_s get_next_pulse_cc(uint8_t reset)
{
    static uint16_t buff_pos = 0;
    static uint8_t buff[WS2812B_WRITE_BITS * STATUS_LED_NUM] = {0};

    if(reset == TRUE)
    {
        buff_pos = 0;
        
        for(uint16_t i = 0; i < (WS2812B_WRITE_BITS * STATUS_LED_NUM); i++)
        {
            status_led_e led = STATUS_LED_NUM - (i / WS2812B_WRITE_BITS) - 1;
            uint8_t bit = WS2812B_WRITE_BITS - (i % WS2812B_WRITE_BITS) - 1;
            buff[i] = ((0x1 << bit) & status_leds[led].grb_color_hex) > 0 ? 2 : 1;
        }
        
        return (status_led_iterator_s) {buff_pos, 0};

    }
    
    uint8_t ccr_val = 0;
    if(buff_pos < (WS2812B_WRITE_BITS * STATUS_LED_NUM))
        ccr_val = buff[buff_pos];
    
    buff_pos++;

    return (status_led_iterator_s) {buff_pos, ccr_val};
}

void update_leds()
{
    get_next_pulse_cc(TRUE);

    //HAL_TIM_Base_Start_IT(&htim3);

    HAL_Delay(500);
}