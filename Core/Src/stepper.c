#include "stepper.h"

stepper_state_s stepper_state;

extern uint32_t tim2_tick_sec;

uint8_t init_stepper()
{
    /* Reset A4988*/
    HAL_GPIO_WritePin(ST_DIR_Y_GPIO_Port, ST_DIR_Y_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_DIR_P_GPIO_Port, ST_DIR_P_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_DIR_R_GPIO_Port, ST_DIR_R_Pin, GPIO_PIN_RESET);
  
    HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_RESET);

    /*
     * Initialize stepper motor structs
     */
    set_microstep_mode(FULL);
    set_stepper_state(SLEEP);
    set_update_freq(tim2_tick_sec);

    for(int i = 0; i < AXES; i++)
    {
        set_stepper_position((axes_e) i, 0);
        set_stepper_rate((axes_e) i, 0);

        stepper_state.stepper[i]._desired_arr_rate = tim2_tick_sec;
        stepper_state.stepper[i]._current_arr_rate = tim2_tick_sec;
        stepper_state.stepper[i]._flags |= (TRUE << INIT_DIRECTION_FLAG);

        switch (i)
        {
            case YAW:
                stepper_state.stepper[i].step_pin_port = ST_STEP_Y_GPIO_Port;
                stepper_state.stepper[i].dir_pin_port = ST_DIR_Y_GPIO_Port;
                stepper_state.stepper[i].step_pin = ST_STEP_Y_Pin;
                stepper_state.stepper[i].dir_pin = ST_DIR_Y_Pin;
                break;
            case ROLL:
                stepper_state.stepper[i].step_pin_port = ST_STEP_R_GPIO_Port;
                stepper_state.stepper[i].dir_pin_port = ST_DIR_R_GPIO_Port;
                stepper_state.stepper[i].step_pin = ST_STEP_R_Pin;
                stepper_state.stepper[i].dir_pin = ST_DIR_R_Pin;
                break;
            case PITCH:
                stepper_state.stepper[i].step_pin_port = ST_STEP_P_GPIO_Port;
                stepper_state.stepper[i].dir_pin_port = ST_DIR_P_GPIO_Port;
                stepper_state.stepper[i].step_pin = ST_STEP_P_Pin;
                stepper_state.stepper[i].dir_pin = ST_DIR_P_Pin;
                break;
        }

    }

    return TRUE;
}

inline microstep_e get_microstep_mode()
{
    return stepper_state.step_mode;
}

inline stepper_op_mode_e get_stepper_state()
{
    return stepper_state.state;
}

inline uint32_t get_stepper_position(axes_e axis)
{
    return stepper_state.stepper[axis].position;
}

inline uint32_t get_stepper_rate(axes_e axis)
{
    return stepper_state.stepper[axis].rate;
}

inline void set_update_freq(uint32_t freq_hz)
{
    stepper_state.update_freq = freq_hz;
}

inline void set_microstep_mode(microstep_e mode)
{
    stepper_state.step_mode = mode;
    
    switch (mode)
    {   
        case FULL:
            HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_RESET);
            break;
        case HALF:
            HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_RESET);
            break;
        case QUARTER:
            HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_RESET);
            HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_RESET);
            break;
        case EIGHTH:
            HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_RESET);
            break;
        case SIXTEENTH:
            HAL_GPIO_WritePin(ST_MS1_GPIO_Port, ST_MS1_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS2_GPIO_Port, ST_MS2_Pin, GPIO_PIN_SET);
            HAL_GPIO_WritePin(ST_MS3_GPIO_Port, ST_MS3_Pin, GPIO_PIN_SET);
            break;
        default:
            break;
    }

}

inline void set_stepper_state(stepper_op_mode_e state)
{
    stepper_state.state = state;
}

inline void set_stepper_position(axes_e axis, uint32_t position)
{
    stepper_state.stepper[axis].position = position;
}

inline uint8_t set_stepper_rate(axes_e axis, int32_t rate)
{
    static int32_t prev_rate_axis[AXES] = {0};

    int32_t *prev_rate = &(prev_rate_axis[axis]);
    uint8_t *flags = get_stepper_flags(axis);

    if(*flags & (TRUE << INIT_DIRECTION_FLAG))
    {
        HAL_GPIO_WritePin(stepper_state.stepper[axis].dir_pin_port, stepper_state.stepper[axis].dir_pin, 
            rate < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
        *flags &= ~(TRUE << INIT_DIRECTION_FLAG);
    }
    else if(rate == 0)
    {
        *flags |= (TRUE << INIT_DIRECTION_FLAG);
    }

    if(((*prev_rate > 0 && rate < 0) || (*prev_rate < 0 && rate >= 0)))
    {   
        *flags = TRUE << DIRECTION_CHANGE_FLAG;
        stepper_state.stepper[axis]._dir_change_des_rate = rate;
        rate = 0;
    }

    if(*prev_rate == rate && rate != 0)
        return *flags;
    
    uint32_t abs_rate = rate < 0 ? -rate : rate;

    stepper_state.stepper[axis].rate = rate;

    uint32_t *desired_arr = &(stepper_state.stepper[axis]._desired_arr_rate);
    uint32_t *current_arr = &(stepper_state.stepper[axis]._current_arr_rate);
    uint32_t *step_count = &(stepper_state.stepper[axis]._step_count);

    if(abs_rate > 0)
        *desired_arr = ((30 * stepper_state.update_freq) / abs_rate) - 1;
    else
        *desired_arr = stepper_state.update_freq;


    if(*desired_arr < stepper_state.update_freq / (RAMP_DIVIDER * 2) &&  *current_arr > stepper_state.update_freq / RAMP_DIVIDER)
        *current_arr = stepper_state.update_freq / RAMP_DIVIDER;

    *step_count = 0;
    *prev_rate = rate;

    return *flags;
}

inline uint8_t increment_tick(axes_e axis)
{ 
    uint32_t *desired_arr = &(stepper_state.stepper[axis]._desired_arr_rate);
    uint32_t *current_arr = &(stepper_state.stepper[axis]._current_arr_rate);
    uint8_t stop_condition = *desired_arr == stepper_state.update_freq;
    uint8_t stopped_condition = *current_arr == stepper_state.update_freq && *desired_arr == stepper_state.update_freq;

    uint8_t *flags = get_stepper_flags(axis);

    if(stopped_condition == TRUE)
    {
        if(*flags & (TRUE << DIRECTION_CHANGE_FLAG))
        {
            *flags &= ~(TRUE << DIRECTION_CHANGE_FLAG);
            HAL_GPIO_WritePin(stepper_state.stepper[axis].dir_pin_port, stepper_state.stepper[axis].dir_pin, 
                stepper_state.stepper[axis]._dir_change_des_rate < 0 ? GPIO_PIN_SET : GPIO_PIN_RESET);
            set_stepper_rate(axis, stepper_state.stepper[axis]._dir_change_des_rate);
        }

        return FALSE;
    }

    stepper_state.stepper[axis]._tick++;
    
    uint8_t step_condition = stepper_state.stepper[axis]._tick > *current_arr;
    uint32_t *step_count = &(stepper_state.stepper[axis]._step_count);

    if(step_condition == TRUE)
    {
        stepper_state.stepper[axis]._tick = 0;
        *step_count = *step_count + 1;
        uint32_t step_size = *current_arr / (4 * *step_count + 1);

        if(*desired_arr > *current_arr)
        {
            *current_arr = *current_arr + (step_size > 1 ? step_size : 1);

            if(*current_arr > *desired_arr)
                *current_arr = *desired_arr;

        }
        else if(*desired_arr < *current_arr)
        {
            *current_arr = *current_arr - (step_size > 1 ? step_size : 1);

            if(*current_arr < *desired_arr)
                *current_arr = *desired_arr;
        }


        if(stop_condition && *current_arr > stepper_state.update_freq / RAMP_DIVIDER)
            *current_arr = stepper_state.update_freq;
    }
        
    return step_condition;
}

inline void step_axis(axes_e axis)
{
    static uint8_t step_alt[AXES] = {1, 1, 1};

    if(get_stepper_state() == SLEEP)
        return;
    /* 
     * HOMING State uses sensors to calibrate position otherwise 
     * allow the stepper to move if it is not moving in the negative direction past 0
     */
    if(get_stepper_state() == HOMING 
        || !(stepper_state.stepper[axis].position == 0 && stepper_state.stepper[axis].rate < 0))
        HAL_GPIO_TogglePin(stepper_state.stepper[axis].step_pin_port, stepper_state.stepper[axis].step_pin);

    if(step_alt[axis] % 2 == 0)
        stepper_state.stepper[axis].position += stepper_state.stepper[axis].rate < 0 ? -1 : 1;

    step_alt[axis]++;
}

inline uint8_t* get_stepper_flags(axes_e axis)
{
    return &(stepper_state.stepper[axis]._flags);
}