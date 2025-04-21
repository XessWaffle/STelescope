/*
 * stepper.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_STEPPER_H_
#define INC_STEPPER_H_

#include "main.h"

// Steps per minute
#define MAX_RATE 100000

#define DIRECTION_CHANGE_FLAG 0
#define INIT_DIRECTION_FLAG 1
#define RESET_POSITION_FLAG 2

#define RAMP_DIVIDER 64

#define MAX_POSITION 0x7FFFFFFF

#define STEPPER(axis) stepper_state.stepper[axis]

typedef enum
{
	YAW = 0,
	ROLL = 1,
	PITCH = 2,
	AXES
} axes_e;

typedef enum
{
	FULL,
	HALF,
	QUARTER,
	EIGHTH,
	SIXTEENTH,
	INVALID
} microstep_e;

typedef enum
{
	TRACKING,
	SLEWING,
	HOMING,
	SLEEP
} stepper_op_mode_e;

typedef struct
{
	// Pins to be accessed in ISR
	GPIO_TypeDef *step_pin_port;
	GPIO_TypeDef *dir_pin_port;
	uint16_t step_pin;
	uint16_t dir_pin;

	// Assuming no missed steps
	int32_t position;

	// Steps per minute
	// dir_pin = rate < 0 ? RESET : SET
	int32_t rate;
	int32_t _prev_rate;

	int32_t _desired_position;

	uint32_t _desired_arr_rate; 
	uint32_t _current_arr_rate; 
	uint32_t _tick;

	int32_t _dir_change_des_rate;
	uint32_t _step_count;
	
	uint8_t _flags;


} stepper_s;

typedef struct
{
	stepper_s stepper[AXES];
	microstep_e step_mode;
	stepper_op_mode_e state;

	uint32_t update_freq;

} stepper_state_s;

extern stepper_state_s stepper_state;

/*
 * Initialize stepper motor structs and pins
 */
uint8_t init_stepper();

uint8_t is_stopped();

microstep_e get_microstep_mode();
stepper_op_mode_e get_stepper_state();
uint32_t get_stepper_position(axes_e axis);
uint32_t get_stepper_rate(axes_e axis);

void set_update_freq(uint32_t freq_hz);
void set_microstep_mode(microstep_e mode);
void set_stepper_state(stepper_op_mode_e state);
void set_stepper_position(axes_e axis, int32_t position);
void set_desired_stepper_position(axes_e axis, int32_t position);
uint8_t set_stepper_rate(axes_e axis, int32_t rate);

uint8_t increment_tick(axes_e axis);
void step_axis(axes_e axis);

uint8_t* get_stepper_flags(axes_e axis);

#endif /* INC_STEPPER_H_ */
