/*
 * stepper_driver.h
 *
 *  Created on: Mar 14, 2025
 *      Author: Vatsal
 */

#ifndef INC_MOBILITY_H_
#define INC_MOBILITY_H_

#include "stepper.h"

#define YAW_MA_LENGTH 12
#define PITCH_MA_LENGTH 3

/*
 * For Altitude 0 = Horizon
 * 	   Azimuth	0 = Sensed North Pole
 *
 * For Declination / Right Ascension values are given as is
 */
typedef struct
{
	uint32_t deg : 9; // 0 - 359
	uint32_t asec : 12; // 0 - 3599

	uint32_t res : 11;
} angle_s;

/*
 * Home azimuth and altitude axes one by one
 */
void home(axes_e axis);

/*
 * Align to target declination and right ascension
 */
void align(angle_s dec, angle_s ra);

/*
 * Triggered after alignment is complete, and adjust rates based on night sky movement
 */
void track();


#endif /* INC_MOBILITY_H_ */
