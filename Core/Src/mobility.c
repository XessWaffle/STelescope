#include "mobility.h"
#include "compass.h"
#include "accelerometer.h"

/*
 * Home azimuth and altitude axes one by one
 */
void home(axes_e axis)
{

    switch (axis)
    {
        case YAW:
        {
            int32_t mag_x[YAW_MA_LENGTH];
            int32_t mag_max = 0, mag_sum = 0;

            const int16_t hysteresis = 1;
            const uint8_t switch_dir_crit = 3;
            int32_t rate = 60000;
            int32_t mag_point = 0, prev_mag_point = 0;
            uint8_t loops = 0;
            uint8_t idx = 0;
            uint8_t switch_dir = 0, num_switches = 0;
            uint8_t exit = FALSE;
            microstep_e mode = get_microstep_mode();
            compass_s *compass_data = get_compass_data();
            

            for(int i = 0; i < YAW_MA_LENGTH; i++)
            {
                mag_x[i] = 0;
            }

            set_stepper_state(HOMING);
            set_microstep_mode(FULL);
            set_stepper_rate(axis, rate);

            do
            {
                // Get mode for conditional checks
                mode = get_microstep_mode();
                
                // Sample the compass data with 16 samples for averaging
                dacq_compass();

                // Delay to allow for stable readings
                HAL_Delay(100);

                // Subtract to be be overwritten value
                mag_sum -= mag_x[idx];
                mag_x[idx] = compass_data->out_x_raw;
                // Add new value
                mag_sum += mag_x[idx];
                prev_mag_point = mag_point;
                mag_point = mag_sum / YAW_MA_LENGTH;

                if(mag_max < mag_point)
                    mag_max = mag_point;

                if(loops <= YAW_MA_LENGTH)
                    loops++;

                // Check for direction switch based on magnetic reading trends
                if((loops > YAW_MA_LENGTH) && (mode == FULL && (mag_point + hysteresis) < prev_mag_point))
                {
                    switch_dir++;
                    // Reverse stepper motor direction after multiple switches
                    if(switch_dir > switch_dir_crit)
                    {
                        rate = -1 * rate;
                        set_stepper_rate(axis, rate);
                        num_switches++;
                        loops = 0;
                    }

                    // Adjust microstepping mode after every two switches
                    if(num_switches % 2 == 0 && num_switches > 0)
                    {
                        set_microstep_mode(HALF);
                        num_switches = 0;
                    }
                }
                else if(mode == FULL)
                {
                    switch_dir = 0;
                }
                else if (mode != FULL && (loops > YAW_MA_LENGTH))
                {
                    int16_t diff = mag_max - mag_point;
                    if(diff < 25 && diff > -25)
                    {
                        set_microstep_mode(SIXTEENTH);
                    }
                    else if(diff < 50 && diff > -50)
                    {
                        set_microstep_mode(EIGHTH);
                    }
                    else if(diff < 100 && diff > -100)
                    {
                        set_microstep_mode(QUARTER);
                    }
                }

                // Update the current and previous indices for magnetic readings
                
                idx++;
                idx = idx % YAW_MA_LENGTH;
                
                /* Homing Criteria met with hysteresis */
                if(get_microstep_mode() == SIXTEENTH && mag_point + hysteresis > mag_max && mag_point - hysteresis < mag_max)
                {
                    HAL_Delay(50);
                    exit = TRUE;
                }

            } while(exit == FALSE);

            set_stepper_rate(axis, 0);
            set_stepper_position(axis, 0);
            break;
        }
        case PITCH:
        {
            // Positive rate, pitch down,
            // Negative rate, pitch up,
            int32_t accel_y[PITCH_MA_LENGTH];
            int32_t accel_sum = 0;

            const int16_t hysteresis = 2;
            const uint8_t switch_dir_crit = 3;
            int32_t rate = 50000;
            int32_t accel_point = 0, prev_accel_point = 0;
            uint8_t loops = 0;
            uint8_t idx = 0;
            uint8_t switch_dir = 0, num_switches = 0;
            uint8_t exit = FALSE;
            microstep_e mode = get_microstep_mode();
            accelerometer_s *accel_data = get_accelerometer_data();
            

            for(int i = 0; i < PITCH_MA_LENGTH; i++)
            {
                accel_y[i] = 0;
            }

            set_stepper_state(HOMING);
            set_microstep_mode(FULL);
            set_stepper_rate(axis, rate);
            
            

            HAL_Delay(5000);

            do
            {
                // Get mode for conditional checks
                mode = get_microstep_mode();
                
                // Sample the compass data with 16 samples for averaging
                dacq_accelerometer();

                // Delay to allow for stable readings
                HAL_Delay(100);
                
                if(accel_data->out_x_a_raw < 0)
                {
                    set_stepper_rate(axis, -50000);
                    num_switches = 1;
                    HAL_Delay(10000);
                }

                // Subtract to be be overwritten value
                accel_sum -= accel_y[idx];
                accel_y[idx] = accel_data->out_y_a_raw;
                // Add new value
                accel_sum += accel_y[idx];
                prev_accel_point = accel_point;
                accel_point = accel_sum / PITCH_MA_LENGTH;

                int32_t accel_test = accel_point < 0 ? -1 * accel_point : accel_point;
                int32_t prev_accel_test = prev_accel_point < 0 ? -1 * prev_accel_point : prev_accel_point;

                if(loops <= PITCH_MA_LENGTH)
                    loops++;

                // Check for direction switch based on magnetic reading trends
                if((loops > PITCH_MA_LENGTH) && (mode == FULL && (accel_test - hysteresis) > prev_accel_test))
                {
                    switch_dir++;
                    // Reverse stepper motor direction after multiple switches
                    if(switch_dir > switch_dir_crit)
                    {
                        rate = -1 * rate;
                        set_stepper_rate(axis, rate);
                        num_switches++;
                        loops = 0;
                    }

                    // Adjust microstepping mode after every two switches
                    if(num_switches == 1)
                    {
                        set_microstep_mode(HALF);
                        num_switches = 0;
                    }
                }
                else if(mode == FULL)
                {
                    switch_dir = 0;
                }
                else if (mode != FULL && (loops > PITCH_MA_LENGTH))
                {
                    int16_t diff = accel_point;
                    if(diff < 10 && diff > -10)
                    {
                        set_stepper_rate(axis, rate < 0 ? -10000: 10000);
                        set_microstep_mode(SIXTEENTH);
                    }
                    else if(diff < 50 && diff > -50)
                    {
                        set_stepper_rate(axis, rate < 0 ? -20000: 20000);
                        set_microstep_mode(EIGHTH);
                    }
                    else if(diff < 100 && diff > -100)
                    {
                        set_microstep_mode(QUARTER);
                    }
                }

                // Update the current and previous indices for magnetic readings
                
                idx++;
                idx = idx % PITCH_MA_LENGTH;
                
                /* Homing Criteria met with hysteresis */
                if(get_microstep_mode() == SIXTEENTH && (accel_point + hysteresis) > 0 && (accel_point - hysteresis) < 0)
                {
                    HAL_Delay(50);
                    exit = TRUE;
                }

            } while(exit == FALSE);

            set_stepper_rate(axis, 0);
            set_stepper_position(axis, 0);
            break;
        }
        case ROLL:
            set_stepper_position(axis, 0);
            set_stepper_rate(axis, 0);
            break;
        default:
            break;
    }
}

/*
 * Align to target declination and right ascension
 */
void align(angle_s dec, angle_s ra);

/*
 * Triggered after alignment is complete, and adjust rates based on night sky movement
 */
void track();