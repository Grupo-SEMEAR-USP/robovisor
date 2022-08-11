#include <stdio.h>

#include "pico/stdlib.h"

#define PICO_MOTOR_L_CHB 22
#define PICO_MOTOR_L_BRK 21
#define PICO_MOTOR_L_PWM 20
#define PICO_MOTOR_L_CHA 19
#define PICO_MOTOR_L_DIR 18

#define PICO_MOTOR_R_CHB 11
#define PICO_MOTOR_R_BRK 12
#define PICO_MOTOR_R_PWM 13
#define PICO_MOTOR_R_CHA 14
#define PICO_MOTOR_R_DIR 15

#define PICO_SDA_5V 6
#define PICO_SCL_5V 7
#define PICO_SDA_0 8
#define PICO_SCL_0 9

/**
 * @brief main functionality of the low level control features of application
 *
 * TODO: 
 * PID logic control
 *
 * FIRST: 
 * Read displacement angles, in degrees, since last call 
 * 
 * SECOND:
 * Return values of previously read angle displacement for left THEN right values
 *
 * THIRD:
 * Read values for velocity commands (in m/s), in the same order as mentioned above
 * receives two uint8 values, that when concatenated (each pair), recovers true velocity command
 * If no velocity command is given, than previous one is maintained 
 *
 * FOURTH:
 * Send this command to set_velocity function, whose implementation is in another file
 * 
 */

int main(void)
{

    

}
