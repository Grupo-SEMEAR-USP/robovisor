#ifndef __COMMON__
#define __COMMON__

#include <stdio.h>

#include "pico/stdlib.h"

#define INITIAL_TIMEOUT_MS 250

typedef enum
{
    LEFT,
    RIGHT
} MOTOR_SIDE;

typedef enum 
{
    CCW,
    CW
} MOTOR_ROTATION;

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

#endif