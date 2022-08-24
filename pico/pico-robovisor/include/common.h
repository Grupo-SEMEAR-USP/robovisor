#ifndef __COMMON__
#define __COMMON__

#include <stdio.h>

#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/time.h"

#define INITIAL_TIMEOUT_MS 10
#define DEBUG_MAIN 1
#define DEBUG_MAIN_RECEIVE 0
#define DEBUG_PWM 0
#define DEBUG_SEND_CORE0 0
#define DEBUG_SEND_ROS 0
#define DEBUG_MEAN_VELOCITY 0
#define DEBUG_ENCODER_CALLBACK 0
#define DEBUG_ENCODER_PROCESS 0
#define DEBUG_PID 0

#define MAX_VELOCITY_SPIKE 2000

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
