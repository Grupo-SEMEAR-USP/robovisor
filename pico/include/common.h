#ifndef __COMMON__
#define __COMMON_

#include <stdio.h>

#include "pico/stdlib.h"

#define INITIAL_TIMEOUT_MS 250

#define LEFT 0
#define RIGHT 1 

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

/*Encoder GPIO*/
// GPIO 10 is Encoder phase A,
// GPIO 11 is Encoder phase B,
// GPIO 12 is the encoder push botton switch.
// change these as needed
#define ENC_A 10
#define ENC_B 11

uint32_t div = 0, top = 0;   
uint slice_num_l, channel_l; 
uint slice_num_r, channel_r; 

#endif