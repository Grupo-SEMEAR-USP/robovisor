#ifndef __PWM__
#define __PWM__

#include "common.h"                                                  
#include "hardware/pwm.h"                                                 
#include "hardware/clocks.h"      
#include "PID.h"                                        

// MIT License (MIT) Damien P. George Copyright (c) 2020 

// Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
#define PWM_FREQ 2000
#define TOP_MAX 65534
#define MAX_PWM 65535
#define MIN_PWM MAX_PWM/10

float absFloat(float value);
void init_pwm_pinnage();                                                  
void set_velocity(float* velocity);
int set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top);   
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty);

#endif