#include "common.h"
#include "pico/time.h"                                                    
#include "hardware/pwm.h"                                                 
#include "hardware/clocks.h"                                              

// MIT License (MIT) Damien P. George Copyright (c) 2020 

// Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
#define TOP_MAX 65534
#define DUTY_50_PCT (TOP_MAX/2) 
                                                                          
bool set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top);   
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty); 