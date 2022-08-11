/*
    Raspberry Pi Pico PWM example

    Scott Beasley (2021) No Copyright.

    Code used by Damien P. George Copyright (c) 2020
    From the RPI Pico MicroPython project. See below for copyright info.
    https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
*/

#include "pico/stdlib.h"
#include <stdio.h>
#include "pico/time.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"

#define PICO_MOTOR_L_BRK 21
#define PICO_MOTOR_L_PWM 20

#define PICO_MOTOR_R_BRK 12
#define PICO_MOTOR_R_PWM 13

#define DUTY_50_PCT 32767
// Maximum "top" is set at 65534 to be able to achieve 100% duty with 65535.
#define TOP_MAX 65534

bool set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top);
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty);

int main (void) 
{
    uint32_t div = 0, top = 0;
    uint slice_num_l, channel_l;
    uint slice_num_r, channel_r;

    gpio_init (PICO_MOTOR_L_BRK);
    gpio_set_dir (PICO_MOTOR_L_BRK, GPIO_OUT);
    
    gpio_init (PICO_MOTOR_R_BRK);
    gpio_set_dir (PICO_MOTOR_R_BRK, GPIO_OUT);

    gpio_set_outover(PICO_MOTOR_L_BRK, GPIO_OVERRIDE_LOW);
    gpio_set_outover(PICO_MOTOR_R_BRK, GPIO_OVERRIDE_LOW);

    slice_num_l = pwm_gpio_to_slice_num (PICO_MOTOR_L_PWM);
    channel_l = pwm_gpio_to_channel (PICO_MOTOR_L_PWM);
    gpio_set_function (PICO_MOTOR_L_PWM, GPIO_FUNC_PWM);

    slice_num_r = pwm_gpio_to_slice_num (PICO_MOTOR_R_PWM);
    channel_r = pwm_gpio_to_channel (PICO_MOTOR_R_PWM);
    gpio_set_function (PICO_MOTOR_R_PWM, GPIO_FUNC_PWM);

    // Set up a 2khz freq PWM
    set_pwm_freq (slice_num_l, (int)2000, &div, &top)
    
    // Set the PWM counter wrap value to reset on
    pwm_set_wrap (slice_num_l, top);
    pwm_set_wrap (slice_num_r, top);

    while (1) {
        tight_loop_contents ( );
        set_pwm_duty (slice_num_l, channel_l, top, (uint32_t)DUTY_50_PCT);
        set_pwm_duty (slice_num_r, channel_r, top, (uint32_t)DUTY_50_PCT);
        sleep_ms(2000);
        set_pwm_duty (slice_num_l, channel_l, top, (uint32_t)DUTY_50_PCT / 2);
        set_pwm_duty (slice_num_r, channel_r, top, (uint32_t)DUTY_50_PCT / 2);
        sleep_ms(2000);
        set_pwm_duty (slice_num_l, channel_l, top, (uint32_t)DUTY_50_PCT / 4);
        set_pwm_duty (slice_num_r, channel_r, top, (uint32_t)DUTY_50_PCT / 4);
        sleep_ms(2000);
        set_pwm_duty (slice_num_l, channel_l, top, (uint32_t)DUTY_50_PCT / 6);
        set_pwm_duty (slice_num_r, channel_r, top, (uint32_t)DUTY_50_PCT / 6);
        sleep_ms(2000);
    }
}

//
// These functions work ok up to 10khz, but get off values after
// 11k and above. It could be type/casting issues, but I have not 
// investigated any further yey. I plan to test the Micropython 
// functions and see if they display the same issues.
//

/*
 * This file is part of the MicroPython project, http://micropython.org/
 *
 * The MIT License (MIT)
 *
 * Copyright (c) 2020 Damien P. George
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_freq
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
bool set_pwm_freq (uint slice, int freq, uint32_t *div, uint32_t *top) 
{
    // Set the frequency, making "top" as large as possible for maximum resolution.
    *div = (uint32_t)(16 * clock_get_hz (clk_sys) / (uint32_t)freq);
    *top = 1;
    for (;;) {
        // Try a few small prime factors to get close to the desired frequency.
        if (*div >= 16 * 5 && *div % 5 == 0 && *top * 5 <= TOP_MAX) {
            *div /= 5;
            *top *= 5;
        } else if (*div >= 16 * 3 && *div % 3 == 0 && *top * 3 <= TOP_MAX) {
            *div /= 3;
            *top *= 3;
        } else if (*div >= 16 * 2 && *top * 2 <= TOP_MAX) {
            *div /= 2;
            *top *= 2;
        } else {
            break;
        }
    }

    if (*div < 16) {
        *div = 0;
        *top = 0;
    } else if (*div >= 256 * 16) {
        *div = 0;
        *top = 0;
    }
}

// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_duty_u16
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
int set_pwm_duty (uint slice, uint channel, uint32_t top, uint32_t duty)
{
    // Set duty cycle.
    uint32_t cc = duty * (top + 1) / 65535;
    pwm_set_chan_level (slice, channel, cc);
    pwm_set_enabled (slice, true);

    return 0;
}


