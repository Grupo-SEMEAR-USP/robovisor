#include "../include/pwm.h"

uint32_t div = 0, top = 0;   
uint slice_num_l, channel_l; 
uint slice_num_r, channel_r; 

void init_pwm_pinnage()
{
    gpio_init(PICO_MOTOR_L_BRK);
    gpio_set_dir(PICO_MOTOR_L_BRK, GPIO_OUT);

    gpio_init(PICO_MOTOR_R_BRK);
    gpio_set_dir(PICO_MOTOR_R_BRK, GPIO_OUT);

    gpio_set_outover(PICO_MOTOR_L_BRK, GPIO_OVERRIDE_HIGH);
    gpio_set_outover(PICO_MOTOR_R_BRK, GPIO_OVERRIDE_HIGH);

    slice_num_l = pwm_gpio_to_slice_num(PICO_MOTOR_L_PWM);
    channel_l = pwm_gpio_to_channel(PICO_MOTOR_L_PWM);
    gpio_set_function(PICO_MOTOR_L_PWM, GPIO_FUNC_PWM);

    slice_num_r = pwm_gpio_to_slice_num(PICO_MOTOR_R_PWM);
    channel_r = pwm_gpio_to_channel(PICO_MOTOR_R_PWM);
    gpio_set_function(PICO_MOTOR_R_PWM, GPIO_FUNC_PWM);

    set_pwm_freq(slice_num_l, (int)PWM_FREQ, &div, &top);

    pwm_set_wrap(slice_num_l, top);
    pwm_set_wrap(slice_num_r, top);
}

void set_velocity(float *velocity)
{
    // Verify velocity signal
    if (velocity[LEFT] >= 0)
    {
        gpio_set_outover(PICO_MOTOR_L_DIR, GPIO_OVERRIDE_LOW);
    }
    else
    {
        gpio_set_outover(PICO_MOTOR_L_DIR, GPIO_OVERRIDE_HIGH);
        velocity[LEFT] *= -1;
    }
    if (velocity[RIGHT] >= 0)
    {
        gpio_set_outover(PICO_MOTOR_R_DIR, GPIO_OVERRIDE_LOW);
    }
    else
    {
        gpio_set_outover(PICO_MOTOR_R_DIR, GPIO_OVERRIDE_HIGH);
        velocity[RIGHT] *= -1;
    }

    printf("[PWM] velocity[LEFT] = %.2f, velocity[RIGHT] = %.2f\n", velocity[LEFT], velocity[RIGHT]);
    set_pwm_duty(slice_num_l, channel_l, top, (uint32_t) velocity[LEFT]);
    set_pwm_duty(slice_num_r, channel_r, top, (uint32_t) velocity[RIGHT]);

    return;
}

bool set_pwm_freq(uint slice, int freq, uint32_t *div, uint32_t *top)
{
    // Set the frequency, making "top" as large as possible for maximum resolution.
    *div = (uint32_t)(16 * clock_get_hz(clk_sys) / (uint32_t)freq);
    *top = 1;

    for (;;)
    {
        // Try a few small prime factors to get close to the desired frequency.
        if (*div >= 16 * 5 && *div % 5 == 0 && *top * 5 <= TOP_MAX)
        {
            *div /= 5;
            *top *= 5;
        }
        else if (*div >= 16 * 3 && *div % 3 == 0 && *top * 3 <= TOP_MAX)
        {
            *div /= 3;
            *top *= 3;
        }
        else if (*div >= 16 * 2 && *top * 2 <= TOP_MAX)
        {
            *div /= 2;
            *top *= 2;
        }
        else
        {
            break;
        }
    }

    if (*div < 16)
    {
        *div = 0;
        *top = 0;
    }
    else if (*div >= 256 * 16)
    {
        *div = 0;
        *top = 0;
    }
}

// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_duty_u16
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
int set_pwm_duty(uint slice, uint channel, uint32_t top, uint32_t duty)
{
    // Set duty cycle.
    uint16_t cc = (uint16_t) duty * (top + 1) / 65535;
    printf("[PWM] cc = %d\n", cc);
    pwm_set_chan_level(slice, channel, cc);
    pwm_set_enabled(slice, true);

    return 0;
}
