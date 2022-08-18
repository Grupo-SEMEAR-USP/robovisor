#include "../include/pwm.h"

uint32_t div_l = 0, top_l = 0;
uint32_t div_r = 0, top_r = 0;
uint slice_num_l, channel_l;
uint slice_num_r, channel_r;
uint8_t error;

float absFloat(float value)
{
    return ((value < 0) ? -1 : 1) * value;
}

float testFloat (float value)
{
    if (absFloat(value) >= MIN_PWM)
        return value;
    else
        return 0; 
}

void init_pwm_pinnage()
{
    // Left
    // --- DIR ---
    gpio_init(PICO_MOTOR_L_DIR);
    gpio_set_dir(PICO_MOTOR_L_DIR, GPIO_OUT);

    // --- BRK ---
    gpio_init(PICO_MOTOR_L_BRK);
    gpio_set_dir(PICO_MOTOR_L_BRK, GPIO_OUT);
    gpio_set_outover(PICO_MOTOR_L_BRK, GPIO_OVERRIDE_HIGH);

    // --- PWM ---
    slice_num_l = pwm_gpio_to_slice_num(PICO_MOTOR_L_PWM);
    channel_l = pwm_gpio_to_channel(PICO_MOTOR_L_PWM);
    gpio_set_function(PICO_MOTOR_L_PWM, GPIO_FUNC_PWM);
    error = set_pwm_freq(slice_num_l, (int)PWM_FREQ, &div_l, &top_l);
    pwm_set_wrap(slice_num_l, top_l);
    //printf("Erro de frequência: %d\n", error);


    // Right
    // --- DIR ---
    gpio_init(PICO_MOTOR_R_DIR);
    gpio_set_dir(PICO_MOTOR_R_DIR, GPIO_OUT);

    // --- BRK ---
    gpio_init(PICO_MOTOR_R_BRK);
    gpio_set_dir(PICO_MOTOR_R_BRK, GPIO_OUT);
    gpio_set_outover(PICO_MOTOR_R_BRK, GPIO_OVERRIDE_HIGH);

    // --- PWM ---
    slice_num_r = pwm_gpio_to_slice_num(PICO_MOTOR_R_PWM);
    channel_r = pwm_gpio_to_channel(PICO_MOTOR_R_PWM);
    gpio_set_function(PICO_MOTOR_R_PWM, GPIO_FUNC_PWM);
    error = set_pwm_freq(slice_num_r, (int)PWM_FREQ, &div_r, &top_r);
    pwm_set_wrap(slice_num_r, top_r);
    //printf("Erro de frequência: %d\n", error);
}

void set_velocity(float *pwm_velocity)
{
    // --- Left
    gpio_set_outover(PICO_MOTOR_L_DIR, ((pwm_velocity[LEFT] >= 0) ? GPIO_OVERRIDE_LOW : GPIO_OVERRIDE_HIGH));
    set_pwm_duty(slice_num_l, channel_l, top_l, (uint32_t)testFloat(pwm_velocity[LEFT]));

    // --- Right
    gpio_set_outover(PICO_MOTOR_R_DIR, ((pwm_velocity[RIGHT] >= 0) ? GPIO_OVERRIDE_LOW : GPIO_OVERRIDE_HIGH));
    set_pwm_duty(slice_num_r, channel_r, top_r, (uint32_t)testFloat(pwm_velocity[RIGHT]));
    
    return;
}

int set_pwm_freq(uint slice, int freq, uint32_t *div, uint32_t *top)
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
        return -1; // freq too large
    }
    else if (*div >= 256 * 16)
    {
        *div = 0;
        *top = 0;
        return -2; // freq too small
    }
    return 0;
}

// Code used from https://github.com/raspberrypi/micropython/blob/pico/ports/rp2/machine_pwm.c
// Function - machine_pwm_duty_u16
// Shaped for my needs (Scott Beasley) No Copyright.
// MIT License (MIT) Damien P. George Copyright (c) 2020
int set_pwm_duty(uint slice, uint channel, uint32_t top, uint32_t duty)
{
    // Set duty cycle.
    uint16_t cc = (uint16_t)duty * (top + 1) / MAX_PWM;

    if(DEBUG_PWM)
        printf("[PWM] cc = %d\n", cc);

    pwm_set_chan_level(slice, channel, cc);
    pwm_set_enabled(slice, true);

    return 0;
}
