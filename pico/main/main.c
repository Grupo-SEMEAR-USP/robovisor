#include "../include/common.h"
#include "../include/encoder.h"
#include "../include/pwm.h"

void init_pinnage()
{

    // General pinnage
    sleep_ms(INITIAL_TIMEOUT_MS);
    stdio_init_all();

    // Init pwm pinnage

    gpio_init(PICO_MOTOR_L_BRK);
    gpio_set_dir(PICO_MOTOR_L_BRK, GPIO_OUT);

    gpio_init(PICO_MOTOR_R_BRK);
    gpio_set_dir(PICO_MOTOR_R_BRK, GPIO_OUT);

    gpio_set_outover(PICO_MOTOR_L_BRK, GPIO_OVERRIDE_LOW);
    gpio_set_outover(PICO_MOTOR_R_BRK, GPIO_OVERRIDE_LOW);

    slice_num_l = pwm_gpio_to_slice_num(PICO_MOTOR_L_PWM);
    channel_l = pwm_gpio_to_channel(PICO_MOTOR_L_PWM);
    gpio_set_function(PICO_MOTOR_L_PWM, GPIO_FUNC_PWM);

    slice_num_r = pwm_gpio_to_slice_num(PICO_MOTOR_R_PWM);
    channel_r = pwm_gpio_to_channel(PICO_MOTOR_R_PWM);
    gpio_set_function(PICO_MOTOR_R_PWM, GPIO_FUNC_PWM);

    set_pwm_freq(slice_num_l, (int)PWM_FREQ, &div, &top);

    pwm_set_wrap(slice_num_l, top);
    pwm_set_wrap(slice_num_r, top);

    // Init encoder pinnage
    gpio_init(ENC_A);
    gpio_set_dir(ENC_A, GPIO_IN);
    gpio_disable_pulls(ENC_A);

    gpio_init(ENC_B);
    gpio_set_dir(ENC_B, GPIO_IN);
    gpio_disable_pulls(ENC_B);

    gpio_set_irq_enabled_with_callback(ENC_SW, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled(ENC_A, GPIO_IRQ_EDGE_FALL, true);
    gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_FALL, true);

    return;
}

/**
 * @brief main functionality of the low level control features of application
 *
 * TODO:
 * PID logic control
 * See any adversities that can arise from uint_32 implicit conversion
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

void send_encoder_values(uint8_t *dtheta)
{
    printf("%u%u", dtheta[LEFT], dtheta[RIGHT]);
    return;
}

void read_velocity_commands(int* velocity){

    uint8_t velocity_l[2], velocity_r[2];

    // Left 
    velocity_l[0] = getchar();
    velocity_l[1] = getchar();
    
    // Right 
    velocity_r[0] = getchar();
    velocity_r[1] = getchar();

    // Possibly problematic
    velocity[LEFT] = (int) ((velocity_l[1] << 8) | velocity_l[0]);
    velocity[RIGHT] = (int) ((velocity_r[1] << 8) | velocity_r[0]);

    return velocity;
}

int main(void)
{

    // Set all pins to it's respective functions, including direction and other related parameters
    init_pinnage();

    uint8_t dtheta[2];
    int velocity[2];

    while (true)
    {

        // AFAIK this is intended for loop optimization 
        tight_loop_contents();

        // Read displacement output from encoders, in degress, and pass those values back to ROS
        dtheta = read_encoders();

        send_encoder_values(dtheta);

        velocity = read_velocity_commands(&velocity);

        set_velocity(velocity);
    }
}
