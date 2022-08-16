#include "../include/common.h"
#include "../include/pwm.h"
#include "../include/core1.h"
#include "hardware/irq.h"

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

#define READ_TIMEOUT_US 5000000

// Global variable that storages what is the next motor info to be read.
int read_order = LEFT;

// Global variable that storages the current velocity of each motor.
float current_velocity[2] = {0.0, 0.0};

// Global variable that storages the target velocity.
float velocity_target[2] = {0.0, 0.0};

// Global variable that storages the PWM value to be fed to the motors.
float output_PWM[2] = {0.0, 0.0};

// TODO
// Debug - CLEAN LATER.
float last_velocity_target[2] = {0.0, 0.0};

// TODO:
// Need's further consideration of types & sizes, using int
// in this manner is bad, since it doesnt especify the size
// eg: ROS is sending unsigned 32 bit integer per motor, which
// only makes sense here if sizeof(int) == 16 bits == 2 bytes,
// which is not according to docs
// https://raspberry-projects.com/pi/programming-in-c/memory/variables
// for now let's abstract that, in the end, velocity
// (which is a signed floating type) is correctly
// especified in the end
void read_velocity_commands(float *velocity)
{
    int velocity_l[2], velocity_r[2];

    // Left
    velocity_l[0] = getchar_timeout_us(READ_TIMEOUT_US);
    velocity_l[1] = getchar_timeout_us(READ_TIMEOUT_US);

    // Right
    velocity_r[0] = getchar_timeout_us(READ_TIMEOUT_US);
    velocity_r[1] = getchar_timeout_us(READ_TIMEOUT_US);

    if (velocity_l[0] == PICO_ERROR_TIMEOUT ||
        velocity_l[1] == PICO_ERROR_TIMEOUT ||
        velocity_r[0] == PICO_ERROR_TIMEOUT ||
        velocity_r[1] == PICO_ERROR_TIMEOUT)
    {
        // If lost serial communication, let's stop running FN 
        for(int i = 0; i < 2; i++)
        {
            velocity_l[i] = 0;
            velocity_r[i] = 0;
        }

        if (DEBUG_MAIN) printf("[RECEIVING] TIMEOUT!\n");

        return;
    }

    velocity[LEFT] = (float)velocity_l[1] * 256 + velocity_l[0];
    velocity[RIGHT] = (float)velocity_r[1] * 256 + velocity_r[0];

    if (DEBUG_MAIN_RECEIVE)
    {
        if (velocity[LEFT] != last_velocity_target[LEFT] || velocity[RIGHT] != last_velocity_target[RIGHT])
        {
            last_velocity_target[LEFT] = velocity[LEFT];
            last_velocity_target[RIGHT] = velocity[RIGHT];
            printf("[RECEIVING] velocity_l[0] = %x, velocity_l[1] = %x, velocity_r[0] = %x, velocity_r[1] = %x\n", velocity_l[0], velocity_l[1], velocity_r[0], velocity_r[1]);
            printf("[RECEIVING] velocity[LEFT] = %.2f, velocity[RIGHT] = %.2f\n", velocity[LEFT], velocity[RIGHT]);
        }
    }
}

void get_current_velocity_interrupt_handle()
{
    while (multicore_fifo_rvalid())
    {
        uint32_t raw = multicore_fifo_pop_blocking();
        // printf("raw = %x\n", raw);

        switch (read_order)
        {
        case LEFT:
            // current_velocity[LEFT] = absFloat((float) raw);
            current_velocity[LEFT] = (float)raw;

            if (DEBUG_MAIN_RECEIVE)
                printf("current_velocity[LEFT] = %.2f\n", current_velocity[LEFT]);

            break;

        case RIGHT:
            // current_velocity[RIGHT] = absFloat((float) raw);
            current_velocity[RIGHT] = (float)raw;

            if (DEBUG_MAIN_RECEIVE)
                printf("current_velocity[RIGHT] = %.2f\n", current_velocity[RIGHT]);

            break;

        default:
            printf("[get_current_velocity_interrupt_handle] Erro no cálculo de velocidade atual!");
            break;
        }

        read_order = 1 - read_order;
    }

    multicore_fifo_clear_irq();
}

void setup_core0()
{
    // Init pinnage
    sleep_ms(INITIAL_TIMEOUT_MS);
    stdio_init_all();

    // As pwm is handled by core0, it's pinnage is setted here
    init_pwm_pinnage();

    // Init multicore
    multicore_launch_core1(main_core1);

    // Configure a interruption to update current velocity each time the encoders are read.
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC0, get_current_velocity_interrupt_handle);
    irq_set_enabled(SIO_IRQ_PROC0, true);

    return;
}

int main(void)
{
    // Set all pins and init multicore and PID.
    setup_core0();

    // Init PID controllers.
    struct pid_controller ctrldata_left, ctrldata_right;
    pid_cont_t pid_left, pid_right;

    double kp = 57;
    double ki = 463.4143;
    double kd = 0;

    pid_left = pid_create(&ctrldata_left, &current_velocity[LEFT], &output_PWM[LEFT], &velocity_target[LEFT], kp, ki, kd);
    pid_right = pid_create(&ctrldata_right, &current_velocity[RIGHT], &output_PWM[RIGHT], &velocity_target[RIGHT], kp, ki, kd);

    pid_limits(pid_left, -MAX_PWM, MAX_PWM);
    pid_limits(pid_right, -MAX_PWM, MAX_PWM);

    pid_auto(pid_left);
    pid_auto(pid_right);

    /*float delta_time_left = 0;
    float delta_time_right = 0;
    float last_time_left = to_ms_since_boot(get_absolute_time());
    float last_time_right = to_ms_since_boot(get_absolute_time());*/

    // Core 0 main loop.
    while (1)
    {
        // AFAIK this is intended for SMALL loop optimization, not sure if useful here
        // tight_loop_contents();

        // Read velocity from Serial
        read_velocity_commands(velocity_target);

        if (pid_need_compute(pid_left))
        {
            // delta_time_left = to_ms_since_boot(get_absolute_time()) - last_time_left;
            pid_compute(pid_left);
            // last_time_left = to_ms_since_boot(get_absolute_time());
        }

        if (pid_need_compute(pid_right))
        {
            // delta_time_right = to_ms_since_boot(get_absolute_time()) - last_time_right;
            pid_compute(pid_right);
            // last_time_right = to_ms_since_boot(get_absolute_time());
        }

        if (DEBUG_MAIN)
            // printf("left frequency = %.2f, right frequency = %.2f\n", delta_time_left, delta_time_right);
            printf("%d, %.2f, %.2f, %.2f\n", to_ms_since_boot(get_absolute_time()), current_velocity[LEFT], output_PWM[LEFT], velocity_target[LEFT]);

        // Send velocity target to motors.
        set_velocity(output_PWM);
    }
}
