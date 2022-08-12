#include "../include/common.h"
#include "../include/pwm.h"
#include "../include/core1.h"

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

void read_velocity_commands(int* velocity)
{
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
}

void setup_core0()
{
    //Init pinnage
    sleep_ms(INITIAL_TIMEOUT_MS);
    stdio_init_all();

    init_pwm_pinnage();

    //Init multicore
    multicore_launch_core1(core1_main);
    
    return;
}

int main(void)
{
    // Set all pins and init multicore.
    setup_core0();

    // Velocity target to be fed into PID.
    int velocity[2];

    //Core 0 main loop.
    while(1)
    {
        // AFAIK this is intended for loop optimization 
        tight_loop_contents();

        // Read velocity from Serial
        read_velocity_commands(velocity);

        // Send velocity target to motors.
        set_velocity(velocity);
    }
}
