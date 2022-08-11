#include "../include/common.h"
#include "../include/encoder.h"
#include "../include/pwm.h"

void init_pinnage()
{
    // General pinnage
    sleep_ms(INITIAL_TIMEOUT_MS);
    stdio_init_all();

    init_pwm_pinnage();
    init_encoder_pinnage();
    
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

void send_encoder_values(int *dtheta)
{
    printf("%d%d", dtheta[LEFT], dtheta[RIGHT]);
    return;
}

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

int main(void)
{
    // Set all pins to it's respective functions, including direction and other related parameters
    init_pinnage();

    int dtheta[2];
    int velocity[2];

    while(1)
    {
        // AFAIK this is intended for loop optimization 
        tight_loop_contents();

        // Read displacement output from encoders, in degress, and pass those values back to ROS
        read_encoders(dtheta);

        send_encoder_values(dtheta);

        read_velocity_commands(velocity);

        set_velocity(velocity);
    }
}
