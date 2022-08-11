#include "../include/common.h"
#include "../include/encoder.h"
#include "../include/pwm.h"

/**
 * @brief main functionality of the low level control features of application
 *
 * TODO: 
 * PID logic control
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

int main(void)
{

    init_pinnage();

}
