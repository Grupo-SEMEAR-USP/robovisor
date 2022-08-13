#include "../include/encoder.h"

/*
 *	Global variables
 *	direction storages the motor movement direction.
 *  pulseCounter storages the angle displacement since last serial connection.
 *  read_current_velocity storages the current motor velocity.
 *  last_time is the last absolute time registered at encoder interruption.
 *  read_delta_time storages the time elapsed between encoder readings.
 */
int direction[2] = {0, 0};
int pulseCounter[2] = {0, 0};
float read_current_velocity[2] = {0, 0};
float last_time = 0;
float read_delta_time = 0;

void init_encoder_pinnage()
{
    gpio_init(PICO_MOTOR_R_CHA);
    gpio_set_dir(PICO_MOTOR_R_CHA, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHA, GPIO_IRQ_EDGE_FALL, 1, encoder_callback);

    gpio_init(PICO_MOTOR_R_CHB);
    gpio_set_dir(PICO_MOTOR_R_CHB, GPIO_IN);

    gpio_init(PICO_MOTOR_L_CHA);
    gpio_set_dir(PICO_MOTOR_L_CHA, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHA, GPIO_IRQ_EDGE_FALL, 1, encoder_callback);

    gpio_init(PICO_MOTOR_L_CHB);
    gpio_set_dir(PICO_MOTOR_L_CHB, GPIO_IN);

	last_time = to_ms_since_boot(get_absolute_time());
}

void read_encoders(int* dtheta)
{
	read_delta_time = (to_ms_since_boot(get_absolute_time()) - last_time)/1000;
	last_time = to_ms_since_boot(get_absolute_time());

	dtheta[LEFT] = (direction[LEFT] == CCW ? -1 : 1)*pulseCounter[LEFT];
	dtheta[RIGHT] = (direction[RIGHT] == CCW ? 1 : -1)*pulseCounter[RIGHT];

	//printf("[ENCODER] dtheta[LEFT] = %d, dtheta[RIGHT] = %d\n", dtheta[LEFT], dtheta[RIGHT]);

	read_current_velocity[LEFT] = ((float) dtheta[LEFT]*TICKS2DEGREES)/read_delta_time;
	read_current_velocity[RIGHT] = ((float) dtheta[RIGHT]*TICKS2DEGREES)/read_delta_time;

	//printf("[ENCODER] read_current_velocity[LEFT] = %.2f, read_current_velocity[RIGHT] = %.2f\n", read_current_velocity[LEFT], read_current_velocity[RIGHT]);

	//Sends encoder information to the Core 0 for PID control.
	multicore_fifo_push_blocking((uint32_t) read_current_velocity[LEFT]);
	multicore_fifo_push_blocking((uint32_t) read_current_velocity[RIGHT]);

	pulseCounter[LEFT] = 0;
	pulseCounter[RIGHT] = 0;
}

void encoder_callback(uint gpio, uint32_t events) 
{
	//printf("[encoder_callback] Active! gpio = %d, event = %d\n", gpio, events);

	switch(gpio)
    {
		case PICO_MOTOR_L_CHA:
            pulseCounter[LEFT] += 1;
            
			//Direction check
			if(gpio_get(PICO_MOTOR_L_CHB))
				direction[LEFT] = CCW;
			else
				direction[LEFT] = CW;
        	break;

        case PICO_MOTOR_R_CHA:
			pulseCounter[RIGHT] += 1;

			//Direction check
            if (gpio_get(PICO_MOTOR_R_CHB))
				direction[RIGHT] = CCW;
			else
				direction[RIGHT] = CW;
        	break;

		default:
			printf("[encoder_callback] Problem reading encoders.");
			break;
    }
}