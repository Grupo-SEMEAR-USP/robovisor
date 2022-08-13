#include "../include/encoder.h"

float current_angle[2] = {0, 0};
float current_velocity[2] = {0, 0};
float last_time[2] = {0, 0};

void send_encoder_values()
{
    //Send encoder values to core0
    multicore_fifo_push_blocking((uint32_t) current_velocity[LEFT]);
	multicore_fifo_push_blocking((uint32_t) current_velocity[RIGHT]);

    //Send encoder values to Serial
    //TODO: SEND THIS SHIT TO JETSON
    //printf("%f%f", dtheta[LEFT], dtheta[RIGHT]);
    //printf("[SENDING] Valores dos encoders: %d, %d\n", dtheta[LEFT], dtheta[RIGHT]);
}

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

	last_time[LEFT] = to_ms_since_boot(get_absolute_time());
	last_time[RIGHT] = to_ms_since_boot(get_absolute_time());
}

/*void read_encoders()
{
	dpulses[LEFT] = (direction[LEFT] == CCW ? -1 : 1)*pulseCounter[LEFT];
	dpulses[RIGHT] = (direction[RIGHT] == CCW ? 1 : -1)*pulseCounter[RIGHT];

	read_current_velocity[LEFT] += ((float) dpulses[LEFT]*TICKS2DEGREES)/(read_delta_time/1000);
	read_current_velocity[RIGHT] += ((float) dpulses[RIGHT]*TICKS2DEGREES)/(read_delta_time/1000);
	//printf("[ENCODER] read_current_velocity[LEFT] = %.2f, read_current_velocity[RIGHT] = %.2f\n", read_current_velocity[LEFT], read_current_velocity[RIGHT]);
	
	//Sends encoder information to the Core 0 for PID control.
	multicore_fifo_push_blocking((uint32_t) read_current_velocity[LEFT]);
	multicore_fifo_push_blocking((uint32_t) read_current_velocity[RIGHT]);

	printf("%.8f, %.8f, %.2f, %.2f\n", read_delta_time, last_time, read_current_velocity[LEFT], read_current_velocity[RIGHT]);
	
	if(last_current_velocity[LEFT] != read_current_velocity[LEFT] || last_current_velocity[RIGHT] != read_current_velocity[RIGHT])
	{
		printf("%.8f, %.8f, %.2f, %.2f\n", read_delta_time, last_time, read_current_velocity[LEFT], read_current_velocity[RIGHT]);
	}
	
	last_current_velocity[LEFT] = read_current_velocity[LEFT];
	last_current_velocity[RIGHT] = read_current_velocity[RIGHT];

	pulseCounter[LEFT] = 0;
	pulseCounter[RIGHT] = 0;
	read_delta_time = 0;
	last_time = to_ms_since_boot(get_absolute_time());
}*/

void encoder_callback(uint gpio, uint32_t events) 
{
	//printf("[encoder_callback] Active! gpio = %d, event = %d\n", gpio, events);

	float time_now = to_ms_since_boot(get_absolute_time());
	float deltaT[2] = {(time_now - last_time[LEFT])/1000, 
					   (time_now - last_time[RIGHT])/1000};
	int increment = 0;

	switch(gpio)
    {
		case PICO_MOTOR_L_CHA:           
			//Direction check
			increment = (gpio_get(PICO_MOTOR_L_CHB) ? -1 : 1)*TICKS2DEGREES;
			current_angle[LEFT] += increment;
			current_velocity[LEFT] = increment/deltaT[LEFT];
			last_time[LEFT] = time_now;
        	break;

        case PICO_MOTOR_R_CHA:
			//Direction check
			increment = (gpio_get(PICO_MOTOR_R_CHB) ? 1 : -1)*TICKS2DEGREES;
			current_angle[RIGHT] += increment;
			current_velocity[RIGHT] = increment/deltaT[RIGHT];
			last_time[RIGHT] = time_now;
        	break;

		default:
			printf("[encoder_callback] Problem reading encoders.");
			break;
    }
}