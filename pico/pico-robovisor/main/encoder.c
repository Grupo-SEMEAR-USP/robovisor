#include "../include/encoder.h"

float current_velocity_[2] = {0.0, 0.0};
float current_angle[2] = {0.0, 0.0};
float last_sent_angle[2] = {0.0, 0.0};
float last_time[2] = {0.0, 0.0};

void send_encoder_values()
{
	// Send encoder values to core0
	multicore_fifo_push_blocking((uint32_t)current_velocity_[LEFT]);
	multicore_fifo_push_blocking((uint32_t)current_velocity_[RIGHT]);

	// Send encoder values to Serial
	// TODO: SEND THIS TO JETSON
	// ROS requires, by the current logic, to be sent
	// the angle increment, in degrees, since last request

	/* Pseudocode

		int32_t dtheta_l = current_angle[LEFT] - last_sent_angle[LEFT] ;
		int32_t dtheta_r = current_angle[RIGHT] - last_sent_angle[RIGHT] ;

		send_ROS(dtheta_l, dtheta_r);

		last_sent_angle[LEFT] = current_angle[LEFT];
		last_sent_angle[RIGHT] = current_angle[RIGHT];

	*/

	// printf("%f%f", dtheta[LEFT], dtheta[RIGHT]);
	// printf("[SENDING] Valores dos encoders: %d, %d\n", dtheta[LEFT], dtheta[RIGHT]);
}

void init_encoder_pinnage()
{
	// --- Left 
	// Channel A
	gpio_init(PICO_MOTOR_L_CHA);
	gpio_set_dir(PICO_MOTOR_L_CHA, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHA, GPIO_IRQ_EDGE_FALL, 1, encoder_callback);

	// Channel B
	gpio_init(PICO_MOTOR_L_CHB);
	gpio_set_dir(PICO_MOTOR_L_CHB, GPIO_IN);


	// --- Right 
	// Channel A
	gpio_init(PICO_MOTOR_R_CHA);
	gpio_set_dir(PICO_MOTOR_R_CHA, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHA, GPIO_IRQ_EDGE_FALL, 1, encoder_callback);

	// Channel B
	gpio_init(PICO_MOTOR_R_CHB);
	gpio_set_dir(PICO_MOTOR_R_CHB, GPIO_IN);

	last_time[LEFT] = to_ms_since_boot(get_absolute_time());
	last_time[RIGHT] = to_ms_since_boot(get_absolute_time());
}

void encoder_callback(uint gpio, uint32_t events)
{
	float time_now = to_ms_since_boot(get_absolute_time());
	float deltaT[2] = {(time_now - last_time[LEFT]) / 1000,
					   (time_now - last_time[RIGHT]) / 1000};
	float angle_increment = 0;

	switch (gpio)
	{
		case PICO_MOTOR_L_CHA:
			// Direction check
			angle_increment = (gpio_get(PICO_MOTOR_L_CHB) ? -1 : 1) * TICKS2DEGREES;
			current_angle[LEFT] += angle_increment;
			current_velocity_[LEFT] = angle_increment / deltaT[LEFT];
			last_time[LEFT] = time_now;
			break;

		case PICO_MOTOR_R_CHA:
			// Direction check
			angle_increment = (gpio_get(PICO_MOTOR_R_CHB) ? 1 : -1) * TICKS2DEGREES;
			current_angle[RIGHT] += angle_increment;
			current_velocity_[RIGHT] = angle_increment / deltaT[RIGHT];
			last_time[RIGHT] = time_now;
			break;

		default:
			printf("[encoder_callback] Problem reading encoders.");
			break;
	}

	if(DEBUG_ENCODER)
	{
		printf("angle_increment = %.2f\n", angle_increment);
		printf("[encoder_callback] deltaT[LEFT] = %.2f, current_angle[LEFT] = %.2f, current_velocity_[LEFT] = %.2f, last_time[LEFT] = %.2f\n", deltaT[LEFT], current_angle[LEFT], current_velocity_[LEFT], last_time[LEFT]);
		printf("[encoder_callback] deltaT[RIGHT] = %.2f, current_angle[RIGHT] = %.2f, current_velocity_[RIGHT] = %.2f, last_time[RIGHT] = %.2f\n", deltaT[RIGHT], current_angle[RIGHT], current_velocity_[RIGHT], last_time[RIGHT]);
	}
}