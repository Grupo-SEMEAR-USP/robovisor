#include "../include/encoder.h"
#include "pico/float.h"

float current_velocity_[2] = {0.0, 0.0};
float current_angle[2] = {0.0, 0.0};
float last_sent_angle[2] = {0.0, 0.0};
float last_time[2] = {0.0, 0.0};

bool status_CHA_L;
bool status_CHA_R;
bool status_CHB_L;
bool status_CHB_R;

uint32_t time_now;
uint32_t deltaT[2];
int8_t increment;

void send_char_via_serial(char c)
{
	if(c < 16)
		printf("%d", 0);
	
	if(c == 0)
		printf("%d", 0);
	else
		printf("%x", c);
}

void send_ROS(float *dtheta)
{
	uint8_t *p = (uint8_t *) dtheta;
	
	printf("%c", 'g');
	send_char_via_serial(p[3]);
	send_char_via_serial(p[2]);
	send_char_via_serial(p[1]);
	send_char_via_serial(p[0]);
	
	printf("%c", 'h');
	send_char_via_serial(p[7]);
	send_char_via_serial(p[6]);
	send_char_via_serial(p[5]);
	send_char_via_serial(p[4]);

	if(DEBUG_SEND_ROS)
	{
		printf("[send_ROS] Valores dos encoders: %f, %f\n", dtheta[LEFT], dtheta[RIGHT]);
	}
}

void send_core0()
{
	uint32_t left_motor_data, right_motor_data;
	memcpy(&left_motor_data, &current_velocity_[LEFT], 4);
	memcpy(&right_motor_data, &current_velocity_[RIGHT], 4);

	//printf("left_motor_data = %x\n", left_motor_data);
	//printf("right_motor_data = %x\n", right_motor_data);

	// Send encoder values to core0
	multicore_fifo_push_blocking(left_motor_data);
	multicore_fifo_push_blocking(right_motor_data);
}

void send_encoder_values()
{
	//Send velocity to core0.
	send_core0();

	//Calculates the movement until last ROS iteration.
	float dtheta[2] = {
		(current_angle[LEFT] - last_sent_angle[LEFT]),
		(current_angle[RIGHT] - last_sent_angle[RIGHT])
	};

	//Send angle shift to ROS.
	send_ROS(dtheta);

	last_sent_angle[LEFT] = current_angle[LEFT];
	last_sent_angle[RIGHT] = current_angle[RIGHT];
}

void init_encoder_pinnage()
{
	// --- Left 
	// Channel A
	gpio_init(PICO_MOTOR_L_CHA);
	gpio_set_dir(PICO_MOTOR_L_CHA, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

	// Channel B
	gpio_init(PICO_MOTOR_L_CHB);
	gpio_set_dir(PICO_MOTOR_L_CHB, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHB, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

	// --- Right 
	// Channel A
	gpio_init(PICO_MOTOR_R_CHA);
	gpio_set_dir(PICO_MOTOR_R_CHA, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

	// Channel B
	gpio_init(PICO_MOTOR_R_CHB);
	gpio_set_dir(PICO_MOTOR_R_CHB, GPIO_IN);
	gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHB, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

	last_time[LEFT] = to_ms_since_boot(get_absolute_time());
	last_time[RIGHT] = to_ms_since_boot(get_absolute_time());

	status_CHB_L = gpio_get(PICO_MOTOR_L_CHB);
	status_CHB_R = gpio_get(PICO_MOTOR_R_CHB);
}

void get_encoder_processed_values()
{
	if(increment)
	{
		float angle_increment = increment*TICKS2DEGREES;

		if(last_time[LEFT] > last_time[RIGHT])
		{
			//Motor esquerdo foi o último a ser acionado.
			current_angle[LEFT] += angle_increment;
			current_velocity_[LEFT] = angle_increment/(deltaT[LEFT]/(float) 1000);
		}
		else
		{
			//Motor direito foi o último a ser acionado.
			current_angle[RIGHT] += angle_increment;
			current_velocity_[RIGHT] = angle_increment/(deltaT[RIGHT]/(float) 1000);
		}

		if(DEBUG_ENCODER_PROCESS)
		{
			printf("[get_encoder_processed_values] increment = %d, angle_increment = %.2f\n",
				   increment,
				   angle_increment);
			printf("[get_encoder_processed_values]  current_angle[LEFT] = %.2f,  current_velocity_[LEFT] = %.2f\n",
				   current_angle[LEFT],
				   current_velocity_[LEFT]);
			printf("[get_encoder_processed_values] current_angle[RIGHT] = %.2f, current_velocity_[RIGHT] = %.2f\n",
				   current_angle[RIGHT],
				   current_velocity_[RIGHT]);
		}

		increment = 0;
	}

	//Checks if the last info from encoders was more than VELOCITY_MOTORS_TIMOUT (in ms).
	for(int i = 0; i < 2; i++)
	{
		float time_now = to_ms_since_boot(get_absolute_time());
		if(time_now - last_time[i] > VELOCITY_MOTORS_TIMEOUT)
			current_velocity_[i] = 0;
	}
}

void encoder_callback(uint gpio, uint32_t events)
{
	time_now = to_ms_since_boot(get_absolute_time());

	switch(gpio)
    {
		case PICO_MOTOR_L_CHA:
			deltaT[LEFT] = time_now - last_time[LEFT];
			last_time[LEFT] = time_now;
			switch(events)
			{
				case GPIO_IRQ_EDGE_FALL:
					status_CHA_L = false;
					increment += (status_CHB_L ? 1 : -1);
					break;

				case GPIO_IRQ_EDGE_RISE:
					status_CHA_L = true;
					increment += (status_CHB_L ? -1 : 1);
					break;
				
				default:
					printf("[encoder_callback] Problem reading encoders.\n");
					break; 
			}         
        	break;

		case PICO_MOTOR_L_CHB:
			deltaT[LEFT] = time_now - last_time[LEFT];
			last_time[LEFT] = time_now;
			switch (events)
			{
				case GPIO_IRQ_EDGE_FALL:
					status_CHB_L = false;
					increment += (status_CHA_L ? -1 : 1);
					break;

				case GPIO_IRQ_EDGE_RISE:
					status_CHB_L = true;
					increment += (status_CHA_L ? 1 : -1);
					break;
				
				default:
					printf("[encoder_callback] Problem reading encoders.\n");
					break;
			}
			break;

        case PICO_MOTOR_R_CHA:
			deltaT[RIGHT] = time_now - last_time[RIGHT];
			last_time[RIGHT] = time_now;
			switch (events)
			{
				case GPIO_IRQ_EDGE_FALL:
					status_CHA_R = false;
					increment += (status_CHB_R ? -1 : 1);
					break;

				case GPIO_IRQ_EDGE_RISE:
					status_CHA_R = true;
					increment += (status_CHB_R ? 1 : -1);
					break;
				
				default:
					printf("[encoder_callback] Problem reading encoders.\n");
					break;
			}
        	break;

		case PICO_MOTOR_R_CHB:
			deltaT[RIGHT] = time_now - last_time[RIGHT];
			last_time[RIGHT] = time_now;
			switch (events)
			{
				case GPIO_IRQ_EDGE_FALL:
					status_CHB_R = false;
					increment += (status_CHA_R ? 1 : -1);
					break;

				case GPIO_IRQ_EDGE_RISE:
					status_CHB_R = true;
					increment += (status_CHA_R ? -1 : 1);
					break;
				
				default:
					printf("[encoder_callback] Problem reading encoders.\n");
					break;
			}
			break;

		default:
			printf("[encoder_callback] Problem reading encoders.\n");
			break;
	}

	if(DEBUG_ENCODER_CALLBACK)
	{
		printf("[encoder_callback] gpio = %d, events = %d\n", gpio, events);
		printf("[encoder_callback] increment = %d\n", increment);
	}
}