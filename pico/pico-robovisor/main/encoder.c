#include "../include/encoder.h"

direction[2] = {0, 0};

pulseCounter[2] = {0, 0};

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
}

void read_encoders(int* dtheta)
{
	dtheta[LEFT] = (direction[LEFT] == CCW ? -1 : 1)*pulseCounter[LEFT];
	dtheta[RIGHT] = (direction[RIGHT] == CCW ? 1 : -1)*pulseCounter[RIGHT];

	pulseCounter[LEFT] = 0;
	pulseCounter[RIGHT] = 0;
}

void encoder_callback(uint gpio, uint32_t events) 
{
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