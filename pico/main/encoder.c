#include "../encoder.h"

int* read_encoders()
{
	int retval_dtheta[2];

	retval_dtheta[LEFT] = (direction[LEFT] == CCW ? -1 : 1)*pulseCounter[LEFT];
	retval_dtheta[RIGHT] = (direction[RIGHT] == CCW ? 1 : -1)*pulseCounter[RIGHT]

	pulseCounter[LEFT] = 0;
	pulseCounter[RIGHT] = 0;

	return retval_dtheta;
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

		case default:
			printf("[encoder_callback] Problem reading encoders.");
			break;
    }
}