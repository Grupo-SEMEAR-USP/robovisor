#include "../encoder.h"

void encoder_callback(uint gpio, uint32_t events) 
{
	
	uint32_t gpio_state = 0;

	gpio_state = (gpio_get_all() >> 10) & 0b0111;  	// get all GPIO them mask out all but bits 10, 11, 12
													// This will need to change to match which GPIO pins are being used.

	
	static bool ccw_fall = 0;  //bool used when falling edge is triggered
	static bool cw_fall = 0;
	
	uint8_t enc_value = 0;
	enc_value = (gpio_state & 0x03);

	
	if (gpio == ENC_A) 
	{
		if ((!cw_fall) && (enc_value == 0b10)) // cw_fall is set to TRUE when phase A interrupt is triggered
			cw_fall = 1; 

		if ((ccw_fall) && (enc_value == 0b00)) // if ccw_fall is already set to true from a previous B phase trigger, the ccw event will be triggered 
		{
			cw_fall = 0;
			ccw_fall = 0;
			//do something here,  for now it is just printing out CW or CCW
			printf("CCW \r\n");
		}

	}	


	if (gpio == ENC_B) 
	{
		if ((!ccw_fall) && (enc_value == 0b01)) //ccw leading edge is true
			ccw_fall = 1;

		if ((cw_fall) && (enc_value == 0b00)) //cw trigger
		{
			cw_fall = 0;
			ccw_fall = 0;
			//do something here,  for now it is just printing out CW or CCW
			printf("CW \r\n");
		}

	}
	
}

//*************************************************************************************************************************************************
//*************************************************************************************************************************************************

/* Example of use
int main()
{
    sleep_ms(500);
    stdio_init_all();

	// GPIO Setup for Encoder
	gpio_init(ENC_SW);					//Initialise a GPIO for (enabled I/O and set func to GPIO_FUNC_SIO)
    gpio_set_dir(ENC_SW,GPIO_IN);
	gpio_disable_pulls(ENC_SW);

	gpio_init(ENC_A);
    gpio_set_dir(ENC_A,GPIO_IN);
	gpio_disable_pulls(ENC_A);

	gpio_init(ENC_B);
    gpio_set_dir(ENC_B,GPIO_IN);
	gpio_disable_pulls(ENC_B);

	gpio_set_irq_enabled_with_callback(ENC_SW, GPIO_IRQ_EDGE_FALL, true, &encoder_callback);
    gpio_set_irq_enabled(ENC_A, GPIO_IRQ_EDGE_FALL, true);
	gpio_set_irq_enabled(ENC_B, GPIO_IRQ_EDGE_FALL, true);

	while (1) 
	{
	// do something here forever
	sleep_ms(1000);
	}
   
}
*/