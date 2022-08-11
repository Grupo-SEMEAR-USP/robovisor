/*
This code is to read and debounce a KY-040 rotary encoder (maybe others) using interrupts on the Raspbery Pi PICO.
This code should also work on other processors with a few changes to the hardware specific parts of the code.

Here is a crude represntation of the output of the encoder

CW rotation 
______       ______
      F1____|			Phase A
_________       ______
	F2_____|		Phase B	
Notice that Phase A falling edge (F1 in the diagram) occurs before Phase B (F2) for CW rotation



CCW Rotation
_________      ______
	F2____|			Phase A	
______       ______
      F1____|			Phase B
Notice that Phase B falling (F1 in the diagram) edge occurs before Phase A (F2)for CCW rotation

This code works by watching for the first falling edge (F1) and setting cw_fall TRUE if Phase A is the first falling edge
and setting ccw_fall TRUE is phase B is the first falling edge. After one of these (cw_fall or ccw_fall) have been set
the code watches to see which phase is next to trigger the interrupt (F2).  After the second (F2) interrupt, the code can detrmine the
direction that the encoder has been turned.

cw  A leads B
ccw B leads A
this may change depending on your wiring and what encoder is used

	  
The code does not handle the encoder push button switch.
This can be implemented by adding additional code to hangle the switch interrupt
       if (gpio == ENC_SW)
		{
			//handle switch event here
		}
	  
*/

/**BEGIN CODE HERE****************************************************************************************************/

#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "stdlib.h"

/*Encoder GPIO*/
// GPIO 10 is Encoder phase A,  
// GPIO 11 is Encoder phase B,
// GPIO 12 is the encoder push botton switch.
// change these as needed

#define ENC_A	10
#define ENC_B	11
#define ENC_SW	12


/* Encoder Callback*/
/*
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
*/
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