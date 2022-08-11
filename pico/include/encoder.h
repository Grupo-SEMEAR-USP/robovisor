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
#include "common.h"
#include "hardware/pio.h"
#include "hardware/gpio.h"
#include "stdlib.h"

/* Encoder Callback*/
/*
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
*/
void encoder_callback(uint gpio, uint32_t events);