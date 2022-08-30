#include "../include/core1.h"

// Core1 is responsible for sending the velocity to the motors and reading the encoder values
// the encoder values are sent to core0 for the calculation of the movement via PID controller

// Initiates the pins for this core
void setup_core1()
{
    // As encoder is handled by core1, it's pinnage is setted here
    init_encoder_pinnage();
}   

// Main function for core1.
void main_core1()
{
    setup_core1();

    //Core 1 main loop.
    while(1)
    {
        // AFAIK this is intended for small loop optimization 
        tight_loop_contents();

        // Read displacement output from encoders, in degress, and pass those values back to ROS
        get_encoder_processed_values();
        send_encoder_values();
     
        // Periodically run this loop withing 1 ms, which implies in a sampling frequency
        // of about 1kHz (cannot say that's exactly 1kHz due to the fact that this is actually the 
        // sampling frequency of the window for which we detect the pulses for the encoder, and send
        // this to the ROS node).  
	    sleep_us(1000);
    }
}
