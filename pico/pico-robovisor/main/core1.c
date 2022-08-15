#include "../include/core1.h"

void setup_core1()
{
    // As encoder is handled by core1, it's pinnage is setted here
    init_encoder_pinnage();
}   

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
        sleep_ms(1);
    }
}