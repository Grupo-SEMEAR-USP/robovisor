#include "../include/core1.h"

void send_encoder_values(int* dtheta)
{
    //Send encoder values
    //printf("%d%d", dtheta[LEFT], dtheta[RIGHT]);
    //printf("[SENDING] Valores dos encoders: %d, %d\n", dtheta[LEFT], dtheta[RIGHT]);
}

void setup_core1()
{
    // As encoder is handled by core1, it's pinnage is setted here
    init_encoder_pinnage();
}   

void main_core1()
{
    setup_core1();

    // Stores the angle displacement sent through serial port.
    int dtheta[2] = {0, 0};

    //Core 1 main loop.
    while(1)
    {
        // AFAIK this is intended for small loop optimization 
        tight_loop_contents();

        // Read displacement output from encoders, in degress, and pass those values back to ROS
        read_encoders(dtheta);
        send_encoder_values(dtheta);

        sleep_ms(20);
    }
}