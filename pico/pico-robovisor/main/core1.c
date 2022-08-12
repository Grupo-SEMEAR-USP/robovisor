#include "../include/core1.h"

void send_encoder_values(int* dtheta)
{
    //Send encoder values
    printf("%d%d", dtheta[LEFT], dtheta[RIGHT]);
}

void core1_setup()
{
    init_encoder_pinnage();
}

void core1_main()
{
    core1_setup();

    // Storages the angle displacement for send through Serial.
    int dtheta[2] = {0, 0};

    //Core 1 main loop.
    while(1)
    {
        // AFAIK this is intended for loop optimization 
        tight_loop_contents();

        // Read displacement output from encoders, in degress, and pass those values back to ROS
        read_encoders(dtheta);
        send_encoder_values(dtheta);
    }
}