#ifndef __ENCODER__
#define __ENCODER__

#include "pico/stdlib.h"
#include "hardware/irq.h"
#include "common.h"

//Motors direction
int direction[2];

//Encoder reading pulse count.
int pulseCounter[2];

/*
Init all encoder relevant pins.
*/
void init_encoder_pinnage();

/* 
Get encoder values.
Positive values means the robot is driving fowards;
Negative values means the robot is driving backwards.
*/
void read_encoders(int* dtheta);

/* Encoder Callback*/
/*
        "LEVEL_LOW",  // 0x1
        "LEVEL_HIGH", // 0x2
        "EDGE_FALL",  // 0x4
        "EDGE_RISE"   // 0x8
*/
void encoder_callback(uint gpio, uint32_t events);

#endif