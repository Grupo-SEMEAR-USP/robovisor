#include "../include/encoder.h"
#include "pico/float.h"

float current_velocity_[2] = {0.0, 0.0};
float left_last_velocity[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float right_last_velocity[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
float current_angle[2] = {0.0, 0.0};
float last_sent_angle[2] = {0.0, 0.0};
uint64_t last_time[2] = {0, 0};

bool status_CHA_L;
bool status_CHA_R;
bool status_CHB_L;
bool status_CHB_R;

uint64_t time_now;
uint64_t deltaT[2] = {0, 0};
int16_t increment[2] = {0, 0};

int8_t left_vector_average_index = 0;
int8_t right_vector_average_index = 0;

void send_char_via_serial(char c)
{
    if (c < 16)
        printf("%d", 0);

    if (c == 0)
        printf("%d", 0);
    else
        printf("%x", c);
}

void send_ROS(float *dtheta)
{
    uint8_t *p = (uint8_t *)dtheta;

    printf("%c", 'g');
    send_char_via_serial(p[3]);
    send_char_via_serial(p[2]);
    send_char_via_serial(p[1]);
    send_char_via_serial(p[0]);

    printf("%c", 'h');
    send_char_via_serial(p[7]);
    send_char_via_serial(p[6]);
    send_char_via_serial(p[5]);
    send_char_via_serial(p[4]);

    if (DEBUG_SEND_ROS)
    {
        printf("[send_ROS] Valores dos encoders: %f, %f\n", dtheta[LEFT], dtheta[RIGHT]);
    }
}

void send_core0()
{
    uint32_t left_motor_data, right_motor_data;
    memcpy(&left_motor_data, &current_velocity_[LEFT], 4);
    memcpy(&right_motor_data, &current_velocity_[RIGHT], 4);

    // printf("left_motor_data = %x\n", left_motor_data);
    // printf("right_motor_data = %x\n", right_motor_data);

    // Send encoder values to core0
    multicore_fifo_push_blocking(left_motor_data);
    multicore_fifo_push_blocking(right_motor_data);
}

void send_encoder_values()
{
    // Send velocity to core0.
    send_core0();

    // Calculates the movement until last ROS iteration.
    float dtheta[2] = {
        (current_angle[LEFT] - last_sent_angle[LEFT]),
        (current_angle[RIGHT] - last_sent_angle[RIGHT])};

    // Send angle shift to ROS.
    //send_ROS(dtheta);

    last_sent_angle[LEFT] = current_angle[LEFT];
    last_sent_angle[RIGHT] = current_angle[RIGHT];
}

void init_encoder_pinnage()
{
    // --- Left
    // Channel A
    gpio_init(PICO_MOTOR_L_CHA);
    gpio_set_dir(PICO_MOTOR_L_CHA, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

    // Channel B
    gpio_init(PICO_MOTOR_L_CHB);
    gpio_set_dir(PICO_MOTOR_L_CHB, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_L_CHB, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

    // --- Right
    // Channel A
    gpio_init(PICO_MOTOR_R_CHA);
    gpio_set_dir(PICO_MOTOR_R_CHA, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHA, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

    // Channel B
    gpio_init(PICO_MOTOR_R_CHB);
    gpio_set_dir(PICO_MOTOR_R_CHB, GPIO_IN);
    gpio_set_irq_enabled_with_callback(PICO_MOTOR_R_CHB, GPIO_IRQ_EDGE_FALL | GPIO_IRQ_EDGE_RISE, 1, encoder_callback);

    last_time[LEFT] = to_us_since_boot(get_absolute_time());
    last_time[RIGHT] = to_us_since_boot(get_absolute_time());
}

float calculate_mean_velocity(float *velocities)
{
    float mean_velocity = 0.0;

    for (int i = 0; i < 5; i++)
    {
        mean_velocity += velocities[i];
    }
    mean_velocity /= 5.0;

    if (DEBUG_MEAN_VELOCITY)
    {
        printf("[calculate_mean_velocity] Mean velocity: %f\n", mean_velocity);
    }

    return mean_velocity;
}

void get_encoder_processed_values()
{
    if (increment[LEFT] != 0 && deltaT[LEFT] != 0)
    {
        float angle_increment_left = increment[LEFT] * TICKS2DEGREES;
        current_angle[LEFT] += angle_increment_left;

        left_last_velocity[left_vector_average_index] = angle_increment_left / (deltaT[LEFT] / (double)1000000);
        left_vector_average_index++;
        left_vector_average_index %= 5;

        current_velocity_[LEFT] = calculate_mean_velocity(left_last_velocity);

        //printf("[get_encoder_processed_values] angle_increment_left = %f\n", angle_increment_left);
        //printf("[get_encoder_processed_values] deltaT[LEFT] = %d\n", deltaT[LEFT]);

        increment[LEFT] = 0;
        deltaT[LEFT] = 0;
    }

    if (increment[RIGHT] != 0 && deltaT[RIGHT] != 0)
    {
        float angle_increment_right = increment[RIGHT] * TICKS2DEGREES;
        current_angle[RIGHT] += angle_increment_right;

        right_last_velocity[right_vector_average_index] = angle_increment_right / (deltaT[RIGHT] / (double)1000000);
        right_vector_average_index++;
        right_vector_average_index %= 5;

        current_velocity_[RIGHT] = calculate_mean_velocity(right_last_velocity);

        //printf("[get_encoder_processed_values] angle_increment_right = %f\n", angle_increment_right);
        //printf("[get_encoder_processed_values] deltaT[RIGHT] = %d\n", deltaT[RIGHT]);

        increment[RIGHT] = 0;
        deltaT[RIGHT] = 0;
   }
   
   if (DEBUG_ENCODER_PROCESS)
   {
       printf("[get_encoder_processed_values]  current_angle[LEFT] = %.2f,  current_velocity_[LEFT] = %.2f\n",
              current_angle[LEFT],
              current_velocity_[LEFT]);
       printf("[get_encoder_processed_values] current_angle[RIGHT] = %.2f, current_velocity_[RIGHT] = %.2f\n",
              current_angle[RIGHT],
              current_velocity_[RIGHT]);
   }
   
   // Checks if the last info from encoders was more than VELOCITY_MOTORS_TIMOUT (in us).
   for (int i = 0; i < 2; i++)
   {
       uint64_t time_now = to_us_since_boot(get_absolute_time());
       if (time_now - last_time[i] > VELOCITY_MOTORS_TIMEOUT)
           current_velocity_[i] = 0;
   }
}

void encoder_callback(uint gpio, uint32_t events)
{
    time_now = to_us_since_boot(get_absolute_time());

    switch (gpio)
    {
    case PICO_MOTOR_L_CHA:
        deltaT[LEFT] += time_now - last_time[LEFT];
        last_time[LEFT] = time_now;
        switch (events)
        {
        case GPIO_IRQ_EDGE_FALL:
            status_CHA_L = false;
            increment[LEFT] += (status_CHB_L ? 1 : -1);
            break;

        case GPIO_IRQ_EDGE_RISE:
            status_CHA_L = true;
            increment[LEFT] += (status_CHB_L ? -1 : 1);
            break;

        default:
            printf("[encoder_callback] Problem reading encoders.\n");
            break;
        }
        break;

    case PICO_MOTOR_L_CHB:
        deltaT[LEFT] += time_now - last_time[LEFT];
        last_time[LEFT] = time_now;
        switch (events)
        {
        case GPIO_IRQ_EDGE_FALL:
            status_CHB_L = false;
            increment[LEFT] += (status_CHA_L ? -1 : 1);
            break;

        case GPIO_IRQ_EDGE_RISE:
            status_CHB_L = true;
            increment[LEFT] += (status_CHA_L ? 1 : -1);
            break;

        default:
            printf("[encoder_callback] Problem reading encoders.\n");
            break;
        }
        break;

    case PICO_MOTOR_R_CHA:
        deltaT[RIGHT] += time_now - last_time[RIGHT];
        last_time[RIGHT] = time_now;
        switch (events)
        {
        case GPIO_IRQ_EDGE_FALL:
            status_CHA_R = false;
            increment[RIGHT] += (status_CHB_R ? -1 : 1);
            break;

        case GPIO_IRQ_EDGE_RISE:
            status_CHA_R = true;
            increment[RIGHT] += (status_CHB_R ? 1 : -1);
            break;

        default:
            printf("[encoder_callback] Problem reading encoders.\n");
            break;
        }
        break;

    case PICO_MOTOR_R_CHB:
        deltaT[RIGHT] += time_now - last_time[RIGHT];
        last_time[RIGHT] = time_now;
        switch (events)
        {
        case GPIO_IRQ_EDGE_FALL:
            status_CHB_R = false;
            increment[RIGHT] += (status_CHA_R ? 1 : -1);
            break;

        case GPIO_IRQ_EDGE_RISE:
            status_CHB_R = true;
            increment[RIGHT] += (status_CHA_R ? -1 : 1);
            break;

        default:
            printf("[encoder_callback] Problem reading encoders.\n");
            break;
        }
        break;

    default:
        printf("[encoder_callback] Problem reading encoders.\n");
        break;
    }

    if (DEBUG_ENCODER_CALLBACK)
    {
        printf("[encoder_callback] gpio = %d, events = %d\n", gpio, events);
        printf("[encoder_callback] increment[LEFT] = %d\n", increment[LEFT]);
        printf("[encoder_callback] increment[RIGHT] = %d\n", increment[RIGHT]);
        printf("[encoder_callback] time_now = %llu\n", time_now);
        printf("[encoder_callback] last_time[LEFT] = %llu\n", last_time[LEFT]);
        printf("[encoder_callback] last_time[RIGHT] = %llu\n", last_time[RIGHT]);
        printf("[encoder_callback] deltaT[LEFT] = %llu\n", deltaT[LEFT]);
        printf("[encoder_callback] deltaT[RIGHT] = %llu\n", deltaT[RIGHT]);
    }
}
