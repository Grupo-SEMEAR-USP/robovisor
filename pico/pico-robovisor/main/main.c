#include "../include/common.h"
#include "../include/pwm.h"
#include "../include/core1.h"
#include "hardware/irq.h"

/**
 * @brief main functionality of the low level control features of application
 *
 * TODO:
 * PID logic control
 * See any adversities that can arise from uint_32 implicit conversion
 *
 * FIRST:
 * Read displacement angles, in degrees, since last call
 *
 * SECOND:
 * Return values of previously read angle displacement for left THEN right values
 *
 * THIRD:
 * Read values for velocity commands (in m/s), in the same order as mentioned above
 * receives two uint8 values, that when concatenated (each pair), recovers true velocity command
 * If no velocity command is given, than previous one is maintained
 *
 * FOURTH:
 * Send this command to set_velocity function, whose implementation is in another file
 *
 */

#define READ_TIMEOUT_US 5000000

// Global variable that storages what is the next motor info to be read.
int read_order = LEFT;

// Global variable that storages the current velocity of each motor.
float current_velocity[2] = {0.0, 0.0};

// Global variable that storages the target velocity.
float velocity_target[2] = {0.0, 0.0};

// Global variable that storages the PWM value to be fed to the motors.
float output_PWM[2] = {0.0, 0.0};

// Global variable that storages the last target velocity.
float last_velocity_target[2] = {0.0, 0.0};

// Global variable that tracks the number of zeros received as target velocity.
int zero_received_count_left = 0;
int zero_received_count_right = 0;
bool zero_left = false;
bool zero_right = false;

// TODO:
// Need's further consideration of types & sizes, using int
// in this manner is bad, since it doesnt especify the size
// eg: ROS is sending unsigned 32 bit integer per motor, which
// only makes sense here if sizeof(int) == 16 bits == 2 bytes,
// which is not according to docs
// https://raspberry-projects.com/pi/programming-in-c/memory/variables
// for now let's abstract that, in the end, velocity
// (which is a signed floating type) is correctly
// especified in the end
void read_velocity_commands(float *velocity)
{
    bool hasTimeout = false;
    uint8_t velocity_l[4], velocity_r[4];
    uint32_t velocity_left_, velocity_right_;
    float velocity_left_temp, velocity_right_temp;

    char serialBuffer[10];
    fgets(serialBuffer, 10, stdin);

    //Waits to ROS get ready.
    if(serialBuffer[0] == 'g')
    {
        for(int i = 0; i < 4; i++)
        {
            velocity_l[i] = serialBuffer[i + 1];
            velocity_r[i] = serialBuffer[i + 5];
        }

        velocity_left_ = velocity_l[0] | velocity_l[1] << 8 | velocity_l[2] << 16 | velocity_l[3] << 24;
        memcpy(&velocity_left_temp, &velocity_left_, 4);
        velocity_right_ = velocity_r[0] | velocity_r[1] << 8 | velocity_r[2] << 16 | velocity_r[3] << 24;
        memcpy(&velocity_right_temp, &velocity_right_, 4);

        //TODO: please, find a better way to do this.
        //Avoid spikes in velocity commands.
        if(absFloat(last_velocity_target[LEFT] - velocity_left_temp) > MAX_VELOCITY_SPIKE)
        {
            velocity_left_temp = last_velocity_target[LEFT];
        }
        
        if(absFloat(last_velocity_target[RIGHT] - velocity_right_temp) > MAX_VELOCITY_SPIKE)
        {
            velocity_right_temp = last_velocity_target[RIGHT];
        }

        //TODO: please, find a better way to do this.
        //Avoid random zeros.
        if(velocity_left_temp == 0)
        {
            zero_received_count_left++;
            if(zero_received_count_left >= 20)
            {
                zero_left = true;
                
            }
        }
        else
        {
            zero_left = false;
            zero_received_count_left = 0;
        }

        if(velocity_right_temp == 0)
        {
            zero_received_count_right++;
            if(zero_received_count_right >= 20)
            {
                zero_right = true;
                zero_received_count_right = 0;
            }
        }
        else
        {
            zero_right = false;
            zero_received_count_right = 0;
        }

        velocity[LEFT] = (zero_left ? 0 : velocity_left_temp);
        velocity[RIGHT] = (zero_right ? 0 : velocity_right_temp);

        if(DEBUG_MAIN_RECEIVE)
        {
            /*for(int i = 0; i < 10; i++)
            {
                printf("[%d] %d\n", i, serialBuffer[i]);
            }*/

            if(last_velocity_target[LEFT] != velocity[LEFT] || last_velocity_target[RIGHT] != velocity[RIGHT])
            {
                printf("[RECEIVING] velocity_l[0] = %x, velocity_l[1] = %x, velocity_l[2] = %x, velocity_l[3] = %x\n", velocity_l[0], velocity_l[1], velocity_l[2], velocity_l[3]);
                printf("[RECEIVING] velocity_r[0] = %x, velocity_r[1] = %x, velocity_r[2] = %x, velocity_r[3] = %x\n", velocity_r[0], velocity_r[1], velocity_r[2], velocity_r[3]);
                printf("[RECEIVING] velocity[LEFT] = %.2f, velocity[RIGHT] = %.2f\n", velocity[LEFT], velocity[RIGHT]);
            }
        }

        last_velocity_target[LEFT] = velocity[LEFT];
        last_velocity_target[RIGHT] = velocity[RIGHT];
    }
}

void get_current_velocity_interrupt_handle()
{
    uint32_t raw;

    while (multicore_fifo_rvalid())   
    {
        raw = multicore_fifo_pop_blocking();
        switch (read_order)
        {
            case LEFT:
                memcpy(&current_velocity[LEFT], &raw, 4);
                break;

            case RIGHT:
                memcpy(&current_velocity[RIGHT], &raw, 4);
                break;

            default:
                printf("[get_current_velocity_interrupt_handle] Erro no cálculo de velocidade atual!");
                break;
        }

        read_order = 1 - read_order;
    }

    multicore_fifo_clear_irq();
}

void setup_core0()
{
    // Init pinnage
    sleep_ms(INITIAL_TIMEOUT_MS);
    stdio_init_all();

    // As pwm is handled by core0, it's pinnage is setted here
    init_pwm_pinnage();

    // Init multicore
    multicore_launch_core1(main_core1);

    // Configure a interruption to update current velocity each time the encoders are read.
    multicore_fifo_clear_irq();
    irq_set_exclusive_handler(SIO_IRQ_PROC0, get_current_velocity_interrupt_handle);
    irq_set_enabled(SIO_IRQ_PROC0, true);

    return;
}

void obtainingMotorCurve()
{
    getchar();
    float desired_velocity[2];
    int test_step = 100;
    int n_tests = 65535/test_step;
    uint32_t time = to_ms_since_boot(get_absolute_time());
    uint32_t actualTime = to_ms_since_boot(get_absolute_time());

    while(actualTime - time < 5000) 
    {
        desired_velocity[LEFT] = 40000;
        desired_velocity[RIGHT] = -40000;
        set_velocity(desired_velocity);
        printf("%.2f, %.2f, %.2f, %.2f, %d, %d, %d,\n", desired_velocity[LEFT], desired_velocity[RIGHT], current_velocity[LEFT], current_velocity[RIGHT], actualTime, time, (actualTime - time));
    	sleep_ms(10);
    	actualTime = to_ms_since_boot(get_absolute_time());
    }

    time = to_ms_since_boot(get_absolute_time());
    actualTime = to_ms_since_boot(get_absolute_time());

    while(actualTime - time < 5000)                                                                                                                                                                  
    {
        desired_velocity[LEFT] = 0;
        desired_velocity[RIGHT] = 0;
        set_velocity(desired_velocity);
        printf("%.2f, %.2f, %.2f, %.2f, %d, %d, %d,\n", desired_velocity[LEFT], desired_velocity[RIGHT], current_velocity[LEFT], current_velocity[RIGHT], actualTime, time, (actualTime - time));
    	sleep_ms(10);
    	actualTime = to_ms_since_boot(get_absolute_time());
    }

    desired_velocity[LEFT] = 0;
    desired_velocity[RIGHT] = 0;
    set_velocity(desired_velocity);    
}

int main(void)
{
    // Set all pins and init multicore and PID.
    setup_core0();

    // Init PID controllers.
    struct pid_controller ctrldata_left, ctrldata_right;
    pid_cont_t pid_left, pid_right;

    double kp_l = 54.1775;
    double ki_l = 267.8801;
    double kd_l = 0;

    double kp_r = 59.5765;
    double ki_r = 229.3613;
    double kd_r = 0;

    pid_left = pid_create(&ctrldata_left, &current_velocity[LEFT], &output_PWM[LEFT], &velocity_target[LEFT], kp_l, ki_l, kd_l);
    pid_right = pid_create(&ctrldata_right, &current_velocity[RIGHT], &output_PWM[RIGHT], &velocity_target[RIGHT], kp_r, ki_r, kd_r);

    pid_limits(pid_left, -MAX_PWM, MAX_PWM);
    pid_limits(pid_right, -MAX_PWM, MAX_PWM);

    pid_auto(pid_left);
    pid_auto(pid_right);

    /*float delta_time_left = 0;
    float delta_time_right = 0;
    float last_time_left = to_ms_since_boot(get_absolute_time());
    float last_time_right = to_ms_since_boot(get_absolute_time());*/

    // Core 0 main loop.
    while (1)
    {
        //Obtains the motor curve for motor control 
        //obtainingMotorCurve();

        // AFAIK this is intended for SMALL loop optimization, not sure if useful here
        //tight_loop_contents();

        // Read velocity from Serial
        read_velocity_commands(velocity_target);

        if (pid_need_compute(pid_left))
        {
            // delta_time_left = to_ms_since_boot(get_absolute_time()) - last_time_left;
            pid_compute(pid_left);
            // last_time_left = to_ms_since_boot(get_absolute_time());
        }

        if (pid_need_compute(pid_right))
        {
            // delta_time_right = to_ms_since_boot(get_absolute_time()) - last_time_right;
            pid_compute(pid_right);
            // last_time_right = to_ms_since_boot(get_absolute_time());
	}

        if (DEBUG_MAIN)
        {
           //printf("left frequency = %.2f, right frequency = %.2f\n", delta_time_left, delta_time_right);
           printf("[RASP/READ] left motor -> %d, %f, %.2f, %.2f\n", to_ms_since_boot(get_absolute_time()), current_velocity[LEFT], output_PWM[LEFT], velocity_target[LEFT]);
           printf("[RASP/READ] right motor -> %d, %f, %.2f, %.2f\n", to_ms_since_boot(get_absolute_time()), current_velocity[RIGHT], output_PWM[RIGHT], velocity_target[RIGHT]);
        }

        // Send velocity target to motors.
        set_velocity(output_PWM);
    }
}
