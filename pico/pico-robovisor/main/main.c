#include "../include/common.h"
#include "../include/pwm.h"
#include "../include/core1.h"
#include "hardware/irq.h"


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

// Reads velocity commands from ROS, through serial, and stores them in the global variables.
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

        //Avoid spikes in velocity commands.
        if(absFloat(last_velocity_target[LEFT] - velocity_left_temp) > MAX_VELOCITY_SPIKE)
        {
            velocity_left_temp = last_velocity_target[LEFT];
        }
        
        if(absFloat(last_velocity_target[RIGHT] - velocity_right_temp) > MAX_VELOCITY_SPIKE)
        {
            velocity_right_temp = last_velocity_target[RIGHT];
        }

        //Avoid random zeros
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

// Gets the current velocity of each motor, from core1, and stores it in the global variables.
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

// Initiates all the core pinnage functionality
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

// Main function for core0.
int main(void)
{
    // Set all pins and init multicore and PID.
    setup_core0();

    // Init PID controllers.
    struct pid_controller ctrldata_left, ctrldata_right;
    pid_cont_t pid_left, pid_right;

    // Current best estimates for PID parameters.
    double kp = 100;
    double ki = 448.5981;
    double kd = 0;

    // Attaches the parameters to the PID controllers.
    pid_left = pid_create(&ctrldata_left, &current_velocity[LEFT], &output_PWM[LEFT], &velocity_target[LEFT], kp, ki, kd);
    pid_right = pid_create(&ctrldata_right, &current_velocity[RIGHT], &output_PWM[RIGHT], &velocity_target[RIGHT], kp, ki, kd);
    
    // Attaches limits to the PID controllers.
    pid_limits(pid_left, -MAX_PWM, MAX_PWM);
    pid_limits(pid_right, -MAX_PWM, MAX_PWM);

    // Let's the PID controllers have full control (auto mode) over the PWM commands
    pid_auto(pid_left);
    pid_auto(pid_right);

    // Core 0 main loop.
    while (1)
    {

        // Reads velocities from Serial
        read_velocity_commands(velocity_target);

        // Updates the PID controllers.
        if (pid_need_compute(pid_left))
        {
            pid_compute(pid_left);
        }

        // Updates the PID controllers.
        if (pid_need_compute(pid_right))
        {
            pid_compute(pid_right);
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
