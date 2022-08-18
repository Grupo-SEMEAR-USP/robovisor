#include <prototipo1/robot_hw_if.hpp>
#include <iostream>
#include <string>

void write_Int32_to_SerialBuffer(uint8_t *serialBuffer, uint32_t value)
{
    for(int i=0; i<4; i++)
        serialBuffer[i] = (value >> 8*i) & LAST_BYTE_TAKE_MASK;
}


RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface()
{
    serialPort->close();
}

void RobotHWInterface::init()
{

    for (int i = 0; i < 2; i++)
    {
        // Create joint state interface
        hardware_interface::JointStateHandle jointStateHandle(joint_name_[i], &joint_position_[i], &joint_velocity_[i], &joint_effort_[i]);
        joint_state_interface_.registerHandle(jointStateHandle);

        // Create velocity joint interface
        hardware_interface::JointHandle jointVelocityHandle(jointStateHandle, &joint_velocity_command_[i]);
        velocity_joint_interface_.registerHandle(jointVelocityHandle);

        // Create Joint Limit interface
        joint_limits_interface::JointLimits limits;
        joint_limits_interface::getJointLimits(joint_name_[i], nh_, limits);
        joint_limits_interface::VelocityJointSaturationHandle jointLimitsHandle(jointVelocityHandle, limits);
        velocityJointSaturationInterface.registerHandle(jointLimitsHandle);
    }

    // Register all joints interfaces
    registerInterface(&joint_state_interface_);
    registerInterface(&velocity_joint_interface_);
    registerInterface(&velocityJointSaturationInterface);
}

void RobotHWInterface::update(const ros::TimerEvent &e)
{
    // std::cout << "[UPDATE!]" << std::endl;
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    //read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

uint8_t convert(char C){

    uint8_t retval;
    if(C >= '0' && C <= '9'){
        retval = C - '0';
    }else{
        // maior que 9 ==> 'a'
        retval = C - 'a' + 10;
    }

    return retval;
}

void RobotHWInterface::read()
{
    std::string serialBuffer;
    while(!readStarted)
    {
        std::cout << "Trying to sync serial port..." << std::endl;
        serialPort->read(serialBuffer, 10*sizeof(char));
        std::cout << "I received: " << serialBuffer.c_str() << std::endl;
        if(serialBuffer.c_str()[0] == 'g')
            readStarted = true;
        serialBuffer.clear();
    }
    
    uint8_t floatBuffer[4];
    float dtheta;
    if(serialPort->available())
    {
        serialPort->readline(serialBuffer, 9, "g");

        for(int i = 0; i < 4; i++)
        {
            uint8_t p1 = convert(serialBuffer.c_str()[2*i]);
            uint8_t p2 = convert(serialBuffer.c_str()[2*i + 1]);
            //std::cout << "p1 = " << p1 << " p2 = " << p2 << std::endl;
            floatBuffer[3 - i] = p1 << 4 | p2;
            //printf("floatBuffer[i] = %d\n", floatBuffer[i]);
        }
        memcpy(&dtheta, floatBuffer, 4);
        left_motor_pos += angles::from_degrees((double)dtheta);
        joint_position_[0] = left_motor_pos;

        std::cout << "[ROS/READ] Left Motor: " << " dtheta = " << (float) dtheta << " joint_position_[0] = " << joint_position_[0] << std::endl;

        /*printf("String recebida = %s \n", serialBuffer.c_str());
        printf("String recebida left = %x %x %x %x\n",
            floatBuffer[0],
            floatBuffer[1],
            floatBuffer[2],
            floatBuffer[3]);
        const unsigned char * pf = reinterpret_cast<const unsigned char*>(&dtheta);
        for(int i = 0; i < 4; i++)
        {
            printf("[READ] float hex[%d] = %x\n", i, pf[i]);
        }
        printf("Float value: %f\n", (float) dtheta);*/
        serialBuffer.clear();

        serialPort->readline(serialBuffer, 9, "g");

        for(int i = 0; i < 4; i++)
        {
            uint8_t p1 = convert(serialBuffer.c_str()[2*i]);
            uint8_t p2 = convert(serialBuffer.c_str()[2*i + 1]);
            //std::cout << "p1 = " << p1 << " p2 = " << p2 << std::endl;
            floatBuffer[3 - i] = p1 << 4 | p2;
            //printf("floatBuffer[i] = %d\n", floatBuffer[i]);
        }
        memcpy(&dtheta, floatBuffer, 4);
        //TODO: check the motor way signal (+=)
        right_motor_pos += angles::from_degrees((double)dtheta);
        joint_position_[1] = right_motor_pos;
        serialBuffer.clear();

        std::cout << "[ROS/READ] Right Motor: " << " dtheta = " << dtheta << " joint_position_[1] = " << joint_position_[1] << std::endl;
    }
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
    //velocityJointSaturationInterface.enforceLimits(elapsed_time);
    
    serialPort->write(write_flag_begin, 1*sizeof(uint8_t));
    uint8_t serialBuffer[4];
    uint32_t velocity, result;
    float velocity_left, velocity_right;

    // --- Left 
    velocity_left = angles::to_degrees(joint_velocity_command_[0]);
    memcpy(&velocity, &velocity_left, 4);
    printf("[ROS/WRITE] left velocity  = %.2f\n", velocity);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    /*for(int i = 0; i < 4; i++)
    {
        printf("[left] serialBuffer[%d] = %x\n", i, serialBuffer[i]);
    }*/
    result = (uint32_t)serialPort->write(serialBuffer, 4*sizeof(uint8_t));

    // --- Right
    velocity_right = angles::to_degrees(joint_velocity_command_[1]);
    memcpy(&velocity, &velocity_right, 4);
    printf("[ROS/WRITE] right velocity  = %.2f\n", velocity);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    /*for(int i = 0; i < 4; i++)
    {
        printf("[right] serialBuffer[%d] = %x\n", i, serialBuffer[i]);
    }*/
    result = (uint32_t)serialPort->write(serialBuffer, 4*sizeof(uint8_t));

    serialPort->write(write_flag_end, 1*sizeof(uint8_t));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    // Not sure why needed
    ros::MultiThreadedSpinner spinner(N_THREADS);
    // Interesting alternative: AsyncSpinner

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RobotHWInterface robot(nh);
    spinner.spin();

    return 0;
}
