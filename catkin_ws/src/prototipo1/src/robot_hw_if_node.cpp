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
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void RobotHWInterface::read()
{
    std::string serialBuffer;
    float dtheta;

    serialPort->read(serialBuffer, 4);
    memcpy(&dtheta, serialBuffer.c_str(), 4);
    left_motor_pos += angles::from_degrees((double)dtheta);
    joint_position_[0] = left_motor_pos;

    std::cout << "[READ] Left Motor: " << " dtheta = " << dtheta << " joint_position_[0] = " << joint_position_[0] << std::endl;

    serialPort->read(serialBuffer, 4);
    memcpy(&dtheta, serialBuffer.c_str(), 4);
    right_motor_pos += angles::from_degrees((double)dtheta);
    joint_position_[1] = right_motor_pos;

    std::cout << "[READ] Right Motor: " << " dtheta = " << dtheta << " joint_position_[1] = " << joint_position_[1] << std::endl;
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
    velocityJointSaturationInterface.enforceLimits(elapsed_time);

    // 4*sizeof(uint8_t) = sizeof(uint32_t)
    uint8_t serialBuffer[4];
    uint32_t velocity, result;

    // --- Left 
    velocity = (uint32_t)angles::to_degrees(joint_velocity_command_[0]);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    result = (uint32_t)serialPort->write(serialBuffer, 4 * sizeof(uint8_t));

    // --- Right
    velocity = (uint32_t)angles::to_degrees(joint_velocity_command_[1]);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    result = (uint32_t)serialPort->write(serialBuffer, 4 * sizeof(uint8_t));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    // Not sure why needed
    ros::MultiThreadedSpinner spinner(N_THREADS);
    // Interesting alternative: AsyncSpinner

    RobotHWInterface robot(nh);
    spinner.spin();

    return 0;
}
