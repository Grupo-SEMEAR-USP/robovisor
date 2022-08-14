#include <prototipo1/robot_hw_if.hpp>
#include <iostream>

void writeInt2SerialBuff(uint8_t *serialBuffer, uint32_t velocity)
{
    serialBuffer[0] = (velocity) & LAST_BYTE_MASK;
    serialBuffer[1] = (velocity >> 8) & LAST_BYTE_MASK;
    serialBuffer[2] = (velocity >> 16) & LAST_BYTE_MASK;
    serialBuffer[3] = (velocity >> 24) & LAST_BYTE_MASK;
}

RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_ = HW_IF_UPDATE_FREQ;
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface()
{
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
    uint8_t serialBuffer[1];
    int dtheta;

    serialPort->read(serialBuffer, sizeof(uint8_t));
    dtheta = (int8_t)serialBuffer[0];
    left_motor_pos += angles::from_degrees(TICKS2DEGREES * (double)dtheta);
    joint_position_[0] = left_motor_pos;

    // std::cout << "[READ] Left Motor: " << " dtheta = " << dtheta << " joint_position_[0] = " << joint_position_[0] << std::endl;

    serialPort->read(serialBuffer, sizeof(uint8_t));
    dtheta = (int8_t)serialBuffer[0];
    right_motor_pos += angles::from_degrees(TICKS2DEGREES * (double)dtheta);
    joint_position_[1] = right_motor_pos;

    // std::cout << "[READ] Right Motor: " << " dtheta = " << dtheta << " joint_position_[0] = " << joint_position_[0] << std::endl;
}

void RobotHWInterface::write(ros::Duration elapsed_time)
{
    velocityJointSaturationInterface.enforceLimits(elapsed_time);

    // 4*sizeof(uint8_t) = sizeof(uint32_t)
    uint8_t serialBuffer[4];

    uint32_t velocity, result;

    velocity = (uint32_t)angles::to_degrees(joint_velocity_command_[0]);
    
    writeInt2SerialBuff(serialBuffer, velocity);

    // ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);
    // std::cout << "[WRITE] Left Motor:" << " joint_velocity_command_[0] = " << joint_velocity_command_[0] << " velocity = " << velocity << " serialBuffer[0] = " << serialBuffer[0] << " serialBuffer[1] = " << serialBuffer[1] << std::endl;

    if (left_prev_cmd != velocity)
    {
        result = (uint32_t)serialPort->write(serialBuffer, 4 * sizeof(uint8_t));
        // ROS_INFO("Writen successfully result=%d", result);
        left_prev_cmd = velocity;
    }

    velocity = (uint32_t)angles::to_degrees(joint_velocity_command_[1]);

    writeInt2SerialBuff(serialBuffer, velocity);

    // ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);
    // std::cout << "[WRITE] Left Motor:" << " joint_velocity_command_[0] = " << joint_velocity_command_[0] << " velocity = " << velocity << " serialBuffer[0] = " << serialBuffer[0] << " serialBuffer[1] = " << serialBuffer[1] << std::endl;

    if (right_prev_cmd != velocity)
    {

        result = (uint32_t)serialPort->write(serialBuffer, 4 * sizeof(uint8_t));
        // ROS_INFO("Writen successfully result=%d", result);
        right_prev_cmd = velocity;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    // Not sure if needed
    ros::MultiThreadedSpinner spinner(4);
    // Interesting alternative: AsyncSpinner
    RobotHWInterface robot(nh);
    spinner.spin();

    return 0;
}
