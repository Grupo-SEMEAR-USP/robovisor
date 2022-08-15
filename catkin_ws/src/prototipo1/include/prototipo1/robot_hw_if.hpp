#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <controller_manager/controller_manager.h>
#include <ros/ros.h>
#include <rospy_tutorials/Floats.h>
#include <angles/angles.h>

#include <string>
#include <serial/serial.h>

#define SERIAL_PORT_NAME "/dev/ttyACM0"
#define PICO_BAUD_RATE 115200
#define PPR_1QUAD 200
#define TICKS2DEGREES ((double) 360)/((double) PPR_1QUAD)
#define HW_IF_UPDATE_FREQ 10000.0
#define LAST_BYTE_TAKE_MASK 0xFF
#define LAST_BYTE_EXCLUDE_MASK 0xFFFFFF00
#define N_THREADS 4
class RobotHWInterface : public hardware_interface::RobotHW 
{

public:
    // Constructor receives the node handler
    RobotHWInterface(ros::NodeHandle &nh);
    ~RobotHWInterface();
    void init();
    void update(const ros::TimerEvent &e);
    void read();
    void write(ros::Duration elapsed_time);

    /* Not sure if needed 
    ros::Publisher pub;
    ros::ServiceClient client;
    rospy_tutorials::Floats joints_pub;
    */

protected: 

    hardware_interface::JointStateInterface joint_state_interface_;
    hardware_interface::VelocityJointInterface velocity_joint_interface_;

    joint_limits_interface::VelocityJointSaturationInterface velocityJointSaturationInterface;

    std::string joint_name_[2] = {"left_wheel_joint", "right_wheel_joint"};
    double joint_position_[2];
    double joint_velocity_[2];
    double joint_effort_[2];
    double joint_velocity_command_[2];

    double left_motor_pos = 0, right_motor_pos = 0;

    serial::Serial* serialPort = new serial::Serial(std::string(SERIAL_PORT_NAME), PICO_BAUD_RATE, serial::Timeout::simpleTimeout(250));

    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_ = HW_IF_UPDATE_FREQ;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
