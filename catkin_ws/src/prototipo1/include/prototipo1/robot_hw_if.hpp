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
#include <prototipo1/i2c_ros.h>

#define BUS_ADDRESS_L 0x3E
#define BUS_ADDRESS_R 0x3F
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
    ros::Publisher pub;
    ros::ServiceClient client;
    rospy_tutorials::Floats joints_pub;

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
    int left_prev_cmd = 0, right_prev_cmd = 0;
    i2c_ros::I2C left_motor = i2c_ros::I2C(0, BUS_ADDRESS_L);
    i2c_ros::I2C right_motor = i2c_ros::I2C(1, BUS_ADDRESS_R);

    ros::NodeHandle nh_;
    ros::Timer non_realtime_loop_;
    ros::Duration elapsed_time_;
    double loop_hz_;
    boost::shared_ptr<controller_manager::ControllerManager> controller_manager_;
};
