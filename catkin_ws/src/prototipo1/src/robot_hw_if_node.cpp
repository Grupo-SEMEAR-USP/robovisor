#include <prototipo1/robot_hw_if.hpp>
#include <prototipo1/i2c.hpp>

RobotHWInterface::RobotHWInterface (ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHWInterface::update, this);
}

RobotHWInterface::~RobotHWInterface() {
}

void RobotHWInterface::init() {
	
	for(int i=0; i<2; i++)
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

void RobotHWInterface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

void RobotHWInterface::read() {
    
    // Convention: 
    // 0 index is left motor 
    // 1 index is right motor 
    uint8_t rbuff[2];
    int dtheta_l, dtheta_r;

    // Size of buffer is 2 due to the fact we are reading from
    // a single Rasp Pico, that controls BOTH motors 
    motors.readBytes(rbuff,2);

    dtheta_l=(int8_t)rbuff[0];
    dtheta_r=(int8_t)rbuff[1];

    left_motor_pos+=angles::from_degrees((double)dtheta_l);
    right_motor_pos+=angles::from_degrees((double)dtheta_r);

    joint_position_[0]=left_motor_pos;
    joint_position_[1]=right_motor_pos;

    ROS_INFO("pos=%.2f dtheta_l=%d ",left_motor_pos,dtheta_l);
    ROS_INFO("pos=%.2f dtheta_r=%d ",right_motor_pos,dtheta_r);

<<<<<<< HEAD
=======
    joint_position_[0]=2*((int)ros::Time::now().toSec()%50);
    joint_position_[1]=1*((int)ros::Time::now().toSec()%50);
>>>>>>> 7a34f1c6123a367447a16907349f59b0c1177902
}

void RobotHWInterface::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

	uint8_t wbuff[2];

    int velocity,result;
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    // Left motor goes first
    if(left_prev_cmd!=velocity)
    {
	    result = motors.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    // Right motor goes first
    if(right_prev_cmd!=velocity)
    {
	    result = motors.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); 
    RobotHWInterface robot(nh);

    spinner.spin();
    return 0;
}