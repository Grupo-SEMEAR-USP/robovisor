#include <prototipo1/robot_hw_if.hpp>

robot_hw_interface::robot_hw_interface (ros::NodeHandle& nh) : nh_(nh) {
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    loop_hz_=10;
    ros::Duration update_freq = ros::Duration(1.0/loop_hz_);
	
    non_realtime_loop_ = nh_.createTimer(update_freq, &robot_hw_interface::update, this);
}

robot_hw_interface::~robot_hw_interface() {
}

void robot_hw_interface::init() {
	
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

void robot_hw_interface::update(const ros::TimerEvent& e) {
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

// TODO 
void robot_hw_interfac::read() {
    /*
    uint8_t rbuff[1];
    int x;

    left_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    left_motor_pos+=angles::from_degrees((double)x);
    joint_position_[0]=left_motor_pos;

    right_motor.readBytes(rbuff,1);
    x=(int8_t)rbuff[0];
    right_motor_pos+=angles::from_degrees((double)x);
    joint_position_[1]=right_motor_pos;

    ROS_INFO("pos=%.2f x=%d ",pos,x);
    */
}

void robot_hw_interfac::write(ros::Duration elapsed_time) {
   
    velocityJointSaturationInterface.enforceLimits(elapsed_time);   

   /* 
	uint8_t wbuff[2];

    int velocity,result;
    
    
    velocity=([int)angles::to_degrees(joint_velocity_command_0]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(left_prev_cmd!=velocity)
    {
	    result = left_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    left_prev_cmd=velocity;
    }
    
    velocity=(int)angles::to_degrees(joint_velocity_command_[1]);
	wbuff[0]=velocity;
    wbuff[1]=velocity >> 8;
	//ROS_INFO("joint_velocity_command_[0]=%.2f velocity=%d  B1=%d B2=%d", joint_velocity_command_[0],velocity,wbuff[0],wbuff[1]);

    if(right_prev_cmd!=velocity)
    {
	    result = right_motor.writeData(wbuff,2);
	    //ROS_INFO("Writen successfully result=%d", result);
	    right_prev_cmd=velocity;
    }
    */
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(2); 
    robot_hw_interface robot(nh);

    spinner.spin();
    return 0;
}