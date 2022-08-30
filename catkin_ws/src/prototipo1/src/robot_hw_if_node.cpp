#include <prototipo1/robot_hw_if.hpp>
#include <iostream>
#include <string>


// Decomposes a uint32_t into a vector of 4 bytes (uint8_t)
void write_Int32_to_SerialBuffer(uint8_t *serialBuffer, uint32_t value)
{
    for(int i=0; i<4; i++)
        serialBuffer[i] = (value >> 8*i) & LAST_BYTE_TAKE_MASK;
}

// Constructor of the class, initializes the timer used for event handling and the controller manager
RobotHWInterface::RobotHWInterface(ros::NodeHandle &nh) : nh_(nh)
{
    init();

    controller_manager_.reset(new controller_manager::ControllerManager(this, nh_));
    ros::Duration update_freq = ros::Duration(1.0 / loop_hz_);

    non_realtime_loop_ = nh_.createTimer(update_freq, &RobotHWInterface::update, this);
}

// Simple class destructor
RobotHWInterface::~RobotHWInterface()
{
    serialPort->close();
}

// Creates the JointVelocity interface and the JointState interface, 
// attaching it to the handles and registering them in the controller manager
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

// Update function needed for the event loop of odometry in controller manager
void RobotHWInterface::update(const ros::TimerEvent &e)
{
    elapsed_time_ = ros::Duration(e.current_real - e.last_real);
    read();
    controller_manager_->update(ros::Time::now(), elapsed_time_);
    write(elapsed_time_);
}

// Converts char value to int value
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

// Read function for receiving data from the serial port of angle displacement
void RobotHWInterface::read()
{
    std::string serialBuffer;
    uint8_t floatBuffer[4];
    float dtheta;
    
    // Technically it would be optimal if left state were to always be read first
    // as it would respect the time ordering of the messages L(t) --> R(t); t++
    // however, as states are unlikely to change much between timestamps, this is
    // not a big deal and works fine for now.
    if(serialPort->available())
    {
        for(int i = 0; i < 2; i++)
        {
	    std::string pos_flag;

 	    do                                                                                 
	    {                                                                                  
		pos_flag.clear();
            	serialPort->read(pos_flag, 1);                                         
	    } while(!((pos_flag.c_str()[0] == 'g') || (pos_flag.c_str()[0] == 'h')));
        // Initial idea was about using a XNOR, but as g = true ==> h == false
        // and as h == true ==> g == false (but not necessarily the other way around)
        // it's easier to use the !(g or h), which is the same for this case

	    serialBuffer.clear();
	    serialPort->read(serialBuffer, 8);

            for(int j = 0; j < 4; j++)
            {
                uint8_t p1 = convert(serialBuffer.c_str()[2*j]);
                uint8_t p2 = convert(serialBuffer.c_str()[2*j + 1]);
                floatBuffer[3 - j] = p1 << 4 | p2;
            }
            memcpy(&dtheta, floatBuffer, 4);

            switch (pos_flag.c_str()[0])
            {
            case 'g':
                left_motor_pos += angles::from_degrees((double)dtheta);
                joint_position_[0] = left_motor_pos;
                //std::cout << "[ROS/READ] Left Motor: " << " dtheta = " << (float) dtheta << " joint_position_[0] = " << joint_position_[0] << std::endl;
                break;

            case 'h':
                right_motor_pos += angles::from_degrees((double)dtheta);
                joint_position_[1] = right_motor_pos;
                //std::cout << "[ROS/READ] Right Motor: " << " dtheta = " << dtheta << " joint_position_[1] = " << joint_position_[1] << std::endl;
                break;
            
            default:
                break;
            }

        }
    }
}

// Writes velocity commands to the serial port 
void RobotHWInterface::write(ros::Duration elapsed_time)
{
    velocityJointSaturationInterface.enforceLimits(elapsed_time);
    
    serialPort->write(write_flag_begin, 1*sizeof(uint8_t));

    uint8_t serialBuffer[4];
    uint32_t velocity, result;
    float velocity_left, velocity_right;

    // --- Left 
    velocity_left = angles::to_degrees(joint_velocity_command_[0]);
    memcpy(&velocity, &velocity_left, 4);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    result = (uint32_t)serialPort->write(serialBuffer, 4*sizeof(uint8_t));

    // --- Right
    velocity_right = angles::to_degrees(joint_velocity_command_[1]);
    memcpy(&velocity, &velocity_right, 4);
    write_Int32_to_SerialBuffer(serialBuffer, velocity);
    result = (uint32_t)serialPort->write(serialBuffer, 4*sizeof(uint8_t));

    serialPort->write(write_flag_end, 1*sizeof(uint8_t));
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "prototipo1_hw_if");
    ros::NodeHandle nh;

    ros::MultiThreadedSpinner spinner(N_THREADS);

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    RobotHWInterface robot(nh);
    spinner.spin();

    return 0;
}
