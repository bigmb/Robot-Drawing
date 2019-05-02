/* Author: Dina Youakim */


#include "RobotHW.h"

RobotHW::RobotHW(ros::NodeHandle &nh)
{
	name_= "robot_hw_interface";
	nh_= nh;
	loadURDF(nh_, "robot_description");
  ros::NodeHandle rpnh(nh_, "hardware_interface");
  rosparam_shortcuts::get(name_, rpnh, "joints", jointNames_);

  std::cout<<"startiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiiing"<<std::endl;
  sub_left = nh.subscribe("encoder_left", 1000, &RobotHW::encoder_left_subscriber, this);
  sub_right = nh.subscribe("/encoder_right", 1000, &RobotHW::encoder_right_subscriber, this);
 pub_left = nh.advertise<std_msgs::Int16>("/motor_left", 100);
 pub_right = nh.advertise<std_msgs::Int16>("/motor_right", 100);




encoder_right = 0;
encoder_left = 0;
}

void RobotHW::init()
{
	numJoints_ = jointNames_.size();

	//TODO: Initialize States and Commands Vectors
	joint_states_velocity = std::vector<double>(numJoints_);
	joint_states_effort = std::vector<double>(numJoints_);
	joint_states_position = std::vector<double>(numJoints_);

	vel_commands = std::vector<double>(numJoints_);


	for (std::size_t joint_id = 0; joint_id < numJoints_; ++joint_id)
	{
  	ROS_DEBUG_STREAM_NAMED(name_, "Loading joint name: " << jointNames_[joint_id]);

    //TODO: Initialize both joint state and command interfaces for each joint

	 // Create joint state handle and register the handle to the interface
	 jointStateInterface.registerHandle(hardware_interface::JointStateHandle(jointNames_[joint_id], &joint_states_position[joint_id], &joint_states_velocity[joint_id], &joint_states_effort[joint_id]));

	 // Create command handle and register it to the interface
	 //jointPositionInterface.registerHandle(hardware_interface::JointHandle(jointStateInterface.getHandle(jointNames_[joint_id]), &vel_commands[joint_id]));

jointVelocityInterface.registerHandle(hardware_interface::JointHandle(jointStateInterface.getHandle(jointNames_[joint_id]), &vel_commands[joint_id]));

	 // TODO: Load the joint limits by calling registerJointLimits
	 //registerJointLimits(jntPosInterface_.getHandle(jointNames_[joint_id]), joint_id);
	}

  //TODO: register the interfaces
	registerInterface(&jointStateInterface);
	registerInterface(&jointVelocityInterface);

	ROS_INFO_STREAM(name_<<" Hardware Interface Ready.");


}

void RobotHW::loadURDF(ros::NodeHandle &nh, std::string param_name)
{
	std::string urdf_string;
  urdfModel_ = new urdf::Model();
  ROS_INFO_STREAM("Look for the model in ns: "<<nh.getNamespace());
	// search and wait for robot_description on param server
	while (urdf_string.empty() && ros::ok())
	{
  	std::string search_param_name;
  	if (nh.searchParam(param_name, search_param_name))
  	{
    		ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: " <<search_param_name);
    		nh.getParam(search_param_name, urdf_string);
  	}
  	else
  	{
    		ROS_INFO_STREAM_NAMED(name_, "Waiting for model URDF on the ROS param server at location: "<< param_name);
    		nh.getParam(param_name, urdf_string);
  	}

  	usleep(100000);
	}

	if (!urdfModel_->initString(urdf_string))
  	ROS_ERROR_STREAM_NAMED(name_, "Unable to load URDF model");
	else
  	ROS_INFO_STREAM_NAMED(name_, "Received URDF from param server");
}

void RobotHW::registerJointLimits(const hardware_interface::JointHandle &joint_handle_velocity, std::size_t joint_id)
{
  	
}

void RobotHW::enforceLimits(ros::Duration &period)
{
  	//velJntSatInterface_.enforceLimits(period);
}

void RobotHW::read(const ros::Duration &elapsed_time)
{
   //ROS_INFO_STREAM("in the read");
   //TODO: read the current state of each joint
	 joint_states_position[0] = 3.1415*encoder_left/900.0;
	 joint_states_position[1] = 3.1415*encoder_right/900.0;
}

void RobotHW::encoder_left_subscriber(const std_msgs::Int16& msg){
	
	encoder_left = msg.data;
}

void RobotHW::encoder_right_subscriber(const std_msgs::Int16& msg){
	encoder_right = msg.data;
}

void RobotHW::write(const ros::Duration &elapsed_time)
{
  //ROS_INFO_STREAM("in the write");
  //TODO: write down the needed commands to move the robot
	//publishers

	std::cout<<vel_commands[0]<<" "<<vel_commands[1]<<std::endl;
	std_msgs::Int16 left, right;
	left.data = vel_commands[0]*60;
	right.data = vel_commands[1]*60;
	pub_left.publish(left);
	pub_right.publish(right);



}
