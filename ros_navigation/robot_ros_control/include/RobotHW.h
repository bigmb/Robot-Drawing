/* Author: Dina Youakim */

#ifndef Robot_HW_H
#define Robot_HW_H


// ROS
#include <ros/ros.h>
#include <urdf/model.h>
#include <rosparam_shortcuts/rosparam_shortcuts.h>
// ROS Controls
#include <hardware_interface/robot_hw.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <controller_manager/controller_manager.h>
#include <joint_limits_interface/joint_limits.h>
#include <joint_limits_interface/joint_limits_interface.h>
#include <joint_limits_interface/joint_limits_rosparam.h>
#include <joint_limits_interface/joint_limits_urdf.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Int16.h>


class RobotHW : public hardware_interface::RobotHW
{
	private:

		ros::NodeHandle nh_;

		// Short name of this class
  	std::string name_;

  	// Configuration to be loaded
  	std::vector<std::string> jointNames_;
  	std::size_t numJoints_;
  	urdf::Model *urdfModel_;

    //TODO: Define the Hardware interfaces needed for each controller you use
    //e.g. for the joint state controller: hardware_interface::JointStateInterface jntStateInterface_;


hardware_interface::JointStateInterface jointStateInterface;
  hardware_interface::VelocityJointInterface jointVelocityInterface;
  hardware_interface::PositionJointInterface jointPositionInterface;
  //hardware_interface::EffortJointInterface jnt_eff_interface;
//hardware_interface::ImuSensorInterface imu_interface;

    //TODO: define vector(s) to save Joint States and the Joint Commands to be linked with the hardware interfaces defined
		std::vector<double> joint_states_velocity, joint_states_effort, joint_states_position;
		std::vector<double> vel_commands;

    //TODO: Define Joint limits interfaces - Saturation

	int encoder_right, encoder_left;
  ros::Subscriber sub_left, sub_right;
ros::Publisher pub_left, pub_right;

  	public:
  		RobotHW(ros::NodeHandle &nh);
      void loadURDF(ros::NodeHandle &nh, std::string param_name);
      void registerJointLimits(const hardware_interface::JointHandle &joint_handle_velocity, std::size_t joint_id);
      void enforceLimits(ros::Duration &period);
      void encoder_left_subscriber(const std_msgs::Int16&);
      void encoder_right_subscriber(const std_msgs::Int16&);

      //These virtual functions are inherited from RobotHW class more info here http://docs.ros.org/kinetic/api/hardware_interface/html/c++/classhardware__interface_1_1RobotHW.html#details
  		virtual void init();
  		virtual void read(const ros::Duration &elapsed_time);
  		virtual void write(const ros::Duration &elapsed_time);

};

#endif
