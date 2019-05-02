/* Author: Dina Youakim */


#include "RobotHW.h"
#include <controller_manager/controller_manager.h>

int main (int argc, char **argv)
{
	ros::init (argc, argv, "robot_controller");
	ros::NodeHandle nh;

	
	RobotHW* robotHW = new RobotHW(nh);
	robotHW->init();


	ros::AsyncSpinner spinner(1); 
    spinner.start();
	
	controller_manager::ControllerManager controller_manager(robotHW,nh);

	ros::Time prev_time = ros::Time::now();
	//TODO: check the rate if it needs to be changed
	ros::Rate rate(10);


	while(ros::ok())
	{
		const ros::Time time = ros::Time::now();
  		const ros::Duration period = time - prev_time;

  		robotHW->read(period);
  		controller_manager.update(time, period);
  		robotHW->write(period);

  		rate.sleep();
  	}
	
    return 0;
}

     


