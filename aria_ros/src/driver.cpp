#include <ros/ros.h>
#include <aria_ros/AriaRobot.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "aria_driver");
	ros::NodeHandle n;
	
	AriaRobot robot(argc, argv);

	ros::Subscriber velocitySubscriber = n.subscribe("cmd_vel", 1, &AriaRobot::setVelocity, &robot);
	ros::Subscriber saySubscriber = n.subscribe("say", 1, &AriaRobot::say, &robot);

	ros::spin();
	return 0;
}
