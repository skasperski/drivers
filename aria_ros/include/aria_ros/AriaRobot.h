#ifndef ARIA_ROBOT_H
#define ARIA_ROBOT_H

#include <Aria.h>

#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Float32.h>

#include <string>

#define PI 3.14159265

class AriaRobot
{
public:
	AriaRobot(int argc, char **argv);
	~AriaRobot();

	// Callbacks
	void publishStatus() const;
	
	// Set the velocity using a navigation-stack compatible Twist-Message
	void setVelocity(const geometry_msgs::Twist::ConstPtr& msg);
	void say(const std_msgs::Int8::ConstPtr& msg);
		
private:
	// Private methods

	// Everything related to ARIA
	ArRobot* mRobot;
	ArArgumentParser* mArgumentParser;
	ArRobotConnector* mRobotConnector;
	
	ArFunctor* mStatusFunc;
	ArActionKeydrive mActionKeydrive;
	ArActionJoydrive mActionJoydrive;
	
	// Everything related to ROS
	tf::TransformBroadcaster* mTransformBroadcaster;
	ros::Publisher mOdometryPublisher;
	ros::Publisher mVoltagePublisher;

	std::string mRobotFrame;
	std::string mOdometryFrame;
};

#endif
