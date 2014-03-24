#include <aria_ros/AriaRobot.h>

#include <math.h>
#include <vector>

// Define sounds to use with the pioneer's buzzer.
// Format is: {duration,tone1,duration,tone2,...}, where 0 means silence
char sound1[] = {40,70,20,0,40,76,20,0,40,82};
char sound2[] = {20,70,2,0,40,76};
char sound3[] = {20,76,2,0,40,70};

AriaRobot::AriaRobot(int argc, char **argv)
{
	ros::NodeHandle node;
	mOdometryPublisher = node.advertise<nav_msgs::Odometry>("odom",50);
	mVoltagePublisher = node.advertise<std_msgs::Float32>("battery_state", 10);

	node.param("base_frame", mRobotFrame, std::string("base_link"));
	node.param("odometry_frame", mOdometryFrame, std::string("odom"));

	// Initialize some global data
	ROS_INFO("Now initializing Aria...");
	Aria::init();

	// If you want ArLog to print "Verbose" level messages uncomment this:
	//ArLog::init(ArLog::StdOut, ArLog::Verbose);

	// This object parses program options from the command line
	mArgumentParser = new ArArgumentParser(&argc, argv);

	// Load some default values for command line arguments from /etc/Aria.args
	// (Linux) or the ARIAARGS environment variable.
	mArgumentParser->loadDefaultArguments();

	// Central object that is an interface to the robot and its integrated
	// devices, and which manages control of the robot by the rest of the program.
	mRobot = new ArRobot();

	// Object that connects to the robot or simulator using program options
	mRobotConnector = new ArRobotConnector(mArgumentParser, mRobot);

	// Connect to the robot, get some initial data from it such as type and name,
	// and then load parameter files for this robot.
	if (!mRobotConnector->connectRobot())
	{
		// Error connecting:
		// if the user gave the -help argumentp, then just print out what happened,
		// and continue so options can be displayed later.
		if (!mArgumentParser->checkHelpAndWarnUnparsed())
		{
			ArLog::log(ArLog::Terse, "Could not connect to robot, will not have parameter file so options displayed later may not include everything");
		}
		// otherwise abort
		else
		{
			ArLog::log(ArLog::Terse, "Error, could not connect to robot.");
			Aria::logOptions();
			Aria::exit(1);
		}
	}

	// Parse the command line options. Fail and print the help message if the parsing fails
	// or if the help was requested with the -help option
	if (!Aria::parseArgs() || !mArgumentParser->checkHelpAndWarnUnparsed())
	{    
		Aria::logOptions();
		Aria::exit(1);
	}

	// Initialize some remaining members
	mTransformBroadcaster = new tf::TransformBroadcaster();
	mStatusFunc = new ArConstFunctorC<AriaRobot>(this, &AriaRobot::publishStatus);
	mRobot->addSensorInterpTask("PublishStatus", 50, mStatusFunc);
	mRobot->enableMotors();
	mActionJoydrive.setSpeeds(1200,30);
	
	// Start the robot task loop running in a new background thread. The 'true' argument means if it loses
	// connection the task loop stops and the thread exits.
	mRobot->runAsync(true);

	// Get some parameters from the robot
	ROS_INFO("Robot '%s' of type %s (%s)",mRobot->getName(), mRobot->getRobotType(), mRobot->getRobotSubType());
	ROS_INFO("Translation - Max: %f, Acceleration: %f, Deceleration: %f", mRobot->getTransVelMax(), mRobot->getTransAccel(), mRobot->getTransDecel());
	ROS_INFO("Rotation    - Max: %f, Acceleration: %f, Deceleration: %f", mRobot->getRotVelMax(), mRobot->getRotAccel(), mRobot->getRotDecel());
	

	// Sleep for a second so some messages from the initial responses
	// from robots and cameras and such can catch up
	ArUtil::sleep(1000);

	bool joy;
	node.param("activate_joy_drive", joy, false);
	if(joy)
	{
		mRobot->addAction(&mActionJoydrive,50);
	}

	ROS_INFO("Finished initialization of AriaRobot.");
}

AriaRobot::~AriaRobot()
{
	mRobot->stopRunning();	
	mRobot->waitForRunExit();
	Aria::shutdown();
	
	// Tidy up
	delete mRobot;
	delete mArgumentParser;
	delete mRobotConnector;
	delete mStatusFunc;
	delete mTransformBroadcaster;
}

void AriaRobot::publishStatus() const
{
	ros::Time now = ros::Time::now();
	tf::Vector3 v;
	v.setX(mRobot->getX()/1000);
	v.setY(mRobot->getY()/1000);
	v.setZ(0.0);
	tf::Quaternion q = tf::createQuaternionFromYaw(mRobot->getTh()/180*PI);

	// Publish odometry via TF
	tf::Transform transform;
	transform.setOrigin(v);
	transform.setRotation(q);
	mTransformBroadcaster->sendTransform(tf::StampedTransform(transform, now, mOdometryFrame, mRobotFrame));

	// Publish via Odometry message
	nav_msgs::Odometry odom;
	odom.header.stamp = now;
	odom.header.frame_id = mOdometryFrame.c_str();

	odom.pose.pose.position.x = mRobot->getX()/1000;
	odom.pose.pose.position.y = mRobot->getY()/1000;
	odom.pose.pose.position.z = 0.0;
	odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(mRobot->getTh()/180*PI);

	odom.child_frame_id = mRobotFrame.c_str();
	odom.twist.twist.linear.x = mRobot->getVel()/1000;
	odom.twist.twist.linear.y = 0;
	odom.twist.twist.linear.z = 0;
	odom.twist.twist.angular.z = mRobot->getRotVel()/180*PI;
	
	mOdometryPublisher.publish(odom);

	//Publish Voltage of Pioneer
	std_msgs::Float32 voltage;
	voltage.data = mRobot->getBatteryVoltage();
	mVoltagePublisher.publish(voltage);
}

void AriaRobot::setVelocity(const geometry_msgs::Twist::ConstPtr& msg)
{
	mRobot->setVel(msg->linear.x * 1000);
	mRobot->setRotVel(msg->angular.z * 60.0); 
}

void AriaRobot::say(const std_msgs::Int8::ConstPtr& msg)
{
	int size = 0;
	char* sound = 0;
	switch(msg->data)
	{
	case 1:
		sound = sound1;
		size = sizeof(sound1);
		break;
		
	case 2:
		sound = sound2;
		size = sizeof(sound2);
		break;
	
	case 3:
		sound = sound3;
		size = sizeof(sound3);
		break;
	}
	mRobot->comStrN(15, sound, size); 
}

