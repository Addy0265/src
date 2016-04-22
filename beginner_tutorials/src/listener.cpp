#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

void listen(const std_msgs::String::ConstPtr& msg)
{
	ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "listener");

	ros::NodeHandle n;

	ros::Subscriber chatter_sub = n.subscribe("chatter", 1000, listen);

	ros::spin();

	return 0;
}