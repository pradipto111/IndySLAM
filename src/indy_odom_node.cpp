#include <ros/ros.h>
#include "indy_odom_handler/IndyOdom.hpp"

int main(int argc, char** argv)
{
	ros::init(argc, argv, "IndyOdom");
	ros::NodeHandle nodeHandle("~");

	indySLAM::indyOdom example(nodeHandle);

	ros::spin();
	return 0;
}