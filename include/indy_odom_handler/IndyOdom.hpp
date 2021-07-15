#pragma once

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>

namespace indySLAM {

class indyOdom
{

public:
	indyOdom(ros::NodeHandle& nodeHandle);
	virtual ~indyOdom();

private:
	// Generic callback function
	void callBack_Odom(const nav_msgs::Odometry::ConstPtr& msg);
	// Adds Gaussian Noise
	void addGausianNoise_Odom(const nav_msgs::Odometry::ConstPtr &msg);
	// Publish Noisy Odometry Data
	void publishNoisyData_Odom();
	void publishCleanData_Odom();

	// ROS Node Handler
	ros::NodeHandle &nh_;
	// ROS Subscriber
	ros::Subscriber sub_;
	// ROS Publisher
	ros::Publisher output_pub_;
	ros::Publisher input_pub_;

	// Data with added noise
	nav_msgs::Odometry noisy_odom_data_;
	nav_msgs::Odometry clean_odom_data_;

}; /* class */

} /* namespace */