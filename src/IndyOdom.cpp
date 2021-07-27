#include "indy_odom_handler/IndyOdom.hpp"
#include <cmath>


namespace indySLAM {

indyOdom::indyOdom(ros::NodeHandle& nodeHandle) : nh_(nodeHandle)
{	
	if (!readParams()) {
		ROS_ERROR("Could not read parameters.");
		ros::requestShutdown();
	}
	sub_ = nh_.subscribe("/carla/vehicle/086/odometry", 10, &indyOdom::callBack_Odom, this);
	output_pub_ = nh_.advertise<nav_msgs::Odometry>("noisy_odom", 10);
	input_pub_ 	= nh_.advertise<nav_msgs::Odometry>("clean_odom", 10);

	ROS_INFO("Node Successfully Launched");
}

indyOdom::~indyOdom()
{
}


bool indyOdom::readParams(){
	bool success = true;
	success &= nh_.getParam("/IndyOdom/position_std_deviation", position_sd);
	success &= nh_.getParam("/IndyOdom/orientation_std_deviation", orientation_sd);
	return success;
}

void indyOdom::callBack_Odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	// Change code as needed
	indyOdom::addGausianNoise_Odom(msg);
	indyOdom::publishCleanData_Odom();
	indyOdom::publishNoisyData_Odom();
}

void indyOdom::addGausianNoise_Odom(const nav_msgs::Odometry::ConstPtr& msg)
{
	clean_odom_data_ = noisy_odom_data_ = *msg;

	std::random_device rd;
	std::default_random_engine generator;
	std::normal_distribution<double> linear_distribution(0, position_sd), angular_distribution(0, M_PI*orientation_sd/180);

	generator.seed(rd()); 

	noisy_odom_data_.pose.pose.position.x += linear_distribution(generator);
	noisy_odom_data_.pose.pose.position.y += linear_distribution(generator);
	noisy_odom_data_.pose.pose.position.z += linear_distribution(generator);

	noisy_odom_data_.pose.pose.orientation.x += angular_distribution(generator);
	noisy_odom_data_.pose.pose.orientation.y += angular_distribution(generator);
	noisy_odom_data_.pose.pose.orientation.z += angular_distribution(generator);

	// noisy_odom_data_.twist.twist.linear.x += linear_distribution(generator);
	// noisy_odom_data_.twist.twist.linear.y += linear_distribution(generator);
	// noisy_odom_data_.twist.twist.linear.z += linear_distribution(generator);

	// noisy_odom_data_.twist.twist.angular.x += angular_distribution(generator);
	// noisy_odom_data_.twist.twist.angular.y += angular_distribution(generator);
	// noisy_odom_data_.twist.twist.angular.z += angular_distribution(generator);

	//ROS_INFO("Data Succesfully Added");
}

void indyOdom::publishNoisyData_Odom()
{
	output_pub_.publish(noisy_odom_data_);
}

void indyOdom::publishCleanData_Odom()
{
	input_pub_.publish(clean_odom_data_);
}

}	/* namespace */
