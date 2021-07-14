#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>
#include <cmath>
nav_msgs::Odometry msg1;

ros::Publisher odom_noisy_publisher;
ros::Publisher odom_publisher;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
	
	std::random_device rd;
	std::default_random_engine generator;
        generator.seed( rd() ); 
        std::normal_distribution<double> distribution(0,0.06);
            
	std::default_random_engine generator1;
        generator1.seed( rd() );
        std::normal_distribution<double> distribution1(0,4*(M_PI)/180);
	msg1.pose.pose.position = msg->pose.pose.position;
	msg1.pose.pose.position.x += distribution(generator);
	msg1.pose.pose.position.y += distribution(generator);
	msg1.pose.pose.position.z += distribution(generator);
	
	msg1.pose.pose.orientation = msg->pose.pose.orientation;
	msg1.pose.pose.orientation.x += distribution1(generator1);
	msg1.pose.pose.orientation.y += distribution1(generator1);
	msg1.pose.pose.orientation.z += distribution1(generator1);
	
	msg1.twist = msg->twist;
	msg1.header = msg->header;
	msg1.child_frame_id = msg->child_frame_id;
	odom_noisy_publisher.publish(msg1);	
	odom_publisher.publish(*msg);
}


int main(int argc, char** argv){
 	ros::init(argc, argv, "indy_odom");
	ros::NodeHandle nh;
	//indy_odom::indyOdom indyOdom_(nh);
	odom_noisy_publisher = nh.advertise<nav_msgs::Odometry>("/noisy_odom",100);
	odom_publisher = nh.advertise<nav_msgs::Odometry>("/odom",100);
	ros::Subscriber odomSubscriber = nh.subscribe("/carla/vehicle/086/odometry", 1000, odomCallback);
	
	//odomPublisher.publish(msg1);
	ros::spin();
 	return 0;
}
