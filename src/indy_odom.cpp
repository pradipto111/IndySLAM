#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <random>

nav_msgs::Odometry msg1;

ros::Publisher odomPublisher;
void odomCallback(const nav_msgs::Odometry::ConstPtr &msg){
	
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,0.06); //0 mean, SD 0.06
	float posx,posy,posz;
	posx = msg->pose.pose.position.x + distribution(generator);
	posy = msg->pose.pose.position.y + distribution(generator);
	posz = msg->pose.pose.position.z + distribution(generator);
	
	float orx,ory,orz;
	orx = msg->pose.pose.orientation.x + distribution(generator);
	ory = msg->pose.pose.orientation.y + distribution(generator);
	orz = msg->pose.pose.orientation.z + distribution(generator);
	
	float lix,liy,liz;
	lix = msg->twist.twist.linear.x + distribution(generator);
	liy = msg->twist.twist.linear.y + distribution(generator);
	liz = msg->twist.twist.linear.z + distribution(generator);
	
	float angx,angy,angz;
	angx = msg->twist.twist.angular.x + distribution(generator);
	angy = msg->twist.twist.angular.y + distribution(generator);
	angz = msg->twist.twist.angular.z + distribution(generator);
	
	
	msg1.pose.pose.position.x = posx;
	msg1.pose.pose.position.y = posy;
	msg1.pose.pose.position.z = posz;
	
	msg1.pose.pose.orientation.x = orx;
	msg1.pose.pose.orientation.y = ory;
	msg1.pose.pose.orientation.z = orz;
	
	msg1.twist.twist.linear.x = lix;
	msg1.twist.twist.linear.y = liy;
	msg1.twist.twist.linear.z = liz;
	
	msg1.twist.twist.angular.x = angx;
	msg1.twist.twist.angular.y = angy;
	msg1.twist.twist.angular.z = angz;
	ROS_INFO_STREAM_THROTTLE(1,"OK");
	
	msg1.header = msg->header;
	msg1.child_frame_id = msg1->child_frame_id;
	odomPublisher.publish(msg1);	
	
}


int main(int argc, char** argv){
 	ros::init(argc, argv, "indy_odom");
	ros::NodeHandle nh;
	//indy_odom::indyOdom indyOdom_(nh);
	odomPublisher = nh.advertise<nav_msgs::Odometry>("/noisy_odom",10);
	ros::Subscriber odomSubscriber = nh.subscribe("/carla/vehicle/086/odometry", 10, odomCallback);
	
	//odomPublisher.publish(msg1);
	ros::spin();
 	return 0;
}
