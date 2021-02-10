#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64MultiArray.h>

#define WHEEL_RAD 0.15
#define WIDTH 0.5
#define LENGTH 0.5

ros::Publisher pub;

void recvFunc(const geometry_msgs::TwistConstPtr& msg)
{
	std_msgs::Float64MultiArray vel;
	vel.data.resize(4);

	double a = 1.0 / WHEEL_RAD;
	double b = 1.0 / WHEEL_RAD;
	double c = 0.5 * (WIDTH + LENGTH) / WHEEL_RAD;

	double front_left = (a * msg->linear.x) + (-b * msg->linear.y) + (-c * msg->angular.z);
	double front_right = (a * msg->linear.x) + (b * msg->linear.y) + (c * msg->angular.z);
	double rear_left = (a * msg->linear.x) + (b * msg->linear.y) + (-c * msg->angular.z);
	double rear_right = (a * msg->linear.x) + (-b * msg->linear.y) + (c * msg->angular.z);

	vel.data[0] = front_left;
	vel.data[1] = front_right;
	vel.data[2] = rear_left;
	vel.data[3] = rear_right;

	pub.publish(vel);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "mecanum_drive");
	ros::NodeHandle node;

	pub = node.advertise<std_msgs::Float64MultiArray>("/omnibot/omni_wheel_speed_controller/wheel_commands", 1);

	ros::Subscriber sub = node.subscribe("/cmd_vel", 1, recvFunc);

	ros::spin();
}
