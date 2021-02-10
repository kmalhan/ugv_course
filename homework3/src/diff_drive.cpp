#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

#define WHEEL_RAD 0.2
#define WIDTH 1.0

ros::Publisher pub_left_speed;
ros::Publisher pub_right_speed;

void recvFunc(const geometry_msgs::TwistPtr& msg)
{
	std_msgs::Float64 left_speed;
	std_msgs::Float64 right_speed;

	double left_vel = 1.0 / WHEEL_RAD * ( msg->linear.x - 0.5 * WIDTH * msg->angular.z );
	double right_vel = 1.0 / WHEEL_RAD * ( msg->linear.x + 0.5 * WIDTH * msg->angular.z );

	left_speed.data = left_vel;
	right_speed.data = right_vel;

	pub_left_speed.publish(left_speed);
	pub_right_speed.publish(right_speed);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "diff_dirve");
	ros::NodeHandle n;

	pub_left_speed = n.advertise<std_msgs::Float64>("/roundbot/wheel_controller/left_command", 1);
	pub_right_speed = n.advertise<std_msgs::Float64>("/roundbot/wheel_controller/right_command", 1);

	ros::Subscriber sub = n.subscribe("/cmd_vel", 1, recvFunc);
	ros::spin();
}
