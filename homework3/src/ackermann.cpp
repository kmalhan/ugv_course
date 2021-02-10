#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Twist.h>

#define LENGTH 3.02367
#define RATIO 16.0

ros::Publisher pub_speed;
ros::Publisher pub_angle;

void recvFunc(const geometry_msgs::TwistPtr& msg)
{
	std_msgs::Float64 speed;
	std_msgs::Float64 angle;

	speed.data = msg->linear.x;

	if (msg->linear.x == 0.0){
		angle.data = 0.0;
	}
	else {
		angle.data  = RATIO * atan(LENGTH * msg->angular.z / msg->linear.x);
	}

	pub_speed.publish(speed);
	pub_angle.publish(angle);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "ackermann");
	ros::NodeHandle n;

	pub_speed = n.advertise<std_msgs::Float64>("/audibot/audi_speed_controller/speed_cmd", 1);
	pub_angle = n.advertise<std_msgs::Float64>("/audibot/audi_steering_controller/steering_cmd", 1);

	ros::Subscriber sub = n.subscribe("/cmd_vel", 1, recvFunc);
	ros::spin();
}
