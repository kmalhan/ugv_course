#include <ros/ros.h>
#include <std_msgs/Float64.h>

// Callback function and global variables go here!
ros::Publisher pub_num;				// publisher is global variable

// Callback function
void recvAdd(const std_msgs::Float64ConstPtr& num)
{
	std_msgs::Float64 new_num;		// declare new float64 message
	new_num.data = num->data + 5.0;		// add 5.0 to received message
	pub_num.publish(new_num);		// publish new message
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "pubsub_node");		// Initialize
  ros::NodeHandle node;						// declare node

  /* Code goes here! */
  ros::Subscriber sub_num = node.subscribe("/topic_in", 1, recvAdd);
  pub_num = node.advertise<std_msgs::Float64>("/topic_out", 1);

  ros::spin();
}
