#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <dynamic_reconfigure/server.h>
#include <homework4/MarkerConfig.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

using namespace visualization_msgs;

ros::Publisher pub_marker;
MarkerArray marker_msg;
tf::StampedTransform transform;

void reconfigure(homework4::MarkerConfig& config, uint32_t level)
{
	transform.setOrigin(tf::Vector3(config.x, config.y, config.z));
	transform.setRotation(tf::createQuaternionFromRPY(config.roll, config.pitch, config.yaw));
}

void timerCallback(const ros::TimerEvent& event)
{
	static tf::TransformBroadcaster broadcaster;
	transform.stamp_ = event.current_real;
	broadcaster.sendTransform(transform);

	// Update Timestamp of each component of MarkerArray
	for (int i=0; i<marker_msg.markers.size(); i++){
		marker_msg.markers[i].header.stamp = event.current_real;
	}

	pub_marker.publish(marker_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "homework4");
	ros::NodeHandle node;

	// advertise
	pub_marker = node.advertise<visualization_msgs::MarkerArray>("marker_array", 1);

	// dynamic reconfigure
	dynamic_reconfigure::Server<homework4::MarkerConfig> srv;
	srv.setCallback(boost::bind(reconfigure, _1, _2));

	// Timer
	ros::Timer marker_timer = node.createTimer(ros::Duration(0.05), timerCallback);

	// Common Marker settings
	Marker new_marker;
	new_marker.header.frame_id = "marker";
	new_marker.action = Marker::ADD;
	new_marker.color.a = 1.0;
	new_marker.id = 0;
	new_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	new_marker.type = Marker::CUBE;
	new_marker.scale.x = 1.0;
	new_marker.scale.y = 1.0;
	new_marker.scale.z = 1.0;

	// Yellow cube at (0, 0, 0)
	new_marker.color.r = 1.0;
	new_marker.color.g = 1.0;
	new_marker.color.b = 0.0;
	new_marker.pose.position.x = 0.0;
	new_marker.pose.position.y = 0.0;
	new_marker.pose.position.z = 0.0;

	marker_msg.markers.push_back(new_marker);
	new_marker.id++;

	// Cyan cube at (1, 1, 1)
	new_marker.color.r = 0.0;
	new_marker.color.g = 1.0;
	new_marker.color.b = 1.0;
	new_marker.pose.position.x = 1.0;
	new_marker.pose.position.y = 1.0;
	new_marker.pose.position.z = 1.0;

	marker_msg.markers.push_back(new_marker);
	new_marker.id++;

	// Magenta cube at (2, 2, 2)
	new_marker.color.r = 1.0;
	new_marker.color.g = 0.0;
	new_marker.color.b = 1.0;
	new_marker.pose.position.x = 2.0;
	new_marker.pose.position.y = 2.0;
	new_marker.pose.position.z = 2.0;

	marker_msg.markers.push_back(new_marker);
	new_marker.id++;

	// tf transform settings
	transform.frame_id_ = "map";
	transform.child_frame_id_ = "marker";

	ros::spin();
}
