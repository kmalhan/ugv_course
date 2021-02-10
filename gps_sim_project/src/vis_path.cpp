/* Visualize vehicle path in Rviz */

#include <ros/ros.h>
#include <ugv_course_libs/gps_conv.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Path.h>

UTMCoords ref_utm;
ros::Publisher pub_path;
nav_msgs::Path path_msg;


void recvFix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	UTMCoords current_utm(*msg);
	tf::Vector3 rel_position = current_utm - ref_utm;

	geometry_msgs::PoseStamped new_pose;
	new_pose.pose.position.x = rel_position.x();
	new_pose.pose.position.y = rel_position.y();
	new_pose.pose.position.z = -0.2;
	new_pose.pose.orientation.w = 1.0;

	path_msg.poses.push_back(new_pose);

	path_msg.header.stamp = msg->header.stamp;
	path_msg.header.frame_id = "/world";
	pub_path.publish(path_msg);
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, "vis_path");
	ros::NodeHandle node;

	/* Calculate ref point */
	double ref_lat, ref_lon;
	node.getParam("/audibot/gps/ref_lat", ref_lat);
	node.getParam("/audibot/gps/ref_lon", ref_lon);
	ref_utm = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));

	ros::Subscriber sub = node.subscribe("/audibot/gps/fix", 1, recvFix);
	pub_path = node.advertise<nav_msgs::Path>("/gps_path", 1);

	ros::spin();
}
