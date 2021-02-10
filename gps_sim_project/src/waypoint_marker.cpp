/* Publish GPS waypoint as Marker Array */

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <ugv_course_libs/gps_conv.h>

using namespace visualization_msgs;

ros::Publisher pub_waypoint;
MarkerArray waypoint;

std::vector<double> waypoint_lat;
std::vector<double> waypoint_lon;
std::vector<tf::Vector3> local_waypoint;
UTMCoords ref_point;

void timerCallback(const ros::TimerEvent& event)
{
	for (int i = 0; i < waypoint.markers.size(); i++){
		waypoint.markers[i].header.stamp = event.current_real;
	}
	pub_waypoint.publish(waypoint);
}

void initializeWaypoint()
{
	waypoint_lat.resize(8);
	waypoint_lat[0] = 42.851358;
	waypoint_lat[1] = 42.851383;
	waypoint_lat[2] = 42.852443;
	waypoint_lat[3] = 42.852021;
	waypoint_lat[4] = 42.851525;
	waypoint_lat[5] = 42.851344;
	waypoint_lat[6] = 42.850836;
	waypoint_lat[7] = 42.849644;
	waypoint_lon.resize(8);
	waypoint_lon[0] = -83.069485;
	waypoint_lon[1] = -83.069007;
	waypoint_lon[2] = -83.068013;
	waypoint_lon[3] = -83.066888;
	waypoint_lon[4] = -83.067044;
	waypoint_lon[5] = -83.066344;
	waypoint_lon[6] = -83.066440;
	waypoint_lon[7] = -83.066060;

	local_waypoint.resize(8);
	for (int i = 0; i < 8; i++) {
	  UTMCoords waypoint(LatLon(waypoint_lat[i], waypoint_lon[i], 0.0));
	  local_waypoint[i] = waypoint - ref_point;
	}
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "waypoint_marker");
	ros::NodeHandle node;

	ros::Timer marker_timer = node.createTimer(ros::Duration(0.05), timerCallback);
	pub_waypoint = node.advertise<visualization_msgs::MarkerArray>("/waypoint", 1);

	/* Initialize waypoint based on reference point */
	double ref_lat, ref_lon;
	node.param("/audibot/gps/ref_lat", ref_lat, 0.0);
	node.param("/audibot/gps/ref_lon", ref_lon, 0.0);
	ref_point = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));
	//ROS_INFO("reg gps: %f, %f\n", ref_lat, ref_lon);
	initializeWaypoint();

	/* Create Marker */
	Marker new_marker;
	new_marker.header.frame_id = "/world";
	new_marker.action = Marker::ADD;
	new_marker.color.a = 1.0;
	new_marker.id = 0;
	new_marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
	new_marker.color.r = 0.0;
	new_marker.color.g = 1.0;
	new_marker.color.b = 0.0;
	new_marker.type = Marker::CYLINDER;
	new_marker.scale.x = 2.0;
	new_marker.scale.y = 2.0;
	new_marker.scale.z = 0.3;
	new_marker.pose.position.z = 0.0;

	// add all points
	for (int i = 0; i < 8; i++){
		new_marker.pose.position.x = local_waypoint[i].x();
		new_marker.pose.position.y = local_waypoint[i].y();
		waypoint.markers.push_back(new_marker);
		new_marker.id++;
		//ROS_INFO("gps points: %f, %f", local_waypoint[i].x(), local_waypoint[i].y());
	}


	ros::spin();
}
