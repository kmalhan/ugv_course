/******* Vehicle Control Logic *******/
#include <ros/ros.h>
#include <ugv_course_libs/gps_conv.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/TwistStamped.h>

#define LENGTH 3.02367

/******* Global variables *******/
ros::Publisher pub_cmd;
double vehicle_heading;						// audibot vehicle heading in radians (GPS frame)

UTMCoords ref_utm;							// reference point in UTM
tf::Vector3 vehicle_position;				// Relative audibot position
std::vector<tf::Vector3> local_waypoint;	// Relative GPS waypoint position
geometry_msgs::Twist cmd_vel;				// Twist message to control audibot

double act_latAcc;							// Actual lateral acceleration
double accLimit;							// Lateral Acceleration Limit
double act_speed;							// Actual Speed
double act_yaw;								// Actual yawrate

/******* Callback when GPS fix is received *******/
void recvFix(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	UTMCoords current_utm(*msg);
	vehicle_position = current_utm - ref_utm;
}

/******* Callback when vehicle heading is received *******/
void recvHeading(const std_msgs::Float64ConstPtr& num)
{
	/*** Convert standard heading into GPS frame ***/
	double heading;
	heading = 450.0 - num->data;
	if (heading > 360.0){
		heading = heading - 360.0;
	}
	/*** Convert from degree to radians ***/
	vehicle_heading = heading * M_PI / 180.0;
}

/****** Callback when actual vechile speed and yawrate are received *******/
void recvBot(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
	act_speed = msg->twist.linear.x;
	act_yaw = msg->twist.angular.z;
	/* Lateral Acceleration */
	act_latAcc = act_speed * fabs(act_yaw);
}

/******* Timer Callbacl (main functionality) *******/
void timerCallback(const ros::TimerEvent& event)
{
	static int index = 0;			// Current index of target GPS point
	/* for GPS point switch */
	double y_diff;					// GPS difference in y direction
	double x_diff;					// GPS difference in x direction
	double distance;				// Distance between waypoint and audibot

	double target_heading;			// target heading in radians
	double heading_err;				// Heading error
	static bool stop = false;		// TRUE if vehicle must stop
	double distancePrev;			// Distance to Previous GPS point
	static double GPSgap;			// Distance between target and next GPS point.
	static double GPSgapPrev;		// Prev

	double VehicleSpeed;			// Vehicle Speed
	double VehicleYaw;				// Vehicle Yaw Rate

	/******** Change to next GPS waypoint when audibot reach target *************/
	if (index < local_waypoint.size()){
		y_diff = local_waypoint[index].y()-vehicle_position.y();
		x_diff = local_waypoint[index].x()-vehicle_position.x();
		distance = hypot(y_diff, x_diff);

		if (distance <= 1.0){
			/* Audibot has reached the waypoint */
			if (index == local_waypoint.size()-1){
				stop = true;
			} else {
				index++;
				// Update Next GPS gap to Current GPS gap.
				GPSgapPrev = GPSgap;
			}
		}
	}

/******** Calculate target heading *****************************************/
	y_diff = local_waypoint[index].y() - vehicle_position.y();
	x_diff = local_waypoint[index].x() - vehicle_position.x();

	target_heading = atan2(y_diff,x_diff);					// Radians
	target_heading = (target_heading + (2.0 * M_PI));
	target_heading = fmod(target_heading, (2.0 *M_PI));

/******** Calculate Heading Error ******************************************/
	heading_err = target_heading - vehicle_heading;

	/* Make sure to have error < 180 */
	if (fabs(heading_err) >= (M_PI)){
		if (heading_err > 0){
			heading_err = heading_err - (2.0 * M_PI);
		} else {
			heading_err = (2.0 * M_PI) + heading_err;
		}
	}

/************** Simple Speed profile based on distance *************************/
	/* Distance between vehicle and target GPS */
	distance = hypot(y_diff, x_diff);
	/* Distance between vehicle and previous GPS */
	if (index > 0){
		y_diff = local_waypoint[index-1].y() - vehicle_position.y();
		x_diff = local_waypoint[index-1].x() - vehicle_position.x();
		distancePrev = hypot(y_diff, x_diff);
	} else {
		distancePrev = 100.0;
	}
	/* Distance between target GPS and Next target GPS */
	if (index + 1 < local_waypoint.size()) {
		y_diff = local_waypoint[index+1].y() - local_waypoint[index].y();
		x_diff = local_waypoint[index+1].x() - local_waypoint[index].x();
		GPSgap = hypot(y_diff, x_diff);
	} else {
		GPSgap = 1000.0;
	}

	//////////////////////////////////////////////////////////////////////////////////////////

	/*********       Vehicle Speed Control Logic         **********/
	// Defalut Speed is 10m/s
	VehicleSpeed = 10.0;

	// If the Next GPS point is greater than 200m
	if (distance >= 200.0){
		if (fabs(heading_err) < (M_PI / 12)) {
			VehicleSpeed = 30.0;
		} else if (fabs(heading_err) < (M_PI / 8)){
			VehicleSpeed = 20.0;
		} else {
			VehicleSpeed = 10.0;
		}
	}
	// If the Next GPS point is greater than 100m
	else if (distance >= 100.0){
		if (fabs(heading_err) < (M_PI / 12)) {
			VehicleSpeed = 30.0;
		} else if (fabs(heading_err) < (M_PI / 8)){
			VehicleSpeed = 25.0;
		} else {
			VehicleSpeed = 15.0;
		}
	}

	// Speed profile for Moderate distance till GPS point (30m)
	else if (distance >= 30.0){
		if (fabs(heading_err) < (M_PI / 8)){
			VehicleSpeed = 25.0;
		} else if (fabs(heading_err) < (M_PI / 4)){
			VehicleSpeed = 20.0;
		} else {
			VehicleSpeed = 10.0;
		}
	}

	// Speed Control for close distance
	else if (distance > 0){
		if (GPSgap >= 70){
			VehicleSpeed = 20.0;
		} else if (GPSgap >= 50.0){
			VehicleSpeed = 15.0;
		} else {
			VehicleSpeed = 10.0;
		}
	}

	// Speed up for last GPS point
	if (index == local_waypoint.size() - 1){
		VehicleSpeed = 35.0;
	}

	// Overwrite VehicleSpeed for Just After passing GPS point
	if (distancePrev <= 20.0){
		if (GPSgapPrev >= 65.0){
			VehicleSpeed = 15.0;
		} else {
			VehicleSpeed = 9.0;
		}
	}

	/*******       Increase YawRate if vehicle is at lower speed     *********/
	VehicleYaw = tan(heading_err) * VehicleSpeed / LENGTH;

	// Make sure Command doesn't exceed roll over limit
	// If close to target, reduce speed, otherwise, reduce Yawrate
	if ( VehicleSpeed * fabs(VehicleYaw) >= accLimit && GPSgapPrev >= 55.0){
		if (distance <= 40.0){
			VehicleSpeed = (accLimit - 1) / fabs(VehicleYaw);
		} else {
			VehicleYaw = (accLimit - 1) / VehicleSpeed;
			if (heading_err < 0){
				VehicleYaw = -1.0 * VehicleYaw;
			}
		}
	}

	// Check the Current Lateral Acceleration.
	if (act_latAcc >= accLimit){
		if (distance <= 40.0) {
			VehicleSpeed = (accLimit - 1) / fabs(VehicleYaw);
		} else {	//if (distancePrev <= 50.0){
			VehicleYaw = (accLimit - 1) / VehicleSpeed;
			if (heading_err < 0){
				VehicleYaw = -1.0 * VehicleYaw;
			}
		}
	}

/************ Audibot has reached the last waypoint, STOP! ********************/
	if (stop){
		VehicleSpeed = 0.0;
		VehicleYaw = 0.0;
	}

/************* Publish audibot velocity and yawrate **************************/
	cmd_vel.linear.x = VehicleSpeed;
	cmd_vel.angular.z = VehicleYaw;
	pub_cmd.publish(cmd_vel);

}

/* Initialize GPS waypoint with reference point */
void initializeWaypoint()
{
	std::vector<double> waypoint_lat;
	std::vector<double> waypoint_lon;

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
	  local_waypoint[i] = waypoint - ref_utm;
	}
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "control");
	ros::NodeHandle node;

	/* Get reference GPS coordinate */
	double ref_lat, ref_lon;
	node.getParam("/audibot/gps/ref_lat", ref_lat);
	node.getParam("/audibot/gps/ref_lon", ref_lon);
	ref_utm = UTMCoords(LatLon(ref_lat, ref_lon, 0.0));

	initializeWaypoint();

	node.param("/control/LatAccLimit", accLimit, 13.0);

	pub_cmd = node.advertise<geometry_msgs::Twist>("/audibot/cmd_vel", 1);

	ros::Subscriber sub_gps = node.subscribe("/audibot/gps/fix", 1, recvFix);
	ros::Subscriber sub_head = node.subscribe("/audibot/gps/heading", 1, recvHeading);
	ros::Subscriber sub_bot = node.subscribe("/audibot/twist", 1, recvBot);

	ros::Timer timer = node.createTimer(ros::Duration(0.02), timerCallback);	// 50 Hz

	ros::spin();
}
