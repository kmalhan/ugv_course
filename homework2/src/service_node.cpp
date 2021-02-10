#include <ros/ros.h>
#include <homework2/string_cat.h>

// Callback function goes here!

bool srvCallback(homework2::string_cat::Request& req,
				 homework2::string_cat::Response& res)
{
	res.string_out = req.string_in + "_with_more_text";
	return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "service_node");
  ros::NodeHandle node;

  /* Code goes here! */
  ros::ServiceServer srv = node.advertiseService("/concatenate_string", srvCallback);

  ros::spin();
}
