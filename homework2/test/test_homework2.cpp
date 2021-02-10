#include <ros/ros.h>
#include <gtest/gtest.h>
#include <std_msgs/Float64.h>
#include <homework2/string_cat.h>
#include "term_color.h"
#include <stdio.h>

ros::NodeHandle* node;
volatile bool received_topic;
double received_value;

inline void test_info(const char* message)
{
  std::cout << term_color::yellow;
  puts(message);
  std::cout << term_color::def;
}

inline void setYellow()
{
  std::cout << term_color::yellow;
}

inline void setDefault()
{
  std::cout << term_color::def;
}

void recvTopic(const std_msgs::Float64::ConstPtr& msg) {
	received_value = msg->data;
	received_topic = true;
}

TEST(Homework2Test, topic_test) {
	ros::WallDuration d(0.1);

	ros::Publisher pub_topic = node->advertise<std_msgs::Float64>("/topic_in",
			0, true);
	ros::Subscriber sub_topic = node->subscribe("/topic_out", 0, recvTopic);

	double test_vals[5] = { 100.0, 4.0, 90.0, 999.0, 2.0 };

	for (int i = 0; i < 5; i++) {
		std_msgs::Float64 test_msg;
		test_msg.data = test_vals[i];

		ros::Time timeout_check = ros::Time::now();
		while (!received_topic) {
			pub_topic.publish(test_msg);
			d.sleep();
			ros::spinOnce();
			if ((ros::Time::now() - timeout_check).toSec() > 1.0) {
				ROS_WARN("Topic receive timeout!");
				break;
			}
		}
		received_topic = false;

    setYellow();
		printf("Input = %f, Output = %f\r\n", test_vals[i], received_value);
    setDefault();
		EXPECT_EQ(test_vals[i] + 5, received_value);
	}
}

TEST(Homework2Test, service_test) {
  std::cout << term_color::yellow;

	homework2::string_cat::Request req;
	homework2::string_cat::Response res;
	req.string_in = "hello";

	ros::ServiceClient srv = node->serviceClient<homework2::string_cat>(
			"/concatenate_string");

  setYellow();
	printf("Calling service with input " "%s" "\r\n",
			req.string_in.c_str());
  setDefault();

	bool successful_service_call = srv.call(req, res);
	ASSERT_TRUE(successful_service_call);

  setYellow();
	printf("Received service response " "%s" "\r\n",
			res.string_out.c_str());
  setDefault();
	EXPECT_STREQ(res.string_out.c_str(),
			(req.string_in + "_with_more_text").c_str());
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	ros::init(argc, argv, "test_homework2");

	node = new ros::NodeHandle();
	received_topic = false;

	ros::Rate r(ros::Duration(0.5));
	r.sleep();

	int result = RUN_ALL_TESTS();
	delete node;

	return 0;
}

