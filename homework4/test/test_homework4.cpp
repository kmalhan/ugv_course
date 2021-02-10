#include <stdio.h>
#include <ros/ros.h>
#include <gtest/gtest.h>
#include <dynamic_reconfigure/Config.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>

#include "term_color.h"

visualization_msgs::MarkerArray marker_array_msg;
bool received_marker_array = false;
ros::Publisher pub_reconfig;
tf::TransformListener* listener;
ros::ServiceClient reconfig_client;

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

TEST(Homework4Test, marker_message_test)
{
  while (ros::ok() && !received_marker_array){}

  test_info("Checking marker array message...");
  test_info("Making sure there are three markers in array:");
  ASSERT_EQ(marker_array_msg.markers.size(), 3);

//  test_info("Making sure each marker ID is different");
  test_info("Making sure each marker ID is different");
  EXPECT_NE(marker_array_msg.markers[0].id, marker_array_msg.markers[1].id);
  EXPECT_NE(marker_array_msg.markers[0].id, marker_array_msg.markers[2].id);
  EXPECT_NE(marker_array_msg.markers[1].id, marker_array_msg.markers[2].id);

  for (int i=0; i<3; i++){
    setYellow();
    printf("Checking marker %d...\r\n", i);
    setDefault();

    test_info("  Time stamp less than half a second old");
    EXPECT_LE((ros::Time::now() - marker_array_msg.markers[i].header.stamp).toSec(), 0.5);
    test_info("  Frame ID set to 'marker'");
    EXPECT_STREQ(marker_array_msg.markers[i].header.frame_id.c_str(), "marker");
    test_info("  Action set to 'ADD'");
    EXPECT_EQ(marker_array_msg.markers[i].action, visualization_msgs::Marker::ADD);
    test_info("  Type set to 'CUBE'");
    EXPECT_EQ(marker_array_msg.markers[i].type, visualization_msgs::Marker::CUBE);
    test_info("  Scale set to 1.0 on all dimensions");
    EXPECT_EQ(marker_array_msg.markers[i].scale.x, 1.0);
    EXPECT_EQ(marker_array_msg.markers[i].scale.y, 1.0);
    EXPECT_EQ(marker_array_msg.markers[i].scale.z, 1.0);
    test_info("  Zero rotation");
    EXPECT_EQ(marker_array_msg.markers[i].pose.orientation.w, 1.0);
    EXPECT_EQ(marker_array_msg.markers[i].pose.orientation.x, 0.0);
    EXPECT_EQ(marker_array_msg.markers[i].pose.orientation.y, 0.0);
    EXPECT_EQ(marker_array_msg.markers[i].pose.orientation.z, 0.0);
    test_info("  Transparency set to 1.0");
    EXPECT_EQ(marker_array_msg.markers[i].color.a, 1.0);

    switch (i){
      case 0:
        test_info("  Color is yellow (r=1, g=1, b=0)");
        EXPECT_EQ(marker_array_msg.markers[i].color.r, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.g, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.b, 0.0);

        test_info("  Position is (0, 0, 0)");
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.x, 0.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.y, 0.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.z, 0.0);
        break;
      case 1:
        test_info("  Color is cyan (r=0, g=1, b=1)");
        EXPECT_EQ(marker_array_msg.markers[i].color.r, 0.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.g, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.b, 1.0);

        test_info("  Position is (1, 1, 1)");
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.x, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.y, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.z, 1.0);
        break;
      case 2:
        test_info("  Color is magenta (r=1, g=0, b=1)");
        EXPECT_EQ(marker_array_msg.markers[i].color.r, 1.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.g, 0.0);
        EXPECT_EQ(marker_array_msg.markers[i].color.b, 1.0);

        test_info("  Position is (2, 2, 2)");
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.x, 2.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.y, 2.0);
        EXPECT_EQ(marker_array_msg.markers[i].pose.position.z, 2.0);
        break;
    }
  }
}

enum{X, Y, Z, ROLL, PITCH, YAW, NUM_VALS};

typedef struct{
  double r;
  double p;
  double y;
} RpyStruct;

TEST(Homework4Test, tf_transform_test)
{
  std::cout << term_color::yellow;
  test_info("Modifying GUI parameters to test TF transform...");

  std::vector<tf::Vector3> translations;
  std::vector<RpyStruct> rpy;

  translations.resize(4);
  rpy.resize(4);

  translations[0] = tf::Vector3(3, -4, 1);
  rpy[0].r = 0;
  rpy[0].p = 0;
  rpy[0].y = 0;

  translations[1] = tf::Vector3(0, 0, 3);
  rpy[1].r = 0.707;
  rpy[1].p = 0;
  rpy[1].y = 0;

  translations[2] = tf::Vector3(0, 3, 0);
  rpy[2].r = 0;
  rpy[2].p = 1.57;
  rpy[2].y = 0;

  translations[3] = tf::Vector3(4, 0, 0);
  rpy[3].r = -0.5;
  rpy[3].p = 0.7;
  rpy[3].y = 2.0;

  for (int i=0; i<4; i++){
    setYellow();
    printf("Setting translation to (%f, %f, %f), RPY to (%f, %f, %f)\r\n", translations[i].x(), translations[i].y(),
             translations[i].z(), rpy[i].r, rpy[i].p, rpy[i].y);
    setDefault();

    dynamic_reconfigure::Config cfg;
    cfg.doubles.resize(NUM_VALS);
    cfg.doubles[X].name = "x";
    cfg.doubles[X].value = translations[i].x();
    cfg.doubles[Y].name = "y";
    cfg.doubles[Y].value = translations[i].y();
    cfg.doubles[Z].name = "z";
    cfg.doubles[Z].value = translations[i].z();
    cfg.doubles[ROLL].name = "roll";
    cfg.doubles[ROLL].value = rpy[i].r;
    cfg.doubles[PITCH].name = "pitch";
    cfg.doubles[PITCH].value = rpy[i].p;
    cfg.doubles[YAW].name = "yaw";
    cfg.doubles[YAW].value = rpy[i].y;

    dynamic_reconfigure::ReconfigureRequest req;
    dynamic_reconfigure::ReconfigureResponse res;
    req.config = cfg;
    ASSERT_TRUE(reconfig_client.call(req, res));

    ros::Duration(0.1).sleep();

    test_info("  Looking up TF transform");
    tf::StampedTransform transform;
    try {
      listener->lookupTransform("map", "marker", marker_array_msg.markers[0].header.stamp, transform);
    } catch (tf::TransformException &ex) {
      ROS_ERROR("%s", ex.what());
      return;
    }

    test_info("  Checking origin of transform");
    EXPECT_FLOAT_EQ(translations[i].x(), transform.getOrigin().x());
    EXPECT_FLOAT_EQ(translations[i].y(), transform.getOrigin().y());
    EXPECT_FLOAT_EQ(translations[i].z(), transform.getOrigin().z());

    test_info("  Checking rotation of transform");
    tf::Quaternion rotation(tf::createQuaternionFromRPY(rpy[i].r, rpy[i].p, rpy[i].y));
    EXPECT_FLOAT_EQ(rotation.w(), transform.getRotation().w());
    EXPECT_FLOAT_EQ(rotation.x(), transform.getRotation().x());
    EXPECT_FLOAT_EQ(rotation.y(), transform.getRotation().y());
    EXPECT_FLOAT_EQ(rotation.z(), transform.getRotation().z());

    ros::Duration(3.0).sleep();
  }
}

void recvMarkerArray(const visualization_msgs::MarkerArray::ConstPtr& msg)
{
  marker_array_msg = *msg;
  received_marker_array = true;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "test_homework4");
  ros::NodeHandle n;

  ros::Subscriber sub_marker_array = n.subscribe("marker_array", 1, recvMarkerArray);
  reconfig_client = n.serviceClient<dynamic_reconfigure::Reconfigure>("/homework4/set_parameters");

  listener = new tf::TransformListener;

  ros::AsyncSpinner spinner(3);
  spinner.start();

  int result = RUN_ALL_TESTS();

  return 0;
}
