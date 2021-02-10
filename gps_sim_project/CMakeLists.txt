cmake_minimum_required(VERSION 2.8.3)
project(gps_sim_project)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  ugv_course_libs
  tf
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(waypoint_marker src/waypoint_marker.cpp)
target_link_libraries(waypoint_marker
  ${catkin_LIBRARIES}
)

add_executable(vis_path src/vis_path.cpp)
target_link_libraries(vis_path
  ${catkin_LIBRARIES}
)

add_executable(control src/control.cpp)
target_link_libraries(control
  ${catkin_LIBRARIES}
)

#################
## Test Script ##
#################
add_executable(test_gps_sim_project test/test_gps_project.cpp)
target_link_libraries(test_gps_sim_project ${catkin_LIBRARIES} ${GTEST_LIBRARIES})
