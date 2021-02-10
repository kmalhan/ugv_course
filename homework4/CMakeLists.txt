cmake_minimum_required(VERSION 2.8.3)
project(homework4)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  roscpp
  tf
  visualization_msgs
)

generate_dynamic_reconfigure_options(
  cfg/Marker.cfg
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(homework4 src/homework4.cpp)
add_dependencies(homework4 ${PROJECT_NAME}_gencfg)
target_link_libraries(homework4
  ${catkin_LIBRARIES}
)

add_executable(test_homework4 test/test_homework4.cpp)
add_dependencies(test_homework4 ${PROJECT_NAME}_gencfg)
target_link_libraries(test_homework4
  ${catkin_LIBRARIES}
  ${GTEST_LIBRARIES}
)
