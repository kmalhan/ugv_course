cmake_minimum_required(VERSION 2.8.3)
project(homework2)

set(${PROJECT_NAME}_CATKIN_DEPS
  message_generation
  std_msgs
  roscpp
  rostest
)

find_package(catkin REQUIRED COMPONENTS ${${PROJECT_NAME}_CATKIN_DEPS})

add_service_files(
	FILES
	string_cat.srv
)

generate_messages()

catkin_package()

add_executable(pubsub_node src/pubsub_node.cpp)
target_link_libraries(pubsub_node
  ${catkin_LIBRARIES}
)

add_executable(service_node src/service_node.cpp)
add_dependencies(service_node ${PROJECT_NAME}_gencpp)
target_link_libraries(service_node
  ${catkin_LIBRARIES}
)

add_executable(test_homework2 test/test_homework2.cpp)
target_link_libraries(test_homework2 ${catkin_LIBRARIES} ${GTEST_LIBRARIES})

