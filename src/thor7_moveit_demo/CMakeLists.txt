cmake_minimum_required(VERSION 3.8)
project(thor7_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)

ament_target_dependencies(move_to_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS move_to_pose
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
