cmake_minimum_required(VERSION 3.8)
project(thor7_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)
add_executable(vision_grasp_plan_only src/vision_grasp_plan_only.cpp)
add_executable(vision_grasp_pregrasp_guarded src/vision_grasp_pregrasp_guarded.cpp)
add_executable(workcell_collision_scene src/workcell_collision_scene.cpp)

ament_target_dependencies(move_to_pose
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

ament_target_dependencies(vision_grasp_plan_only
  rclcpp geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface std_srvs
)

ament_target_dependencies(vision_grasp_pregrasp_guarded
  rclcpp geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface std_srvs
)

ament_target_dependencies(workcell_collision_scene
  rclcpp geometry_msgs moveit_msgs moveit_ros_planning_interface shape_msgs std_srvs
)

install(
  TARGETS move_to_pose vision_grasp_plan_only vision_grasp_pregrasp_guarded
          workcell_collision_scene
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
