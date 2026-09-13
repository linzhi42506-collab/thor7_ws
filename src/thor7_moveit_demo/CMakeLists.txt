cmake_minimum_required(VERSION 3.8)
project(thor7_moveit_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

add_executable(move_to_pose src/move_to_pose.cpp)
add_executable(vision_grasp_plan_only src/vision_grasp_plan_only.cpp)
add_executable(vision_grasp_pregrasp_guarded src/vision_grasp_pregrasp_guarded.cpp)
add_executable(vision_grasp_auto_sequence src/vision_grasp_auto_sequence.cpp)
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
  rclcpp geometry_msgs moveit_core moveit_msgs moveit_ros_planning_interface sensor_msgs std_srvs
)

ament_target_dependencies(vision_grasp_auto_sequence
  control_msgs geometry_msgs rclcpp rclcpp_action std_srvs sensor_msgs
  moveit_core moveit_msgs moveit_ros_planning_interface shape_msgs
)

ament_target_dependencies(workcell_collision_scene
  rclcpp geometry_msgs moveit_msgs moveit_ros_planning_interface shape_msgs std_srvs
)

install(
  TARGETS move_to_pose vision_grasp_plan_only vision_grasp_pregrasp_guarded
          vision_grasp_auto_sequence
          workcell_collision_scene
  DESTINATION lib/${PROJECT_NAME}
)


install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(Python3 REQUIRED COMPONENTS Interpreter)
  add_test(NAME pick_place_targets COMMAND /usr/bin/python3 ${CMAKE_CURRENT_SOURCE_DIR}/test/test_pick_place_targets.py)
  find_package(urdf REQUIRED)
  find_package(srdfdom REQUIRED)
  find_package(pluginlib REQUIRED)
  enable_testing()
  add_executable(test_command_lease test/test_command_lease.cpp)
  target_include_directories(test_command_lease PRIVATE src)
  add_test(NAME command_lease COMMAND test_command_lease)
  add_executable(test_new_target_gate test/test_new_target_gate.cpp)
  target_include_directories(test_new_target_gate PRIVATE src)
  target_link_libraries(test_new_target_gate Eigen3::Eigen)
  add_test(NAME new_target_gate COMMAND test_new_target_gate)
  add_executable(test_grasp_geometry test/test_grasp_geometry.cpp)
  target_include_directories(test_grasp_geometry PRIVATE src)
  target_link_libraries(test_grasp_geometry Eigen3::Eigen)
  add_test(NAME grasp_geometry COMMAND test_grasp_geometry)
  add_executable(test_target_lock test/test_target_lock.cpp)
  target_include_directories(test_target_lock PRIVATE src)
  target_link_libraries(test_target_lock Eigen3::Eigen)
  add_test(NAME target_lock COMMAND test_target_lock)
  add_executable(test_refreshed_tool_pose test/test_refreshed_tool_pose.cpp)
  target_include_directories(test_refreshed_tool_pose PRIVATE src)
  ament_target_dependencies(test_refreshed_tool_pose moveit_core urdf srdfdom)
  add_test(NAME refreshed_tool_pose COMMAND test_refreshed_tool_pose)
  add_executable(test_pregrasp_ik test/test_pregrasp_ik.cpp)
  target_include_directories(test_pregrasp_ik PRIVATE src)
  ament_target_dependencies(test_pregrasp_ik moveit_core rclcpp urdf srdfdom pluginlib)
  target_compile_definitions(test_pregrasp_ik PRIVATE
    THOR7_TEST_URDF="${CMAKE_CURRENT_SOURCE_DIR}/../thor7v3_description/urdf/thor7v3_robot.urdf"
    THOR7_TEST_SRDF="${CMAKE_CURRENT_SOURCE_DIR}/../thor7v3_moveit_config/config/thor7v3.srdf")
  add_test(NAME pregrasp_ik COMMAND test_pregrasp_ik)
  set_tests_properties(pregrasp_ik PROPERTIES TIMEOUT 30 ENVIRONMENT "ROS_DOMAIN_ID=187;ROS_LOCALHOST_ONLY=1;ROS_LOG_DIR=/tmp/thor7-pregrasp-ik-test")
endif()

ament_package()
