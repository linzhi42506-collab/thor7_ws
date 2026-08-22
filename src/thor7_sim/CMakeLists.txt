cmake_minimum_required(VERSION 3.8)
project(thor7_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(urdf REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# AG-145 夹爪 mimic 关节状态发布：把 finger1 的位置按 URDF <mimic> 换算成
# 所有夹爪关节位置，合并发布到 /joint_states（让 RViz/MoveIt 正确显示手指开合）
add_executable(gripper_state_publisher src/gripper_state_publisher.cpp)
ament_target_dependencies(gripper_state_publisher
  rclcpp
  sensor_msgs
  urdf
)

# 在 MoveIt 规划场景中添加一个可抓取的测试物体（默认立方体）
add_executable(spawn_test_object src/spawn_test_object.cpp)
ament_target_dependencies(spawn_test_object
  rclcpp
  geometry_msgs
  shape_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

# 抓取流程演示：预抓取 → 下降抓取 → 夹紧 → 抬起 → 松开
add_executable(grasp_demo src/grasp_demo.cpp)
ament_target_dependencies(grasp_demo
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

# Cartesian 下降诊断（奇异/可达性定位，只计算 fraction 不执行）
add_executable(cartesian_diagnostic src/cartesian_diagnostic.cpp)
ament_target_dependencies(cartesian_diagnostic
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  tf2
  tf2_geometry_msgs
)

# 夹爪开合测试：打印主关节反馈 + open→closed→open
add_executable(gripper_test src/gripper_test.cpp)
ament_target_dependencies(gripper_test
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
)

# 碰撞诊断：FK 测指尖间距(定 open/closed) + 打印各状态 contact link pairs
add_executable(collision_diagnostic src/collision_diagnostic.cpp)
ament_target_dependencies(collision_diagnostic
  rclcpp
  moveit_msgs
  moveit_ros_planning_interface
)

install(
  TARGETS gripper_state_publisher spawn_test_object grasp_demo cartesian_diagnostic gripper_test collision_diagnostic
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
