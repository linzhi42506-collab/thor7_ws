// thor7_sim: grasp_demo — 极简 A/B 定位版（TEST 1~11）
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_msgs/msg/motion_plan_request.hpp>

#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "grasp_demo",
    rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(true));

  RCLCPP_WARN(node->get_logger(), "===== GRASP DEMO DEBUG BUILD 2026-08-19 A =====");

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  RCLCPP_INFO(node->get_logger(), "TEST 1: creating MoveGroupInterface");

  MoveGroupInterface arm(node, "thor7_arm");

  RCLCPP_INFO(node->get_logger(), "TEST 2: MoveGroupInterface created");

  arm.setPlanningTime(5.0);
  arm.setNumPlanningAttempts(2);
  arm.setMaxVelocityScalingFactor(0.1);
  arm.setMaxAccelerationScalingFactor(0.1);
  arm.setStartStateToCurrentState();

  RCLCPP_INFO(node->get_logger(), "TEST 3: waiting for current state");

  auto state = arm.getCurrentState(2.0);

  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "Cannot obtain current state");
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "TEST 4: current state OK");

  std::vector<double> home_joints = {
    0.0,
    -0.0863,
    0.0,
    0.0,
    0.0,
    0.0
  };

  bool target_ok = arm.setJointValueTarget(home_joints);

  RCLCPP_INFO(
    node->get_logger(),
    "TEST 5: setJointValueTarget = %s",
    target_ok ? "true" : "false");

  if (!target_ok) {
    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 1;
  }

  // 单独测试请求构造
  moveit_msgs::msg::MotionPlanRequest request;

  RCLCPP_INFO(node->get_logger(), "TEST 6: before constructMotionPlanRequest");

  arm.constructMotionPlanRequest(request);

  RCLCPP_INFO(
    node->get_logger(),
    "TEST 7: request constructed, group=%s goals=%zu",
    request.group_name.c_str(),
    request.goal_constraints.size());

  MoveGroupInterface::Plan plan;

  RCLCPP_INFO(node->get_logger(), "TEST 8: BEFORE plan()");

  auto result = arm.plan(plan);

  RCLCPP_INFO(
    node->get_logger(),
    "TEST 9: AFTER plan(), code=%d",
    result.val);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "TEST 10: executing");

    auto exec_result = arm.execute(plan);

    RCLCPP_INFO(
      node->get_logger(),
      "TEST 11: execute result=%d",
      exec_result.val);
  }

  executor.cancel();
  spinner.join();
  rclcpp::shutdown();

  return 0;
}
