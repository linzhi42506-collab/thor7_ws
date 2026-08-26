#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <moveit/move_group_interface/move_group_interface.h>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared(
    "thor7_move_to_pose",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);

  std::thread spinner([&executor]() {
    executor.spin();
  });

  moveit::planning_interface::MoveGroupInterface move_group(
    node, "thor7_arm");

  move_group.setPlanningTime(10.0);
  move_group.setNumPlanningAttempts(10);
  move_group.setMaxVelocityScalingFactor(0.1);
  move_group.setMaxAccelerationScalingFactor(0.1);
  move_group.setStartStateToCurrentState();

  std::vector<double> target_joints =
    move_group.getCurrentJointValues();

  if (target_joints.size() != 6) {
    RCLCPP_ERROR(
      node->get_logger(),
      "Expected 6 joints, but received %zu",
      target_joints.size());

    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Current joint values:");

  for (std::size_t i = 0; i < target_joints.size(); ++i) {
    RCLCPP_INFO(
      node->get_logger(),
      "joint%zu = %.4f rad",
      i + 1,
      target_joints[i]);
  }

  // 小幅弯曲 J2 和 J3，离开竖直奇异位形。
	target_joints[0] = 0.0;
	target_joints[1] = -0.35;
	target_joints[2] = 0.45;
	target_joints[3] = 0.0;
	target_joints[4] = 0.20;
	target_joints[5] = 0.0;
  if (!move_group.setJointValueTarget(target_joints)) {
    RCLCPP_ERROR(node->get_logger(), "Joint target is outside limits.");

    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  moveit::planning_interface::MoveGroupInterface::Plan plan;

  const bool success =
    static_cast<bool>(move_group.plan(plan));

  if (!success) {
    RCLCPP_ERROR(node->get_logger(), "Joint-space planning failed.");

    rclcpp::shutdown();
    spinner.join();
    return 1;
  }

  RCLCPP_INFO(node->get_logger(), "Planning succeeded. Executing...");

  const auto result = move_group.execute(plan);

  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Execution succeeded.");
  } else {
    RCLCPP_ERROR(node->get_logger(), "Execution failed.");
  }

  rclcpp::shutdown();
  spinner.join();
  return 0;
}
