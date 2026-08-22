// thor7_sim: gripper_test
//
// 夹爪开合验收：OPEN(-0.65) → wait 2s → CLOSED(0.0) → wait 2s → OPEN(-0.65)
// 每一步打印：
//   - /joint_states 中 ag145_gripper_finger1_joint 实际值
//   - 7 个 mimic joint 值
//   - full-robot 碰撞 contact pairs
//
// 运行：ros2 run thor7_sim gripper_test（需 move_group + 控制器已启动）

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "gripper_test",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const auto wait_s = node->declare_parameter<double>("wait_s", 2.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroupInterface gripper(node, "gripper");
  gripper.setPlanningTime(2.0);

  auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  validity_client->wait_for_service(std::chrono::seconds(5));

  const std::vector<std::string> mimic_joints = {
    "ag145_gripper_finger2_joint",
    "ag145_gripper_finger1_finger_joint",
    "ag145_gripper_finger2_finger_joint",
    "ag145_gripper_finger1_inner_knuckle_joint",
    "ag145_gripper_finger2_inner_knuckle_joint",
    "ag145_gripper_finger1_finger_tip_joint",
    "ag145_gripper_finger2_finger_tip_joint",
  };

  // 订阅 /joint_states，合并两个发布者（broadcaster 发 finger1，mimic publisher 只发 mimic）
  std::map<std::string, double> js_map;
  auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
    "/joint_states", 10,
    [&js_map](const sensor_msgs::msg::JointState::SharedPtr m) {
      for (size_t i = 0; i < m->name.size() && i < m->position.size(); ++i) {
        js_map[m->name[i]] = m->position[i];
      }
    });

  auto readJsJoint = [&](const std::string & name) -> double {
    auto it = js_map.find(name);
    return it != js_map.end() ? it->second : 0.0;
  };

  auto reportState = [&](const std::string & what) {
    RCLCPP_INFO(node->get_logger(), "--- [%s] ---", what.c_str());
    RCLCPP_INFO(node->get_logger(), "  finger1 = %.4f rad", readJsJoint("ag145_gripper_finger1_joint"));
    for (const auto & m : mimic_joints) {
      RCLCPP_INFO(node->get_logger(), "  %s = %.4f rad", m.c_str(), readJsJoint(m));
    }

    // full-robot 碰撞检查
    auto state = gripper.getCurrentState(2.0);
    if (!state) {
      RCLCPP_ERROR(node->get_logger(), "  full-robot: state unavailable");
      return;
    }
    const auto & all_names = gripper.getRobotModel()->getVariableNames();
    const double * all_pos = state->getVariablePositions();
    auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
    req->group_name = "";
    req->robot_state.joint_state.name = all_names;
    req->robot_state.joint_state.position.assign(all_pos, all_pos + all_names.size());
    auto fut = validity_client->async_send_request(req);
    if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
      RCLCPP_WARN(node->get_logger(), "  full-robot: timeout");
      return;
    }
    const auto res = fut.get();
    if (res->contacts.empty()) {
      RCLCPP_INFO(node->get_logger(), "  full-robot: VALID (no contact)");
    } else {
      RCLCPP_ERROR(node->get_logger(), "  full-robot: %zu contact pair(s):", res->contacts.size());
      for (const auto & c : res->contacts) {
        RCLCPP_ERROR(node->get_logger(), "    %s <-> %s (depth %.4f)", c.contact_body_1.c_str(), c.contact_body_2.c_str(), c.depth);
      }
    }
  };

  const auto moveNamed = [&](const std::string & named, const std::string & what) -> bool {
    gripper.setNamedTarget(named);
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (!static_cast<bool>(gripper.plan(plan))) {
      RCLCPP_ERROR(node->get_logger(), "plan failed: %s", what.c_str());
      return false;
    }
    const auto r = gripper.execute(plan);
    RCLCPP_INFO(node->get_logger(), "%s execute = %s", what.c_str(), (r == moveit::core::MoveItErrorCode::SUCCESS) ? "OK" : "FAIL");
    return (r == moveit::core::MoveItErrorCode::SUCCESS);
  };

  RCLCPP_INFO(node->get_logger(), "=== gripper-only acceptance ===");
  std::this_thread::sleep_for(std::chrono::milliseconds(500));
  reportState("startup");

  moveNamed("open", "OPEN(-0.65)");
  std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
  reportState("after OPEN");

  moveNamed("closed", "CLOSED(0.0)");
  std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
  reportState("after CLOSED");

  moveNamed("open", "OPEN(-0.65)");
  std::this_thread::sleep_for(std::chrono::duration<double>(wait_s));
  reportState("after OPEN again");

  RCLCPP_INFO(node->get_logger(), "=== gripper-only acceptance done ===");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
