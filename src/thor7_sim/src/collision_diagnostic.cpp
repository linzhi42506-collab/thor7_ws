// thor7_sim: collision_diagnostic
//
// 目的：
//  1) 用 FK 计算 ag145 两指尖的间距，确定 finger1_joint 哪个值对应 OPEN / CLOSED
//  2) 对 startup / open / closed 状态调用 /check_state_validity，打印所有 contact link pairs
//
// 运行方式（需要 move_group + 测试物体已就绪）：
//   ros2 launch thor7_sim sim.launch.py use_rviz:=false run_diagnostic:=true

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/srv/get_state_validity.hpp>

#include <chrono>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

template <typename T>
T getOrDeclare(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & d)
{
  T v = d;
  if (node->has_parameter(name)) { node->get_parameter(name, v); }
  else { v = node->declare_parameter<T>(name, d); }
  return v;
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "collision_diagnostic",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  MoveGroupInterface arm(node, "thor7_arm");
  arm.setPoseReferenceFrame("base_link");
  arm.setEndEffectorLink("tool0");

  auto robot_model = arm.getRobotModel();
  const auto * arm_jmg = robot_model->getJointModelGroup("thor7_arm");
  const auto & all_names = robot_model->getVariableNames();

  auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  if (!validity_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "/check_state_validity not available");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  const std::string f1 = "ag145_gripper_finger1_joint";
  const std::string tip1 = "ag145_gripper_finger1_finger_tip_link";
  const std::string tip2 = "ag145_gripper_finger2_finger_tip_link";
  const std::vector<double> home = {0.0, -0.0863, 0.0, 0.0, 0.0, 0.0};

  // ---- 1) 指尖间距 vs finger1_joint ----
  RCLCPP_INFO(node->get_logger(), "=== fingertip gap vs finger1_joint ===");
  for (double f : {0.0, -0.2, -0.4, -0.65}) {
    moveit::core::RobotState st(robot_model);
    st.setToDefaultValues();
    st.setJointGroupPositions(arm_jmg, home);
    st.setVariablePosition(f1, f);
    st.update();

    const auto p1 = st.getGlobalLinkTransform(tip1).translation();
    const auto p2 = st.getGlobalLinkTransform(tip2).translation();
    const double gap = (p1 - p2).norm();
    RCLCPP_INFO(
      node->get_logger(),
      "finger1=%.2f rad -> fingertip gap = %.4f m (tip1 x=%.3f tip2 x=%.3f)",
      f, gap, p1.x(), p2.x());
  }

  // ---- 2) 碰撞检查：不同臂构型 × 夹爪状态 ----
  RCLCPP_INFO(node->get_logger(), "=== collision states ===");
  const std::vector<std::pair<std::string, std::vector<double>>> arm_cfgs = {
    {"startup(all-zero)", {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
    {"home", home},
  };
  const std::vector<std::pair<std::string, double>> finger_vals = {
    {"OPEN(-0.65)", -0.65},
    {"CLOSED(0.0)", 0.0},
  };

  for (const auto & [arm_label, arm_joints] : arm_cfgs) {
    for (const auto & [f_label, f] : finger_vals) {
      moveit::core::RobotState st(robot_model);
      st.setToDefaultValues();
      st.setJointGroupPositions(arm_jmg, arm_joints);
      st.setVariablePosition(f1, f);
      st.update();

      const double * all_pos_ptr = st.getVariablePositions();
      std::vector<double> all_pos(all_pos_ptr, all_pos_ptr + all_names.size());

      auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      req->group_name = "thor7_arm";
      req->robot_state.joint_state.name = all_names;
      req->robot_state.joint_state.position = all_pos;

      auto fut = validity_client->async_send_request(req);
      if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_WARN(node->get_logger(), "[%s/%s] validity check timeout", arm_label.c_str(), f_label.c_str());
        continue;
      }
      const auto res = fut.get();
      if (res->contacts.empty()) {
        RCLCPP_INFO(node->get_logger(), "[%s / %s] NO CONTACT (valid)", arm_label.c_str(), f_label.c_str());
      } else {
        RCLCPP_INFO(node->get_logger(), "[%s / %s] %zu contact pair(s):", arm_label.c_str(), f_label.c_str(), res->contacts.size());
        for (const auto & c : res->contacts) {
          RCLCPP_INFO(
            node->get_logger(), "    %s <-> %s  (depth %.4f)",
            c.contact_body_1.c_str(), c.contact_body_2.c_str(), c.depth);
        }
      }
    }
  }

  RCLCPP_INFO(node->get_logger(), "=== done ===");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
