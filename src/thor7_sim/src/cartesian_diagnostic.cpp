// thor7_sim: cartesian_diagnostic — 局部扰动鲁棒性测试
//
// 以物体中心 (0.35, 0, grasp_z) 为基准，测试 x±2cm / y±2cm / z±1cm 共 7 个位置：
// 每个位置运行同一个 candidate evaluator（yaw × seed → IK → 等价角 →
// bounds/collision/margin → motion_cost 排序），报告是否找到可行 pre-grasp。

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <string>
#include <thread>
#include <vector>

using moveit::planning_interface::MoveGroupInterface;

constexpr double SAFETY_MARGIN = 0.05;

template <typename T>
T getOrDeclare(const rclcpp::Node::SharedPtr & node, const std::string & name, const T & d)
{
  T v = d;
  if (node->has_parameter(name)) { node->get_parameter(name, v); }
  else { v = node->declare_parameter<T>(name, d); }
  return v;
}

namespace
{
geometry_msgs::msg::Quaternion quatRPY(double r, double p, double y)
{
  tf2::Quaternion q;
  q.setRPY(r, p, y);
  q.normalize();
  return tf2::toMsg(q);
}
}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "cartesian_diagnostic",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  const auto obj_name = getOrDeclare<std::string>(node, "object_name", "test_box");
  const auto cx = getOrDeclare<double>(node, "object_x", 0.35);
  const auto cy = getOrDeclare<double>(node, "object_y", 0.0);
  const auto grasp_z = getOrDeclare<double>(node, "grasp_z", 0.13);
  const auto pre_grasp_offset = getOrDeclare<double>(node, "pre_grasp_offset", 0.06);
  const auto wait_timeout_s = getOrDeclare<double>(node, "wait_object_timeout", 30.0);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread spinner([&executor]() { executor.spin(); });

  moveit::planning_interface::PlanningSceneInterface psi;
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(wait_timeout_s));
    bool seen = false;
    while (std::chrono::steady_clock::now() < deadline) {
      if (psi.getObjects().count(obj_name) > 0) { seen = true; break; }
      std::this_thread::sleep_for(std::chrono::milliseconds(300));
    }
    if (!seen) {
      RCLCPP_ERROR(node->get_logger(), "Object not found.");
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  MoveGroupInterface arm(node, "thor7_arm");
  arm.setPoseReferenceFrame("base_link");
  arm.setEndEffectorLink("tool0");
  auto robot_model = arm.getRobotModel();
  const auto * jmg = robot_model->getJointModelGroup("thor7_arm");
  const auto & var_names = jmg->getVariableNames();

  std::vector<double> lower, upper;
  for (const auto * jm : jmg->getActiveJointModels()) {
    for (const auto & vb : jm->getVariableBounds()) {
      lower.push_back(vb.min_position_);
      upper.push_back(vb.max_position_);
    }
  }
  auto validity_client = node->create_client<moveit_msgs::srv::GetStateValidity>("/check_state_validity");
  if (!validity_client->wait_for_service(std::chrono::seconds(5))) {
    RCLCPP_ERROR(node->get_logger(), "/check_state_validity not available");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  const std::vector<double> home = {0.0, -0.0863, 0.0, 0.0, 0.0, 0.0};
  const std::vector<std::pair<std::string, std::vector<double>>> seeds = {
    {"home",       {0.0, -0.0863, 0.0, 0.0, 0.0, 0.0}},
    {"wrist-flip", {0.0, -0.0863, 0.0, 0.0, M_PI, 0.0}},
    {"known-good", {-3.33, -2.18, 2.18, -1.01, M_PI, -1.20}},
  };
  const std::vector<std::pair<std::string, double>> yaws = {
    {"yaw_0", 0.0}, {"yaw_+15", 15.0}, {"yaw_-15", -15.0},
    {"yaw_+30", 30.0}, {"yaw_-30", -30.0},
    {"yaw_+45", 45.0}, {"yaw_-45", -45.0}, {"yaw_+90", 90.0},
  };

  // 7 个扰动位置（object 中心，单位 m）
  const std::vector<std::pair<std::string, std::array<double, 3>>> positions = {
    {"center", {cx, cy, grasp_z}},
    {"x+2cm", {cx + 0.02, cy, grasp_z}},
    {"x-2cm", {cx - 0.02, cy, grasp_z}},
    {"y+2cm", {cx, cy + 0.02, grasp_z}},
    {"y-2cm", {cx, cy - 0.02, grasp_z}},
    {"z+1cm", {cx, cy, grasp_z + 0.01}},
    {"z-1cm", {cx, cy, grasp_z - 0.01}},
  };

  RCLCPP_INFO(node->get_logger(), "=== perturbation robustness test ===");
  RCLCPP_INFO(node->get_logger(), "%-8s %s", "pos", "feasible best_yaw/seed motion margin");

  int feasible_count = 0;
  for (const auto & [pname, p] : positions) {
    const double px = p[0], py = p[1], pz = p[2];
    const double pre_z = pz + pre_grasp_offset;

    bool found = false;
    std::string best_yaw, best_seed;
    double best_cost = 1e9, best_margin = 0.0;

    for (const auto & [yaw_name, yaw_deg] : yaws) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = px;
      pose.position.y = py;
      pose.position.z = pre_z;
      pose.orientation = quatRPY(M_PI, 0.0, yaw_deg * M_PI / 180.0);

      for (const auto & [seed_name, seed] : seeds) {
        moveit::core::RobotState state(robot_model);
        state.setToDefaultValues();
        state.setJointGroupPositions(jmg, seed);
        if (!state.setFromIK(jmg, pose, "tool0", 0.0)) continue;
        std::vector<double> sol;
        state.copyJointGroupPositions(jmg, sol);

        for (size_t i = 0; i < sol.size(); ++i) {
          const double span = upper[i] - lower[i];
          if (span >= 2.0 * M_PI - 1e-6) {
            double bq = sol[i], bc = std::fabs(sol[i] - home[i]);
            for (double delta : {2.0 * M_PI, -2.0 * M_PI}) {
              const double qr = sol[i] + delta;
              if (qr < lower[i] - 1e-6 || qr > upper[i] + 1e-6) continue;
              const double c = std::fabs(qr - home[i]);
              if (c < bc - 1e-9) { bq = qr; bc = c; }
            }
            sol[i] = bq;
          }
        }

        bool bounds_ok = true;
        double margin = 1e9;
        for (size_t i = 0; i < sol.size(); ++i) {
          const double q = sol[i], lo = lower[i], hi = upper[i];
          if (q < lo - 1e-6 || q > hi + 1e-6) bounds_ok = false;
          const double span = hi - lo;
          margin = std::min(margin, span > 1e-9 ? std::min(q - lo, hi - q) / span : 0.0);
        }
        if (!bounds_ok || margin <= SAFETY_MARGIN) continue;

        auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
        req->group_name = "thor7_arm";
        req->robot_state.joint_state.name = var_names;
        req->robot_state.joint_state.position = sol;
        auto fut = validity_client->async_send_request(req);
        if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) continue;
        if (!fut.get()->contacts.empty()) continue;

        double cost = 0.0;
        for (size_t i = 0; i < sol.size(); ++i) cost += std::fabs(sol[i] - home[i]);
        if (cost < best_cost) {
          found = true;
          best_cost = cost;
          best_margin = margin;
          best_yaw = yaw_name;
          best_seed = seed_name;
        }
      }
    }

    if (found) ++feasible_count;
    RCLCPP_INFO(
      node->get_logger(), "%-8s %s  %s/%s motion=%.2f margin=%.3f",
      pname.c_str(), found ? "YES" : "NO ",
      found ? best_yaw.c_str() : "-", found ? best_seed.c_str() : "-",
      found ? best_cost : 0.0, found ? best_margin : 0.0);
  }

  RCLCPP_INFO(node->get_logger(), "feasible: %d/%zu positions", feasible_count, positions.size());
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
