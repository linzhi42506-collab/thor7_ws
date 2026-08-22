// thor7_sim: grasp_demo — arm-only 抓取流程（candidate evaluator 版）
//
// 流程：
//   home → selectBestPreGrasp(物体位置 → yaw×seed IK 评估 → 最小运动代价解)
//        → pre-grasp(关节目标) → Cartesian 下降 → Cartesian 抬升
// 不再写死关节值：每次根据物体位置动态生成 yaw candidates + multi-seed IK，
// 硬淘汰(bounds/collision/margin) + motion_cost 排序，选最优解。

#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/srv/get_state_validity.hpp>
#include <moveit_msgs/msg/motion_plan_request.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <map>
#include <sstream>
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
geometry_msgs::msg::Quaternion quatRPY(double roll, double pitch, double yaw)
{
  tf2::Quaternion q;
  q.setRPY(roll, pitch, yaw);
  q.normalize();
  return tf2::toMsg(q);
}

bool planAndExecute(MoveGroupInterface & mg, rclcpp::Node::SharedPtr node, const std::string & what)
{
  for (int attempt = 1; attempt <= 3; ++attempt) {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (static_cast<bool>(mg.plan(plan))) {
      const auto result = mg.execute(plan);
      if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        RCLCPP_INFO(node->get_logger(), "Executed: %s (attempt %d)", what.c_str(), attempt);
        return true;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(400));
  }
  RCLCPP_ERROR(node->get_logger(), "planAndExecute gave up: %s", what.c_str());
  return false;
}

bool guardCartesianTrajectory(
  rclcpp::Node::SharedPtr node,
  const moveit_msgs::msg::RobotTrajectory & traj,
  const std::string & what);

bool cartesianMove(
  MoveGroupInterface & mg, rclcpp::Node::SharedPtr node,
  const geometry_msgs::msg::Pose & target, const std::string & what)
{
  moveit_msgs::msg::RobotTrajectory traj;
  std::vector<geometry_msgs::msg::Pose> waypoints{target};
  const double fraction = mg.computeCartesianPath(waypoints, 0.005, 0.0, traj, true);
  RCLCPP_INFO(node->get_logger(), "Cartesian %s fraction = %.3f", what.c_str(), fraction);
  if (fraction < 0.98) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian %s incomplete (%.3f), refusing", what.c_str(), fraction);
    return false;
  }
  if (!guardCartesianTrajectory(node, traj, what)) {
    RCLCPP_ERROR(node->get_logger(), "Cartesian %s trajectory guard rejected", what.c_str());
    return false;
  }
  const auto result = mg.execute(traj);
  if (result == moveit::core::MoveItErrorCode::SUCCESS) {
    RCLCPP_INFO(node->get_logger(), "Cartesian executed: %s", what.c_str());
    return true;
  }
  RCLCPP_ERROR(node->get_logger(), "Cartesian execution failed: %s (code %d)", what.c_str(), result.val);
  return false;
}

// 碰撞状态检查：全机器人（group_name=""），打印 res->valid 和所有 contact pairs。
// 返回 false 表示 invalid/碰撞，调用方必须 abort。
bool checkStateValid(
  rclcpp::Node::SharedPtr node,
  MoveGroupInterface & arm,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  const std::string & what)
{
  auto state = arm.getCurrentState(2.0);
  if (!state) {
    RCLCPP_ERROR(node->get_logger(), "[%s] current state unavailable -> INVALID", what.c_str());
    return false;
  }
  const auto & all_names = arm.getRobotModel()->getVariableNames();
  const double * all_pos = state->getVariablePositions();

  auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
  req->group_name = "";  // 全机器人
  req->robot_state.joint_state.name = all_names;
  req->robot_state.joint_state.position.assign(all_pos, all_pos + all_names.size());

  auto fut = validity_client->async_send_request(req);
  if (fut.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
    RCLCPP_ERROR(node->get_logger(), "[%s] validity timeout -> INVALID", what.c_str());
    return false;
  }
  const auto res = fut.get();
  if (res->valid && res->contacts.empty()) {
    RCLCPP_INFO(node->get_logger(), "[%s] full-robot VALID", what.c_str());
    return true;
  }
  RCLCPP_ERROR(
    node->get_logger(), "[%s] full-robot INVALID (valid=%s, %zu contacts):",
    what.c_str(), res->valid ? "true" : "false", res->contacts.size());
  for (const auto & c : res->contacts) {
    RCLCPP_ERROR(
      node->get_logger(), "    %s <-> %s (depth %.4f)",
      c.contact_body_1.c_str(), c.contact_body_2.c_str(), c.depth);
  }
  return false;
}

// Cartesian 轨迹 guard：逐点 joint delta / min/max/excursion，拒绝 IK branch jump 或大绕转
bool guardCartesianTrajectory(
  rclcpp::Node::SharedPtr node,
  const moveit_msgs::msg::RobotTrajectory & traj,
  const std::string & what)
{
  const auto & pts = traj.joint_trajectory.points;
  if (pts.size() < 2) {
    RCLCPP_WARN(node->get_logger(), "[%s] cartesian trajectory has <2 points", what.c_str());
    return true;
  }
  const size_t n = pts[0].positions.size();
  bool ok = true;
  for (size_t j = 0; j < n; ++j) {
    double mn = 1e9, mx = -1e9, max_step = 0.0;
    double prev = pts[0].positions[j];
    for (const auto & p : pts) {
      if (j >= p.positions.size()) continue;
      const double q = p.positions[j];
      mn = std::min(mn, q);
      mx = std::max(mx, q);
      max_step = std::max(max_step, std::fabs(q - prev));
      prev = q;
    }
    const double exc = mx - mn;
    RCLCPP_INFO(
      node->get_logger(), "[%s] J%zu min=%.3f max=%.3f exc=%.3f max_step=%.3f rad",
      what.c_str(), j + 1, mn, mx, exc, max_step);
    if (max_step > 1.0) {
      RCLCPP_ERROR(node->get_logger(), "[%s] J%zu IK branch jump (max_step=%.3f)", what.c_str(), j + 1, max_step);
      ok = false;
    }
    if (exc > 3.0) {
      RCLCPP_ERROR(node->get_logger(), "[%s] J%zu large winding (exc=%.3f)", what.c_str(), j + 1, exc);
      ok = false;
    }
  }
  return ok;
}

// 轨迹 guard：打印六关节 start/goal/min/max/excursion，拒绝不必要的大角度绕转
bool guardTrajectory(
  rclcpp::Node::SharedPtr node,
  const moveit::planning_interface::MoveGroupInterface::Plan & plan,
  const std::vector<double> & start, const std::vector<double> & goal)
{
  const auto & pts = plan.trajectory_.joint_trajectory.points;
  if (pts.empty()) {
    RCLCPP_WARN(node->get_logger(), "[trajectory guard] empty trajectory");
    return true;
  }
  const size_t n = std::min<size_t>(6, goal.size());
  bool ok = true;
  for (size_t j = 0; j < n; ++j) {
    double mn = 1e9, mx = -1e9;
    for (const auto & p : pts) {
      if (j < p.positions.size()) {
        mn = std::min(mn, p.positions[j]);
        mx = std::max(mx, p.positions[j]);
      }
    }
    const double exc = mx - mn;
    const double direct = std::fabs(goal[j] - start[j]);
    RCLCPP_INFO(
      node->get_logger(),
      "  J%zu start=%.2f goal=%.2f min=%.2f max=%.2f exc=%.2f rad",
      j + 1, start[j], goal[j], mn, mx, exc);
    if (exc > direct + 2.0) {  // 超出直接变化约 115° 视为绕转
      RCLCPP_ERROR(
        node->get_logger(),
        "  J%zu unnecessary winding (exc=%.2f > direct=%.2f + 2.0), rejecting",
        j + 1, exc, direct);
      ok = false;
    }
  }
  return ok;
}

struct PreGrasp
{
  bool ok = false;
  std::vector<double> joints;
  double yaw_deg = 0.0;
  std::string yaw_name, seed_name;
  double margin = 0.0, motion_cost = 1e9;
  int sample_idx = -1;
};

// branch 聚类 key：关节值四舍五入到 0.1 rad
std::string branchKey(const std::vector<double> & j)
{
  std::ostringstream os;
  for (size_t i = 0; i < j.size(); ++i) {
    if (i) os << "_";
    os << static_cast<int>(std::lround(j[i] * 10.0));
  }
  return os.str();
}

// 候选评估器：多采样收集不同 IK branch，统一评分（motion_cost 相对当前状态）。
// goal displacement guard：|ΔJ1| 与 total motion_cost 超过阈值则拒绝该候选；
// 一轮无候选通过则重采样；多轮仍失败则安全 abort。
PreGrasp selectBestPreGrasp(
  rclcpp::Node::SharedPtr node,
  MoveGroupInterface & arm,
  const moveit::core::JointModelGroup * jmg,
  const std::vector<std::string> & var_names,
  const std::vector<double> & lower, const std::vector<double> & upper,
  const rclcpp::Client<moveit_msgs::srv::GetStateValidity>::SharedPtr & validity_client,
  double obj_x, double obj_y, double pre_z,
  double max_j1_delta, double max_motion_cost, int samples_per_group, int sampling_rounds)
{
  const auto current = arm.getCurrentJointValues();

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

  for (int round = 1; round <= sampling_rounds; ++round) {
    RCLCPP_INFO(node->get_logger(), "IK sampling round %d/%d", round, sampling_rounds);

    struct Cand
    {
      std::vector<double> joints;
      std::string yaw_name, seed_name;
      double yaw_deg = 0.0, margin = 0.0;
      int sample_idx = -1;
    };
    std::map<std::string, Cand> branches;

    for (const auto & [yaw_name, yaw_deg] : yaws) {
      geometry_msgs::msg::Pose pose;
      pose.position.x = obj_x;
      pose.position.y = obj_y;
      pose.position.z = pre_z;
      pose.orientation = quatRPY(M_PI, 0.0, yaw_deg * M_PI / 180.0);

      for (const auto & [seed_name, seed] : seeds) {
        for (int s = 0; s < samples_per_group; ++s) {
          moveit::core::RobotState state(arm.getRobotModel());
          state.setToDefaultValues();
          state.setJointGroupPositions(jmg, seed);
          if (!state.setFromIK(jmg, pose, "tool0", 0.0)) continue;
          std::vector<double> sol;
          state.copyJointGroupPositions(jmg, sol);

          // 等价角：逐关节选 q/q±2π 中距 current 最近且在限位内的表示
          for (size_t i = 0; i < sol.size(); ++i) {
            const double span = upper[i] - lower[i];
            if (span >= 2.0 * M_PI - 1e-6) {
              double bq = sol[i], bc = std::fabs(sol[i] - current[i]);
              for (double delta : {2.0 * M_PI, -2.0 * M_PI}) {
                const double qr = sol[i] + delta;
                if (qr < lower[i] - 1e-6 || qr > upper[i] + 1e-6) continue;
                const double c = std::fabs(qr - current[i]);
                if (c < bc - 1e-9) { bq = qr; bc = c; }
              }
              sol[i] = bq;
            }
          }

          // bounds + margin（hard reject）
          bool bounds_ok = true;
          double margin = 1e9;
          for (size_t i = 0; i < sol.size(); ++i) {
            const double q = sol[i], lo = lower[i], hi = upper[i];
            if (q < lo - 1e-6 || q > hi + 1e-6) bounds_ok = false;
            const double span = hi - lo;
            margin = std::min(margin, span > 1e-9 ? std::min(q - lo, hi - q) / span : 0.0);
          }
          if (!bounds_ok || margin <= SAFETY_MARGIN) continue;

          const std::string key = branchKey(sol);
          if (branches.count(key)) continue;  // 同一 branch 去重
          branches[key] = Cand{sol, yaw_name, seed_name, yaw_deg, margin, s};
        }
      }
    }
    RCLCPP_INFO(node->get_logger(), "  round %d: collected %zu distinct IK branches", round, branches.size());

    // 统一评分
    PreGrasp best;
    for (const auto & [key, cand] : branches) {
      // collision（hard reject）
      auto req = std::make_shared<moveit_msgs::srv::GetStateValidity::Request>();
      req->group_name = "thor7_arm";
      req->robot_state.joint_state.name = var_names;
      req->robot_state.joint_state.position = cand.joints;
      auto fut = validity_client->async_send_request(req);
      bool coll_ok = true;
      if (fut.wait_for(std::chrono::seconds(3)) == std::future_status::ready) {
        coll_ok = fut.get()->contacts.empty();
      }
      if (!coll_ok) {
        RCLCPP_INFO(node->get_logger(), "  reject %s/%s: in collision", cand.yaw_name.c_str(), cand.seed_name.c_str());
        continue;
      }

      // motion cost（相对当前状态）
      double motion_cost = 0.0;
      for (size_t i = 0; i < cand.joints.size(); ++i) {
        motion_cost += std::fabs(cand.joints[i] - current[i]);
      }

      // goal displacement guard（与 trajectory winding guard 分开）
      const double dJ1 = std::fabs(cand.joints[0] - current[0]);
      if (dJ1 > max_j1_delta) {
        RCLCPP_INFO(
          node->get_logger(), "  reject %s/%s: |dJ1|=%.2f rad > %.2f",
          cand.yaw_name.c_str(), cand.seed_name.c_str(), dJ1, max_j1_delta);
        continue;
      }
      if (motion_cost > max_motion_cost) {
        RCLCPP_INFO(
          node->get_logger(), "  reject %s/%s: motion_cost=%.2f > %.2f",
          cand.yaw_name.c_str(), cand.seed_name.c_str(), motion_cost, max_motion_cost);
        continue;
      }

      if (motion_cost < best.motion_cost) {
        best.ok = true;
        best.joints = cand.joints;
        best.yaw_deg = cand.yaw_deg;
        best.yaw_name = cand.yaw_name;
        best.seed_name = cand.seed_name;
        best.margin = cand.margin;
        best.motion_cost = motion_cost;
        best.sample_idx = cand.sample_idx;
      }
    }

    if (best.ok) {
      RCLCPP_INFO(
        node->get_logger(),
        "Best pre-grasp: yaw=%s seed=%s sample=%d, motion_cost=%.2f rad, margin=%.3f",
        best.yaw_name.c_str(), best.seed_name.c_str(), best.sample_idx, best.motion_cost, best.margin);
      std::ostringstream cs, ts;
      cs << std::fixed << std::setprecision(2) << "[";
      ts << std::fixed << std::setprecision(2) << "[";
      for (size_t i = 0; i < best.joints.size(); ++i) {
        if (i) { cs << ","; ts << ","; }
        cs << current[i] * 180.0 / M_PI;
        ts << best.joints[i] * 180.0 / M_PI;
      }
      cs << "]"; ts << "]";
      RCLCPP_INFO(node->get_logger(), "  current J1..J6(deg): %s", cs.str().c_str());
      RCLCPP_INFO(node->get_logger(), "  target  J1..J6(deg): %s", ts.str().c_str());
      for (size_t i = 0; i < best.joints.size(); ++i) {
        RCLCPP_INFO(
          node->get_logger(), "  ΔJ%zu = %.3f rad", i + 1, best.joints[i] - current[i]);
      }
      return best;
    }
    RCLCPP_WARN(node->get_logger(), "round %d: no candidate passed all guards, re-sampling", round);
  }
  RCLCPP_ERROR(
    node->get_logger(), "no feasible pre-grasp after %d sampling rounds, aborting", sampling_rounds);
  return PreGrasp{};
}

}  // namespace

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared(
    "grasp_demo",
    rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));

  RCLCPP_WARN(node->get_logger(), "===== GRASP DEMO (candidate evaluator) BUILD 2026-08-19 D =====");

  const auto obj_name = getOrDeclare<std::string>(node, "object_name", "test_box");
  const auto obj_x = getOrDeclare<double>(node, "object_x", 0.35);
  const auto obj_y = getOrDeclare<double>(node, "object_y", 0.0);
  const auto grasp_z = getOrDeclare<double>(node, "grasp_z", 0.10);
  const auto pre_grasp_offset = getOrDeclare<double>(node, "pre_grasp_offset", 0.06);
  const auto lift_z = getOrDeclare<double>(node, "lift_z", 0.23);
  const auto velocity_scale = getOrDeclare<double>(node, "velocity_scale", 0.1);
  const auto wait_timeout_s = getOrDeclare<double>(node, "wait_object_timeout", 30.0);
  // goal displacement guard 阈值（ROS 参数，非硬编码永久限制）
  const auto max_j1_delta = getOrDeclare<double>(node, "max_j1_delta", M_PI / 2.0);   // |ΔJ1| > 90° 拒绝
  const auto max_motion_cost = getOrDeclare<double>(node, "max_motion_cost", 8.0);    // total motion > 8 rad 拒绝
  const auto samples_per_group = getOrDeclare<int>(node, "samples_per_group", 5);     // 每 yaw×seed 采样次数
  const auto sampling_rounds = getOrDeclare<int>(node, "sampling_rounds", 3);         // 重采样轮数

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
    RCLCPP_INFO(node->get_logger(), "Object detected.");
  }

  MoveGroupInterface arm(node, "thor7_arm");
  arm.setPoseReferenceFrame("base_link");
  arm.setEndEffectorLink("tool0");
  arm.setPlanningTime(3.0);
  arm.setNumPlanningAttempts(3);
  arm.setMaxVelocityScalingFactor(velocity_scale);
  arm.setMaxAccelerationScalingFactor(velocity_scale);
  arm.setStartStateToCurrentState();

  // AG-145 夹爪接口
  MoveGroupInterface gripper(node, "gripper");
  gripper.setPlanningTime(2.0);
  gripper.setMaxVelocityScalingFactor(velocity_scale);
  gripper.setMaxAccelerationScalingFactor(velocity_scale);
  gripper.setStartStateToCurrentState();

  std::this_thread::sleep_for(std::chrono::milliseconds(500));

  const auto * jmg = arm.getRobotModel()->getJointModelGroup("thor7_arm");
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

  // 0) home
  {
    arm.setJointValueTarget(home);
    if (!planAndExecute(arm, node, "move to home")) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  // 0.5) 张开夹爪（SRDF 命名姿态 open = -0.65）
  {
    gripper.setNamedTarget("open");
    if (!planAndExecute(gripper, node, "open gripper")) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  // 1) 选最优 pre-grasp（多采样 + goal displacement guard + 重采样）
  const double pre_z = grasp_z + pre_grasp_offset;
  const PreGrasp pg = selectBestPreGrasp(
    node, arm, jmg, var_names, lower, upper, validity_client, obj_x, obj_y, pre_z,
    max_j1_delta, max_motion_cost, samples_per_group, sampling_rounds);
  if (!pg.ok) {
    RCLCPP_ERROR(node->get_logger(), "No safe pre-grasp found. Aborting (no large-angle fallback).");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // 2) pre-grasp 关节目标规划 + 执行（Pilz PTP 显式 pipeline，OMPL 回落，trajectory guard）
  {
    arm.setJointValueTarget(pg.joints);
    const auto cur = arm.getCurrentJointValues();
    const std::vector<double> start = cur.size() == 6 ? cur : home;

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    bool planned = false;
    std::string planner_used;

    // 优先 Pilz PTP（显式 pipeline + planner）
    arm.setPlanningPipelineId("pilz_industrial_motion_planner");
    arm.setPlannerId("PTP");
    if (static_cast<bool>(arm.plan(plan))) {
      planned = true;
      planner_used = "pilz_industrial_motion_planner/PTP";
    } else {
      RCLCPP_WARN(node->get_logger(), "Pilz PTP failed, fallback to OMPL RRTConnect");
      arm.setPlanningPipelineId("ompl");
      arm.setPlannerId("RRTConnect");
      planned = static_cast<bool>(arm.plan(plan));
      planner_used = "ompl/RRTConnect";
    }

    if (!planned) {
      RCLCPP_ERROR(node->get_logger(), "pre-grasp planning failed (Pilz PTP + OMPL)");
      rclcpp::shutdown(); spinner.join(); return 1;
    }

    RCLCPP_INFO(node->get_logger(), "pre-grasp planned with %s", planner_used.c_str());
    if (!guardTrajectory(node, plan, start, pg.joints)) {
      RCLCPP_ERROR(node->get_logger(), "trajectory guard rejected pre-grasp");
      rclcpp::shutdown(); spinner.join(); return 1;
    }

    const auto t0 = std::chrono::steady_clock::now();
    const auto result = arm.execute(plan);
    const auto t1 = std::chrono::steady_clock::now();
    RCLCPP_INFO(
      node->get_logger(), "pre-grasp execute = %.3f s, result %d",
      std::chrono::duration<double>(t1 - t0).count(), result.val);
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  // 3) Cartesian 下降
  {
    geometry_msgs::msg::Pose grasp;
    grasp.position.x = obj_x;
    grasp.position.y = obj_y;
    grasp.position.z = grasp_z;
    grasp.orientation = quatRPY(M_PI, 0.0, pg.yaw_deg * M_PI / 180.0);
    if (!cartesianMove(arm, node, grasp, "descend")) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }
  if (!checkStateValid(node, arm, validity_client, "after descend")) {
    RCLCPP_ERROR(node->get_logger(), "aborting: invalid state at after descend");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // 3.5) attach 物体到夹爪基座（touch_links = 手指，允许手指接触物体）
  {
    const std::vector<std::string> touch_links = {
      "ag145_gripper_finger1_finger_link",
      "ag145_gripper_finger2_finger_link",
      "ag145_gripper_finger1_finger_tip_link",
      "ag145_gripper_finger2_finger_tip_link",
      "ag145_gripper_finger1_inner_knuckle_link",
      "ag145_gripper_finger2_inner_knuckle_link",
    };
    if (!arm.attachObject(obj_name, "ag145_gripper_base_link", touch_links)) {
      RCLCPP_ERROR(node->get_logger(), "attachObject failed");
      rclcpp::shutdown(); spinner.join(); return 1;
    }
    RCLCPP_INFO(node->get_logger(), "=== attach detail ===");
    RCLCPP_INFO(node->get_logger(), "  attached object : %s", obj_name.c_str());
    RCLCPP_INFO(node->get_logger(), "  attach link     : ag145_gripper_base_link");
    for (size_t i = 0; i < touch_links.size(); ++i) {
      RCLCPP_INFO(node->get_logger(), "  touch_link[%zu]  : %s", i, touch_links[i].c_str());
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
  }
  if (!checkStateValid(node, arm, validity_client, "after attach")) {
    RCLCPP_ERROR(node->get_logger(), "aborting: invalid state at after attach");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // 3.6) 闭合夹爪（SRDF 命名姿态 closed = 0.0）
  if (!checkStateValid(node, arm, validity_client, "before close")) {
    RCLCPP_ERROR(node->get_logger(), "aborting: invalid state at before close");
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  {
    gripper.setNamedTarget("closed");
    if (!planAndExecute(gripper, node, "close gripper")) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }
  if (!checkStateValid(node, arm, validity_client, "after close")) {
    RCLCPP_ERROR(node->get_logger(), "aborting: invalid state at after close");
    rclcpp::shutdown(); spinner.join(); return 1;
  }

  // 4) Cartesian 抬升（物体随夹爪一起上升）
  if (!checkStateValid(node, arm, validity_client, "before lift")) {
    RCLCPP_ERROR(node->get_logger(), "aborting: invalid state at before lift");
    rclcpp::shutdown(); spinner.join(); return 1;
  }
  {
    geometry_msgs::msg::Pose lift;
    lift.position.x = obj_x;
    lift.position.y = obj_y;
    lift.position.z = lift_z;
    lift.orientation = quatRPY(M_PI, 0.0, pg.yaw_deg * M_PI / 180.0);
    if (!cartesianMove(arm, node, lift, "lift")) {
      rclcpp::shutdown(); spinner.join(); return 1;
    }
  }

  RCLCPP_INFO(node->get_logger(), "Grasp demo (candidate evaluator) finished.");
  rclcpp::shutdown();
  spinner.join();
  return 0;
}
