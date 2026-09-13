#include <atomic>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <unordered_map>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit/utils/moveit_error_code.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "grasp_geometry.hpp"
#include "pregrasp_ik.hpp"
#include "refreshed_tool_pose.hpp"
#include "target_lock.hpp"
#include "command_lease.hpp"

class VisionGraspPregraspGuarded : public rclcpp::Node
{
public:
  VisionGraspPregraspGuarded()
  : Node("vision_grasp_pregrasp_guarded")
  {
    declare_parameter("point_topic", "/yolo_target_point");
    declare_parameter("grasp_pose_topic", "/yolo_grasp_pose");
    declare_parameter("use_vision_yaw", false);
    declare_parameter("joint_state_topic", "/joint_states");
    declare_parameter("base_frame", "base_link");
    declare_parameter("eef_link", "tool0");
    declare_parameter("grasp_yaw", 0.0);
    declare_parameter("orientation_tolerance", 0.05235987756);
    declare_parameter("endpoint_position_tolerance", 0.003);
    declare_parameter("pregrasp_clearance", 0.070);
    // Commissioned 5 mm lower than the 25 mm endpoint after a successful
    // foam-supported grasp, retaining 20 mm clearance above the visual target.
    declare_parameter("near_clearance", 0.020);
    declare_parameter("pregrasp_position_tolerance", 0.010);
    declare_parameter("cartesian_step", 0.002);
    declare_parameter("max_cartesian_joint_step", 0.20);
    declare_parameter("descent_velocity_scaling", 0.03);
    declare_parameter("descent_acceleration_scaling", 0.03);
    declare_parameter("velocity_scaling", 0.15);
    declare_parameter("acceleration_scaling", 0.10);
    declare_parameter("ik_strict_joint_delta", 1.50);
    declare_parameter("ik_relaxed_joint_delta", 2.50);
    declare_parameter("pregrasp_candidate_planning_time", 8.0);
    declare_parameter("pregrasp_total_planning_time", 40.0);
    declare_parameter("target_x_min", -0.70);
    declare_parameter("target_x_max", -0.25);
    declare_parameter("target_y_min", 0.30);
    declare_parameter("target_y_max", 0.75);
    declare_parameter("target_z_min", -0.05);
    declare_parameter("target_z_max", 0.12);
    declare_parameter("target_timeout_sec", 0.50);
    declare_parameter("target_motion_tolerance", 0.005);
    declare_parameter("target_yaw_tolerance", 0.08726646260);
    declare_parameter("target_motion_confirmations", 3);
    declare_parameter("target_lock_sec", 60.0);
    declare_parameter("plan_max_age_sec", 60.0);
    declare_parameter("start_state_tolerance", 0.020);
    declare_parameter("enable_execution", false);
    declare_parameter("operator_confirmation", "DENIED");

    for (const auto * name : {"orientation_tolerance", "endpoint_position_tolerance",
        "cartesian_step", "max_cartesian_joint_step", "target_lock_sec",
        "target_yaw_tolerance", "ik_strict_joint_delta", "ik_relaxed_joint_delta",
        "pregrasp_candidate_planning_time", "pregrasp_total_planning_time"}) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    }
    (void)thor7_grasp::downward(get_parameter("grasp_yaw").as_double());
    if (get_parameter("target_motion_confirmations").as_int() < 2) {
      throw std::invalid_argument("target_motion_confirmations must be at least 2");
    }

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      get_parameter("point_topic").as_string(), 10,
      std::bind(&VisionGraspPregraspGuarded::targetCallback, this, std::placeholders::_1));
    grasp_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      get_parameter("grasp_pose_topic").as_string(), 10,
      std::bind(&VisionGraspPregraspGuarded::graspPoseCallback, this, std::placeholders::_1));
    joint_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      get_parameter("joint_state_topic").as_string(), 20,
      std::bind(&VisionGraspPregraspGuarded::jointStateCallback, this, std::placeholders::_1));

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10);

    operation_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    prepare_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/prepare_pregrasp",
      leased(&VisionGraspPregraspGuarded::prepare),
      rmw_qos_profile_services_default, operation_group_);

    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/execute_pregrasp",
      leased(&VisionGraspPregraspGuarded::execute),
      rmw_qos_profile_services_default, operation_group_);

    prepare_near_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/prepare_near",
      leased(&VisionGraspPregraspGuarded::prepareNear),
      rmw_qos_profile_services_default, operation_group_);

    execute_near_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/execute_near",
      leased(&VisionGraspPregraspGuarded::executeNear),
      rmw_qos_profile_services_default, operation_group_);

    // Internal endpoints are used only by the coordinator while it owns the
    // host command lease for the ENTIRE transaction, including gripper calls.
    const std::vector<std::pair<std::string, Operation>> operations{
      {"prepare_pregrasp", &VisionGraspPregraspGuarded::prepare},
      {"execute_pregrasp", &VisionGraspPregraspGuarded::execute},
      {"prepare_near", &VisionGraspPregraspGuarded::prepareNear},
      {"execute_near", &VisionGraspPregraspGuarded::executeNear}};
    for (const auto & operation : operations) {
      sequence_services_.push_back(create_service<Trigger>(
        "/thor7_vision_grasp/sequence/" + operation.first,
        [this, operation](const std::shared_ptr<Trigger::Request> req,
                         std::shared_ptr<Trigger::Response> res) {
          thor7_grasp::CommandLease probe;
          if (probe.acquired()) {
            res->success = false; res->message = "internal endpoint requires coordinator lease";
            return;
          }
          (this->*operation.second)(req, res);
        }, rmw_qos_profile_services_default, operation_group_));
    }

    RCLCPP_WARN(
      get_logger(),
      "Guarded pregrasp/near ready. Nothing runs automatically. Execution is %s. "
      "Pregrasp and near descent require separate prepare/execute calls; "
      "this node never commands the gripper. Yaw source=%s.",
      executionEnabled() ? "ENABLED" : "DISABLED",
      get_parameter("use_vision_yaw").as_bool() ? "VISION POSE" : "MANUAL PARAMETER");
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
  using Operation = void (VisionGraspPregraspGuarded::*)(
    const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>);
  std::function<void(const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response>)>
  leased(Operation operation)
  {
    return [this, operation](const std::shared_ptr<Trigger::Request> req,
                             std::shared_ptr<Trigger::Response> res) {
      thor7_grasp::CommandLease lease;
      if (!lease.acquired()) {
        res->success = false; res->message = "automatic sequence or another operation owns the robot";
        return;
      }
      (this->*operation)(req, res);
    };
  }
  std::vector<rclcpp::Service<Trigger>::SharedPtr> sequence_services_;

  struct BusyGuard
  {
    std::atomic_bool & busy;
    ~BusyGuard() { busy.store(false); }
  };

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (get_parameter("use_vision_yaw").as_bool()) {
      return;
    }
    if (msg->header.frame_id != get_parameter("base_frame").as_string()) {
      return;
    }
    if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
        !std::isfinite(msg->point.z)) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    const double yaw = get_parameter("grasp_yaw").as_double();
    target_lock_.observe(Eigen::Vector3d(msg->point.x, msg->point.y, msg->point.z), yaw,
      get_parameter("target_motion_tolerance").as_double(),
      get_parameter("target_yaw_tolerance").as_double(),
      static_cast<int>(get_parameter("target_motion_confirmations").as_int()));
    latest_target_ = *msg;
    latest_target_yaw_ = yaw;
    target_received_at_ = now();
    have_target_ = true;
  }

  static double axialDistance(double first, double second)
  {
    return std::abs(std::remainder(first - second, std::acos(-1.0)));
  }

  bool yawFromGraspPose(const geometry_msgs::msg::Pose & pose, double & yaw) const
  {
    Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    if (!std::isfinite(q.norm()) || q.norm() < 1e-9) {
      return false;
    }
    q.normalize();
    const Eigen::Vector3d tool_x = q * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d tool_z = q * Eigen::Vector3d::UnitZ();
    if (!tool_x.allFinite() || !tool_z.allFinite() ||
        tool_z.dot(-Eigen::Vector3d::UnitZ()) <
        std::cos(get_parameter("orientation_tolerance").as_double())) {
      return false;
    }
    yaw = std::atan2(tool_x.y(), tool_x.x());
    return std::isfinite(yaw);
  }

  void graspPoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!get_parameter("use_vision_yaw").as_bool() ||
        msg->header.frame_id != get_parameter("base_frame").as_string()) {
      return;
    }
    const auto & p = msg->pose.position;
    double yaw = 0.0;
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z) ||
        !yawFromGraspPose(msg->pose, yaw)) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    target_lock_.observe(Eigen::Vector3d(p.x, p.y, p.z), yaw,
      get_parameter("target_motion_tolerance").as_double(),
      get_parameter("target_yaw_tolerance").as_double(),
      static_cast<int>(get_parameter("target_motion_confirmations").as_int()));
    latest_target_.header = msg->header;
    latest_target_.point.x = p.x;
    latest_target_.point.y = p.y;
    latest_target_.point.z = p.z;
    latest_target_yaw_ = yaw;
    target_received_at_ = now();
    have_target_ = true;
  }

  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (msg->name.empty() || msg->name.size() != msg->position.size()) {
      return;
    }
    std::lock_guard<std::mutex> lock(joint_state_mutex_);
    const auto received = std::chrono::steady_clock::now();
    for (std::size_t i = 0; i < msg->name.size(); ++i) {
      if (std::isfinite(msg->position[i])) {
        joint_feedback_[msg->name[i]] = JointFeedback{msg->position[i], received};
      }
    }
  }

  bool executionEnabled() const
  {
    return get_parameter("enable_execution").as_bool() &&
      get_parameter("operator_confirmation").as_string() == "ARM_THOR7_PREGRASP";
  }

  void invalidatePlans()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    have_plan_ = false;
    have_near_plan_ = false;
    moveit_msgs::msg::DisplayTrajectory clear;
    display_pub_->publish(clear);
  }

  geometry_msgs::msg::Quaternion downwardOrientation(double yaw) const
  {
    // URDF: tool0 +Z points out through the fingers; +X is the opening axis.
    // Rz(yaw) Rx(pi): +Z -> base -Z, +X -> horizontal requested yaw.
    const auto q = thor7_grasp::downward(yaw);
    geometry_msgs::msg::Quaternion out;
    out.x = q.x(); out.y = q.y(); out.z = q.z(); out.w = q.w();
    return out;
  }

  moveit_msgs::msg::Constraints downwardConstraints(double yaw) const
  {
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::OrientationConstraint oc;
    oc.header.frame_id = get_parameter("base_frame").as_string();
    oc.link_name = get_parameter("eef_link").as_string();
    oc.orientation = downwardOrientation(yaw);
    oc.parameterization = moveit_msgs::msg::OrientationConstraint::ROTATION_VECTOR;
    oc.absolute_x_axis_tolerance = oc.absolute_y_axis_tolerance =
      oc.absolute_z_axis_tolerance = get_parameter("orientation_tolerance").as_double();
    oc.weight = 1.0;
    constraints.orientation_constraints.push_back(oc);
    return constraints;
  }

  Eigen::Isometry3d toolPose(const moveit::core::RobotState & state) const
  {
    // CurrentStateMonitor updates joints but may leave FK caches dirty.
    // Const getGlobalLinkTransform does NOT update them (Release skips assert).
    return thor7_grasp::refreshedToolPose(state,
      get_parameter("base_frame").as_string(), get_parameter("eef_link").as_string());
  }

  double orientationError(const Eigen::Isometry3d & actual, double yaw) const
  {
    const auto q = downwardOrientation(yaw);
    return Eigen::Quaterniond(actual.rotation()).angularDistance(
      Eigen::Quaterniond(q.w, q.x, q.y, q.z));
  }

  void verifyEndpoint(const moveit::core::RobotState & state,
                      const geometry_msgs::msg::Pose & expected, double yaw) const
  {
    const auto actual = toolPose(state);
    const double distance = (actual.translation() - Eigen::Vector3d(
      expected.position.x, expected.position.y, expected.position.z)).norm();
    const double angle = orientationError(actual, yaw);
    if (!thor7_grasp::endpointWithin(actual,
        Eigen::Vector3d(expected.position.x, expected.position.y, expected.position.z),
        yaw,
        get_parameter("endpoint_position_tolerance").as_double(),
        get_parameter("orientation_tolerance").as_double())) {
      std::ostringstream text;
      text << "tool0 FK endpoint rejected: position_error=" << distance * 1000.0
           << " mm orientation_error=" << angle * 180.0 / std::acos(-1.0) << " deg";
      throw std::runtime_error(text.str());
    }
  }

  void verifyPath(const robot_trajectory::RobotTrajectory & trajectory,
                  double minimum_z, bool require_downward, double yaw) const
  {
    // Check interpolated states as well as returned waypoints. These checks
    // supplement, not replace, MoveIt's collision checking.
    const auto * group = trajectory.getGroup();
    for (std::size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      const auto & a = trajectory.getWayPoint(i == 0 ? 0 : i - 1);
      const auto & b = trajectory.getWayPoint(i);
      std::vector<double> qa, qb;
      a.copyJointGroupPositions(group, qa); b.copyJointGroupPositions(group, qb);
      double largest = 0.0;
      for (std::size_t j = 0; j < qa.size(); ++j) {
        largest = std::max(largest, std::abs(qb[j] - qa[j]));
      }
      if (require_downward && largest > get_parameter("max_cartesian_joint_step").as_double()) {
        throw std::runtime_error("approach rejected: large joint jump");
      }
      const int steps = std::max(1, static_cast<int>(std::ceil(largest / 0.01)));
      for (int k = 0; k <= steps; ++k) {
        moveit::core::RobotState sample(a);
        a.interpolate(b, static_cast<double>(k) / steps, sample); sample.update();
        if (!sample.satisfiesBounds(group)) {
          throw std::runtime_error("path rejected: arm joint position exceeds model bounds");
        }
        const auto pose = toolPose(sample);
        if (!pose.matrix().allFinite() || pose.translation().z() < minimum_z - 0.003 ||
            (require_downward && orientationError(pose, yaw) >
             get_parameter("orientation_tolerance").as_double())) {
          std::ostringstream error;
          error << (require_downward ? "approach/descent" : "alignment")
                << " path rejected at waypoint " << i
                << ": tool0_z=" << pose.translation().z()
                << " m minimum_z=" << minimum_z - 0.003
                << " m orientation_error_deg=" << orientationError(pose, yaw) * 180.0 / std::acos(-1.0)
                << " downward_required=" << require_downward;
          throw std::runtime_error(error.str());
        }
      }
    }
  }

  void verifyExecution(const Plan & plan, double yaw)
  {
    const auto model = arm().getRobotModel();
    moveit::core::RobotState actual(model);
    actual.setToDefaultValues();
    const auto & joint_names = plan.trajectory_.joint_trajectory.joint_names;
    const auto check_time = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(joint_state_mutex_);
      for (const auto & joint_name : joint_names) {
        const auto found = joint_feedback_.find(joint_name);
        if (found == joint_feedback_.end()) {
          throw std::runtime_error("/joint_states has never reported " + joint_name);
        }
        const double age = std::chrono::duration<double>(
          check_time - found->second.received).count();
        if (!std::isfinite(age) || age < 0.0 || age > 1.0) {
          throw std::runtime_error("/joint_states feedback is stale for " + joint_name);
        }
        actual.setVariablePosition(joint_name, found->second.position);
      }
    }
    actual.update();

    robot_trajectory::RobotTrajectory planned(model, "thor7_arm");
    planned.setRobotTrajectoryMsg(actual, plan.trajectory_);
    const auto pose = toolPose(planned.getLastWayPoint());
    geometry_msgs::msg::Pose expected;
    expected.position.x = pose.translation().x();
    expected.position.y = pose.translation().y();
    expected.position.z = pose.translation().z();
    verifyEndpoint(actual, expected, yaw);
  }

  bool checkPlanBeforeExecution(
    Plan & plan,
    geometry_msgs::msg::PointStamped & planned_target,
    double planned_yaw,
    rclcpp::Time & created,
    bool & have_plan,
    std::string & reason, bool near = false)
  {
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_plan) {
        reason = "no prepared plan";
        return false;
      }
      have_plan = false;  // Every prepared trajectory is single-use.
    }

    const double plan_age = (now() - created).seconds();
    const double max_plan_age = get_parameter("plan_max_age_sec").as_double();
    if (plan_age > max_plan_age) {
      std::ostringstream message;
      message << "prepared plan expired after " << plan_age
              << " s (limit " << max_plan_age << " s); prepare again";
      reason = message.str();
      return false;
    }

    geometry_msgs::msg::PointStamped current_target;
    double current_yaw = 0.0;
    if (!(near ? getLockedTarget(current_target, current_yaw, reason) :
          getFreshTarget(current_target, current_yaw, reason))) {
      return false;
    }
    const double target_shift = std::hypot(
      std::hypot(
        current_target.point.x - planned_target.point.x,
        current_target.point.y - planned_target.point.y),
      current_target.point.z - planned_target.point.z);
    if (target_shift > get_parameter("target_motion_tolerance").as_double()) {
      reason = "target moved more than tolerance; prepare again";
      return false;
    }
    if (axialDistance(current_yaw, planned_yaw) >
        get_parameter("target_yaw_tolerance").as_double()) {
      reason = "target yaw moved more than tolerance; prepare again";
      return false;
    }

    auto current_state = arm().getCurrentState(1.0);
    if (!current_state) {
      reason = "cannot read current robot state";
      return false;
    }
    const auto & trajectory = plan.trajectory_.joint_trajectory;
    if (trajectory.points.empty()) {
      reason = "prepared trajectory is empty";
      return false;
    }
    const auto & start = trajectory.points.front().positions;
    if (start.size() != trajectory.joint_names.size()) {
      reason = "prepared trajectory has invalid start state";
      return false;
    }
    const double tolerance = get_parameter("start_state_tolerance").as_double();
    for (std::size_t i = 0; i < start.size(); ++i) {
      const double actual = current_state->getVariablePosition(trajectory.joint_names[i]);
      if (std::abs(actual - start[i]) > tolerance) {
        reason = "robot moved since planning; prepare again";
        return false;
      }
    }
    // A changed grasp yaw must not silently execute a plan for the old yaw.
    robot_trajectory::RobotTrajectory checked(arm().getRobotModel(), "thor7_arm");
    checked.setRobotTrajectoryMsg(*current_state, plan.trajectory_);
    if (orientationError(toolPose(checked.getLastWayPoint()), planned_yaw) >
        get_parameter("orientation_tolerance").as_double()) {
      reason = "prepared endpoint does not match downward grasp orientation; prepare again";
      return false;
    }
    return true;
  }

  bool getLockedTarget(
    geometry_msgs::msg::PointStamped & target, double & yaw, std::string & reason)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!target_lock_.valid(thor7_grasp::TargetLock::Clock::now(),
        get_parameter("target_lock_sec").as_double())) {
      reason = target_lock_.moved() ?
        "locked target moved; prepare and execute pregrasp again" :
        "no valid target lock (not executed, failed, consumed or expired); prepare and execute pregrasp again";
      return false;
    }
    target = locked_target_;
    yaw = locked_target_yaw_;
    return true;
  }

  bool getFreshTarget(
    geometry_msgs::msg::PointStamped & target, double & yaw, std::string & reason)
  {
    rclcpp::Time received;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_target_) {
        reason = get_parameter("use_vision_yaw").as_bool() ?
          "no stable /yolo_grasp_pose received" : "no /yolo_target_point received";
        return false;
      }
      target = latest_target_;
      yaw = get_parameter("use_vision_yaw").as_bool() ?
        latest_target_yaw_ : get_parameter("grasp_yaw").as_double();
      received = target_received_at_;
    }

    const double age = (now() - received).seconds();
    if (age > get_parameter("target_timeout_sec").as_double()) {
      reason = get_parameter("use_vision_yaw").as_bool() ?
        "grasp pose is stale" : "target is stale";
      return false;
    }

    const auto & p = target.point;
    if (p.x < get_parameter("target_x_min").as_double() ||
        p.x > get_parameter("target_x_max").as_double() ||
        p.y < get_parameter("target_y_min").as_double() ||
        p.y > get_parameter("target_y_max").as_double() ||
        p.z < get_parameter("target_z_min").as_double() ||
        p.z > get_parameter("target_z_max").as_double()) {
      reason = "target is outside the configured pick workspace";
      return false;
    }
    return true;
  }

  moveit::planning_interface::MoveGroupInterface & arm()
  {
    if (!arm_) {
      arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
        shared_from_this(), "thor7_arm");
      arm_->setPoseReferenceFrame(get_parameter("base_frame").as_string());
      arm_->setEndEffectorLink(get_parameter("eef_link").as_string());
      if (!arm_->getRobotModel()->hasLinkModel(get_parameter("eef_link").as_string()) ||
          !arm_->getRobotModel()->hasLinkModel(get_parameter("base_frame").as_string())) {
        throw std::runtime_error("configured base/eef link is missing from the robot model");
      }
      arm_->setPlanningPipelineId("ompl");
      arm_->setPlanningTime(5.0);
      arm_->setNumPlanningAttempts(5);
    }
    arm_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    arm_->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    return *arm_;
  }

  void prepare(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "another prepare/execute operation is active";
      return;
    }
    BusyGuard guard{busy_};

    // A new prepare request always invalidates every previous trajectory.
    invalidatePlans();
    { std::lock_guard<std::mutex> lock(data_mutex_); target_lock_.reset(); }

    try {
      geometry_msgs::msg::PointStamped target;
      double grasp_yaw = 0.0;
      std::string reason;
      if (!getFreshTarget(target, grasp_yaw, reason)) {
        response->success = false;
        response->message = reason;
        return;
      }

      auto & move_group = arm();
      move_group.clearPathConstraints();
      auto start = move_group.getCurrentState(1.0);
      if (!start) { throw std::runtime_error("cannot read current robot state"); }
      const auto start_pose = toolPose(*start);
      geometry_msgs::msg::Pose pregrasp;
      pregrasp.orientation = downwardOrientation(grasp_yaw);
      pregrasp.position = target.point;
      pregrasp.position.z += get_parameter("pregrasp_clearance").as_double();

      if (start_pose.translation().z() < pregrasp.position.z) {
        std::ostringstream message;
        message << "start tool0 z=" << start_pose.translation().z()
                << " m is below pregrasp z=" << pregrasp.position.z
                << " m; move to a clear staging pose first";
        throw std::runtime_error(message.str());
      }

      // The start pose may be tilted, so there is intentionally no path-wide
      // orientation constraint. The endpoint and feedback are checked below.
      // This lets yaw rotation overlap with the translational part of travel.
      moveit_msgs::msg::Constraints transit_constraints;
      moveit_msgs::msg::PositionConstraint height;
      height.header.frame_id = get_parameter("base_frame").as_string();
      height.link_name = get_parameter("eef_link").as_string();
      height.weight = 1.0;
      shape_msgs::msg::SolidPrimitive region;
      region.type = shape_msgs::msg::SolidPrimitive::BOX;
      region.dimensions = {10.0, 10.0, 10.0};
      geometry_msgs::msg::Pose region_pose;
      region_pose.orientation.w = 1.0;
      region_pose.position.z = pregrasp.position.z - 0.003 + 5.0;
      height.constraint_region.primitives.push_back(region);
      height.constraint_region.primitive_poses.push_back(region_pose);
      transit_constraints.position_constraints.push_back(height);
      Plan plan;
      bool planned = false;
      std::size_t ik_count = 0, plan_attempts = 0;
      std::string last_failure;
      const Eigen::Vector3d position(pregrasp.position.x, pregrasp.position.y, pregrasp.position.z);
      std::vector<double> start_joints;
      start->copyJointGroupPositions("thor7_arm", start_joints);
      std::ostringstream start_text;
      for (const double joint : start_joints) { start_text << joint << ' '; }
      RCLCPP_INFO(get_logger(), "Pregrasp IK: xyz=(%.6f, %.6f, %.6f), yaw candidates=%.3f/%.3f deg, start joints=[%s] rad",
        position.x(), position.y(), position.z(), grasp_yaw * 180.0 / std::acos(-1.0),
        grasp_yaw * 180.0 / std::acos(-1.0) + 180.0, start_text.str().c_str());
      const auto planning_deadline = std::chrono::steady_clock::now() +
        std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(
          get_parameter("pregrasp_total_planning_time").as_double()));
      std::vector<std::vector<double>> tried;
      for (const double window : {get_parameter("ik_strict_joint_delta").as_double(),
          get_parameter("ik_relaxed_joint_delta").as_double(), 0.0}) {
        if (std::chrono::steady_clock::now() >= planning_deadline) { break; }
        const auto candidates = thor7_grasp::pregraspIkCandidates(*start, position, grasp_yaw,
          get_parameter("base_frame").as_string(), get_parameter("eef_link").as_string(), window,
          get_parameter("endpoint_position_tolerance").as_double(),
          get_parameter("orientation_tolerance").as_double());
        ik_count += candidates.size();
        RCLCPP_INFO(get_logger(), "Pregrasp IK search: window=%.2f rad (0=model bounds), candidates=%zu",
          window, candidates.size());
        for (const auto & candidate : candidates) {
          const double remaining = std::chrono::duration<double>(
            planning_deadline - std::chrono::steady_clock::now()).count();
          if (remaining <= 0.0) { break; }
          const bool duplicate = std::any_of(tried.begin(), tried.end(), [&](const auto & values) {
            double delta = 0.0;
            for (std::size_t i = 0; i < values.size(); ++i) {
              delta = std::max(delta, std::abs(values[i] - candidate.joints[i]));
            }
            return delta < 0.01;
          });
          if (duplicate) { continue; }
          tried.push_back(candidate.joints);
          auto candidate_pose = pregrasp;
          candidate_pose.orientation = downwardOrientation(candidate.yaw);

          enum class CandidateResult { accepted, planning_failed, verification_failed };
          auto attempt_plan = [&](bool constrained, Plan & candidate_plan) {
            const double time_left = std::chrono::duration<double>(
              planning_deadline - std::chrono::steady_clock::now()).count();
            if (time_left <= 0.0) {
              last_failure = "pregrasp total planning deadline expired";
              return CandidateResult::planning_failed;
            }
            ++plan_attempts;
            move_group.setStartState(*start);
            move_group.setJointValueTarget(candidate.joints);
            if (constrained) {
              move_group.setPathConstraints(transit_constraints);
            } else {
              move_group.clearPathConstraints();
            }
            move_group.setPlanningTime(std::min(
              get_parameter("pregrasp_candidate_planning_time").as_double(), time_left));
            move_group.setNumPlanningAttempts(1);
            const auto result = move_group.plan(candidate_plan);
            move_group.clearPathConstraints();
            move_group.setPlanningTime(5.0);
            move_group.setNumPlanningAttempts(5);
            const std::string mode = constrained ? "height-constrained" : "unconstrained";
            if (!static_cast<bool>(result) ||
                candidate_plan.trajectory_.joint_trajectory.points.empty()) {
              last_failure = mode + " MoveIt planning=" +
                moveit::core::error_code_to_string(result) + " (code=" +
                std::to_string(result.val) + ")";
              RCLCPP_WARN(
                get_logger(), "Pregrasp candidate yaw=%.3f rad %s rejected: %s",
                candidate.yaw, mode.c_str(), last_failure.c_str());
              return CandidateResult::planning_failed;
            }
            try {
              robot_trajectory::RobotTrajectory transit(
                move_group.getRobotModel(), "thor7_arm");
              transit.setRobotTrajectoryMsg(*start, candidate_plan.trajectory_);
              // OMPL supplies collision checking. This dense FK pass additionally
              // rejects any unconstrained route that dips into the pick volume.
              verifyPath(transit, pregrasp.position.z, false, candidate.yaw);
              verifyEndpoint(transit.getLastWayPoint(), candidate_pose, candidate.yaw);
            } catch (const std::exception & error) {
              last_failure = mode + " trajectory verification: " + error.what();
              RCLCPP_WARN(
                get_logger(), "Pregrasp candidate yaw=%.3f rad %s rejected: %s",
                candidate.yaw, mode.c_str(), last_failure.c_str());
              return CandidateResult::verification_failed;
            }
            return CandidateResult::accepted;
          };

          Plan candidate_plan;
          auto candidate_result = attempt_plan(false, candidate_plan);
          if (candidate_result == CandidateResult::verification_failed) {
            RCLCPP_INFO(
              get_logger(),
              "Unconstrained candidate was unsafe by FK height/endpoint verification; retrying with path height constraint");
            Plan constrained_plan;
            candidate_result = attempt_plan(true, constrained_plan);
            if (candidate_result == CandidateResult::accepted) {
              candidate_plan = std::move(constrained_plan);
            }
          }
          if (candidate_result != CandidateResult::accepted) { continue; }

          plan = std::move(candidate_plan);
          grasp_yaw = candidate.yaw;
          pregrasp = candidate_pose;
          planned = true;
          RCLCPP_INFO(get_logger(), "Pregrasp candidate selected: yaw=%.3f rad max_joint_delta=%.3f rad, window=%.2f rad",
            grasp_yaw, candidate.max_delta, window);
          break;
        }
        if (planned) { break; }
      }
      if (!planned) {
        response->success = false;
        response->message = ik_count == 0 ?
          "no IK candidate found for either downward yaw after local and model-bounded search; target may be unreachable or solver did not converge" :
          "no verified pregrasp path after " + std::to_string(plan_attempts) + " candidate plans: " + last_failure;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      moveit_msgs::msg::DisplayTrajectory display;
      display.model_id = "thor7v3";
      display.trajectory_start = plan.start_state_;
      display.trajectory.push_back(plan.trajectory_);
      display_pub_->publish(display);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prepared_plan_ = plan;
        planned_target_ = target;
        planned_target_yaw_ = grasp_yaw;
        plan_created_at_ = now();
        have_plan_ = true;
      }

      std::ostringstream message;
      message << "prepared pregrasp at (" << target.point.x << ", "
              << target.point.y << ", " << pregrasp.position.z
              << "); combined collision-aware transit and yaw rotation to the final pregrasp pose; "
              << "the next descent is authorized only after endpoint feedback FK passes; "
              << "tool0 +Z=base -Z, opening yaw=" << grasp_yaw
              << (get_parameter("use_vision_yaw").as_bool() ? " rad from vision" : " rad manual")
              << "; FK endpoint PASS; inspect RViz before execute_pregrasp";
      response->success = true;
      response->message = message.str();
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("prepare exception: ") + error.what();
      if (arm_) { arm_->clearPathConstraints(); }
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    }
  }

  void execute(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "another prepare/execute operation is active";
      return;
    }
    BusyGuard guard{busy_};

    if (!executionEnabled()) {
      response->success = false;
      response->message =
        "execution disabled; require enable_execution=true and "
        "operator_confirmation=ARM_THOR7_PREGRASP";
      return;
    }

    try {
      Plan plan;
      geometry_msgs::msg::PointStamped planned_target;
      double planned_yaw = 0.0;
      rclcpp::Time created;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        plan = prepared_plan_;
        planned_target = planned_target_;
        planned_yaw = planned_target_yaw_;
        created = plan_created_at_;
      }
      std::string reason;
      if (!checkPlanBeforeExecution(
          plan, planned_target, planned_yaw, created, have_plan_, reason)) {
        response->success = false;
        response->message = reason;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      auto & move_group = arm();
      RCLCPP_WARN(
        get_logger(),
        "Executing PREGRASP ONLY at %.1f%% velocity; no descent, no gripper command",
        get_parameter("velocity_scaling").as_double() * 100.0);
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        locked_target_ = planned_target;
        locked_target_yaw_ = planned_yaw;
        target_lock_.begin(Eigen::Vector3d(planned_target.point.x,
          planned_target.point.y, planned_target.point.z), planned_yaw);
        if (!target_lock_.commit()) {
          throw std::runtime_error("could not commit the verified pregrasp target");
        }
      }
      const auto result = move_group.execute(plan);
      response->success = result == moveit::core::MoveItErrorCode::SUCCESS;
      if (response->success) {
        RCLCPP_INFO(get_logger(), "PREGRASP controller success; validating cached joint feedback FK");
        verifyExecution(plan, planned_yaw);
        RCLCPP_INFO(get_logger(), "PREGRASP cached joint feedback FK PASS");
      }
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (response->success) {
          if (!target_lock_.arrive(thor7_grasp::TargetLock::Clock::now())) {
            throw std::runtime_error("pregrasp reached but target moved during approach; near descent not authorized");
          }
        } else { target_lock_.reset(); }
      }
      response->message = response->success ?
        "pregrasp execution and feedback FK passed; clearance_mm=" +
        std::to_string(get_parameter("pregrasp_clearance").as_double() * 1000.0) +
        "; target locked for " + std::to_string(get_parameter("target_lock_sec").as_double()) +
        " s from arrival; descent uses the pre-dispatch XYZ/yaw snapshot; keep object fixed" :
        "pregrasp execution failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("execute exception: ") + error.what();
      { std::lock_guard<std::mutex> lock(data_mutex_); target_lock_.reset(); }
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  void prepareNear(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "another prepare/execute operation is active";
      return;
    }
    BusyGuard guard{busy_};
    invalidatePlans();

    try {
      geometry_msgs::msg::PointStamped target;
      double grasp_yaw = 0.0;
      std::string reason;
      if (!getLockedTarget(target, grasp_yaw, reason)) {
        response->success = false;
        response->message = reason;
        return;
      }

      auto & move_group = arm();
      auto current_state = move_group.getCurrentState(1.0);
      if (!current_state) {
        response->success = false;
        response->message = "cannot read current robot state";
        return;
      }
      const auto actual_tool = toolPose(*current_state);
      geometry_msgs::msg::Pose current_pose;
      current_pose.position.x = actual_tool.translation().x();
      current_pose.position.y = actual_tool.translation().y();
      current_pose.position.z = actual_tool.translation().z();
      current_pose.orientation = downwardOrientation(grasp_yaw);
      if (orientationError(actual_tool, grasp_yaw) >
          get_parameter("orientation_tolerance").as_double()) {
        throw std::runtime_error("near descent refused: tool0 is not at the downward grasp orientation");
      }
      const double pregrasp_z =
        target.point.z + get_parameter("pregrasp_clearance").as_double();
      const double xy_error = std::hypot(
        current_pose.position.x - target.point.x,
        current_pose.position.y - target.point.y);
      const double z_error = std::abs(current_pose.position.z - pregrasp_z);
      const double position_tolerance =
        get_parameter("pregrasp_position_tolerance").as_double();
      if (xy_error > position_tolerance || z_error > position_tolerance) {
        std::ostringstream message;
        message << "robot is not at the expected pregrasp: xy_error="
                << xy_error << " m z_error=" << z_error << " m";
        response->success = false;
        response->message = message.str();
        return;
      }

      geometry_msgs::msg::Pose near_pose = current_pose;
      near_pose.orientation = downwardOrientation(grasp_yaw);
      near_pose.position.z =
        target.point.z + get_parameter("near_clearance").as_double();
      const double descent_distance = current_pose.position.z - near_pose.position.z;
      if (descent_distance <= 0.005 || descent_distance > 0.060) {
        response->success = false;
        response->message = "requested near descent is outside the guarded 5-60 mm range";
        return;
      }

      move_group.setStartState(*current_state);
      moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
      const std::vector<geometry_msgs::msg::Pose> waypoints{near_pose};
      const double fraction = move_group.computeCartesianPath(
        waypoints,
        get_parameter("cartesian_step").as_double(),
        0.0,
        cartesian_trajectory,
        downwardConstraints(grasp_yaw),
        true);
      if (!std::isfinite(fraction) || fraction < 0.999) {
        std::ostringstream message;
        message << "collision-aware near descent incomplete: fraction=" << fraction;
        response->success = false;
        response->message = message.str();
        RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
        return;
      }

      const auto & points = cartesian_trajectory.joint_trajectory.points;
      if (points.empty()) {
        response->success = false;
        response->message = "Cartesian planner returned an empty trajectory";
        return;
      }
      const double max_joint_step =
        get_parameter("max_cartesian_joint_step").as_double();
      for (std::size_t i = 1; i < points.size(); ++i) {
        const auto & previous = points[i - 1].positions;
        const auto & current = points[i].positions;
        if (previous.size() != current.size()) {
          response->success = false;
          response->message = "Cartesian trajectory has inconsistent joint positions";
          return;
        }
        for (std::size_t joint = 0; joint < current.size(); ++joint) {
          if (std::abs(current[joint] - previous[joint]) > max_joint_step) {
            response->success = false;
            response->message = "Cartesian trajectory contains a large joint jump";
            return;
          }
        }
      }

      robot_trajectory::RobotTrajectory timed_trajectory(
        move_group.getRobotModel(), "thor7_arm");
      timed_trajectory.setRobotTrajectoryMsg(
        *current_state, cartesian_trajectory.joint_trajectory);
      verifyPath(timed_trajectory, near_pose.position.z, true, grasp_yaw);
      verifyEndpoint(timed_trajectory.getLastWayPoint(), near_pose, grasp_yaw);
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterizer;
      if (!time_parameterizer.computeTimeStamps(
          timed_trajectory,
          get_parameter("descent_velocity_scaling").as_double(),
          get_parameter("descent_acceleration_scaling").as_double())) {
        response->success = false;
        response->message = "could not time-parameterize the near descent";
        return;
      }
      timed_trajectory.getRobotTrajectoryMsg(cartesian_trajectory);

      geometry_msgs::msg::PointStamped still_locked;
      double still_locked_yaw = 0.0;
      if (!getLockedTarget(still_locked, still_locked_yaw, reason)) {
        throw std::runtime_error(reason);
      }
      if (axialDistance(still_locked_yaw, grasp_yaw) >
          get_parameter("target_yaw_tolerance").as_double()) {
        throw std::runtime_error("locked target yaw changed while planning near descent");
      }

      Plan plan;
      moveit::core::robotStateToRobotStateMsg(*current_state, plan.start_state_);
      plan.trajectory_ = cartesian_trajectory;

      moveit_msgs::msg::DisplayTrajectory display;
      display.model_id = "thor7v3";
      display.trajectory_start = plan.start_state_;
      display.trajectory.push_back(plan.trajectory_);
      display_pub_->publish(display);

      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        prepared_near_plan_ = plan;
        planned_near_target_ = target;
        planned_near_target_yaw_ = grasp_yaw;
        near_plan_created_at_ = now();
        have_near_plan_ = true;
      }

      std::ostringstream message;
      message << "prepared vertical near descent " << descent_distance * 1000.0
              << " mm using locked target; object must remain fixed; stop "
              << get_parameter("near_clearance").as_double() * 1000.0
              << " mm above target; inspect RViz before execute_near";
      response->success = true;
      response->message = message.str();
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("prepare near exception: ") + error.what();
    }
  }

  void executeNear(
    const std::shared_ptr<Trigger::Request>,
    std::shared_ptr<Trigger::Response> response)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "another prepare/execute operation is active";
      return;
    }
    BusyGuard guard{busy_};

    if (!executionEnabled()) {
      response->success = false;
      response->message =
        "execution disabled; require enable_execution=true and "
        "operator_confirmation=ARM_THOR7_PREGRASP";
      return;
    }

    try {
      Plan plan;
      geometry_msgs::msg::PointStamped planned_target;
      double planned_yaw = 0.0;
      rclcpp::Time created;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        plan = prepared_near_plan_;
        planned_target = planned_near_target_;
        planned_yaw = planned_near_target_yaw_;
        created = near_plan_created_at_;
      }
      std::string reason;
      if (!checkPlanBeforeExecution(
          plan, planned_target, planned_yaw, created, have_near_plan_, reason, true)) {
        response->success = false;
        response->message = reason;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Executing VERTICAL NEAR DESCENT at %.1f%% velocity; no gripper command",
        get_parameter("descent_velocity_scaling").as_double() * 100.0);
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        if (!target_lock_.valid(thor7_grasp::TargetLock::Clock::now(),
            get_parameter("target_lock_sec").as_double())) {
          throw std::runtime_error("target lock invalidated before descent dispatch");
        }
        target_lock_.reset();  // One descent attempt, including failed executions.
      }
      const auto result = arm().execute(plan);
      response->success = result == moveit::core::MoveItErrorCode::SUCCESS;
      if (response->success) {
        RCLCPP_INFO(get_logger(), "NEAR controller success; validating cached joint feedback FK");
        verifyExecution(plan, planned_yaw);
        RCLCPP_INFO(get_logger(), "NEAR cached joint feedback FK PASS");
      }
      response->message = response->success ?
        "near descent and feedback FK passed; clearance_mm=" +
        std::to_string(get_parameter("near_clearance").as_double() * 1000.0) :
        "near descent execution failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("execute near exception: ") + error.what();
      RCLCPP_ERROR(get_logger(), "%s", response->message.c_str());
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr grasp_pose_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Service<Trigger>::SharedPtr prepare_service_;
  rclcpp::Service<Trigger>::SharedPtr execute_service_;
  rclcpp::Service<Trigger>::SharedPtr prepare_near_service_;
  rclcpp::Service<Trigger>::SharedPtr execute_near_service_;
  rclcpp::CallbackGroup::SharedPtr operation_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

  std::mutex data_mutex_;
  std::mutex joint_state_mutex_;
  struct JointFeedback
  {
    double position;
    std::chrono::steady_clock::time_point received;
  };
  std::unordered_map<std::string, JointFeedback> joint_feedback_;
  geometry_msgs::msg::PointStamped latest_target_;
  geometry_msgs::msg::PointStamped planned_target_;
  geometry_msgs::msg::PointStamped planned_near_target_;
  double latest_target_yaw_{0.0};
  double planned_target_yaw_{0.0};
  double planned_near_target_yaw_{0.0};
  double locked_target_yaw_{0.0};
  rclcpp::Time target_received_at_;
  rclcpp::Time plan_created_at_;
  rclcpp::Time near_plan_created_at_;
  Plan prepared_plan_;
  Plan prepared_near_plan_;
  bool have_target_{false};
  bool have_plan_{false};
  bool have_near_plan_{false};
  thor7_grasp::TargetLock target_lock_;
  geometry_msgs::msg::PointStamped locked_target_;
  std::atomic_bool busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionGraspPregraspGuarded>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
