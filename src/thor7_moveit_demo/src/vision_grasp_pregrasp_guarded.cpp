#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class VisionGraspPregraspGuarded : public rclcpp::Node
{
public:
  VisionGraspPregraspGuarded()
  : Node("vision_grasp_pregrasp_guarded")
  {
    declare_parameter("point_topic", "/yolo_target_point");
    declare_parameter("base_frame", "base_link");
    declare_parameter("eef_link", "tool0");
    declare_parameter("pregrasp_clearance", 0.050);
    declare_parameter("near_clearance", 0.005);
    declare_parameter("pregrasp_position_tolerance", 0.010);
    declare_parameter("cartesian_step", 0.002);
    declare_parameter("max_cartesian_joint_step", 0.20);
    declare_parameter("descent_velocity_scaling", 0.03);
    declare_parameter("descent_acceleration_scaling", 0.03);
    declare_parameter("velocity_scaling", 0.05);
    declare_parameter("acceleration_scaling", 0.05);
    declare_parameter("ik_strict_joint_delta", 1.50);
    declare_parameter("ik_relaxed_joint_delta", 2.50);
    declare_parameter("target_x_min", -0.70);
    declare_parameter("target_x_max", -0.25);
    declare_parameter("target_y_min", 0.30);
    declare_parameter("target_y_max", 0.75);
    declare_parameter("target_z_min", -0.05);
    declare_parameter("target_z_max", 0.08);
    declare_parameter("target_timeout_sec", 0.50);
    declare_parameter("target_motion_tolerance", 0.005);
    declare_parameter("plan_max_age_sec", 60.0);
    declare_parameter("start_state_tolerance", 0.020);
    declare_parameter("enable_execution", false);
    declare_parameter("operator_confirmation", "DENIED");

    target_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      get_parameter("point_topic").as_string(), 10,
      std::bind(&VisionGraspPregraspGuarded::targetCallback, this, std::placeholders::_1));

    display_pub_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>(
      "/display_planned_path", 10);

    operation_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    prepare_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/prepare_pregrasp",
      std::bind(
        &VisionGraspPregraspGuarded::prepare,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_group_);

    execute_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/execute_pregrasp",
      std::bind(
        &VisionGraspPregraspGuarded::execute,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_group_);

    prepare_near_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/prepare_near",
      std::bind(
        &VisionGraspPregraspGuarded::prepareNear,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_group_);

    execute_near_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/execute_near",
      std::bind(
        &VisionGraspPregraspGuarded::executeNear,
        this, std::placeholders::_1, std::placeholders::_2),
      rmw_qos_profile_services_default, operation_group_);

    RCLCPP_WARN(
      get_logger(),
      "Guarded pregrasp/near ready. Nothing runs automatically. Execution is %s. "
      "Pregrasp and near descent require separate prepare/execute calls; "
      "this node never commands the gripper.",
      executionEnabled() ? "ENABLED" : "DISABLED");
  }

private:
  using Trigger = std_srvs::srv::Trigger;
  using Plan = moveit::planning_interface::MoveGroupInterface::Plan;

  struct BusyGuard
  {
    std::atomic_bool & busy;
    ~BusyGuard() { busy.store(false); }
  };

  void targetCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
  {
    if (msg->header.frame_id != get_parameter("base_frame").as_string()) {
      return;
    }
    if (!std::isfinite(msg->point.x) || !std::isfinite(msg->point.y) ||
        !std::isfinite(msg->point.z)) {
      return;
    }
    std::lock_guard<std::mutex> lock(data_mutex_);
    latest_target_ = *msg;
    target_received_at_ = now();
    have_target_ = true;
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
  }

  bool checkPlanBeforeExecution(
    Plan & plan,
    geometry_msgs::msg::PointStamped & planned_target,
    rclcpp::Time & created,
    bool & have_plan,
    std::string & reason)
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
    if (!getFreshTarget(current_target, reason)) {
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
    return true;
  }

  bool getFreshTarget(geometry_msgs::msg::PointStamped & target, std::string & reason)
  {
    rclcpp::Time received;
    {
      std::lock_guard<std::mutex> lock(data_mutex_);
      if (!have_target_) {
        reason = "no /yolo_target_point received";
        return false;
      }
      target = latest_target_;
      received = target_received_at_;
    }

    const double age = (now() - received).seconds();
    if (age > get_parameter("target_timeout_sec").as_double()) {
      reason = "target is stale";
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
      arm_->setPlanningPipelineId("ompl");
      arm_->setPlanningTime(5.0);
      arm_->setNumPlanningAttempts(5);
    }
    arm_->setMaxVelocityScalingFactor(get_parameter("velocity_scaling").as_double());
    arm_->setMaxAccelerationScalingFactor(get_parameter("acceleration_scaling").as_double());
    return *arm_;
  }

  bool solveClosestIk(
    moveit::planning_interface::MoveGroupInterface & move_group,
    const geometry_msgs::msg::Pose & pose,
    std::vector<double> & goal)
  {
    auto current_state = move_group.getCurrentState(1.0);
    if (!current_state) {
      return false;
    }
    const auto * group = move_group.getRobotModel()->getJointModelGroup("thor7_arm");
    if (!group) {
      return false;
    }

    std::vector<double> seed;
    current_state->copyJointGroupPositions(group, seed);

    Eigen::Isometry3d target = Eigen::Isometry3d::Identity();
    target.translation() = Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z);
    const Eigen::Quaterniond q(
      pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z);
    target.linear() = q.normalized().toRotationMatrix();

    const std::vector<double> windows{
      get_parameter("ik_strict_joint_delta").as_double(),
      get_parameter("ik_relaxed_joint_delta").as_double()};

    for (const double window : windows) {
      double best_cost = std::numeric_limits<double>::infinity();
      std::vector<double> best;
      for (int attempt = 0; attempt < 8; ++attempt) {
        auto candidate = std::make_shared<moveit::core::RobotState>(*current_state);
        const std::vector<double> limits(seed.size(), window);
        if (!candidate->setFromIK(
              group, target, get_parameter("eef_link").as_string(), limits, 0.05)) {
          continue;
        }
        std::vector<double> values;
        candidate->copyJointGroupPositions(group, values);
        double cost = 0.0;
        for (std::size_t i = 0; i < seed.size(); ++i) {
          const double delta = values[i] - seed[i];
          cost += delta * delta;
        }
        if (cost < best_cost) {
          best_cost = cost;
          best = values;
        }
      }
      if (!best.empty()) {
        goal = best;
        return true;
      }
    }
    return false;
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

    try {
      geometry_msgs::msg::PointStamped target;
      std::string reason;
      if (!getFreshTarget(target, reason)) {
        response->success = false;
        response->message = reason;
        return;
      }

      auto & move_group = arm();
      move_group.setStartStateToCurrentState();
      geometry_msgs::msg::Pose pregrasp;
      pregrasp.orientation = move_group.getCurrentPose().pose.orientation;
      pregrasp.position = target.point;
      pregrasp.position.z += get_parameter("pregrasp_clearance").as_double();

      std::vector<double> goal;
      if (!solveClosestIk(move_group, pregrasp, goal)) {
        response->success = false;
        response->message = "no nearby IK solution for pregrasp";
        return;
      }

      move_group.setStartStateToCurrentState();
      move_group.setJointValueTarget(goal);
      Plan plan;
      if (!static_cast<bool>(move_group.plan(plan))) {
        response->success = false;
        response->message = "collision-aware pregrasp planning failed";
        return;
      }
      if (plan.trajectory_.joint_trajectory.points.empty()) {
        response->success = false;
        response->message = "planner returned an empty trajectory";
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
        plan_created_at_ = now();
        have_plan_ = true;
      }

      std::ostringstream message;
      message << "prepared pregrasp at (" << target.point.x << ", "
              << target.point.y << ", " << pregrasp.position.z
              << "); inspect RViz before execute_pregrasp";
      response->success = true;
      response->message = message.str();
      RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("prepare exception: ") + error.what();
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
      rclcpp::Time created;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        plan = prepared_plan_;
        planned_target = planned_target_;
        created = plan_created_at_;
      }
      std::string reason;
      if (!checkPlanBeforeExecution(
          plan, planned_target, created, have_plan_, reason)) {
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
      const auto result = move_group.execute(plan);
      response->success = result == moveit::core::MoveItErrorCode::SUCCESS;
      response->message = response->success ?
        "pregrasp execution succeeded; robot stopped 50 mm above target" :
        "pregrasp execution failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("execute exception: ") + error.what();
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
      std::string reason;
      if (!getFreshTarget(target, reason)) {
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
      const auto current_pose =
        move_group.getCurrentPose(get_parameter("eef_link").as_string()).pose;
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
        true);
      if (fraction < 0.999) {
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
        near_plan_created_at_ = now();
        have_near_plan_ = true;
      }

      std::ostringstream message;
      message << "prepared vertical near descent " << descent_distance * 1000.0
              << " mm; stop "
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
      rclcpp::Time created;
      {
        std::lock_guard<std::mutex> lock(data_mutex_);
        plan = prepared_near_plan_;
        planned_target = planned_near_target_;
        created = near_plan_created_at_;
      }
      std::string reason;
      if (!checkPlanBeforeExecution(
          plan, planned_target, created, have_near_plan_, reason)) {
        response->success = false;
        response->message = reason;
        RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
        return;
      }

      RCLCPP_WARN(
        get_logger(),
        "Executing VERTICAL NEAR DESCENT at %.1f%% velocity; no gripper command",
        get_parameter("descent_velocity_scaling").as_double() * 100.0);
      const auto result = arm().execute(plan);
      response->success = result == moveit::core::MoveItErrorCode::SUCCESS;
      response->message = response->success ?
        "near descent succeeded; robot stopped 5 mm above target" :
        "near descent execution failed";
    } catch (const std::exception & error) {
      response->success = false;
      response->message = std::string("execute near exception: ") + error.what();
    }
  }

  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_pub_;
  rclcpp::Service<Trigger>::SharedPtr prepare_service_;
  rclcpp::Service<Trigger>::SharedPtr execute_service_;
  rclcpp::Service<Trigger>::SharedPtr prepare_near_service_;
  rclcpp::Service<Trigger>::SharedPtr execute_near_service_;
  rclcpp::CallbackGroup::SharedPtr operation_group_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;

  std::mutex data_mutex_;
  geometry_msgs::msg::PointStamped latest_target_;
  geometry_msgs::msg::PointStamped planned_target_;
  geometry_msgs::msg::PointStamped planned_near_target_;
  rclcpp::Time target_received_at_;
  rclcpp::Time plan_created_at_;
  rclcpp::Time near_plan_created_at_;
  Plan prepared_plan_;
  Plan prepared_near_plan_;
  bool have_target_{false};
  bool have_plan_{false};
  bool have_near_plan_{false};
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
