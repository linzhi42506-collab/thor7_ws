#include <atomic>
#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <vector>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

class VisionGraspPlanOnly : public rclcpp::Node {
public:
  VisionGraspPlanOnly() : Node("vision_grasp_plan_only") {
    declare_parameter("point_topic", "/yolo_target_point");
    declare_parameter("base_frame", "base_link");
    declare_parameter("eef_link", "tool0");
    declare_parameter("pregrasp_clearance", 0.050);
    declare_parameter("near_clearance", 0.005);
    declare_parameter("grasp_descent", 0.013);
    declare_parameter("plan_descent", false);
    declare_parameter("auto_replan", false);
    declare_parameter("replan_distance", 0.005);
    declare_parameter("ik_strict_joint_delta", 1.50);
    declare_parameter("ik_relaxed_joint_delta", 2.50);
    declare_parameter("cartesian_step", 0.002);
    declare_parameter("max_cartesian_joint_step", 0.20);
    sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      get_parameter("point_topic").as_string(), 10,
      std::bind(&VisionGraspPlanOnly::pointCallback, this, std::placeholders::_1));
    display_ = create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
    replan_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_vision_grasp/replan",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response) {
        replan_requested_.store(true);
        response->success = true;
        response->message = "replan armed; the latest valid target point will be used";
      });
    planning_group_ = create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    timer_ = create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&VisionGraspPlanOnly::plan, this),
      planning_group_);
    RCLCPP_INFO(
      get_logger(),
      "Ready. Automatic replanning is %s; request another plan with: "
      "ros2 service call /thor7_vision_grasp/replan std_srvs/srv/Trigger '{}'",
      get_parameter("auto_replan").as_bool() ? "enabled" : "disabled");
  }
private:
  void pointCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
    if (msg->header.frame_id != get_parameter("base_frame").as_string()) return;
    std::lock_guard<std::mutex> lock(point_mutex_);
    point_ = *msg;
    have_point_ = true;
  }
  void plan() {
    geometry_msgs::msg::PointStamped point;
    {
      std::lock_guard<std::mutex> lock(point_mutex_);
      if (!have_point_) return;
      point = point_;
    }
    const auto & p = point.point;
    const bool manual_replan = replan_requested_.exchange(false);
    if (have_attempted_ && !manual_replan) {
      if (!get_parameter("auto_replan").as_bool()) return;
      if (std::hypot(std::hypot(p.x-last_x_, p.y-last_y_), p.z-last_z_) <
          get_parameter("replan_distance").as_double()) return;
    }
    have_attempted_ = true;
    last_x_=p.x; last_y_=p.y; last_z_=p.z;

    // Clear any trajectory cached by RViz before attempting a new plan. A
    // failed collision-aware plan must not leave an older unsafe-looking
    // descent on screen.
    moveit_msgs::msg::DisplayTrajectory clear_display;
    clear_display.model_id = "thor7v3";
    display_->publish(clear_display);

    try {
      moveit::planning_interface::MoveGroupInterface arm(shared_from_this(), "thor7_arm");
      arm.setPoseReferenceFrame(get_parameter("base_frame").as_string());
      arm.setEndEffectorLink(get_parameter("eef_link").as_string());
      arm.setPlanningTime(5.0); arm.setNumPlanningAttempts(5);
      arm.setMaxVelocityScalingFactor(0.15); arm.setMaxAccelerationScalingFactor(0.15);
      arm.setStartStateToCurrentState();
      const auto orientation = arm.getCurrentPose().pose.orientation;
      const double pre = get_parameter("pregrasp_clearance").as_double();
      const double near = get_parameter("near_clearance").as_double();
      const double down = get_parameter("grasp_descent").as_double();
      const auto eef_link = get_parameter("eef_link").as_string();
      geometry_msgs::msg::Pose pregrasp_pose;
      pregrasp_pose.orientation = orientation;
      pregrasp_pose.position.x = p.x;
      pregrasp_pose.position.y = p.y;
      pregrasp_pose.position.z = p.z + pre;

      // Solve the pregrasp pose from the measured current state and plan to
      // that concrete joint solution. A pose goal lets OMPL choose any IK
      // branch, which can cause a needless wrist or elbow flip.
      auto current_state = arm.getCurrentState(1.0);
      if (!current_state) {
        RCLCPP_ERROR(get_logger(), "Could not obtain the current robot state for IK seeding");
        return;
      }
      const auto * joint_group = arm.getRobotModel()->getJointModelGroup("thor7_arm");
      if (!joint_group) {
        RCLCPP_ERROR(get_logger(), "No thor7_arm joint group found for IK");
        return;
      }

      std::vector<double> seed_joints;
      current_state->copyJointGroupPositions(joint_group, seed_joints);
      Eigen::Isometry3d pregrasp_eigen = Eigen::Isometry3d::Identity();
      pregrasp_eigen.translation() = Eigen::Vector3d(
        pregrasp_pose.position.x, pregrasp_pose.position.y, pregrasp_pose.position.z);
      const auto q = Eigen::Quaterniond(
        pregrasp_pose.orientation.w, pregrasp_pose.orientation.x,
        pregrasp_pose.orientation.y, pregrasp_pose.orientation.z).normalized();
      pregrasp_eigen.linear() = q.toRotationMatrix();

      // Try progressively wider consistency windows and keep the closest IK
      // solution. This prevents KDL from selecting a distant wrist branch
      // when a nearby solution exists, while retaining a fallback for poses
      // that genuinely require a larger joint change.
      const double strict_delta = get_parameter("ik_strict_joint_delta").as_double();
      const double relaxed_delta = get_parameter("ik_relaxed_joint_delta").as_double();
      const std::vector<double> ik_windows{strict_delta, relaxed_delta};
      std::vector<double> pregrasp_goal;
      double best_cost = std::numeric_limits<double>::infinity();
      double selected_window = 0.0;
      for (const double window : ik_windows) {
        for (int attempt = 0; attempt < 8; ++attempt) {
          std::vector<double> consistency_limits(seed_joints.size(), window);
          auto candidate = std::make_shared<moveit::core::RobotState>(*current_state);
          if (!candidate->setFromIK(
                joint_group, pregrasp_eigen, eef_link, consistency_limits, 0.05)) {
            continue;
          }
          std::vector<double> candidate_joints;
          candidate->copyJointGroupPositions(joint_group, candidate_joints);
          double cost = 0.0;
          for (std::size_t i = 0; i < seed_joints.size(); ++i) {
            const double delta = candidate_joints[i] - seed_joints[i];
            cost += delta * delta;
          }
          if (cost < best_cost) {
            best_cost = cost;
            pregrasp_goal = candidate_joints;
            selected_window = window;
          }
        }
        // A solution in the strict window is preferred; the relaxed window
        // is only tried if repeated nearby searches fail. Distant pi-radian
        // fallback branches are deliberately forbidden.
        if (!pregrasp_goal.empty()) break;
      }
      if (pregrasp_goal.empty()) {
        RCLCPP_ERROR(get_logger(), "No IK solution for the pregrasp pose near the current state");
        return;
      }
      const auto current_joints = arm.getCurrentJointValues();
      if (current_joints.size() == pregrasp_goal.size()) {
        RCLCPP_INFO(get_logger(), "Pregrasp IK joint deltas: j1=%.3f j2=%.3f j3=%.3f j4=%.3f j5=%.3f j6=%.3f",
          pregrasp_goal[0] - current_joints[0], pregrasp_goal[1] - current_joints[1],
          pregrasp_goal[2] - current_joints[2], pregrasp_goal[3] - current_joints[3],
          pregrasp_goal[4] - current_joints[4], pregrasp_goal[5] - current_joints[5]);
      }
      if (selected_window > strict_delta) {
        RCLCPP_WARN(
          get_logger(),
          "Pregrasp IK required relaxed joint window %.2f rad (strict %.2f rad)",
          selected_window, strict_delta);
      }
      arm.setStartStateToCurrentState();
      arm.setJointValueTarget(pregrasp_goal);
      arm.setPlanningPipelineId("ompl");

      moveit::planning_interface::MoveGroupInterface::Plan pregrasp_plan;
      const bool pregrasp_ok = static_cast<bool>(arm.plan(pregrasp_plan));
      if (!pregrasp_ok) {
        RCLCPP_ERROR(get_logger(), "Planning failed: pregrasp joint target");
        return;
      }

      const auto & pregrasp_joints = pregrasp_plan.trajectory_.joint_trajectory;
      if (pregrasp_joints.points.empty() ||
          pregrasp_joints.points.back().positions.size() != pregrasp_joints.joint_names.size()) {
        RCLCPP_ERROR(get_logger(), "Pregrasp planning returned an invalid joint trajectory");
        return;
      }

      if (!get_parameter("plan_descent").as_bool()) {
        moveit_msgs::msg::DisplayTrajectory display;
        display.model_id = "thor7v3";
        display.trajectory_start = pregrasp_plan.start_state_;
        display.trajectory.push_back(pregrasp_plan.trajectory_);
        display_->publish(display);
        RCLCPP_INFO(
          get_logger(),
          "Pregrasp-only plan succeeded: target_z=%.4f m stop_z=%.4f m "
          "clearance=%.1f mm points=%zu; no descent and no gripper command",
          p.z, pregrasp_pose.position.z, pre * 1000.0,
          pregrasp_joints.points.size());
        have_planned_=true;
        return;
      }

      auto cartesian_start = arm.getCurrentState(1.0);
      if (!cartesian_start) {
        RCLCPP_ERROR(get_logger(), "Could not obtain a MoveIt robot state for Cartesian planning");
        return;
      }
      cartesian_start->setVariablePositions(
        pregrasp_joints.joint_names, pregrasp_joints.points.back().positions);
      cartesian_start->update();
      arm.setStartState(*cartesian_start);

      geometry_msgs::msg::Pose near_pose = pregrasp_pose;
      near_pose.position.z = p.z + near;
      geometry_msgs::msg::Pose final_pose = pregrasp_pose;
      final_pose.position.z = p.z - down;
      std::vector<geometry_msgs::msg::Pose> waypoints{near_pose, final_pose};
      moveit_msgs::msg::RobotTrajectory cartesian_trajectory;
      const double fraction = arm.computeCartesianPath(
        waypoints,
        get_parameter("cartesian_step").as_double(),
        0.0,
        cartesian_trajectory,
        true);
      if (fraction < 0.999) {
        RCLCPP_ERROR(get_logger(), "Cartesian descent incomplete: fraction=%.3f", fraction);
        return;
      }

      const auto & cartesian_joints = cartesian_trajectory.joint_trajectory;
      const double max_joint_step = get_parameter("max_cartesian_joint_step").as_double();
      for (std::size_t i = 1; i < cartesian_joints.points.size(); ++i) {
        const auto & previous = cartesian_joints.points[i - 1].positions;
        const auto & current = cartesian_joints.points[i].positions;
        if (previous.size() != current.size()) {
          RCLCPP_ERROR(get_logger(), "Cartesian trajectory contains inconsistent joint positions");
          return;
        }
        for (std::size_t joint = 0; joint < current.size(); ++joint) {
          if (std::fabs(current[joint] - previous[joint]) > max_joint_step) {
            RCLCPP_ERROR(
              get_logger(),
              "Cartesian trajectory rejected: joint %zu jumps %.3f rad",
              joint + 1, std::fabs(current[joint] - previous[joint]));
            return;
          }
        }
      }

      // computeCartesianPath provides the geometric waypoints but not a
      // smooth time profile. Parameterize this segment before displaying it,
      // otherwise RViz may render the last few points as a sudden jump.
      robot_trajectory::RobotTrajectory timed_cartesian(
        arm.getRobotModel(), "thor7_arm");
      timed_cartesian.setRobotTrajectoryMsg(
        *cartesian_start, cartesian_trajectory.joint_trajectory);
      trajectory_processing::IterativeParabolicTimeParameterization time_parameterizer;
      if (!time_parameterizer.computeTimeStamps(timed_cartesian, 0.15, 0.15)) {
        RCLCPP_ERROR(get_logger(), "Could not time-parameterize the Cartesian descent");
        return;
      }
      timed_cartesian.getRobotTrajectoryMsg(cartesian_trajectory);

      moveit_msgs::msg::DisplayTrajectory display;
      display.model_id = "thor7v3";
      display.trajectory_start = pregrasp_plan.start_state_;
      display.trajectory.push_back(pregrasp_plan.trajectory_);
      display.trajectory.push_back(cartesian_trajectory);
      display_->publish(display);
      RCLCPP_INFO(
        get_logger(),
        "Combined plan succeeded: pregrasp points=%zu, Cartesian points=%zu, fraction=%.3f",
        pregrasp_joints.points.size(), cartesian_joints.points.size(), fraction);
      have_planned_=true;
    } catch (const std::exception & e) { RCLCPP_ERROR(get_logger(), "MoveIt exception: %s", e.what()); }
  }
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr replan_service_;
  rclcpp::CallbackGroup::SharedPtr planning_group_;
  rclcpp::TimerBase::SharedPtr timer_;
  geometry_msgs::msg::PointStamped point_;
  std::mutex point_mutex_;
  bool have_point_{false}, have_attempted_{false}, have_planned_{false};
  std::atomic_bool replan_requested_{false};
  double last_x_{},last_y_{},last_z_{};
};
int main(int argc,char ** argv){
  rclcpp::init(argc,argv);
  auto n=std::make_shared<VisionGraspPlanOnly>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
  executor.add_node(n);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
