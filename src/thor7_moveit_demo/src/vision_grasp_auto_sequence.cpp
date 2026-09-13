#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

#include <control_msgs/action/gripper_command.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_srvs/srv/trigger.hpp>
#include "command_lease.hpp"
#include "pick_place_motion.hpp"
#include "new_target_gate.hpp"

using namespace std::chrono_literals;

class VisionGraspAutoSequence : public rclcpp::Node
{
public:
  using Trigger = std_srvs::srv::Trigger;
  using GripperCommand = control_msgs::action::GripperCommand;
  using GoalHandle = rclcpp_action::ClientGoalHandle<GripperCommand>;

  VisionGraspAutoSequence()
  : Node("vision_grasp_auto_sequence", rclcpp::NodeOptions()
      .automatically_declare_parameters_from_overrides(false))
  {
    declare_parameter("enable_auto_grasp", false);
    declare_parameter("enable_pick_place", false);
    declare_parameter("operator_confirmation", "DENIED");
    declare_parameter("service_wait_sec", 10.0);
    declare_parameter("service_timeout_sec", 180.0);
    declare_parameter("gripper_timeout_sec", 15.0);
    declare_parameter("gripper_settle_sec", 0.5);
    declare_parameter("pregrasp_settle_sec", 0.5);
    declare_parameter("target_acquire_timeout_sec", 10.0);
    declare_parameter("target_retry_sec", 0.2);
    declare_parameter("pregrasp_replan_attempts", 2);
    declare_parameter("gripper_open_position", -0.65);
    declare_parameter("gripper_close_position", 0.0);
    declare_parameter("near_clearance", 0.020);
    declare_parameter("use_vision_yaw", true);
    declare_parameter("enable_loop", false);
    declare_parameter("max_cycles", 0);  // 0 = unlimited while the service is active
    declare_parameter("new_target_min_distance", 0.050);
    declare_parameter("new_target_wait_sec", 0.0);  // 0 = wait indefinitely
    declare_parameter("close_during_return", true);
    declare_parameter("target_x_min", -.70);
    declare_parameter("target_x_max", -.25);
    declare_parameter("target_y_min", .30);
    declare_parameter("target_y_max", .75);
    declare_parameter("target_z_min", -.05);
    declare_parameter("target_z_max", .12);
    motion_ = std::make_unique<thor7_grasp::PickPlaceMotion>(*this);

    for (const auto * name : {
        "service_wait_sec", "service_timeout_sec", "gripper_timeout_sec",
        "gripper_settle_sec", "pregrasp_settle_sec", "target_acquire_timeout_sec", "target_retry_sec",
        "near_clearance"}) {
      const double value = get_parameter(name).as_double();
      if (!std::isfinite(value) || value <= 0.0) {
        throw std::invalid_argument(std::string(name) + " must be finite and positive");
      }
    }
    const double open = get_parameter("gripper_open_position").as_double();
    const double close = get_parameter("gripper_close_position").as_double();
    if (!std::isfinite(open) || !std::isfinite(close) ||
        open < -0.65 || open > 0.0 || close < -0.65 || close > 0.0) {
      throw std::invalid_argument("gripper positions must be within [-0.65, 0.0] rad");
    }
    const auto replan_attempts = get_parameter("pregrasp_replan_attempts").as_int();
    if (replan_attempts < 0 || replan_attempts > 5) {
      throw std::invalid_argument("pregrasp_replan_attempts must be within 0..5");
    }
    const double new_target_distance = get_parameter("new_target_min_distance").as_double();
    const double new_target_wait = get_parameter("new_target_wait_sec").as_double();
    if (!std::isfinite(new_target_distance) || new_target_distance <= 0.0 ||
        !std::isfinite(new_target_wait) || new_target_wait < 0.0) {
      throw std::invalid_argument("new target distance must be positive and wait must be nonnegative");
    }
    if (get_parameter("max_cycles").as_int() < 0) {
      throw std::invalid_argument("max_cycles must be nonnegative; zero means unlimited");
    }
    for (const auto & axis : {"x", "y", "z"}) {
      const std::string prefix = "target_" + std::string(axis);
      if (get_parameter(prefix + "_min").as_double() >=
          get_parameter(prefix + "_max").as_double()) {
        throw std::invalid_argument("invalid pick workspace: " + prefix);
      }
    }

    target_point_sub_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "/yolo_target_point", 10,
      [this](geometry_msgs::msg::PointStamped::ConstSharedPtr msg) {
        if (get_parameter("use_vision_yaw").as_bool() || msg->header.frame_id != "base_link") { return; }
        updateVisionTarget(msg->point.x, msg->point.y, msg->point.z);
      });
    target_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>(
      "/yolo_grasp_pose", 10,
      [this](geometry_msgs::msg::PoseStamped::ConstSharedPtr msg) {
        if (!get_parameter("use_vision_yaw").as_bool() || msg->header.frame_id != "base_link") { return; }
        updateVisionTarget(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);
      });

    callback_group_ = create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    prepare_pregrasp_ = create_client<Trigger>(
      "/thor7_vision_grasp/sequence/prepare_pregrasp", rmw_qos_profile_services_default,
      callback_group_);
    execute_pregrasp_ = create_client<Trigger>(
      "/thor7_vision_grasp/sequence/execute_pregrasp", rmw_qos_profile_services_default,
      callback_group_);
    prepare_near_ = create_client<Trigger>(
      "/thor7_vision_grasp/sequence/prepare_near", rmw_qos_profile_services_default,
      callback_group_);
    execute_near_ = create_client<Trigger>(
      "/thor7_vision_grasp/sequence/execute_near", rmw_qos_profile_services_default,
      callback_group_);
    gripper_ = rclcpp_action::create_client<GripperCommand>(
      get_node_base_interface(), get_node_graph_interface(), get_node_logging_interface(),
      get_node_waitables_interface(), "/ag145_gripper_controller/gripper_cmd", callback_group_);

    run_service_ = create_service<Trigger>(
      "/thor7_vision_grasp/run_grasp",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        dispatch(res, false, false);
      },
      rmw_qos_profile_services_default, callback_group_);
    cycle_service_ = create_service<Trigger>("/thor7_vision_grasp/run_pick_place",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        dispatch(res, true, false);
      }, rmw_qos_profile_services_default, callback_group_);
    preview_service_ = create_service<Trigger>("/thor7_vision_grasp/preview_place_home",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        dispatch(res, true, true);
      }, rmw_qos_profile_services_default, callback_group_);
    stop_service_ = create_service<Trigger>("/thor7_vision_grasp/stop_after_cycle",
      [this](const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> res) {
        if (!busy_.load() || !get_parameter("enable_loop").as_bool()) {
          res->success = false; res->message = "no running continuous sequence";
          return;
        }
        stop_requested_.store(true);
        res->success = true;
        res->message = "stop requested; will stop at Home after the current cycle, or immediately if waiting";
      }, rmw_qos_profile_services_default, callback_group_);

    RCLCPP_WARN(
      get_logger(),
      "Automatic grasp coordinator ready. It moves only after a service call. "
      "Final near clearance is %.1f mm. /run_grasp stops holding; /run_pick_place adds one place/Home cycle. "
      "/preview_place_home plans from current pose without motion.",
      get_parameter("near_clearance").as_double() * 1000.0);
  }

private:
  struct BusyGuard
  {
    std::atomic_bool & busy;
    ~BusyGuard() { busy.store(false); }
  };

  struct GripperResult
  {
    bool command_ok{false};
    bool reached_goal{false};
    bool holding{false};
    double position{0.0};
    std::string reason;
  };

  enum class GraspResult
  {
    success,
    retryable_before_arm_motion,
    latch_required
  };

  void updateVisionTarget(double x, double y, double z)
  {
    if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(z) ||
        x < get_parameter("target_x_min").as_double() || x > get_parameter("target_x_max").as_double() ||
        y < get_parameter("target_y_min").as_double() || y > get_parameter("target_y_max").as_double() ||
        z < get_parameter("target_z_min").as_double() || z > get_parameter("target_z_max").as_double()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Ignoring vision target outside pick workspace: xyz=(%.3f, %.3f, %.3f), "
        "allowed x=[%.3f, %.3f] y=[%.3f, %.3f] z=[%.3f, %.3f]",
        x, y, z,
        get_parameter("target_x_min").as_double(), get_parameter("target_x_max").as_double(),
        get_parameter("target_y_min").as_double(), get_parameter("target_y_max").as_double(),
        get_parameter("target_z_min").as_double(), get_parameter("target_z_max").as_double());
      return;
    }
    std::lock_guard<std::mutex> lock(target_mutex_);
    latest_target_ = Eigen::Vector3d(x, y, z);
    latest_target_received_ = std::chrono::steady_clock::now();
    have_latest_target_ = true;
    ++latest_target_sequence_;
  }

  bool latestTarget(Eigen::Vector3d & target) const
  {
    std::lock_guard<std::mutex> lock(target_mutex_);
    if (!have_latest_target_ ||
        std::chrono::steady_clock::now() - latest_target_received_ > std::chrono::seconds(1)) {
      return false;
    }
    target = latest_target_;
    return true;
  }

  bool waitForNewTarget(const Eigen::Vector3d & previous, std::string & reason)
  {
    const double wait_sec = get_parameter("new_target_wait_sec").as_double();
    const auto started = std::chrono::steady_clock::now();
    thor7_grasp::NewTargetGate gate(previous,
      get_parameter("new_target_min_distance").as_double());
    std::uint64_t last_sequence = 0;
    while (rclcpp::ok() && !stop_requested_.load()) {
      Eigen::Vector3d candidate;
      bool fresh = false;
      {
        std::lock_guard<std::mutex> lock(target_mutex_);
        fresh = have_latest_target_ && latest_target_sequence_ != last_sequence &&
          std::chrono::steady_clock::now() - latest_target_received_ < std::chrono::seconds(1);
        if (fresh) { candidate = latest_target_; last_sequence = latest_target_sequence_; }
      }
      if (fresh && gate.observe(candidate)) {
        RCLCPP_INFO(get_logger(),
          "NEW TARGET accepted: displacement=%.1f mm",
          (candidate - previous).norm() * 1000.0);
        return true;
      }
      if (wait_sec > 0.0 && std::chrono::duration<double>(
          std::chrono::steady_clock::now() - started).count() >= wait_sec) {
        reason = "timed out waiting for a new target displaced by at least " +
          std::to_string(get_parameter("new_target_min_distance").as_double() * 1000.0) + " mm";
        return false;
      }
      std::this_thread::sleep_for(100ms);
    }
    reason = stop_requested_.load() ? "operator stopped loop at Home" :
      "ROS shutdown while waiting for a new target";
    return false;
  }

  bool authorized() const
  {
    return get_parameter("enable_auto_grasp").as_bool() &&
      get_parameter("operator_confirmation").as_string() == "ARM_THOR7_AUTO_GRASP";
  }

  bool callTrigger(
    const rclcpp::Client<Trigger>::SharedPtr & client,
    const std::string & stage,
    std::string & detail)
  {
    const auto wait = std::chrono::duration<double>(
      get_parameter("service_wait_sec").as_double());
    if (!client->wait_for_service(wait)) {
      detail = stage + ": service unavailable";
      return false;
    }

    RCLCPP_WARN(get_logger(), "AUTO GRASP stage: %s", stage.c_str());
    auto future = client->async_send_request(std::make_shared<Trigger::Request>());
    const auto timeout = std::chrono::duration<double>(
      get_parameter("service_timeout_sec").as_double());
    if (future.wait_for(timeout) != std::future_status::ready) {
      detail = stage + ": service response timed out; stop and inspect the robot";
      return false;
    }
    const auto response = future.get();
    detail = response->message;
    if (!response->success) {
      detail = stage + " rejected: " + detail;
      return false;
    }
    RCLCPP_INFO(get_logger(), "%s PASS: %s", stage.c_str(), detail.c_str());
    return true;
  }

  bool prepareWithFreshTarget(std::string & detail,
                              const Eigen::Vector3d * previous = nullptr)
  {
    const auto deadline = std::chrono::steady_clock::now() +
      std::chrono::duration<double>(
      get_parameter("target_acquire_timeout_sec").as_double());
    do {
      // Capture the accepted pose before the potentially long MoveIt plan.
      // /yolo_grasp_pose is intentionally published only while its orientation
      // estimate is valid, so requiring another one after planning rejects a
      // valid plan solely because the camera has stopped publishing it.
      Eigen::Vector3d requested_target;
      if (!latestTarget(requested_target)) {
        std::this_thread::sleep_for(std::chrono::duration<double>(
          get_parameter("target_retry_sec").as_double()));
        continue;
      }
      if (callTrigger(prepare_pregrasp_, "prepare_pregrasp", detail)) {
        planned_pick_target_ = requested_target;
        if (previous && (planned_pick_target_ - *previous).norm() <
            get_parameter("new_target_min_distance").as_double()) {
          detail = "visual target returned to previous pick position before pregrasp execution";
          return false;
        }
        return true;
      }
      const bool waiting_for_target =
        detail.find("target is stale") != std::string::npos ||
        detail.find("grasp pose is stale") != std::string::npos ||
        detail.find("no stable /yolo_grasp_pose received") != std::string::npos ||
        detail.find("no /yolo_target_point received") != std::string::npos;
      if (!waiting_for_target) {
        return false;
      }
      std::this_thread::sleep_for(std::chrono::duration<double>(
        get_parameter("target_retry_sec").as_double()));
    } while (std::chrono::steady_clock::now() < deadline && rclcpp::ok());
    detail = "prepare_pregrasp: no fresh vision target within acquisition timeout";
    return false;
  }

  static bool isConfirmedTargetMotion(const std::string & detail)
  {
    return detail.find("target moved") != std::string::npos ||
      detail.find("target yaw moved") != std::string::npos ||
      detail.find("locked target moved") != std::string::npos;
  }

  GripperResult commandGripper(double position, bool closing)
  {
    GripperResult out;
    const auto wait = std::chrono::duration<double>(
      get_parameter("service_wait_sec").as_double());
    if (!gripper_->wait_for_action_server(wait)) {
      out.reason = "AG145 action server unavailable";
      return out;
    }

    GripperCommand::Goal goal;
    goal.command.position = position;
    goal.command.max_effort = 0.0;
    RCLCPP_WARN(
      get_logger(), "AUTO GRASP stage: %s AG145 to %.3f rad",
      closing ? "CLOSE" : "OPEN", position);

    auto goal_future = gripper_->async_send_goal(goal);
    if (goal_future.wait_for(wait) != std::future_status::ready) {
      out.reason = "AG145 goal acknowledgement timed out";
      return out;
    }
    const auto handle = goal_future.get();
    if (!handle) {
      out.reason = "AG145 goal was rejected";
      return out;
    }

    auto result_future = gripper_->async_get_result(handle);
    const auto timeout = std::chrono::duration<double>(
      get_parameter("gripper_timeout_sec").as_double());
    if (result_future.wait_for(timeout) != std::future_status::ready) {
      (void)gripper_->async_cancel_goal(handle);
      out.reason = "AG145 result timed out; cancel requested";
      return out;
    }
    const auto wrapped = result_future.get();
    if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED || !wrapped.result) {
      out.reason = "AG145 action did not succeed";
      return out;
    }

    out.position = wrapped.result->position;
    out.reached_goal = wrapped.result->reached_goal;
    out.holding = wrapped.result->stalled;
    out.command_ok = closing ? (out.holding || out.reached_goal) : out.reached_goal;
    if (!out.command_ok) {
      out.reason = closing ? "AG145 neither reached closed position nor reported holding" :
        "AG145 did not reach the fully open position";
    }
    return out;
  }

  void dispatch(std::shared_ptr<Trigger::Response> response, bool cycle, bool preview)
  {
    bool expected = false;
    if (!busy_.compare_exchange_strong(expected, true)) {
      response->success = false;
      response->message = "automatic grasp is already running";
      return;
    }
    BusyGuard guard{busy_};
    stop_requested_.store(false);
    thor7_grasp::CommandLease lease;
    if (!lease.acquired()) {
      response->success = false;
      response->message = "another guarded operation/coordinator owns the robot";
      return;
    }
    if (faulted_) {
      response->success = false;
      response->message = "sequence failure latched; inspect robot and recover before restarting coordinator";
      return;
    }
    if ((completed_cycle_ || holding_after_grasp_) && !preview) {
      response->success = false;
      response->message = "previous grasp/cycle completed; inspect held/placed wood before another session; automatic reopen disabled";
      return;
    }
    try {
      if (preview) {
        motion_->preview(); response->success = true;
        response->message = "place/Home tail preview from CURRENT tool pose published; no motion or gripper command; not a vision-grasp preview";
        return;
      }
      if (!authorized() || (cycle && !get_parameter("enable_pick_place").as_bool())) {
        response->success = false; response->message = "requested sequence disabled; use --pick-place for the full cycle";
        return;
      }
      const bool loop = cycle && get_parameter("enable_loop").as_bool();
      if (cycle && !get_parameter("enable_pick_place").as_bool()) {
        response->success = false; response->message = "requested sequence disabled; use --pick-place or --auto-loop";
        return;
      }
      if (loop && get_parameter("retain_placed_objects").as_bool()) {
        // A repeated cycle revisits a fixed drop pose. Keeping each released
        // box in MoveIt's scene would make the next cycle self-blocking.
        const auto result = set_parameter(rclcpp::Parameter("retain_placed_objects", false));
        if (!result.successful) {
          throw std::runtime_error(
            "failed to disable retain_placed_objects for continuous mode: " + result.reason);
        }
        RCLCPP_WARN(
          get_logger(),
          "Continuous mode disabled retain_placed_objects; clear the physical drop area before every cycle");
      }
      Eigen::Vector3d previous_target;
      bool have_previous_target = false;
      int completed = 0;
      while (rclcpp::ok()) {
        if (loop && have_previous_target) {
          std::string wait_reason;
          if (!waitForNewTarget(previous_target, wait_reason)) {
            response->success = stop_requested_.load();
            response->message = "loop stopped at Home: " + wait_reason;
            return;
          }
        }
        motion_->requireEmptyScene();
        const auto grasp_result = runGrasp(
          response, loop && have_previous_target ? &previous_target : nullptr);
        if (grasp_result != GraspResult::success) {
          if (grasp_result == GraspResult::latch_required) {
            faulted_ = true;
          } else {
            response->message += "; safe to retry run_pick_place after correcting the target/planning condition";
          }
          return;
        }
        holding_after_grasp_ = true;
        previous_target = planned_pick_target_;
        have_previous_target = true;
        motion_->carryAndPlace(
          [this]() {
            const auto result = commandGripper(get_parameter("gripper_open_position").as_double(), false);
            if (!result.command_ok) { throw std::runtime_error("release failed: " + result.reason); }
            std::this_thread::sleep_for(std::chrono::duration<double>(get_parameter("gripper_settle_sec").as_double()));
          },
          [this]() {
            if (!get_parameter("close_during_return").as_bool()) { return; }
            const auto result = commandGripper(get_parameter("gripper_close_position").as_double(), true);
            if (!result.command_ok) { throw std::runtime_error("empty-gripper close before return failed: " + result.reason); }
          });
        ++completed;
        holding_after_grasp_ = false;
        completed_cycle_ = true;
        const int max_cycles = static_cast<int>(get_parameter("max_cycles").as_int());
        if (!loop || stop_requested_.load() || (max_cycles > 0 && completed >= max_cycles)) {
          response->success = true;
          response->message = "pick/place completed: cycles=" + std::to_string(completed) +
            "; AG145 closed during return/Home; stopped";
          return;
        }
        RCLCPP_WARN(get_logger(), "Cycle %d complete at Home; waiting for a new target", completed);
      }
    } catch (const std::exception & error) {
      // No reopen, no retry and no recovery motion after an uncertain result.
      if (!preview) { faulted_ = true; }
      response->success = false;
      response->message = std::string(preview ? "preview failed: " : "pick/place stopped: ") + error.what();
    }
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
  }

  GraspResult runGrasp(std::shared_ptr<Trigger::Response> response,
                       const Eigen::Vector3d * previous = nullptr)
  {

    if (!authorized()) {
      response->success = false;
      response->message =
        "automatic grasp disabled; start with --auto-grasp and explicit confirmation";
      return GraspResult::retryable_before_arm_motion;
    }

    const auto opened = commandGripper(
      get_parameter("gripper_open_position").as_double(), false);
    if (!opened.command_ok) {
      response->success = false;
      response->message = "automatic grasp stopped before arm motion: " + opened.reason;
      return GraspResult::latch_required;
    }
    std::this_thread::sleep_for(std::chrono::duration<double>(
      get_parameter("gripper_settle_sec").as_double()));

    std::string detail;
    bool near_prepared = false;
    const int max_replans = static_cast<int>(
      get_parameter("pregrasp_replan_attempts").as_int());
    for (int attempt = 0; attempt <= max_replans; ++attempt) {
      if (!prepareWithFreshTarget(detail, previous)) {
        response->success = false;
        response->message = "automatic grasp stopped: " + detail +
          "; pregrasp was not executed";
        return GraspResult::retryable_before_arm_motion;
      }
      if (!callTrigger(execute_pregrasp_, "execute_pregrasp", detail)) {
        if (isConfirmedTargetMotion(detail) && attempt < max_replans) {
          RCLCPP_WARN(
            get_logger(),
            "Target motion confirmed during pregrasp; reacquiring vision and replanning (%d/%d)",
            attempt + 1, max_replans);
          continue;
        }
        response->success = false;
        response->message = "automatic grasp stopped: " + detail +
          "; gripper remains open and no near descent was sent";
        return isConfirmedTargetMotion(detail) ?
          GraspResult::retryable_before_arm_motion : GraspResult::latch_required;
      }

      // The pregrasp action already verifies the endpoint. This short pause
      // ensures the arm is stationary at that feedback-confirmed pose before
      // authorizing the vertical descent if yaw converged near the endpoint.
      std::this_thread::sleep_for(std::chrono::duration<double>(
        get_parameter("pregrasp_settle_sec").as_double()));

      if (callTrigger(prepare_near_, "prepare_near", detail)) {
        near_prepared = true;
        break;
      }
      if (isConfirmedTargetMotion(detail) && attempt < max_replans) {
        RCLCPP_WARN(
          get_logger(),
          "Target motion confirmed before descent; reacquiring vision and replanning (%d/%d)",
          attempt + 1, max_replans);
        continue;
      }
      response->success = false;
      response->message = "automatic grasp stopped: " + detail +
        "; gripper remains open and no descent was sent";
      return GraspResult::latch_required;
    }

    if (!near_prepared) {
      response->success = false;
      response->message =
        "automatic grasp stopped after exhausting target-motion replans; no descent was sent";
      return GraspResult::latch_required;
    }
    if (!callTrigger(execute_near_, "execute_near", detail)) {
      response->success = false;
      response->message = "automatic grasp stopped: " + detail +
        "; gripper remains open and no close command was sent";
      return GraspResult::latch_required;
    }

    std::this_thread::sleep_for(std::chrono::duration<double>(
      get_parameter("gripper_settle_sec").as_double()));
    const auto closed = commandGripper(
      get_parameter("gripper_close_position").as_double(), true);
    if (!closed.command_ok) {
      response->success = false;
      response->message = "arm reached grasp height but AG145 close failed: " + closed.reason;
      return GraspResult::latch_required;
    }

    std::ostringstream message;
    if (closed.holding) {
      response->success = true;
      message << "automatic grasp completed; AG145 reports HOLDING at "
              << closed.position << " rad; no lift commanded";
    } else {
      response->success = false;
      message << "automatic sequence completed but AG145 reached fully closed position at "
              << closed.position << " rad; object hold was not confirmed; no lift commanded";
    }
    response->message = message.str();
    RCLCPP_WARN(get_logger(), "%s", response->message.c_str());
    return response->success ? GraspResult::success : GraspResult::latch_required;
  }

  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::Client<Trigger>::SharedPtr prepare_pregrasp_;
  rclcpp::Client<Trigger>::SharedPtr execute_pregrasp_;
  rclcpp::Client<Trigger>::SharedPtr prepare_near_;
  rclcpp::Client<Trigger>::SharedPtr execute_near_;
  rclcpp_action::Client<GripperCommand>::SharedPtr gripper_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr target_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr target_pose_sub_;
  rclcpp::Service<Trigger>::SharedPtr run_service_;
  rclcpp::Service<Trigger>::SharedPtr cycle_service_, preview_service_;
  rclcpp::Service<Trigger>::SharedPtr stop_service_;
  std::unique_ptr<thor7_grasp::PickPlaceMotion> motion_;
  bool faulted_{false}, completed_cycle_{false}, holding_after_grasp_{false};
  mutable std::mutex target_mutex_;
  Eigen::Vector3d latest_target_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d planned_pick_target_{Eigen::Vector3d::Zero()};
  std::chrono::steady_clock::time_point latest_target_received_{};
  bool have_latest_target_{false};
  std::uint64_t latest_target_sequence_{0};
  std::atomic_bool stop_requested_{false};
  std::atomic_bool busy_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<VisionGraspAutoSequence>();
  rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 3);
  executor.add_node(node);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
