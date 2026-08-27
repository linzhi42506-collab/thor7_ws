#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>

#include <control_msgs/action/gripper_command.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <limits>
#include <stdexcept>
#include <memory>
#include <string>
#include <thread>

#include "thor7_hardware/ag145_modbus_client.hpp"
#include "thor7_hardware/modbus_utils.hpp"

using namespace std::chrono_literals;

class Ag145GripperActionServer : public rclcpp::Node
{
public:
  using GripperCommand =
    control_msgs::action::GripperCommand;

  using GoalHandle =
    rclcpp_action::ServerGoalHandle<GripperCommand>;

  Ag145GripperActionServer()
  : Node("ag145_gripper_action_server")
  {
    const auto port =
      declare_parameter<std::string>(
        "serial_port", "/dev/ttyUSB0");

    const int baud =
      declare_parameter<int>(
        "baud_rate", 115200);

    const int slave_id =
      declare_parameter<int>(
        "slave_id", 1);

    const int timeout_ms =
      declare_parameter<int>(
        "timeout_ms", 200);

    const int retries =
      declare_parameter<int>(
        "retries", 3);

    publish_rate_ =
      declare_parameter<double>(
        "publish_rate", 10.0);

    joint_state_topic_ =
      declare_parameter<std::string>(
        "joint_state_topic", "/joint_states");

    force_percent_ =
      declare_parameter<int>(
        "force_percent", 20);

    tolerance_counts_ =
      declare_parameter<int>(
        "position_tolerance_counts", 15);

    motion_timeout_sec_ =
      declare_parameter<double>(
        "motion_timeout_sec", 10.0);

    if (
      force_percent_ < 20 ||
      force_percent_ > 100)
    {
      throw std::runtime_error(
        "force_percent must be 20..100");
    }

    client_ =
      std::make_unique<
        thor7_hardware::Ag145ModbusClient>(
          port,
          static_cast<uint32_t>(baud),
          static_cast<uint8_t>(slave_id),
          timeout_ms,
          retries);

    if (!client_->open()) {
      throw std::runtime_error(
        "Cannot open AG145 serial port: " +
        port);
    }

    uint16_t init = 0;
    uint16_t status = 0;
    uint16_t pos = 0;
    uint16_t force = 0;

    if (
      !client_->readInitFeedback(init) ||
      !client_->readStatus(status) ||
      !client_->readPosition(pos) ||
      !client_->readForce(force))
    {
      throw std::runtime_error(
        "Initial AG145 Modbus read failed");
    }

    if (init != 1) {
      RCLCPP_WARN(
        get_logger(),
        "AG145 is not initialized. "
        "Starting automatic initialization; "
        "THE GRIPPER MAY MOVE.");

      if (!client_->writeInitialize()) {
        throw std::runtime_error(
          "Failed to send AG145 initialization command");
      }

      const auto deadline =
        std::chrono::steady_clock::now() +
        std::chrono::seconds(20);

      bool initialized = false;

      while (
        rclcpp::ok() &&
        std::chrono::steady_clock::now() < deadline)
      {
        std::this_thread::sleep_for(
          std::chrono::milliseconds(100));

        if (!client_->readInitFeedback(init)) {
          continue;
        }

        if (!client_->readStatus(status)) {
          continue;
        }

        if (status == 4) {
          throw std::runtime_error(
            "AG145 entered ERROR status "
            "during initialization");
        }

        if (init == 1) {
          initialized = true;
          break;
        }
      }

      if (!initialized) {
        throw std::runtime_error(
          "AG145 initialization timed out");
      }

      // Refresh state after initialization because
      // calibration may have changed the measured position.
      if (
        !client_->readStatus(status) ||
        !client_->readPosition(pos) ||
        !client_->readForce(force))
      {
        throw std::runtime_error(
          "AG145 post-initialization read failed");
      }

      RCLCPP_INFO(
        get_logger(),
        "AG145 initialization complete: "
        "pos=%u status=%u force=%u%%",
        static_cast<unsigned>(pos),
        static_cast<unsigned>(status),
        static_cast<unsigned>(force));
    }

    if (status == 4) {
      throw std::runtime_error(
        "AG145 reports ERROR status");
    }

    startup_force_ = force;
    startup_force_valid_ =
      (force >= 20 && force <= 100);

    RCLCPP_INFO(
      get_logger(),
      "AG145 connected: pos=%u "
      "joint=%.6f status=%u force=%u%%",
      static_cast<unsigned>(pos),
      thor7_hardware::
        devicePositionToJointRad(pos),
      static_cast<unsigned>(status),
      static_cast<unsigned>(force));

    RCLCPP_WARN(
      get_logger(),
      "AG145 command mode enabled. "
      "Configured force=%d%%",
      force_percent_);

    joint_pub_ =
      create_publisher<
        sensor_msgs::msg::JointState>(
          joint_state_topic_, 10);

    const auto period =
      std::chrono::duration<double>(
        1.0 / publish_rate_);

    state_timer_ =
      create_wall_timer(
        period,
        std::bind(
          &Ag145GripperActionServer::
            publishState,
          this));

    action_server_ =
      rclcpp_action::create_server<
        GripperCommand>(
          this,
          "/ag145_gripper_controller/"
          "gripper_cmd",
          std::bind(
            &Ag145GripperActionServer::
              handleGoal,
            this,
            std::placeholders::_1,
            std::placeholders::_2),
          std::bind(
            &Ag145GripperActionServer::
              handleCancel,
            this,
            std::placeholders::_1),
          std::bind(
            &Ag145GripperActionServer::
              handleAccepted,
            this,
            std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "GripperCommand ready at "
      "/ag145_gripper_controller/gripper_cmd");
  }

  ~Ag145GripperActionServer() override
  {
    if (
      client_ &&
      client_->isOpen() &&
      startup_force_valid_)
    {
      (void)client_->writeForce(
        startup_force_);
    }

    if (client_) {
      client_->close();
    }
  }

private:
  rclcpp_action::GoalResponse handleGoal(
    const rclcpp_action::GoalUUID &,
    std::shared_ptr<
      const GripperCommand::Goal> goal)
  {
    const double q =
      goal->command.position;

    if (!std::isfinite(q)) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting non-finite position");
      return
        rclcpp_action::GoalResponse::REJECT;
    }

    //
    // AG145 MoveIt joint:
    // closed = 0.0 rad
    // open   = -0.65 rad
    //
    if (q < -0.65 || q > 0.0) {
      RCLCPP_ERROR(
        get_logger(),
        "Rejecting position %.6f; "
        "valid range is [-0.65, 0.0]",
        q);

      return
        rclcpp_action::GoalResponse::REJECT;
    }

    bool expected = false;

    if (!busy_.compare_exchange_strong(
          expected, true))
    {
      RCLCPP_WARN(
        get_logger(),
        "Rejecting goal: AG145 already busy");

      return
        rclcpp_action::GoalResponse::REJECT;
    }

    return
      rclcpp_action::
        GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handleCancel(
    const std::shared_ptr<GoalHandle>)
  {
    RCLCPP_WARN(
      get_logger(),
      "AG145 cancel requested");

    return
      rclcpp_action::
        CancelResponse::ACCEPT;
  }

  void handleAccepted(
    const std::shared_ptr<GoalHandle>
      goal_handle)
  {
    std::thread{
      std::bind(
        &Ag145GripperActionServer::execute,
        this,
        goal_handle)
    }.detach();
  }

  void execute(
    const std::shared_ptr<GoalHandle>
      goal_handle)
  {
    struct BusyGuard
    {
      std::atomic<bool> & busy;

      ~BusyGuard()
      {
        busy.store(false);
      }
    } guard{busy_};

    const auto goal =
      goal_handle->get_goal();

    const double target_joint =
      goal->command.position;

    const int target_device =
      thor7_hardware::
        jointRadToDevicePosition(
          target_joint);

    auto result =
      std::make_shared<
        GripperCommand::Result>();

    uint16_t init = 0;
    uint16_t status = 0;
    uint16_t start_pos = 0;

    if (
      !client_->readInitFeedback(init) ||
      !client_->readStatus(status) ||
      !client_->readPosition(start_pos))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Pre-command read failed");

      result->position =
        std::numeric_limits<double>::
          quiet_NaN();

      result->effort = 0.0;
      result->stalled = false;
      result->reached_goal = false;

      goal_handle->abort(result);
      return;
    }

    if (init != 1 || status == 4) {
      RCLCPP_ERROR(
        get_logger(),
        "Command refused: "
        "init=%u status=%u",
        static_cast<unsigned>(init),
        static_cast<unsigned>(status));

      result->position =
        thor7_hardware::
          devicePositionToJointRad(
            start_pos);

      result->effort = 0.0;
      result->stalled = false;
      result->reached_goal = false;

      goal_handle->abort(result);
      return;
    }

    //
    // For now we deliberately use a known device
    // percentage rather than pretending max_effort
    // has calibrated Newton semantics.
    //
    if (!client_->writeForce(
          static_cast<uint16_t>(
            force_percent_)))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to set AG145 force");

      result->position =
        thor7_hardware::
          devicePositionToJointRad(
            start_pos);

      result->effort = 0.0;
      result->stalled = false;
      result->reached_goal = false;

      goal_handle->abort(result);
      return;
    }

    RCLCPP_WARN(
      get_logger(),
      "AG145 MOVE: device %u -> %d, "
      "joint %.6f -> %.6f rad, force=%d%%",
      static_cast<unsigned>(start_pos),
      target_device,
      thor7_hardware::
        devicePositionToJointRad(
          start_pos),
      target_joint,
      force_percent_);

    if (!client_->writeTargetPosition(
          static_cast<uint16_t>(
            target_device)))
    {
      RCLCPP_ERROR(
        get_logger(),
        "Position command failed");

      result->position =
        thor7_hardware::
          devicePositionToJointRad(
            start_pos);

      result->effort = 0.0;
      result->stalled = false;
      result->reached_goal = false;

      goal_handle->abort(result);
      return;
    }

    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::milliseconds(
        static_cast<int>(
          motion_timeout_sec_ * 1000.0));

    while (
      rclcpp::ok() &&
      std::chrono::steady_clock::now() <
      deadline)
    {
      uint16_t pos = 0;
      uint16_t s = 0;

      if (
        !client_->readPosition(pos) ||
        !client_->readStatus(s))
      {
        RCLCPP_ERROR(
          get_logger(),
          "Feedback read failed");

        result->position =
          std::numeric_limits<double>::
            quiet_NaN();

        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;

        goal_handle->abort(result);
        return;
      }

      const double joint =
        thor7_hardware::
          devicePositionToJointRad(pos);

      auto feedback =
        std::make_shared<
          GripperCommand::Feedback>();

      feedback->position = joint;
      feedback->effort = 0.0;
      feedback->stalled = false;
      feedback->reached_goal = false;

      goal_handle->publish_feedback(
        feedback);

      if (goal_handle->is_canceling()) {
        //
        // Best available stop behavior:
        // command current measured position.
        //
        (void)client_->writeTargetPosition(
          pos);

        result->position = joint;
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;

        goal_handle->canceled(result);

        RCLCPP_WARN(
          get_logger(),
          "AG145 goal canceled at "
          "device_pos=%u",
          static_cast<unsigned>(pos));

        return;
      }

      if (s == 4) {
        RCLCPP_ERROR(
          get_logger(),
          "AG145 reports ERROR");

        result->position = joint;
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;

        goal_handle->abort(result);
        return;
      }

      if (s == 3) {
        RCLCPP_ERROR(
          get_logger(),
          "AG145 reports DROPPED");

        result->position = joint;
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = false;

        goal_handle->abort(result);
        return;
      }

      const int error =
        std::abs(
          static_cast<int>(pos) -
          target_device);

      //
      // Normal reached position.
      //
      if (
        error <= tolerance_counts_ &&
        s != 0)
      {
        result->position = joint;
        result->effort = 0.0;
        result->stalled = false;
        result->reached_goal = true;

        goal_handle->succeed(result);

        RCLCPP_INFO(
          get_logger(),
          "AG145 GOAL PASS: "
          "device=%u target=%d "
          "error=%d status=%u",
          static_cast<unsigned>(pos),
          target_device,
          error,
          static_cast<unsigned>(s));

        return;
      }

      //
      // Future real grasp case:
      // status=2 means holding an object.
      //
      if (s == 2) {
        result->position = joint;
        result->effort = 0.0;
        result->stalled = true;
        result->reached_goal = false;

        goal_handle->succeed(result);

        RCLCPP_INFO(
          get_logger(),
          "AG145 HOLDING object at "
          "device_pos=%u",
          static_cast<unsigned>(pos));

        return;
      }

      std::this_thread::sleep_for(50ms);
    }

    uint16_t final_pos = 0;

    (void)client_->readPosition(
      final_pos);

    result->position =
      thor7_hardware::
        devicePositionToJointRad(
          final_pos);

    result->effort = 0.0;
    result->stalled = false;
    result->reached_goal = false;

    RCLCPP_ERROR(
      get_logger(),
      "AG145 goal timeout");

    goal_handle->abort(result);
  }

  void publishState()
  {
    if (!client_ || !client_->isOpen()) {
      return;
    }

    uint16_t init = 0;
    uint16_t pos = 0;

    if (
      !client_->readInitFeedback(init) ||
      !client_->readPosition(pos))
    {
      return;
    }

    if (init != 1 || pos > 1000) {
      return;
    }

    sensor_msgs::msg::JointState js;

    js.header.stamp = now();
    js.name = {
      "ag145_gripper_finger1_joint"
    };

    js.position = {
      thor7_hardware::
        devicePositionToJointRad(pos)
    };

    joint_pub_->publish(js);
  }

  std::unique_ptr<
    thor7_hardware::Ag145ModbusClient>
      client_;

  rclcpp_action::Server<
    GripperCommand>::SharedPtr
      action_server_;

  rclcpp::Publisher<
    sensor_msgs::msg::JointState>::
      SharedPtr joint_pub_;

  rclcpp::TimerBase::SharedPtr
    state_timer_;

  std::atomic<bool> busy_{false};

  double publish_rate_{10.0};
  std::string joint_state_topic_;
  int force_percent_{20};
  int tolerance_counts_{15};
  double motion_timeout_sec_{10.0};

  uint16_t startup_force_{0};
  bool startup_force_valid_{false};
};


int main(
  int argc,
  char ** argv)
{
  rclcpp::init(argc, argv);

  try {
    auto node =
      std::make_shared<
        Ag145GripperActionServer>();

    rclcpp::spin(node);
  }
  catch (const std::exception & e) {
    RCLCPP_FATAL(
      rclcpp::get_logger(
        "ag145_gripper_action_server"),
      "%s",
      e.what());

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
