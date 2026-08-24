#include "thor7_hardware/thor7_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include "DianaAPI.h"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace thor7_hardware
{

bool Thor7SystemHardware::commandModeEnabled() const
{
  return control_mode_ != Thor7ControlMode::READ_ONLY;
}

bool Thor7SystemHardware::realServoJMode() const
{
  return control_mode_ == Thor7ControlMode::REAL_SERVOJ;
}

const char * Thor7SystemHardware::controlModeName() const
{
  switch (control_mode_) {
    case Thor7ControlMode::READ_ONLY:
      return "READ-ONLY";

    case Thor7ControlMode::REAL_DRY_RUN:
      return "REAL DRY-RUN";

    case Thor7ControlMode::REAL_SERVOJ:
      return "REAL SERVOJ";
  }

  return "UNKNOWN";
}

void Thor7SystemHardware::stopServoStream(const char * reason)
{
  if (!servo_stream_active_) {
    return;
  }

  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (client_ && client_->isConnected()) {
    const int ret = ::stop(robot_ip_.c_str());

    if (ret < 0) {
      RCLCPP_ERROR(
        logger,
        "REAL SERVOJ stop() failed: return=%d reason=%s",
        ret,
        reason);
    } else {
      RCLCPP_WARN(
        logger,
        "REAL SERVOJ stream stopped: reason=%s",
        reason);
    }
  } else {
    RCLCPP_ERROR(
      logger,
      "REAL SERVOJ stream lost connection before stop(): reason=%s",
      reason);
  }

  servo_stream_active_ = false;
  servo_stable_time_sec_ = 0.0;
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(
      logger,
      "Thor7 hardware expects exactly 6 arm joints, but URDF provides %zu",
      info_.joints.size());

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---------------------------------------------------------------
  // Basic hardware parameters
  // ---------------------------------------------------------------
  auto it = info_.hardware_parameters.find("robot_ip");

  if (it != info_.hardware_parameters.end()) {
    robot_ip_ = it->second;
  }

  it = info_.hardware_parameters.find("connection_retry_count");

  if (it != info_.hardware_parameters.end()) {
    try {
      retry_count_ = std::stoi(it->second);
    } catch (...) {
      RCLCPP_ERROR(
        logger,
        "Invalid connection_retry_count: %s",
        it->second.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  it = info_.hardware_parameters.find("connection_retry_delay_ms");

  if (it != info_.hardware_parameters.end()) {
    try {
      retry_delay_ms_ = std::stoi(it->second);
    } catch (...) {
      RCLCPP_ERROR(
        logger,
        "Invalid connection_retry_delay_ms: %s",
        it->second.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // ---------------------------------------------------------------
  // Mutually-exclusive control mode
  //
  // Preferred:
  //   control_mode = readonly | dry_run | real_servoj
  //
  // Backward compatibility:
  //   existing real_dryrun Xacro still uses dry_run_command_mode=true.
  // ---------------------------------------------------------------
  const auto mode_it =
    info_.hardware_parameters.find("control_mode");

  const auto legacy_dry_run_it =
    info_.hardware_parameters.find("dry_run_command_mode");

  if (mode_it != info_.hardware_parameters.end() &&
      legacy_dry_run_it != info_.hardware_parameters.end())
  {
    RCLCPP_ERROR(
      logger,
      "Do not configure both control_mode and dry_run_command_mode.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (mode_it != info_.hardware_parameters.end()) {
    const std::string mode = mode_it->second;

    if (mode == "readonly") {
      control_mode_ = Thor7ControlMode::READ_ONLY;
    } else if (mode == "dry_run") {
      control_mode_ = Thor7ControlMode::REAL_DRY_RUN;
    } else if (mode == "real_servoj") {
      control_mode_ = Thor7ControlMode::REAL_SERVOJ;
    } else {
      RCLCPP_ERROR(
        logger,
        "Invalid control_mode='%s'. "
        "Allowed: readonly, dry_run, real_servoj",
        mode.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }
  } else if (legacy_dry_run_it != info_.hardware_parameters.end()) {
    const std::string value = legacy_dry_run_it->second;

    const bool is_true =
      value == "true" ||
      value == "True" ||
      value == "TRUE" ||
      value == "1" ||
      value == "yes";

    const bool is_false =
      value == "false" ||
      value == "False" ||
      value == "FALSE" ||
      value == "0" ||
      value == "no";

    if (!is_true && !is_false) {
      RCLCPP_ERROR(
        logger,
        "Invalid dry_run_command_mode='%s'",
        value.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    control_mode_ =
      is_true ?
      Thor7ControlMode::REAL_DRY_RUN :
      Thor7ControlMode::READ_ONLY;
  }

  auto parse_double_parameter =
    [&](const std::string & name, double & output) -> bool
    {
      const auto param_it = info_.hardware_parameters.find(name);

      if (param_it == info_.hardware_parameters.end()) {
        return true;
      }

      try {
        output = std::stod(param_it->second);

        if (!std::isfinite(output)) {
          throw std::runtime_error("non-finite value");
        }

        return true;
      } catch (...) {
        RCLCPP_ERROR(
          logger,
          "Invalid floating-point hardware parameter %s=%s",
          name.c_str(),
          param_it->second.c_str());

        return false;
      }
    };

  if (!parse_double_parameter(
        "startup_pose_tolerance_rad",
        startup_pose_tolerance_rad_) ||
      !parse_double_parameter(
        "max_command_delta_rad",
        max_command_delta_rad_) ||
      !parse_double_parameter(
        "max_safe_deviation_rad",
        max_safe_deviation_rad_) ||
      !parse_double_parameter(
        "servoj_time_sec",
        servoj_time_sec_) ||
      !parse_double_parameter(
        "servoj_lookahead_sec",
        servoj_lookahead_sec_) ||
      !parse_double_parameter(
        "servoj_gain",
        servoj_gain_) ||
      !parse_double_parameter(
        "servoj_command_epsilon_rad",
        servoj_command_epsilon_rad_) ||
      !parse_double_parameter(
        "servoj_stop_after_stable_sec",
        servoj_stop_after_stable_sec_))
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string parameter =
      "safe_j" + std::to_string(i + 1) + "_rad";

    if (!parse_double_parameter(parameter, safe_positions_[i])) {
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  if (startup_pose_tolerance_rad_ < 0.0 ||
      max_command_delta_rad_ <= 0.0 ||
      max_safe_deviation_rad_ <= 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "Invalid safety tolerance configuration.");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (servoj_time_sec_ <= 0.0 ||
      servoj_lookahead_sec_ < 0.03 ||
      servoj_lookahead_sec_ > 0.20 ||
      servoj_gain_ < 100.0 ||
      servoj_gain_ > 2000.0 ||
      servoj_command_epsilon_rad_ <= 0.0 ||
      servoj_stop_after_stable_sec_ < 0.0)
  {
    RCLCPP_ERROR(
      logger,
      "Invalid servoJ configuration: "
      "time=%.6f lookahead=%.6f gain=%.3f epsilon=%.9f stable=%.3f",
      servoj_time_sec_,
      servoj_lookahead_sec_,
      servoj_gain_,
      servoj_command_epsilon_rad_,
      servoj_stop_after_stable_sec_);

    return hardware_interface::CallbackReturn::ERROR;
  }

  // ---------------------------------------------------------------
  // Interface validation
  // ---------------------------------------------------------------
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string expected_name =
      "joint" + std::to_string(i + 1);

    const auto & joint = info_.joints[i];

    if (joint.name != expected_name) {
      RCLCPP_ERROR(
        logger,
        "Joint order/name mismatch: index %zu expected '%s', got '%s'",
        i,
        expected_name.c_str(),
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name !=
          hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger,
        "%s must contain exactly one state interface: position",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    if (commandModeEnabled()) {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(
          logger,
          "%s mode requires exactly one position command interface on %s",
          controlModeName(),
          joint.name.c_str());

        return hardware_interface::CallbackReturn::ERROR;
      }
    } else {
      if (!joint.command_interfaces.empty()) {
        RCLCPP_ERROR(
          logger,
          "READ-ONLY mode refuses command interface on %s",
          joint.name.c_str());

        return hardware_interface::CallbackReturn::ERROR;
      }
    }
  }

  const double nan =
    std::numeric_limits<double>::quiet_NaN();

  hw_positions_.fill(nan);
  hw_commands_.fill(nan);
  last_logged_commands_.fill(nan);
  last_servoj_commands_.fill(nan);

  client_ = std::make_unique<DianaReadonlyClient>(
    robot_ip_,
    retry_count_,
    retry_delay_ms_);

  RCLCPP_WARN(
    logger,
    "Initialized Thor7 hardware: mode=%s ip=%s",
    controlModeName(),
    robot_ip_.c_str());

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
Thor7SystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);
  }

  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
Thor7SystemHardware::export_command_interfaces()
{
  if (!commandModeEnabled()) {
    return {};
  }

  std::vector<hardware_interface::CommandInterface> interfaces;
  interfaces.reserve(kJointCount);

  for (std::size_t i = 0; i < kJointCount; ++i) {
    interfaces.emplace_back(
      info_.joints[i].name,
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return interfaces;
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (!client_) {
    RCLCPP_ERROR(
      logger,
      "DianaReadonlyClient was not initialized");

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!client_->connect()) {
    RCLCPP_ERROR(
      logger,
      "Diana SDK connection failed: ip=%s error=%d (%s)",
      robot_ip_.c_str(),
      client_->lastSdkError(),
      client_->describeError(client_->lastSdkError()).c_str());

    return hardware_interface::CallbackReturn::ERROR;
  }

  std::array<double, kJointCount> first_read{};

  if (!client_->readJointPositions(first_read)) {
    RCLCPP_ERROR(
      logger,
      "Initial getJointPos() failed: error=%d",
      client_->lastSdkError());

    client_->disconnect();

    return hardware_interface::CallbackReturn::ERROR;
  }

  hw_positions_ = first_read;

  if (!commandModeEnabled()) {
    RCLCPP_INFO(
      logger,
      "Thor7 READ-ONLY hardware activated: ip=%s "
      "J=[%.6f %.6f %.6f %.6f %.6f %.6f]",
      robot_ip_.c_str(),
      hw_positions_[0],
      hw_positions_[1],
      hw_positions_[2],
      hw_positions_[3],
      hw_positions_[4],
      hw_positions_[5]);

    return hardware_interface::CallbackReturn::SUCCESS;
  }

  // ---------------------------------------------------------------
  // Startup safe-pose guard for BOTH command-capable modes.
  // ---------------------------------------------------------------
  bool pose_ok = true;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double delta =
      std::abs(hw_positions_[i] - safe_positions_[i]);

    RCLCPP_INFO(
      logger,
      "Startup guard J%zu: actual=%.6f safe=%.6f delta=%.6f rad",
      i + 1,
      hw_positions_[i],
      safe_positions_[i],
      delta);

    if (delta > startup_pose_tolerance_rad_) {
      pose_ok = false;

      RCLCPP_ERROR(
        logger,
        "Startup guard FAILED on J%zu: delta %.6f rad "
        "> tolerance %.6f rad",
        i + 1,
        delta,
        startup_pose_tolerance_rad_);
    }
  }

  if (!pose_ok) {
    startup_pose_ok_ = false;

    RCLCPP_ERROR(
      logger,
      "%s activation refused: robot is outside safe startup pose.",
      controlModeName());

    client_->disconnect();

    return hardware_interface::CallbackReturn::ERROR;
  }

  if (realServoJMode()) {
    const char robot_state =
      client_->readRobotState();

    if (robot_state != 2) {
      RCLCPP_ERROR(
        logger,
        "REAL SERVOJ activation refused: robot_state=%d, expected idle=2",
        static_cast<int>(robot_state));

      client_->disconnect();

      return hardware_interface::CallbackReturn::ERROR;
    }

    const double control_cycle_ms =
      client_->controlCycleMs();

    if (control_cycle_ms > 0.0 &&
        servoj_time_sec_ * 1000.0 < control_cycle_ms)
    {
      RCLCPP_ERROR(
        logger,
        "REAL SERVOJ activation refused: servoJ time %.3f ms "
        "< Diana control cycle %.3f ms",
        servoj_time_sec_ * 1000.0,
        control_cycle_ms);

      client_->disconnect();

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Critical safety property:
  // command buffers ALWAYS start from measured real position.
  hw_commands_ = hw_positions_;
  last_logged_commands_ = hw_positions_;
  last_servoj_commands_ = hw_positions_;

  startup_pose_ok_ = true;
  have_logged_command_ = false;

  servo_stream_active_ = false;
  servo_stable_time_sec_ = 0.0;

  if (control_mode_ == Thor7ControlMode::REAL_DRY_RUN) {
    RCLCPP_WARN(
      logger,
      "Thor7 REAL DRY-RUN ACTIVATED. "
      "Startup guard PASS. "
      "NO HARDWARE MOTION COMMAND WILL BE SENT.");
  } else {
    RCLCPP_WARN(
      logger,
      "Thor7 REAL SERVOJ MODE ARMED. "
      "Startup guard PASS. robot_state=idle. "
      "servoJ time=%.3fms lookahead=%.3fms gain=%.1f. "
      "No servoJ command is sent until command values change.",
      servoj_time_sec_ * 1000.0,
      servoj_lookahead_sec_ * 1000.0,
      servoj_gain_);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (realServoJMode()) {
    stopServoStream("hardware deactivation");
  }

  startup_pose_ok_ = false;

  if (client_) {
    client_->disconnect();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware"),
    "Thor7 hardware deactivated: mode=%s",
    controlModeName());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Thor7SystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!client_ || !client_->isConnected()) {
    if (realServoJMode()) {
      stopServoStream("SDK health check failed in read()");
    }

    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kJointCount> joints{};

  if (!client_->readJointPositions(joints)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("thor7_hardware.Thor7SystemHardware"),
      "getJointPos() failed: sdk_error=%d",
      client_->lastSdkError());

    if (realServoJMode()) {
      stopServoStream("getJointPos() failure");
    }

    return hardware_interface::return_type::ERROR;
  }

  hw_positions_ = joints;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Thor7SystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!commandModeEnabled()) {
    return hardware_interface::return_type::OK;
  }

  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (!startup_pose_ok_) {
    if (realServoJMode()) {
      stopServoStream("startup pose guard invalid");
    }

    RCLCPP_ERROR(
      logger,
      "%s write rejected: startup pose guard is not valid",
      controlModeName());

    return hardware_interface::return_type::ERROR;
  }

  if (!client_ || !client_->isConnected()) {
    if (realServoJMode()) {
      stopServoStream("Diana connection inactive");
    }

    RCLCPP_ERROR(
      logger,
      "%s write rejected: Diana connection is not active",
      controlModeName());

    return hardware_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------------
  // Common command safety guards.
  // ---------------------------------------------------------------
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double command =
      hw_commands_[i];

    if (!std::isfinite(command)) {
      if (realServoJMode()) {
        stopServoStream("NaN/Inf command");
      }

      RCLCPP_ERROR(
        logger,
        "%s command rejected: J%zu command is NaN/Inf",
        controlModeName(),
        i + 1);

      return hardware_interface::return_type::ERROR;
    }

    const double from_current =
      std::abs(command - hw_positions_[i]);

    if (from_current > max_command_delta_rad_) {
      if (realServoJMode()) {
        stopServoStream("command-current safety limit");
      }

      RCLCPP_ERROR(
        logger,
        "%s command rejected: J%zu target-current=%.6f rad "
        "> max_command_delta=%.6f rad",
        controlModeName(),
        i + 1,
        from_current,
        max_command_delta_rad_);

      return hardware_interface::return_type::ERROR;
    }

    const double from_safe =
      std::abs(command - safe_positions_[i]);

    if (from_safe > max_safe_deviation_rad_) {
      if (realServoJMode()) {
        stopServoStream("command-safe safety limit");
      }

      RCLCPP_ERROR(
        logger,
        "%s command rejected: J%zu target-safe=%.6f rad "
        "> max_safe_deviation=%.6f rad",
        controlModeName(),
        i + 1,
        from_safe,
        max_safe_deviation_rad_);

      return hardware_interface::return_type::ERROR;
    }
  }

  bool changed =
    !have_logged_command_;

  if (!changed) {
    for (std::size_t i = 0; i < kJointCount; ++i) {
      if (std::abs(
            hw_commands_[i] -
            last_logged_commands_[i]) > 0.0005)
      {
        changed = true;
        break;
      }
    }
  }

  // ---------------------------------------------------------------
  // REAL DRY-RUN: absolute safety barrier.
  // ---------------------------------------------------------------
  if (control_mode_ == Thor7ControlMode::REAL_DRY_RUN) {
    if (changed) {
      RCLCPP_WARN(
        logger,
        "DRY-RUN ONLY - NO HARDWARE COMMAND SENT | "
        "target=[%.6f %.6f %.6f %.6f %.6f %.6f] | "
        "actual=[%.6f %.6f %.6f %.6f %.6f %.6f]",
        hw_commands_[0],
        hw_commands_[1],
        hw_commands_[2],
        hw_commands_[3],
        hw_commands_[4],
        hw_commands_[5],
        hw_positions_[0],
        hw_positions_[1],
        hw_positions_[2],
        hw_positions_[3],
        hw_positions_[4],
        hw_positions_[5]);

      last_logged_commands_ = hw_commands_;
      have_logged_command_ = true;
    }

    return hardware_interface::return_type::OK;
  }

  // ---------------------------------------------------------------
  // REAL SERVOJ.
  //
  // Important:
  // - activation initialized last_servoj_commands_ from REAL state
  // - no servoJ() is sent while command remains unchanged
  // - once JTC begins changing the target, stream at controller rate
  // - after target is stable for configured hold time, call stop()
  // ---------------------------------------------------------------
  double max_command_change = 0.0;

  for (std::size_t i = 0; i < kJointCount; ++i) {
    max_command_change =
      std::max(
        max_command_change,
        std::abs(
          hw_commands_[i] -
          last_servoj_commands_[i]));
  }

  if (!servo_stream_active_) {
    if (max_command_change <= servoj_command_epsilon_rad_) {
      return hardware_interface::return_type::OK;
    }

    servo_stream_active_ = true;
    servo_stable_time_sec_ = 0.0;

    RCLCPP_WARN(
      logger,
      "REAL SERVOJ stream STARTED by changing command interface.");
  }

  if (max_command_change > servoj_command_epsilon_rad_) {
    servo_stable_time_sec_ = 0.0;
  } else {
    const double dt =
      std::max(0.0, period.seconds());

    servo_stable_time_sec_ += dt;
  }

  if (servo_stable_time_sec_ >= servoj_stop_after_stable_sec_) {
    last_servoj_commands_ = hw_commands_;
    stopServoStream("command target stable");
    return hardware_interface::return_type::OK;
  }

  std::array<double, 7> command{
    hw_commands_[0],
    hw_commands_[1],
    hw_commands_[2],
    hw_commands_[3],
    hw_commands_[4],
    hw_commands_[5],
    0.0
  };

  const int ret =
    ::servoJ(
      command.data(),
      servoj_time_sec_,
      servoj_lookahead_sec_,
      servoj_gain_,
      robot_ip_.c_str());

  if (ret < 0) {
    RCLCPP_ERROR(
      logger,
      "REAL SERVOJ servoJ() failed: return=%d sdk_error=%d",
      ret,
      client_->lastSdkError());

    stopServoStream("servoJ() failure");

    return hardware_interface::return_type::ERROR;
  }

  last_servoj_commands_ = hw_commands_;

  if (changed) {
    RCLCPP_WARN(
      logger,
      "REAL SERVOJ COMMAND SENT | "
      "target=[%.6f %.6f %.6f %.6f %.6f %.6f] | "
      "actual=[%.6f %.6f %.6f %.6f %.6f %.6f]",
      hw_commands_[0],
      hw_commands_[1],
      hw_commands_[2],
      hw_commands_[3],
      hw_commands_[4],
      hw_commands_[5],
      hw_positions_[0],
      hw_positions_[1],
      hw_positions_[2],
      hw_positions_[3],
      hw_positions_[4],
      hw_positions_[5]);

    last_logged_commands_ = hw_commands_;
    have_logged_command_ = true;
  }

  return hardware_interface::return_type::OK;
}

}  // namespace thor7_hardware

PLUGINLIB_EXPORT_CLASS(
  thor7_hardware::Thor7SystemHardware,
  hardware_interface::SystemInterface)
