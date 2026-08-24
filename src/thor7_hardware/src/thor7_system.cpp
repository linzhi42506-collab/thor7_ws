#include "thor7_hardware/thor7_system.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace thor7_hardware
{

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
  // Hardware parameters
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
        logger, "Invalid connection_retry_count: %s", it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  it = info_.hardware_parameters.find("connection_retry_delay_ms");
  if (it != info_.hardware_parameters.end()) {
    try {
      retry_delay_ms_ = std::stoi(it->second);
    } catch (...) {
      RCLCPP_ERROR(
        logger, "Invalid connection_retry_delay_ms: %s", it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  it = info_.hardware_parameters.find("dry_run_command_mode");
  if (it != info_.hardware_parameters.end()) {
    const std::string value = it->second;
    dry_run_command_mode_ =
      value == "true" ||
      value == "True" ||
      value == "TRUE" ||
      value == "1" ||
      value == "yes";
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
        return std::isfinite(output);
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
        "startup_pose_tolerance_rad", startup_pose_tolerance_rad_) ||
      !parse_double_parameter(
        "max_command_delta_rad", max_command_delta_rad_) ||
      !parse_double_parameter(
        "max_safe_deviation_rad", max_safe_deviation_rad_))
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

    if (dry_run_command_mode_) {
      if (joint.command_interfaces.size() != 1 ||
          joint.command_interfaces[0].name !=
            hardware_interface::HW_IF_POSITION)
      {
        RCLCPP_ERROR(
          logger,
          "DRY-RUN mode requires exactly one position command interface on %s",
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

  client_ = std::make_unique<DianaReadonlyClient>(
    robot_ip_,
    retry_count_,
    retry_delay_ms_);

  if (dry_run_command_mode_) {
    RCLCPP_WARN(
      logger,
      "Initialized Thor7 DRY-RUN command mode for %s. "
      "Command interfaces will exist, but NO hardware motion command is sent.",
      robot_ip_.c_str());
  } else {
    RCLCPP_INFO(
      logger,
      "Initialized READ-ONLY Thor7 hardware interface for %s",
      robot_ip_.c_str());
  }

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
  if (!dry_run_command_mode_) {
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

  if (!dry_run_command_mode_) {
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
  // DRY-RUN startup pose guard
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
        "Startup guard FAILED on J%zu: delta %.6f rad > tolerance %.6f rad",
        i + 1,
        delta,
        startup_pose_tolerance_rad_);
    }
  }

  if (!pose_ok) {
    startup_pose_ok_ = false;

    RCLCPP_ERROR(
      logger,
      "DRY-RUN activation refused: Thor7 is not close enough to "
      "the configured safe startup pose.");

    client_->disconnect();

    return hardware_interface::CallbackReturn::ERROR;
  }

  // Critical:
  // command buffer starts from the REAL current position.
  // Never initialize command targets to zero.
  hw_commands_ = hw_positions_;
  last_logged_commands_ = hw_positions_;

  startup_pose_ok_ = true;
  have_logged_command_ = false;

  RCLCPP_WARN(
    logger,
    "Thor7 DRY-RUN command mode ACTIVATED. "
    "Startup pose guard PASS. "
    "NO HARDWARE MOTION COMMAND WILL BE SENT.");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  startup_pose_ok_ = false;

  if (client_) {
    client_->disconnect();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware"),
    "Thor7 hardware deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type Thor7SystemHardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!client_ || !client_->isConnected()) {
    return hardware_interface::return_type::ERROR;
  }

  std::array<double, kJointCount> joints{};

  if (!client_->readJointPositions(joints)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("thor7_hardware.Thor7SystemHardware"),
      "getJointPos() failed: sdk_error=%d",
      client_->lastSdkError());

    return hardware_interface::return_type::ERROR;
  }

  hw_positions_ = joints;

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type Thor7SystemHardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!dry_run_command_mode_) {
    // Original READ-ONLY mode.
    return hardware_interface::return_type::OK;
  }

  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (!startup_pose_ok_) {
    RCLCPP_ERROR(
      logger,
      "DRY-RUN write rejected: startup pose guard is not valid");
    return hardware_interface::return_type::ERROR;
  }

  if (!client_ || !client_->isConnected()) {
    RCLCPP_ERROR(
      logger,
      "DRY-RUN write rejected: Diana connection is not active");
    return hardware_interface::return_type::ERROR;
  }

  // ---------------------------------------------------------------
  // Validate every command.
  // ---------------------------------------------------------------
  for (std::size_t i = 0; i < kJointCount; ++i) {
    const double command = hw_commands_[i];

    if (!std::isfinite(command)) {
      RCLCPP_ERROR(
        logger,
        "DRY-RUN command rejected: J%zu command is NaN/Inf",
        i + 1);
      return hardware_interface::return_type::ERROR;
    }

    const double from_current =
      std::abs(command - hw_positions_[i]);

    if (from_current > max_command_delta_rad_) {
      RCLCPP_ERROR(
        logger,
        "DRY-RUN command rejected: J%zu target-current=%.6f rad "
        "> max_command_delta=%.6f rad",
        i + 1,
        from_current,
        max_command_delta_rad_);

      return hardware_interface::return_type::ERROR;
    }

    const double from_safe =
      std::abs(command - safe_positions_[i]);

    if (from_safe > max_safe_deviation_rad_) {
      RCLCPP_ERROR(
        logger,
        "DRY-RUN command rejected: J%zu target-safe=%.6f rad "
        "> max_safe_deviation=%.6f rad",
        i + 1,
        from_safe,
        max_safe_deviation_rad_);

      return hardware_interface::return_type::ERROR;
    }
  }

  bool changed = !have_logged_command_;

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

  // ===============================================================
  // HARD SAFETY BARRIER
  //
  // This function intentionally performs NO Diana SDK actuation call.
  // The real Thor7 remains physically untouched by ROS commands.
  // ===============================================================

  return hardware_interface::return_type::OK;
}

}  // namespace thor7_hardware

PLUGINLIB_EXPORT_CLASS(
  thor7_hardware::Thor7SystemHardware,
  hardware_interface::SystemInterface)
