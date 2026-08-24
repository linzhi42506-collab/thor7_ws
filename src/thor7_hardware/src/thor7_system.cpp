#include "thor7_hardware/thor7_system.hpp"

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

  // 第一阶段只允许 Thor7 六轴机械臂。
  if (info_.joints.size() != kJointCount) {
    RCLCPP_ERROR(
      logger,
      "READ-ONLY hardware expects exactly 6 arm joints, but URDF provides %zu",
      info_.joints.size());
    return hardware_interface::CallbackReturn::ERROR;
  }

  for (std::size_t i = 0; i < kJointCount; ++i) {
    const std::string expected_name = "joint" + std::to_string(i + 1);
    const auto & joint = info_.joints[i];

    if (joint.name != expected_name) {
      RCLCPP_ERROR(
        logger,
        "Joint order/name mismatch: index %zu expected '%s', got '%s'",
        i, expected_name.c_str(), joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 真机只读阶段：不允许出现任何 command interface。
    if (!joint.command_interfaces.empty()) {
      RCLCPP_ERROR(
        logger,
        "READ-ONLY hardware refuses command interface on %s",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // 第一阶段只导出真实 position，绝不伪造 velocity。
    if (joint.state_interfaces.size() != 1 ||
        joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        logger,
        "%s must contain exactly one state interface: position",
        joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  auto it = info_.hardware_parameters.find("robot_ip");
  if (it != info_.hardware_parameters.end()) {
    robot_ip_ = it->second;
  }

  it = info_.hardware_parameters.find("connection_retry_count");
  if (it != info_.hardware_parameters.end()) {
    try {
      retry_count_ = std::stoi(it->second);
    } catch (...) {
      RCLCPP_ERROR(logger, "Invalid connection_retry_count: %s", it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  it = info_.hardware_parameters.find("connection_retry_delay_ms");
  if (it != info_.hardware_parameters.end()) {
    try {
      retry_delay_ms_ = std::stoi(it->second);
    } catch (...) {
      RCLCPP_ERROR(logger, "Invalid connection_retry_delay_ms: %s", it->second.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  for (auto & position : hw_positions_) {
    position = std::numeric_limits<double>::quiet_NaN();
  }

  client_ = std::make_unique<DianaReadonlyClient>(
    robot_ip_, retry_count_, retry_delay_ms_);

  RCLCPP_INFO(
    logger,
    "Initialized READ-ONLY Thor7 hardware interface for %s",
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
  // 安全要求：
  // 第一阶段绝不向 ros2_control 暴露任何真机 command interface。
  return {};
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  const auto logger =
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware");

  if (!client_) {
    RCLCPP_ERROR(logger, "DianaReadonlyClient was not initialized");
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

  RCLCPP_INFO(
    logger,
    "Thor7 READ-ONLY hardware activated: ip=%s "
    "J=[%.4f %.4f %.4f %.4f %.4f %.4f]",
    robot_ip_.c_str(),
    hw_positions_[0],
    hw_positions_[1],
    hw_positions_[2],
    hw_positions_[3],
    hw_positions_[4],
    hw_positions_[5]);

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn Thor7SystemHardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  if (client_) {
    client_->disconnect();
  }

  RCLCPP_INFO(
    rclcpp::get_logger("thor7_hardware.Thor7SystemHardware"),
    "Thor7 READ-ONLY hardware deactivated");

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
  // ================================================================
  // READ-ONLY SAFETY BARRIER
  //
  // 不调用 releaseBrake()
  // 不调用 moveJToTarget()
  // 不调用 servoJ()
  // 不调用 speedJ()
  // 不调用任何会改变真实机械臂状态的 Diana SDK API
  // ================================================================
  return hardware_interface::return_type::OK;
}

}  // namespace thor7_hardware

PLUGINLIB_EXPORT_CLASS(
  thor7_hardware::Thor7SystemHardware,
  hardware_interface::SystemInterface)
