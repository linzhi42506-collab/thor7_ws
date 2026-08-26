#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "thor7_hardware/diana_readonly_client.hpp"

namespace thor7_hardware
{

enum class Thor7ControlMode
{
  READ_ONLY,
  REAL_DRY_RUN,
  REAL_SERVOJ
};

class Thor7SystemHardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  static constexpr std::size_t kJointCount = 6;

  bool commandModeEnabled() const;
  bool realServoJMode() const;
  const char * controlModeName() const;

  void stopServoStream(const char * reason);

  Thor7ControlMode control_mode_{Thor7ControlMode::READ_ONLY};

  std::string robot_ip_{"192.168.10.75"};
  int retry_count_{3};
  int retry_delay_ms_{1000};

  // Command-mode safety values must be supplied by robot_description.
  // Zero is deliberately not a usable command-mode fallback.
  double startup_pose_tolerance_rad_{0.0};
  double max_command_delta_rad_{0.0};
  double max_safe_deviation_rad_{0.0};

  // 已经通过独立真机测试验证的第一版 servoJ 参数。
  double servoj_time_sec_{0.020};
  double servoj_lookahead_sec_{0.100};
  double servoj_gain_{300.0};

  // 用于判断 JTC 是否真正开始更新 command。
  double servoj_command_epsilon_rad_{1.0e-6};

  // 轨迹目标稳定后继续保持一段时间，再明确 stop()。
  double servoj_stop_after_stable_sec_{0.5};

  // Loaded from the canonical safety YAML through ros2_control Xacro.
  std::array<double, kJointCount> safe_positions_{};

  // Defense in depth: on_activate() refuses command mode unless every
  // required safety parameter was explicitly loaded and validated.
  bool command_safety_parameters_loaded_{false};

  std::array<double, kJointCount> hw_positions_{};
  std::array<double, kJointCount> hw_commands_{};

  std::array<double, kJointCount> last_logged_commands_{};
  std::array<double, kJointCount> last_servoj_commands_{};

  bool startup_pose_ok_{false};
  bool have_logged_command_{false};

  bool servo_stream_active_{false};
  double servo_stable_time_sec_{0.0};

  std::unique_ptr<DianaReadonlyClient> client_;
};

}  // namespace thor7_hardware
