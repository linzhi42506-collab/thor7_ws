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

  std::string robot_ip_{"192.168.10.75"};
  int retry_count_{3};
  int retry_delay_ms_{1000};

  // false:
  //   原来的严格 READ-ONLY 模式，没有 command interface。
  //
  // true:
  //   暴露 position command interface，但 write() 只做安全检查和打印，
  //   绝不向 Diana SDK 发送运动命令。
  bool dry_run_command_mode_{false};

  // 启动时真机必须接近指定安全姿态。
  // 默认 ±3 degrees。
  double startup_pose_tolerance_rad_{0.0523598776};

  // DRY-RUN 中单个命令相对真机当前实际位置最大允许偏差。
  // 默认 1 degree。
  double max_command_delta_rad_{0.0174532925};

  // DRY-RUN 中允许目标偏离基准安全姿态的最大范围。
  // 默认 5 degrees。
  double max_safe_deviation_rad_{0.0872664626};

  // 用户指定的 Thor7 基准安全姿态。
  std::array<double, kJointCount> safe_positions_{
    1.876752545,
   -0.048520153,
    1.863313509,
    0.008377580,
    1.436405974,
   -0.287979327
  };

  // 真机真实反馈。
  std::array<double, kJointCount> hw_positions_{};

  // ros2_control 写入的目标。
  // DRY-RUN 阶段只读取并验证，不发送给真机。
  std::array<double, kJointCount> hw_commands_{};

  std::array<double, kJointCount> last_logged_commands_{};

  bool startup_pose_ok_{false};
  bool have_logged_command_{false};

  std::unique_ptr<DianaReadonlyClient> client_;
};

}  // namespace thor7_hardware
