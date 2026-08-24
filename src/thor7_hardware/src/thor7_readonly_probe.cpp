// thor7_hardware: thor7_readonly_probe
//
// 只读连接验证节点：连接 Diana SDK，发布 6 个关节的实际位置与诊断信息。
// 本节点绝不调用任何运动/状态改变接口。
//
// 话题：
//   /thor7/readback_joint_states (sensor_msgs/JointState)
//   /diagnostics (diagnostic_msgs/DiagnosticArray)
//
// 参数：
//   robot_ip                默认 192.168.10.75
//   publish_rate            默认 10.0 Hz
//   connection_retry_count  默认 3
//   connection_retry_delay_ms 默认 1000
//   publish_joint_states    默认 true
//   joint_state_topic       默认 /thor7/readback_joint_states

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <string>
#include <vector>

#include "thor7_hardware/diana_readonly_client.hpp"

using thor7_hardware::DianaReadonlyClient;

namespace {

std::string robotStateText(char s)
{
  switch (s) {
    case 0: return "running";
    case 1: return "paused";
    case 2: return "idle";
    case 3: return "free-driving";
    case 4: return "zero-space-free-driving";
    case 5: return "hold-brake";
    case 6: return "error";
    case 7: return "handling-error";
    case 8: return "rescueMode";
    default: return "unknown";
  }
}

}  // namespace

class Thor7ReadonlyProbe : public rclcpp::Node
{
public:
  Thor7ReadonlyProbe() : Node("thor7_readonly_probe")
  {
    const auto robot_ip = this->declare_parameter<std::string>("robot_ip", "192.168.10.75");
    const auto publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
    const auto retry_count = this->declare_parameter<int>("connection_retry_count", 3);
    const auto retry_delay_ms = this->declare_parameter<int>("connection_retry_delay_ms", 1000);
    const auto publish_js = this->declare_parameter<bool>("publish_joint_states", true);
    const auto js_topic = this->declare_parameter<std::string>("joint_state_topic", "/thor7/readback_joint_states");

    client_ = std::make_unique<DianaReadonlyClient>(robot_ip, retry_count, retry_delay_ms);

    if (publish_js) {
      js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
    }
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    if (!client_->connect()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Diana SDK connect failed (ip=%s, joint_count=%d, last_sdk_error=%d: %s). "
        "No joint states will be published until connected.",
        robot_ip.c_str(), client_->jointCount(), client_->lastSdkError(),
        client_->describeError(client_->lastSdkError()).c_str());
      connected_ = false;
    } else {
      connected_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Diana SDK connected (ip=%s, joint_count=%d, control_cycle=%.3f ms)",
        robot_ip.c_str(), client_->jointCount(), client_->controlCycleMs());
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
  }

private:
  void tick()
  {
    if (!client_->isConnected()) {
      // 断线：不发布旧值冒充真实状态，报告 ERROR
      if (connected_) {
        RCLCPP_ERROR(this->get_logger(), "Diana SDK connection lost. Stopping publication.");
        connected_ = false;
      }
      publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::ERROR,
                         "connection lost / not connected",
                         "no fresh data; not publishing stale joint states");
      return;
    }

    const char state = client_->readRobotState();
    std::array<double, 6> joints{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    const bool ok = client_->readJointPositions(joints);

    if (!ok) {
      // 数据读失败：标 STALE，不发布旧值
      publishDiagnostics(diagnostic_msgs::msg::DiagnosticStatus::STALE,
                         "read failed",
                         "getJointPos() failed, sdk_error=" +
                           std::to_string(client_->lastSdkError()));
      return;
    }

    if (js_pub_) {
      sensor_msgs::msg::JointState js;
      js.header.stamp = this->now();
      js.header.frame_id = "base_link";
      js.name = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
      js.position.assign(joints.begin(), joints.end());
      // velocity 无可靠数据，留空，不伪造 0
      js_pub_->publish(js);
    }

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), 2000,
      "robot_state=%d(%s) joint_count=%d control_cycle=%.3fms "
      "J=[%.4f %.4f %.4f %.4f %.4f %.4f]",
      static_cast<int>(state), robotStateText(state).c_str(), client_->jointCount(),
      client_->controlCycleMs(), joints[0], joints[1], joints[2], joints[3], joints[4], joints[5]);

    publishDiagnostics(
      diagnostic_msgs::msg::DiagnosticStatus::OK,
      "connected",
      "robot_state=" + std::string(robotStateText(state)) +
        " sdk_error=" + std::to_string(client_->lastSdkError()));
  }

  void publishDiagnostics(uint8_t level, const std::string & summary, const std::string & detail)
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = level;
    st.name = "thor7_readonly_probe";
    st.message = summary;
    st.hardware_id = client_->ip();
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "detail";
    kv.value = detail;
    st.values.push_back(kv);
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  std::unique_ptr<DianaReadonlyClient> client_;
  bool connected_{false};
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Thor7ReadonlyProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
