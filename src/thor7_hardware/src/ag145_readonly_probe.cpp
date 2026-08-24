// thor7_hardware: ag145_readonly_probe
//
// 只读连接验证节点：通过 USB-RS485 (Modbus RTU) 读取 AG145 夹爪状态。
// 本节点绝不写任何寄存器、绝不调用 Initialize()。
//
// 话题：
//   /ag145/readback_joint_state (sensor_msgs/JointState)
//   /diagnostics (diagnostic_msgs/DiagnosticArray)
//
// 参数：
//   serial_port    默认 /dev/ttyUSB0
//   baud_rate      默认 115200
//   slave_id       默认 1
//   timeout_ms     默认 200
//   retries        默认 3
//   publish_rate   默认 10.0 Hz
//   joint_state_topic 默认 /ag145/readback_joint_state

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <diagnostic_msgs/msg/diagnostic_array.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <chrono>
#include <string>

#include "thor7_hardware/ag145_modbus_client.hpp"
#include "thor7_hardware/modbus_utils.hpp"

using thor7_hardware::Ag145ModbusClient;

class Ag145ReadonlyProbe : public rclcpp::Node
{
public:
  Ag145ReadonlyProbe() : Node("ag145_readonly_probe")
  {
    const auto port = this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    const auto baud = this->declare_parameter<int>("baud_rate", 115200);
    const auto slave_id = this->declare_parameter<int>("slave_id", 1);
    const auto timeout_ms = this->declare_parameter<int>("timeout_ms", 200);
    const auto retries = this->declare_parameter<int>("retries", 3);
    const auto publish_rate = this->declare_parameter<double>("publish_rate", 10.0);
    const auto js_topic = this->declare_parameter<std::string>(
      "joint_state_topic", "/ag145/readback_joint_state");

    client_ = std::make_unique<Ag145ModbusClient>(
      port, static_cast<uint32_t>(baud), static_cast<uint8_t>(slave_id), timeout_ms, retries);

    js_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(js_topic, 10);
    diag_pub_ = this->create_publisher<diagnostic_msgs::msg::DiagnosticArray>("/diagnostics", 10);

    if (!client_->open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open serial port %s", port.c_str());
    } else {
      RCLCPP_INFO(
        this->get_logger(),
        "Serial port opened (%s, %ld baud, slave 0x%02lx). READ-ONLY: no writes will be issued.",
        port.c_str(), baud, slave_id);
    }

    const auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(period, [this]() { tick(); });
  }

private:
  void tick()
  {
    if (!client_->isOpen()) {
      publishDiag(
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "serial port not open",
        "cannot read AG145; not publishing stale data");
      return;
    }

    uint16_t init_fb = 0, status = 0, device_pos = 0;
    const bool ok_init = client_->readInitFeedback(init_fb);
    const bool ok_status = client_->readStatus(status);
    const bool ok_pos = client_->readPosition(device_pos);

    if (!(ok_init && ok_status && ok_pos)) {
      publishDiag(
        diagnostic_msgs::msg::DiagnosticStatus::STALE, "read failed",
        "init_fb_ok=" + std::to_string(ok_init) +
          " status_ok=" + std::to_string(ok_status) +
          " pos_ok=" + std::to_string(ok_pos) +
          " crc_errors=" + std::to_string(client_->crcErrorCount()) +
          " timeouts=" + std::to_string(client_->timeoutCount()));
      return;
    }

    const double joint_rad = thor7_hardware::devicePositionToJointRad(
      static_cast<int>(device_pos));
    const std::string status_text = thor7_hardware::gripperStatusText(status);

    sensor_msgs::msg::JointState js;
    js.header.stamp = this->now();
    js.header.frame_id = "base_link";
    js.name = {"ag145_gripper_finger1_joint"};
    js.position = {joint_rad};
    // velocity 无可靠数据，留空，不伪造
    js_pub_->publish(js);

    // 状态语义：2 holding / 3 dropped(WARN) / 4 error(ERROR) / unknown
    uint8_t diag_level = diagnostic_msgs::msg::DiagnosticStatus::OK;
    if (status == 3) {
      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      RCLCPP_WARN(this->get_logger(), "AG145 status=dropped");
    } else if (status == 4) {
      diag_level = diagnostic_msgs::msg::DiagnosticStatus::ERROR;
      RCLCPP_ERROR(this->get_logger(), "AG145 status=error");
    } else if (status > 4) {
      diag_level = diagnostic_msgs::msg::DiagnosticStatus::WARN;
      RCLCPP_WARN(this->get_logger(), "AG145 status=%u unknown", static_cast<unsigned>(status));
    }

    publishDiag(
      diag_level, "read ok",
      "init_fb=" + std::to_string(init_fb) +
        " status=" + std::to_string(status) + "(" + status_text + ")" +
        " device_pos=" + std::to_string(device_pos) +
        " joint_rad=" + std::to_string(joint_rad) +
        " crc_errors=" + std::to_string(client_->crcErrorCount()) +
        " timeouts=" + std::to_string(client_->timeoutCount()));
  }

  void publishDiag(uint8_t level, const std::string & summary, const std::string & detail)
  {
    diagnostic_msgs::msg::DiagnosticArray arr;
    arr.header.stamp = this->now();
    diagnostic_msgs::msg::DiagnosticStatus st;
    st.level = level;
    st.name = "ag145_readonly_probe";
    st.message = summary;
    st.hardware_id = client_->isOpen() ? "serial-open" : "serial-closed";
    diagnostic_msgs::msg::KeyValue kv;
    kv.key = "detail";
    kv.value = detail;
    st.values.push_back(kv);
    arr.status.push_back(st);
    diag_pub_->publish(arr);
  }

  std::unique_ptr<Ag145ModbusClient> client_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Publisher<diagnostic_msgs::msg::DiagnosticArray>::SharedPtr diag_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Ag145ReadonlyProbe>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
