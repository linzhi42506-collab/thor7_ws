// thor7_sim: gripper_state_publisher
//
// 作用：AG-145 夹爪的 <mimic> 关节（手指、内指节、指尖等）在 ros2_control
//       FakeSystem 里只有主控关节 ag145_gripper_finger1_joint 有真实状态。
//       本节点从 /joint_states 读取 finger1 位置/速度，按 URDF 中声明的
//       mimic 关系（multiplier/offset）计算其余夹爪关节，合并后重新发布
//       到 /joint_states，让 robot_state_publisher / RViz / MoveIt 能正确
//       显示并规划夹爪手指开合。
//
// 说明：发布的消息带 header.frame_id="gripper_state_publisher" 标记，
//       收到该标记的消息直接忽略，避免自发布回声死循环。

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <urdf/model.h>

#include <map>
#include <string>
#include <vector>

class GripperStatePublisher : public rclcpp::Node
{
public:
  GripperStatePublisher() : Node("gripper_state_publisher")
  {
    // 先声明再读取：launch 传入的 robot_description 覆盖默认值
    this->declare_parameter("robot_description", "");
    std::string robot_description;
    try {
      robot_description = this->get_parameter("robot_description").as_string();
    } catch (const std::exception & e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to read robot_description: %s", e.what());
    }

    if (robot_description.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "robot_description parameter is empty. Pass the full URDF as 'robot_description'.");
    } else {
      urdf::Model model;
      if (!model.initString(robot_description)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to parse robot_description");
      } else {
        for (const auto & kv : model.joints_) {
          const auto & joint = kv.second;
          if (joint && joint->mimic) {
            MimicSpec spec;
            spec.master = joint->mimic->joint_name;
            spec.multiplier = joint->mimic->multiplier;
            spec.offset = joint->mimic->offset;
            mimic_map_[kv.first] = spec;
            RCLCPP_INFO(
              this->get_logger(), "mimic: %s -> %s * %g + %g",
              kv.first.c_str(), spec.master.c_str(), spec.multiplier, spec.offset);
          }
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %zu mimic joints", mimic_map_.size());
      }
    }

    sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      [this](const sensor_msgs::msg::JointState::SharedPtr msg) { jointStateCb(msg); });
    pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
  }

private:
  struct MimicSpec
  {
    std::string master;
    double multiplier = 1.0;
    double offset = 0.0;
  };

  void jointStateCb(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // 忽略自己发布的合并消息
    if (msg->header.frame_id == kFrameMarker) {
      return;
    }

    // 建立 关节名 -> 位置/速度 索引
    std::map<std::string, double> pos, vel;
    for (size_t i = 0; i < msg->name.size() && i < msg->position.size(); ++i) {
      pos[msg->name[i]] = msg->position[i];
    }
    for (size_t i = 0; i < msg->name.size() && i < msg->velocity.size(); ++i) {
      vel[msg->name[i]] = msg->velocity[i];
    }

    // 只发布 mimic 关节（不含 joint1~6 / finger1，避免与 joint_state_broadcaster
    // 对同一关节名重复发布不同值）
    sensor_msgs::msg::JointState out;
    out.header = msg->header;
    out.header.frame_id = kFrameMarker;
    bool appended = false;

    for (const auto & kv : mimic_map_) {
      const std::string & mimic = kv.first;
      const MimicSpec & spec = kv.second;

      auto master_pos = pos.find(spec.master);
      if (master_pos == pos.end()) {
        continue;  // 主控关节当前帧没有状态，跳过
      }
      const double p = spec.multiplier * master_pos->second + spec.offset;
      const double v = [&]() {
        auto it = vel.find(spec.master);
        return it != vel.end() ? spec.multiplier * it->second : 0.0;
      }();

      out.name.push_back(mimic);
      out.position.push_back(p);
      out.velocity.push_back(v);
      out.effort.push_back(0.0);
      appended = true;
    }

    if (appended) {
      pub_->publish(out);
    }
  }

  static constexpr const char * kFrameMarker = "gripper_state_publisher";
  std::map<std::string, MimicSpec> mimic_map_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr pub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GripperStatePublisher>());
  rclcpp::shutdown();
  return 0;
}
