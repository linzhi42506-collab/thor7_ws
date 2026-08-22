// thor7_sim: spawn_test_object
//
// 作用：在 MoveIt 规划场景（planning scene）中添加一个可抓取的测试物体
//       （默认立方体），参数可在 config/sim.yaml 中配置。
//       move_group 启动需要约 2~3 秒，本节点会重试发布多次，避免漏发。

#include <rclcpp/rclcpp.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <chrono>
#include <thread>

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("spawn_test_object");

  const auto frame_id = node->declare_parameter<std::string>("frame_id", "base_link");
  const auto obj_name = node->declare_parameter<std::string>("object_name", "test_box");
  const auto obj_x = node->declare_parameter<double>("object_x", 0.35);
  const auto obj_y = node->declare_parameter<double>("object_y", 0.0);
  const auto obj_z = node->declare_parameter<double>("object_z", 0.10);
  const auto obj_size = node->declare_parameter<double>("object_size", 0.05);
  const auto retries = node->declare_parameter<int>("retries", 10);

  RCLCPP_INFO(
    node->get_logger(), "Spawn '%s' at (%.2f, %.2f, %.2f) size %.2f in frame '%s'",
    obj_name.c_str(), obj_x, obj_y, obj_z, obj_size, frame_id.c_str());

  moveit::planning_interface::PlanningSceneInterface psi;

  moveit_msgs::msg::CollisionObject co;
  co.header.frame_id = frame_id;
  co.id = obj_name;

  shape_msgs::msg::SolidPrimitive box;
  box.type = shape_msgs::msg::SolidPrimitive::BOX;
  box.dimensions = {obj_size, obj_size, obj_size};

  geometry_msgs::msg::Pose pose;
  pose.position.x = obj_x;
  pose.position.y = obj_y;
  pose.position.z = obj_z;
  pose.orientation.w = 1.0;

  co.primitives.push_back(box);
  co.primitive_poses.push_back(pose);
  co.operation = moveit_msgs::msg::CollisionObject::ADD;

  bool ok = false;
  for (int i = 0; i < retries; ++i) {
    psi.applyCollisionObject(co);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // 确认规划场景中已经出现该物体
    auto objs = psi.getObjects();
    if (objs.count(obj_name) > 0) {
      ok = true;
      break;
    }
  }

  if (ok) {
    RCLCPP_INFO(node->get_logger(), "Object '%s' is in the planning scene.", obj_name.c_str());
  } else {
    RCLCPP_WARN(
      node->get_logger(),
      "Could not confirm '%s' in planning scene (move_group may not be ready yet).",
      obj_name.c_str());
  }

  rclcpp::shutdown();
  return ok ? 0 : 1;
}
