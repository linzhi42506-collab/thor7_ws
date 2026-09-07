#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <rclcpp/rclcpp.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <std_srvs/srv/trigger.hpp>

using namespace std::chrono_literals;

class WorkcellCollisionScene : public rclcpp::Node
{
public:
  WorkcellCollisionScene()
  : Node("thor7_workcell_collision_scene")
  {
    declare_parameter("frame_id", "base_link");
    declare_parameter("table_id", "thor7_pick_table");
    declare_parameter("table_center_x", -0.473);
    declare_parameter("table_center_y", 0.502);
    declare_parameter("table_top_z", -0.023);
    declare_parameter("table_size_x", 0.600);
    declare_parameter("table_size_y", 0.600);
    declare_parameter("table_thickness", 0.050);

    reload_service_ = create_service<std_srvs::srv::Trigger>(
      "/thor7_workcell/reload_collision_scene",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
      {
        response->success = applyTable();
        response->message = response->success ?
          "table collision object applied" :
          "failed to apply table collision object";
      });

    // Reapply periodically so the object is restored if move_group starts
    // after this node or its planning scene is cleared.
    timer_ = create_wall_timer(2s, [this]() { (void)applyTable(); });

    RCLCPP_INFO(
      get_logger(),
      "Collision-scene publisher ready. It only updates MoveIt geometry; "
      "it never sends motion commands.");
  }

private:
  bool applyTable()
  {
    const double size_x = get_parameter("table_size_x").as_double();
    const double size_y = get_parameter("table_size_y").as_double();
    const double thickness = get_parameter("table_thickness").as_double();
    const double top_z = get_parameter("table_top_z").as_double();

    if (size_x <= 0.0 || size_y <= 0.0 || thickness <= 0.0) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "Table dimensions must all be greater than zero");
      return false;
    }

    moveit_msgs::msg::CollisionObject table;
    table.header.frame_id = get_parameter("frame_id").as_string();
    table.id = get_parameter("table_id").as_string();

    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = {size_x, size_y, thickness};

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = get_parameter("table_center_x").as_double();
    pose.position.y = get_parameter("table_center_y").as_double();
    pose.position.z = top_z - thickness / 2.0;

    table.primitives.push_back(box);
    table.primitive_poses.push_back(pose);
    table.operation = moveit_msgs::msg::CollisionObject::ADD;

    moveit::planning_interface::PlanningSceneInterface scene;
    const bool applied = scene.applyCollisionObject(table);

    if (applied && !logged_success_) {
      RCLCPP_INFO(
        get_logger(),
        "Applied table '%s' in %s: center=(%.3f, %.3f), "
        "top_z=%.3f, size=(%.3f, %.3f, %.3f) m",
        table.id.c_str(), table.header.frame_id.c_str(),
        pose.position.x, pose.position.y, top_z,
        size_x, size_y, thickness);
      logged_success_ = true;
    } else if (!applied) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "MoveIt has not accepted the table collision object yet; retrying");
    }

    return applied;
  }

  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reload_service_;
  rclcpp::TimerBase::SharedPtr timer_;
  bool logged_success_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WorkcellCollisionScene>());
  rclcpp::shutdown();
  return 0;
}
