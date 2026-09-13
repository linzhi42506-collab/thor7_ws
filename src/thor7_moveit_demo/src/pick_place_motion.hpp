#pragma once
#include <algorithm>
#include <functional>
#include <map>
#include <mutex>
#include <set>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <moveit/collision_detection/collision_matrix.h>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometric_shapes/shapes.h>
#include "refreshed_tool_pose.hpp"

namespace thor7_grasp
{
class PickPlaceMotion
{
public:
  using State = moveit::core::RobotState;
  using Plan = moveit::planning_interface::MoveGroupInterface::Plan;
  explicit PickPlaceMotion(rclcpp::Node & node) : node_(node)
  {
    node_.declare_parameter("home_joints", std::vector<double>{});
    node_.declare_parameter("place_joints", std::vector<double>{});
    node_.declare_parameter("pick_lift", .080);
    node_.declare_parameter("place_approach", .080);
    node_.declare_parameter("place_retreat", .080);
    node_.declare_parameter("carried_size", std::vector<double>{.05, .10, .025});
    node_.declare_parameter("carried_padding", .003);
    node_.declare_parameter("transfer_velocity_scaling", .15);
    node_.declare_parameter("transfer_acceleration_scaling", .10);
    node_.declare_parameter("vertical_velocity_scaling", .03);
    node_.declare_parameter("vertical_acceleration_scaling", .03);
    node_.declare_parameter("retain_placed_objects", true);
    joints_ = node_.create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 20, [this](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
        if (msg->name.size() != msg->position.size()) { return; }
        std::lock_guard<std::mutex> lock(mutex_);
        for (size_t i = 0; i < msg->name.size(); ++i) {
          if (std::isfinite(msg->position[i])) {
            feedback_[msg->name[i]] = {msg->position[i], std::chrono::steady_clock::now()};
          }
        }
      });
    display_ = node_.create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path", 10);
  }

  void initialize()
  {
    if (initialized_) { return; }
    arm_ = std::make_unique<moveit::planning_interface::MoveGroupInterface>(
      node_.shared_from_this(), "thor7_arm");
    arm_->setEndEffectorLink("tool0"); arm_->setPoseReferenceFrame("base_link");
    arm_->setPlanningTime(10.0); arm_->setNumPlanningAttempts(5);
    arm_->setGoalJointTolerance(.001);
    scene_ = std::make_unique<moveit::planning_interface::PlanningSceneInterface>();
    for (const auto * name : {"home_joints", "place_joints"}) {
      const auto values = node_.get_parameter(name).as_double_array();
      if (values.size() != 6 || !std::all_of(values.begin(), values.end(),
          [](double v) { return std::isfinite(v); })) {
        throw std::runtime_error(std::string(name) + " requires six finite radians in joint1..6 order");
      }
    }
    for (const auto * name : {"pick_lift", "place_approach", "place_retreat"}) {
      if (!std::isfinite(number(name)) || number(name) < .01 || number(name) > .20) {
        throw std::runtime_error(std::string(name) + " must be within .01..20 m");
      }
    }
    for (const auto * name : {"transfer_velocity_scaling", "transfer_acceleration_scaling",
        "vertical_velocity_scaling", "vertical_acceleration_scaling"}) {
      if (!std::isfinite(number(name)) || number(name) <= 0.0 || number(name) > .30) {
        throw std::runtime_error(std::string(name) + " must be within (0,.30]");
      }
    }
    const auto size = node_.get_parameter("carried_size").as_double_array();
    if (size.size() != 3 || !std::all_of(size.begin(), size.end(),
        [](double v) { return std::isfinite(v) && v > .005 && v <= .20; }) ||
        !std::isfinite(number("carried_padding")) || number("carried_padding") < 0.0 ||
        number("carried_padding") > .01) { throw std::runtime_error("invalid carried wood dimensions"); }
    touch_.clear();
    for (const auto & link : arm_->getRobotModel()->getLinkModelNames()) {
      if (link.find("ag145_gripper_") == 0) { touch_.push_back(link); }
    }
    touch_.push_back("tool0");
    initialized_ = true;
  }

  void requireEmptyScene()
  {
    initialize();
    if (!scene_->getAttachedObjects().empty()) {
      throw std::runtime_error("planning scene contains an attached object; recover it before another grasp");
    }
    if (node_.get_parameter("retain_placed_objects").as_bool() &&
        !scene_->getObjects({objectId()}).empty()) {
      throw std::runtime_error("previous placed wood still occupies the destination; first version runs one cycle");
    }
    const auto start = actual();
    (void)target(start, "home_joints"); (void)target(start, "place_joints");
  }

  // No hardware commands and no global planning-scene writes. This previews
  // the tail from the CURRENT tool pose, not the as-yet unknown vision grasp.
  void preview()
  {
    requireEmptyScene();
    auto start = actual();
    addVirtualWood(start);
    auto route = std::vector<Plan>{};
    auto at_release = start;
    auto place = target(start, "place_joints");
    place.setVariablePosition("joint6", start.getVariablePosition("joint6"));
    place.update();
    const bool already_at_release =
      (pose(start).translation() - pose(place).translation()).norm() <= .015 &&
      Eigen::Quaterniond(pose(start).rotation()).angularDistance(
      Eigen::Quaterniond(pose(place).rotation())) <= .10;
    if (!already_at_release) {
      route = routeToRelease(start);
      at_release = endState(start, route.back());
    }
    // The gripper opens at the taught release pose. Preview the return with an
    // empty gripper, matching execution; carrying the wood back to Home was a
    // false constraint and could reject an otherwise valid return path.
    at_release.clearAttachedBody(objectId());
    at_release.update();
    auto retreat = cartesian(at_release, raised(pose(at_release), number("place_retreat")));
    auto after_retreat = endState(at_release, retreat);
    route.push_back(retreat);
    route.push_back(jointPlan(after_retreat, target(after_retreat, "home_joints"),
      std::min(pose(after_retreat).translation().z(),
      pose(target(after_retreat, "home_joints")).translation().z()) - .003,
      "preview retreat-to-Home", false));
    publish(route);
  }

  // Called only after AG145 has reported HOLDING. Failures deliberately leave
  // the attached object in the scene, so a failed transfer cannot be restarted
  // by blindly opening the gripper. No automatic recovery motion is sent.
  void carryAndPlace(const std::function<void()> & open,
                     const std::function<void()> & close)
  {
    auto start = actual();
    addVirtualWood(start);
    moveit_msgs::msg::RobotState msg;
    moveit::core::robotStateToRobotStateMsg(start, msg);
    auto it = std::find_if(msg.attached_collision_objects.begin(), msg.attached_collision_objects.end(),
      [](const auto & object) { return object.object.id == objectId(); });
    if (it == msg.attached_collision_objects.end() || !scene_->applyAttachedCollisionObject(*it)) {
      throw std::runtime_error("cannot register carried wood; no lift sent");
    }
    attached_ = true;
    const auto route = routeToRelease(start);  // All carry segments must plan before lifting.
    publish(route);
    for (size_t i = 0; i < route.size(); ++i) {
      execute(route[i], i == 0 ? "LIFT_80MM" : (i == 1 ? "TRANSFER_ABOVE_PLACE" : "LOWER_TO_PLACE"));
    }
    auto release_state = actual();
    auto expected = target(start, "place_joints");
    expected.setVariablePosition("joint6", start.getVariablePosition("joint6")); expected.update();
    verifyState(release_state, expected, .01);
    RCLCPP_WARN(node_.get_logger(), "PICK PLACE: validated taught release pose; opening AG145");
    open();
    // In continuous mode the released box is not retained in the scene;
    // the operator must keep the physical drop surface clear before each cycle.
    releaseWood(release_state);
    auto after_open = actual();
    execute(cartesian(after_open, raised(pose(after_open), number("place_retreat"))), "RETREAT_80MM");
    RCLCPP_WARN(node_.get_logger(), "PICK PLACE: closing empty AG145 after retreat, before Home");
    close();
    // The gripper no longer touches the placed wood after the retreat.
    allowReleasedTouch(false);
    auto after_retreat = actual();
    const auto home = target(after_retreat, "home_joints");
    execute(jointPlan(after_retreat, home,
      std::min(pose(after_retreat).translation().z(), pose(home).translation().z()) - .003,
      "retreat-to-Home", false), "HOME");
  }

private:
  static std::string objectId() { return "thor7_cycle_wood"; }
  double number(const char * name) const { return node_.get_parameter(name).as_double(); }
  Eigen::Isometry3d pose(const State & state) const { return refreshedToolPose(state, "base_link", "tool0"); }
  static Eigen::Isometry3d raised(Eigen::Isometry3d value, double z) { value.translation().z() += z; return value; }
  static geometry_msgs::msg::Pose poseMsg(const Eigen::Isometry3d & value)
  {
    geometry_msgs::msg::Pose out;
    out.position.x = value.translation().x(); out.position.y = value.translation().y(); out.position.z = value.translation().z();
    const Eigen::Quaterniond q(value.rotation());
    out.orientation.x = q.x(); out.orientation.y = q.y(); out.orientation.z = q.z(); out.orientation.w = q.w();
    return out;
  }
  State actual()
  {
    auto current = arm_->getCurrentState(2.0);
    if (!current) { throw std::runtime_error("robot state unavailable"); }
    State out(*current);
    std::lock_guard<std::mutex> lock(mutex_);
    for (const auto & joint : arm_->getRobotModel()->getActiveJointModels()) {
      if (joint->getVariableCount() != 1) { continue; }
      const auto & name = joint->getVariableNames().front();
      const auto it = feedback_.find(name);
      if (it == feedback_.end() || std::chrono::steady_clock::now() - it->second.second > std::chrono::seconds(1)) {
        throw std::runtime_error("missing or stale joint feedback: " + name);
      }
      out.setVariablePosition(name, it->second.first);
    }
    out.clearAttachedBody(objectId());
    if (attached_) { addVirtualWood(out); }
    out.update();
    if (!out.satisfiesBounds()) { throw std::runtime_error("actual joint feedback outside model limits"); }
    return out;
  }
  State target(const State & seed, const char * parameter) const
  {
    State out(seed);
    const auto values = node_.get_parameter(parameter).as_double_array();
    for (size_t i = 0; i < 6; ++i) { out.setVariablePosition("joint" + std::to_string(i + 1), values.at(i)); }
    out.update();
    if (!out.satisfiesBounds()) { throw std::runtime_error(std::string(parameter) + " outside model bounds"); }
    return out;
  }
  void addVirtualWood(State & state)
  {
    state.clearAttachedBody(objectId());
    const auto size = node_.get_parameter("carried_size").as_double_array();
    const double pad = number("carried_padding");
    Eigen::Isometry3d offset = Eigen::Isometry3d::Identity();
    offset.translation().z() = number("near_clearance") + size[2] / 2.0;
    state.attachBody(objectId(), offset,
      {shapes::ShapeConstPtr(new shapes::Box(size[0]+2*pad, size[1]+2*pad, size[2]+2*pad))},
      {Eigen::Isometry3d::Identity()}, touch_, "tool0");
    state.update();
  }
  State endState(const State & start, const Plan & plan) const
  {
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "thor7_arm");
    trajectory.setRobotTrajectoryMsg(start, plan.trajectory_);
    if (trajectory.empty()) { throw std::runtime_error("empty trajectory"); }
    State out(trajectory.getLastWayPoint()); out.update(); return out;
  }
  void verifyState(const State & actual_state, const State & goal, double tolerance) const
  {
    for (int i = 1; i <= 6; ++i) {
      const auto name = "joint" + std::to_string(i);
      // Do NOT wrap: the taught J6=190 degrees must retain its winding.
      if (std::abs(actual_state.getVariablePosition(name) - goal.getVariablePosition(name)) > tolerance) {
        throw std::runtime_error("joint endpoint/start mismatch: " + name);
      }
    }
    if ((pose(actual_state).translation() - pose(goal).translation()).norm() > .005 ||
        Eigen::Quaterniond(pose(actual_state).rotation()).angularDistance(
        Eigen::Quaterniond(pose(goal).rotation())) > .05236) {
      throw std::runtime_error("tool0 feedback position/orientation mismatch");
    }
  }
  void verifyPath(const State & start, const Plan & plan, double floor, bool vertical,
                  bool require_downward = true) const
  {
    robot_trajectory::RobotTrajectory trajectory(arm_->getRobotModel(), "thor7_arm");
    trajectory.setRobotTrajectoryMsg(start, plan.trajectory_);
    if (trajectory.empty()) { throw std::runtime_error("empty planned path"); }
    const auto origin = pose(start);
    for (size_t i = 0; i < trajectory.getWayPointCount(); ++i) {
      const auto & a = i ? trajectory.getWayPoint(i-1) : start;
      const auto & b = trajectory.getWayPoint(i);
      double delta = 0.0;
      for (int j = 1; j <= 6; ++j) {
        const auto name = "joint" + std::to_string(j);
        delta = std::max(delta, std::abs(a.getVariablePosition(name)-b.getVariablePosition(name)));
      }
      if (vertical && delta > .20) { throw std::runtime_error("Cartesian joint jump > .20 rad"); }
      const int count = std::max(1, static_cast<int>(std::ceil(delta / .02)));
      for (int k = 0; k <= count; ++k) {
        State sample(a); a.interpolate(b, static_cast<double>(k)/count, sample); sample.update();
        const auto p = pose(sample);
        if (!p.matrix().allFinite() || p.translation().z() < floor ||
            (require_downward &&
            p.linear().col(2).dot(-Eigen::Vector3d::UnitZ()) < std::cos(.05236))) {
          throw std::runtime_error("carry path violates height/downward orientation");
        }
        if (vertical && ((p.translation()-origin.translation()).head<2>().norm() > .003 ||
            Eigen::Quaterniond(p.rotation()).angularDistance(Eigen::Quaterniond(origin.rotation())) > .05236)) {
          throw std::runtime_error("vertical path drifts in XY/orientation");
        }
      }
    }
  }
  Plan cartesian(const State & start, const Eigen::Isometry3d & goal,
                 double velocity = -1.0, double acceleration = -1.0,
                 bool vertical = true)
  {
    arm_->clearPathConstraints(); arm_->setStartState(start);
    Plan plan;
    const double fraction = arm_->computeCartesianPath({poseMsg(goal)}, .002, 0.0, plan.trajectory_, true);
    if (!std::isfinite(fraction) || fraction < 1.0 - 1e-6) {
      throw std::runtime_error("vertical collision-aware path is incomplete: " + std::to_string(fraction));
    }
    verifyPath(start, plan, std::min(pose(start).translation().z(), goal.translation().z())-.003, vertical);
    const auto final = pose(endState(start, plan));
    if ((final.translation()-goal.translation()).norm() > .003 ||
        Eigen::Quaterniond(final.rotation()).angularDistance(Eigen::Quaterniond(goal.rotation())) > .05236) {
      throw std::runtime_error("Cartesian FK endpoint mismatch");
    }
    robot_trajectory::RobotTrajectory timed(arm_->getRobotModel(), "thor7_arm");
    timed.setRobotTrajectoryMsg(start, plan.trajectory_);
    trajectory_processing::IterativeParabolicTimeParameterization timing;
    const double v = velocity > 0.0 ? velocity : number("vertical_velocity_scaling");
    const double a = acceleration > 0.0 ? acceleration : number("vertical_acceleration_scaling");
    if (!timing.computeTimeStamps(timed, v, a)) {
      throw std::runtime_error("vertical time parameterization failed");
    }
    timed.getRobotTrajectoryMsg(plan.trajectory_);
    moveit::core::robotStateToRobotStateMsg(start, plan.start_state_);
    return plan;
  }
  Plan jointPlan(
    const State & start, const State & goal, double floor, const std::string & stage,
    bool require_downward = true)
  {
    arm_->setStartState(start); arm_->setJointValueTarget(goal);
    arm_->setMaxVelocityScalingFactor(number("transfer_velocity_scaling"));
    arm_->setMaxAccelerationScalingFactor(number("transfer_acceleration_scaling"));
    moveit_msgs::msg::Constraints constraints;
    moveit_msgs::msg::PositionConstraint height;
    height.header.frame_id = "base_link"; height.link_name = "tool0"; height.weight = 1.0;
    shape_msgs::msg::SolidPrimitive box; box.type = box.BOX; box.dimensions = {10., 10., 10.};
    geometry_msgs::msg::Pose box_pose; box_pose.orientation.w = 1.; box_pose.position.z = floor+5.;
    height.constraint_region.primitives.push_back(box); height.constraint_region.primitive_poses.push_back(box_pose);
    constraints.position_constraints.push_back(height);
    if (require_downward) {
      moveit_msgs::msg::OrientationConstraint down;
      down.header.frame_id = "base_link"; down.link_name = "tool0";
      // geometry_msgs/Quaternion defaults w to 1. Setting only x produced the
      // invalid [x,y,z,w]=[1,0,0,1] constraint seen in MoveIt. Set all four.
      down.orientation.x = 1.; down.orientation.y = 0.;
      down.orientation.z = 0.; down.orientation.w = 0.;
      down.parameterization = down.ROTATION_VECTOR;
      down.absolute_x_axis_tolerance = .05236; down.absolute_y_axis_tolerance = .05236;
      down.absolute_z_axis_tolerance = 2.0*std::acos(-1.0); down.weight = 1.;
      constraints.orientation_constraints.push_back(down);
    }
    arm_->setPathConstraints(constraints);
    Plan plan;
    const auto result = arm_->plan(plan);
    arm_->clearPathConstraints();
    if (result != moveit::core::MoveItErrorCode::SUCCESS) {
      throw std::runtime_error(stage + " planning failed; MoveIt code=" +
        std::to_string(result.val));
    }
    verifyPath(start, plan, floor-.0001, false, require_downward);
    verifyState(endState(start, plan), goal, .003);
    return plan;
  }
  std::vector<Plan> routeToRelease(const State & start)
  {
    auto place = target(start, "place_joints");
    place.setVariablePosition("joint6", start.getVariablePosition("joint6")); place.update();
    const auto taught = pose(target(start, "place_joints"));
    if ((pose(place).translation()-taught.translation()).norm() > .001 ||
        (pose(place).linear().col(2)-taught.linear().col(2)).norm() > .001) {
      throw std::runtime_error("J6 changes tool0 position/approach axis in this model; cannot ignore taught J6");
    }
    const auto above_pose = raised(pose(place), number("place_approach"));
    State above(place);
    const auto * group = arm_->getRobotModel()->getJointModelGroup("thor7_arm");
    if (!above.setFromIK(group, above_pose, "tool0", 1.0)) { throw std::runtime_error("no IK above place"); }
    above.update();
    std::vector<Plan> route;
    route.push_back(cartesian(start, raised(pose(start), number("pick_lift"))));
    const auto lifted = endState(start, route.back());
    // The transfer keeps tool0 downward while translating horizontally. OMPL
    // was unnecessarily sampling a six-joint manifold here and frequently
    // timed out even though a straight collision-checked Cartesian path exists.
    route.push_back(cartesian(lifted, above_pose,
      number("transfer_velocity_scaling"), number("transfer_acceleration_scaling"), false));
    const auto at_above = endState(lifted, route.back());
    route.push_back(cartesian(at_above, pose(place)));
    verifyState(endState(at_above, route.back()), place, .01);
    return route;
  }
  void execute(const Plan & plan, const char * stage)
  {
    auto current = actual(); State expected(current);
    moveit::core::robotStateMsgToRobotState(plan.start_state_, expected);
    expected.update(); verifyState(current, expected, .02);
    // Republish each segment immediately before execution. The complete
    // route is published during planning, but RViz may replace that display
    // when another node publishes a trajectory or when the robot state
    // refreshes. This keeps the segment currently being sent visible,
    // especially the final retreat-to-Home path.
    moveit_msgs::msg::DisplayTrajectory display;
    display.model_id = arm_->getRobotModel()->getName();
    display.trajectory_start = plan.start_state_;
    display.trajectory.push_back(plan.trajectory_);
    display_->publish(display);
    RCLCPP_WARN(node_.get_logger(), "PICK PLACE stage: %s", stage);
    if (arm_->execute(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
      arm_->stop(); throw std::runtime_error(std::string(stage)+" execution failed; remaining stages cancelled");
    }
    const auto goal = endState(expected, plan);
    // Allow joint feedback to catch the action completion, but never plan from
    // commanded states or missing feedback.
    for (int i = 0; ; ++i) {
      try { verifyState(actual(), goal, .01); break; }
      catch (const std::exception &) { if (i >= 10) { throw; } }
      std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
  }
  void releaseWood(State & state)
  {
    const auto * body = state.getAttachedBody(objectId());
    if (!body) { throw std::runtime_error("attached wood geometry missing at release"); }
    moveit_msgs::msg::PlanningScene diff; diff.is_diff = true; diff.robot_state.is_diff = true;
    moveit_msgs::msg::AttachedCollisionObject remove;
    remove.link_name = "tool0"; remove.object.id = objectId(); remove.object.operation = remove.object.REMOVE;
    diff.robot_state.attached_collision_objects.push_back(remove);
    moveit_msgs::msg::CollisionObject world;
    world.id = objectId(); world.header.frame_id = arm_->getRobotModel()->getModelFrame();
    world.operation = world.ADD;
    shape_msgs::msg::SolidPrimitive box; box.type = box.BOX;
    const auto size = node_.get_parameter("carried_size").as_double_array();
    for (double value : size) { box.dimensions.push_back(value+2*number("carried_padding")); }
    world.primitives.push_back(box);
    world.primitive_poses.push_back(poseMsg(body->getGlobalCollisionBodyTransforms().at(0)));
    if (node_.get_parameter("retain_placed_objects").as_bool()) {
      diff.world.collision_objects.push_back(world);
    }
    if (!scene_->applyPlanningScene(diff)) { throw std::runtime_error("release scene update failed; no retreat sent"); }
    attached_ = false;
    allowReleasedTouch(true);
  }
  void allowReleasedTouch(bool allow)
  {
    // Fetch the existing ACM; never replace unrelated collision allowances.
    auto client = node_.create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!client->wait_for_service(std::chrono::seconds(5))) { throw std::runtime_error("planning scene service unavailable"); }
    auto req = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    req->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
    auto future = client->async_send_request(req);
    if (future.wait_for(std::chrono::seconds(5)) != std::future_status::ready) {
      throw std::runtime_error("planning scene ACM read timed out");
    }
    collision_detection::AllowedCollisionMatrix acm(future.get()->scene.allowed_collision_matrix);
    for (const auto & link : touch_) { acm.setEntry(objectId(), link, allow); }
    moveit_msgs::msg::PlanningScene diff; diff.is_diff = true; diff.robot_state.is_diff = true;
    acm.getMessage(diff.allowed_collision_matrix);
    if (!scene_->applyPlanningScene(diff)) { throw std::runtime_error("released wood collision update failed"); }
  }
  void publish(const std::vector<Plan> & plans)
  {
    moveit_msgs::msg::DisplayTrajectory msg;
    msg.model_id = arm_->getRobotModel()->getName(); msg.trajectory_start = plans.front().start_state_;
    for (const auto & plan : plans) { msg.trajectory.push_back(plan.trajectory_); }
    display_->publish(msg);
  }
  rclcpp::Node & node_;
  std::unique_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
  std::unique_ptr<moveit::planning_interface::PlanningSceneInterface> scene_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joints_;
  rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr display_;
  std::mutex mutex_;
  std::map<std::string, std::pair<double, std::chrono::steady_clock::time_point>> feedback_;
  std::vector<std::string> touch_;
  bool attached_{false};
  bool initialized_{false};
};
}
