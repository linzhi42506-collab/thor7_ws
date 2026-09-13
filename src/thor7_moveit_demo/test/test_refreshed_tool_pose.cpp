#include "refreshed_tool_pose.hpp"
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <iostream>

int main()
{
  const auto urdf = urdf::parseURDF(R"(
    <robot name="fk_test">
      <link name="world"/><link name="base"/><link name="tool"/>
      <joint name="mount" type="fixed"><parent link="world"/><child link="base"/>
        <origin xyz="0 0 1"/></joint>
      <joint name="height" type="prismatic"><parent link="base"/><child link="tool"/>
        <axis xyz="0 0 1"/><limit lower="0" upper="1" velocity="1" effort="1"/></joint>
    </robot>)");
  auto srdf = std::make_shared<srdf::Model>();
  if (!urdf || !srdf->initString(*urdf, "<robot name='fk_test'/>") ) { return 1; }
  auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
  moveit::core::RobotState state(model);
  state.setToDefaultValues(); state.update();
  // Reproduce CurrentStateMonitor's updated joint values with stale FK caches.
  state.setVariablePosition("height", .373);
  const auto actual = thor7_grasp::refreshedToolPose(state, "base", "tool");
  if (std::abs(actual.translation().z() - .373) > 1e-12) { return 2; }
  if (actual.translation().z() < .093) { return 3; }
  // A genuinely low state must still trigger the height guard.
  state.setVariablePosition("height", .02);
  const auto low = thor7_grasp::refreshedToolPose(state, "base", "tool");
  if (!(low.translation().z() < .093)) { return 4; }
  std::cout << "Dirty FK cache refreshed; nonidentity base and true low pose PASS\n";
}
