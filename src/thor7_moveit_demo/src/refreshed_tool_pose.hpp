#pragma once
#include <moveit/robot_state/robot_state.h>

namespace thor7_grasp
{
inline Eigen::Isometry3d refreshedToolPose(const moveit::core::RobotState & state,
                                         const std::string & base, const std::string & tool)
{
  moveit::core::RobotState updated(state);
  updated.update();
  return updated.getGlobalLinkTransform(base).inverse() * updated.getGlobalLinkTransform(tool);
}
}
