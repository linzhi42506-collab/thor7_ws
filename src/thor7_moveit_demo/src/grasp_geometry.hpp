#pragma once
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>

namespace thor7_grasp
{
inline Eigen::Quaterniond downward(double yaw)
{
  if (!std::isfinite(yaw)) { throw std::invalid_argument("grasp yaw must be finite"); }
  return Eigen::Quaterniond(
    Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(std::acos(-1.0), Eigen::Vector3d::UnitX()));
}

inline bool endpointWithin(const Eigen::Isometry3d & actual,
                           const Eigen::Vector3d & position, double yaw,
                           double position_tolerance, double angle_tolerance)
{
  return actual.matrix().allFinite() && position.allFinite() &&
    (actual.translation() - position).norm() <= position_tolerance &&
    Eigen::Quaterniond(actual.rotation()).angularDistance(downward(yaw)) <= angle_tolerance;
}
}  // namespace thor7_grasp
