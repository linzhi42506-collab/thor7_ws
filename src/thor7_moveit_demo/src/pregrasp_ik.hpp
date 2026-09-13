#pragma once
#include "grasp_geometry.hpp"
#include <moveit/robot_state/robot_state.h>
#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <vector>

namespace thor7_grasp
{
struct PregraspIkCandidate
{
  std::vector<double> joints;
  double yaw;
  double cost;
  double max_delta;
};

// Choose a legally bounded winding nearest the measured joint value. Never
// clamp an IK solution: clamping would change the requested Cartesian pose.
inline double nearestLegalWinding(double value, double seed, double lower, double upper)
{
  if (!std::isfinite(value) || !std::isfinite(seed) ||
      !std::isfinite(lower) || !std::isfinite(upper) || lower > upper) {
    return std::numeric_limits<double>::quiet_NaN();
  }
  const double turn = 2.0 * std::acos(-1.0);
  const double first = std::ceil((lower - value) / turn);
  const double last = std::floor((upper - value) / turn);
  if (first > last) { return std::numeric_limits<double>::quiet_NaN(); }
  const double winding = std::max(first, std::min(last, std::round((seed - value) / turn)));
  return value + turn * winding;
}

// window > 0: local consistency search. window == 0: multiple seeds within
// actual model bounds. The returned list includes both physical finger-axis
// equivalents; callers must still plan/check collisions for each candidate.
inline std::vector<PregraspIkCandidate> pregraspIkCandidates(
  const moveit::core::RobotState & start, const Eigen::Vector3d & position,
  double yaw, const std::string & base, const std::string & tip, double window,
  double position_tolerance, double angle_tolerance)
{
  const auto * group = start.getRobotModel()->getJointModelGroup("thor7_arm");
  if (!group || !group->getSolverInstance()) {
    throw std::runtime_error("thor7_arm IK solver unavailable");
  }
  std::vector<double> seed;
  start.copyJointGroupPositions(group, seed);
  if (!start.satisfiesBounds(group)) {
    throw std::runtime_error("current arm joints outside model bounds; check feedback and joint limits");
  }
  auto reference = start;
  reference.update();
  const auto model_to_base = reference.getGlobalLinkTransform(base);
  const auto & names = group->getVariableNames();
  std::vector<PregraspIkCandidate> result;
  for (const double candidate_yaw : {yaw, yaw + std::acos(-1.0)}) {
    Eigen::Isometry3d goal = Eigen::Isometry3d::Identity();
    goal.translation() = position;
    goal.linear() = downward(candidate_yaw).toRotationMatrix();
    const Eigen::Isometry3d model_goal = model_to_base * goal;
    std::vector<PregraspIkCandidate> per_yaw;
    for (int attempt = 0; attempt < (window > 0.0 ? 8 : 24); ++attempt) {
      auto state = reference;
      if (window == 0.0 && attempt > 0) {
        state.setToRandomPositions(group);
      }
      const std::vector<double> consistency = window > 0.0 ?
        std::vector<double>(seed.size(), window) : std::vector<double>{};
      if (!state.setFromIK(group, model_goal, tip, consistency, 0.05)) { continue; }
      std::vector<double> values;
      state.copyJointGroupPositions(group, values);
      if (values.size() != seed.size()) { continue; }
      double cost = 0.0, max_delta = 0.0;
      bool valid = true;
      for (std::size_t i = 0; i < values.size(); ++i) {
        const auto & bounds = start.getRobotModel()->getVariableBounds(names[i]);
        const auto * joint = start.getRobotModel()->getJointOfVariable(names[i]);
        if (joint->getType() == moveit::core::JointModel::REVOLUTE && bounds.position_bounded_) {
          values[i] = nearestLegalWinding(values[i], seed[i],
            bounds.min_position_, bounds.max_position_);
        }
        if (!std::isfinite(values[i])) { valid = false; break; }
        const double delta = values[i] - seed[i];
        max_delta = std::max(max_delta, std::abs(delta));
        cost += delta * delta;
      }
      if (!valid || (window > 0.0 && max_delta > window + 1e-6)) { continue; }
      state.setJointGroupPositions(group, values);
      state.update();
      const Eigen::Isometry3d actual = model_to_base.inverse() * state.getGlobalLinkTransform(tip);
      if (!state.satisfiesBounds(group) ||
          !endpointWithin(actual, position, candidate_yaw, position_tolerance, angle_tolerance)) {
        continue;
      }
      const bool duplicate = std::any_of(per_yaw.begin(), per_yaw.end(), [&](const auto & other) {
        double largest = 0.0;
        for (std::size_t i = 0; i < values.size(); ++i) {
          largest = std::max(largest, std::abs(values[i] - other.joints[i]));
        }
        return largest < 0.01;
      });
      if (!duplicate) { per_yaw.push_back({values, candidate_yaw, cost, max_delta}); }
    }
    std::sort(per_yaw.begin(), per_yaw.end(), [](const auto & a, const auto & b) { return a.cost < b.cost; });
    // Reserve candidates for each yaw instead of letting one yaw occupy all slots.
    if (per_yaw.size() > 2) { per_yaw.resize(2); }
    result.insert(result.end(), per_yaw.begin(), per_yaw.end());
  }
  std::sort(result.begin(), result.end(), [](const auto & a, const auto & b) { return a.cost < b.cost; });
  return result;
}
}  // namespace thor7_grasp
