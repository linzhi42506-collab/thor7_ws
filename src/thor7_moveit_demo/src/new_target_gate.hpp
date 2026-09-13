#pragma once
#include <Eigen/Core>
#include <cmath>

namespace thor7_grasp
{
// Require three fresh, mutually consistent detections far from the previous
// pick. This is not object identity tracking: identical blocks at the same
// coordinate cannot be distinguished without another sensor/event.
class NewTargetGate
{
public:
  explicit NewTargetGate(const Eigen::Vector3d & previous, double min_displacement,
                         double consistency = .010)
  : previous_(previous), min_displacement_(min_displacement), consistency_(consistency) {}

  bool observe(const Eigen::Vector3d & point)
  {
    if (!point.allFinite() || (point - previous_).norm() < min_displacement_) {
      count_ = 0;
      return false;
    }
    if (count_ && (point - candidate_).norm() <= consistency_) {
      ++count_;
    } else {
      count_ = 1;
    }
    candidate_ = point;
    return count_ >= 3;
  }

private:
  Eigen::Vector3d previous_;
  Eigen::Vector3d candidate_{Eigen::Vector3d::Zero()};
  double min_displacement_, consistency_;
  int count_{0};
};
}
