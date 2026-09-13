#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <chrono>
#include <cmath>

namespace thor7_grasp
{
// Caller serializes access. Movement must be confirmed by consecutive,
// mutually consistent observations so one partially occluded pose cannot
// permanently revoke an otherwise valid attempt.
class TargetLock
{
public:
  using Clock = std::chrono::steady_clock;
  void reset()
  {
    pending_ = ready_ = moved_ = false;
    committed_ = false;
    have_motion_candidate_ = false;
    motion_candidate_count_ = 0;
  }
  void begin(const Eigen::Vector3d & point, double yaw)
  {
    reset(); point_ = point; yaw_ = yaw; pending_ = true;
  }
  // Called only after the final fresh-target/start-state checks, immediately
  // before dispatching pregrasp. Subsequent observations are likely corrupted
  // by the approaching gripper and cannot safely revoke this cycle's snapshot.
  bool commit()
  {
    if (!pending_ || moved_) { return false; }
    committed_ = true;
    have_motion_candidate_ = false;
    motion_candidate_count_ = 0;
    return true;
  }
  void observe(const Eigen::Vector3d & point, double yaw,
               double position_tolerance, double yaw_tolerance,
               int required_confirmations = 3)
  {
    if (!(pending_ || ready_) || moved_ || committed_) { return; }
    const double yaw_error = std::abs(std::remainder(yaw - yaw_, std::acos(-1.0)));
    const bool outside_lock =
      (point - point_).norm() > position_tolerance || yaw_error > yaw_tolerance;
    if (!outside_lock) {
      have_motion_candidate_ = false;
      motion_candidate_count_ = 0;
      return;
    }

    const double candidate_yaw_error = std::abs(
      std::remainder(yaw - motion_candidate_yaw_, std::acos(-1.0)));
    const bool agrees_with_candidate = have_motion_candidate_ &&
      (point - motion_candidate_).norm() <= position_tolerance &&
      candidate_yaw_error <= yaw_tolerance;
    if (agrees_with_candidate) {
      ++motion_candidate_count_;
    } else {
      motion_candidate_ = point;
      motion_candidate_yaw_ = yaw;
      have_motion_candidate_ = true;
      motion_candidate_count_ = 1;
    }
    if (motion_candidate_count_ >= std::max(1, required_confirmations)) {
      moved_ = true;
    }
  }
  bool arrive(Clock::time_point now)
  {
    if (!pending_ || moved_ || !committed_) { return false; }
    pending_ = false; ready_ = true; arrived_ = now; return true;
  }
  bool valid(Clock::time_point now, double seconds) const
  {
    const double age = std::chrono::duration<double>(now - arrived_).count();
    return ready_ && !moved_ && seconds > 0 && age >= 0 && age <= seconds;
  }
  bool moved() const { return moved_; }
private:
  Eigen::Vector3d point_{Eigen::Vector3d::Zero()};
  Eigen::Vector3d motion_candidate_{Eigen::Vector3d::Zero()};
  double yaw_{0.0};
  double motion_candidate_yaw_{0.0};
  Clock::time_point arrived_{};
  bool pending_{false}, ready_{false}, moved_{false}, committed_{false};
  bool have_motion_candidate_{false};
  int motion_candidate_count_{0};
};
}
