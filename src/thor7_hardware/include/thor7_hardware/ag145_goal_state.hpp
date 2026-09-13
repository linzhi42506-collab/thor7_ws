#pragma once

#include <cstdlib>

namespace thor7_hardware
{

enum class Ag145GoalProgress
{
  kContinue,
  kReached,
  kHolding,
  kDropped,
  kError,
};

inline Ag145GoalProgress classifyAg145GoalProgress(
  int start_position,
  int target_position,
  int current_position,
  unsigned int status,
  int tolerance_counts,
  bool seen_running)
{
  if (status == 4U) {
    return Ag145GoalProgress::kError;
  }

  const bool opening = target_position > start_position + tolerance_counts;
  const int error = std::abs(current_position - target_position);

  // HOLDING and DROPPED describe object state, so they must not abort an
  // opening command. The device can also briefly retain DROPPED from the
  // previous command until the new motion changes status to RUNNING.
  if (!opening && status == 2U) {
    return Ag145GoalProgress::kHolding;
  }
  if (!opening && status == 3U && seen_running) {
    return Ag145GoalProgress::kDropped;
  }
  if (error <= tolerance_counts && status != 0U) {
    return Ag145GoalProgress::kReached;
  }
  return Ag145GoalProgress::kContinue;
}

}  // namespace thor7_hardware
