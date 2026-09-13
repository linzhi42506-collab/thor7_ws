#include <gtest/gtest.h>

#include "thor7_hardware/ag145_goal_state.hpp"

using thor7_hardware::Ag145GoalProgress;
using thor7_hardware::classifyAg145GoalProgress;

TEST(Ag145GoalState, OpeningIgnoresPreviousDroppedState)
{
  EXPECT_EQ(
    classifyAg145GoalProgress(0, 1000, 0, 3, 15, false),
    Ag145GoalProgress::kContinue);
  EXPECT_EQ(
    classifyAg145GoalProgress(0, 1000, 500, 3, 15, true),
    Ag145GoalProgress::kContinue);
  EXPECT_EQ(
    classifyAg145GoalProgress(0, 1000, 995, 1, 15, true),
    Ag145GoalProgress::kReached);
}

TEST(Ag145GoalState, OpeningIgnoresPreviousHoldingState)
{
  EXPECT_EQ(
    classifyAg145GoalProgress(0, 1000, 0, 2, 15, false),
    Ag145GoalProgress::kContinue);
}

TEST(Ag145GoalState, ClosingRequiresMotionBeforeDroppedIsFinal)
{
  EXPECT_EQ(
    classifyAg145GoalProgress(1000, 0, 1000, 3, 15, false),
    Ag145GoalProgress::kContinue);
  EXPECT_EQ(
    classifyAg145GoalProgress(1000, 0, 700, 3, 15, true),
    Ag145GoalProgress::kDropped);
}

TEST(Ag145GoalState, ClosingReportsHoldingAndNormalCompletion)
{
  EXPECT_EQ(
    classifyAg145GoalProgress(1000, 0, 450, 2, 15, true),
    Ag145GoalProgress::kHolding);
  EXPECT_EQ(
    classifyAg145GoalProgress(1000, 0, 8, 1, 15, true),
    Ag145GoalProgress::kReached);
}

TEST(Ag145GoalState, DeviceErrorAlwaysWins)
{
  EXPECT_EQ(
    classifyAg145GoalProgress(0, 1000, 1000, 4, 15, true),
    Ag145GoalProgress::kError);
}
