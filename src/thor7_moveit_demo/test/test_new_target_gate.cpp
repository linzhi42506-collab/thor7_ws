#include "new_target_gate.hpp"
#include <iostream>

int main()
{
  const Eigen::Vector3d previous(-.35, .48, .03);
  thor7_grasp::NewTargetGate gate(previous, .05);
  if (gate.observe(previous + Eigen::Vector3d(.049, 0, 0))) { return 1; }
  if (gate.observe(previous + Eigen::Vector3d(.06, 0, 0))) { return 2; }
  if (gate.observe(previous + Eigen::Vector3d(.1, 0, 0))) { return 3; }
  if (gate.observe(previous + Eigen::Vector3d(.05, 0, 0))) { return 4; }
  if (gate.observe(previous + Eigen::Vector3d(.002, 0, 0))) { return 5; }
  if (gate.observe(previous + Eigen::Vector3d(.061, .002, 0))) { return 6; }
  if (gate.observe(previous + Eigen::Vector3d(.060, .001, 0))) { return 7; }
  if (!gate.observe(previous + Eigen::Vector3d(.059, .002, 0))) { return 8; }
  std::cout << "New target needs 50 mm displacement and three consistent observations\n";
}
