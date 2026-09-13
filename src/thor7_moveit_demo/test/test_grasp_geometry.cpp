#include "grasp_geometry.hpp"
#include <limits>
#include <iostream>

int main()
{
  auto require = [](bool pass) { if (!pass) { throw std::runtime_error("grasp geometry regression"); } };
  for (double yaw : {0.0, 0.5, -1.2, 3.141592653589793}) {
    const auto q = thor7_grasp::downward(yaw);
    require((q * Eigen::Vector3d::UnitZ() + Eigen::Vector3d::UnitZ()).norm() < 1e-12);
    require((q * Eigen::Vector3d::UnitX() -
      Eigen::Vector3d(std::cos(yaw), std::sin(yaw), 0)).norm() < 1e-12);
  }
  Eigen::Isometry3d endpoint = Eigen::Isometry3d::Identity();
  const Eigen::Vector3d target(-0.289, 0.418, 0.094);
  endpoint.translation() = target;
  endpoint.linear() = thor7_grasp::downward(0).toRotationMatrix();
  require(thor7_grasp::endpointWithin(endpoint, target, 0, .003, .05236));
  // Correct position with the previously observed ~42-degree tilt must fail.
  endpoint.linear() *= Eigen::AngleAxisd(.733, Eigen::Vector3d::UnitY()).toRotationMatrix();
  require(!thor7_grasp::endpointWithin(endpoint, target, 0, .003, .05236));
  endpoint.linear() = thor7_grasp::downward(0).toRotationMatrix();
  endpoint.translation().x() += .004;
  require(!thor7_grasp::endpointWithin(endpoint, target, 0, .003, .05236));
  endpoint.translation().x() = std::numeric_limits<double>::quiet_NaN();
  require(!thor7_grasp::endpointWithin(endpoint, target, 0, .003, .05236));
  bool rejected = false;
  try { thor7_grasp::downward(std::numeric_limits<double>::quiet_NaN()); }
  catch (const std::invalid_argument &) { rejected = true; }
  require(rejected);
  std::cout << "Downward axes, yaw, tilt rejection, endpoint offset and NaN checks passed\n";
}
