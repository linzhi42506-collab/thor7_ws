#include "pregrasp_ik.hpp"
#include <pluginlib/class_loader.hpp>
#include <urdf_parser/urdf_parser.h>
#include <srdfdom/model.h>
#include <fstream>
#include <iostream>
#include <iterator>

// In-process model/KDL test only: no controller clients or motion services.
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int result = 0;
  try {
    pluginlib::ClassLoader<kinematics::KinematicsBase> loader("moveit_core", "kinematics::KinematicsBase");
    const double pi = std::acos(-1.0);
    if (std::abs(thor7_grasp::nearestLegalWinding(-3.0, 3.3, -2*pi, 2*pi) - (2*pi-3.0)) > 1e-9 ||
        std::isfinite(thor7_grasp::nearestLegalWinding(3.0, 0.0, -2.0, 2.0))) {
      throw std::runtime_error("bounded winding regression");
    }
    const auto urdf = urdf::parseURDFFile(THOR7_TEST_URDF);
    std::ifstream stream(THOR7_TEST_SRDF);
    const std::string semantic((std::istreambuf_iterator<char>(stream)), std::istreambuf_iterator<char>());
    auto srdf = std::make_shared<srdf::Model>();
    if (!urdf || !srdf->initString(*urdf, semantic)) { throw std::runtime_error("model load failed"); }
    auto model = std::make_shared<moveit::core::RobotModel>(urdf, srdf);
    auto node = std::make_shared<rclcpp::Node>("pregrasp_ik_offline_test");
    auto solver = loader.createSharedInstance("kdl_kinematics_plugin/KDLKinematicsPlugin");
    if (!solver->initialize(node, *model, "thor7_arm", "base_link", {"tool0"}, .005)) {
      throw std::runtime_error("KDL init failed");
    }
    model->setKinematicsAllocators({{"thor7_arm", [solver](const moveit::core::JointModelGroup *) { return solver; }}});
    const auto * group = model->getJointModelGroup("thor7_arm");
    moveit::core::RobotState start(model);
    start.setToDefaultValues();
    // User's taught Home, not the SRDF's unrelated named home.
    start.setJointGroupPositions(group, std::vector<double>{
      2.0158478661, -.2619504546, 2.1521946480, .0000355782, 1.2515935750, 3.3187155759});
    start.update();
    const auto home = start.getGlobalLinkTransform("tool0");
    const double home_yaw = std::atan2(home(1, 0), home(0, 0));
    // Give the opposite yaw first. A tiny local window must still recover
    // the other equivalent yaw that is exactly the current pose.
    const auto alternate = thor7_grasp::pregraspIkCandidates(start, home.translation(), home_yaw + pi,
      "base_link", "tool0", .02, .003, .053);
    if (alternate.empty() || alternate.front().max_delta > .02 + 1e-6) {
      throw std::runtime_error("alternate yaw was not searched");
    }
    // The reported failed target is far outside a deliberately tiny local window.
    const Eigen::Vector3d position(-.2893, .3977, .0872);
    const auto local = thor7_grasp::pregraspIkCandidates(start, position, .897,
      "base_link", "tool0", .02, .003, .053);
    if (!local.empty()) { throw std::runtime_error("local consistency bounds not enforced"); }
    const auto broad = thor7_grasp::pregraspIkCandidates(start, position, .897,
      "base_link", "tool0", 0.0, .003, .053);
    if (broad.empty()) { throw std::runtime_error("expanded KDL search failed reported target"); }
    bool first_yaw = false, second_yaw = false;
    for (const auto & candidate : broad) {
      auto state = start;
      state.setJointGroupPositions(group, candidate.joints);
      state.update();
      if (!state.satisfiesBounds(group) || !thor7_grasp::endpointWithin(
          state.getGlobalLinkTransform("tool0"), position, candidate.yaw, .003, .053)) {
        throw std::runtime_error("candidate violates bounds or FK");
      }
      first_yaw |= std::abs(candidate.yaw - .897) < 1e-6;
      second_yaw |= std::abs(candidate.yaw - (.897+pi)) < 1e-6;
    }
    if (!first_yaw || !second_yaw) { throw std::runtime_error("missing equivalent yaw candidates"); }
    std::cout << "Reported target: bounded KDL candidates=" << broad.size()
              << ", both yaws and FK PASS; alternate/local/winding regressions PASS\n";
    const auto impossible = thor7_grasp::pregraspIkCandidates(start, Eigen::Vector3d(10, 10, 10), .897,
      "base_link", "tool0", 0.0, .003, .053);
    if (!impossible.empty()) { throw std::runtime_error("unreachable target was accepted"); }
  } catch (const std::exception & error) {
    std::cerr << error.what() << '\n';
    result = 1;
  }
  rclcpp::shutdown();
  return result;
}
