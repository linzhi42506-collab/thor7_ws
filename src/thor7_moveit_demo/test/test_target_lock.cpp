#include "target_lock.hpp"
#include <stdexcept>
#include <iostream>
int main()
{
  using Lock = thor7_grasp::TargetLock;
  const auto t = Lock::Clock::now();
  auto require = [](bool x) { if (!x) throw std::runtime_error("target lock regression"); };
  Lock lock;
  const Eigen::Vector3d p(-.32, .43, .02);
  require(!lock.valid(t,60));
  lock.begin(p,.3); require(!lock.valid(t,60)); require(lock.commit());
  // No observations during occlusion: only verified arrival authorizes use.
  require(lock.arrive(t)); require(lock.valid(t+std::chrono::seconds(59),60));
  require(!lock.valid(t+std::chrono::seconds(61),60));
  require(!lock.valid(t-std::chrono::seconds(1),60));
  lock.observe(p+Eigen::Vector3d(.001,0,0),.31,.005,.05); require(lock.valid(t,60));
  // Once committed for motion, partial occlusion/false detections cannot
  // revoke the snapshot used by the vertical descent.
  lock.observe(p+Eigen::Vector3d(.006,0,0),.3,.005,.05);
  lock.observe(p,.3,.005,.05); require(lock.valid(t,60)); require(!lock.moved());
  lock.observe(p+Eigen::Vector3d(.010,0,0),.3,.005,.05);
  lock.observe(p+Eigen::Vector3d(.011,0,0),.3,.005,.05);
  lock.observe(p+Eigen::Vector3d(.009,0,0),.3,.005,.05);
  require(lock.valid(t,60)); require(!lock.moved());
  // Before commit, three consistent shifted observations still reject motion.
  lock.begin(p,.3);
  lock.observe(p+Eigen::Vector3d(.010,0,0),.3,.005,.05);
  lock.observe(p+Eigen::Vector3d(.011,0,0),.3,.005,.05);
  lock.observe(p+Eigen::Vector3d(.009,0,0),.3,.005,.05);
  require(lock.moved()); require(!lock.commit());
  require(!lock.arrive(t));
  lock.begin(p,.3);require(lock.commit());require(lock.arrive(t));lock.reset(); require(!lock.valid(t,60));
  require(!lock.arrive(t));
  // Yaw is axial: +pi and -pi describe the same opening line.
  lock.begin(p,3.13);lock.observe(p,-3.13,.005,.05);require(lock.commit());require(lock.arrive(t));
  lock.begin(p,.3);
  lock.observe(p,.40,.005,.05);lock.observe(p,.41,.005,.05);lock.observe(p,.39,.005,.05);
  require(!lock.arrive(t));require(lock.moved());
  std::cout << "Lock commit, arrival, occlusion, expiry and pre-commit movement PASS\n";
}
