#include "command_lease.hpp"
#include <sys/wait.h>
#include <iostream>

int main()
{
  // Isolated domain for offline testing; no ROS/network/hardware is started.
  setenv("ROS_DOMAIN_ID", "979797", 1);
  {
    thor7_grasp::CommandLease first;
    if (!first.acquired()) { return 1; }
    thor7_grasp::CommandLease concurrent;
    if (concurrent.acquired()) { return 2; }
    const auto child = fork();
    if (child < 0) { return 3; }
    if (child == 0) {
      thor7_grasp::CommandLease other_process;
      _exit(other_process.acquired() ? 4 : 0);
    }
    int status = 0;
    if (waitpid(child, &status, 0) != child || !WIFEXITED(status) || WEXITSTATUS(status)) { return 5; }
  }
  thor7_grasp::CommandLease after_release;
  if (!after_release.acquired()) { return 6; }
  std::cout << "Concurrent same-process and cross-process commands blocked; released lease reusable\n";
}
