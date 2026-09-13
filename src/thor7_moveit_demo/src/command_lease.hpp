#pragma once
#include <sys/file.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstdlib>
#include <string>

namespace thor7_grasp
{
// Cooperating ROS nodes on this host share a lease. This is not a hardware
// interlock: RViz Execute, pendant motion and direct action clients bypass it.
class CommandLease
{
public:
  CommandLease()
  {
    const char * domain = std::getenv("ROS_DOMAIN_ID");
    std::string suffix = domain ? domain : "0";
    if (suffix.find_first_not_of("0123456789") != std::string::npos) { return; }
    const auto path = "/tmp/thor7-sequence-" + std::to_string(getuid()) + "-" + suffix + ".lock";
    fd_ = open(path.c_str(), O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW, 0600);
    acquired_ = fd_ >= 0 && flock(fd_, LOCK_EX | LOCK_NB) == 0;
  }
  ~CommandLease() { if (fd_ >= 0) { close(fd_); } }
  CommandLease(const CommandLease &) = delete;
  CommandLease & operator=(const CommandLease &) = delete;
  bool acquired() const { return acquired_; }
private:
  int fd_{-1};
  bool acquired_{false};
};
}
