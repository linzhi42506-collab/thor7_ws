#include <cstring>
#include <iomanip>
#include <iostream>

#include "DianaAPI.h"
#include "DianaAPIDef.h"

int main()
{
  constexpr const char * robot_ip = "192.168.10.75";

  srv_net_st net_info;
  std::memset(&net_info, 0, sizeof(net_info));

  initSrvNetInfo(&net_info);

  std::strncpy(
    net_info.SrvIp,
    robot_ip,
    sizeof(net_info.SrvIp) - 1);

  net_info.SrvIp[sizeof(net_info.SrvIp) - 1] = '\0';

  const int init_ret =
    initSrvV2(nullptr, nullptr, nullptr, &net_info);

  if (init_ret != 0) {
    std::cerr
      << "initSrvV2 failed: "
      << init_ret
      << std::endl;

    return 1;
  }

  double min_pos[7]{};
  double max_pos[7]{};

  double max_vel[7]{};
  double max_acc[7]{};

  double mech_min_pos[7]{};
  double mech_max_pos[7]{};

  double mech_max_vel[7]{};
  double mech_max_acc[7]{};

  const int pos_ret =
    getJointsPositionRange(
      min_pos,
      max_pos,
      robot_ip);

  const int vel_ret =
    getMaxJointsVel(
      max_vel,
      robot_ip);

  const int acc_ret =
    getMaxJointsAcc(
      max_acc,
      robot_ip);

  const int mech_pos_ret =
    getMechanicalJointsPositionRange(
      mech_min_pos,
      mech_max_pos,
      robot_ip);

  const int mech_vel_ret =
    getMechanicalMaxJointsVel(
      mech_max_vel,
      robot_ip);

  const int mech_acc_ret =
    getMechanicalMaxJointsAcc(
      mech_max_acc,
      robot_ip);

  std::cout << std::fixed << std::setprecision(6);

  std::cout << "=== Thor7 READ-ONLY limits probe ===\n";
  std::cout << "robot_ip = " << robot_ip << "\n\n";

  std::cout
    << "return codes:"
    << " position=" << pos_ret
    << " velocity=" << vel_ret
    << " acceleration=" << acc_ret
    << " mechanical_position=" << mech_pos_ret
    << " mechanical_velocity=" << mech_vel_ret
    << " mechanical_acceleration=" << mech_acc_ret
    << "\n\n";

  for (int i = 0; i < 6; ++i) {
    std::cout
      << "J" << (i + 1)
      << " configured_pos=["
      << min_pos[i] << ", "
      << max_pos[i] << "]"
      << " max_vel=" << max_vel[i]
      << " max_acc=" << max_acc[i]
      << "\n";

    std::cout
      << "   mechanical_pos=["
      << mech_min_pos[i] << ", "
      << mech_max_pos[i] << "]"
      << " mechanical_max_vel="
      << mech_max_vel[i]
      << " mechanical_max_acc="
      << mech_max_acc[i]
      << "\n";
  }

  const char state =
    getRobotState(robot_ip);

  std::cout
    << "\nrobot_state="
    << static_cast<int>(state)
    << "\n";

  destroySrv(robot_ip);

  return 0;
}
