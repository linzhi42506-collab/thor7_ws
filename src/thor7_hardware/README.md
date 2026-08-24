# thor7_hardware

真机**只读**接入验证包（第一阶段）。只读取 Diana 机械臂与 AG145 夹爪的反馈，
**不实现、不调用任何运动指令**。

> ⚠️ 重要：本包没有任何运动代码。`diana_readonly_client` 只调用
> `initSrvNetInfo/initSrvV2/getJointCount/getJointPos/getRobotState/getInterpolaterCycleInterval/formatError/getLastError/destroySrv`；
> `ag145_modbus_client` 只读寄存器 `0x0200/0x0201/0x0202`，绝不写 `0x0100/0x0101/0x0103`，
> 绝不调用 `Initialize()`。

## 目录

```
include/thor7_hardware/
  diana_readonly_client.hpp   Diana SDK 只读客户端（RAII）
  ag145_modbus_client.hpp     AG145 Modbus RTU 只读客户端
  modbus_utils.hpp            纯函数（CRC/映射/帧解析，header-only，可单测）
src/
  diana_readonly_client.cpp
  ag145_modbus_client.cpp
  thor7_readonly_probe.cpp    机械臂只读节点
  ag145_readonly_probe.cpp    夹爪只读节点
launch/hardware_readonly.launch.py
config/real_readonly.yaml
test/                        单元测试（不接硬件）
vendor/serial/               厂商 serial 库源码（MIT，来源：大寰官方 DhGripper/third_party/serial）
```

## 依赖

- **Diana SDK**：外部 SDK，不复制到 `/usr/lib`，通过 `DIANA_API_ROOT` 定位。
  解压 `Diana_new12.tar` 后（例如 `/home/robo/3rdparty/Diana_new12/DianaApi`），
  其结构应含 `include/`、`lib/`、`bin/`。
- **vendor/serial**：大寰官方 `DhGripper/third_party/serial` 的源码 + 头文件 + LICENSE，
  本包直接编译，不依赖 build 目录中的 .a/.o。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/thor7_ws
export DIANA_API_ROOT=/home/robo/3rdparty/Diana_new12/DianaApi
export LD_LIBRARY_PATH="$DIANA_API_ROOT/bin:$DIANA_API_ROOT/lib:${LD_LIBRARY_PATH:-}"
colcon build --symlink-install --packages-select thor7_hardware
colcon test --packages-select thor7_hardware
colcon test-result --verbose
```

也可用 CMake 缓存变量代替环境变量：
`colcon build --packages-select thor7_hardware --cmake-args -DDIANA_API_ROOT=...`

运行时库解析：CMake 已为可执行文件设置指向
`${DIANA_API_ROOT}/lib` 与 `${DIANA_API_ROOT}/bin` 的 RPATH；如仍缺库，
按上面的 `LD_LIBRARY_PATH` 设置即可（不要复制 .so 到 /usr/lib，不要覆盖系统 libxml2）。

## 只读运行

```bash
ros2 launch thor7_hardware hardware_readonly.launch.py
# 只读机械臂： ros2 launch thor7_hardware hardware_readonly.launch.py start_gripper:=false
# 只读夹爪：   ros2 launch thor7_hardware hardware_readonly.launch.py start_arm:=false
```

launch 会打印 `READ-ONLY HARDWARE MODE` 与 `NO MOTION COMMANDS ARE IMPLEMENTED`。
不启动 MoveIt / RViz / ros2_control / 任何 controller / grasp_demo。

话题：
- `/thor7/readback_joint_states`：`sensor_msgs/JointState`，name=joint1..joint6，position 来自 `getJointPos()`，velocity 留空（不伪造 0）
- `/ag145/readback_joint_state`：`sensor_msgs/JointState`，name=`ag145_gripper_finger1_joint`，position=换算 rad
- `/diagnostics`：连接状态 / SDK 错误码 / robot state / 数据时间戳；断线或读失败标 ERROR/STALE，**不发布旧值冒充真实状态**

## 为什么不可能发送运动指令

1. `diana_readonly_client.cpp` 只调用上面列出的只读 API；任何运动函数
   （`releaseBrake/moveJToTarget/moveLToPose/servoJ/speedJ/stop/resume`）都未声明、未调用。
2. `ag145_modbus_client.cpp` 只构造 `0x03` 读保持寄存器请求；`0x06` 写单寄存器
   （初始化/力/目标位置）路径完全不存在，`Initialize()` 未被调用。
3. 两个 probe 节点只用定时器读反馈 + 发布，无任何命令写入路径。

## 真机只读测试（下一步由用户手动执行）

```bash
source /opt/ros/humble/setup.bash
source ~/thor7_ws/install/setup.bash
export LD_LIBRARY_PATH="$DIANA_API_ROOT/bin:$DIANA_API_ROOT/lib:${LD_LIBRARY_PATH:-}"
ros2 launch thor7_hardware hardware_readonly.launch.py
# 另开终端观察：
ros2 topic echo /thor7/readback_joint_states
ros2 topic echo /ag145/readback_joint_state
ros2 topic echo /diagnostics
```

> ⚠️ 在只读数据未验证关节顺序、方向和零位之前，**不得实现或启用任何运动命令**。

## 单元测试（不接硬件）

- `test_ag145_crc`：Modbus CRC16 固定向量 + 篡改字节失败
- `test_ag145_mapping`：设备位置 <-> MoveIt 关节映射、钳制、往返误差
- `test_ag145_frame`：0x03 响应解析（正常/CRC/地址/功能码/长度/异常响应）
