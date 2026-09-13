# 单轮视觉抓取、放置和回 Home

2026-09-12：使用附件 `8f16de91` 的反馈数据。第一组为 Home；第二组 TF 和第三组关节值为同一个放置终点。
程序按 **joint1～joint6 的名称**配置，不照抄 `/joint_states` 消息中的数组顺序。

2026-09-12 预抓取 IK 更新：两个相差 180° 的夹爪开合方向分别求解，按实际关节角变化排序。
先搜索每轴相对起点 1.5/2.5 rad 的局部解；若没有可验证轨迹，再从模型关节限位内的多个初值搜索。
保持模型关节限位，等价转数选择也必须在限位内。候选仍须通过 MoveIt 碰撞规划、沿途关节范围/
最低高度及终点 FK 校验；一次预抓取候选规划约限时 30 秒，超时拒绝，不保证任意起点可达。
日志 `Pregrasp IK` 包含起始关节角、目标、两种 yaw 和候选数量，用于区分未找到 IK 与未找到合格路径。

若读取关节时遇到 `xmlrpc.client.Fault ... !rclpy.ok()`，这是 ROS CLI daemon 上下文失效；
它不表示关节超限。反馈节点正在运行时，可绕过 daemon 只读获取关节值：

```bash
source /opt/ros/humble/setup.bash
source /home/landau/Documents/thor7_ws/install/setup.bash
export ROS_DOMAIN_ID=0
/usr/bin/python3 /opt/ros/humble/bin/ros2 topic echo /joint_states \
  sensor_msgs/msg/JointState --once --no-daemon --qos-profile sensor_data
```

也可执行 `ros2 daemon stop`、`ros2 daemon start` 重建命令行缓存进程；这不会重启机械臂控制节点。

| 目标 | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---:|---:|---:|---:|---:|---:|
| Home，rad | 2.0158478661 | -0.2619504546 | 2.1521946480 | 0.0000355782 | 1.2515935750 | 3.3187155759 |
| 放置示教值，rad | 1.4140899372 | 0.5663726303 | 1.4147925124 | 0.0000715308 | 1.1602741503 | 2.7171044538 |

放置采用 J1～J5；J6 替换为抓住物体时的实际值，并检查 FK 的位置、朝下轴是否仍与示教点一致。
这不是把 J6 置零，也不是让 J6 不受控。到放置终点时必须到达替换后的六关节目标。
平面允许木块以不同水平角度落下，搬运过程末端保持朝下，yaw 可以变化。
Home 则使用上述全部六个关节值，包括 J6=3.3187 rad，不转换成负角度，不使用旧 SRDF 的 home。

现有模型 FK 与用户提供的 Home/放置 TF 平移分别相差约 0.022/0.026 mm。
这只验证数据顺序和 ROS 模型一致，**不代表真机定位精度达到这些数值**。

## 新增行为

`/thor7_vision_grasp/run_pick_place` 只执行一轮：

1. 从当前实际姿态开始，张开夹爪，沿用现有视觉位置/yaw 抓取流程。
2. AG145 返回 HOLDING 后，将约 100×50×25 mm 木块加入附着碰撞体。
3. 先规划完抬升、水平搬运、放置下降三段；任一段无法完整规划则不抬升。水平搬运使用保持朝下姿态的
   笛卡尔路径和碰撞检查，不再让 OMPL 在六关节约束流形上盲目采样。
4. 保持朝下，沿 base Z 抬升 80 mm；搬运至放置点上方 80 mm；竖直下降到放置点。
5. 实际关节和 tool0 FK 到位校验通过后才张开夹爪。
6. 木块从附着体转为放置点的场景障碍物；张开得到确认后竖直撤离 80 mm，再规划回 Home。
7. 默认到达 Home 后停止，不等待新目标；连续模式另见本文末尾。

竖直路径必须 100% 完成，检查关节跳变、XY 漂移、朝向、最低高度，并按低速重新计时。
抓取仍使用现有 20 mm `near_clearance`，没有再次降低抓取落点。
搬运和竖直段的默认速度、加速度均为 15%。

`/run_grasp` 仍是原有“抓住后停止”服务，**它不是完整放置流程**。不要先调用它再调用 `/run_pick_place`；
完整流程会自己完成抓取。新协调器会阻止成功抓取后重复触发而意外张开夹爪。

## 现场必须先确认

- 末端没有拿着东西，起始姿态高于目标预抓取高度；木块完整可见，方向箭头稳定。
- 放置点 `tool0 ≈ [0.113, 0.716, 0.215] m` 是你示教的释放点。必须确认**持物时**这个位置可以释放：
  夹爪不撞支撑面、木块底面得到支撑或只留可接受的小间隙。空夹爪示教的位置不能证明木块底面已经接触支撑面。
- 该放置点在现有抓取桌面模型范围之外。代码没有编造放置台高度，也没有自动建模放置台、泡沫、支架或沿途杂物。
  搬运通道、放置点上方 80 mm 和 Home 回程应先净空；实际障碍物未建模时，RViz 通过并不保证避障。
- 夹持体中心暂按 `tool0` 局部 +Z 方向 `near_clearance + 木块高度/2` 建模，另加每面 3 mm 碰撞裕量。
  这是视觉顶面和夹持关系的近似模型，要结合 RViz 和实际夹持核对。

## 第一次操作：先预览

先正常结束旧的一键启动终端，避免重复节点。机器人停稳且空夹爪处于朝下净空姿态后：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --preview-pick-place \
  use_vision_yaw:=true
```

另一终端：

```bash
source /opt/ros/humble/setup.bash
source /home/landau/Documents/thor7_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 service call /thor7_vision_grasp/preview_place_home std_srvs/srv/Trigger '{}'
```

等待 `success=True`，在 RViz 的 `/display_planned_path` 查看完整轨迹。
这是**从当前 tool0 位置假设持物后**的“抬升→放置→撤离→Home”预览，不是从视觉抓取终点开始的整轮预演。
预览不执行机械臂/夹爪动作，也不改全局场景。预览在示教放置点模拟松开木块，随后用空夹爪规划
竖直撤离和回 Home，与真实动作顺序一致；它不会在全局场景中留下预览木块。
若机器人当前位置已经很高，额外抬升 80 mm 可能不可达；不要为通过预览修改已确认的目标关节值。
预览失败返回具体原因，不执行。预览通过也不缓存可直接执行的轨迹；真机每轮从实际反馈重新规划。

2026-09-12 首次预览曾返回 `carry/Home planning failed`。后台明确显示朝向约束被生成为未归一化的
`[x,y,z,w]=[1,0,0,1]`，MoveIt 将当前朝下姿态判成无效起点；这不是视觉方向误差。
现已显式设为 `[1,0,0,0]`，并把失败消息细分为 `lifted-to-place-above` 或
`preview retreat-to-Home`，便于确定是哪一段规划失败。

## 检查后执行一轮

结束预览启动终端后，启动真机版本。执行模式下 AG145 的初始化可能产生夹爪运动：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --pick-place \
  use_vision_yaw:=true \
  pregrasp_clearance:=0.070 near_clearance:=0.020 \
  velocity_scaling:=0.15 acceleration_scaling:=0.15 \
  descent_velocity_scaling:=0.15 descent_acceleration_scaling:=0.15 \
  target_lock_sec:=120.0
```

在另一已 source 环境的终端，只调用一次：

```bash
ros2 service call /thor7_vision_grasp/run_pick_place std_srvs/srv/Trigger '{}'
```

调用后会真实运动直到完成一轮或某一步失败，不必再手动调用 prepare/execute/open。
成功消息包含 `single pick/place completed` 和 `Home feedback PASS`。
日志阶段为 `LIFT_80MM → TRANSFER_ABOVE_PLACE → LOWER_TO_PLACE → RETREAT_80MM → HOME`。
抓住后可能先静止数秒完成搬运规划，这是正常现象，不要重复发命令。

## 失败、互斥和本版边界

- `MoveIt planning code=-6` 表示 `TIMED_OUT`，不是“没有 IK”或“视觉未识别”。预抓取先尝试普通碰撞安全
  关节路径，再对整条轨迹密集检查 `tool0` 最低高度、关节限位和终点姿态；只有普通路径未通过安全复核时，
  才回退到带高度约束的规划。单候选默认 8 秒，总预算 40 秒，可分别用
  `pregrasp_candidate_planning_time` 和 `pregrasp_total_planning_time` 调整。
- 视觉过期、工作区拒绝、IK/路径规划失败等发生在机械臂轨迹执行前，协调器不会再永久锁存；修正目标或起始
  姿态后可直接重新调用 `run_pick_place`。返回消息会明确包含 `pregrasp was not executed` 和 `safe to retry`。
- 任一执行、夹爪或反馈检查失败都不发送后续动作，并会锁住本次协调器，避免再次触发时自动松爪。
  服务超时不等于底层动作已停止；需要立即现场检查，不要用反复重启/重复请求解决。
- 搬运失败不主动松爪，附着物保留；放置后木块保留为场景障碍物。下次有同名木块或其他附着物会拒绝开始，
  防止重复往占用位置投放。恢复前需先确认实物、夹爪和场景一致，不提供一键清空场景的绕过命令。
- 正常公开的 prepare/execute 服务与协调器之间加入本机互斥；不要手动调用 `/sequence/*` 内部服务。
  这不是硬件互锁：RViz Execute、示教器、直接夹爪/轨迹 action 和另一台主机的命令不受此锁管理。
  自动运行期间只保留一个控制入口，急停仍使用现有硬件方式。
- HOLDING 是闭合动作结束时的夹爪反馈；本版没有增加搬运期间持续掉块监控或接触放置检测。
- 本次没有改变视觉追踪/遮挡策略：预抓取重规划沿用现有逻辑；遮挡时仍可能在有效锁定时限内沿已保存目标下降，
  并非执行中的实时视觉伺服。预抓取轨迹执行前会最后核对新鲜 XYZ+yaw；通过后提交本轮快照，夹爪进入
  视野产生的遮挡或伪检测不会再撤销它，预抓取到达并通过 FK 后按该快照竖直下降。物体从轨迹开始执行后
  必须保持固定。连续模式只在机械臂已回 Home 后接受新目标，不会在下降或搬运中在线跟踪目标。

## 连续抓取循环

单轮验证通过后，连续模式从任意当前起点开始执行第一轮；之后每轮到 Home 才等待下一件木块。
启动命令：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --auto-loop \
  use_vision_yaw:=true \
  new_target_min_distance:=0.050 \
  new_target_wait_sec:=0.0
```

随后只调用一次：

```bash
ros2 service call /thor7_vision_grasp/run_pick_place std_srvs/srv/Trigger '{}'
```

每轮顺序为：视觉抓取、抬升、放置、张开、竖直撤离 80 mm、闭合空夹爪、回 Home、等待下一目标。
闭合发生在撤离之后，因此不会在放置面旁夹住刚释放的木块，但回 Home 的整个过程夹爪是闭合的。

新目标必须在 `base_link` 中相对上一抓取位置至少移动 50 mm，并连续三帧落在 10 mm 内才会被接纳；
进入下一轮抓取前还会再次检查，避免旧木块残留在画面中时重复抓取。`new_target_wait_sec:=0.0` 表示在 Home
无限等待；给正数（例如 `300.0`）会在超时后停止于 Home。`max_cycles:=3` 可限制三轮后停止；`0` 是无限循环。

可在任意阶段请求“本轮结束后停”：

```bash
ros2 service call /thor7_vision_grasp/stop_after_cycle std_srvs/srv/Trigger '{}'
```

若此时正在 Home 等待，新服务会立即结束循环；若正在搬运，会完成当前轮并回 Home 后停止。

连续模式默认不把已放下木块留在 MoveIt 场景中（`retain_placed_objects:=false`），否则下一轮会被同名放置物阻断。
这不代表真实障碍物消失：必须在每轮前人工取走木块或保证放置位置/堆叠不会碰到机械臂和夹爪。单轮 `--pick-place`
仍默认保留放下的木块为碰撞物。

配置在 `src/thor7_moveit_demo/config/pick_place.yaml`。改配置后重新 build 并重启；运行过程中不修改关节目标和木块尺寸。
