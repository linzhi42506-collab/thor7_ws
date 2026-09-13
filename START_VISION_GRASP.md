# THOR7 一键启动

首次使用前，正常 Ctrl+C 结束之前分别启动的机械臂、夹爪、相机、外参、视觉、桌面和抓取节点。
不要保留旧的控制器管理器或占用 `/dev/ttyUSB0` 的探测程序。
之后只使用本入口；一个终端启动，Ctrl+C 结束整套由它启动的节点。

## 只读规划（默认）

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh
```

自动加载 ROS Humble 和工作空间环境。未设置 ROS_DOMAIN_ID 时使用 0。
启动真实只读机械臂反馈、只读夹爪反馈、MoveIt、RViz、D415、新相机外参、YOLO、桌面碰撞场景、朝下预抓取规划节点。
默认模式没有机械臂执行能力或夹爪动作服务，不是虚拟关节模拟。

## 分阶段真机执行

先 Ctrl+C 结束只读规划，再运行：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --execute
```

此选项明确启用 REAL SERVOJ 控制器和 AG145 动作服务，沿用项目原有操作确认标记。
夹爪未初始化时可能自动初始化并运动。启动不会自动规划、执行机械臂轨迹或闭合抓取。
之前的物理工作区、急停和启动姿态要求仍适用。

## 默认配置

* D415 序列号：802212060718；彩色、深度均为 1280×720@30；深度对齐到彩色。
* 外参：工作空间根目录 `handeye_calibration_result.yaml`，每次启动读取。
  不启动旧外参 launch，也不同时启动根目录的 static_camera_tf.launch.py。
* 模型：根目录 `best.pt`；类别 tree；检测到目标时发布红球，启动本身不保证检测成功。
* 木块方向：默认启用 RGB-D 顶面点云 PCA 诊断，绿色箭头为木块长轴，蓝色箭头为
  AG145 开合轴。只有方向连续稳定、长宽比和开口检查通过时才发布 `/yolo_grasp_pose`。
  `use_vision_yaw` 默认仍为 false，所以未经现场验证时规划继续使用手动 `grasp_yaw`。
* 桌面：中心 (-0.473, 0.502)，顶面 Z=-0.023 m，尺寸 0.6×0.6×0.05 m。
* 预抓取高度 70 mm，接近高度 25 mm；手动兼容模式开合方向 yaw=0，工具 +Z 朝下。
* 机械臂和下降速度/加速度比例均为 0.15；夹爪力度 20%。

物理相机移动或设备更换后，需要对应的新标定。桌面/工作区改变时要同步修改参数。

## 木块方向验证和启用

先只做方向诊断，不执行真机。结束旧的一键启动进程，把木块无遮挡地放在工作区，运行：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh \
  enable_orientation_estimation:=true use_vision_yaw:=false
```

另开终端并加载环境：

```bash
source /opt/ros/humble/setup.bash
source /home/landau/Documents/thor7_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 topic hz /yolo_grasp_pose
```

RViz 中红球仍代表抓取中心。绿色箭头应沿木块物理长边，蓝色箭头应与绿色箭头垂直，
即沿夹爪开合方向。终端出现稳定的 `/yolo_grasp_pose` 频率表示方向通过连续样本、角度
标准差、长宽比和最大开口检查；只有 `/yolo_target_point` 而没有该话题表示方向未通过，
此时不得启用视觉 yaw。

依次将同一木块静止摆为约 0、30、45、60、90 度。每个位置观察至少 3 秒，并用：

```bash
timeout 5 ros2 topic echo /yolo_grasp_pose
```

记录画面中的实际长边角度、绿色箭头和日志 `long=...deg opening=...deg std=...deg`。
五个位置都应满足：绿色箭头不沿短边跳转；蓝绿近似垂直；静止时方向抖动不超过 5 度。
正方形、顶面点不足、估计短边超过 145 mm、偏离当前木块约 100×50 mm 的尺寸范围，
或方向不稳定会被有意拒绝。

上述验证通过后，仍先在无执行能力的规划模式启用视觉方向：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh \
  enable_orientation_estimation:=true use_vision_yaw:=true
```

请求一次预抓取规划：

```bash
ros2 service call /thor7_vision_grasp/prepare_pregrasp std_srvs/srv/Trigger '{}'
```

成功信息必须包含 `opening yaw=... rad from vision`。在 RViz 完整播放轨迹，确认 tool0
在高位移动中同步转向，预抓取终点完全朝下且夹爪开合方向与蓝色箭头一致，轨迹不碰桌面或物体。分别验证五个角度，
不得调用 execute 服务。

五个方向的规划都正确后，才结束规划模式并启动真机自动抓取：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --auto-grasp \
  enable_orientation_estimation:=true use_vision_yaw:=true
```

确认木块静止、方向箭头稳定、机械臂处于净空起始姿态且急停可用，再在已加载环境的
另一终端触发一次：

```bash
ros2 service call /thor7_vision_grasp/run_grasp std_srvs/srv/Trigger '{}'
```

视觉位置和 yaw 会作为同一 `PoseStamped` 同时锁定。预抓取运动后即使夹爪遮挡木块，
下降仍沿用同一锁定位置与方向。为避免局部遮挡产生一次错误中心点，只有连续 3 个相互
一致的有效姿态相对锁定位置移动超过 5 mm 或方向变化超过 5 度，才确认目标移动。目标
仍可见时，自动流程会重新获取当前 `/yolo_grasp_pose` 并最多重规划 2 次；若已被遮挡、
没有新的可靠姿态，则停止且禁止盲目下降。不能通过重复调用下降服务绕过。

常用诊断参数：`orientation_height_tolerance` 是顶面相对中心深度的 Z 容差，默认 8 mm；
`orientation_min_aspect_ratio` 默认 1.30，越大越严格；`orientation_max_std_deg` 默认 5 度；
`orientation_max_grasp_width` 默认 0.145 m。当前木块期望长宽为 0.100/0.050 m，允许误差为
0.015/0.010 m。更换不同尺寸木块时必须同步覆盖这四个尺寸参数。只有现场证据表明木块顶面点
被错误排除时才调整，
不要为了让话题出现而直接放宽所有阈值。

## 参数覆盖

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh \
  grasp_yaw:=0.0 pregrasp_clearance:=0.070 near_clearance:=0.020
```

也可以直接使用 ROS launch（先 source 环境）：

```bash
ros2 launch thor7_moveit_demo thor7_vision_grasp_all.launch.py
ros2 launch thor7_moveit_demo thor7_vision_grasp_all.launch.py \
  mode:=execute operator_confirmation:=ARM_THOR7
```

查看所有参数（不启动节点）：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --show-args
```

## 请求规划

另一终端先加载环境，然后手动请求：

```bash
source /opt/ros/humble/setup.bash
source /home/landau/Documents/thor7_ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 service call /thor7_vision_grasp/prepare_pregrasp std_srvs/srv/Trigger '{}'
```

执行服务仍为 execute_pregrasp；接近服务为 prepare_near / execute_near。
预抓取执行并通过反馈 FK 校验后，保存本次目标，默认从到达时起有效 60 秒
（target_lock_sec）。遮挡时接近使用该目标，不要求新的 YOLO 点；物体必须固定。
任何检测点相对锁定位置偏移超过 5 mm 都会使该次锁定失效，包括预抓取运动中。
prepare_near 和 execute_near 均检查有效期、目标移动和现有姿态/起点限制。
重新 prepare_pregrasp、节点重启、执行失败或一次下降派发会清除相应锁定。
prepare_near 不会刷新锁定时间，延长 plan_max_age_sec 也不会延长锁定时间。

## 单轮抓取、放置、回 Home

新增流程及现场步骤见 [PICK_PLACE.md](PICK_PLACE.md)。使用 `--pick-place` 和 `/run_pick_place`，
先通过 `--preview-pick-place` 检查搬运/放置/Home 轨迹。下面的 `--auto-grasp` 仍只完成抓取。

## 单次自动抓取（真机）

已完成手动分步验证后，可启动带自动编排服务的真机模式：

```bash
bash /home/landau/Documents/thor7_ws/start_vision_grasp.sh --auto-grasp
```

节点启动不会自行运动。确认木块完整可见、夹爪方向正确、路径净空并且急停可用后，调用一次：

```bash
ros2 service call /thor7_vision_grasp/run_grasp std_srvs/srv/Trigger '{}'
```

服务依次执行：夹爪张开、预抓取规划、预抓取执行、预抓取终点静止确认、近距离规划、近距离执行、夹爪闭合。
任何一步失败都会停止，不发送后续步骤。默认最终 `near_clearance=0.020 m`，比此前成功的
`0.025 m` 终点低 5 mm。预抓取时平移与视觉 yaw 旋转由同一条碰撞检查轨迹同步完成；到位且
反馈 FK 通过后会额外静止 0.5 秒，才执行竖直下降。夹爪报告 HOLDING 才返回抓取成功；流程不会自动抬升。
调用前机械臂仍需位于清晰、无碰撞并且高于预抓取面的起始姿态；自动流程不会猜测或生成
一个未经验证的回零/起始姿态。对于暂未收到目标或目标刚好过期的情况，流程会等待重试最多
10 秒；工作空间越界、目标移动、规划或执行失败不会重试。
锁定失效后必须重新完成预抓取流程，不能直接采用重启前的旧位置。
只读模式不创建可供下降的执行后锁定。该机制检查下降派发前的条件，
不是对遮挡期间物体移动的实时监测，也未增加执行中视觉急停。
先检查轨迹，再单独执行；没有加入自动下降、自动闭合或抬升流程。
脚本的文件锁防止该脚本重复运行，但不能识别或接管此前手动启动的 ROS 节点。

## 验证范围

验证了 shell 语法、两种模式的 launch 动作构造、执行确认检查和参数展示。
这次没有实际启动相机、机械臂或 ROS 抓取服务；硬件启动结果需在现场运行确认。
