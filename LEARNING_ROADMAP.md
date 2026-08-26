# Thor7 视觉抓取项目 — 初学者学习路线

> 写给：学过 C、C++ 一般、Python 大致能看懂，ROS2/MoveIt2 只了解概念，想**读懂这个项目 → 自己动手改/写 → 最终做视觉抓取接机械臂**的你。
>
> 使用方式：按顺序阅读。**每一步先跑起来、再读代码**，不要一头扎进源码。
> 文档中标注了每一部分该"**读懂逻辑**"（知道它在干什么、为什么）还是"**精读代码**"（能自己改）。

---

## 第 0 章 项目全景：这个项目到底在干什么

### 0.1 一句话

用 **ROS2 + MoveIt2** 在仿真里让一台 6 轴机械臂（Thor7）装上两指夹爪（AG-145），完成 **"看到物体 → 规划 → 下降 → 夹住 → 抬起"** 的抓取流程；以后接上相机和真机驱动，就变成真正的视觉抓取。

### 0.2 数据流（最重要的一张图，记住它）

```
[模型] URDF/xacro 描述机械臂长什么样
   ↓
[规划] MoveIt2 (move_group) 算出"关节怎么动才能到目标"
   ↓  FollowJointTrajectory action
[执行] ros2_control 控制器 (joint_trajectory_controller)
   ↓  位置指令
[硬件] 仿真: FakeSystem(mock)  /  真机: Diana API (servoL/servoJ)
   ↓
/joint_states 关节实际值
   ↓
[显示] RViz 画出来 / robot_state_publisher 发 TF
```

**你的学习主线就是从左到右走通这条链**，每一个包负责链上的一段。

### 0.3 每个包的角色

| 包 | 对应概念 | 负责什么 | 学习优先级 |
|---|---|---|---|
| `thor7v3_description` | URDF/xacro | 机械臂本体的模型（6 关节、网格） | ★★★ 第一步 |
| `dh_robotics_ag145_description` | URDF/xacro | AG-145 夹爪模型（mimic 关节） | ★★☆ 第二步 |
| `thor7v3_moveit_config` | MoveIt2 | 规划组、IK、控制器接线 | ★★★ 第三步 |
| `thor7_moveit_demo` | MoveIt2 编程 | **最简单的 C++ 示例**（学习起点） | ★★★ 第四步 |
| `thor7_sim` | 综合 | 仿真辅助：夹爪状态、物体、抓取流程、诊断 | ★★☆ 六~九步 |
| （`~/haptic_teleop`） | 真机 SDK | Diana API（现在**用不到**，接真机才用） | ☆ 展望 |

### 0.4 三个必须搞懂的核心概念

1. **URDF/xacro**：用 XML 描述"机器人由哪些连杆(link)和关节(joint)组成，坐标怎么摆"。xacro 是"带变量和宏的 URDF"，方便复用。
2. **MoveIt2**：运动规划框架。核心是"规划组(planning group)"——把一组关节打包，问它"末端到这个位姿，关节该转多少"。
3. **ros2_control**：执行层。MoveIt 算出轨迹，ros2_control 的控制器负责真的（或假装）把关节转过去。仿真用 `mock_components` 假装。

---

## 第 1 章 环境与基本命令（第 0 步，半天）

### 1.1 工作区结构

```
thor7_ws/
├── src/     # 源码（你只改这里）
├── build/   # 编译中间产物（不用管）
├── install/ # 编译结果，运行时被 source（不用管）
└── log/     # 日志（报错来这里看）
```

### 1.2 必须记住的命令（先会用，再懂原理）

```bash
# 每次开新终端先 source（加载 ROS2 + 本工作区）
source /opt/ros/humble/setup.bash
source ~/thor7_ws/install/setup.bash

# 编译（只编一个包 / 全部）
cd ~/thor7_ws
colcon build --packages-select thor7_sim

# 运行
ros2 run <包名> <节点名>
ros2 launch <包名> <launch文件名>

# 观察（"看"比"写"重要）
ros2 node list            # 有哪些节点
ros2 topic list           # 有哪些话题
ros2 topic echo /joint_states   # 看关节实际值
ros2 topic info /joint_states -v   # 谁在发、谁在收
ros2 control list_controllers     # 控制器状态
```

> 💡 建议：花半天把上面命令在项目上跑一遍。能"看到"机器人动起来，比读懂任何代码都有用。

---

## 第 2 章 分步学习路线（核心章节）

### 第 1 步：URDF/xacro — 机械臂怎么被描述（读懂逻辑 + 精读前半部分）

**学什么**：
- `<link>`：连杆（有视觉/碰撞/惯量），`<joint>`：关节（revolute 旋转关节有角度范围）
- 坐标变换：每个 joint 的 `<origin>` 是"子连杆在父连杆坐标系里怎么摆"
- xacro 的 `<xacro:property>`（变量）、`<xacro:macro>`（宏，像函数）、`$(find 包名)`（找文件路径）

**看哪个文件**：
- `src/thor7v3_description/urdf/thor7v3_robot.urdf.xacro` ← **从这里开始，全文不到 50 行**
- 它 include 了 `thor7v3.urdf.xacro`（真正的 6 关节定义，DH 参数）
- 最后 `xacro thor7v3_robot.urdf.xacro > thor7v3_robot.urdf` 展开成纯 URDF（MoveIt 实际加载的是这个）

**跑什么**：
```bash
cd ~/thor7_ws && source /opt/ros/humble/setup.bash && source install/setup.bash
cd src/thor7v3_description/urdf
check_urdf thor7v3_robot.urdf   # 检查模型结构，会打印关节树
ros2 launch thor7v3_description view_thor7v3.launch  # RViz 里看模型
```

**注意**：
- 机械臂 6 个关节是 `joint1~joint6`，范围大多 ±360°，其中 J2/J3 范围小（±179°/±165°）
- **坐标系很关键**：改错 origin 机械臂就"错位"，先用 RViz 建立"哪里是 +X/+Z"的直觉

### 第 2 步：夹爪模型 + mimic 关节（读懂逻辑即可，重点学 mimic）

**学什么**：
- **mimic 关节**（本项目最有教学价值的点）：夹爪只有 1 个"主动"关节 `ag145_gripper_finger1_joint`，其余 7 个关节都 `<mimic>` 跟随它（乘一个系数），就像真的夹爪一个电机带动所有手指
- 例如 `gripper_finger2_joint` mimic finger1 ×1.0，`finger1_finger_joint` ×0.5，指节 ×1.49

**看哪个文件**：`src/dh_robotics_ag145_description/urdf/dh_robotics_ag145_gripper.urdf.xacro`

**注意（本项目的坑）**：
- **mimic 关节的值不会自动出现在 /joint_states 里**，RViz 想显示手指开合，需要一个节点把 finger1 换算成 7 个 mimic 值再发布——这就是 `thor7_sim/src/gripper_state_publisher.cpp` 干的事（先读懂它的逻辑）
- 本项目踩过的大坑：**开合方向**——实测 `finger1 = -0.65 rad = 张开(165mm)`，`finger1 = 0 = 闭合(15.8mm)`，和直觉相反！这种"数值方向"问题只能靠实测/可视化确认，别猜

### 第 3 步：MoveIt2 配置（读懂逻辑）

**学什么**：
- **规划组(planning group)**：`thor7_arm`（base_link→link6 的 6 个关节）、`gripper`（夹爪）
- **SRDF**：规划组、`<end_effector>`（末端执行器）、`<disable_collisions>`（碰撞矩阵：告诉 MoveIt 哪些连杆对不用检查碰撞）
- **kinematics.yaml**：用什么 IK 求解器（KDL）
- 怎么把"模型 + 配置 + 控制器"组合起来（`demo.launch.py`）

**看哪个文件**：
- `src/thor7v3_moveit_config/config/thor7v3.srdf` ← 规划组和碰撞矩阵都在这
- `src/thor7v3_moveit_config/config/kinematics.yaml`
- `src/thor7v3_moveit_config/launch/demo.launch.py`（只有几行，看它怎么用 MoveItConfigsBuilder 组装）

**跑什么**：`ros2 launch thor7v3_moveit_config demo.launch.py` → RViz 里会出现 MoveIt 面板，可以手动拖末端、Plan & Execute

**注意**：
- **碰撞矩阵别乱删**：项目里专门有诊断工具确认"哪些碰撞是真实的"，只有确认是固定安装/必然接触的才对才删
- SRDF 是 MoveIt 的"语义层"：URDF 描述"长什么样"，SRDF 描述"怎么用它规划"

### 第 4 步：第一个 C++ MoveIt 程序（★精读，你的学习核心）

**看哪个文件**：`src/thor7_moveit_demo/src/move_to_pose.cpp`（只有 98 行，全读完）

**学什么**（MoveGroupInterface 最小套路，记住这个模板）：
```cpp
// 1. 创建节点 + 后台 spin 线程（ROS2 消息要靠它转）
auto node = rclcpp::Node::make_shared("名字",
              rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true));
rclcpp::executors::SingleThreadedExecutor executor;
executor.add_node(node);
std::thread spinner([&executor]() { executor.spin(); });

// 2. 创建规划组接口（"我要控制 thor7_arm 这组关节"）
MoveGroupInterface move_group(node, "thor7_arm");
move_group.setMaxVelocityScalingFactor(0.1);   // 速度缩放到 10%（安全）
move_group.setStartStateToCurrentState();

// 3. 设目标（关节目标）
move_group.setJointValueTarget(joints);
// 4. 规划
move_group.plan(plan);
// 5. 执行
move_group.execute(plan);
```

**注意（都是踩过的坑）**：
- **必须** `automatically_declare_parameters_from_overrides(true)`，否则 MoveIt 读不到 launch 传的模型参数，`plan()` 会卡死（本项目真实踩过）
- `setPlanningTime`/`setNumPlanningAttempts` 控制规划耗时和尝试次数
- 命名姿态（`setNamedTarget("home")`）比手动设关节值更稳

### 第 5 步：ros2_control 执行层（读懂逻辑）

**学什么**：
- MoveIt 算出的轨迹 → `joint_trajectory_controller`（FollowJointTrajectory action）→ 硬件接口
- 仿真用 `mock_components/GenericSystem` 假装硬件（`FakeSystem`）
- `controller_manager` 负责加载/激活控制器；`joint_state_broadcaster` 发布关节状态

**看哪个文件**：
- `src/thor7v3_moveit_config/config/ros2_controllers.yaml`（定义了 thor7_arm_controller、gripper_controller、joint_state_broadcaster）
- `src/thor7v3_moveit_config/config/thor7v3.ros2_control.xacro`（FakeSystem 定义 + 每个关节的初始值）
- `src/thor7v3_moveit_config/config/initial_positions.yaml`（启动时关节在哪，含夹爪=张开）

**跑什么**：`ros2 control list_controllers` 看三个控制器是否 active

**注意**：
- **启动初始值**：夹爪初始 = -0.65（张开），这是"验收：启动时夹爪张开"的来源
- 真机时这一层换成 Diana API 驱动（见第 6 章），其他不用动——**这就是分层设计的好处**

### 第 6 步：仿真辅助节点（读懂逻辑）

**看哪个文件**（都在 `src/thor7_sim/src/`）：
- `gripper_state_publisher.cpp`（136 行）：读 finger1 → 算 7 个 mimic → 只发布 mimic（不重复发 finger1）。注意它用 `header.frame_id` 标记自己的消息防止死循环
- `spawn_test_object.cpp`（76 行）：往 MoveIt 规划场景(planning scene)里放一个 50mm 立方体，用 `PlanningSceneInterface::applyCollisionObject`。**物体不是物理实体，是"规划时的障碍物"**

**学什么**：
- 规划场景(planning scene)：MoveIt 里"世界有什么障碍物"的数据库
- 一个话题多个发布者的处理（/joint_states 有两个发布者）

### 第 7 步：抓取流程 grasp_demo（★精读，项目的心脏）

**看哪个文件**：`src/thor7_sim/src/grasp_demo.cpp`（约 850 行，**先读 main() 的流程注释和主干**，别一次读完全部）

**流程主干**（记住这条链）：
```
home(回到初始) → open(夹爪张开) → 选 pre-grasp(算出一个好姿态的关节解)
  → 规划并执行(到 pre-grasp) → Cartesian下降(直线下去) 
  → attach(告诉 MoveIt"物体被抓在手里了") → close(夹爪闭合)
  → Cartesian抬升(带着物体上来)
```

**每一步学什么**：
- `selectBestPreGrasp()`：核心算法。**多采样 IK**（每个 yaw×seed 采样 5 次收集不同解）→ **hard reject**（越界/碰撞/margin 小/|ΔJ1|>90°/运动代价>8rad 全拒绝）→ **选运动代价最小** → 没有好解就**重新采样**，仍失败就**安全放弃**（不硬抓）
- `guardTrajectory()` / `guardCartesianTrajectory()`：**安全护栏**——打印每个关节的 start/goal/min/max/excursion，出现"绕圈"或"IK 跳变"就拒绝执行
- `cartesianMove()`：`computeCartesianPath` 做直线运动（fraction<0.98 拒绝）
- `checkStateValid()`：每一步检查整机碰撞，无效就 abort

**学什么概念**：
- **IK 多解问题**：同一个末端位姿，6 轴臂有多个关节解（绕圈解/翻转腕解/自然解）。KDL 求 IK 有随机性，所以要"采样多个、选最好的、拒绝差的"
- **运动代价(motion_cost)**：Σ|目标关节 - 当前关节|，选动作最小的解
- **为什么"找到解≠好解"**：解可能撞限位、自碰撞、或导致后面直线运动失败

**注意**：
- 这个文件是**你最该精读的**，但先读注释和 main() 主干，再一层层往里看函数
- 它体现了项目最重要的思想：**候选选择器有权说"不"**——宁可放弃，也不执行危险的解

### 第 8 步：调试工具（读懂逻辑，以后自己排查用）

**看哪个文件**：
- `collision_diagnostic.cpp`：FK 算夹爪指尖间距（判断开合方向）+ 各状态碰撞 pair 打印
- `cartesian_diagnostic.cpp`：扰动测试（物体位置 ±2cm 看是否稳定）
- `gripper_test.cpp`：夹爪开合验收

**学什么**：怎么用"诊断程序"定位机器人问题——**先确认状态（关节值/碰撞），再改代码**

---

## 第 3 章 初学者建议（我的经验）

1. **先跑起来，再看代码**。让机器人动起来的那一刻，你对整个系统的理解比读 10 小时代码都深。
2. **学会"看"**：`ros2 topic echo`、RViz、日志。遇到问题第一件事是看 /joint_states 和日志，而不是改代码。
3. **从"改一个数"开始**：改 `move_to_pose.cpp` 里的目标关节值 → 改坐标 → 再写自己的节点。不要一开始就想写完整功能。
4. **C++ 一般的处理**：大部分文件"读懂逻辑"就行（看注释和流程），只有 `move_to_pose.cpp` 和 `grasp_demo.cpp` 的 main() 值得精读。语法不懂的查一下即可，ROS2 的 API 都是固定套路。
5. **参数声明的坑**：ROS2 节点读参数要么 `automatically_declare_parameters_from_overrides(true)`（launch 传的），要么 `declare_parameter(name, 默认值)`。混用会报错。
6. **DDS 抖动**：偶尔出现 `unknown result`、卡住、时断时续，多是网络中间件问题，重跑通常就好；容器/沙箱里可用 `export FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 规避。
7. **仿真≠真机**：现在验证的一切都是"模型级"的。真机上关节方向、限位、精度都要重新标定。

---

## 第 4 章 动手路线：从读到写（按这个顺序练）

| 阶段 | 做什么 | 涉及文件 |
|---|---|---|
| ① 改一个数 | 改 `move_to_pose.cpp` 的目标关节值，看机械臂去别的位置 | `move_to_pose.cpp` |
| ② 改一个目标 | 改成 `setNamedTarget("home")`、或末端位姿 `setPoseTarget` | 同上 |
| ③ 写自己的节点 | 复制 move_to_pose 骨架，加一个参数、打印、订阅 | 新建 .cpp + 改 CMakeLists |
| ④ 加物体 | 用 `spawn_test_object.cpp` 的写法，换个位置/大小/形状 | `spawn_test_object.cpp` |
| ⑤ 写抓取 | 照 `grasp_demo.cpp` 的流程注释，先只写 home→pre-grasp 两步 | `grasp_demo.cpp` |
| ⑥ 加安全 | 给自己的流程加 `checkStateValid` 和轨迹 guard | 复用现有函数 |

---

## 第 5 章 未来展望：视觉抓取 + 真机（你现在不用学，但知道方向）

### 5.1 视觉怎么接进来

现在的物体坐标是写死的 `(0.35, 0, 0.10)`。以后流程：
```
相机检测到物体 → 发布 /object_pose (PoseStamped，camera 坐标系)
   ↓ TF2 坐标变换
base_link 坐标系下的物体位置
   ↓
现在的 selectBestPreGrasp()（不用改！它已经接受任意 x/y/z）
   ↓
抓取
```
**这就是为什么 grasp_demo 的"候选评估器"设计成接受物体坐标**——以后把写死的坐标换成话题订阅就行，抓取逻辑几乎不用重写。

### 5.2 真机怎么接

现在的执行层是 `FakeSystem`（假装）。真机时：
```
ros2_control 控制器 ← 换成 Diana API 驱动
  ├─ 方案A：写 ros2_control Hardware Interface 插件（标准做法）
  │        把关节位置指令 → servoJ/servoL 发给机械臂
  └─ 方案B：thor7_driver 桥接节点（订阅轨迹直接 servoL）
夹爪：DhGripper 串口（/dev/ttyUSB0, 115200）
```
需要的 SDK：`libDianaApi.so`（现在在另一台电脑上，本机还没有）。**仿真阶段完全不需要它。**

### 5.3 仿真 vs 真机的差距（心里要有数）

- 真机有**关节限位、速度限制、奇异点**——仿真规划成功的轨迹真机上可能不可行
- 真机需要**手眼标定**（相机坐标↔机械臂坐标），仿真里是直接给的
- 真机有**通信延迟和抖动**，控制周期要求更高

---

## 第 6 章 学习顺序速查表

| 顺序 | 内容 | 精读/读懂 | 时间 |
|---|---|---|---|
| 0 | 环境 + 命令 | 会用 | 半天 |
| 1 | URDF/xacro（thor7v3_description） | 精读前 50 行 | 1 天 |
| 2 | 夹爪 + mimic | 读懂逻辑 | 半天 |
| 3 | MoveIt2 配置（SRDF/kinematics） | 读懂逻辑 | 1 天 |
| 4 | move_to_pose.cpp | ★精读 | 1 天 |
| 5 | ros2_control 执行层 | 读懂逻辑 | 半天 |
| 6 | gripper_state_publisher / spawn_object | 读懂逻辑 | 半天 |
| 7 | grasp_demo.cpp 主干流程 | ★精读 main() | 2 天 |
| 8 | 诊断工具 | 读懂逻辑 | 半天 |
| — | 动手：改数→改目标→写节点→加物体→写抓取 | 实践 | 1~2 周 |

**最后一句忠告**：这个项目最值钱的地方不是"能抓"，而是它展示了真实机器人系统怎么分层（模型/规划/执行/安全），以及**"候选生成 + 评估 + 拒绝"**这个通用思路。把这两点吃透，无论换机器人还是接视觉，你都走在前头了。
