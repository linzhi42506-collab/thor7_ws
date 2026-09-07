# Thor7 固定相机手眼标定

本工程采用固定相机（eye-to-hand）方案：ChArUco 板刚性安装在 `link6`，RealSense 固定在工作台。采集器在每张图像的时间戳查询 `base_link -> link6`，再用 ChArUco/PnP 得到 `camera_color_optical_frame -> handeye_target`。求解器使用 OpenCV `calibrateRobotWorldHandEye`（Shah）作为初值，再用 SciPy 鲁棒 SE(3) 全局优化，同时估计相机外参和板安装变换。

相关开源实现：

- OpenCV `calibrateRobotWorldHandEye` Shah/Li
- ROS-Industrial `industrial_calibration_ros2` 的 Ceres 非线性优化思路
- MoveIt Calibration 的 ChArUco 检测（本采集器增加了时间同步和亚像素角点）

## 1. 先确认板尺寸

当前图片是 `11 x 8` 个方格，因此：

```text
板宽 = 11 * square_size
板高 =  8 * square_size
```

`square_size` 是完整黑白方格的边长，`marker_size` 是 ArUco 黑色 marker 的边长，单位必须是米。你发来的 RViz 参数对应 `longest board side = 0.165 m`、横向 11 个方格，因此完整方格边长是 `0.165 / 11 = 0.015 m`，黑色 marker 是 `0.011 m`。如果打印文件确实就是这组参数生成的，采集器必须使用 `0.015/0.011`，不能继续使用之前示例中的 `0.013/0.009`。若用尺子量出的打印尺寸不同，以实测尺寸为准，并同步修改 RViz 和命令行。

## 2. 启动采集器

ROS 2 Humble 的 `rclpy` 使用系统 Python 3.10，不要用 Miniforge 的 Python 3.13：

```bash
cd /home/landau/Documents/thor7_ws
source /opt/ros/humble/setup.bash
source install/setup.bash 2>/dev/null || true

/usr/bin/python3 src/thor7v3_moveit_config/scripts/collect_handeye_charuco.py \
  --overwrite \
  --output handeye_samples.yaml \
  --square-size 0.015 \
  --marker-size 0.011 \
  --squares-x 11 --squares-y 8 \
  --dictionary DICT_4X4_250 \
  --image-topic /camera/camera/color/image_raw \
  --camera-info-topic /camera/camera/color/camera_info \
  --base-frame base_link --eef-frame link6 \
  --sensor-frame camera_color_optical_frame
```

采集器默认允许图像时间戳比最新 TF 超前最多 5 秒，这是因为机械臂停止后驱动器可能不再发布新的 TF 时间戳；只有在确认机械臂完全停止后才应触发采集。如需更严格的时间限制，可增加 `--max-tf-lag-ms 500`。

看到 `Ready` 后，每到一个新姿态并确认整块板在图像内、机械臂完全停止，另开终端执行一次：

```bash
source /opt/ros/humble/setup.bash
ros2 service call /thor7_handeye/capture std_srvs/srv/Trigger {}
```

采集 20--25 个真正不同的姿态。相邻姿态至少改变 5 cm 或 10 度；要覆盖 roll、pitch、yaw 中至少两个轴，并改变相机到板的距离。每个姿态只保存一帧。采集器会拒绝重复姿态、少于 20 个角点或 PnP 重投影误差超过 1.5 px 的图像。

这张板是旧版 RViz/OpenCV 生成的 legacy ChArUco pattern，采集器已显式启用 `legacyPattern=True`。修改脚本后必须先退出旧采集器，再重新启动；否则旧进程仍会报告 `no ChArUco corners detected`。

## 3. 求解和验收

```bash
python3 calibrate_robot_world_handeye.py \
  --samples handeye_samples.yaml \
  --output handeye_calibration_result.yaml \
  --launch-output static_camera_tf.launch.py \
  --compare-methods \
  --parent-frame base_link \
  --child-frame camera_color_optical_frame
```

只接受报告中 RMS 平移小于约 5 mm、RMS 旋转小于约 0.5 度的结果。若仍是厘米级，先不要发布 TF；结果文件中的 `per_sample` 会列出异常姿态。旧的 `movoelt vi.yaml` 只有 8 个独立姿态，运行该方法仍约 23 mm，因此不能继续用于标定。

## 4. 发布前检查

工程中曾存在多个 `base_link -> camera_color_optical_frame` 静态 TF（`123.launch.py`、`moveit_pose1.launch.py` 等）。启动新结果前必须停止所有旧发布器，只保留一个：

```bash
ros2 launch /home/landau/Documents/thor7_ws/static_camera_tf.launch.py
ros2 run tf2_ros tf2_echo base_link camera_color_optical_frame
```

`tf2_echo` 的结果应与 `handeye_calibration_result.yaml` 中的平移和四元数一致。视觉抓取节点使用的相机坐标系也是 `camera_color_optical_frame`，因此不要再把结果改写成其他 child frame。

## 当前新外参

`234.launch.py` 的新结果为：

```text
T_base_camera = [-0.426692, 0.961884, 0.711320]
quaternion_xyzw = [-0.304199, 0.888102, -0.326487, 0.110199]
```

已同步到包内的 `thor7_calibrated_camera.launch.py`。安装/编译后启动：

```bash
cd /home/landau/Documents/thor7_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select thor7v3_moveit_config --symlink-install
source install/setup.bash
ros2 launch thor7v3_moveit_config thor7_calibrated_camera.launch.py
```

先启动一次 RealSense ROS 驱动，并确认下面三个话题存在（抓取脚本不会再通过 Python SDK 打开相机）：

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true enable_depth:=true \
  align_depth.enable:=true enable_sync:=true publish_tf:=false
```

`publish_tf:=false` 很重要：标定结果已经直接发布 `base_link -> camera_color_optical_frame`，不能再让相机驱动为同一个 child frame 发布另一条父链。
如安装了多台 RealSense，在上面的命令中增加 `serial_no:=相机序列号`；不指定时使用驱动默认选择的设备，抓取节点不会重复打开设备。

另一个终端再启动视觉节点（目标类别默认就是 `tree`）：

```bash
cd /home/landau/Documents/thor7_ws
source /opt/ros/humble/setup.bash
/usr/bin/python3 ros2_vision_grasp.py \
  --ros-args \
  -p model_path:=/home/landau/Documents/thor7_ws/best.pt \
  -p base_frame:=base_link \
  -p camera_frame:=camera_color_optical_frame \
  -p color_image_topic:=/camera/camera/color/image_raw \
  -p depth_image_topic:=/camera/camera/aligned_depth_to_color/image_raw \
  -p camera_info_topic:=/camera/camera/color/camera_info
```

若只想检查相机画面而暂时不显示 OpenCV 窗口，可附加 `-p show_window:=false`。`depth_scale` 默认是 RealSense `16UC1` 深度图常用的 `0.001 m/单位`；如果 `ros2 topic echo --once /camera/camera/aligned_depth_to_color/image_raw` 显示的是 `32FC1`，脚本会直接按米读取。
