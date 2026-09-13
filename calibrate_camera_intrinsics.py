#!/usr/bin/env python3
"""Thor7 固定相机(RealSense D455 color) 内参标定。

复用工程 ChArUco 板(11x8, DICT_4X4_250, legacy pattern)，与
collect_handeye_charuco.py 完全一致。相机由 realsense2_camera 驱动持有，
本脚本只订阅 color 话题，不直接开相机。

OpenCV 5.0 注意：calibrateCameraCharuco 已被移除，这里用 cv2.calibrateCamera
替代 —— 用 board.getChessboardCorners()[charuco_ids] 构造 objectPoints。

输入方式(双通道，任选其一)：
  - OpenCV 窗口 "intrinsic_calib" 有焦点时按 s / c / q
  - 终端里按 s / c / q(不用回车，由后台线程读取 stdin)
用法:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 calibrate_camera_intrinsics.py \
      --ros-args -p image_topic:=/camera/camera/color/image_raw
"""

import cv2
import numpy as np
import rclpy
import select
import sys
import termios
import threading
import tty
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# ---- 标定板参数：与 collect_handeye_charuco.py 保持一致，实测为准 ----
SQUARES_X = 11
SQUARES_Y = 8
SQUARE_SIZE = 0.017   # 米。务必量打印板确认(文档0.165/11=0.015，脚本help旧写0.025)
MARKER_SIZE = 0.011
DICT_ID = cv2.aruco.DICT_4X4_250


def make_board():
    dictionary = cv2.aruco.getPredefinedDictionary(DICT_ID)
    board = cv2.aruco.CharucoBoard(
        (SQUARES_X, SQUARES_Y), SQUARE_SIZE, MARKER_SIZE, dictionary)
    board.setLegacyPattern(True)  # 与手眼采集器保持一致
    return board


def decode_bgr(msg: Image):
    enc = msg.encoding.lower()
    ch = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(enc)
    if ch is None:
        raise ValueError(f"unsupported encoding {msg.encoding!r}")
    arr = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    arr = arr[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if enc == "rgb8":
        return cv2.cvtColor(arr, cv2.COLOR_RGB2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(arr, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(arr, cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
    return arr.copy()


class IntrinsicCalibNode(Node):
    def __init__(self):
        super().__init__("camera_intrinsic_calib")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.board = make_board()
        self.detector = cv2.aruco.CharucoDetector(self.board)
        self.obj_points = []  # list of (N,1,3) float32
        self.img_points = []  # list of (N,1,2) float32
        self.latest = None
        self.size = None
        self._stop = False
        self.create_subscription(
            Image,
            self.get_parameter("image_topic").value,
            self._callback,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            "s=采集 c=标定 q=退出；目标20-30张，覆盖四角+中心+不同距离/角度")

    def _callback(self, msg):
        self.latest = decode_bgr(msg)
        self.size = (msg.width, msg.height)

    def _detect(self):
        if self.latest is None:
            return None
        gray = cv2.cvtColor(self.latest, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, _, _ = self.detector.detectBoard(gray)
        if charuco_corners is None or charuco_ids is None or len(charuco_ids) < 8:
            return None
        ids = np.asarray(charuco_ids, dtype=np.int64).reshape(-1)
        return charuco_corners, ids

    def add_sample(self):
        r = self._detect()
        if r is None:
            print("角点不足(<8)，跳过")
            return
        corners, ids = r
        all3d = self.board.getChessboardCorners().reshape(-1, 3)  # (70,3)
        if ids.min() < 0 or ids.max() >= len(all3d):
            print("角点 id 越界，跳过")
            return
        self.obj_points.append(all3d[ids].astype(np.float32).reshape(-1, 1, 3))
        self.img_points.append(corners.astype(np.float32))
        print(f"已采集 {len(self.obj_points)} 张，本帧 {len(ids)} 个角点")

    def calibrate(self):
        if len(self.obj_points) < 8:
            print(f"样本不足({len(self.obj_points)} < 8)，继续采集")
            return
        ret, K, D, _, _ = cv2.calibrateCamera(
            self.obj_points, self.img_points, self.size, None, None)
        print("=" * 56)
        print(f"重投影 RMS = {ret:.4f} px   (目标 < 0.5 px)")
        print("K =\n" + np.array2string(K, precision=5, suppress_small=True))
        print("D = " + np.array2string(D.reshape(-1), precision=6, suppress_small=True))
        print("=" * 56)
        np.savez("intrinsics.npz", K=K, D=D.reshape(-1))
        print("已保存 intrinsics.npz (K, D)")

    def run(self):
        while rclpy.ok() and not self._stop:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is not None:
                disp = self.latest.copy()
                gray = cv2.cvtColor(disp, cv2.COLOR_BGR2GRAY)
                corners, ids, _, _ = self.detector.detectBoard(gray)
                if corners is not None and ids is not None and len(ids) > 0:
                    for p in corners.reshape(-1, 2):
                        cv2.circle(disp, (int(p[0]), int(p[1])), 4, (0, 255, 0), 2)
                    cv2.putText(
                        disp, f"corners:{len(ids)} samples:{len(self.obj_points)}",
                        (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.imshow("intrinsic_calib", disp)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("s"):
                self.add_sample()
            elif key == ord("c"):
                self.calibrate()
            elif key == ord("q"):
                break


def _terminal_key_listener(node):
    """后台线程：直接从终端 stdin 读单个键(不需回车)，转发到 node.

    这样在 ToDesk 桌面终端里按 s/c/q 就能采集/标定/退出，
    不必切到 OpenCV 窗口，也不会和 rqt_image_view 抢焦点。
    """
    if not sys.stdin.isatty():
        print("stdin 不是 tty，跳过终端按键监听（请用 OpenCV 窗口按键）")
        return
    fd = sys.stdin.fileno()
    try:
        old_settings = termios.tcgetattr(fd)
    except termios.error:
        return
    try:
        tty.setraw(fd)
        print("终端按键已启用: 在终端里直接按 s / c / q 即可，不用回车不用切窗口")
        while rclpy.ok() and not node._stop:
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            if r:
                ch = sys.stdin.read(1)
                if ch == "s":
                    node.add_sample()
                elif ch == "c":
                    node.calibrate()
                elif ch == "q" or ch == "\x03":  # q 或 Ctrl+C
                    break
    finally:
        try:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        except termios.error:
            pass


def main():
    rclpy.init()
    node = IntrinsicCalibNode()
    listener = threading.Thread(
        target=_terminal_key_listener, args=(node,), daemon=True
    )
    listener.start()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node._stop = True
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
