#!/usr/bin/env python3
"""诊断 ChArUco 检测：终端定时打印 marker 数与角点数(不弹窗)。

用法:
  source /opt/ros/humble/setup.bash
  /usr/bin/python3 diagnose_charuco.py --ros-args -p image_topic:=/camera/camera/color/image_raw

每 0.5s 打印一行:  markers:N  charuco_corners:M
  markers=0          -> 板没进画面/太远太小/反光/字典错
  charuco_corners>=8 -> 检测正常，可回标定脚本采集
Ctrl+C 退出。
"""
import time
import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

SQUARES_X, SQUARES_Y = 11, 8
SQUARE_SIZE, MARKER_SIZE = 0.015, 0.011
DICT_ID = cv2.aruco.DICT_4X4_250


def make_board():
    d = cv2.aruco.getPredefinedDictionary(DICT_ID)
    b = cv2.aruco.CharucoBoard((SQUARES_X, SQUARES_Y), SQUARE_SIZE, MARKER_SIZE, d)
    b.setLegacyPattern(True)
    return b


def decode_bgr(msg):
    enc = msg.encoding.lower()
    ch = {"bgr8": 3, "rgb8": 3, "bgra8": 4, "rgba8": 4, "mono8": 1}.get(enc)
    if ch is None:
        raise ValueError(enc)
    a = np.frombuffer(msg.data, np.uint8).reshape(msg.height, msg.step)
    a = a[:, :msg.width * ch].reshape(msg.height, msg.width, ch)
    if enc == "rgb8":
        return cv2.cvtColor(a, cv2.COLOR_RGB2BGR)
    if enc == "bgra8":
        return cv2.cvtColor(a, cv2.COLOR_BGRA2BGR)
    if enc == "rgba8":
        return cv2.cvtColor(a, cv2.COLOR_RGBA2BGR)
    if enc == "mono8":
        return cv2.cvtColor(a, cv2.COLOR_GRAY2BGR)
    return a.copy()


class DiagNode(Node):
    def __init__(self):
        super().__init__("charuco_diag")
        self.declare_parameter("image_topic", "/camera/camera/color/image_raw")
        self.board = make_board()
        self.marker_det = cv2.aruco.ArucoDetector(self.board.getDictionary())
        self.charuco_det = cv2.aruco.CharucoDetector(self.board)
        self.latest = None
        self.create_subscription(
            Image, self.get_parameter("image_topic").value,
            self._cb, qos_profile_sensor_data)

    def _cb(self, msg):
        self.latest = decode_bgr(msg)

    def run(self):
        last = 0.0
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.latest is None:
                continue
            gray = cv2.cvtColor(self.latest, cv2.COLOR_BGR2GRAY)
            res = self.marker_det.detectMarkers(gray)
            mids = res[1] if len(res) > 1 else None
            cc, cids, _, _ = self.charuco_det.detectBoard(gray)
            nm = 0 if mids is None else len(mids)
            nc = 0 if cids is None else len(cids)
            now = time.time()
            if now - last >= 0.5:
                last = now
                ids_str = "" if mids is None else f" ids={np.asarray(mids).reshape(-1).tolist()}"
                print(f"markers:{nm}  charuco_corners:{nc}{ids_str}", flush=True)


def main():
    rclpy.init()
    n = DiagNode()
    try:
        n.run()
    except KeyboardInterrupt:
        pass
    finally:
        n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
