#!/usr/bin/env python3
"""
对比出厂内参(D=0) vs 自标定内参 的真实重投影误差。

目的：判断当前 D415 出的彩色图是 raw(带畸变) 还是 rectified(已校正)，
进而决定视觉抓取该用哪套内参。

方法：每帧检测 Charuco 角点后，分别用两套内参 solvePnP 求板位姿再重投影，
重投影 RMS 越小 = 该套内参对这枚镜头的数据拟合越好。

判断依据：
- 若 factory(D=0) 重投影 RMS < 1px，且误差不随离光轴距离单调增大
  -> 图像是 rectified，出厂内参自洽，自标定的大畸变(k2/k3)是标定板污染。
- 若 factory(D=0) 重投影 RMS 明显偏大，且误差集中在边缘并随 r 增大(桶形)
  -> 图像是 raw，出厂 D=0 不准，抓取必须用自标定 K+D 先去畸变。

注意：本机 cv_bridge 的 C++ 扩展用 NumPy 1.x 编译，在 NumPy 2.2.6 下 import 即崩
(_ARRAY_API not found)，故这里用 np.frombuffer 直接解码 Image 消息，不依赖 cv_bridge。
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

# ---- 标定板（与 calibrate_camera_intrinsics.py 一致）----
SQUARES_X = 11
SQUARES_Y = 8
SQUARE_SIZE = 0.017
MARKER_SIZE = 0.011
DICT_ID = cv2.aruco.DICT_4X4_250

# ---- 出厂内参 (1280x720, D=0，来自 /camera/camera/color/camera_info) ----
FACTORY_K = np.array([
    [925.586669921875, 0.0, 630.60595703125],
    [0.0, 925.2575073242188, 367.77471923828125],
    [0.0, 0.0, 1.0],
])
FACTORY_D = np.zeros(5)

# ---- 自标定内参（截图值；p1/p2 未知暂填 0，分辨率若 !=1280x720 需先缩放）----
CALIB_K = np.array([
    [927.24, 0.0, 627.90],
    [0.0, 938.72, 347.42],
    [0.0, 0.0, 1.0],
])
CALIB_D = np.array([-0.058, -0.579, 0.0, 0.0, -2.39])


def decode_bgr(msg):
    """从 Image 消息解码为 BGR ndarray（绕开 cv_bridge）。"""
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


def reproj(K, D, obj, imgp):
    """用内参 K/D 对 obj 做 solvePnP 求位姿，再重投影，返回每点误差(px)。"""
    _, rvec, tvec = cv2.solvePnP(obj, imgp, K, D)
    rp, _ = cv2.projectPoints(obj, rvec, tvec, K, D)
    return np.linalg.norm((rp - imgp).reshape(-1, 2), axis=1)


class CompareNode(Node):
    def __init__(self):
        super().__init__('compare_intrinsics')
        self.sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.cb,
            qos_profile_sensor_data)

        aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_ID)
        board = cv2.aruco.CharucoBoard(
            (SQUARES_X, SQUARES_Y), SQUARE_SIZE, MARKER_SIZE, aruco_dict)
        board.setLegacyPattern(True)
        self.detector = cv2.aruco.CharucoDetector(board)
        self.cb_pts = board.getChessboardCorners()  # (70,3)

        self.frame_cnt = 0
        self.sum_f = 0.0
        self.sum_c = 0.0
        self.sum_maxf = 0.0
        self._printed_res = False
        self.get_logger().info('compare node up, waiting for corners...')

    def cb(self, msg):
        img = decode_bgr(msg)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        ch_corners, ch_ids, _, _ = self.detector.detectBoard(gray)
        if ch_ids is None or len(ch_ids) < 8:
            return

        if not self._printed_res:
            self._printed_res = True
            print(f"图像分辨率: {msg.width}x{msg.height}  encoding={msg.encoding}")

        obj = self.cb_pts[ch_ids.flatten()].astype(np.float32).reshape(-1, 1, 3)
        imgp = ch_corners.astype(np.float32).reshape(-1, 1, 2)

        err_f = reproj(FACTORY_K, FACTORY_D, obj, imgp)
        err_c = reproj(CALIB_K, CALIB_D, obj, imgp)

        rms_f = float(np.sqrt(np.mean(err_f ** 2)))
        rms_c = float(np.sqrt(np.mean(err_c ** 2)))

        # 误差随离主点距离的分布（桶形畸变 => 误差随 r 单调增大）
        cx, cy = FACTORY_K[0, 2], FACTORY_K[1, 2]
        r = np.linalg.norm(imgp.reshape(-1, 2) - np.array([cx, cy]), axis=1)
        q = np.quantile(r, [0.0, 0.5, 1.0])
        inner = err_f[r < q[1]]
        outer = err_f[r >= q[1]]

        self.frame_cnt += 1
        self.sum_f += rms_f
        self.sum_c += rms_c
        self.sum_maxf += float(err_f.max())

        print(f"[frame {self.frame_cnt:2d}] corners={len(ch_ids):2d}  "
              f"factory_RMS={rms_f:.3f}px  calib_RMS={rms_c:.3f}px  "
              f"factory_inner={np.mean(inner):.3f}px  factory_outer={np.mean(outer):.3f}px  "
              f"max_f={err_f.max():.3f}px")

        if self.frame_cnt >= 30:
            n = self.frame_cnt
            print("\n================ 汇总 ================")
            print(f"factory(D=0)  平均 RMS : {self.sum_f / n:.3f} px")
            print(f"calib(K+D)    平均 RMS : {self.sum_c / n:.3f} px")
            print(f"factory 平均最大误差 : {self.sum_maxf / n:.3f} px")
            print("判断：")
            print("  factory_RMS < 1px 且 inner/outer 接近 => 图像已校正, 用出厂")
            print("  factory_RMS 明显大且 outer >> inner   => 图像 raw, 用自标定 K+D")
            rclpy.shutdown()


def main():
    rclpy.init()
    node = CompareNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
