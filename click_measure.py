#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pyrealsense2 as rs

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker


class ClickMeasureNode(Node):

    def __init__(self):
        super().__init__('click_measure_node')

        # ====================================================
        # 手眼标定矩阵：相机坐标系 -> 机械臂 base_link
        # P_base = T_base_camera @ P_camera
        # ====================================================
        self.T_base_camera = np.array([
            [-0.1341, -0.6417, -0.7551, 0.3769],
            [ 0.5085,  0.6095, -0.6082, 0.3357],
            [ 0.8505, -0.4656,  0.2446, 0.2836],
            [ 0.0000,  0.0000,  0.0000, 1.0000]
        ], dtype=np.float64)

        self.base_frame = 'base_link'

        # 有效深度范围，单位：米
        self.min_depth = 0.10
        self.max_depth = 2.00

        # 保存最新画面
        self.latest_color_image = None
        self.latest_color_frame = None
        self.latest_depth_frame = None

        # 保存最后点击位置和计算结果
        self.clicked_pixel = None
        self.last_camera_point = None
        self.last_base_point = None
        self.last_depth = None

        # 发布点击目标坐标
        self.point_pub = self.create_publisher(
            PointStamped,
            'clicked_target_point',
            10
        )

        # 发布 RViz Marker
        self.marker_pub = self.create_publisher(
            Marker,
            'clicked_target_marker',
            10
        )

        self.get_logger().info(
            '正在启动 RealSense D455...'
        )

        self.pipeline = rs.pipeline()
        config = rs.config()

        config.enable_stream(
            rs.stream.color,
            640,
            480,
            rs.format.bgr8,
            30
        )

        config.enable_stream(
            rs.stream.depth,
            640,
            480,
            rs.format.z16,
            30
        )

        self.pipeline.start(config)

        # 将深度图对齐到彩色图
        self.align = rs.align(rs.stream.color)

        # 创建OpenCV窗口，并绑定鼠标回调
        self.window_name = 'D455 Click Measure'

        cv2.namedWindow(
            self.window_name,
            cv2.WINDOW_AUTOSIZE
        )

        cv2.setMouseCallback(
            self.window_name,
            self.mouse_callback
        )

        self.get_logger().info(
            '启动完成！请在相机窗口中用鼠标左键点击木块顶面。'
        )

        self.get_logger().info(
            '点击坐标发布到 /clicked_target_point，'
            'RViz Marker发布到 /clicked_target_marker'
        )

        self.timer = self.create_timer(
            0.03,
            self.process_camera
        )

    def camera_to_base(self, point_camera):
        """相机坐标转换到机械臂底座坐标。"""

        point_camera_h = np.array([
            float(point_camera[0]),
            float(point_camera[1]),
            float(point_camera[2]),
            1.0
        ], dtype=np.float64)

        point_base_h = (
            self.T_base_camera @ point_camera_h
        )

        return point_base_h[:3]

    def get_median_depth(
        self,
        depth_frame,
        center_x,
        center_y,
        radius=5
    ):
        """
        读取点击位置周围区域的深度中位数。

        radius=5 表示使用约11×11像素区域，
        比只读取单个像素稳定。
        """

        width = depth_frame.get_width()
        height = depth_frame.get_height()

        start_x = max(0, center_x - radius)
        end_x = min(width - 1, center_x + radius)

        start_y = max(0, center_y - radius)
        end_y = min(height - 1, center_y + radius)

        valid_depths = []

        for pixel_y in range(start_y, end_y + 1):
            for pixel_x in range(start_x, end_x + 1):

                depth = depth_frame.get_distance(
                    pixel_x,
                    pixel_y
                )

                if self.min_depth < depth < self.max_depth:
                    valid_depths.append(depth)

        if len(valid_depths) < 5:
            return None

        valid_depths = np.asarray(
            valid_depths,
            dtype=np.float64
        )

        return float(np.median(valid_depths))

    def mouse_callback(
        self,
        event,
        pixel_x,
        pixel_y,
        flags,
        parameter
    ):
        """处理鼠标左键点击。"""

        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if (
            self.latest_depth_frame is None
            or self.latest_color_frame is None
        ):
            self.get_logger().warning(
                '当前还没有有效相机画面'
            )
            return

        depth = self.get_median_depth(
            self.latest_depth_frame,
            pixel_x,
            pixel_y,
            radius=5
        )

        if depth is None:
            self.get_logger().warning(
                f'点击位置({pixel_x}, {pixel_y})没有有效深度'
            )
            return

        # 深度图已经对齐到彩色图，所以使用彩色相机内参
        color_intrinsics = (
            self.latest_color_frame
            .profile
            .as_video_stream_profile()
            .get_intrinsics()
        )

        point_camera = (
            rs.rs2_deproject_pixel_to_point(
                color_intrinsics,
                [pixel_x, pixel_y],
                depth
            )
        )

        point_camera = np.asarray(
            point_camera,
            dtype=np.float64
        )

        point_base = self.camera_to_base(
            point_camera
        )

        if not np.all(np.isfinite(point_base)):
            self.get_logger().warning(
                '转换后的坐标包含无效数值'
            )
            return

        self.clicked_pixel = (
            pixel_x,
            pixel_y
        )

        self.last_depth = depth
        self.last_camera_point = point_camera
        self.last_base_point = point_base

        self.publish_point(point_base)
        self.publish_marker(point_base)

        self.get_logger().info(
            '\n'
            '================ 点击测量结果 ================\n'
            f'点击像素：u={pixel_x}, v={pixel_y}\n'
            f'深度：{depth:.4f} m\n'
            f'相机坐标：'
            f'X={point_camera[0]:.4f}, '
            f'Y={point_camera[1]:.4f}, '
            f'Z={point_camera[2]:.4f} m\n'
            f'底座坐标：'
            f'X={point_base[0]:.4f}, '
            f'Y={point_base[1]:.4f}, '
            f'Z={point_base[2]:.4f} m\n'
            '=============================================='
        )

    def publish_point(self, point_base):
        """发布点击位置坐标。"""

        message = PointStamped()

        message.header.frame_id = self.base_frame
        message.header.stamp = (
            self.get_clock().now().to_msg()
        )

        message.point.x = float(point_base[0])
        message.point.y = float(point_base[1])
        message.point.z = float(point_base[2])

        self.point_pub.publish(message)

    def publish_marker(self, point_base):
        """在RViz中发布红色球体。"""

        marker = Marker()

        marker.header.frame_id = self.base_frame
        marker.header.stamp = (
            self.get_clock().now().to_msg()
        )

        marker.ns = 'clicked_target'
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = float(point_base[0])
        marker.pose.position.y = float(point_base[1])
        marker.pose.position.z = float(point_base[2])
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.05
        marker.scale.y = 0.05
        marker.scale.z = 0.05

        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        self.marker_pub.publish(marker)

    def draw_information(self, image):
        """在相机画面上显示点击位置和坐标。"""

        if self.clicked_pixel is None:
            cv2.putText(
                image,
                'Left click on the wooden block',
                (20, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2
            )
            return

        pixel_x, pixel_y = self.clicked_pixel

        # 点击位置十字
        cv2.drawMarker(
            image,
            (pixel_x, pixel_y),
            (0, 0, 255),
            cv2.MARKER_CROSS,
            24,
            2
        )

        if self.last_base_point is None:
            return

        point = self.last_base_point

        text1 = (
            f'Base X:{point[0]:.3f} '
            f'Y:{point[1]:.3f} '
            f'Z:{point[2]:.3f}'
        )

        text2 = (
            f'Depth:{self.last_depth:.3f}m'
        )

        cv2.putText(
            image,
            text1,
            (20, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2
        )

        cv2.putText(
            image,
            text2,
            (20, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 255),
            2
        )

    def process_camera(self):
        """读取并显示RealSense画面。"""

        try:
            frames = self.pipeline.wait_for_frames(
                timeout_ms=1000
            )

            aligned_frames = self.align.process(frames)

            color_frame = (
                aligned_frames.get_color_frame()
            )

            depth_frame = (
                aligned_frames.get_depth_frame()
            )

            if not color_frame or not depth_frame:
                return

            self.latest_color_frame = color_frame
            self.latest_depth_frame = depth_frame

            color_image = np.asanyarray(
                color_frame.get_data()
            ).copy()

            self.latest_color_image = color_image

            self.draw_information(color_image)

            cv2.imshow(
                self.window_name,
                color_image
            )

            key = cv2.waitKey(1) & 0xFF

            if key == ord('q'):
                self.get_logger().info(
                    '按下Q键，正在退出'
                )
                rclpy.shutdown()

        except RuntimeError as error:
            self.get_logger().warning(
                f'RealSense错误：{error}'
            )

        except Exception as error:
            self.get_logger().error(
                f'处理异常：{type(error).__name__}: {error}'
            )

    def shutdown(self):
        try:
            self.pipeline.stop()
        except Exception:
            pass

        cv2.destroyAllWindows()


def main(args=None):
    rclpy.init(args=args)

    node = ClickMeasureNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.shutdown()
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
