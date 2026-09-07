#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""YOLO target localisation using the ROS RealSense image topics.

The RealSense driver owns the camera. This node only subscribes to the
colour image, aligned depth image and colour CameraInfo, then transforms the
detected point from ``camera_color_optical_frame`` into ``base_link``.
"""

from pathlib import Path

import cv2
import message_filters
import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from ultralytics import YOLO
from visualization_msgs.msg import Marker


def image_to_bgr(msg: Image) -> np.ndarray:
    """Decode the colour encodings emitted by the ROS RealSense driver."""

    encoding = msg.encoding.lower()
    channels = {
        "bgr8": 3,
        "rgb8": 3,
        "bgra8": 4,
        "rgba8": 4,
        "mono8": 1,
    }.get(encoding)
    if channels is None:
        raise ValueError(f"unsupported colour encoding {msg.encoding!r}")

    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    row_bytes = width * channels
    if width <= 0 or height <= 0 or step < row_bytes:
        raise ValueError(f"invalid colour image dimensions/step: {width}x{height}, {step}")

    raw = np.frombuffer(msg.data, dtype=np.uint8)
    required = height * step
    if raw.size < required:
        raise ValueError("colour image data is shorter than height*step")

    rows = raw[:required].reshape(height, step)[:, :row_bytes]
    image = rows.reshape(height, width, channels)
    if encoding == "bgr8":
        return image.copy()
    if encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)


def depth_to_meters(msg: Image, depth_scale: float) -> np.ndarray:
    """Decode a ROS depth image into a float32 array whose unit is metres."""

    encoding = msg.encoding.lower()
    if encoding in ("16uc1", "mono16"):
        dtype = np.dtype(">u2" if msg.is_bigendian else "<u2")
        itemsize = 2
        scale = float(depth_scale)
    elif encoding == "32fc1":
        dtype = np.dtype(">f4" if msg.is_bigendian else "<f4")
        itemsize = 4
        scale = 1.0
    else:
        raise ValueError(f"unsupported depth encoding {msg.encoding!r}")

    width = int(msg.width)
    height = int(msg.height)
    step = int(msg.step)
    row_bytes = width * itemsize
    if width <= 0 or height <= 0 or step < row_bytes or step % itemsize:
        raise ValueError(f"invalid depth image dimensions/step: {width}x{height}, {step}")

    raw = np.frombuffer(msg.data, dtype=dtype)
    elements_per_row = step // itemsize
    required_elements = height * elements_per_row
    if raw.size < required_elements:
        raise ValueError("depth image data is shorter than height*step")

    rows = raw[:required_elements].reshape(height, elements_per_row)[:, :width]
    depth = rows.astype(np.float32, copy=False) * scale
    return np.asarray(depth, dtype=np.float32)


class VisionGraspNode(Node):
    """Detect ``tree`` targets and publish their calibrated 3-D positions."""

    def __init__(self) -> None:
        super().__init__("vision_grasp_node")

        self.declare_parameter("model_path", "best.pt")
        self.declare_parameter("target_class", "tree")
        self.declare_parameter("min_confidence", 0.50)

        # These are the topics from the single realsense2_camera instance.
        self.declare_parameter("color_image_topic", "/camera/camera/color/image_raw")
        self.declare_parameter(
            "depth_image_topic",
            "/camera/camera/aligned_depth_to_color/image_raw",
        )
        self.declare_parameter("camera_info_topic", "/camera/camera/color/camera_info")
        self.declare_parameter("depth_scale", 0.001)
        self.declare_parameter("sync_slop", 0.10)
        self.declare_parameter("sync_queue_size", 10)
        self.declare_parameter("camera_serial", "")

        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("camera_frame", "camera_color_optical_frame")
        self.declare_parameter("min_depth", 0.15)
        self.declare_parameter("max_depth", 2.0)
        self.declare_parameter("depth_outlier_threshold", 0.08)
        self.declare_parameter("min_depth_samples", 5)
        self.declare_parameter("filter_alpha", 0.30)
        self.declare_parameter("show_window", True)

        self.color_image_topic = str(self.get_parameter("color_image_topic").value)
        self.depth_image_topic = str(self.get_parameter("depth_image_topic").value)
        self.camera_info_topic = str(self.get_parameter("camera_info_topic").value)
        self.depth_scale = float(self.get_parameter("depth_scale").value)
        self.sync_slop = float(self.get_parameter("sync_slop").value)
        self.sync_queue_size = int(self.get_parameter("sync_queue_size").value)
        self.camera_serial = str(self.get_parameter("camera_serial").value).strip()
        self.base_frame = str(self.get_parameter("base_frame").value)
        self.camera_frame = str(self.get_parameter("camera_frame").value)
        self.min_depth = float(self.get_parameter("min_depth").value)
        self.max_depth = float(self.get_parameter("max_depth").value)
        self.depth_outlier_threshold = float(
            self.get_parameter("depth_outlier_threshold").value
        )
        self.min_depth_samples = int(self.get_parameter("min_depth_samples").value)
        self.filter_alpha = float(self.get_parameter("filter_alpha").value)
        self.show_window = bool(self.get_parameter("show_window").value)

        if self.depth_scale <= 0.0:
            raise ValueError("depth_scale must be positive")
        if self.sync_slop <= 0.0:
            raise ValueError("sync_slop must be positive")
        if self.sync_queue_size < 2:
            raise ValueError("sync_queue_size must be at least 2")
        if not 0.0 < self.filter_alpha <= 1.0:
            raise ValueError("filter_alpha must be in the interval (0, 1]")
        if self.min_depth <= 0.0 or self.max_depth <= self.min_depth:
            raise ValueError("depth range is invalid")
        if self.min_depth_samples < 1:
            raise ValueError("min_depth_samples must be positive")

        requested_model_path = Path(
            str(self.get_parameter("model_path").value)
        ).expanduser()
        if not requested_model_path.is_absolute():
            candidates = [
                Path.cwd() / requested_model_path,
                Path(__file__).resolve().parent / requested_model_path,
            ]
            requested_model_path = next(
                (path for path in candidates if path.exists()), candidates[0]
            )
        self.model_path = str(requested_model_path)
        self.target_class = str(self.get_parameter("target_class").value).strip()
        self.min_confidence = float(self.get_parameter("min_confidence").value)
        if not self.target_class:
            raise ValueError("target_class must not be empty")
        if not 0.0 <= self.min_confidence <= 1.0:
            raise ValueError("min_confidence must be in the interval [0, 1]")

        self.filtered_point: np.ndarray | None = None
        self.last_target_stamp_ns: int | None = None
        self.filter_reset_interval_ns = 1_000_000_000
        self.last_tf_warning_ns = 0
        self.last_sync_warning_ns = 0
        self.processing = False
        self.window_disabled = False

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.marker_pub = self.create_publisher(Marker, "yolo_target_marker", 10)
        self.target_pub = self.create_publisher(PointStamped, "yolo_target_point", 10)

        self.get_logger().info(f"loading YOLO model: {self.model_path}")
        self.model = YOLO(self.model_path)

        # message_filters preserves the image/depth/camera-info timestamp
        # relationship. The depth topic must already be aligned to colour.
        self.color_sub = message_filters.Subscriber(
            self,
            Image,
            self.color_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.depth_sub = message_filters.Subscriber(
            self,
            Image,
            self.depth_image_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.camera_info_sub = message_filters.Subscriber(
            self,
            CameraInfo,
            self.camera_info_topic,
            qos_profile=qos_profile_sensor_data,
        )
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [self.color_sub, self.depth_sub, self.camera_info_sub],
            queue_size=self.sync_queue_size,
            slop=self.sync_slop,
        )
        self.sync.registerCallback(self.synced_callback)

        self.get_logger().info(
            "ready: "
            f"image={self.color_image_topic} "
            f"depth={self.depth_image_topic} "
            f"camera_info={self.camera_info_topic} "
            f"sync_slop={self.sync_slop:.3f}s "
            f"target={self.target_class!r} "
            f"serial={self.camera_serial or 'driver default'}"
        )
        self.get_logger().info(
            f"TF lookup: {self.base_frame} <- {self.camera_frame}; "
            "the camera is opened by realsense2_camera, not this node"
        )

    def _warn_throttled(
        self,
        key: str,
        message: str,
        interval_ns: int = 2_000_000_000,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        attribute = f"last_{key}_warning_ns"
        previous_ns = int(getattr(self, attribute, 0))
        if now_ns - previous_ns >= interval_ns:
            self.get_logger().warning(message)
            setattr(self, attribute, now_ns)

    @staticmethod
    def _stamp_ns(stamp) -> int:
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)

    def camera_to_base(self, point_camera: np.ndarray, stamp) -> np.ndarray | None:
        """Transform a point using the calibrated base-to-camera TF."""

        query_time = Time.from_msg(stamp)
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                query_time,
                timeout=Duration(seconds=0.10),
            )
        except TransformException as error:
            # A static transform has no meaningful image timestamp. The
            # latest fallback also handles a robot TF publisher that lags one
            # camera frame, while still preferring the synchronized timestamp.
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.base_frame,
                    self.camera_frame,
                    Time(),
                    timeout=Duration(seconds=0.02),
                )
            except TransformException:
                self._warn_throttled(
                    "tf",
                    f"cannot get TF {self.base_frame} <- {self.camera_frame}: {error}",
                )
                return None

        translation = transform.transform.translation
        quaternion = transform.transform.rotation
        qx = float(quaternion.x)
        qy = float(quaternion.y)
        qz = float(quaternion.z)
        qw = float(quaternion.w)
        norm = float(np.linalg.norm([qx, qy, qz, qw]))
        if norm < 1e-12:
            self.get_logger().error("TF quaternion is invalid")
            return None
        qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm

        rotation = np.array(
            [
                [
                    1.0 - 2.0 * (qy * qy + qz * qz),
                    2.0 * (qx * qy - qz * qw),
                    2.0 * (qx * qz + qy * qw),
                ],
                [
                    2.0 * (qx * qy + qz * qw),
                    1.0 - 2.0 * (qx * qx + qz * qz),
                    2.0 * (qy * qz - qx * qw),
                ],
                [
                    2.0 * (qx * qz - qy * qw),
                    2.0 * (qy * qz + qx * qw),
                    1.0 - 2.0 * (qx * qx + qy * qy),
                ],
            ],
            dtype=np.float64,
        )
        translation_vector = np.array(
            [float(translation.x), float(translation.y), float(translation.z)],
            dtype=np.float64,
        )
        point_camera = np.asarray(point_camera, dtype=np.float64).reshape(3)
        point_base = rotation @ point_camera + translation_vector
        return point_base if np.all(np.isfinite(point_base)) else None

    def get_stable_depth(
        self,
        depth_meters: np.ndarray,
        x1: int,
        y1: int,
        x2: int,
        y2: int,
    ) -> float | None:
        """Return a robust median depth from the centre of a detection box."""

        if depth_meters.ndim != 2:
            return None
        height, width = depth_meters.shape
        x1 = max(0, min(int(x1), width - 1))
        x2 = max(0, min(int(x2), width - 1))
        y1 = max(0, min(int(y1), height - 1))
        y2 = max(0, min(int(y2), height - 1))
        if x2 < x1 or y2 < y1:
            return None

        box_width = x2 - x1
        box_height = y2 - y1
        roi_x1 = int(x1 + box_width * 0.35)
        roi_x2 = int(x2 - box_width * 0.35)
        roi_y1 = int(y1 + box_height * 0.30)
        roi_y2 = int(y2 - box_height * 0.30)
        if roi_x2 < roi_x1:
            roi_x1 = roi_x2 = (x1 + x2) // 2
        if roi_y2 < roi_y1:
            roi_y1 = roi_y2 = (y1 + y2) // 2

        roi = depth_meters[roi_y1 : roi_y2 + 1 : 2, roi_x1 : roi_x2 + 1 : 2]
        valid = roi[np.isfinite(roi)]
        valid = valid[(valid > self.min_depth) & (valid < self.max_depth)]
        if valid.size < self.min_depth_samples:
            return None

        median = float(np.median(valid))
        inliers = valid[np.abs(valid - median) <= self.depth_outlier_threshold]
        if inliers.size >= max(3, self.min_depth_samples // 2):
            return float(np.median(inliers))
        return median

    @staticmethod
    def camera_point_from_pixel(
        center_x: int,
        center_y: int,
        depth: float,
        camera_info: CameraInfo,
    ) -> np.ndarray | None:
        """Back-project a colour pixel with the synchronized CameraInfo."""

        k = np.asarray(camera_info.k, dtype=np.float64).reshape(3, 3)
        if not np.all(np.isfinite(k)) or k[0, 0] <= 0.0 or k[1, 1] <= 0.0:
            return None
        pixel = np.array([[[float(center_x), float(center_y)]]], dtype=np.float64)
        distortion = np.asarray(camera_info.d, dtype=np.float64)
        try:
            if camera_info.distortion_model == "equidistant" and distortion.size >= 4:
                normalized = cv2.fisheye.undistortPoints(
                    pixel, k, distortion[:4].reshape(4, 1)
                )
            else:
                normalized = cv2.undistortPoints(
                    pixel, k, distortion if distortion.size else None
                )
        except cv2.error:
            return None

        x_normalized, y_normalized = normalized[0, 0]
        point = np.array(
            [x_normalized * depth, y_normalized * depth, depth],
            dtype=np.float64,
        )
        return point if np.all(np.isfinite(point)) else None

    def smooth_point(self, point: np.ndarray, stamp) -> np.ndarray:
        point = np.asarray(point, dtype=np.float64).reshape(3)
        stamp_ns = self._stamp_ns(stamp)
        if stamp_ns <= 0:
            stamp_ns = self.get_clock().now().nanoseconds
        if (
            self.last_target_stamp_ns is not None
            and stamp_ns - self.last_target_stamp_ns > self.filter_reset_interval_ns
        ):
            self.filtered_point = None
        self.last_target_stamp_ns = stamp_ns

        if self.filtered_point is None:
            self.filtered_point = point.copy()
        else:
            self.filtered_point = (
                self.filter_alpha * point
                + (1.0 - self.filter_alpha) * self.filtered_point
            )
        return self.filtered_point.copy()

    def find_best_target(self, results):
        best_target = None
        best_confidence = 0.0
        names = self.model.names
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                class_id = int(box.cls[0].item())
                class_name = str(names[class_id])
                confidence = float(box.conf[0].item())
                if class_name != self.target_class or confidence < self.min_confidence:
                    continue
                if confidence <= best_confidence:
                    continue
                coordinates = (
                    box.xyxy[0].detach().cpu().numpy().astype(int).tolist()
                )
                best_target = {
                    "x1": coordinates[0],
                    "y1": coordinates[1],
                    "x2": coordinates[2],
                    "y2": coordinates[3],
                    "confidence": confidence,
                }
                best_confidence = confidence
        return best_target

    def publish_target(self, point_base: np.ndarray, stamp) -> None:
        message = PointStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = stamp
        message.point.x = float(point_base[0])
        message.point.y = float(point_base[1])
        message.point.z = float(point_base[2])
        self.target_pub.publish(message)

    def publish_marker(self, point_base: np.ndarray, stamp) -> None:
        marker = Marker()
        marker.header.frame_id = self.base_frame
        marker.header.stamp = stamp
        marker.ns = "yolo_target"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(point_base[0])
        marker.pose.position.y = float(point_base[1])
        marker.pose.position.z = float(point_base[2])
        marker.pose.orientation.w = 1.0
        marker.scale.x = marker.scale.y = marker.scale.z = 0.06
        marker.color.r = 1.0
        marker.color.a = 0.9
        marker.lifetime.nanosec = 300_000_000
        self.marker_pub.publish(marker)

    def handle_target(
        self,
        color_image: np.ndarray,
        depth_meters: np.ndarray,
        camera_info: CameraInfo,
        target: dict,
        stamp,
    ) -> None:
        image_height, image_width = color_image.shape[:2]
        x1 = max(0, min(int(target["x1"]), image_width - 1))
        y1 = max(0, min(int(target["y1"]), image_height - 1))
        x2 = max(0, min(int(target["x2"]), image_width - 1))
        y2 = max(0, min(int(target["y2"]), image_height - 1))
        if x2 < x1 or y2 < y1:
            return

        center_x = (x1 + x2) // 2
        center_y = (y1 + y2) // 2
        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
        cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)

        depth = self.get_stable_depth(depth_meters, x1, y1, x2, y2)
        if depth is None:
            cv2.putText(
                color_image,
                "Invalid depth",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
            return

        point_camera = self.camera_point_from_pixel(
            center_x, center_y, depth, camera_info
        )
        if point_camera is None:
            cv2.putText(
                color_image,
                "Invalid camera info",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 0, 255),
                2,
            )
            return

        point_base = self.camera_to_base(point_camera, stamp)
        if point_base is None:
            cv2.putText(
                color_image,
                "Waiting for TF",
                (x1, max(20, y1 - 10)),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 0, 255),
                2,
            )
            return

        point_base = self.smooth_point(point_base, stamp)
        self.publish_target(point_base, stamp)
        self.publish_marker(point_base, stamp)

        confidence = float(target["confidence"])
        cv2.putText(
            color_image,
            f"Base X:{point_base[0]:.3f} Y:{point_base[1]:.3f} "
            f"Z:{point_base[2]:.3f} C:{confidence:.2f}",
            (x1, max(20, y1 - 10)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            2,
        )
        cv2.putText(
            color_image,
            f"Camera X:{point_camera[0]:.3f} Y:{point_camera[1]:.3f} "
            f"Z:{point_camera[2]:.3f} D:{depth:.3f}m",
            (x1, min(image_height - 10, y2 + 20)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.45,
            (255, 255, 0),
            1,
        )

    def _show_image(self, image: np.ndarray) -> None:
        if not self.show_window or self.window_disabled:
            return
        try:
            cv2.imshow("ROS2 Vision Grasp", image)
            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                self.get_logger().info("q pressed; closing image window")
                self.show_window = False
        except cv2.error as error:
            self.window_disabled = True
            self.get_logger().warning(f"OpenCV display disabled: {error}")

    def synced_callback(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info: CameraInfo,
    ) -> None:
        if self.processing:
            return
        self.processing = True
        try:
            color_image = image_to_bgr(color_msg)
            depth_meters = depth_to_meters(depth_msg, self.depth_scale)
            reported_frame = camera_info.header.frame_id.strip().lstrip("/")
            expected_frame = self.camera_frame.strip().lstrip("/")
            if reported_frame and reported_frame != expected_frame:
                self._warn_throttled(
                    "sync",
                    f"CameraInfo frame is {reported_frame!r}, expected "
                    f"{expected_frame!r}; refusing to apply the wrong calibration TF",
                )
                self._show_image(color_image)
                return
            if depth_meters.shape != color_image.shape[:2]:
                self._warn_throttled(
                    "sync",
                    "colour/depth dimensions differ; use the aligned depth topic "
                    f"({color_image.shape[1]}x{color_image.shape[0]} vs "
                    f"{depth_meters.shape[1]}x{depth_meters.shape[0]})",
                )
                self._show_image(color_image)
                return

            results = self.model(color_image, verbose=False)
            target = self.find_best_target(results)
            if target is not None:
                self.handle_target(
                    color_image,
                    depth_meters,
                    camera_info,
                    target,
                    color_msg.header.stamp,
                )
            self._show_image(color_image)
        except Exception as error:  # keep one malformed frame from killing ROS
            self._warn_throttled(
                "sync",
                f"vision callback error: {type(error).__name__}: {error}",
            )
        finally:
            self.processing = False

    def shutdown(self) -> None:
        """Close the optional OpenCV window; the ROS camera remains driver-owned."""

        try:
            cv2.destroyAllWindows()
        except cv2.error:
            pass


def main(args=None) -> None:
    rclpy.init(args=args)
    node = None
    try:
        node = VisionGraspNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
