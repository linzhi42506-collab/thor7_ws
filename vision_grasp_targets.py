#!/usr/bin/env python3
"""Generate staged grasp poses from the calibrated YOLO target point.

This node is deliberately plan-only: it publishes poses and markers but never
commands the robot.  The current tool0 orientation is reused so the first
test does not introduce an unknown wrist rotation.
"""

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray


class GraspTargetNode(Node):
    def __init__(self):
        super().__init__("vision_grasp_target_node")
        self.declare_parameter("input_topic", "/yolo_target_point")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tool_frame", "tool0")
        self.declare_parameter("pregrasp_clearance", 0.050)
        self.declare_parameter("near_clearance", 0.005)
        self.declare_parameter("grasp_descent", 0.013)
        self.declare_parameter("publish_rate", 10.0)

        self.base_frame = str(self.get_parameter("base_frame").value)
        self.tool_frame = str(self.get_parameter("tool_frame").value)
        self.pregrasp = float(self.get_parameter("pregrasp_clearance").value)
        self.near = float(self.get_parameter("near_clearance").value)
        self.descent = float(self.get_parameter("grasp_descent").value)
        if self.pregrasp <= self.near or self.near < 0.0 or self.descent < 0.0:
            raise ValueError("invalid grasp offsets")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.latest_point = None
        self.point_sub = self.create_subscription(
            PointStamped,
            str(self.get_parameter("input_topic").value),
            self.point_callback,
            qos_profile_sensor_data,
        )
        self.pose_pubs = {
            "pregrasp": self.create_publisher(PoseStamped, "grasp/pregrasp", 10),
            "near": self.create_publisher(PoseStamped, "grasp/near", 10),
            "grasp": self.create_publisher(PoseStamped, "grasp/final", 10),
        }
        self.marker_pub = self.create_publisher(MarkerArray, "grasp/targets", 10)
        period = 1.0 / float(self.get_parameter("publish_rate").value)
        self.timer = self.create_timer(period, self.publish_targets)
        self.get_logger().info(
            f"plan-only targets: pregrasp +{self.pregrasp:.3f} m, "
            f"near +{self.near:.3f} m, final -{self.descent:.3f} m"
        )

    def point_callback(self, msg: PointStamped):
        if msg.header.frame_id.strip("/") != self.base_frame.strip("/"):
            self.get_logger().warning(
                f"expected point frame {self.base_frame}, got {msg.header.frame_id}"
            )
            return
        self.latest_point = msg

    def current_orientation(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tool_frame, Time(), timeout=Duration(seconds=0.05)
            )
        except TransformException as error:
            self.get_logger().warning(f"waiting for {self.base_frame} <- {self.tool_frame}: {error}")
            return None
        return tf.transform.rotation

    def make_pose(self, point, dz, orientation, stamp):
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = stamp
        pose.pose.position.x = float(point.x)
        pose.pose.position.y = float(point.y)
        pose.pose.position.z = float(point.z + dz)
        pose.pose.orientation = orientation
        return pose

    def make_marker(self, pose, marker_id, color):
        marker = Marker()
        marker.header = pose.header
        marker.ns = "grasp_targets"
        marker.id = marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose = pose.pose
        marker.scale.x = marker.scale.y = marker.scale.z = 0.025
        marker.color.r, marker.color.g, marker.color.b, marker.color.a = (*color, 0.9)
        marker.lifetime.sec = 1
        return marker

    def publish_targets(self):
        if self.latest_point is None:
            return
        orientation = self.current_orientation()
        if orientation is None:
            return
        point = self.latest_point.point
        stamp = self.latest_point.header.stamp
        poses = {
            "pregrasp": self.make_pose(point, self.pregrasp, orientation, stamp),
            "near": self.make_pose(point, self.near, orientation, stamp),
            "grasp": self.make_pose(point, -self.descent, orientation, stamp),
        }
        for name, pose in poses.items():
            self.pose_pubs[name].publish(pose)
        self.marker_pub.publish(MarkerArray(markers=[
            self.make_marker(poses["pregrasp"], 0, (0.1, 0.8, 0.1)),
            self.make_marker(poses["near"], 1, (1.0, 0.8, 0.0)),
            self.make_marker(poses["grasp"], 2, (1.0, 0.1, 0.1)),
        ]))


def main(args=None):
    rclpy.init(args=args)
    node = GraspTargetNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
