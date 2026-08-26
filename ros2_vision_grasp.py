import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

class VisionGraspNode(Node):
    def __init__(self):
        super().__init__('vision_grasp_node')
        
        # 1. 创建 RViz 标记发布者 (话题名为 /yolo_target_marker)
        self.marker_pub = self.create_publisher(Marker, 'yolo_target_marker', 10)
        
        self.get_logger().info("⏳ 正在加载 YOLO11 模型...")
        self.model = YOLO('yolo11n.pt')
        self.target_class = 'bottle' # 你可以改成 'cup', 'mouse' 等

        self.get_logger().info("������ 正在启动 RealSense 相机...")
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.profile = self.pipeline.start(config)
        self.align = rs.align(rs.stream.color)
        self.depth_intrinsics = rs.video_stream_profile(self.profile.get_stream(rs.stream.depth)).get_intrinsics()

        self.get_logger().info("✅ 视觉节点就绪！请在 RViz 中添加 Marker 显示。")
        
        # 设置定时器，按 20 FPS 处理画面
        self.timer = self.create_timer(0.05, self.process_vision)

    def cam_to_base_transform(self, p_cam):
        """
        ������ 核心手眼标定转换 ������
        当前为测试占位公式，需要你后续填入真实的 T_cam2base 平移和旋转
        """
        x_cam, y_cam, z_cam = p_cam
        # 假设相机固定在底座前方 0.5m，高度 0.2m (请根据实际修改)
        x_base = z_cam + 0.5   
        y_base = -x_cam        
        z_base = -y_cam + 0.2  
        return [x_base, y_base, z_base]

    def publish_rviz_marker(self, p_base):
        """将转换后的底座坐标发布到 RViz 中进行 3D 可视化"""
        marker = Marker()
        marker.header.frame_id = "base_link"  # Thor7 机械臂的默认基座坐标系名
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "yolo_target"
        marker.id = 0
        marker.type = Marker.CYLINDER # 用一个圆柱体代表要抓的瓶子/水杯
        marker.action = Marker.ADD

        # 设置物体在 RViz 里的 3D 坐标
        marker.pose.position.x = float(p_base[0])
        marker.pose.position.y = float(p_base[1])
        marker.pose.position.z = float(p_base[2])
        marker.pose.orientation.w = 1.0

        # 设置圆柱体大小 (半径 4cm, 高 12cm)
        marker.scale.x = 0.08
        marker.scale.y = 0.08
        marker.scale.z = 0.12

        # 设置颜色 (半透明红色)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.6

        self.marker_pub.publish(marker)

    def process_vision(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame: return
        
        color_image = np.asanyarray(color_frame.get_data())
        results = self.model(color_image, verbose=False)
        
        for r in results:
            for box in r.boxes:
                if self.model.names[int(box.cls[0])] == self.target_class:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    depth = depth_frame.get_distance(cx, cy)
                    
                    if 0 < depth < 2.0:
                        # 1. 算相机 3D 坐标 P_cam
                        p_cam = rs.rs2_deproject_pixel_to_point(self.depth_intrinsics, [cx, cy], depth)
                        # 2. 转成机械臂底座坐标 P_base
                        p_base = self.cam_to_base_transform(p_cam)
                        # 3. 把这个坐标作为 3D 实体发给 RViz
                        self.publish_rviz_marker(p_base)
                        
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                        cv2.putText(color_image, f"X:{p_base[0]:.2f} Y:{p_base[1]:.2f} Z:{p_base[2]:.2f}", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        break

        cv2.imshow('ROS2 Vision Grasp', color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = VisionGraspNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pipeline.stop()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()