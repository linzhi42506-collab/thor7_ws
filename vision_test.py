import pyrealsense2 as rs
import numpy as np
import cv2
from ultralytics import YOLO

# 1. 加载官方预训练的 YOLO 模型 (首次运行会自动下载 yolov8n.pt，很小很快)
print("正在加载 YOLO 模型...")
model = YOLO('yolo11n.pt')

# 2. 配置并启动 RealSense 相机
print("正在连接 RealSense 相机...")
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

try:
    profile = pipeline.start(config)
except Exception as e:
    print(f"相机启动失败，请检查 USB 连接: {e}")
    exit()

# 深度图对齐到彩色图 (必须要有，否则像素位置和深度对不上)
align_to = rs.stream.color
align = rs.align(align_to)

# 获取相机的内参 (用于将 2D 像素转换为 3D 坐标)
depth_profile = rs.video_stream_profile(profile.get_stream(rs.stream.depth))
depth_intrinsics = depth_profile.get_intrinsics()

print("✅ 系统就绪！开始获取画面... (在画面窗口按 'q' 键退出)")

try:
    while True:
        # 等待一对连贯的彩色和深度帧
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        
        color_frame = aligned_frames.get_color_frame()
        depth_frame = aligned_frames.get_depth_frame()
        
        if not color_frame or not depth_frame:
            continue
            
        # 转换为 numpy 数组供 OpenCV 和 YOLO 使用
        color_image = np.asanyarray(color_frame.get_data())
        
        # 3. 让 YOLO 进行目标检测
        results = model(color_image, verbose=False)
        
        # 4. 处理检测结果
        for r in results:
            boxes = r.boxes
            for box in boxes:
                cls_id = int(box.cls[0])
                class_name = model.names[cls_id]
                
                # 为了测试，我们只筛选常见物品：bottle(瓶子), cup(杯子), mouse(鼠标), cell phone(手机)
                if class_name in ['bottle', 'cup', 'mouse', 'cell phone']:
                    # 获取边界框坐标
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    
                    # 计算物体中心点
                    cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                    
                    # 获取该中心点距离相机的深度 (单位：米)
                    depth = depth_frame.get_distance(cx, cy)
                    
                    # 如果深度有效，则计算 3D 坐标
                    if depth > 0 and depth < 2.0: # 假设抓取距离在2米以内
                        # 计算相机坐标系下的 3D 坐标 (P_cam)
                        spatial_point = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth)
                        x, y, z = spatial_point
                        
                        # 在画面上画框和打印坐标
                        cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                        cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                        label = f"{class_name}: X={x:.2f}m, Y={y:.2f}m, Z={z:.2f}m"
                        cv2.putText(color_image, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                        
                        # 在终端打印，方便你观察
                        print(f"������ 发现 [{class_name}] -> 相机坐标: X={x:.3f}, Y={y:.3f}, Z={z:.3f} 米")

        # 显示实时画面
        cv2.imshow('RealSense + YOLO Vision Test', color_image)
        
        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
            
finally:
    pipeline.stop()
    cv2.destroyAllWindows()