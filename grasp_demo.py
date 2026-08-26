import pyrealsense2 as rs
import numpy as np
import cv2
import time
from ultralytics import YOLO

# ==========================================
# ⚙️ 配置区域 (需要你后续根据实际情况修改)
# ==========================================
TARGET_CLASS = 'bottle'  # 你想抓的物体类别 (比如 'bottle', 'cup')
Z_OFFSET = 0.10          # 抓取准备高度：悬停在物体上方 10cm 处

# 假定的手眼标定矩阵 (相机坐标系到底座坐标系的转换)
# ⚠️ 注意：你需要把这里替换成你真实的 T_cam2base 平移和旋转
def transform_cam_to_base(p_cam):
    x_cam, y_cam, z_cam = p_cam
    # 这里是一个演示转换（比如相机在底座前方 0.5m，高度 0.2m 的位置）
    x_base = z_cam + 0.5   
    y_base = -x_cam        
    z_base = -y_cam + 0.2  
    return [x_base, y_base, z_base]

# 假定的机械臂和夹爪控制接口
def move_arm_to(x, y, z):
    print(f"������ [机械臂] 正在移动至世界坐标: X={x:.3f}, Y={y:.3f}, Z={z:.3f}")
    time.sleep(2) # 模拟移动时间

def control_gripper(action):
    if action == "open":
        print("������ [夹爪] 张开")
    elif action == "close":
        print("✊ [夹爪] 闭合")
    time.sleep(1)

# ==========================================
# ������ 主程序逻辑
# ==========================================
def main():
    print("⏳ 正在加载 YOLO11 模型...")
    model = YOLO('yolo11n.pt')

    print("������ 正在启动 RealSense 相机...")
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    profile = pipeline.start(config)
    align = rs.align(rs.stream.color)
    depth_intrinsics = rs.video_stream_profile(profile.get_stream(rs.stream.depth)).get_intrinsics()

    print(f"✅ 系统就绪！请将 [{TARGET_CLASS}] 放入视野，按 's' 键锁定并执行抓取，按 'q' 键退出。")

    target_p_cam = None

    try:
        while True:
            frames = pipeline.wait_for_frames()
            aligned_frames = align.process(frames)
            color_frame = aligned_frames.get_color_frame()
            depth_frame = aligned_frames.get_depth_frame()
            if not color_frame or not depth_frame: continue
            
            color_image = np.asanyarray(color_frame.get_data())
            results = model(color_image, verbose=False)
            
            found_target = False
            for r in results:
                for box in r.boxes:
                    class_name = model.names[int(box.cls[0])]
                    if class_name == TARGET_CLASS:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        cx, cy = (x1 + x2) // 2, (y1 + y2) // 2
                        depth = depth_frame.get_distance(cx, cy)
                        
                        if 0 < depth < 2.0:
                            target_p_cam = rs.rs2_deproject_pixel_to_point(depth_intrinsics, [cx, cy], depth)
                            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                            cv2.circle(color_image, (cx, cy), 5, (0, 0, 255), -1)
                            cv2.putText(color_image, f"Target: {depth:.2f}m", (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                            found_target = True
                            break # 只锁定第一个找到的目标
                if found_target: break

            cv2.imshow('Grasp Vision', color_image)
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s') and target_p_cam:
                print("\n" + "="*40)
                print("������ 锁定目标！开始执行抓取流程...")
                print("="*40)
                
                # 1. 坐标转换
                p_base = transform_cam_to_base(target_p_cam)
                print(f"������ 目标相机坐标: {target_p_cam}")
                print(f"������ 转换到底座坐标: {p_base}")
                
                # 2. 抓取动作流
                control_gripper("open")
                
                print("✈️ 移动到准备点 (物体正上方)...")
                move_arm_to(p_base[0], p_base[1], p_base[2] + Z_OFFSET)
                
                print("⬇️ 下降至抓取点...")
                move_arm_to(p_base[0], p_base[1], p_base[2])
                
                control_gripper("close")
                
                print("⬆️ 提起物体...")
                move_arm_to(p_base[0], p_base[1], p_base[2] + Z_OFFSET + 0.2)
                
                print("✅ 抓取流程演示完成！恢复视觉监测...\n")
                target_p_cam = None # 重置目标
                
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()