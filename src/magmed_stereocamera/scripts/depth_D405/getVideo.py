import pyrealsense2 as rs
import numpy as np
import cv2
from datetime import datetime

# 获取当前时间并格式化为字符串
current_time = datetime.now().strftime("%Y%m%d_%H%M%S")

# 配置 RealSense 相机
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)  # 设置分辨率和帧率

# 开始流
pipeline.start(config)

# 创建视频写入器
fourcc = cv2.VideoWriter_fourcc(*'XVID')
filename = f'optCtrl_fixed_sideview_{current_time}.avi'
out = cv2.VideoWriter(filename, fourcc, 25.0, (1280, 720))

try:
    while True:
        # 获取帧
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        
        if not color_frame:
            continue
        
        # 转换为 numpy 数组
        color_image = np.asanyarray(color_frame.get_data())
        
        # 显示图像
        cv2.imshow('RealSense', color_image)
        
        # 写入视频
        out.write(color_image)
        
        # 按下 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # 停止流
    pipeline.stop()
    # 释放视频写入器
    out.release()
    # 关闭所有窗口
    cv2.destroyAllWindows()
