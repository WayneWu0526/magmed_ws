import pyzed.sl as sl
import cv2
import numpy as np
import torch

# 加载YOLOv5自定义模型
model_path = '/home/zhang/yolov5-master/'
model = torch.hub.load(model_path, 'custom', path='weights/best.pt', source='local')

# 创建ZED相机对象
zed = sl.Camera()

# 设置初始化参数
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD2K  # 使用2K分辨率
init_params.coordinate_units = sl.UNIT.MILLIMETER  # 使用毫米作为单位
init_params.depth_minimum_distance = 200  # 设置最小深度为200mm
init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # 使用深度神经网络模式

# 打开相机
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("无法打开ZED相机")
    exit(1)

zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 0)  # 设置亮度
zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)  # 设置对比度
zed.set_camera_settings(sl.VIDEO_SETTINGS.HUE, 0)  # 设置色调
zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 8)  # 设置饱和度
zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 4)  # 设置锐度
zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, 1)  # 设置锐度
zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 4500)  # 设置白平衡

zed.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)  # 关闭自动曝光/增益
zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 42)  # 设置曝光
zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 21)  # 设置增益

# 创建运行时参数对象
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.confidence_threshold = 100
runtime_parameters.texture_confidence_threshold = 100

# 创建深度图和RGB图对象
depth = sl.Mat()
image = sl.Mat()

# 设置感兴趣区域的坐标（例如一个矩形区域）
roi_x, roi_y, roi_width, roi_height = 1150, 450, 250, 250

# 创建一个用于多帧平均的缓冲区
frame_buffer = []
buffer_size = 5

while True:
    if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
        # 获取深度图和左视图
        zed.retrieve_measure(depth, sl.MEASURE.DEPTH)
        zed.retrieve_image(image, sl.VIEW.LEFT)

        # 提取感兴趣区域的深度信息和图像
        roi_depth = depth.get_data()[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
        image_ocv = image.get_data()[roi_y:roi_y+roi_height, roi_x:roi_x+roi_width]
        if image_ocv.shape[2] == 4:  # BGRA8格式
            image_ocv_bgr = cv2.cvtColor(image_ocv, cv2.COLOR_BGRA2BGR)
        else:
            image_ocv_bgr = image_ocv

        # 使用YOLOv5进行物体检测
        results = model(image_ocv_bgr)
        # print(len(results.xyxy))

        for *xyxy, conf, cls in results.xyxy[0]:
            label = f'{model.names[int(cls)]} {conf:.2f}'
            x1, y1, x2, y2 = map(int, xyxy)
            # 绘制检测框和标签
            cv2.rectangle(image_ocv_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(image_ocv_bgr, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # 将当前帧的深度数据添加到缓冲区
        frame_buffer.append(roi_depth)

        # # 如果缓冲区已满，则计算平均深度
        # if len(frame_buffer) >= buffer_size:
        #     averaged_depth = np.mean(frame_buffer, axis=0)
        #     frame_buffer.pop(0)  # 移除最旧的帧

        #     # 将平均深度数据转换为伪彩色图像
        #     normalized_depth = cv2.normalize(averaged_depth, None, 0, 255, cv2.NORM_MINMAX)
        #     colored_depth = cv2.applyColorMap(np.uint8(normalized_depth), cv2.COLORMAP_JET)
            
        #     # 绘制检测框和标签，获取物体中心点的深度信息
        #         # 获取物体中心点的深度信息
        #     cx = (x1 + x2) // 2
        #     cy = (y1 + y2) // 2
        #     distance = averaged_depth[cy, cx]
            
        # print(f'Depth: {distance:.2f} mm')

        # 显示伪彩色深度图和RGB图像
        # cv2.imshow("Colored Depth Map", colored_depth)
        cv2.imshow("RGB Image", image_ocv_bgr)

        # 按下ESC键退出循环
        if cv2.waitKey(1) == 27:
            break

# 关闭相机
zed.close()
cv2.destroyAllWindows()
