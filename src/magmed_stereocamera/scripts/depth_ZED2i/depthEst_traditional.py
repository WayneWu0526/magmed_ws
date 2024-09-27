import pyzed.sl as sl
import cv2
import numpy as np

# 创建ZED相机对象
zed = sl.Camera()

# 设置初始化参数
init_params = sl.InitParameters()
init_params.camera_resolution = sl.RESOLUTION.HD2K  # 使用2K分辨率
init_params.coordinate_units = sl.UNIT.MILLIMETER  # 使用毫米作为单位
# init_params.coordinate_units = sl.UNIT.METER  # 使用米作为单位
init_params.depth_minimum_distance = 200  # 设置最小深度为200mm
# init_params.depth_maximum_distance = 300  # 设置最大深度为300mm
# init_params.depth_minimum_distance = 0.15  # 设置最小深度为200mm
# init_params.depth_mode = sl.DEPTH_MODE.ULTRA  # 使用超高精度模式
init_params.depth_mode = sl.DEPTH_MODE.NEURAL  # 使用深度神经网络模式
# init_params.enable_image_enhancement = True  # 启用图像增强

# 打开相机
if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
    print("无法打开ZED相机")
    exit(1)

zed.set_camera_settings(sl.VIDEO_SETTINGS.BRIGHTNESS, 5)  # 设置亮度
zed.set_camera_settings(sl.VIDEO_SETTINGS.CONTRAST, 4)  # 设置对比度
zed.set_camera_settings(sl.VIDEO_SETTINGS.HUE, 0)  # 设置色调
zed.set_camera_settings(sl.VIDEO_SETTINGS.SATURATION, 8)  # 设置饱和度
zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 8)  # 设置锐度
zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, 1)  # 设置Gamma
zed.set_camera_settings(sl.VIDEO_SETTINGS.WHITEBALANCE_TEMPERATURE, 4500)  # 设置白平衡

zed.set_camera_settings(sl.VIDEO_SETTINGS.AEC_AGC, 0)  # 关闭自动曝光/增益
zed.set_camera_settings(sl.VIDEO_SETTINGS.GAIN, 21)  # 设置增益
zed.set_camera_settings(sl.VIDEO_SETTINGS.EXPOSURE, 42)  # 设置曝光

# 创建运行时参数对象
runtime_parameters = sl.RuntimeParameters()
runtime_parameters.confidence_threshold = 100
runtime_parameters.texture_confidence_threshold = 100

# 获取相机内参
calib_params = zed.get_camera_information().camera_configuration.calibration_parameters
calib_fx = calib_params.left_cam.fx
calib_fy = calib_params.left_cam.fy
calib_cx = calib_params.left_cam.cx
calib_cy = calib_params.left_cam.cy

# 创建深度图和RGB图对象
depth = sl.Mat()
image = sl.Mat()

# 设置感兴趣区域的坐标（例如一个矩形区域）
roi_x, roi_y, roi_width, roi_height = 1150, 455, 250, 250

# 创建一个用于多帧平均的缓冲区
frame_buffer = []
buffer_size = 1

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

        # 转换为HSV颜色空间
        hsv = cv2.cvtColor(image_ocv_bgr, cv2.COLOR_BGR2HSV)

        # 设置从暗红色到亮红色的HSV范围
        lower_red1 = np.array([0, 50, 50])
        upper_red1 = np.array([10, 255, 255])
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)

        lower_red2 = np.array([160, 50, 50])
        upper_red2 = np.array([180, 255, 255])
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)

        # 合并两个掩码
        mask = mask1 + mask2

        # 使用形态学操作去除噪声
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

        # 找到轮廓
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # 将当前帧的深度数据添加到缓冲区
        frame_buffer.append(roi_depth)

        # 如果缓冲区已满，则计算平均深度
        if len(frame_buffer) >= buffer_size:
            averaged_depth = np.mean(frame_buffer, axis=0)
            frame_buffer.pop(0)  # 移除最旧的帧

            # 绘制矩形框并获取深度信息
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                cv2.rectangle(image_ocv_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                # 获取红点中心的深度信息
                cx = x + w // 2
                cy = y + h // 2
                depth_value = averaged_depth[cy, cx]
                # 在图像上显示深度信息
                label = f'Depth: {depth_value:.2f} mm'
                cv2.putText(image_ocv_bgr, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            x_ndc = (cx + roi_x - calib_cx) / calib_fx
            y_ndc = (cy + roi_y - calib_cy) / calib_fy

            print("calib_cx: {}, calib_cy: {} ".format(calib_cx, calib_cy))

            X = x_ndc * depth_value
            Y = y_ndc * depth_value
            Z = depth_value

            print("Camera coordinates: (X: {}, Y: {}, Z: {})".format(X, Y, Z))

            # 获取图像中心的深度信息
            center_x, center_y = image_ocv_bgr.shape[1] // 2, image_ocv_bgr.shape[0] // 2
            center_y = center_y - 50
            center_depth_value = averaged_depth[center_y, center_x]

            # 在图像上显示中心的深度信息
            center_label = f'Center Depth: {center_depth_value:.2f} mm'
            cv2.putText(image_ocv_bgr, center_label, (center_x - 50, center_y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)

            # 将深度数据转换为伪彩色图像
            normalized_depth = cv2.normalize(averaged_depth, None, 0, 255, cv2.NORM_MINMAX)
            colored_depth = cv2.applyColorMap(np.uint8(normalized_depth), cv2.COLORMAP_JET)

            # # 显示结果
            # cv2.imshow("Colored Depth Map", colored_depth)
            # cv2.imshow("RGB Image", image_ocv_bgr)
            
            # 按下ESC键退出循环
            if cv2.waitKey(1) == 27:
                break

# 关闭相机
zed.close()
cv2.destroyAllWindows()
