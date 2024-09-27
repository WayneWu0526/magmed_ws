import pyzed.sl as sl
import cv2
import numpy as np
import rospy
from math import *
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Float64
from scipy.signal import butter, filtfilt

def main():
    # 初始化ROS节点
    rospy.init_node('zed_depth_publisher', anonymous=True)
    point_pub = rospy.Publisher('/magmed_stereoCamera/tipPoint', PointStamped, queue_size=10)
    depth_bkg_pub = rospy.Publisher('/magmed_stereoCamera/backgroundDepth', Float64, queue_size=10)

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
    zed.set_camera_settings(sl.VIDEO_SETTINGS.SHARPNESS, 8)  # 设置锐度
    zed.set_camera_settings(sl.VIDEO_SETTINGS.GAMMA, 5)  # 设置Gamma
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

    # 设置感兴趣区域的坐标
    roi_x, roi_y, roi_width, roi_height = 990, 460, 225, 225

    # Low-pass filter parameters
    fs = 30  # 假设循环大约每秒运行30次
    cutoff_freq = 0.1  # 截止频率 (Hz)
    b, a = butter(2, cutoff_freq / (fs / 2), btype='low')

    # 缓冲区用于存储历史 depth_value
    depth_buffer = []
    buffer_size = 50  # 缓冲区大小
    
    hist_X = 0
    hist_Y = 0
    hist_Z = 0

    def apply_low_pass_filter(data_buffer, new_value):
        data_buffer.append(new_value)
        
        # 保持缓冲区只包含最近的N个值
        if len(data_buffer) > buffer_size:
            data_buffer.pop(0)
        
        # 如果有足够的数据点，则应用低通滤波器
        if len(data_buffer) > 10:
            return filtfilt(b, a, data_buffer)[-1]  # 返回最新的滤波值
        else:
            return new_value  # 如果数据点不足，返回原始值

    while not rospy.is_shutdown():

        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
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
            lower_red1 = np.array([0, 80, 80])
            upper_red1 = np.array([10, 255, 255])
            mask = cv2.inRange(hsv, lower_red1, upper_red1)

            # 使用形态学操作去除噪声
            kernel = np.ones((3, 3), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

            # 找到轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            if len(contours) != 0:
                min_dist = 1000000
                for contour in contours:
                    x, y, w, h = cv2.boundingRect(contour)
                    cv2.rectangle(image_ocv_bgr, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cx = x + w // 2
                    cy = y + h // 2
                    depth_value = roi_depth[cy, cx]

                    # 对 depth_value 进行低通滤波
                    filtered_depth_value = apply_low_pass_filter(depth_buffer, depth_value)

                    x_ndc = (cx + roi_x - calib_cx) / calib_fx
                    y_ndc = (cy + roi_y - calib_cy) / calib_fy
                    X = x_ndc * filtered_depth_value
                    Y = y_ndc * filtered_depth_value
                    Z = filtered_depth_value

                    dist = sqrt((X - hist_X) ** 2 + (Y - hist_Y) ** 2 + (Z - hist_Z) ** 2)
                    if dist < min_dist:
                        min_dist = dist
                        hist_X = X
                        hist_Y = Y
                        hist_Z = Z

                    # 在图像上显示深度信息
                    label = f'Depth: {filtered_depth_value:.2f} mm'
                    cv2.putText(image_ocv_bgr, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # 发布点信息到ROS话题
            point_msg = PointStamped()
            point_msg.header.stamp = rospy.Time.now()
            point_msg.header.frame_id = "zed_camera"
            point_msg.point.x = hist_X / 1000  # 转换为米
            point_msg.point.y = hist_Y / 1000  # 转换为米
            point_msg.point.z = hist_Z / 1000  # 转换为米
            point_pub.publish(point_msg)

            # 将深度数据转换为伪彩色图像
            normalized_depth = cv2.normalize(roi_depth, None, 0, 255, cv2.NORM_MINMAX)
            colored_depth = cv2.applyColorMap(np.uint8(normalized_depth), cv2.COLORMAP_JET)

            # 显示结果
            cv2.imshow("Colored Depth Map", colored_depth)
            cv2.imshow("RGB Image", image_ocv_bgr)

            # 按下ESC键退出循环
            if cv2.waitKey(1) == 27:
                break

    # 关闭相机
    zed.close()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
