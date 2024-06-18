import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
import pyzed.sl as sl
import cv2

def main():
    # 初始化ROS节点
    rospy.init_node('zed2i_image_publisher', anonymous=True)
    image_pub = rospy.Publisher('/zed2i/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    
    # 创建ZED相机对象
    zed = sl.Camera()
    
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
    
    # 设置相机参数
    init_params = sl.InitParameters()
    init_params.camera_resolution = sl.RESOLUTION.HD2K  # 设置分辨率为2K
    init_params.coordinate_units = sl.UNIT.MILLIMETER
    
    # 打开相机
    if zed.open(init_params) != sl.ERROR_CODE.SUCCESS:
        print("无法打开ZED相机")
        zed.close()
        sys.exit(1)
    
    runtime_parameters = sl.RuntimeParameters()
    mat = sl.Mat()
    
    while not rospy.is_shutdown():
        # 捕捉图像
        if zed.grab(runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            zed.retrieve_image(mat, sl.VIEW.LEFT)
            image = mat.get_data()

            # 提取图像中的某一块区域 (例如左上角的100x100像素区域)
            image_roi = image[450:450+250, 1150:1150+250]
            
            # 确保图像是BGRA8格式，并转换为BGR8
            if image_roi.shape[2] == 4:  # BGRA8格式
                image_roi_bgr = cv2.cvtColor(image_roi, cv2.COLOR_BGRA2BGR)
            else:
                image_roi_bgr = image_roi

            # 将图像转换为ROS消息并发布
            image_msg = bridge.cv2_to_imgmsg(image_roi_bgr, "bgr8")
            image_pub.publish(image_msg)
        else:
            rospy.logerr("抓取图像失败")
    
    # 关闭相机
    zed.close()

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
