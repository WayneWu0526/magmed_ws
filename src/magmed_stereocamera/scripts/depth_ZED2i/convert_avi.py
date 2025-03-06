import pyzed.sl as sl
import cv2
import os

# 输入和输出文件名
input_svo_file = '/home/zhang/文档/ZED/HD1080_SN39073032_17-18-00.svo2'
output_avi_file = './output_avi/output_left.avi'

# 检查输出目录是否存在，不存在则创建
os.makedirs(os.path.dirname(output_avi_file), exist_ok=True)

# 初始化ZED相机
init_params = sl.InitParameters()
init_params.set_from_svo_file(input_svo_file)
init_params.svo_real_time_mode = False
zed = sl.Camera()

status = zed.open(init_params)
if status != sl.ERROR_CODE.SUCCESS:
    print("无法打开SVO文件:", status)
    exit()

# 获取图像尺寸
cam_info = zed.get_camera_information().camera_configuration.resolution
width, height = cam_info.width, cam_info.height
print(f"视频尺寸: {width} x {height}")

# 配置视频编码器，推荐MJPG，兼容性更好
fourcc = cv2.VideoWriter_fourcc(*'MJPG')
out = cv2.VideoWriter(output_avi_file, fourcc, 30.0, (width, height))

# 检查 VideoWriter 是否打开
if not out.isOpened():
    print("视频写入器初始化失败，请检查路径或编码器")
    exit()

# 创建图像容器
image = sl.Mat()

# 逐帧转换
frame_count = 0
while True:
    grab_status = zed.grab()
    if grab_status == sl.ERROR_CODE.SUCCESS:
        zed.retrieve_image(image, sl.VIEW.LEFT)
        frame = image.get_data()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_BGRA2BGR)
        out.write(frame_bgr)
        frame_count += 1
    elif grab_status == sl.ERROR_CODE.END_OF_SVOFILE_REACHED:
        print("已到达SVO文件结尾")
        break
    else:
        print("发生其他错误:", grab_status)
        break

# 释放资源
out.release()
zed.close()

print(f'成功转换为AVI文件：{output_avi_file}，总帧数：{frame_count}')
