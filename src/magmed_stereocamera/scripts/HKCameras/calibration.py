import cv2
import numpy as np
import glob
import pickle
import os

# 1. 设置棋盘格参数
chessboard_size = (12, 18)  # 角点数 = 方格数 - 1
square_size = 15  # 每个方格的实际尺寸 (mm)

# 2. 生成棋盘格的世界坐标
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2) * square_size

# 3. 存储3D点和2D点
obj_points = []  # 3D 世界坐标
img_points = []  # 2D 图像坐标

# 4. 读取所有标定图像
image_folder = "/opt/MVS/bin/Temp/Data/left_camera_calibration/"
image_files = [os.path.join(image_folder, f) for f in os.listdir(image_folder) if f.endswith(".bmp")]

for image_file in image_files:
    img = cv2.imread(image_file)
    if img is None:
        print(f"❌ Warning: Could not read image {image_file}")
        continue

    print(f"📂 Loaded image: {image_file} with shape {img.shape}")

    # ✅ 降低分辨率，提高计算速度
    scale_factor = 0.5
    img = cv2.resize(img, (0, 0), fx=scale_factor, fy=scale_factor)
    
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # ✅ 只显示第一张图片进行调试
    # if image_file == image_files[0]:  
    #     cv2.imshow("Sample Image", img)
    #     cv2.waitKey(1000)
    #     cv2.destroyAllWindows()
    
    # ✅ 改用更鲁棒的角点检测算法
    ret, corners = cv2.findChessboardCornersSB(gray, chessboard_size, None)

    if not ret:
        print(f"❌ Chessboard not found in {image_file}")
        continue  # 跳过未检测到棋盘格的图片
    else:
        print(f"✅ Chessboard detected in {image_file}")

    obj_points.append(objp)
    img_points.append(corners)

cv2.destroyAllWindows()

# ✅ 在标定前检查是否有有效数据
if len(obj_points) == 0 or len(img_points) == 0:
    raise ValueError("Error: No valid chessboard corners detected in any image. Please check your images and parameters.")

# 7. 进行相机标定
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(obj_points, img_points, gray.shape[::-1], None, None)

print("📷 Camera Matrix:\n", camera_matrix)
print("🎯 Distortion Coefficients:\n", dist_coeffs)

# 8. 保存标定数据
with open("calibration_data.pkl", "wb") as f:
    pickle.dump((camera_matrix, dist_coeffs), f)
