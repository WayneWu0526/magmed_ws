import cv2
import numpy as np
import pickle
import glob
import matplotlib.pyplot as plt

def load_calibration_data(file_path):
    """ 加载相机标定数据 """
    with open(file_path, 'rb') as f:
        data = pickle.load(f)

    if isinstance(data, tuple) and len(data) == 2:
        camera_matrix, dist_coeffs = data
        print("Only camera_matrix and dist_coeffs found, returning them.")
        return camera_matrix, dist_coeffs, None, None, None, None  # 用 None 占位
    else:
        raise ValueError("Unexpected data format in calibration file.")



def compute_reprojection_error(obj_points, img_points, rvecs, tvecs, camera_matrix, dist_coeffs):
    """ 计算重投影误差 """
    total_error = 0
    for i in range(len(obj_points)):
        img_points_proj, _ = cv2.projectPoints(obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(img_points[i], img_points_proj, cv2.NORM_L2) / len(img_points_proj)
        total_error += error
    mean_error = total_error / len(obj_points)
    print(f"平均重投影误差: {mean_error:.4f} 像素")
    return mean_error

def undistort_image(image_path, camera_matrix, dist_coeffs):
    """ 对图像进行畸变校正 """
    img = cv2.imread(image_path)
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    undistorted_img = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    
    # 裁剪有效区域
    x, y, w, h = roi
    undistorted_img = undistorted_img[y:y+h, x:x+w]
    
    # 显示原始和校正后的图像
    plt.figure(figsize=(10,5))
    plt.subplot(1,2,1)
    plt.title("Original Image")
    plt.imshow(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    plt.subplot(1,2,2)
    plt.title("Undistorted Image")
    plt.imshow(cv2.cvtColor(undistorted_img, cv2.COLOR_BGR2RGB))
    plt.show()
    
def main():
    # 加载左右相机的标定数据
    left_camera_matrix, left_dist_coeffs, left_rvecs, left_tvecs, left_obj_points, left_img_points = load_calibration_data("left_camera_calibration_data.pkl")
    right_camera_matrix, right_dist_coeffs, right_rvecs, right_tvecs, right_obj_points, right_img_points = load_calibration_data("right_camera_calibration_data.pkl")

    print("左相机参数:")
    print("Camera Matrix:\n", left_camera_matrix)
    print("Distortion Coefficients:\n", left_dist_coeffs)

    # 计算重投影误差（如果数据完整）
    if left_rvecs is not None and left_tvecs is not None and left_obj_points is not None and left_img_points is not None:
        print("左相机重投影误差:")
        compute_reprojection_error(left_obj_points, left_img_points, left_rvecs, left_tvecs, left_camera_matrix, left_dist_coeffs)
    else:
        print("⚠️  标定文件缺少位姿数据（rvecs, tvecs, obj_points, img_points），无法计算重投影误差。")

    print("右相机参数:")
    print("Camera Matrix:\n", right_camera_matrix)
    print("Distortion Coefficients:\n", right_dist_coeffs)

    if right_rvecs is not None and right_tvecs is not None and right_obj_points is not None and right_img_points is not None:
        print("右相机重投影误差:")
        compute_reprojection_error(right_obj_points, right_img_points, right_rvecs, right_tvecs, right_camera_matrix, right_dist_coeffs)
    else:
        print("⚠️  标定文件缺少位姿数据（rvecs, tvecs, obj_points, img_points），无法计算重投影误差。")

    # 选择测试图像进行畸变校正
    left_test_image = "./images/left_test_image.bmp"  # 替换为你的测试图像路径
    right_test_image = "./images/right_test_image.bmp"  # 替换为你的测试图像路径

    print("左相机畸变校正:")
    undistort_image(left_test_image, left_camera_matrix, left_dist_coeffs)
    
    print("右相机畸变校正:")
    undistort_image(right_test_image, right_camera_matrix, right_dist_coeffs)

    
if __name__ == "__main__":
    main()
