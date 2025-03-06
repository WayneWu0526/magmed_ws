import cv2
import numpy as np
import pickle
import os
import re
import glob

def sort_images(image_path):
    """ 根据文件名中的数字对图像排序 """
    return sorted(glob.glob(image_path), key=lambda x: int(re.findall(r'\d+', os.path.basename(x))[0]))

def stereo_calibration(calib_images_path, chessboard_size, square_size, image_size):
    """ 进行双目标定并存储标定数据 """
    
    # 获取所有左右相机图像
    left_images = sort_images(f"{calib_images_path}/left*.jpg")
    right_images = sort_images(f"{calib_images_path}/right*.jpg")
    
    if len(left_images) != len(right_images):
        raise ValueError("左相机和右相机的图像数量不匹配，请检查文件！")

    # 3D 世界坐标点
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    objp *= square_size  # 格子实际尺寸（米）

    obj_points, left_img_points, right_img_points = [], [], []

    for left_img_path, right_img_path in zip(left_images, right_images):
        left_img = cv2.imread(left_img_path)
        right_img = cv2.imread(right_img_path)
        gray_left = cv2.cvtColor(left_img, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(right_img, cv2.COLOR_BGR2GRAY)

        # 检测棋盘格角点
        ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)

        if ret_left and ret_right:
            obj_points.append(objp)
            # 进一步优化角点
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
            corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
            
            left_img_points.append(corners_left)
            right_img_points.append(corners_right)

            # 可视化检测的角点
            cv2.drawChessboardCorners(left_img, chessboard_size, corners_left, ret_left)
            cv2.drawChessboardCorners(right_img, chessboard_size, corners_right, ret_right)
            cv2.imshow("Left Chessboard", left_img)
            cv2.imshow("Right Chessboard", right_img)
            cv2.waitKey(500)  # 暂停 0.5 秒，观察结果

    cv2.destroyAllWindows()
    print(f"成功检测到 {len(obj_points)} 组有效的棋盘格图像")

    # 单目标定
    _, left_camera_matrix, left_dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, left_img_points, image_size, None, None)
    _, right_camera_matrix, right_dist_coeffs, _, _ = cv2.calibrateCamera(obj_points, right_img_points, image_size, None, None)

    # 双目标定
    _, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
        obj_points, left_img_points, right_img_points,
        left_camera_matrix, left_dist_coeffs,
        right_camera_matrix, right_dist_coeffs,
        image_size, criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-5),
        flags=cv2.CALIB_FIX_INTRINSIC)

    # 立体校正
    R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
        left_camera_matrix, left_dist_coeffs,
        right_camera_matrix, right_dist_coeffs,
        image_size, R, T, alpha=0)

    # 存储标定数据
    stereo_calibration_data = {
        "left_camera_matrix": left_camera_matrix,
        "left_dist_coeffs": left_dist_coeffs,
        "right_camera_matrix": right_camera_matrix,
        "right_dist_coeffs": right_dist_coeffs,
        "R": R, "T": T, "E": E, "F": F,
        "R1": R1, "R2": R2, "P1": P1, "P2": P2, "Q": Q
    }

    with open("stereo_calibration_data.pkl", "wb") as f:
        pickle.dump(stereo_calibration_data, f)

    print("双目标定数据已保存！")

if __name__ == "__main__":
    chessboard_size = (12, 18)  # 棋盘格交叉点数目
    square_size = 0.015  # 格子大小（米）
    image_size = (1280, 720)  # 相机分辨率
    calib_images_path = "calib_images"  # 你的图片路径
    stereo_calibration(calib_images_path, chessboard_size, square_size, image_size)
