#include "magmed_camera/imageProcess.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>

namespace magmed_camera{

float imageProcess::getTipAngle(unsigned short Height, unsigned short Width, unsigned char * pData){

    // load image
    cv::Mat img = cv::Mat(Height, Width, CV_8UC3, pData);
    // turn BGR to gray
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    // threshold the image
    cv::threshold(img, img, 180, 255, cv::THRESH_BINARY_INV);
    // // show the image
    // cv::imshow("img", img);
    // cv::waitKey(1);
    // extract the nonzero coordinates
    cv::Mat nonzeroCoordinates;
    cv::findNonZero(img, nonzeroCoordinates);
    // if there is no nonzero coordinates, print error
    if (nonzeroCoordinates.empty()){
        std::cout << "No nonzero coordinates!" << std::endl;
        return 0.0;
    }
    // create an Eigen matrix to store the coordinates
    Eigen::MatrixXf coordinates(nonzeroCoordinates.rows, 2);
    // copy every point in nonzeroCoordinates to coordinates
    for (int i = 0; i < nonzeroCoordinates.rows; i++){
        coordinates(i, 0) = nonzeroCoordinates.at<cv::Point>(i).x;
        coordinates(i, 1) = nonzeroCoordinates.at<cv::Point>(i).y;
    }
    // 创建扩展矩阵，添加 x^2 和常数项 1
    Eigen::MatrixXf extendedCoordinates(coordinates.rows(), 3);
    extendedCoordinates.col(0) = coordinates.col(0).array().square();
    extendedCoordinates.col(1) = coordinates.col(0);
    extendedCoordinates.col(2) = Eigen::VectorXf::Ones(coordinates.rows());
    // 使用 QR 分解求解拟合系数
    Eigen::Vector3f curve = extendedCoordinates.colPivHouseholderQr().solve(coordinates.col(1));

    // print the curve
    std::cout << "a = " << curve(0) << ", b = " << curve(1) << ", c = " << curve(2) << std::endl; 
    // 在图像上绘制拟合曲线
    // create a new image
    cv::Mat img2 = cv::Mat::zeros(Height, Width, CV_8UC3);
    // draw the nonzero coordinates
    for (int i = 0; i < nonzeroCoordinates.rows; i++){
        img2.at<cv::Vec3b>(nonzeroCoordinates.at<cv::Point>(i)) = cv::Vec3b(255, 255, 255);
    }
    // draw the curve
    for (int i = 0; i < Width; i++){
        int y = curve(0) * i * i + curve(1) * i + curve(2);
        if (y >= 0 && y < Height){
            img2.at<cv::Vec3b>(y, i) = cv::Vec3b(0, 0, 255);
        }
    } 
    // show the image
    cv::imshow("img2", img2);
    cv::waitKey(1);
    
    // find the point with maximum x coordinate use Eigen
    float maxX = coordinates.col(0).maxCoeff();
    // calculate the derivative of the curve at the point with maximum x coordinate
    float derivative = 2 * curve(0) * maxX + curve(1);
    // print the derivative
    std::cout << "derivative = " << derivative << std::endl;
    // calculate the angle in radians
    float tipAngle = -atan(derivative); // because the y axis is downward
    // print the angle in degrees
    std::cout << "tipAngle = " << tipAngle * 180 / 3.1415926 << std::endl;
    return tipAngle;
}

}