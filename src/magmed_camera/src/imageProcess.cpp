#include "magmed_camera/imageProcess.h"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <cmath>
#define PI 3.1415926

namespace magmed_camera{

float imageProcess::getTipAngle(unsigned short Height, unsigned short Width, unsigned char * pData){

    // load image
    cv::Mat img = cv::Mat(Height, Width, CV_8UC3, pData);
    // turn BGR to gray
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    // threshold the image
    cv::threshold(img, img, 170, 255, cv::THRESH_BINARY_INV);
    // show the image
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
    // fit a line to the nonzeroCoordinates and calculate the coefficient of determination
    cv::Vec4f line;
    cv::fitLine(nonzeroCoordinates, line, cv::DIST_L2, 0, 0.01, 0.01);
    // calculate the coefficient of determination
    float r2 = 0.0;
    for (int i = 0; i < nonzeroCoordinates.rows; i++){
        float y = line(1) / line(0) * (nonzeroCoordinates.at<cv::Point>(i).x - line(2)) + line(3);
        r2 += pow(y - nonzeroCoordinates.at<cv::Point>(i).y, 2);
    }
    r2 /= nonzeroCoordinates.rows;
    // if r2 < 120, use quadratic function to fit the curve
    if(r2 < 120.0){
        // create an Eigen matrix to store the coordinates
        Eigen::MatrixXf coordinates(nonzeroCoordinates.rows, 2);
        // copy every point in nonzeroCoordinates to coordinates
        for (int i = 0; i < nonzeroCoordinates.rows; i++){
            coordinates(i, 0) = nonzeroCoordinates.at<cv::Point>(i).x;
            coordinates(i, 1) = nonzeroCoordinates.at<cv::Point>(i).y;
        }

        // 采用二次函数拟合    
        // 创建扩展矩阵，添加 x^2 和常数项 1
        Eigen::MatrixXf extendedCoordinates(coordinates.rows(), 3);
        extendedCoordinates.col(0) = coordinates.col(0).array().square();
        extendedCoordinates.col(1) = coordinates.col(0);
        extendedCoordinates.col(2) = Eigen::VectorXf::Ones(coordinates.rows());
        // 使用 QR 分解求解拟合系数
        Eigen::Vector3f curve = extendedCoordinates.colPivHouseholderQr().solve(coordinates.col(1));
        // find the point with maximum x coordinate use Eigen
        float maxX = coordinates.col(0).maxCoeff();
        // calculate the derivative of the curve at the point with maximum x coordinate
        float derivative = 2 * curve(0) * maxX + curve(1);
        // print the derivative
        std::cout << "derivative = " << derivative << std::endl;
        // calculate the angle in radians
        float tipAngle = -atan(derivative); // because the y axis is downward
        // print the angle in degrees
        std::cout << "tipAngle = " << tipAngle * 180 / PI << std::endl;

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
    }
    else{
        // fit an ellipse to the nonzeroCoordinates
        cv::RotatedRect ellipse = cv::fitEllipse(nonzeroCoordinates);
        // print the ellipse
        // std::cout << "ellipse center = " << ellipse.center << ", ellipse size = " << ellipse.size << ", ellipse angle = " << ellipse.angle << std::endl;
        // calculate the coefficients of the ellipse in the form of ax^2 + bxy + cy^2 + dx + ey + f = 0
        float a = pow(ellipse.size.width / 2, 2) * pow(sin(ellipse.angle / 180 * PI), 2) + pow(ellipse.size.height / 2, 2) * pow(cos(ellipse.angle / 180 * PI), 2);
        float b = 2 * (pow(ellipse.size.height / 2, 2) - pow(ellipse.size.width / 2, 2)) * sin(ellipse.angle / 180 * PI) * cos(ellipse.angle / 180 * PI);
        float c = pow(ellipse.size.width / 2, 2) * pow(cos(ellipse.angle / 180 * PI), 2) + pow(ellipse.size.height / 2, 2) * pow(sin(ellipse.angle / 180 * PI), 2);
        float d = -2 * a * ellipse.center.x - b * ellipse.center.y;
        float e = -b * ellipse.center.x - 2 * c * ellipse.center.y;
        float f = a * pow(ellipse.center.x, 2) + b * ellipse.center.x * ellipse.center.y + c * pow(ellipse.center.y, 2) - pow(ellipse.size.width / 2, 2) * pow(ellipse.size.height / 2, 2);
        // calculate the scope of the ellipse at the point with maximum x in nonzeroCoordinates
        float maxX = nonzeroCoordinates.at<cv::Point>(0).x;
        for (int i = 1; i < nonzeroCoordinates.rows; i++){
            if (nonzeroCoordinates.at<cv::Point>(i).x > maxX){
                maxX = nonzeroCoordinates.at<cv::Point>(i).x;
            }
        }
        

        // 在图像上绘制拟合椭圆
        // create a new image
        cv::Mat img1 = cv::Mat::zeros(Height, Width, CV_8UC3);
        // draw the nonzero coordinates
        for (int i = 0; i < nonzeroCoordinates.rows; i++){
            img1.at<cv::Vec3b>(nonzeroCoordinates.at<cv::Point>(i)) = cv::Vec3b(255, 255, 255);
        }
        // draw the ellipse
        cv::ellipse(img1, ellipse, cv::Scalar(0, 0, 255), 1);
        // show the image
        cv::imshow("img1", img1);
        cv::waitKey(1);
    }

    return 0.0;
}

}