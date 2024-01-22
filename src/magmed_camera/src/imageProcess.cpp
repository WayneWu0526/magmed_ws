#include "magmed_camera/imageProcess.hpp"
// #include <opencv2/opencv.hpp>
#include <iostream>
#include <eigen3/Eigen/Dense>
// #include <opencv2/core/eigen.hpp>
#include <cmath>
#include <ros/ros.h>
#define PI 3.1415926

namespace magmed_camera
{

    float imageProcess::getTipAngle(unsigned short Height, unsigned short Width, unsigned char *pData, int nFlag)
    {
        // load image
        cv::Mat img = cv::Mat(Height, Width, CV_8UC3, pData);
        // turn BGR to gray
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
        // threshold the image
        cv::threshold(img, img, 150, 255, cv::THRESH_BINARY_INV); // 170

        float tipAngle = 0.0;
        switch (nFlag)
        {
        case 0:
            cv::imshow("img", img);
            cv::waitKey(1);
            break;
        case 1:
            try{ // 这种写法貌似还是不能解决问题。为什么不显示错误信息？
                tipAngle = ellipticaFunFit(Height, Width, img, 0);
            }catch (const std::exception& e){
                ROS_ERROR("Caught exception: %s", e.what());
                // std::cout << "Caught exception: " << e.what() << std::endl;
            }
            break;
        case 2:
            try{
                tipAngle = ellipticaFunFit(Height, Width, img, 0);
            }catch (const std::exception& e){
                ROS_ERROR("Caught exception: %s", e.what());
            }
            break;
        default:
            break;
        }
        return tipAngle;
    }

    float imageProcess::ellipticaFunFit(unsigned short Height, unsigned short Width, cv::Mat img, bool isImageShow)
    {
        // // dilate the image
        // cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        // cv::dilate(img, img, element);
        // element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        // cv::erode(img, img, element);

        // 快速滤波
        // cv::fastNlMeansDenoising(img, img, 3, 7, 21);
        // show the image
        // cv::imshow("img", img);
        // cv::waitKey(1);

        // extract the nonzero coordinates
        cv::Mat nonzeroCoordinates;
        cv::findNonZero(img, nonzeroCoordinates);
        // if there is no nonzero coordinates, print error
        if (nonzeroCoordinates.empty())
        {
            ROS_INFO("No nonzero coordinates!");
            return 0.0;
        }

        // create an Eigen matrix to store the coordinates
        Eigen::MatrixXf coordinates(nonzeroCoordinates.rows, 2);
        // copy every point in nonzeroCoordinates to coordinates
        for (int i = 0; i < nonzeroCoordinates.rows; i++)
        {
            coordinates(i, 0) = nonzeroCoordinates.at<cv::Point>(i).x;
            coordinates(i, 1) = nonzeroCoordinates.at<cv::Point>(i).y;
        }
        // calculate the baryCenter of the rod in the image using Eigen
        Eigen::Vector2f baryCenter = {coordinates.col(0).sum() / coordinates.rows(), coordinates.col(1).sum() / coordinates.rows()};

        // fit a line to the nonzeroCoordinates and calculate the coefficient of determination
        cv::Vec4f line;
        cv::fitLine(nonzeroCoordinates, line, cv::DIST_L2, 0, 0.01, 0.01);
        // calculate the coefficient of determination using Eigen
        Eigen::MatrixXf y = line(1) / line(0) * (coordinates.col(0).array() - line(2)) + line(3);
        float r2 = (y - coordinates.col(1)).array().square().sum() / coordinates.rows();
        // std::cout << "r2 = " << r2 << std::endl;

        Eigen::Vector2f distalEnd = {0.0, 0.0};
        Eigen::Vector2f proximalEnd = {0.0, 0.0};

        // if r2 < 130, use quadratic function to fit the curve
        if (r2 < 130.0)
        {
            // 采用二次函数拟合
            // 创建扩展矩阵，添加 x^2 和常数项 1
            Eigen::MatrixXf extendedCoordinates(coordinates.rows(), 3);
            extendedCoordinates.col(0) = coordinates.col(0).array().square();
            extendedCoordinates.col(1) = coordinates.col(0);
            extendedCoordinates.col(2) = Eigen::VectorXf::Ones(coordinates.rows());
            // 使用 QR 分解求解拟合系数
            Eigen::Vector3f curve = extendedCoordinates.colPivHouseholderQr().solve(coordinates.col(1));

            // find the proximal and distal end of the rod in the image
            proximalEnd = {baryCenter(0), curve(0) * baryCenter(0) * baryCenter(0) + curve(1) * baryCenter(0) + curve(2)};
            // calculate the sum of the pixels in a 3x3 neighborhood of the tip position in img
            int sum = 0;
            do
            {
                sum = 0;
                sum += img.at<uchar>(round(proximalEnd(1)), round(proximalEnd(0)));
                // for more robustness, calculate the sum of the pixels in a 3x3 neighborhood of the tip position in img
                // for (int i = -1; i <= 1; i++)
                // {
                //     for (int j = -1; j <= 1; j++)
                //     {
                //         sum += img.at<uchar>(round(proximalEnd(1)) + i, round(proximalEnd(0)) + j);
                //     }
                // }
                // iterate the proximalEnd
                proximalEnd(1) += -curve(0) * 2.0 * proximalEnd(0) + curve(0) - curve(1);
                proximalEnd(0) -= 1.0;
            } while (sum);

            distalEnd = {baryCenter(0), curve(0) * baryCenter(0) * baryCenter(0) + curve(1) * baryCenter(0) + curve(2)};
            // calculate the sum of the pixels in a 3x3 neighborhood of the distal position in img
            do
            {
                sum = 0;
                sum += img.at<uchar>(round(distalEnd(1)), round(distalEnd(0)));
                // for more robustness, calculate the sum of the pixels in a 3x3 neighborhood of the tip position in img
                // for (int i = -1; i <= 1; i++)
                // {
                //     for (int j = -1; j <= 1; j++)
                //     {
                //         sum += img.at<uchar>(round(distalEnd(1)) + i, round(distalEnd(0)) + j);
                //     }
                // }
                // iterate the distalEnd
                distalEnd(1) += curve(0) * 2.0 * distalEnd(0) + curve(1) + curve(0);
                distalEnd(0) += 1.0;
            } while (sum);
            // calculate the derivative of two tipPoints
            float derivariveOfProximalEnd = 2 * curve(0) * (proximalEnd(0)) + curve(1);   // proximalEnd(0) + 1.0
            float derivativeOfDistalEnd = 2 * curve(0) * (distalEnd(0) + 1.0) + curve(1); // distalEnd(0) + 1.0
            // print the derivative
            // std::cout << "derivariveOfProximalEnd = " << derivariveOfProximalEnd << "derivativeOfDistalEnd = " << derivativeOfDistalEnd <<  std::endl;
            // calculate the angle in radians
            float tipAngle = -atan(derivativeOfDistalEnd); // because the y axis is downward
            // print the angle in degrees
            // std::cout << "tipAngleParabola = " << tipAngle << std::endl;

            // print the curve
            // std::cout << "a = " << curve(0) << ", b = " << curve(1) << ", c = " << curve(2) << std::endl;
            if (isImageShow)
            {
                // 在图像上绘制拟合曲线
                // create a new image
                cv::Mat img1 = cv::Mat::zeros(Height, Width, CV_8UC3);
                // draw the nonzero coordinates
                for (int i = 0; i < nonzeroCoordinates.rows; i++)
                {
                    img1.at<cv::Vec3b>(nonzeroCoordinates.at<cv::Point>(i)) = cv::Vec3b(255, 255, 255);
                }
                // draw a cross at crossPoint
                for (int i = 0; i < Width; i++)
                {
                    img1.at<cv::Vec3b>(distalEnd(1), i) = cv::Vec3b(0, 0, 255);
                }
                for (int i = 0; i < Height; i++)
                {
                    img1.at<cv::Vec3b>(i, distalEnd(0)) = cv::Vec3b(0, 0, 255);
                }
                // draw the curve
                for (int i = 0; i < Width; i++)
                {
                    int y = curve(0) * i * i + curve(1) * i + curve(2);
                    if (y >= 0 && y < Height)
                    {
                        img1.at<cv::Vec3b>(y, i) = cv::Vec3b(0, 0, 255);
                    }
                }
                // show the image
                cv::imshow("img1", img1);
                cv::waitKey(1);
            }

            return tipAngle;
        }
        else
        {
            // fit an ellipse to the nonzeroCoordinates
            cv::RotatedRect ellipse = cv::fitEllipse(nonzeroCoordinates);
            Eigen::Vector2f ellipseCenter = {ellipse.center.x, ellipse.center.y};
            Eigen::Vector2f ellipseCoeff = {ellipse.size.height / 2.0, ellipse.size.width / 2.0};
            float ellipseAngle = ellipse.angle * PI / 180.0;

            // create a 2x2 Eigen rotation matrix
            Eigen::Matrix2f Rot;
            Rot << cos(ellipseAngle), sin(ellipseAngle),
                -sin(ellipseAngle), cos(ellipseAngle);
            Eigen::Vector2f transformedbaryCenter = Rot * (baryCenter - ellipseCenter);

            // calculate the angle of the baryCenter
            float baryCenterAngle0 = atan2(ellipseCoeff(1) * transformedbaryCenter(1), ellipseCoeff(0) * transformedbaryCenter(0));
            if (baryCenterAngle0 < 0)
            {
                baryCenterAngle0 += 2 * PI;
            }

            int sum = 0;
            Eigen::Vector2f originalCrossPoint = {0.0, 0.0};
            Eigen::Vector2f crossPoint;
            float baryCenterAngle = baryCenterAngle0;
            do
            {
                sum = 0;
                crossPoint = {ellipseCoeff(1) * cos(baryCenterAngle), ellipseCoeff(0) * sin(baryCenterAngle)};
                originalCrossPoint = Rot.transpose() * crossPoint + ellipseCenter;
                // transform the crossPoint back to the original coordinate system
                sum += img.at<uchar>(round(originalCrossPoint(1)), round(originalCrossPoint(0)));
                // for more accuracy, we can also add the 3x3 neighborhood of the crossPoint
                // for (int i = -1; i <= 1; i++)
                // {
                //     for (int j = -1; j <= 1; j++)
                //     {
                //         sum += img.at<uchar>(round(originalCrossPoint(1)) + i, round(originalCrossPoint(0)) + j);
                //     }
                // }
                // iterate the baryCenterAngle
                baryCenterAngle -= 0.01;
            } while (sum);

            baryCenterAngle += 0.03;
            crossPoint = {ellipseCoeff(1) * cos(baryCenterAngle), ellipseCoeff(0) * sin(baryCenterAngle)};
            originalCrossPoint = Rot.transpose() * crossPoint + ellipseCenter;
            Eigen::Vector2f originalCrossPoint1 = originalCrossPoint;

            double A = ellipseCoeff(0) * ellipseCoeff(0) * ((originalCrossPoint(0) - ellipseCenter(0)) * cos(ellipseAngle) + (originalCrossPoint(1) - ellipseCenter(1)) * sin(ellipseAngle));
            double B = ellipseCoeff(1) * ellipseCoeff(1) * (-(originalCrossPoint(0) - ellipseCenter(0)) * sin(ellipseAngle) + (originalCrossPoint(1) - ellipseCenter(1)) * cos(ellipseAngle));
            float tipAngle1 = atan((B * sin(ellipseAngle) - A * cos(ellipseAngle)) / (A * sin(ellipseAngle) + B * cos(ellipseAngle)));

            baryCenterAngle = baryCenterAngle0;
            do
            {
                sum = 0;
                crossPoint = {ellipseCoeff(1) * cos(baryCenterAngle), ellipseCoeff(0) * sin(baryCenterAngle)};
                originalCrossPoint = Rot.transpose() * crossPoint + ellipseCenter;
                // transform the crossPoint back to the original coordinate system
                sum += img.at<uchar>(round(originalCrossPoint(1)), round(originalCrossPoint(0)));
                // for more accuracy, we can also add the 3x3 neighborhood of the crossPoint
                // for (int i = -1; i <= 1; i++)
                // {
                //     for (int j = -1; j <= 1; j++)
                //     {
                //         sum += img.at<uchar>(round(originalCrossPoint(1)) + i, round(originalCrossPoint(0)) + j);
                //     }
                // }
                // iterate the baryCenterAngle
                baryCenterAngle += 0.01;
            } while (sum);

            baryCenterAngle -= 0.03;
            crossPoint = {ellipseCoeff(1) * cos(baryCenterAngle), ellipseCoeff(0) * sin(baryCenterAngle)};
            originalCrossPoint = Rot.transpose() * crossPoint + ellipseCenter;
            Eigen::Vector2f originalCrossPoint2 = originalCrossPoint;

            A = ellipseCoeff(0) * ellipseCoeff(0) * ((originalCrossPoint(0) - ellipseCenter(0)) * cos(ellipseAngle) + (originalCrossPoint(1) - ellipseCenter(1)) * sin(ellipseAngle));
            B = ellipseCoeff(1) * ellipseCoeff(1) * (-(originalCrossPoint(0) - ellipseCenter(0)) * sin(ellipseAngle) + (originalCrossPoint(1) - ellipseCenter(1)) * cos(ellipseAngle));
            float tipAngle2 = atan((B * sin(ellipseAngle) - A * cos(ellipseAngle)) / (A * sin(ellipseAngle) + B * cos(ellipseAngle)));

            // when abs(tipAngle1) >= abs(tipAngle2), distalEnd = originalCrossPoint1, else proximalEnd = originalCrossPoint2, and vice versa
            Eigen::Vector2f proximalEnd = abs(tipAngle1) >= abs(tipAngle2) ? originalCrossPoint2 : originalCrossPoint1;
            Eigen::Vector2f distalEnd = abs(tipAngle1) >= abs(tipAngle2) ? originalCrossPoint1 : originalCrossPoint2;
            float tipAngle = 0.0;
            if (distalEnd(1) < proximalEnd(1)) // the rod is upward deflected
            {
                // choose the larger to be tipAngle
                tipAngle = abs(tipAngle1) >= abs(tipAngle2) ? tipAngle1 : tipAngle2;
                if (tipAngle < 0)
                {
                    tipAngle = -tipAngle;
                }
                else
                {
                    tipAngle = PI - tipAngle;
                }
            }
            else // the rod is downward deflected
            {
                // choose the larger to be tipAngle
                tipAngle = abs(tipAngle1) >= abs(tipAngle2) ? tipAngle1 : tipAngle2;
                if (tipAngle > 0)
                {
                    tipAngle = -tipAngle;
                }
                else
                {
                    tipAngle = -PI - tipAngle;
                }
            }

            // std::cout << "tipAngleEllipse = " << tipAngle << std::endl;

            // // print the ellipse
            // std::cout << "ellipse center = " << ellipse.center << ", ellipse size = " << ellipse.size << ", ellipse angle = " << ellipse.angle << std::endl;

            if (isImageShow)
            {
                // 在图像上绘制拟合椭圆
                // create a new image
                cv::Mat img1 = cv::Mat::zeros(Height, Width, CV_8UC3);
                // draw the nonzero coordinates
                for (int i = 0; i < nonzeroCoordinates.rows; i++)
                {
                    img1.at<cv::Vec3b>(nonzeroCoordinates.at<cv::Point>(i)) = cv::Vec3b(255, 255, 255);
                }
                // draw a cross at crossPoint
                for (int i = 0; i < Width; i++)
                {
                    img1.at<cv::Vec3b>(distalEnd(1), i) = cv::Vec3b(0, 0, 255);
                }
                for (int i = 0; i < Height; i++)
                {
                    img1.at<cv::Vec3b>(i, distalEnd(0)) = cv::Vec3b(0, 0, 255);
                }
                // draw the ellipse
                cv::ellipse(img1, ellipse, cv::Scalar(0, 0, 255), 1);
                // show the image
                cv::imshow("img1", img1);
                cv::waitKey(1);
            }

            return tipAngle;
        }
    }
}