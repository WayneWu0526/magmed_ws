#include "magmed_camera/imageProcess.h"
#include <opencv2/opencv.hpp>

namespace magmed_camera{

double imageProcess::getTipAngle(unsigned short Height, unsigned short Width, unsigned char * pData){

    // load image
    cv::Mat img = cv::Mat(Height, Width, CV_8UC3, pData);
    // turn BGR to gray
    cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    // threshold the image
    cv::threshold(img, img, 100, 255, cv::THRESH_BINARY);
    // show the image
    cv::imshow("img", img);
    cv::waitKey(1);

    return 0.0;
}

}