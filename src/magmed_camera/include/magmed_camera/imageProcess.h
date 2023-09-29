#ifndef _IMAGEPROCESS_H
#define _IMAGEPROCESS_H
#include <opencv2/opencv.hpp>

namespace magmed_camera
{
    class imageProcess
    {
    public:
        // getTipAngle函数：获取尖端偏转角，输入图像宽，高和数据存储指针，输出尖端偏转角度
        float getTipAngle(unsigned short Height, unsigned short Width, unsigned char * pData, int nFlag);
    private:
        float ellipticaFunFit(unsigned short Height, unsigned short Width, cv::Mat img, bool isImageShow);
    };
}

#endif