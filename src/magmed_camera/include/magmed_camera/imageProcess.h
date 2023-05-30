#ifndef _IMAGEPROCESS_H
#define _IMAGEPROCESS_H

namespace magmed_camera
{
    class imageProcess
    {
    public:
        // getTipAngle函数：获取尖端偏转角，输入图像宽，高和数据存储指针，输出尖端偏转角度
        float getTipAngle(unsigned short Height, unsigned short Width, unsigned char * pData, bool isImageShow);
    };
}

#endif