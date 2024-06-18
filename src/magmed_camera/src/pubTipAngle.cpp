#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include <MvCameraControl.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <std_msgs/Float64.h>
#include "magmed_camera/imageProcess.hpp"

// #include <time.h>

bool g_bExit = false;
unsigned int g_nPayloadSize = 0;
float g_fTipAngle = 0.0;
int g_nFlag = 0; // 0: image test mode; 1: data feedback mode; 2: data feedback (image show) mode

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        printf("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        printf("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        printf("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        printf("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        printf("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        printf("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        printf("Not support.\n");
    }
    return true;
}

static  void* WorkThread(void* pUser) // 工作线程 | work thread
{
    int nRet = MV_OK;
    unsigned char *pDataForRGB = NULL;
    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

    // clock_t start, finish;
    // start = clock();
    while(1)
    {
        nRet = MV_CC_GetImageBuffer(pUser, &stOutFrame, 1000);
        if (nRet == MV_OK)
        {
            printf("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
                stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);

            pDataForRGB = (unsigned char*)malloc(stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight * 4 + 2048);
            if (NULL == pDataForRGB)
            {
                break;
            }
            // 像素格式转换
            // convert pixel format 
            MV_CC_PIXEL_CONVERT_PARAM stConvertParam = {0};
            // 从上到下依次是：图像宽，图像高，输入数据缓存，输入数据大小，源像素格式，
            // 目标像素格式，输出数据缓存，提供的输出缓冲区大小
            // Top to bottom are：image width, image height, input data buffer, input data size, source pixel format, 
            // destination pixel format, output data buffer, provided output buffer size
            stConvertParam.nWidth = stOutFrame.stFrameInfo.nWidth;
            stConvertParam.nHeight = stOutFrame.stFrameInfo.nHeight;
            stConvertParam.pSrcData = stOutFrame.pBufAddr;
            stConvertParam.nSrcDataLen = stOutFrame.stFrameInfo.nFrameLen;
            stConvertParam.enSrcPixelType = stOutFrame.stFrameInfo.enPixelType;
            stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed;
            stConvertParam.pDstBuffer = pDataForRGB;
            stConvertParam.nDstBufferSize = stOutFrame.stFrameInfo.nWidth * stOutFrame.stFrameInfo.nHeight *  4 + 2048;
            nRet = MV_CC_ConvertPixelType(pUser, &stConvertParam);
            if (MV_OK != nRet)
            {
                printf("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                break;
            }
            magmed_camera::imageProcess imageProcess;
            g_fTipAngle = imageProcess.getTipAngle(stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth, pDataForRGB, g_nFlag);      
        }
        else
        {
            printf("No data[0x%x]\n", nRet);
        }
        if(NULL != stOutFrame.pBufAddr)
        {
            nRet = MV_CC_FreeImageBuffer(pUser, &stOutFrame);
            if(nRet != MV_OK)
            {
                printf("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
        }
        if(pDataForRGB)
        {
            free(pDataForRGB);
            pDataForRGB = NULL;
        }
        if(g_bExit)
        {
            break;
        }
        // if(stOutFrame.stFrameInfo.nFrameNum > 1000)
        // {
        //     finish = clock();
        //     std::cout << "time: " << (finish - start) / CLOCKS_PER_SEC << std::endl;
        //     break;
        // }
    }
    return 0;
}

int main(int argc, char **argv)
{
    int nRet = MV_OK;
    void* handle = NULL;

    ros::init(argc, argv, "pubTipAngle");

    ros::NodeHandle nh;

    nh.param<int>("nFlag", g_nFlag, 1); // 这里修改模式 | change the mode here
    // whether to show the image
    switch(g_nFlag){
        case 0:
            ROS_INFO("Image test mode activated!\n");
            break;
        case 1:
            ROS_INFO("Data feedback mode activated!\n");
            break;
        case 2:
            ROS_INFO("Data feedback (image show) mode activated!n");
            break;
        default:
            ROS_WARN("Invalid mode!\n");
    }

    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/magmed_camera/tipAngle", 1000);
    
    // publish the tip angle at 100Hz
    ros::Rate rate(100);

    do 
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            printf("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                printf("[device %d]:\n", i);
                MV_CC_DEVICE_INFO* pDeviceInfo = stDeviceList.pDeviceInfo[i];
                if (NULL == pDeviceInfo)
                {
                    break;
                } 
                PrintDeviceInfo(pDeviceInfo);            
            }  
        } 
        else
        {
            printf("Find No Devices!\n");
            break;
        }
        unsigned int nIndex = 0;
        printf("Default camera index: %d\n", nIndex);
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            printf("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            printf("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            printf("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            printf("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }
        g_nPayloadSize = stParam.nCurValue;
        // ch:设置ROI大小
         // 设置int型变量
        // set IInteger variablet
        // 宽高设置时需考虑步进(16)，即设置宽高需16的倍数
        // Step (16) should be considered when setting width and height, that is the width and height should be a multiple of 16
        nRet = MV_CC_SetIntValue(handle, "Width", 16*25);    
        if (MV_OK != nRet)
        {
            printf("set width failed! nRet [%x]\n\n", nRet);
            break;
        }
        nRet = MV_CC_SetIntValue(handle, "OffsetX", 16*60);    
        if (MV_OK != nRet)
        {
            printf("set OffsetX failed! nRet [%x]\n\n", nRet);
            break;
        }
        nRet = MV_CC_SetIntValue(handle, "Height", 16*25);    
        if (MV_OK != nRet)
        {
            printf("set height failed! nRet [%x]\n\n", nRet);
            break;
        }
        nRet = MV_CC_SetIntValue(handle, "OffsetY", 16*80);    
        if (MV_OK != nRet)
        {
            printf("set OffsetY failed! nRet [%x]\n\n", nRet);
            break;
        }
        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }
        // wait for the camera to start
        ros::Duration(3.0).sleep();

        while(ros::ok())
        {
            std_msgs::Float64 msg;
            msg.data = g_fTipAngle;
            pub.publish(msg);
            // ROS_INFO("tipAngle: %f", msg.data);
            rate.sleep();
            ros::spinOnce();
        }
        g_bExit = true;

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            printf("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            printf("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            printf("Destroy Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
    } while (0);
    
    if (nRet != MV_OK)
    {
        if (handle != NULL)
        {
            MV_CC_DestroyHandle(handle);
            handle = NULL;
        }
    }
    return 0;
}