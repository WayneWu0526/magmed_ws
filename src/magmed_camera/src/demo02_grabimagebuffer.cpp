#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "/opt/MVS/include/MvCameraControl.h"
#include <ros/ros.h>
#include <opencv2/opencv.hpp>

bool g_bExit = false;
unsigned int g_nPayloadSize = 0;

bool PrintDeviceInfo(MV_CC_DEVICE_INFO* pstMVDevInfo)
{
    if (NULL == pstMVDevInfo)
    {
        ROS_INFO("The Pointer of pstMVDevInfo is NULL!\n");
        return false;
    }
    if (pstMVDevInfo->nTLayerType == MV_GIGE_DEVICE)
    {
        int nIp1 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0xff000000) >> 24);
        int nIp2 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x00ff0000) >> 16);
        int nIp3 = ((pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x0000ff00) >> 8);
        int nIp4 = (pstMVDevInfo->SpecialInfo.stGigEInfo.nCurrentIp & 0x000000ff);
        // ch:打印当前相机ip和用户自定义名字 | en:print current ip and user defined name
        ROS_INFO("CurrentIp: %d.%d.%d.%d\n" , nIp1, nIp2, nIp3, nIp4);
        ROS_INFO("UserDefinedName: %s\n\n" , pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        ROS_INFO("UserDefinedName: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
        ROS_INFO("Serial Number: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chSerialNumber);
        ROS_INFO("Device Number: %d\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.nDeviceNumber);
    }
    else
    {
        ROS_INFO("Not support.\n");
    }
    return true;
}
static  void* WorkThread(void* pUser)
{
    int nRet = MV_OK;
    unsigned char *pDataForRGB = NULL;
    MV_FRAME_OUT stOutFrame = {0};
    memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));
    while(1)
    {
        nRet = MV_CC_GetImageBuffer(pUser, &stOutFrame, 1000);
        if (nRet == MV_OK)
        {
            ROS_INFO("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
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
                        ROS_INFO("MV_CC_ConvertPixelType fail! nRet [%x]\n", nRet);
                        break;
                    }
            // imshow
            cv::Mat img = cv::Mat(stConvertParam.nHeight, stConvertParam.nWidth, CV_8UC3, pDataForRGB);
            cv::imshow("img", img);
            cv::waitKey(1);
        }
        else
        {
            ROS_INFO("No data[0x%x]\n", nRet);
        }
        if(NULL != stOutFrame.pBufAddr)
        {
            nRet = MV_CC_FreeImageBuffer(pUser, &stOutFrame);
            if(nRet != MV_OK)
            {
                ROS_INFO("Free Image Buffer fail! nRet [0x%x]\n", nRet);
            }
        }
        if(g_bExit)
        {
            break;
        }
    }
    if (pDataForRGB)
    {
        free(pDataForRGB);
        pDataForRGB = NULL;
    }
    return 0;
}
int main(int argc, char **argv)
{
    int nRet = MV_OK;
    void* handle = NULL;
    do 
    {
        // ch:枚举设备 | en:Enum device
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            ROS_INFO("Enum Devices fail! nRet [0x%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (unsigned int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                ROS_INFO("[device %d]:\n", i);
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
            ROS_INFO("Find No Devices!\n");
            break;
        }
        unsigned int nIndex = 0;
        ROS_INFO("Default camera index: %d", nIndex);
        // if (nIndex >= stDeviceList.nDeviceNum)
        // {
        //     ROS_INFO("Intput error!\n");
        //     break;
        // }
        // ch:选择设备并创建句柄 | en:Select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            ROS_INFO("Create Handle fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:打开设备 | en:Open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("Open Device fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:设置触发模式为off | en:Set trigger mode as off
        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            ROS_INFO("Set Trigger Mode fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:获取数据包大小 | en:Get payload size
        MVCC_INTVALUE stParam;
        memset(&stParam, 0, sizeof(MVCC_INTVALUE));
        nRet = MV_CC_GetIntValue(handle, "PayloadSize", &stParam);
        if (MV_OK != nRet)
        {
            ROS_INFO("Get PayloadSize fail! nRet [0x%x]\n", nRet);
            break;
        }
        g_nPayloadSize = stParam.nCurValue;
        // ch:开始取流 | en:Start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("Start Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL ,WorkThread , handle);
        if (nRet != 0)
        {
            ROS_INFO("thread create failed.ret = %d\n",nRet);
            break;
        }
        
        ros::init(argc, argv, "image_publisher");

        ros::NodeHandle nh;

        while(ros::ok())
        {

        }
        g_bExit = true;

        // ch:停止取流 | en:Stop grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("Stop Grabbing fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:关闭设备 | Close device
        nRet = MV_CC_CloseDevice(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("ClosDevice fail! nRet [0x%x]\n", nRet);
            break;
        }
        // ch:销毁句柄 | Destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("Destroy Handle fail! nRet [0x%x]\n", nRet);
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