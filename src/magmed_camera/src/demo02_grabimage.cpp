#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <pthread.h>
#include "/opt/MVS/include/MvCameraControl.h"
#include <ros/ros.h>

bool PrintDeviceInfo(MV_CC_DEVICE_INFO *pstMVDevInfo)
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
        ROS_INFO("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chModelName);
        ROS_INFO("CurrentIp: %d.%d.%d.%d\n", nIp1, nIp2, nIp3, nIp4);
        ROS_INFO("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stGigEInfo.chUserDefinedName);
    }
    else if (pstMVDevInfo->nTLayerType == MV_USB_DEVICE)
    {
        ROS_INFO("Device Model Name: %s\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chModelName);
        ROS_INFO("UserDefinedName: %s\n\n", pstMVDevInfo->SpecialInfo.stUsb3VInfo.chUserDefinedName);
    }
    else
    {
        ROS_INFO("Not support.\n");
    }
    return true;
}
int main(int argc, char  *argv[])
{
    int nRet = MV_OK;
    void *handle = NULL;
    unsigned char *pData = NULL;
    unsigned char *pDataForRGB = NULL;
    unsigned char *pDataForSaveImage = NULL;
    do
    {
        MV_CC_DEVICE_INFO_LIST stDeviceList;
        memset(&stDeviceList, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
        // 枚举设备
        // enum device
        nRet = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &stDeviceList);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_EnumDevices fail! nRet [%x]\n", nRet);
            break;
        }
        if (stDeviceList.nDeviceNum > 0)
        {
            for (int i = 0; i < stDeviceList.nDeviceNum; i++)
            {
                ROS_INFO("[device %d]:\n", i);
                MV_CC_DEVICE_INFO *pDeviceInfo = stDeviceList.pDeviceInfo[i];
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
        // ROS_INFO("Please Intput camera index: ");
        unsigned int nIndex = 0; // 默认使用第0号摄像头
        // scanf("%d", &nIndex);
        if (nIndex >= stDeviceList.nDeviceNum)
        {
            ROS_INFO("Intput error!\n");
            break;
        }
        // 选择设备并创建句柄
        // select device and create handle
        nRet = MV_CC_CreateHandle(&handle, stDeviceList.pDeviceInfo[nIndex]);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_CreateHandle fail! nRet [%x]\n", nRet);
            break;
        }

        // 注册中断信号处理函数
        // 打开设备
        // open device
        nRet = MV_CC_OpenDevice(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_OpenDevice fail! nRet [%x]\n", nRet);
            break;
        }

        nRet = MV_CC_SetEnumValue(handle, "TriggerMode", 0);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_SetTriggerMode fail! nRet [%x]\n", nRet);
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
        // 开始取流
        // start grab image
        nRet = MV_CC_StartGrabbing(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_StartGrabbing fail! nRet [%x]\n", nRet);
            break;
        }
        
        MV_FRAME_OUT stOutFrame = {0};
        memset(&stOutFrame, 0, sizeof(MV_FRAME_OUT));

        ros::init(argc, argv, "camera01");
        
        ros::NodeHandle nh;
        
        int ii = 0;
        while (ros::ok())
        {
            nRet = MV_CC_GetImageBuffer(handle, &stOutFrame, 1000);
            if (nRet == MV_OK)
            {
                ROS_INFO("Get One Frame: Width[%d], Height[%d], nFrameNum[%d]\n",
                       stOutFrame.stFrameInfo.nWidth, stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nFrameNum);
            }
            else
            {
                ROS_INFO("No data[0x%x]\n", nRet);
            }
            if (NULL != stOutFrame.pBufAddr)
            {   
                nRet = MV_CC_FreeImageBuffer(handle, &stOutFrame);
                if (nRet != MV_OK)
                {
                    ROS_INFO("Free Image Buffer fail! nRet [0x%x]\n", nRet); // 错误不用管，因为本身就无参数
                }
            }

            ros::spinOnce();
        }

        // 停止取流
        // end grab image
        nRet = MV_CC_StopGrabbing(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_StopGrabbing fail! nRet [%x]\n", nRet);
        }
        // 销毁句柄
        // destroy handle
        nRet = MV_CC_DestroyHandle(handle);
        if (MV_OK != nRet)
        {
            ROS_INFO("MV_CC_DestroyHandle fail! nRet [%x]\n", nRet);
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
    if (pData)
    {
        free(pData);
        pData = NULL;
    }
    // if (pDataForRGB)
    // {
    //     free(pDataForRGB);
    //     pDataForRGB = NULL;
    // }
    // if (pDataForSaveImage)
    // {
    //     free(pDataForSaveImage);
    //     pDataForSaveImage = NULL;
    // }
    return 0;
}