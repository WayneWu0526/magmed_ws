#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <cstring>
#include <iostream>
#include <unistd.h> // 类似于windows.h
#include <time.h> // 常用标准库
#define JOINT_NUM 7

// M_SLEEP宏定义
void M_SLEEP(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress) // Heart beat server
{
    strIpAddress = "192.168.10.75";
    static int staCnt = 1;
    if ((staCnt++ % 1000 == 0) && pinfo)
    {
        for (int i = 0; i < JOINT_NUM; ++i)
        {
            printf("jointPos[%d] = %f \n", i, pinfo->jointPos[i]);
            printf("jointCurrent [%d] = %f \n", i, pinfo->jointCurrent[i]);
            printf("jointTorque [%d] = %f \n", i, pinfo->jointTorque[i]);
            if (i < 6)
            {
                printf("tcpPos [%d] = %f \n", i, pinfo->tcpPos[i]); // Tool Center Point;
            }
        }
    }
}

void errorControl(int e, const char *strIpAddress)
{
    strIpAddress = "192.168.10.75";
    const char *strError = formatError(e); // 该函数后面会介绍
    printf("error code (%d):%s\n", e, strError);
}

int main(int argc, char *argv[])
{
    // 初始化 API，完成其他功能函数使用前的初始化准备工作。
    const char *strIpAddress = "192.168.10.75";
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.10.75", strlen("192.168.10.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int nRet = 0;

    do{
        nRet = initSrv(errorControl, logRobotState, pinfo); // 机械臂的心跳服务会一直向上位机发送信号，可以屏蔽
        if (nRet < 0)
        {
            printf("192.168.10.75 initSrv failed! nReturn value = %d\n", nRet);
            break;
        }
        if (pinfo)
        {
            delete pinfo;
            pinfo = nullptr;
        }
        // 打开指定 IP 地址机械臂的抱闸，启动机械臂。调用该接口后，需要调用者延时 2s后再做其他操作。
        nRet = releaseBrake(strIpAddress);
        if (nRet < 0)
        {
            printf("releaseBrake failed! nReturn value = %d\n", nRet);
            break;
        }
        M_SLEEP(2000);

        // 初始化ros节点
        ros::init(argc, argv, "diana7api_connect_test");
        // 创建节点句柄
        ros::NodeHandle nh;
        // 创建发布对象
        ros::Rate rate(1);

        joint_direction_e dtype = T_MOVE_UP;
        double vel = 0.5;
        double acc = 0.5;
        int index = 0;

        while(ros::ok())
        {
            ros::Rate rate(1);
        }

        /*
            codes
        */

        // double joints[JOINT_NUM] = {0.0};
        // nRet = getJointPos(joints, strIpAddress);
        // if (nRet < 0)
        // {
        //     printf("getJointPos failed! nReturn value =%d\n", nRet);
        // }
        // else
        // {
        //     printf("getJointPos: %f, %f, %f, %f, %f, %f, %f\n", joints[0],
        //            joints[1], joints[2], joints[3], joints[4], joints[5], joints[6]);
        // }

        nRet = moveJoint(dtype, index, vel, acc, strIpAddress);
        if (nRet < 0)
        {
            printf("moveJoint failed! nReturn value = %d\n", nRet);
            break;
        }
        M_SLEEP(4000);
        stop(strIpAddress);

        // 关闭指定 IP 地址机械臂的抱闸，停止机械臂。
        nRet = holdBrake(strIpAddress);
        if (nRet < 0)
        {
            printf("holdBrake failed! nReturn value = %d\n", nRet);
            break;
        }

    }while(0);
    // 结束调用 API，用于结束时释放指定 IP 地址机械臂的资源。
    // 如果该函数未被调用就退出系统（例如客户端程序在运行期间崩溃），服务端将因为检测不到心跳而认为客户端异常掉线，直至客户端再次运行，重新连接。除此之外不会引起严重后果。
    destroySrv(strIpAddress);
}
