#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <ros/ros.h>
#include "std_msgs/String.h"
#include <cstring>
#include <iostream>
#include <pthread.h>
#include <unistd.h> // 类似于windows.h
#include <time.h> // 常用标准库
#include "std_msgs/Float64.h"
#define JOINT_NUM 7

bool g_bExit = false;

// M_SLEEP宏定义
void M_SLEEP(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
}

// work thread
static void* WorkThread(void* pUser)
{
    int nRet = 0;
    const char *strIpAddress = "192.168.10.75";
    while(1)
    {
        double speeds[JOINT_NUM] = {0.0};
        speeds[6]=0.0;
        double acc =0.0;
        const char* strIpAddress = "192.168.10.75";
        int ret = speedJ(speeds, acc, 0, strIpAddress);
        if(ret < 0)
        {
            printf("speedJ failed! Return value = %d\n", ret);
        }


        if(g_bExit)
        {
            break;
        }
    }
    stop(strIpAddress);
    return 0;
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
    // 初始化ros节点
    ros::init(argc, argv, "diana7api_connect_test");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 创建订阅对象
    ros::Rate rate(100);

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
        M_SLEEP(2000); // delay 2s

        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL, WorkThread, NULL);
        if (nRet != 0)
        {
            printf("thread create failed.ret = %d\n",nRet);
            break;
        }

        double dblMaxAcc[7]={0};
        nRet = getMechanicalMaxJointsAcc(dblMaxAcc, strIpAddress);
        printf("getMechanicalMaxJointsAcc ret = %d dblMaxAcc = {%f,%f,%f,%f,%f,%f,%f}\n"
        , nRet
        , dblMaxAcc[0]
        , dblMaxAcc[1]
        , dblMaxAcc[2]
        , dblMaxAcc[3]
        , dblMaxAcc[4]
        , dblMaxAcc[5]
        , dblMaxAcc[6]);

        double dblMaxVel[7]={0};
        nRet = getMechanicalMaxJointsVel(dblMaxVel, strIpAddress);
        printf("getMechanicalMaxJointsVel ret = %d dblMaxVel = {%f,%f,%f,%f,%f,%f,%f}\n"
        , nRet
        , dblMaxVel[0]
        , dblMaxVel[1]
        , dblMaxVel[2]
        , dblMaxVel[3]
        , dblMaxVel[4]
        , dblMaxVel[5]
        , dblMaxVel[6]);

        while(ros::ok())
        {
            rate.sleep();

            ros::spinOnce();
        }
        g_bExit = true;

        M_SLEEP(2000); // delay 2s 否则可能会报和socket有关的错误。初步猜测是因为线程还没退出，就调用了destroySrv，导致线程中的socket被关闭了。
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
