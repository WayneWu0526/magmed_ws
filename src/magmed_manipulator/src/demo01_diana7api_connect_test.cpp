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
    double active_tcp[6] = {0.0, 0.0, -0.810, 0.0, 0.0, - M_PI / 2.0};
    double acc[2] = {0.10, 0.10}; // velocity/angular velocity
    double speeds[JOINT_NUM] = {0.0};

    // 发送频率
    ros::Rate rate(100); // 推送周期是100Hz

    while(1)
    {
        // speeds[5] = 0.5;
        // int nRet = speedL(speeds, acc, 0, active_tcp, strIpAddress);
        // if(nRet < 0)
        // {
        //     printf("speedL failed! Return value = %d\n", nRet);
        // }
        // else
        // {
        //     // printf("Activate speedL model\n");
        // }

        double default_tcp[16] = {0.0};
        nRet = getDefaultActiveTcp(default_tcp, strIpAddress);
        if(nRet < 0)
        {
            ROS_ERROR("getDefaultActiveTcp failed! Return value = %d\n", nRet);
        }
        std::cout << "default_tcp: ";
        for(int i = 0; i < 16; i++)
        {
            std::cout << default_tcp[i] << " ";
        }
        std::cout << std::endl;

        double pose[6] = {0.0};
        if (getDefaultActiveTcpPose (pose,strIpAddress) < 0)
        {
            ROS_INFO("Diana API getDefaultActiveTcpPose failed!\n");
        }
        else
        {
            printf("getDefaultActiveTcpPose: %f, %f, %f, %f, %f, %f\n", pose[0], pose[1], pose[2], 
                pose[3], pose[4], pose[5]);
        }

        // g_bExit = true;


        if(g_bExit)
        {
            break;
        }

        rate.sleep(); // 若不设置延时可能导致线程占用过高，socket满了，导致报错        
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

    return 0;
}
