#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <unistd.h> // 类似于windows.h
#include <time.h> // 常用标准库
#include <geometry_msgs/Twist.h>
#include <cmath>
#define JOINT_NUM 7

bool g_bExit = false;
double g_speeds[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

// work thread
static void* WorkThread(void* pUser)
{
    int nRet = 0;
    const char *strIpAddress = "192.168.10.75";
    while(1)
    {
        double acc[2] = {0.30, 0.50}; // 1.0; // acceleration
        double speed[6] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
        nRet = speedL(g_speeds, acc, 0, nullptr, strIpAddress);
        /* 控制指定 IP 地址的机械臂进入速度模式，关节空间运动。时间 t 为可选项，如果提供了 t
        值，控制指定 IP 地址的机械臂将在 t 时间后减速。如果没有提供时间 t 值，机械臂将在达
        到目标速度时减速。该函数调用后立即返回。停止运动需要调用 stop 函数。*/
        if(nRet < 0)
        {
            printf("speedJ failed! Return value = %d\n", nRet);
        }
        else
        {
            // printf("Activate speedJ model\n");
        }

        if(g_bExit)
        {
            break;
        }
    }
    stop(strIpAddress);
    return 0;
}

// M_SLEEP 函数用于延时，单位为毫秒
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

void twistCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float MAX_LINEAR_SPEED = 3.0;
    float MAX_ANGULAR_SPEED = 3.0;
    // if msg->linear.x < 0.1
    g_speeds[0] = MAX_LINEAR_SPEED * (msg->linear.x);
    g_speeds[1] = MAX_LINEAR_SPEED * (msg->linear.y);
    g_speeds[2] = MAX_LINEAR_SPEED * (msg->linear.z);
    g_speeds[3] = MAX_ANGULAR_SPEED * (msg->angular.x);
    g_speeds[4] = MAX_ANGULAR_SPEED * (msg->angular.y);
    g_speeds[5] =MAX_ANGULAR_SPEED * (msg->angular.z);
    for(int i=0; i < 6; i++)
    {
        std::cout << g_speeds[i] << " ";
    }
    std::cout << std::endl;
}

int main(int argc, char *argv[])
{

    // 初始化ros节点
    ros::init(argc, argv, "dianaManipulator");
    // 创建节点句柄
    ros::NodeHandle nh;

    // subscribe the rotation angular velocity of the magnet
    ros::Subscriber sub = nh.subscribe("/magmed_joystick/joystick_controller", 1000, twistCallback);
    // 发送频率
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
