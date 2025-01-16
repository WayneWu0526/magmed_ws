#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <unistd.h> // 类似于windows.h
#include <time.h>   // 常用标准库
#include "std_msgs/Float64.h"
#define JOINT_NUM 7

bool g_bExit = false;
double g_dPsi = 0.0;

// work thread
static void *WorkThread(void *pUser)
{
    int nRet = 0;
    const char *strIpAddress = "192.168.31.201";
    ros::Rate rate(100); // 推送周期是100Hz
    while (1)
    {
        double speeds[JOINT_NUM] = {0.0};
        speeds[6] = g_dPsi; // 3.0; // g_dPsi; // the sign of the speed is the direction of the joint
        std::cout << "g_dPsi: " << g_dPsi << std::endl;
        double acc = 0.3; // 1.0; // acceleration
        nRet = speedJ(speeds, acc, 0, strIpAddress);
        /* 控制指定 IP 地址的机械臂进入速度模式，关节空间运动。时间 t 为可选项，如果提供了 t
        值，控制指定 IP 地址的机械臂将在 t 时间后减速。如果没有提供时间 t 值，机械臂将在达
        到目标速度时减速。该函数调用后立即返回。停止运动需要调用 stop 函数。*/
        if (nRet < 0)
        {
            ROS_ERROR("speedJ failed! Return value = %d\n", nRet);
        }
        else
        {
            // printf("Activate speedJ model\n");
        }

        if (g_bExit)
        {
            break;
        }

        rate.sleep(); // 若不设置延时可能导致线程占用过高，socket满了，导致报错
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

void wait_move(const char *strIpAddress)
{
    M_SLEEP(20);
    while (true)
    {
        const char state = getRobotState(strIpAddress);
        if (state != 0)
        {
            break;
        }
        else
        {
            M_SLEEP(1);
        }
    }
    stop(strIpAddress);
}

void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress) // Heart beat server
{
    strIpAddress = "192.168.31.201";
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
    strIpAddress = "192.168.31.201";
    const char *strError = formatError(e); // 该函数后面会介绍
    ROS_ERROR("error code (%d):%s\n", e, strError);
}

void psiCallback(const std_msgs::Float64::ConstPtr &msg)
{
    ROS_INFO("Psi reseived: [%f]", msg->data);
    g_dPsi = msg->data;
}

int main(int argc, char *argv[])
{

    // 初始化ros节点
    ros::init(argc, argv, "dianaManipulator");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 创建发布对象
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/magmed_manipulator/magnetAngle", 1000);

    // subscribe the rotation angular velocity of the magnet
    ros::Subscriber sub = nh.subscribe("/magmed_controller/angularVelocity", 1000, psiCallback);
    // 发送频率
    ros::Rate rate(100);

    // 初始化 API，完成其他功能函数使用前的初始化准备工作。
    const char *strIpAddress = "192.168.31.201";
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.31.201", strlen("192.168.31.201"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;
    int nRet = 0;
    double joints[JOINT_NUM] = {0.0};

    do
    {
        nRet = initSrv(errorControl, logRobotState, pinfo); // 机械臂的心跳服务会一直向上位机发送信号，可以屏蔽
        if (nRet < 0)
        {
            ROS_ERROR("192.168.31.201 initSrv failed! nReturn value = %d\n", nRet);
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
            ROS_ERROR("releaseBrake failed! nReturn value = %d\n", nRet);
            break;
        }
        M_SLEEP(2000); // delay 2s

        // 开启前将机械臂移动到初始位置
        nRet = getJointPos(joints, strIpAddress);
        if (nRet < 0)
        {
            ROS_ERROR("getJointPos failed! Return value = %d\n", nRet);
            break;
        }
        joints[6] = 0.0; // 0.0;
        nRet = moveJToTarget(joints, 3, 3, strIpAddress);
        if (nRet < 0)
        {
            ROS_ERROR("moveLToTarget failed! Return value = %d\n", nRet);
            break;
        }
        wait_move(strIpAddress);

        // 启动机械臂的速度控制模式，该模式下，机械臂的运动由用户控制。
        pthread_t nThreadID;
        nRet = pthread_create(&nThreadID, NULL, WorkThread, NULL);
        if (nRet != 0)
        {
            ROS_ERROR("thread create failed.ret = %d\n", nRet);
            break;
        }

        while (ros::ok())
        {
            nRet = getJointPos(joints, strIpAddress);
            if (nRet < 0)
            {
                ROS_ERROR("getJointPos failed! Return value = %d\n", nRet);
                break;
            }
            else
            {
                std_msgs::Float64 msg;
                msg.data = joints[JOINT_NUM - 1]; // pub the angle of end_effector
                pub.publish(msg);

                // printf("getJointPos: %f, %f, %f, %f, %f, %f, %f\n", joints[0],
                // joints[1],joints[2],joints[3],joints[4],joints[5],joints[6]);
            }
            rate.sleep();

            ros::spinOnce();
        }
        g_bExit = true;

        // 关闭指定 IP 地址机械臂的抱闸，停止机械臂。
        nRet = holdBrake(strIpAddress);
        if (nRet < 0)
        {
            ROS_ERROR("holdBrake failed! nReturn value = %d\n", nRet);
            break;
        }

    } while (0);
    // 结束调用 API，用于结束时释放指定 IP 地址机械臂的资源。

    // 如果该函数未被调用就退出系统（例如客户端程序在运行期间崩溃），服务端将因为检测不到心跳而认为客户端异常掉线，直至客户端再次运行，重新连接。除此之外不会引起严重后果。
    destroySrv(strIpAddress);

    return 0;
}
