#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <ros/ros.h>
#include <cstring>
#include <iostream>
#include <unistd.h> // 类似于windows.h
#include <time.h>   // 常用标准库
#include "std_msgs/Float64MultiArray.h"
#include "magmed_msgs/MagPose.h"
#include "magmed_controller/MSCRJacobi.h"
#include <eigen3/Eigen/Dense>
#define JOINT_NUM 7

bool g_bExit = false;
double g_dTwist[6] = {0.0}; // Note: twist is defined as [Vx, Vy, Vz, Wx, Wy, Wz];
double g_dJoints[JOINT_NUM] = {0.0};
JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();

// receive the rotation angular velocity of the magnet
// Note: receive msg is defined as [Vx, Vy, Vz, Wz];
// Note: send msg is defined as [Wz, Vx, Vy, Vz];
void magPosCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    ROS_INFO("Pos reseived: [%f], [%f], [%f], [%f]", msg->data[0], msg->data[1], msg->data[2], msg->data[3]);
    // Note: receive msg order: [Vx, Vy, Vz, Wz];
    g_dTwist[0] = msg->data[0];
    g_dTwist[1] = msg->data[1];
    g_dTwist[2] = msg->data[2];
    g_dTwist[5] = msg->data[3];
    // g_dJoints[JOINT_NUM - 1] = msg->data[3];
}

// work thread
static void *WorkThread(void *pUser)
{
    int ret = 0;
    const char *strIpAddress = "192.168.10.75";
    ros::Rate rate(100); // 推送周期是100Hz
    while (1)
    {
        double speeds[6] = {0.05, 0.0, 0.0, 0.0, 0.0, 0.3}; // 3.0;
        // the sign of the speed is the direction of the joint
        // std::cout << "g_dPsi: " << g_dPsi << std::endl;
        double acc[2] = {0.005, 0.50}; //acceleration
        // replace speeds with g_dTwist
        ret = speedLOnTcp(g_dTwist, acc, 0, nullptr, strIpAddress);
        // ret = speedLOnTcp(speeds, acc, 0, nullptr, strIpAddress);
        /* 控制指定 IP 地址的机械臂进入速度模式，关节空间运动。时间 t 为可选项，如果提供了 t
        值，控制指定 IP 地址的机械臂将在 t 时间后减速。如果没有提供时间 t 值，机械臂将在达
        到目标速度时减速。该函数调用后立即返回。停止运动需要调用 stop 函数。*/
        if (ret < 0)
        {
            ROS_ERROR("speedLOnTcp failed! Return value = %d\n", ret);
        }
        else
        {
            // printf("Activate speedJ model\n");
        }
        // ret = speedJ(g_dJoints, 0.3, 0, strIpAddress);
        // if (ret < 0)
        // {
        //     ROS_ERROR("speedJ failed! Return value = %d\n", ret);
        // }
        // else
        // {
        //     // printf("Activate speedJ model\n");
        // }
        rate.sleep(); // 若不设置延时可能导致线程占用过高，socket满了，导致报错

        if (g_bExit)
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
    strIpAddress = "192.168.10.75";
    static int staCnt = 1;
    if ((staCnt++ % 1000 == 0) && pinfo)
    {
        for (int i = 0; i < JOINT_NUM; ++i)
        {
            // Heartbeat serve
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
    ROS_ERROR("error code (%d):%s\n", e, strError);
}

int main(int argc, char *argv[])
{

    // 初始化ros节点
    ros::init(argc, argv, "dianaManipulator");
    // 创建节点句柄
    ros::NodeHandle nh;
    // 创建发布对象
    ros::Publisher pub = nh.advertise<magmed_msgs::MagPose>("/magmed_manipulator/magPos", 1000);

    // subscribe the rotation angular velocity of the magnet
    ros::Subscriber sub = nh.subscribe("/magmed_controller/magVel", 1000, magPosCallback);
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
    double joints[JOINT_NUM] = {0.0};
    double poses[6] = {0.0};
    int ret = 0;

    // 机械臂坐标系到工作空间坐标系的转换矩阵
    // 需要提供机器人坐标与机械臂的相对位置
    Eigen::Matrix4d Tsg_eigen;
    Tsg_eigen << 0.0, 0.0, 1.0, 0.51, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
        1.0, 0.0, 0.0, 0.0,           // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
        0.0, 1.0, 0.0, 0.06,          // 底盘高度：0.06m
        0.0, 0.0, 0.0, 1.0;
    // 初始位置矩阵
    Eigen::Matrix4d Tgd0_eigen;            // 考虑上参数空间获取机器人长度，后期改为加装外部摄像头获得穿刺位置
    Tgd0_eigen << -1.0, 0.0, 0.0, pr.L, // 机器人长度
        0.0, 1.0, 0.0, pr.H0,             // 默认高度
        0.0, 0.0, -1.0, 0.0,               // z轴垂直且不允许移动
        0.0, 0.0, 0.0, 1.0;
    Eigen::Matrix4d Tsd0_eigen = Tsg_eigen * Tgd0_eigen;
    double Tsd0[16] = {0.0};
    Eigen::Map<Eigen::MatrixXd>(Tsd0, 4, 4) = Tsd0_eigen;

    // 初始pos矩阵
    double pos0[6] = {0.0};
    ret = homogeneous2Pose(Tsd0, pos0);

    do
    {
        ret = initSrv(errorControl, logRobotState, pinfo); // 机械臂的心跳服务会一直向上位机发送信号，可以屏蔽
        if (ret < 0)
        {
            ROS_ERROR("192.168.10.75 initSrv failed! return value = %d\n", ret);
            break;
        }
        if (pinfo)
        {
            delete pinfo;
            pinfo = nullptr;
        }
        // 清除指定 IP 地址机械臂的错误信息。
        ret = cleanErrorInfo(strIpAddress);
        if(ret < 0)
        {
            ROS_ERROR("cleanErrorInfo failed!\n");
            break;
        }
        // 打开指定 IP 地址机械臂的抱闸，启动机械臂。调用该接口后，需要调用者延时 2s后再做其他操作。
        ret = releaseBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("releaseBrake failed! return value = %d\n", ret);
            break;
        }
        M_SLEEP(2000); // delay 2s
        ROS_INFO("------------------------\n");
        ROS_INFO("Moving to initial poses.\n");
        ROS_INFO("------------------------\n");

        // 开机前将机械臂移动到初始位置
        ret = moveJToTarget(joints, 0.5, 0.5, strIpAddress);
        wait_move(strIpAddress);
        ROS_INFO("------------------------\n");
        ROS_INFO("Moving to target poses.\n");
        ROS_INFO("------------------------\n");
        // 再将机械臂移动到初始位置
        ret = moveJToPose(pos0, 0.2, 0.2, nullptr, strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("moveLToTarget failed! Return value = %d\n", ret);
            break;
        }
        wait_move(strIpAddress);
        ROS_INFO("Initial pos arrived! Starting velocity control. \n");

        // 启动机械臂的速度控制模式，该模式下，机械臂的运动由用户控制。
        pthread_t nThreadID;
        ret = pthread_create(&nThreadID, NULL, WorkThread, NULL);
        if (ret != 0)
        {
            ROS_ERROR("thread create failed.ret = %d\n", ret);
            break;
        }

        // ros main codes: publish magnet poses
        magmed_msgs::MagPose magPose;
        while (ros::ok())
        {

            ret = getJointPos(joints, strIpAddress);
            if (ret < 0)
            {
                ROS_ERROR("getJointPos failed! Return value = %d\n", ret);
                break;
            }
            ret = getTcpPos(poses, strIpAddress);
            if (ret < 0)
            {
                ROS_ERROR("getTcpPos failed! Return value = %d\n", ret);
                break;
            }
            // transform poses from Tsb to Tgb (Tgb = Tsb * Tbg)
            double Tsb[16] = {0.0};
            ret = pose2Homogeneous(poses, Tsb);
            // use Eigen::Map to map 1x16 array to 4x4 matrix
            Eigen::Map<Eigen::Matrix<double, 4, 4>, Eigen::RowMajor> Tsb_eigen(Tsb);
            // std::cout << "Tsb_eigen:" << Tsb_eigen << std::endl;
            Eigen::Matrix<double, 4, 4> Tgb_eigen;
            Tgb_eigen = Tsg_eigen.inverse() * Tsb_eigen;

            magPose.psi = joints[JOINT_NUM - 1];
            magPose.pos = {Tgb_eigen(0, 3), Tgb_eigen(1, 3), Tgb_eigen(2, 3)};

            // diana的TCP后三个坐标以轴角表示。
            // std::cout << "Tgb_eigen:" << Tgb_eigen << std::endl;
            // printf("initPos: %f, %f, %f, %f, %f, %f\n", pos0[0], pos0[1],pos0[2],pos0[3],pos0[4],pos0[5]);
            // printf("getTcpPos: %f, %f, %f, %f, %f, %f\n", poses[0], poses[1],poses[2],poses[3],poses[4],poses[5]);
            // array_msg.data.push_back(joints[6]); // angle of the magnet

            pub.publish(magPose);

            ros::spinOnce();
            rate.sleep();

        }
        g_bExit = true;

        // 关闭指定 IP 地址机械臂的抱闸，停止机械臂。
        ret = holdBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("holdBrake failed! return value = %d\n", ret);
            break;
        }

    } while (0);
    // 结束调用 API，用于结束时释放指定 IP 地址机械臂的资源。

    // 如果该函数未被调用就退出系统（例如客户端程序在运行期间崩溃），服务端将因为检测不到心跳而认为客户端异常掉线，直至客户端再次运行，重新连接。除此之外不会引起严重后果。
    destroySrv(strIpAddress);

    return 0;
}