#ifndef DIANA_RUN_H
#define DIANA_RUN_H

#include <ros/ros.h>
#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include "magmed_msgs/RoboStates.h"
#include "magmed_msgs/GetRobotState.h"
#include "magmed_msgs/RoboJoints.h"

#define JOINTNUM 7
#define TCPNUM 6

class DianaSpeedSets
{
public:
    struct DianaJointSets
    {
        double acc = {0.5};
        double t = 0.0;
        bool realiable = true;
        DianaJointSets() {};
        DianaJointSets(double acc)
        {
            this->acc = acc;
        }
    } diana_jointsets;

    struct DianaTcpSets
    {
        double speeds[TCPNUM] = {0.0};
        double acc[2] = {0.0};
        DianaTcpSets() {};
        DianaTcpSets(double speeds[TCPNUM], double acc[2])
        {
            for (int i = 0; i < TCPNUM; ++i)
            {
                this->speeds[i] = speeds[i];
            }
            for (int i = 0; i < 2; ++i)
            {
                this->acc[i] = acc[i];
            }
        }
    } diana_tcpsets;

    DianaSpeedSets() {};
    DianaSpeedSets(DianaTcpSets diana_tcpsets, DianaJointSets diana_jointsets)
    {
        this->diana_tcpsets = diana_tcpsets;
        this->diana_jointsets = diana_jointsets;
    };
};

class JointVels
{
public:
    magmed_msgs::RoboJoints joint_vel;
    double joint_vel_array[JOINTNUM] = {0.0};
    JointVels();
    void feed(magmed_msgs::RoboJointsConstPtr pMsg);
};

class DianaStateManage
{
public:
    JointVels dsr_joint_vels;
    DianaSpeedSets pred_sets;

    DianaStateManage(ros::NodeHandle &nh) : nh(nh)
    {
        state = magmed_msgs::RoboStates::INIT; // 初始化状态为INIT
        dianajoints_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints", 10);
        get_state_server = nh.advertiseService("/magmed_manipulator/get_state", &DianaStateManage::getState, this);
        // 使用boost::bind()函数可以将类成员函数作为回调函数，将得到的joint_vel赋值给类成员变量joint_vel
        joint_vel_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_controller/joint_vels",
                                                              10,
                                                              boost::bind(&JointVels::feed, &dsr_joint_vels, _1));
    }
    bool getState(magmed_msgs::GetRobotState::Request &req, magmed_msgs::GetRobotState::Response &res)
    {
        res.response = state;
        return true;
    }
    int srvJointVels();
    int pubJoints();
    void run();

    static void logRobotState(StrRobotStateInfo *pinfo, const char *strIpAddress) // Heart beat server
    {
        strIpAddress = "192.168.10.75";
        static int staCnt = 1;
        if ((staCnt++ % 1000 == 0) && pinfo)
        {
            for (int i = 0; i < JOINTNUM; ++i)
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

    static void errorControl(int e, const char *strIpAddress)
    {
        strIpAddress = "192.168.10.75";
        const char *strError = formatError(e); // 该函数后面会介绍
        ROS_ERROR("error code (%d):%s\n", e, strError);
    }

    // M_SLEEP 函数用于延时，单位为毫秒

private:
    int state;
    int ret = 0;
    double joint_vels[JOINTNUM] = {0.0};
    magmed_msgs::RoboJoints robojoints;
    const char *strIpAddress = "192.168.10.75";
    void M_SLEEP(int milliseconds);
    void wait_move(const char *strIpAddress);
    ros::NodeHandle nh;
    ros::Publisher dianajoints_pub;
    ros::ServiceServer get_state_server;
    ros::Subscriber joint_vel_sub;
};

int DianaStateManage::srvJointVels()
{
    ret = speedJ_ex(dsr_joint_vels.joint_vel_array, pred_sets.diana_jointsets.acc,
                    pred_sets.diana_jointsets.t, pred_sets.diana_jointsets.realiable, strIpAddress);
    if (ret < 0)
    {
        ROS_ERROR("speedJ_ex failed! Return value = %d\n", ret);
        state = magmed_msgs::RoboStates::TERM;
        return -1;
    }
    return 0;
};

int DianaStateManage::pubJoints()
{
    // 获取joint角度
    // 发布joints
    double joints[JOINTNUM] = {0.0};
    ret = getJointPos(joints, strIpAddress);
    if (ret < 0)
    {
        ROS_ERROR("getJointPos failed! Return value = %d\n", ret);
        state = magmed_msgs::RoboStates::TERM;
        return -1;
    }
    for (int i = 0; i < JOINTNUM; ++i)
    {
        robojoints.joints.push_back(joints[i]);
    }
    dianajoints_pub.publish(robojoints);

    return 0;
};

void DianaStateManage::run()
{
    // set state to INIT
    state = magmed_msgs::RoboStates::INIT;

    // init diana7
    // 初始化 API，完成其他功能函数使用前的初始化准备工作。
    srv_net_st *pinfo = new srv_net_st();
    memset(pinfo->SrvIp, 0x00, sizeof(pinfo->SrvIp));
    memcpy(pinfo->SrvIp, "192.168.10.75", strlen("192.168.10.75"));
    pinfo->LocHeartbeatPort = 0;
    pinfo->LocRobotStatePort = 0;
    pinfo->LocSrvPort = 0;

    do
    {
        ret = initSrv(errorControl, logRobotState, pinfo);
        if (ret < 0)
        {
            ROS_ERROR("192.168.10.75 initSrv failed! nReturn value = %d\n", ret);
            state = magmed_msgs::RoboStates::TERM;
            break;
        }
        // 清除指定 IP 地址机械臂的错误信息。
        ret = cleanErrorInfo(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("cleanErrorInfo failed!\n");
            state = magmed_msgs::RoboStates::TERM;
            break;
        }
        // 打开指定 IP 地址机械臂的抱闸，启动机械臂。调用该接口后，需要调用者延时 2s后再做其他操作。
        ret = releaseBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("releaseBrake failed! return value = %d\n", ret);
            state = magmed_msgs::RoboStates::TERM;
            break;
        }
        M_SLEEP(2000); // delay 2s
        // 将机械臂移动到关节角度为 0 的位置
        double jointzero[JOINTNUM] = {0.0};
        ret = moveJToTarget(jointzero, 0.5, 0.5, strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("moveJToTarget failed! Return value = %d\n", ret);
            state = magmed_msgs::RoboStates::TERM;
            break;
        }
        wait_move(strIpAddress);
        // 从参数服务器中获取机械臂的初始位置
        double init_pos[TCPNUM] = {0.0};
        ret = moveJToPose(init_pos, 0.2, 0.2, nullptr, strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("moveLToTarget failed! Return value = %d\n", ret);
            state = magmed_msgs::RoboStates::TERM;
            break;
        }
        wait_move(strIpAddress);
        ROS_INFO("Initial pos arrived! Starting velocity control. \n");

        // set state to RUN
        state = magmed_msgs::RoboStates::RUN;

        ros::Rate rate(100);
        double joints[JOINTNUM] = {0.0};
        double poses[6] = {0.0};
        while (ros::ok())
        {
            // 发布磁体位置和角度
            ret = pubJoints();
            if (ret < 0)
            {
                ROS_ERROR("pubJoints failed! Return value = %d\n", ret);
                state = magmed_msgs::RoboStates::TERM;
                break;
            }
            ret = srvJointVels();
            if (ret < 0)
            {
                ROS_ERROR("srvJointVels failed! Return value = %d\n", ret);
                state = magmed_msgs::RoboStates::TERM;
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }

        // 关闭指定 IP 地址机械臂的抱闸，停止机械臂。
        ret = holdBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("holdBrake failed! return value = %d\n", ret);
            break;
        }

    } while (0);

    // 如果该函数未被调用就退出系统（例如客户端程序在运行期间崩溃），服务端将因为检测不到心跳而认为客户端异常掉线，直至客户端再次运行，重新连接。除此之外不会引起严重后果。
    destroySrv(strIpAddress);
    if (pinfo)
    {
        delete pinfo;
        pinfo = nullptr;
    }
};

// M_SLEEP 函数用于延时，单位为毫秒
void DianaStateManage::M_SLEEP(int milliseconds)
{
    struct timespec ts;
    ts.tv_sec = milliseconds / 1000;
    ts.tv_nsec = (milliseconds % 1000) * 1000000;
    nanosleep(&ts, NULL);
};

void DianaStateManage::wait_move(const char *strIpAddress)
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
};

JointVels::JointVels()
{
};

void JointVels::feed(magmed_msgs::RoboJointsConstPtr pMsg)
{
    joint_vel = *pMsg;
    // 将RoboJoints类型变量的*pMsg赋值给joint_vel_arra


    // convert to array
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_vel_array[i] = joint_vel.joints[i];
    }
};


#endif