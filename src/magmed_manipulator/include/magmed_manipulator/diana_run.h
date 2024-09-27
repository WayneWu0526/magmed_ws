#ifndef DIANA_RUN_H
#define DIANA_RUN_H

#include <ros/ros.h>
#include <DianaAPIDef.h>
#include <DianaAPI.h>
#include <thread>
#include "magmed_msgs/RoboStates.h"
#include "magmed_msgs/RoboJoints.h"

const int JOINTNUM = 7;
const int TCPNUM = 6;

class DianaSpeedSets
{
public:
    struct DianaJointSets
    {
        double acc = {0.5}; // set the acceleration
        double t = 0.0;
        bool realiable = false;
        DianaJointSets(){};
        DianaJointSets(double acc)
        {
            this->acc = acc;
        }
    } diana_jointsets;

    struct DianaTcpSets
    {
        double speeds[TCPNUM] = {0.0};
        double acc[2] = {0.0};
        DianaTcpSets(){};
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

    DianaSpeedSets(){};
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
    JointVels(){};
    void feed(magmed_msgs::RoboJointsConstPtr pMsg);
};

class DianaStateManage
{
public:
    DianaStateManage(ros::NodeHandle &nh) : nh(nh)
    {
        dianajoints_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints", 10);

        dianastate_pub = nh.advertise<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate", 10);

        joint_vel_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels",
                                                              10,
                                                              boost::bind(&JointVels::feed, &dsr_joint_vels, _1));
    }
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
                // printf("jointPos[%d] = %f \n", i, pinfo->jointPos[i]);
                // printf("jointCurrent [%d] = %f \n", i, pinfo->jointCurrent[i]);
                // printf("jointTorque [%d] = %f \n", i, pinfo->jointTorque[i]);
                // if (i < 6)
                // {
                //     printf("tcpPos [%d] = %f \n", i, pinfo->tcpPos[i]); // Tool Center Point;
                // }
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
    JointVels dsr_joint_vels;
    DianaSpeedSets pred_sets;

    magmed_msgs::RoboStates state;
    int ret = 0;

    double joint_vels[JOINTNUM] = {0.0};
    double joints[JOINTNUM] = {0.0};
    const char *strIpAddress = "192.168.10.75";
    void M_SLEEP(int milliseconds);
    void wait_move(const char *strIpAddress);
    ros::NodeHandle nh;
    ros::Publisher dianajoints_pub;
    ros::Publisher dianastate_pub;
    ros::Subscriber joint_vel_sub;

    int srvJointVels();
    int pubJoints();
    int mvJointZeros();
    int srvJointInit();
    void serviceLoop();
    int setJointsSoftLimit();
};

int DianaStateManage::srvJointInit()
{
    // 从参数服务器中获取机械臂的初始位置
    std::vector<double> initPose;
    if (nh.getParam("/magmed_controller/initPose", initPose))
    {
        // 检查获取的初始位置数据是否包含 TCPNUM 个值
        if (initPose.size() != TCPNUM)
        {
            ROS_ERROR("[magmed_manipulator] Parameter '/magmed_controller/initPose' should contain exactly %d values.", TCPNUM);
            return -1;
        }
    }
    else
    {
        // 参数获取失败
        ROS_ERROR_STREAM("[magmed_manipulator] Failed to retrieve initPose parameter. Make sure it's set.");
        return -1;
    }
    /****当SYSCTRLMODE=0时候设置*****/
    ret = moveJToPose(initPose.data(), 0.2, 0.2, nullptr, strIpAddress);
    // ret = 1;
    if (ret < 0)
    {
        ROS_ERROR("[magmed_manipulator] moveJToPose failed! Return value = %d\n", ret);
        state.VAL = magmed_msgs::RoboStates::TERM;
        return ret;
    }
    else{
        wait_move(strIpAddress);
        ROS_INFO("[magmed_manipulator] Initial pos arrived! Starting velocity control. \n");
    }
    return 0;
};

int DianaStateManage::mvJointZeros()
{
    // 将机械臂移动到关节角度为 0 的位置
    double jointzero[JOINTNUM] = {0.0};
    ret = moveJToTarget(jointzero, 0.1, 0.1, strIpAddress);
    if (ret < 0)
    {
        ROS_ERROR("[magmed_manipulator] moveJToTarget failed! Return value = %d\n", ret);
        state.VAL = magmed_msgs::RoboStates::TERM;
        return ret;
    }
    else
    {
        wait_move(strIpAddress);
    }
    return 0;
};

int DianaStateManage::srvJointVels()
{
    ret = speedJ_ex(dsr_joint_vels.joint_vel_array, pred_sets.diana_jointsets.acc,
                    pred_sets.diana_jointsets.t, pred_sets.diana_jointsets.realiable, strIpAddress);
    // ret = speedJ(dsr_joint_vels.joint_vel_array, pred_sets.diana_jointsets.acc,
    //                 pred_sets.diana_jointsets.t, strIpAddress);
    if (ret < 0)
    {
        ROS_ERROR("[magmed_manipulator] speedJ_ex failed! Return value = %d\n", ret);
        state.VAL = magmed_msgs::RoboStates::TERM;
        return -1;
    }
    return 0;
};

int DianaStateManage::pubJoints()
{
    // 获取joint角度
    // 发布joints
    ret = getJointPos(joints, strIpAddress);
    if (ret < 0)
    {
        ROS_ERROR("[magmed_manipulator] getJointPos failed! Return value = %d\n", ret);
        state.VAL = magmed_msgs::RoboStates::TERM;
        return -1;
    }
    magmed_msgs::RoboJoints robojoints;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        // push_back()函数用于在vector的尾部添加一个数据
        robojoints.joints.push_back(joints[i]);
    }
    dianajoints_pub.publish(robojoints);

    return 0;
};

void DianaStateManage::serviceLoop()
{
    // pub dianastate
    ros::Rate rate(1);
    while (ros::ok())
    {
        dianastate_pub.publish(state);
        ros::spinOnce();
        rate.sleep();
    }
};

int DianaStateManage::setJointsSoftLimit()
{
    double dblMinPos[7] = {0}, dblMaxPos[7]={0};
    // int ret = getJointsSoftLimitRange(dblMinPos, dblMaxPos, strIpAddress);
    // for (int i = 0; i < 7; ++i) {
    // dblMinPos[i];
    // dblMaxPos[i];
    // }ROS_WARN("[magmed_manipulator] getJointsSoftLimitRange return %d\n",ret);
    // // print all the joint limits
    // for (int i = 0; i < 7; ++i) {
    //     ROS_INFO("Joint %d: MinPos = %f, MaxPos = %f\n", i, dblMinPos[i], dblMaxPos[i]);
    // }
    // dblMinPos[0] = -178.00 / M_PI, dblMaxPos[0] = 178.00 / M_PI;
    // dblMinPos[1] = -89.00 / M_PI, dblMaxPos[1] = 89.00 / M_PI;
    // dblMinPos[2] = -178.00 / M_PI, dblMaxPos[2] = 178.00 / M_PI;
    // dblMinPos[3] = 0.10 / M_PI, dblMaxPos[3] = 174.00 / M_PI;
    // dblMinPos[4] = -178.00 / M_PI, dblMaxPos[4] = 178.00 / M_PI;
    // dblMinPos[5] = -178.00 / M_PI, dblMaxPos[5] = 178.00 / M_PI;
    // dblMinPos[6] = -178.00 / M_PI, dblMaxPos[6] = 178.00 / M_PI;
    // ret = setJointsSoftLimitRange(dblMinPos, dblMaxPos, strIpAddress);
    // if(ret < 0){
    //     ROS_ERROR("[magmed_manipulator] setJointsSoftLimitRange failed! Return value = %d\n", ret);
    //     state.VAL = magmed_msgs::RoboStates::TERM;
    //     return -1;
    // }
    int ret = getJointsSoftLimitRange(dblMinPos, dblMaxPos, strIpAddress);
    for (int i = 0; i < 7; ++i) {
        dblMinPos[i]--;
        dblMaxPos[i]++;
    }
    setJointsSoftLimitRange(dblMinPos, dblMaxPos, strIpAddress);
    return 0;
}

void DianaStateManage::run()
{
    // set state to INIT
    state.VAL = magmed_msgs::RoboStates::INIT;
    // state.VAL = magmed_msgs::RoboStates::RUN;
    std::thread serviceThread(&DianaStateManage::serviceLoop, this);
    // 如果线程创建失败，那么就退出系统。
    if (!serviceThread.joinable())
    {
        ROS_ERROR("[magmed_manipulator] serviceThread create failed!\n");
        state.VAL = magmed_msgs::RoboStates::TERM;
        return;
    }
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
            ROS_ERROR("[magmed_manipulator] 192.168.10.75 initSrv failed! nReturn value = %d\n", ret);
            state.VAL = magmed_msgs::RoboStates::TERM;
            break;
        }
        // 清除指定 IP 地址机械臂的错误信息。
        ret = cleanErrorInfo(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("[magmed_manipulator] cleanErrorInfo failed!\n");
            state.VAL = magmed_msgs::RoboStates::TERM;
            break;
        }
        // 打开指定 IP 地址机械臂的抱闸，启动机械臂。调用该接口后，需要调用者延时 2s后再做其他操作。
        ret = releaseBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("[magmed_manipulator] releaseBrake failed! return value = %d\n", ret);
            state.VAL = magmed_msgs::RoboStates::TERM;
            break;
        }
        M_SLEEP(2000); // delay 2s

        // ret = setJointsSoftLimit();
        // if (ret < 0)
        // {
        //     ROS_ERROR("[magmed_manipulator] setJointsSoftLimit failed! return value = %d\n", ret);
        //     state.VAL = magmed_msgs::RoboStates::TERM;
        //     break;
        // }

        if (state.VAL == magmed_msgs::RoboStates::INIT)
        {
            // 将机械臂移动到关节角度为 0 的位置
            /******* [note]: 当SYSCTRLMODE = 0的时候，如下不设置。*******/
            // if (mvJointZeros() < 0)
            // {
            //     ROS_ERROR("[magmed_manipulator] move joints to zeros failed, return value = %d\n", ret);
            //     state.VAL = magmed_msgs::RoboStates::TERM;
            //     break;
            // }

            // 将机械臂移动到TCP为 init 的位置
            if (srvJointInit() < 0)
            {
                ROS_ERROR("[magmed_manipulator] srv joint to initstates failed, return value = %d\n", ret);
                state.VAL = magmed_msgs::RoboStates::TERM;
                break;
            }
        }
        // set state to RUN
        ros::Rate rate(100);
        state.VAL = magmed_msgs::RoboStates::RUN;
        while (ros::ok())
        {
            // check state
            const char dianastate = getRobotState(strIpAddress);
            if (dianastate == 6)
            {
                ROS_ERROR("[magmed_manipulator] Robot state error! Return value = %d\n", dianastate);
                state.VAL = magmed_msgs::RoboStates::TERM;
                break;
            }
            else if (dianastate == 7)
            {
                ROS_ERROR("[magmed_manipulator] Joint out of range! Return value = %d\n", dianastate);
                state.VAL = magmed_msgs::RoboStates::TERM;
                break;
            }
            else
            {
                // ROS_INFO("Robot state: %d\n", dianastate);
            }
            // 发布机械臂各个关节
            ret = pubJoints();
            if (ret < 0)
            {
                ROS_ERROR("[magmed_manipulator] pubJoints failed! Return value = %d\n", ret);
                state.VAL = magmed_msgs::RoboStates::TERM;
                break;
            }
            ret = srvJointVels();
            if (ret < 0)
            {
                ROS_ERROR("[magmed_manipulator] srvJointVels failed! Return value = %d\n", ret);
                state.VAL = magmed_msgs::RoboStates::TERM;
                break;
            }

            ros::spinOnce();
            rate.sleep();
        }
        // 关闭指定 IP 地址机械臂的抱闸，停止机械臂。
        ret = holdBrake(strIpAddress);
        if (ret < 0)
        {
            ROS_ERROR("[magmed_manipulator] holdBrake failed! return value = %d\n", ret);
            break;
        }

    } while (0);
    // set state to TERM
    state.VAL = magmed_msgs::RoboStates::TERM;
    // 等待线程结束
    serviceThread.join();

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
        const char dianastate = getRobotState(strIpAddress);
        if (dianastate != 0)
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

void JointVels::feed(magmed_msgs::RoboJointsConstPtr pMsg)
{
    joint_vel = *pMsg;
    // 将RoboJoints类型变量的*pMsg赋值给joint_vel_arra

    // convert to array
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_vel_array[i] = joint_vel.joints[i];
        // printf("joint_vel_array[%d] = %f\n", i, joint_vel_array[i]);
    }
};

#endif