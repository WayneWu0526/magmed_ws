#pragma once

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/diffKine.h"
#include "magmed_controller/optCtrl.h"

const int SYSCTRLMODE = 0; // 0：手动版本，1：自动版本

// camera feedback
class TipAngle
{
public:
    magmed_msgs::TipAngle tip_angle;
    TipAngle() { tip_angle.tipAngle = 0.0; };
    void feed(magmed_msgs::TipAngleConstPtr pMsg);
};

// joystick input (manual control)
class Joystick
{
public:
    magmed_msgs::PFjoystick joystick;
    Feeder feeder;
    DianaTcp dianaTcp;
    MagCR magCR;
    void feed(magmed_msgs::PFjoystickConstPtr pMsg);
    Joystick(){};
};

// diana states
class Diana
{
public:
    magmed_msgs::RoboStates robo_state;
    double joint_states_array[JOINTNUM] = {0.0};
    magmed_msgs::RoboJoints joint_states;
    void feedState(magmed_msgs::RoboStatesConstPtr pMsg);
    void feedJoints(magmed_msgs::RoboJointsConstPtr pMsg);
    Diana(){};
};

// velocity control algorithm
class VelCtrlNode
{
public:
    void run();

    VelCtrlNode(){};
    VelCtrlNode(ros::NodeHandle &nh) : nh(nh)
    {
        joystick_sub = nh.subscribe<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller",
                                                             10,
                                                             boost::bind(&Joystick::feed, &joystick, _1));

        tipAngle_sub = nh.subscribe<magmed_msgs::TipAngle>("/magmed_camera/tipAngle",
                                                           10,
                                                           boost::bind(&TipAngle::feed, &tipAngle, _1));

        diana_jointStates_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints",
                                                                      10,
                                                                      boost::bind(&Diana::feedJoints, &diana, _1));

        diana_roboState_sub = nh.subscribe<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate",
                                                                    10,
                                                                    boost::bind(&Diana::feedState, &diana, _1));

        diana_jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels", 10);

        feeder_vel_pub = nh.advertise<std_msgs::UInt32>("/magmed_feeder/vel", 10);

        selfcollision_client = nh.serviceClient<magmed_msgs::SelfCollisionCheck>("/magmed_modules/selfCollisionCheck"); 
        // ros::service::waitForService("/magmed_manipulator/roboStates", -1);
    };

private:
    Joystick joystick;
    TipAngle tipAngle;
    Diana diana;

    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    ros::Subscriber tipAngle_sub;
    ros::Subscriber diana_jointStates_sub;
    ros::Subscriber diana_roboState_sub;
    ros::Publisher diana_jointVels_pub;
    ros::Publisher feeder_vel_pub;
    ros::ServiceClient selfcollision_client;

    optCtrl optctrl;
    diffKine diffkine;
    int pubVels();
    int loadInitPose();
    int nRet = 0;
    int CTRLMODE = enum_CTRLMODE::NM;
    bool SCC_FLAG = false;
    // bool isSwitching = false;
};

// 自动和手动模式。手动：0，自动：1
void VelCtrlNode::run()
{
    int initFlag = 0;
    ros::Rate loop_rate(CTRLFREQ);
    ros::service::waitForService("/magmed_modules/selfCollisionCheck", -1);
    while (ros::ok())
    {
        switch (diana.robo_state.VAL)
        {
        case 0: // robot init
            if (initFlag == 0)
            {
                loadInitPose();
                initFlag = 1;
            }
            break;
        // 机械臂已到达初始位置
        case 1: // robot run
            // if (initFlag == 1)
            // {
            //     diffkine.initConfig(diana.joint_states_array);
            //     initFlag = -1;
            //     ROS_INFO("Ctrl init finished\n");
            // }
            nRet = pubVels();
            break;
        case -1: // robot term
            ROS_WARN("Robot term, shutting down ros\n");
            ros::shutdown();
            break;
        default:
            break;
        }
        ros::spinOnce();
        // pubJoints();
        loop_rate.sleep();
    }
};

int VelCtrlNode::loadInitPose()
{
    // count init config
    std::vector<double> pose(6, 0.0);
    std::vector<Eigen::MatrixXd> Rp = TransToRp(diffkine.params.Tsg);
    Vector3d P0 = Rp[0] * diffkine.params.Pgb0 + Rp[1];
    pose[0] = P0(0);
    pose[1] = P0(1);
    pose[2] = P0(2);
    Matrix3d R0 = Rp[0] * diffkine.params.Rgb0;
    Eigen::AngleAxisd axis_angle(R0);
    pose[3] = axis_angle.angle() * axis_angle.axis()(0);
    pose[4] = axis_angle.angle() * axis_angle.axis()(1);
    pose[5] = axis_angle.angle() * axis_angle.axis()(2);
    // load init config
    std_msgs::Float64MultiArray msg;
    msg.data = pose;
    ros::param::set("/magmed_controller/initPose", msg.data);
    ROS_INFO("Init pose loaded\n");
    return 0;
}

// 闭环控制还需要分两种，一种是手柄操作一种是直接操作
int VelCtrlNode::pubVels()
{
    magmed_msgs::RoboJoints joint_vels;
    // get real mag pose
    diffkine.getMagPose(optctrl.magPose, diana.joint_states_array);
    // get mag twist
    // diffkine.magTwist = optctrl.generateMagTwist(refSignal.ref_theta, tipAngle.tip_angle);
    diffkine.magTwist.psi = joystick.magCR.theta[2];
    // get joint vels
    // refSignal.ref_phi.dphi = 0.01;

    // pub feeder
    std_msgs::UInt32 feeder_msg;
    switch(SYSCTRLMODE)
    {
    case 0: // 手动控制
    {
        VectorXd jointVels = diffkine.jacobiMap_tcp(joystick.dianaTcp.tcp_vel, diana.joint_states_array);
        for (int i = 0; i < JOINTNUM; ++i)
        {
            // push_back joint_vels
            joint_vels.joints.push_back(jointVels(i));
        }
        diana_jointVels_pub.publish(joint_vels);

        feeder_msg.data = joystick.feeder.feeder_vel;
        feeder_vel_pub.publish(feeder_msg);
        break;
    }
    case 1: // 自动控制
    {
        VectorXd jointVels = VectorXd::Zero(JOINTNUM);
        switch(CTRLMODE)
        {
            enum_TRANSMETHOD TRANSMETHOD = enum_TRANSMETHOD::NT;
            case enum_CTRLMODE::TRANS:
            {
                // 1.2.1. 如果检查后显示不会碰撞，则让diffKine.jacobiMap发布切换过程的jointVels
                // 1.2.2. 如果检查后仍碰撞，则停止机械臂运动
                // VectorXd jointVels = diffkine.jacobiMap(refSignal.ref_phi, jointStates.joint_states_array);
                // jointVels = diffkine.jacobiMap(joystick.magCR.phi, diana.joint_states_array, CTRLMODE);
                jointVels = diffkine.ctrlModeTrans(diana.joint_states_array, &CTRLMODE, TRANSMETHOD);
                break;
            }
            default:
            {
                jointVels = diffkine.jacobiMap(joystick.magCR.phi, diana.joint_states_array, CTRLMODE);
                // 1. 基于当前的diana.joint_states和jointVels一步预测未来的q会不会发生碰撞
                magmed_msgs::RoboJoints pred_robojoints;
                for(int i = 0; i < JOINTNUM; ++i)
                {
                    pred_robojoints.joints.push_back(diana.joint_states_array[i] + jointVels(i) / CTRLFREQ);
                }
                // selfCollisionCheck
                magmed_msgs::SelfCollisionCheck sCC;
                sCC.request.joints = pred_robojoints;
                if(sCC.response.checkResult == true) // 1.1.1 如果会碰撞，则检测其他的控制模式
                {
                    CTRLMODE = enum_CTRLMODE::TRANS;
                }
                else // 1.2. 如果selfCollisionCheck调用失败
                {
                    ROS_INFO("Failed/Not to call service selfCollisionCheck\n");
                    // break;
                }
                break;
            }
        }

        // 发布jointVels

        // 将jointVels缩放至合适的范围

        for (int i = 0; i < JOINTNUM; ++i)
        {
            // push_back joint_vels
            joint_vels.joints.push_back(jointVels(i));
        }
        diana_jointVels_pub.publish(joint_vels);

        feeder_msg.data = joystick.feeder.feeder_vel;
        feeder_vel_pub.publish(feeder_msg);
        break;
    }
    default:
        break;
    }

    return 0;
}

void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
{
    tip_angle = *pMsg;
};

void Diana::feedJoints(magmed_msgs::RoboJointsConstPtr pMsg)
{

    joint_states = *pMsg;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_states_array[i] = joint_states.joints[i];
    }
};

void Diana::feedState(magmed_msgs::RoboStatesConstPtr pMsg)
{
    robo_state = *pMsg;
};

void Joystick::feed(magmed_msgs::PFjoystickConstPtr pMsg)
{
    joystick = *pMsg;
    // feeder 数据
    feeder.FEEDER_VEL_GAIN = joystick.POTA * 50;
    feeder.feeder_vel = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN * joystick.nJOY3[0]);
    // diana tcp 数据
    dianaTcp.TCP_VEL_GAIN = joystick.POTB / 1000.0 * M_PI;
    if (joystick.bJOYD)
    {
        dianaTcp.tcp_vel[0] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[1];
        dianaTcp.tcp_vel[1] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[0];
        dianaTcp.tcp_vel[2] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[2];
        dianaTcp.tcp_vel[3] = 0.0;
        dianaTcp.tcp_vel[4] = 0.0;
        dianaTcp.tcp_vel[5] = 0.0;
    }
    else
    {
        dianaTcp.tcp_vel[0] = 0.0;
        dianaTcp.tcp_vel[1] = 0.0;
        dianaTcp.tcp_vel[2] = 0.0;
        dianaTcp.tcp_vel[3] = - dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[1];
        dianaTcp.tcp_vel[4] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[0];
        dianaTcp.tcp_vel[5] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[2];
    }
    // magCR 数据 增量数据
    magCR.phi[0] = M_PI / 2.0 * joystick.nJOY1[1];
    magCR.phi[1] = 0.0;
    // magCR.phi[0] += magCR.phi[1] / CTRLFREQ;
    magCR.theta[0] = 0.0;
    magCR.theta[1] = joystick.nJOY1[2];

    // magCR.phi[0] = joystick.nJOY1[0];
    // magCR.phi[1] = 0.0;
    // magCR.theta[0] = sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2)); // 模值开方
    // magCR.theta[1] = 0.0;

}