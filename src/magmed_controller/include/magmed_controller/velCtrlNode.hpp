#pragma once

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/diffKine.h"
#include "magmed_controller/optCtrl.h"

/*** 重要!!!! set SYSCTRLMODE = 0 for manually-tcp****/
const int SYSCTRLMODE = 2; // -1: test 0：Manually-tcp，1：Open-loop-circle, 2: Closed-loop 

template <typename T>
class LinearTrackingDifferentiator {
public:
    LinearTrackingDifferentiator(T k1, T k2) 
        : k1_(k1), k2_(k2), x1_(0), x2_(0) {}

    void update(T v, T dt) {
        // Calculate derivatives
        T dx1 = x2_;
        T dx2 = -k1_ * (x1_ - v) - k2_ * x2_;
        
        // Update states
        x1_ += dx1 * dt;
        x2_ += dx2 * dt;
    }

    T getX1() const { return x1_; }
    T getX2() const { return x2_; }

private:
    T k1_;
    T k2_;
    T x1_;
    T x2_;
};

class TsgPoseTwist
{
public:
    magmed_msgs::PoseTwist pose_twist;
    VectorXd Tsg_pose_twist;
    TsgPoseTwist(): Tsg_pose_twist(VectorXd::Zero(2*TCPNUM)) {
        // Tsg_pose_twist = VectorXd::Zero(2*TCPNUM);
        // se3ToVec(MatrixLog6(diffKine::params.Tsg));

        pose_twist.pose.position.x = 0.0;
        pose_twist.pose.position.y = 0.0;
        pose_twist.pose.position.z = 0.0;
        pose_twist.pose.orientation.x = 0.0;
        pose_twist.pose.orientation.y = 0.0;
        pose_twist.pose.orientation.z = 0.0;
        pose_twist.pose.orientation.w = 1.0;

        pose_twist.twist.linear.x = 0.0;
        pose_twist.twist.linear.y = 0.0;
        pose_twist.twist.linear.z = 0.0;
        pose_twist.twist.angular.x = 0.0;
        pose_twist.twist.angular.y = 0.0;
        pose_twist.twist.angular.z = 0.0;
    };
    void feed(magmed_msgs::PoseTwistConstPtr pMsg);
};

class Linear_actuator
{
public:
    geometry_msgs::TwistStamped gframe_twistStamped;
    VectorXd gframe_twist;
    void feed(geometry_msgs::TwistStampedConstPtr pMsg)
    {
        gframe_twistStamped = *pMsg;
        gframe_twist << 
            gframe_twistStamped.twist.angular.x,
            gframe_twistStamped.twist.angular.y,
            gframe_twistStamped.twist.angular.z,
            gframe_twistStamped.twist.linear.x,
            gframe_twistStamped.twist.linear.y,
            gframe_twistStamped.twist.linear.z;
    }   
    Linear_actuator() {
        gframe_twistStamped.twist.linear.x = 0.0;
        gframe_twistStamped.twist.linear.y = 0.0;
        gframe_twistStamped.twist.linear.z = 0.0;
        gframe_twistStamped.twist.angular.x = 0.0;
        gframe_twistStamped.twist.angular.y = 0.0;
        gframe_twistStamped.twist.angular.z = 0.0;
        gframe_twist = VectorXd::Zero(6);
    };
};

// camera feedback
// class TipAngle
// {
// public:
//     magmed_msgs::TipAngle tip_angle;
//     TipAngle() { tip_angle.tipAngle = 0.0; };
//     void feed(magmed_msgs::TipAngleConstPtr pMsg);
// };

// joystick input (manual control)
class Joystick
{
public:
    magmed_msgs::PFjoystick joystick;
    Feeder feeder;
    DianaTcp dianaTcp;
    MagCR magCR;

    LinearTrackingDifferentiator<double> phirltd;
    LinearTrackingDifferentiator<double> thetarltd;
    void feed(magmed_msgs::PFjoystickConstPtr pMsg);
    // Joystick(): phirltd(3000.0, 100.0), thetarltd(3000.0, 100.0) {};
    // 1000.0: 增益系数，控制速度
    Joystick(): phirltd(1000.0, 50.0), thetarltd(1000.0, 50.0) {};
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

class StereoCamera
{
public:
    geometry_msgs::PointStamped tip_point;
    Vector3d tip_point_vec = Vector3d::Zero();
    void feedTip(geometry_msgs::PointStampedConstPtr pMsg);
    StereoCamera(){};
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

        // tipAngle_sub = nh.subscribe<magmed_msgs::TipAngle>("/magmed_camera/tipAngle",
        //                                                    10,
        //                                                    boost::bind(&TipAngle::feed, &tipAngle, _1));

        stereoCamera_sub = nh.subscribe<geometry_msgs::PointStamped>("/magmed_stereoCamera/tipPoint",
                                                              10,
                                                              boost::bind(&StereoCamera::feedTip, &stereoCamera, _1));

        TsgPoseTwist_sub = nh.subscribe<magmed_msgs::PoseTwist>("/magmed_stereoCamera/twist",
                                                            10,
                                                            boost::bind(&TsgPoseTwist::feed, &tsgPoseTwist, _1));

        diana_jointStates_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints",
                                                                      10,
                                                                      boost::bind(&Diana::feedJoints, &diana, _1));

        diana_roboState_sub = nh.subscribe<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate",
                                                                    10,
                                                                    boost::bind(&Diana::feedState, &diana, _1));
        
        linear_actuator_sub = nh.subscribe<geometry_msgs::TwistStamped>("/linear_actuator/gframe_twist",
                                                                        10, 
                                                                        boost::bind(&Linear_actuator::feed, &la, _1));

        diana_jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels", 10);

        feeder_vel_pub = nh.advertise<std_msgs::UInt32MultiArray>("/magmed_feeder/vel", 10);

        selfcollision_client = nh.serviceClient<magmed_msgs::SelfCollisionCheck>("/magmed_modules/selfCollisionCheck"); 
        // ros::service::waitForService("/magmed_manipulator/roboStates", -1);
    };

private:
    Joystick joystick;
    // TipAngle tipAngle;
    TsgPoseTwist tsgPoseTwist;
    Diana diana;
    StereoCamera stereoCamera;
    Linear_actuator la;

    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    // ros::Subscriber tipAngle_sub;
    ros::Subscriber diana_jointStates_sub;
    ros::Subscriber diana_roboState_sub;
    ros::Subscriber TsgPoseTwist_sub;
    ros::Subscriber stereoCamera_sub;
    ros::Subscriber linear_actuator_sub;

    ros::Publisher diana_jointVels_pub;
    ros::Publisher feeder_vel_pub;

    ros::ServiceClient selfcollision_client;

    optCtrl optctrl;
    diffKine diffkine;
    int pubVels();
    int loadInitPose();
    double calcTipAngle(const double alpha,const double (&thetaList)[JOINTNUM], int DEEPFLAG);
    int nRet = 0;
    enum_CTRLMODE CTRLMODE = enum_CTRLMODE::NM; // CTRLMODE
    enum_TRANSMETHOD TRANSMETHOD = enum_TRANSMETHOD::OFT; // Translation method
    bool SCC_FLAG = false;
    // bool isSwitching = false;
};


// void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
// {
//     tip_angle = *pMsg;
// };

void StereoCamera::feedTip(geometry_msgs::PointStampedConstPtr pMsg)
{
    tip_point = *pMsg;
    tip_point_vec << tip_point.point.x, tip_point.point.y, tip_point.point.z;
};

void TsgPoseTwist::feed(magmed_msgs::PoseTwistConstPtr pMsg)
{
    pose_twist = *pMsg;
    // convert orientation to rotation matrix
    Quaterniond quat(pose_twist.pose.orientation.w, pose_twist.pose.orientation.x,
        pose_twist.pose.orientation.y, pose_twist.pose.orientation.z);
    Vector3d rot = so3ToVec((quat.toRotationMatrix()));
    Tsg_pose_twist << 
        rot,
        pose_twist.pose.position.x,
        pose_twist.pose.position.y,
        pose_twist.pose.position.z,
        pose_twist.twist.angular.x,
        pose_twist.twist.angular.y,
        pose_twist.twist.angular.z,
        pose_twist.twist.linear.x,
        pose_twist.twist.linear.y,
        pose_twist.twist.linear.z;
    // mock data
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
    feeder.FEEDER_VEL_GAIN_FB = joystick.POTA * 50;
    feeder.feeder_vel_FB = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_FB * joystick.nJOY3[0]);
    feeder.FEEDER_VEL_GAIN_UD = joystick.POTB * 50;
    feeder.feeder_vel_UD = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_UD * joystick.nJOY2[1]);
    // diana tcp 数据
    dianaTcp.TCP_VEL_GAIN = joystick.POTB / 10000.0 * M_PI;
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
    // magCR.phi[0] = M_PI / 2.0 * joystick.nJOY1[1];
    // magCR.phi[1] = 0.0; // under estimated using ADRC

    switch(SYSCTRLMODE)
    {
        case 0: // manually-tcp
        {
            magCR.phi[0] = 0.0;
            magCR.phi[1] = 0.0;
            magCR.theta[0] = 0.0;
            magCR.theta[1] = 0.0;
            break;
        }
        case 1: // open-loop circle
        {
            magCR.phi[0] += joystick.nJOY1[1] / CTRLFREQ;
            magCR.phi[1] = joystick.nJOY1[1];

            magCR.theta[0] += joystick.nJOY1[2] / CTRLFREQ;
            magCR.theta[1] = joystick.nJOY1[2];
            break;
        }
        case 2:
        {
            double joyPhi = atan2(-joystick.nJOY1[0], joystick.nJOY1[1]);
            double joyTheta = sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2));

            if(joyPhi > M_PI / 2.0)
            {
                joyPhi -= M_PI;
                joyTheta = -joyTheta;
            }
            else if(joyPhi < -M_PI / 2.0)
            {
                joyPhi += M_PI;
                joyTheta = -joyTheta;
            }
            phirltd.update(joyPhi, 1.0 / CTRLFREQ);
            thetarltd.update(joyTheta, 1.0 / CTRLFREQ);

            magCR.phi[0] = phirltd.getX1();
            magCR.phi[1] = phirltd.getX2();

            magCR.theta[0] = thetarltd.getX1();
            magCR.theta[1] = thetarltd.getX2();

            // printf("ref phi: %f, ref theta: %f\n", joyPhi, joyTheta);           
            // printf("phi: %f, theta: %f\n", magCR.phi[0], magCR.theta[0]);
            // printf("dphi: %f, dtheta: %f\n", magCR.phi[1], magCR.theta[1]);
            break;
        }
        default:
        {
            break;
        }
    }
    // printf("phi circle %f\n", atan2(-joystick.nJOY1[0], joystick.nJOY1[1]));
    // printf("theta circle %f\n", sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2)));

    // magCR.phi[0] += magCR.phi[1] / CTRLFREQ;
    // magCR.theta[0] = 0.0;
    // magCR.theta[1] = joystick.nJOY1[2];
};

