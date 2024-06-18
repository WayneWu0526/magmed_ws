#pragma once

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/diffKine.h"
#include "magmed_controller/optCtrl.h"

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
    geometry_msgs::Point tip_point;
    Vector3d tip_point_vec = Vector3d::Zero();
    void feedTip(geometry_msgs::PointConstPtr pMsg);
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

        tipAngle_sub = nh.subscribe<magmed_msgs::TipAngle>("/magmed_camera/tipAngle",
                                                           10,
                                                           boost::bind(&TipAngle::feed, &tipAngle, _1));

        stereoCamera_sub = nh.subscribe<geometry_msgs::Point>("/magmed_stereoCamera/tipPoint",
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

        diana_jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels", 10);

        feeder_vel_pub = nh.advertise<std_msgs::UInt32MultiArray>("/magmed_feeder/vel", 10);

        selfcollision_client = nh.serviceClient<magmed_msgs::SelfCollisionCheck>("/magmed_modules/selfCollisionCheck"); 
        // ros::service::waitForService("/magmed_manipulator/roboStates", -1);
    };

private:
    Joystick joystick;
    TipAngle tipAngle;
    TsgPoseTwist tsgPoseTwist;
    Diana diana;
    StereoCamera stereoCamera;

    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    ros::Subscriber tipAngle_sub;
    ros::Subscriber diana_jointStates_sub;
    ros::Subscriber diana_roboState_sub;
    ros::Subscriber TsgPoseTwist_sub;
    ros::Subscriber stereoCamera_sub;
    ros::Publisher diana_jointVels_pub;
    ros::Publisher feeder_vel_pub;
    ros::ServiceClient selfcollision_client;

    optCtrl optctrl;
    diffKine diffkine;
    int pubVels();
    int loadInitPose();
    double calcTipAngle(const double alpha,const double (&thetaList)[JOINTNUM]);
    int nRet = 0;
    enum_CTRLMODE CTRLMODE = enum_CTRLMODE::NM; // CTRLMODE
    enum_TRANSMETHOD TRANSMETHOD = enum_TRANSMETHOD::OFT; // Translation method
    bool SCC_FLAG = false;
    // bool isSwitching = false;
};

