#ifndef VELCTRLNODE_H
#define VELCTRLNODE_H

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/diffKine.h"
#include "magmed_controller/optCtrl.h"

class TipAngle
{
public:
    magmed_msgs::TipAngle tip_angle;
    TipAngle() { tip_angle.tipAngle = 0.0; };
    void feed(magmed_msgs::TipAngleConstPtr pMsg);
};

class JoyRef
{
public:
    magmed_msgs::JoyRef joy_ref;
    magmed_msgs::RefPhi ref_phi;
    magmed_msgs::RefTheta ref_theta;
    JoyRef()
    {
        joy_ref.refPhi.phi = 0.0;
        joy_ref.refTheta.theta = 0.0;
        joy_ref.refPhi.dphi = 0.0;
        joy_ref.refTheta.dtheta = 0.0;
    };
    void feed(magmed_msgs::JoyRefConstPtr pMsg);
};

class RoboStates
{
public:
    magmed_msgs::RoboStates robo_state;
    RoboStates(){};
    void feed(magmed_msgs::RoboStatesConstPtr pMsg);
};

class RoboJoints
{
public:
    double joint_states_array[JOINTNUM] = {0.0};
    magmed_msgs::RoboJoints joint_states;
    RoboJoints(){};
    void feed(magmed_msgs::RoboJointsConstPtr pMsg);
};

class VelCtrlNode
{
public:
    JoyRef refSignal;
    TipAngle tipAngle;
    RoboJoints jointStates;
    RoboStates roboState;

    VelCtrlNode(){};
    VelCtrlNode(ros::NodeHandle &nh) : nh(nh)
    {
        refSignal_sub = nh.subscribe<magmed_msgs::JoyRef>("/magmed_joystick/referenceSignal",
                                                          10,
                                                          boost::bind(&JoyRef::feed, &refSignal, _1));

        tipAngle_sub = nh.subscribe<magmed_msgs::TipAngle>("/magmed_controller/tipAngle",
                                                           10,
                                                           boost::bind(&TipAngle::feed, &tipAngle, _1));

        jointStates_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints",
                                                                10,
                                                                boost::bind(&RoboJoints::feed, &jointStates, _1));

        roboState_sub = nh.subscribe<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate",
                                                              10,
                                                              boost::bind(&RoboStates::feed, &roboState, _1));

        jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_controller/joint_vels", 10);
        // ros::service::waitForService("/magmed_manipulator/roboStates", -1);
    };
    int pubJointVels();
    void run();

private:
    ros::NodeHandle nh;
    ros::Subscriber refSignal_sub;
    ros::Subscriber tipAngle_sub;
    ros::Subscriber jointStates_sub;
    ros::Subscriber roboState_sub;
    ros::Publisher jointVels_pub;
    RoboJoints joint_vels;
    optCtrl optctrl;
    diffKine diffkine;
};

int VelCtrlNode::pubJointVels()
{
    double phi[2] = {0.0};
    magmed_msgs::RoboJoints joint_vels;
    // get real mag pose
    diffkine.getRealMagPose(optctrl.magPose, jointStates.joint_states_array);
    // get mag twist
    diffkine.magTwist = optctrl.generateMagTwist(refSignal.ref_theta, tipAngle.tip_angle);
    // get joint vels
    VectorXd jointVels = diffkine.jacobiMap(refSignal.ref_phi, jointStates.joint_states_array);
    // print jointVels
    std::cout << "jointVels: " << jointVels << std::endl;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_vels.joints[i] = jointVels(i);
    }
    jointVels_pub.publish(joint_vels);
    return 0;
}

void VelCtrlNode::run()
{
    int initFlag = 0;
    ros::Rate loop_rate(CTRLFREQ);
    while (ros::ok())
    {
        switch (roboState.robo_state.VAL)
        {
        case 0: // robot init
            if (initFlag == 0)
            {
                // upload init flag
                initFlag = 1;
            }
            break;
        case 1: // robot run
            if (initFlag == 1)
            {
                diffkine.initConfig(jointStates.joint_states_array);
                initFlag = -1;
                ROS_INFO("Ctrl init finished\n");
            }
            pubJointVels();
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

void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
{
    tip_angle = *pMsg;
};

void JoyRef::feed(magmed_msgs::JoyRefConstPtr pMsg)
{
    joy_ref = *pMsg;
    ref_theta = joy_ref.refTheta;
    ref_phi = joy_ref.refPhi;
};

void RoboJoints::feed(magmed_msgs::RoboJointsConstPtr pMsg)
{

    joint_states = *pMsg;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_states_array[i] = joint_states.joints[i];
    }
};

void RoboStates::feed(magmed_msgs::RoboStatesConstPtr pMsg)
{
    robo_state = *pMsg;
};

#endif