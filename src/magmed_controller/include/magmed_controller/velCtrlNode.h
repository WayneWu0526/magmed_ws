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
    void run();

    VelCtrlNode(){};
    VelCtrlNode(ros::NodeHandle &nh) : nh(nh)
    {
        refSignal_sub = nh.subscribe<magmed_msgs::JoyRef>("/magmed_joystick/joyRef",
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

private:
    JoyRef refSignal;
    TipAngle tipAngle;
    RoboJoints jointStates;
    RoboStates roboState;

    ros::NodeHandle nh;
    ros::Subscriber refSignal_sub;
    ros::Subscriber tipAngle_sub;
    ros::Subscriber jointStates_sub;
    ros::Subscriber roboState_sub;
    ros::Publisher jointVels_pub;

    optCtrl optctrl;
    diffKine diffkine;
    int pubJointVels();
    int loadInitPose();
};

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
                loadInitPose();
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
    Vector3d P0 =  Rp[0] * diffkine.params.Pgb0 + Rp[1];
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

int VelCtrlNode::pubJointVels()
{
    magmed_msgs::RoboJoints joint_vels;
    // get real mag pose
    diffkine.getMagPose(optctrl.magPose, jointStates.joint_states_array);
    // get mag twist
    // diffkine.magTwist = optctrl.generateMagTwist(refSignal.ref_theta, tipAngle.tip_angle);
    diffkine.magTwist.psi = refSignal.ref_theta.dtheta;
    // get joint vels
    // refSignal.ref_phi.dphi = 0.01;
    // VectorXd jointVels = diffkine.jacobiMap(refSignal.ref_phi, jointStates.joint_states_array);
    VectorXd jointVels = diffkine.jacobiMap_dlt(refSignal.ref_phi, jointStates.joint_states_array);
    // print jointVels
    // std::cout << "jointVels: " << jointVels << std::endl;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        // push_back joint_vels
        joint_vels.joints.push_back(jointVels(i));
    }
    jointVels_pub.publish(joint_vels);
    return 0;
}


void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
{
    tip_angle = *pMsg;
};

void JoyRef::feed(magmed_msgs::JoyRefConstPtr pMsg)
{
    joy_ref = *pMsg;
    ref_theta = joy_ref.refTheta;
    ref_phi = joy_ref.refPhi;
    ROS_INFO("ref_phi: %f, ref_dphi: %f\n", ref_phi.phi, ref_phi.dphi);
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