#include "magmed_controller/velCtrlNode.h"
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include "magmed_controller/optCtrl.h"
#include "magmed_controller/diffKine.h"
#include "magmed_msgs/MagPose.h"

float g_fThetaL = 0.0;
double g_dPsi = 0.0;
Vector3d g_dPos = {0.0, 0.0, 0.0};
double g_dThetaR[2] = {0.0, 0.0};

void tipAngleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // ROS_INFO("tipAngle received: [%f]", msg->data);
    g_fThetaL = msg->data;
}

void magPoseCallback(const magmed_msgs::MagPose::ConstPtr& msg)
{
    // ROS_INFO("psi received: [%f]", msg->data);
    g_dPsi = msg->psi;
    g_dPos = {msg->pos[0], msg->pos[1], msg->pos[2]};
}

void refSignalCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // ROS_INFO("refSignal received: [%f, %f]", msg->data[0], msg->data[1]);
    g_dThetaR[0] = msg->data[0];
    g_dThetaR[1] = msg->data[1];
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");

    ros::NodeHandle nh;

// get reference theta
    // ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/TDSignal", 1000, refSignalCallback);
    ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/referenceSignal", 10, refSignalCallback);
    
    // get measured theta
    ros::Subscriber subTipAngle = nh.subscribe("/magmed_camera/tipAngle", 10, tipAngleCallback);

    // get angle and position of the magnet
    ros::Subscriber subPos = nh.subscribe<magmed_msgs::MagPose>("/magmed_manipulator/magPos", 10, magPoseCallback);

    // publish the angular velocity of the magnet
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_controller/magVel", 10);

    // frequency of the controller
    int nf = 100;

    ros::Rate rate(nf);

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    // initialize LESO

    while (ros::ok())
    {
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;

        // // set coefficient of the controller (positive)
        // float k = 3.0;
        // // applying quasi-static controller to calculate the angular velocity of the magnet
        // double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", g_dThetaR[0] - g_fThetaL);

        // 

        // 控制器自带软限位
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}