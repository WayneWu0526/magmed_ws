#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "magmed_controller/getJacobianOfMCR.h"
#include <std_msgs/Float64.h>
#include <iostream>

float g_fThetaL = 0.0;
float g_fPsi = 0.0;

void tipAngleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // ROS_INFO("tipAngle received: [%f]", msg->data);
    g_fThetaL = msg->data;
}

void psiCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // ROS_INFO("psi received: [%f]", msg->data);
    g_fPsi = msg->data;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velocityController");

    ros::NodeHandle nh;

    // get measured theta
    ros::Subscriber subTipAngle = nh.subscribe("/magmed_camera/tipAngle", 1000, tipAngleCallback);

    // get rotation angle of the magnet
    ros::Subscriber subPsi = nh.subscribe("/magmed_manipulator/magnetAngle", 1000, psiCallback);

    // publish the angular velocity of the magnet
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/magmed_controller/angularVelocity", 1000);

    ros::Rate rate(100);

    magmed_controller::MCR mcr;

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    while(ros::ok())
    {   
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;

        // input reference theta
        Vector2d v2dThetaR = {M_PI / 3.0, 0.0};

        // set coefficient of the controller (positive)
        float k = 3.0;

        // position of the magnet
        // Eigen::Vector3d pa = { mcr.pr.L, 0, 150.0e-3};
        Eigen::Vector3d pa = { 0.0, 0.0, 191.0e-3};
        // get jacobian of the robot
        double jacobian = mcr.get_jacobian(g_fPsi + M_PI, pa);
        ROS_INFO("jacobian: %f", jacobian);

        // applying quasi-static controller to calculate the angular velocity of the magnet
        double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", v2dThetaR(0) - g_fThetaL);

        // 控制器自带软限位

        std_msgs::Float64 msg;
        msg.data = dPsi;
        pub.publish(msg);
        ROS_INFO("angular velocity: %f", msg.data);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}