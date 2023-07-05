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

Vector2d LESO(double dThetaR, double controlInput, double jacobian, Vector2d hatx, int nf)
{
    float beta1 = 0.001;
    float beta2 = 0.001;
    float epsilon = 0.1;

    Vector2d next_hatx = {0.0, 0.0};
    next_hatx(0) = hatx(0) + 1.0 / nf * (hatx(1) + beta1/epsilon * (dThetaR - hatx(0)) + jacobian * controlInput);
    next_hatx(1) = hatx(1) + 1.0 / nf * (beta2/(epsilon*epsilon) * (dThetaR - hatx(0)));
    return next_hatx;
}

double PD_controller(Vector2d dThetaR, Vector2d hatx, double jacobian, double thetaL)
{
    float fk = 20.0;

    return 1.0/jacobian * (dThetaR(1) + fk * (dThetaR(0) - thetaL) - hatx(1));
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

    // frequency of the controller
    int nf = 100;

    ros::Rate rate(nf);

    magmed_controller::MCR mcr;

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    // input reference theta
    Vector2d v2dThetaR = {0.0, 0.0};

    // position of the magnet
    // Eigen::Vector3d pa = { mcr.pr.L, 0, 150.0e-3};
    Eigen::Vector3d pa = { 0.0, 0.0, 191.0e-3};

    // initialize LESO
    Vector2d v2dhatx = {0.0, 0.0};

    // record the initial time
    ros::Time t0 = ros::Time::now();

    while(ros::ok())
    {   
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;

        // calculate the time difference to reach t0 and return seconds
        double dt = (ros::Time::now() - t0).toSec();

        // input reference theta in sine wave
        float fphaseshift = 0.0;
        v2dThetaR(0) = 0.33 * M_PI * sin(2 * M_PI * dt + fphaseshift);
        v2dThetaR(1) = 0.33 * M_PI * 2 * M_PI * cos(2 * M_PI * dt + fphaseshift);

        // get jacobian of the robot
        double jacobian = mcr.get_jacobian(g_fPsi + M_PI, pa);
        ROS_INFO("jacobian: %f", jacobian);

        // calculate the control input
        double controlInput = PD_controller(v2dThetaR, v2dhatx, jacobian, g_fThetaL);

        // update LESO
        v2dhatx = LESO(v2dThetaR(0), controlInput, jacobian, v2dhatx, nf);

        // // set coefficient of the controller (positive)
        // float k = 3.0;
        // // applying quasi-static controller to calculate the angular velocity of the magnet
        // double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", v2dThetaR(0) - g_fThetaL);

        // 控制器自带软限位

        std_msgs::Float64 msg;
        msg.data = controlInput;
        pub.publish(msg);
        ROS_INFO("angular velocity: %f", msg.data);

        ros::spinOnce();
        rate.sleep();
    }


    return 0;
}
