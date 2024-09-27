#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include "magmed_controller/MSCRJacobi.h"
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>

float g_fThetaL = 0.0;
float g_fPsi = 0.0;
double g_dThetaR[2] = {0.0, 0.0};

void tipAngleCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // ROS_INFO("tipAngle received: [%f]", msg->data);
    g_fThetaL = msg->data;
}

void psiCallback(const std_msgs::Float64::ConstPtr& msg)
{
    ROS_INFO("psi received: [%f]", msg->data);
    g_fPsi = msg->data;
}

void refSignalCallback(const std_msgs::Float64MultiArray::ConstPtr& msg)
{
    // ROS_INFO("refSignal received: [%f, %f]", msg->data[0], msg->data[1]);
    g_dThetaR[0] = msg->data[0];
    g_dThetaR[1] = msg->data[1];
}

void LESO(double controlInput, double jacobian, std::vector<float>& hatx, int nf)
{
    float beta1 = 1;
    float beta2 = 0.01;
    float epsilon = 0.01;

    double error = g_dThetaR[0] - hatx[0];
    hatx[0] += (hatx[0] + beta1 / epsilon * error + jacobian * controlInput) / nf;
    hatx[1] += (beta2 / (epsilon * epsilon) * error) / nf;
    std::cout << "estimate perturbation: " << hatx[1] << std::endl;
}

double PD_controller(std::vector<float>& hatx, double jacobian, double thetaL)
{
    float fk = 0.1;
    hatx[1] = 0.0;
    std::cout << "PD controller:" << g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) << std::endl;
    return (g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) - hatx[1]) / jacobian;
}

// saturation function
double sat(double x, double xmin, double xmax)
{
    if (x < xmin)
    {
        return xmin;
    }
    else if (x > xmax)
    {
        return xmax;
    }
    else
    {
        return x;
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "velocityController");

    ros::NodeHandle nh;

    // get measured theta
    ros::Subscriber subTipAngle = nh.subscribe("/magmed_camera/tipAngle", 1000, tipAngleCallback);

    // get rotation angle of the magnet
    ros::Subscriber subPsi = nh.subscribe("/magmed_manipulator/magnetAngle", 1000, psiCallback);

    // get reference theta
    // ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/TDSignal", 1000, refSignalCallback);
    ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/referenceSignal", 1000, refSignalCallback);

    // publish the angular velocity of the magnet
    ros::Publisher pub = nh.advertise<std_msgs::Float64>("/magmed_controller/angularVelocity", 1000);

    // frequency of the controller
    int nf = 100;

    ros::Rate rate(nf);

    JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();

    MSCRJacobi mcr;

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    // position of the magnet
    // Eigen::Vector3d pa = { mcr.pr.L, 0, 150.0e-3};
    Eigen::Vector3d pa = { pr.L, pr.H0, 0.0};

    // initialize LESO
    std::vector<float> hatx = {0.0, 0.0};

    while(ros::ok())
    {   
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;        

        // get jacobian of the robot
        double thetaL_;
        RowVector4d jacobian_;
        Vector3d Jx;
        std::tie(thetaL_, jacobian_, Jx) = mcr.get_states(g_fPsi, pa);
        double jacobian = jacobian_[0];
        ROS_INFO("jacobian: %f", jacobian);

        // calculate the control input
        double controlInput = PD_controller(hatx, jacobian, g_fThetaL);

        // update LESO
        LESO(controlInput, jacobian, hatx, nf);

        // // set coefficient of the controller (positive)
        // float k = 3.0;
        // // applying quasi-static controller to calculate the angular velocity of the magnet
        // double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", g_dThetaR[0] - g_fThetaL);

        // 控制器自带软限位

        std_msgs::Float64 msg;
        float fLimit = 10.0;
        msg.data = sat(controlInput, -fLimit, fLimit);
        pub.publish(msg);
        ROS_INFO("angular velocity: %f", msg.data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
