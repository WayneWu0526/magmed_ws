#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include "magmed_controller/getJacobianOfMCR.h"
#define H0 183.0e-3

float g_fThetaL = 0.0;
float g_fPsi = 0.0;
double g_dThetaR[2] = {0.0, 0.0};

void tipAngleCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // ROS_INFO("tipAngle received: [%f]", msg->data);
    g_fThetaL = msg->data;
}

void psiCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // ROS_INFO("psi received: [%f]", msg->data);
    g_fPsi = msg->data;
}

void refSignalCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // ROS_INFO("refSignal received: [%f, %f]", msg->data[0], msg->data[1]);
    g_dThetaR[0] = msg->data[0];
    g_dThetaR[1] = msg->data[1];
}

void LESO(Vector4d controlInput, RowVector4d jacobian, std::vector<float> &hatx, int nf)
{
    float beta1 = 1;
    float beta2 = 0.01;
    float epsilon = 0.01;

    double error = g_dThetaR[0] - hatx[0];
    hatx[0] += (hatx[0] + beta1 / epsilon * error + jacobian * controlInput) / nf;
    hatx[1] += (beta2 / (epsilon * epsilon) * error) / nf;
    std::cout << "estimate perturbation: " << hatx[1] << std::endl;
}

double FF_controller(std::vector<float> &hatx, double thetaL)
{
    float fk = 200.0;
    hatx[1] = 0.0;
    std::cout << "virtual FF_controller:" << g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) << std::endl;
    return g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) - hatx[1];
}

Vector4d controlAllocation(Vector4d u, double virtualControlLaw, RowVector4d jacobian, double L, int nf)
{
    USING_NAMESPACE_QPOASES
    double inf = qpOASES::INFTY;
    Vector4d UMAX = {3.839724, 1.0, 1.0, 1.0};
    
    // diagonal
    float T = 1.0 / nf;
    Vector4d Umin = {-M_PI / 2.0, L, H0 - 20.0e-3, 0.0};
    Vector4d Umax = {M_PI / 2.0, L, H0 + 40.0e-3, 0.0};
    Umin = (Umin - u) / T;
    Umax = (Umax - u) / T;
    Umin = Umin.cwiseMax(-UMAX);
    Umax = Umax.cwiseMin(UMAX);

    real_t H[5 * 5] = {0.001, 0.0, 0.0, 0.0, 0.0,
                       0.0, 100.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 100.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 100.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 100.0};
    real_t A[1 * 5] = {jacobian[0], jacobian[1], jacobian[2], jacobian[3], 1.0};
    real_t lb[5] = {Umin[0], Umin[1], Umin[2], Umin[3], -inf};
    real_t ub[5] = {Umax[0], Umax[1], Umax[2], Umax[3], inf};
    real_t lbA[1] = {virtualControlLaw};
    real_t ubA[1] = {virtualControlLaw};

    QProblem qpCA( 5,1 );
    int_t nWSR = 10;
    qpCA.init(H, NULL, A, lb, ub, lbA, ubA, nWSR);
    real_t xOpt[5];
    qpCA.getPrimalSolution( xOpt );

    Vector4d optController;
    optController << xOpt[0], xOpt[1], xOpt[2], xOpt[3];
    std::cout << "optimal control input: " << optController << std::endl;
    return optController;
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

    magmed_controller::MCR mcr;

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    // position of the magnet
    Vector3d pa = {mcr.pr.L, H0, 0.0};

    // initialize LESO
    std::vector<float> hatx = {0.0, 0.0};

    while (ros::ok())
    {
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;

        // get jacobian of the robot
        RowVector4d jacobian = mcr.get_jacobian(g_fPsi, pa);

        std::cout << "jacobian: " << jacobian << std::endl;
        // virtual control law

        // ROS_INFO("jacobian: %f", jacobian);

        // calculate the control input
        double virtualControlLaw = FF_controller(hatx, g_fThetaL);

        Vector4d u;
        u << g_fPsi, pa;

        // control allocation
        Vector4d actualControlLaw = controlAllocation(u, virtualControlLaw, jacobian, mcr.pr.L, nf);

        // update LESO
        LESO(actualControlLaw, jacobian, hatx, nf);

        // // set coefficient of the controller (positive)
        // float k = 3.0;
        // // applying quasi-static controller to calculate the angular velocity of the magnet
        // double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", g_dThetaR[0] - g_fThetaL);

        // 控制器自带软限位

        std_msgs::Float64 msg;
        pub.publish(msg);
        ROS_INFO("angular velocity: %f", msg.data);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}