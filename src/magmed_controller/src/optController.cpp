#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <iostream>
#include <vector>
#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include "getJacobianOfMCR.h"
#include "diffKine.h"

magmed_controller::MCR mcr;

float g_fThetaL = 0.0;
double g_dPsi = 0.0;
Vector3d g_dPos = {mcr.pr.L, mcr.pr.H0, 0.0};
double g_dThetaR[2] = {0.0, 0.0};

void tipAngleCallback(const std_msgs::Float64::ConstPtr &msg)
{
    // ROS_INFO("tipAngle received: [%f]", msg->data);
    g_fThetaL = msg->data;
}

void magPosCallback(const std_msgs::Float64MultiArray::ConstPtr &msg)
{
    // ROS_INFO("psi received: [%f]", msg->data);
    g_dPsi = msg->data[0];
    g_dPos[0] = msg->data[1];
    g_dPos[1] = msg->data[2];
    g_dPos[2] = msg->data[3];
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
    // std::cout << "estimate perturbation: " << hatx[1] << std::endl;
}

double FF_controller(std::vector<float> &hatx, double thetaL)
{
    float fk = 200.0;
    hatx[1] = 0.0;
    std::cout << "virtual FF_controller:" << g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) << std::endl;
    return g_dThetaR[1] + fk * (g_dThetaR[0] - thetaL) - hatx[1];
}

Vector4d controlAllocation(double virtualControlLaw, RowVector4d jacobian, int nf)
{
    USING_NAMESPACE_QPOASES
    double inf = qpOASES::INFTY;
    Vector4d UMAX = {0.2, 0.05, 0.01, 0.05};
    Vector4d twist;
    twist << g_dPsi, g_dPos;
    std::cout << "twist: " << twist << std::endl;
    
    // diagonal
    float T = 1.0 / nf;
    Vector4d Umin = {-M_PI / 2.0, mcr.pr.L, mcr.pr.H0 - 20.0e-3, 0.0};
    Vector4d Umax = {M_PI / 2.0, mcr.pr.L, mcr.pr.H0 + 40.0e-3, 0.0};
    // std::cout << "Umin:" << Umin << std::endl;
    // std::cout << "Umax:" << Umax << std::endl;
    Umin = (Umin - twist) / (2 * T);
    std::cout << Umin(2) << std::endl;
    Umax = (Umax - twist) / (2 * T);
    // std::cout << "Umin:" << Umin << std::endl;
    // std::cout << "Umax:" << Umax << std::endl;
    Umin = Umin.cwiseMax(-UMAX);
    Umax = Umax.cwiseMin(UMAX);


    real_t H[5 * 5] = {0.001, 0.0, 0.0, 0.0, 0.0,
                       0.0, 100.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 100.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 100.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 100.0};
    real_t g[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    real_t A[1 * 5] = {jacobian[0], jacobian[1], jacobian[2], jacobian[3], 1.0};
    // real_t lb[5] = {Umin[0], Umin[1], Umin[2], Umin[3], -inf};
    // real_t ub[5] = {Umax[0], Umax[1], Umax[2], Umax[3], inf};
    // if(Umin(2) >= Umax(2)){
    //     Umax(2) = Umin(2);
    // }
    // if(Umax(2) <= Umin(2))
    // {
    //     Umin(2) = Umax(2);
    // }
    real_t lb[5] = {Umin[0], 0.0, Umin[2], 0.0, -inf};
    real_t ub[5] = {Umax[0], 0.0, Umax[2], 0.0, inf};

    std::cout << "Umin" << Umin << std::endl;
    std::cout << "Umax" << Umax << std::endl;
    real_t lbA[1] = {virtualControlLaw};
    real_t ubA[1] = {virtualControlLaw};

    Options options;
    options.printLevel = PL_LOW;
    QProblem qpCA( 5,1);
    qpCA.setOptions( options );
    int_t nWSR = 10;
    qpCA.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    real_t xOpt[5];
    qpCA.getPrimalSolution( xOpt );

    Vector4d optController;
    optController << xOpt[0], xOpt[1], xOpt[2], xOpt[3]; // optController order: psi, x, y, z
    std::cout << "optimal control input: " << optController << std::endl;
    return optController;
}

Vector3d velFrmTrns(Vector3d vel, double psi)
{
    Matrix3d RotZ = mcr.RotZ(-psi);
    Eigen::DiagonalMatrix<double, 3> Rgb0;
    Rgb0.diagonal() << -1, 1, -1;
    Vector3d velTrans = RotZ * Rgb0 * vel;
    return velTrans;
}

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");

    ros::NodeHandle nh;

// get reference theta
    // ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/TDSignal", 1000, refSignalCallback);
    ros::Subscriber subRefSignal = nh.subscribe("/magmed_joystick/referenceSignal", 1000, refSignalCallback);
    
    // get measured theta
    ros::Subscriber subTipAngle = nh.subscribe("/magmed_camera/tipAngle", 1000, tipAngleCallback);

    // get angle and position of the magnet
    ros::Subscriber subPos = nh.subscribe("/magmed_manipulator/magPos", 1000, magPosCallback);

    // publish the angular velocity of the magnet
    ros::Publisher pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_controller/magVel", 1000);

    // frequency of the controller
    int nf = 100;

    ros::Rate rate(nf);

    // wait for the camera to start
    ros::Duration(3.0).sleep();

    // initialize LESO
    std::vector<float> hatx = {0.0, 0.0};

    while (ros::ok())
    {
        // // the rotation angle of work space
        // double phi = M_PI / 2.0;

        // get jacobian of the robot
        RowVector4d jacobian = mcr.get_jacobian(g_dPsi, g_dPos);

        std::cout << "jacobian: " << jacobian << std::endl;
        // virtual control law

        // ROS_INFO("jacobian: %f", jacobian);

        // calculate the control input
        double virtualControlLaw = FF_controller(hatx, g_fThetaL);

        // control allocation
        // virtualControlLaw = 15.08; // temp define
        Vector4d actualControlLaw = controlAllocation(virtualControlLaw, jacobian, nf);

        // update LESO
        LESO(actualControlLaw, jacobian, hatx, nf);

        // trans velocity frame from robot to end-effector
        actualControlLaw.segment(1, 3) = velFrmTrns(actualControlLaw.segment(1, 3), g_dPsi);

        // // set coefficient of the controller (positive)
        // float k = 3.0;
        // // applying quasi-static controller to calculate the angular velocity of the magnet
        // double dPsi = (v2dThetaR(1) + k * (v2dThetaR(0) - g_fThetaL)) / jacobian;
        // error of theta
        ROS_INFO("error of theta: %f", g_dThetaR[0] - g_fThetaL);

        // 控制器自带软限位

        std_msgs::Float64MultiArray array_msg;
        array_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        array_msg.layout.dim[0].size = 4;
        array_msg.layout.dim[0].label = "columns";

        array_msg.data = {actualControlLaw[1], actualControlLaw[2], actualControlLaw[3], actualControlLaw[0]}; // pub order: x, y, z, psi
        pub.publish(array_msg);
        ROS_INFO("velocity: [%f], [%f], [%f], [%f]", array_msg.data[0], array_msg.data[1], array_msg.data[2], array_msg.data[3]);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
