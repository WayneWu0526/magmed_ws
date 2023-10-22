#ifndef DIFFKINE_H
#define DIFFKINE_H

// #include <modern_robotics.h>
// #include <eigen3/Eigen/Dense>
// #include <eigen3/Eigen/QR>
// #include "magmed_msgs/MagPose.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/velCtrlDef.h"

// using namespace mr;
// using namespace Eigen;
// using namespace magmed_msgs;

struct diana7KineSpaceParam
{
    // parameters of diana7
    double d1 = 285.6e-3;
    double d3 = 458.6e-3;
    double d5 = 455.4e-3;
    double d7 = 116.9e-3;
    double de = 81.0e-3;
    double a4 = 65.0e-3;
    double a6 = -12.2e-3;
    double a7 = 87.0e-3;

    double dsum = d1 + d3 + d5 + d7 + de;

    Matrix4d M;
    Eigen::Matrix<double, 6, JOINTNUM> Slist;
    Matrix3d Rgb0;
    Vector3d Pgb0;
    Matrix4d Tsg;

    diana7KineSpaceParam() // 构造函数。当直接执行时，会按照下面的参数初始化
    {
        JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();

        M << 0.0, -1.0, 0.0, -a7,
            -1.0, 0.0, 0.0, 0.0,
            0.0, 0.0, -1.0, dsum,
            0.0, 0.0, 0.0, 1.0;

        Slist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0,
            -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0,
            0.0, -d1, 0.0, -d1 - d3, 0.0, d1 + d3 + d5, 0.0,
            0.0, 0.0, 0.0, 0.0, a6, 0.0, -a7,
            0.0, 0.0, 0.0, a4, 0.0, 0.0, 0.0;

        Tsg << 0.0, 0.0, 1.0, 0.51, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            1.0, 0.0, 0.0, 0.0,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, 1.0, 0.0, 0.06,    // 底盘高度：0.06m
            0.0, 0.0, 0.0, 1.0;

        Rgb0.diagonal() << -1.0, 1.0, -1.0;

        Pgb0 << pr.L, pr.H0, 0.0;
    };

    // 构造函数，执行时会按照输入的函数进行初始化
    diana7KineSpaceParam(Matrix4d M, Eigen::Matrix<double, 6, JOINTNUM> Slist) : M(M), Slist(Slist){};
};

struct PICtrlParam
{
    double kp_; // 比例系数
    double ki_; // 积分系数
    int nf_;    //
    PICtrlParam()
    {
        kp_ = 30.0;
        ki_ = 10.0;
        nf_ = CTRLFREQ;
    }; // 构造函数

    PICtrlParam(double kp, double ki, int nf) // 构造函数
    {
        kp_ = kp;
        ki_ = ki;
        nf_ = nf;
    };
};

class diffKine
{
public:
    diana7KineSpaceParam params;
    void initConfig(const double (&thetaList)[JOINTNUM]);
    void getRealMagPose(MagPose &magPose, const double (&thetaList)[JOINTNUM]);
    VectorXd jacobiMap(magmed_msgs::RefPhi const refPhi, const double (&thetaList)[JOINTNUM]);
    MagPose magTwist;
    diffKine()
    {
        magTwist.psi = 0.0;
        magTwist.pos << 0.0, 0.0, 0.0;
    };

private:
    Matrix3d Rinit = Matrix3d::Identity();
    Matrix4d T0 = Matrix4d::Identity(); // 初始位姿
    // 构建static TR, 初始时为单位矩阵
    Matrix4d TR = Matrix4d::Identity();
    PICtrlParam piparams;
    // JacobiMap function: MagPos -> DsrTwist
    Eigen::MatrixXd calcJacobi();
    Matrix3d Rphi(double phi);
    Matrix3d Rpsi(double psi);
};

#endif