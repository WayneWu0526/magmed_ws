#pragma once

#include <modern_robotics.h>
#include <eigen3/Eigen/Dense>
#define JOINTNUM 7

using namespace mr;
using namespace Eigen;

struct diana7KineSpaceParam
{
    // parameters of diana7
    double d1 = 285.6e-3;
    double d3 = 458.6e-3;
    double d5 = 455.4e-3;
    double d7 = 116.9e-3;
    double de = 81e-3;
    double a4 = 65e-3;
    double a6 = -12.2e-3;
    double a7 = 87e-3;

    double dsum = d1 + d3 + d5 + d7 + de;

    Matrix4d M;
    MatrixXd Slist;
    Matrix3d R0;
    Matrix4d Tsg;

    diana7KineSpaceParam() // 构造函数。当直接执行时，会按照下面的参数初始化
    {
        Tsg << 0.0, 0.0, 1.0, 0.51, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
        1.0, 0.0, 0.0, 0.0,           // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
        0.0, 1.0, 0.0, 0.06,          // 底盘高度：0.06m
        0.0, 0.0, 0.0, 1.0;

        R0.diagonal() << -1.0, 1.0, -1.0;

        M << 0.0, -1.0, 0.0, 0.0,
             -1.0, 0.0, 0.0, 0.0,
             0.0, 0.0, -1.0, 0.0,
             -a7, 0.0, dsum, 1.0;
        
        Slist << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 1.0, 0.0, 1.0, 0.0, -1.0, 0.0,
                 -1.0, 0.0, -1.0, 0.0, -1.0, 0.0, -1.0,
                 0.0, -d1, 0.0, -d1 - d3, 0.0, d1+d3+d5, 0.0,
                 0.0, 0.0, 0.0, 0.0, a6, 0.0, -a7,
                 0.0, 0.0, 0.0, a4, 0.0, 0.0, 0.0;
    };

    // 构造函数，执行时会按照输入的函数进行初始化
    diana7KineSpaceParam(Eigen::Matrix4d M, Eigen::MatrixXd Slist) : 
        M(M), Slist(Slist) {};
};

class MagPos{
    public:
        double phi;
        double psi;
        struct {
            double x;
            double y;
            double z;
        }P;
        // 构造函数
        MagPos(double phi, double psi, double P_x, double P_y, double P_z) : 
            phi(phi), psi(psi), P{P_x, P_y, P_z} {};
};

class DsrTwist{
    struct {
        double x;
        double y;
        double z;
    }Omg;
    struct {
        double x;
        double y;
        double z;
    }V;
    // 构造函数
    DsrTwist(double Omg_x, double Omg_y, double Omg_z, double V_x, double V_y, double V_z) : 
        Omg{Omg_x, Omg_y, Omg_z}, V{V_x, V_y, V_z} {};
};

class diffKine
{
    public:
        // JacobiMap function: MagPos -> DsrTwist
        DsrTwist JacobiMap(MagPos magPos, MagPos magdPos, VectorXd thetalist,
            const diana7KineSpaceParam& params);
    private:
        Matrix3d Rphi(double phi);
        Matrix3d Rpsi(double psi);
};