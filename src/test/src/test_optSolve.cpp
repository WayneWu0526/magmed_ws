#include <iostream>
#include <eigen3/Eigen/Dense>
#include "nlopt.hpp"
#include "modern_robotics.h"

using namespace Eigen;
using namespace mr;

const int JOINTNUM = 7;

MatrixXd Tsd_trans = MatrixXd::Identity(4, 4);
VectorXd qd_trans = VectorXd::Zero(JOINTNUM);

int solveDualModeJointAngles(VectorXd &q0);

struct diana7KineSpaceParam
{
    // parameters of diana7
    double d1 = 285.6e-3;
    double d3 = 458.6e-3;
    double d5 = 455.4e-3;
    double d7 = 116.9e-3;
    double de = 81.0e-3;
    // double de = 130.0e-3;
    double a4 = 65.0e-3;
    double a6 = -12.2e-3;
    double a7 = 87.0e-3;

    double dsum = d1 + d3 + d5 + d7 + de;
    double H_base = 0.02;

    Matrix4d M;
    Eigen::Matrix<double, 6, JOINTNUM> Slist;
    Matrix3d Rgb0 = Matrix3d::Identity();
    Vector3d Pgb0;
    Matrix4d Tsg;
    Matrix4d Tsc;
    // create a vector to store the minimum joint velocity limits
    VectorXd jointVelLimits;
    // create a std::vector to store the joint lower limit
    std::vector<double> jointLowerLimits;
    // create a std::vector to store the joint upper limit
    std::vector<double> jointUpperLimits;

    diana7KineSpaceParam() // 构造函数。当直接执行时，会按照下面的参数初始化
    {
        // JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();

        // 机械臂内部参数，请勿修改
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
        
        // 默认的磁性机器人坐标对于机器人坐标的初始位置
        // double xg = 0.51, yg = 0.0, zg = 0.06;
        double xg = 0.62, yg = 0.0, zg = 0.135 - H_base;

        // 默认的磁性机器人初始位置
        Tsg << 0.0, 0.0, 1.0, xg, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            1.0, 0.0, 0.0, yg,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, 1.0, 0.0, zg,    // 底盘高度：0.06m
            0.0, 0.0, 0.0, 1.0;

        Rgb0.diagonal() << -1.0, 1.0, -1.0; // 机械臂相对于机器人的初始位置

        Pgb0 << 0.0, 180.0e-3, 0.0;
        // Pgb0 << 0.0, pr.H0, 0.0;

        // 默认的深度相机与机械臂的初始位置
        Tsc << -1.0, 0.0, 0.0, xg, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            0.0, 0.0, -1.0, 0.30,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, -1.0, 0.0, 0.125 - H_base,    // 相机高度
            0.0, 0.0, 0.0, 1.0;

        // jointVelLimits << 2.61, 2.618, 2.61, 2.61, 3.14, 3.14, 3.14;
        // 关节最大运动速度限幅
        jointVelLimits = 0.5 * VectorXd::Ones(JOINTNUM);

        // jointLowerLimits = {-3.1241, -1.5708, -3.1241, 0, -3.1241, -3.1241, -3.1241};
        jointLowerLimits = {-3.036873, -1.483530, -3.036873, 0.087266, -3.036873, -3.036873, -3.036873};

        // jointUpperLimits = {3.1241, 1.5708, 3.1241, 3.0543, 3.1241, 3.1241, 3.1241};

        jointUpperLimits = {3.036873, 1.483530, 3.036873, 2.967060, 3.036873, 3.036873, 3.036873};
        // jointVelLimits << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        // jointVelLimits << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
    };

    // 构造函数，执行时会按照输入的函数进行初始化
    diana7KineSpaceParam(Matrix4d M, Eigen::Matrix<double, 6, JOINTNUM> Slist) : M(M), Slist(Slist){};
};

diana7KineSpaceParam params = diana7KineSpaceParam();

Matrix3d Rphi(double phi)
{
    Matrix3d R;
    R << 1.0, 0.0, 0.0,
        0.0, cos(phi), -sin(phi),
        0.0, sin(phi), cos(phi);
    return R;
};

Matrix3d Rpsi(double psi)
{
    Matrix3d R;
    R << cos(psi), -sin(psi), 0.0,
        sin(psi), cos(psi), 0.0,
        0.0, 0.0, 1.0;
    return R;
};

int initTransMode(const double (&thetaList)[JOINTNUM], const int TRANSMETHOD)
{
    // CTRLMODEb4Trans = CTRLMODE;
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    std::cout << "thetalist: " << thetalist << std::endl;
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    Tsd_trans = Tsb * RpToTrans(Rphi(M_PI), Vector3d::Zero());
    qd_trans << thetalist[0], thetalist[1], thetalist[2], thetalist[3], 0.0, -thetalist[5], -thetalist[6];
    int nRet = solveDualModeJointAngles(qd_trans);
    return nRet;
    // qd_trans = ...
};

double myvfunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
{
    // convert x to VectorXd
    Matrix4d * fd_ptr = (Matrix4d *) my_func_data;
    Matrix4d fd = *fd_ptr;
    VectorXd q = VectorXd::Zero(JOINTNUM);
    for (int i = 0; i < JOINTNUM; ++i) {
        q[i] = x[i];
    }
    diana7KineSpaceParam params = diana7KineSpaceParam();
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, q);
    // VectorXd f = se3ToVec(MatrixLog6(Tsb));
    if (!grad.empty()) {
        MatrixXd Js = JacobianSpace(params.Slist, q);
        MatrixXd Jb = Adjoint(Tsb) * Js;
        VectorXd Grad = Jb.transpose() * se3ToVec(MatrixLog6(TransInv(Tsb) * fd));
        for (int i = 0; i < JOINTNUM; ++i) {
            grad[i] = Grad(i);
        }
    }
    return 0.5 * se3ToVec(MatrixLog6(TransInv(Tsb) * fd)).squaredNorm();
};

int solveDualModeJointAngles(VectorXd &q0){
    // nlopt::opt opt(nlopt::LN_PRAXIS, JOINTNUM);
    printf("q0: %f, %f, %f, %f, %f, %f, %f\n", q0[0], q0[1], q0[2], q0[3], q0[4], q0[5], q0[6]);
    // printf Tsd_trans
    // nlopt::opt opt(nlopt::LN_BOBYQA, JOINTNUM);
    nlopt::opt opt(nlopt::LN_BOBYQA, JOINTNUM);
    // lb[0] = -HUGE_VAL; lb[1] = 0;
    int flag = 1;
    if(flag == 0) {
        // copy the params.jointLowerLimits to a std::vector lb
        std::vector<double> ub(params.jointUpperLimits);
        ub[5] = 0.0;
        opt.set_lower_bounds(params.jointLowerLimits);
        opt.set_upper_bounds(ub);
        // print ub
        // for (int i = 0; i < JOINTNUM; ++i) {
        //     std::cout << ub[i] << std::endl;
        // }
    }
    else{
        std::vector<double> lb(params.jointLowerLimits);
        lb[5] = 0.0;
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(params.jointUpperLimits);
        // opt.set_ftol_rel(1e-3);
        // opt.set_ftol_rel(1e-8);
        // opt.set_maxeval(1000);
        // opt.set_initial_step(1e-2);
    }
    // opt.set_min_objective(myvfunc, NULL);
    // VectorXd fd = se3ToVec(MatrixLog6(Tsd_trans));
    // opt.set_min_objective(myvfunc, &fd);
    opt.set_min_objective(myvfunc, &Tsd_trans);
    std::cout << "Tsd_trans:" << Tsd_trans << std::endl;
    // opt.set_min_objective(objectiveFunction, NULL);
    // opt.add_equality_constraint(constraintFunction, &fd, 1e-2);
    opt.set_ftol_abs(1e-3);
    // std::vector<double> x(2);
    // x[0] = 1.234; x[1] = 5.678;
    std::vector<double> x(JOINTNUM);
    for (int i = 0; i < JOINTNUM; ++i) {
        x[i] = q0[i];
    }
    double minf;

    try{
        nlopt::result result = opt.optimize(x, minf);
        // convert x to VectorXd and print it
        // VectorXd q = VectorXd::Zero(JOINTNUM);
        for (int i = 0; i < JOINTNUM; ++i) {
            q0[i] = x[i];
        }
        // std::cout << "[magmed_controller] Target joint angles found with minuum f: " << minf << std::endl;
        std::cout << "found minimum at f(" << q0.transpose() << ") = " << minf << std::endl;
        return 0;
    }
    catch(std::exception &e) {
        std::cout << "[magmed_controller] nlopt failed, trying initial value " << e.what() << std::endl;
        return 1;
    }
}

int main(int argc, char const *argv[])
{
    /* code */
    // double thetaList[JOINTNUM] = {0.0450, 0.4205, -0.0511, 2.7211, -0.0016, 1.5704, 0.0209};
    // double thetaList[JOINTNUM] = {-0.0711, 0.9185, 0.0968, 1.2668, 0.0648, -2.8931, -1.4317};
    // double thetaList[JOINTNUM] = {-0.040339, 1.086981, 0.110267, 0.902484, 0.000000, -2.978739, -0.003643};
    double thetaList[JOINTNUM] = {-0.0476854, 1.11003, 0.145956, 0.885227, 0.350754, -2.98834, 0.0221347};
    initTransMode(thetaList, 0);
    return 0;
}