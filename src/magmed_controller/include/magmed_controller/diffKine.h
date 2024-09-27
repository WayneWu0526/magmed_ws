#ifndef DIFFKINE_H
#define DIFFKINE_H

#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/velCtrlDef.h"

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
    Matrix4d Tsw;
    // create a vector to store the minimum joint velocity limits
    VectorXd jointVelLimits;
    // create a std::vector to store the joint lower limit
    std::vector<double> jointLowerLimits;
    // create a std::vector to store the joint upper limit
    std::vector<double> jointUpperLimits;

    diana7KineSpaceParam() // 构造函数。当直接执行时，会按照下面的参数初始化
    {
        JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();

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
        double xg = 0.63, yg = 0.04, zg = 0.132 - H_base;

        // 默认的磁性机器人初始位置
        Tsg << 0.0, 0.0, 1.0, xg, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            1.0, 0.0, 0.0, yg,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, 1.0, 0.0, zg,    // 底盘高度：0.06m
            0.0, 0.0, 0.0, 1.0;

        Tsw << 0.0, 0.0, 1.0, xg, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            1.0, 0.0, 0.0, 0.0,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, 1.0, 0.0, zg,    // 底盘高度：0.06m
            0.0, 0.0, 0.0, 1.0;

        Rgb0.diagonal() << -1.0, 1.0, -1.0; // 机械臂相对于机器人的初始位置

        // Pgb0 << pr.L, pr.H0, 0.0;
        Pgb0 << 0.0, pr.H0, 0.0;

        // 默认的深度相机与机械臂的初始位置
        Tsc << -1.0, 0.0, 0.0, xg, // x轴延伸0.3+0.32=0.92m, 0.19+0.32=0.51m
            0.0, 0.0, -1.0, yg + 0.2906,     // 机械臂坐标系的x轴与工作空间坐标系的y轴重合
            0.0, -1.0, 0.0, 0.115 - H_base,    // 相机高度
            0.0, 0.0, 0.0, 1.0;

        // jointVelLimits << 2.61, 2.618, 2.61, 2.61, 3.14, 3.14, 3.14;
        // 关节最大运动速度限幅
        jointVelLimits = 0.5 * VectorXd::Ones(JOINTNUM);

        // jointLowerLimits = {-3.1241, -1.5708, -3.1241, 0, -3.1241, -3.1241, -3.1241};
        // jointLowerLimits = {-3.036873, -1.483530, -3.036873, 0.087266, -3.036873, -3.036873, -3.036873};
        jointLowerLimits = {-3.036873, -1.483530, -3.036873, 0.087266, -1.0, -3.036873, -3.036873};

        // jointUpperLimits = {3.1241, 1.5708, 3.1241, 3.0543, 3.1241, 3.1241, 3.1241};

        jointUpperLimits = {3.036873, 1.483530, 3.036873, 2.967060, 1.0, 3.036873, 3.036873};
        // jointUpperLimits = {3.036873, 1.483530, 3.036873, 2.967060, 3.036873, 3.036873, 3.036873};
        // jointVelLimits << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        // jointVelLimits << 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5;
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
        kp_ = 1.0; // default: 2.0
        ki_ = 0.0;
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
    // void initConfig(const double (&thetaList)[JOINTNUM]);
    void getMagPose(MagPose &magPose, const double (&thetaList)[JOINTNUM], Matrix4d Tsg, const int CTRLMODE);
    VectorXd jacobiMap(const double (&refPhi)[2], const VectorXd &V_sg, const double (&thetaList)[JOINTNUM], const int CTRLMODE);
    VectorXd jacobiMap_tcp(const double (&tcpVels)[TCPNUM], const double (&thetaList)[JOINTNUM], const unsigned char BANA_MODE);
    int initTransMode(const double (&thetaList)[JOINTNUM], const int TRANSMETHOD, enum_CTRLMODE CTRLMODE);
    int solveDualModeJointAngles(VectorXd &q0);
    VectorXd ctrlModeTrans(const double (&thetaList)[JOINTNUM], enum_CTRLMODE* CTRLMODE, int TRANSMETHOD);
    int scalingJointVels(const double (&thetaList)[JOINTNUM], VectorXd &dq, const VectorXd &dqMax);
    MagPose magTwist; // 磁性机器人的期望速度，包括psi和pos
    double magPhi; // 磁性机器人的滚转角度
    std_msgs::Float64MultiArray magPoseMsg;
    geometry_msgs::PoseStamped magPoseStamped;
    void pushMagPoseMsg(MagPose magPose, MagPose magTwist)
    {
        magPoseMsg.data.clear();
        magPoseMsg.data.push_back(magPose.psi);
        magPoseMsg.data.push_back(magPose.pos[0]);
        magPoseMsg.data.push_back(magPose.pos[1]);
        magPoseMsg.data.push_back(magPose.pos[2]);
        magPoseMsg.data.push_back(magTwist.psi);
        magPoseMsg.data.push_back(magTwist.pos[0]);
        magPoseMsg.data.push_back(magTwist.pos[1]);
        magPoseMsg.data.push_back(magTwist.pos[2]);
        return;
    }
    void pushMagPoseStamped(std::vector<MatrixXd> Rp)
    {
        magPoseStamped.header.stamp = ros::Time::now();
        magPoseStamped.pose.position.x = Rp[1](0, 0);
        magPoseStamped.pose.position.y = Rp[1](1, 0);
        magPoseStamped.pose.position.z = Rp[1](2, 0);
        Eigen::Quaterniond q(Rp[0].block<3, 3>(0, 0));
        magPoseStamped.pose.orientation.x = q.x();
        magPoseStamped.pose.orientation.y = q.y();
        magPoseStamped.pose.orientation.z = q.z();
        magPoseStamped.pose.orientation.w = q.w();
        return;
    }
    diffKine(): params(diana7KineSpaceParam()) {   
        magTwist.psi = 0.0;
        magTwist.pos << 0.0, 0.0, 0.0;
        magPhi = 0.0;
        magPoseMsg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        magPoseMsg.layout.dim[0].size = 7;
        magPoseMsg.layout.dim[0].stride = 7;
        magPoseMsg.layout.dim[0].label = "magPose";

        magPoseStamped.pose.position.x = 0.0;
        magPoseStamped.pose.position.y = 0.0;
        magPoseStamped.pose.position.z = 0.0;
        magPoseStamped.pose.orientation.x = 0.0;
        magPoseStamped.pose.orientation.y = 0.0;
        magPoseStamped.pose.orientation.z = 0.0;
        magPoseStamped.pose.orientation.w = 0.0;
    };

private:
    Matrix4d Tsd_trans = Matrix4d::Identity();
    VectorXd qd_trans = VectorXd::Zero(JOINTNUM);
    enum_CTRLMODE CTRLMODEb4Trans = enum_CTRLMODE::TRANS;
    Matrix3d Rotphi = Matrix3d::Identity();
    Matrix3d Rinit = Matrix3d::Identity();
    // void smoothTraj(Vector2d &Omega, const Vector2d &OMEGA, double T);
    PICtrlParam piparams = PICtrlParam();
    // JacobiMap function: MagPos -> DsrTwist
    Matrix3d Rphi(double phi);
    Matrix3d Rpsi(double psi);

    double phi_d[2] = {0.0};
    double psi_d[2] = {0.0};
    Vector3d pos_d[2] = {Vector3d::Zero()};
};

#endif