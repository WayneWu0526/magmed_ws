#pragma Once

#include <ros/ros.h>
// #include "magmed_controller/include/magmed_controller/diffKine.h"
#include <eigen3/Eigen/Dense> // 矩阵
#include <modern_robotics.h> // Jacobian
#include <cmath>
#include <iostream>

#include "magmed_msgs/RoboStates.h"
#include "magmed_msgs/RoboJoints.h"
#include "std_msgs/Float64MultiArray.h"

using namespace Eigen;
using namespace mr;

const int JOINTNUM = 7;
const int TCPNUM = 6;
const int CTRLFREQ = 100;

struct diana7KineSpaceParam
{
    // parameters of diana7
    double d1 = 285.6e-3;
    double d3 = 458.6e-3;
    double d5 = 455.4e-3;
    double d7 = 116.9e-3;
    double de = 81.0e-3; // end-effector
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
        
        /* g 就是工件坐标系*/
        // 默认的磁性机器人坐标对于机器人坐标的初始位置
        // double xg = 0.51, yg = 0.0, zg = 0.06;
        double xg = 0.63, yg = 0.04, zg = 0.132 - H_base;

        // 默认的磁传感器pose
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
        Pgb0 << 0.0, 300.00e-3, 0.0;

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

struct DianaTcp
{
    float TCP_VEL_GAIN = 0.0;
    double tcp_vel[TCPNUM] = {0.0};
    double tcp_pos[TCPNUM] = {0.0};
    double tcp_acc[TCPNUM] = {0.0};

    DianaTcp(){};
};

// diana states
class Diana
{
public:
    magmed_msgs::RoboStates robo_state;
    double joint_states_array[JOINTNUM] = {0.0};
    magmed_msgs::RoboJoints joint_states;
    void feedState(magmed_msgs::RoboStatesConstPtr pMsg);
    void feedJoints(magmed_msgs::RoboJointsConstPtr pMsg);
    Diana(){};
};

// 你的程序
class VelCtrlNode
{
public:
    void run(); // 执行

    VelCtrlNode(){};
    VelCtrlNode(ros::NodeHandle &nh) : nh(nh)
    {
        diana_jointStates_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints",
                                                                      10,
                                                                      boost::bind(&Diana::feedJoints, &diana, _1));

        diana_roboState_sub = nh.subscribe<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate",
                                                                    10,
                                                                    boost::bind(&Diana::feedState, &diana, _1));

        diana_jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels", 10);
    }

private:
    Diana diana;
    diana7KineSpaceParam params = diana7KineSpaceParam();
    ros::NodeHandle nh;
    ros::Subscriber diana_jointStates_sub; // 订阅Diana关节角度
    ros::Subscriber diana_roboState_sub; // 机械臂状态

    ros::Publisher diana_jointVels_pub; // 发布Diana关节速度
    int pubVels();
    int loadInitPose();

    int nRet = 0;
};


void Diana::feedJoints(magmed_msgs::RoboJointsConstPtr pMsg)
{
    joint_states = *pMsg;
    for (int i = 0; i < JOINTNUM; ++i)
    {
        joint_states_array[i] = joint_states.joints[i];
    }
};

void Diana::feedState(magmed_msgs::RoboStatesConstPtr pMsg)
{
    robo_state = *pMsg;
};