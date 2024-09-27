#pragma once

#include <ros/ros.h>
#include "magmed_controller/velCtrlDef.h"
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/diffKine.h"
#include "magmed_controller/optCtrl.h"

/*** 重要!!!! set SYSCTRLMODE = 0 for manually-tcp****/
const int SYSCTRLMODE = 2; // -1: test 0：Manually-tcp，1：Open-loop-circle, 2: Closed-loop 

template <typename T>
class LinearTrackingDifferentiator {
public:
    static double v_last;
    LinearTrackingDifferentiator(T k1, T k2) 
        : k1_(k1), k2_(k2), x1_(0), x2_(0) {}

    void update(T v, T dt) {
        // Calculate derivatives
        T dx1 = x2_;
        T dx2 = -k1_ * (x1_ - v) - k2_ * x2_;
        
        // Update states
        x1_ += dx1 * dt;
        x2_ += dx2 * dt;
    }

    void update_variablegain(T v, T dt) {
        // Calculate derivatives
        T dx1 = x2_;
        T dx2 = -((100.0 - 10.0) / (0.12 - M_PI) * (abs(x1_ - v) - M_PI) + 10.0) * (x1_ - v) - k2_ * x2_;
        
        // Update states
        x1_ += dx1 * dt;
        x2_ += dx2 * dt;        
    }

    T getX1() const { return x1_; }
    T getX2() const { return x2_; }

private:
    T k1_;
    T k2_;
    T x1_;
    T x2_;
};

class TsgPoseTwist
{
public:
    magmed_msgs::PoseTwist pose_twist;
    VectorXd Tsg_pose_twist;
    TsgPoseTwist(): Tsg_pose_twist(VectorXd::Zero(2*TCPNUM)) {
        // Tsg_pose_twist = VectorXd::Zero(2*TCPNUM);
        // se3ToVec(MatrixLog6(diffKine::params.Tsg));

        pose_twist.pose.position.x = 0.0;
        pose_twist.pose.position.y = 0.0;
        pose_twist.pose.position.z = 0.0;
        pose_twist.pose.orientation.x = 0.0;
        pose_twist.pose.orientation.y = 0.0;
        pose_twist.pose.orientation.z = 0.0;
        pose_twist.pose.orientation.w = 1.0;

        pose_twist.twist.linear.x = 0.0;
        pose_twist.twist.linear.y = 0.0;
        pose_twist.twist.linear.z = 0.0;
        pose_twist.twist.angular.x = 0.0;
        pose_twist.twist.angular.y = 0.0;
        pose_twist.twist.angular.z = 0.0;
    };
    void feed(magmed_msgs::PoseTwistConstPtr pMsg);
};

class Linear_actuator
{
public:
    VectorXd gframe_twist;
    Matrix4d Tsg;
    std_msgs::Float64MultiArray la_msg;
    void feed(std_msgs::Float64ConstPtr pMsg)
    {
        // save the linear actuator velocity to gframe_twist(3)
        gframe_twist = VectorXd::Zero(6);
        gframe_twist(3) = pMsg->data;
        // Tsg *= MatrixExp6( 1.0 / CTRLFREQ * VecTose3(gframe_twist));
        // std::cout << "Tsg: " << Tsg << std::endl;
    }
    void feed_TsgY(std_msgs::Float64ConstPtr pMsg)
    {
        // Tsg(1, 3) = pMsg->data - 0.28;
        Tsg(1, 3) = pMsg->data - 0.229;
    }   
    Linear_actuator() {
        diana7KineSpaceParam params;
        Tsg = params.Tsg;
        gframe_twist = VectorXd::Zero(6);

        la_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        la_msg.layout.dim[0].label = "la_vel";
        la_msg.layout.dim[0].size = 2;
        la_msg.layout.dim[0].stride = 2;
    };
};

// camera feedback
// class TipAngle
// {
// public:
//     magmed_msgs::TipAngle tip_angle;
//     TipAngle() { tip_angle.tipAngle = 0.0; };
//     void feed(magmed_msgs::TipAngleConstPtr pMsg);
// };

// joystick input (manual control)
class Joystick
{
public:
    magmed_msgs::PFjoystick joystick;
    Feeder feeder;
    DianaTcp dianaTcp;
    MagCR magCR;
    double gamma; // theta的增益系数
    // LinearTrackingDifferentiator<double> phirltd;
    // LinearTrackingDifferentiator<double> thetarltd;
    void feed(magmed_msgs::PFjoystickConstPtr pMsg);
    Joystick(): gamma(0.8) {};
    // 1000.0: 增益系数，控制速度
};


class RefGenerator
{
public:
    double refPhi = 0.0; // phi=0.0开始
    double refTheta = 0.0; // theta=0.0开始
    double refTsgX = -0.06; // 从--100mm开始
    Vector3d refPoint = Vector3d::Zero();
    // double refTsgX = 0.0;

    double last_phi = 0.0;
    double last_theta = 0.0;
    double last_TsgX = 0.0;
    Vector3d last_xL = Vector3d::Zero();

    double GAIN_THETA = 0.6;
    int CIRCLE_PHI = 2; // 转两圈
    double LENGTH_TsgX = 0.1;
    double t = 0.0;

    int NUMOFPATH = 100; // 200 采样点
    // pub ref_signal
    std_msgs::Float64MultiArray ref_state;
    void updateRef(double phi_r, double phi, double theta_r, double theta, double TsgX)
    {
        // generate ref signals
        refPhi = 0.0;
        refTheta = 0.0;
        refTsgX = -0.01;        
        // use ros time to generate ref signals
    };

    std::vector<std::vector<double>> data;
    void updateRef_point(double phi_r, double phi, Vector3d xL)
    {
        std::cout << "phi_r: " << phi_r << " phi: " << phi << std::endl;
        std::cout << "ref_xL: " << refPoint << " xL: " << xL << "xe:" << (refPoint.segment(0, 2) - xL.segment(0, 2)).norm() << std::endl;
        if((fabs(phi_r - phi) < 0.01 && (refPoint.segment(0, 2) - xL.segment(0, 2)).norm() < 0.001) || (path_trying_num > MAX_TRY))
        {
            if((fabs(phi - last_phi) < 0.001 && (xL.segment(0, 2) - last_xL.segment(0, 2)).norm() < 0.01) || (path_trying_num > MAX_TRY))
            {
                if(path_trying_num > MAX_TRY){
                    ROS_INFO("Path point index %d reach maximum trying num, moving to next point", ind);
                }
                refPhi = data[ind][0];
                refPoint << data[ind][1], data[ind][2], data[ind][3];
                ind++;
                path_trying_num = 0;
                if(ind >= data.size())
                {
                    ros::shutdown();
                }
            }
        }
        std::cout << "current path index: " << ind << std::endl;
        last_phi = phi;
        last_xL = xL;
        if(ind != 0)    path_trying_num++;
        std::cout << "path_trying_num: " << path_trying_num << std::endl;
    };

    void loadRef(std::string filename){
        std::ifstream inputFile(filename);  // 替换为你的文件名
        std::cout << filename << std::endl;
        if (!inputFile) {
            std::cerr << "无法打开文件! 错误原因: " << std::strerror(errno) << std::endl;
            return;
        }

        std::string line;
        
        while (std::getline(inputFile, line)) {
            std::istringstream ss(line);
            std::vector<double> row;
            double value;
            while (ss >> value) {
                row.push_back(value);
                if (ss.peek() == '\t') {
                    ss.ignore();  // 忽略tab分隔符
                }
            }
            data.push_back(row);
        }

        inputFile.close();

        // 输出读取的数据进行验证
        for (const auto& row : data) {
            for (const auto& value : row) {
                std::cout << value << " ";
            }
            std::cout << std::endl;
        }
        return;
    };


    void updateRef_ex(double phi_r, double phi, double theta_r, double theta, double TsgX)
    {
        // cout refsignals
        std::cout << "refPhi: " << phi_r << " refTheta: " << theta_r << " refTsgX: " << refTsgX << std::endl;
        std::cout << "phi: " << phi << " theta: " << theta << " TsgX: " << TsgX << std::endl;
        std::cout << "phi_e: " << abs(phi_r - phi) << "theta_e: " << abs(theta_r - theta) << " TsgXe: " << abs(refTsgX - TsgX) << std::endl;
        // 角度误差至少在0.1弧度以内，位置误差至少在0.001m以内
        if((fabs(phi_r - phi) <= 0.01) && ((fabs(theta_r - theta) <= 0.01) || (fabs(theta - last_theta) <= 0.05)) && (fabs(refTsgX - TsgX) <= 0.001))
        {
            // 误差不再增加
            if((fabs(phi - last_phi) <= 0.005) && (fabs(theta - last_theta) <= 0.1) && (fabs(TsgX - last_TsgX) <= 0.0001))
            {
                // refTheta = ind / NUMOFPATH * GAIN_THETA;
                refTheta = GAIN_THETA;
                // refPhi = sin(ind / NUMOFPATH * CIRCLE_PHI * 2 * M_PI) * M_PI / 2.0;
                refPhi = -M_PI + 2.0 * M_PI * ((double) (ind % (NUMOFPATH / CIRCLE_PHI)) / (NUMOFPATH / CIRCLE_PHI));
                // std::cout << (ind % (NUMOFPATH / 4)) / (NUMOFPATH / 4) << std::endl;
                refTsgX += LENGTH_TsgX / NUMOFPATH;
                ROS_INFO("Path point index %d reached, moving to next point", ind);
                ind++;
                if (ind == NUMOFPATH)
                {
                    refTsgX = 0.0;
                    refPhi = 0.0;
                    refTheta = GAIN_THETA;
                    ROS_INFO("Path finished, moving to the beginning");
                    ind = 0;
                }
            }
        }
        last_phi = phi;
        last_theta = theta;
        last_TsgX = TsgX;
        return;
    };

    RefGenerator()
    {
        loadRef("/home/zhang/magmed_ws/src/magmed_controller/path_following_control_ref/pathfollow_points_square.txt");
        refPhi = data[0][0];
        refPoint << data[0][1], data[0][2], data[0][3];
        
        ref_state.layout.dim.push_back(std_msgs::MultiArrayDimension());
        ref_state.layout.dim[0].label = "ref_state";
        ref_state.layout.dim[0].size = 5;
        ref_state.layout.dim[0].stride = 5;
    };
private:
    int ind = 0;
    int path_trying_num = 0;
    int MAX_TRY = 300;
};

class RefSmoother
{
public:
    MagCR magCR;
    LinearTrackingDifferentiator<double> phirltd;
    LinearTrackingDifferentiator<double> thetarltd;
    double refPhi_;
    double refTheta_;
    double t;
    void refsmooth(double refPhi, double refTheta)
    {
        if(refPhi > M_PI / 2.0)
        {
            refPhi -= M_PI;
            refTheta = -refTheta;
        }
        else if(refPhi < -M_PI / 2.0)
        {
            refPhi += M_PI;
            refTheta = -refTheta;
        }

        refPhi_ = refPhi;
        refTheta_ = refTheta;
        // phirltd.update(refPhi_, 1.0 / CTRLFREQ);
        phirltd.update_variablegain(refPhi, 1.0 / CTRLFREQ);
        thetarltd.update(refTheta_, 1.0 / CTRLFREQ);

        magCR.phi[0] = phirltd.getX1();
        magCR.phi[1] = phirltd.getX2();

        magCR.theta[0] = thetarltd.getX1();
        magCR.theta[1] = thetarltd.getX2();

        std::cout << "phi_ref: " << magCR.phi[0] << " theta_ref: " << magCR.theta[0] << std::endl;
        // t += 1.0 / CTRLFREQ;
        // magCR.theta[0] = M_PI / 2.0 * sin(2 * M_PI / 10.0 * t + M_PI / 2.0);
        // std::cout << "theta_ref: " << magCR.theta[0] << std::endl;
        // magCR.theta[1] = 2 * M_PI / 10.0 * M_PI / 2.0 * cos(2 * M_PI / 10.0 * t + M_PI / 2.0);
    };
    // RefSmoother(): phirltd(3000.0, 100.0), thetarltd(3000.0, 100.0) 
    RefSmoother() : phirltd(20.0, 100.0), thetarltd(3000.0, 100.0)
    {
        t = 0.0;
        refsmooth(0.0, 0.0);
    };
private:
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

class StereoCamera
{
public:
    geometry_msgs::PointStamped tip_point;
    Vector3d tip_point_vec = Vector3d::Zero();
    std_msgs::Float64MultiArray magCR_state_measured_msg;
    void feedTip(geometry_msgs::PointStampedConstPtr pMsg);
    void push_magCR_state_measured_msg(std::array<double, 5> magCRstate, Matrix4d Tsg){
        magCR_state_measured_msg.data.clear();
        magCR_state_measured_msg.data.push_back(magCRstate[0]);
        magCR_state_measured_msg.data.push_back(magCRstate[1]);
        magCR_state_measured_msg.data.push_back(magCRstate[2]);
        magCR_state_measured_msg.data.push_back(magCRstate[3]);
        magCR_state_measured_msg.data.push_back(magCRstate[4]);
        magCR_state_measured_msg.data.push_back(Tsg(1, 3));
    }
    StereoCamera(){
        magCR_state_measured_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
        magCR_state_measured_msg.layout.dim[0].label = "magCR_state_measured";
        magCR_state_measured_msg.layout.dim[0].size = 6;
        magCR_state_measured_msg.layout.dim[0].stride = 6;
    };
};

// velocity control algorithm
class VelCtrlNode
{
public:
    void run();

    VelCtrlNode(){};
    VelCtrlNode(ros::NodeHandle &nh) : nh(nh)
    {
        joystick_sub = nh.subscribe<magmed_msgs::PFjoystick>("/magmed_joystick/joystick_controller",
                                                             10,
                                                             boost::bind(&Joystick::feed, &joystick, _1));

        // tipAngle_sub = nh.subscribe<magmed_msgs::TipAngle>("/magmed_camera/tipAngle",
        //                                                    10,
        //                                                    boost::bind(&TipAngle::feed, &tipAngle, _1));

        stereoCamera_sub = nh.subscribe<geometry_msgs::PointStamped>("/magmed_stereoCamera/tipPoint",
                                                              10,
                                                              boost::bind(&StereoCamera::feedTip, &stereoCamera, _1));

        TsgPoseTwist_sub = nh.subscribe<magmed_msgs::PoseTwist>("/magmed_stereoCamera/twist",
                                                            10,
                                                            boost::bind(&TsgPoseTwist::feed, &tsgPoseTwist, _1));

        diana_jointStates_sub = nh.subscribe<magmed_msgs::RoboJoints>("/magmed_manipulator/dianajoints",
                                                                      10,
                                                                      boost::bind(&Diana::feedJoints, &diana, _1));

        diana_roboState_sub = nh.subscribe<magmed_msgs::RoboStates>("/magmed_manipulator/dianastate",
                                                                    10,
                                                                    boost::bind(&Diana::feedState, &diana, _1));
        
        linear_actuator_sub = nh.subscribe<std_msgs::Float64>("/linear_actuator/vel",
                                                                        10, 
                                                                        boost::bind(&Linear_actuator::feed, &la, _1));

        // TsgY_sub = nh.subscribe<std_msgs::Float64>("/magmed_stereoCamera/backgroundDepth",
        //                                             10,
        //                                             boost::bind(&Linear_actuator::feed_TsgY, &la, _1));

        TsgY_sub = nh.subscribe<std_msgs::Float64>("/magmed_stereoCamera_D405/april_tag_depth",
                                                    10,
                                                    boost::bind(&Linear_actuator::feed_TsgY, &la, _1));

        diana_jointVels_pub = nh.advertise<magmed_msgs::RoboJoints>("/magmed_manipulator/joint_vels", 10);

        feeder_vel_pub = nh.advertise<std_msgs::UInt32MultiArray>("/magmed_feeder/vel", 10);

        linear_actuator_pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_linearactuator/vel_ctrl", 10);

        magCR_state_pub = nh.advertise<magmed_msgs::MagCR>("/magmed_controller/magCR_state", 10);

        ref_state_pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_controller/ref_state", 10);

        magPose_pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_controller/magPose", 10);

        magTgb_pub = nh.advertise<geometry_msgs::PoseStamped>("/magmed_controller/magTgb", 10);

        magCR_state_measured_pub = nh.advertise<std_msgs::Float64MultiArray>("/magmed_controller/magCR_state_measured", 10);

        control_mode_pub = nh.advertise<std_msgs::UInt32>("/magmed_controller/control_mode", 10);

        selfcollision_client = nh.serviceClient<magmed_msgs::SelfCollisionCheck>("/magmed_modules/selfCollisionCheck"); 
                // ros::service::waitForService("/magmed_manipulator/roboStates", -1);
    };

private:
    Joystick joystick;
    // TipAngle tipAngle;
    TsgPoseTwist tsgPoseTwist;
    Diana diana;
    StereoCamera stereoCamera;
    Linear_actuator la;
    RefSmoother refsmoother;
    RefGenerator refgenerator;

    ros::NodeHandle nh;
    ros::Subscriber joystick_sub;
    // ros::Subscriber tipAngle_sub;
    ros::Subscriber diana_jointStates_sub;
    ros::Subscriber diana_roboState_sub;
    ros::Subscriber TsgPoseTwist_sub;
    ros::Subscriber stereoCamera_sub;
    ros::Subscriber linear_actuator_sub;
    ros::Subscriber TsgY_sub;

    ros::Publisher diana_jointVels_pub;
    ros::Publisher feeder_vel_pub;
    ros::Publisher linear_actuator_pub;
    ros::Publisher magCR_state_pub;
    ros::Publisher magCR_state_measured_pub;
    ros::Publisher magPose_pub;
    ros::Publisher magTgb_pub;
    ros::Publisher control_mode_pub;

    ros::Publisher ref_state_pub;

    ros::ServiceClient selfcollision_client;

    MSCRJacobi mscrjacobi;
    optCtrl optctrl;
    diffKine diffkine;
    int pubVels();
    int loadInitPose();
    std::array<double, 5> calcMagCRstate(const double alpha, double thetaL_mock, Matrix4d Tsg, int DEEPFLAG);

    int nRet = 0;
    enum_CTRLMODE CTRLMODE = enum_CTRLMODE::NM; // CTRLMODE
    enum_TRANSMETHOD TRANSMETHOD = enum_TRANSMETHOD::OFT; // Translation method
    bool SCC_FLAG = false;
    // bool isSwitching = false;
};

// void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
// {
//     tip_angle = *pMsg;
// };

void StereoCamera::feedTip(geometry_msgs::PointStampedConstPtr pMsg)
{
    tip_point = *pMsg;
    tip_point_vec << tip_point.point.x, tip_point.point.y, tip_point.point.z;
};

void TsgPoseTwist::feed(magmed_msgs::PoseTwistConstPtr pMsg)
{
    pose_twist = *pMsg;
    // convert orientation to rotation matrix
    Quaterniond quat(pose_twist.pose.orientation.w, pose_twist.pose.orientation.x,
        pose_twist.pose.orientation.y, pose_twist.pose.orientation.z);
    Vector3d rot = so3ToVec((quat.toRotationMatrix()));
    Tsg_pose_twist << 
        rot,
        pose_twist.pose.position.x,
        pose_twist.pose.position.y,
        pose_twist.pose.position.z,
        pose_twist.twist.angular.x,
        pose_twist.twist.angular.y,
        pose_twist.twist.angular.z,
        pose_twist.twist.linear.x,
        pose_twist.twist.linear.y,
        pose_twist.twist.linear.z;
    // mock data
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

void Joystick::feed(magmed_msgs::PFjoystickConstPtr pMsg)
{
    joystick = *pMsg;
    // feeder 数据
    feeder.FEEDER_VEL_GAIN_FB = joystick.POTA * 50;
    feeder.feeder_vel_FB = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_FB * joystick.nJOY3[0]);
    feeder.FEEDER_VEL_GAIN_UD = joystick.POTB * 50;
    feeder.feeder_vel_UD = static_cast<uint32_t>(feeder.FEEDER_VEL_GAIN_UD * joystick.nJOY2[1]);
    // diana tcp 数据
    dianaTcp.TCP_VEL_GAIN = joystick.POTB / 10000.0 * M_PI;
    if (joystick.bJOYD)
    {
        dianaTcp.tcp_vel[0] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[1];
        dianaTcp.tcp_vel[1] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[0];
        dianaTcp.tcp_vel[2] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[2];
        dianaTcp.tcp_vel[3] = 0.0;
        dianaTcp.tcp_vel[4] = 0.0;
        dianaTcp.tcp_vel[5] = 0.0;
    }
    else
    {
        dianaTcp.tcp_vel[0] = 0.0;
        dianaTcp.tcp_vel[1] = 0.0;
        dianaTcp.tcp_vel[2] = 0.0;
        dianaTcp.tcp_vel[3] = - dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[1];
        dianaTcp.tcp_vel[4] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[0];
        dianaTcp.tcp_vel[5] = dianaTcp.TCP_VEL_GAIN * joystick.nJOY1[2];
    }
    // magCR 数据 增量数据
    // magCR.phi[0] = M_PI / 2.0 * joystick.nJOY1[1];
    // magCR.phi[1] = 0.0; // under estimated using ADRC

    switch(SYSCTRLMODE)
    {
        case 0: // manually-tcp
        {
            magCR.phi[0] = 0.0;
            magCR.phi[1] = 0.0;
            magCR.theta[0] = 0.0;
            magCR.theta[1] = 0.0;
            break;
        }
        case 1: // open-loop circle
        {
            magCR.phi[0] += joystick.nJOY1[1] / CTRLFREQ;
            magCR.phi[1] = joystick.nJOY1[1];

            magCR.theta[0] += joystick.nJOY1[2] / CTRLFREQ;
            magCR.theta[1] = joystick.nJOY1[2];
            break;
        }
        case 2:
        {
            /* using mock value */
            magCR.phi[0] = atan2(-joystick.nJOY1[0], joystick.nJOY1[1]);
            magCR.theta[0] = gamma * sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2));

            // printf("ref phi: %f, ref theta: %f\n", joyPhi, joyTheta);           
            // printf("phi: %f, theta: %f\n", magCR.phi[0], magCR.theta[0]);
            // printf("dphi: %f, dtheta: %f\n", magCR.phi[1], magCR.theta[1]);
            break;
        }
        default:
        {
            break;
        }
    }
    // printf("phi circle %f\n", atan2(-joystick.nJOY1[0], joystick.nJOY1[1]));
    // printf("theta circle %f\n", sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2)));

    // magCR.phi[0] += magCR.phi[1] / CTRLFREQ;
    // magCR.theta[0] = 0.0;
    // magCR.theta[1] = joystick.nJOY1[2];
};

