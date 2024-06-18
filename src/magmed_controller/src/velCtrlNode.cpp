#include "magmed_controller/velCtrlNode.hpp"

// 自动和手动模式。手动：0，openloop：1 cls loop2
void VelCtrlNode::run()
{
    int initFlag = 0;
    ros::Rate loop_rate(CTRLFREQ);
    ros::service::waitForService("/magmed_modules/selfCollisionCheck", -1);
    ROS_WARN("[mamged_controller] Current SYSCTRLMODE: %d", SYSCTRLMODE);
    while (ros::ok())
    {
        // printf("diana.robo_state.VAL: %d\n", diana.robo_state.VAL);
        switch (diana.robo_state.VAL)
        {
            case 0: // robot init
                if (initFlag == 0)
                {
                    loadInitPose();
                    initFlag = 1;
                }
                break;
            // 机械臂已到达初始位置
            case 1: // robot run
                // if (initFlag == 1)
                // {
                //     diffkine.initConfig(diana.joint_states_array);
                //     initFlag = -1;
                //     ROS_INFO("Ctrl init finished\n");
                // }
                nRet = pubVels();
                break;
            case -1: // robot term
                ROS_WARN("Robot term, shutting down ros\n");
                ros::shutdown();
                break;
            default:
                break;
        }
        ros::spinOnce();
        // pubJoints();
        loop_rate.sleep();
    }
};

int VelCtrlNode::loadInitPose()
{
    // count init config
    std::vector<double> pose(TCPNUM, 0.0);
    std::vector<Eigen::MatrixXd> Rp = TransToRp(diffkine.params.Tsg);
    Vector3d P0 = Rp[0] * diffkine.params.Pgb0 + Rp[1];
    pose[0] = P0(0);
    pose[1] = P0(1);
    pose[2] = P0(2);
    Matrix3d R0 = Rp[0] * diffkine.params.Rgb0;
    Eigen::AngleAxisd axis_angle(R0);
    pose[3] = axis_angle.angle() * axis_angle.axis()(0);
    pose[4] = axis_angle.angle() * axis_angle.axis()(1);
    pose[5] = axis_angle.angle() * axis_angle.axis()(2);
    // load init config
    std_msgs::Float64MultiArray msg;
    msg.data = pose;
    ros::param::set("/magmed_controller/initPose", msg.data);
    ROS_INFO("[magmed_controller] Init pose loaded\n");
    return 0;
};

// 闭环控制还需要分两种，一种是手柄操作一种是直接操作
int VelCtrlNode::pubVels()
{
    magmed_msgs::RoboJoints joint_vels;
    // get real mag pose
    diffkine.getMagPose(optctrl.magPose, diana.joint_states_array);
    // get mag twist
    // double tip_angle = calcTipAngle(1.52, diana.joint_states_array); // alpha
    // diffkine.magTwist = optctrl.generateMagTwist(refSignal.ref_theta, tipAngle.tip_angle);
    // diffkine.magTwist.psi = joystick.magCR.theta[1];
    // get joint vels
    // refSignal.ref_phi.dphi = 0.01;

    // pub feeder
    std_msgs::UInt32MultiArray feeder_msg;
    feeder_msg.layout.dim.push_back(std_msgs::MultiArrayDimension());
    feeder_msg.layout.dim[0].label = "feeder_vel";
    feeder_msg.layout.dim[0].size = 2;
    feeder_msg.layout.dim[0].stride = 2;
    switch(SYSCTRLMODE)
    {
    case 0: // manually-tcp
    {
        VectorXd jointVels = diffkine.jacobiMap_tcp(joystick.dianaTcp.tcp_vel, diana.joint_states_array);
        for (int i = 0; i < JOINTNUM; ++i)
        {
            // push_back joint_vels
            joint_vels.joints.push_back(jointVels(i));
        }

        // using joystick data to control feeder
        feeder_msg.data.clear();
        feeder_msg.data.push_back(joystick.feeder.feeder_vel_FB);
        feeder_msg.data.push_back(joystick.feeder.feeder_vel_UD);
        break;
    }
    case 1: // open-loop circle
    {
        diffkine.magTwist.psi = joystick.magCR.theta[1];
    }
    case 2: // closed-loop version
    {
        diffkine.magTwist.psi = joystick.magCR.theta[1];
        VectorXd jointVels = VectorXd::Zero(JOINTNUM);
        switch(CTRLMODE)
        {
            case enum_CTRLMODE::TRANS:
            {
                // CTRLMODE = enum_CTRLMODE::NM;
                // nRet = diffkine.initTransMode(diana.joint_states_array, TRANSMETHOD, CTRLMODE);
                // if(nRet != 0)
                // {
                //     ROS_ERROR("[magmed_controller] failed to initialize transmode\n");
                // }
                // else
                // {
                //     ROS_INFO("[magmed_controller] transmode initialized\n");
                //     // 切换到TRANS模式
                //     // 切换到TRANS模式
                //     CTRLMODE = enum_CTRLMODE::TRANS;
                // }
                // ROS_INFO("[magmed_controller] Current CTRLMODE: %d", CTRLMODE);
                // 1.2.1. 如果检查后显示不会碰撞，则让diffKine.jacobiMap发布切换过程的jointVels
                // 1.2.2. 如果检查后仍碰撞，则停止机械臂运动
                // VectorXd jointVels = diffkine.jacobiMap(refSignal.ref_phi, jointStates.joint_states_array);
                // jointVels = diffkine.jacobiMap(joystick.magCR.phi, diana.joint_states_array, CTRLMODE);
                // jointVels = diffkine.ctrlModeTrans(diana.joint_states_array, &CTRLMODE, TRANSMETHOD);
                jointVels = diffkine.ctrlModeTrans(diana.joint_states_array, &CTRLMODE, TRANSMETHOD);
                break;
            }
            default:
            {
                // ROS_INFO("[magmed_controller] Current CTRLMODE: %d", CTRLMODE);
                // using feedback value: tsgPoseTwist.Tsg_pose_twist
                // using mock value:
                jointVels = diffkine.jacobiMap(joystick.magCR.phi, tsgPoseTwist.Tsg_pose_twist, diana.joint_states_array, CTRLMODE);
                // 1. 基于当前的diana.joint_states和jointVels一步预测未来的q会不会发生碰撞
                magmed_msgs::RoboJoints pred_robojoints;
                for(int i = 0; i < JOINTNUM; ++i)
                {
                    pred_robojoints.joints.push_back(diana.joint_states_array[i] + jointVels(i) / CTRLFREQ);
                    // replacing by current joint_states
                    // pred_robojoints.joints.push_back(diana.joint_states_array[i]);
                }
                // selfCollisionCheck
                magmed_msgs::SelfCollisionCheck sCC;
                sCC.request.joints = pred_robojoints;
                bool flag = selfcollision_client.call(sCC);
                if (flag)
                {
                    if(sCC.response.checkResult == true) // 1.1.1 如果会碰撞，则检测其他的控制模式
                    // if(true)
                    {
                        jointVels = VectorXd::Zero(JOINTNUM);
                        if(CTRLMODE == enum_CTRLMODE::DM)
                        {
                            TRANSMETHOD = enum_TRANSMETHOD::OFT;    
                        } // 对DM模式，强制采用OFT运动
                        ROS_INFO("[magmed_controller] self-collision detected, translation mode: %d", TRANSMETHOD);
                        // 为TRANS模式初始化
                        nRet = diffkine.initTransMode(diana.joint_states_array, TRANSMETHOD, CTRLMODE);
                        // nRet = 0;
                        if(nRet != 0)
                        {
                            ROS_ERROR("[magmed_controller] failed to initialize transmode\n");
                        }
                        else
                        {
                            ROS_INFO("[magmed_controller] transmode initialized\n");
                            // 切换到TRANS模式
                            // 切换到TRANS模式
                            CTRLMODE = enum_CTRLMODE::TRANS;
                        }
                    }
                    else // 1.1.2 如果不会碰撞，则继续执行当前的控制模式
                    {
                        // ROS_INFO("[magmed_controller] SelfCollisionCheck Pass!\n");
                        // break;
                    }
                }
                else
                {
                    ROS_ERROR("[magmed_controller] failed to call selfCollisionCheck service\n");
                }

                feeder_msg.data.clear();
                feeder_msg.data.push_back(joystick.feeder.feeder_vel_FB);
                feeder_msg.data.push_back(static_cast<uint32_t>(0));

                break;
            }
        }

        // 发布jointVels

        // 将jointVels缩放至合适的范围
        // VectorXd JOINTMAX = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
        VectorXd JOINTMAX = diffkine.params.jointVelLimits;
        diffkine.scalingJointVels(diana.joint_states_array, jointVels, JOINTMAX);

        for (int i = 0; i < JOINTNUM; ++i)
        {
            // push_back joint_vels
            joint_vels.joints.push_back(jointVels(i));
        }
        break;
    }
    default:
        break;
    }

    // pub vels for diana
    diana_jointVels_pub.publish(joint_vels);

    // pub vels for feeder
    feeder_vel_pub.publish(feeder_msg);

    return 0;
};

double VelCtrlNode::calcTipAngle(const double alpha, const double (&thetaList)[JOINTNUM])
{
    Vector3d proximal_point_vec_sg = tsgPoseTwist.Tsg_pose_twist.segment(2, 3);
    Vector3d tip_point_vec_g = stereoCamera.tip_point_vec - proximal_point_vec_sg;
    Vector2d v2tip_point_vec_g = tip_point_vec_g.segment(1, 2);
    double tipAngle = atan(v2tip_point_vec_g.norm() / 
        (Vector3d::UnitX().transpose() * tip_point_vec_g));
    // convert thetaList from double to VectorXd
    VectorXd thetaListVec = Map<VectorXd>(const_cast<double *>(thetaList), JOINTNUM);
    Matrix4d Tsb = FKinSpace(diffkine.params.M, diffkine.params.Slist, thetaListVec);
    Matrix4d Tsg = MatrixExp6(VecTose3(tsgPoseTwist.Tsg_pose_twist.head(6)));
    Matrix4d Tgb = Adjoint(Tsg) * Tsb;
    std::vector<MatrixXd> Rp = TransToRp(Tgb);
    Vector3d Pgb = Rp[2];
    Vector2d v2Pgb(Pgb[1], Pgb[2]);
    if(v2Pgb.normalized().transpose() * v2tip_point_vec_g >= 0)
    {
        return alpha * tipAngle;
    }
    else
    {
        return -alpha * tipAngle;
    }
    // return tipAngle;
};

void TipAngle::feed(magmed_msgs::TipAngleConstPtr pMsg)
{
    tip_angle = *pMsg;
};

void StereoCamera::feedTip(geometry_msgs::PointConstPtr pMsg)
{
    tip_point = *pMsg;
    tip_point_vec << tip_point.x, tip_point.y, tip_point.z;
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
    dianaTcp.TCP_VEL_GAIN = joystick.POTB / 1000.0 * M_PI;
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
            double joyPhi = atan2(-joystick.nJOY1[0], joystick.nJOY1[1]);
            double joyTheta = sqrt(pow(joystick.nJOY1[0], 2) + pow(joystick.nJOY1[1], 2));

            if(joyPhi > M_PI / 2.0)
            {
                joyPhi -= M_PI;
                joyTheta = -joyTheta;
            }
            else if(joyPhi < -M_PI / 2.0)
            {
                joyPhi += M_PI;
                joyTheta = -joyTheta;
            }
            phirltd.update(joyPhi, 1.0 / CTRLFREQ);
            thetarltd.update(joyTheta, 1.0 / CTRLFREQ);

            magCR.phi[0] = phirltd.getX1();
            magCR.phi[1] = phirltd.getX2();

            magCR.theta[0] = thetarltd.getX1();
            magCR.theta[1] = thetarltd.getX2();

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

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");
    ros::NodeHandle nh("~");

    VelCtrlNode velCtrlNode(nh);
    velCtrlNode.run();

    return 0;
}