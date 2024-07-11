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
                /* 将拖拽结果作为目的地 */
                // if (initFlag == 1)
                // {
                //     // printf the position of Tsg
                //     std::cout << "Tsg before: " << diffkine.params.Tsg << std::endl;
                //     // diana.joint_states_array to VectorXd
                //     VectorXd q0 = Map<VectorXd>(diana.joint_states_array, JOINTNUM);
                //     Matrix4d Tsb_init = FKinSpace(diffkine.params.M, diffkine.params.Slist, q0);
                //     // separate Tsb_init into Rsb_init and psb_init
                //     std::vector<Eigen::MatrixXd> Rp = TransToRp(Tsb_init);
                //     Vector3d Psb_init = Rp[1];
                //     Vector3d Psg_init = Psb_init;
                //     Psg_init[2] -= diffkine.params.Pgb0[1];
                //     // change the position of Tsg to Psg_init, where Tsg is an Eigen:Matrix4d and Psg_init is an Eigen::Vector3d
                //     diffkine.params.Tsg(0, 3) = Psg_init[0];
                //     diffkine.params.Tsg(1, 3) = Psg_init[1];
                //     diffkine.params.Tsg(2, 3) = Psg_init[2];
                //     std::cout << "Tsg after: " << diffkine.params.Tsg << std::endl;
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
    // pub joint vels
    magmed_msgs::RoboJoints joint_vels;
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
        VectorXd jointVels = diffkine.jacobiMap_tcp(joystick.dianaTcp.tcp_vel, diana.joint_states_array, static_cast<unsigned char>(joystick.joystick.BANA));
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
        // update Tsg
        diffkine.params.Tsg *= MatrixExp6(VecTose3(1.0 / CTRLFREQ * la.gframe_twist));
        // std::cout << diffkine.params.Tsg << std::endl;
        // get real mag pose
        diffkine.getMagPose(optctrl.magPose, diana.joint_states_array, CTRLMODE);
        // get mag twist
        double thetaL = calcTipAngle(1.52, diana.joint_states_array, 0); // alpha

        // std::cout << tip_angle << std::endl;
        /* using mock value */
        diffkine.magTwist = optctrl.generateMagTwist(joystick.magCR.theta, thetaL);

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
                jointVels = diffkine.ctrlModeTrans(diana.joint_states_array, &CTRLMODE, TRANSMETHOD);
                break;
            }
            default:
            {
                // ROS_INFO("[magmed_controller] Current CTRLMODE: %d", CTRLMODE);
                // using feedback value: tsgPoseTwist.Tsg_pose_twist
                
                /* using mock Vsg */
                // Vsg_mock.head(6) += 1.0 / CTRLFREQ * Vsg_mock.segment(6, 6);
                // VectorXd Vsg_mock = tsgPoseTwist.Tsg_pose_twist;
                // Vsg_mock[TCPNUM + 3] = joystick.joystick.nJOY1[0];
                
                /* using real Vsg */
                VectorXd Tsg_pose = se3ToVec(MatrixLog6(diffkine.params.Tsg));
                // create a 12 dimension vector that contains the twist of the end-effector
                VectorXd Vsg = VectorXd::Zero(12);
                // set the first 6 elements to the pose of the end-effector
                Vsg.head(6) = Tsg_pose;
                // set the last 6 elements to the twist of the end-effector
                Vsg.tail(6) = la.gframe_twist;

                // std::cout << Vsg << std::endl;

                jointVels = diffkine.jacobiMap(joystick.magCR.phi, Vsg, diana.joint_states_array, CTRLMODE);
                // jointVels = diffkine.jacobiMap(joystick.magCR.phi, Vsg_mock, diana.joint_states_array, CTRLMODE);
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
                    // if(false) // 关闭 self-collision check
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

double VelCtrlNode::calcTipAngle(const double alpha, const double (&thetaList)[JOINTNUM], int DEEPFLAG)
{
    // 计算tip在g坐标系下的位置
    // Vector3d proximal_point_vec_sg = tsgPoseTwist.Tsg_pose_twist.segment(2, 3);
    // Vector3d tip_point_vec_g = stereoCamera.tip_point_vec - proximal_point_vec_sg;
    double tipAngle = 0.0;

    std::vector<MatrixXd> Rp_sg = TransToRp(diffkine.params.Tsg);
    std::vector<MatrixXd> Rp_sc = TransToRp(diffkine.params.Tsc);
    Vector3d tip_point_vec_g = Rp_sg[0].transpose() * (Rp_sc[0] * stereoCamera.tip_point_vec + Rp_sc[1] - Rp_sg[1]);
    Vector2d v2tip_point_vec_g = tip_point_vec_g.segment(1, 2);

    switch(DEEPFLAG)
    {
        case 0:
        {
            double yL = v2tip_point_vec_g.norm();
            tipAngle = 70.0531 * yL;
            break;
        }
        case 1:
        {

            // 计算角度
            tipAngle = alpha * atan(v2tip_point_vec_g.norm() / 
                (Vector3d::UnitX().transpose() * tip_point_vec_g));
            // convert thetaList from double to VectorXd
            // VectorXd thetaListVec = Map<VectorXd>(const_cast<double *>(thetaList), JOINTNUM);
            // Matrix4d Tsb = FKinSpace(diffkine.params.M, diffkine.params.Slist, thetaListVec);
            // // Matrix4d Tsg = MatrixExp6(VecTose3(tsgPoseTwist.Tsg_pose_twist.head(6)));
            // // std::cout << "Tsg:" << diffkine.params.Tsg << std::endl;
            // // std::cout << "Tsb:" << Tsb << std::endl;
            // Matrix4d Tgb = TransInv(diffkine.params.Tsg) * Tsb;
            // std::vector<MatrixXd> Rp = TransToRp(Tgb);
            // Vector3d Pgb = Rp[1];
            // Vector2d v2Pgb(Pgb(1), Pgb(2));
        }
        default:
        {
            break;
        }
    }
    Vector2d v2Pgb(cos(diffkine.magPhi), sin(diffkine.magPhi));
    // std::cout << "v2Pgb:" << v2Pgb << std::endl;
    if(v2Pgb.normalized().transpose() * v2tip_point_vec_g >= 0)
    {
        return tipAngle;
    }
    else
    {
        return - tipAngle;
    }
};

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");
    ros::NodeHandle nh("~");

    VelCtrlNode velCtrlNode(nh);
    velCtrlNode.run();

    return 0;
}