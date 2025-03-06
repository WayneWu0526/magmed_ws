#include "magmed_controller/velCtrlNode.hpp"

// 自动和手动模式。手动：0，openloop：1, closed loop: 2
void VelCtrlNode::run()
{
    int initFlag = 0;

    ros::AsyncSpinner spinner(4);
    spinner.start();

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

                    double thetaList[JOINTNUM] = {-0.0476854, 1.11003, 0.145956, 0.885227, 0.350754, -2.98834, 0.0221347};
                    diffkine.initTransMode(thetaList, enum_TRANSMETHOD::OFT, enum_CTRLMODE::DM);
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
        // ros::spinOnce();
        // pubJoints();
        loop_rate.sleep();
    }
};

int VelCtrlNode::loadInitPose()
{
    // count init config
    std::vector<double> pose(TCPNUM, 0.0);
    std::vector<Eigen::MatrixXd> Rp = TransToRp(diffkine.params.Tsg);
    // std::vector<Eigen::MatrixXd> Rp = TransToRp(la.Tsg);
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
    magmed_msgs::MagCR magCR_msg; // magCR state

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
        joystick.feeder.feeder_msg.data.clear();
        joystick.feeder.feeder_msg.data.push_back(joystick.feeder.feeder_vel_FB);
        joystick.feeder.feeder_msg.data.push_back(joystick.feeder.feeder_vel_UD);
        break;
    }
    case 1: // open-loop circle
    {
        diffkine.magTwist.psi = joystick.magCR.theta[1];
    }
    case 2: // closed-loop version
    {
        /*********************** 获取磁铁位姿 magPose ******************************/
        diffkine.getMagPose(optctrl.magPose, diana.joint_states_array, tsgPoseTwist.Tsg, CTRLMODE);
        /***********************************************************/
        
        /******************** 计算当前磁铁位姿下的Jacobian matrix ***********************/
        double thetaL_mock;
        RowVector4d jacobian_;
        Vector3d Jx;
        std::tie(thetaL_mock, jacobian_, Jx) = mscrjacobi.get_states(optctrl.magPose.psi, optctrl.magPose.pos);
        // std::cout << "thetaL_mock: " << thetaL_mock << std::endl; 
        /***********************************************************/

        /*********************** 获取磁铁位姿 magCRstate ******************************/
        // 获取magCRstate（包含导丝角度、滚转角和尖端位置），利用的是camera计算出来的实际测量值
        std::array<double, 5> magCRstate = calcMagCRstate(diffkine.magPhi, thetaL_mock, tsgPoseTwist.Tsg, 1);
        // std::cout << "phi:" << magCRstate[0] << " thetaL:" << magCRstate[1] << std::endl;
        /****************************************************************************/

        /******************************** 打包信息 ************************************/
        pushMagCR_msg(diffkine.magPhi, thetaL_mock, tsgPoseTwist.Tsg, magCRstate, magCR_msg);
        /****************************************************************************/

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
                // la.la_msg.data.clear();
                // la.la_msg.data.push_back(0.0);
                // la.la_msg.data.push_back(1);
                break;
            }
            default:
            {
                /******************** user commands ********************/
                /* using joystick control */
                refsmoother.refsmooth(joystick.magCR.phi[0], joystick.magCR.theta[0]);

                /* using generator value */
                // refsmoother.refsmooth(refgenerator.refPhi, refgenerator.refTheta);

                /******************************************************/

                /****************  calculate (u)  **************/
                /* using theta controller */
                diffkine.magTwist = optctrl.generateMagTwist(refsmoother.magCR.theta, magCRstate[1], jacobian_);

                /* using point controller */
                // double TsgXe;
                // Vector4d point_g = {magCRstate[2], magCRstate[3], magCRstate[4], 1.0};
                // Vector4d point_w = TransInv(diffkine.params.Tsw) * Tsg * point_g;
                // Vector3d point = point_w.segment(0, 3);
                // std::tie(diffkine.magTwist, TsgXe) = optctrl.generateMagTwist_pos(refgenerator.refPoint, point, Jx); // magCRstate[0]phi的测量值 refgenerator.refPoint
                
                diffkine.pushMagPoseMsg(optctrl.magPose, diffkine.magTwist); // 打包生成的磁铁数据 
                /************************************************/
                
                /**************** update reference value  *******************/
                /** using generator value **/
                /* using mock value */
                // refgenerator.updateRef_ex(refsmoother.refPhi_, magCR_msg.phi, refsmoother.refTheta_, magCR_msg.theta, magCR_msg.Tsg.position.y);
                
                /* using real value */
                /* using theta controller */
                // refgenerator.updateRef_ex(refsmoother.refPhi_, magCR_msg.phi, refsmoother.refTheta_, magCRstate[1], magCR_msg.Tsg.position.y);
                /* using point controller */
                // refgenerator.updateRef_point(refgenerator.refPhi, diffkine.magPhi, point);
                /***********************************************************/

                // std::cout << "tsgPoseTwist:" << tsgPoseTwist.Vsg << std::endl;

                /********************** update jointVels ************************/
                jointVels = diffkine.jacobiMap(refsmoother.magCR.phi, refsmoother.magCR.theta, tsgPoseTwist.Vsg, diana.joint_states_array, CTRLMODE);
                /***********************************************************/

                // std::cout << "jointVels:" << jointVels << std::endl;

                /********************** self-collision check ************************/
                // 1. 基于当前的diana.joint_states和jointVels一步预测未来的q会不会发生碰撞
                magmed_msgs::RoboJoints pred_robojoints;
                for(int i = 0; i < JOINTNUM; ++i)
                {
                    pred_robojoints.joints.push_back(diana.joint_states_array[i] + jointVels(i) / CTRLFREQ);
                    // replacing by current joint_states
                    // pred_robojoints.joints.push_back(diana.joint_states_array[i]);
                }
                // 关节接近极限时，切换控制模态

                // selfCollisionCheck
                magmed_msgs::SelfCollisionCheck sCC;
                sCC.request.joints = pred_robojoints;
                bool flag = selfcollision_client.call(sCC);
                if (flag)
                {
                    if(sCC.response.checkResult == true) // 1.1.1 如果会碰撞，则检测其他的控制模式 // 如果关节接近极限，则切换控制模态 
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
                        // if(CTRLMODE == enum_CTRLMODE::DM && refsmoother.refPhi_ > 0)
                        // {
                        //     nRet = diffkine.initTransMode(diana.joint_states_array, TRANSMETHOD, CTRLMODE);
                        //     if(nRet != 0)
                        //     {
                        //         ROS_ERROR("[magmed_controller] failed to initialize transmode\n");
                        //     }
                        //     else
                        //     {
                        //         ROS_INFO("[magmed_controller] transmode initialized\n");
                        //         // 切换到TRANS模式
                        //         // 切换到TRANS模式
                        //         CTRLMODE = enum_CTRLMODE::TRANS;
                        //     }
                        //     // TRANSMETHOD = enum_TRANSMETHOD::OFT;    
                        // } // 对DM模式，强制采用OFT运动
                    }
                }
                else
                {
                    ROS_ERROR("[magmed_controller] failed to call selfCollisionCheck service\n");
                }
                /***********************************************************/

                /********************** update ref_state ************************/
                refgenerator.ref_state.data.clear();
                /* using joystick control */
                refgenerator.pushRef_state(joystick.magCR.phi[0], joystick.magCR.theta[0],
                     refsmoother.magCR, joystick.feeder.feeder_vel_FB);
                
                /********************* using generator value ******************/
                /* using theta control */
                // refgenerator.pushRef_state(refgenerator.refPhi, refgenerator.refTheta,
                //      refsmoother.magCR, refgenerator.refTsgX);
                /* using point control */
                // refgenerator.ref_state.data.push_back(refgenerator.refPoint(0));
                // refgenerator.ref_state.data.push_back(refgenerator.refPoint(1));
                // refgenerator.ref_state.data.push_back(refgenerator.refPoint(2));
                // refgenerator.ref_state.data.push_back(refgenerator.refPhi);
                // refgenerator.ref_state.data.push_back(TsgXe);
                /***************************************************************/

                /********************** update feeder ************************/
                /* using feeder to insert MSCR */
                joystick.feeder.feeder_msg.data.clear();
                joystick.feeder.feeder_msg.data.push_back(joystick.feeder.feeder_vel_FB);
                joystick.feeder.feeder_msg.data.push_back(static_cast<uint32_t>(0));

                /* using linear_actuator to insert MSCR */
                // la.la_msg.data.clear();

                /* using joystick to control la */
                // la.la_msg.data.push_back(joystick.joystick.POTA);
                // la.la_msg.data.push_back(joystick.joystick.nJOY3[0]);
                
                /* using generated signals to control la */
                // double TsgXe = refgenerator.refTsgX - magCR_msg.Tsg.position.y;
                // la.push_la_msg(la, TsgXe);
                
                // 这里调整路径
                tsgPoseTwist.Vsg_linear_compute_openloop(joystick.joystick.nJOY3[0], joystick.joystick.nJOY3[1], joystick.joystick.nJOY2[1]);
                /***************************************************************/

                break;
            }
        }

        /**************************  set jointlimits ***************************/
        VectorXd JOINTMAX = diffkine.params.jointVelLimits;
        diffkine.scalingJointVels(diana.joint_states_array, jointVels, JOINTMAX);
        // std::cout << "jointVels:" << jointVels << std::endl;
        /***********************************************************************/
        
        joint_vels.header.stamp = ros::Time::now();
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
    feeder_vel_pub.publish(joystick.feeder.feeder_msg);

    // pub vels for magCR
    magCR_state_pub.publish(magCR_msg);

    // pub ref_state
    ref_state_pub.publish(refgenerator.ref_state);

    // pub vels for linearactuator
    // linear_actuator_pub.publish(la.la_msg);

    // pub magPoseMsg
    magPose_pub.publish(diffkine.magPoseMsg);

    // pub magTgb_pub
    magTgb_pub.publish(diffkine.magPoseStamped);

    // pub control mode
    std_msgs::UInt32 control_mode_msg;
    control_mode_msg.data = CTRLMODE;
    control_mode_pub.publish(control_mode_msg);

    return 0;
};

std::array<double, 5> VelCtrlNode::calcMagCRstate(double phi_mock, double thetaL_mock, Matrix4d Tsg, int IFMOCK)
{
    double tipAngle = 0.0;
    std::array<double, 5> magCRstate = {0.0, 0.0, 0.0, 0.0, 0.0};
    /** using mock control **/
    if(IFMOCK)
    {
        tipAngle = thetaL_mock;
        magCRstate[0] = phi_mock; // using magnet angle
        magCRstate[1] = ((fabs(magCRstate[0]  -  joystick.magCR.phi[0])) <= M_PI / 2.0 ? 1.0 : -1.0) * fabs(tipAngle); // thetaL
    }
    else
    {    
        double alpha = 1.52;
        // Matrix4d Tgc = TransInv(diffkine.params.Tsg) * diffkine.params.Tsc;
        Matrix4d Tgc = TransInv(tsgPoseTwist.Tsg) * diffkine.params.Tsc;
        // std::cout << "Tgc:" << Tgc << std::endl;
        std::vector<MatrixXd> Rp_gc = TransToRp(Tgc);
        // std::cout << "Rp_gc[0]:" << Rp_gc[0] << std::endl;
        Vector3d tip_point_vec_g = Rp_gc[0] * stereoCamera.tip_point_vec + Rp_gc[1];
        // std::cout << "tip_point_vec_g:" << tip_point_vec_g << std::endl;
        Vector2d v2tip_point_vec_g = tip_point_vec_g.segment(1, 2);
        double yL = v2tip_point_vec_g.norm();
        tipAngle = alpha * atan(yL / (Vector3d::UnitX().transpose() * tip_point_vec_g));
        /** using real control **/
        /* using thetaL control */
        magCRstate[0] = atan2(v2tip_point_vec_g(1), v2tip_point_vec_g(0));
        magCRstate[1] = ((fabs(magCRstate[0]  -  joystick.magCR.phi[0])) <= M_PI / 2.0 ? 1.0 : -1.0) * fabs(tipAngle);
        magCRstate[2] = tip_point_vec_g[0];
        magCRstate[3] = tip_point_vec_g[1];
        magCRstate[4] = tip_point_vec_g[2];

        /* using position control */
        // magCRstate[2] = tip_point_vec_g[0];
        // magCRstate[3] = ((fabs(magCRstate[0]  -  diffkine.magPhi)) <= M_PI / 2.0 ? 1.0 : -1.0) * yL;
        // magCRstate[4] = 0.0;
        // std::cout << "tip_point_vec_g:" << tip_point_vec_g << std::endl;
    }

    return magCRstate;
};

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "velocityController");
    ros::NodeHandle nh("~");

    VelCtrlNode velCtrlNode(nh);
    velCtrlNode.run();

    return 0;
}