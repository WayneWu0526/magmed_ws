#include "draw_circle.hpp"

// 自动和手动模式。手动：0，openloop：1 cls loop2
void VelCtrlNode::run()
{
    int initFlag = 0; // 机械臂是否到达初始位置

    // ros::AsyncSpinner spinner(4);
    // spinner.start();

    ros::Rate loop_rate(CTRLFREQ);
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
    std::vector<Eigen::MatrixXd> Rp = TransToRp(params.Tsg);
    // std::vector<Eigen::MatrixXd> Rp = TransToRp(la.Tsg);
    Vector3d P0 = Rp[0] * params.Pgb0 + Rp[1];
    pose[0] = P0(0);
    pose[1] = P0(1);
    pose[2] = P0(2);
    Matrix3d R0 = Rp[0] * params.Rgb0;
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

int VelCtrlNode::pubVels()
{
    magmed_msgs::RoboJoints joint_vels;

    // 

    // jointvels =  jacobiMap(dtcp)

    VectorXd jointVels = VectorXd::Zero(JOINTNUM);
    jointVels << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01;

    // VectorXd JOINTMAX = params.jointVelLimits;
    // diffkine.scalingJointVels(diana.joint_states_array, jointVels, JOINTMAX);
    // std::cout << "jointVels:" << jointVels << std::endl;

    for (int i = 0; i < JOINTNUM; ++i)
    {
        // push_back joint_vels
        joint_vels.joints.push_back(jointVels(i));
    }
    // break;

    // pub vels for diana
    diana_jointVels_pub.publish(joint_vels);

    return 0;

};

int main(int argc, char *argv[])
{   
    ros::init(argc, argv, "maglocator_velocityController");
    ros::NodeHandle nh("~");

    VelCtrlNode velCtrlNode(nh);
    velCtrlNode.run();

    return 0;
}
