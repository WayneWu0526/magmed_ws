#include "magmed_controller/diffKine.h"
// MagPos和MagPose之间的转换。MagPos是NagPose的扩张。

double sgn(double x);

Matrix3d diffKine::Rphi(double phi)
{
    Matrix3d R;
    R << 1.0, 0.0, 0.0,
        0.0, cos(phi), -sin(phi),
        0.0, sin(phi), cos(phi);
    return R;
};

Matrix3d diffKine::Rpsi(double psi)
{
    Matrix3d R;
    R << cos(psi), -sin(psi), 0.0,
        sin(psi), cos(psi), 0.0,
        0.0, 0.0, 1.0;
    return R;
};

// void diffKine::initConfig(const double (&thetaList)[JOINTNUM])
// {
//     VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
//     MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
//     T0 = TransInv(params.Tsg) * Tsb; // init config
//     std::vector<Eigen::MatrixXd> Rp = TransToRp(T0);
//     Rinit = Rp[0] * Rpsi(thetalist(JOINTNUM - 1)).transpose(); // init rotation matrix
// };

// getRealMagPose: Matrix3d Rgb = getRealMagPose(magPose)
void diffKine::getMagPose(MagPose &magPose, const double (&thetaList)[JOINTNUM], Matrix4d Tsg, const int CTRLMODE)
{
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // calculate [Rgb, Pgb] = Tgb, Rgb = Rp[0], Pgb = Rp[1];
    std::vector<Eigen::MatrixXd> Rp_gb;
    switch(CTRLMODE)
    {
        case enum_CTRLMODE::NM:
        {
            Rp_gb = TransToRp(TransInv(Tsg) * Tsb);
            break;
        }
        case enum_CTRLMODE::DM:
        {
            Rp_gb = TransToRp(TransInv(Tsg) * Tsb * RpToTrans(Rphi(M_PI), Vector3d::Zero()));            
            break;
        }
        default:
        {
            Rp_gb = TransToRp(TransInv(Tsg) * Tsb);
            break;
        }
    }
    pushMagPoseStamped(Rp_gb);
    // diffkine.pushMagPoseMsg(optctrl.magPose, diffkine.magTwist);
    // magPose.psi = thetalist(JOINTNUM - 1);
    Matrix3d Rz;
    Rz.col(0) << -1, 0, 0;
    Vector3d z = Rp_gb[0].col(2);
    Rz.col(1) = Vector3d::UnitX().cross(z);
    Rz.col(2) = Rp_gb[0].col(2);

    // std::cout << "Rz: " << Rz << std::endl;
    // std::cout << "Rp_gb[0]: " << Rp_gb[0] << std::endl;

    magPose.psi = Vector3d::UnitZ().transpose() * so3ToVec(MatrixLog3(Rz.transpose() * Rp_gb[0]));
    Matrix3d rotz = Eigen::AngleAxisd(magPose.psi, Eigen::Vector3d::UnitZ()).toRotationMatrix();
    magPhi = Vector3d::UnitX().transpose() * so3ToVec(MatrixLog3(Rp_gb[0] * (diffKine::params.Rgb0 * rotz).transpose()));
    Matrix3d rotx = Eigen::AngleAxisd(magPhi, Eigen::Vector3d::UnitX()).toRotationMatrix();
    magPose.pos = rotx.transpose() * Rp_gb[1];
    /* [warning] */
    magPose.pos[1] = 180.0e-3;

    // std::cout << "magPhi: " << magPhi << std::endl;
    // std::cout << "magPose.psi: " << magPose.psi << std::endl;
    // std::cout << "magPose.pos: " << magPose.pos << std::endl;
};

VectorXd diffKine::jacobiMap_tcp(const double (&tcpVels)[TCPNUM], const double (&thetaList)[JOINTNUM], const unsigned char BANA_MODE)
{
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    // Eigen::VectorXd tcpVels_vector = Eigen::Map<const Eigen::VectorXd>(tcpVels, TCPNUM);

    // compute dthetalist
    VectorXd dthetalist(JOINTNUM);

    /***************regular persudo-inverse jacobian***************/
    // MatrixXd Jbpinv = Jb.completeOrthogonalDecomposition().pseudoInverse();

    /***************weighted persudo-inverse jacobian*************/
    // define diag matrix W = [10,10,10,10,10,10,1];
    DiagonalMatrix<double, JOINTNUM> W;
    W.diagonal() << 50.0, 50.0, 30.0, 30.0, 10.0, 10.0, 1.0;
    // MatrixXd Jbpinv = W.inverse() * Jb.transpose() * (Jb * W.inverse() * Jb.transpose()).inverse();
    // dthetalist = Jbpinv * tcpVels_vector;
    Eigen::VectorXd tcpVels_s = Eigen::VectorXd::Zero(TCPNUM);
    Eigen::VectorXd tcpVels_b = Eigen::VectorXd::Zero(TCPNUM);
    if (BANA_MODE == 0x01) // normal mode
    {
        tcpVels_s << 0.0, 0.0, 0.0, tcpVels[3], tcpVels[4], tcpVels[5];
        tcpVels_b << tcpVels[0], tcpVels[1], tcpVels[2], 0.0, 0.0, 0.0;
        tcpVels_s = tcpVels_s + Adjoint(Tsb) * tcpVels_b;
    }
    else if (BANA_MODE == 0x02) // {s} frame
    {
        tcpVels_s << tcpVels[0], tcpVels[1], tcpVels[2], tcpVels[3], tcpVels[4], tcpVels[5];
    }
    else if (BANA_MODE == 0x03) // {b} frame
    {
        tcpVels_b << tcpVels[0], tcpVels[1], tcpVels[2], tcpVels[3], tcpVels[4], tcpVels[5];
        tcpVels_s = Adjoint(Tsb) * tcpVels_b;
    }
    else if (BANA_MODE != 0x04)
    {
        printf("[magmed_controller] Invalid TOG_MODE: %d\n", BANA_MODE);
    }
    if (BANA_MODE == 0x04) // 矫正回原
    {
        dthetalist = - 0.05 * thetalist;
    }
    else
    {
        MatrixXd Jspinv = W.inverse() * Js.transpose() * (Js * W.inverse() * Js.transpose()).inverse();
        dthetalist = Jspinv * tcpVels_s;    
    }

    return dthetalist;
};

VectorXd diffKine::jacobiMap(const double (&refPhi)[2], const double (&refTheta)[2], const VectorXd &V_sg, const double (&thetaList)[JOINTNUM], const int CTRLMODE)
{
    // compute Jb
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    /******************* use real inputs *******************/
    phi_d[0] = refPhi[0];
    phi_d[1] = refPhi[1];

    /* using feedback */
    // psi_d[0] += magTwist.psi / CTRLFREQ;
    // psi_d[1] = magTwist.psi;

    psi_d[0] = refTheta[0];
    psi_d[1] = refTheta[1];

    pos_d[1] = magTwist.pos;
    pos_d[0] += pos_d[1] / CTRLFREQ; // dpos

    // printf("phi_d[0]: %f, phi_d[1]: %f\n", phi_d[0], phi_d[1]);

    /******************* use mock inputs *******************/
    // VectorXd v_sg = VectorXd::Zero(6); // 机器人坐标系的twist
    // double PHI_D = -M_PI / 2.0;
    // phi_d[1] = PHI_D / 30.0; // dphi
    // phi_d[0] += phi_d[1] / CTRLFREQ; // phi

    // psi_d[1] = 0.0; // dpsi
    // psi_d[0] += psi_d[1] / CTRLFREQ; // psi

    // pos_d[1] = Vector3d::Zero();
    // pos_d[0] += pos_d[1] / CTRLFREQ; // dpos

    // 设置对偶模态切换角度和角速度
    double omega = 0.0;
    switch(CTRLMODE)
    {
        case enum_CTRLMODE::NM:
        {
            omega = 0.0;
            break;
        }
        default:
        {
            omega = M_PI;
            break;
        }
    }

    // compute desired pose
    MatrixXd T0 = RpToTrans(diffKine::params.Rgb0, diffKine::params.Pgb0);
    MatrixXd T1 = RpToTrans(Rphi(phi_d[0]), Vector3d::Zero());
    MatrixXd T2 = RpToTrans(Rpsi(psi_d[0]), diffKine::params.Rgb0.transpose() * pos_d[0]);
    MatrixXd T3 = RpToTrans(Rphi(omega), Vector3d::Zero());
    MatrixXd Tgd = T1 * T0 * T2 * T3;
    // 预留接口：change of robot pose
    // diffKine::params.Tsg 与 v_sg 的增量
    // MatrixXd Tsg = diffKine::params.Tsg;
    VectorXd tau_sg(TCPNUM);
    tau_sg << V_sg[0], V_sg[1], V_sg[2], V_sg[3], V_sg[4], V_sg[5];
    // using real value
    MatrixXd Tsg = MatrixExp6(VecTose3(tau_sg));

    // using mock value
    // static MatrixXd Tsg_mock = diffKine::params.Tsg;
    // VectorXd vsg_mock = VectorXd::Zero(6);
    // vsg_mock(3) = - 0.001 * V_sg[TCPNUM + 3];
    // Tsg_mock *= MatrixExp6(VecTose3(vsg_mock));
    // MatrixXd Tsg = Tsg_mock;

    MatrixXd Tsd = Tsg * Tgd;

    // 控制模态切换矩阵T3
    Eigen::Matrix<double, TCPNUM, INPUTNUM> J;
    // 初始化J，赋0元素
    J.setZero();
    J.block<6, 1>(0, 0) = Adjoint(TransInv(T0 * T2 * T3)) * VectorXd::Unit(6, 0);
    J.block<6, 1>(0, 1) = Adjoint(TransInv(T3)) * VectorXd::Unit(6, 2);
    Eigen::Matrix<double, 6, 3> ZR;
    ZR << Matrix3d::Zero(), Rpsi(psi_d[0]).transpose() * diffKine::params.Rgb0.transpose();
    J.block<6, 3>(0, 2) = Adjoint(TransInv(T3)) * ZR;

    // desired twist Vd
    VectorXd dPos(INPUTNUM);
    dPos << phi_d[1], psi_d[1], pos_d[1]; // For closed-loop control

    VectorXd nu_sd(TCPNUM);
    // 令V_sg的TCPNUM:2*TCPNUM-1的元素赋值给v_sg， as inputs
    VectorXd v_sg(TCPNUM);
    v_sg << V_sg[TCPNUM], V_sg[TCPNUM + 1], V_sg[TCPNUM + 2], V_sg[TCPNUM + 3], V_sg[TCPNUM + 4], V_sg[TCPNUM + 5];
    // v_sg = vsg_mock;
    nu_sd = J * dPos + Adjoint(TransInv((Tgd))) * v_sg; // + VectorXd::Unit(6, 0) * omega_d[1];

    // weighted damped persudo-inverse
    double LAMBDA_MAX = 0.005; // max damping factor
    double EPSILON = 0.005;    // min damping factor
    double lambda = 0.0;
    DiagonalMatrix<double, JOINTNUM> W;
    // W.diagonal() << 10.0, 10.0, 0.1, 10.0, 0.1, 1.0, 1.0; // W越小表示对应关节优先级越高
    W.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
    // W.diagonal() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.0;
    JacobiSVD<MatrixXd> svd(Jb * W);

    // std::cout << "可操作度：" << sqrt((Jb * Jb.transpose()).determinant()) << std::endl;

    VectorXd singular_values = svd.singularValues();
    if (singular_values.minCoeff() < EPSILON)
    {
        lambda = (1-pow(singular_values.minCoeff() / EPSILON, 2))*LAMBDA_MAX;
    }
    // dont need to use damping
    lambda = 0.0;

    MatrixXd pinvJb = W.inverse() * Jb.transpose() * (Jb * W.inverse() * Jb.transpose() + lambda * MatrixXd::Identity(TCPNUM, TCPNUM)).inverse();

    MatrixXd P = MatrixXd::Identity(JOINTNUM, JOINTNUM) - pinvJb * Jb;

    // compute dthetalist
    VectorXd taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    VectorXd dthetalist(JOINTNUM);
    dthetalist = pinvJb * (Adjoint(TransInv(Tsb) * Tsd) * nu_sd + diffKine::piparams.kp_ * taue) - 1.5 * P * thetalist;

    return dthetalist;
};

VectorXd diffKine::ctrlModeTrans(const double (&thetaList)[JOINTNUM], enum_CTRLMODE* CTRLMODE, int TRANSMETHOD)
{

    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;   

    VectorXd dthetalist = VectorXd::Zero(JOINTNUM);
    if(CTRLMODEb4Trans == enum_CTRLMODE::DM)
    {
        TRANSMETHOD = enum_TRANSMETHOD::OFT;
        // TRANSMETHOD = enum_TRANSMETHOD::optOFT;
    }
    switch (TRANSMETHOD)
    {
        case enum_TRANSMETHOD::OCT:
        {
            /* code */
            // Unrecommanded transmethod.
            break;
        }
        case enum_TRANSMETHOD::OFT:
        case enum_TRANSMETHOD::optOFT:
        {
            // create matrix N in 3\times 6 that N = [zeros(3,3), eye(3)];
            MatrixXd N = MatrixXd::Zero(3, 6);
            N.block(0, 3, 3, 3) = Matrix3d::Identity();
            MatrixXd Jt = N * Jb;
            MatrixXd pinvJt = Jt.completeOrthogonalDecomposition().pseudoInverse();
            MatrixXd P = MatrixXd::Identity(JOINTNUM, JOINTNUM) - pinvJt * Jt;
            
            VectorXd K = VectorXd::Zero(JOINTNUM);
            if(TRANSMETHOD == enum_TRANSMETHOD::OFT)
            {
                K = 0.3 * VectorXd::Ones(JOINTNUM);
            }
            else
            {
                // solve K using MPC: unimplemented
            }
            // convert K to a diagonal matrix
            double kp = 2.0;
            dthetalist = kp * P * K.asDiagonal() * (qd_trans - thetalist);
            break;
        }
        default: // default:NT
        {
            // create a vector [1;0;0;0;0;0] using Eigen
            VectorXd e1 = VectorXd::Unit(TCPNUM, 0);
            double Jt = -(TransInv(Tsb) * Tsd_trans * VecTose3(e1)).trace();
            double y = 4 - (TransInv(Tsb) * Tsd_trans).trace();
            float epsilon = 0.05;

            // double domega = -0.5 * (abs(Jt) > epsilon ? sgn(Jt) : Jt / epsilon);
            double dphi = 0;
            if (CTRLMODEb4Trans == enum_CTRLMODE::NM) {
                dphi = -1.0;
            } else {
                dphi = 1.0;
            }
            double GAIN = 0.5; // gain of the controller
            // double domega = -sgn((M_PI / 2.0 - fabs(thetalist[JOINTNUM - 1])) * dphi) * (y > epsilon ? GAIN : y / epsilon);
            printf("Jt: %f\n", Jt);
            double domega = - sgn((M_PI / 2.0 - fabs(psi_d[0])) * dphi) * (y > epsilon ? GAIN : 0.0);
            printf("y: %f, domega: %f, Jt: %f", y, domega, Jt);
            MatrixXd pinvJb = Jb.completeOrthogonalDecomposition().pseudoInverse();
            dthetalist = pinvJb * e1 * domega;
            break;
        }
    };

    // 判断需要被切换到的控制模态
    // if (dthetalist.norm() < 0.0)
    // printf("[magmed_controller] dthetalist.norm(): %f\n", dthetalist.norm());
    if(dthetalist.norm() < 0.05)
    {
        if (CTRLMODEb4Trans == enum_CTRLMODE::NM)
        {
            if (TRANSMETHOD == enum_TRANSMETHOD::NT)
            {
                if (thetaList[6] > 0.0)
                {
                    *CTRLMODE = enum_CTRLMODE::SM;
                }
                else
                {
                    *CTRLMODE = enum_CTRLMODE::DM;
                    TRANSMETHOD = enum_TRANSMETHOD::OFT;
                }
            }
            else if (TRANSMETHOD == enum_TRANSMETHOD::OFT)
            {
                *CTRLMODE = enum_CTRLMODE::DM;
            }
            printf("[magmed_controller] Mode translation finished. Current mode: %d", *CTRLMODE);
        }
        else
        {
            *CTRLMODE = enum_CTRLMODE::NM;
        }
    }

    return dthetalist;
};

// double objectiveFunction(const std::vector<double> &q, std::vector<double> &grad, void *data) {
//     double normQ = 0;
//     for (double qi : q) {
//         normQ += qi * qi;
//     }
//     return 0.5 * normQ;
// }

// double constraintFunction(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data)
// {
//     // convert x to VectorXd
//     VectorXd * fd_ptr = (VectorXd *) my_func_data;
//     VectorXd fd = *fd_ptr;
//     VectorXd q = VectorXd::Zero(JOINTNUM);
//     for (int i = 0; i < JOINTNUM; ++i) {
//         q[i] = x[i];
//     }
//     diana7KineSpaceParam params = diana7KineSpaceParam();
//     MatrixXd Tsb = FKinSpace(params.M, params.Slist, q);
//     VectorXd f = se3ToVec(MatrixLog6(Tsb));
//     if (!grad.empty()) {
//         MatrixXd Js = JacobianSpace(params.Slist, q);
//         MatrixXd Jb = Adjoint(Tsb) * Js;
//         VectorXd Grad = Jb.transpose() * (f - fd);
//         for (int i = 0; i < JOINTNUM; ++i) {
//             grad[i] = Grad(i);
//         }
//     }
//     return 0.5 * (f - fd).squaredNorm();
// }

int diffKine::initTransMode(const double (&thetaList)[JOINTNUM], const int TRANSMETHOD, enum_CTRLMODE CTRLMODE)
{
    CTRLMODEb4Trans = CTRLMODE;
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    std::cout << "thetalist: " << thetalist << std::endl;
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    Tsd_trans = Tsb * RpToTrans(Rphi(M_PI), Vector3d::Zero());
    if((TRANSMETHOD == enum_TRANSMETHOD::OFT) || (TRANSMETHOD == enum_TRANSMETHOD::optOFT)) // compute desired joint angles
    {
        // qd_trans << thetalist[0], thetalist[1], thetalist[2], thetalist[3], 0.0, -thetalist[5], -thetalist[6];
        // if(abs(thetalist[6]) <= M_PI / 2.0){
        qd_trans << thetalist[0], thetalist[1], thetalist[2], thetalist[3], 0.0, -thetalist[5], 0.0;
        // else if(thetalist[6] > M_PI / 2.0){
        //     qd_trans << thetalist[0], thetalist[1], thetalist[2], thetalist[3], 0.0, -thetalist[5], M_PI / 2.0;
        // }
        // else if(thetalist[6] < -M_PI / 2.0){
        //     qd_trans << thetalist[0], thetalist[1], thetalist[2], thetalist[3], 0.0, -thetalist[5], -M_PI / 2.0;
        // }

        int nRet = solveDualModeJointAngles(qd_trans);
        return nRet;
        // qd_trans = ...
    };
    return 0;
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
    // return 0.5 * (se3ToVec(MatrixLog6(Tsb)) - se3ToVec(MatrixLog6(fd))).squaredNorm();
};

int diffKine::solveDualModeJointAngles(VectorXd &q0){
    // nlopt::opt opt(nlopt::LN_PRAXIS, JOINTNUM);
    printf("q0: %f, %f, %f, %f, %f, %f, %f\n", q0[0], q0[1], q0[2], q0[3], q0[4], q0[5], q0[6]);
    // printf Tsd_trans
    // nlopt::opt opt(nlopt::LN_PRAXIS, JOINTNUM);
    // lb[0] = -HUGE_VAL; lb[1] = 0;
    nlopt::opt opt;
    std::vector<double> ub(params.jointUpperLimits);
    std::vector<double> lb(params.jointLowerLimits);
    ub[4] = 0.5;
    lb[4] = -0.5;
    // if(qd_trans[6] == 0.0){
    //     ub[6] = M_PI / 2.0;
    //     lb[6] = -M_PI / 2.0;
    // }

    if(CTRLMODEb4Trans == enum_CTRLMODE::NM) {
        opt = nlopt::opt(nlopt::LN_PRAXIS, JOINTNUM);
        // copy the params.jointLowerLimits to a std::vector lb
        ub[5] = 0.0;
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        // print ub
        // for (int i = 0; i < JOINTNUM; ++i) {
        //     std::cout << ub[i] << std::endl;
        // }
    }
    else{
        // opt = nlopt::opt(nlopt::LN_BOBYQA, JOINTNUM);
        /* 12-21 changes */
        q0[6] = 0.0; // set initial position as 0
        opt = nlopt::opt(nlopt::GN_MLSL, JOINTNUM);
        lb[5] = 0.0;
        // ub[6] = M_PI / 2.0;
        // lb[6] = -M_PI / 2.0;
        opt.set_lower_bounds(lb);
        opt.set_upper_bounds(ub);
        opt.set_ftol_rel(1e-8);
        opt.set_maxeval(1000);
        opt.set_initial_step(1e-2);
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
        /* 12-21 changes */
        // q0[6] = 0.0; // set q6 to 0
        // if (CTRLMODEb4Trans == enum_CTRLMODE::DM) {
        //     q0[6] = -M_PI / 2.0;
        // }
        // if (CTRLMODEb4Trans == enum_CTRLMODE::NM) {
        //     q0[6] = M_PI / 2.0;
        // }
        // else {
        //     q0[6] = -M_PI / 2.0;
        // }
        // std::cout << "[magmed_controller] Target joint angles found with minuum f: " << minf << std::endl;
        std::cout << "found minimum at f(" << q0.transpose() << ") = " << minf << std::endl;
        return 0;
    }
    catch(std::exception &e) {
        std::cout << "[magmed_controller] nlopt failed, trying initial value " << e.what() << std::endl;
        return 1;
    }
}

int diffKine::scalingJointVels(const double (&thetaList)[JOINTNUM], VectorXd &dq, const VectorXd &dqMax)
{
    VectorXd s = (dqMax.array() / dq.array()).abs();

    for (int i = 0; i < s.size(); ++i) {
        if (s(i) > 1) {
            s(i) = 1;
        }
    }

    double min_s = s.minCoeff();

    dq = dq * min_s;
    // set dq that q is out of the limits to 0
    for (int i = 0; i < dq.size(); ++i) {
        if (thetaList[i] + dq[i] / CTRLFREQ <= params.jointLowerLimits[i] || thetaList[i] + dq[i] / CTRLFREQ >= params.jointUpperLimits[i]) {
            dq[i] = 0.0;
        }
    }
    // printf("thetaList[4]: %f, dq[4]: %f\n", thetaList[4] / M_PI * 180.0, dq[4]);
    // printf("[magmed_controller] Joint velocities scaled by %f", min_s);

    return 0;
}

double sgn(double x)
{
    return (x >= 0) - (x < 0);
};