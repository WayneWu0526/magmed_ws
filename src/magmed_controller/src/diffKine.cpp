#include "magmed_controller/diffKine.h"
// MagPos和MagPose之间的转换。MagPos是NagPose的扩张。

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
void diffKine::getMagPose(MagPose &magPose, const double (&thetaList)[JOINTNUM])
{
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // calculate [Rgb, Pgb] = Tgb, Rgb = Rp[0], Pgb = Rp[1];
    std::vector<Eigen::MatrixXd> Rp = TransToRp(TransInv(params.Tsg) * Tsb);
    magPose.psi = thetalist(JOINTNUM - 1);
    magPose.pos = Rp[0] * Rp[1];
};

VectorXd diffKine::jacobiMap(const double (&refPhi)[2], const double (&thetaList)[JOINTNUM], const int CTRLMODE)
{
    // compute Jb
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    // use real input
    // VectorXd v_sg = VectorXd::Zero(6); // 预留：机器人的twist

    phi_d[0] = refPhi[0];
    phi_d[1] = refPhi[1];
    printf("phi_d[0]: %f, phi_d[1]: %f\n", phi_d[0], phi_d[1]);

    // psi_d[0] +=  magTwist.psi / CTRLFREQ;
    // psi_d[1] = magTwist.psi;

    // pos_d[0] += magTwist.pos / CTRLFREQ;
    // pos_d[1] = magTwist.pos;

    // use mock input
    VectorXd v_sg = VectorXd::Zero(6); // 机器人坐标系的twist
    // double PHI_D = -M_PI / 2.0;
    // phi_d[1] = PHI_D / 30.0; // dphi
    // phi_d[0] += phi_d[1] / CTRLFREQ; // phi

    psi_d[1] = 0.0; // dpsi
    psi_d[0] += psi_d[1] / CTRLFREQ; // psi
    
    pos_d[1] = Vector3d::Zero();
    pos_d[0] += pos_d[1] / CTRLFREQ; // dpos

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
    MatrixXd T2 = RpToTrans(Rpsi(psi_d[0]), pos_d[0]);
    MatrixXd T3 = RpToTrans(Rphi(omega), Vector3d::Zero());
    MatrixXd Tgd = T1 * T0 * T2 * T3;
    // 预留接口：change of robot pose
    // diffKine::params.Tsg 与 v_sg 的增量
    MatrixXd Tsd = diffKine::params.Tsg * Tgd;

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
    nu_sd = J * dPos + Adjoint(TransInv((Tgd))) * v_sg; // + VectorXd::Unit(6, 0) * omega_d[1];

    // weighted damped persudo-inverse
    double LAMBDA_MAX = 0.005; // max damping factor
    double EPSILON = 0.005;    // min damping factor
    double lambda = 0.0;
    DiagonalMatrix<double, JOINTNUM> W;
    // W.diagonal() << 10.0, 10.0, 0.1, 10.0, 0.1, 1.0, 1.0; // W越小表示对应关节优先级越高
    W.diagonal() << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 0.1;
    JacobiSVD<MatrixXd> svd(Jb * W);
    VectorXd singular_values = svd.singularValues();
    if (singular_values.minCoeff() < EPSILON)
    {
        lambda = (1-pow(singular_values.minCoeff() / EPSILON, 2))*LAMBDA_MAX;
    }
    // dont need to use damping
    lambda = 0.0;

    MatrixXd pinvJb = W.inverse() * Jb.transpose() * (Jb * W.inverse() * Jb.transpose() + lambda * MatrixXd::Identity(TCPNUM, TCPNUM)).inverse();

    // compute dthetalist
    VectorXd taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    VectorXd dthetalist(JOINTNUM);
    dthetalist = pinvJb * (Adjoint(TransInv(Tsb) * Tsd) * nu_sd + diffKine::piparams.kp_ * taue);

    return dthetalist;
};

VectorXd diffKine::ctrlModeTrans(const double (&thetaList)[JOINTNUM], int* CTRLMODE, const int TRANSMETHOD)
{

    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;   

    VectorXd dthetalist = VectorXd::Zero(JOINTNUM);
    switch (TRANSMETHOD)
    {
        case enum_TRANSMETHOD::OCT:
        {
            /* code */
            break;
        }
        case enum_TRANSMETHOD::OFT:
        {
            /* code */
            break;
        }
        case enum_TRANSMETHOD::optOFT:
        {
            /* code */
            break;
        }
        default: // default:NT
        {
            
            break;
        }
    };

    // 判断需要被切换到的控制模态
    if (dthetalist.norm() < 0.01)
    {
        if (*CTRLMODE == enum_CTRLMODE::NM)
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
                }
            }
            else
            {
                *CTRLMODE = enum_CTRLMODE::DM;
            }
        }
        else
        {
            *CTRLMODE = enum_CTRLMODE::NM;
        }
    }

    return dthetalist;
};

// void diffKine::smoothTraj(Vector2d &Omega, const Vector2d &OMEGA, double T)
// {
//     // smooth the trajectory using 3-times differentiable function
//     static double t = 0.0;
//     if (t < T)
//     {
//         Omega[0] = OMEGA[0] + (3.0 * pow(t, 2) / pow(T, 2) - 2.0 * pow(t, 3) / pow(T, 3)) * (OMEGA[1] - OMEGA[0]);
//         Omega[1] = (6.0 * t / pow(T, 2) - 6.0 * pow(t, 2) / pow(T, 3)) * (OMEGA[1] - OMEGA[0]);
//         t += 1.0 / CTRLFREQ;
//     }
//     else
//     {
//         Omega[0] = OMEGA[1];
//         Omega[1] = 0.0;
//     }
// };

// VectorXd diffKine::jacobiMap(const double (&refPhi)[2], const double (&thetaList)[JOINTNUM])
// {

//     VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
//     MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
//     // compute the spatial Jacobian
//     MatrixXd Js = JacobianSpace(params.Slist, thetalist);
//     // compute the body Jacobian
//     MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

//     // calculate [Rgb, Pgb] = Tgb, Rgb = Rp[0], Pgb = Rp[1];
//     std::vector<Eigen::MatrixXd> Rp = TransToRp(TransInv(params.Tsg) * Tsb);
//     Matrix3d Rgb = Rp[0];
//     Vector3d Pgb = Rp[1];

//     // calculate Rotphi
//     Vector3d xghat = Vector3d::UnitX();
//     Vector3d zghat = Vector3d::UnitZ();
//     // compute cross product of xghat and Pgb
//     Vector3d t = VecToso3(xghat) * Pgb;
//     Vector3d v = VecToso3(zghat) * t / t.norm();
//     Matrix3d Rotphi = Matrix3d::Identity() + VecToso3(v) +
//                       VecToso3(v) * VecToso3(v) / (1.0 + zghat.dot(t) / t.norm());

//     Eigen::Matrix<double, TCPNUM, INPUTNUM> J;
//     // 初始化J，赋0元素
//     J.setZero();
//     J.block(0, 0, 3, 1) = Rgb.transpose() * xghat;
//     J.block(0, 1, 3, 1) = zghat;
//     J.block(3, 0, 3, 1) = Rgb.transpose() * VecToso3(xghat) * Pgb;
//     J.block(3, 2, 3, 3) = Rgb.transpose() * Rotphi;

//     // desired twist Vd
//     VectorXd dPos(INPUTNUM);
//     dPos << refPhi[1], magTwist.psi, magTwist.pos; // For closed-loop control

//     // compute dthetalist
//     VectorXd dthetalist(JOINTNUM);

//     /***************regular persudo-inverse jacobian***************/
//     // MatrixXd Jbpinv = Jb.completeOrthogonalDecomposition().pseudoInverse();

//     /***************weighted persudo-inverse jacobian*************/
//     // define diag matrix W = [10,10,10,10,10,10,1];
//     DiagonalMatrix<double, JOINTNUM> W;
//     W.diagonal() << 10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.0;
//     MatrixXd Jbpinv = W.inverse() * Jb.transpose() * (Jb * W.inverse() * Jb.transpose()).inverse();

//     dthetalist = Jbpinv * J * dPos;

//     return dthetalist;
// };

VectorXd diffKine::jacobiMap_tcp(const double (&tcpVels)[TCPNUM], const double (&thetaList)[JOINTNUM])
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

    Eigen::VectorXd tcpVels_s(TCPNUM);
    tcpVels_s << 0.0, 0.0, 0.0, tcpVels[3], tcpVels[4], tcpVels[5];
    Eigen::VectorXd tcpVels_b(TCPNUM);
    tcpVels_b << tcpVels[0], tcpVels[1], tcpVels[2], 0.0, 0.0, 0.0;
    tcpVels_s = tcpVels_s + Adjoint(Tsb) * tcpVels_b;

    MatrixXd Jspinv = W.inverse() * Js.transpose() * (Js * W.inverse() * Js.transpose()).inverse();
    dthetalist = Jspinv * tcpVels_s;

    return dthetalist;
};