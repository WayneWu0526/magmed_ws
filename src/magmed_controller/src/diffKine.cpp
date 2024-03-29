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

VectorXd diffKine::jacobiMap(const double (&refPhi)[2], const double (&thetaList)[JOINTNUM])
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

    // phi_d[0] = refPhi[0];
    // phi_d[1] = refPhi[1];

    // psi_d[0] +=  magTwist.psi / CTRLFREQ;
    // psi_d[1] = magTwist.psi;
    
    // pos_d[0] += magTwist.pos / CTRLFREQ;
    // pos_d[1] = magTwist.pos;

    // use mock input
    VectorXd v_sg = VectorXd::Zero(6); // 机器人坐标系的twist
    phi_d[0] = 0.0;
    phi_d[1] += phi_d[0] / CTRLFREQ;
    psi_d[0] = 0.0;
    psi_d[1] += psi_d[0] / CTRLFREQ;
    pos_d[0] = Vector3d::Zero();
    pos_d[1] = Vector3d::Zero();

    // compute desired pose
    MatrixXd T0 = RpToTrans(diffKine::params.Rgb0, diffKine::params.Pgb0);
    MatrixXd T1 = RpToTrans(Rphi(phi_d[0]), Vector3d::Zero());
    MatrixXd T2 = RpToTrans(Rpsi(psi_d[0]), pos_d[0]);
    MatrixXd Tgd = T1 * T0 * T2;
    // 预留接口：change of robot pose
    // diffKine::params.Tsg 与 v_sg 的增量
    MatrixXd Tsd = diffKine::params.Tsg * Tgd;

    // 控制模态切换矩阵T3
    MatrixXd T3 = MatrixXd::Identity(4, 4);
    if(true)
    {
        MatrixXd T3 = MatrixXd::Identity(4, 4);
        Tsd = Tsd * T3;
        Tgd = TransInv(diffKine::params.Tsg) * Tsd;
    }
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
    dPos << refPhi[1], magTwist.psi, magTwist.pos; // For closed-loop control

    VectorXd nu_sd(TCPNUM);
    nu_sd = J * dPos + Adjoint(TransInv((Tgd))) * v_sg;

    DiagonalMatrix<double, JOINTNUM> W;
    W.diagonal() << 100.0, 10.0, 10.0, 10.0, 10.0, 10.0, 1.0;
    MatrixXd pinvJb = W.inverse() * Jb.transpose() * (Jb * W.inverse() * Jb.transpose()).inverse();

    // compute dthetalist
    VectorXd taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    VectorXd dthetalist(JOINTNUM);
    dthetalist = pinvJb * (Adjoint(TransInv(Tsb) * Tsd) * nu_sd + diffKine::piparams.kp_ * taue);

    // saturate the joint velocity
    // Eigen::VectorXd dqmax(7);
    // dqmax << 2.61, 2.618, 2.61, 2.61, 3.14, 3.14, 3.14;
    // dthetalist = dthetalist.cwiseMin(dqmax);
    // dthetalist = dthetalist.cwiseMax(-dqmax);

    return dthetalist;
};


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