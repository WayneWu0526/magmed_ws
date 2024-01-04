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

void diffKine::initConfig(const double (&thetaList)[JOINTNUM])
{
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    T0 = TransInv(params.Tsg) * Tsb; // init config
    std::vector<Eigen::MatrixXd> Rp = TransToRp(T0);
    Rinit = Rp[0] * Rpsi(thetalist(JOINTNUM - 1)).transpose(); // init rotation matrix
};

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

Eigen::MatrixXd diffKine::calcJacobi()
{
    Eigen::Matrix<double, TCPNUM, TCPNUM - 1> J;
    Matrix3d Sez = VecToso3(Vector3d(0.0, 0.0, 1.0));
    std::vector<Eigen::MatrixXd> Rp = TransToRp(TR);
    Vector3d R_star = -Rp[0] * Sez * Rp[0] * params.Rgb0.transpose() * Rp[1];
    VectorXd E1(TCPNUM);
    E1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    J.block(0, 0, TCPNUM, 1) = Adjoint(TransInv(T0 * TR)) * E1;
    J(2, 1) = 1.0;
    J.block(3, 1, 3, 1) = R_star;
    J.block(3, 2, 3, 3) = Rp[0].transpose() * params.Rgb0.transpose();
    return J;
};

VectorXd diffKine::jacobiMap(magmed_msgs::RefPhi const refPhi, const double (&thetaList)[JOINTNUM])
{

    /* compute desired twist Vd */
    // desired configuration
    Matrix4d TL = RpToTrans(Rphi(refPhi.phi), Vector3d(0.0, 0.0, 0.0));

    // calculate jacobi matrix
    MatrixXd J = calcJacobi();

    // desired twist Vd
    VectorXd dPos(INPUTNUM);
    dPos << refPhi.dphi, magTwist.psi, magTwist.pos;
    VectorXd Vd = J * dPos;

    /* compute Vb */
    // compute the forward kinematics
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    Matrix4d Tgd = TL * T0 * TR;
    Matrix4d Tsd = params.Tsg * Tgd;

    // compute the twist error
    static VectorXd TaueInt = VectorXd::Zero(6);
    VectorXd Taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    TaueInt = TaueInt + Taue / piparams.nf_;

    // compute the body twist
    VectorXd Vb = Adjoint(TransInv(Tsb) * Tsd) * Vd + piparams.kp_ * Taue + piparams.ki_ * TaueInt;
    /* Inverse velocity kinematics */

    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;
    MatrixXd Jbpinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
    VectorXd dthetalist = Jbpinv * Vb;

    /* Update TR */
    // discrete manifold integration
    VectorXd Nu_u(TCPNUM);
    Nu_u << 0.0, 0.0, magTwist.psi,
        TR.block(0, 0, 3, 3).transpose() * params.Rgb0.transpose() * magTwist.pos;
    TR = TR * MatrixExp6(VecTose3(Nu_u / piparams.nf_));

    return dthetalist;
};

VectorXd diffKine::jacobiMap_dlt(magmed_msgs::RefPhi const refPhi, const double (&thetaList)[JOINTNUM])
{

    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);
    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);
    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    // calculate [Rgb, Pgb] = Tgb, Rgb = Rp[0], Pgb = Rp[1];
    std::vector<Eigen::MatrixXd> Rp = TransToRp(TransInv(params.Tsg) * Tsb);
    Matrix3d Rgb = Rp[0];
    Vector3d Pgb = Rp[1];

    // calculate Rotphi
    Vector3d xghat = Vector3d::UnitX();
    Vector3d zghat = Vector3d::UnitZ();
    // compute cross product of xghat and Pgb
    Vector3d t = VecToso3(xghat) * Pgb;
    Vector3d v = VecToso3(zghat) * t / t.norm();
    Matrix3d Rotphi = Matrix3d::Identity() + VecToso3(v) +
                      VecToso3(v) * VecToso3(v) / (1.0 + zghat.dot(t) / t.norm());

    Eigen::Matrix<double, TCPNUM, INPUTNUM> J;
    // 初始化J，赋0元素
    J.setZero();
    J.block(0, 0, 3, 1) = Rgb.transpose() * xghat;
    J.block(0, 1, 3, 1) = zghat;
    J.block(3, 0, 3, 1) = Rgb.transpose() * VecToso3(xghat) * Pgb;
    J.block(3, 2, 3, 3) = Rgb.transpose() * Rotphi;

    // desired twist Vd
    VectorXd dPos(INPUTNUM);
    dPos << refPhi.dphi, magTwist.psi, magTwist.pos; // For closed-loop control

    // compute dthetalist
    VectorXd dthetalist(JOINTNUM);
    MatrixXd Jbpinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
    dthetalist = Jbpinv * J * dPos;

    return dthetalist;
};
