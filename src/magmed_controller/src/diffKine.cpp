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

// getMagPose: Matrix3d Rgb = getMagPose(magPose)
// return the rotation matrix of the TCP if needed
Matrix3d diffKine::getMagPose(MagPose &magPose, const double (&thetaList)[JOINTNUM],
                          const diffKineParam &diffkineparam)
{
    diana7KineSpaceParam params = diffkineparam.params_;
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);

    magPose.psi = thetalist(JOINTNUM - 1);
    std::vector<Eigen::MatrixXd> Rp = TransToRp(TransInv(params.Tsg) * Tsb);
    magPose.pos = Rp[1];
    return Rp[0];
};

VectorXd diffKine::jacobiMap(const double (&phi)[2], MagPose magPose, MagPose magTwist, const double (&thetaList)[JOINTNUM],
                             const diffKineParam &diffkineparam)
{
    diana7KineSpaceParam params = diffkineparam.params_;
    diffKineParam::PICtrlParam piparams = diffkineparam.pictrlparam_;
    VectorXd thetalist = Map<const VectorXd>(thetaList, JOINTNUM);

    // compute the forward kinematics
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);

    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);

    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    // desired configuration
    Matrix4d T1 = RpToTrans(Rphi(phi[0]), Vector3d(0.0, 0.0, 0.0));
    Matrix4d T2 = RpToTrans(params.Rgb0 * Rpsi(magPose.psi), Rphi(phi[0]).transpose() * magPose.pos);
    Matrix4d Tgd = T1 * T2;
    Matrix4d Tsd = params.Tsg * Tgd;

    // compute JacobiMatrix
    MatrixXd J = MatrixXd::Zero(TCPNUM, INPUTNUM);
    VectorXd E1(TCPNUM);
    E1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    J.block(0, 0, TCPNUM, 1) = Adjoint(TransInv(T2)) * E1;
    J(2, 1) = 1.0;
    J.block(3, 2, 3, 3) = Rpsi(magPose.psi).transpose();

    // desired twist Vd
    VectorXd dPos(INPUTNUM);
    dPos << phi[1], magTwist.psi, magTwist.pos[0], magTwist.pos[1], magTwist.pos[2];
    VectorXd Vd = J * dPos;

    // compute Vb
    static VectorXd TaueInt = VectorXd::Zero(6);
    VectorXd Taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    TaueInt = TaueInt + Taue / piparams.nf_;
    VectorXd Vb = Adjoint(TransInv(Tsb) * Tsd) * Vd + piparams.kp_ * Taue + piparams.ki_ * TaueInt;

    // Inverse velocity kinematics
    MatrixXd Jbpinv = Jb.completeOrthogonalDecomposition().pseudoInverse();
    VectorXd dthetalist = Jbpinv * Vb;

    return dthetalist;
};