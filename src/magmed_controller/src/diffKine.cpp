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

VectorXd diffKine::jacobiMap(const double (&phi)[2], MagPose magPos, MagPose magTwist, VectorXd thetalist,
                   const diffKineParam &diffkineparam)
{
    diana7KineSpaceParam params = diffkineparam.params_;
    diffKineParam::PICtrlParam piparams = diffkineparam.pictrlparam_;

    // compute the forward kinematics
    MatrixXd Tsb = FKinSpace(params.M, params.Slist, thetalist);

    // compute the spatial Jacobian
    MatrixXd Js = JacobianSpace(params.Slist, thetalist);

    // compute the body Jacobian
    MatrixXd Jb = Adjoint(TransInv(Tsb)) * Js;

    // desired configuration
    Matrix4d T1 = Matrix4d::Identity();
    T1.block(0, 0, 3, 3) = Rpsi(magPos.psi);

    Matrix4d T2 = Matrix4d::Identity();
    T2.block(0, 0, 3, 3) = params.R0 * Rphi(phi[0]);
    T2.block(0, 3, 3, 1) = Vector3d(magPos.pos[0], magPos.pos[1], magPos.pos[2]);

    Matrix4d Tgd = T1 * T2;
    Matrix4d Tsd = params.Tsg * Tgd;

    // compute JacobiMatrix
    MatrixXd J = MatrixXd::Zero(6, 5);
    VectorXd E1;
    E1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    J.block(0, 0, 6, 1) = Adjoint(TransInv(T2)) * E1;
    J(2, 1) = 1.0;
    J.block(3, 2, 3, 3) = Rpsi(magPos.psi).transpose();

    // desired twist Vd
    VectorXd dPos(5);
    dPos << phi[1], magTwist.psi, magTwist.pos[0], magTwist.pos[1], magTwist.pos[2];
    VectorXd Vd = J * dPos;

    // compute Vb
    static VectorXd TaueInt = VectorXd::Zero(6);
    VectorXd Taue = se3ToVec(MatrixLog6(TransInv(Tsb) * Tsd));
    TaueInt = TaueInt + Taue / piparams.nf_;
    VectorXd Vb = Adjoint(TransInv(Tsb) * Tsd) * Vd + piparams.kp_ * Taue + piparams.ki_ * TaueInt;

    // Inverse velocity kinematics
    VectorXd dthetalist = Jb.completeOrthogonalDecomposition().pseudoInverse() * Vb;

    return dthetalist;
};