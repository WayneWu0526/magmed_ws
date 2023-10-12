#include "diffKine.h"

DsrTwist JacobiMap(MagPos magPos, MagPos magdPos, VectorXd thetalist, const diana7KineSpaceParam& params)
{
    diana7KineSpaceParam(); // 有必要吗？

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
    T2.block(0, 0, 3, 3) = params.R0 * Rphi(magPos.phi);
    T2.block(0, 3, 3, 1) = Vector3d(magPos.P.x, magPos.P.y, magPos.P.z);

    Matrix4d Tgd = T1 * T2;
    Matrix4d Tsd = params.Tsg * Tgd;

    // compute JacobiMatrix
    MatrixXd J = MatrixXd::Zero(6, 5);
    VectorXd E1;
    E1 << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0;
    J.block(0, 0, 6, 1) = Adjoint(TransInv(T2)) * E1;
    J(2, 1) = 1.0;




};

Matrix3d Rphi(double phi)
{
    Matrix3d Rphi;
    Rphi << 1.0, 0.0, 0.0,
            0.0, cos(phi), -sin(phi),
            0.0, sin(phi), cos(phi);
    return Rphi;
};

Matrix3d Rpsi(double psi)
{
    Matrix3d Rpsi;
    Rpsi << cos(psi), -sin(psi), 0.0,
            sin(psi), cos(psi), 0.0,
            0.0, 0.0, 1.0;
    return Rpsi;
};