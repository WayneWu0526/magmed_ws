#include "magmed_controller/optCtrl.h"

void optCtrl::LESO(const double (&thetaR)[2], double (&hatx)[2], double vrtlCtrlLaw)
{
    OptCtrlParam::LESOParam param = optCtrlParam.lesoparam;
    double error = thetaR[0] - hatx[0];
    hatx[0] += (hatx[0] + param.beta1 / param.epsilon * error + vrtlCtrlLaw) / param.nf;
    hatx[1] += (param.beta2 / (param.epsilon * param.epsilon) * error) / param.nf;
    // std::cout << "estimate perturbation: " << hatx[1] << std::endl;
}

// feedforward_feedback controller:input thetaR, hatx, output virtual control law
double optCtrl::FF_controller(const double (&thetaR)[2], double (&hatx)[2], double thetaL)
{
    OptCtrlParam::FFParam param = optCtrlParam.ffparam;
    double hatx1 = 0.0; 
    // double hatx1 = hatx[1];
    // std::cout << "virtual FF_controller:" << thetaR[1] + param.fk * (thetaR[0] - thetaL) << std::endl;
    std::cout << "theta_r:" << thetaR[0] << "dtheta_r:" << thetaR[1] << std::endl;
    return thetaR[1] + param.fk * (thetaR[0] - thetaL) - hatx1;
}

MagPose optCtrl::controlAllocation(double virtualControlLaw, RowVector4d jacobian)
{
    OptCtrlParam::CAParam param = optCtrlParam.caparam;
    USING_NAMESPACE_QPOASES
    double inf = qpOASES::INFTY;
    // extend magPose to magTwist
    Vector4d twist;
    twist << magPose.psi, magPose.pos[0], magPose.pos[1], magPose.pos[2];
    // std::cout << "twist: " << twist << std::endl;

    // diagonal
    // std::cout << "Umin:" << Umin << std::endl;
    // std::cout << "Umax:" << Umax << std::endl;
    Vector4d Umin = (param.Umin - twist) * param.nf;
    // std::cout << Umin(2) << std::endl;
    Vector4d Umax = (param.Umax - twist) * param.nf;
    // std::cout << "Umin:" << Umin << std::endl;
    // std::cout << "Umax:" << Umax << std::endl;
    Umin = Umin.cwiseMax(-param.UMAX);
    Umax = Umax.cwiseMin(param.UMAX);

    real_t H[5 * 5] = {0.001, 0.0, 0.0, 0.0, 0.0,
                       0.0, 100.0, 0.0, 0.0, 0.0,
                       0.0, 0.0, 100.0, 0.0, 0.0,
                       0.0, 0.0, 0.0, 100.0, 0.0,
                       0.0, 0.0, 0.0, 0.0, 100.0};
    real_t g[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    real_t A[1 * 5] = {jacobian[0], jacobian[1], jacobian[2], jacobian[3], 1.0};
    // real_t lb[5] = {Umin[0], Umin[1], Umin[2], Umin[3], -inf};
    // real_t ub[5] = {Umax[0], Umax[1], Umax[2], Umax[3], inf};
    // if(Umin(2) >= Umax(2)){
    //     Umax(2) = Umin(2);
    // }
    // if(Umax(2) <= Umin(2))
    // {
    //     Umin(2) = Umax(2);
    // }
    real_t lb[5] = {Umin[0], 0.0, Umin[2], 0.0, -inf};
    real_t ub[5] = {Umax[0], 0.0, Umax[2], 0.0, inf};

    // std::cout << "Umin" << Umin << std::endl;
    // std::cout << "Umax" << Umax << std::endl;
    real_t lbA[1] = {virtualControlLaw};
    real_t ubA[1] = {virtualControlLaw};

    Options options;
    options.printLevel = PL_LOW;
    QProblem qpCA(5, 1);
    qpCA.setOptions(options);
    int_t nWSR = 10;
    // qpCA.init(H, g, A, lb, ub, lbA, ubA, nWSR);
    real_t xOpt[5];
    // qpCA.getPrimalSolution(xOpt);

    MagPose magTwist;
    magTwist.psi = xOpt[0];
    magTwist.pos = {xOpt[1], xOpt[2], xOpt[3]};

    /* using pesudo-inverse controller */
    magTwist.psi = 1.0 / jacobian[0] * virtualControlLaw;
    // std::cout << "jacobian[0]:" << jacobian[0] << std::endl;
    std::cout << "magTwist.psi:" << magTwist.psi << std::endl;
    magTwist.pos = {0.0, 0.0, 0.0};
    
    // std::cout << "optimal control input: " << optController << std::endl;
    return magTwist;
}

MagPose optCtrl::generateMagTwist(const double (&refTheta)[2], double thetaL)
{
    MSCRJacobi mscrjacobi;
    // calculate jacobi
    RowVector4d jacobi;
    // std::cout << "psi:" << optCtrl::magPose.psi << std::endl;optCtrl::magPose.psi
    // std::cout << "pos:" << optCtrl::magPose.pos << std::endl;
    auto [thetaL_mock, jacobian_] = mscrjacobi.get_states(optCtrl::magPose.psi, optCtrl::magPose.pos);
    // std::cout << "jacobian: " << jacobi << std::endl;
    jacobi = jacobian_;

    // initialize hatx
    static double hatx[2] = {0.0, 0.0}; // 不会被销毁，只会在第一次调用时初始化
    // std::cout << "hatx: " << hatx[0] << " " << hatx[1] << std::endl;

    // calculate virtualControlLaw
    /* using mock value */
    // double virtualControlLaw = FF_controller(refTheta, hatx, thetaL);
    std::cout << "thetaL: " << thetaL_mock << std::endl;
    double virtualControlLaw = FF_controller(refTheta, hatx, thetaL_mock);

    // calculate control allocation
    MagPose magTwist = controlAllocation(virtualControlLaw, jacobi);

    // update hatx
    LESO(refTheta, hatx, virtualControlLaw);

    // trans velocity frame from robot to end-effector
    // WARNING: 平面偏转时使用，否则禁用

    return magTwist;
}