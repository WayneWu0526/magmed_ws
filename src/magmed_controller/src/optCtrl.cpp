#include "magmed_controller/optCtrl.h"

void resolveMagtwist(double x, double xmin, double xmax, double &dx);

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
    // std::cout << "theta_r:" << thetaR[0] << "dtheta_r:" << thetaR[1] << std::endl;
    return thetaR[1] + param.fk * (thetaR[0] - thetaL) - hatx1;
}

MagPose optCtrl::controlAllocation(const double (&refTheta)[2], double virtualControlLaw, RowVector4d jacobian, int CTRLMODE)
{
    MagPose magTwist;
    OptCtrlParam::CAParam param = optCtrlParam.caparam;
    switch (CTRLMODE)
    {
        case 0:
        {
            // double k = jacobian[0];
            double k = 1.0;
            // double lambda = 1.0e-2;
            // // std::cout << "jacobian[0]:" << jacobian[0] << std::endl;
            // /* using pesudo-inverse controller */
            // if(abs(jacobian[0]) < lambda){
            //     if(jacobian[0] > 0){
            //         k = lambda;
            //     }
            //     else{
            //         k = -lambda;
            //     }
            // }

            magTwist.psi = 1.0 / k * virtualControlLaw;

            if(magTwist.psi > param.UMAX[0])
            {
                magTwist.psi = param.UMAX[0];
            }
            if(magTwist.psi < -param.UMAX[0])
            {
                magTwist.psi = -param.UMAX[0];
            }
            // std::cout << "jacobian[0]:" << jacobian[0] << std::endl;
            // std::cout << "magTwist.psi:" << magTwist.psi << std::endl;
            magTwist.pos = {0.0, 0.0, 0.0};
            // std::cout << "using pesudo-inverse controller" << std::endl;
            break;
        };
        case 1:
        {
            // double k = jacobian[0];
            double k = 1.0;
            // double lambda = 1.0e-2;
            // // std::cout << "jacobian[0]:" << jacobian[0] << std::endl;
            // /* using pesudo-inverse controller */
            // if(abs(jacobian[0]) < lambda){
            //     if(jacobian[0] > 0){
            //         k = lambda;
            //     }
            //     else{
            //         k = -lambda;
            //     }
            // }

            // magTwist.psi = 1.0 / k * virtualControlLaw;
            magTwist.psi = k * refTheta[1];

            if(magTwist.psi > param.UMAX[0])
            {
                magTwist.psi = param.UMAX[0];
            }
            if(magTwist.psi < -param.UMAX[0])
            {
                magTwist.psi = -param.UMAX[0];
            }
            // std::cout << "jacobian[0]:" << jacobian[0] << std::endl;
            // std::cout << "magTwist.psi:" << magTwist.psi << std::endl;
            magTwist.pos = {0.0, 0.0, 0.0};
            // std::cout << "using pesudo-inverse controller" << std::endl;
            break;
        }
        default :
        {
            USING_NAMESPACE_QPOASES
            double inf = qpOASES::INFTY;
            // extend magPose to magTwist
            Vector4d pose;
            pose << magPose.psi, magPose.pos[0], magPose.pos[1], magPose.pos[2];
            // 把 pose中最大的值限制在param.Umax中
            pose = pose.cwiseMin(param.Umax);
            pose = pose.cwiseMax(param.Umin);
            // std::cout << "twist: " << twist << std::endl;

            // diagonal
            // std::cout << "Umin:" << Umin << std::endl;
            // std::cout << "Umax:" << Umax << std::endl;
            Vector4d Umin = (param.Umin - pose) * param.nf;
            // std::cout << Umin(2) << std::endl;
            Vector4d Umax = (param.Umax - pose) * param.nf;
            // std::cout << "Umin:" << Umin << std::endl;
            // std::cout << "Umax:" << Umax << std::endl;
            Umin = Umin.cwiseMax(-param.UMAX);
            Umax = Umax.cwiseMin(param.UMAX);

            // real_t H[5 * 5] = {1.0, 0.0, 0.0, 0.0, 0.0,
            //                    0.0, 1.0e5, 0.0, 0.0, 0.0,
            //                    0.0, 0.0, 1.0e5, 0.0, 0.0,
            //                    0.0, 0.0, 0.0, 1.0e5, 0.0,
            //                    0.0, 0.0, 0.0, 0.0, 1.0e5};
            real_t H[5 * 5] = {1.0, 0.0, 0.0, 0.0, 0.0,
                            0.0, 1.0e3, 0.0, 0.0, 0.0,
                            0.0, 0.0, 1.0e3, 0.0, 0.0,
                            0.0, 0.0, 0.0, 1.0e3, 0.0,
                            0.0, 0.0, 0.0, 0.0, 1.0};
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
            // real_t lb[5] = {Umin[0], 0.0, Umin[2], 0.0, -inf};
            // real_t ub[5] = {Umax[0], 0.0, Umax[2], 0.0, inf};
            real_t lb[5] = {Umin[0], Umin[1], Umin[2], Umin[3], -inf};
            real_t ub[5] = {Umax[0], Umax[1], Umax[2], Umax[3], inf};
            std::cout << "lb[0]: " << lb[0] << " lb[1]: " << lb[1] << " lb[2]: " << lb[2] << " lb[3]: " << lb[3] << std::endl;
            std::cout << "ub[0]: " << ub[0] << " ub[1]: " << ub[1] << " ub[2]: " << ub[2] << " ub[3]: " << ub[3] << std::endl;

            // std::cout << "Umin" << Umin << std::endl;
            // std::cout << "Umax" << Umax << std::endl;
            real_t lbA[1] = {virtualControlLaw};
            real_t ubA[1] = {virtualControlLaw};

            Options options;
            options.printLevel = PL_LOW;
            QProblem qpCA(5, 1);
            qpCA.setOptions(options);
            int_t nWSR = 10;
            qpCA.init(H, g, A, lb, ub, lbA, ubA, nWSR);
            real_t xOpt[5];
            qpCA.getPrimalSolution(xOpt);
            // std::cout << "optimal control input: " << xOpt[0] << " " << xOpt[1] << " " << xOpt[2] << " " << xOpt[3] << std::endl;

            magTwist.psi = xOpt[0];
            // magTwist.pos = {xOpt[1], xOpt[2], 0.0};
            magTwist.pos = {0.0, 0.0, 0.0};
            
            resolveMagtwist(magPose.psi, param.Umin(0), param.Umax(0), magTwist.psi);
            resolveMagtwist(magPose.pos(0), param.Umin(1), param.Umax(1), magTwist.pos(0));
            resolveMagtwist(magPose.pos(1), param.Umin(2), param.Umax(2), magTwist.pos(1));

            // resolveMagtwist(magPose.pos(2), -param.UMAX[3], param.UMAX[3], magTwist.pos(2));
            
            // std::cout << "optimal control input: " << magTwist.psi << " " << magTwist.pos(0) << " " << magTwist.pos(1) << " " << magTwist.pos(2) << std::endl;
            // MagPose magTwist;
            // std::cout << "using qp controller" << std::endl;
            
        }   
        break;     
    };
    // std::cout << "optimal control input: " << optController << std::endl;
    return magTwist;
}

MagPose optCtrl::generateMagTwist(const double (&refTheta)[2], double thetaL, RowVector4d jacobi)
{
    // calculate jacobi
    // std::cout << "psi:" << optCtrl::magPose.psi << std::endl;optCtrl::magPose.psi
    // std::cout << "pos:" << optCtrl::magPose.pos << std::endl;
    // std::cout << "jacobian: " << jacobi << std::endl;

    // initialize hatx
    static double hatx[2] = {0.0, 0.0}; // 不会被销毁，只会在第一次调用时初始化
    // std::cout << "hatx: " << hatx[0] << " " << hatx[1] << std::endl;

    // calculate virtualControlLaw
    /* using mock value */
    // double virtualControlLaw = FF_controller(refTheta, hatx, thetaL);
    // std::cout << "thetaL_mock: " << thetaL_mock << std::endl;
    double virtualControlLaw = FF_controller(refTheta, hatx, thetaL);

    // calculate control allocation
    // 0: pesudo-inverse controller, 1: qp controller
    MagPose magTwist = controlAllocation(refTheta, virtualControlLaw, jacobi, 1);

    // update hatx
    LESO(refTheta, hatx, virtualControlLaw);

    // trans velocity frame from robot to end-effector
    // WARNING: 平面偏转时使用，否则禁用

    return magTwist;
}


std::tuple<MagPose, double> optCtrl::generateMagTwist_pos(Vector3d refPoint, Vector3d point, Vector3d jacobian){

    // create matrix B with first column in UnitX() and second column in jacobian
    MatrixXd B(3, 2);
    B << Vector3d::UnitX(), jacobian;

    // refPoint and point are both in {s} frame;
    Vector3d xe = refPoint - point;
    std::cout << xe << std::endl;
    double kp = 1.0;
    // calculate u using the Moore-Penrose pseudoinverse
    /* using kinematics */
    MatrixXd pinvB = B.completeOrthogonalDecomposition().pseudoInverse();
    Vector2d u = kp * pinvB * xe;
    /* using PID */
    // Vector2d u = kp * xe;

    MagPose magTwist;
    magTwist.psi = u[1];
    magTwist.pos = {0.0, 0.0, 0.0};
    double nu = u[0];
    return std::make_tuple(magTwist, nu);
}



void resolveMagtwist(double x, double xmin, double xmax, double &dx)
{
    if (x < xmin)
    {
        // dx = 0.001;
        if (dx < 0.0)
        {
            dx = 0.0;
        }
    }
    else if (x > xmax)
    {
        // dx = -0.001;
        if (dx > 0.0)
        {
            dx = 0.0;
        }
    }
};