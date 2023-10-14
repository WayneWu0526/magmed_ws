#ifndef OPTCTRL_H
#define OPTCTRL_H

#include <eigen3/Eigen/Dense>
#include <qpOASES.hpp>
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_msgs/MagPose.h"
#include <iostream>

using namespace Eigen;
using namespace qpOASES;    
using namespace magmed_msgs;

#define FREQ 100

class OptCtrlParam
{
public:
    struct LESOParam
    {
        float beta1 = 1.0;
        float beta2 = 0.01;
        float epsilon = 0.01;
        int nf = FREQ;
    } lesoparam;

    struct FFParam
    {
        float fk = 200.0;
    } ffparam;

    struct CAParam
    {
        // float T = 1.0 / 100.0;
        JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();
        int nf = FREQ;
        Vector4d UMAX = {0.2, 0.05, 0.01, 0.05};
        Vector4d Umin = {-M_PI / 2.0, pr.L, pr.H0 - 20.0e-3, 0.0};
        Vector4d Umax = {M_PI / 2.0, pr.L, pr.H0 + 40.0e-3, 0.0};
    } caparam;

    OptCtrlParam() {};

    OptCtrlParam(int nf) {lesoparam.nf = nf; caparam.nf = nf;};
};

class optCtrl
{
public:
    void LESO(const double (&thetaR)[2], double (&hatx)[2], double vrtlCtrlLaw,
              const OptCtrlParam::LESOParam &param);
    double FF_controller(const double (&thetaR)[2], double (&hatx)[2], double thetaL,
                         const OptCtrlParam::FFParam &param);
    MagPose controlAllocation(MagPose magPose, double virtualControlLaw, RowVector4d jacobian,
                              const OptCtrlParam::CAParam &param);
    void velFrmTrns(MagPose &magTwist);
    MagPose generateMagTwist(const double (&thetaR)[2], MagPose magPose, const double thetaL,
                             const OptCtrlParam &optCtrlParam);
    RowVector4d getJacobi(MagPose magPose);
};

#endif