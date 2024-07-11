#ifndef OPTCTRL_H
#define OPTCTRL_H

// #include <eigen3/Eigen/Dense>
// #include <qpOASES.hpp>
// #include "magmed_msgs/MagPose.h"
// #include <iostream>
#include "magmed_controller/MSCRJacobi.h"
#include "magmed_controller/velCtrlDef.h"

// using namespace Eigen;
// using namespace qpOASES;
// using namespace magmed_msgs;

class OptCtrlParam
{
public:
    struct LESOParam
    {
        float beta1 = 1.0;
        float beta2 = 0.01;
        float epsilon = 0.01;
        int nf = CTRLFREQ;
    } lesoparam;

    struct FFParam
    {
        // float fk = 200.0;
        float fk = 1.1;
    } ffparam;
    struct CAParam
    {
        JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();
        // float T = 1.0 / 100.0;
        int nf = CTRLFREQ;
        Vector4d UMAX = {0.2, 0.05, 0.01, 0.05};
        Vector4d Umin = {-M_PI / 2.0, pr.L, pr.H0 - 20.0e-3, 0.0};
        Vector4d Umax = {M_PI / 2.0, pr.L, pr.H0 + 40.0e-3, 0.0};
    } caparam;

    OptCtrlParam(){};

    OptCtrlParam(int nf)
    {
        lesoparam.nf = nf;
        caparam.nf = nf;
    };
};

class optCtrl
{
public:
    MagPose generateMagTwist(const double (&refTheta)[2], double thetaL);
    double getTheta();
    MagPose magPose;
    optCtrl()
    {
        // initialize
        magPose.psi = 0.0;
        magPose.pos << optCtrlParam.caparam.pr.L,
            optCtrlParam.caparam.pr.H0, 0.0;
    };

private:
    OptCtrlParam optCtrlParam;
    void LESO(const double (&thetaR)[2], double (&hatx)[2], double vrtlCtrlLaw);
    double FF_controller(const double (&thetaR)[2], double (&hatx)[2], double thetaL);
    MagPose controlAllocation(double virtualControlLaw, RowVector4d jacobian);
    RowVector4d getJacobi(MagPose magPose);
};

#endif