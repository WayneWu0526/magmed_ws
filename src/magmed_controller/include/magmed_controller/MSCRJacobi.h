#ifndef MSCRJACOBI_H
#define MSCRJACOBI_H

// #include <eigen3/Eigen/Dense>
// #include <iostream>
// #include <cmath>
#include "magmed_controller/velCtrlDef.h"

// using namespace Eigen;

// JacobiParams
class JacobiParams
{
public:
    struct MSCRProperties
    {
        double E; // 909.0e3; // the Young's modulus of the robot
        double r;
        double I;
        double A;
        double L; // the length of the robot
        double H0;
        int N; // number of segments

        // double W = 1.0e-3;
        // double I = pow(W, 4)/12.0;
        // double A = W*W;
        // double L = 10.0*W;
        Vector3d hatM; // the magnetization direction of the robot
        double normM;  // 144.0e3; // the norm of the magnetization of the robot

        MSCRProperties()
        {
            E = 3.0e6;
            r = (1.086e-03) / 2.0;
            I = M_PI * pow(r, 4.0) / 4.0;
            A = M_PI * r * r;
            L = 24.0e-3;
            H0 = 200.0e-3;
            // H0 = 250.0e-3;
            hatM = {1.0, 0.0, 0.0};
            normM = 10.0e4;
            N = 1000;
        };
        MSCRProperties(double E, double r, double I, double A, double L, double H0, Vector3d hatM, double normM) : 
            E(E), r(r), I(I), A(A), L(L), H0(H0), hatM(hatM), normM(normM){};
    }mscrproperties;

    struct MagnetParams
    {
        double k;

        MagnetParams() { k = 3.4286e-05; };
        MagnetParams(double k) : k(k){};
    }magnetparams;

    JacobiParams() { MSCRProperties(); MagnetParams(); };
    JacobiParams(MSCRProperties mscrproperties, MagnetParams magnetparams) : mscrproperties(mscrproperties), magnetparams(magnetparams){};
};

class MSCRJacobi
{
public:
    // physical properties of the robot
    // properties of the robot
    JacobiParams::MSCRProperties pr = JacobiParams::MSCRProperties();              
    double get_theta(double psi, const Vector3d pa);         // get the tip angle of the robot
    RowVector4d get_jacobian(double psi, const Vector3d pa); // get the Jacobian of the robot
    Matrix3d RotZ(double theta)
    {
        AngleAxisd vecZ(theta, Vector3d(0, 0, 1));
        return vecZ.matrix();
    }
    Matrix3d pRotZ(double theta)
    {
        Matrix3d pRZ;
        pRZ << -sin(theta), -cos(theta), 0.0,
            cos(theta), -sin(theta), 0.0,
            0.0, 0.0, 0.0;
        return pRZ;
    }
    Matrix3d ppRotZ(double theta)
    {
        Matrix3d ppRZ;
        ppRZ << -cos(theta), sin(theta), 0.0,
            -sin(theta), -cos(theta), 0.0,
            0.0, 0.0, 0.0;
        return ppRZ;
    }
    MSCRJacobi() {};

private:
    MatrixXd x = MatrixXd::Zero(3, pr.N);  // the position of the robot
    Vector3d vecM = pr.normM * pr.hatM; // the magnetization of the robot
    double ds = pr.L / pr.N;
    Matrix3d RotY(double theta)
    {
        AngleAxisd vecY(-theta, Vector3d(0, 1, 0));
        return vecY.matrix();
    }
    Matrix3d RotX(double phi)
    {
        AngleAxisd vecX(phi, Vector3d(1, 0, 0));
        return vecX.matrix();
    }
    Matrix3d pRotY(double theta)
    {
        Matrix3d pRY;
        pRY << -sin(theta), 0.0, cos(theta),
            0.0, 0.0, 0.0,
            -cos(theta), 0.0, -sin(theta);
        return pRY;
    }
    struct MSCRState 
    {
        double gamma;
        double beta;
        MatrixXd theta;
        MatrixXd x;
        MatrixXd dx;
    }mscrstate;

    struct MSCRResult
    {
        double thetaL;
        double jacobian;
    }mscrresult;

    void fgamma(const Vector3d& xa, const Vector3d& hatma, double gamma, MSCRState& state);
    double g(const Vector3d x, const Vector3d dx, const Vector2d theta, const Vector3d hatma, const Vector3d pa);
    RowVector3d g_ex(const Vector3d x, const Vector3d dx, const Vector2d theta, const Vector3d hatma, const Vector3d pa);
};

class Magnet
{
    public:
        JacobiParams::MagnetParams magpr = JacobiParams::MagnetParams();
        // coefficients for a cylindrical magnet
        // position and orientation of a cylindrical magnet
        Vector3d pa = {0.0, 0.0, 0.0};
        Vector3d hatma = {-1.0, 0.0, 0.0};
        // get magnetic field and its gradient
        Vector3d get_b(const Vector3d ps);
        Matrix3d get_gradb(const Vector3d ps);
        Magnet(){};
};

#endif // MSCRJACOBI_H