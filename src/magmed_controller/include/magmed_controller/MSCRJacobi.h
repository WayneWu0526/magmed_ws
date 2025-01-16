#ifndef MSCRJACOBI_H
#define MSCRJACOBI_H

#include "magmed_controller/velCtrlDef.h"
// #include <cmath>
// #include <eigen3/Eigen/Dense>
// #include <algorithm>
// #include <iostream>
// #include <utility>
// #include <tuple>

using namespace Eigen;
// using namespace std;

// JacobiParams
class JacobiParams
{
public:
    struct MSCRProperties
    {
        double E; // 909.0e3; // the Young's modulus of the robot
        double r;
        double W;
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

        // void loadFromYAML(const std::string& filename) {
        //     YAML::Node config = YAML::LoadFile(filename);
        //     const YAML::Node& mscrProperties = config["MSCRProperties"];

        //     E = mscrProperties["E"].as<double>();
        //     r = mscrProperties["r"].as<double>();
        //     I = mscrProperties["I"].as<double>();
        //     A = mscrProperties["A"].as<double>();
        //     L = mscrProperties["L"].as<double>();
        //     H0 = mscrProperties["H0"].as<double>();
        //     normM = mscrProperties["normM"].as<double>();
        //     N = mscrProperties["N"].as<int>();
            
        //     const YAML::Node& hatMNode = mscrProperties["hatM"];
        //     hatM(0) = hatMNode[0].as<double>();
        //     hatM(1) = hatMNode[1].as<double>();
        //     hatM(2) = hatMNode[2].as<double>();
        // }

        MSCRProperties()
        {
            // loadFromYAML("../config/MSCRproperties.yaml");
            /* MSCR parameters 1 */
            E = 30;
            // r = (1.086e-03) / 2.0;
            r = 1.5e-3/2;
            I = M_PI * pow(r, 4.0) / 4.0;
            A = M_PI * r * r;
            // L = 24.0e-3;
            L = 30.0e-3;
            // normM = 10.0e4;
            normM = 1.0;
            hatM = {1.0, 0.0, 0.0};

            /* MSCR parameters 2 */
            // E = 60.0;
            // W = 1.0e-3;
            // I = pow(W, 4) / 12.0;
            // A = W * W;
            // L = 30.0e-3;
            // hatM = {1.0, 0.0, 0.0};
            // normM = 1.0;
            // normM = 10.0e4;
            H0 = 160.0e-3;
            N = 100;
            
        };
        MSCRProperties(double E, double r, double I, double A, double L, Vector3d hatM, double normM) : 
            E(E), r(r), I(I), A(A), L(L), H0(H0), hatM(hatM), normM(normM){};
    }mscrproperties;

    struct MagnetParams
    {
        double k;

        MagnetParams() { k = 3.4286e-05; }; // Magnet D50
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
    // double get_theta();         // get the tip angle of the robot
    struct MSCRstates{
        VectorXd theta;
        RowVector4d jacobian;
        MatrixXd x;
        Vector3d jacobian_x;
    };
    double thetaL = 0.0;
    std::tuple<double, RowVector4d, Vector3d> get_states(double psi, const Vector3d pa); // get the Jacobian of the robot
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

    std::tuple<double, VectorXd, MatrixXd, MatrixXd> fgamma(const Vector3d& xa, const Vector3d& hatma, double gamma);
    std::pair<double, double> g(const double theta, const Vector3d x, const Vector3d dx, const Vector3d pa, const Vector3d hatma);
    RowVector3d g_ex(double theta, const Vector3d x, const Vector3d dx, const Vector3d hatma, const Vector3d pa);
    std::tuple<VectorXd, MatrixXd, RowVector4d, Vector3d> newton_method_bvp(const Vector3d pa, const Vector3d hatma, const Vector3d phatma);
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