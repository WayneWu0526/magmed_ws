#ifndef GETJACOBIANOFMCR_H
#define GETJACOBIANOFMCR_H

#include <eigen3/Eigen/Dense>
#include <cmath>
#define M_PI 3.14159265358979323846
#define N 1000 // about 0.01 mm error

using namespace Eigen;

namespace magmed_controller
{
    // create a class for magnetic continuum robot
    class MCR
    {
        // physical properties of the robot
        struct Properties
        {
            double E = 3.0e6; // 909.0e3; // the Young's modulus of the robot
            double r = (1.086e-03) / 2.0;
            double I = M_PI * pow(r, 4.0) / 4.0;
            double A = M_PI * r * r;
            double L = 24.0e-3; // the length of the robot
            double H0 = 183.0e-3;

            // double W = 1.0e-3;
            // double I = pow(W, 4)/12.0;
            // double A = W*W;
            // double L = 10.0*W;
            Vector3d hatM = {1.0, 0.0, 0.0}; // the magnetization direction of the robot
            double normM = 10.0e4;           // 144.0e3; // the norm of the magnetization of the robot
        };

    public:
        Properties pr;                                            // properties of the robot
        double get_theta(double psi, const Vector3d &pa);         // get the tip angle of the robot
        RowVector4d get_jacobian(double psi, const Vector3d &pa); // get the Jacobian of the robot

    private:
        MatrixXd x = MatrixXd::Zero(3, N);  // the position of the robot
        Vector3d vecM = pr.normM * pr.hatM; // the magnetization of the robot
        double ds = pr.L / N;
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
        double g(const Vector3d &x, const Vector3d &dx, const Vector2d &theta, const Vector3d &hatma, const Vector3d &pa);
        RowVector3d g_ex(const Vector3d &x, const Vector3d &dx, const Vector2d &theta, const Vector3d &hatma, const Vector3d &pa);
    };

    class Magnet
    {
    public:
        // coefficients for a cylindrical magnet
        double k = 3.4286e-05;
        // position and orientation of a cylindrical magnet
        Vector3d pa = {0.0, 0.0, 0.0};
        Vector3d hatma{-1.0, 0.0, 0.0};
        // get magnetic field and its gradient
        Vector3d get_b(const Vector3d &ps);
        Matrix3d get_gradb(const Vector3d &ps);
    };
};

#endif