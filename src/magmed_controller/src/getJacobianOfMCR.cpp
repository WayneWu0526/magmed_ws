#include <iostream>
#include "magmed_controller/getJacobianOfMCR.h"
#include <eigen3/Eigen/Dense>
#include <cmath>

namespace magmed_controller
{
    // create a class for a cylindrical magnet
    class Magnet
    {
        public:
            // coefficients for a cylindrical magnet
            double k = 3.4286e-05;
            // position and orientation of a cylindrical magnet
            Vector3d pa = {0.0, 0.0, 0.0};
            Vector3d hatma{0.0, 0.0, 1.0};
            // get magnetic field and its gradient
            Vector3d get_b(const Vector3d& ps);
            Matrix3d get_gradb(const Vector3d& ps);
    };

    Vector3d Magnet::get_b(const Vector3d& ps)
    {
        Vector3d p = ps - pa;
        Vector3d hatp = p / p.norm();
        Matrix3d I3 = MatrixXd::Identity(3, 3);
        Vector3d dB = (k / pow(p.norm(), 3)) * (3.0 * (hatp * (hatp.transpose())) - I3) * hatma;
        return (k / pow(p.norm(), 3)) * (3.0 * (hatp * (hatp.transpose())) - I3) * hatma;
    }

    Matrix3d Magnet::get_gradb(const Vector3d& ps)
    {
        Vector3d p = ps - pa;
        Vector3d hatp = p / p.norm();
        Matrix3d I3 = MatrixXd::Identity(3, 3);
        Vector3d Zhatma = (MatrixXd::Identity(3, 3) - 5.0 * (hatp * hatp.transpose())) * hatma;
        return (3.0 * k / pow(p.norm(), 4)) * (hatp * hatma.transpose() + hatp.dot(hatma) * I3 + Zhatma * hatp.transpose());
    }

    Vector2d MCR::g(const Vector3d& x, const Vector3d& dx, const Vector2d& theta, const Vector3d& hatma, const Vector3d& pa)
    {
        Magnet magnet;
        magnet.hatma = hatma;
        magnet.pa = pa;
        Vector3d b = magnet.get_b(x);
        Matrix3d gradb = magnet.get_gradb(x);
        Matrix3d Ry = RotY(-theta(0));
        Vector3d pRyM = pRotY(-theta(0)) * vecM;
        return Vector2d(theta(1), -pr.A / (pr.E * pr.I) * (pRyM.dot(b) + dx.dot(gradb.transpose() * Ry * vecM)));
    };

    double MCR::get_theta(double psi, const Vector3d& pa)
    {
        Vector2d theta = Vector2d::Zero(2, 1);
        Vector3d dx = { 0, 0, 0 };
        Vector3d pa_bar(pa[0], 0.0, sqrt((pa[1] * pa[1] + pa[2] * pa[2])) );

        Vector3d hatma = { -cos(psi), 0, sin(psi) };
        for (int i = 0; i < N - 1; i++)
        {
            // // 4-order Runge-Kutta method (unfinished)
            // Vector2d k1 = g(x.col(i), dx, theta, hatma, pa_bar);
            // Vector2d k2 = g(x.col(i), dx, theta + ds / 2.0 * k1, hatma, pa_bar);
            // Vector2d k3 = g(x.col(i), dx, theta + ds / 2.0 * k2, hatma, pa_bar);
            // Vector2d k4 = g(x.col(i), dx, theta + ds * k3, hatma, pa_bar);
            // theta += ds / 6.0 * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
            theta += ds * g(x.col(i), dx, theta, hatma, pa_bar);

            x.col(static_cast<Eigen::Index>(i) + 1) = x.col(i) + (Vector3d(cos(theta(0)), 0, sin(theta(0)))) * ds;
            dx += (Vector3d(-sin(theta(0)), 0, cos(theta(0)))) * ds;
        }
        return theta(0);
    };

    double MCR::get_jacobian(double psi, const Vector3d& pa)
    {
        Vector2d theta = Vector2d::Zero(2, 1);
        Vector2d J = Vector2d::Zero(2, 1);
        Vector3d dx = {0.0, 0.0, 0.0};

        Vector3d hatma = { -cos(psi), 0, sin(psi) };
        Vector3d phatma = { sin(psi), 0, cos(psi) };

        // for 3D deflection 
        // Vector3d pa_bar(pa[0], 0.0, sqrt((pa[1] * pa[1] + pa[2] * pa[2])));

        for (int i = 0; i < N - 1; i++)
        {
            // 4-order Runge-Kutta method (unfinished)
            // Vector2d k21 = g(x.col(i), dx, Vector2d(theta(0), J(1)), phatma, pa);

            // Vector2d k22 = g(x.col(i), dx, Vector2d(theta(0), J(1)) + ds / 2.0 * k21, phatma, pa);
            // Vector2d k23 = g(x.col(i), dx, Vector2d(theta(0), J(1)) + ds / 2.0 * k22, phatma, pa);
            // Vector2d k24 = g(x.col(i), dx, Vector2d(theta(0), J(1)) + ds * k23, phatma, pa);

            // J += ds / 6.0 * (k21 + 2 * k22 + 2 * k23 + k24);

            // Vector2d k11 = g(x.col(i) + (Vector3d(cos(theta(0) + ds / 2.0 * k11(0)), 0, sin(theta(0) + ds / 2.0 * k11(0)))) * ds / 2.0,
            //  dx + (Vector3d(-sin(theta(0) + ds / 2.0 * k11(0)), 0, cos(theta(0) + ds / 2.0 * k11(0)))) * ds / 2.0, 
            //  theta + ds / 2.0 * k11, hatma, pa);
            // Vector2d k12 = g(x.col(i), dx, theta + ds / 2.0 * k11, hatma, pa);
            // Vector2d k13 = g(x.col(i), dx, theta + ds / 2.0 * k12, hatma, pa);
            // Vector2d k14 = g(x.col(i), dx, theta + ds * k13, hatma, pa);

            // theta += ds / 6.0 * (k11 + 2 * k12 + 2 * k13 + k14);


            // std::cout << J << std::endl;

            // for 3D deflection
            // J += ds * g(x.col(i), dx, Vector2d(theta(0), J(1)), phatma, pa_bar);
            // theta += ds* g(x.col(i), dx, theta, hatma, pa_bar);

            J += ds * g(x.col(i), dx, Vector2d(theta(0), J(1)), phatma, pa);
            // std::cout << J << std::endl;

            theta += ds* g(x.col(i), dx, theta, hatma, pa);

            x.col(static_cast<Eigen::Index>(i) + 1) = x.col(i) + (Vector3d(cos(theta(0)), 0, sin(theta(0)))) * ds;
            dx += (Vector3d(-sin(theta(0)), 0, cos(theta(0)))) * ds;
        }
        return J(0);
    };
}