#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

using namespace Eigen;
using namespace std;

struct Parameters {
    double L;
    double A;
    double E;
    double I;
    Vector3d M;
    double k;
};

Vector3d hatma(double psi) {
    return Vector3d(-cos(psi), sin(psi), 0);
}

Vector3d phatma(double psi) {
    return Vector3d(sin(psi), cos(psi), 0);
}

Vector3d dipoleB(const Vector3d &x, double k, const Vector3d &xa, const Vector3d &hatma) {
    Vector3d p = x - xa;
    Vector3d hatp = p.normalized();
    return (k / pow(p.norm(), 3) * (3 * hatp * hatp.transpose() - Matrix3d::Identity())) * hatma;
}

Matrix3d gradB(const Vector3d &x, double k, const Vector3d &xa, const Vector3d &hatma) {
    Vector3d p = x - xa;
    Vector3d hatp = p.normalized();
    Matrix3d Zhatma = (Matrix3d::Identity() - 5 * (hatp * hatp.transpose())) * hatma;
    return (3 * k / pow(p.norm(), 4)) * (hatp * hatma.transpose() + hatp.transpose() * hatma * Matrix3d::Identity() + Zhatma * hatp.transpose());
}

Matrix3d RotZ(double theta) {
    Matrix3d Rz;
    Rz << cos(theta), -sin(theta), 0,
          sin(theta), cos(theta), 0,
          0, 0, 1;
    return Rz;
}

Matrix3d pRotZ(double theta) {
    Matrix3d pRz;
    pRz << -sin(theta), -cos(theta), 0,
           cos(theta), -sin(theta), 0,
           0, 0, 0;
    return pRz;
}

Matrix3d ppRotZ(double theta) {
    Matrix3d ppRz;
    ppRz << -cos(theta), sin(theta), 0,
            -sin(theta), -cos(theta), 0,
            0, 0, 0;
    return ppRz;
}

double fgamma(double gamma, const Parameters &pa, const Vector3d &xa, const Vector3d &hatma, vector<double> &theta, vector<Vector3d> &x, vector<Vector3d> &dx) {
    int N = 100;
    double ds = pa.L / N;
    theta.resize(N, 0);
    vector<double> dtheta(N, 0);
    x.resize(N, Vector3d::Zero());
    dx.resize(N, Vector3d::Zero());
    dtheta[0] = gamma;

    for (int i = 0; i < N - 1; ++i) {
        dtheta[i + 1] = dtheta[i] + ds * g(theta[i], x[i], dx[i], pa, xa, hatma);
        theta[i + 1] = theta[i] + ds * dtheta[i];
        x[i + 1] = x[i] + Vector3d(cos(theta[i]), sin(theta[i]), 0) * ds;
        dx[i + 1] = dx[i] + Vector3d(-sin(theta[i]), cos(theta[i]), 0) * ds;
    }

    return dtheta.back();
}

double g(double theta, const Vector3d &x, const Vector3d &dx, const Parameters &pa, const Vector3d &xa, const Vector3d &hatma) {
    Vector3d B = dipoleB(x, pa.k, xa, hatma);
    Matrix3d gB = gradB(x, pa.k, xa, hatma);
    Matrix3d Rz = RotZ(theta);
    Matrix3d pRz = pRotZ(theta);
    return -pa.A / (pa.E * pa.I) * ((pRz * pa.M).transpose() * B + ((gB.transpose() * Rz * pa.M).transpose() * dx))(0, 0);
}

void Newton_method(const Parameters &pa, const Vector3d &xa, const Vector3d &hatma, const Vector3d &phatma, double &theta, double &jacobian) {
    vector<double> gamma(2, 0), beta(2, 0);
    vector<double> theta_vec;
    vector<Vector3d> x, dx;

    gamma[0] = 0;
    beta[0] = fgamma(gamma[0], pa, xa, hatma, theta_vec, x, dx);

    gamma[1] = -beta[0];
    beta[1] = fgamma(gamma[1], pa, xa, hatma, theta_vec, x, dx);

    int i = 1;
    while (true) {
        gamma.push_back(gamma[i] - beta[i] * (gamma[i] - gamma[i - 1]) / (beta[i] - beta[i - 1]));
        beta.push_back(fgamma(gamma.back(), pa, xa, hatma, theta_vec, x, dx));
        if (i > 20 || abs(beta.back()) < 1e-3) {
            theta = theta_vec.back();
            break;
        }
        ++i;
    }

    // Calculate Jacobian
    int N = 100;
    double ds = pa.L / N;
    MatrixXd u(2, N), v(2, N);
    u.setZero();
    v.setZero();
    v(1, 0) = 1;

    for (int i = 0; i < N - 1; ++i) {
        Vector3d x_i = x[i];
        Vector3d dx_i = dx[i];
        double theta_i = theta_vec[i];

        double As = g(theta_i, x_i, dx_i, pa, xa, hatma);
        double Bs = g(theta_i, x_i, dx_i, pa, xa, phatma);

        u.col(i + 1) = u.col(i) + ds * Vector2d(u(1, i), As * u(0, i) + Bs);
        v.col(i + 1) = v.col(i) + ds * Vector2d(v(1, i), As * v(0, i));
    }

    double ga = -u(1, N - 1) / v(1, N - 1);
    jacobian = (u.row(0) + ga * v.row(0)).tail(1)(0);
}

void jacobianOfpsiv3(const VectorXd &u, const Parameters &pa, double &theta, double &jacobian) {
    Vector3d hatma_vec = hatma(u(2));
    Vector3d phatma_vec = phatma(u(2));
    Vector3d xa = u.head<3>();

    Newton_method(pa, xa, hatma_vec, phatma_vec, theta, jacobian);
}

int main() {
    Parameters pa = {1.0, 1.0, 1.0, 1.0, Vector3d(1, 1, 1), 1.0};
    VectorXd u(3);
    u << 1.0, 0.0, M_PI / 4;

    double theta, jacobian;
    jacobianOfpsiv3(u, pa, theta, jacobian);

    cout << "Theta: " << theta << endl;
    cout << "Jacobian: " << jacobian << endl;

    return 0;
}
