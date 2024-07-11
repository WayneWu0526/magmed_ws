#include "magmed_controller/MSCRJacobi.h"

// create a class for a cylindrical magnet
Vector3d Magnet::get_b(const Vector3d ps)
{
    Vector3d p = ps - pa;
    Vector3d hatp = p / p.norm();
    Matrix3d I3 = MatrixXd::Identity(3, 3);
    Vector3d dB = (magpr.k / pow(p.norm(), 3.0)) * (3.0 * (hatp * (hatp.transpose())) - I3) * hatma;
    return (magpr.k / pow(p.norm(), 3.0)) * (3.0 * (hatp * (hatp.transpose())) - I3) * hatma;
}

Matrix3d Magnet::get_gradb(const Vector3d ps)
{
    Vector3d p = ps - pa;
    Vector3d hatp = p / p.norm();
    Matrix3d I3 = MatrixXd::Identity(3, 3);
    Vector3d Zhatma = (I3 - 5.0 * (hatp * hatp.transpose())) * hatma;
    return (3.0 * magpr.k / pow(p.norm(), 4.0)) * (hatp * hatma.transpose() + hatp.dot(hatma) * I3 + Zhatma * hatp.transpose());
}

// return two double values: theta and J

std::pair<double, double> MSCRJacobi::g(const double theta, const Vector3d x, const Vector3d dx, const Vector3d pa, const Vector3d hatma)
{
    Magnet magnet;
    magnet.hatma = hatma;
    magnet.pa = pa;
    Vector3d b = magnet.get_b(x);
    Matrix3d gradb = magnet.get_gradb(x);
    Matrix3d Rz = RotZ(theta);
    Vector3d pRzM = pRotZ(theta) * vecM;
    Vector3d ppRzM = ppRotZ(theta) * vecM;
    double fval = -pr.A / (pr.E * pr.I) * (pRzM.dot(b) + dx.dot(gradb.transpose() * Rz * vecM));
    double As = -pr.A / (pr.E * pr.I) * (ppRzM.dot(b) + 2.0 * (pRzM.dot(gradb * dx)));
    return std::make_pair(fval, As);
    // return 0.0;
};

RowVector3d MSCRJacobi::g_ex(const Vector3d x, const Vector3d dx, const Vector2d theta, const Vector3d hatma, const Vector3d pa)
{
    Magnet magnet;
    magnet.hatma = hatma;
    magnet.pa = pa;
    double k = magnet.magpr.k;
    Matrix3d gradb = magnet.get_gradb(x);
    Matrix3d Rz = RotZ(theta(0));
    Matrix3d pRz = pRotZ(theta(0));
    Vector3d v = Rz * vecM;
    Vector3d p = x - pa;
    Vector3d hatp = p / p.norm();
    Matrix3d I3 = MatrixXd::Identity(3, 3);
    Matrix3d Z = I3 - 5.0 * hatp * hatp.transpose();
    Matrix3d D = v.dot(hatma) * I3 + v * hatma.transpose() + Z * (hatma * v.transpose());
    Matrix3d Db1 = -12.0 * k / (pow(p.norm(), 5.0)) * D * (hatp * hatp.transpose());
    Matrix3d dhatp = (I3 - (hatp * hatp.transpose())) / p.norm();
    Matrix3d Db2 = 3.0 * k / (pow(p.norm(), 4.0)) * (D - 5.0 * hatp.dot(v) * (hatma.transpose() * hatp * I3 + hatp * hatma.transpose())) * dhatp;
    return pr.A / (pr.E * pr.I) * ((pRz * vecM).transpose() * gradb + dx.transpose() * (Db1 + Db2));
};

std::tuple<double, VectorXd, MatrixXd, MatrixXd> MSCRJacobi::fgamma(const Vector3d& xa, const Vector3d& hatma, double gamma) {
    int N = 100;
    double ds = pr.L / N;

    VectorXd theta = VectorXd::Zero(N);
    VectorXd dtheta = VectorXd::Zero(N);
    MatrixXd x = MatrixXd::Zero(3, N);
    MatrixXd dx = MatrixXd::Zero(3, N);

    dtheta(0) = gamma;

    for (int i = 0; i < N - 1; ++i) {
        auto [fval, _] = g(theta(i), x.col(i), dx.col(i), xa, hatma);
        dtheta(i + 1) = dtheta(i) + ds * fval;
        theta(i + 1) = theta(i) + ds * dtheta(i);
        x.col(i + 1) = x.col(i) + ds * Vector3d(cos(theta(i)), sin(theta(i)), 0);
        dx.col(i + 1) = dx.col(i) + ds * Vector3d(-sin(theta(i)), cos(theta(i)), 0);
    }
    double beta = dtheta(N - 1);
    return std::make_tuple(beta, theta, x, dx);
}

std::tuple<VectorXd, double, MatrixXd> MSCRJacobi::newton_method_bvp(const Vector3d pa, const Vector3d hatma, const Vector3d phatma)
{
    int MAX_ITER = 20;
    VectorXd gamma(MAX_ITER);
    VectorXd beta(MAX_ITER);
    int N = 100;
    VectorXd theta = VectorXd::Zero(N);
    MatrixXd x = MatrixXd::Zero(3, N); 
    MatrixXd dx = MatrixXd::Zero(3, N);

    gamma(0) = 0.0;
    beta(0) = std::get<0>(fgamma(pa, hatma, gamma(0)));

    gamma(1) = -beta(0);
    beta(1) = std::get<0>(fgamma(pa, hatma, gamma(1)));

    for(int i = 1; i < MAX_ITER; ++i)
    {
        gamma(i+1) = gamma(i) - beta(i) * (gamma(i) - gamma(i-1)) / (beta(i) - beta(i-1));
        auto [beta_, theta_, x_, dx_] = fgamma(pa, hatma, gamma(i+1));
        beta(i+1) = beta_;
        theta = theta_;
        x = x_;
        dx = dx_;
        // std::cout << beta_ << std::endl;
        if(i == MAX_ITER - 1)
        {
            printf("shooting method failed!\n");
        }
        if(fabs(beta_) < 1e-3)
        {
            break;
        }
    };
    // print theta
    // std::cout << theta << std::endl;
    double ds = pr.L / N;

    MatrixXd u(2, N);
    MatrixXd v(2, N);
    u.setZero();
    v.setZero();
    v(1, 0) = 1;
    for (int i = 0; i < N - 1; ++i) {
        auto [fval, As] = g(theta(i), x.col(i), dx.col(i), pa, hatma);
        // std::cout << As << std::endl;
        auto [Bs, _] = g(theta(i), x.col(i), dx.col(i), pa, phatma);
        // std::cout << Bs << std::endl;
        u.col(i + 1) = u.col(i) + ds * Vector2d(u(1, i), As * u(0, i) + Bs);
        v.col(i + 1) = v.col(i) + ds * Vector2d(v(1, i), As * v(0, i));
    }

    double ga = -u(1, N - 1) / v(1, N - 1);
    // std::cout << ga << std::endl;
    VectorXd J = u.row(0).transpose() + ga * v.row(0).transpose();
    double jacobian = J(N - 1);

    // std::cout << jacobian << std::endl;
    return std::make_tuple(theta, jacobian, x);

};

// double MSCRJacobi::get_theta(double psi, const Vector3d pa)
// {
//     int N = 100;
//     VectorXd Theta = VectorXd::Zero(N);

//     Vector3d hatma = {-cos(psi), sin(psi), 0.0};
//     Vector3d phatma = {sin(psi), cos(psi), 0.0};

//     Theta = std::get<0>(newton_method_bvp(pa, hatma, phatma));
//     // std::cout << Theta(N - 1) << std::endl;
//     return Theta(N - 1);
// };

std::pair<double, RowVector4d> MSCRJacobi::get_states(double psi, const Vector3d pa)
{
    Vector2d theta = Vector2d::Zero(2, 1);
    Vector2d J_psi = Vector2d::Zero(2, 1);
    Eigen::Matrix<double, 2, 3> J_p;
    J_p << 0.0, 0.0, 0.0,
        0.0, 0.0, 0.0;
    Vector3d dx = {0.0, 0.0, 0.0};

    Vector3d hatma = {-cos(psi), sin(psi), 0.0};
    Vector3d phatma = {sin(psi), cos(psi), 0.0};

    // for 3D deflection
    // Vector3d pa_bar(pa[0], 0.0, sqrt((pa[1] * pa[1] + pa[2] * pa[2])));
    auto [theta_, Jpsi_, x_] = newton_method_bvp(pa, hatma, phatma);
    J_psi(0) = Jpsi_;
    // thetaL = theta_(100 - 1);
    // J_psi(0) = std::get<1>(newton_method_bvp(pa, hatma, phatma));
    // printf("J_psi(0): %f\n", J_psi(0));
    for (int i = 0; i < pr.N - 1; i++)
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

        // Vector2d dJpsi = Vector2d(J_psi(1), g(x.col(i), dx, theta, phatma, pa));
        // J_psi += ds * dJpsi;
        Eigen::Matrix<double, 2, 3> dJp;
        dJp.row(0) = J_p.row(1);
        dJp.row(1) = g_ex(x.col(i), dx, theta, hatma, pa);
        J_p += ds * dJp;
        // std::cout << J << std::endl;
        Vector2d dtheta = Vector2d(theta(1), std::get<0>(g(theta(0), x.col(i), dx, pa, hatma)));
        theta += ds * dtheta;

        x.col(static_cast<Eigen::Index>(i) + 1) = x.col(i) + (Vector3d(cos(theta(0)), 0, sin(theta(0)))) * ds;
        dx += (Vector3d(-sin(theta(0)), 0, cos(theta(0)))) * ds;
    }
    RowVector4d jacobian;
    jacobian << J_psi(0), -J_p.row(0);
    return std::make_pair(theta_(100-1), jacobian);
};

// int main()
// {
//     // printf("hello world\n");
//     RowVector4d jacobian;
//     MSCRJacobi mscrjacobi;
//     auto [thetaL_, jacobian_] = mscrjacobi.get_states(M_PI/2.0, Vector3d(mscrjacobi.pr.L, 180e-3, 0.0));
//     // std::cout << jacobian << std::endl;
//     std::cout << "thetaL:" << thetaL_ << std::endl;
//     std::cout << "jacobian:" << jacobian_ << std::endl;
//     return 0;
// };