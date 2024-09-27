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

RowVector3d MSCRJacobi::g_ex(double theta, const Vector3d x, const Vector3d dx, const Vector3d hatma, const Vector3d pa)
{
    Magnet magnet;
    magnet.hatma = hatma;
    magnet.pa = pa;
    double k = magnet.magpr.k;
    Matrix3d gradb = magnet.get_gradb(x);
    Matrix3d Rz = RotZ(theta);
    Matrix3d pRz = pRotZ(theta);
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
    double ds = pr.L / pr.N;

    VectorXd theta = VectorXd::Zero(pr.N);
    VectorXd dtheta = VectorXd::Zero(pr.N);
    MatrixXd x = MatrixXd::Zero(3, pr.N);
    MatrixXd dx = MatrixXd::Zero(3, pr.N);
    double fval;
    dtheta(0) = gamma;

    for (int i = 0; i < pr.N - 1; ++i) {
        std::tie(fval, std::ignore) = g(theta(i), x.col(i), dx.col(i), xa, hatma);
        dtheta(i + 1) = dtheta(i) + ds * fval;
        theta(i + 1) = theta(i) + ds * dtheta(i);
        x.col(i + 1) = x.col(i) + ds * Vector3d(cos(theta(i)), sin(theta(i)), 0);
        dx.col(i + 1) = dx.col(i) + ds * Vector3d(-sin(theta(i)), cos(theta(i)), 0);
    }
    double beta = dtheta(pr.N - 1);
    return std::make_tuple(beta, theta, x, dx);
}


// std::tuple<VectorXd, RowVector4d, MatrixXd>
std::tuple<VectorXd, MatrixXd, RowVector4d, Vector3d> MSCRJacobi::newton_method_bvp(const Vector3d pa, const Vector3d hatma, const Vector3d phatma)
{
    int MAX_ITER = 20;
    VectorXd gamma = VectorXd::Zero(MAX_ITER);
    VectorXd beta = VectorXd::Zero(MAX_ITER);
    VectorXd theta = VectorXd::Zero(pr.N);
    MatrixXd x = MatrixXd::Zero(3, pr.N); 
    MatrixXd dx = MatrixXd::Zero(3, pr.N);
    MatrixXd dJp = MatrixXd::Zero(pr.N, 3);
    MatrixXd Jp = MatrixXd::Zero(pr.N, 3);
    Vector3d Jx = Vector3d::Zero();
    RowVector4d jacobian = RowVector4d::Zero();

    gamma(0) = 0.0;
    beta(0) = std::get<0>(fgamma(pa, hatma, gamma(0)));

    gamma(1) = -beta(0);
    beta(1) = std::get<0>(fgamma(pa, hatma, gamma(1)));

    for(int i = 1; i < MAX_ITER - 1; ++i)
    {
        gamma(i+1) = gamma(i) - beta(i) * (gamma(i) - gamma(i-1)) / (beta(i) - beta(i-1));
        std::tie(beta(i+1), theta, x, dx) = fgamma(pa, hatma, gamma(i+1));
        // auto [beta_, theta_, x_, dx_] = fgamma(pa, hatma, gamma(i+1));
        // beta(i+1) = beta_;
        // theta = theta_;
        // x = x_;
        // dx = dx_;
        // std::cout << beta_ << std::endl;
        if(fabs(beta(i+1)) < 1e-3)
        {   
            break;
            // std::cout << jacobian << std::endl;
        }
        else if (i == MAX_ITER - 1)
        {
            
            std::cout << "Newton method did not converge" << std::endl;

            break;
        }
    };

    double ds = pr.L / pr.N;
    MatrixXd u(2, pr.N);
    MatrixXd v(2, pr.N);
    u.setZero();
    v.setZero();
    v(1, 0) = 1;
    double As, Bs;
    for (int i = 0; i < pr.N - 1; ++i) {
        std::tie(std::ignore, As) = g(theta(i), x.col(i), dx.col(i), pa, hatma);
        // std::cout << As << std::endl;
        std::tie(Bs, std::ignore) = g(theta(i), x.col(i), dx.col(i), pa, phatma);
        // std::cout << Bs << std::endl;
        u.col(i + 1) = u.col(i) + ds * Vector2d(u(1, i), As * u(0, i) + Bs);
        v.col(i + 1) = v.col(i) + ds * Vector2d(v(1, i), As * v(0, i));
        
        dJp.row(i + 1) = dJp.row(i) + ds * g_ex(theta(i), x.col(i), dx.col(i), hatma, pa);
        Jp.row(i + 1) = Jp.row(i) + ds * dJp.row(i);
    }

    double ga = -u(1, pr.N - 1) / v(1, pr.N - 1);
    // std::cout << ga << std::endl;
    VectorXd J = u.row(0).transpose() + ga * v.row(0).transpose();

    for (int i = 0; i < pr.N - 1; ++i) {
        Jx += ds * Vector3d(-sin(theta(i)) * J(i), cos(theta(i)) * J(i), 0.0);
    }
    jacobian << J(pr.N - 1), -Jp.row(pr.N - 1);

    // return std::make_tuple(theta, jacobian, x);
    return std::make_tuple(theta, x, jacobian, Jx);
    // print theta
    // std::cout << theta << std::endl;
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

std::tuple<double, RowVector4d, Vector3d> MSCRJacobi::get_states(double psi, const Vector3d pa)
{
    Vector3d hatma = {-cos(psi), sin(psi), 0.0};
    Vector3d phatma = {sin(psi), cos(psi), 0.0};

    // for 3D deflection
    // Vector3d pa_bar(pa[0], 0.0, sqrt((pa[1] * pa[1] + pa[2] * pa[2])));
    VectorXd theta(pr.N);
    MatrixXd x(3, pr.N);
    RowVector4d jacobian = RowVector4d::Zero();
    Vector3d Jx = Vector3d::Zero();
    std::tie(theta, x, jacobian, Jx) = newton_method_bvp(pa, hatma, phatma);
    // thetaL = theta_(100 - 1);
    // J_psi(0) = std::get<1>(newton_method_bvp(pa, hatma, phatma));
    // printf("J_psi(0): %f\n", J_psi(0));
    return std::make_tuple(theta(pr.N-1), jacobian, Jx);
};

// int main()
// {
//     // printf("hello world\n");
//     RowVector4d jacobian;
//     double thetaL;
//     Vector3d Jx;
//     MSCRJacobi mscrjacobi;
//     std::tie(thetaL, jacobian, Jx) = mscrjacobi.get_states(M_PI / 2.0, Vector3d(mscrjacobi.pr.L, 180.0e-3, 0.0));
//     // std::cout << jacobian << std::endl;
//     std::cout << "thetaL:" << thetaL << std::endl;
//     std::cout << "jacobian:" << jacobian << std::endl;
//     std::cout << "Jx:" << Jx << std::endl;
//     return 0;
// };