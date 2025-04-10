#include "RungeKuttaSolver.h"

Eigen::MatrixXd RungeKuttaSolver::solve(
    const Eigen::MatrixXd& A
    , const Eigen::VectorXd& y0
    , double tStart, double tEnd, int steps
) {
    double h = (tEnd - tStart) / (steps - 1);
    Eigen::MatrixXd y(y0.size(), steps);
    y.col(0) = y0;

    for (int i = 0; i < steps - 1; ++i) {
        Eigen::VectorXd k1 = A.transpose() * y.col(i);
        Eigen::VectorXd k2 = A.transpose() * (y.col(i) + h/2 * k1);
        Eigen::VectorXd k3 = A.transpose() * (y.col(i) + h/2 * k2);
        Eigen::VectorXd k4 = A.transpose() * (y.col(i) + h * k3);

        y.col(i+1) = y.col(i) + h/6 * (k1 + 2*k2 + 2*k3 + k4);
    }

    return y;
}

double RungeKuttaSolver::simpsonIntegration(const Eigen::VectorXd& y, const Eigen::VectorXd& x) {
    int n = y.size();
    if (n < 2) return 0.0;

    double sum = 0.0;
    for (int i = 0; i < n - 1; ++i) {
        sum += (x(i+1) - x(i)) * (y(i) + y(i+1)) / 2.0;
    }

    return sum;
}
