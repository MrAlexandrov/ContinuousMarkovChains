#pragma once
#include <Eigen/Dense>

class RungeKuttaSolver {
public:
    static Eigen::MatrixXd solve(
        const Eigen::MatrixXd& A
        , const Eigen::VectorXd& y0
        , double tStart, double tEnd, int steps);

    static double simpsonIntegration(const Eigen::VectorXd& y, const Eigen::VectorXd& x);
};
