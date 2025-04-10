#pragma once

#include "System.h"

#include <Eigen/Dense>
#include <utility>

class MarkovModel {
private:
    Eigen::MatrixXd Q;
    Eigen::VectorXd initialState;
    int numStates;
    SystemParams params;

public:
    MarkovModel(const SystemParams& params);

    void buildTransitionMatrix();

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> solveKolmogorovEquations(double tMax, int steps);

    Eigen::VectorXd getReliabilityFunction(const Eigen::MatrixXd& probabilities);

    double calculateMTTF(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability);

    Eigen::VectorXd getFailureStates() const;

    const Eigen::MatrixXd& getTransitionMatrix() const { return Q; }
};
