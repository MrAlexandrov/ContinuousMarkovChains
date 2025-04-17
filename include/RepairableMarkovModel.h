#pragma once

#include "RepairableSystem.h"

#include <eigen3/Eigen/Dense>
#include <utility>

class RepairableMarkovModel {
private:
    Eigen::MatrixXd Q;
    Eigen::VectorXd initialState;
    int numStates;
    RepairableSystemParams params;

    double getFailureRate(int a, int b, char deviceType) const;
    double getRepairRate(int a, int b, int r) const;

public:
    RepairableMarkovModel(const RepairableSystemParams& params);
    RepairableMarkovModel(double lambdaA, double lambdaB, int NA, int NB, int RA, int RB, double lambdaS);

    void buildTransitionMatrix();

    Eigen::VectorXd solveSteadyStateEquations();

    std::pair<Eigen::VectorXd, Eigen::MatrixXd> solveKolmogorovEquations(double tMax, int steps);

    double estimateTransientTime(const Eigen::VectorXd& steadyStateProbs, double tolerance = 0.01);

    double calculateFailureProbability(const Eigen::VectorXd& stateProbs);

    std::pair<double, double> calculateReadyDevices(const Eigen::VectorXd& stateProbs);

    double calculateRepairUtilization(const Eigen::VectorXd& stateProbs);

    const Eigen::MatrixXd& getTransitionMatrix() const { return Q; }

    Eigen::MatrixXd buildGraphTransitionMatrix() const;

    Eigen::VectorXd getOperationalStates() const;

    Eigen::VectorXd getAggregatedStateProbabilities(const Eigen::VectorXd& stateProbs) const;
};
