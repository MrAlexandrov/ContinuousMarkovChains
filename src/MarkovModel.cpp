#include "MarkovModel.h"

MarkovModel::MarkovModel(const SystemParams& params) : params(params) {
    System system(params);
    numStates = system.getTotalStates();

    Q = Eigen::MatrixXd::Zero(numStates, numStates);
    initialState = Eigen::VectorXd::Zero(numStates);
    initialState(0) = 1.0;

    buildTransitionMatrix();
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> MarkovModel::solveKolmogorovEquations(double tMax, int steps) {
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(steps, 0, tMax);

    const int internalSteps = 40;

    Eigen::MatrixXd probabilities(numStates, steps);
    probabilities.col(0) = initialState;

    for (int i = 0; i < steps - 1; ++i) {
        double dt = (times(i + 1) - times(i)) / internalSteps;
        Eigen::VectorXd pi = probabilities.col(i);

        for (int j = 0; j < internalSteps; ++j) {
            Eigen::VectorXd k1 = Q.transpose() * pi;
            Eigen::VectorXd k2 = Q.transpose() * (pi + dt / 2 * k1);
            Eigen::VectorXd k3 = Q.transpose() * (pi + dt / 2 * k2);
            Eigen::VectorXd k4 = Q.transpose() * (pi + dt * k3);

            pi = pi + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);

            for (int k = 0; k < numStates; ++k) {
                if (pi(k) < 0) {
                    pi(k) = 0;
                }
            }

            double sum = pi.sum();
            if (sum > 0) {
                pi /= sum;
            }
        }

        probabilities.col(i + 1) = pi;
    }

    return {times, probabilities};
}

Eigen::VectorXd MarkovModel::getReliabilityFunction(const Eigen::MatrixXd& probabilities) {
    Eigen::VectorXd reliability(probabilities.cols());

    System system(params);
    for (int t = 0; t < probabilities.cols(); ++t) {
        double failureProb = 0.0;

        for (int s = 0; s < numStates; ++s) {
            auto [a, b] = system.indexToState(s);
            if (a < 1 || b < params.NB) {
                failureProb += probabilities(s, t);
            }
        }

        reliability(t) = 1.0 - failureProb;
    }

    return reliability;
}

double MarkovModel::calculateMTTF(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability) {
    double mttf = 0.0;

    for (int i = 0; i < times.size() - 1; ++i) {
        double dt = times(i + 1) - times(i);
        mttf += dt * (reliability(i) + reliability(i + 1)) / 2.0;
    }

    return mttf;
}

void MarkovModel::buildTransitionMatrix() {
    Q = Eigen::MatrixXd::Zero(numStates, numStates);

    System system(params);
    const int totalA = params.NA + params.RA;
    const int totalB = params.NB + params.RB;

    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int currentState = system.stateToIndex(a, b);

            // Only add transitions if the system is operational
            if (a >= 1 && b >= params.NB) {
                // Calculate failure rates based on exactly NA devices of type A and NB devices of type B
                // If we have more than NA devices of type A, only NA devices contribute to failure rate
                // If we have more than NB devices of type B, only NB devices contribute to failure rate
                double rateA = 0.0;
                double rateB = 0.0;
                
                // Only NA devices of type A contribute to failure rate
                if (a <= params.NA) {
                    rateA = a * params.lambdaA;
                } else {
                    rateA = params.NA * params.lambdaA;
                }
                
                // Only NB devices of type B contribute to failure rate
                if (b <= params.NB) {
                    rateB = b * params.lambdaB;
                } else {
                    rateB = params.NB * params.lambdaB;
                }
                
                double totalRate = rateA + rateB;

                if (a > 0 && rateA > 0) {
                    int nextState = system.stateToIndex(a - 1, b);
                    Q(currentState, nextState) = rateA;
                }

                if (b > 0 && rateB > 0) {
                    int nextState = system.stateToIndex(a, b - 1);
                    Q(currentState, nextState) = rateB;
                }

                Q(currentState, currentState) = -totalRate;
            }
            // For non-operational states, no transitions are possible
            // This ensures vertex 17 (and other non-operational states) are unreachable
        }
    }
}
