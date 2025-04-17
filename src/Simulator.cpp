#include "Simulator.h"

#include <random>
#include <cmath>
#include <numeric>

Simulator::Simulator(const SystemParams& params)
    : params(params)
{
    std::random_device rd;
    rng = std::mt19937(rd());
}

double Simulator::simulateSingleRun() {
    int A = params.NA + params.RA;
    int B = params.NB + params.RB;
    double t = 0.0;

    std::exponential_distribution<double> expDist(1.0);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    while (A >= 1 && B >= params.NB) {
        double totalRate = A * params.lambdaA + B * params.lambdaB;
        double timeToNext = expDist(rng) / totalRate;
        t += timeToNext;

        double probA = (A * params.lambdaA) / totalRate;

        if (uniformDist(rng) < probA) {
            --A;
        } else {
            --B;
        }
    }

    return t;
}

std::pair<double, double> Simulator::simulateMultipleRuns(int numExperiments) {
    std::vector<double> failureTimes;
    failureTimes.reserve(numExperiments);

    for (int i = 0; i < numExperiments; ++i) {
        failureTimes.push_back(simulateSingleRun());
    }

    double mean = std::accumulate(failureTimes.begin(), failureTimes.end(), 0.0) / numExperiments;

    double variance = 0.0;
    for (double time : failureTimes) {
        variance += (time - mean) * (time - mean);
    }
    variance /= numExperiments;

    return {mean, std::sqrt(variance)};
}

std::vector<std::pair<double, int>> Simulator::simulateTrajectory() {
    std::vector<std::pair<double, int>> trajectory;

    System system(params);
    int A = params.NA + params.RA;
    int B = params.NB + params.RB;
    double t = 0.0;

    trajectory.push_back({t, system.stateToIndex(A, B)});

    std::exponential_distribution<double> expDist(1.0);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    while (A >= 1 && B >= params.NB) {
        double totalRate = A * params.lambdaA + B * params.lambdaB;
        double timeToNext = expDist(rng) / totalRate;
        t += timeToNext;

        if (uniformDist(rng) < (A * params.lambdaA) / totalRate) {
            --A;
        } else {
            --B;
        }

        trajectory.push_back({t, system.stateToIndex(A, B)});
    }

    return trajectory;
}
