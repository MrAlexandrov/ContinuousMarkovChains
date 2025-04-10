#pragma once
#include "System.h"

#include <random>
#include <vector>
#include <utility>

class Simulator {
private:
    SystemParams params;
    std::mt19937 rng;

public:
    Simulator(const SystemParams& params);

    double simulateSingleRun();

    std::pair<double, double> simulateMultipleRuns(int numExperiments);

    std::vector<std::pair<double, int>> simulateTrajectory();
};
