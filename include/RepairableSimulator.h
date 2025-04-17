#pragma once

#include "RepairableSystem.h"

#include <random>
#include <vector>
#include <tuple>

enum class EventType {
    DEVICE_FAILURE_A,
    DEVICE_FAILURE_B,
    DEVICE_REPAIR,
    SIMULATION_END
};

struct Event {
    double time;
    EventType type;

    Event(double t, EventType et) : time(t), type(et) {}

    bool operator>(const Event& other) const {
        return time > other.time;
    }
};

class RepairableSimulator {
private:
    RepairableSystemParams params;
    std::mt19937 rng;

    std::vector<std::tuple<double, int, int, int>> simulateMarkovChain(double simulationTime);

    std::vector<std::tuple<double, int, int, int>> simulateDiscreteEvents(double simulationTime);

public:
    RepairableSimulator(const RepairableSystemParams& params);

    void runMarkovChainSimulation(double simulationTime);

    void runDiscreteEventSimulation(double simulationTime);

    void calculateStatistics(const std::vector<std::tuple<double, int, int, int>>& trajectory);
};
