#pragma once

#include "RepairableSystemParams.h"

#include <utility>

class RepairableSystem {
private:
    RepairableSystemParams params;
    int workingA;
    int workingB;
    int repairingA;
    int repairingB;
    bool repairInProgress;

public:
    RepairableSystem(const RepairableSystemParams& params);

    bool isOperational() const;

    void deviceFailure(char deviceType);

    void deviceRepair();

    char getDeviceToRepair() const;

    bool hasDevicesToRepair() const;

    std::tuple<int, int, int> getCurrentState() const;

    int stateToIndex(int a, int b, int r) const;

    int stateToGraphIndex(int a, int b) const;

    std::tuple<int, int, int> indexToState(int index) const;

    int getTotalStates() const;
    
    int getAggregatedStates() const;

    const RepairableSystemParams& getParams() const;

    std::pair<int, int> getMaxDevices() const;
};
