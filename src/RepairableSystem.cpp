#include "RepairableSystem.h"

#include <tuple>

RepairableSystem::RepairableSystem(const RepairableSystemParams& params)
    : params(params)
    , workingA(params.NA + params.RA)
    , workingB(params.NB + params.RB)
    , repairingA(0)
    , repairingB(0)
    , repairInProgress(false)
{
}

bool RepairableSystem::isOperational() const {
    return workingA >= 1 && workingB >= params.NB;
}

void RepairableSystem::deviceFailure(char deviceType) {
    if (deviceType == 'A' && workingA > 0) {
        workingA--;
        repairingA++;
    } else if (deviceType == 'B' && workingB > 0) {
        workingB--;
        repairingB++;
    }
}

void RepairableSystem::deviceRepair() {
    char deviceType = getDeviceToRepair();
    if (deviceType == 'A') {
        repairingA--;
        workingA++;
    } else if (deviceType == 'B') {
        repairingB--;
        workingB++;
    }
}

char RepairableSystem::getDeviceToRepair() const {
    if (repairingA == 0 && repairingB == 0) {
        return '0';
    }

    if (repairingA > repairingB) {
        return 'A';
    } else if (repairingB > repairingA) {
        return 'B';
    } else {
        return (params.lambdaA >= params.lambdaB) ? 'A' : 'B';
    }
}

bool RepairableSystem::hasDevicesToRepair() const {
    return repairingA > 0 || repairingB > 0;
}

std::tuple<int, int, int> RepairableSystem::getCurrentState() const {
    int repairStatus = 0;
    if (hasDevicesToRepair()) {
        repairStatus = (getDeviceToRepair() == 'A') ? 1 : 2;
    }
    return {workingA, workingB, repairStatus};
}

int RepairableSystem::stateToIndex(int a, int b, int r) const {
    int totalA = params.NA + params.RA + 1;
    int totalB = params.NB + params.RB + 1;
    int repairStates = 3;

    return (totalA * totalB * r) + (totalB * a) + b;
}

std::tuple<int, int, int> RepairableSystem::indexToState(int index) const {
    int totalA = params.NA + params.RA + 1;
    int totalB = params.NB + params.RB + 1;
    int repairStates = 3;

    int r = index / (totalA * totalB);
    int remainder = index % (totalA * totalB);
    int a = remainder / totalB;
    int b = remainder % totalB;

    return {a, b, r};
}

int RepairableSystem::getTotalStates() const {
    int totalA = params.NA + params.RA + 1;
    int totalB = params.NB + params.RB + 1;
    int repairStates = 3;

    return totalA * totalB * repairStates;
}

int RepairableSystem::stateToGraphIndex(int a, int b) const {
    int totalA = params.NA + params.RA + 1;
    int totalB = params.NB + params.RB + 1;

    return (totalA - a - 1) * totalB + (totalB - b - 1);
}

const RepairableSystemParams& RepairableSystem::getParams() const {
    return params;
}

std::pair<int, int> RepairableSystem::getMaxDevices() const {
    return {params.NA + params.RA, params.NB + params.RB};
}