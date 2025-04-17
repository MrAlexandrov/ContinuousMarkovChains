#pragma once

#include "SystemParams.h"

#include <utility>

class System {
private:
    SystemParams params;
    int workingA;
    int workingB;

public:
    System(const SystemParams& params);

    bool isOperational() const;

    void deviceFailure(char deviceType);

    std::pair<int, int> getCurrentState() const;

    int stateToIndex(int a, int b) const;

    std::pair<int, int> indexToState(int index) const;

    int getTotalStates() const;

    const SystemParams& getParams() const;
};
