#include "System.h"

System::System(const SystemParams& params)
    : params(params)
    , workingA(params.NA + params.RA)
    , workingB(params.NB + params.RB)
{
}

bool System::isOperational() const {
    return workingA >= 1 && workingB >= params.NB;
}

void System::deviceFailure(char deviceType) {
    if (deviceType == 'A' && workingA > 0) {
        --workingA;
    } else if (deviceType == 'B' && workingB > 0) {
        --workingB;
    }
}

std::pair<int, int> System::getCurrentState() const {
    return {workingA, workingB};
}

int System::stateToIndex(int a, int b) const {
    int totalA = params.NA + params.RA + 1;
    int totalB = params.NB + params.RB + 1;
    return (totalA - a - 1) * totalB + (totalB - b - 1);
}

std::pair<int, int> System::indexToState(int index) const {
    int totalB = params.NB + params.RB;
    int a = params.NA + params.RA - (index / (totalB + 1));
    int b = params.NB + params.RB - (index % (totalB + 1));
    return {a, b};
}

int System::getTotalStates() const {
    int totalA = params.NA + params.RA;
    int totalB = params.NB + params.RB;
    return (totalA + 1) * (totalB + 1);
}

const SystemParams& System::getParams() const {
    return params;
}
