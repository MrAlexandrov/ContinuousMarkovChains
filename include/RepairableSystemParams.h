#pragma once
#include "SystemParams.h"

struct RepairableSystemParams : public SystemParams {
    double lambdaS;

    RepairableSystemParams(int N, int G)
        : SystemParams(N, G)
        , lambdaS((NA + NB - (G % 2)) * (G + (N % 4)))
    {
    }
};
