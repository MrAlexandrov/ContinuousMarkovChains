#pragma once
#include "SystemParams.h"

struct RepairableSystemParams : public SystemParams {
    double lambdaS;

    RepairableSystemParams(int N, int G)
        : SystemParams(N, G)
        , lambdaS((NA + NB - (G % 2)) * (G + (N % 4)))
    {
    }
    
    // Constructor with explicit parameters for both tasks
    RepairableSystemParams(double lambdaA, double lambdaB, int NA, int NB, int RA, int RB, double lambdaS)
        : SystemParams(0, 0) // N and G not used in this case
        , lambdaS(lambdaS)
    {
        this->lambdaA = lambdaA;
        this->lambdaB = lambdaB;
        this->NA = NA;
        this->NB = NB;
        this->RA = RA;
        this->RB = RB;
    }
};
