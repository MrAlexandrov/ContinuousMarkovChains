#pragma once

struct SystemParams {
    double lambdaA;
    double lambdaB;
    int NA;
    int NB;
    int RA;
    int RB;

    SystemParams(int N, int G)
        : lambdaA(G + (N % 3))
        , lambdaB(G + (N % 5))
        , NA(2 + (G % 2))
        , NB(1 + (N % 2))
        , RA(1 + (G % 2))
        , RB(2 - (G % 2))
    {
    }
};
