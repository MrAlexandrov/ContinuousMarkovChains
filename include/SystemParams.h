#pragma once

struct SystemParams {
    double lambdaA;  // Интенсивность отказов устройств типа A
    double lambdaB;  // Интенсивность отказов устройств типа B
    int NA;          // Число рабочих устройств A
    int NB;          // Минимальное число устройств B для работы
    int RA;          // Число резервных устройств A
    int RB;          // Число резервных устройств B

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
