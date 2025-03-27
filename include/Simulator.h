#pragma once
#include "System.h"
#include <random>
#include <vector>
#include <utility>

class Simulator {
private:
    SystemParams params;
    std::mt19937 rng;  // Генератор случайных чисел

public:
    Simulator(const SystemParams& params);
    
    // Моделирует один эксперимент и возвращает время до отказа
    double simulateSingleRun();
    
    // Проводит несколько экспериментов и возвращает среднее и стандартное отклонение
    std::pair<double, double> simulateMultipleRuns(int numExperiments);
    
    // Моделирует и записывает траекторию состояний системы
    std::vector<std::pair<double, int>> simulateTrajectory();
};
