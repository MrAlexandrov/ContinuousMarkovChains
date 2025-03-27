// #include "System.h"

#include "MarkovModel.h"
#include "Simulator.h"
#include "GnuplotPlotter.h"
#include "DotGraphGenerator.h"

#include <iostream>
#include <fstream>

int main() {
    // Инициализация параметров из условия
    int N = 260; // Номер зачетки
    int G = 1;   // Группа

    SystemParams params(N, G);

    std::cout << "Параметры системы:" << "\n";
    std::cout << "λA = " << params.lambdaA << "\n";
    std::cout << "λB = " << params.lambdaB << "\n";
    std::cout << "NA = " << params.NA << "\n";
    std::cout << "NB = " << params.NB << "\n";
    std::cout << "RA = " << params.RA << "\n";
    std::cout << "RB = " << params.RB << "\n";

    // Создаем марковскую модель
    MarkovModel model(params);
    model.buildTransitionMatrix();

    std::ofstream transitionMatrix("transition_matrix.dat");
    transitionMatrix << model.getTransitionMatrix() << "\n";

    System system(params);
    DotGraphGenerator::generateStateGraph(system, "state_graph");
    DotGraphGenerator::generateTransitionGraph(model.getTransitionMatrix(), params, "transition_graph");

    // Решаем уравнения Колмогорова
    auto [times, probabilities] = model.solveKolmogorovEquations(10.0, 100);

    std::cout << "Probabilities:\n";
    for (int i = 0; i < probabilities.rows(); ++i) {
        for (int j = 0; j < probabilities.cols(); ++j) {
            std::cout << probabilities(i, j) << " ";
        } 
        std::cout << "\n";
    }
    std::cout << "\n";

    // Вычисляем функцию надежности
    Eigen::VectorXd reliability = model.getReliabilityFunction(probabilities);

    // Вычисляем MTTF
    double mttf = model.calculateMTTF(times, reliability);
    std::cout << "MTTF (аналитический): " << mttf << "\n";

    // Визуализация результатов
    GnuplotPlotter::plotStatesProbabilities(times, probabilities);
    GnuplotPlotter::plotReliabilityFunction(times, reliability);

    // Имитационное моделирование
    Simulator simulator(params);
    auto [meanTime, stdDev] = simulator.simulateMultipleRuns(100);

    std::cout << "MTTF (имитационный): " << meanTime << "\n";
    std::cout << "Стандартное отклонение: " << stdDev << "\n";

    // Моделирование траекторий
    std::vector<std::vector<std::pair<double, int>>> trajectories;
    trajectories.reserve(10);
    for (int i = 0; i < 10; ++i) {
        trajectories.push_back(simulator.simulateTrajectory());
    }
    GnuplotPlotter::plotTrajectories(trajectories);

    return 0;
}
