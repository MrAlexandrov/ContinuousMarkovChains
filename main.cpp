#include "MarkovModel.h"
#include "Simulator.h"
#include "GnuplotPlotter.h"
#include "DotGraphGenerator.h"
#include "RepairableSystem.h"
#include "RepairableMarkovModel.h"
#include "RepairableSimulator.h"

#include <exception>
#include <iostream>
#include <fstream>
#include <string>

void saveMatrix(const Eigen::MatrixXd& matrix, const std::string& filename) {
    std::ofstream transitionMatrix(filename);
    transitionMatrix << matrix << "\n";
    transitionMatrix.close();
}

void runTask1(int N, int G) {
    SystemParams params(N, G);

    std::cout << "Параметры системы:" << "\n";
    std::cout << "lambda_A = " << params.lambdaA << "\n";
    std::cout << "lambda_B = " << params.lambdaB << "\n";
    std::cout << "N_A = " << params.NA << "\n";
    std::cout << "N_B = " << params.NB << "\n";
    std::cout << "R_A = " << params.RA << "\n";
    std::cout << "R_B = " << params.RB << "\n";

    MarkovModel model(params);
    model.buildTransitionMatrix();

    saveMatrix(model.getTransitionMatrix(), "transition_matrix_task1.dat");

    System system(params);
    DotGraphGenerator::generateStateGraph(system, "state_graph_task1");
    DotGraphGenerator::generateTransitionGraph(model.getTransitionMatrix(), params, "transition_graph_task1");
    std::cout << "Граф состояний сохранен в файлах state_graph_task1.dot и state_graph_task1.png" << std::endl;
    std::cout << "Граф переходов сохранен в файлах transition_graph_task1.dot и transition_graph_task1.png" << std::endl;

    auto [times, probabilities] = model.solveKolmogorovEquations(2.0, 100);

    Eigen::VectorXd reliability = model.getReliabilityFunction(probabilities);

    double mttf = model.calculateMTTF(times, reliability);
    std::cout << "MTTF (аналитический): " << mttf << "\n";

    GnuplotPlotter::plotStatesProbabilities(times, probabilities, "states_probabilities_task1");
    GnuplotPlotter::plotReliabilityFunction(times, reliability, "reliability_function_task1");
    std::cout << "График вероятностей состояний сохранен в файл states_probabilities_task1.png" << std::endl;
    std::cout << "График функции надежности сохранен в файл reliability_function_task1.png" << std::endl;

    Simulator simulator(params);
    auto [meanTime, stdDev] = simulator.simulateMultipleRuns(100);

    std::cout << "MTTF (имитационный): " << meanTime << "\n";
    std::cout << "Стандартное отклонение: " << stdDev << "\n";

    std::vector<std::vector<std::pair<double, int>>> trajectories;
    trajectories.reserve(10);
    for (int i = 0; i < 10; ++i) {
        trajectories.push_back(simulator.simulateTrajectory());
    }
    GnuplotPlotter::plotTrajectories(trajectories, "state_trajectories_task1");
    std::cout << "График траекторий сохранен в файл state_trajectories_task1.png" << std::endl;
}

void runTask2(int N, int G) {
    RepairableSystemParams params(N, G);

    std::cout << "Параметры системы:" << "\n";
    std::cout << "lambda_A = " << params.lambdaA << "\n";
    std::cout << "lambda_B = " << params.lambdaB << "\n";
    std::cout << "lambda_S = " << params.lambdaS << "\n";
    std::cout << "NA = " << params.NA << "\n";
    std::cout << "NB = " << params.NB << "\n";
    std::cout << "RA = " << params.RA << "\n";
    std::cout << "RB = " << params.RB << "\n";

    // Создаем модель для построения матрицы переходов
    RepairableMarkovModel model(params);
    model.buildTransitionMatrix();

    // Сохраняем полную матрицу переходов
    saveMatrix(model.getTransitionMatrix(), "transition_matrix_task2.dat");
    
    // Создаем и сохраняем матрицу переходов в формате графа состояний
    Eigen::MatrixXd graphQ = model.buildGraphTransitionMatrix();
    saveMatrix(graphQ, "graph_transition_matrix_task2.dat");
    
    std::cout << "Матрица переходов сохранена в файлах transition_matrix_task2.dat и graph_transition_matrix_task2.dat" << std::endl;
    
    // Создаем систему только для генерации графа состояний

    RepairableSystem system(params);
    DotGraphGenerator::generateRepairableStateGraph(system, "state_graph_task2");
    // Закомментировано, так как требует модель
    // DotGraphGenerator::generateRepairableTransitionGraph(model.getTransitionMatrix(), params, "transition_graph_task2");
    std::cout << "Граф состояний сохранен в файлах state_graph_task2.dot и state_graph_task2.png" << std::endl;
    // std::cout << "Граф переходов сохранен в файлах transition_graph_task2.dot и transition_graph_task2.png" << std::endl;

    // Решаем стационарные уравнения Колмогорова
    Eigen::VectorXd steadyStateProbs = model.solveSteadyStateEquations();

    // Вычисляем характеристики системы
    double failureProb = model.calculateFailureProbability(steadyStateProbs);
    auto [avgA, avgB] = model.calculateReadyDevices(steadyStateProbs);
    double repairUtil = model.calculateRepairUtilization(steadyStateProbs);

    std::cout << "\nСтационарные характеристики системы:" << std::endl;
    std::cout << "Вероятность отказа системы: " << failureProb << std::endl;
    std::cout << "Среднее число готовых устройств типа A: " << avgA << std::endl;
    std::cout << "Среднее число готовых устройств типа B: " << avgB << std::endl;
    std::cout << "Коэффициент загрузки ремонтной службы: " << repairUtil << std::endl;

    // Оценка времени переходного процесса
    double transientTime = model.estimateTransientTime(steadyStateProbs);
    std::cout << "Оценка времени переходного процесса: " << transientTime << std::endl;

    // Решаем нестационарные уравнения Колмогорова (переходный процесс)
    double modelingTime = 2 * transientTime;  // Время моделирования вдвое больше времени переходного процесса
    auto [times, probabilities] = model.solveKolmogorovEquations(modelingTime, 200);

    // Визуализация результатов
    GnuplotPlotter::plotStatesProbabilities(times, probabilities, "states_probabilities_task2");
    std::cout << "График вероятностей состояний сохранен в файл states_probabilities_task2.png" << std::endl;

    // Имитационное моделирование (непрерывные марковские цепи)
    RepairableSimulator simulator(params);
    simulator.runMarkovChainSimulation(modelingTime);

    // Визуализация траектории марковского процесса
    std::vector<std::tuple<double, int, int, int>> markovTrajectory;
    std::ifstream markovFile("repairable_markov_chain_trajectory.dat");
    std::string line;

    // Пропускаем заголовок
    std::getline(markovFile, line);

    double time;
    int a, b, r;
    while (markovFile >> time >> a >> b >> r) {
        markovTrajectory.push_back(std::make_tuple(time, a, b, r));
    }
    markovFile.close();

    if (!markovTrajectory.empty()) {
        GnuplotPlotter::plotRepairableTrajectory(markovTrajectory, "repairable_markov_chain_trajectory");
        std::cout << "График траектории марковского процесса сохранен в файл repairable_markov_chain_trajectory.png" << std::endl;
    }

    // Имитационное моделирование (дискретно-событийное)
    simulator.runDiscreteEventSimulation(modelingTime);

    // Визуализация траектории дискретно-событийного процесса
    std::vector<std::tuple<double, int, int, int>> discreteTrajectory;
    std::ifstream discreteFile("repairable_discrete_event_trajectory.dat");

    // Пропускаем заголовок
    std::getline(discreteFile, line);

    while (discreteFile >> time >> a >> b >> r) {
        discreteTrajectory.push_back(std::make_tuple(time, a, b, r));
    }
    discreteFile.close();

    if (!discreteTrajectory.empty()) {
        GnuplotPlotter::plotRepairableTrajectory(discreteTrajectory, "repairable_discrete_event_trajectory");
        std::cout << "График траектории дискретно-событийного процесса сохранен в файл repairable_discrete_event_trajectory.png" << std::endl;
    }
}

int main() {
    int N = 260;
    int G = 1;

    std::string taskChoice;
    std::cin >> taskChoice;
    // std::ifstream input("input.txt");
    // std::getline(input, taskChoice);
    // input.close();

    std::cout << "Выбрана опция: " << taskChoice << std::endl;

    if (taskChoice == "1") {
        runTask1(N, G);
    } else if (taskChoice == "2") {
        runTask2(N, G);
    } else if (taskChoice == "all") {
        runTask1(N, G);
        runTask2(N, G);
    } else {
        std::cout << "Неверный выбор. Выполняются обе задачи." << std::endl;
        runTask1(N, G);
        runTask2(N, G);
    }

    return 0;
}
