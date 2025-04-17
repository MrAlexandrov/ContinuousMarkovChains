#include "../include/MarkovModel.h"
#include "../include/Simulator.h"
#include "../include/GnuplotPlotter.h"
#include "../include/DotGraphGenerator.h"
#include "../include/RepairableSystem.h"
#include "../include/RepairableMarkovModel.h"
#include "../include/RepairableSimulator.h"
#include "../include/System.h"
#include "../include/SystemParams.h"
#include <eigen3/Eigen/Dense>

#include <exception>
#include <iostream>
#include <fstream>
#include <string>

namespace {

// Функция для вычисления среднего значения вектора
double calculateMean(const std::vector<double>& values) {
    if (values.empty()) {
        return 0.0;
    }

    double sum = 0.0;
    for (double value : values) {
        sum += value;
    }
    return sum / values.size();
}

// Функция для вычисления стандартного отклонения вектора
double calculateStandardDeviation(const std::vector<double>& values, double mean) {
    if (values.empty() || values.size() == 1) {
        return 0.0;
    }

    double variance = 0.0;
    for (double value : values) {
        variance += (value - mean) * (value - mean);
    }
    variance /= values.size();
    return std::sqrt(variance);
}

// Функция для проведения имитационного моделирования и сбора статистики
std::vector<double> runSimulation(Simulator& simulator, int numRuns) {
    std::vector<double> failureTimes;
    failureTimes.reserve(numRuns);

    for (int i = 0; i < numRuns; ++i) {
        failureTimes.push_back(simulator.simulateSingleRun());
    }

    return failureTimes;
}

// Функция для визуализации результатов имитационного моделирования
void visualizeSimulationResults(const std::vector<double>& failureTimes, const std::string& histogramTitle, const std::string& outputPrefix) {
    // Строим гистограмму времен отказов
    GnuplotPlotter::plotHistogram(failureTimes, histogramTitle, outputPrefix);
    std::cout << "Гистограмма времен отказов сохранена в файл " << outputPrefix << ".png" << std::endl;
}

// Функция для генерации и визуализации траекторий системы
void generateAndVisualizeTrajectories(Simulator& simulator, int numTrajectories, const std::string& outputPrefix) {
    std::vector<std::vector<std::pair<double, int>>> trajectories;
    trajectories.reserve(numTrajectories);

    for (int i = 0; i < numTrajectories; ++i) {
        trajectories.push_back(simulator.simulateTrajectory());
    }

    GnuplotPlotter::plotTrajectories(trajectories, outputPrefix);
    std::cout << "График траекторий сохранен в файл " << outputPrefix << ".png" << std::endl;
}

// Структура для хранения статистических характеристик
struct SimulationStats {
    double mean;
    double stdDev;

    SimulationStats(double m, double s) : mean(m), stdDev(s) {}
};

// Функция для вычисления и вывода статистических характеристик
SimulationStats calculateAndPrintStats(const std::vector<double>& failureTimes) {
    double mean = calculateMean(failureTimes);
    double stdDev = calculateStandardDeviation(failureTimes, mean);

    std::cout << "MTTF (имитационный): " << mean << "\n";
    std::cout << "Стандартное отклонение: " << stdDev << "\n";

    return SimulationStats(mean, stdDev);
}

void Separate(int x) {
    constexpr auto SEPARATOR = "----------------------------------------";
    std::cout << x << ". " << SEPARATOR << "\n";
}

void saveMatrix(const Eigen::MatrixXd& matrix, const std::string& filename) {
    std::ofstream transitionMatrix(filename);
    transitionMatrix << matrix << "\n";
    transitionMatrix.close();
}

void runTask1(int N, int G) {
    SystemParams params(N, G);

    std::cout << "Параметры системы (Задача 1):" << "\n";
    std::cout << "lambda_A = " << params.lambdaA << "\n";
    std::cout << "lambda_B = " << params.lambdaB << "\n";
    std::cout << "N_A = " << params.NA << "\n";
    std::cout << "N_B = " << params.NB << "\n";
    std::cout << "R_A = " << params.RA << "\n";
    std::cout << "R_B = " << params.RB << "\n";

    Separate(1);
    std::cout << "Построение графа состояний системы:" << "\n";
    System system(params);
    DotGraphGenerator::generateStateGraph(system, "state_graph_task1");
    std::cout << "State graph saved in state_graph_task1.png" << "\n";

    Separate(2);
    std::cout << "Построение матрицы интенсивностей переходов:" << "\n";
    MarkovModel model(params);
    model.buildTransitionMatrix();
    saveMatrix(model.getTransitionMatrix(), "transition_matrix_task1.dat");
    std::cout << "Transition matrix saved in transition_matrix_task1.dat" << "\n";

    Separate(3);
    std::cout << "Запись дифференциальных уравнений Колмогорова:" << "\n";
    std::cout << "dp(t)/dt = Q^T * p(t), где Q - матрица интенсивностей переходов" << "\n";

    Separate(4);
    std::cout << "Решение системы дифференциальных уравнений:" << "\n";
    auto [times, probabilities] = model.solveKolmogorovEquations(2.0, 100);

    Separate(5);
    std::cout << "Построение графиков вероятностей состояний:" << "\n";
    GnuplotPlotter::plotStatesProbabilities(times, probabilities, "states_probabilities_task1");
    std::cout << "График вероятностей состояний сохранен в файл states_probabilities_task1.png" << std::endl;

    Separate(6);
    std::cout << "Построение графика функции надежности:" << "\n";
    Eigen::VectorXd reliability = model.getReliabilityFunction(probabilities);
    GnuplotPlotter::plotReliabilityFunction(times, reliability, "reliability_function_task1");
    std::cout << "График функции надежности сохранен в файл reliability_function_task1.png" << std::endl;

    Separate(7);
    std::cout << "Расчет математического ожидания времени безотказной работы:" << "\n";
    double mttf = model.calculateMTTF(times, reliability);
    std::cout << "MTTF (аналитический): " << mttf << "\n";

    Separate(8);
    std::cout << "Имитационное моделирование системы:" << "\n";
    Simulator simulator(params);

    // Проводим имитационное моделирование (100 экспериментов)
    const int numSimulations = 100;
    std::vector<double> failureTimes = runSimulation(simulator, numSimulations);

    // Вычисляем и выводим статистические характеристики
    SimulationStats stats = calculateAndPrintStats(failureTimes);

    // Визуализируем результаты
    visualizeSimulationResults(failureTimes, "Распределение времени безотказной работы", "failure_times_histogram_task1");

    // Генерируем и визуализируем 10 траекторий системы (дополнительно)
    generateAndVisualizeTrajectories(simulator, 10, "state_trajectories_task1");
}

void runTask2(int N, int G) {
    RepairableSystemParams params(N, G);

    std::cout << "\n\n";
    std::cout << "Параметры системы (Задача 2):" << "\n";
    std::cout << "lambda_A = " << params.lambdaA << "\n";
    std::cout << "lambda_B = " << params.lambdaB << "\n";
    std::cout << "lambda_S = " << params.lambdaS << "\n";
    std::cout << "NA = " << params.NA << "\n";
    std::cout << "NB = " << params.NB << "\n";
    std::cout << "RA = " << params.RA << "\n";
    std::cout << "RB = " << params.RB << "\n";

    Separate(1);
    std::cout << "Построение графа состояний ремонтируемой системы:" << "\n";
    RepairableSystem system(params);
    DotGraphGenerator::generateRepairableStateGraph(system, "state_graph_task2");
    std::cout << "Граф состояний сохранен в файлах state_graph_task2.dot и state_graph_task2.png" << std::endl;


    Separate(2);
    std::cout << "Построение матрицы интенсивностей переходов ремонтируемой системы:" << "\n";
    RepairableMarkovModel model(params);
    model.buildTransitionMatrix();
    saveMatrix(model.getTransitionMatrix(), "transition_matrix_task2.dat");

    // Создаем и сохраняем матрицу переходов в формате графа состояний
    Eigen::MatrixXd graphQ = model.buildGraphTransitionMatrix();
    saveMatrix(graphQ, "graph_transition_matrix_task2.dat");

    std::cout << "Матрица переходов сохранена в файлах transition_matrix_task2.dat и graph_transition_matrix_task2.dat" << std::endl;

    Separate(3);
    std::cout << "Запись алгебраических уравнений Колмогорова для установившегося режима:" << "\n";
    std::cout << "Q^T * π = 0, где π - вектор предельных вероятностей" << "\n";
    std::cout << "Σπ_i = 1 (условие нормировки)" << "\n";

    Separate(4);
    std::cout << "Расчет предельных вероятностей состояний системы:" << "\n";
    // Решаем стационарные уравнения Колмогорова
    Eigen::VectorXd steadyStateProbs = model.solveSteadyStateEquations();
    std::cout << "Предельные вероятности рассчитаны" << "\n";

    Separate(5);
    std::cout << "Расчет математических ожиданий прикладных характеристик системы:" << std::endl;
    // Вычисляем характеристики системы
    double failureProb = model.calculateFailureProbability(steadyStateProbs);
    auto [avgA, avgB] = model.calculateReadyDevices(steadyStateProbs);
    double repairUtil = model.calculateRepairUtilization(steadyStateProbs);

    std::cout << "Вероятность отказа системы: " << failureProb << std::endl;
    std::cout << "Среднее число готовых устройств типа A: " << avgA << std::endl;
    std::cout << "Среднее число готовых устройств типа B: " << avgB << std::endl;
    std::cout << "Коэффициент загрузки ремонтной службы: " << repairUtil << std::endl;

    Separate(6);
    std::cout << "Запись дифференциальных уравнений Колмогорова:" << "\n";
    std::cout << "dp(t)/dt = Q^T * p(t), где Q - матрица интенсивностей переходов" << "\n";

    Separate(7);
    std::cout << "Решение системы дифференциальных уравнений:" << "\n";
    // Оценка времени переходного процесса
    double transientTime = model.estimateTransientTime(steadyStateProbs);
    std::cout << "Оценка времени переходного процесса: " << transientTime << std::endl;

    // Решаем нестационарные уравнения Колмогорова (переходный процесс)
    double modelingTime = 2 * transientTime;  // Время моделирования вдвое больше времени переходного процесса
    auto [times, probabilities] = model.solveKolmogorovEquations(modelingTime, 200);

    Separate(8);
    std::cout << "Построение графиков вероятностей состояний:" << "\n";
    GnuplotPlotter::plotAggregatedStates(times, probabilities, model, "states_probabilities_task2");
    std::cout << "График агрегированных вероятностей состояний (18 состояний) сохранен в файл states_probabilities_task2.png" << std::endl;

    Separate(9);
    std::cout << "Имитационное моделирование в терминах непрерывных марковских цепей:" << "\n";
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

    Separate(10);
    std::cout << "Имитационное моделирование в терминах дискретно-событийного моделирования:" << "\n";
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

} // anonimous namespace

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
