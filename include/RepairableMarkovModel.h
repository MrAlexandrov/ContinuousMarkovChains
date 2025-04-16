#pragma once

#include "RepairableSystem.h"

#include <eigen3/Eigen/Dense>
#include <utility>
#include <vector>

class RepairableMarkovModel {
private:
    Eigen::MatrixXd Q;              // Матрица интенсивностей переходов
    Eigen::VectorXd initialState;   // Начальное состояние системы
    int numStates;                  // Общее число состояний
    RepairableSystemParams params;  // Параметры системы

    // Вспомогательные функции для построения матрицы переходов
    double getFailureRate(int a, int b, char deviceType) const;
    double getRepairRate(int a, int b, int r) const;

public:
    RepairableMarkovModel(const RepairableSystemParams& params);
    RepairableMarkovModel(double lambdaA, double lambdaB, int NA, int NB, int RA, int RB, double lambdaS);

    // Строит матрицу интенсивностей переходов
    void buildTransitionMatrix();

    // Решает стационарные уравнения Колмогорова
    Eigen::VectorXd solveSteadyStateEquations();

    // Решает ОДУ Колмогорова
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> solveKolmogorovEquations(double tMax, int steps);

    // Определяет время переходного процесса
    double estimateTransientTime(const Eigen::VectorXd& steadyStateProbs, double tolerance = 0.01);

    // Вычисляет вероятность отказа системы
    double calculateFailureProbability(const Eigen::VectorXd& stateProbs);

    // Вычисляет среднее число готовых к эксплуатации устройств
    std::pair<double, double> calculateReadyDevices(const Eigen::VectorXd& stateProbs);

    // Вычисляет коэффициент загрузки ремонтной службы
    double calculateRepairUtilization(const Eigen::VectorXd& stateProbs);

    // Возвращает матрицу интенсивностей
    const Eigen::MatrixXd& getTransitionMatrix() const { return Q; }
    
    // Создает и возвращает матрицу интенсивностей переходов в формате графа состояний
    // (без учета состояния ремонта, как в визуализации графа)
    Eigen::MatrixXd buildGraphTransitionMatrix() const;

    // Возвращает вектор указывающий на работоспособность системы
    Eigen::VectorXd getOperationalStates() const;

    // Агрегирует вероятности состояний по устройствам (игнорируя состояния ремонта)
    Eigen::VectorXd getAggregatedStateProbabilities(const Eigen::VectorXd& stateProbs) const;
};
