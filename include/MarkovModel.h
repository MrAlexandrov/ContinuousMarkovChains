#pragma once

#include "System.h"

#include <Eigen/Dense>
#include <utility>
// #include <vector>

class MarkovModel {
private:
    Eigen::MatrixXd Q;              // Матрица интенсивностей переходов
    Eigen::VectorXd initialState;   // Начальное состояние системы
    int numStates;                  // Общее число состояний
    SystemParams params;            // Параметры системы

public:
    MarkovModel(const SystemParams& params);
    
    // Строит матрицу интенсивностей переходов
    void buildTransitionMatrix();
    
    // Решает ОДУ Колмогорова
    std::pair<Eigen::VectorXd, Eigen::MatrixXd> solveKolmogorovEquations(double tMax, int steps);
    
    // Вычисляет функцию надежности
    Eigen::VectorXd getReliabilityFunction(const Eigen::MatrixXd& probabilities);
    
    // Вычисляет MTTF (среднее время до отказа)
    double calculateMTTF(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability);
    
    // Возвращает вектор, указывающий, какие состояния соответствуют отказу
    Eigen::VectorXd getFailureStates() const;
    
    // Возвращает матрицу интенсивностей
    const Eigen::MatrixXd& getTransitionMatrix() const { return Q; }
};
