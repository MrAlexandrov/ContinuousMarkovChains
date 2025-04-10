#pragma once

#include <Eigen/Dense>

#include <vector>
#include <string>
#include <tuple>

class GnuplotPlotter {
public:
    // Строит графики вероятностей состояний
    static void plotStatesProbabilities(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities, const std::string& outputPrefix = "states_probabilities");
    
    // Строит график функции надежности
    static void plotReliabilityFunction(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability, const std::string& outputPrefix = "reliability_function");
    
    // Строит гистограмму времени до отказа
    static void plotHistogram(const std::vector<double>& data, const std::string& title, const std::string& outputPrefix = "histogram");
    
    // Строит траектории состояний для нескольких экспериментов
    static void plotTrajectories(const std::vector<std::vector<std::pair<double, int>>>& trajectories, const std::string& outputPrefix = "state_trajectories");
    
    // Методы для системы с ремонтом
    
    // Строит графики вероятностей состояний для системы с ремонтом
    static void plotRepairableStates(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities, const std::string& outputPrefix = "repairable_states");
    
    // Строит траекторию состояний системы с ремонтом (рабочие устройства и статус ремонта)
    static void plotRepairableTrajectory(const std::vector<std::tuple<double, int, int, int>>& trajectory, const std::string& outputPrefix = "repairable_trajectory");
};
