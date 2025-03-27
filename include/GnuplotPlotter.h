#pragma once

#include <Eigen/Dense>

#include <vector>
#include <string>

class GnuplotPlotter {
public:
    // Строит графики вероятностей состояний
    static void plotStatesProbabilities(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities);
    
    // Строит график функции надежности
    static void plotReliabilityFunction(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability);
    
    // Строит гистограмму времени до отказа
    static void plotHistogram(const std::vector<double>& data, const std::string& title);
    
    // Строит траектории состояний для нескольких экспериментов
    static void plotTrajectories(const std::vector<std::vector<std::pair<double, int>>>& trajectories);
};
