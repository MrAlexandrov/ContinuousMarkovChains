#pragma once

#include <eigen3/Eigen/Dense>

// Forward declaration
class RepairableMarkovModel;

#include <vector>
#include <string>
#include <tuple>

class GnuplotPlotter {
public:
    static void plotStatesProbabilities(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities, const std::string& outputPrefix = "states_probabilities");

    static void plotReliabilityFunction(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability, const std::string& outputPrefix = "reliability_function");

    static void plotHistogram(const std::vector<double>& data, const std::string& title, const std::string& outputPrefix = "histogram");

    static void plotTrajectories(const std::vector<std::vector<std::pair<double, int>>>& trajectories, const std::string& outputPrefix = "state_trajectories");

    // Second task
    static void plotRepairableStates(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities, const std::string& outputPrefix = "repairable_states");

    static void plotRepairableTrajectory(const std::vector<std::tuple<double, int, int, int>>& trajectory, const std::string& outputPrefix = "repairable_trajectory");

    static void plotAggregatedStates(
        const Eigen::VectorXd& times,
        const Eigen::MatrixXd& probabilities,
        const RepairableMarkovModel& model,
        const std::string& outputPrefix = "aggregated_states");
};
