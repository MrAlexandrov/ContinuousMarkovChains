#pragma once
#include <Eigen/Dense>
#include <string>
#include "System.h"
#include "SystemParams.h"

class DotGraphGenerator {
public:
    static void generateTransitionGraph(const Eigen::MatrixXd& Q, const SystemParams& params, const std::string& filename = "transition_graph");

    static void generateStateGraph(const System& system, const std::string& filename = "state_graph");
};
