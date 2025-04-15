#pragma once
#include <Eigen/Dense>
#include <string>
#include "System.h"
#include "SystemParams.h"
#include "RepairableSystem.h"
#include "RepairableSystemParams.h"

class DotGraphGenerator {
public:
    static void generateTransitionGraph(const Eigen::MatrixXd& Q, const SystemParams& params, const std::string& filename = "transition_graph");

    static void generateStateGraph(const System& system, const std::string& filename = "state_graph");
    
    // Новые методы для работы с RepairableSystem
    static void generateRepairableStateGraph(const RepairableSystem& system, const std::string& filename = "state_graph_task2");
    
    static void generateRepairableTransitionGraph(const Eigen::MatrixXd& Q, const RepairableSystemParams& params, const std::string& filename = "transition_graph_task2");
};
