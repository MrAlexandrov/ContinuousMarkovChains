#pragma once

#include "System.h"
// #include "SystemParams.h"
#include "RepairableSystem.h"
// #include "RepairableSystemParams.h"

#include <Eigen/Dense>

#include <string>

class DotGraphGenerator {
public:
    static void generateStateGraph(const System& system, const std::string& filename = "state_graph");

    static void generateRepairableStateGraph(const RepairableSystem& system, const std::string& filename = "state_graph_task2");
};
