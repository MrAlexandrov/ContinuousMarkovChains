#include "DotGraphGenerator.h"

#include <fstream>
#include <iostream>
#include <cstdlib>
#include <iomanip>

void DotGraphGenerator::generateStateGraph(const System& system, const std::string& filename) {
    const SystemParams& params = system.getParams();
    std::ofstream dotFile(filename + ".dot");

    dotFile << "digraph StateGraph {" << "\n";
    dotFile << "  rankdir=LR;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";

    int totalA = params.NA + params.RA;
    int totalB = params.NB + params.RB;

    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int stateIndex = system.stateToIndex(a, b);

            std::string nodeStyle;
            if (a >= 1 && b >= params.NB) {
                nodeStyle = "style=filled, fillcolor=lightblue";
            } else {
                nodeStyle = "style=filled, fillcolor=lightcoral";
            }

            dotFile << "  S" << stateIndex << " [label=\"S" << stateIndex << "\\n(" << a << "," << b << ")\", " << nodeStyle << "];" << "\n";
        }
    }

    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int currentIndex = system.stateToIndex(a, b);
            
            // Only draw edges from operational states
            if (a >= 1 && b >= params.NB) {
                // Only active devices contribute to failure rate
                if (a > 0) {
                    int nextIndex = system.stateToIndex(a - 1, b);
                    // Only exactly NA devices of type A contribute to failure rate
                    double rate = std::min(a, params.NA) * params.lambdaA;
                    if (rate > 0) {
                        dotFile << "  S" << currentIndex << " -> S" << nextIndex << " [label=\"" << std::fixed << std::setprecision(2) << rate << "\"];" << "\n";
                    }
                }

                if (b > 0) {
                    int nextIndex = system.stateToIndex(a, b - 1);
                    // Only exactly NB devices of type B contribute to failure rate
                    double rate = std::min(b, params.NB) * params.lambdaB;
                    if (rate > 0) {
                        dotFile << "  S" << currentIndex << " -> S" << nextIndex << " [label=\"" << std::fixed << std::setprecision(2) << rate << "\"];" << "\n";
                    }
                }
            }
        }
    }

    dotFile << "}" << "\n";
    dotFile.close();

    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());

    std::cout << "Граф состояний сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}

void DotGraphGenerator::generateTransitionGraph(const Eigen::MatrixXd& Q, const SystemParams& params, const std::string& filename) {
    System system(params);
    std::ofstream dotFile(filename + ".dot");

    dotFile << "digraph TransitionGraph {" << "\n";
    dotFile << "  rankdir=LR;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";

    for (int i = 0; i < Q.rows(); i++) {
        auto [a, b] = system.indexToState(i);

        std::string nodeStyle;
        if (a >= 1 && b >= params.NB) {
            nodeStyle = "style=filled, fillcolor=lightblue";
        } else {
            nodeStyle = "style=filled, fillcolor=lightcoral";
        }

        dotFile << "  " << i << " [label=\"S" << i << "\\n(" << a << "," << b << ")\", " << nodeStyle << "];" << "\n";
    }

    for (int i = 0; i < Q.rows(); ++i) {
        for (int j = 0; j < Q.cols(); ++j) {
            if (i != j && Q(i, j) > 0) {
                dotFile << "  " << i << " -> " << j << " [label=\"" << std::fixed << std::setprecision(2) << Q(i, j) << "\"];" << "\n";
            }
        }
    }

    dotFile << "}" << "\n";
    dotFile.close();

    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());

    std::cout << "Граф переходов сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}

void DotGraphGenerator::generateRepairableStateGraph(const RepairableSystem& system, const std::string& filename) {
    const RepairableSystemParams& params = system.getParams();
    std::ofstream dotFile(filename + ".dot");

    dotFile << "digraph RepairableStateGraph {" << "\n";
    dotFile << "  rankdir=RL;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";

    int totalA = params.NA + params.RA;
    int totalB = params.NB + params.RB;

    // Создаем группы рангов для организации узлов по уровням
    for (int level = 0; level <= totalA + totalB; ++level) {
        dotFile << "  { rank = same; ";
        for (int a = 0; a <= totalA; ++a) {
            for (int b = 0; b <= totalB; ++b) {
                if (a + b == level) {
                    dotFile << "S" << system.stateToGraphIndex(a, b) << "; ";
                }
            }
        }
        dotFile << "}\n";
    }
    
    // Создаем узлы
    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int stateIndex = system.stateToGraphIndex(a, b);

            std::string nodeStyle;
            if (a >= 1 && b >= params.NB) {
                nodeStyle = "style=filled, fillcolor=lightblue";
            } else {
                nodeStyle = "style=filled, fillcolor=lightcoral";
            }

            // Добавляем информацию о позиции для лучшего размещения
            dotFile << "  S" << stateIndex << " [label=\"S" << stateIndex << "\\n(" << a << "," << b << ")\", " << nodeStyle << "];" << "\n";
        }
    }

    // Добавляем ребра для отказов и ремонтов
    // Обходим состояния в обратном порядке (от большего к меньшему)
    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int currentIndex = system.stateToGraphIndex(a, b);

            // Only draw failure edges from operational states
            if (a >= 1 && b >= params.NB) {
                // Ребра для отказов устройств типа A (красные сплошные)
                if (a > 0) {
                    int nextIndex = system.stateToGraphIndex(a - 1, b);
                    // Only exactly NA devices of type A contribute to failure rate
                    double rate = std::min(a, params.NA) * params.lambdaA;
                    if (rate > 0) {
                        dotFile << "  S" << currentIndex << " -> S" << nextIndex
                               << " [label=\"" << std::fixed << std::setprecision(2) << rate
                               << "\", color=red];" << "\n";
                    }
                }

                // Ребра для отказов устройств типа B (красные сплошные)
                if (b > 0) {
                    int nextIndex = system.stateToGraphIndex(a, b - 1);
                    // Only exactly NB devices of type B contribute to failure rate
                    double rate = std::min(b, params.NB) * params.lambdaB;
                    if (rate > 0) {
                        dotFile << "  S" << currentIndex << " -> S" << nextIndex
                               << " [label=\"" << std::fixed << std::setprecision(2) << rate
                               << "\", color=red];" << "\n";
                    }
                }
            }

            // Добавляем ребра для ремонта (синие пунктирные)
            // Ремонт устройства A (переход из (a,b) в (a+1,b))
            if (a < totalA) {
                int nextIndex = system.stateToGraphIndex(a + 1, b);
                // Проверяем, возможен ли ремонт A в этом состоянии
                // Ремонт возможен, если есть сломанные устройства A и приоритет у A
                if ((totalA - a) > 0 && ((totalA - a) > (totalB - b) ||
                    ((totalA - a) == (totalB - b) && params.lambdaA >= params.lambdaB))) {
                    dotFile << "  S" << currentIndex << " -> S" << nextIndex
                           << " [label=\"" << std::fixed << std::setprecision(2) << params.lambdaS
                           << "\", style=dashed, color=blue];" << "\n";
                }
            }

            // Ремонт устройства B (переход из (a,b) в (a,b+1))
            if (b < totalB) {
                int nextIndex = system.stateToGraphIndex(a, b + 1);
                // Проверяем, возможен ли ремонт B в этом состоянии
                // Ремонт возможен, если есть сломанные устройства B и приоритет у B
                if ((totalB - b) > 0 && ((totalB - b) > (totalA - a) ||
                    ((totalB - b) == (totalA - a) && params.lambdaB > params.lambdaA))) {
                    dotFile << "  S" << currentIndex << " -> S" << nextIndex
                           << " [label=\"" << std::fixed << std::setprecision(2) << params.lambdaS
                           << "\", style=dashed, color=blue];" << "\n";
                }
            }
        }
    }

    dotFile << "}" << "\n";
    dotFile.close();

    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());

    std::cout << "Граф состояний ремонтируемой системы сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}

void DotGraphGenerator::generateRepairableTransitionGraph(const Eigen::MatrixXd& Q, const RepairableSystemParams& params, const std::string& filename) {
    RepairableSystem system(params);
    std::ofstream dotFile(filename + ".dot");

    dotFile << "digraph RepairableTransitionGraph {" << "\n";
    dotFile << "  rankdir=LR;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";

    for (int i = 0; i < Q.rows(); i++) {
        auto [a, b, r] = system.indexToState(i);

        std::string nodeStyle;
        if (a >= 1 && b >= params.NB) {
            nodeStyle = "style=filled, fillcolor=lightblue";
        } else {
            nodeStyle = "style=filled, fillcolor=lightcoral";
        }

        std::string repairStatus;
        if (r == 0) {
            repairStatus = "нет ремонта";
        } else if (r == 1) {
            repairStatus = "ремонт A";
        } else {
            repairStatus = "ремонт B";
        }

        dotFile << "  " << i << " [label=\"S" << i << "\\n(" << a << "," << b << "," << r << ")\\n" << repairStatus << "\", " << nodeStyle << "];" << "\n";
    }

    for (int i = 0; i < Q.rows(); ++i) {
        for (int j = 0; j < Q.cols(); ++j) {
            if (i != j && Q(i, j) > 0) {
                dotFile << "  " << i << " -> " << j << " [label=\"" << std::fixed << std::setprecision(2) << Q(i, j) << "\"];" << "\n";
            }
        }
    }

    dotFile << "}" << "\n";
    dotFile.close();

    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());

    std::cout << "Граф переходов ремонтируемой системы сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}
