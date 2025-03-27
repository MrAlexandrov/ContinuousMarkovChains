#include "DotGraphGenerator.h"
#include <fstream>
#include <iostream>
#include <cstdlib>
#include <iomanip>
// #include <sstream>

void DotGraphGenerator::generateTransitionGraph(const Eigen::MatrixXd& Q, const SystemParams& params, const std::string& filename) {
    System system(params);
    std::ofstream dotFile(filename + ".dot");
    
    dotFile << "digraph TransitionGraph {" << "\n";
    dotFile << "  rankdir=LR;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";
    
    // Добавляем узлы
    for (int i = 0; i < Q.rows(); i++) {
        auto [a, b] = system.indexToState(i);
        
        // Определяем стиль узла: рабочее состояние или состояние отказа
        std::string nodeStyle;
        if (a >= 1 && b >= params.NB) {
            nodeStyle = "style=filled, fillcolor=lightblue";
        } else {
            nodeStyle = "style=filled, fillcolor=lightcoral";
        }
        
        dotFile << "  " << i+1 << " [label=\"S" << i+1 << "\\n(" << a << "," << b << ")\", " << nodeStyle << "];" << "\n";
    }
    
    // Добавляем переходы с метками интенсивностей
    for (int i = 0; i < Q.rows(); i++) {
        for (int j = 0; j < Q.cols(); j++) {
            if (i != j && std::abs(Q(i, j)) > 1e-10) {
                dotFile << "  " << i+1 << " -> " << j+1 << " [label=\"" << std::fixed << std::setprecision(2) << Q(i, j) << "\"];" << "\n";
            }
        }
    }
    
    dotFile << "}" << "\n";
    dotFile.close();
    
    // Генерируем PNG-изображение с помощью dot
    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());
    
    std::cout << "Граф переходов сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}

void DotGraphGenerator::generateStateGraph(const System& system, const std::string& filename) {
    const SystemParams& params = system.getParams();
    std::ofstream dotFile(filename + ".dot");
    
    dotFile << "digraph StateGraph {" << "\n";
    dotFile << "  rankdir=LR;" << "\n";
    dotFile << "  node [shape=circle];" << "\n";
    
    int totalA = params.NA + params.RA;
    int totalB = params.NB + params.RB;
    
    // Создаем узлы для всех возможных состояний системы
    for (int a = 0; a <= totalA; a++) {
        for (int b = 0; b <= totalB; b++) {
            int stateIndex = system.stateToIndex(a, b) + 1; // +1 для S1-индексации
            
            std::string nodeStyle;
            if (a >= 1 && b >= params.NB) {
                nodeStyle = "style=filled, fillcolor=lightblue";
            } else {
                nodeStyle = "style=filled, fillcolor=lightcoral";
            }
            
            dotFile << "  S" << stateIndex << " [label=\"S" << stateIndex << "\\n(" << a << "," << b << ")\", " << nodeStyle << "];" << "\n";
        }
    }
    
    // Добавляем переходы
    for (int a = 0; a <= totalA; a++) {
        for (int b = 0; b <= totalB; b++) {
            int currentIndex = system.stateToIndex(a, b) + 1;
            
            // Переход из-за отказа устройства A
            if (a > 0) {
                int nextIndex = system.stateToIndex(a-1, b) + 1;
                double rate = a * params.lambdaA;
                dotFile << "  S" << currentIndex << " -> S" << nextIndex << " [label=\"" << std::fixed << std::setprecision(2) << rate << "\"];" << "\n";
            }
            
            // Переход из-за отказа устройства B
            if (b > 0) {
                int nextIndex = system.stateToIndex(a, b-1) + 1;
                double rate = b * params.lambdaB;
                dotFile << "  S" << currentIndex << " -> S" << nextIndex << " [label=\"" << std::fixed << std::setprecision(2) << rate << "\"];" << "\n";
            }
        }
    }
    
    dotFile << "}" << "\n";
    dotFile.close();
    
    // Генерируем PNG-изображение с помощью dot
    std::string command = "dot -Tpng " + filename + ".dot -o " + filename + ".png";
    ::system(command.c_str());
    
    std::cout << "Граф состояний сохранен в файлах " << filename << ".dot и " << filename << ".png" << "\n";
}
