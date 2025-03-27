#include "GnuplotPlotter.h"
#include <fstream>
#include <iostream>
#include <cstdlib>

void GnuplotPlotter::plotStatesProbabilities(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities) {
    // Создаем файл с данными
    std::ofstream dataFile("state_probs.dat");
    dataFile << "# time";
    for (int i = 0; i < probabilities.rows(); ++i) {
        dataFile << " state" << i+1;
    }
    dataFile << "\n";
    
    for (int t = 0; t < times.size(); ++t) {
        dataFile << times(t);
        for (int s = 0; s < probabilities.rows(); ++s) {
            dataFile << " " << probabilities(s, t);
        }
        dataFile << "\n";
    }
    dataFile.close();
    
    // Создаем gnuplot скрипт
    std::ofstream scriptFile("plot_states.gp");
    // Установка терминала в начале скрипта
    scriptFile << "set terminal png\n";
    scriptFile << "set output 'states_probabilities.png'\n";
    scriptFile << "set title 'Вероятности состояний системы'\n";
    scriptFile << "set xlabel 'Время'\n";
    scriptFile << "set ylabel 'Вероятность'\n";
    scriptFile << "set grid\n";
    scriptFile << "plot ";
    
    for (int i = 0; i < probabilities.rows(); ++i) {
        if (i > 0) scriptFile << ", ";
        scriptFile << "'state_probs.dat' using 1:" << i+2 << " with lines title 'P" << i+1 << "'";
    }
    
    scriptFile << "\n";
    scriptFile.close();
    
    // Запускаем gnuplot в тихом режиме без интерактивного вывода
    system("gnuplot -c plot_states.gp");
    std::cout << "График вероятностей состояний сохранен в файл states_probabilities.png" << "\n";
}

void GnuplotPlotter::plotReliabilityFunction(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability) {
    // Создаем файл с данными
    std::ofstream dataFile("reliability.dat");
    dataFile << "# time reliability\n";
    
    for (int t = 0; t < times.size(); ++t) {
        dataFile << times(t) << " " << reliability(t) << "\n";
    }
    dataFile.close();
    
    // Создаем gnuplot скрипт
    std::ofstream scriptFile("plot_reliability.gp");
    scriptFile << "set terminal png enhanced font 'Arial,12' size 800,600\n";
    scriptFile << "set output 'reliability_function.png'\n";
    scriptFile << "set title 'Функция надежности системы' font 'Arial,14'\n";
    scriptFile << "set xlabel 'Время t' font 'Arial,12'\n";
    scriptFile << "set ylabel 'Вероятность безотказной работы P(t)' font 'Arial,12'\n";
    scriptFile << "set grid\n";
    scriptFile << "set key top right\n";
    scriptFile << "set xrange [0:*]\n";
    scriptFile << "set yrange [0:1]\n";
    scriptFile << "plot 'reliability.dat' using 1:2 with lines lw 2 lc rgb '#0060ad' title 'P(t)'\n";
    scriptFile.close();
    
    // Запускаем gnuplot
    system("gnuplot -c plot_reliability.gp");
    std::cout << "График функции надежности сохранен в файл reliability_function.png" << "\n";
}


void GnuplotPlotter::plotTrajectories(const std::vector<std::vector<std::pair<double, int>>>& trajectories) {
    // Создаем отдельные файлы для каждой траектории
    for (size_t i = 0; i < trajectories.size(); ++i) {
        std::string filename = "trajectory_" + std::to_string(i+1) + ".dat";
        std::ofstream dataFile(filename);
        
        for (const auto& point : trajectories[i]) {
            dataFile << point.first << " " << point.second + 1 << "\n";
        }
        dataFile.close();
    }
    
    // Создаем gnuplot скрипт
    std::ofstream scriptFile("plot_trajectories.gp");
    // Установка терминала в начале скрипта
    scriptFile << "set terminal png\n";
    scriptFile << "set output 'state_trajectories.png'\n";
    scriptFile << "set title 'Траектории состояний системы'\n";
    scriptFile << "set xlabel 'Время'\n";
    scriptFile << "set ylabel 'Состояние'\n";
    scriptFile << "set grid\n";
    scriptFile << "plot ";
    
    for (size_t i = 0; i < trajectories.size(); ++i) {
        if (i > 0) scriptFile << ", ";
        scriptFile << "'trajectory_" << i+1 << ".dat' with steps title 'Эксперимент " << i+1 << "'";
    }
    
    scriptFile << "\n";
    scriptFile.close();
    
    // Запускаем gnuplot в тихом режиме
    system("gnuplot -c plot_trajectories.gp");
    std::cout << "График траекторий сохранен в файл state_trajectories.png" << "\n";
}
