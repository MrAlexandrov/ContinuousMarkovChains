#include "GnuplotPlotter.h"
#include "RepairableMarkovModel.h"

#include <fstream>
#include <cstdlib>
#include <sstream>
#include <vector>

void GnuplotPlotter::plotStatesProbabilities(
    const Eigen::VectorXd& times
    , const Eigen::MatrixXd& probabilities
    , const std::string& outputPrefix
) {
    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Вероятности состояний системы'\n";
    script << "set xlabel 'Время'\n";
    script << "set ylabel 'Вероятность'\n";
    script << "set grid\n";
    script << "set key outside\n";

    std::ofstream datafile(outputPrefix + ".dat");
    datafile << "# Time ";
    for (int i = 0; i < probabilities.rows(); ++i) {
        datafile << "State" << i << " ";
    }
    datafile << "\n";

    for (int t = 0; t < times.size(); ++t) {
        datafile << times(t) << " ";
        for (int i = 0; i < probabilities.rows(); ++i) {
            datafile << probabilities(i, t) << " ";
        }
        datafile << "\n";
    }
    datafile.close();

    int maxStates = static_cast<int>(probabilities.rows());

    script << "plot ";
    for (int i = 0; i < maxStates; ++i) {
        if (i > 0) script << ", ";
        script << "'" << outputPrefix << ".dat' using 1:" << (i + 2) << " with lines title 'State " << i << "'";
    }
    script << "\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}

void GnuplotPlotter::plotReliabilityFunction(
    const Eigen::VectorXd& times
    , const Eigen::VectorXd& reliability
    , const std::string& outputPrefix
) {
    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Функция надежности системы'\n";
    script << "set xlabel 'Время'\n";
    script << "set ylabel 'Вероятность'\n";
    script << "set grid\n";

    std::ofstream datafile(outputPrefix + ".dat");
    datafile << "# Time Reliability\n";

    for (int t = 0; t < times.size(); ++t) {
        datafile << times(t) << " " << reliability(t) << "\n";
    }
    datafile.close();

    script << "plot '" << outputPrefix << ".dat' using 1:2 with lines title 'Reliability' lw 2\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}

void GnuplotPlotter::plotTrajectories(
    const std::vector<std::vector<std::pair<double, int>>>& trajectories
    , const std::string& outputPrefix
) {
    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Траектории состояний системы'\n";
    script << "set xlabel 'Время'\n";
    script << "set ylabel 'Состояние'\n";
    script << "set grid\n";

    for (size_t i = 0; i < trajectories.size(); ++i) {
        std::ostringstream filename;
        filename << outputPrefix << "_" << i << ".dat";
        std::ofstream datafile(filename.str());
        datafile << "# Time State\n";

        for (const auto& [time, state] : trajectories[i]) {
            datafile << time << " " << state << "\n";
        }
        datafile.close();
    }

    script << "plot ";
    for (size_t i = 0; i < trajectories.size(); ++i) {
        if (i > 0) script << ", ";
        script << "'" << outputPrefix << "_" << i << ".dat' using 1:2 with steps title 'Traj " << i << "'";
    }
    script << "\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}

#include "RepairableMarkovModel.h"

void GnuplotPlotter::plotRepairableStates(const Eigen::VectorXd& times, const Eigen::MatrixXd& probabilities, const std::string& outputPrefix) {
    Eigen::MatrixXd aggregatedProbs(18, probabilities.cols());
    RepairableMarkovModel model(RepairableSystemParams(0,0));

    for (int t = 0; t < probabilities.cols(); ++t) {
        aggregatedProbs.col(t) = model.getAggregatedStateProbabilities(probabilities.col(t));
    }

    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Вероятности состояний системы (агрегированные)'\n";
    script << "set xlabel 'Время'\n";
    script << "set ylabel 'Вероятность'\n";
    script << "set grid\n";
    script << "set key outside\n";

    std::ofstream datafile(outputPrefix + ".dat");
    datafile << "# Time ";
    for (int i = 0; i < aggregatedProbs.rows(); ++i) {
        datafile << "State" << i << " ";
    }
    datafile << "\n";

    for (int t = 0; t < times.size(); ++t) {
        datafile << times(t) << " ";
        for (int i = 0; i < aggregatedProbs.rows(); ++i) {
            datafile << aggregatedProbs(i, t) << " ";
        }
        datafile << "\n";
    }
    datafile.close();

    script << "plot ";
    for (int i = 0; i < aggregatedProbs.rows(); ++i) {
        if (i > 0) script << ", ";
        script << "'" << outputPrefix << ".dat' using 1:" << (i + 2) << " with lines title 'State " << i << "'";
    }
    script << "\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}

void GnuplotPlotter::plotRepairableTrajectory(const std::vector<std::tuple<double, int, int, int>>& trajectory, const std::string& outputPrefix) {
    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Траектория состояний системы с ремонтом'\n";
    script << "set xlabel 'Время'\n";
    script << "set grid\n";

    std::ofstream datafile(outputPrefix + ".dat");
    datafile << "# Time WorkingA WorkingB RepairStatus\n";

    for (const auto& [time, a, b, r] : trajectory) {
        datafile << time << " " << a << " " << b << " " << r << "\n";
    }
    datafile.close();

    script << "set multiplot layout 3,1 title 'Траектория системы с ремонтом'\n";

    script << "set ylabel 'Устройства A'\n";
    script << "plot '" << outputPrefix << ".dat' using 1:2 with steps title 'Working A'\n";

    script << "set ylabel 'Устройства B'\n";
    script << "plot '" << outputPrefix << ".dat' using 1:3 with steps title 'Working B'\n";

    script << "set ylabel 'Ремонт'\n";
    script << "set yrange [-0.5:2.5]\n";
    script << "set ytics ('Нет' 0, 'A' 1, 'B' 2)\n";
    script << "plot '" << outputPrefix << ".dat' using 1:4 with steps title 'Repair Status'\n";

    script << "unset multiplot\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}
