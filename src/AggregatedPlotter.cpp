#include "../include/GnuplotPlotter.h"
#include "../include/RepairableMarkovModel.h"

#include <fstream>
#include <cstdlib>

void GnuplotPlotter::plotAggregatedStates(
    const Eigen::VectorXd& times,
    const Eigen::MatrixXd& probabilities,
    const RepairableMarkovModel& model,
    const std::string& outputPrefix) {

    constexpr int AMOUNT = 18;

    // Get aggregated probabilities
    Eigen::MatrixXd aggregatedProbs(AMOUNT, probabilities.cols());
    for (int t = 0; t < probabilities.cols(); ++t) {
        aggregatedProbs.col(t) = model.getAggregatedStateProbabilities(probabilities.col(t));
    }

    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 1200,800\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title 'Aggregated State Probabilities (18 states)'\n";
    script << "set xlabel 'Time'\n";
    script << "set ylabel 'Probability'\n";
    script << "set grid\n";
    script << "set key outside\n";

    std::ofstream datafile(outputPrefix + ".dat");
    datafile << "#Time ";
    for (int i = 0; i < AMOUNT; ++i) {
        datafile << "State" << i << " ";
    }
    datafile << "\n";

    for (int t = 0; t < times.size(); ++t) {
        datafile << times(t) << " ";
        for (int i = 0; i < AMOUNT; ++i) {
            datafile << aggregatedProbs(i, t) << " ";
        }
        datafile << "\n";
    }
    datafile.close();

    script << "plot ";
    for (int i = 0; i < AMOUNT; ++i) {
        if (i > 0) script << ", ";
        script << "'" << outputPrefix << ".dat' using 1:" << (i+2)
               << " with lines title 'State " << i << "'";
    }
    script << "\n";

    script.close();
    std::system(("gnuplot " + outputPrefix + ".gp").c_str());
}
