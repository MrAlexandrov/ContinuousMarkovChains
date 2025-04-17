#include "../include/GnuplotPlotter.h"

#include <fstream>
#include <cstdlib>
#include <algorithm>
#include <cmath>

void GnuplotPlotter::plotHistogram(
    const std::vector<double>& data,
    const std::string& title,
    const std::string& outputPrefix
) {
    std::ofstream script(outputPrefix + ".gp");
    script << "set terminal png size 800,600\n";
    script << "set output '" << outputPrefix << ".png'\n";
    script << "set title '" << title << "'\n";
    script << "set xlabel 'Время до отказа'\n";
    script << "set ylabel 'Частота'\n";
    script << "set grid\n";

    int numBins = 1 + 3.322 * log10(data.size());

    double minVal = *std::min_element(data.begin(), data.end());
    double maxVal = *std::max_element(data.begin(), data.end());

    double binWidth = (maxVal - minVal) / numBins;

    script << "binwidth = " << binWidth << "\n";
    script << "bin(x,width) = width*floor(x/width)\n";
    script << "set boxwidth binwidth\n";

    std::ofstream datafile(outputPrefix + ".dat");
    for (double value : data) {
        datafile << value << "\n";
    }
    datafile.close();

    script << "plot '" << outputPrefix << ".dat' using (bin($1,binwidth)):(1.0) "
           << "smooth freq with boxes title 'Frequency'\n";

    script.close();

    std::string command = "gnuplot " + outputPrefix + ".gp";
    std::system(command.c_str());
}
