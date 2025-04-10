#include "MarkovModel.h"
// #include <numeric>

MarkovModel::MarkovModel(const SystemParams& params) : params(params) {
    System system(params);
    numStates = system.getTotalStates();
    
    // Инициализация матрицы и начального состояния
    Q = Eigen::MatrixXd::Zero(numStates, numStates);
    initialState = Eigen::VectorXd::Zero(numStates);
    initialState(0) = 1.0; // Начальное состояние - все устройства работают
    
    buildTransitionMatrix();
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> MarkovModel::solveKolmogorovEquations(double tMax, int steps) {
    // Создаем вектор времени
    Eigen::VectorXd times = Eigen::VectorXd::LinSpaced(steps, 0, tMax);
    
    // Решаем систему ОДУ с более мелким шагом
    const int internalSteps = 20;  // Увеличьте это значение при необходимости
    
    Eigen::MatrixXd probabilities(numStates, steps);
    probabilities.col(0) = initialState;
    
    for (int i = 0; i < steps - 1; ++i) {
        double dt = (times(i + 1) - times(i)) / internalSteps;
        Eigen::VectorXd pi = probabilities.col(i);
        
        // Выполняем несколько маленьких шагов между основными шагами
        for (int j = 0; j < internalSteps; ++j) {
            Eigen::VectorXd k1 = Q.transpose() * pi;
            Eigen::VectorXd k2 = Q.transpose() * (pi + dt / 2 * k1);
            Eigen::VectorXd k3 = Q.transpose() * (pi + dt / 2 * k2);
            Eigen::VectorXd k4 = Q.transpose() * (pi + dt * k3);
            
            pi = pi + dt / 6 * (k1 + 2 * k2 + 2 * k3 + k4);
            
            // Коррекция вероятностей
            for (int k = 0; k < numStates; ++k) {
                if (pi(k) < 0) {
                    pi(k) = 0;
                }
            }
            
            // Нормализация
            double sum = pi.sum();
            if (sum > 0) {
                pi /= sum;
            }
        }
        
        probabilities.col(i + 1) = pi;
    }
    
    return {times, probabilities};
}

Eigen::VectorXd MarkovModel::getReliabilityFunction(const Eigen::MatrixXd& probabilities) {
    Eigen::VectorXd reliability(probabilities.cols());

    System system(params);
    for (int t = 0; t < probabilities.cols(); ++t) {
        double failureProb = 0.0;

        for (int s = 0; s < numStates; ++s) {
            auto [a, b] = system.indexToState(s);
            if (a < 1 || b < params.NB) {
                failureProb += probabilities(s, t);
            }
        }

        reliability(t) = 1.0 - failureProb;
    }

    return reliability;
}

double MarkovModel::calculateMTTF(const Eigen::VectorXd& times, const Eigen::VectorXd& reliability) {
    double mttf = 0.0;
    
    for (int i = 0; i < times.size() - 1; ++i) {
        double dt = times(i + 1) - times(i);
        mttf += dt * (reliability(i) + reliability(i + 1)) / 2.0;
    }

    return mttf;
}

void MarkovModel::buildTransitionMatrix() {
    Q = Eigen::MatrixXd::Zero(numStates, numStates);
    
    System system(params);
    const int totalA = params.NA + params.RA;
    const int totalB = params.NB + params.RB;

    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int currentState = system.stateToIndex(a, b);

            if (a >= 1 && b >= params.NB) {
                double rateA = a * params.lambdaA;
                double rateB = b * params.lambdaB;
                double totalRate = rateA + rateB;

                if (a > 0) {
                    int nextState = system.stateToIndex(a - 1, b);
                    Q(currentState, nextState) = rateA;
                }
                
                if (b > 0) {
                    int nextState = system.stateToIndex(a, b - 1);
                    Q(currentState, nextState) = rateB;
                }
                
                Q(currentState, currentState) = -totalRate;
            }
        }
    }
}
