#include "RepairableMarkovModel.h"

#include <numeric>
#include <cmath>

RepairableMarkovModel::RepairableMarkovModel(const RepairableSystemParams& params)
    : params(params)
{
    RepairableSystem system(params);
    numStates = system.getTotalStates();

    Q = Eigen::MatrixXd::Zero(numStates, numStates);
    initialState = Eigen::VectorXd::Zero(numStates);

    auto [totalA, totalB] = system.getMaxDevices();
    initialState(system.stateToIndex(totalA, totalB, 0)) = 1.0;

    buildTransitionMatrix();
}

void RepairableMarkovModel::buildTransitionMatrix() {
    Q = Eigen::MatrixXd::Zero(numStates, numStates);

    RepairableSystem system(params);
    const int totalA = params.NA + params.RA;
    const int totalB = params.NB + params.RB;

    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            for (int r = 0; r < 3; ++r) {
                int currentState = system.stateToIndex(a, b, r);

                if (currentState < 0 || currentState >= numStates) {
                    continue;
                }

                double totalRate = 0.0;

                // Only add failure transitions if the system is operational
                if (a >= 1 && b >= params.NB) {
                    // Отказ устройства A
                    if (a > 0) {
                        double rateA = getFailureRate(a, b, 'A');
                        if (rateA > 0) {
                            int nextA = a - 1;
                            int nextR = (r == 0) ? 1 : r;

                            if (nextA >= 0 && nextA <= totalA) {
                                int nextState = system.stateToIndex(nextA, b, nextR);

                                if (nextState >= 0 && nextState < numStates) {
                                    Q(currentState, nextState) = rateA;
                                    totalRate += rateA;
                                }
                            }
                        }
                    }

                    if (b > 0) {
                        double rateB = getFailureRate(a, b, 'B');
                        if (rateB > 0) {
                            int nextB = b - 1;
                            int nextR = (r == 0) ? 2 : r;

                            if (nextB >= 0 && nextB <= totalB) {
                                int nextState = system.stateToIndex(a, nextB, nextR);

                                if (nextState >= 0 && nextState < numStates) {
                                    Q(currentState, nextState) = rateB;
                                    totalRate += rateB;
                                }
                            }
                        }
                    }
                }

                // Repair transitions are always possible if there are devices to repair
                if (r > 0) {
                    double repairRate = getRepairRate(a, b, r);
                    int nextA = a + (r == 1 ? 1 : 0);
                    int nextB = b + (r == 2 ? 1 : 0);

                    if (nextA <= totalA && nextB <= totalB) {
                        int nextR = 0;
                        int repairingA = totalA - nextA;
                        int repairingB = totalB - nextB;

                        if (repairingA > 0 || repairingB > 0) {
                            if (repairingA > repairingB) {
                                nextR = 1;
                            } else if (repairingB > repairingA) {
                                nextR = 2;
                            } else if (repairingA > 0) {
                                nextR = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                            }
                        }

                        int nextState = system.stateToIndex(nextA, nextB, nextR);

                        if (nextState >= 0 && nextState < numStates) {
                            Q(currentState, nextState) = repairRate;
                            totalRate += repairRate;
                        }
                    }
                }

                Q(currentState, currentState) = -totalRate;
            }
        }
    }
}

double RepairableMarkovModel::getFailureRate(int a, int b, char deviceType) const {
    // Only exactly NA devices of type A and NB devices of type B contribute to failure rate
    if (deviceType == 'A') {
        // If we have more than NA devices, only NA devices contribute to failure rate
        if (a <= params.NA) {
            return a * params.lambdaA;
        } else {
            return params.NA * params.lambdaA;
        }
    } else {
        // If we have more than NB devices, only NB devices contribute to failure rate
        if (b <= params.NB) {
            return b * params.lambdaB;
        } else {
            return params.NB * params.lambdaB;
        }
    }
}

double RepairableMarkovModel::getRepairRate(int a, int b, int r) const {
    return params.lambdaS;
}

Eigen::VectorXd RepairableMarkovModel::solveSteadyStateEquations() {
    // Решаем систему линейных алгебраических уравнений для стационарного режима
    // Q^T * p = 0, sum(p) = 1

    // Матрица для решения системы уравнений
    Eigen::MatrixXd A = Q.transpose();

    // Заменяем последнюю строку на условие нормировки
    for (int i = 0; i < A.cols(); ++i) {
        A(A.rows() - 1, i) = 1.0;
    }

    // Вектор правой части
    Eigen::VectorXd b = Eigen::VectorXd::Zero(numStates);
    b(numStates - 1) = 1.0;

    // Решаем систему уравнений
    Eigen::VectorXd steadyStateProbs = A.colPivHouseholderQr().solve(b);

    return steadyStateProbs;
}

std::pair<Eigen::VectorXd, Eigen::MatrixXd> RepairableMarkovModel::solveKolmogorovEquations(double tMax, int steps) {
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

double RepairableMarkovModel::estimateTransientTime(const Eigen::VectorXd& steadyStateProbs, double tolerance) {
    // Оценка времени переходного процесса - момент, когда евклидова норма вектора невязки
    // становится меньше tolerance от евклидовой нормы предельного вектора

    // Вычисляем евклидову норму предельного вектора
    double steadyStateNorm = steadyStateProbs.norm();

    // Предварительное моделирование для оценки времени
    double initialTMax = 100.0;  // Начальное время моделирования
    int steps = 1000;

    auto [times, probabilities] = solveKolmogorovEquations(initialTMax, steps);

    for (int i = 0; i < steps; ++i) {
        Eigen::VectorXd diff = probabilities.col(i) - steadyStateProbs;
        double diffNorm = diff.norm();

        if (diffNorm <= tolerance * steadyStateNorm) {
            return times(i);
        }
    }

    // Если не достигли нужной точности, возвращаем conservative estimate
    return initialTMax;
}

double RepairableMarkovModel::calculateFailureProbability(const Eigen::VectorXd& stateProbs) {
    RepairableSystem system(params);
    double failureProb = 0.0;

    for (int i = 0; i < numStates; ++i) {
        auto [a, b, r] = system.indexToState(i);
        if (a < 1 || b < params.NB) {  // Если состояние означает отказ системы
            failureProb += stateProbs(i);
        }
    }

    return failureProb;
}

std::pair<double, double> RepairableMarkovModel::calculateReadyDevices(const Eigen::VectorXd& stateProbs) {
    RepairableSystem system(params);
    double avgA = 0.0;
    double avgB = 0.0;

    for (int i = 0; i < numStates; ++i) {
        auto [a, b, r] = system.indexToState(i);
        avgA += a * stateProbs(i);
        avgB += b * stateProbs(i);
    }

    return {avgA, avgB};
}

double RepairableMarkovModel::calculateRepairUtilization(const Eigen::VectorXd& stateProbs) {
    RepairableSystem system(params);
    double repairUtilization = 0.0;

    for (int i = 0; i < numStates; ++i) {
        auto [a, b, r] = system.indexToState(i);
        if (r > 0) {  // Если ремонт идет
            repairUtilization += stateProbs(i);
        }
    }

    return repairUtilization;
}

Eigen::VectorXd RepairableMarkovModel::getOperationalStates() const {
    Eigen::VectorXd operationalStates = Eigen::VectorXd::Zero(numStates);
    RepairableSystem system(params);

    for (int i = 0; i < numStates; ++i) {
        auto [a, b, r] = system.indexToState(i);
        operationalStates(i) = (a >= 1 && b >= params.NB) ? 1.0 : 0.0;
    }

    return operationalStates;
}

Eigen::MatrixXd RepairableMarkovModel::buildGraphTransitionMatrix() const {
    RepairableSystem system(params);
    const int totalA = params.NA + params.RA;
    const int totalB = params.NB + params.RB;
    
    // Размер матрицы для графа состояний (без учета состояния ремонта)
    int graphStates = (totalA + 1) * (totalB + 1);
    Eigen::MatrixXd graphQ = Eigen::MatrixXd::Zero(graphStates, graphStates);
    
    // Для каждого состояния (a, b) в графе
    for (int a = 0; a <= totalA; ++a) {
        for (int b = 0; b <= totalB; ++b) {
            int currentGraphIndex = system.stateToGraphIndex(a, b);
            
            // Суммарная интенсивность выхода из состояния
            double totalOutRate = 0.0;
            
            // Only add failure transitions if the system is operational
            if (a >= 1 && b >= params.NB) {
                // Отказы устройств типа A (переход в состояние с меньшим a)
                if (a > 0) {
                    int nextGraphIndex = system.stateToGraphIndex(a - 1, b);
                    // Only exactly NA devices of type A contribute to failure rate
                    double rate = 0.0;
                    if (a <= params.NA) {
                        rate = a * params.lambdaA;
                    } else {
                        rate = params.NA * params.lambdaA;
                    }
                    
                    if (rate > 0) {
                        graphQ(currentGraphIndex, nextGraphIndex) += rate;
                        totalOutRate += rate;
                    }
                }
                
                // Отказы устройств типа B (переход в состояние с меньшим b)
                if (b > 0) {
                    int nextGraphIndex = system.stateToGraphIndex(a, b - 1);
                    // Only exactly NB devices of type B contribute to failure rate
                    double rate = 0.0;
                    if (b <= params.NB) {
                        rate = b * params.lambdaB;
                    } else {
                        rate = params.NB * params.lambdaB;
                    }
                    
                    if (rate > 0) {
                        graphQ(currentGraphIndex, nextGraphIndex) += rate;
                        totalOutRate += rate;
                    }
                }
            }
            
            // Ремонт устройств типа A (переход в состояние с большим a)
            if (a < totalA) {
                int nextGraphIndex = system.stateToGraphIndex(a + 1, b);
                
                // Проверяем, возможен ли ремонт A в этом состоянии
                int repairingA = totalA - a;
                int repairingB = totalB - b;
                
                if (repairingA > 0 && (repairingA > repairingB ||
                    (repairingA == repairingB && params.lambdaA >= params.lambdaB))) {
                    double rate = params.lambdaS;
                    graphQ(currentGraphIndex, nextGraphIndex) += rate;
                    totalOutRate += rate;
                }
            }
            
            // Ремонт устройств типа B (переход в состояние с большим b)
            if (b < totalB) {
                int nextGraphIndex = system.stateToGraphIndex(a, b + 1);
                
                // Проверяем, возможен ли ремонт B в этом состоянии
                int repairingA = totalA - a;
                int repairingB = totalB - b;
                
                if (repairingB > 0 && (repairingB > repairingA ||
                    (repairingB == repairingA && params.lambdaB > params.lambdaA))) {
                    double rate = params.lambdaS;
                    graphQ(currentGraphIndex, nextGraphIndex) += rate;
                    totalOutRate += rate;
                }
            }
            
            // Диагональный элемент - отрицательная сумма интенсивностей выхода
            graphQ(currentGraphIndex, currentGraphIndex) = -totalOutRate;
        }
    }
    
    return graphQ;
}