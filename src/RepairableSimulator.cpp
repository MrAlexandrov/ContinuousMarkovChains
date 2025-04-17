#include "RepairableSimulator.h"

#include <iostream>
#include <iomanip>
#include <cmath>
#include <queue>
#include <fstream>
#include <map>

RepairableSimulator::RepairableSimulator(const RepairableSystemParams& params) : params(params) {
    std::random_device rd;
    rng = std::mt19937(rd());
}

std::vector<std::tuple<double, int, int, int>> RepairableSimulator::simulateMarkovChain(double simulationTime) {
    std::vector<std::tuple<double, int, int, int>> trajectory;
    RepairableSystem system(params);

    int workingA = params.NA + params.RA;
    int workingB = params.NB + params.RB;
    int repairStatus = 0;

    double t = 0.0;

    trajectory.push_back(std::make_tuple(t, workingA, workingB, repairStatus));

    std::exponential_distribution<double> expDist(1.0);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    while (t < simulationTime) {
        int repairingA = (params.NA + params.RA) - workingA;
        int repairingB = (params.NB + params.RB) - workingB;

        double lambdaA = workingA * params.lambdaA;
        double lambdaB = workingB * params.lambdaB;
        double lambdaRepair = (repairStatus > 0) ? params.lambdaS : 0.0;

        double totalRate = lambdaA + lambdaB + lambdaRepair;

        if (totalRate <= 0.0) {
            break;
        }

        double timeToNext = expDist(rng) / totalRate;
        t += timeToNext;

        if (t > simulationTime) {
            break;
        }

        double randomValue = uniformDist(rng);
        double cumulativeProb = lambdaA / totalRate;

        if (randomValue < cumulativeProb) {
            if (workingA > 0) {
                workingA--;
                if (repairStatus == 0) {
                    repairStatus = 1;
                }
            }
        } else {
            cumulativeProb += lambdaB / totalRate;
            if (randomValue < cumulativeProb) {
                if (workingB > 0) {
                    workingB--;

                    if (repairStatus == 0) {
                        repairStatus = 2;
                    }
                }
            } else {
                if (repairStatus == 1) {
                    workingA++;
                    repairStatus = 0;

                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > repairingB) {
                        repairStatus = 1;
                    } else if (repairingB > repairingA) {
                        repairStatus = 2;
                    } else if (repairingA > 0) {
                        repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                    }
                } else if (repairStatus == 2) {
                    workingB++;
                    repairStatus = 0;

                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > repairingB) {
                        repairStatus = 1;
                    } else if (repairingB > repairingA) {
                        repairStatus = 2;
                    } else if (repairingA > 0) {
                        repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                    }
                }
            }
        }

        trajectory.push_back(std::make_tuple(t, workingA, workingB, repairStatus));
    }

    return trajectory;
}

std::vector<std::tuple<double, int, int, int>> RepairableSimulator::simulateDiscreteEvents(double simulationTime) {
    std::vector<std::tuple<double, int, int, int>> trajectory;

    int workingA = params.NA + params.RA;
    int workingB = params.NB + params.RB;
    int repairStatus = 0;

    trajectory.push_back(std::make_tuple(0.0, workingA, workingB, repairStatus));

    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events;

    std::exponential_distribution<double> expDistA(params.lambdaA);
    std::exponential_distribution<double> expDistB(params.lambdaB);
    std::exponential_distribution<double> expDistRepair(params.lambdaS);

    for (int i = 0; i < workingA; ++i) {
        double failureTime = expDistA(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_A));
    }

    for (int i = 0; i < workingB; ++i) {
        double failureTime = expDistB(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_B));
    }

    events.push(Event(simulationTime, EventType::SIMULATION_END));

    double currentTime = 0.0;

    while (!events.empty()) {
        Event nextEvent = events.top();
        events.pop();

        currentTime = nextEvent.time;

        if (nextEvent.type == EventType::SIMULATION_END || currentTime > simulationTime) {
            break;
        }

        switch (nextEvent.type) {
        case EventType::DEVICE_FAILURE_A:
            if (workingA > 0) {
                workingA--;

                if (repairStatus == 0) {
                    repairStatus = 1;
                    double repairTime = currentTime + expDistRepair(rng);
                    events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                }

                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_FAILURE_B:
            if (workingB > 0) {
                workingB--;

                if (repairStatus == 0) {
                    repairStatus = 2;
                    double repairTime = currentTime + expDistRepair(rng);
                    events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                }

                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_REPAIR:
            {
                if (repairStatus == 1) {
                    workingA++;
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > 0 || repairingB > 0) {
                        if (repairingA > repairingB) {
                            repairStatus = 1;
                        } else if (repairingB > repairingA) {
                            repairStatus = 2;
                        } else {
                            repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                        }

                        double repairTime = currentTime + expDistRepair(rng);
                        events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                    } else {
                        repairStatus = 0;
                    }
                } else if (repairStatus == 2) {
                    workingB++;
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > 0 || repairingB > 0) {
                        if (repairingA > repairingB) {
                            repairStatus = 1;
                        } else if (repairingB > repairingA) {
                            repairStatus = 2;
                        } else {
                            repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                        }

                        double repairTime = currentTime + expDistRepair(rng);
                        events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                    } else {
                        repairStatus = 0;
                    }
                }

                if (repairStatus == 1) {
                    double failureTime = currentTime + expDistA(rng);
                    events.push(Event(failureTime, EventType::DEVICE_FAILURE_A));
                } else if (repairStatus == 2) {
                    double failureTime = currentTime + expDistB(rng);
                    events.push(Event(failureTime, EventType::DEVICE_FAILURE_B));
                }

                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        default:
            break;
        }
    }

    return trajectory;
}

void RepairableSimulator::runMarkovChainSimulation(double simulationTime) {
    std::cout << "Запуск моделирования непрерывной марковской цепи..." << "\n";

    auto trajectory = simulateMarkovChain(simulationTime);

    std::cout << "Моделирование завершено. Количество точек: " << trajectory.size() << "\n";

    calculateStatistics(trajectory);

    std::ofstream outputFile("repairable_markov_chain_trajectory.dat");
    outputFile << "# Time WorkingA WorkingB RepairStatus\n";

    for (const auto& [time, a, b, r] : trajectory) {
        outputFile << time << " " << a << " " << b << " " << r << "\n";
    }

    outputFile.close();

    std::cout << "Результаты сохранены в файл repairable_markov_chain_trajectory.dat" << "\n";
}

void RepairableSimulator::runDiscreteEventSimulation(double simulationTime) {
    std::cout << "Запуск дискретно-событийного моделирования..." << "\n";

    auto trajectory = simulateDiscreteEvents(simulationTime);

    std::cout << "Моделирование завершено. Количество точек: " << trajectory.size() << "\n";

    calculateStatistics(trajectory);

    std::ofstream outputFile("repairable_discrete_event_trajectory.dat");
    outputFile << "# Time WorkingA WorkingB RepairStatus\n";

    for (const auto& [time, a, b, r] : trajectory) {
        outputFile << time << " " << a << " " << b << " " << r << "\n";
    }

    outputFile.close();

    std::cout << "Результаты сохранены в файл repairable_discrete_event_trajectory.dat" << "\n";
}

void RepairableSimulator::calculateStatistics(const std::vector<std::tuple<double, int, int, int>>& trajectory) {
    if (trajectory.empty()) {
        std::cout << "Нет данных для анализа." << "\n";
        return;
    }

    RepairableSystem system(params);

    double totalTime = std::get<0>(trajectory.back());

    std::map<std::tuple<int, int, int>, double> stateDurations;

    double prevTime = 0.0;
    int prevA = std::get<1>(trajectory[0]);
    int prevB = std::get<2>(trajectory[0]);
    int prevR = std::get<3>(trajectory[0]);

    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& [time, a, b, r] = trajectory[i];

        std::tuple<int, int, int> state = {prevA, prevB, prevR};
        stateDurations[state] += (time - prevTime);

        prevTime = time;
        prevA = a;
        prevB = b;
        prevR = r;
    }

    std::map<std::tuple<int, int, int>, double> stateProbs;
    for (const auto& [state, duration] : stateDurations) {
        stateProbs[state] = duration / totalTime;
    }

    double failureProb = 0.0;
    double avgWorkingA = 0.0;
    double avgWorkingB = 0.0;
    double repairUtilization = 0.0;

    for (const auto& [state, prob] : stateProbs) {
        const auto& [a, b, r] = state;

        if (a < 1 || b < params.NB) {
            failureProb += prob;
        }

        avgWorkingA += a * prob;
        avgWorkingB += b * prob;

        if (r > 0) {
            repairUtilization += prob;
        }
    }

    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Статистика по результатам моделирования:" << "\n";
    std::cout << "Вероятность отказа системы: " << failureProb << "\n";
    std::cout << "Среднее число готовых устройств типа A: " << avgWorkingA << "\n";
    std::cout << "Среднее число готовых устройств типа B: " << avgWorkingB << "\n";
    std::cout << "Коэффициент загрузки ремонтной службы: " << repairUtilization << "\n";
}
