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
        // Вычисляем интенсивности переходов из текущего состояния
        double lambdaA = workingA * params.lambdaA;  // Интенсивность отказа устройства A
        double lambdaB = workingB * params.lambdaB;  // Интенсивность отказа устройства B
        double lambdaRepair = 0.0;  // Интенсивность ремонта
        
        // Определяем, какое устройство ремонтируется в данный момент
        int repairingA = (params.NA + params.RA) - workingA;
        int repairingB = (params.NB + params.RB) - workingB;
        
        // Если есть устройства для ремонта, определяем интенсивность ремонта
        if (repairingA > 0 || repairingB > 0) {
            // Определяем, какое устройство ремонтируется
            char deviceToRepair = '0';
            if (repairingA > repairingB) {
                deviceToRepair = 'A';
            } else if (repairingB > repairingA) {
                deviceToRepair = 'B';
            } else if (repairingA > 0) {
                deviceToRepair = (params.lambdaA >= params.lambdaB) ? 'A' : 'B';
            }
            
            // Устанавливаем интенсивность ремонта
            if (deviceToRepair != '0') {
                lambdaRepair = params.lambdaS;
                repairStatus = (deviceToRepair == 'A') ? 1 : 2;
            }
        } else {
            repairStatus = 0;
        }

        double totalRate = lambdaA + lambdaB + lambdaRepair;

        if (totalRate <= 0.0) {
            break;
        }

        // Генерируем время до следующего события
        double timeToNext = expDist(rng) / totalRate;
        t += timeToNext;

        if (t > simulationTime) {
            break;
        }

        // Определяем, какое событие произошло
        double randomValue = uniformDist(rng);
        double cumulativeProb = lambdaA / totalRate;

        if (randomValue < cumulativeProb) {
            // Отказ устройства A
            if (workingA > 0) {
                workingA--;
            }
        } else {
            cumulativeProb += lambdaB / totalRate;
            if (randomValue < cumulativeProb) {
                // Отказ устройства B
                if (workingB > 0) {
                    workingB--;
                }
            } else {
                // Ремонт устройства
                if (repairStatus == 1 && repairingA > 0) {
                    workingA++;
                } else if (repairStatus == 2 && repairingB > 0) {
                    workingB++;
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

    // Планируем начальные события отказов для всех работающих устройств
    for (int i = 0; i < workingA; ++i) {
        double failureTime = std::exponential_distribution<double>(params.lambdaA)(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_A));
    }

    for (int i = 0; i < workingB; ++i) {
        double failureTime = std::exponential_distribution<double>(params.lambdaB)(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_B));
    }

    // Планируем событие окончания симуляции
    events.push(Event(simulationTime, EventType::SIMULATION_END));

    double currentTime = 0.0;

    while (!events.empty()) {
        Event nextEvent = events.top();
        events.pop();

        currentTime = nextEvent.time;

        if (nextEvent.type == EventType::SIMULATION_END || currentTime > simulationTime) {
            break;
        }

        // Определяем, какое устройство ремонтируется в данный момент
        int repairingA = (params.NA + params.RA) - workingA;
        int repairingB = (params.NB + params.RB) - workingB;
        
        // Если есть устройства для ремонта, определяем, какое ремонтируется
        if (repairingA > 0 || repairingB > 0) {
            if (repairingA > repairingB) {
                repairStatus = 1;  // Ремонт устройства A
            } else if (repairingB > repairingA) {
                repairStatus = 2;  // Ремонт устройства B
            } else if (repairingA > 0) {
                repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
            }
            
            // Если еще не запланировано событие ремонта, планируем его
            bool repairEventExists = false;
            std::priority_queue<Event, std::vector<Event>, std::greater<Event>> tempEvents;
            
            while (!events.empty()) {
                Event event = events.top();
                events.pop();
                
                if (event.type == EventType::DEVICE_REPAIR) {
                    repairEventExists = true;
                }
                
                tempEvents.push(event);
            }
            
            events = tempEvents;
            
            if (!repairEventExists && repairStatus > 0) {
                double repairTime = currentTime + std::exponential_distribution<double>(params.lambdaS)(rng);
                events.push(Event(repairTime, EventType::DEVICE_REPAIR));
            }
        } else {
            repairStatus = 0;
        }

        switch (nextEvent.type) {
        case EventType::DEVICE_FAILURE_A:
            if (workingA > 0) {
                workingA--;
                
                // Планируем следующий отказ для оставшихся устройств A
                if (workingA > 0) {
                    double nextFailureTime = currentTime + std::exponential_distribution<double>(params.lambdaA)(rng);
                    events.push(Event(nextFailureTime, EventType::DEVICE_FAILURE_A));
                }
                
                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_FAILURE_B:
            if (workingB > 0) {
                workingB--;
                
                // Планируем следующий отказ для оставшихся устройств B
                if (workingB > 0) {
                    double nextFailureTime = currentTime + std::exponential_distribution<double>(params.lambdaB)(rng);
                    events.push(Event(nextFailureTime, EventType::DEVICE_FAILURE_B));
                }
                
                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_REPAIR:
            {
                if (repairStatus == 1 && repairingA > 0) {
                    workingA++;
                    
                    // Планируем отказ для отремонтированного устройства A
                    double nextFailureTime = currentTime + std::exponential_distribution<double>(params.lambdaA)(rng);
                    events.push(Event(nextFailureTime, EventType::DEVICE_FAILURE_A));
                    
                } else if (repairStatus == 2 && repairingB > 0) {
                    workingB++;
                    
                    // Планируем отказ для отремонтированного устройства B
                    double nextFailureTime = currentTime + std::exponential_distribution<double>(params.lambdaB)(rng);
                    events.push(Event(nextFailureTime, EventType::DEVICE_FAILURE_B));
                }
                
                // Обновляем количество ремонтируемых устройств
                repairingA = (params.NA + params.RA) - workingA;
                repairingB = (params.NB + params.RB) - workingB;
                
                // Если еще есть устройства для ремонта, планируем следующий ремонт
                if (repairingA > 0 || repairingB > 0) {
                    if (repairingA > repairingB) {
                        repairStatus = 1;
                    } else if (repairingB > repairingA) {
                        repairStatus = 2;
                    } else {
                        repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                    }
                    
                    double nextRepairTime = currentTime + std::exponential_distribution<double>(params.lambdaS)(rng);
                    events.push(Event(nextRepairTime, EventType::DEVICE_REPAIR));
                } else {
                    repairStatus = 0;
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
