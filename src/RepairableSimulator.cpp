#include "RepairableSimulator.h"
#include "GnuplotPlotter.h"
#include <algorithm>
#include <iostream>
#include <iomanip>
#include <cmath>
#include <functional>
#include <fstream>
#include <map>

RepairableSimulator::RepairableSimulator(const RepairableSystemParams& params) : params(params) {
    std::random_device rd;
    rng = std::mt19937(rd());
}

std::vector<std::tuple<double, int, int, int>> RepairableSimulator::simulateMarkovChain(double simulationTime) {
    std::vector<std::tuple<double, int, int, int>> trajectory;
    RepairableSystem system(params);

    // Начальное состояние - все устройства работают, нет ремонта
    int workingA = params.NA + params.RA;
    int workingB = params.NB + params.RB;
    int repairStatus = 0;  // 0 - нет ремонта, 1 - ремонт A, 2 - ремонт B

    double t = 0.0;

    // Записываем начальное состояние
    trajectory.push_back(std::make_tuple(t, workingA, workingB, repairStatus));

    std::exponential_distribution<double> expDist(1.0);
    std::uniform_real_distribution<double> uniformDist(0.0, 1.0);

    while (t < simulationTime) {
        // Количество устройств в ремонте
        int repairingA = (params.NA + params.RA) - workingA;
        int repairingB = (params.NB + params.RB) - workingB;

        // Рассчитываем интенсивности отказов и ремонта
        double lambdaA = workingA * params.lambdaA;
        double lambdaB = workingB * params.lambdaB;
        double lambdaRepair = (repairStatus > 0) ? params.lambdaS : 0.0;

        double totalRate = lambdaA + lambdaB + lambdaRepair;

        // Если нет возможных переходов, завершаем моделирование
        if (totalRate <= 0.0) {
            break;
        }

        // Время до следующего события
        double timeToNext = expDist(rng) / totalRate;
        t += timeToNext;

        if (t > simulationTime) {
            break;
        }

        // Определяем тип события
        double randomValue = uniformDist(rng);
        double cumulativeProb = lambdaA / totalRate;

        if (randomValue < cumulativeProb) {
            // Отказ устройства типа A
            if (workingA > 0) {
                workingA--;

                // Если ремонт не идет, начинаем ремонт A
                if (repairStatus == 0) {
                    repairStatus = 1;
                }
            }
        } else {
            cumulativeProb += lambdaB / totalRate;
            if (randomValue < cumulativeProb) {
                // Отказ устройства типа B
                if (workingB > 0) {
                    workingB--;

                    // Если ремонт не идет, начинаем ремонт B
                    if (repairStatus == 0) {
                        repairStatus = 2;
                    }
                }
            } else {
                // Ремонт устройства
                if (repairStatus == 1) {
                    workingA++;
                    repairStatus = 0;  // Ремонт завершен

                    // Проверяем, что ремонтировать дальше
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > repairingB) {
                        repairStatus = 1;  // Продолжаем ремонтировать A
                    } else if (repairingB > repairingA) {
                        repairStatus = 2;  // Переключаемся на ремонт B
                    } else if (repairingA > 0) {
                        // Если поровну, выбираем по интенсивности
                        repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                    }
                } else if (repairStatus == 2) {
                    workingB++;
                    repairStatus = 0;  // Ремонт завершен

                    // Проверяем, что ремонтировать дальше
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > repairingB) {
                        repairStatus = 1;  // Переключаемся на ремонт A
                    } else if (repairingB > repairingA) {
                        repairStatus = 2;  // Продолжаем ремонтировать B
                    } else if (repairingA > 0) {
                        // Если поровну, выбираем по интенсивности
                        repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                    }
                }
            }
        }

        // Записываем новое состояние
        trajectory.push_back(std::make_tuple(t, workingA, workingB, repairStatus));
    }

    return trajectory;
}

std::vector<std::tuple<double, int, int, int>> RepairableSimulator::simulateDiscreteEvents(double simulationTime) {
    std::vector<std::tuple<double, int, int, int>> trajectory;

    // Начальное состояние - все устройства работают, нет ремонта
    int workingA = params.NA + params.RA;
    int workingB = params.NB + params.RB;
    int repairStatus = 0;  // 0 - нет ремонта, 1 - ремонт A, 2 - ремонт B

    // Записываем начальное состояние
    trajectory.push_back(std::make_tuple(0.0, workingA, workingB, repairStatus));

    // Очередь событий с приоритетом (наименьшее время имеет наивысший приоритет)
    std::priority_queue<Event, std::vector<Event>, std::greater<Event>> events;

    // Генераторы случайных чисел
    std::exponential_distribution<double> expDistA(params.lambdaA);
    std::exponential_distribution<double> expDistB(params.lambdaB);
    std::exponential_distribution<double> expDistRepair(params.lambdaS);

    // Планируем начальные отказы для всех устройств
    for (int i = 0; i < workingA; ++i) {
        double failureTime = expDistA(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_A));
    }

    for (int i = 0; i < workingB; ++i) {
        double failureTime = expDistB(rng);
        events.push(Event(failureTime, EventType::DEVICE_FAILURE_B));
    }

    // Добавляем событие окончания моделирования
    events.push(Event(simulationTime, EventType::SIMULATION_END));

    // Текущее время
    double currentTime = 0.0;

    // Основной цикл моделирования
    while (!events.empty()) {
        // Извлекаем следующее событие
        Event nextEvent = events.top();
        events.pop();

        // Обновляем текущее время
        currentTime = nextEvent.time;

        // Если время вышло или система отказала, выходим из цикла
        if (nextEvent.type == EventType::SIMULATION_END || currentTime > simulationTime) {
            break;
        }

        // Обрабатываем событие
        switch (nextEvent.type) {
        case EventType::DEVICE_FAILURE_A:
            if (workingA > 0) {
                workingA--;

                // Если не было ремонта, начинаем ремонт A
                if (repairStatus == 0) {
                    repairStatus = 1;
                    // Планируем завершение ремонта
                    double repairTime = currentTime + expDistRepair(rng);
                    events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                }

                // Записываем новое состояние
                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_FAILURE_B:
            if (workingB > 0) {
                workingB--;

                // Если не было ремонта, начинаем ремонт B
                if (repairStatus == 0) {
                    repairStatus = 2;
                    // Планируем завершение ремонта
                    double repairTime = currentTime + expDistRepair(rng);
                    events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                }

                // Записываем новое состояние
                trajectory.push_back(std::make_tuple(currentTime, workingA, workingB, repairStatus));
            }
            break;

        case EventType::DEVICE_REPAIR:
            {
                // Определяем, какое устройство отремонтировано
                if (repairStatus == 1) {
                    workingA++;
                    // Если есть еще устройства в ремонте, планируем следующий ремонт
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > 0 || repairingB > 0) {
                        // Выбираем, что ремонтировать дальше
                        if (repairingA > repairingB) {
                            repairStatus = 1;  // Продолжаем ремонтировать A
                        } else if (repairingB > repairingA) {
                            repairStatus = 2;  // Переключаемся на ремонт B
                        } else {
                            // Если поровну, выбираем по интенсивности
                            repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                        }

                        // Планируем завершение ремонта
                        double repairTime = currentTime + expDistRepair(rng);
                        events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                    } else {
                        repairStatus = 0;  // Нет устройств для ремонта
                    }
                } else if (repairStatus == 2) {
                    workingB++;
                    // Если есть еще устройства в ремонте, планируем следующий ремонт
                    int repairingA = (params.NA + params.RA) - workingA;
                    int repairingB = (params.NB + params.RB) - workingB;

                    if (repairingA > 0 || repairingB > 0) {
                        // Выбираем, что ремонтировать дальше
                        if (repairingA > repairingB) {
                            repairStatus = 1;  // Переключаемся на ремонт A
                        } else if (repairingB > repairingA) {
                            repairStatus = 2;  // Продолжаем ремонтировать B
                        } else {
                            // Если поровну, выбираем по интенсивности
                            repairStatus = (params.lambdaA >= params.lambdaB) ? 1 : 2;
                        }

                        // Планируем завершение ремонта
                        double repairTime = currentTime + expDistRepair(rng);
                        events.push(Event(repairTime, EventType::DEVICE_REPAIR));
                    } else {
                        repairStatus = 0;  // Нет устройств для ремонта
                    }
                }

                // Планируем отказ отремонтированного устройства
                if (repairStatus == 1) {
                    double failureTime = currentTime + expDistA(rng);
                    events.push(Event(failureTime, EventType::DEVICE_FAILURE_A));
                } else if (repairStatus == 2) {
                    double failureTime = currentTime + expDistB(rng);
                    events.push(Event(failureTime, EventType::DEVICE_FAILURE_B));
                }

                // Записываем новое состояние
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
    std::cout << "Запуск моделирования непрерывной марковской цепи..." << std::endl;

    auto trajectory = simulateMarkovChain(simulationTime);

    std::cout << "Моделирование завершено. Количество точек: " << trajectory.size() << std::endl;

    // Вычисляем статистику
    calculateStatistics(trajectory);

    // Сохраняем траекторию в файл для построения графика
    std::ofstream outputFile("repairable_markov_chain_trajectory.dat");
    outputFile << "# Time WorkingA WorkingB RepairStatus\n";

    for (const auto& [time, a, b, r] : trajectory) {
        outputFile << time << " " << a << " " << b << " " << r << "\n";
    }

    outputFile.close();

    std::cout << "Результаты сохранены в файл repairable_markov_chain_trajectory.dat" << std::endl;
}

void RepairableSimulator::runDiscreteEventSimulation(double simulationTime) {
    std::cout << "Запуск дискретно-событийного моделирования..." << std::endl;

    auto trajectory = simulateDiscreteEvents(simulationTime);

    std::cout << "Моделирование завершено. Количество точек: " << trajectory.size() << std::endl;

    // Вычисляем статистику
    calculateStatistics(trajectory);

    // Сохраняем траекторию в файл для построения графика
    std::ofstream outputFile("repairable_discrete_event_trajectory.dat");
    outputFile << "# Time WorkingA WorkingB RepairStatus\n";

    for (const auto& [time, a, b, r] : trajectory) {
        outputFile << time << " " << a << " " << b << " " << r << "\n";
    }

    outputFile.close();

    std::cout << "Результаты сохранены в файл repairable_discrete_event_trajectory.dat" << std::endl;
}

void RepairableSimulator::calculateStatistics(const std::vector<std::tuple<double, int, int, int>>& trajectory) {
    if (trajectory.empty()) {
        std::cout << "Нет данных для анализа." << std::endl;
        return;
    }

    RepairableSystem system(params);

    // Общее время моделирования
    double totalTime = std::get<0>(trajectory.back());

    // Статистика по состояниям
    std::map<std::tuple<int, int, int>, double> stateDurations;

    // Для первого состояния время начинается с 0
    double prevTime = 0.0;
    int prevA = std::get<1>(trajectory[0]);
    int prevB = std::get<2>(trajectory[0]);
    int prevR = std::get<3>(trajectory[0]);

    // Проходим по всем точкам траектории, кроме первой
    for (size_t i = 1; i < trajectory.size(); ++i) {
        const auto& [time, a, b, r] = trajectory[i];

        // Добавляем время к предыдущему состоянию
        std::tuple<int, int, int> state = {prevA, prevB, prevR};
        stateDurations[state] += (time - prevTime);

        // Обновляем для следующей итерации
        prevTime = time;
        prevA = a;
        prevB = b;
        prevR = r;
    }

    // Вычисляем вероятности состояний
    std::map<std::tuple<int, int, int>, double> stateProbs;
    for (const auto& [state, duration] : stateDurations) {
        stateProbs[state] = duration / totalTime;
    }

    // Вычисляем характеристики системы
    double failureProb = 0.0;
    double avgWorkingA = 0.0;
    double avgWorkingB = 0.0;
    double repairUtilization = 0.0;

    for (const auto& [state, prob] : stateProbs) {
        const auto& [a, b, r] = state;

        // Вероятность отказа
        if (a < 1 || b < params.NB) {
            failureProb += prob;
        }

        // Среднее число готовых устройств
        avgWorkingA += a * prob;
        avgWorkingB += b * prob;

        // Коэффициент загрузки ремонтной службы
        if (r > 0) {
            repairUtilization += prob;
        }
    }

    // Вывод статистики
    std::cout << std::fixed << std::setprecision(6);
    std::cout << "Статистика по результатам моделирования:" << std::endl;
    std::cout << "Вероятность отказа системы: " << failureProb << std::endl;
    std::cout << "Среднее число готовых устройств типа A: " << avgWorkingA << std::endl;
    std::cout << "Среднее число готовых устройств типа B: " << avgWorkingB << std::endl;
    std::cout << "Коэффициент загрузки ремонтной службы: " << repairUtilization << std::endl;
}