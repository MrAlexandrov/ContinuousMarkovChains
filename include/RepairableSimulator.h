#pragma once
#include "RepairableSystem.h"
#include <random>
#include <vector>
#include <utility>
#include <queue>
#include <tuple>

// Типы событий для дискретно-событийного моделирования
enum class EventType {
    DEVICE_FAILURE_A,   // Отказ устройства типа A
    DEVICE_FAILURE_B,   // Отказ устройства типа B
    DEVICE_REPAIR,      // Завершение ремонта устройства
    SIMULATION_END      // Конец моделирования
};

// Структура события
struct Event {
    double time;        // Время наступления события
    EventType type;     // Тип события

    // Конструктор события
    Event(double t, EventType et) : time(t), type(et) {}

    // Оператор сравнения для очереди с приоритетом
    bool operator>(const Event& other) const {
        return time > other.time;
    }
};

class RepairableSimulator {
private:
    RepairableSystemParams params;
    std::mt19937 rng;  // Генератор случайных чисел

    // Моделирование непрерывной марковской цепи
    std::vector<std::tuple<double, int, int, int>> simulateMarkovChain(double simulationTime);

    // Моделирование дискретно-событийного процесса
    std::vector<std::tuple<double, int, int, int>> simulateDiscreteEvents(double simulationTime);

public:
    RepairableSimulator(const RepairableSystemParams& params);

    // Проводит имитационное моделирование в терминах непрерывных марковских цепей
    void runMarkovChainSimulation(double simulationTime);

    // Проводит имитационное моделирование в терминах дискретно-событийного моделирования
    void runDiscreteEventSimulation(double simulationTime);

    // Вычисляет статистику по результатам моделирования
    void calculateStatistics(const std::vector<std::tuple<double, int, int, int>>& trajectory);
};