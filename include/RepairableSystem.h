#pragma once

#include "RepairableSystemParams.h"

#include <utility>
#include <vector>

class RepairableSystem {
private:
    RepairableSystemParams params;
    int workingA;     // Текущее количество исправных устройств A
    int workingB;     // Текущее количество исправных устройств B
    int repairingA;   // Количество устройств A, находящихся в ремонте
    int repairingB;   // Количество устройств B, находящихся в ремонте
    bool repairInProgress; // Флаг, указывающий, идет ли ремонт в данный момент

public:
    RepairableSystem(const RepairableSystemParams& params);

    // Проверяет, работает ли система
    bool isOperational() const;

    // Обновляет состояние системы после отказа устройства
    void deviceFailure(char deviceType);

    // Обновляет состояние системы после ремонта устройства
    void deviceRepair();

    // Проверяет, какое устройство следует ремонтировать (A или B)
    char getDeviceToRepair() const;

    // Проверяет, есть ли устройства для ремонта
    bool hasDevicesToRepair() const;

    // Получает текущее состояние (кол-во исправных A, кол-во исправных B, ремонтируемое устройство)
    // Для ремонтируемого устройства: 0 - нет ремонта, 1 - ремонт A, 2 - ремонт B
    std::tuple<int, int, int> getCurrentState() const;

    // Конвертирует состояние (a, b, r) в индекс состояния
    // Конвертирует состояние (a, b, r) в индекс состояния
    int stateToIndex(int a, int b, int r) const;
    
    // Конвертирует состояние (a, b) в индекс для графа состояний (без учета ремонта)
    // Использует ту же логику, что и в System::stateToIndex для совместимости
    int stateToGraphIndex(int a, int b) const;

    // Конвертирует индекс состояния в кортеж (a, b, r)
    std::tuple<int, int, int> indexToState(int index) const;

    // Возвращает общее число возможных состояний
    int getTotalStates() const;

    // Возвращает параметры системы
    const RepairableSystemParams& getParams() const;

    // Возвращает максимальное количество устройств каждого типа
    std::pair<int, int> getMaxDevices() const;
};
