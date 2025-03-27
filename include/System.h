#pragma once
#include "SystemParams.h"
#include <utility>

class System {
private:
    SystemParams params;
    int workingA;  // Текущее количество исправных устройств A
    int workingB;  // Текущее количество исправных устройств B

public:
    System(const SystemParams& params);
    
    // Проверяет, работает ли система
    bool isOperational() const;
    
    // Обновляет состояние системы после отказа устройства
    void deviceFailure(char deviceType);
    
    // Получает текущее состояние (кол-во исправных A, кол-во исправных B)
    std::pair<int, int> getCurrentState() const;
    
    // Конвертирует состояние (a, b) в индекс состояния
    int stateToIndex(int a, int b) const;
    
    // Конвертирует индекс состояния в пару (a, b)
    std::pair<int, int> indexToState(int index) const;
    
    // Возвращает общее число возможных состояний
    int getTotalStates() const;

    const SystemParams& getParams() const;
};
