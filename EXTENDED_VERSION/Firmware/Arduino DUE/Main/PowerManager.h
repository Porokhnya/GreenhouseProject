#pragma once

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class PowerManagerClass
{
  public:

    PowerManagerClass();
    void begin();
    void update();

    uint16_t WindowWantMove(uint8_t window); // вызывается фрамугой перед сменой позиции, возвращает кол-во миллисекунд, которое надо подождать фрамуге перед началом движения
    void WindowMoveDone(uint8_t window); // вызывается фрамугой после окончания смены позиции

    uint16_t WateringOn(uint8_t channel); // канал полива включается
    void WateringOff(uint8_t channel); // канал полива выключается

    uint16_t PumpOn(uint8_t pump); // насос полива включается
    void PumpOff(uint8_t pump); // насос полива выключается

    uint16_t DoorWantMove(uint8_t door); // дверь хочет начать движение
    void DoorMoveDone(uint8_t door); // дверь закончила движение

  private:    

    bool bOn, bOnTimer, bOffTimer;
    uint16_t timer, delayCounter;
    uint16_t windowsState;
    uint32_t offTimer;

    void out(bool state); // пишет в выход нужный уровень
    bool isNoConsumers(); // возвращает true, если нет активных потребителей питания
    void turnOn();
    void turnOff();

    bool pump1On, pump2On;
    uint16_t wateringState;

    uint16_t doorsState;
    
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern PowerManagerClass PowerManager;
