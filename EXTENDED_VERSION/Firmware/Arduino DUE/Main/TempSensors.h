#ifndef _TEMP_SENSORS_H
#define _TEMP_SENSORS_H

#include "AbstractModule.h"
#include "DS18B20Query.h"
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS

typedef enum
{
  wmAutomatic, // автоматический режим управления окнами
  wmManual // мануальный режим управления окнами
  
} WindowWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  dirNOTHING,
  dirOPEN,
  dirCLOSE
  
} DIRECTION;
//--------------------------------------------------------------------------------------------------------------------------------------
class TempSensors;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool OnMyWay : 1; // флаг того, что фрамуга в процессе открытия/закрытия
  uint8_t Direction : 3; // направление, которое задали
  uint8_t Index : 4;
  
} WindowStateFlags;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
class WindowState
{
 private:
 
  uint32_t CurrentPosition; // текущая позиция фрамуги
  uint32_t TimerInterval; // сколько работать фрамуге?

  uint32_t _timer;
  uint32_t _fullCloseTimer;
  bool _waitFullClose;

  void SwitchRelays(uint8_t rel1State, uint8_t rel2State);

  uint8_t RelayChannel1;
  uint8_t RelayChannel2;

  WindowStateFlags flags;
  bool waitForChangePositionDone;
  uint16_t powerTimerDelay;

public:

  bool IsBusy() {return flags.OnMyWay;} // заняты или нет?

  bool IsInUninterruptedWay() // проверяет, можно ли прерывать движение окна
  {
    return IsBusy() && waitForChangePositionDone;
  }
  
  bool ChangePosition(uint32_t newPos, bool waitForChangePositionDone=false); // меняет позицию
  
  uint32_t GetCurrentPosition() {return CurrentPosition;}
  uint8_t GetCurrentPositionPercents();
  void ResetToMaxPosition();
  uint8_t GetDirection() {return flags.Direction;}

  uint32_t GetWorkTime();

  void UpdateState(); // обновляет состояние фрамуги
  
  void Setup(uint8_t index, uint8_t relayChannel1, uint8_t relayChannel2); // настраиваем перед пуском

  void Feedback(bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents,bool isFirstFeedback);


  WindowState() 
  {
    CurrentPosition = 0;
    flags.OnMyWay = false;
    TimerInterval = 0;
    RelayChannel1 = 0;
    RelayChannel2 = 0;
    flags.Direction = dirNOTHING;
    waitForChangePositionDone = false;
  }  
  
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
struct SeparateManageStruct
{
  WindowState* window;
  unsigned long targetPosition;
  bool waitFor;
  bool resetToMaxPositionBeforeStart;
};
////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
//--------------------------------------------------------------------------------------------------------------------------------------
class TempSensors : public AbstractModule // модуль опроса температурных датчиков и управления фрамугами
{
  private:
  
    uint16_t lastUpdateCall;

    WindowState Windows[SUPPORTED_WINDOWS];
    void SetupWindows();

    void WriteToMCP(); // пишем в сдвиговый регистр
    uint8_t* shiftRegisterData; // данные для сдвигового регистра
    uint8_t* lastShiftRegisterData; // последние данные, запиханные в сдвиговый регистр (чтоб не дёргать каждый раз, а только при изменениях)
    uint8_t shiftRegisterDataSize; // кол-во байт, хранящихся в массиве для сдвигового регистра


    uint8_t workMode; // текущий режим работы (автоматический или ручной)
    // добавляем сюда небольшое значение, когда меняется режим работы.
    // это нужно для того, чтобы нормально работали правила, когда
    // происходит смена режима работы.
    uint8_t smallSensorsChange; 

    BlinkModeInterop blinker;
    bool canUseBlinker;

////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
    Vector<SeparateManageStruct> separateManagerList;
    void updateSeparateManager();
    void addToSeparateList(WindowState* window,unsigned long targetPosition, bool waitFor,bool resetToMaxPositionBeforeStart);
////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

	uint32_t timer;
    
  public:
    TempSensors();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    uint8_t GetWorkMode() {return workMode;}
    void SetWorkMode(uint8_t m) {workMode = m;}

    void SaveChannelState(uint8_t channel, uint8_t state); // сохраняем состояние каналов

    bool IsAnyWindowOpen();
    bool IsWindowOpen(uint8_t windowNumber); // сообщает, открывается или открыто ли нужное окно

    void CloseAllWindows(); // вызывается менеджером обратной связи !!!
    void CloseWindow(uint8_t num);

    WindowState* GetWindow(uint8_t num) { return &(Windows[num]); }

    bool CanDriveWindows(); // возвращает true, если окнами можно управлять при помощи внешних команд (т.е. после старта они закрылись)

    // получена информация обратной связи по состоянию окна
    void WindowFeedback(uint8_t windowNumber, bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents, bool isFirstFeedback);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern TempSensors* WindowModule; // тут будет лежать указатель на класс диспетчера окон, чтобы его публичные методы можно было дёргать напрямую
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TEMP_SENSORS

#endif
