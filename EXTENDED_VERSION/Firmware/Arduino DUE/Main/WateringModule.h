#pragma once
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "Globals.h"
#include "InteropStream.h"
#include "RTCSupport.h"
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE

typedef enum
{
  wwmAutomatic, // в автоматическом режиме
  wwmManual // в ручном режиме
  
} WateringWorkMode; // режим работы полива
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t workMode: 2; // текущий режим работы модуля полива
  bool isPump1On : 1; // включен ли первый насос
  bool isPump2On : 1; // включен ли второй насос
  
} WateringFlags; // структура флагов модуля полива
#pragma pack(pop)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool isON : 1; // включен ли канал ?
  bool lastIsON: 1; // последнее состояние канала
  uint8_t index : 6; // индекс канала

  int8_t lastSavedStateMinute; // крайняя минута, когда мы сохраняли статус полива на канале
  
  uint32_t wateringTimer; // таймер полива для канала

  // управление питанием
  bool wantON; // запрошенное состояние выхода
  uint16_t powerTimerDelay; // таймер задержки
  
} WateringChannelFlags; // структура флагов канала полива
#pragma pack(pop)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  moistureWaitLowBorder,
  moistureWaitHighBorder
  
} WateringChannelMoistureState;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringModule;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringChannel
{
  private:

    WateringChannelFlags flags;
    WateringChannelMoistureState state;
    uint32_t _timer;

    void SignalToHardware(); // запоминает, какое состояние надо установить на выходе
    void ChangeHardwareState(); // изменяет состояние выхода

    void DoLoadState(uint8_t addressOffset); // загружает состояние
    void DoSaveState(uint8_t addressOffset, uint32_t wateringTimer = 0); // сохраняет состояние
  
  public:
    WateringChannel();

    void Setup(uint8_t index); // настраиваемся перед работой
    void On(WateringModule* m); // включаем канал
    void Off(WateringModule* m); // выключаем канал

    void LoadState(); // загружаем состояние из EEPROM
    void SaveState(uint32_t wateringTimer = 0); // сохраняем настройки в EEPROM
    
    bool IsChanged(); // изменилось ли состояние канала после вызова On() или Off() ?
    bool IsActive(); // активен ли полив на канале ?

    void Update(WateringModule* m, WateringWorkMode currentWorkMode, const RTCTime& currentTime, int8_t savedDayOfWeek); // обновляет состояние канала
};
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringModule : public AbstractModule // модуль управления поливом
{
  private:

  uint8_t lastDOW; // день недели с момента предыдущего опроса
  uint8_t currentDOW; // текущий день недели

  uint8_t switchToAutoAfterMidnight;

  #if WATER_RELAYS_COUNT > 0

  WateringFlags flags; // флаги состояний
  WateringChannel wateringChannels[WATER_RELAYS_COUNT]; // каналы полива

  #endif // WATER_RELAYS_COUNT > 0

  BlinkModeInterop blinker;
  bool canUseBlinker;

  void SwitchToAutomaticMode(); // переключаемся в автоматический режим работы
  void SwitchToManualMode(); // переключаемся в ручной режим работы

  void ResetChannelsState(); // сбрасываем сохранённое состояние для всех каналов в EEPROM
  void Skip(bool skipOrReset); // пропускаем полив за сегодня

  void TurnChannelsOff(); // выключает все каналы
  void TurnChannelsOn(); // включает все каналы
  
  void TurnChannelOff(uint8_t channelIndex); // выключает канал
  void TurnChannelOn(uint8_t channelIndex); // включает канал

  bool IsAnyChannelActive(); // проверяет, активен ли хоть один канал полива

  void SetupPumps();
  void UpdatePumps();
  void TurnPump1(bool isOn);
  void TurnPump2(bool isOn);
  void GetPumpsState(bool& pump1State, bool& pump2State);

  bool pump1OnTimerActive = false;
  uint32_t pump1Timer = 0;
  uint16_t pump1PowerDelay = 0;
  
  bool pump2OnTimerActive = false;
  uint32_t pump2Timer = 0;
  uint16_t pump2PowerDelay = 0;
    
  public:
    WateringModule() : AbstractModule("WATER") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

};
#endif // USE_WATERING_MODULE
