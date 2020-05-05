#ifndef _WATERING_MODULE_H
#define _WATERING_MODULE_H
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
  byte workMode: 2; // текущий режим работы модуля полива
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
  byte index : 6; // индекс канала

  int8_t lastSavedStateMinute; // крайняя минута, когда мы сохраняли статус полива на канале
  
  unsigned long wateringTimer; // таймер полива для канала
  //unsigned long wateringDelta; // дельта дополива    
  
} WateringChannelFlags; // структура флагов канала полива
#pragma pack(pop)
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringModule;
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringChannel
{
  private:

    WateringChannelFlags flags;

    void SignalToHardware(); // записывает текущее состояние канала в пин управления

    void DoLoadState(byte addressOffset); // загружает состояние
    void DoSaveState(byte addressOffset, unsigned long wateringTimer = 0); // сохраняет состояние
  
  public:
    WateringChannel();

    void Setup(byte index); // настраиваемся перед работой
    void On(WateringModule* m); // включаем канал
    void Off(WateringModule* m); // выключаем канал

    void LoadState(); // загружаем состояние из EEPROM
    void SaveState(unsigned long wateringTimer = 0); // сохраняем настройки в EEPROM
    
    bool IsChanged(); // изменилось ли состояние канала после вызова On() или Off() ?
    bool IsActive(); // активен ли полив на канале ?

    void Update(WateringModule* m, uint16_t dt,WateringWorkMode currentWorkMode, const RTCTime& currentTime, int8_t savedDayOfWeek); // обновляет состояние канала
};
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class WateringModule : public AbstractModule // модуль управления поливом
{
  private:

  uint8_t lastDOW; // день недели с момента предыдущего опроса
  uint8_t currentDOW; // текущий день недели

  #if WATER_RELAYS_COUNT > 0

  WateringFlags flags; // флаги состояний
  WateringChannel wateringChannels[WATER_RELAYS_COUNT]; // каналы полива

  #endif // WATER_RELAYS_COUNT > 0

#ifdef USE_WATERING_MANUAL_MODE_DIODE
  BlinkModeInterop blinker;
#endif

  void SwitchToAutomaticMode(); // переключаемся в автоматический режим работы
  void SwitchToManualMode(); // переключаемся в ручной режим работы

  void ResetChannelsState(); // сбрасываем сохранённое состояние для всех каналов в EEPROM
  void Skip(bool skipOrReset); // пропускаем полив за сегодня

  void TurnChannelsOff(); // выключает все каналы
  void TurnChannelsOn(); // включает все каналы
  
  void TurnChannelOff(byte channelIndex); // выключает канал
  void TurnChannelOn(byte channelIndex); // включает канал

  bool IsAnyChannelActive(); // проверяет, активен ли хоть один канал полива

  #ifdef USE_PUMP_RELAY
      void SetupPumps();
      void UpdatePumps();
      void TurnPump1(bool isOn);
      void TurnPump2(bool isOn);
      void GetPumpsState(bool& pump1State, bool& pump2State);
  #endif
    
  public:
    WateringModule() : AbstractModule("WATER") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
#endif // USE_WATERING_MODULE

#endif
