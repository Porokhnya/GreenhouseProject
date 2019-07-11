#ifndef _SETTINGS_H
#define _SETTINGS_H

#include <Arduino.h>
#include "Globals.h"

// класс настроек, которые сохраняются и читаются в/из EEPROM
// здесь будут всякие настройки, типа уставок срабатывания и пр. лабуды

enum WateringOption // какая опция управления поливом выбрана
{
  wateringOFF = 0, // автоматическое управление поливом выключено
  wateringWeekDays = 1, // управление поливом по дням недели, все каналы одновременно
  wateringSeparateChannels = 2 // раздельное управление каналами по дням недели  
};

#pragma pack(push,1)
typedef struct
{
  uint8_t wateringWeekDays; // в какие дни недели управляем поливом на этом канале?
  uint16_t wateringTime; // время полива на этом канале
  uint16_t startWateringTime; // время начала полива для этого канала (в минутах от начала суток)
  int8_t sensorIndex; // индекс датчика, который привязан к каналу
  uint8_t stopBorder; // показания датчика, по достижению которых канал полива выключается
  
} WateringChannelOptions; // настройки для отдельного канала полива
#pragma pack(pop)

// функция, которая вызывается при чтении/записи установок дельт - чтобы не хранить их в классе настроек.
// При чтении настроек класс настроек вызывает функцию OnDeltaRead, передавая прочитанные значения вовне.
// При записи настроек класс настроек вызывает функцию OnDeltaWrite.
typedef void (*DeltaReadWriteFunction)(uint8_t& sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2);

// функция, которая вызывается при чтении/записи установок дельт. Класс настроек вызывает OnDeltaGetCount, чтобы получить кол-во записей, которые следует сохранить,
// и OnDeltaSetCount - чтобы сообщить подписчику - сколько записей он передаст в вызове OnDeltaRead.
typedef void (*DeltaCountFunction)(uint8_t& count);

#pragma pack(push,1)
typedef struct
{
  byte ThingSpeakEnabled : 1;
  byte pad : 7;
  
} IoTSettingsFlags;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
  byte ModuleID;
  byte Type;
  byte SensorIndex;
  
} IoTSensorSettings;
#pragma pack(pop)

#pragma pack(push,1)
typedef struct
{
  byte Header1;
  byte Header2;
  IoTSettingsFlags Flags; // флаги
  uint32_t UpdateInterval; // интервал обновления, мс
  char ThingSpeakChannelID[20]; // ID канала ThingSpeak
  IoTSensorSettings Sensors[8]; // датчики для отсыла
  
} IoTSettings;
#pragma pack(pop)

enum
{
  MTS,
  Beeline,
  Megafon,
  Tele2,
  Yota,
  MTS_Bel,
  Velcom_Bel,
  Privet_Bel,
  Life_Bel,
  Dummy_Last_Op
};

class GlobalSettings
{
  private:

   uint8_t read8(uint16_t address, uint8_t defaultVal);
   
   uint16_t read16(uint16_t address, uint16_t defaultVal);
   void write16(uint16_t address, uint16_t val);

   unsigned long read32(uint16_t address, unsigned long defaultVal);
   void write32(uint16_t address, unsigned long val);

   String readString(uint16_t address, byte maxlength);
   void writeString(uint16_t address, const String& v, byte maxlength);
 
  public:
    GlobalSettings();

    IoTSettings GetIoTSettings();
    void SetIoTSettings(IoTSettings& sett);

    byte GetGSMProvider();
    bool SetGSMProvider(byte p);
    String GetGSMProviderName();
      
    uint8_t GetControllerID();
    void SetControllerID(uint8_t val);

    void ReadDeltaSettings(DeltaCountFunction OnDeltaSetCount, DeltaReadWriteFunction OnDeltaRead); // читаем настройки дельт 
    void WriteDeltaSettings(DeltaCountFunction OnDeltaGetCount, DeltaReadWriteFunction OnDeltaWrite); // пишем настройки дельт 

    uint8_t GetWateringOption();
    void SetWateringOption(uint8_t val);

     uint8_t GetWateringWeekDays();
     void SetWateringWeekDays(uint8_t val);

     uint16_t GetWateringTime();
     void SetWateringTime(uint16_t val);

     uint16_t GetStartWateringTime();
     void SetStartWateringTime(uint16_t val);

     int8_t GetWateringSensorIndex();
     void SetWateringSensorIndex(int8_t val);

     uint8_t GetWateringStopBorder();
     void SetWateringStopBorder(uint8_t val);

    uint8_t GetTurnOnPump();
    void SetTurnOnPump(uint8_t val);

    uint8_t GetTurnWateringToAutoAfterMidnight();
    void SetTurnWateringToAutoAfterMidnight(uint8_t val);

    uint8_t GetChannelWateringWeekDays(uint8_t idx);
    void SetChannelWateringWeekDays(uint8_t idx, uint8_t val);

     uint16_t GetChannelWateringTime(uint8_t idx);
     void SetChannelWateringTime(uint8_t idx,uint16_t val);

     uint16_t GetChannelStartWateringTime(uint8_t idx);
     void SetChannelStartWateringTime(uint8_t idx,uint16_t val);

     int8_t GetChannelWateringSensorIndex(uint8_t idx);
     void SetChannelWateringSensorIndex(uint8_t idx,int8_t val);

     uint8_t GetChannelWateringStopBorder(uint8_t idx);
     void SetChannelWateringStopBorder(uint8_t idx,uint8_t val);



    uint8_t GetOpenTemp();
    void SetOpenTemp(uint8_t val);

    uint8_t GetCloseTemp();
    void SetCloseTemp(uint8_t val);

    unsigned long GetOpenInterval();
    void SetOpenInterval(unsigned long val);

    String GetSmsPhoneNumber();
    void SetSmsPhoneNumber(const String& v);

    uint8_t GetWiFiState();
    void SetWiFiState(uint8_t st);
    
    String GetRouterID();
    void SetRouterID(const String& val);
    String GetRouterPassword();
    void SetRouterPassword(const String& val);
    
    String GetStationID();
    void SetStationID(const String& val);
    String GetStationPassword();
    void SetStationPassword(const String& val);


    String GetHttpApiKey();
    void SetHttpApiKey(const char* val);
    bool IsHttpApiEnabled();
    void SetHttpApiEnabled(bool val);

    int16_t GetTimezone();
    void SetTimezone(int16_t val);

    bool CanSendSensorsDataToHTTP();
    void SetSendSensorsDataFlag(bool val);

    bool CanSendControllerStatusToHTTP();
    void SetSendControllerStatusFlag(bool val);
    
    
};

#endif
