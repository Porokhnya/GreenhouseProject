#ifndef _SETTINGS_H
#define _SETTINGS_H

#include <Arduino.h>
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// класс настроек, которые сохраняются и читаются в/из EEPROM
// здесь будут всякие настройки, типа уставок срабатывания и пр. лабуды
//--------------------------------------------------------------------------------------------------------------------------------------
enum WateringOption // какая опция управления поливом выбрана
{
  wateringOFF = 0, // автоматическое управление поливом выключено
  wateringWeekDays = 1, // управление поливом по дням недели, все каналы одновременно
  wateringSeparateChannels = 2 // раздельное управление каналами по дням недели  
};
//--------------------------------------------------------------------------------------------------------------------------------------
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
//--------------------------------------------------------------------------------------------------------------------------------------
// функция, которая вызывается при чтении/записи установок дельт - чтобы не хранить их в классе настроек.
// При чтении настроек класс настроек вызывает функцию OnDeltaRead, передавая прочитанные значения вовне.
// При записи настроек класс настроек вызывает функцию OnDeltaWrite.
typedef void (*DeltaReadWriteFunction)(uint8_t& sensorType, String& moduleName1,uint8_t& sensorIdx1, String& moduleName2, uint8_t& sensorIdx2);
//--------------------------------------------------------------------------------------------------------------------------------------
// функция, которая вызывается при чтении/записи установок дельт. Класс настроек вызывает OnDeltaGetCount, чтобы получить кол-во записей, которые следует сохранить,
// и OnDeltaSetCount - чтобы сообщить подписчику - сколько записей он передаст в вызове OnDeltaRead.
typedef void (*DeltaCountFunction)(uint8_t& count);
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte ThingSpeakEnabled : 1;
  byte pad : 7;
  
} IoTSettingsFlags;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte ModuleID;
  byte Type;
  byte SensorIndex;
  
} IoTSensorSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
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
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t binding[16];
  
} WindowsChannelsBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  int16_t minTemp;
  int16_t maxTemp;
  int16_t ethalonTemp;
  int16_t histeresis;
  int16_t sensorIndex; // датчик на контуре отопления
  int16_t airSensorIndex; // датчик оокружения
  bool active;
} HeatSettings; 
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool active;
  int8_t sensorIndex;
  int16_t temp;
  int16_t histeresis;
  int32_t minWorkTime;
  int32_t maxWorkTime;
  int32_t restTime;
} VentSettings; 
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool active;
  int8_t sensorIndex;
  uint8_t sprayOnValue;
  uint8_t sprayOffValue;
  int8_t histeresis;
  int32_t startWorkTime;
  int32_t endWorkTime;
} HumiditySpraySettings; 
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool active;
  int8_t sensorIndex;
  int16_t temp;
  int16_t histeresis;
} ThermostatSettings; 
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  int16_t hour;
  int16_t histeresis;
  int16_t durationHour;
  int32_t lux;
  bool active;
} LightSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  int32_t lux;
  int16_t histeresis;
  int8_t sensorIndex;
  bool active;
} ShadowSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool gsmActive;
  uint8_t gsmInterval; // интервал обновления, часы
  
  bool wifiActive;
  uint8_t wifiInterval; // интервал обновления, часы
  char ntpServer[50];
  uint16_t ntpPort;
  int16_t ntpTimezone; // таймзона
  
} TimeSyncSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool active;
  uint8_t weekdays;
  uint32_t startTime;
  uint32_t endTime;
  uint16_t workTime;
  uint16_t idleTime;
  
} CycleVentSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool active;
  uint8_t weekdays;
  uint32_t startTime;
  uint32_t endTime;
  uint16_t ppm;
  uint16_t histeresis;
  
} CO2Settings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint16_t driveWorkTime; // время работы привода дверей
  int16_t minOpenTemp; // минимальная температура, при которой двери открываться не будут
  int16_t reservedU16; // зарезервированные 2 байта
  uint16_t startWorkTime; // время, в минутах от начала суток, когда дверь может открываться
  uint16_t endWorkTime; // время, в минутах от начала суток, когда дверь будет закрыта
  int8_t sensorIndex; // индекс датчика температуры для двери
  bool active; // активность модуля двери
} DoorSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
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

   bool checkHeader(uint16_t addr);
   void writeHeader(uint16_t addr);

   uint8_t wateringOption;
   uint8_t wateringWeekDays;
   uint16_t wateringTime;
   uint16_t startWateringTime;
   int8_t wateringSensorIndex;
   uint8_t wateringStopBorder;
   uint8_t turnOnPump;
   uint8_t turnWateringToAutoAfterMidnight;
   uint8_t wateringStartBorder;


   WateringChannelOptions wateringChannels[WATER_RELAYS_COUNT];
   uint8_t wateringChannelsStartBorders[WATER_RELAYS_COUNT];

   IoTSettings iotSettings;

   TimeSyncSettings timeSyncSettings;

   uint8_t controllerID;
   uint8_t openTemp;
   uint8_t closeTemp;

   unsigned long openInterval;
   String smsPhoneNumber;
   uint8_t wifiState;
   String routerID;
   String routerPassword;
   String stationID;
   String stationPassword;
   String httpApiKey;
   bool isHTTPEnabled;
   int16_t timezone;
   bool canSendSensorsToHTTP;
   bool canSendControllerStatusToHTTP;

   String key;

   bool scheduleActiveFlag;
 
  public:
    GlobalSettings();

    void setup();

    TimeSyncSettings* getTimeSyncSettings() { return &timeSyncSettings; }
    void setTimeSyncSettings(TimeSyncSettings& val);

    String getKey();
    void setKey(const String& key);


    IoTSettings GetIoTSettings();
    void SetIoTSettings(IoTSettings& sett);

    String GetGSMProviderName();
    void SetGSMProviderName(const String& val);

    String GetGSMAPNAddress();
    void SetGSMAPNAddress(const String& val);

    String GetGSMAPNUser();
    void SetGSMAPNUser(const String& val);

    String GetGSMAPNPassword();
    void SetGSMAPNPassword(const String& val);

    String GetGSMBalanceCommand();
    void SetGSMBalanceCommand(const String& val);
      
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

     uint8_t GetWateringStartBorder();
     void SetWateringStartBorder(uint8_t val);

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

     uint8_t GetChannelWateringStartBorder(uint8_t idx);
     void SetChannelWateringStartBorder(uint8_t idx,uint8_t val);


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

    int16_t Get25PercentsOpenTemp(uint8_t channel,int16_t defVal=WM_25PERCENTS_OPEN_TEMP);
    void Set25PercentsOpenTemp(uint8_t channel,int16_t val);

    int16_t Get50PercentsOpenTemp(uint8_t channel,int16_t defVal=WM_50PERCENTS_OPEN_TEMP);
    void Set50PercentsOpenTemp(uint8_t channel,int16_t val);

    int16_t Get75PercentsOpenTemp(uint8_t channel,int16_t defVal=WM_75PERCENTS_OPEN_TEMP);
    void Set75PercentsOpenTemp(uint8_t channel,int16_t val);

    int16_t Get100PercentsOpenTemp(uint8_t channel,int16_t defVal=WM_100PERCENTS_OPEN_TEMP);
    void Set100PercentsOpenTemp(uint8_t channel,int16_t val);

    int16_t GetWMHisteresis(uint8_t channel,int16_t defVal=WM_DEFAULT_HISTERESIS);
    void SetWMHisteresis(uint8_t channel,int16_t val);
    
    int16_t GetWMSensor(uint8_t channel,int16_t defVal=0);
    void SetWMSensor(uint8_t channel,int16_t val);

    bool GetWMActive(uint8_t channel,bool defVal=WM_ACTIVE_FLAG);
    void SetWMActive(uint8_t channel,bool val);

    WindowsChannelsBinding GetWMBinding();
    void SetWMBinding(WindowsChannelsBinding& val);

    WindowsChannelsBinding GetRainBinding();
    void SetRainBinding(WindowsChannelsBinding& val);

    WindowsChannelsBinding GetOrientationBinding();
    void SetOrientationBinding(WindowsChannelsBinding& val);

    WindowsChannelsBinding readWBinding(uint16_t address);
    void writeWBinding(uint16_t address, WindowsChannelsBinding& val);

    int16_t GetWindSpeed(int16_t defVal=WM_DEFAULT_WIND_SPEED);
    void SetWindSpeed(int16_t val);
    
    int16_t GetHurricaneSpeed(int16_t defVal=WM_DEFAULT_HURRICANE_SPEED);
    void SetHurricaneSpeed(int16_t val);

    HeatSettings GetHeatSettings(uint8_t channel);
    void SetHeatSettings(uint8_t channel,HeatSettings& val);

    int16_t GetHeatDriveWorkTime(uint8_t channel, int16_t defVal=5);
    void SetHeatDriveWorkTime(uint8_t channel, int16_t val);

    int16_t GetSealevel(int16_t defVal=PRESSURE_SEALEVEL);
    void SetSealevel(int16_t val);

    int16_t GetShadowDriveWorkTime(uint8_t channel, int16_t defVal=5);
    void SetShadowDriveWorkTime(uint8_t channel, int16_t val);

    LightSettings GetLightSettings();
    void SetLightSettings(LightSettings& val);

    ShadowSettings GetShadowSettings(uint8_t channel);
    void SetShadowSettings(uint8_t channel,ShadowSettings& val);

    CycleVentSettings GetCycleVentSettings(uint8_t channel);
    void SetCycleVentSettings(uint8_t channel,CycleVentSettings& val);

    CO2Settings GetCO2Settings();
    void SetCO2Settings(CO2Settings& val);

    VentSettings GetVentSettings(uint8_t channel);
    void SetVentSettings(uint8_t channel,VentSettings& val);

    ThermostatSettings GetThermostatSettings(uint8_t channel);
    void SetThermostatSettings(uint8_t channel,ThermostatSettings& val);

    DoorSettings GetDoorSettings(uint8_t channel);
    void SetDoorSettings(uint8_t channel,DoorSettings& val);

    HumiditySpraySettings GetHumiditySpraySettings(uint8_t channel);
    void SetHumiditySpraySettings(uint8_t channel,HumiditySpraySettings& val);

    bool isScheduleActive();
    void setScheduleActive(bool en);
    
    
};

#endif
