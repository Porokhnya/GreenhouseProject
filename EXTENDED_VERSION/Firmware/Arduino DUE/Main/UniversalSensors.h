#ifndef _UNUVERSAL_SENSORS_H
#define _UNUVERSAL_SENSORS_H
#include <Arduino.h>
#include "ModuleController.h"
#include "TinyVector.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// команды
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#define UNI_START_MEASURE 0x44 // запустить конвертацию
#define UNI_READ_SCRATCHPAD 0xBE // прочитать скратчпад
#define UNI_WRITE_SCRATCHPAD  0x4E // записать скратчпад
#define UNI_SAVE_EEPROM 0x25 // сохранить настройки в EEPROM

//-------------------------------------------------------------------------------------------------------------------------------------------------------
// максимальное кол-во датчиков в универсальном модуле
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#define MAX_UNI_SENSORS 3
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// значение, говорящее, что датчика нет
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#define NO_SENSOR_REGISTERED 0xFF
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// признак широковещательного пакета
#define BROADCAST_PACKET_ID 0xFF
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// описание разных частей скратчпада
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte packet_type; // тип пакета
  byte packet_subtype; // подтип пакета
  byte config; // конфигурация
  byte controller_id; // ID контроллера, к которому привязан модуль
  byte rf_id; // идентификатор RF-канала модуля
  
} UniScratchpadHead; // голова скратчпада, общая для всех типов модулей
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  UniScratchpadHead head; // голова
  byte data[24]; // сырые данные
  byte crc8; // контрольная сумма
  
} UniRawScratchpad; // "сырой" скратчпад, байты данных могут меняться в зависимости от типа модуля
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  slotEmpty, // пустой слот, без настроек
  slotWindowLeftChannel, // настройки привязки к левому каналу одного окна
  slotWindowRightChannel, // настройки привязки к правому каналу одного окна
  slotWateringChannel, // настройки привязки к статусу канала полива 
  slotLightChannel, // настройки привязки к статусу канала досветки
  slotPin // настройки привязки к статусу пина
  
} UniSlotType; // тип слота, для которого указаны настройки
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte slotType; // тип слота, одно из значений UniSlotType 
  byte slotLinkedData; // данные, привязанные к слоту мастером, должны хранится слейвом без изменений
  byte slotStatus; // статус слота (HIGH или LOW)
  
} UniSlotData; // данные одного слота настроек универсального модуля 
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  UniSlotData slots[8]; // слоты настроек
  
} UniExecutionModuleScratchpad; // скратчпад исполнительного модуля
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte index; // индекс датчика
  byte type; // тип датчика
  byte data[4]; // данные датчика
  
} UniSensorData; // данные с датчика универсального модуля
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte battery_status;
  byte calibration_factor1;
  byte calibration_factor2;
  byte query_interval_min;
  byte query_interval_sec;
  byte reserved;
  UniSensorData sensors[MAX_UNI_SENSORS];
   
} UniSensorsScratchpad; // скратчпад модуля с датчиками
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte sensorType; // тип датчика
  byte sensorData[2]; // данные датчика
   
} UniNextionData; // данные для отображения на Nextion
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t reserved[3]; // резерв, добитие до 24 байт
  uint8_t controllerStatus;
  uint8_t nextionStatus1;
  uint8_t nextionStatus2;  
  uint8_t openTemperature; // температура открытия окон
  uint8_t closeTemperature; // температура закрытия окон

  uint8_t dataCount; // кол-во записанных показаний с датчиков
  UniNextionData data[5]; // показания с датчиков
  
} UniNextionScratchpad; // скратчпад выносного модуля Nextion
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс для работы со скратчпадом, представляет основные функции, никак не изменяет переданный скратчпад до вызова функции read и функции write.
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class UniScratchpadClass
{
  public:
    UniScratchpadClass();

    void begin(byte pin,UniRawScratchpad* scratch); // привязываемся к пину и куску памяти, куда будем писать данные
    bool read(); // читаем скратчпад
    bool write(); // пишем скратчпад
    bool save(); // сохраняем скратчпад в EEPROM модуля
    bool startMeasure(); // запускаем конвертацию

  private:

    byte pin;
    UniRawScratchpad* scratchpad;

    bool canWork(); // проверяем, можем ли работать
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
/*

Все неиспользуемые поля инициализируются 0xFF

структура скратчпада модуля с датчиками:

packet_type - 1 байт, тип пакета (прописано значение 1)
packet_subtype - 1 байт, подтип пакета (прописано значение 0)
config - 1 байт, конфигурация (бит 0 - вкл/выкл передатчик, бит 1 - поддерживается ли фактор калибровки)
controller_id - 1 байт, идентификатор контроллера, к которому привязан модуль
rf_id - 1 байт, уникальный идентификатор модуля
battery_status - 1 байт, статус заряда батареи
calibration_factor1 - 1 байт, фактор калибровки
calibration_factor2 - 1 байт, фактор калибровки
query_interval - 1 байт, интервал обновления показаний (старшие 4 бита - минуты, младшие 4 бита - секунды)
reserved - 2 байт, резерв
index1 - 1 байт, индекс первого датчика в системе
type1 - 1 байт, тип первого датчика
data1 - 4 байта, данные первого датчика
index2 - 1 байт
type2 - 1 байт
data2 - 4 байта
index3 - 1 байт
type3 - 1 байт
data3 - 4 байта
crc8 - 1 байт, контрольная сумма скратчпада

*/
//-------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  uniNone = 0, // ничего нет
  uniTemp = 1, // только температура, значащие - два байта
  uniHumidity = 2, // влажность (первые два байта), температура (вторые два байта) 
  uniLuminosity = 3, // освещённость, 4 байта
  uniSoilMoisture = 4, // влажность почвы (два байта)
  uniPH = 5, // pH (два байта)
  uniPinsMap = 6, // карта пинов (4 байта)
  
} UniSensorType; // тип датчика
//-------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
 OneState* State1; // первое внутреннее состояние в контроллере
 OneState* State2; // второе внутреннее состояние в контроллере  
 
} UniSensorState; // состояние для датчика, максимум два (например, для влажности надо ещё и температуру тянуть)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  uniSensorsClient = 1, // packet_type == 1
  uniNextionClient = 2, // packet_type == 2
  uniExecutionClient = 3, // packet_type == 3
  uniWindRainClient = 4, // packet_type == 4
  uniSunControllerClient = 5, // packet_type == 5
  uniWaterTankClient = 6, // packet_type == 6
  
} UniClientType; // тип клиента
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t windDirection;
  uint32_t windSpeed;
  bool hasRain;
  uint8_t reserved[17]; // добитие до 23 байт

} WindRainDataPacket;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#define NO_DECLINATION_DATA -5000 // нет информации по склонению
#define NO_DIRECTION_DATA -10000 // нет информации по направлению
#define NO_HEADING_DATA -10000 // нет информации по heading
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
struct SunControllerDataPacket // данные, которые мы храним у себя и отправляем контроллеру теплицы отсюда (размер структуры должен быть 23 байта !!!)
{
  // температура датчика №1
  int8_t T1whole;
  uint8_t T1fract;

  // температура датчика №2
  int8_t T2whole;
  uint8_t T2fract;

  // температура датчика №3
  int8_t T3whole;
  uint8_t T3fract;

  // температура датчика №4
  int8_t T4whole;
  uint8_t T4fract;

  // температура датчика №5
  int8_t T5whole;
  uint8_t T5fract;
  
  int16_t angle; // наклон от горизонта
  int16_t direction; // направление в градусах, от севера
  int32_t luminosity; // показание засветки
  int16_t heading;
  
  
  uint8_t reserved[3]; // добитие до 23 байт

  SunControllerDataPacket()
  {
    T1whole = NO_TEMPERATURE_DATA;
    T2whole = NO_TEMPERATURE_DATA;
    T3whole = NO_TEMPERATURE_DATA;
    T4whole = NO_TEMPERATURE_DATA;
    T5whole = NO_TEMPERATURE_DATA;
    angle = NO_DECLINATION_DATA;
    direction = NO_DIRECTION_DATA;
    luminosity = NO_LUMINOSITY_DATA;
    heading = NO_HEADING_DATA;    
  }

};
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
// класс поддержки регистрации универсальных датчиков в системе
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class UniRegDispatcher 
 {
  public:
    UniRegDispatcher();

    void Setup(); // настраивает диспетчер регистрации перед работой

    // добавляет тип универсального датчика в систему, возвращает true, если добавилось (т.е. индекс был больше, чем есть в системе)
    bool AddUniSensor(UniSensorType type, uint8_t sensorIndex);

    // возвращает состояния для ранее зарегистрированного датчика
    bool GetRegisteredStates(UniSensorType type, uint8_t sensorIndex, UniSensorState& resultStates);

    // возвращает кол-во жёстко прописанных в прошивке датчиков того или иного типа
    uint8_t GetHardCodedSensorsCount(UniSensorType type); 
    // возвращает кол-во зарегистрированных универсальных модулей нужного типа
    uint8_t GetUniSensorsCount(UniSensorType type);

    uint8_t GetControllerID(); // возвращает уникальный ID контроллера

    void SaveState(); // сохраняет текущее состояние

    uint8_t GetRFChannel(); // возвращает текущий канал для nRF
    void SetRFChannel(uint8_t channel); // устанавливает канал для nRF


 private:

    void ReadState(); // читает последнее запомненное состояние
    void RestoreState(); // восстанавливает последнее запомненное состояние


    // модули разного типа, для быстрого доступа к ним
    AbstractModule* temperatureModule; // модуль температуры
    AbstractModule* humidityModule; // модуль влажности
    AbstractModule* luminosityModule; // модуль освещенности
    AbstractModule* soilMoistureModule; // модуль влажности почвы
    AbstractModule* phModule; // модуль контроля pH

    // жёстко указанные в прошивке датчики
    uint8_t hardCodedTemperatureCount;
    uint8_t hardCodedHumidityCount;
    uint8_t hardCodedLuminosityCount;
    uint8_t hardCodedSoilMoistureCount;
    uint8_t hardCodedPHCount;


    // последние выданные индексы для универсальных датчиков
    uint8_t currentTemperatureCount;
    uint8_t currentHumidityCount;
    uint8_t currentLuminosityCount;
    uint8_t currentSoilMoistureCount;
    uint8_t currentPHCount;
    uint8_t currentPinsMapCount;

    uint8_t rfChannel; // номер канала для nRF
  
 };
//-------------------------------------------------------------------------------------------------------------------------------------------------------
extern UniRegDispatcher UniDispatcher; // экземпляр класса диспетчера, доступный отовсюду
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// абстрактный класс клиента, работающего с универсальным модулем по шине 1-Wire
//-------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
    ssOneWire // с линии 1-Wire
  , ssRadio // по радиоканаду
  
} UniScratchpadSource; // откуда был получен скратчпад
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class AbstractUniClient
{
    public:
      AbstractUniClient() {};

      // регистрирует модуль в системе, если надо - прописывает индексы виртуальным датчикам и т.п.
      virtual void Register(UniRawScratchpad* scratchpad) = 0; 

      // обновляет данные с модуля, в receivedThrough - откуда был получен скратчпад
      virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough) = 0;

      void SetPin(byte p) { pin = p; }

   protected:
      byte pin;
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс ничего не делающего клиента
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class DummyUniClient : public AbstractUniClient
{
  public:
    DummyUniClient() : AbstractUniClient() {}
    virtual void Register(UniRawScratchpad* scratchpad) { UNUSED(scratchpad); }
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough) { UNUSED(scratchpad); UNUSED(isModuleOnline); UNUSED(receivedThrough); }
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс клиента для работы с модулями датчиков
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class SensorsUniClient : public AbstractUniClient
{
  public:
    SensorsUniClient();
    virtual void Register(UniRawScratchpad* scratchpad);
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough);

  private:

    unsigned long measureTimer;
    void UpdateStateData(const UniSensorState& states,const UniSensorData* data,bool IsModuleOnline);
    void UpdateOneState(OneState* os, const UniSensorData* data, bool IsModuleOnline);
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс клиента для работы с модулем контроля уровня воды в баке
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATER_TANK_MODULE
class WaterTankClient : public AbstractUniClient
{
  public:
    WaterTankClient();
    virtual void Register(UniRawScratchpad* scratchpad);
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough);
  
};
#endif // USE_WATER_TANK_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс клиента для работы с модулем показаний скорости, направления ветра и датчика дождя
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class WindRainClient : public AbstractUniClient
{
  public:
    WindRainClient();
    virtual void Register(UniRawScratchpad* scratchpad);
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough);
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс клиента для работы с модулем контроллера солнечной установки
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class SunControllerClient : public AbstractUniClient
{
  public:
    SunControllerClient();
    virtual void Register(UniRawScratchpad* scratchpad);
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough);
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс клиента исполнительного модуля
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_UNI_EXECUTION_MODULE
class UniExecutionModuleClient  : public AbstractUniClient
{
  public:

    UniExecutionModuleClient();
    virtual void Register(UniRawScratchpad* scratchpad);
    virtual void Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough);
  
};
#endif
//-------------------------------------------------------------------------------------------------------------------------------------------------------
struct UniNextionWaitScreenData
{
  byte sensorType;
  byte sensorIndex;
  const char* moduleName;
  UniNextionWaitScreenData(byte a, byte b, const char* c) 
  {
    sensorType = a;
    sensorIndex = b;
    moduleName = c;
  }
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
enum 
{
  RS485ControllerStatePacket = 1, 
  RS485SensorDataPacket = 2, 
  RS485WindowsPositionPacket = 3,
  RS485RequestCommandsPacket = 4,
  RS485CommandsToExecuteReceipt = 5,
  RS485SensorDataForRemoteDisplay = 6,
  RS485SettingsForRemoteDisplay = 7,
  RS485WindRainData = 8, // запрос данных по дождю, скорости, направлению ветра
  RS485SunControllerData = 9, // пакет с данными контроллера солнечной установки
  RS485WaterTankCommands = 10, // пакет с командами для модуля контроля бака воды
  RS485WaterTankSettings = 11, // пакет с настройками для модуля контроля уровня бака
  RS485WaterTankRequestData = 12, // пакет с запросом данных по баку с водой
  RS485WaterTankDataAnswer = 13,  // пакет с ответом на запрос данных по баку с водой
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_RS485_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
enum 
{
  RS485FromMaster = 1, 
  RS485FromSlave = 2
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte header1;
  byte header2;

  byte direction; // направление: 1 - от меги, 2 - от слейва
  byte type; // тип: 1 - пакет исполнительного модуля, 2 - пакет модуля с датчиками

  byte data[sizeof(ControllerState)]; // 23 байта данных, для исполнительного модуля в этих данных содержится состояние контроллера
  // для модуля с датчиками: первый байт - тип датчика, 2 байт - его индекс в системе. В обратку модуль с датчиками должен заполнить показания (4 байта следом за индексом 
  // датчика в системе и отправить пакет назад, выставив direction и type.

  byte tail1;
  byte tail2;
  byte crc8; // контрольная сумма пакета
  
} RS485Packet; // пакет, гоняющийся по RS-485 туда/сюда (30 байт)
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte moduleNumber; // номер модуля, от 1 до 4-х
  byte windowsSupported; // сколько окон поддерживает модуль (максимум - 16)
  byte windowsStatus[20]; // массив состояний окон
  byte reserved; // добитие до 23 байт
  
} WindowFeedbackPacket;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
enum // команды с модуля управления
{
  emCommandNone = 0,          // нет команды
  emCommandOpenWindows,       // открыть все окна
  emCommandCloseWindows,      // закрыть все осна
  emCommandOpenWindow,        // открыть определённое окно
  emCommandCloseWindow,       // закрыть определённое окно
  emCommandWaterOn,           // включить волив
  emCommandWaterOff,          // выключить полив
  emCommandWaterChannelOn,    // включить канал полива
  emCommandWaterChannelOff,   // выключить канал полива
  emCommandLightOn,           // включить досветку
  emCommandLigntOff,          // выключить досветку
  emCommandPinOn,             // включить пин
  emCommandPinOff,            // выключить пин
  emCommandAutoMode,          // перейти в автоматический режим работы
  emCommandWindowsAutoMode,   // автоматический режим работы окон
  emCommandWindowsManualMode,   // ручной режим работы окон
  emCommandWaterAutoMode,   // автоматический режим работы полива
  emCommandWaterManualMode,   // ручной режим работы полива
  emCommandLightAutoMode,   // автоматический режим работы досветки
  emCommandLightManualMode,   // ручной режим работы досветки
  emCommandSetOpenTemp, // установить температуру открытия
  emCommandSetCloseTemp, // установить температуру закрытия
  emCommandSetMotorsInterval, // установить интервал работы моторов
  emCommandStartScene, // запустить сценарий
  emCommandStopScene, // остановить сценарий
  emCommandWaterSkip, // пропустить полив на сегодня    
    
};
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{ 
  byte whichCommand;
  byte param1;
  byte param2;
  
} CommandToExecute;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte moduleID; // ID модуля, от которого пришла команда
  CommandToExecute commands[7]; // 21 байт
  byte reserved; // добитие до 23 байт
  
} CommandsToExecutePacket; // пакет с командами на выполнение
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t type; // тип датчика
  uint8_t data[4]; // данные датчика
  
} RemoteDisplaySensorData; // данные одного датчика для выносного дисплея
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t firstOrLastPacket; // признак начала/окончания всех данных (0 - нет данных, 1 - начало данных, 2 - окончание данных)
  uint8_t sensorsInPacket; // кол-во датчиков в пакете
  uint8_t hasDataFlags; // флаги, с каких датчиков в пакете есть показания
  RemoteDisplaySensorData data[4]; // данные датчиков
  
} RemoteDisplaySensorsPacket; // пакет показаний датчиков для выносного дисплея
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t openTemp; // температура открытия
  uint8_t closeTemp; // температура закрытия
  uint16_t interval; // интервал работы моторов
  uint8_t isWindowsOpen; // открыты ли окна
  uint8_t isWindowAutoMode; // автоматический режим работы окон?
  uint16_t windowsStatus; // статус окон по каналам (1 - открыто, 0 - закрыто)
  uint8_t isWaterOn; // включен ли полив?
  uint8_t isWaterAutoMode; // автоматический режим работы полива?
  uint8_t isLightOn; // включена ли досветка
  uint8_t isLightAutoMode; // автоматический режим работы досветки?
  uint8_t reserved[11]; // добитие до 23 байт

  
} RemoteDisplaySettingsPacket; // данные настроек для выносного дисплея
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#define REMOTE_DISPLAY_FIRST_SENSORS_PACKET 1
#define REMOTE_DISPLAY_LAST_SENSORS_PACKET 2
#define REMOTE_DISPLAY_SETTINGS_PACKET 4
//----------------------------------------------------------------------------------------------------------------
typedef Vector<RemoteDisplaySensorData> RemoteDisplaySensors;
typedef struct
{
  uint8_t type;
  uint8_t index;
  const char* moduleName;
  
} RemoteDisplaySensorSetting;
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte sensorType; // тип датчика
  byte sensorIndex; // зарегистрированный в системе индекс
  byte badReadingAttempts; // кол-во неудачных чтений с датчика
  
} RS485QueueItem; // запись в очереди на чтение показаний из шины
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
typedef Vector<RS485QueueItem> RS485Queue; // очередь к опросу
//----------------------------------------------------------------------------------------------------------------
class UniRS485Gate // класс для работы универсальных модулей через RS-485
{
  public:
    UniRS485Gate();
    void Setup();
    void Update();

    void sendFillTankCommand(bool on);
    void sendWaterTankSettingsPacket();    

  private:
  
#ifdef USE_UNI_EXECUTION_MODULE
    unsigned long updateTimer;
#endif    

#ifdef USE_REMOTE_DISPLAY_MODULE
    RemoteDisplaySensors remoteDisplaySensors;
    Vector<bool> remoteDisplaySensorsHasDataFlags;
    void collectSensorsForRemoteDisplay();
    void sendRemoteDisplaySettings();
#endif

    Stream* workStream;

    void sendControllerStatePacket();

    void waitTransmitComplete();
    void writeToStream(const uint8_t* buffer, size_t len);
    void enableSend();
    void enableReceive();

    void executeCommands(const RS485Packet& packet);

  #ifdef USE_UNIVERSAL_MODULES // если комплимся с поддержкой универсальных модулей - тогда обрабатываем очередь

    bool isInOnlineQueue(const RS485QueueItem& item);
    RS485Queue sensorsOnlineQueue; // очередь датчиков, с которых были показания
    RS485Queue queue;
    byte currentQueuePos;
    unsigned long sensorsTimer;
  #endif  
    
};

extern UniRS485Gate RS485;
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_RS485_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_NRF_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс работы с универсальными модулями через радиоканал
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte controller_id; // ID контроллера, который выплюнул в эфир пакет
  byte packetType; // тип пакета
  ControllerState state; // состояние контроллера
  byte reserved[4]; // резерв, добитие до 30 байт
  byte crc8; // контрольная сумма
  
} NRFControllerStatePacket; // пакет с состоянием контроллера
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  
  byte sensorType; // тип датчика
  byte sensorIndex; // зарегистрированный в системе индекс
  uint16_t queryInterval; // интервал между получениями информации с датчика
  unsigned long gotLastDataAt; // колда были получены последние данные
  
} NRFQueueItem;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
typedef Vector<NRFQueueItem> NRFQueue;
//----------------------------------------------------------------------------------------------------------------
class UniNRFGate
{
  public:
    UniNRFGate();
    void Setup();
    void Update();

    void SetChannel(byte channel);
    int ScanChannel(byte channel);

  private:
  
    void initNRF();
    void readFromPipes();
    
    bool bFirstCall;
    NRFControllerStatePacket packet;
    bool nRFInited;

    NRFQueue sensorsOnlineQueue;
    bool isInOnlineQueue(byte sensorType,byte sensorIndex, byte& result_index);
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_NRF_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATER_TANK_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte controller_id; // ID контроллера, который выплюнул в эфир пакет
  byte packetType; // тип пакета
  byte valveCommand; // флаг - включить клапан бака воды или выключить
  byte reserved[26]; // резерв, добитие до 30 байт
  byte crc8; // контрольная сумма
  
} NRFWaterTankExecutionPacket; // пакет с командами для модуля контроля бака воды
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte controller_id; // ID контроллера, который выплюнул в эфир пакет
  byte packetType; // тип пакета
  byte level; // уровень срабатывания датчиков
  uint32_t maxWorkTime; // максимальное время работы, секунд
  uint16_t distanceEmpty; // расстояние до пустого бака, см
  uint16_t distanceFull; // расстояние до полного бака, см              
  byte reserved[18]; // резерв, добитие до 30 байт
  byte crc8; // контрольная сумма
    
} NRFWaterTankSettingsPacket; // пакет с настройками для модуля контроля бака воды
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t valveState; // статус клапана наполнения бочки
  uint8_t fillStatus; // статус наполнения (0-100%)
  uint8_t errorFlag;  // флаг наличия ошибки
  uint8_t errorType;  // тип ошибки
  uint8_t reserved[19]; // добитие до 23 байт

} WaterTankDataPacket;
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t valveCommand; // флаг - включить клапан бака воды или выключить
  uint8_t reserved[22]; // добитие до 23 байт

} RS485WaterTankCommandPacket;
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t level; // флаг - включить клапан бака воды или выключить
  uint32_t maxWorkTime;
  uint32_t distanceEmpty;
  uint32_t distanceFull;
  uint8_t reserved[14]; // добитие до 23 байт

} RS485WaterTankSettingsPacket;
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATER_TANK_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс работы с универсальными модулями через радиоканал LoRa
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte controller_id; // ID контроллера, который выплюнул в эфир пакет
  byte packetType; // тип пакета
  ControllerState state; // состояние контроллера
  byte reserved[4]; // резерв, добитие до 30 байт
  byte crc8; // контрольная сумма
  
} LoRaControllerStatePacket; // пакет с состоянием контроллера
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  
  byte sensorType; // тип датчика
  byte sensorIndex; // зарегистрированный в системе индекс
  uint16_t queryInterval; // интервал между получениями информации с датчика
  unsigned long gotLastDataAt; // колда были получены последние данные
  
} LoRaQueueItem;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
typedef Vector<LoRaQueueItem> LoRaQueue;
//----------------------------------------------------------------------------------------------------------------
class UniLoRaGate
{
  public:
    UniLoRaGate();
    void Setup();
    void Update();
    int getRSSI() { return rssi; }
    bool isLoraInited() { return loRaInited; }

    void sendFillTankCommand(bool on);
    void sendWaterTankSettingsPacket();

  private:
  
    void initLoRa();

    int rssi;
      
    bool bFirstCall;
    LoRaControllerStatePacket packet;
    bool loRaInited;

    LoRaQueue sensorsOnlineQueue;
    bool isInOnlineQueue(byte sensorType,byte sensorIndex, byte& result_index);
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LORA_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Фабрика клиентов
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class UniClientsFactory
{
  private:

    DummyUniClient dummyClient;
    SensorsUniClient sensorsClient;
    WindRainClient windRainClient;
    SunControllerClient sunControllerClient;

    #ifdef USE_WATER_TANK_MODULE
    WaterTankClient waterTankClient;
    #endif // USE_WATER_TANK_MODULE

    #ifdef USE_UNI_EXECUTION_MODULE
    UniExecutionModuleClient executionClient;
    #endif
  
  public:
    UniClientsFactory();
    // возвращает клиента по типу пакета скратчпада
    AbstractUniClient* GetClient(UniRawScratchpad* scratchpad);
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
extern UniClientsFactory UniFactory; // наша фабрика клиентов
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс опроса универсальных модулей, постоянно висящих на линии
//-------------------------------------------------------------------------------------------------------------------------------------------------------
 class UniPermanentLine
 {
  public:
    UniPermanentLine(uint8_t pinNumber);

    void Update();

  private:


    bool IsRegistered();

    AbstractUniClient* lastClient; // последний известный клиент
    byte pin;
    unsigned long timer; // таймер обновления
    uint32_t _timer;
  
 };
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// класс регистрации универсальных модулей в системе 
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class UniRegistrationLine
{
  public:
    UniRegistrationLine(byte pin);

    bool IsModulePresent(); // проверяет, есть ли модуль на линии
    void CopyScratchpad(UniRawScratchpad* dest); // копирует скратчпад в другой

    bool SetScratchpadData(UniRawScratchpad* src); // копирует данные из переданного скратчпада во внутренний

    void Register(); // регистрирует универсальный модуль в системе

  private:

    bool IsSameScratchpadType(UniRawScratchpad* src); // тестирует, такой же тип скратчпада или нет

    // скратчпад модуля на линии, отдельный, т.к. регистрация у нас разнесена по времени с чтением скратчпада,
    // и поэтому мы не можем здесь использовать общий скратчпад/
    UniRawScratchpad scratchpad; 

    // пин, на котором мы висим
    byte pin;
  
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif
