#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "Memory.h"
#include "Utils.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define UNBINDED_PIN 0xFF // пин, к которому нет привязки - имеет значение 255
//--------------------------------------------------------------------------------------------------------------------------------------
#define linkUnbinded    0 // нет привязки
#define linkDirect      1 // привязка напрямую к пинам
#define linkMCP23S17    2 // привязка к MCP23S17
#define linkMCP23017    3 // привязка к MCP23017
#define link74HC595     4 // привязка к 74HC595  
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // куда привязан модуль?
  uint8_t MCPAddress; // адрес MCP
  uint8_t RelayPin; // номер пина для реле контроля CO2
  uint8_t AlertPin; // номер пина для реле тревоги
  uint8_t Level; // уровень включения выхода (1 - высокий, 0 - низкий)
  uint8_t VentPin; // номер пина, который управляет вентиляцией
  uint8_t SensorPin; // номер пина, на котором сидит датчик CO2
  uint16_t MaxPPM; // макcимальное значение PPM с датчика
  uint16_t MinADCPPMValue; // значение АЦП для минимального показания датчика
  uint16_t MaxADCPPMValue; // значение АЦП для максимального показания датчика
  uint8_t AlertPercents; // проценты превышения по тревоге
  uint8_t MeasureMode; // 0 - аналоговое измерение, 1 - PWM
  
} CO2Binding; // настройки привязки модуля CO2 к железу
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkTypeOutputs; // режим управления выходами
  uint8_t LinkTypeInputs; // режим управления входами
  uint8_t MCPAddressOutputs; // адрес MCP для выходов
  uint8_t MCPAddressInputs; // адрес MCP для входов
  uint8_t SensorPin; // номер аналогового пина датчика, прикреплённого к контроллеру
  uint16_t MVPer7Ph;  // кол-во милливольт, при  которых датчик показывает 7 pH
  uint8_t ReversiveMeasure; // флаг реверсивного измерения
  uint8_t FlowLevelSensorPin; // пин, на который заведён сигнал с датчика уровня воды (нижнего)
  uint8_t FlowLevelSensorPin2; // пин, на который заведён сигнал с датчика уровня воды (верхнего)
  uint8_t FlowAddPin; // номер пина включения подачи воды
  uint8_t PhPlusPin; // номер пина для повышения уровня pH
  uint8_t PhMinusPin; // номер пина для понижения уровня pH
  uint8_t PhMixPin; // номер пина для перемешивания
  uint8_t FlowSensorsLevel; // уровень срабатывания датчиков уровня воды
  uint8_t OutputsLevel; // уровень срабатывания выходов
  uint16_t VRef; // значение опорного напряжения, милливольт
  
} PHBinding; // настройки привязки модуля pH к железу
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  // ДЛЯ ЭТОЙ СТРУКТУРЫ ДОСТУПНО ВСЕГО 18 байт, и она может хранится по тому же адресу, что и настройки модуля PH
  uint8_t SensorPin; // номер аналогового пина датчика, прикреплённого к контроллеру
  uint16_t MVPer7Ph;  // кол-во милливольт, при  которых датчик показывает 7 pH
  uint8_t ReversiveMeasure; // флаг реверсивного измерения
  uint16_t VRef; // значение опорного напряжения, милливольт
  
  uint8_t reserved[12]; // запас
  
} ECPHBinding; // настройки привязки модуля EC/pH к железу
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t SensorPins[4]; // номера пинов датчиков EC
  uint8_t PinA; // пин подачи раствора А
  uint8_t PinB; // пин подачи раствора B
  uint8_t PinC; // пин подачи раствора C
  uint8_t PinPhPlus; // пин подачи раствора увеличения pH
  uint8_t PinPhMinus; // пин подачи раствора уменьшения pH
  uint8_t PinWater; // пин подачи воды (уменьшение ЕС)
  uint8_t WaterValve; // клапан блокировки автозаполнения бака водой
  uint8_t WaterContour; // клапан включения замкнутого контура
  uint8_t PumpPin; // пин насоса перемешивания и подачи
  uint16_t MaxPpm; // максимальное значение ppm для датчиков EC (сторожевое)  
  uint8_t Level; // уровень включения выходов
  
} ECBinding; // настройки железа для модуля EC
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень включения
  
} ThermostatBinding; // настройки железа для модуля термостата
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень включения
  
} VentBinding; // настройки железа для модуля вентиляции
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень включения
  
} HumiditySprayBinding; // настройки железа для модуля распрыскивания
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень включения
  
} CycleVentBinding; // настройки железа для модуля воздухообмена
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin1; // номер пина 1
  uint8_t Pin2; // номер пина 1
  uint8_t Level; // уровень включения
  
} ShadowBinding; // настройки железа для модуля затенения
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin1; // номер пина 1
  uint8_t Pin2; // номер пина 1
  uint8_t PumpPin; // пин насоса
  uint8_t Level; // уровень включения
  
} HeatBinding; // настройки железа для модуля отопления
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
enum
{
  NO_LIGHT_SENSOR,
  BH1750_SENSOR,
  MAX44009_SENSOR
  
}; // тип датчика освещённости
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Sensors[4]; // тип датчиков освещённости (максимум 4 датчика)
  uint8_t LampRelays[8]; // максимум 8 выходов реле досветки
  uint8_t Level; // уровень включения досветки
  uint8_t AveragingEnabled; // активно ли усреднение?
  uint8_t AveragingSamples; // кол-во показаний за цикл усреднения, минимум 4
  uint8_t HarboringEnabled; // активно ли загрубление?
  uint16_t HarboringStep; // шаг загрубления, люкс, минимум 2
  uint8_t SensorIndex; // индекс датчика, с которого следим за досветкой
  
} LightBinding; // настройки железа для модуля досветки
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  wrsNotUsed,
  wrsDirect,
  wrsExternalModule,
  wrsMisol
  
} WindRainSensorWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень срабатывания
  uint16_t ResetInterval; // через сколько секунд, если не был пойман сигнал с датчика дождя, сбрасывать флаг наличия дождя
  uint8_t  WorkMode; // режим работы датчика (0  не используется, 1 - напрямую к контроллеру, 2 - через выносной модуль, 3 - метеостанция типа Misol)
  
} RainSensorBinding; // настройки привязки железа для датчика дождя
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t Pin; // номер пина, с прерываниями
  uint8_t DirectionPin; // аналоговый пин направления ветра
  uint16_t SpeedCoeff; // коэффициент пересчёта частоты импульсов (в герцах) в скорость ветра (м/с). На этот коэффициент - умножается частота.
  uint16_t EastAdcFrom;
  uint16_t EastAdcTo;
  uint16_t SouthAdcFrom;
  uint16_t SouthAdcTo;
  uint16_t WestAdcFrom;
  uint16_t WestAdcTo;
  uint16_t NorthAdcFrom;
  uint16_t NorthAdcTo;
  uint8_t Histeresis; // гистерезис показаний АЦП для направления ветра (для проверки попадания в интервал)
  uint8_t NumSamples; // кол-во показаний, по которому рассчитывается направление ветра. Полный цикл сбора показаний составит WIND_NUM_SAMPLES*WIND_SENSOR_UPDATE_INTERVAL
  uint8_t WorkMode; // режим работы датчика (0  не используется, 1 - напрямую к контроллеру, 2 - через выносной модуль, 3 - метеостанция типа Misol)
} WindSensorBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина
  uint8_t Level; // уровень срабатывания
  uint8_t Duration;
  
} BuzzerBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления питанием
  uint8_t MCPAddress; // адрес MCP
  uint8_t CEPin; // номер пина CE (напрямую к контроллеру)
  uint8_t CSNPin; // номер пина CSN (напрямую к контроллеру)
  uint8_t PowerPin; // номер пина управления питанием
  uint8_t Level; // уровень срабатывания пина управления питанием
  uint8_t AutoAckInverted; // режим инвертирования auto ack
} NrfBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления питанием
  uint8_t MCPAddress; // адрес MCP
  uint8_t SSPin; // номер пина SS (напрямую к контроллеру)
  uint8_t ResetPin; // номер пина RESET (напрямую к контроллеру)
  uint8_t PowerPin; // номер пина управления питанием
  uint8_t Level; // уровень срабатывания пина управления питанием
  uint8_t TXPower; // мощность передатчика (1 - 17)
  uint16_t Frequency; // частота работы (433E6, 868E6, 915E6)
} LoraBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления таймерами
  uint8_t MCPAddress; // адрес MCP
  uint8_t Level; // уровень срабатывания пина таймера
} TimerBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t Pin[20]; // номера пинов для датчиков DS18B20
  
} DallasBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t Pin[20]; // номера пинов для датчиков-эмуляторов DS18B20
  uint8_t Type[20]; // типы датчиков (0-нет, 1 - влажность, 2 - влажность почвы, 3 - освещённость)
  uint8_t Index[20]; // индексы датчиков в результирующих модулях
  
} DS18B20EmulationBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t RegistrationLine; // пин линии регистрации универсальных модулей
  uint8_t Pin[5]; // номера пинов для линий 1-Wire
  
} OneWireBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления диодами
  uint8_t MCPAddress; // адрес MCP
  uint8_t Level; // уровень включения диодов
  uint8_t ReadyDiodePin;
  uint8_t BlinkReadyDiode;
  uint8_t WindowsManualModeDiodePin;
  uint8_t WateringManualModeDiodePin;
  uint8_t LightManualModeDiodePin;  
} DiodesBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления окнами
  uint8_t MCPAddress1; // адрес MCP для первой половины окон (8 штук)
  uint8_t MCPAddress2; // адрес MCP для второй половины окон (8 штук)
  uint8_t Level; // уровень включения окна
  uint8_t ManageMode; // 0 - одновременно, 1 - попеременно
  uint8_t WindowsPins[32]; // пины для окон, на 16 окон - 32 пина, идут попарно, последовательно
  uint8_t ShiftOEPin;
  uint8_t ShiftLatchPin;
  uint8_t ShiftDataPin;
  uint8_t ShiftClockPin;
  uint32_t AdditionalCloseTime;
} WindowsBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления концевиками
  uint8_t MCPAddress1; // адрес MCP для концевиков открытия (16 штук)
  uint8_t MCPAddress2; // адрес MCP для концевиков закрытия (16 штук)
  uint8_t Level; // уровень срабатывания концевика
  uint8_t EndstopOpenPins[16]; // пины для концевиков открытия окон
  uint8_t EndstopClosePins[16]; // пины для концевиков закрытия окон
} EndstopsBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления питанием датчиков влажности почвы
  uint8_t MCPAddress; // адрес MCP для управления питанием датчиков влажности почвы
  uint8_t PowerPin; // пин включения питания
  uint8_t Level; // уровень включения питания
  uint16_t PowerOnDelay; // сколько миллисекунд ждать перед подачей питания до чтения с датчиков
  uint16_t Soil100Percent; // какие показания analogRead соответствуют датчику, погруженному в воду
  uint16_t Soil0Percent; // какие показания analogRead соответствуют датчику на воздухе, т.е. полностью сухой почве
  uint16_t NoDataBorder; // значение АЦП, ниже (или выше) которого будет считаться, что данных с датчика нет
  uint8_t SensorType[10]; // типы датчиков
  uint8_t SensorPin[10]; // пины датчиков
} SoilBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t StrobeLevel; // уровень разрыва строба датчика
  uint8_t DHTReadDelay; // // пауза (в микросекундах) перед чтением с датчика DHT. По даташиту - 40 микросекунд, иногда помогает увеличить эту паузу.
  uint8_t Pin1[10]; // пины 1 датчиков
  uint8_t Pin2[10]; // пины 2 датчиков
  uint8_t Type[10]; // типы датчиков
} HumidityBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // режим управления приемом-передачей RS-485
  uint8_t MCPAddress; // адрес MCP
  uint8_t DEPin; // пин управления приемом-передачей
  uint16_t StatePushFrequency; // через сколько миллисекунд писать в шину RS-485 слепок состояния контроллера
  uint16_t OneSensorUpdateInterval; // через сколько миллисекунд запрашивать с шины RS-485 показания одного датчика (полный цикл опроса будет равен интервалу*кол-во датчиков в системе)
  uint8_t BytesTimeout; // кол-во байт, после неуспешной попытки вычитки которых принимать решение о таймауте (если данные по RS-485 не ходят - увеличьте это значение)
  uint8_t ResetSensorBadReadings; // через сколько неудачных чтений с датчика сбрасывать его значения на вид "<нет данных>"
  uint8_t SerialNumber; // номер Serial (0 - нет, 1 - Serial1, 2 - Serial2, 3 - Serial3)
} RS485Binding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t ChannelsLinkType; // режим управления каналами полива (16 штук)
  uint8_t ChannelsMCPAddress; // адрес MCP для каналов полива
  uint8_t Level; // уровень срабатывания каналов полива
  uint8_t PumpLinkType; // режим управления первым насосом
  uint8_t PumpMCPAddress; // адрес MCP для первого насоса
  uint8_t PumpPin; // номер пина первого насоса
  uint8_t Pump2LinkType; // режим управления вторым насосом
  uint8_t Pump2MCPAddress; // адрес MCP для второго насоса
  uint8_t Pump2Pin; // номер пина второго насоса
  uint8_t PumpLevel; // уровень включения насосов
  uint8_t Pump2StartChannel; // с какого канала полива включать второй насос
  uint8_t SaveInterval; // интервал сохранений времени полива на каналах, в минутах
  uint8_t RelaysPins[16]; // номера пинов реле каналов полива
} WateringBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t SerialNumber; // какой хардварный Serial будем использовать при работе с модемом? 0 - не используется, 1 - Serial1 и т.д.
  uint32_t AvailableCheckTime; // через сколько миллисекунд проверять доступность модема посылкой команды AT+CSQ (заодно получаем мощность сигнала)
  uint32_t RebootTime; // сколько миллисекунд держать питание выключенным при перезагрузке модема, если он завис
  uint32_t WaitAfterRebootTime; // сколько мс ждать после перезагрузки модема прежде, чем начать обрабатывать команды
  uint32_t MaxAnswerTime; // через сколько мс, если не получен ответ на команду от модема, считать его зависшим
  uint8_t RebootPinLinkType; // тип управления пином питания
  uint8_t RebootPinMCPAddress; // адрес MCP для пина питания
  uint8_t RebootPin; // номер пина, на котором будет управление питанием GSM-модема
  uint8_t PowerOnLevel; // уровень для включения питания
  uint8_t PullGPRSByPing; // пинговать или нет GPRS?
  uint32_t PingInterval; // интервал в миллисекундах для пинга
  uint8_t PowerkeyLinkType; // тип управления POWERKEY
  uint8_t PowerkeyMCPAddress; // адрес MCP для управления выходом POWERKEY
  uint8_t PowerkeyPin; // номер пина, на котором будет импульс для включения модема в работу
  uint16_t PowerkeyPulseDuration; // сколько миллисекунд держать импульс
  uint8_t PowerkeyOnLevel; // уровень, который держится нужное кол-во времени
  uint16_t WaitPowerkeyAfterPowerOn; // сколько миллисекунд ждать после подачи питания до подачи импульса POWERKEY
  uint8_t SendWorkStartedSMS; // флаг отсылки SMS при старте контроллера в работу
  uint32_t IncomingDataTimeout; // сколько миллисекунд ждать полного вычитывания данных
  uint8_t CalmWatchdog;  // успокаивать внешний ватчдог в процессе вычитки входящих данных?
  uint8_t SendWindowStateInStatusSMS; // посылать статус окон в СМС статистики ?
  uint8_t SendWaterStateInStatusSMS; // посылать статус полива в СМС статистики ?
  
} GSMBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t SerialNumber; // какой хардварный Serial будем использовать при работе с модемом? 0 - не используется, 1 - Serial1 и т.д.
  uint32_t AvailableCheckTime; // через сколько миллисекунд проверять доступность модема посылкой команды AT
  uint32_t RebootTime; // сколько миллисекунд держать питание выключенным при перезагрузке модема, если он завис
  uint32_t WaitAfterRebootTime; // сколько мс ждать после перезагрузки модема прежде, чем начать обрабатывать команды
  uint32_t MaxAnswerTime; // через сколько мс, если не получен ответ на команду от модема, считать его зависшим
  uint8_t RebootPinLinkType; // тип управления пином питания
  uint8_t RebootPinMCPAddress; // адрес MCP для пина питания
  uint8_t RebootPin; // номер пина, на котором будет управление питанием модема
  uint8_t PowerOnLevel; // уровень для включения питания
  
} WiFiBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t BacklightLinkType; // тип управления пином подсветки
  uint8_t BacklightMCPAddress; // адрес MCP для пина подсветки
  uint8_t BacklightPin; // номер пина, на котором будет управление подсветкой
  uint8_t BacklightOnLevel; // уровень для включения подсветки
  uint16_t InitDelay; // сколько миллисекунд (при необходимости) ждать перед отсылкой первых команд на дисплей
  uint32_t OffDelay; // время выключения подсветки экрана при бездействии, миллисекунд
  uint8_t DisplayType; // тип дисплея (0 - не используется, 1 - 7 дюймов, 2 - 4 дюйма)
  
} TFTBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t Flow1; // первый пин расходомера
  uint8_t Flow2; // второй пин расходомера
  uint8_t SaveDelta; // через сколько накопленных литров сохранять в EEPROM значение с датчика
  
} FlowBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  tftSensor_NoModule,
  tftSensorModule_State_Temperature,
  tftSensorModule_Humidity_Temperature,
  tftSensorModule_Humidity_Humidity,
  tftSensorModule_Luminosity,
  tftSensorModule_CO2,
  tftSensorModule_Soil,
  tftSensorModule_PH,
  tftSensorModule_EC,
  
} TFTSensorModuleType; // типы датчиков и модулей для отображения на TFT-экране
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t Type; // тип датчика
  uint8_t Index; // индекс датчика
  char Caption[35]; // подпись датчика
  
} TFTSensorBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // тип привязки
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin; // номер пина для управления питанием фрамуг
  uint8_t Level; // уровень включения питания
  uint16_t PowerOnDelay; // задержка после включения питания до начала движения фрамуг
  uint16_t PowerOffDelay; // задержка перед выключением питания
  uint16_t LinkedChannels; // битовая маска привязанных к управлению питанием фрамуг
  uint16_t WateringChannels; // битовая маска привязанных к управлению питанием каналов полива
  uint8_t DoorBinding; // битовая маска дверей, управляемых питанием
  
} WPowerBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // тип привязки
  uint8_t MCPAddress; // адрес MCP
  uint8_t Pin1; // номер пина 1 для приводом двери
  uint8_t Pin2; // номер пина 2 для приводом двери
  uint8_t Level; // уровень включения питания
  uint8_t DriveType; // тип управления: 0 - перед любым действием делаем противоположное, 1 - прямое действие  
} DoorBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  int16_t Interval[16]; // интервалы открытия на 16 окон, в секундах (-1 - брать из общего интервала)
  
} WindowsIntervals;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t LinkType; // тип привязки, пока используется только LoRa (значение 1)
  uint8_t Level; // уровень срабатывания датчиков в баке
  uint32_t MaxWorkTime; // максимальное время работы, секунд
  uint16_t DistanceEmpty; // расстояние до пустого бака, см
  uint16_t DistanceFull; // расстояние до полного бака, см
  
} WaterTankBinding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
class EEPROMSettingsModule : public AbstractModule // модуль хранения настроек привязок в EEPROM
{
  private:

    template< typename T >
    bool read(uint32_t address, T& result)
    {
        if(MemRead(address++) != SETT_HEADER1)
        {
          return false;
        }
    
        if(MemRead(address++) != SETT_HEADER2)
        {
          return false;
        }
    
       uint8_t* ptr = (uint8_t*)&result;
    
       for(size_t i=0;i<sizeof(T);i++)
       {
        *ptr++ = MemRead(address++);
       }

      // читаем контрольную сумму
      uint8_t crc = MemRead(address);       
      return crc == crc8((const uint8_t*)&result, sizeof(result));
    }

    template< typename T >
    void write(uint32_t address, T& val)
    {
        MemWrite(address++,SETT_HEADER1);
        MemWrite(address++,SETT_HEADER2);
      
        uint8_t* ptr = (uint8_t*)&val;
        
        for(size_t i=0;i<sizeof(T);i++)
          MemWrite(address++,*ptr++); 

       // пишем контрольную сумму
        uint8_t crc = crc8((const uint8_t*)&val, sizeof(val));
        MemWrite(address,crc);
    }


  // настройки, которые мы храним
  CO2Binding co2Binding;
  PHBinding phBinding;
  ECPHBinding ecphBinding;
  ECBinding ecBinding;
  ThermostatBinding thermostatBinding[3];
  VentBinding ventBinding[3];
  HumiditySprayBinding sprayBinding[3];
  CycleVentBinding cycleVentBinding[3];
  ShadowBinding shadowBinding[3];
  HeatBinding heatBinding[3];
  LightBinding lightBinding;
  RainSensorBinding rainSensorBinding;
  WindSensorBinding windSensorBinding;
  BuzzerBinding buzzerBinding;
  NrfBinding nrfBinding;
  LoraBinding loraBinding;
  TimerBinding timerBinding;
  DallasBinding dallasBinding;
  DS18B20EmulationBinding ds18B20EmulationBinding;
  OneWireBinding oneWireBinding;
  DiodesBinding diodesBinding;
  WindowsBinding windowsBinding;
  EndstopsBinding endstopsBinding;
  SoilBinding soilBinding;
  HumidityBinding humidityBinding;
  RS485Binding rs485Binding;
  WateringBinding wateringBinding;
  GSMBinding gsmBinding;
  WiFiBinding wifiBinding;
  TFTBinding tftBinding;
  FlowBinding flowBinding;
  TFTSensorBinding tftSensorBinding[12];
  WPowerBinding wPowerBinding;
  DoorBinding doorBinding[2];
  WindowsIntervals windowsIntervals;
  WaterTankBinding waterTankBinding;
  
  public:
    EEPROMSettingsModule();

    static bool SafePin(uint8_t pin); // проверяет пин на безопасную работу с ним (чтобы не дай бог не залезть в TFT и т.п.)

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    // возвращает настройки привязки модуля CO2 к железу
    CO2Binding& GetCO2Binding() { return co2Binding; }

    // возвращает настройки привязки модуля pH к железу
    PHBinding& GetPHBinding() { return phBinding; }

    // возвращает настройки привязки модуля EC/pH к железу
    ECPHBinding& GetECPHBinding() { return ecphBinding; }

    // возвращает настройки привязки модуля EC к железу
    ECBinding& GetECBinding() { return ecBinding; }

    // возвращает настройки привязки модуля термостата к железу
    ThermostatBinding& GetThermostatBinding(uint8_t channel){return thermostatBinding[channel];}

    // возвращает настройки привязки модуля вентиляции к железу
    VentBinding& GetVentBinding(uint8_t channel){return ventBinding[channel];}

    // возвращает настройки привязки модуля распрыскивания к железу
    HumiditySprayBinding& GetHumiditySprayBinding(uint8_t channel){return sprayBinding[channel];}

    // возвращает настройки привязки модуля воздухообмена к железу
    CycleVentBinding& GetCycleVentBinding(uint8_t channel){return cycleVentBinding[channel];}

    // возвращает настройки привязки модуля затенения к железу
    ShadowBinding& GetShadowBinding(uint8_t channel){return shadowBinding[channel];}

    // возвращает настройки привязки модуля отопления к железу
    HeatBinding& GetHeatBinding(uint8_t channel){return heatBinding[channel];}

    // возвращает настройки привязки модуля освещённости к железу
    LightBinding& GetLightBinding() { return lightBinding; }

    // возвращает настройки привязки датчика дождя к железу
    RainSensorBinding& GetRainSensorBinding() { return rainSensorBinding; }

    // возвращает настройки датчика ветра к железу
    WindSensorBinding& GetWindSensorBinding() { return windSensorBinding; }

    // возвращает настройки пищалки к железу
    BuzzerBinding& GetBuzzerBinding(){ return buzzerBinding; }

    // возвращает настройки привязки nRF в железу
    NrfBinding& GetNrfBinding(){ return nrfBinding; }

    // возвращает настройки привязки Lora в железу
    LoraBinding& GetLoraBinding(){ return loraBinding; }

    // возвращает настройки привязки таймеров к железу
    TimerBinding& GetTimerBinding() { return timerBinding; }

    // возвращает настройки привязки DS18B20 к железу
    DallasBinding& GetDallasBinding() { return dallasBinding; }

    // возвращает настройки привязки эмуляторов DS18B20 к железу
    DS18B20EmulationBinding& GetDS18B20EmulationBinding() { return ds18B20EmulationBinding; }

    // возвращает настройки привязки 1-Wire к железу
    OneWireBinding& GetOneWireBinding() { return oneWireBinding; }

    // возвращает настройки привязки информационных диодов к железу
    DiodesBinding& GetDiodesBinding() { return diodesBinding; }

    // возвращает настройки привязки фрамуг к железу
    WindowsBinding& GetWindowsBinding() { return windowsBinding; }

    // возвращает настройки привязки концевиков к железу
    EndstopsBinding& GetEndstopsBinding() { return endstopsBinding; }

    // возвращает настройки привязки датчиков влажности почвы к железу
    SoilBinding& GetSoilBinding() { return soilBinding; }

    // возвращает настройки привязки датчиков влажности к железу
    HumidityBinding& GetHumidityBinding() { return humidityBinding; }

    // возвращает настройки RS485
    RS485Binding& GetRS485Binding() { return rs485Binding; }

    // возвращает настройки привязки полива к железу
    WateringBinding& GetWateringBinding() { return wateringBinding; }

    // возвращает настройки привязки SIM800 к железу
    GSMBinding& GetGSMBinding() { return gsmBinding; }

    // возвращает настройки привязки ESP к железу
    WiFiBinding& GetWiFiBinding() { return wifiBinding; }

    // возвращает настройки привязки TFT к железу
    TFTBinding& GetTFTBinding() { return tftBinding; }

    // возвращает настройки привязки расходомеров к железу
    FlowBinding& GetFlowBinding() { return flowBinding; }

    // возвращает настройки датчика для отображения на TFT 
    TFTSensorBinding& GetTFTSensorBinding(uint8_t idx){return tftSensorBinding[idx];}

    // возвращает настройки привязки управления питанием фрамуг
    WPowerBinding& GetWPowerBinding() { return wPowerBinding; }

    // возвращает настройки привязки управления дверями
    DoorBinding& GetDoorBinding(uint8_t channel) { return doorBinding[channel]; }    

    // возвращает настройки интервалов открытия окон
    WindowsIntervals& GetWindowsIntervals() { return windowsIntervals; }

    // возвращает настройки модуля наполнения бака
    WaterTankBinding& GetWaterTankBinding() { return waterTankBinding; }

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern EEPROMSettingsModule* HardwareBinding;

