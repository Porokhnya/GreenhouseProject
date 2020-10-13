#ifndef _UNI_GLOBALS_H
#define _UNI_GLOBALS_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#define UNUSED(expr) do { (void)(expr); } while (0)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t valveState; // статус клапана наполнения бочки
  uint8_t countOfLevelSensors; // кол-во датчиков уровня в бочке (максимум 10)
  uint8_t levelSensors[10]; // данные с датчиков уровня в бочке (10 штук). Значение 0xFF - нет данных с датчика, 0 - датчик не сработал, 1 - датчик сработал
  uint8_t reserved[11]; // добитие до 23 байт

} WaterTankDataPacket;
#pragma pack(pop)
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
  uniSensorsClient = 1, // packet_type == 1
  uniNextionClient = 2, // packet_type == 2
  uniExecutionClient = 3, // packet_type == 3
  uniWindRainClient = 4, // packet_type == 4
  uniSunControllerClient = 5, // packet_type == 5
  uniWaterTankClient = 6, // packet_type == 6
  
} UniClientType; // тип клиента
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
};
//----------------------------------------------------------------------------------------------------------------
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
//----------------------------------------------------------------------------------------------------------------
#endif
