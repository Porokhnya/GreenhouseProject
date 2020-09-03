#ifndef _COMMON_H
#define _COMMON_H
//----------------------------------------------------------------------------------------------------------------
#define UNUSED(expr) do { (void)(expr); } while (0)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte Pin; // номер пина
  byte State; // состояние
  
} SlotSettings;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
//Структура передаваемая мастеру и обратно
//----------------------------------------------------------------------------------------------------------------
typedef enum
{
  slotEmpty, // пустой слот, без настроек
  slotWindowLeftChannel, // настройки привязки к левому каналу одного окна
  slotWindowRightChannel, // настройки привязки к правому каналу одного окна
  slotWateringChannel, // настройки привязки к статусу канала полива 
  slotLightChannel, // настройки привязки к статусу канала досветки
  slotPin // настройки привязки к статусу пина
  
} UniSlotType; // тип слота, для которого указаны настройки
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
struct UniSlotData
{
  byte slotType; // тип слота, одно из значений UniSlotType 
  byte slotLinkedData; // данные, привязанные к слоту мастером, должны хранится слейвом без изменений
  byte slotStatus; // статус слота (ВКЛ или ВЫКЛ)
    
};
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint32_t WindowsState; // состояние каналов окон, 4 байта = 32 бита = 16 окон)
  uint16_t WaterChannelsState; // состояние каналов полива, 2 байта, (16 каналов)
  byte LightChannelsState; // состояние каналов досветки, 1 байт (8 каналов)
  byte PinsState[16]; // состояние пинов, 16 байт, 128 пинов
  
} ControllerState; // состояние контроллера
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
enum {RS485FromMaster = 1, RS485FromSlave = 2};
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
};
//----------------------------------------------------------------------------------------------------------------
/*
 структура одной записи информации об окне:
 
    первый бит - сработал ли концевик открытия
    второй бит - сработал ли концевик закрытия
    третий бит - есть ли информация о позиции окна
    
    7 бит - информация о положении окна

   Итого у нас на одно окно - 10 бит информации, или 160 бит (20 байт) на 16 окон
 
 */
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
#pragma pack(push,1)
typedef struct
{
  byte mcpNumber;
  byte mcpChannel;
  
} InclinometerSettings;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte mcpNumber;
  byte openSwitchChannel;
  byte closeSwitchChannel;
    
} FeedbackEndstop;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte hasPosition;
  byte position;
  byte isCloseSwitchTriggered;
  byte isOpenSwitchTriggered;
  
}WindowStatus;
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  byte header1;
  byte header2;

  byte direction; // направление: 1 - от меги, 2 - от слейва
  byte type; // тип: 1 - пакет исполнительного модуля, 2 - пакет модуля с датчиками

  byte data[sizeof(ControllerState)]; // N байт данных, для исполнительного модуля в этих данных содержится состояние контроллера
  // для модуля с датчиками: первый байт - тип датчика, 2 байт - его индекс в системе. В обратку модуль с датчиками должен заполнить показания (4 байта следом за индексом 
  // датчика в системе и отправить пакет назад, выставив direction и type.

  byte tail1;
  byte tail2;
  byte crc8;
  
} RS485Packet; // пакет, гоняющийся по RS-485 туда/сюда (30 байт)
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
typedef enum
{
  uniSensorsClient = 1, // packet_type == 1
  uniNextionClient = 2, // packet_type == 2
  uniExecutionClient = 3, // packet_type == 3
  uniWindRainClient = 4, // packet_type == 4
  uniSunControllerClient = 5, // packet_type == 5
  
} UniClientType; // тип клиента
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
    byte packet_type;
    byte packet_subtype;
    byte config;
    byte controller_id;
    byte rf_id;
    
    UniSlotData slots[8]; // слоты настроек

    byte crc8;
    
} t_scratchpad; // 30 байт
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
#endif
