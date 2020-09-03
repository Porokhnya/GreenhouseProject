#include <avr/io.h>
#include <avr/interrupt.h>
#include "Common.h"
#include "LowLevel.h"
#include "OneWireSlave.h"
//----------------------------------------------------------------------------------------------------------------
/*
Прошивка для универсального модуля, предназначена для выноса
состояния контроллера по 1-Wire.

Также поддерживается работа по RS-485, для включения этой возможности
надо раскомментировать USE_RS485_GATE.

Также поддерживается работа по радиоканалу, используя модуль nRF24L01+,
для этой возможности раскомментируйте USE_NRF.

Также поддерживается работа по радиоканалу, используя модуль LoRa,
для этой возможности раскомментируйте USE_LORA.

Также поддерживается возможность использования обратной связи по положению
окон и срабатывания концевиков, для этой возможности раскомментируйте USE_FEEDBACK.
Обратная связь поддерживается только по RS-485!

ВНИМАНИЕ!

Если возможность обратной связи не используется (директива USE_FEEDBACK закомментирована) -
надо ногу разрешения приёма микросхемы преобразователя RS485 принудительно посадить на землю, чтобы не занимать лишний пин МК!


ВНИМАНИЕ!

RS-485 работает через аппаратный UART (RX0 и TX0 ардуины)!
Перед прошивкой выдёргивать контакты модуля RS-485 из пинов аппаратного UART!

ВНИМАНИЕ!

Пин 2 - не занимать, через него работает регистрация на центральном контроллере!

*/
//----------------------------------------------------------------------------------------------------------------
// НАСТРОЙКИ ОБРАТНОЙ СВЯЗИ
//----------------------------------------------------------------------------------------------------------------
//#define USE_FEEDBACK // закомментировать, если не нужен функционал обратной связи (4 канала положения окон + 
// состояние концевиков открытия и закрытия каждого окна)
//----------------------------------------------------------------------------------------------------------------
//#define USE_INCLINOMETERS // закомментировать, если не надо использовать инклинометры (в этом случае будут использоваться
// только концевики)
//----------------------------------------------------------------------------------------------------------------
#define WINDOWS_SERVED 4 // Сколько окон обслуживается (максимум - 4, минимум - 1)
//----------------------------------------------------------------------------------------------------------------
#define FEEDBACK_UPDATE_INTERVAL 1000 // интервал между обновлениями статусов окон. Каждое окно обновляет свой статус
// через этот интервал, таким образом полный цикл обновления равняется FEEDBACK_UPDATE_INTERVAL*WINDOWS_SERVED.
// мы не можем читать информацию прямо в процессе обработки входящего по RS485 паката, поэтому делаем слепок
// состояния через определённые промежутки времени.
//----------------------------------------------------------------------------------------------------------------
// если эта настройка раскомментирована - управление обратной связью будет доступно только для одного компаса,
// т.е. будет поддерживаться только  одно окно. В этом случае компас HMC5883 подсоединяется напрямую к шине I2C
// (пины A4 и A5), плюс используются ещё два пина для отслеживания концевиков. Адресация модуля при этой настройке 
// доступна только через конфигуратор.
#define FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
// номер пина, с которого считается положение концевика закрытия в режиме FEEDBACK_DIRECT_MODE
#define CLOSE_SWITCH_PIN 8
// номер пина, с которого считается положение концевика открытия в режиме FEEDBACK_DIRECT_MODE
#define OPEN_SWITCH_PIN 9
// ВНИМАНИЕ: в конфигурации с включённым nRF и включённым RS-485 при прямом управлении обратной связью - не 
// хватает пинов, т.к. заняты следующие: 0,1,2,3,4,5,6,7,9,10,11.12,13,A0,А1,А2,А3,А4,А5, оставляя под концевики
// только один пин, вместо двух!!! Поэтому - придётся чем-то жертвовать при прямом управлении компасом и концевиками.
//----------------------------------------------------------------------------------------------------------------
// настройки MCP23017, актуально при закомментированной настройке FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
#define COUNT_OF_MCP23017_EXTENDERS 2 // сколько расширителей портов MCP23017 используется (актуально при закомментированной настройке FEEDBACK_DIRECT_MODE)
//----------------------------------------------------------------------------------------------------------------
// адреса расширителей MCP23017, через запятую, кол-вом COUNT_OF_MCP23017_EXTENDERS (актуально при закомментированной настройке FEEDBACK_DIRECT_MODE)
// 0 - первый адрес 0x20, 1 - второй адрес 0x21 и т.п.
#define MCP23017_ADDRESSES 4,5
//----------------------------------------------------------------------------------------------------------------
// настройки адресации модуля обратной связи в системе, актуально при закомментированной настройке FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
/*
 адресация осуществляется путём чтения определённых каналов микросхему MCP23017 (актуально при закомментированной настройке FEEDBACK_DIRECT_MODE) 
 или путём регистрации на контроллере.
 Если используется адресация через MCP23017 - каждый их этих каналов заведён на переключатель, всего каналов - 4.
 Таким образом, кол-во адресов - 16, т.е. можно использовать максимально 16 модулей.
 */

// закомментировать эту строчку, если не нужна адресация переключателями на плате - 
// в этом случае адресация будет осуществляться через регистрацию в контроллере
//#define ADDRESS_THROUGH_MCP 

#define ADDRESS_MCP_NUMBER 0 // номер микросхемы MCP23017, обслуживающей каналы адресации, на шине I2C
#define ADDRESS_CHANNEL1 1 // номер канала микросхемы MCP23017 для первого бита адреса
#define ADDRESS_CHANNEL2 2 // номер канала микросхемы MCP23017 для второго бита адреса
#define ADDRESS_CHANNEL3 3 // номер канала микросхемы MCP23017 для третьего бита адреса
#define ADDRESS_CHANNEL4 4 // номер канала микросхемы MCP23017 для четвёртого бита адреса

//----------------------------------------------------------------------------------------------------------------
// настройки привязок управления (актуально при закомментированной настройке FEEDBACK_DIRECT_MODE) 
// каналами активности инклинометров на шине I2C 
// (управление каналами I2C осуществляется через микросхему PCA9516A)
// используемые инклинометры - HMC5883
// ВНИМАНИЕ! кол-во записей - равно WINDOWS_SERVED !!!
// записи - через запятую, одна запись имеет формат { MCP_NUMBER, CHANNEL_NUMBER }, где
// MCP_NUMBER - номер микросхемы по порядку (0 - первая микросхема, 1 - вторая и т.п.),
// CHANNEL_NUMBER - номер канала микросхемы, который обслуживает инклинометр
//----------------------------------------------------------------------------------------------------------------
// по умолчанию -  MCP23017 номер 0 (адрес на шине из MCP23017_ADDRESSES - 4).
// каналы управления линиями инклинометров - 5,6,7,8
#define MCP23017_INCLINOMETER_SETTINGS {0,5}, {0,6}, {0,7}, {0,8} 
//----------------------------------------------------------------------------------------------------------------
// настройки привязок концевиков крайних положений (актуально при закомментированной настройке FEEDBACK_DIRECT_MODE)
// концевики крайних положений обслуживаются через MCP23017, их количество равно WINDOWS_SERVED,
// для каждого окна - два концевика на открытие и закрытие.
// записи - через запятую, каждая запись имеет формат { MCP_NUMBER, MCP_CHANNEL_OPEN_SWITCH, MCP_CHANNEL_CLOSE_SWITCH }, где
// MCP_NUMBER - номер микросхемы по порядку (0 - первая микросхема, 1 - вторая и т.п.),
// MCP_CHANNEL_OPEN_SWITCH - канал микросхемы, с которого читается уровень концевика открытия
// MCP_CHANNEL_CLOSE_SWITCH - канал микросхемы, с которого читается уровень концевика закрытия
//----------------------------------------------------------------------------------------------------------------
// по умолчанию -  MCP23017 номер 1 (адрес на шине из MCP23017_ADDRESSES - 5).
// каналы считывания позиций концевиков - 0,1     2,3      4,5      6,7
#define MCP23017_SWITCH_SETTINGS        {0,0,1}, {0,2,3}, {0,4,5}, {0,6,7} 
//----------------------------------------------------------------------------------------------------------------
// настройки управления приёмом/передачей RS485
//----------------------------------------------------------------------------------------------------------------
#define RS485_MCP23017_NUMBER 0 // номер микросхемы MCP23017, через которую управляем направлением приём/передача по RS485
#define RS485_MCP23017_CNANNEL 9 // номер канала микросхемы MCP23017, через который управляем направлением приём/передача по RS485

#define USE_DIRECT_RS485_DE_PIN // если раскомментировано, то управление пином приёма/передачи RS485 будет идти по пину ниже,
// иначе - по каналу MCP23017 (настройки чуть выше)
#define DIRECT_RS485_PIN 4 // номер пина для прямого управления переключением приём/передача RS485
//----------------------------------------------------------------------------------------------------------------
// настройки уровней
//----------------------------------------------------------------------------------------------------------------
#define INCLINOMETER_CHANNEL_OFF LOW // уровень, нужный для выключения канала I2C инклинометров, не участвующих в опросе позиции конкретного окна
#define INCLINOMETER_CHANNEL_ON HIGH // уровень, нужный для включения канала I2C инклинометров, не участвующих в опросе позиции конкретного окна

#define OPEN_SWITCH_TRIGGERED_LEVEL LOW // уровень, при котором концевик открытия считается сработавшим
#define CLOSE_SWITCH_TRIGGERED_LEVEL LOW // уровень, при котором концевик закрытия считается сработавшим
//----------------------------------------------------------------------------------------------------------------
// КОНЕЦ НАСТРОЕК ОБРАТНОЙ СВЯЗИ
//----------------------------------------------------------------------------------------------------------------

//----------------------------------------------------------------------------------------------------------------
// настройки RS-485
//----------------------------------------------------------------------------------------------------------------
//#define USE_RS485_GATE // закомментировать, если не нужна работа через RS-485
#define RS485_SPEED 57600 // скорость работы по RS-485
//----------------------------------------------------------------------------------------------------------------
// настройки nRF
//----------------------------------------------------------------------------------------------------------------
#define USE_NRF // закомментировать, если не надо работать через nRF.
/*
 nRF для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались в слотах, или с RS-485, или ещё где.
 */
#define NRF_CE_PIN 9 // номер пина CE для модуля nRF
#define NRF_CSN_PIN 10 // номер пина CSN для модуля nRF
#define DEFAULT_RF_CHANNEL 19 // номер канала для nRF по умолчанию
//#define NRF_AUTOACK_INVERTED // раскомментировать эту строчку здесь и в главной прошивке, если у вас они не коннектятся. 
// Иногда auto aсk в китайских модулях имеет инвертированное значение.
//----------------------------------------------------------------------------------------------------------------
// настройки LoRa
//----------------------------------------------------------------------------------------------------------------
//#define USE_LORA // закомментировать, если не надо работать через LoRa.
/*
 LoRa для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались в слотах, или с RS-485, или ещё где.
 */
#define LORA_SS_PIN 10 // пин SS для LoRa
#define LORA_RESET_PIN 9 // пин Reset для LoRa
#define LORA_FREQUENCY 868E6 // частота работы (433E6, 868E6, 915E6)
#define LORA_TX_POWER 17 // мощность передатчика (1 - 17)
//----------------------------------------------------------------------------------------------------------------
// настройки
//----------------------------------------------------------------------------------------------------------------
#define ROM_ADDRESS 0 // по какому адресу у нас настройки скратчпада?
#define INCLINOMETERS_ADDRESS 100 // по какому адресу у нас настройки инклинометров
//----------------------------------------------------------------------------------------------------------------
// настройки инициализации, привязка слотов к пинам и первоначальному состоянию
//----------------------------------------------------------------------------------------------------------------
#define RELAY_ON HIGH// уровень для включения нагрузки на канале
#define RELAY_OFF LOW // уровень для выключения нагрузки на канале
//----------------------------------------------------------------------------------------------------------------
// настройки привязки номеров локальных пинов к слотам контроллера.
// нужны для трансляции типа слота в конкретный пин, на которому будет управление нагрузкой
//----------------------------------------------------------------------------------------------------------------
/* 
Пины для платы исполнительного модуля Сергея HANTER333

 D6
 A0
 A1
 D3
 A2
 A3
 A4
 A5
 
 По умолчанию на пинах - низкий уровень !!!
 */
//----------------------------------------------------------------------------------------------------------------
SlotSettings SLOTS[8] = 
{
  {6,   RELAY_OFF} // пин номер такой-то, начальное состояние RELAY_OFF
 ,{7,   RELAY_OFF} // и т.д. 0 вместо номера пина - нет поддержки привязки канала к пину
 ,{3,   RELAY_OFF}
 ,{5,   RELAY_OFF}
 ,{A0,  RELAY_OFF}
 ,{A1,  RELAY_OFF}
 ,{A2,  RELAY_OFF}
 ,{A3,  RELAY_OFF}
  
};
//----------------------------------------------------------------------------------------------------------------
#define _DEBUG // раскомментировать для отладочного режима (плюётся в Serial, RS485, ясное дело, в таком режиме не работает)
//----------------------------------------------------------------------------------------------------------------
// Дальше лазить - неосмотрительно :)
//----------------------------------------------------------------------------------------------------------------
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// \/
//----------------------------------------------------------------------------------------------------------------
// ДАЛЕЕ ИДУТ СЛУЖЕБНЫЕ НАСТРОЙКИ И КОД - МЕНЯТЬ С ПОЛНЫМ ПОНИМАНИЕМ ТОГО, ЧТО ХОДИМ СДЕЛАТЬ !!!
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_FEEDBACK
  #ifdef FEEDBACK_DIRECT_MODE
    #undef ADDRESS_THROUGH_MCP
    #undef WINDOWS_SERVED
    #define WINDOWS_SERVED 1
    #pragma message "Switch to ONE window served due to FEEDBACK_DIRECT_MODE..."
  #endif
#endif
//----------------------------------------------------------------------------------------------------------------
t_scratchpad scratchpadS, scratchpadToSend;
volatile char* scratchpad = (char *)&scratchpadS; //что бы обратиться к scratchpad как к линейному массиву
volatile bool scratchpadReceivedFromMaster = false; // флаг, что мы получили данные с мастера
//----------------------------------------------------------------------------------------------------------------
// Настройки 1-Wire
//----------------------------------------------------------------------------------------------------------------
Pin oneWireData(2); // на втором пине у нас висит 1-Wire
const byte owROM[7] = { 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02 }; // адрес датчика, менять не обязательно, т.к. у нас не честный 1-Wire
// команды 1-Wire
const byte COMMAND_START_CONVERSION = 0x44; // запустить конвертацию
const byte COMMAND_READ_SCRATCHPAD = 0xBE; // попросили отдать скратчпад мастеру
const byte COMMAND_WRITE_SCRATCHPAD = 0x4E; // попросили записать скратчпад, следом пойдёт скратчпад
const byte COMMAND_SAVE_SCRATCHPAD = 0x25; // попросили сохранить скратчпад в EEPROM
enum DeviceState {
  DS_WaitingReset,
  DS_WaitingCommand,
  DS_ReadingScratchpad,
  DS_SendingScratchpad
};
volatile DeviceState state = DS_WaitingReset;
volatile byte scratchpadWritePtr = 0; // указатель на байт в скратчпаде, куда надо записать пришедший от мастера байт
volatile byte scratchpadNumOfBytesReceived = 0; // сколько байт прочитали от мастера
//----------------------------------------------------------------------------------------------------------------
RS485Packet rs485Packet; // пакет, в который мы принимаем данные
volatile byte* rsPacketPtr = (byte*) &rs485Packet;
volatile byte  rs485WritePtr = 0; // указатель записи в пакет
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_FEEDBACK
//----------------------------------------------------------------------------------------------------------------
typedef enum
{
  dirNothing,
  dirOpen,
  dirClose
  
} Direction;
//----------------------------------------------------------------------------------------------------------------
typedef struct
{
 // Direction lastDirection;
  Direction currentDirection;
  
} WindowMoveStatus;
//----------------------------------------------------------------------------------------------------------------
 // у нас 8 слотов, каждый слот может быть привязан к каналу окна, однако - для обратной связи мы используем
 // только 4 окна максимум, т.е. 8 концевиков. Поэтому приходящую от контроллера информацию по состоянию
 // каналов окон мы всегда можем транслировать на номер окна, к которому привязаны концевики
WindowMoveStatus moveStatus[WINDOWS_SERVED];
//----------------------------------------------------------------------------------------------------------------
#include "MCP23017.h"
#include "HMC5883.h"
#include <EEPROM.h>
//----------------------------------------------------------------------------------------------------------------
#define RECORD_HEADER1 0xDE
#define RECORD_HEADER2 0xAF
#define RECORD_HEADER3 0x0D
#define NO_FEEDBACK_VALUE -31111
//----------------------------------------------------------------------------------------------------------------
int GetFeedbackStoreAddress(uint8_t inclinometerNumber)
{
  int addr = INCLINOMETERS_ADDRESS;
  int recordSize = 3 + sizeof(int)*2;
  addr += recordSize*inclinometerNumber;

  return addr;
}
//----------------------------------------------------------------------------------------------------------------
bool CheckFeedbackHeaders(uint8_t inclinometerNumber)
{
  int addr =  GetFeedbackStoreAddress(inclinometerNumber);
 
  uint8_t h1 = EEPROM.read(addr); addr++;
  uint8_t h2 = EEPROM.read(addr); addr++;
  uint8_t h3 = EEPROM.read(addr);

  return (h1 == RECORD_HEADER1 && h2 == RECORD_HEADER2 && h3 == RECORD_HEADER3);
}
//----------------------------------------------------------------------------------------------------------------
void WriteFeedbackHeader(uint8_t inclinometerNumber)
{
  int addr =  GetFeedbackStoreAddress(inclinometerNumber);
  
  EEPROM.update(addr,RECORD_HEADER1); addr++;
  EEPROM.update(addr,RECORD_HEADER2); addr++;
  EEPROM.update(addr,RECORD_HEADER3); addr++;
  
}
//----------------------------------------------------------------------------------------------------------------
void SaveFeedbackBorder(uint8_t inclinometerNumber, int value, bool isStartInterval)
{

  #ifdef _DEBUG
    Serial.print(F("SaveFeedbackBorder, window #"));
    Serial.print(inclinometerNumber);
    Serial.print(F("; value="));
    Serial.print(value);
    Serial.print(F("; first value? "));
    if(isStartInterval)
      Serial.println(F("yes"));
    else
      Serial.println(F("no"));
  #endif
  
  WriteFeedbackHeader(inclinometerNumber);
  
  int addr =  GetFeedbackStoreAddress(inclinometerNumber);
  addr += 3; // skip header
  
  if(!isStartInterval)
    addr += sizeof(int); // skip first interval

  byte* b = (byte*) &value;
  for(size_t i=0;i<sizeof(int);i++)
  {
    EEPROM.update(addr,*b);
    addr++;
    b++;
  }
}
//----------------------------------------------------------------------------------------------------------------
void ReadFeedbackBorders(uint8_t inclinometerNumber, int& from, int& to)
{
  from = NO_FEEDBACK_VALUE;
  to = NO_FEEDBACK_VALUE;

  if(!CheckFeedbackHeaders(inclinometerNumber))
  {
    #ifdef _DEBUG
      Serial.println(F("NO BORDERS SAVED!!!"));
    #endif
    return;
  }

  int addr =  GetFeedbackStoreAddress(inclinometerNumber);
  addr += 3; // skip header

  byte* b = (byte*) &from;

  for(size_t i=0;i<sizeof(int);i++)
  {
    *b++ = EEPROM.read(addr);
    addr++;
    
  }

  b = (byte*) &to;

  for(size_t i=0;i<sizeof(int);i++)
  {
    *b++ = EEPROM.read(addr);
    addr++;
  }
    
}
//----------------------------------------------------------------------------------------------------------------
WindowStatus windowStatuses[WINDOWS_SERVED];
//----------------------------------------------------------------------------------------------------------------
#ifndef FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
Adafruit_MCP23017* mcpExtenders[COUNT_OF_MCP23017_EXTENDERS] = {NULL};
byte mcpAddresses[COUNT_OF_MCP23017_EXTENDERS] = {MCP23017_ADDRESSES};
//----------------------------------------------------------------------------------------------------------------
void InitMCP23017()
{
  #ifdef _DEBUG
    Serial.println(F("Init MCP23017..."));
  #endif
  
  for(byte i=0;i<COUNT_OF_MCP23017_EXTENDERS;i++)
  {
    mcpExtenders[i] = new Adafruit_MCP23017;
    mcpExtenders[i]->begin(mcpAddresses[i]);
  }

  #ifdef _DEBUG
    Serial.println(F("MCP23017 inited!"));
  #endif
  
}
//----------------------------------------------------------------------------------------------------------------
#endif // FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
byte moduleAddress = 0;
//----------------------------------------------------------------------------------------------------------------
void ReadModuleAddress()
{
  #ifdef _DEBUG
    Serial.println(F("Read module address..."));
  #endif

 #ifdef ADDRESS_THROUGH_MCP // адресуемся переключателями на плате

    #ifdef _DEBUG
      Serial.println(F("Read address from MCP..."));
    #endif

    Adafruit_MCP23017* mcp = mcpExtenders[ADDRESS_MCP_NUMBER];
  
    mcp->pinMode(ADDRESS_CHANNEL4,INPUT);
    byte bit1 = mcp->digitalRead(ADDRESS_CHANNEL4);
  
    mcp->pinMode(ADDRESS_CHANNEL3,INPUT);
    byte bit2 = mcp->digitalRead(ADDRESS_CHANNEL3);
  
    mcp->pinMode(ADDRESS_CHANNEL2,INPUT);
    byte bit3 = mcp->digitalRead(ADDRESS_CHANNEL2);
  
    mcp->pinMode(ADDRESS_CHANNEL1,INPUT);
    byte bit4 = mcp->digitalRead(ADDRESS_CHANNEL1);
  
    moduleAddress = bit1 | (bit2 << 1) | (bit3 << 2) | (bit4 << 3);

 #else
  #ifdef _DEBUG
    Serial.println(F("Read address from EEPROM..."));
  #endif 
  // адресуемся чтением с конфига
  // старшие 4 бита байта config - это наш адрес
  moduleAddress = scratchpadS.config & 0xF0;
  if(moduleAddress > 15)
    moduleAddress = 0;  
 #endif

  #ifdef _DEBUG
    Serial.print(F("Module address is: "));
    Serial.println(moduleAddress);
  #endif    
}
//----------------------------------------------------------------------------------------------------------------
  #ifdef FEEDBACK_DIRECT_MODE
    #ifdef USE_INCLINOMETERS
      HMC5883* compass = NULL;
    #endif
  #else
    #ifdef USE_INCLINOMETERS
    InclinometerSettings inclinometers[WINDOWS_SERVED] = {MCP23017_INCLINOMETER_SETTINGS};
    HMC5883* compasses[WINDOWS_SERVED] = {NULL};
    #endif
    FeedbackEndstop endstops[WINDOWS_SERVED] = { MCP23017_SWITCH_SETTINGS };
  #endif
//----------------------------------------------------------------------------------------------------------------
  #ifndef FEEDBACK_DIRECT_MODE
    #ifdef USE_INCLINOMETERS
      void TurnInclinometerOff(InclinometerSettings& is)
      {
       mcpExtenders[is.mcpNumber]->digitalWrite(is.mcpChannel,INCLINOMETER_CHANNEL_OFF); 
      }
      //----------------------------------------------------------------------------------------------------------------
      void TurnInclinometerOn(InclinometerSettings& is)
      {
        mcpExtenders[is.mcpNumber]->digitalWrite(is.mcpChannel,INCLINOMETER_CHANNEL_ON); 
      }
      //----------------------------------------------------------------------------------------------------------------
      void TurnInclinometersOff()
      {
        for(byte i=0;i<WINDOWS_SERVED;i++)
        {
            InclinometerSettings is = inclinometers[i];
            TurnInclinometerOff(is);
        }
      }
    #endif // USE_INCLINOMETERS
    //----------------------------------------------------------------------------------------------------------------
  #endif // !FEEDBACK_DIRECT_MODE
//----------------------------------------------------------------------------------------------------------------
void GetWindowsStatus(byte windowNumber, byte& isCloseSwitchTriggered, byte& isOpenSwitchTriggered, byte& hasPosition, byte& position)
{
  WindowStatus* ws = &(windowStatuses[windowNumber]);
  isCloseSwitchTriggered = ws->isCloseSwitchTriggered;
  isOpenSwitchTriggered = ws->isOpenSwitchTriggered;
  hasPosition = ws->hasPosition;
  position = ws->position;
  
 
  #ifdef _DEBUG
    Serial.print(F("Window #"));
    Serial.print(windowNumber);
    Serial.print(F(" status: hasPosition="));
    Serial.print(hasPosition);
    Serial.print(F("; position="));
    Serial.print(position);
    Serial.print(F("; close switch="));
    Serial.print(isCloseSwitchTriggered);
    Serial.print(F("; open switch="));
    Serial.println(isOpenSwitchTriggered);
  #endif

}
//----------------------------------------------------------------------------------------------------------------
void TurnWindowMotorOff(byte window)
{
     for(byte i=0;i<8;i++)
     {
        UniSlotData* slotData = &(scratchpadS.slots[i]);
        byte slotType = slotData->slotType;
        if(slotType == slotWindowLeftChannel || slotType == slotWindowRightChannel)
        {
          // у нас есть номер окна, к которому привязан слот. Однако - этот номер окна
          // не означает номер канала в нашем представлении. В нашем представлении
          // номер канала - это i/2, т.к. у нас максимум 4 окна на концевиках,
          // следовательно, мы в любом случае получим номер канала, которому
          // надо выключить мотор, просто поделив номер слота на 2.
          byte actualWindowNumber = i/2;
          if(actualWindowNumber == window && actualWindowNumber < WINDOWS_SERVED)
          {
            SLOTS[i].State = RELAY_OFF;
            digitalWrite(SLOTS[i].Pin, RELAY_OFF);
          }
        }
     }
}
//----------------------------------------------------------------------------------------------------------------
bool EndstopTriggered(byte windowNumber,bool isCloseEndstop)
{

  if(windowNumber >= WINDOWS_SERVED)
    return false;
  
   #ifndef FEEDBACK_DIRECT_MODE

      FeedbackEndstop endstop = endstops[windowNumber];
      
      Adafruit_MCP23017* mcp = mcpExtenders[endstop.mcpNumber];
    
      if(isCloseEndstop)
        return mcp->digitalRead(endstop.closeSwitchChannel) == CLOSE_SWITCH_TRIGGERED_LEVEL;
      else
        return mcp->digitalRead(endstop.openSwitchChannel) == OPEN_SWITCH_TRIGGERED_LEVEL;
      
  #else
    UNUSED(windowNumber);

    if(isCloseEndstop)
      return digitalRead(CLOSE_SWITCH_PIN) == CLOSE_SWITCH_TRIGGERED_LEVEL;
    else
      return digitalRead(OPEN_SWITCH_PIN) == OPEN_SWITCH_TRIGGERED_LEVEL;
        
  #endif 
}
//----------------------------------------------------------------------------------------------------------------
void UpdateWindowStatus(byte windowNumber)
{
  #ifndef FEEDBACK_DIRECT_MODE

    #ifdef _DEBUG
      Serial.print(F("UpdateWindowStatus - MCP MODE, window #"));
      Serial.println(windowNumber);
    #endif
    
    #ifdef USE_INCLINOMETERS
      TurnInclinometersOff();
      InclinometerSettings inclinometer = inclinometers[windowNumber];
    
      // включаем инклинометр на шине I2C
      TurnInclinometerOn(inclinometer);
    #endif
      
  // теперь читаем позицию концевиков
  
  windowStatuses[windowNumber].isCloseSwitchTriggered = EndstopTriggered(windowNumber,true) ? 1 : 0;
  windowStatuses[windowNumber].isOpenSwitchTriggered = EndstopTriggered(windowNumber,false) ? 1 : 0;

  switch(moveStatus[windowNumber].currentDirection)
  {
     case dirNothing:
     {
     }
     break;

     case dirClose: // закрываемся
     {
        // пока закрываемся - надо игнорировать концевик открытия
        windowStatuses[windowNumber].isOpenSwitchTriggered = false;
        
        if(windowStatuses[windowNumber].isCloseSwitchTriggered)
        {
           // moveStatus[windowNumber].lastDirection = moveStatus[windowNumber].currentDirection;
            moveStatus[windowNumber].currentDirection = dirNothing;
        }
     }
     break;

     case dirOpen: // открываемся
     {
        // пока открываемся - надо игнорировать концевик закрытия
        windowStatuses[windowNumber].isCloseSwitchTriggered = false; 
             
        if(windowStatuses[windowNumber].isOpenSwitchTriggered)
        {
           // moveStatus[windowNumber].lastDirection = moveStatus[windowNumber].currentDirection;
            moveStatus[windowNumber].currentDirection = dirNothing;
        }
     }
     break;
    
  } // switch
  

  // читаем с инклинометра
  #ifdef USE_INCLINOMETERS
  int x,y,z;
  compasses[windowNumber]->read(&x,&y,&z);
  #endif

  #else // direct mode

    #ifdef _DEBUG
      Serial.print(F("UpdateWindowStatus - DIRECT MODE, window #"));
      Serial.println(windowNumber);
    #endif
  
    windowStatuses[windowNumber].isCloseSwitchTriggered = EndstopTriggered(windowNumber,true) ? 1 : 0;
    windowStatuses[windowNumber].isOpenSwitchTriggered = EndstopTriggered(windowNumber,false) ? 1 : 0; 

  
  switch(moveStatus[windowNumber].currentDirection)
  {
     case dirNothing:
     {
     }
     break;

     case dirClose: // закрываемся
     {
        // пока закрываемся - надо игнорировать концевик открытия
        windowStatuses[windowNumber].isOpenSwitchTriggered = false;
        
        if(windowStatuses[windowNumber].isCloseSwitchTriggered)
        {
            //moveStatus[windowNumber].lastDirection = moveStatus[windowNumber].currentDirection;
            moveStatus[windowNumber].currentDirection = dirNothing;
        }
     }
     break;

     case dirOpen: // открываемся
     {
        // пока открываемся - надо игнорировать концевик закрытия
        windowStatuses[windowNumber].isCloseSwitchTriggered = false; 
             
        if(windowStatuses[windowNumber].isOpenSwitchTriggered)
        {
            //moveStatus[windowNumber].lastDirection = moveStatus[windowNumber].currentDirection;
            moveStatus[windowNumber].currentDirection = dirNothing;
        }
     }
     break;
    
  } // switch  

    #ifdef USE_INCLINOMETERS
    int x,y,z;
    compass->read(&x,&y,&z);
    #endif
    
  #endif // direct mode

    #ifdef _DEBUG
      if(windowStatuses[windowNumber].isCloseSwitchTriggered)
        Serial.println(F("Close switch triggered!"));
        
      if(windowStatuses[windowNumber].isOpenSwitchTriggered)
        Serial.println(F("Open switch triggered!"));

      #ifdef USE_INCLINOMETERS  
      Serial.print(F("UpdateWindowStatus, Z is: "));
      Serial.println(z);
      #endif
      
    #endif 

  #ifdef USE_INCLINOMETERS
  // если сработал один из концевиков - сохраняем значение оси Z с компаса
  // первым у нас идёт концевик закрытия, т.к. мы меряем от закрытия (0%)
  if(windowStatuses[windowNumber].isCloseSwitchTriggered)
  {
    SaveFeedbackBorder(windowNumber,z,true);
  }

  // если сработал концевик открытия - также сохраняем значение в EEPROM
  if(windowStatuses[windowNumber].isOpenSwitchTriggered)
  {
    SaveFeedbackBorder(windowNumber,z,false);
  }

   // читаем значения интервалов
   int fromInterval, toInterval;
   ReadFeedbackBorders(windowNumber,fromInterval,toInterval);

    #ifdef _DEBUG
      Serial.print(F("UpdateWindowStatus, borders are: FROM="));
      Serial.print(fromInterval);
      Serial.print(F("; TO="));
      Serial.println(toInterval);
    #endif 
   
   bool hasPosition = (fromInterval != NO_FEEDBACK_VALUE && toInterval != NO_FEEDBACK_VALUE);

   if(hasPosition)
    hasPosition = (fromInterval != toInterval);

  windowStatuses[windowNumber].hasPosition = hasPosition;
  windowStatuses[windowNumber].position = 0;


  if(hasPosition)
  {
    // для начала - приведём Z в интервал
    if(fromInterval < toInterval) // [-100, 100]
    {
      if(z < fromInterval)
        z = fromInterval;

      if(z > toInterval)
        z = toInterval;
    }
    else // [100, -100]
    {
      if( z > fromInterval)
        z = fromInterval;

      if(z < toInterval)
        z = toInterval;
        
    }

    // теперь у нас z - в интервале, и мы можем посчитать, на какой точке интервала, в процентах,
    // находится текущее значение Z

    // приводим интервал в положительную сторону
    int zBegin = fromInterval + abs(fromInterval);
    int zEnd = toInterval + abs(fromInterval);
    z = z + abs(fromInterval);

    // нормализуем интервал, чтобы направление вектора было вправо
    int maxVal = max(zBegin,zEnd);
    int minVal = min(zBegin,zEnd);
    zBegin = minVal;
    zEnd = maxVal;

    // приводим к началу точки отсчёта - нулю
    zEnd -= zBegin;
    z -= zBegin;

    // мы получили вектор, смотрящий вправо, и теперь можем получить значение z в процентах
    // между началом и концом вектора
    unsigned long z100 = z;
    z100 *= 100;  
    int zPercents = z100/zEnd;

    // но для случая fromInterval > toInterval у нас процентовка отсчитывается от 100%,
    // поэтому меняем процентовку
    if(fromInterval > toInterval)
      zPercents = 100 - zPercents;
    
   // вычислили процентовку, приводим её к дискрету в 5%, чтобы убрать дребезг датчика
   int discreteStep = 5;
   int halfStep = discreteStep/2;

   if(zPercents <= halfStep)
   {
     // близко к нулю
     zPercents = 0;
   }
   else
   if(zPercents >= (100 - halfStep))
   {
      // близко к 100
      zPercents = 100;      
   }
   else
   {
     // приводим к дискретности в 5%

     int value = zPercents/discreteStep;
     int fract = zPercents%discreteStep;
     if(fract > halfStep)
      value++;
  
      zPercents = value*discreteStep; // привели к дискретности в 5%
   }

    // сохраняем текущую позицию окна
    windowStatuses[windowNumber].position = zPercents;

    #ifdef _DEBUG
      Serial.print(F("UpdateWindowStatus, window #"));
      Serial.print(windowNumber);
      Serial.print(F("; position is: "));
      Serial.print(zPercents);
      Serial.println("%");
    #endif    
    
  } // if(hasPosition)
  #ifdef _DEBUG
  else
  {
      Serial.print(F("UpdateWindowStatus, window #"));
      Serial.print(windowNumber);
      Serial.println(F(" has no feedback position!"));
  }
  #endif // _DEBUG  

 #else
 
    windowStatuses[windowNumber].hasPosition = false;

 #endif // USE_INCLINOMETERS
   
}
//----------------------------------------------------------------------------------------------------------------
void FillRS485PacketWithData(WindowFeedbackPacket* packet) // заполняем пакет обратной связи данными для RS485
{
  byte isCloseSwitchTriggered = 0;
  byte isOpenSwitchTriggered = 0;
  byte hasPosition = 0;
  byte position = 0;

  byte currentByteNumber = 0;
  int8_t currentBitNumber = 7;

  memset(packet->windowsStatus,0,20);
 
  
  for(int i=0;i<WINDOWS_SERVED;i++)
  {
    GetWindowsStatus(i, isCloseSwitchTriggered, isOpenSwitchTriggered, hasPosition, position);
    // тут мы получили состояния. У нас есть номер байта и номер бита,
    // с которого надо писать в поток. Как только бит исчерпан - переходим на следующий байт.
    // position  у нас занимает старшие биты, причём самый старший там - нам не нужен

    for(int k=6;k>=0;k--)
    {
      byte b = bitRead(position,k);
      packet->windowsStatus[currentByteNumber] |= (b << currentBitNumber);
      
      currentBitNumber--;
      if(currentBitNumber < 0)
      {
        currentBitNumber = 7;
        currentByteNumber++;
      }
      
    } // for

    // записали позицию, пишем информацию о том, есть ли позиция окна (третий по старшинству бит)
    packet->windowsStatus[currentByteNumber] |= (hasPosition << currentBitNumber);
    currentBitNumber--;
    if(currentBitNumber < 0)
    {
      currentBitNumber = 7;
      currentByteNumber++;
    }
    // теперь пишем информацию о концевике закрытия (второй бит)
    packet->windowsStatus[currentByteNumber] |= (isCloseSwitchTriggered << currentBitNumber);
    currentBitNumber--;
    if(currentBitNumber < 0)
    {
      currentBitNumber = 7;
      currentByteNumber++;
    }
    
    // теперь пишем информацию о концевике открытия (первый бит)
    packet->windowsStatus[currentByteNumber] |= (isOpenSwitchTriggered << currentBitNumber);
    currentBitNumber--;
    if(currentBitNumber < 0)
    {
      currentBitNumber = 7;
      currentByteNumber++;
    }
    
  } // for
 
}
//----------------------------------------------------------------------------------------------------------------
void ProcessFeedbackPacket()
{  
 
  WindowFeedbackPacket* packet = (WindowFeedbackPacket*) &(rs485Packet.data);
  
  // тут обрабатываем входящий пакет запроса о позиции окон
 
  if(packet->moduleNumber != moduleAddress)
  {      
    return; // пакет на для нас
  }

     rs485Packet.direction = RS485FromSlave;
     rs485Packet.type = RS485WindowsPositionPacket;
     packet->moduleNumber = moduleAddress;
     packet->windowsSupported = WINDOWS_SERVED;

     // тут заполняем пакет данными
     FillRS485PacketWithData(packet);     

     // подсчитываем CRC
     rs485Packet.crc8 = OneWireSlave::crc8((const byte*) &rs485Packet,sizeof(RS485Packet)-1 );

     #ifndef _DEBUG // в дебаг-режиме ничего не отсылаем
     
        // теперь переключаемся на передачу
        RS485Send();
        
        // пишем в порт данные
        Serial.write((const uint8_t *)&rs485Packet,sizeof(RS485Packet));
        
        // ждём окончания передачи
        RS485waitTransmitComplete();
        
        // переключаемся на приём
        RS485Receive();
      
        
    #endif  
   
      
}
//----------------------------------------------------------------------------------------------------------------
void RS485Receive()
{
  
  // переводим контроллер RS-485 на приём
  #ifdef USE_DIRECT_RS485_DE_PIN
    digitalWrite(DIRECT_RS485_PIN,LOW);
  #else
    #ifdef FEEDBACK_DIRECT_MODE
      #error "NO MCP AVAILABLE THROUGH DIRECT MODE, CAN'T COMPILE!"
    #endif    
    mcpExtenders[RS485_MCP23017_NUMBER]->digitalWrite(RS485_MCP23017_CNANNEL,LOW);
  #endif
  
  #ifdef _DEBUG
    Serial.println(F("Switch RS485 to receive."));
  #endif
  
}
//----------------------------------------------------------------------------------------------------------------
void RS485Send()
{
  // переводим контроллер RS-485 на передачу
  #ifdef USE_DIRECT_RS485_DE_PIN
    digitalWrite(DIRECT_RS485_PIN,HIGH);
  #else
    #ifdef FEEDBACK_DIRECT_MODE
      #error "NO MCP AVAILABLE THROUGH DIRECT MODE, CAN'T COMPILE!"
    #endif    
    mcpExtenders[RS485_MCP23017_NUMBER]->digitalWrite(RS485_MCP23017_CNANNEL,HIGH);
  #endif
  
  #ifdef _DEBUG
    Serial.println(F("Switch RS485 to send."));
  #endif

}
//----------------------------------------------------------------------------------------------------------------
void RS485waitTransmitComplete()
{
  // ждём завершения передачи по UART
  while(!(UCSR0A & _BV(TXC0) ));
}
//----------------------------------------------------------------------------------------------------------------
void InitEndstops()
{
  #ifdef _DEBUG
    Serial.println(F("Init endstops...."));
  #endif  

  #ifdef FEEDBACK_DIRECT_MODE
      pinMode(OPEN_SWITCH_PIN,INPUT);
      pinMode(CLOSE_SWITCH_PIN,INPUT);
  #else

    for(byte i=0;i<WINDOWS_SERVED;i++)
    {
      FeedbackEndstop es = endstops[i];
      mcpExtenders[es.mcpNumber]->pinMode(es.openSwitchChannel,INPUT);
      mcpExtenders[es.mcpNumber]->pinMode(es.closeSwitchChannel,INPUT);
    } // for

  #endif

  for(byte i=0;i<WINDOWS_SERVED;i++)
  {
   // moveStatus[i].lastDirection = dirNothing;
    moveStatus[i].currentDirection = dirNothing;
  }
  
  #ifdef _DEBUG
    Serial.println(F("Endstops inited."));
  #endif  
}
//----------------------------------------------------------------------------------------------------------------
void InitInclinometers()
{
#ifdef USE_INCLINOMETERS
  
  #ifdef _DEBUG
    Serial.println(F("Init inclinometers...."));
  #endif  

    #ifdef FEEDBACK_DIRECT_MODE
    
      compass = new HMC5883();
      compass->init();
      // проверяем, есть ли сохранённые данные? Если нет - инициализируем значениями "нет данных"
      if(!CheckFeedbackHeaders(0))
      {
        SaveFeedbackBorder(0,NO_FEEDBACK_VALUE,true);
        SaveFeedbackBorder(0,NO_FEEDBACK_VALUE,false);
      }
      
    #else
    
    for(byte i=0;i<WINDOWS_SERVED;i++)
    {
      InclinometerSettings is = inclinometers[i];
      mcpExtenders[is.mcpNumber]->pinMode(is.mcpChannel,OUTPUT);
      TurnInclinometerOff(is);
    } // for

    for(byte i=0;i<WINDOWS_SERVED;i++)
    {
      InclinometerSettings is = inclinometers[i];
      
      TurnInclinometerOn(is);

      //Тут инициализация инклинометра
      compasses[i] = new HMC5883();
      compasses[i]->init();

      // проверяем, есть ли сохранённые данные? Если нет - инициализируем значениями "нет данных"
      if(!CheckFeedbackHeaders(i))
      {
        SaveFeedbackBorder(i,NO_FEEDBACK_VALUE,true);
        SaveFeedbackBorder(i,NO_FEEDBACK_VALUE,false);
      }

      
      TurnInclinometerOff(is);
    } // for

    #endif
      
  
  #ifdef _DEBUG
    Serial.println(F("Inclinometers inited."));
  #endif

  #endif // USE_INCLINOMETERS    
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_FEEDBACK
//----------------------------------------------------------------------------------------------------------------
void UpdateFromControllerState(ControllerState* state)
{
  
     // у нас есть слепок состояния контроллера, надо искать в слотах привязки
     // при этом интересная ситуация - если мы используем обратную связь
     // и получили слепок состояния контроллера - то нам надо обновить
     // текущее состояние окон, если хотя бы одно окно пришло в движение
     
     for(byte i=0;i<8;i++)
     {
        UniSlotData* slotData = &(scratchpadS.slots[i]);

        byte slotStatus = RELAY_OFF;
        byte slotType = slotData->slotType;
       
        if(slotType == 0 || slotType == 0xFF) // нет привязки
          continue;

        switch(slotType)
        {

            case slotWindowLeftChannel:
            {
              // состояние левого канала окна, в slotLinkedData - номер окна
              byte windowNumber = slotData->slotLinkedData;
              
              if(windowNumber < 16)
              {
                // окна у нас нумеруются от 0 до 15, всего 16 окон.
                // на каждое окно - два бита, для левого и правого канала.
                // следовательно, чтобы получить стартовый бит - надо номер окна
                // умножить на 2.
                byte bitNum = windowNumber*2;           
                if(state->WindowsState & (1 << bitNum))
                {
                  #ifdef USE_FEEDBACK
                    byte actualWindowNumber = i/2;

                    if(actualWindowNumber < WINDOWS_SERVED)
                    {

                    // если здесь выставлена 1 - окно попросили открыться. Выставляем RELAY_ON только тогда, когда не сработал концевик открытия.
                    //moveStatus[actualWindowNumber].lastDirection = moveStatus[actualWindowNumber].currentDirection;
                    moveStatus[actualWindowNumber].currentDirection = dirOpen;
                    // поскольку открываемся - игнорируем концевик закрытия
                    windowStatuses[actualWindowNumber].isCloseSwitchTriggered = false;

                    if(!EndstopTriggered(actualWindowNumber,false))
                      slotStatus = RELAY_ON;
                    }
                    else
                      slotStatus = RELAY_ON;
                      
                  #else
                    slotStatus = RELAY_ON; // выставляем в слоте значение 1
                  #endif
                }
              }
            }
            break;
    
            case slotWindowRightChannel:
            {
              // состояние левого канала окна, в slotLinkedData - номер окна
              byte windowNumber = slotData->slotLinkedData;
              //byte actualWindowNumber = i/2;
              if(windowNumber < 16)
              {
                // окна у нас нумеруются от 0 до 15, всего 16 окон.
                // на каждое окно - два бита, для левого и правого канала.
                // следовательно, чтобы получить стартовый бит - надо номер окна
                // умножить на 2.
                byte bitNum = windowNumber*2;
    
                // поскольку канал у нас правый - его бит идёт следом за левым.
                bitNum++;
                           
                if(state->WindowsState & (1 << bitNum))
                {
                  #ifdef USE_FEEDBACK
                    byte actualWindowNumber = i/2;

                    if(actualWindowNumber < WINDOWS_SERVED)
                    {

                    // если здесь выставлена 1 - окно попросили закрыться. Выставляем RELAY_ON только тогда, когда не сработал концевик закрытия.
                    //moveStatus[actualWindowNumber].lastDirection = moveStatus[actualWindowNumber].currentDirection;
                    moveStatus[actualWindowNumber].currentDirection = dirClose;
                    // поскольку закрываемся - игнорируем концевик открытия
                    windowStatuses[actualWindowNumber].isOpenSwitchTriggered = false;

                    
                    if(!EndstopTriggered(actualWindowNumber,true))
                      slotStatus = RELAY_ON;
                    }
                    else
                      slotStatus = RELAY_ON;
                    
                  #else
                    slotStatus = RELAY_ON; // выставляем в слоте значение 1
                  #endif
                }
              }
            }
            break;
    
            case slotWateringChannel:
            {
              // состояние канала полива, в slotLinkedData - номер канала полива
              byte wateringChannel = slotData->slotLinkedData;
              if(wateringChannel< 16)
              {
                if(state->WaterChannelsState & (1 << wateringChannel))
                  slotStatus = RELAY_ON; // выставляем в слоте значение 1
                  
              }
            }        
            break;
    
            case slotLightChannel:
            {
              // состояние канала досветки, в slotLinkedData - номер канала досветки
              byte lightChannel = slotData->slotLinkedData;
              if(lightChannel < 8)
              {
                if(state->LightChannelsState & (1 << lightChannel))
                  slotStatus = RELAY_ON; // выставляем в слоте значение 1
                  
              }
            }
            break;
    
            case slotPin:
            {
              // получаем статус пина
              byte pinNumber = slotData->slotLinkedData;
              byte byteNum = pinNumber/8;
              byte bitNum = pinNumber%8;

              slotStatus = LOW;
     
              if(byteNum < 16)
              {
                // если нужный бит с номером пина установлен - на пине высокий уровень
                if(state->PinsState[byteNum] & (1 << bitNum))
                  slotStatus = HIGH; // выставляем в слоте значение 1
              }
              
            }
            break;

            default:
              continue;
            
          } // switch


            // проверяем на изменения
             if(slotStatus != SLOTS[i].State)
              {
                 // состояние слота изменилось, запоминаем его
                SLOTS[i].State = slotStatus;
                
                if(SLOTS[i].Pin)
                {
                    digitalWrite(SLOTS[i].Pin, slotStatus);
                }
                
              } // if(slotStatus != SLOTS[i].State)

                    
     } // for  
}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_NRF
//----------------------------------------------------------------------------------------------------------------
uint64_t controllerStatePipe = 0xF0F0F0F0E0LL; // труба, с которой мы слушаем состояние контроллера
//----------------------------------------------------------------------------------------------------------------
#include "RF24.h"
RF24 radio(NRF_CE_PIN,NRF_CSN_PIN);
bool nRFInited = false;
//----------------------------------------------------------------------------------------------------------------
void initNRF()
{  
  // инициализируем nRF
  nRFInited = radio.begin();

  if(nRFInited)
  {
  delay(200); // чуть-чуть подождём

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(scratchpadS.rf_id);
  radio.setRetries(15,15);
  radio.setPayloadSize(sizeof(NRFControllerStatePacket)); // у нас 30 байт на пакет
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(
    #ifdef NRF_AUTOACK_INVERTED
      false
    #else
    true
    #endif
    );

  // открываем трубу состояния контроллера на прослушку
  radio.openReadingPipe(1,controllerStatePipe);
  radio.startListening(); // начинаем слушать

  //radio.printDetails();

  #ifdef _DEBUG
    Serial.print(F("nRF inited, payload size: "));
    Serial.println(sizeof(NRFControllerStatePacket));
  #endif
  } // nRFInited
  else
  {
  #ifdef _DEBUG
    Serial.println(F("Can't init nRF!"));
  #endif
    
  }
  
}
//----------------------------------------------------------------------------------------------------------------
void ProcessNRF()
{
  if(!nRFInited)
    return;
    
  static NRFControllerStatePacket nrfPacket; // наш пакет, в который мы принимаем данные с контроллера
  uint8_t pipe_num = 0; // из какой трубы пришло
  if(radio.available(&pipe_num))
  {
   
     #ifdef _DEBUG
     Serial.println(F("Got packet from nRF"));
     #endif
    
    memset(&nrfPacket,0,sizeof(NRFControllerStatePacket));
    radio.read(&nrfPacket,sizeof(NRFControllerStatePacket));

    if(nrfPacket.controller_id == scratchpadS.controller_id)
    {
       // это пакет с нашего контроллера пришёл, обновляем данные
       byte checksum = OneWireSlave::crc8((const byte*) &nrfPacket,sizeof(NRFControllerStatePacket)-1);
       
       if(checksum == nrfPacket.crc8) // чексумма сошлась
       {         
         switch(nrfPacket.packetType)
         {
            case RS485ControllerStatePacket: // пакет с состоянием контроллера
            {
              UpdateFromControllerState(&(nrfPacket.state));
            }
            break; // RS485ControllerStatePacket
         } // switch
       } // good checksum
    }
  }
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_NRF
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA
//----------------------------------------------------------------------------------------------------------------
#include "LoRa.h"
bool loRaInited = false;
//----------------------------------------------------------------------------------------------------------------
void initLoRa()
{  
  // инициализируем LoRa
  LoRa.setPins(LORA_SS_PIN,LORA_RESET_PIN,-1);
  loRaInited = LoRa.begin(LORA_FREQUENCY);

  if(loRaInited)
  {
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.receive(); // начинаем слушать
  } // nRFInited
  
}
//----------------------------------------------------------------------------------------------------------------
void ProcessLoRa()
{
  if(!loRaInited)
    return;
    
  static NRFControllerStatePacket nrfPacket; // наш пакет, в который мы принимаем данные с контроллера
  int packetSize = LoRa.parsePacket();
  if(packetSize >= sizeof(NRFControllerStatePacket))
  {
    memset(&nrfPacket,0,sizeof(NRFControllerStatePacket));
    LoRa.readBytes((byte*)&nrfPacket,sizeof(NRFControllerStatePacket));

    if(nrfPacket.controller_id == scratchpadS.controller_id)
    {
       // это пакет с нашего контроллера пришёл, обновляем данные
       byte checksum = OneWireSlave::crc8((const byte*) &nrfPacket,sizeof(NRFControllerStatePacket)-1);
       
       if(checksum == nrfPacket.crc8) // чексумма сошлась
       {
         switch(nrfPacket.packetType)
         {
            case RS485ControllerStatePacket: // пакет с состоянием контроллера
            {
              UpdateFromControllerState(&(nrfPacket.state));
            }
            break; // RS485ControllerStatePacket
         } // switch
       } //  // good checksum
    }
  }
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_LORA
//----------------------------------------------------------------------------------------------------------------
void ReadROM()
{
    memset((void*)&scratchpadS,0,sizeof(scratchpadS));
    eeprom_read_block((void*)&scratchpadS, (void*) ROM_ADDRESS, 29);

    // пишем номер канала по умолчанию
    if(scratchpadS.rf_id == 0xFF || scratchpadS.rf_id == 0)
      scratchpadS.rf_id = DEFAULT_RF_CHANNEL; 
      
    scratchpadS.packet_type = uniExecutionClient; // говорим, что это тип пакета - исполнительный модуль
    scratchpadS.packet_subtype = 0;

    // говорим, что никакой адресации не поддерживаем
    scratchpadS.config &= ~2; // второй бит убираем по-любому


    #ifdef USE_FEEDBACK
      #if  !defined(ADDRESS_THROUGH_MCP) || defined(FEEDBACK_DIRECT_MODE)
        // адресуемся из конфига, ставим второй бит
        scratchpadS.config |= 2;
      #endif
    #endif

}
//----------------------------------------------------------------------------------------------------------------
void WriteROM()
{
    eeprom_write_block( (void*)scratchpad,(void*) ROM_ADDRESS,29);
    memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));

    #ifdef USE_NRF
      if(nRFInited)
      {
      // переназначаем канал радио
      radio.stopListening();
      radio.setChannel(scratchpadS.rf_id);
      radio.startListening();
      }
    #endif


}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_RS485_GATE // сказали работать ещё и через RS-485
//----------------------------------------------------------------------------------------------------------------
/*
 Структура пакета, передаваемого по RS-495:
 
   0xAB - первый байт заголовка
   0xBA - второй байт заголовка

   данные, в зависимости от типа пакета
   
   0xDE - первый байт окончания
   0xAD - второй байт окончания

 
 */
//----------------------------------------------------------------------------------------------------------------
bool GotRS485Packet()
{
  // проверяем, есть ли у нас валидный RS-485 пакет
  return rs485WritePtr > ( sizeof(RS485Packet)-1 );
}
//----------------------------------------------------------------------------------------------------------------
void ProcessRS485Packet()
{
  // обрабатываем входящий пакет. Тут могут возникнуть проблемы с синхронизацией
  // начала пакета, поэтому мы сначала ищем заголовок и убеждаемся, что он валидный. 
  // если мы нашли заголовок и он не в начале пакета - значит, с синхронизацией проблемы,
  // и мы должны сдвинуть заголовок в начало пакета, чтобы потом дочитать остаток.
  if(!(rs485Packet.header1 == 0xAB && rs485Packet.header2 == 0xBA))
  {
     // заголовок неправильный, ищем возможное начало пакета
     byte readPtr = 0;
     bool startPacketFound = false;
     while(readPtr < sizeof(RS485Packet))
     {
       if(rsPacketPtr[readPtr] == 0xAB)
       {
        startPacketFound = true;
        break;
       }
        readPtr++;
     } // while

     if(!startPacketFound) // не нашли начало пакета
     {
        rs485WritePtr = 0; // сбрасываем указатель чтения и выходим
        return;
     }

     if(readPtr == 0)
     {
      // стартовый байт заголовка найден, но он в нулевой позиции, следовательно - что-то пошло не так
        rs485WritePtr = 0; // сбрасываем указатель чтения и выходим
        return;       
     } // if

     // начало пакета найдено, копируем всё, что после него, перемещая в начало буфера
     byte writePtr = 0;
     byte bytesWritten = 0;
     while(readPtr < sizeof(RS485Packet) )
     {
      rsPacketPtr[writePtr++] = rsPacketPtr[readPtr++];
      bytesWritten++;
     }

     rs485WritePtr = bytesWritten; // запоминаем, куда писать следующий байт
     return;
         
  } // if
  else
  {
    // заголовок правильный, проверяем окончание
    if(!(rs485Packet.tail1 == 0xDE && rs485Packet.tail2 == 0xAD))
    {
      // окончание неправильное, сбрасываем указатель чтения и выходим
      rs485WritePtr = 0;
      return;
    }
    // данные мы получили, сразу обнуляем указатель записи, чтобы не забыть
    rs485WritePtr = 0;

    // проверяем контрольную сумму
    byte crc = OneWireSlave::crc8((const byte*) rsPacketPtr,sizeof(RS485Packet) - 1);
    if(crc != rs485Packet.crc8)
    {
      // не сошлось, игнорируем
      return;
    }


    // всё в пакете правильно, анализируем и выполняем
    // проверяем, наш ли пакет
    if(rs485Packet.direction != RS485FromMaster) // не от мастера пакет
      return;

    if(!(rs485Packet.type == RS485ControllerStatePacket 

    #ifdef USE_FEEDBACK
    || rs485Packet.type == RS485WindowsPositionPacket
    #endif
    
    )) // пакет не c состоянием контроллера
      return;


      if(rs485Packet.type == RS485ControllerStatePacket)
      {
       // теперь приводим пакет к нужному виду
       ControllerState* state = (ControllerState*) &(rs485Packet.data);
       UpdateFromControllerState(state);
      }
      #ifdef USE_FEEDBACK
      else if(rs485Packet.type == RS485WindowsPositionPacket)
      {
        ProcessFeedbackPacket();
      }
      #endif


    
  } // else
}
//----------------------------------------------------------------------------------------------------------------
void ProcessIncomingRS485Packets() // обрабатываем входящие пакеты по RS-485
{
  while(Serial.available())
  {
    rsPacketPtr[rs485WritePtr++] = (byte) Serial.read();
   
    if(GotRS485Packet())
      ProcessRS485Packet();
  } // while
  
}
//----------------------------------------------------------------------------------------------------------------
void InitRS485()
{
  memset(&rs485Packet,0,sizeof(RS485Packet));

  #ifdef USE_FEEDBACK

  #ifdef _DEBUG
    Serial.println(F("Init RS485 DE pin..."));
  #endif

  #ifdef USE_DIRECT_RS485_DE_PIN
    pinMode(DIRECT_RS485_PIN,OUTPUT);
  #else
  
    #ifdef FEEDBACK_DIRECT_MODE
      #error "NO MCP AVAILABLE THROUGH DIRECT MODE, CAN'T COMPILE!"
    #endif  
    
    mcpExtenders[RS485_MCP23017_NUMBER]->pinMode(RS485_MCP23017_CNANNEL,OUTPUT);
  #endif
  
  RS485Receive();

  #ifdef _DEBUG
    Serial.println(F("RS485 DE pin inited!"));
  #endif
    
  #endif // USE_FEEDBACK

}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_RS485_GATE
//----------------------------------------------------------------------------------------------------------------
void UpdateSlots1Wire()
{  
 #ifdef _DEBUG
  Serial.println(F("Update slots from 1-Wire..."));
 #endif

 //TODO: По 1-Wire состояние концевиков игнорируется !!!
  
  for(byte i=0;i<8;i++)
  {
    byte slotType = scratchpadS.slots[i].slotType;
    if(slotType > 0 && slotType != 0xFF)
    {
      // на слот назначены настройки, надо обновить состояние связанного пина
      byte slotStatus =  scratchpadS.slots[i].slotStatus ? RELAY_ON : RELAY_OFF;
      if(slotType == slotPin)
        slotStatus = scratchpadS.slots[i].slotStatus ? HIGH : LOW;
      
      if(!(slotStatus == HIGH || slotStatus == LOW)) // записан мусор в статусе слота
        continue;
        
      if(slotStatus != SLOTS[i].State)
      {        
        // состояние изменилось
        SLOTS[i].State = slotStatus;
        
        if(SLOTS[i].Pin) {
           #ifdef _DEBUG
            Serial.print(F("Writing "));
            Serial.print(slotStatus);
            Serial.print(F(" to pin "));
            Serial.println(SLOTS[i].Pin);
           #endif          
            digitalWrite(SLOTS[i].Pin, slotStatus);
        }
      }
      
    } // if
    
  } // for
}
//----------------------------------------------------------------------------------------------------------------
void owReceive(OneWireSlave::ReceiveEvent evt, byte data);
//----------------------------------------------------------------------------------------------------------------
void setup()
{
 ReadROM(); // читаем наши настройки
 
 #ifdef USE_RS485_GATE // если сказано работать через RS-485 - работаем 
    Serial.begin(RS485_SPEED);
 #endif

 #ifdef _DEBUG
  #ifndef USE_RS485_GATE 
    Serial.begin(57600);
  #endif
  Serial.println(F("Debug mode..."));
 #endif
  
  #ifdef USE_FEEDBACK

    #ifndef FEEDBACK_DIRECT_MODE
    InitMCP23017(); // инициализируем расширители
    #endif
    InitEndstops(); // инициализируем концевики
    #ifdef USE_INCLINOMETERS
    InitInclinometers(); // инициализируем инклинометры
    #endif
    ReadModuleAddress(); // читаем наш адрес
  
  #endif
  
 #ifdef USE_RS485_GATE // если сказано работать через RS-485 - работаем 
    InitRS485(); // настраиваем RS-485 на приём
 #endif
  
   

    // настраиваем слоты, назначая им конкретные уровни при старте
    for(byte i=0;i<8;i++)
    {
      byte pin = SLOTS[i].Pin;
      if(pin)
      {
        Pin pPin(pin);
        pPin.outputMode();
        pPin.write(SLOTS[i].State);
      }
    } // for

    // настраиваем nRF
    #ifdef USE_NRF
    initNRF();
    #endif

    // настраиваем LoRa
    #ifdef USE_LORA
    initLoRa();
    #endif
    
  scratchpadS.crc8 = OneWireSlave::crc8((const byte*) scratchpad,sizeof(scratchpadS)-1);
  memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));
  
  OWSlave.setReceiveCallback(&owReceive);
  OWSlave.begin(owROM, oneWireData.getPinNumber());    

  #ifdef _DEBUG
    Serial.println(F("Setup done."));
  #endif
}
//----------------------------------------------------------------------------------------------------------------
void owSendDone(bool error) {
  UNUSED(error);
 // закончили посылать скратчпад мастеру
 state = DS_WaitingReset;
}
//----------------------------------------------------------------------------------------------------------------
// обработчик прерывания на пине
void owReceive(OneWireSlave::ReceiveEvent evt, byte data)
{  
 switch (evt)
  {
  case OneWireSlave::RE_Byte:
    switch (state)
    {

     case DS_ReadingScratchpad: // читаем скратчпад от мастера

        // увеличиваем кол-во прочитанных байт
        scratchpadNumOfBytesReceived++;

        // пишем в скратчпад принятый байт
        scratchpad[scratchpadWritePtr] = data;
        // увеличиваем указатель записи
        scratchpadWritePtr++;

        // проверяем, всё ли прочитали
        if(scratchpadNumOfBytesReceived >= sizeof(scratchpadS)) {
          // всё прочитали, сбрасываем состояние на ожидание резета
          state = DS_WaitingReset;
          scratchpadNumOfBytesReceived = 0;
          scratchpadWritePtr = 0;
          scratchpadReceivedFromMaster = true; // говорим, что мы получили скратчпад от мастера
        }
        
     break; // DS_ReadingScratchpad
      
    case DS_WaitingCommand:
      switch (data)
      {
      case COMMAND_START_CONVERSION: // запустить конвертацию
        state = DS_WaitingReset;
        break;

      case COMMAND_READ_SCRATCHPAD: // попросили отдать скратчпад мастеру
        state = DS_SendingScratchpad;
        OWSlave.beginWrite((const byte*)&scratchpadToSend, sizeof(scratchpadToSend), owSendDone);
        break;

      case COMMAND_WRITE_SCRATCHPAD:  // попросили записать скратчпад, следом пойдёт скратчпад
          state = DS_ReadingScratchpad; // ждём скратчпада
          scratchpadWritePtr = 0;
          scratchpadNumOfBytesReceived = 0;
        break;

        case COMMAND_SAVE_SCRATCHPAD: // сохраняем скратчпад в память
          state = DS_WaitingReset;
          WriteROM();
        break;

      } // switch (data)
      break; // case DS_WaitingCommand

      case DS_WaitingReset:
      break;

      case DS_SendingScratchpad:
      break;
    } // switch(state)
    break; 

  case OneWireSlave::RE_Reset:
    state = DS_WaitingCommand;
    break;

  case OneWireSlave::RE_Error:
    state = DS_WaitingReset;
    break;
    
  } // switch (evt)
}
//----------------------------------------------------------------------------------------------------------------
void loop()
{
  if(scratchpadReceivedFromMaster)
  {
    // скратч был получен от мастера, тут можно что-то делать
    scratchpadReceivedFromMaster = false;

    UpdateSlots1Wire(); // обновляем состояние слотов
      
  } // scratchpadReceivedFromMaster

  #ifdef USE_RS485_GATE
    ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
  #endif

  #ifdef USE_NRF
    ProcessNRF(); // обрабатываем входящие пакаты nRF
  #endif

  #ifdef USE_LORA
    ProcessLoRa(); // обрабатываем входящие пакаты LoRa
  #endif  

  #ifdef USE_FEEDBACK

    static bool isFeedbackInited = false;
    if(!isFeedbackInited)
    {
      isFeedbackInited = true;
      for(byte i=0;i<WINDOWS_SERVED;i++)
      {
        UpdateWindowStatus(i);
      }
    }
    else
    {
  
      static unsigned long feedackTimer = millis();
      static int8_t cntr = -1;
      unsigned long curMillis = millis();
      
      if(curMillis - feedackTimer > FEEDBACK_UPDATE_INTERVAL)
      {
        
        feedackTimer = curMillis;
        cntr++;
        if(cntr >= WINDOWS_SERVED)
          cntr = 0;
  
          UpdateWindowStatus(cntr);
        
      } // if
    }  // else

    for(byte i=0;i<WINDOWS_SERVED;i++)
    {

      switch(moveStatus[i].currentDirection)
      {
        case dirNothing:
        break;
        
        case dirClose:
        {
          if(EndstopTriggered(i,true))
          {
            //moveStatus[i].lastDirection = moveStatus[i].currentDirection;
            moveStatus[i].currentDirection = dirNothing;
            TurnWindowMotorOff(i);
          }
        }
        break;
        
        case dirOpen:
        {
          if(EndstopTriggered(i,false))
          {
            //moveStatus[i].lastDirection = moveStatus[i].currentDirection;
            moveStatus[i].currentDirection = dirNothing;
            TurnWindowMotorOff(i);
          }
          
        }
        break;
        
      } // switch
    }
    
  #endif // USE_FEEDBACK


}
//----------------------------------------------------------------------------------------------------------------
