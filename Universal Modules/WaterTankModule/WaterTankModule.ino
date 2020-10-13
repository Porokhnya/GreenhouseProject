//----------------------------------------------------------------------------------------------------------------
/*
ПАМЯТКА ПО КОМАНДАМ КОНТРОЛЛЕРА:

  1. КАЖДАЯ КОМАНДА ЗАКАНЧИВАЕТСЯ ПЕРЕВОДОМ СТРОКИ!

  2. ДЛЯ ПРОСМОТРА ID КОНТРОЛЛЕРА, К КОТОРОМУ БУДЕТ ПРИВЯЗАН МОДУЛЬ, НАДО ПОСЛАТЬ КОМАНДУ

    GET=ID

  3. ДЛЯ УСТАНОВКИ ID КОНТРОЛЛЕРА - ПОСЫЛАЕТСЯ КОМАНДА

    SET=ID|ТУТ_ID

    например,

    SET=ID|24


 ПЕРЕД ОТПРАВКОЙ МОДУЛЯ КЛИЕНТУ ЛУЧШЕ ПРИНУДИТЕЛЬНО ВЫПОЛНИТЬ КОМАНДЫ, УСТАНАВЛИВАЮЩИЕ ID КОНТРОЛЛЕРА
 НА ПАРАМЕТРЫ ПО УМОЛЧАНИЮ, А ИМЕННО - ВЫПОЛНИТЬ ДВЕ КОМАНДЫ:

    SET=ID|0 // привязка к контроллеру с номером 0
    SET=ID|255 // пакеты будут принимать все контроллеры (броадкастовый пакет)
  
*/
//----------------------------------------------------------------------------------------------------------------
#include "UniGlobals.h"
#include "CommandBuffer.h"
#include "CommandParser.h"
#include <OneWire.h>
#include <EEPROM.h>
#include <avr/wdt.h>
//----------------------------------------------------------------------------------------------------------------
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
//----------------------------------------------------------------------------------------------------------------
#define _DEBUG // раскомментировать для отладочного режима (плюётся в Serial)
//----------------------------------------------------------------------------------------------------------------
#define DEFAULT_CONTROLLER_ID 0 // ID контроллера по умолчанию
#define RADIO_SEND_INTERVAL 5000 // интервал между отсылкой данных по радиоканалу, миллисекунд
#define SENSORS_UPDATE_INTERVAL 10000 // интервал обновления датчиков, миллисекунд
#define USE_WATCHDOG // использовать или нет внутренний ватчдог
#define WDT_UPDATE_INTERVAL 5000      // интервал сброса сторожевого таймера
#define CONTROLLER_ID_ADDRESS 55      // по какому адресу в EEPROM храним ID контроллера (1 байт)

#define USE_RANDOM_SEED_PIN // закомментировать, если не надо использовать пин для инициализации генератора псевдослучайных чисел
#define RANDOM_SEED_PIN A0 // какой пин (АНАЛОГОВЫЙ !!!) использовать для инициализации генератора псевдослучайных чисел (пин должен быть висящим в воздухе)

//----------------------------------------------------------------------------------------------------------------
// настройки датчиков уровня
//----------------------------------------------------------------------------------------------------------------
#define LEVEL_SENSORS_PINS  4, 5, 6, A1, A2, A3 // пины для датчиков уровней, через запятую, МАКСИМУМ 10 !!!
#define LEVEL_SENSOR_TRIGGER_LEVEL  HIGH  // уровень срабатывания датчика


//----------------------------------------------------------------------------------------------------------------
// настройки LoRa
//----------------------------------------------------------------------------------------------------------------
/*
 LoRa для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались в слотах, или с RS-485, или ещё где.
 */
#define LORA_SS_PIN 8 // пин SS для LoRa
#define LORA_RESET_PIN 7 // пин Reset для LoRa
#define LORA_FREQUENCY 868E6 // частота работы (433E6, 868E6, 915E6)
#define LORA_TX_POWER 17 // мощность передатчика (1 - 17)

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
int controllerID = DEFAULT_CONTROLLER_ID;
uint32_t radioSendInterval = RADIO_SEND_INTERVAL;
uint32_t lastRadioSentAt = 0;
uint32_t updateTimer = 0;
#ifdef USE_WATCHDOG
uint32_t updateTimerWdt = 0;
#endif // USE_WATCHDOG

bool waterTankValveIsOn = false; // флаг включения клапана заполнения бочки
uint8_t levelSensors[] = {LEVEL_SENSORS_PINS}; // наши пины для датчиков уровня
uint8_t levelSensorsState[] = {LEVEL_SENSORS_PINS}; // состояние пинов датчиков уровня
uint8_t countOfLevelSensors = 0;
//----------------------------------------------------------------------------------------------------------------
CommandBuffer commandBuffer(&Serial);
//----------------------------------------------------------------------------------------------------------------
void fillDataPacket(WaterTankDataPacket* packet)
{
  packet->valveState = waterTankValveIsOn;
  packet->countOfLevelSensors = countOfLevelSensors;

  for(size_t i=0;i<10;i++) // максимум 10 датчиков уровня
  {
    packet->levelSensors[i] = 0xFF; // нет данных с датчика
  }  

  //тут заполняем данные с датчиков уровня
  for(size_t i=0;i<countOfLevelSensors;i++)
  {
    packet->levelSensors[i] = levelSensorsState[i];
  }

  // пакет с данными заполнен и готов к отправке по радиоканалу
}
//----------------------------------------------------------------------------------------------------------------
#include "LoRa.h"
bool loRaInited = false;
//----------------------------------------------------------------------------------------------------------------
void initLoRa()
{
  #ifdef _DEBUG
  Serial.begin(57600);
  #endif
  
  // инициализируем LoRa
  LoRa.setPins(LORA_SS_PIN,LORA_RESET_PIN,-1);
  loRaInited = LoRa.begin(LORA_FREQUENCY);

  if(loRaInited)
  {
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.sleep(); // засыпаем
  } // loRaInited
  
}
//----------------------------------------------------------------------------------------------------------------
void UpdateSensors()
{
  //TODO: обновление данных с датчиков здесь!!!

  // каждый раз при обновлении состояния - инвертируем статус клапана в бочке
  waterTankValveIsOn = !waterTankValveIsOn;

  // теперь проходим по всем пинам датчиков уровня - и читаем их состояние
  for(size_t i=0;i<countOfLevelSensors;i++)
  {
    uint8_t state = digitalRead(levelSensors[i]);
    levelSensorsState[i] = (state == LEVEL_SENSOR_TRIGGER_LEVEL) ? 1 : 0;
  }
}
//----------------------------------------------------------------------------------------------------------------
void sendDataViaLoRa()
{
  if(!loRaInited) {
 #ifdef _DEBUG
  Serial.println(F("LoRa not inited!"));
 #endif    
    return;
  }
    
  #ifdef _DEBUG
    Serial.println(F("Send sensors data via LoRa..."));
  #endif

    bool sendDone = false;

    UniRawScratchpad scratchpadS;
    scratchpadS.head.packet_type = uniWaterTankClient;
    scratchpadS.head.controller_id = controllerID;
    
    WaterTankDataPacket* dataPacket = (WaterTankDataPacket*) &(scratchpadS.data);

    // заполняем пакет данными
    fillDataPacket(dataPacket);

    // подсчитываем контрольную сумму
    scratchpadS.crc8 = OneWire::crc8((const byte*)&scratchpadS,sizeof(scratchpadS)-1);  

    for(int i=0;i<5;i++) // пытаемся послать 5 раз
    {
        LoRa.beginPacket();
        LoRa.write((byte*)&scratchpadS,sizeof(scratchpadS)); // пишем в эфир
        if(LoRa.endPacket()) // пишем в него
        {
          sendDone = true;
          break;
        }
        else
        {
          delay(random(10));
        }
    } // for

    if(!sendDone)
    {
      #ifdef _DEBUG
        Serial.println(F("NO RECEIVING SIDE FOUND!"));
      #endif      
    }
    else
    {
      #ifdef _DEBUG
        Serial.println(F("Sensors data sent."));
      #endif
    }
    
  // рандомная задержка
  delay(random(50));

  LoRa.sleep(); // засыпаем

}
//----------------------------------------------------------------------------------------------------------------
void processPacket(NRFWaterTankExecutionPacket& packet)
{
    #ifdef _DEBUG
      Serial.print(F("RECEIVED WATER TANK COMMAND! VALVE SHOULD BE: ["));
      if(packet.valveCommand)
      {
        Serial.print(F("ON"));
      }
      else
      {
        Serial.print(F("OFF"));
      }
      Serial.println(F("]."));
      
    #endif
}
//----------------------------------------------------------------------------------------------------------------
void processIncomingLoRaPackets()
{
    // обрабатываем входящие данные по LoRa  
  if(!loRaInited)
    return;
    
  static NRFWaterTankExecutionPacket nrfPacket; // наш пакет, в который мы принимаем данные с контроллера
  int packetSize = LoRa.parsePacket();
  if(packetSize >= sizeof(NRFWaterTankExecutionPacket))
  {
    memset(&nrfPacket,0,sizeof(NRFWaterTankExecutionPacket));
    LoRa.readBytes((byte*)&nrfPacket,sizeof(NRFWaterTankExecutionPacket));

    if(nrfPacket.controller_id == controllerID)
    {
       // это пакет с нашего контроллера пришёл, обновляем данные
       byte checksum = OneWire::crc8((const byte*) &nrfPacket,sizeof(NRFWaterTankExecutionPacket)-1);
       
       if(checksum == nrfPacket.crc8) // чексумма сошлась
       {
         switch(nrfPacket.packetType)
         {
            case RS485WaterTankCommands: // пакет с командами для модуля контроля бака воды
            {
              processPacket(nrfPacket);
            }
            break; // RS485ControllerStatePacket
         } // switch
       } //  // good checksum
    }
  }    
}
//----------------------------------------------------------------------------------------------------------------
void SetControllerID(uint8_t id)
{
    controllerID = id;
    EEPROM.write(CONTROLLER_ID_ADDRESS,controllerID);
}
//----------------------------------------------------------------------------------------------------------------
void readROM()
{
  controllerID = EEPROM.read(CONTROLLER_ID_ADDRESS);
  if(controllerID == 0xFF)
    controllerID = DEFAULT_CONTROLLER_ID;
}
//----------------------------------------------------------------------------------------------------------------
void setupLevelSensors()
{
  countOfLevelSensors = sizeof(levelSensors)/sizeof(levelSensors[0]);
  for(size_t i=0;i<countOfLevelSensors;i++)
  {
    pinMode(levelSensors[i],INPUT);
    levelSensorsState[i] = 0xFF; // нет данных с датчика
  }
}
//----------------------------------------------------------------------------------------------------------------
void setup()
{
  #ifdef USE_RANDOM_SEED_PIN
    randomSeed(analogRead(RANDOM_SEED_PIN));
  #endif
  
  radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  
  #ifdef USE_WATCHDOG
  delay(5000);                        // Задержка, чтобы было время перепрошить устройство в случае bootloop
  wdt_enable (WDTO_8S);               // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  #endif // USE_WATCHDOG
  
 

  readROM();
  
  #ifdef _DEBUG
    Serial.begin(57600);
    Serial.println(F("DEBUG MODE!"));
  #endif

  setupLevelSensors();
 
  initLoRa();

  
}
//----------------------------------------------------------------------------------------------------------------
void ProcessIDCommand(Command& cmd)
{

  size_t argsCount = cmd.GetArgsCount();
  if(cmd.GetType() == ctGET)
  {
    // GET

      Serial.print("OK=");
      Serial.println(controllerID);
      return;
    
  }
  else
  {
    // SET
      if(argsCount > 0)
      {
        uint8_t id = atoi(cmd.GetArg(0));
        SetControllerID(id);
        Serial.print("OK=");
        Serial.println("ADDED");
        return;
    }
  }

  Serial.print("ER=");
  Serial.println("UNKNOWN_COMMAND");
}
//----------------------------------------------------------------------------------------------------------------
void ProcessIncomingCommand(String& cmd)
{

  CommandParser cParser;
  Command parsed;
  if(cParser.ParseCommand(cmd,parsed))
  {

      String module = parsed.GetTargetModuleID();
      if(module == F("ID"))
      {
        ProcessIDCommand(parsed);
      }
  }
}
//----------------------------------------------------------------------------------------------------------------
void loop()
{ 
   
  if(millis() - updateTimer >= SENSORS_UPDATE_INTERVAL)
  {
    UpdateSensors();
    updateTimer = millis();
  }

  #ifdef USE_WATCHDOG
  if(millis() - updateTimerWdt > WDT_UPDATE_INTERVAL)
  {
    updateTimerWdt = millis();
    wdt_reset();
  }
  #endif // USE_WATCHDOG

    if(commandBuffer.HasCommand())
    {
      String cmd = commandBuffer.GetCommand();
      commandBuffer.ClearCommand();
      ProcessIncomingCommand(cmd);
    }

   processIncomingLoRaPackets();
    
  if(millis() - lastRadioSentAt >= radioSendInterval)
  {
    sendDataViaLoRa();
    lastRadioSentAt = millis();
    radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  }

}
//----------------------------------------------------------------------------------------------------------------

