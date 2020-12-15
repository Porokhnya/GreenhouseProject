//----------------------------------------------------------------------------------------------------------------
/*
ПАМЯТКА ПО КОМАНДАМ КОНТРОЛЛЕРА:

  0. КАЖДАЯ КОМАНДА ЗАКАНЧИВАЕТСЯ ПЕРЕВОДОМ СТРОКИ!

  1. КОМАНДЫ ЧЕРЕЗ UART РАБОТАЮТ, ТОЛЬКО ЕСЛИ USE_RS485_GATE ЗАКОММЕНТИРОВАНО;

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
//#define _DEBUG // раскомментировать для отладочного режима (плюётся в Serial)
//----------------------------------------------------------------------------------------------------------------
#define DEFAULT_CONTROLLER_ID 0 // ID контроллера по умолчанию
#define RADIO_SEND_INTERVAL 5000 // интервал между отсылкой данных по радиоканалу, миллисекунд
#define USE_WATCHDOG // использовать или нет внутренний ватчдог
#define WDT_UPDATE_INTERVAL 5000      // интервал сброса сторожевого таймера
#define USE_RANDOM_SEED_PIN // закомментировать, если не надо использовать пин для инициализации генератора псевдослучайных чисел
#define RANDOM_SEED_PIN A0 // какой пин (АНАЛОГОВЫЙ !!!) использовать для инициализации генератора псевдослучайных чисел (пин должен быть висящим в воздухе)



//----------------------------------------------------------------------------------------------------------------
// настройки хранения в EEPROM
//----------------------------------------------------------------------------------------------------------------
#define CONTROLLER_ID_ADDRESS 55      // по какому адресу в EEPROM храним ID контроллера (1 байт)
#define LEVEL_SENSOR_ADDRESS  60 // по какому адресу будет хранится уровень срабатывания датчика (1 байт)
#define MAX_WORKTIME_ADDRESS  70  // по какому адресу хранятся настройки максимального времени наполнения (4 байта)

//----------------------------------------------------------------------------------------------------------------
// настройки датчиков уровня
//----------------------------------------------------------------------------------------------------------------
// пины для датчиков уровней, через запятую, МИНИМУМ - 5, МАКСИМУМ - 10. Датчики идут по порядку: первый - самый нижний, т.е. снизу вверх.
// система считается работоспособной, пока как минимум 3 датчика остаются работоспособными.
// неисправностью верхнего датчика считается превышение по времени максимального заполнения (настройка MAX_WORK_TIME ниже).
// неисправностью нижнего датчика считается ситуация, когда два датчика выше него - показывают воду, а он - нет.
// при неисправности верхнего датчика его функции передаются нижележащему датчику.
// при неисправности нижнего датчика - его функции передаются вышележащему датчику.
// таким образом, минимально допустимое кол-во рабочих датчиков - 3.
// заполнение бака рассчитывается только по рабочим датчикам и сигналам с них.
#define LEVEL_SENSORS_PINS  4, 5, 6, A1, A2, A3 
#define LEVEL_SENSOR_TRIGGER_LEVEL  LOW  // уровень срабатывания датчика по умолчанию

//----------------------------------------------------------------------------------------------------------------
// настройки управления клапаном
//----------------------------------------------------------------------------------------------------------------
#define VALVE_PIN         A4 // пин управления клапаном
#define VALVE_ON_LEVEL    HIGH // уровень включения клапана
#define MAX_WORK_TIME     600  // время для максимального наполнения, секунд (по умолчанию, будет меняться через конфигуратор)

//----------------------------------------------------------------------------------------------------------------
// настройки LoRa
//----------------------------------------------------------------------------------------------------------------
#define USE_LORA // закомментировать, если не надо работать через LoRa.

/*
 LoRa для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались в слотах, или с RS-485, или ещё где.
 */
#define LORA_SS_PIN 8 // пин SS для LoRa
#define LORA_RESET_PIN 7 // пин Reset для LoRa
#define LORA_FREQUENCY 868E6 // частота работы (433E6, 868E6, 915E6)
#define LORA_TX_POWER 17 // мощность передатчика (1 - 17)

//----------------------------------------------------------------------------------------------------------------
// настройки RS-485
//----------------------------------------------------------------------------------------------------------------
//#define USE_RS485_GATE // закомментировать, если не нужна работа через RS-485
//----------------------------------------------------------------------------------------------------------------
#define RS485_DE_PIN 3 // номер пина, на котором будем управлять направлением приём/передача по RS-485


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
#ifdef USE_WATCHDOG
uint32_t updateTimerWdt = 0;
#endif // USE_WATCHDOG

bool waterTankValveIsOn = true; // флаг включения клапана заполнения бочки
uint8_t levelSensors[] = {LEVEL_SENSORS_PINS}; // наши пины для датчиков уровня
uint8_t levelSensorsState[] = {LEVEL_SENSORS_PINS}; // состояние пинов датчиков уровня
uint8_t levelSensorTriggerLevel = LEVEL_SENSOR_TRIGGER_LEVEL; // уровень срабатывания датчика по умолчанию
uint8_t criticalLevelSensorIndex = 0; // индекс датчика критического уровня
uint8_t fullLevelSensorIndex = sizeof(levelSensors)/sizeof(levelSensors[0]) - 1; // индекс датчика полноты бака
uint32_t maxWorkTime = 1000ul*MAX_WORK_TIME; // максимальное время наполнения
uint8_t errorType = waterTankNoErrors;
uint8_t errorFlag = 0;
MachineState machineState = msNormal;
uint32_t fullFillTimer = 0; 
//----------------------------------------------------------------------------------------------------------------
#if defined(USE_LORA) && !defined(USE_RS485_GATE)
CommandBuffer commandBuffer(&Serial);
#endif
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA
//----------------------------------------------------------------------------------------------------------------
#include "LoRa.h"
bool loRaInited = false;
//----------------------------------------------------------------------------------------------------------------
void initLoRa()
{  
  #ifdef _DEBUG
    Serial.println(F(" Init LoRa..."));
  #endif  
  
  // инициализируем LoRa
  LoRa.setPins(LORA_SS_PIN,LORA_RESET_PIN,-1);
  loRaInited = LoRa.begin(LORA_FREQUENCY);

  if(loRaInited)
  {
    LoRa.setTxPower(LORA_TX_POWER);
    LoRa.sleep(); // засыпаем

  #ifdef _DEBUG
    Serial.println(F("LoRa inited."));
  #endif  
    
  } // loRaInited
  #ifdef _DEBUG
  else
  {
    Serial.println(F("LoRa init FAIL!"));
  }
  #endif  
  
  
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_LORA
//----------------------------------------------------------------------------------------------------------------
void turnValve(bool on) // включение или выключение клапана
{
    if(waterTankValveIsOn != on)
    {
      #ifdef _DEBUG
        Serial.print(F("Turn valve to: "));
        Serial.println(on ? F("ON") : F("OFF"));
      #endif  
      
      waterTankValveIsOn = on;
      digitalWrite(VALVE_PIN, on ? VALVE_ON_LEVEL : !(VALVE_ON_LEVEL));
    }
}
//----------------------------------------------------------------------------------------------------------------
uint8_t getCountOfWorkingSensors() // возвращает кол-во рабочих датчиков
{
  return ((fullLevelSensorIndex - criticalLevelSensorIndex) + 1);
}
//----------------------------------------------------------------------------------------------------------------
bool canWork() // проверяем, можем ли мы работать в штатном режиме? это возможно, только если кол-во рабочих датчиков - минимум 3
{
  return (getCountOfWorkingSensors() > 2);
}
//----------------------------------------------------------------------------------------------------------------
void checkCriticalSensorIsGood()
{
  // проверяем, в порядке ли датчик критического уровня.
  // рабочих датчиков должно быть минимум 3.
  // при этом неработоспособность датчика критического уровня
  // принимается как состояние, когда на датчике нет воды,
  // а на двух вышележащих - есть вода.

  uint8_t cnt = getCountOfWorkingSensors();
  if(cnt > 2)
  {
      bool criticalSensorHasWater = levelSensorsState[criticalLevelSensorIndex] > 0;
      bool topHasWater = (levelSensorsState[criticalLevelSensorIndex+1] > 0) && (levelSensorsState[criticalLevelSensorIndex+2] > 0);

      if(!criticalSensorHasWater && topHasWater)
      {
      #ifdef _DEBUG
        Serial.println(F("Critical sensor ERROR detected!"));
      #endif  
        
        // ситуация, когда верхние два датчика показывают воду, а датчик критического уровня - воду не показывает.
        // передаём управление вышестоящему датчику, взводим флаг ошибки и запоминаем тип ошибки
        criticalLevelSensorIndex++;
        errorFlag = 1;
        errorType = waterTankBottomSensorFailure;
      }
  } // if
}
//----------------------------------------------------------------------------------------------------------------
void updateSensors()
{
  //обновление данных с датчиков. обновлять можно только тогда, когда работоспособность системы - не нарушена,
  // т.е. кол-во рабочих датчиков - минимум 3. Эта функция вызывается только в случае, если кол-во рабочих датчиков - больше 2.

  // теперь проходим по всем пинам датчиков уровня - и читаем их состояние
  for(size_t i=criticalLevelSensorIndex;i<=fullLevelSensorIndex;i++)
  {
    uint8_t state = digitalRead(levelSensors[i]);
    levelSensorsState[i] = (state == levelSensorTriggerLevel) ? 1 : 0;
  }

}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA
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
#endif // USE_LORA
//----------------------------------------------------------------------------------------------------------------
void processSettingsPacket(uint8_t _level, uint32_t _maxWorkTime)
{
  #ifdef _DEBUG
    Serial.println(F("SETTINGS RECEIVED!"));
  #endif 

  uint8_t oldLevel = levelSensorTriggerLevel;
  levelSensorTriggerLevel = _level;

  if(levelSensorTriggerLevel != oldLevel)
  {
    EEPROM.write(LEVEL_SENSOR_ADDRESS,levelSensorTriggerLevel);
  }

  uint32_t newWorkTime = 1000ul*_maxWorkTime;
  
  if(newWorkTime != maxWorkTime)
  {
    maxWorkTime = newWorkTime;
    EEPROM.put(MAX_WORKTIME_ADDRESS,maxWorkTime);
  }

  //TODO: ТУТ УСТАНОВКА НОВЫХ НАСТРОЕК!  
}
//----------------------------------------------------------------------------------------------------------------
void processSettingsPacket(RS485WaterTankSettingsPacket* packet)
{
  processSettingsPacket(packet->level,packet->maxWorkTime); 
}
//----------------------------------------------------------------------------------------------------------------
void processSettingsPacket(NRFWaterTankSettingsPacket* packet)
{
  processSettingsPacket(packet->level,packet->maxWorkTime);  
}
//----------------------------------------------------------------------------------------------------------------
void processCommandPacket(uint8_t valveCommand)
{
  #ifdef _DEBUG
      Serial.print(F("RECEIVED WATER TANK COMMAND! VALVE SHOULD BE: ["));
      if(valveCommand)
      {
        Serial.print(F("ON"));
      }
      else
      {
        Serial.print(F("OFF"));
      }
      Serial.println(F("]."));
      
    #endif

    if(canWork())
    {
      // можем работать, исправных датчиков - минимум 3
      
      bool requestedWalveState = valveCommand > 0;
      
      if(waterTankValveIsOn != requestedWalveState)
      {
        // попросили изменить состояние клапана.
        // здесь мы должны учитывать, что при включении/выключении клапана - мы должны переместиться
        // на нужную ветку конечного автомата
        if(requestedWalveState)
        {
            // попросили включиться
            turnValve(true);
            fullFillTimer = millis();
            machineState = msWaitForTankFullfilled;
        }
        else
        {
          // попросили выключиться
          turnValve(false);
          machineState = msNormal;
        }
      }
    } // canWork  
}
//----------------------------------------------------------------------------------------------------------------
void processCommandPacket(RS485WaterTankCommandPacket* packet)
{
  processCommandPacket(packet->valveCommand);
}
//----------------------------------------------------------------------------------------------------------------
void processCommandPacket(NRFWaterTankExecutionPacket* packet)
{
    processCommandPacket(packet->valveCommand);    
}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA
//----------------------------------------------------------------------------------------------------------------
void processIncomingLoRaPackets()
{
    // обрабатываем входящие данные по LoRa  
  if(!loRaInited)
  {
    return;
  }
      
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
              processCommandPacket(&nrfPacket);
            }
            break; // RS485WaterTankCommands

            case RS485WaterTankSettings:
            {              
              processSettingsPacket((NRFWaterTankSettingsPacket*) &nrfPacket);
            }
            break; // RS485WaterTankSettings
         } // switch
       } //  // good checksum
    }
  }    
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_LORA
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
  {
    controllerID = DEFAULT_CONTROLLER_ID;
  }

  levelSensorTriggerLevel = EEPROM.read(LEVEL_SENSOR_ADDRESS);
  if(levelSensorTriggerLevel == 0xFF)
  {
    levelSensorTriggerLevel = LEVEL_SENSOR_TRIGGER_LEVEL;
  }

  EEPROM.get(MAX_WORKTIME_ADDRESS,maxWorkTime);
  if(maxWorkTime == 0xFFFFFFFF)
  {
    maxWorkTime = 1000ul*MAX_WORK_TIME; // максимальное время наполнения
  }
}
//----------------------------------------------------------------------------------------------------------------
void setupLevelSensors()
{
  #ifdef _DEBUG
    Serial.println(F("Setup level sensors..."));
  #endif  
  
  size_t cnt = sizeof(levelSensors)/sizeof(levelSensors[0]);
  for(size_t i=0;i<cnt;i++)
  {
    pinMode(levelSensors[i],INPUT);
    levelSensorsState[i] = 0; // нет воды на датчике
  }
}
//----------------------------------------------------------------------------------------------------------------
uint8_t getFillStatus() // возвращает процент заполнения (0-100%)
{
  float cnt = getCountOfWorkingSensors(); // количество рабочих датчиков
  // считаем, что все датчики исправны и расположены равномерно по баку.
  // рассчитываем, сколько датчиков показывают воду, прерываемся на первом, кто воду не показывает

  uint16_t hasWater = 0;

  for(uint8_t i=criticalLevelSensorIndex;i<=fullLevelSensorIndex;i++)
  {
    if(levelSensorsState[i])
    {
      hasWater++;
    }
    else
    {
      break;
    }
  } // for

  // пропорция
  // cnt = 100%
  // hasWater = x%
  // x = (hasWater*100)/cnt;
  float f1 = hasWater;
  f1*=100;

  float result = f1/cnt;

  return roundf(result);
}
//----------------------------------------------------------------------------------------------------------------
void fillDataPacket(WaterTankDataPacket* packet)
{
  packet->valveState = waterTankValveIsOn;
  packet->errorFlag = errorFlag;
  packet->errorType = errorType;
  packet->fillStatus = getFillStatus();

  #ifdef _DEBUG
    Serial.print(F("Fill status is: "));
    Serial.println(packet->fillStatus);
  #endif  
  
  // пакет с данными заполнен и готов к отправке по радиоканалу
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
   CRC - контрольная сумма пакета

 
 */
//----------------------------------------------------------------------------------------------------------------
RS485Packet rs485Packet; // пакет, в который мы принимаем данные
volatile byte* rsPacketPtr = (byte*) &rs485Packet;
volatile byte  rs485WritePtr = 0; // указатель записи в пакет
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
    byte crc = OneWire::crc8((const byte*) rsPacketPtr,sizeof(RS485Packet) - 1);
    if(crc != rs485Packet.crc8)
    {
      // не сошлось, игнорируем
      return;
    }


    // всё в пакете правильно, анализируем и выполняем
    // проверяем, наш ли пакет
    if(rs485Packet.direction != RS485FromMaster) // не от мастера пакет
      return;

    switch(rs485Packet.type)
    {
      case RS485WaterTankRequestData: // запросили данные по баку
      {
        WaterTankDataPacket* packet = (WaterTankDataPacket*) &rs485Packet.data;
        fillDataPacket(packet);        
      }
      break; // RS485WaterTankRequestData

      case RS485WaterTankCommands: // пакет с командами для модуля контроля бака воды
      {
        RS485WaterTankCommandPacket* packet = (RS485WaterTankCommandPacket*) &rs485Packet.data;
        processCommandPacket(packet);
        
      }
      return; // RS485WaterTankCommands

      case RS485WaterTankSettings: // пакет с настройками для модуля контроля уровня бака
      {
        RS485WaterTankSettingsPacket* packet = (RS485WaterTankSettingsPacket*) &rs485Packet.data;
        processSettingsPacket(packet);
      }
      return; // RS485WaterTankSettings

      default: return;
    } // switch


     // выставляем нужное направление пакета
     rs485Packet.direction = RS485FromSlave;
     rs485Packet.type = RS485WaterTankDataAnswer;

     // подсчитываем CRC
     rs485Packet.crc8 = OneWire::crc8((const byte*) &rs485Packet,sizeof(RS485Packet)-1 );

     // теперь переключаемся на передачу
     RS485Send();

     // пишем в порт данные
     Serial.write((const uint8_t *)&rs485Packet,sizeof(RS485Packet));

     // ждём окончания передачи
     RS485waitTransmitComplete();
     
    // переключаемся на приём
    RS485Receive();

    
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
void RS485Receive()
{
  digitalWrite(RS485_DE_PIN,LOW); // переводим контроллер RS-485 на приём
}
//----------------------------------------------------------------------------------------------------------------
void RS485Send()
{
  digitalWrite(RS485_DE_PIN,HIGH); // переводим контроллер RS-485 на передачу
}
//----------------------------------------------------------------------------------------------------------------
void InitRS485()
{
  #ifdef _DEBUG
    Serial.println(F("Init RS-485..."));
  #endif

  memset(&rs485Packet,0,sizeof(RS485Packet));
  // тут настраиваем RS-485 на приём
  pinMode(RS485_DE_PIN,OUTPUT);
  RS485Receive();

  #ifdef _DEBUG
    Serial.println(F("RS-485 Inited."));
  #endif
}
//----------------------------------------------------------------------------------------------------------------
void RS485waitTransmitComplete()
{
  // ждём завершения передачи по UART
  while(!(UCSR0A & _BV(TXC0) ));
}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_RS485_GATE
//----------------------------------------------------------------------------------------------------------------
void setup()
{
  bool serialStarted = false;
  
  #ifdef _DEBUG
    Serial.begin(57600);
    Serial.println(F("DEBUG MODE!"));
    serialStarted = true;
  #endif
  
  #ifdef USE_RANDOM_SEED_PIN
    randomSeed(analogRead(RANDOM_SEED_PIN));
  #endif

  pinMode(VALVE_PIN,OUTPUT);
  
  radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  
  #ifdef USE_WATCHDOG
    delay(5000);                        // Задержка, чтобы было время перепрошить устройство в случае bootloop
    wdt_enable (WDTO_8S);               // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  #endif // USE_WATCHDOG
  

  #if defined(USE_LORA) && !defined(USE_RS485_GATE)
    if(!serialStarted)
    {
      Serial.begin(57600);
      serialStarted = true;
    }
  #endif 

 #ifdef USE_RS485_GATE // если сказано работать через RS-485 - работаем 
 
    #ifndef _DEBUG
      if(!serialStarted)
      {
        Serial.begin(57600);        
      }
    #endif
    
    InitRS485(); // настраиваем RS-485 на приём
 #endif  

  readROM(); // читаем настройки
  setupLevelSensors(); // настраиваем датчики
  #ifdef USE_LORA
  initLoRa(); // поднимаем радиоканал
  #endif // USE_LORA
  updateSensors(); // сразу получаем данные при старте
  turnValve(false); // выключаем клапан при старте
  
}
//----------------------------------------------------------------------------------------------------------------
#if defined(USE_LORA) && !defined(USE_RS485_GATE)
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
#endif // #if defined(USE_LORA) && !defined(USE_RS485_GATE)
//----------------------------------------------------------------------------------------------------------------
void loop()
{ 

  if(canWork())
  {
      updateSensors(); // обновляем данные с датчиков
      checkCriticalSensorIsGood(); // проверяем, в порядке ли датчик критического уровня?      
  } // canWork
  else
  {
    // работать не можем, минимальное кол-во рабочих датчиков - меньше трёх, выключаем подающий клапан
    turnValve(false);    
  } // else


  #ifdef USE_WATCHDOG
  if(millis() - updateTimerWdt > WDT_UPDATE_INTERVAL)
  {
    updateTimerWdt = millis();
    wdt_reset();
  }
  #endif // USE_WATCHDOG

    #if defined(USE_LORA) && !defined(USE_RS485_GATE)
    if(commandBuffer.HasCommand())
    {
      String cmd = commandBuffer.GetCommand();
      commandBuffer.ClearCommand();
      ProcessIncomingCommand(cmd);
    }
    #endif

  #ifdef USE_RS485_GATE
      ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
  #endif
    

   #ifdef USE_LORA
   processIncomingLoRaPackets();
   #endif // USE_LORA
    
  if(millis() - lastRadioSentAt >= radioSendInterval)
  {
    #ifdef USE_LORA
    sendDataViaLoRa();
    #endif // USE_LORA
    
    lastRadioSentAt = millis();
    radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  }

  // тут можно проверять состояния конечного автомата
  if(canWork())
  {
    // можем работать, смотрим, в какой ветке конечного автомата мы находимся
    switch(machineState)
    {
      case msNormal:
      {
        // нормальное состояние, проверяем датчик критического уровня
        if(!levelSensorsState[criticalLevelSensorIndex])
        {
          #ifdef _DEBUG
            Serial.println(F("Critical level detected, turn valve ON!"));
          #endif  
          
          // датчик критического уровня показывает, что нет воды - надо открывать клапан
          turnValve(true);

          // включили таймер, перешли на другую ветку
          fullFillTimer = millis(); 
          machineState = msWaitForTankFullfilled;
        }
        
      }
      break; // msNormal

      case msWaitForTankFullfilled:
      {
        // ждём наполнения бака в течение максимального времени
        
        if(levelSensorsState[fullLevelSensorIndex])
        {
          #ifdef _DEBUG
            Serial.println(F("Tank is full, turn valve OFF!"));
          #endif  
          
          // датчик верхнего уровня показывает воду, выключаем клапан, переключаемся в нормальный режим
          turnValve(false);
          machineState = msNormal;
        }
        else
        {
          // датчик всё ещё не показывает воду, смотрим - не истекло ли время ожидания наполнения
          if(millis() - fullFillTimer >= maxWorkTime)
          {
            #ifdef _DEBUG
              Serial.println(F("Full sensor ERROR detected!"));
            #endif  
            
            // истекло время максимальной работы, это ошибка верхнего датчика !!!
            fullLevelSensorIndex--; // передаём его функции нижележащему датчику

            // взводим флаг ошибки
            errorFlag = 1;
            errorType = waterTankFullSensorError;
          }
        }
      }
      break; // msWaitForTankFullfilled
      
    } // switch
    
  } // canWork
  else
  {
    // работать не можем, минимальное кол-во рабочих датчиков - меньше трёх, выключаем подающий клапан
    turnValve(false);

    // тут переведение конечного автомата в исходную ветку
    machineState = msNormal;
    
  } // else
}
//----------------------------------------------------------------------------------------------------------------
void yield()
{
   #ifdef USE_RS485_GATE
      ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
  #endif   
}
//----------------------------------------------------------------------------------------------------------------


