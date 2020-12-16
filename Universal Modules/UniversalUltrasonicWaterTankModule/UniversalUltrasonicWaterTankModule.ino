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
#define DISTANCE_EMPTY_ADDRESS  60 // по какому адресу будет хранится расстояние до воды пустого бака, см (2 байта)
#define DISTANCE_FULL_ADDRESS  65 // по какому адресу будет хранится расстояние до воды полного бака, см (2 байта)
#define MAX_WORKTIME_ADDRESS  70  // по какому адресу хранятся настройки максимального времени наполнения (4 байта)

//----------------------------------------------------------------------------------------------------------------
// настройки расстояния по умолчанию
//----------------------------------------------------------------------------------------------------------------
#define DISTANCE_EMPTY_DEFAULT  100 // расстояние до воды пустого бака, см (по умолчанию, меняется через конфигуратор)
#define DISTANCE_FULL_DEFAULT  10 // расстояние до воды полного бака, см (по умолчанию, меняется через конфигуратор)

//----------------------------------------------------------------------------------------------------------------
// настройки ультразвукового датчика
//----------------------------------------------------------------------------------------------------------------
#define SENSOR_TRIG_PIN  4 // номер пина TRIG, на котором сидит УЗ-датчик (пин - цифровой!)
#define SENSOR_ECHO_PIN  5 // номер пина ECHO, на котором сидит УЗ-датчик (пин - цифровой!)
#define SENSOR_UPDATE_INTERVAL 5000 // интервал обновления данных с датчика, миллисекунд

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
#define USE_RS485_GATE // закомментировать, если не нужна работа через RS-485
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
uint32_t maxWorkTime = 1000ul*MAX_WORK_TIME; // максимальное время наполнения
uint16_t distanceEmpty = DISTANCE_EMPTY_DEFAULT; // расстояние до пустого бака, см
uint16_t distanceFull = DISTANCE_FULL_DEFAULT; // расстояние до полного бака, см
uint8_t errorType = waterTankNoErrors;
uint8_t errorFlag = 0;
MachineState machineState = msNormal;
uint32_t fullFillTimer = 0; 
uint32_t sensorUpdateTimer = 0;
uint32_t catchedDistance = 0; // уловленная дистанция до зеркала воды
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
bool canWork() // проверяем, можем ли мы работать в штатном режиме?
{
  return (errorFlag == 0); // можем работать только тогда, когда нет флага ошибки
}
//----------------------------------------------------------------------------------------------------------------
void updateSensor(bool immediate)
{
  //обновление данных с датчика.
  // если сигнал не получен - выставляем ошибку waterTankFullSensorError если клапан включён, и ошибку waterTankBottomSensorFailure - если клапан выключен (флаг waterTankValveIsOn)

  // если параметр immediate выставлен в true, то обновляем немедленно, иначе - по таймеру
  if(!immediate)
  {
    if(!(millis() - sensorUpdateTimer >= SENSOR_UPDATE_INTERVAL))
    {
      return; // не вышло время обновления
    }
  }

  digitalWrite(SENSOR_TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(SENSOR_TRIG_PIN, HIGH);
  delayMicroseconds(15);
  digitalWrite(SENSOR_TRIG_PIN, LOW);
  
  unsigned long duration = pulseIn(SENSOR_ECHO_PIN,HIGH);

  catchedDistance = 0; // считаем, что не получили расстояние
  
  if(duration < 1) // не уловили расстояние, ошибка!
  {
    errorFlag = 1;
    if(waterTankValveIsOn)
    {
       errorType = waterTankFullSensorError;
    }
    else
    {
      errorType = waterTankBottomSensorFailure;
    }
  }
  else // нет ошибок, сбрасываем флаг ошибки, высчитываем расстояние
  {
    errorFlag = 0;
    errorType = waterTankNoErrors;

    // высчитываем расстояние в сантиметрах
    catchedDistance = duration/29/2;
  }
  

  sensorUpdateTimer = millis();
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
void processSettingsPacket(uint16_t _distanceEmpty, uint16_t _distanceFull, uint32_t _maxWorkTime)
{
  #ifdef _DEBUG
    Serial.println(F("SETTINGS RECEIVED!"));
  #endif 

  uint16_t oldDistanceEmpty = distanceEmpty;
  distanceEmpty = _distanceEmpty;

  if(distanceEmpty != oldDistanceEmpty)
  {
    EEPROM.put(DISTANCE_EMPTY_ADDRESS,distanceEmpty);
  }

  uint16_t oldDistanceFull = distanceFull;
  distanceFull = _distanceEmpty;

  if(distanceFull != _distanceFull)
  {
    EEPROM.put(DISTANCE_FULL_ADDRESS,distanceFull);
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
  processSettingsPacket(packet->distanceEmpty,packet->distanceFull,packet->maxWorkTime); 
}
//----------------------------------------------------------------------------------------------------------------
void processSettingsPacket(NRFWaterTankSettingsPacket* packet)
{
  processSettingsPacket(packet->distanceEmpty,packet->distanceFull,packet->maxWorkTime); 
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

    bool canProcess = canWork(); // проверяем, можем ли мы работать (не выставлен ли флаг ошибки?)

    if(!canProcess)
    {
      // почему-то работать не можем - попробуем обновить данные с датчика, вдруг заработает?
      updateSensor(true);
      canProcess = canWork(); // проверяем ещё раз - получилось прочитать дистанцию до воды?
    }

    if(canProcess)
    {
      // можем работать, флага ошибки - не выставлено.
      
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

  EEPROM.get(MAX_WORKTIME_ADDRESS,maxWorkTime);
  if(maxWorkTime == 0xFFFFFFFF)
  {
    maxWorkTime = 1000ul*MAX_WORK_TIME; // максимальное время наполнения
  }

  EEPROM.get(DISTANCE_EMPTY_ADDRESS,distanceEmpty);
  if(distanceEmpty == 0xFFFF)
  {
    distanceEmpty = DISTANCE_EMPTY_DEFAULT; // по умолчанию
  }

  EEPROM.get(DISTANCE_FULL_ADDRESS,distanceFull);
  if(distanceFull == 0xFFFF)
  {
    distanceFull = DISTANCE_FULL_DEFAULT; // по умолчанию
  }
  
}
//----------------------------------------------------------------------------------------------------------------
void setupSensor()
{
  #ifdef _DEBUG
    Serial.println(F("Setup level sensor..."));
  #endif  
  
  pinMode(SENSOR_TRIG_PIN, OUTPUT);
  pinMode(SENSOR_ECHO_PIN, INPUT);

}
//----------------------------------------------------------------------------------------------------------------
uint8_t getFillStatus() // возвращает процент заполнения (0-100%)
{
  if(catchedDistance < 1) // нет данных по расстоянию до зеркала воды
  {
    return 0;
  }

  // в distanceEmpty - расстояние до зеркала воды пустого бака
  // в distanceFull - расстояние до зеркала воды полного бака

  if(catchedDistance <= distanceFull) // полный бак
  {
    return 100;
  }

  if(catchedDistance >= distanceEmpty) // пустой бак
  {
    return 0;
  }

  // пропорция: distanceEmpty = 0%, distanceFull = 100%, catchedDistance = х%
  return map(catchedDistance,distanceFull,distanceEmpty,0,100);
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
  setupSensor(); // настраиваем датчик
  #ifdef USE_LORA
  initLoRa(); // поднимаем радиоканал
  #endif // USE_LORA
  
  turnValve(false); // выключаем клапан при старте

  delay(1000); // подождём секунду, мало ли.
  updateSensor(true); // сразу получаем данные при старте
  
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

  if(canWork()) // можем работать, флага ошибки - не выставлено.
  {
      updateSensor(false); // обновляем данные с датчика
  } // canWork
  else
  {
    // работать не можем, выставлен флаг ошибки, выключаем клапан
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
        // нормальное состояние, проверяем на критический уровень
        if(catchedDistance > 0) // есть показания с датчика
        {
          if(catchedDistance >= distanceEmpty) // очень далеко до зеркала воды
          {
            #ifdef _DEBUG
              Serial.println(F("Critical level detected, turn valve ON!"));
            #endif  
            
            // расстояние до воды очень большое - надо открывать клапан
            turnValve(true);
  
            // включили таймер, перешли на другую ветку
            fullFillTimer = millis(); 
            machineState = msWaitForTankFullfilled;
            
          } // if(catchedDistance >= distanceEmpty)
          
        } // if(catchedDistance > 0)
        
      }
      break; // msNormal

      case msWaitForTankFullfilled:
      {
        // ждём наполнения бака в течение максимального времени
        if(catchedDistance > 0) // есть показания с датчика
        {
            if(catchedDistance <= distanceFull) // уровень выше, чем максимальный - надо выключать клапан
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
              // датчик всё ещё не показывает минимальное расстояние до воды, смотрим - не истекло ли время ожидания наполнения
              
              if(millis() - fullFillTimer >= maxWorkTime)
              {
                #ifdef _DEBUG
                  Serial.println(F("Full sensor ERROR detected!"));
                #endif  
                
                // истекло время максимальной работы, это ошибка датчика !!!
    
                // взводим флаг ошибки
                errorFlag = 1;
                errorType = waterTankFullSensorError;
                
                turnValve(false); // выключаем клапан
                machineState = msNormal;
                
              }
            }
        } // if(catchedDistance > 0) // есть показания с датчика
        else
        {
          // нет показаний с датчика, выключаем клапан!
          
                #ifdef _DEBUG
                  Serial.println(F("Sensor data ERROR detected!"));
                #endif  
                
                // взводим флаг ошибки
                errorFlag = 1;
                errorType = waterTankFullSensorError;
                
                turnValve(false);
                machineState = msNormal;

        } // else
      }
      break; // msWaitForTankFullfilled
      
    } // switch
    
  } // canWork
  else
  {
    // работать не можем, был взведён флаг ошибки
    
    turnValve(false); // выключаем клапан

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


