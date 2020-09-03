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

  4. ДЛЯ ПРОСМОТРА ТЕКУЩЕГО КАНАЛА ДЛЯ nRF24l01 - ВЫПОЛНИТЬ КОМАНДУ

    GET=RF|CHANNEL

  5. ДЛЯ УСТАНОВКИ КАНАЛА ДЛЯ nRF24l01 - ВЫПОЛНИТЬ КОМАНДУ

    SET=RF|CHANNEL|ТУТ_КАНАЛ

    например,

    SET=RF|CHANNEL|19


 ПЕРЕД ОТПРАВКОЙ МОДУЛЯ КЛИЕНТУ ЛУЧШЕ ПРИНУДИТЕЛЬНО ВЫПОЛНИТЬ КОМАНДЫ, УСТАНАВЛИВАЮЩИЕ ID КОНТРОЛЛЕРА И КАНАЛ nRF
 НА ПАРАМЕТРЫ ПО УМОЛЧАНИЮ, А ИМЕННО - ВЫПОЛНИТЬ ДВЕ КОМАНДЫ:

    SET=ID|0
    SET=RF|CHANNEL|19
  
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
#define _DEBUG // раскомментировать для отладочного режима (плюётся в Serial, не использовать с подключённым RS-485 !!!)
//#define _DEBUG_RANDOM_DATA // раскомментировать, если надо посылать отбалдовые данные (для отладки)
//----------------------------------------------------------------------------------------------------------------
#define DEFAULT_CONTROLLER_ID 0 // ID контроллера по умолчанию
#define RADIO_SEND_INTERVAL 5000 // интервал между отсылкой данных по радиоканалу, миллисекунд
#define SENSORS_UPDATE_INTERVAL 10000 // интервал обновления датчиков, миллисекунд
#define USE_WATCHDOG // использовать или нет внутренний ватчдог
#define WDT_UPDATE_INTERVAL 5000      // интервал сброса сторожевого таймера
#define RF_CHANNEL_ADDRESS 50         // по какому адресу в EEPROM храним номер канала для NRF (1 байт)
#define CONTROLLER_ID_ADDRESS 55      // по какому адресу в EEPROM храним ID контроллера (1 байт)

#define USE_RANDOM_SEED_PIN // закомментировать, если не надо использовать пин для инициализации генератора псевдослучайных чисел
#define RANDOM_SEED_PIN A0 // какой пин (АНАЛОГОВЫЙ !!!) использовать для инициализации генератора псевдослучайных чисел (пин должен быть висящим в воздухе)
//----------------------------------------------------------------------------------------------------------------
// настройки датчиков дождя, ветра, направления ветра
//----------------------------------------------------------------------------------------------------------------
#define RAIN_SENSOR_PIN 6 // номер пина для датчика дождя
#define RAIN_TRIGGERED_LEVEL LOW // уровень срабатывания датчика дождя
#define RAIN_RESET_INTERVAL 120000ul // через сколько миллисекунд, если не был пойман сигнал с датчика дождя, сбрасывать флаг наличия дождя

#define WIND_SPEED_SENSOR_PIN 3 // номер пина для датчика скорости ветра (ПИН С ПРЕРЫВАНИЯМИ !!!)
#define WIND_SPEED_INT_LEVEL RISING // уровень для прерывания
#define WIND_SPEED_COEFF 0.34 // коэффициент пересчёта частоты импульсов (в герцах) в скорость ветра (м/с). На этот коэффициент - умножается частота.

#define WIND_DIRECTION_PIN A1 // номер пина для направления ветра (ПИН АНАЛОГОВЫЙ)

/*
Показания АЦП, соответствующие направлению ветра. 
Расчёты приведены для опорного напряжения 5В, 1024 градации АЦП, и верхнего плеча делителя в 10К.
*/

// направление - восток, значения АЦП, соответствующие востоку, указываются через запятую
#define EAST_ADC_VALUES 91 /*0.8K*/, 183 /*2.0K*/

// направление - юг, значения АЦП, соответствующие югу, указываются через запятую
#define SOUTH_ADC_VALUES 286 /*3.7K*/, 630 /*15.7K*/

// направление - запад, значения АЦП, соответствующие западу, указываются через запятую
#define WEST_ADC_VALUES 945 /*119K*/, 888 /*64.3K*/

// направление - север, значения АЦП, соответствующие северу, указываются через запятую
#define NORTH_ADC_VALUES 786 /*32.7K*/, 460 /*7.9K*/

// гистерезис показаний АЦП для направления ветра (для проверки попадания в интервал)
#define WIND_DIRECTION_HISTERESIS 20

// кол-во показаний, по которому рассчитывается направление ветра. Полный цикл сбора показаний составит WIND_NUM_SAMPLES*SENSORS_UPDATE_INTERVAL
#define WIND_NUM_SAMPLES 10

#define RF_POWER_PIN 5     // пин включения питания радиомодулей

// настройки nRF
//----------------------------------------------------------------------------------------------------------------
//#define USE_NRF // закомментировать, если не надо работать через nRF.
//----------------------------------------------------------------------------------------------------------------
/*
 nRF для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались c номерами пинов датчиков, или с RS-485.
 */
#define NRF_CE_PIN 9 // номер пина CE для модуля nRF
#define NRF_CSN_PIN 10 // номер пина CSN для модуля nRF
#define DEFAULT_RF_CHANNEL 19 // номер канала для nRF по умолчанию
//#define NRF_AUTOACK_INVERTED // раскомментировать эту строчку здесь и в главной прошивке, если у вас они не коннектятся. 
// Иногда auto aсk в китайских модулях имеет инвертированное значение.

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
#define RS485_SPEED 57600 // скорость работы по RS-485
#define RS485_DE_PIN 4 // номер пина, на котором будем управлять направлением приём/передача по RS-485


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
int rf_id = DEFAULT_RF_CHANNEL; // номер канала nRF
int controllerID = DEFAULT_CONTROLLER_ID;
uint32_t radioSendInterval = RADIO_SEND_INTERVAL;
uint32_t lastRadioSentAt = 0;
CompassPoints windDirection = cpUnknown;
uint32_t windSpeed = 0;
bool hasRain = false;
uint32_t rainDetectedAt = 0;
uint32_t updateTimer = 0;
#ifdef USE_WATCHDOG
uint32_t updateTimerWdt = 0;
#endif // USE_WATCHDOG
//----------------------------------------------------------------------------------------------------------------
uint16_t east_ADC[] = {EAST_ADC_VALUES};
uint16_t south_ADC[] = {SOUTH_ADC_VALUES};
uint16_t west_ADC[] = {WEST_ADC_VALUES};
uint16_t north_ADC[] = {NORTH_ADC_VALUES};
CompassPoints compassSamples[WIND_NUM_SAMPLES] = {cpUnknown};
uint16_t compassSamplesWriteIterator = 0;
//----------------------------------------------------------------------------------------------------------------
#if (defined(USE_NRF) || defined(USE_LORA)) && !defined(USE_RS485_GATE)
  CommandBuffer commandBuffer(&Serial);
#endif

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

    if(rs485Packet.type != RS485WindRainData) // пакет не c запросом показаний датчиков ветра и дождя
      return;

     // теперь приводим пакет к нужному виду
     WindRainDataPacket* windRainData = (WindRainDataPacket*) &rs485Packet.data;

     windRainData->windDirection = windDirection;
     windRainData->windSpeed = windSpeed;
     windRainData->hasRain = hasRain;

     // выставляем нужное направление пакета
     rs485Packet.direction = RS485FromSlave;
     rs485Packet.type = RS485WindRainData;

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
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_RS485_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_NRF
//----------------------------------------------------------------------------------------------------------------
// трубы, в которые мы можем писать
const uint64_t writingPipes[5] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL, 0xF0F0F0F0E4LL, 0xF0F0F0F0E5LL };
//----------------------------------------------------------------------------------------------------------------
#include "RF24.h"
RF24 radio(NRF_CE_PIN,NRF_CSN_PIN);
bool nRFInited = false;
//----------------------------------------------------------------------------------------------------------------
#ifdef _DEBUG
int serial_putc( char c, FILE * ) {
  Serial.write( c );
  return c;
}

void printf_begin(void) {
  fdevopen( &serial_putc, 0 );
  Serial.begin(57600);
  Serial.println(F("Init nRF..."));
}
#endif
//----------------------------------------------------------------------------------------------------------------
void initNRF()
{
  #ifdef _DEBUG
  Serial.begin(57600);
  printf_begin();
  #endif
  
  // инициализируем nRF
  nRFInited = radio.begin();

  if(nRFInited) {
  delay(200); // чуть-чуть подождём

  radio.setDataRate(RF24_1MBPS);
  radio.setPALevel(RF24_PA_MAX);
  radio.setChannel(rf_id);
  radio.setRetries(15,15);
  radio.setPayloadSize(sizeof(RS485Packet)); // у нас 30 байт на пакет
  radio.setCRCLength(RF24_CRC_16);
  radio.setAutoAck(
    #ifdef NRF_AUTOACK_INVERTED
      false
    #else
    true
    #endif
    );

  #ifdef _DEBUG
    radio.printDetails();
  #endif

  radio.powerDown(); // входим в режим энергосбережения
  
  } // nRFInited
  
}
//----------------------------------------------------------------------------------------------------------------
void sendDataViaNRF()
{
  if(!nRFInited) {
 #ifdef _DEBUG
  Serial.println(F("nRF not inited!"));
 #endif    
    return;
  }

   /*
  if(!((scratchpadS.config & 1) == 1))
  {
    #ifdef _DEBUG
    Serial.println(F("Transiever disabled."));
    #endif
    return;
  }
  */
  
  radio.powerUp(); // просыпаемся
  
  #ifdef _DEBUG
    Serial.println(F("Send sensors data via nRF..."));
  #endif

    bool sendDone = false;

    UniRawScratchpad scratchpadS;
    scratchpadS.head.packet_type = uniWindRainClient;
    scratchpadS.head.controller_id = controllerID;
    scratchpadS.head.rf_id = rf_id;
    
    WindRainDataPacket* windRainData = (WindRainDataPacket*) &(scratchpadS.data);
    windRainData->windDirection = windDirection;
    windRainData->windSpeed = windSpeed;
    windRainData->hasRain = hasRain;
  
    // подсчитываем контрольную сумму
    scratchpadS.crc8 = OneWire::crc8((const byte*)&scratchpadS,sizeof(scratchpadS)-1);

    for(int i=0;i<5;i++) // пытаемся послать 5 раз, в разные трубы
    {
      // посылаем данные через nRF
        uint8_t writePipeNum = random(0,5);
        radio.openWritingPipe(writingPipes[writePipeNum]); // открываем канал для записи
          
        if(radio.write(&scratchpadS,sizeof(scratchpadS))) // пишем в него
        {
          sendDone = true;
          break;
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
 radio.powerDown(); // входим в режим энергосбережения

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
  } // nRFInited
  
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

  /*
  if(!((scratchpadS.config & 1) == 1))
  {
    #ifdef _DEBUG
    Serial.println(F("LoRa: transiever disabled."));
    #endif
    return;
  }
  */
    
  #ifdef _DEBUG
    Serial.println(F("Send sensors data via LoRa..."));
  #endif

  bool sendDone = false;

    UniRawScratchpad scratchpadS;
    scratchpadS.head.packet_type = uniWindRainClient;
    scratchpadS.head.controller_id = controllerID;
    
    WindRainDataPacket* windRainData = (WindRainDataPacket*) &(scratchpadS.data);
    windRainData->windDirection = windDirection;
    windRainData->windSpeed = windSpeed;
    windRainData->hasRain = hasRain;

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
uint32_t pulses = 0;
void pulse()
{
  pulses++;
}
//----------------------------------------------------------------------------------------------------------------
void SetRFChannel(uint8_t channel)
{
    rf_id = channel;
    EEPROM.write(RF_CHANNEL_ADDRESS,rf_id);
    #ifdef USE_NRF
    radio.setChannel(channel);
    #endif
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
  rf_id = EEPROM.read(RF_CHANNEL_ADDRESS);
  if(rf_id == 0xFF)
    rf_id = DEFAULT_RF_CHANNEL;

  controllerID = EEPROM.read(CONTROLLER_ID_ADDRESS);
  if(controllerID == 0xFF)
    controllerID = DEFAULT_CONTROLLER_ID;
}
//----------------------------------------------------------------------------------------------------------------
void setup()
{
  #ifdef USE_RANDOM_SEED_PIN
    randomSeed(analogRead(RANDOM_SEED_PIN));
  #endif
  
  radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  
 // setup pins
  pinMode(RF_POWER_PIN, OUTPUT);
  digitalWrite(RF_POWER_PIN, LOW);    // Включить питание радиомодуля
  delay(1000);                        // Задержка, моргнуть при старте
  digitalWrite(RF_POWER_PIN, HIGH);   // Отключить питание радиомодуля

  #ifdef USE_WATCHDOG
  delay(5000);                        // Задержка, чтобы было время перепрошить устройство в случае bootloop
  wdt_enable (WDTO_8S);               // Для тестов не рекомендуется устанавливать значение менее 8 сек.
  #endif // USE_WATCHDOG
  
  digitalWrite(RF_POWER_PIN, LOW);    // Включить питание радиомодуля
 
  bool serialStarted = false;

  readROM();
  
  #ifdef _DEBUG
    Serial.begin(57600);
    Serial.println(F("DEBUG MODE!"));
    serialStarted = true;
  #endif

  #if (defined(USE_NRF) || defined(USE_LORA)) && !defined(USE_RS485_GATE)
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
        Serial.begin(RS485_SPEED);        
      }
    #endif
    
    InitRS485(); // настраиваем RS-485 на приём
 #endif
    
    #ifdef USE_NRF
      initNRF();
    #endif

    #ifdef USE_LORA
      initLoRa();
    #endif

  pinMode(RAIN_SENSOR_PIN,INPUT);
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_SENSOR_PIN),pulse,WIND_SPEED_INT_LEVEL);
  
}
//----------------------------------------------------------------------------------------------------------------
void UpdateRainFlag()
{
  bool thisHasRain = digitalRead(RAIN_SENSOR_PIN) == RAIN_TRIGGERED_LEVEL;

  if(thisHasRain)
  {
    rainDetectedAt = millis();
  }

  if(millis() - rainDetectedAt > RAIN_RESET_INTERVAL)
  {
    thisHasRain = false; 
  }

  hasRain = thisHasRain;
  
}
//----------------------------------------------------------------------------------------------------------------
bool DirectionFound(uint16_t adcVal, uint16_t* buff, size_t buffSz)
{
  if(!buff || !buffSz)
    return false;

  for(size_t i=0;i<buffSz;i++)
  {
      int32_t ethalonVal = buff[i];
      int32_t lowBorder = ethalonVal - WIND_DIRECTION_HISTERESIS;
      int32_t highBorder = ethalonVal + WIND_DIRECTION_HISTERESIS;

      if(adcVal >= lowBorder && adcVal <= highBorder)
        return true;
      
  } // for

  return false;
}
//----------------------------------------------------------------------------------------------------------------
#include <stdlib.h>
//----------------------------------------------------------------------------------------------------------------
struct CompassCompare
{
  uint16_t count;
  CompassPoints point;

  CompassCompare()
  {
    point = cpUnknown;
    count = 0;
  }
  
  CompassCompare(CompassPoints d, uint16_t c)
  {
    point = d;
    count = c;
  }

  bool operator < (const CompassCompare& rhs)
  {
    return count < rhs.count;
  }

  bool operator > (const CompassCompare& rhs)
  {
    return count > rhs.count;
  }

  bool operator == (const CompassCompare& rhs)
  {
    return count == rhs.count;
  }
    
};
//----------------------------------------------------------------------------------------------------------------
int sort_desc(const void *cmp1, const void *cmp2)
{
  CompassCompare a = *((CompassCompare *)cmp1);
  CompassCompare b = *((CompassCompare *)cmp2);

  return a > b ? -1 : (a < b ? 1 : 0);
}
//----------------------------------------------------------------------------------------------------------------
CompassPoints GetWindDirection(CompassPoints dir)
{
  if(dir == cpUnknown)
  {
    return dir;
  }

  compassSamples[compassSamplesWriteIterator] = dir;
  compassSamplesWriteIterator++;
  
  if(compassSamplesWriteIterator >= WIND_NUM_SAMPLES)
    compassSamplesWriteIterator = 0;

  CompassPoints result = cpUnknown;
  
  // тут ищем в массиве значение направления, у которого наибольшее кол-во вхождений
  uint16_t eastCount = 0;
  uint16_t westCount = 0;
  uint16_t northCount = 0;
  uint16_t southCount = 0;

  for(uint16_t i=0;i<WIND_NUM_SAMPLES; i++)
  {
      if(compassSamples[i] == cpUnknown)
        continue;

      if(compassSamples[i] == cpEast)
      {
        eastCount++;
      }
      else
      if(compassSamples[i] == cpWest)
      {
        westCount++;
      }
      else
      if(compassSamples[i] == cpNorth)
      {
        northCount++;
      }
      else
      if(compassSamples[i] == cpSouth)
      {
        southCount++;
      }
  } // for

  // получили кол-во вхождений для каждого из направлений ветра, теперь надо рассчитать, какое из них - максимальное
  // создаём массив из 4-х элементов, и сортируем по убыванию
  CompassCompare comp[4];
  comp[0] = CompassCompare(cpEast,eastCount);
  comp[1] = CompassCompare(cpWest,westCount);
  comp[2] = CompassCompare(cpNorth,northCount);
  comp[3] = CompassCompare(cpSouth,southCount);

  qsort(comp,4,sizeof(CompassCompare),sort_desc);

  // отсортировали, и получили результат направления
  result = comp[0].point;

  return result;
}
//----------------------------------------------------------------------------------------------------------------
void UpdateWindDirection()
{
    uint16_t adcVal = analogRead(WIND_DIRECTION_PIN);
    CompassPoints thisDirection = cpUnknown;

    if(DirectionFound(adcVal,east_ADC,sizeof(east_ADC)/sizeof(east_ADC[0])))
    {
      thisDirection = cpEast;
      windDirection = GetWindDirection(thisDirection);
      return;
    }

    if(DirectionFound(adcVal,south_ADC,sizeof(south_ADC)/sizeof(south_ADC[0])))
    {
      thisDirection = cpSouth;
      windDirection = GetWindDirection(thisDirection);
      return;
    }

    if(DirectionFound(adcVal,west_ADC,sizeof(west_ADC)/sizeof(west_ADC[0])))
    {
      thisDirection = cpWest;
      windDirection = GetWindDirection(thisDirection);
      return;
    }

    if(DirectionFound(adcVal,north_ADC,sizeof(north_ADC)/sizeof(north_ADC[0])))
    {
      thisDirection = cpNorth;
      windDirection = GetWindDirection(thisDirection);
      return;
    }
    
  windDirection = GetWindDirection(thisDirection);
}
//----------------------------------------------------------------------------------------------------------------
void UpdateSensors()
{
  uint32_t elapsed = millis() - updateTimer;
  
  detachInterrupt(digitalPinToInterrupt(WIND_SPEED_SENSOR_PIN));
  uint32_t thisPulses = pulses;
  pulses = 0;
  attachInterrupt(digitalPinToInterrupt(WIND_SPEED_SENSOR_PIN),pulse,WIND_SPEED_INT_LEVEL);

    float herz = float(thisPulses*1000ul)/(1.0f*elapsed);

    #ifdef _DEBUG
      Serial.print("Pulses: ");
      Serial.println(thisPulses);
      
      Serial.print("Herz: ");
      Serial.println(herz);
    #endif
    
    // теперь считаем по формуле китайцев
    float ws = herz*(WIND_SPEED_COEFF);
    
    // теперь переводим в сотые доли
    windSpeed = ws*100;  

    // получаем направление ветра
    UpdateWindDirection();
    

  #ifdef _DEBUG
    Serial.print(F("Has rain: "));
    Serial.println(hasRain);
    Serial.print(F("Wind speed: "));
    Serial.println(windSpeed);
  #endif

  #ifdef _DEBUG_RANDOM_DATA
  
      hasRain = random(0,1);
      windSpeed = random(0,12000);
      windDirection = (CompassPoints) random(cpUnknown,cpNorth);

    #ifdef _DEBUG
      Serial.println(F("Create random data..."));      
      Serial.print(F("Has rain (random): "));
      Serial.println(hasRain);
      Serial.print(F("Wind speed (random): "));
      Serial.println(windSpeed);
      Serial.print(F("Wind direction (random): "));
      Serial.println(windDirection);      
    #endif // _DEBUG
    
  #endif // _DEBUG_RANDOM_DATA
}
//----------------------------------------------------------------------------------------------------------------
#if (defined(USE_NRF) || defined(USE_LORA)) && !defined(USE_RS485_GATE)
//----------------------------------------------------------------------------------------------------------------
void ProcessRFCommand(Command& cmd)
{
  size_t argsCount = cmd.GetArgsCount();
  if(cmd.GetType() == ctGET)
  {
    // GET
    if(argsCount > 0)
    {
      String arg = cmd.GetArg(0);
      if(arg == "CHANNEL")
      {
          Serial.print("OK=");
          Serial.println(rf_id);
          return;
      }

       
    } // argsCount > 0
    
  }
  else
  {
    // SET
    String arg = cmd.GetArg(0);
    if(arg == "CHANNEL")
    {
      if(argsCount > 1)
      {
        uint8_t channel = atoi(cmd.GetArg(1));
        SetRFChannel(channel);
        Serial.print("OK=");
        Serial.println("ADDED");
        return;
      }
    }
  }

  Serial.print("ER=");
  Serial.println("UNKNOWN_COMMAND");
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
      if(module == F("RF"))
      {
        ProcessRFCommand(parsed);
      }
      else
      if(module == F("ID"))
      {
        ProcessIDCommand(parsed);
      }
  }
}
//----------------------------------------------------------------------------------------------------------------
#endif // #if (defined(USE_NRF) || defined(USE_LORA)) && !defined(USE_RS485_GATE)
//----------------------------------------------------------------------------------------------------------------
void loop()
{ 
  UpdateRainFlag();
   
  if(millis() - updateTimer > SENSORS_UPDATE_INTERVAL)
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

  #if (defined(USE_NRF) || defined(USE_LORA)) && !defined(USE_RS485_GATE)
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

  if(millis() - lastRadioSentAt > radioSendInterval)
  {
      #ifdef USE_NRF
          sendDataViaNRF();
       #endif

       #ifdef USE_LORA
          sendDataViaLoRa();
       #endif      
    lastRadioSentAt = millis();
    radioSendInterval = RADIO_SEND_INTERVAL + random(100);
  }

}
//----------------------------------------------------------------------------------------------------------------
void yield()
{
   #ifdef USE_RS485_GATE
      ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
  #endif   
}
//----------------------------------------------------------------------------------------------------------------


