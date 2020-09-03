//----------------------------------------------------------------------------------------------------------------
/*
Прошивка для универсального модуля, предназначена для подключения
любого типа поддерживаемых датчиков и передачи с них показаний по шине 1-Wire.

Также поддерживается работа по RS-485, для включения этой возможности
надо раскомментировать USE_RS485_GATE.

Также поддерживается работа по радиоканалу, используя модуль nRF24L01+,
для этой возможности раскомментируйте USE_NRF.

Также поддерживается работа по радиоканалу, используя модуль LoRa,
для этой возможности раскомментируйте USE_LORA.

ВНИМАНИЕ!

RS-485 работает через аппаратный UART (RX0 и TX0 ардуины)!
Перед прошивкой выдёргивать контакты модуля RS-485 из пинов аппаратного UART!
*/
//----------------------------------------------------------------------------------------------------------------
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <OneWire.h>
#include "BH1750.h"
#include "Max44009.h"
#include "UniGlobals.h"
#include "Si7021Support.h"
#include "DHTSupport.h"
#include "LowLevel.h"
#include "OneWireSlave.h"
#include "SHT1x.h"
#include "LowPower.h"
//----------------------------------------------------------------------------------------------------------------
/*
 Пины, которые использует плата модуля с датчиками:
 
  D2 - линия регистрации модуля в системе (1-Wire)
  A0, A1 - чтение показаний с DS18B20, притянуты 4K7
  D4  - переключение режима приём/передача для RS485
  D0, D1 = RX/TX RS485
  D5, D6 - с подтяжками к питанию на 10К
  A4, A5 - I2C
 */
//----------------------------------------------------------------------------------------------------------------
// ВНИМАНИЕ! Пин номер 2 - не занимать, через него работает 1-Wire!
//----------------------------------------------------------------------------------------------------------------
// ПОЛЬЗОВАТЕЛЬСКИЕ НАСТРОЙКИ
//----------------------------------------------------------------------------------------------------------------
//#define USE_RANDOM_SEED_PIN // закомментировать, если не надо использовать пин для инициализации генератора псевдослучайных чисел
#define RANDOM_SEED_PIN A0 // какой пин (АНАЛОГОВЫЙ !!!) использовать для инициализации генератора псевдослучайных чисел (пин должен быть висящим в воздухе)

//#define USE_WATCHDOG // раскомментировать, если надо использовать контроль зависания при помощи внутреннего ватчдога (период 8 с)

#define SOIL_MOISTURE_0_PERCENT 1023 // вольтаж для 0% влажности почвы, китайский датчик влажности (0-1023)
#define SOIL_MOISTURE_100_PERCENT 450 // вольтаж для 100% влажности почвы, китайский датчик влажности (0-1023)
//----------------------------------------------------------------------------------------------------------------
// настройки управляющих пинов
//----------------------------------------------------------------------------------------------------------------
//#define USE_POWER_SAVING // закомментировать, если не надо использовать режим энергосбережения
#define LINES_POWER_DOWN_PIN 8 // номер пина, на котором будет управление питанием линий I2C и линий датчиков
#define LINES_POWER_DOWN_LEVEL LOW // уровень на пине для выключения линий
#define LINES_POWER_UP_LEVEL HIGH // уровень на пине для включения линий
#define LINES_POWER_UP_DELAY 20 // сколько миллисекунд ждать прочухивания питания
//----------------------------------------------------------------------------------------------------------------
#define _DEBUG // раскомментировать для отладочного режима (плюётся в Serial, не использовать с подключённым RS-485 !!!)
//----------------------------------------------------------------------------------------------------------------
// настройки энергосбережения (не применяются, когда используется USE_RS485_GATE)
//----------------------------------------------------------------------------------------------------------------
//#define USE_LOW_POWER // закомментировать, если не надо засыпать после отсылки показаний по радиоканалу.
// засыпание идёт на секунду, т.е. периодически модуль просыпается.
//----------------------------------------------------------------------------------------------------------------
// настройки nRF
//----------------------------------------------------------------------------------------------------------------
#define USE_NRF // закомментировать, если не надо работать через nRF.
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
//#define USE_LORA // закомментировать, если не надо работать через LoRa.
/*
 LoRa для своей работы занимает следующие пины: 9,10,11,12,13. 
 Следите за тем, чтобы номера пинов не пересекались в слотах, или с RS-485, или ещё где.
 */
#define LORA_SS_PIN 10 // пин SS для LoRa
#define LORA_RESET_PIN 9 // пин Reset для LoRa
#define LORA_FREQUENCY 433E6 // частота работы (433E6, 866E6, 915E6)
#define LORA_TX_POWER 17 // мощность передатчика (1 - 17)

//----------------------------------------------------------------------------------------------------------------
// настройки RS-485
//----------------------------------------------------------------------------------------------------------------
//#define USE_RS485_GATE // закомментировать, если не нужна работа через RS-485
//----------------------------------------------------------------------------------------------------------------
#define RS485_SPEED 57600 // скорость работы по RS-485
#define RS485_DE_PIN 3 //v номер пина, на котором будем управлять направлением приём/передача по RS-485

//----------------------------------------------------------------------------------------------------------------
// привязки карты пинов для слота типа mstPinsMap - 8 пинов, если в записи указано -1 - нет привязки пина.
// состояния пинов передаются на контроллер, как прописаны, при регистрации модуля на контроллере в поле 
// "индекс датчика" прописывается сдвиг в карте виртуальных пинов. Таким образом, можно получить состояние
// пинов модуля в карте виртуальных пинов контроллера. Например, если у нас индекс такого датчика 1, и виртуальные
// пины начинаются с индекса 80, то наши пины - с 88-го, 8 записей. Таким образом, максимальное кол-во таких модулей
// (со слотом "карта пинов") составляет 6 штук, поскольку у нас виртуальных пинов 128-80 = 48 штук, 48/8 = 6.
//----------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
int8_t PINS_MAP[8] = { // в примере указано, что мы следим за тремя пинами - А1, А2, А3
  A1,
  A2,
  A3,
  -1,
  -1,
  -1,
  -1,
  -1,  
};
#pragma pack(pop)
//----------------------------------------------------------------------------------------------------------------
const SensorSettings Sensors[3] = {

// настройки датчиков для модуля (РОВНО 3 ШТУКИ), МЕНЯТЬ ЗДЕСЬ!

{mstSi7021,0,0}, // датчик влажности Si7021 на шине I2C
{mstDS18B20,3,0}, // DS18B20 на пине 3
{mstDS18B20,4,0} // DS18B20 на пине 4
/* 
 
 поддерживаемые типы датчиков: 
 
  {mstSi7021,0,0} - датчик температуры и влажности Si7021 на шине I2C  
  {mstBH1750,BH1750Address1,0} - датчик освещённости BH1750 на шине I2C, его первый адрес I2C
  {mstBH1750,BH1750Address2,0} - датчик освещённости BH1750 на шине I2C, его второй адрес I2C
  {mstDS18B20,A0,0} - датчик DS18B20 на пине A0
  {mstChinaSoilMoistureMeter,A7,0} - китайский датчик влажности почвы на пине A7
  {mstDHT22, 6, 0} - датчик DHT2x на пине 6
  {mstDHT11, 5, 0} - датчик DHT11 на пине 5
  {mstPHMeter,A0, 0} // датчик pH на пине A0
  {mstSHT10,10,11} // датчик SHT10 на пине 10 (линия данных) и пине 11 (строб)
  {mstMAX44009,MAX44009_ADDRESS1,0} - датчик освещённости MAX44009 на шине I2C, его первый адрес I2C
  {mstMAX44009,MAX44009_ADDRESS2,0} - датчик освещённости MAX44009 на шине I2C, его второй адрес I2C

  // Частотные датчики влажности почвы должны на выходе выдавать ШИМ, по заполнению которого рассчитывается влажность почвы !!! Максимальный коэффициент заполнения - 254, минимальный - 1.
  {mstFrequencySoilMoistureMeter,A5, 0} - частотный датчик влажности почвы на аналоговом пине A5
  {mstFrequencySoilMoistureMeter,A4, 0} - частотный датчик влажности почвы на аналоговом пине A4
  {mstFrequencySoilMoistureMeter,A3, 0} - частотный датчик влажности почвы на аналоговом пине A3

  {mstPinsMap,0,0} - карта состояния пинов, МОЖЕТ БЫТЬ ТОЛЬКО ОДНА НА МОДУЛЬ, ПРИВЯЗКИ ПИНОВ - В НАСТРОЙКЕ PINS_MAP
  

  если в слоте записано
    {mstNone,0, 0}
  то это значит, что датчика на этом слоте нет   

 */

};
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
// Настройки 1-Wire
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
//-------------------------------------------------------------------------------------------------ы---------------
#ifdef USE_POWER_SAVING
Pin linesPowerDown(LINES_POWER_DOWN_PIN);
#endif
//----------------------------------------------------------------------------------------------------------------
#define ROM_ADDRESS (void*) 123 // по какому адресу у нас настройки?
//----------------------------------------------------------------------------------------------------------------
t_scratchpad scratchpadS, scratchpadToSend;
volatile char* scratchpad = (char *)&scratchpadS; //что бы обратиться к scratchpad как к линейному массиву

volatile bool scratchpadReceivedFromMaster = false; // флаг, что мы получили данные с мастера
volatile bool needToMeasure = false; // флаг, что мы должны запустить конвертацию
volatile unsigned long sensorsUpdateTimer = 0; // таймер получения информации с датчиков и обновления данных в скратчпаде
volatile bool measureTimerEnabled = false; // флаг, что мы должны прочитать данные с датчиков после старта измерений
volatile unsigned long query_interval = MEASURE_MIN_TIME; // тут будет интервал опроса
unsigned long last_measure_at = 0; // когда в последний раз запускали конвертацию

volatile bool connectedViaOneWire = false; // флаг, что мы присоединены к линии 1-Wire, при этом мы не сорим в эфир по nRF и не обновляем состояние по RS-485
volatile bool needResetOneWireLastCommandTimer = false;
volatile unsigned long oneWireLastCommandTimer = 0;

volatile uint16_t timeShift = 0;
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

    if(rs485Packet.type != RS485SensorDataPacket) // пакет не c запросом показаний датчика
      return;

     // теперь приводим пакет к нужному виду
     byte* readPtr = rs485Packet.data;
     // в первом байте у нас идёт тип датчика для опроса
     byte sensorType =  *readPtr++;
     // во втором - индекс датчика в системе
     byte sensorIndex = *readPtr++;

     // теперь нам надо найти, есть ли у нас этот датчик
     sensor* sMatch = NULL;
     if(scratchpadS.sensor1.type == sensorType && scratchpadS.sensor1.index == sensorIndex)
        sMatch = &(scratchpadS.sensor1);

     if(!sMatch)
     {
     if(scratchpadS.sensor2.type == sensorType && scratchpadS.sensor2.index == sensorIndex)
        sMatch = &(scratchpadS.sensor2);        
     }

     if(!sMatch)
     {
     if(scratchpadS.sensor3.type == sensorType && scratchpadS.sensor3.index == sensorIndex)
        sMatch = &(scratchpadS.sensor3);        
     }

     if(!sMatch) {// не нашли у нас такого датчика

      return;
     }

     memcpy(readPtr,sMatch->data,4); // у нас 4 байта на показания, копируем их все

     // выставляем нужное направление пакета
     rs485Packet.direction = RS485FromSlave;
     rs485Packet.type = RS485SensorDataPacket;

     // подсчитываем CRC
     rs485Packet.crc8 = OneWireSlave::crc8((const byte*) &rs485Packet,sizeof(RS485Packet)-1 );

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
  memset(&rs485Packet,0,sizeof(RS485Packet));
  // тут настраиваем RS-485 на приём
  pinMode(RS485_DE_PIN,OUTPUT);
  RS485Receive();
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
byte GetSensorType(const SensorSettings& sett)
{
  switch(sett.Type)
  {
    case mstNone:
      return uniNone;
    
    case mstDS18B20:
      return uniTemp;
      
    case mstBH1750:
    case mstMAX44009:
      return uniLuminosity;

    case mstSi7021:
    case mstDHT11:
    case mstDHT22:
    case mstSHT10:
      return uniHumidity;

    case mstChinaSoilMoistureMeter:
    case mstFrequencySoilMoistureMeter:
      return uniSoilMoisture;

    case mstPHMeter:
      return uniPH;

    case mstPinsMap:
      return uniPinsMap;
    
  }

  return uniNone;
}
//----------------------------------------------------------------------------------------------------------------
void SetDefaultValue(const SensorSettings& sett, byte* data)
{
  switch(sett.Type)
  {
    case mstNone:
      *data = 0xFF;
    break;
    
    case mstDS18B20:
    case mstChinaSoilMoistureMeter:
    case mstPHMeter:
    case mstFrequencySoilMoistureMeter:
    {
      *data = NO_TEMPERATURE_DATA;
    }
    break;
      
    case mstBH1750:
    case mstMAX44009:
    {
    long lum = NO_LUMINOSITY_DATA;
    memcpy(data,&lum,sizeof(lum));
    }
    break;

    case mstPinsMap:
    {
      memset(data,0,4);
    }
    break;

    case mstSi7021:
    case mstDHT11:
    case mstDHT22:
    case mstSHT10:
    {
    *data = NO_TEMPERATURE_DATA;
    data++; data++;
    *data = NO_TEMPERATURE_DATA;
    }
    break;

  }
}
//----------------------------------------------------------------------------------------------------------------
void* SensorDefinedData[3] = {NULL}; // данные, определённые датчиками при инициализации
//----------------------------------------------------------------------------------------------------------------
void* InitSensor(const SensorSettings& sett)
{
  switch(sett.Type)
  {
    case mstNone:
      return NULL;
    
    case mstDS18B20:
      return InitDS18B20(sett);

    case mstFrequencySoilMoistureMeter:
        return InitFrequencySoilMoistureMeter(sett);
    break;
      
    case mstBH1750:
      return InitBH1750(sett);

   case mstMAX44009:
    return InitMax44009(sett);

    case mstSi7021:
      return InitSi7021(sett);

    case mstDHT11:
      return InitDHT(sett,DHT_11);

    case mstDHT22:
      return InitDHT(sett,DHT_2x);

    case mstChinaSoilMoistureMeter:
      return NULL;

    case mstSHT10:
      return NULL;

    case mstPinsMap:
      return InitPinsMap(sett);

    case mstPHMeter: // инициализируем структуру для опроса pH
    {      
      PHMeasure* m = new PHMeasure;
      m->samplesDone = 0;
      m->samplesTimer = 0;
      m->data = 0;
      m->inMeasure = false; // ничего не измеряем
      return m;
    }
    break;
  }

  return NULL;  
}
//----------------------------------------------------------------------------------------------------------------
void ReadROM()
{
    memset((void*)&scratchpadS,0,sizeof(scratchpadS));
    eeprom_read_block((void*)&scratchpadS, ROM_ADDRESS, 29);

    // пишем номер канала по умолчанию
    if(scratchpadS.rf_id == 0xFF || scratchpadS.rf_id == 0)
      scratchpadS.rf_id = DEFAULT_RF_CHANNEL; 

    scratchpadS.packet_type = ptSensorsData; // говорим, что это тип пакета - данные с датчиками
    scratchpadS.packet_subtype = 0;


    // если интервала опроса не сохранено - выставляем по умолчанию
    if(scratchpadS.query_interval_min == 0xFF)
      scratchpadS.query_interval_min = 0;
      
    if(scratchpadS.query_interval_sec == 0xFF)
      scratchpadS.query_interval_sec =  MEASURE_MIN_TIME/1000;

   if(scratchpadS.query_interval_min == 0 && scratchpadS.query_interval_sec < 5) // минимум 5 секунд между обновлениями датчиков
    scratchpadS.query_interval_sec = 5;

    // вычисляем интервал опроса
    query_interval = (scratchpadS.query_interval_min*60 + scratchpadS.query_interval_sec)*1000;

    #ifdef _DEBUG
      Serial.print(F("Query interval: "));
      Serial.println(query_interval);
    #endif
    
    scratchpadS.sensor1.type = GetSensorType(Sensors[0]);
    scratchpadS.sensor2.type = GetSensorType(Sensors[1]);
    scratchpadS.sensor3.type = GetSensorType(Sensors[2]);

    SetDefaultValue(Sensors[0],scratchpadS.sensor1.data);
    SetDefaultValue(Sensors[1],scratchpadS.sensor2.data);
    SetDefaultValue(Sensors[2],scratchpadS.sensor3.data);

    // смотрим, есть ли у нас калибровка?
    byte calibration_enabled = false;
    for(byte i=0;i<3;i++)
    {
        switch(Sensors[i].Type)
        {
            case mstChinaSoilMoistureMeter:
            {
              calibration_enabled = true;
              // устанавливаем значения по умолчанию
              if(scratchpadS.calibration_factor1 == 0xFF && scratchpadS.calibration_factor2 == 0xFF)
              {
                // в EEPROM ничего нет по этому поводу, оба значения одинаковы
                scratchpadS.calibration_factor1 = map(SOIL_MOISTURE_0_PERCENT,0,1023,0,255);
                scratchpadS.calibration_factor2 = map(SOIL_MOISTURE_100_PERCENT,0,1023,0,255);
              }

              // мы поддерживаем два фактора калибровки
              scratchpadS.config |= (4 | 8);
            }
            break; // mstChinaSoilMoistureMeter

            case mstPHMeter:
            {
              calibration_enabled = true;
              // устанавливаем значения по умолчанию
              if(scratchpadS.calibration_factor1 == 0xFF)
              {
                // поскольку у нас беззнаковый байт - значение 127 соответствует калибровке 0,
                // всё, что меньше - отрицательная калибровка, что больше - положительная калибровка
                scratchpadS.calibration_factor1 = 127; 
              }

              scratchpadS.config |= 4; // поддерживаем всего один фактор калибровки
            }
            break; // mstPHMeter
          
        } // switch

        if(calibration_enabled)
          break;
    
    } // for

    if(calibration_enabled)
    {
      // включён фактор калибровки
      scratchpadS.config |= 2; // устанавливаем второй бит, говоря, что мы поддерживаем калибровку
    } // if
    else
    {
      scratchpadS.config &= ~2; // второй бит убираем по-любому
    }

}
//----------------------------------------------------------------------------------------------------------------
void WakeUpPinsMap()
{
  for(size_t i=0;i<sizeof(PINS_MAP);i++)
  {
    if(PINS_MAP[i] == -1)
      continue;

      Pin pin(PINS_MAP[i]);
      pin.inputMode();
  }
}
//----------------------------------------------------------------------------------------------------------------
void WakeUpSensor(const SensorSettings& sett, void* sensorDefinedData)
{  
  // просыпаем сенсоры
  switch(sett.Type)
  {
    case mstNone:
      break;
    
    case mstDS18B20:
      InitDS18B20(sett);
    break;

    case mstBH1750:
    {
      BH1750Support* bh = (BH1750Support*) sensorDefinedData;
      bh->begin((BH1750Address)sett.Pin);
    }
    break;

    case mstMAX44009:
    {
      Max44009* bh = (Max44009*) sensorDefinedData;
      bh->begin((MAX44009_ADDRESS) sett.Pin);
    }
    break;

    case mstSi7021:
    {
      Si7021* si = (Si7021*) sensorDefinedData;
      si->begin();
    }
    break;

    case mstChinaSoilMoistureMeter:
    case mstDHT11:
    case mstDHT22:
    case mstSHT10:
    break;

    case mstPinsMap:
    {
      WakeUpPinsMap();
    }
    break;

    case mstFrequencySoilMoistureMeter:
    {
      Pin pin(sett.Pin);
      pin.inputMode();
      pin.writeHigh();      
    }
    break;
    
    case mstPHMeter:
    {
      // надо подтянуть пин к питанию
      Pin pin(sett.Pin);
      pin.inputMode();
      pin.writeHigh();
      analogRead(sett.Pin);
    }
    break;
  }    
}
//----------------------------------------------------------------------------------------------------------------
void WakeUpSensors() // будим все датчики
{
#ifdef USE_POWER_SAVING  
  #ifdef _DEBUG
    Serial.println(F("Wake up sensors..."));
  #endif
  
  // включаем все линии
  linesPowerDown.write(LINES_POWER_UP_LEVEL);
  // ждём, пока питание прочухается
  delay(LINES_POWER_UP_DELAY);
  
 if(HasI2CSensors())
  PowerUpI2C(); // поднимаем I2C
 
   // будим датчики
   for(byte i=0;i<3;i++)
    WakeUpSensor(Sensors[i],SensorDefinedData[i]);

#endif
   
}
//----------------------------------------------------------------------------------------------------------------
void PowerDownSensors()
{
#ifdef USE_POWER_SAVING
  
   #ifdef _DEBUG
    Serial.println(F("Power down sensors..."));
  #endif 
  // выключаем все линии
  linesPowerDown.write(LINES_POWER_DOWN_LEVEL);
  
  PowerDownI2C(); // глушим шину I2C
  
#endif  
      
}
//----------------------------------------------------------------------------------------------------------------
void* InitDHT(const SensorSettings& sett, DHTType dhtType) // инициализируем датчик влажности DHT*
{
  UNUSED(sett);
  
  DHTSupport* dht = new DHTSupport(dhtType);
  
  return dht;
}
//----------------------------------------------------------------------------------------------------------------
void* InitSi7021(const SensorSettings& sett) // инициализируем датчик влажности Si7021
{
  UNUSED(sett);
  
  Si7021* si = new Si7021();
  si->begin();

  return si;
}
//----------------------------------------------------------------------------------------------------------------
void* InitFrequencySoilMoistureMeter(const SensorSettings& sett)
{
    UNUSED(sett);
    return NULL;  
}
//----------------------------------------------------------------------------------------------------------------
void* InitPinsMap(const SensorSettings& sett)
{
    UNUSED(sett);
    for(size_t i=0;i<sizeof(PINS_MAP);i++)
    {
      if(PINS_MAP[i] == -1)
        continue;

      Pin pin(PINS_MAP[i]);
      pin.inputMode();
    }
    return NULL;  
}
//----------------------------------------------------------------------------------------------------------------
void* InitMax44009(const SensorSettings& sett) // инициализируем датчик освещённости MAX44009
{
  Max44009* bh = new Max44009();
  
  bh->begin((MAX44009_ADDRESS)sett.Pin);
  
  return bh;
}
//----------------------------------------------------------------------------------------------------------------
void* InitBH1750(const SensorSettings& sett) // инициализируем датчик освещённости BH1750
{
  BH1750Support* bh = new BH1750Support();
  
  bh->begin((BH1750Address)sett.Pin);
  
  return bh;
}
//----------------------------------------------------------------------------------------------------------------
void* InitDS18B20(const SensorSettings& sett) // инициализируем датчик температуры
{
  #ifdef _DEBUG
    Serial.println(F("Init DS18B20..."));
  #endif
  
  if(!sett.Pin) {
    #ifdef _DEBUG
      Serial.println(F("DS18B20 - no pin number!!!"));
    #endif
    return NULL; 
  }  

   OneWire ow(sett.Pin);

  if(!ow.reset()) // нет датчика
  {
     #ifdef _DEBUG
      Serial.println(F("DS18B20 - not found during init!!!"));
    #endif
    return NULL;  
  }

   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x4E); // запускаем запись в scratchpad

   ow.write(0); // верхний температурный порог 
   ow.write(0); // нижний температурный порог
   ow.write(0x7F); // разрешение датчика 12 бит

   ow.reset();
   ow.write(0xCC); // пофиг на адреса (SKIP ROM)
   ow.write(0x48); // COPY SCRATCHPAD
   delay(10);
   ow.reset();

  #ifdef _DEBUG
    Serial.println(F("DS18B20 - inited."));
  #endif

   return NULL;
    
}
//----------------------------------------------------------------------------------------------------------------
void InitSensors()
{
  #ifdef _DEBUG
    Serial.println(F("Init sensors..."));
  #endif
  
  // инициализируем датчики
  for(byte i=0;i<3;i++)
    SensorDefinedData[i] = InitSensor(Sensors[i]);
         
}
//----------------------------------------------------------------------------------------------------------------
 void ReadDS18B20(const SensorSettings& sett, struct sensor* s) // читаем данные с датчика температуры
{ 
  
  #ifdef _DEBUG
    Serial.println(F("Read DS18B20..."));
  #endif
  
  s->data[0] = NO_TEMPERATURE_DATA;
  s->data[1] = 0;
  
  if(!sett.Pin)
  {
    #ifdef _DEBUG
      Serial.println(F("DS18B20 - no pin number!!!"));
    #endif       
    return;
  }

   OneWire ow(sett.Pin);
    
    if(!ow.reset()) // нет датчика на линии
    {
    #ifdef _DEBUG
      Serial.println(F("DS18B20 - not found!"));
    #endif   
    return; 
    }

  byte data[9] = {0};
  
  ow.write(0xCC); // пофиг на адреса (SKIP ROM)
  ow.write(0xBE); // читаем scratchpad датчика на пине

  for(uint8_t i=0;i<9;i++)
    data[i] = ow.read();


 if (OneWire::crc8(data, 8) != data[8]) // проверяем контрольную сумму
 {
  #ifdef _DEBUG
    Serial.println(F("DS18B20 - bad checksum!!!"));
  #endif     
      return;
 }
  int loByte = data[0];
  int hiByte = data[1];

  int temp = (hiByte << 8) + loByte;
  
  bool isNegative = (temp & 0x8000);
  
  if(isNegative)
    temp = (temp ^ 0xFFFF) + 1;

  int tc_100 = (6 * temp) + temp/4;

  if(isNegative)
    tc_100 = -tc_100;
   
  s->data[0] = tc_100/100;
  s->data[1] = abs(tc_100 % 100);

  #ifdef _DEBUG
    Serial.print(F("DS18B20: "));
    Serial.print(s->data[0]);
    Serial.print(F(","));
    Serial.println(s->data[1]);
    
  #endif     
    
}
//----------------------------------------------------------------------------------------------------------------
void ReadFrequencySoilMoistureMeter(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s)
{
    UNUSED(sensorDefinedData);

 int highTime = pulseIn(sett.Pin,HIGH);
 
 if(!highTime) // always HIGH ?
 {
   s->data[0] = NO_TEMPERATURE_DATA;

  // Serial.println("ALWAYS HIGH,  BUS ERROR!");

   return;
 }
 highTime = pulseIn(sett.Pin,HIGH);
 int lowTime = pulseIn(sett.Pin,LOW);


 if(!lowTime)
 {
  return;
 }
  int totalTime = lowTime + highTime;
  // теперь смотрим отношение highTime к общей длине импульсов - это и будет влажность почвы
  // totalTime = 100%
  // highTime = x%
  // x = (highTime*100)/totalTime;

  float moisture = (highTime*100.0)/totalTime;

  int moistureInt = moisture*100;

   s->data[0] = moistureInt/100;
   s->data[1] = moistureInt%100;
 
}
//----------------------------------------------------------------------------------------------------------------
void ReadPinsMap(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем карту пинов
{
  UNUSED(sett);
  UNUSED(sensorDefinedData);

  uint8_t pMap = 0;
  
  for(size_t i=0;i<sizeof(PINS_MAP);i++)
  {
    if(PINS_MAP[i] == -1)
      continue;

    Pin pin(PINS_MAP[i]);
    if(pin.read())
    {
      pMap |= (1 << i);
    }
  }
  
  memcpy(s->data,&pMap,sizeof(pMap));

}
//----------------------------------------------------------------------------------------------------------------
void ReadMax44009(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем данные с датчика освещённости MAX44009
{
  UNUSED(sett);
  Max44009* bh = (Max44009*) sensorDefinedData;

  long lum;
  float curLum =  bh->readLuminosity();

   if(curLum < 0)
      lum = NO_LUMINOSITY_DATA;
    else
    {
      unsigned long ulLum = (unsigned long) curLum;
      if(ulLum > 65535)
        ulLum = 65535;

      lum = ulLum;
    }
  
  memcpy(s->data,&lum,sizeof(lum));

}
//----------------------------------------------------------------------------------------------------------------
void ReadBH1750(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем данные с датчика освещённости BH1750
{
  UNUSED(sett);
  BH1750Support* bh = (BH1750Support*) sensorDefinedData;
  long lum = bh->GetCurrentLuminosity();
  memcpy(s->data,&lum,sizeof(lum));

}
//----------------------------------------------------------------------------------------------------------------
void ReadDHT(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем данные с датчика влажности Si7021
{
  DHTSupport* dht = (DHTSupport*) sensorDefinedData;
    
  HumidityAnswer ha;
  dht->read(sett.Pin,ha);

  memcpy(s->data,&ha,sizeof(ha));

}
//----------------------------------------------------------------------------------------------------------------
void ReadSi7021(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем данные с датчика влажности Si7021
{
  UNUSED(sett);
  Si7021* si = (Si7021*) sensorDefinedData;
  HumidityAnswer ha;
  si->read(ha);

  memcpy(s->data,&ha,sizeof(ha));



}
//----------------------------------------------------------------------------------------------------------------
void ReadSHT10(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s) // читаем данные с датчика влажности SHT10
{
  UNUSED(sett);
  UNUSED(sensorDefinedData);

  SHT1x sht(sett.Pin,sett.Pin2);
  float temp = sht.readTemperatureC();
  float hum = sht.readHumidity();

  HumidityAnswer ha;
  ha.Temperature = NO_TEMPERATURE_DATA;
  ha.Humidity =   NO_TEMPERATURE_DATA;

  if(((int)temp) != -40)
  {
    // has temperature
    int conv = temp * 100;
    ha.Temperature = conv/100;
    ha.TemperatureDecimal = abs(conv%100);
  }

  if(!(hum < 0))
  {
    // has humidity
    int conv = hum*100;
    ha.Humidity = conv/100;
    ha.HumidityDecimal = conv%100;
  }

  memcpy(s->data,&ha,sizeof(ha));

}
//----------------------------------------------------------------------------------------------------------------
void ReadChinaSoilMoistureMeter(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s)
{
   UNUSED(sensorDefinedData);
   
   int val = analogRead(sett.Pin);
   
   int soilMoisture0Percent = map(scratchpadS.calibration_factor1,0,255,0,1023);
   int soilMoisture100Percent = map(scratchpadS.calibration_factor2,0,255,0,1023);

   val = constrain(val,min(soilMoisture0Percent,soilMoisture100Percent),max(soilMoisture0Percent,soilMoisture100Percent));

   int percentsInterval = map(val,min(soilMoisture0Percent,soilMoisture100Percent),max(soilMoisture0Percent,soilMoisture100Percent),0,10000);
   
  // теперь, если у нас значение 0% влажности больше, чем значение 100% влажности - надо от 10000 отнять полученное значение
  if(soilMoisture0Percent > soilMoisture100Percent)
    percentsInterval = 10000 - percentsInterval;

   int8_t sensorValue;
   byte sensorFract;

   sensorValue = percentsInterval/100;
   sensorFract = percentsInterval%100;

   if(sensorValue > 99)
   {
      sensorValue = 100;
      sensorFract = 0;
   }

   if(sensorValue < 0)
   {
      sensorValue = NO_TEMPERATURE_DATA;
      sensorFract = 0;
   }

   s->data[0] = sensorValue;
   s->data[1] = sensorFract;
   
}
//----------------------------------------------------------------------------------------------------------------
void ReadPHValue(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s)
{
  
  UNUSED(sett);
  
  // подсчитываем актуальное значение pH
 PHMeasure* pm = (PHMeasure*) sensorDefinedData;

 pm->inMeasure = false; // говорим, что уже ничего не измеряем

 s->data[0] = NO_TEMPERATURE_DATA;
 
 if(pm->samplesDone > 0)
 {
  // сначала получаем значение калибровки, преобразовывая его в знаковое число
  int8_t calibration = map(scratchpadS.calibration_factor1,0,255,-128,127);
  
  // преобразуем полученное значение в среднее
  float avgSample = (pm->data*1.0)/pm->samplesDone;
  
  // теперь считаем вольтаж
  float voltage = avgSample*5.0/1024;
        
  // теперь получаем значение pH
  //unsigned long phValue = voltage*350 + calibration;
  float coeff = 700000/PH_MV_PER_7_PH;
  unsigned long phValue = voltage*coeff + calibration;
  
  #ifdef PH_REVERSIVE_MEASURE
    // считаем значение pH в условиях реверсивных измерений
    int16_t rev = phValue - 700; // поскольку у нас 7 pH - это средняя точка, то в условии реверсивных изменений от
    // средней точки pH (7.0) надо отнять разницу между значением 7 pH и полученным значением, что мы и делаем
    phValue = 700 - rev;
   #endif
             
    if(avgSample > 1000)
    {
      // не прочитали из порта ничего, потому что у нас включена подтяжка к питанию
    }
    else
    {
      s->data[0] = phValue/100;
      s->data[1] = phValue%100;
    } // else
  
 } // pm->samplesDone > 0

 // сбрасываем данные в 0
 pm->samplesDone = 0;
 pm->samplesTimer = 0;
 pm->data = 0;  
  
}
//----------------------------------------------------------------------------------------------------------------
void ReadSensor(const SensorSettings& sett, void* sensorDefinedData, struct sensor* s)
{
  switch(sett.Type)
  {
    case mstNone:
      
    break;

    case mstDS18B20:
    ReadDS18B20(sett,s);
    break;

    case mstPinsMap:
    ReadPinsMap(sett,sensorDefinedData,s);
    break;

    case mstBH1750:
    ReadBH1750(sett,sensorDefinedData,s);
    break;

    case mstMAX44009:
    ReadMax44009(sett,sensorDefinedData,s);
    break;

    case mstSi7021:
    ReadSi7021(sett,sensorDefinedData,s);
    break;

    case mstSHT10:
    ReadSHT10(sett,sensorDefinedData,s);
    break;

    case mstChinaSoilMoistureMeter:
      ReadChinaSoilMoistureMeter(sett,sensorDefinedData,s);
    break;

    case mstPHMeter:
      ReadPHValue(sett,sensorDefinedData,s);
    break;

    case mstDHT11:
    case mstDHT22:
      ReadDHT(sett,sensorDefinedData,s);
    break;

    case mstFrequencySoilMoistureMeter:
      ReadFrequencySoilMoistureMeter(sett,sensorDefinedData,s);
    break;
  }
}
//----------------------------------------------------------------------------------------------------------------
void ReadSensors()
{
  #ifdef _DEBUG
    Serial.println(F("Read sensors..."));
  #endif  
  // читаем информацию с датчиков
    
  ReadSensor(Sensors[0],SensorDefinedData[0],&scratchpadS.sensor1);
  
  #ifdef USE_WATCHDOG
    wdt_reset();
  #endif
  
  ReadSensor(Sensors[1],SensorDefinedData[1],&scratchpadS.sensor2);

  #ifdef USE_WATCHDOG
    wdt_reset();
  #endif
  
  ReadSensor(Sensors[2],SensorDefinedData[2],&scratchpadS.sensor3);

  #ifdef USE_WATCHDOG
    wdt_reset();
  #endif

}
//----------------------------------------------------------------------------------------------------------------
void MeasureDS18B20(const SensorSettings& sett)
{
    
  #ifdef _DEBUG
    Serial.println(F("DS18B20 - start conversion..."));
  #endif
  
  if(!sett.Pin)
  {
    #ifdef _DEBUG
      Serial.println(F("DS18B20 - no pin number!!!"));
    #endif
    return;
  }

   OneWire ow(sett.Pin);
    
    if(!ow.reset()) // нет датчика на линии
    {
      #ifdef _DEBUG
        Serial.println(F("DS18B20 - not found!!!"));
      #endif    
      return; 
    }

    ow.write(0xCC);
    ow.write(0x44); // посылаем команду на старт измерений
    
    ow.reset();    

  #ifdef _DEBUG
    Serial.println(F("DS18B20 - converted."));
  #endif    
  
}
//----------------------------------------------------------------------------------------------------------------
bool HasI2CSensors()
{
  // проверяем, есть ли у нас хоть один датчик на I2C
  for(byte i=0;i<3;i++)
  {
    switch(Sensors[i].Type)
    {
      case mstBH1750:
      case mstSi7021:
      case mstMAX44009:
        return true;
    }
    
  } // for
  return false;
}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_POWER_SAVING
inline void PowerUpI2C()
{
  power_twi_enable();
  delay(20);
}
//----------------------------------------------------------------------------------------------------------------
inline void PowerDownI2C()
{
  power_twi_disable();
}
#endif
//----------------------------------------------------------------------------------------------------------------
void MeasurePH(const SensorSettings& sett,void* sensorDefinedData)
{
 // начинаем измерения pH здесь 
 PHMeasure* pm = (PHMeasure*) sensorDefinedData;
 
 if(pm->inMeasure) // уже измеряем
  return;
  
 pm->samplesDone = 0;
 pm->samplesTimer = millis();
 pm->data = 0;
 // читаем из пина и игнорируем это значение
 analogRead(sett.Pin);
 pm->inMeasure = true; // говорим, что готовы измерять
}
//----------------------------------------------------------------------------------------------------------------
void MeasureSensor(const SensorSettings& sett,void* sensorDefinedData) // запускаем конвертацию с датчика, если надо
{
  switch(sett.Type)
  {
    case mstNone:    
    break;

    case mstDS18B20:
    MeasureDS18B20(sett);
    break;

    case mstPHMeter:
      MeasurePH(sett,sensorDefinedData);
    break;

    case mstBH1750:
    case mstMAX44009:
    case mstSi7021:
    case mstChinaSoilMoistureMeter:
    case mstDHT11:
    case mstDHT22:
    case mstFrequencySoilMoistureMeter:
    case mstSHT10:
    case mstPinsMap:
    break;
  }  
}
//----------------------------------------------------------------------------------------------------------------
void UpdatePH(const SensorSettings& sett,void* sensorDefinedData)
{
  PHMeasure* pm = (PHMeasure*) sensorDefinedData;
  
  if(!pm->inMeasure) // ничего не меряем
    return;
    
  if(pm->samplesDone >= PH_NUM_SAMPLES) // закончили измерения
  {    
    pm->inMeasure = false;
    return;
  }
    
  if((millis() - pm->samplesTimer) > PH_SAMPLES_INTERVAL)
  {
    
    // пора прочитать из порта
    pm->samplesDone++;
    pm->data += analogRead(sett.Pin);
    pm->samplesTimer = millis(); // запоминаем, когда замерили
  }
}
//----------------------------------------------------------------------------------------------------------------
void UpdateSensor(const SensorSettings& sett,void* sensorDefinedData)
{
  // обновляем датчики здесь
  switch(sett.Type)
  {

    case mstPHMeter:
      UpdatePH(sett,sensorDefinedData);
    break;

    case mstNone:    
    case mstDS18B20:
    case mstBH1750:
    case mstMAX44009:
    case mstSi7021:
    case mstChinaSoilMoistureMeter:
    case mstDHT11:
    case mstDHT22:
    case mstFrequencySoilMoistureMeter:
    case mstSHT10:
    case mstPinsMap:
    break;
  }  
}
//----------------------------------------------------------------------------------------------------------------
void UpdateSensors()
{
  for(byte i=0;i<3;i++)
  {
    UpdateSensor(Sensors[i],SensorDefinedData[i]);
    #ifdef USE_WATCHDOG
      wdt_reset();
    #endif    
  }
}
//----------------------------------------------------------------------------------------------------------------
void StartMeasure()
{  
  #ifdef _DEBUG
    Serial.println(F("Start measure..."));
  #endif
    
 WakeUpSensors(); // будим все датчики
  
  // запускаем конвертацию
  for(byte i=0;i<3;i++)
    MeasureSensor(Sensors[i],SensorDefinedData[i]);

  last_measure_at = millis();
}
//----------------------------------------------------------------------------------------------------------------
#ifdef USE_NRF
//----------------------------------------------------------------------------------------------------------------
//uint64_t controllerStatePipe = 0xF0F0F0F0E0LL; // труба, с которой мы слушаем состояние контроллера
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
  radio.setChannel(scratchpadS.rf_id);
  radio.setRetries(15,15);
  radio.setPayloadSize(sizeof(t_scratchpad)); // у нас 30 байт на пакет
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
    
  if(!((scratchpadS.config & 1) == 1))
  {
    #ifdef _DEBUG
    Serial.println(F("Transiever disabled."));
    #endif
    return;
  }
  
  radio.powerUp(); // просыпаемся
  
  #ifdef _DEBUG
    Serial.println(F("Send sensors data via nRF..."));
  #endif

    bool sendDone = false;

    for(int i=0;i<5;i++) // пытаемся послать 5 раз, в разные трубы
    {

      #ifdef USE_WATCHDOG
        wdt_reset();
      #endif
      
      // посылаем данные через nRF
        uint8_t writePipeNum = random(0,5);
        radio.openWritingPipe(writingPipes[writePipeNum]); // открываем канал для записи
    
        // подсчитываем контрольную сумму
        scratchpadS.crc8 = OneWireSlave::crc8((const byte*)&scratchpadS,sizeof(scratchpadS)-1);
            
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
    //LoRa.receive(); // начинаем слушать
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
    
  if(!((scratchpadS.config & 1) == 1))
  {
    #ifdef _DEBUG
    Serial.println(F("LoRa: transiever disabled."));
    #endif
    return;
  }
    
  #ifdef _DEBUG
    Serial.println(F("Send sensors data via LoRa..."));
  #endif

  bool sendDone = false;

    for(int i=0;i<5;i++) // пытаемся послать 5 раз
    {
      #ifdef USE_WATCHDOG
        wdt_reset();
      #endif
      
        // подсчитываем контрольную сумму
        scratchpadS.crc8 = OneWireSlave::crc8((const byte*)&scratchpadS,sizeof(scratchpadS)-1);  
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
    
  //LoRa.receive();

  // рандомная задержка
  delay(random(50));

  LoRa.sleep(); // засыпаем

}
//----------------------------------------------------------------------------------------------------------------
#endif // USE_LORA
//----------------------------------------------------------------------------------------------------------------
void WriteROM()
{

    scratchpadS.sensor1.type = GetSensorType(Sensors[0]);
    scratchpadS.sensor2.type = GetSensorType(Sensors[1]);
    scratchpadS.sensor3.type = GetSensorType(Sensors[2]);
  
    eeprom_write_block( (void*)scratchpad,ROM_ADDRESS,29);
    memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));
    scratchpadToSend.crc8 = OneWireSlave::crc8((const byte*)&scratchpadToSend,sizeof(scratchpadS)-1);

    #ifdef USE_NRF
      // переназначаем канал радио
      if(nRFInited)
        radio.setChannel(scratchpadS.rf_id);
        
    #endif
    

}
//----------------------------------------------------------------------------------------------------------------
void owReceive(OneWireSlave::ReceiveEvent evt, byte data);
//----------------------------------------------------------------------------------------------------------------
void setup()
{

 #ifdef USE_RANDOM_SEED_PIN
    randomSeed(analogRead(RANDOM_SEED_PIN));
 #endif

  // небольшой сдвиг по времени
  timeShift = random(100);

  
  #ifdef _DEBUG
    Serial.begin(57600);
  #endif
  
 #ifdef USE_RS485_GATE // если сказано работать через RS-485 - работаем 
 
    #ifndef _DEBUG
      Serial.begin(RS485_SPEED);
    #endif
    
    InitRS485(); // настраиваем RS-485 на приём
 #endif

  #ifdef USE_POWER_SAVING
    // настраиваем пин управления линиями на выход
    linesPowerDown.outputMode();
    
    // включаем все линии на период настройки
   linesPowerDown.write(LINES_POWER_UP_LEVEL);
  #endif
    
  ReadROM();
  
  scratchpadS.crc8 = OneWireSlave::crc8((const byte*) scratchpad,sizeof(scratchpadS)-1);
  memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));

   InitSensors(); // инициализируем датчики   
   PowerDownSensors(); // и выключаем их нафик при старте
    
    #ifdef USE_NRF
      initNRF();
    #endif

    #ifdef USE_LORA
      initLoRa();
    #endif

  oneWireLastCommandTimer = millis();
  
  OWSlave.setReceiveCallback(&owReceive);
  OWSlave.begin(owROM, oneWireData.getPinNumber());

  #ifdef USE_WATCHDOG
    wdt_enable(WDTO_8S);
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
  connectedViaOneWire = true; // говорим, что мы подключены через 1-Wire
  needResetOneWireLastCommandTimer = true; // просим, чтобы сбросили таймер момента получения последней команды
  
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
        if(!measureTimerEnabled && !needToMeasure) // только если она уже не запущена
          needToMeasure = true;
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

        default:
          state = DS_WaitingReset;
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
    scratchpadReceivedFromMaster = false;

    
    // скратч был получен от мастера, тут можно что-то делать
    memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));
    scratchpadToSend.crc8 = OneWireSlave::crc8((const byte*) &scratchpadToSend,sizeof(scratchpadS)-1);

    // вычисляем новый интервал опроса
    query_interval = (scratchpadToSend.query_interval_min*60 + scratchpadToSend.query_interval_sec)*1000;
     #ifdef _DEBUG
        Serial.println(F("Scratch received from master!"));
     #endif
      
  } // scratchpadReceivedFromMaster

  
  unsigned long curMillis = millis();

  // если попросили сбросить таймер получения последней команды по линии 1-Wire - делаем это
  if(needResetOneWireLastCommandTimer) 
  {
    oneWireLastCommandTimer = curMillis;
    needResetOneWireLastCommandTimer = false;
  }

  // проверяем - когда приходила последняя команда по 1-Wire: если её не было больше 15 секунд - активируем nRF и RS-485
  if(connectedViaOneWire) 
  {             
      if((curMillis - oneWireLastCommandTimer) > 15000) 
      {
         #ifdef _DEBUG
            Serial.print(F("Last command at: "));
            Serial.print(oneWireLastCommandTimer);
            Serial.print(F("; curMillis: "));
            Serial.print(curMillis);
            Serial.print(F("; diff = "));
            Serial.println((curMillis - oneWireLastCommandTimer));
         #endif
        
          connectedViaOneWire = false; // соединение через 1-Wire разорвано
      }
  }
  


  if(!connectedViaOneWire && ((curMillis - last_measure_at) > (query_interval + timeShift)) && !measureTimerEnabled && !needToMeasure) 
  {
    // чего-то долго не запускали конвертацию, запустим, пожалуй
      // и запустим только тогда, когда мы не подключены к 1-Wire, иначе - мастер сам запросит конвертацию.
        needToMeasure = true;
        #ifdef _DEBUG
          Serial.println(F("Want measure by timeout..."));
        #endif        
  }

  // только если ничего не делаем на линии 1-Wire и запросили конвертацию
  
  if(needToMeasure && !measureTimerEnabled) 
  {
    #ifdef _DEBUG
      Serial.println(F("Want measure..."));
    #endif    

    measureTimerEnabled = true; // включаем флаг, что мы должны прочитать данные с датчиков
    sensorsUpdateTimer = curMillis; // сбрасываем таймер обновления
    StartMeasure();

    needToMeasure = false;
    
    #ifdef _DEBUG
      Serial.println(F("Wait for measure complete..."));
    #endif    
  }
 

  if(measureTimerEnabled) 
  {
    UpdateSensors(); // обновляем датчики, если кому-то из них нужно периодическое обновление
  }
  
  if(measureTimerEnabled && ((curMillis - sensorsUpdateTimer) > MEASURE_MIN_TIME)) 
  {
    
    if(state != DS_SendingScratchpad)
    {

          #ifdef _DEBUG
            Serial.println(F("Measure completed, start read..."));
          #endif
        
             // можно читать информацию с датчиков
             ReadSensors();
             
             //noInterrupts();
             memcpy(&scratchpadToSend,&scratchpadS,sizeof(scratchpadS));
             scratchpadToSend.crc8 = OneWireSlave::crc8((const byte*) &scratchpadToSend,sizeof(scratchpadS)-1);
             //interrupts();
        

          #ifdef _DEBUG
            Serial.println(F("Sensors data readed."));
          #endif       
        
             // теперь усыпляем все датчики
             PowerDownSensors();
        
             // прочитали, отправляем
             #ifdef USE_NRF
              if(!connectedViaOneWire)
                sendDataViaNRF();
             #endif

             #ifdef USE_LORA
              if(!connectedViaOneWire)
                sendDataViaLoRa();
             #endif             
        
          #ifdef _DEBUG
            Serial.println(F(""));
          #endif 
          
             sensorsUpdateTimer = curMillis;
             measureTimerEnabled = false;

           // пытаемся заснуть на одну секунду
           #ifdef USE_LOW_POWER
            #ifndef USE_RS485_GATE
                if(!connectedViaOneWire)
                {
                   LowPower.powerDown(SLEEP_1S, ADC_OFF, BOD_OFF);  
                   extern volatile unsigned long timer0_millis;
                   noInterrupts();
                    timer0_millis += 1000;
                   interrupts();
                }
            #endif // USE_RS485_GATE
           #endif // USE_LOW_POWER


         timeShift = random(100);
             
      } // if(state != DS_SendingScratchpad)      
  }

  #ifdef USE_RS485_GATE
    if(!connectedViaOneWire)
    {
      ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
    }
  #endif  

  #ifdef USE_WATCHDOG
    wdt_reset();
  #endif
}
//----------------------------------------------------------------------------------------------------------------
void yield()
{
   #ifdef USE_RS485_GATE
    if(!connectedViaOneWire)
    {
      ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485
    }
  #endif   

  #ifdef USE_WATCHDOG
    wdt_reset();
  #endif  
}
//----------------------------------------------------------------------------------------------------------------


