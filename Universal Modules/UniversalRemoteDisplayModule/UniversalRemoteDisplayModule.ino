#include <avr/io.h>
#include <avr/interrupt.h>
#include "Common.h"
#include "CorePinScenario.h"
#include "NextionModule.h"
//----------------------------------------------------------------------------------------------------------------
/*
Прошивка для модуля, предназначена для выносного дисплея Nextion на шине RS-485.

Прошивка тестировалась под Arduino Uno, но должна без особых помех
работать и под Pro Mini и пр. семейством на Atmega328.

Все настройки - в Common.h !!!

*/
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
RS485Packet rs485Packet; // пакет, в который мы принимаем данные
volatile byte* rsPacketPtr = (byte*) &rs485Packet;
volatile byte  rs485WritePtr = 0; // указатель записи в пакет
uint32_t lastMillis = 0;
CommandsPool commandsPool; // пул команд
#ifdef USE_INFO_DIODE
  CorePinScenario blinker;
  unsigned long blinkerTimer = 0;
  bool anyCommandExistsToExecute = false;
#endif
//----------------------------------------------------------------------------------------------------------------
void RS485Receive()
{
   digitalWrite(RS485_DE_PIN,LOW);
  #ifdef _DEBUG
    Serial.println(F("Switch RS485 to receive."));
  #endif  
}
//----------------------------------------------------------------------------------------------------------------
void RS485Send()
{
  // переводим контроллер RS-485 на передачу
    digitalWrite(RS485_DE_PIN,HIGH);
    
  #ifdef _DEBUG
    Serial.println(F("Switch RS485 to send."));
  #endif  
}
//----------------------------------------------------------------------------------------------------------------
void RS485waitTransmitComplete()
{
  // ждём завершения передачи по UART
  //while(!(UCSR0A & _BV(TXC0) ));
  RS485_SERIAL.flush();
}
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
byte crc8(const byte *addr, byte len)
{
  byte crc = 0;
  while (len--) 
    {
    byte inbyte = *addr++;
    for (byte i = 8; i; i--)
      {
      byte mix = (crc ^ inbyte) & 0x01;
      crc >>= 1;
      if (mix) 
        crc ^= 0x8C;
      inbyte >>= 1;
      }  // end of for
    }  // end of while
  return crc;  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SendCurrentCommandsState(RS485Packet& packet)
{

  if(!commandsPool.size()) // нечего отправлять
    return;
  
  CommandsToExecutePacket* ce = (CommandsToExecutePacket*) &(packet.data);
  packet.direction = RS485FromSlave;
  packet.type = RS485RequestCommandsPacket;

  // тут копируем первые 7 команд из пула
  CommandsPool toExecute, remaining;
  uint8_t stopIndex = min(7,commandsPool.size());

  for(uint8_t i=0;i<stopIndex;i++)
  {
    toExecute.push_back(commandsPool[i]);
  }

  for(uint8_t i=stopIndex;i<commandsPool.size();i++)
  {
    remaining.push_back(commandsPool[i]);
  }

  commandsPool = remaining;
  
  
  for(uint8_t i=0;i<toExecute.size();i++)
  {
      CommandToExecute cmd = toExecute[i];
      memcpy(&(ce->commands[i]),&cmd,sizeof(CommandToExecute));      
  }

  #ifdef USE_INFO_DIODE
    anyCommandExistsToExecute = toExecute.size() > 0;
  #endif


  packet.crc8 = crc8((const byte*)&packet,sizeof(packet)-1);

  RS485Send();

  RS485_SERIAL.write((byte*) &packet,sizeof(packet));

  RS485waitTransmitComplete();

  RS485Receive();
  
}
//----------------------------------------------------------------------------------------------------------------
void ProcessReceiptPacket(const RS485Packet& packet)
{
  UNUSED(packet);
  
  // сообщаем пользователю, что команда отработана контроллером
  #ifdef USE_INFO_DIODE

     if(anyCommandExistsToExecute)
     {
      anyCommandExistsToExecute = false;
      blinkerTimer = millis();
      blinker.reset();
      blinker.enable();
     }
      
  #endif    
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
    byte crc = crc8((const byte*) rsPacketPtr,sizeof(RS485Packet) - 1);
    if(crc != rs485Packet.crc8)
    {
      // не сошлось, игнорируем
      return;
    }


    // всё в пакете правильно, анализируем и выполняем
    // проверяем, наш ли пакет
    if(rs485Packet.direction != RS485FromMaster) // не от мастера пакет
      return;

    if(!(rs485Packet.type == RS485RequestCommandsPacket 
    || rs485Packet.type == RS485CommandsToExecuteReceipt
    || rs485Packet.type == RS485ControllerStatePacket
    || rs485Packet.type == RS485SensorDataForRemoteDisplay
    || rs485Packet.type == RS485SettingsForRemoteDisplay
    
    )) // пакет не наш
      return;


      if(rs485Packet.type == RS485RequestCommandsPacket)
      {
       SendCurrentCommandsState(rs485Packet);
      }
      else if(rs485Packet.type == RS485CommandsToExecuteReceipt)
      {
        ProcessReceiptPacket(rs485Packet);
      }
      else if(rs485Packet.type == RS485ControllerStatePacket)
      {
       // пакет состояния контроллера
       ControllerState* state = (ControllerState*) &(rs485Packet.data);
       display.setControllerState(*state);
      }
      else if(rs485Packet.type == RS485SensorDataForRemoteDisplay)
      {
        // данные для дисплея
        RemoteDisplaySensorsPacket* data = (RemoteDisplaySensorsPacket*) &(rs485Packet.data);
       
          if(data->firstOrLastPacket & REMOTE_DISPLAY_FIRST_SENSORS_PACKET)
          {
            display.clearSensors();
          }
          for(uint8_t qq = 0; qq < data->sensorsInPacket; qq++)
          {
            SensorDisplayData sdd;
            sdd.hasData = data->hasDataFlags & (1 << qq);
            sdd.type = data->data[qq].type;
            memcpy(sdd.data,data->data[qq].data,4);
            display.addSensor(sdd);          
          } // for
       
      } // RS485SensorDataForRemoteDisplay
      else if(rs485Packet.type == RS485SettingsForRemoteDisplay)
      {
        RemoteDisplaySettingsPacket* data = (RemoteDisplaySettingsPacket*) &(rs485Packet.data);
        display.setSettings(*data);
      } // RS485SettingsForRemoteDisplay


    
  } // else
}
//----------------------------------------------------------------------------------------------------------------
void ProcessIncomingRS485Packets() // обрабатываем входящие пакеты по RS-485
{
  while(RS485_SERIAL.available())
  {
    rsPacketPtr[rs485WritePtr++] = (byte) RS485_SERIAL.read();
   
    if(GotRS485Packet())
      ProcessRS485Packet();
  } // while
  
}
//----------------------------------------------------------------------------------------------------------------
void InitRS485()
{
  memset(&rs485Packet,0,sizeof(RS485Packet));

  #ifdef _DEBUG
    Serial.println(F("Init RS485 DE pin..."));
  #endif

    pinMode(RS485_DE_PIN,OUTPUT);
    
  RS485Receive();

}
//----------------------------------------------------------------------------------------------------------------
void setup()
{
 
  RS485_SERIAL.begin(RS485_SPEED);

 #ifdef _DEBUG
  if(&RS485_SERIAL != &Serial)
  {
    Serial.begin(RS485_SPEED);
  }
  Serial.println(F("Debug mode..."));
 #endif

  display.begin(); // настраиваем дисплей

  InitRS485(); // настраиваем RS-485 на приём

  #ifdef USE_INFO_DIODE
  blinker.add({INFO_DIODE_PIN,INFO_DIODE_ON_LEVEL,100});
  blinker.add({INFO_DIODE_PIN,!INFO_DIODE_ON_LEVEL,100});
  blinker.add({INFO_DIODE_PIN,INFO_DIODE_ON_LEVEL,100});
  blinker.add({INFO_DIODE_PIN,!INFO_DIODE_ON_LEVEL,100});
  blinker.add({INFO_DIODE_PIN,INFO_DIODE_ON_LEVEL,500});
  blinker.add({INFO_DIODE_PIN,!INFO_DIODE_ON_LEVEL,1000});
  blinker.disable();
  pinMode(INFO_DIODE_PIN,OUTPUT);
  digitalWrite(INFO_DIODE_PIN,!INFO_DIODE_ON_LEVEL);
  #endif
  
  
}
//----------------------------------------------------------------------------------------------------------------
void loop()
{
    ProcessIncomingRS485Packets(); // обрабатываем входящие пакеты по RS-485

    uint32_t curMillis = millis();
    uint16_t dt = curMillis - lastMillis;
    lastMillis = curMillis;
    display.update(dt); // обновили дисплей
    // если с дисплея есть команды - сохраняем их в пул
    uint8_t cnt = display.getOutgoingCommandsCount();
    
    for(uint8_t i=0;i<cnt;i++)
    {
      commandsPool.push_back(display.getOutgoingCommand(i));
    }

    display.clearOutgoingCommands();

    #ifdef USE_INFO_DIODE
      if(blinker.enabled())
      {
        blinker.update();
        unsigned long now = millis();
        if(now - blinkerTimer > 1900)
        {
          blinker.disable();
          digitalWrite(INFO_DIODE_PIN,!INFO_DIODE_ON_LEVEL);
        }
      }
    #endif
}
//----------------------------------------------------------------------------------------------------------------
