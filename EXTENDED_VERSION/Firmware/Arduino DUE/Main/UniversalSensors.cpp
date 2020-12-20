#include "UniversalSensors.h"
#include "Globals.h"
#if TARGET_BOARD == STM32_BOARD
#include "OneWireSTM.h"
#else
#include <OneWire.h>
#endif
#include "Memory.h"
#include "InteropStream.h"
#ifdef USE_TEMP_SENSORS
#include "TempSensors.h"
#endif
#include "LogicManageModule.h"
#include "EEPROMSettingsModule.h"
#include "Utils.h"
#include "WaterTankModule.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniRegDispatcher UniDispatcher;
UniScratchpadClass UniScratchpad; // наш пишичитай скратчпада
UniClientsFactory UniFactory; // наша фабрика клиентов
UniRawScratchpad SHARED_SCRATCHPAD; // общий скратчпад для классов опроса модулей, висящих на линиях
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void ProcessPinsMap(uint8_t index, uint8_t* data)
{
  #ifdef UNI_DEBUG
    DEBUG_LOGLN(F("Process PINS MAP!"));
  #endif

  // проходим карту пинов, по 8 пинов на один модуль со слотом "карта пинов"
  // выясняем кол-во возможных модулей
  const uint8_t totalModulesAvailable = (128 - VIRTUAL_PIN_START_NUMBER)/8;

  if(index >= totalModulesAvailable)// что-то пошло не так
    return;

  uint16_t startIndex = VIRTUAL_PIN_START_NUMBER +(8ul*index);

  if(startIndex > 120) // что-то пошло не так
    return;

  int endIndex = startIndex + 8;
  if(endIndex > 128)
    endIndex = 128;

  // получаем первый байт, в котором и содержится статут пинов
  uint8_t pinsState = *data;

  // теперь проходим по карте пинов, и выставляем статусы
  uint8_t shiftCounter = 0;

  for(uint8_t i=startIndex; i<endIndex;i++)
  {
    // сбрасываем нужный пин
    WORK_STATUS.PinWrite(i,LOW);

    // проверяем - если нужный бит в карте состояний установлен - пишем в виртуальный пин высокий уровень
    if(pinsState & (1 << shiftCounter))
    {
      WORK_STATUS.PinWrite(i,HIGH);
    }

    shiftCounter++;
  }

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_RS485_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniRS485Gate::UniRS485Gate()
{
    workStream = NULL;
#ifdef USE_UNI_EXECUTION_MODULE  
  updateTimer = 0;
#endif  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_UNIVERSAL_MODULES
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRS485Gate::isInOnlineQueue(const RS485QueueItem& item)
{
  for(size_t i=0;i<sensorsOnlineQueue.size();i++)
  {
    if(sensorsOnlineQueue[i].sensorType == item.sensorType && sensorsOnlineQueue[i].sensorIndex == item.sensorIndex)
    {
      return true;
    }
  }
  
  return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_UNIVERSAL_MODULES
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::enableSend()
{
  if(!workStream)
  {
    return;
  }
  
  // переводим контроллер RS-485 на передачу
  RS485Binding bnd = HardwareBinding->GetRS485Binding();
  
  if(bnd.LinkType != linkUnbinded && bnd.DEPin != UNBINDED_PIN)
  {
    if(bnd.LinkType == linkDirect)
    {
        #ifndef DISABLE_RS485_CONFIGURE
        if(EEPROMSettingsModule::SafePin(bnd.DEPin))
        #else
        if(bnd.DEPin > 1) // не даём блокировать Serial
        #endif
        {
            digitalWrite(bnd.DEPin,HIGH); 
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress, bnd.DEPin, HIGH);
      #endif 
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress, bnd.DEPin, HIGH);
      #endif
    }
  } // if(bnd.LinkType != linkUnbinded
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::enableReceive()
{
  if(!workStream)
  {
    return;
  }
  
  // освобождаем буфер перед приёмом, чтобы точно не было никакого мусора
  while(workStream->available())
  {
    workStream->read();
  }

  // переводим контроллер RS-485 на приём
  RS485Binding bnd = HardwareBinding->GetRS485Binding();
  
  if(bnd.LinkType != linkUnbinded && bnd.DEPin != UNBINDED_PIN)
  {
    if(bnd.LinkType == linkDirect)
    {
        #ifndef DISABLE_RS485_CONFIGURE
        if(EEPROMSettingsModule::SafePin(bnd.DEPin))
        #else
        if(bnd.DEPin > 1) // не даём блокировать Serial
        #endif
        {
            digitalWrite(bnd.DEPin,LOW);
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress, bnd.DEPin, LOW);
      #endif 
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress, bnd.DEPin, LOW);
      #endif
    }
  } // if(bnd.LinkType != linkUnbinded  
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::Setup()
{

  RS485Binding bnd = HardwareBinding->GetRS485Binding();
  
  if(bnd.LinkType != linkUnbinded && bnd.DEPin != UNBINDED_PIN)
  {
    if(bnd.LinkType == linkDirect)
    {
        #ifndef DISABLE_RS485_CONFIGURE
        if(EEPROMSettingsModule::SafePin(bnd.DEPin))
        #else
        if(bnd.DEPin > 1) // не даём блокировать Serial
        #endif
        {
           WORK_STATUS.PinMode(bnd.DEPin,OUTPUT);
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress, bnd.DEPin, OUTPUT);
      #endif 
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress, bnd.DEPin, OUTPUT);
      #endif
    }
  } // if(bnd.LinkType != linkUnbinded  

  #if TARGET_BOARD == STM32_BOARD
    SerialUART* hs = NULL;
  #else
    HardwareSerial* hs = NULL;
  #endif

  if(bnd.SerialNumber == 1) // Serial1
  {
    hs = &Serial1;
  }
  else
  if(bnd.SerialNumber == 2) // Serial2
  {
    hs = &Serial2;
  }
  else
  if(bnd.SerialNumber == 3) // Serial3
  {
    hs = &Serial3;
  }
  #if TARGET_BOARD == STM32_BOARD
  else
  if(bnd.SerialNumber == 4) // Serial4
  {
    hs = &Serial4;
  }
  else
  if(bnd.SerialNumber == 5) // Serial5
  {
    hs = &Serial5;
  }
  else
  if(bnd.SerialNumber == 6) // Serial6
  {
    hs = &Serial6;
  }
  #endif
  
  workStream = hs;

  if(hs)
  {
    hs->begin(SERIAL_BAUD_RATE);
  }


  if(workStream)
  {
    // на всякий случай освобождаем буфер от возможного мусора
    while(workStream->available())
    {
      workStream->read();
    }
  }

  enableSend();
 
  
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::waitTransmitComplete()
{
  if(!workStream)
  {
    return;
  }
  
  // ждём завершения передачи по UART
  workStream->flush();
 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::executeCommands(const RS485Packet& packet)
{
  CommandsToExecutePacket* cePacket = (CommandsToExecutePacket*) &(packet.data);
  for(int i=0;i<7;i++)
  {
      switch(cePacket->commands[i].whichCommand)
      {
          case emCommandNone:
          break;

          case emCommandOpenWindows:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Open windows!"));        
              #endif

              String cmd = F("STATE|WINDOW|ALL|OPEN");
              
              if(cePacket->commands[i].param2 > 0)
              {
                // запросили открыть на проценты
                cmd += '|';
                cmd += cePacket->commands[i].param2;
                cmd += "%";
              }
              
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;
          
          case emCommandCloseWindows:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Close windows!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("STATE|WINDOW|ALL|CLOSE"),false);          
          }
          break;

          case emCommandOpenWindow:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: open window!"));        
              #endif
              String cmd = F("STATE|WINDOW|");
              cmd += cePacket->commands[i].param1;
              cmd += F("|OPEN");

              if(cePacket->commands[i].param2 > 0)
              {
                // запросили открыть на проценты
                cmd += '|';
                cmd += cePacket->commands[i].param2;
                cmd += "%";
              }
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;          

          case emCommandCloseWindow:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: close window!"));        
              #endif
              String cmd = F("STATE|WINDOW|");
              cmd += cePacket->commands[i].param1;
              cmd += F("|CLOSE");
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;

          case emCommandWaterOn:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Water on!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("WATER|ON"),false);          
          }
          break;

          case emCommandWaterSkip:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Water skip!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("WATER|SKIP"),false);          
            
          }
          break;

          case emCommandWaterOff:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Water off!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("WATER|OFF"),false);          
          }
          break;

          case emCommandWaterChannelOn:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: water channel on!"));        
              #endif
              String cmd = F("WATER|ON|");
              cmd += cePacket->commands[i].param1;
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;

          case emCommandWaterChannelOff:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: water channel off!"));        
              #endif
              String cmd = F("WATER|OFF|");
              cmd += cePacket->commands[i].param1;
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;

          case emCommandLightOn:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Light on!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("LIGHT|ON"),false);          
          }
          break; 

          case emCommandLigntOff:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: Light off!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("LIGHT|OFF"),false);          
          }
          break;

          case emCommandPinOn:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: pin on!"));        
              #endif
              String cmd = F("PIN|");
              cmd += cePacket->commands[i].param1;
              cmd += F("|ON");
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;            
          
          case emCommandPinOff:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: pin off!"));        
              #endif
              String cmd = F("PIN|");
              cmd += cePacket->commands[i].param1;
              cmd += F("|OFF");
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;

          case emCommandAutoMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: AUTO MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("0|AUTO"),false);               
          }
          break;

          case emCommandWindowsAutoMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: WINDOWS AUTO MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("STATE|MODE|AUTO"),false);               
          }
          break;          

          case emCommandWindowsManualMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: WINDOWS MANUAL MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("STATE|MODE|MANUAL"),false);               
          }
          break;

          case emCommandWaterAutoMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: WATER AUTO MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("WATER|MODE|AUTO"),false);               
          }
          break;          

          case emCommandWaterManualMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: WATER MANUAL MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("WATER|MODE|MANUAL"),false);               
          }
          break;

          case emCommandLightAutoMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: LIGHT AUTO MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("LIGHT|MODE|AUTO"),false);               
          }
          break;          

          case emCommandLightManualMode:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: LIGHT MANUAL MODE!"));        
              #endif
              ModuleInterop.QueryCommand(ctSET, F("LIGHT|MODE|MANUAL"),false);               
          }
          break;

          case emCommandSetOpenTemp:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: SET OPEN TEMP!"));        
              #endif
              GlobalSettings* sett = MainController->GetSettings();
              sett->SetOpenTemp(cePacket->commands[i].param1);              
          }
          break;

          case emCommandSetCloseTemp:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: SET CLOSE TEMP!"));        
              #endif
              GlobalSettings* sett = MainController->GetSettings();
              sett->SetCloseTemp(cePacket->commands[i].param1);              
          }
          break;               

          case emCommandSetMotorsInterval:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: SET MOTORS INTERVAL!"));        
              #endif
              GlobalSettings* sett = MainController->GetSettings();
              byte b[2]; b[0] = cePacket->commands[i].param1; b[1] = cePacket->commands[i].param2;
              uint16_t dt; memcpy(&dt,b,2);
              uint32_t oInterval = dt; oInterval *= 1000;              
              sett->SetOpenInterval(oInterval);              
          }
          break;

          case emCommandStartScene:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: start scene!"));        
              #endif
              String cmd = F("SCN|EXEC|");
              cmd += cePacket->commands[i].param1;
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;
          
          case emCommandStopScene:
          {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("RS485: stop scene!"));        
              #endif
              String cmd = F("SCN|STOP|");
              cmd += cePacket->commands[i].param1;
              ModuleInterop.QueryCommand(ctSET, cmd,false);          
          }
          break;
      } // switch
    
  } // for
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::writeToStream(const uint8_t* buffer, size_t len)
{
  if(!workStream)
  {
    return;
  }
  
   workStream->write(buffer,len);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::sendFillTankCommand(bool on)
{
  #ifdef USE_WATER_TANK_MODULE


      #ifdef WATER_TANK_MODULE_DEBUG
       Serial.print(F("RS-485: Send WATER TANK command packet with valve command: "));
       Serial.println(on);
      #endif // WATER_TANK_MODULE_DEBUG
  
   enableSend();
    
    RS485Packet packet;
    memset(&packet,0,sizeof(RS485Packet));
    
    packet.header1 = 0xAB;
    packet.header2 = 0xBA;
    packet.tail1 = 0xDE;
    packet.tail2 = 0xAD;

    packet.direction = RS485FromMaster;
    packet.type = RS485WaterTankCommands;

    RS485WaterTankCommandPacket* dest = (RS485WaterTankCommandPacket*) &(packet.data);
    dest->valveCommand = on ? 1 : 0;

    const byte* b = (const byte*) &packet;
    packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

    // пишем в шину RS-495 слепок состояния контроллера
    writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));

    // теперь ждём завершения передачи
    waitTransmitComplete();

/// Этой строчки не надо, поскольку на этот пакет мы не ждём ответа
 ///    enableReceive();
                
  #endif // USE_WATER_TANK_MODULE
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::sendWaterTankSettingsPacket()
{
  #ifdef USE_WATER_TANK_MODULE

        #ifdef WATER_TANK_MODULE_DEBUG
         Serial.println(F("RS-485: Send WATER TANK settings packet..."));
         #endif // WATER_TANK_MODULE_DEBUG  

   enableSend();
    
    RS485Packet packet;
    memset(&packet,0,sizeof(RS485Packet));
    
    packet.header1 = 0xAB;
    packet.header2 = 0xBA;
    packet.tail1 = 0xDE;
    packet.tail2 = 0xAD;

    packet.direction = RS485FromMaster;
    packet.type = RS485WaterTankSettings;

    WaterTankBinding bnd = HardwareBinding->GetWaterTankBinding();
    RS485WaterTankSettingsPacket* dest = (RS485WaterTankSettingsPacket*) &(packet.data);
    dest->level = bnd.Level; // ТУТ ИЗ НАСТРОЕК НАДО БРАТЬ УРОВЕНЬ СРАБАТЫВАНИЯ ДАТЧИКА !!!
    dest->maxWorkTime = bnd.MaxWorkTime;
    dest->distanceEmpty = bnd.DistanceEmpty;
    dest->distanceFull = bnd.DistanceFull;

    const byte* b = (const byte*) &packet;
    packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

    // пишем в шину RS-495 слепок состояния контроллера
    writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));

    // теперь ждём завершения передачи
    waitTransmitComplete();

/// Этой строчки не надо, поскольку на этот пакет мы не ждём ответа
 ///    enableReceive();
                  
  
  #endif // USE_WATER_TANK_MODULE
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::sendControllerStatePacket()
{
    
    enableSend();
    
    RS485Packet packet;
    memset(&packet,0,sizeof(RS485Packet));
    
    packet.header1 = 0xAB;
    packet.header2 = 0xBA;
    packet.tail1 = 0xDE;
    packet.tail2 = 0xAD;

    packet.direction = RS485FromMaster;
    packet.type = RS485ControllerStatePacket;
  
    void* dest = &(packet.data);
    ControllerState curState = WORK_STATUS.GetState();
    void* src = &curState;
    memcpy(dest,src,sizeof(ControllerState));

    const byte* b = (const byte*) &packet;
    packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

    // пишем в шину RS-495 слепок состояния контроллера
    writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));

    // теперь ждём завершения передачи
    waitTransmitComplete();

/// Этой строчки не надо, поскольку на этот пакет мы не ждём ответа
 ///    enableReceive();


    #ifdef USE_REMOTE_DISPLAY_MODULE

      sendRemoteDisplaySettings(); 
    
    #endif // USE_REMOTE_DISPLAY_MODULE
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_REMOTE_DISPLAY_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::sendRemoteDisplaySettings()
{
  // совсем немного подождём, и будем отправлять настройки контроллера
    delay(10);

    RS485Packet packet;
    memset(&packet,0,sizeof(RS485Packet));
    packet.header1 = 0xAB;
    packet.header2 = 0xBA;
    packet.tail1 = 0xDE;
    packet.tail2 = 0xAD;

    packet.direction = RS485FromMaster;
    packet.type = RS485SettingsForRemoteDisplay;

    // заполняем пакет данными
    RemoteDisplaySettingsPacket rds;
    memset(&rds,0,sizeof(RemoteDisplaySettingsPacket));

      GlobalSettings* sett = MainController->GetSettings();
      if(sett)
      {
        
        rds.openTemp = sett->GetOpenTemp();
        rds.closeTemp = sett->GetCloseTemp();
        rds.interval = sett->GetOpenInterval()/1000;
        rds.isWindowsOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
        rds.isWindowAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);      
        rds.isWaterOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
        rds.isWaterAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);      
        rds.isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
        rds.isLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
        
      }

      
      #if defined(USE_TEMP_SENSORS) && (SUPPORTED_WINDOWS > 0)
      if(WindowModule)
      {
        
        for(size_t k = 0; k < SUPPORTED_WINDOWS; k++)
        {
          if(WindowModule->IsWindowOpen(k))
            rds.windowsStatus |= (1 << k);
        }
        
      }
      #endif // USE_TEMP_SENSORS
      

      // и копируем данные в пакет
      void* pDest = packet.data;
      void* pSrc = &rds;
      memcpy(pDest,pSrc,sizeof(RemoteDisplaySettingsPacket));


    const uint8_t* bPtr = (const uint8_t*) &packet;
    packet.crc8 = crc8(bPtr,sizeof(RS485Packet)-1);


    enableSend();
    writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
    waitTransmitComplete();
/// Этой строчки не надо, поскольку на этот пакет мы не ждём ответа
///    enableReceive();  

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
static RemoteDisplaySensorSetting REMOTE_DISPLAY_SENSOR_SETTINGS[] = { REMOTE_DISPLAY_SENSORS };
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::collectSensorsForRemoteDisplay()
{
 //  Serial.println("collectSensorsForRemoteDisplay()");

  // тут собираем данные с датчиков для выносных дисплеев
  remoteDisplaySensors.clear();
  remoteDisplaySensorsHasDataFlags.clear();

  const uint8_t sensorsCount = sizeof(REMOTE_DISPLAY_SENSOR_SETTINGS)/sizeof(REMOTE_DISPLAY_SENSOR_SETTINGS[0]);
  
 // Serial.println(sensorsCount);
  
  for(uint8_t i=0;i<sensorsCount;i++)
  {
    AbstractModule* mod = MainController->GetModuleByID(REMOTE_DISPLAY_SENSOR_SETTINGS[i].moduleName);
    if(!mod)
    {
      continue;
    }

   // Serial.println(REMOTE_DISPLAY_SENSOR_SETTINGS[i].moduleName);

    OneState* os = mod->State.GetState((ModuleStates) REMOTE_DISPLAY_SENSOR_SETTINGS[i].type,REMOTE_DISPLAY_SENSOR_SETTINGS[i].index);
    bool hasData = false;
    RemoteDisplaySensorData rds;
    memset(&rds,0,sizeof(RemoteDisplaySensorData));
    rds.type = REMOTE_DISPLAY_SENSOR_SETTINGS[i].type;

    if(os)
    {
      hasData = os->HasData();
      if(hasData)
      {
        os->GetRawData(rds.data);
      }
    }

    remoteDisplaySensors.push_back(rds);
    remoteDisplaySensorsHasDataFlags.push_back(hasData);
      
  } // for
  // Serial.println("collectSensorsForRemoteDisplay() DONE.");

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_REMOTE_DISPLAY_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRS485Gate::Update()
{

  if(!workStream) // нет привязанного Serial, не стоит и работать
  {
    return;
  }

  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;

  static RS485Packet packet;
  RS485Binding rsBnd = HardwareBinding->GetRS485Binding();
  uint32_t readTimeout  = (10000000ul/SERIAL_BAUD_RATE)*rsBnd.BytesTimeout; // кол-во микросекунд, необходимое для вычитки N байт


  #if defined(USE_RS485_EXTERNAL_CONTROL_MODULE) || defined(USE_REMOTE_DISPLAY_MODULE)
  ///////////////////////////////////////////////////////////////////////
  // Опрашиваем модули управления или выносной экран
  ///////////////////////////////////////////////////////////////////////
  static uint16_t controlModuleTimer = 0;
  controlModuleTimer += dt;
  
  if(controlModuleTimer > 2000)
  {
      // пора опрашивать модули управления
    
      controlModuleTimer = 0;
      
      enableSend();
      
      #ifdef RS485_DEBUG
        DEBUG_LOGLN(F("Request information from control modules..."));        
      #endif

      memset(&packet,0,sizeof(RS485Packet));
      packet.header1 = 0xAB;
      packet.header2 = 0xBA;
      packet.tail1 = 0xDE;
      packet.tail2 = 0xAD;

      packet.direction = RS485FromMaster;
      packet.type = RS485RequestCommandsPacket;

      CommandsToExecutePacket* cePacket = (CommandsToExecutePacket*) &(packet.data);
      cePacket->moduleID = 0;

      const uint8_t* b = (const uint8_t*) &packet;
      packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

      writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
    
      // теперь ждём завершения передачи
      waitTransmitComplete();

      // начинаем принимать
      enableReceive();

      yield();

            memset(&packet,0,sizeof(RS485Packet));
            uint8_t* writePtr = (uint8_t*) &packet;
            uint8_t bytesReaded = 0; // кол-во прочитанных байт
            
            // запоминаем время начала чтения
            uint32_t startReadingTime = micros();

            // начинаем читать данные
            while(1)
            {
              if( micros() - startReadingTime > readTimeout)
              {
                
                #ifdef RS485_DEBUG
                  DEBUG_LOG(F("Control module #"));
                  DEBUG_LOG(String(0));
                  DEBUG_LOGLN(F(" not answering!"));
                #endif
                
                break;
              } // if
    
              if(workStream->available())
              {
                startReadingTime = micros(); // сбрасываем таймаут
                *writePtr++ = (uint8_t) workStream->read();
                bytesReaded++;
              } // if available

              if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet received from control module!"));
                #endif
                
                break;
              }
          
           } // while

            // затем опять переключаемся на передачу
            enableSend();

        // теперь парсим пакет
        if(bytesReaded == sizeof(RS485Packet))
        {
          bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
          bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
          
          if(headOk && tailOk)
          {
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("Header and tail ok."));
            #endif
         
            // вычисляем crc
            uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
            if(crc == packet.crc8)
            {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("Checksum ok."));
              #endif
              
              // теперь проверяем, нам ли пакет
              if(packet.direction == RS485FromSlave && packet.type == RS485RequestCommandsPacket)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet type ok, start analyze commands..."));
                #endif

                  executeCommands(packet);

                  // и посылаем квитанцию
                  packet.direction = RS485FromMaster;
                  packet.type = RS485CommandsToExecuteReceipt;
                  packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

                  writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
    
                   // теперь ждём завершения передачи
                  waitTransmitComplete();

                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Commands from control module executed."));
                #endif                  

              }
            } // if crc ok
            
          } // if(headOk && tailOk)
                
        } // if(bytesReaded == sizeof(RS485Packet))
      
    
  } // if
  ///////////////////////////////////////////////////////////////////////
  // Конец опроса модулей управления
  ///////////////////////////////////////////////////////////////////////
  #endif // USE_RS485_EXTERNAL_CONTROL_MODULE || USE_REMOTE_DISPLAY_MODULE

  ///////////////////////////////////////////////////////////////////////
  // начало опроса по данным ветра и дождя
  ///////////////////////////////////////////////////////////////////////

  // проверяем - есть ли привязки работы датчиков дождя и ветра через внешний модуль?
  WindSensorBinding bndWind = HardwareBinding->GetWindSensorBinding();
  RainSensorBinding bndRain = HardwareBinding->GetRainSensorBinding();
  
  if(bndRain.WorkMode == wrsExternalModule || bndWind.WorkMode == wrsExternalModule)
  {
        // есть привязки через выносной модуль, работаем    
      static uint16_t windRainTimer = 0;
      windRainTimer += dt;
      
      if(windRainTimer > WIND_RAIN_UPDATE_INTERVAL)
      {
          // пора опрашивать модуль датчиков скорости ветра и дождя
        
          windRainTimer = 0;
          
          enableSend();
          
          #ifdef RS485_DEBUG
            DEBUG_LOGLN(F("Request information about WIND and RAIN..."));        
          #endif
    
          memset(&packet,0,sizeof(RS485Packet));
          packet.header1 = 0xAB;
          packet.header2 = 0xBA;
          packet.tail1 = 0xDE;
          packet.tail2 = 0xAD;
    
          packet.direction = RS485FromMaster;
          packet.type = RS485WindRainData;
    
          const uint8_t* b = (const uint8_t*) &packet;
          packet.crc8 = crc8(b,sizeof(RS485Packet)-1);
    
          writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
        
          // теперь ждём завершения передачи
          waitTransmitComplete();
    
          // начинаем принимать
          enableReceive();
    
          yield();
    
                memset(&packet,0,sizeof(RS485Packet));
                uint8_t* writePtr = (uint8_t*) &packet;
                uint8_t bytesReaded = 0; // кол-во прочитанных байт
                
                // запоминаем время начала чтения
                uint32_t startReadingTime = micros();
    
                // начинаем читать данные
                while(1)
                {
                  if( micros() - startReadingTime > readTimeout)
                  {
                    
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("Wind and rain module not answering!"));
                    #endif
                    
                    break;
                  } // if
        
                  if(workStream->available())
                  {
                    startReadingTime = micros(); // сбрасываем таймаут
                    *writePtr++ = (uint8_t) workStream->read();
                    bytesReaded++;
                  } // if available
    
                  if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
                  {
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("Packet received from wind and rain module!"));
                    #endif
                    
                    break;
                  }
              
               } // while
    
                // затем опять переключаемся на передачу
                enableSend();
    
            // теперь парсим пакет
            if(bytesReaded == sizeof(RS485Packet))
            {
              bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
              bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
              
              if(headOk && tailOk)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Header and tail ok."));
                #endif
             
                // вычисляем crc
                uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
                if(crc == packet.crc8)
                {
                  #ifdef RS485_DEBUG
                    DEBUG_LOGLN(F("Checksum ok."));
                  #endif
                  
                  // теперь проверяем, нам ли пакет
                  if(packet.direction == RS485FromSlave && packet.type == RS485WindRainData)
                  {
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("Packet type ok, start analyze wind and rain data..."));
                    #endif
    
                    WindRainDataPacket* wrData = (WindRainDataPacket*) &(packet.data);                 
                    LogicManageModule->SetHasRain(wrData->hasRain);
                    LogicManageModule->SetWindSpeed(wrData->windSpeed);
                    LogicManageModule->SetWindDirection((CompassPoints)wrData->windDirection);
    
                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("HAS RAIN: "));
                      DEBUG_LOGLN(String(wrData->hasRain));
    
                      DEBUG_LOG(F("WIND SPEED: "));
                      DEBUG_LOGLN(String(wrData->windSpeed));
    
                      DEBUG_LOG(F("WIND DIRECTION: "));
                      DEBUG_LOGLN(String(wrData->windDirection));
                      
                      DEBUG_LOGLN(F("Wind and rain data received."));
                    #endif                  
    
                  }
                } // if crc ok
                
              } // if(headOk && tailOk)
                    
            } // if(bytesReaded == sizeof(RS485Packet))
                  
      } // if interval reached
      
  } // if(bndRain.WorkMode == wrsExternalModule || bndWind.WorkMode == wrsExternalModule)
  ///////////////////////////////////////////////////////////////////////
  // конец опроса по данным ветра и дождя
  ///////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////
  // начало опроса по данным модуля бака с водой
  ///////////////////////////////////////////////////////////////////////
  #ifdef USE_WATER_TANK_MODULE
  
      static uint16_t waterTankTimer = 0;
      waterTankTimer += dt;
      
      if(waterTankTimer > WATER_TANK_UPDATE_INTERVAL)
      {
          // пора опрашивать модуль датчиков скорости ветра и дождя
        
          waterTankTimer = 0;
          
          enableSend();
          
          #ifdef RS485_DEBUG
            DEBUG_LOGLN(F("Request information from WATER TANK module..."));        
          #endif
    
          memset(&packet,0,sizeof(RS485Packet));
          packet.header1 = 0xAB;
          packet.header2 = 0xBA;
          packet.tail1 = 0xDE;
          packet.tail2 = 0xAD;
    
          packet.direction = RS485FromMaster;
          packet.type = RS485WaterTankRequestData;
    
          const uint8_t* b = (const uint8_t*) &packet;
          packet.crc8 = crc8(b,sizeof(RS485Packet)-1);
    
          writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
        
          // теперь ждём завершения передачи
          waitTransmitComplete();
    
          // начинаем принимать
          enableReceive();
    
          yield();
    
                memset(&packet,0,sizeof(RS485Packet));
                uint8_t* writePtr = (uint8_t*) &packet;
                uint8_t bytesReaded = 0; // кол-во прочитанных байт
                
                // запоминаем время начала чтения
                uint32_t startReadingTime = micros();
    
                // начинаем читать данные
                while(1)
                {
                  if( micros() - startReadingTime > readTimeout)
                  {
                    
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("WATER TANK module not answering!"));
                    #endif
                    
                    break;
                  } // if
        
                  if(workStream->available())
                  {
                    startReadingTime = micros(); // сбрасываем таймаут
                    *writePtr++ = (uint8_t) workStream->read();
                    bytesReaded++;
                  } // if available
    
                  if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
                  {
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("Packet received from WATER TANK module!"));
                    #endif
                    
                    break;
                  }
              
               } // while
    
                // затем опять переключаемся на передачу
                enableSend();
    
            // теперь парсим пакет
            if(bytesReaded == sizeof(RS485Packet))
            {
              bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
              bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
              
              if(headOk && tailOk)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Header and tail ok."));
                #endif
             
                // вычисляем crc
                uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
                if(crc == packet.crc8)
                {
                  #ifdef RS485_DEBUG
                    DEBUG_LOGLN(F("Checksum ok."));
                  #endif
                  
                  // теперь проверяем, нам ли пакет
                  if(packet.direction == RS485FromSlave && packet.type == RS485WaterTankDataAnswer)
                  {
                    #ifdef RS485_DEBUG
                      DEBUG_LOGLN(F("Packet type ok, start analyze WATER TANK data..."));
                    #endif
    
                    WaterTankDataPacket* waterTankData = (WaterTankDataPacket*) &(packet.data);                 
    
                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("VALVE STATE: "));
                      DEBUG_LOGLN(String(waterTankData->valveState));
    
                      DEBUG_LOG(F("FILL STATUS: "));
                      DEBUG_LOGLN(String(waterTankData->fillStatus));
    
                      DEBUG_LOG(F("ERROR FLAG: "));
                      DEBUG_LOGLN(String(waterTankData->errorFlag));

                      DEBUG_LOG(F("ERROR TYPE: "));
                      DEBUG_LOGLN(String(waterTankData->errorType));
                      
                      DEBUG_LOGLN(F("WATER TANK data received."));
                    #endif                  

                   // ТУТ ОБНОВЛЕНИЕ ДАННЫХ В КОНТРОЛЛЕРЕ
                    WaterTank->UpdateState(waterTankData->valveState,waterTankData->fillStatus,waterTankData->errorFlag,waterTankData->errorType);

                  }
                } // if crc ok
                
              } // if(headOk && tailOk)
                    
            } // if(bytesReaded == sizeof(RS485Packet))
                  
      } // if interval reached
  #endif // USE_WATER_TANK_MODULE
  ///////////////////////////////////////////////////////////////////////
  // конец опроса по данным модуля бака с водой
  ///////////////////////////////////////////////////////////////////////


  ///////////////////////////////////////////////////////////////////////
  // начало опроса солнечной установки
  ///////////////////////////////////////////////////////////////////////
  #ifdef USE_UNI_SUN_CONTROLLER_MODULE
  static uint16_t sunControllerTimer = 0;
  sunControllerTimer += dt;
  
  if(sunControllerTimer > UNI_SUN_CONTROLLER_UPDATE_INTERVAL)
  {
      // пора опрашивать модуль солнечной установки
    
      sunControllerTimer = 0;
      
      enableSend();
      
      #ifdef RS485_DEBUG
        DEBUG_LOGLN(F("Request information fron sun controller..."));        
      #endif

      memset(&packet,0,sizeof(RS485Packet));
      packet.header1 = 0xAB;
      packet.header2 = 0xBA;
      packet.tail1 = 0xDE;
      packet.tail2 = 0xAD;

      packet.direction = RS485FromMaster;
      packet.type = RS485SunControllerData;

      const uint8_t* b = (const uint8_t*) &packet;
      packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

      writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
    
      // теперь ждём завершения передачи
      waitTransmitComplete();

      // начинаем принимать
      enableReceive();

      yield();

            memset(&packet,0,sizeof(RS485Packet));
            uint8_t* writePtr = (uint8_t*) &packet;
            uint8_t bytesReaded = 0; // кол-во прочитанных байт
            
            // запоминаем время начала чтения
            uint32_t startReadingTime = micros();

            // начинаем читать данные
            while(1)
            {
              if( micros() - startReadingTime > readTimeout)
              {
                
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Sun controller module not answering!"));
                #endif
                
                break;
              } // if
    
              if(workStream->available())
              {
                startReadingTime = micros(); // сбрасываем таймаут
                *writePtr++ = (uint8_t) workStream->read();
                bytesReaded++;
              } // if available

              if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet received from sun controller module!"));
                #endif
                
                break;
              }
          
           } // while

            // затем опять переключаемся на передачу
            enableSend();

        // теперь парсим пакет
        if(bytesReaded == sizeof(RS485Packet))
        {
          bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
          bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
          
          if(headOk && tailOk)
          {
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("Header and tail ok."));
            #endif
         
            // вычисляем crc
            uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
            if(crc == packet.crc8)
            {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("Checksum ok."));
              #endif
              
              // теперь проверяем, нам ли пакет
              if(packet.direction == RS485FromSlave && packet.type == RS485SunControllerData)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet type ok, start analyze sun controller data..."));
                #endif

                SunControllerDataPacket* sunData = (SunControllerDataPacket*) &(packet.data);                 

                //TODO: ТУТ ОТПРАВЛЯЕМ НАСТРОЙКИ, КУДА НАДО !!!

                #ifdef RS485_DEBUG
                 
                  DEBUG_LOG(F("T1: "));
                  DEBUG_LOGLN(String(sunData->T1whole) + String(".") + String(sunData->T1fract));
          
                  DEBUG_LOG(F("T2: "));
                  DEBUG_LOGLN(String(sunData->T2whole) + String(".") + String(sunData->T2fract));
          
                  DEBUG_LOG(F("T3: "));
                  DEBUG_LOGLN(String(sunData->T3whole) + String(".") + String(sunData->T3fract));
          
                  DEBUG_LOG(F("T4: "));
                  DEBUG_LOGLN(String(sunData->T4whole) + String(".") + String(sunData->T4fract));
          
                  DEBUG_LOG(F("T5: "));
                  DEBUG_LOGLN(String(sunData->T5whole) + String(".") + String(sunData->T5fract));
          
                  DEBUG_LOG(F("Angle: "));
                  DEBUG_LOGLN(String(sunData->angle));
          
                  DEBUG_LOG(F("Direction: "));
                  DEBUG_LOGLN(String(sunData->direction));
          
                  DEBUG_LOG(F("Luminosity: "));
                  DEBUG_LOGLN(String(sunData->luminosity));

                  DEBUG_LOG(F("Heading: "));
                  DEBUG_LOGLN(String(sunData->heading));
                  
                  DEBUG_LOGLN(F("Sun controller data received."));
                #endif                  

              }
            } // if crc ok
            
          } // if(headOk && tailOk)
                
        } // if(bytesReaded == sizeof(RS485Packet))
      
    
  } // if
  #endif // USE_UNI_SUN_CONTROLLER_MODULE
  ///////////////////////////////////////////////////////////////////////
  // конец опроса по данным контроллера солнечной установки
  ///////////////////////////////////////////////////////////////////////


  
  ///////////////////////////////////////////////////////////////////////
  // Посылаем данные датчиков для выносных дисплеев
  ///////////////////////////////////////////////////////////////////////
  #ifdef USE_REMOTE_DISPLAY_MODULE
  static uint16_t remoteDisplayTimer = 0;
  remoteDisplayTimer += dt;
  
  if(remoteDisplayTimer > REMOTE_DISPLAY_UPDATE_INTERVAL)
  {
    remoteDisplayTimer = 0;
   // Serial.println("Collect sensors for remote display...");
    
    // тут надо упаковать показания датчиков в один пакет 30 байт, причём делать это циклически.
    // у нас показания одного датчика - максимум 4 байта, т.е. в пакете поместится 4 датчика,
    // из расчёта информации "1 байт - тип датчика, 4 байта - его показания". Т.е. на 4
    // датчика у нас займётся 20 байт, остаётся три байта на флаги: признаки того, с каких датчиков
    // в пакете есть показания, флаг, что пакет - последний.

    // сначала проверяем, есть ли данные в массиве
    // если нет - выставляем флаг, что это будет первый пакет, и собираем данные с датчиков.
    // потом формируем пакет максимум из 4 датчиков.
    // если после формирования пакета в массиве не осталось данных - выставляем флаг, что пакет - последний.
    // потом отправляем пакет.

    memset(&packet,0,sizeof(RS485Packet));
    packet.header1 = 0xAB;
    packet.header2 = 0xBA;
    packet.tail1 = 0xDE;
    packet.tail2 = 0xAD;

    packet.direction = RS485FromMaster;
    packet.type = RS485SensorDataForRemoteDisplay;

    RemoteDisplaySensorsPacket* data = (RemoteDisplaySensorsPacket*) &(packet.data);

    if(!remoteDisplaySensors.size())
    {
       //Serial.println("no data for display, collect it...");

      // нет данных с датчиков, надо выставить, что это первый пакет
      data->firstOrLastPacket = REMOTE_DISPLAY_FIRST_SENSORS_PACKET;
      // и собрать показания
      collectSensorsForRemoteDisplay();
    }

    // тут заполняем пакет данными. метод remove вектора написан криво, и корректно работает
    // только для вектора байт, поэтому мы должны корректно сформировать пакет для отправки,
    // правильно удалив отправленные датчики из очереди.
    uint8_t stopIndex = min(4,remoteDisplaySensors.size());
    RemoteDisplaySensors toSend, remaining;
    Vector<bool> flags, remainingFlags;
    
    for(uint8_t kk=0;kk<stopIndex;kk++)
    {
      toSend.push_back(remoteDisplaySensors[kk]);
      flags.push_back(remoteDisplaySensorsHasDataFlags[kk]);
    }

    for(uint8_t kk=stopIndex;kk<remoteDisplaySensors.size();kk++)
    {
      remaining.push_back(remoteDisplaySensors[kk]);
      remainingFlags.push_back(remoteDisplaySensorsHasDataFlags[kk]);      
    }

    remoteDisplaySensors = remaining;
    remoteDisplaySensorsHasDataFlags = remainingFlags;

    for(uint8_t kk=0;kk<toSend.size();kk++)
    {
        data->sensorsInPacket++;

        RemoteDisplaySensorData thisDt = toSend[kk];
        bool thisHasData = flags[kk];

        if(thisHasData)
        {
          data->hasDataFlags |= (1 << kk);
          memcpy(&(data->data[kk]),&thisDt,sizeof(RemoteDisplaySensorData));
        }
        else
          data->data[kk].type = thisDt.type;
    } // for      


    //Serial.println(data->sensorsInPacket);

    // тут заполнили пакет, надо глянуть - последний ли он был?
    if(!remoteDisplaySensors.size())
    {
      data->firstOrLastPacket |= REMOTE_DISPLAY_LAST_SENSORS_PACKET; // битовой маской, потому что пакет может быть первым и последним
    }

    // контрольная сумма
    const uint8_t* b = (const uint8_t*) &packet;
    packet.crc8 = crc8(b,sizeof(RS485Packet)-1);

      // Serial.println("send packet...");

    // отправляем пакет
    enableSend();

    writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));

    // теперь ждём завершения передачи
    waitTransmitComplete();

/// Этой строчки не надо, поскольку на этот пакет мы не ждём ответа
///     enableReceive();    

   //    Serial.println("packet was sent.");

    
  } // if
  
  #endif // USE_REMOTE_DISPLAY_MODULE
  ///////////////////////////////////////////////////////////////////////
  // Конец отсыла датчиков для выносных дисплеев
  ///////////////////////////////////////////////////////////////////////

  
  bool controllerStateWasSentOnThisIteration = false;

  #if defined(USE_FEEDBACK_MANAGER) && defined(USE_TEMP_SENSORS) && (SUPPORTED_WINDOWS > 0)
  
    // тут работаем с модулями обратной связи
    
    static uint16_t feedbackCounter = 1000;
    feedbackCounter += dt;
    
    if(feedbackCounter > FEEDBACK_MANAGER_UPDATE_INTERVAL)
    {
      // здесь нам надо сперва послать пакет с состоянием контроллера, по-любому!
      // это связано с тем, что может быть рассинхрон по времени опроса состояний, например:
      // попросили закрыть окно, при этом концевик открытия у модуля - сработал.
      // мы при выполнении команды выставляем статус, что окно открывается.
      // и если в этот момент, ДО отсыла состояния контроллера, мы сперва получим от модуля обратную связь,
      // то в ней будет состояние "Окно открыто". Следовательно, внутреннее состояние контроллера изменится,
      // и запрошенной команды к модулю - не уйдёт. Поэтому ВСЕГДА перед опросом модулей обратной связи
      // мы должны посылать им актуальное состояние контроллера!
       sendControllerStatePacket();
       controllerStateWasSentOnThisIteration = true;
      
      feedbackCounter = 0;
      // пора собирать информацию по обратной связи
      uint8_t currentWindowNumber = 0; // с каким окном сейчас работаем
      bool anyFeedbackReceived = false;
      int maxFeedbacksQueries = min(SUPPORTED_WINDOWS,16);
      
        for(int i=0;i<maxFeedbacksQueries;i++) // у нас максимум 16 адресов на шине
        {
            memset(&packet,0,sizeof(RS485Packet));
            
            packet.header1 = 0xAB;
            packet.header2 = 0xBA;
            packet.tail1 = 0xDE;
            packet.tail2 = 0xAD;
    
            packet.direction = RS485FromMaster;
            packet.type = RS485WindowsPositionPacket;

            // говорим, что мы хотим получить информацию с модуля определённого номера
            WindowFeedbackPacket* wfPacket = (WindowFeedbackPacket*) &(packet.data);
            wfPacket->moduleNumber = i;

              #ifdef RS485_DEBUG
      
              // отладочная информация
              DEBUG_LOG(F("Send query for feedback packet #"));
              DEBUG_LOGLN(String(wfPacket->moduleNumber));
                    
              #endif

            // считаем контрольную сумму
            const uint8_t* b = (const uint8_t*) &packet;
            packet.crc8 = crc8(b,sizeof(RS485Packet)-1);  

            enableSend();
            
           // посылаем пакет   
           // пишем в шину RS-495 запрос об обратной связи
            writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
    
            // теперь ждём завершения передачи
            waitTransmitComplete();

            // начинаем принимать
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            // переключаемся на приём
            enableReceive();

            // поскольку мы сразу же переключились на приём - можем дать поработать критичному ко времени коду
            yield();

            // и получаем наши байты
            memset(&packet,0,sizeof(RS485Packet));
            uint8_t* writePtr = (uint8_t*) &packet;
            uint8_t bytesReaded = 0; // кол-во прочитанных байт
            
            // запоминаем время начала чтения
            uint32_t startReadingTime = micros();

            // начинаем читать данные
            while(1)
            {
              if( micros() - startReadingTime > readTimeout)
              {
                
                #ifdef RS485_DEBUG
                  DEBUG_LOG(F("Feedback module #"));
                  DEBUG_LOG(String(i));
                  DEBUG_LOGLN(F(" not answering!"));
                #endif
                
                break;
              } // if
    
              if(workStream->available())
              {
                startReadingTime = micros(); // сбрасываем таймаут
                *writePtr++ = (uint8_t) workStream->read();
                bytesReaded++;
              } // if available

              if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet received from slave!"));
                #endif
                
                break;
              }
          
           } // while

            // затем опять переключаемся на передачу
            enableSend();

        // теперь парсим пакет
        if(bytesReaded == sizeof(RS485Packet))
        {
          // пакет получен полностью, парсим его
          #ifdef RS485_DEBUG
            DEBUG_LOGLN(F("Packet from feedback received, parse it..."));
          #endif
          
          bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
          bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
          
          if(headOk && tailOk)
          {
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("Header and tail ok."));
            #endif
            
            // вычисляем crc
            uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
            if(crc == packet.crc8)
            {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("Checksum ok."));
              #endif
              
              // теперь проверяем, нам ли пакет
              if(packet.direction == RS485FromSlave && packet.type == RS485WindowsPositionPacket)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet type ok"));
                #endif

                anyFeedbackReceived = true; // получили фидбак по крайней мере от одного модуля

                // тут пришли данные, надо разбирать
                WindowFeedbackPacket* wfPacket = (WindowFeedbackPacket*) &(packet.data);
                int moduleSupportedWindows = wfPacket->windowsSupported;
                // теперь разбираем, что там в пакете
                uint8_t* windowsStatus = wfPacket->windowsStatus;

                // проходим по всем поддерживаемым модулем окнам
                uint8_t currentByteNumber = 0;
                int8_t currentBitNumber = 7;
                
                for(int k=0;k<moduleSupportedWindows;k++)
                {
                  // на каждое окно у нас 10 бит информации
                  // в старших семи битах - информация о позиции
                  // в третьем бите - флаг наличия информации о позиции
                  // второй бит - сработал ли концевик закрытия
                  // первый бит - сработал ли концевик открытия
                  uint8_t position = 0;
                  for(int z=0;z<7;z++)
                  {
                    uint8_t b = bitRead(windowsStatus[currentByteNumber],currentBitNumber);
                    position |= b;
                    position <<= 1;

                    currentBitNumber--;
                    if(currentBitNumber < 0)
                    {
                      currentBitNumber = 7;
                      currentByteNumber++;
                    }
                  } // for

                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("Position of window #"));
                      DEBUG_LOG(String(currentWindowNumber));
                      DEBUG_LOG(F(" is "));
                      DEBUG_LOGLN(String(position));
                    #endif

                    // теперь читаем бит - есть ли позиция
                    uint8_t hasPosition = bitRead(windowsStatus[currentByteNumber],currentBitNumber);
                    currentBitNumber--;
                    if(currentBitNumber < 0)
                    {
                      currentBitNumber = 7;
                      currentByteNumber++;
                    }

                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("hasPosition of window #"));
                      DEBUG_LOG(String(currentWindowNumber));
                      DEBUG_LOG(F(" is "));
                      DEBUG_LOGLN(String(hasPosition));
                    #endif
                                        
                    // теперь читаем бит - сработал ли концевик закрытия
                    uint8_t isCloseSwitchTriggered = bitRead(windowsStatus[currentByteNumber],currentBitNumber);
                    currentBitNumber--;
                    if(currentBitNumber < 0)
                    {
                      currentBitNumber = 7;
                      currentByteNumber++;
                    }

                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("isCloseSensorTriggered of window #"));
                      DEBUG_LOG(String(currentWindowNumber));
                      DEBUG_LOG(F(" is "));
                      DEBUG_LOGLN(String(isCloseSwitchTriggered));
                    #endif

                    // теперь читаем бит - сработал ли концевик открытия
                    uint8_t isOpenSwitchTriggered = bitRead(windowsStatus[currentByteNumber],currentBitNumber);
                    currentBitNumber--;
                    if(currentBitNumber < 0)
                    {
                      currentBitNumber = 7;
                      currentByteNumber++;
                    }

                    #ifdef RS485_DEBUG
                      DEBUG_LOG(F("isOpenSwitchTriggered of window #"));
                      DEBUG_LOG(String(currentWindowNumber));
                      DEBUG_LOG(F(" is "));
                      DEBUG_LOGLN(String(isOpenSwitchTriggered));
                    #endif

                   // теперь просим менеджера сообщить окну информацию о позиции
                   FeedbackManager.WindowFeedback(currentWindowNumber, isCloseSwitchTriggered, isOpenSwitchTriggered, hasPosition, position);

                  currentWindowNumber++;
                  if(currentWindowNumber >= SUPPORTED_WINDOWS) // всё, дошли до последнего окна
                  {
                    break;
                  }
                } // for


              } // type ok
              #ifdef RS485_DEBUG
              else
              {
                DEBUG_LOGLN(F("Wrong packet type :("));
              }
              #endif
            } // if(crc)
            #ifdef RS485_DEBUG
            else
            {
              DEBUG_LOGLN(F("Bad checksum :("));
            }
            #endif
          }
          #ifdef RS485_DEBUG
          else
          {
            DEBUG_LOGLN(F("Head or tail of packet is invalid :("));
          } // else
          #endif
        }
        #ifdef RS485_DEBUG
        else
        {
          DEBUG_LOGLN(F("Uncompleted feedback packet :("));
        } // else
        #endif
            ////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
           yield(); // даём поработать модулям 
        } // for по каждому модулю

        if(anyFeedbackReceived)
        {
           // получили хотя бы один фидбак - надо проинформировать менеджера, что мы закончили текущий цикл
           FeedbackManager.WindowFeedbackDone();
        }
    } // if можно проверять
    
  #endif // USE_FEEDBACK_MANAGER
  
  #ifdef USE_UNI_EXECUTION_MODULE

  // посылаем в шину данные для исполнительных модулей
  
    updateTimer += dt;
    if(updateTimer >= rsBnd.StatePushFrequency)
    {
      updateTimer = 0;

      // тут посылаем слепок состояния контроллера
       if(!controllerStateWasSentOnThisIteration)
       {
        sendControllerStatePacket();
       }

        
    }
  #endif // USE_UNI_EXECUTION_MODULE

  #ifdef USE_UNIVERSAL_MODULES


   static bool _is_inited = false;
   if(!_is_inited)
   {
      _is_inited = true;
      // инициализируем очередь
       for(uint8_t sensorType=uniTemp;sensorType<=uniPinsMap;sensorType++)
       {
         uint8_t cnt = UniDispatcher.GetUniSensorsCount((UniSensorType) sensorType);
    
          for(uint8_t k=0;k<cnt;k++)
          {
            RS485QueueItem qi;
            qi.sensorType = sensorType;
            qi.sensorIndex = k;
            qi.badReadingAttempts = 0;
            queue.push_back(qi);
          } // for
          
       } // for
    
       currentQueuePos = 0;
       sensorsTimer = 0;      
    
   } // if
  

    sensorsTimer += dt;
    if(sensorsTimer >= rsBnd.OneSensorUpdateInterval)
    {
      sensorsTimer = 0;

      // настало время опроса датчиков на шине
      if(queue.size())
      {
        // есть очередь для опроса
        RS485QueueItem* qi = &(queue[currentQueuePos]);
        currentQueuePos++;

        // мы не можем обновлять состояние датчика в дефолтные значения здесь, поскольку
        // мы не знаем, откуда с него могут придти данные. В случае с работой через 1-Wire
        // состояние автоматически обновляется, поскольку считается, что если модуль есть
        // на линии - с него будут данные. У нас же ситуация обстоит по-другому:
        // мы проходим все зарегистрированные универсальные датчики, и не можем
        // делать вывод - висит ли модуль с датчиком на линии RS-485, или работает по радиоканалу,
        // или - работает по 1-Wire. Поэтому мы не вправе делать никаких предположений и менять
        // показания датчика на вид <нет данных>, поскольку очерёдность вызовов опроса
        // универсальных модулей по разным шлюзам не определена. 
        // поэтому мы сбрасываем состояния только тех датчиков, которые хотя бы однажды
        // откликнулись по шине RS-495.

        if(isInOnlineQueue(*qi) && qi->badReadingAttempts >= rsBnd.ResetSensorBadReadings)
        {
          uint8_t sType = qi->sensorType;
          uint8_t sIndex = qi->sensorIndex;
          // датчик был онлайн, сбрасываем его показания в "нет данных" перед опросом
          UniDispatcher.AddUniSensor((UniSensorType)sType,sIndex);
          qi->badReadingAttempts = 0;

                    // проверяем тип датчика, которому надо выставить "нет данных"
                    switch(qi->sensorType)
                    {
                      case uniTemp:
                      {
                        // температура
                        Temperature t;
                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            states.State1->Update(&t);
                          }
                        } // if
                      }
                      break;

                      case uniHumidity:
                      {
                        // влажность
                        Humidity h;
                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            states.State1->Update(&h);
                          }
                                              
                          if(states.State2)
                          {
                            states.State2->Update(&h);
                          }
                        } // if                        
                      }
                      break;

                      case uniLuminosity:
                      {
                        // освещённость
                        long lum = NO_LUMINOSITY_DATA;
                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            states.State1->Update(&lum);
                          }
                        } // if                        
                        
                        
                      }
                      break;

                      case uniSoilMoisture: // влажность почвы
                      case uniPH: // показания pH
                      {
                        
                        Humidity h;
                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            states.State1->Update(&h);
                          }
                        } // if                        
                        
                      }
                      break;

                      case uniPinsMap:
                      break;
                      
                    } // switch
          
        } // if in online queue
        
        
        if(currentQueuePos >= queue.size()) // достигли конца очереди, начинаем сначала
        {
          currentQueuePos = 0;
        }


        memset(&packet,0,sizeof(RS485Packet)); 
        packet.header1 = 0xAB;
        packet.header2 = 0xBA;
        packet.tail1 = 0xDE;
        packet.tail2 = 0xAD;

        packet.direction = RS485FromMaster; // направление - от нас ведомым
        packet.type = RS485SensorDataPacket; // это пакет - запрос на показания с датчиков

        uint8_t* dest = packet.data;
        // в первом байте - тип датчика для опроса
        *dest = qi->sensorType;
        dest++;
        // во втором байте - индекс датчика, зарегистрированный в системе
        *dest = qi->sensorIndex;

        // считаем контрольную сумму
        const uint8_t* b = (const uint8_t*) &packet;
        packet.crc8 = crc8(b,sizeof(RS485Packet)-1);


        #ifdef RS485_DEBUG

        // отладочная информация
        DEBUG_LOG(F("Request data for sensor type="));
        DEBUG_LOG(String(qi->sensorType));
        DEBUG_LOG(F(" and index="));
        DEBUG_LOGLN(String(qi->sensorIndex));

        #endif

        enableSend();
        // пакет готов к отправке, отправляем его
        writeToStream((const uint8_t *)&packet,sizeof(RS485Packet));
        
        waitTransmitComplete(); // ждём окончания посыла
        // теперь переключаемся на приём
        enableReceive();

        // поскольку мы сразу же переключились на приём - можем дать поработать критичному ко времени коду
        yield();

        // и получаем наши байты
        memset(&packet,0,sizeof(RS485Packet));
        uint8_t* writePtr = (uint8_t*) &packet;
        uint8_t bytesReaded = 0; // кол-во прочитанных байт
        // запоминаем время начала чтения
        uint32_t startReadingTime = micros();
        
        // начинаем читать данные
        while(1)
        {
          if( micros() - startReadingTime > readTimeout)
          {

            qi->badReadingAttempts++; // поймали таймаут, увеличиваем кол-во неудачных попыток чтения
            
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("TIMEOUT REACHED!!!"));
            #endif
            
            break;
          } // if

          if(workStream->available())
          {
            startReadingTime = micros(); // сбрасываем таймаут
            *writePtr++ = (uint8_t) workStream->read();
            bytesReaded++;
          } // if available

          if(bytesReaded == sizeof(RS485Packet)) // прочитали весь пакет
          {
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("Packet received from slave!"));
            #endif
            
            break;
          }
          
        } // while

        // затем опять переключаемся на передачу
        enableSend();

        // теперь парсим пакет
        if(bytesReaded == sizeof(RS485Packet))
        {
          // пакет получен полностью, парсим его
          #ifdef RS485_DEBUG
            DEBUG_LOGLN(F("Packet from slave received, parse it..."));
          #endif
          
          bool headOk = packet.header1 == 0xAB && packet.header2 == 0xBA;
          bool tailOk = packet.tail1 == 0xDE && packet.tail2 == 0xAD;
          if(headOk && tailOk)
          {
            #ifdef RS485_DEBUG
              DEBUG_LOGLN(F("Header and tail ok."));
            #endif
            
            // вычисляем crc
            uint8_t crc = crc8((const uint8_t*)&packet,sizeof(RS485Packet)-1);
            if(crc == packet.crc8)
            {
              #ifdef RS485_DEBUG
                DEBUG_LOGLN(F("Checksum ok."));
              #endif
              
              // теперь проверяем, нам ли пакет
              if(packet.direction == RS485FromSlave && packet.type == RS485SensorDataPacket)
              {
                #ifdef RS485_DEBUG
                  DEBUG_LOGLN(F("Packet type ok"));
                #endif

                uint8_t* readDataPtr = packet.data;
                // проверяем - байт типа и байт индекса должны совпадать с посланными в шину
                uint8_t sType = *readDataPtr++;
                uint8_t sIndex = *readDataPtr++;
                
                if(sType == qi->sensorType && sIndex == qi->sensorIndex)
                {
                  #ifdef RS485_DEBUG
                    DEBUG_LOGLN(F("Reading sensor data..."));
                  #endif

                  // добавляем наш тип сенсора в систему, если этого ещё не сделано
                  UniDispatcher.AddUniSensor((UniSensorType)sType,sIndex);

                  // добавляем датчик в список онлайн-датчиков
                  if(!isInOnlineQueue(*qi))
                  {
                    sensorsOnlineQueue.push_back(*qi);
                  }

                  // сбрасываем кол-во неудачных попыток чтения
                  qi->badReadingAttempts = 0;

                    // проверяем тип датчика, с которого читали показания
                    switch(sType)
                    {
                      case uniPinsMap:
                      {
                        ProcessPinsMap(sIndex,readDataPtr);
                      }
                      break;
                      
                      case uniTemp:
                      {
                        // температура
                        // получаем данные температуры
                        Temperature t;
                        t.Value = (int8_t) *readDataPtr++;
                        t.Fract = *readDataPtr;

                        // convert to Fahrenheit if needed
                        #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
                         t = Temperature::ConvertToFahrenheit(t);
                        #endif                              

                        #ifdef RS485_DEBUG
                          DEBUG_LOG(F("Temperature: "));
                          DEBUG_LOGLN(t);
                        #endif

                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            #ifdef RS485_DEBUG
                              DEBUG_LOGLN(F("Update data in controller..."));
                            #endif
                            
                            states.State1->Update(&t);
                          }
                        } // if
                      }
                      break;

                      case uniHumidity:
                      {
                        // влажность
                        Humidity h;
                        h.Value = (int8_t) *readDataPtr++;
                        h.Fract = *readDataPtr++;

                        // температура
                        Temperature t;
                        t.Value = (int8_t) *readDataPtr++;
                        t.Fract = *readDataPtr++;

                        // convert to Fahrenheit if needed
                        #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
                         t = Temperature::ConvertToFahrenheit(t);
                        #endif                              

                        #ifdef RS485_DEBUG
                          DEBUG_LOG(F("Humidity: "));
                          DEBUG_LOGLN(h);
                        #endif

                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                            #ifdef RS485_DEBUG
                              DEBUG_LOGLN(F("Update data in controller..."));
                            #endif

                          if(states.State1)
                          {
                            states.State1->Update(&h);
                          }

                          if(states.State2)
                          {
                            states.State2->Update(&t);
                          }
                            
                        } // if                        
                      }
                      break;

                      case uniLuminosity:
                      {
                        // освещённость
                        int32_t lum;
                        memcpy(&lum,readDataPtr,sizeof(int32_t));

                        #ifdef RS485_DEBUG
                          DEBUG_LOG(F("Luminosity: "));
                          DEBUG_LOGLN(String(lum));
                        #endif

                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            #ifdef RS485_DEBUG
                              DEBUG_LOGLN(F("Update data in controller..."));
                            #endif
                            
                            states.State1->Update(&lum);
                          }
                        } // if                        
                        
                        
                      }
                      break;

                      case uniSoilMoisture: // влажность почвы
                      case uniPH:  // показания pH
                      {
                        
                        Humidity h;
                        h.Value = (int8_t) *readDataPtr++;
                        h.Fract = *readDataPtr;

                        #ifdef RS485_DEBUG
                          if(sType == uniSoilMoisture)
                            DEBUG_LOG(F("Soil moisture: "));
                          else
                            DEBUG_LOG(F("pH: "));
                            
                          DEBUG_LOGLN(h);
                        #endif

                        // получаем состояния
                        UniSensorState states;
                        if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                        {
                          if(states.State1)
                          {
                            #ifdef RS485_DEBUG
                              DEBUG_LOGLN(F("Update data in controller..."));
                            #endif
                            
                            states.State1->Update(&h);
                          }
                        } // if                        
                        
                      }
                      break;
                      
                    } // switch
                }
                #ifdef RS485_DEBUG
                else
                {
                  DEBUG_LOGLN(F("Received data from unknown sensor :("));
                }
                #endif
              }
              #ifdef RS485_DEBUG
              else
              {
                DEBUG_LOGLN(F("Wrong packet type :("));
              }
              #endif
            }
            #ifdef RS485_DEBUG
            else
            {
              DEBUG_LOGLN(F("Bad checksum :("));
            }
            #endif
          }
          #ifdef RS485_DEBUG
          else
          {
            DEBUG_LOGLN(F("Head or tail of packet is invalid :("));
          } // else
          #endif
        }
        #ifdef RS485_DEBUG
        else
        {
          DEBUG_LOGLN(F("Received uncompleted packet :("));
        } // else
        #endif
          
        
      } // if(queue.size())
      /*
      #ifdef RS485_DEBUG
      else
      {
        DEBUG_LOGLN(F("No queue size :("));
      }
      #endif
      */
        
      
    } // if(sensorsTimer > _upd_interval)
    
  #endif // USE_UNIVERSAL_MODULES
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniRS485Gate RS485;
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_RS485_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// UniClientsFactory
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniClientsFactory::UniClientsFactory()
{
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
AbstractUniClient* UniClientsFactory::GetClient(UniRawScratchpad* scratchpad)
{
  if(!scratchpad)
    return &dummyClient;

  UniClientType ct = (UniClientType) scratchpad->head.packet_type;
  
  switch(ct)
  {
    case uniSensorsClient:
      return &sensorsClient;

    case uniWindRainClient:
      return &windRainClient;

    case uniSunControllerClient:
      return &sunControllerClient;

    case uniNextionClient:     
      break;

    case uniExecutionClient:
    #ifdef USE_UNI_EXECUTION_MODULE
      return &executionClient;
    #else
      break;
    #endif

    case uniWaterTankClient:
    #ifdef USE_WATER_TANK_MODULE
      return &waterTankClient;
    #else
      break;
    #endif
    
  }

  return &dummyClient;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_UNI_EXECUTION_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniExecutionModuleClient::UniExecutionModuleClient()
{
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniExecutionModuleClient::Register(UniRawScratchpad* scratchpad)
{
  // нам регистрироваться в системе дополнительно не надо
  UNUSED(scratchpad);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniExecutionModuleClient::Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough)
{
  if(!isModuleOnline) // когда модуль офлайн - ничего делать не надо
    return;

   // приводим к типу нашего скратча
   UniExecutionModuleScratchpad* ourScratch = (UniExecutionModuleScratchpad*) &(scratchpad->data);

   // получаем состояние контроллера
   ControllerState state = WORK_STATUS.GetState();

   // теперь проходимся по всем слотам
   for(byte i=0;i<8;i++)
   {
      byte slotStatus = 0; // статус слота - 0 по умолчанию
      
      switch(ourScratch->slots[i].slotType)
      {
        case slotEmpty: // пустой слот, ничего делать не надо
        case 0xFF: // если вычитали из EEPROM, а там ничего не было
        break;

        case slotWindowLeftChannel:
        {
          // состояние левого канала окна, в slotLinkedData - номер окна
          byte windowNumber = ourScratch->slots[i].slotLinkedData;
          if(windowNumber < 16)
          {
            // окна у нас нумеруются от 0 до 15, всего 16 окон.
            // на каждое окно - два бита, для левого и правого канала.
            // следовательно, чтобы получить стартовый бит - надо номер окна
            // умножить на 2.
            byte bitNum = windowNumber*2;           
            if(state.WindowsState & (1 << bitNum))
              slotStatus = 1; // выставляем в слоте значение 1
          }
        }
        break;

        case slotWindowRightChannel:
        {
          // состояние левого канала окна, в slotLinkedData - номер окна
          byte windowNumber = ourScratch->slots[i].slotLinkedData;
          if(windowNumber < 16)
          {
            // окна у нас нумеруются от 0 до 15, всего 16 окон.
            // на каждое окно - два бита, для левого и правого канала.
            // следовательно, чтобы получить стартовый бит - надо номер окна
            // умножить на 2.
            byte bitNum = windowNumber*2;

            // поскольку канал у нас правый - его бит идёт следом за левым.
            bitNum++;
                       
            if(state.WindowsState & (1 << bitNum))
              slotStatus = 1; // выставляем в слоте значение 1
          }
        }
        break;

        case slotWateringChannel:
        {
          // состояние канала полива, в slotLinkedData - номер канала полива
          byte wateringChannel = ourScratch->slots[i].slotLinkedData;
          if(wateringChannel< 16)
          {
            if(state.WaterChannelsState & (1 << wateringChannel))
              slotStatus = 1; // выставляем в слоте значение 1
              
          }
        }        
        break;

        case slotLightChannel:
        {
          // состояние канала досветки, в slotLinkedData - номер канала досветки
          byte lightChannel = ourScratch->slots[i].slotLinkedData;
          if(lightChannel < 8)
          {
            if(state.LightChannelsState & (1 << lightChannel))
              slotStatus = 1; // выставляем в слоте значение 1
              
          }
        }
        break;

        case slotPin:
        {
          // получаем статус пина
          byte pinNumber = ourScratch->slots[i].slotLinkedData;
          byte byteNum = pinNumber/8;
          byte bitNum = pinNumber%8;

          if(byteNum < 16)
          {
            // если нужный бит с номером пина установлен - на пине высокий уровень
            if(state.PinsState[byteNum] & (1 << bitNum))
              slotStatus = 1; // выставляем в слоте значение 1
          }
          
        }
        break;
        
      } // switch

      // мы получили slotStatus, записываем его обратно в слот
      ourScratch->slots[i].slotStatus = slotStatus;
   } // for

  if(receivedThrough == ssOneWire)
  {
    // пишем актуальное состояние слотов клиенту, если скратч был получен по 1-Wire, иначе - вызывающая сторона сама разберётся, что делать с изменениями
    UniScratchpad.begin(pin,scratchpad);
    UniScratchpad.write();
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_UNI_EXECUTION_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// SensorsUniClient
//-------------------------------------------------------------------------------------------------------------------------------------------------------
SensorsUniClient::SensorsUniClient() : AbstractUniClient()
{
  measureTimer = 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SensorsUniClient::Register(UniRawScratchpad* scratchpad)
{
  // регистрируем модуль тут, добавляя нужные индексы датчиков в контроллер
  UniSensorsScratchpad* ourScrath = (UniSensorsScratchpad*) &(scratchpad->data);
  byte addedCount = 0;

  for(byte i=0;i<MAX_UNI_SENSORS;i++)
  {
    byte type = ourScrath->sensors[i].type;
    if(type == NO_SENSOR_REGISTERED) // нет типа датчика 
      continue;

    UniSensorType ut = (UniSensorType) type;
    
    if(ut == uniNone) // нет типа датчика
      continue;

    // имеем тип датчика, можем регистрировать
    if(UniDispatcher.AddUniSensor(ut,ourScrath->sensors[i].index))
      addedCount++;
    
  } // for

  if(addedCount > 0) // добавили датчики, надо сохранить состояние контроллера в EEPROM
    UniDispatcher.SaveState();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SensorsUniClient::Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough)
{
  
    // тут обновляем данные, полученный по проводу с модуля. 
    // нам передали адрес скратчпада, куда можно писать данные, полученные
    // с клиента, при необходимости.

    // нас дёргают после вычитки скратчпада из модуля, всё, что мы должны сделать - 
    // это обновить данные в контроллере.

    UniSensorsScratchpad* ourScratch = (UniSensorsScratchpad*) &(scratchpad->data);
    UniSensorState states;
    
    
    for(byte i=0;i<MAX_UNI_SENSORS;i++)
    {

      byte type = ourScratch->sensors[i].type;
      if(type == NO_SENSOR_REGISTERED) // нет типа датчика 
        continue;
  
      UniSensorType ut = (UniSensorType) type;
      
      if(ut == uniNone) // нет типа датчика
        continue;

      if(ut == uniPinsMap) // слот с картой пинов
      {
        ProcessPinsMap(ourScratch->sensors[i].index,ourScratch->sensors[i].data);
        continue;
      }
      
      if(UniDispatcher.GetRegisteredStates(ut, ourScratch->sensors[i].index, states))
      {
        // получили состояния, можно обновлять
        UpdateStateData(states, &(ourScratch->sensors[i]), isModuleOnline);
      } // if
    } // for

    // тут запускаем конвертацию, чтобы при следующем вызове вычитать актуальные данные.
    // конвертацию не стоит запускать чаще, чем в 5, скажем, секунд.
    if(receivedThrough == ssOneWire && isModuleOnline)
    {
      // работаем таким образом только по шине 1-Wire, в остальном вызывающая сторона разберётся, что делать со скратчпадом
      unsigned long curMillis = millis();
      if(curMillis - measureTimer > 5000)
      {
        #ifdef UNI_DEBUG
          DEBUG_LOG(F("Start measure on 1-Wire pin "));
          DEBUG_LOGLN(String(pin));
         #endif    
        
        measureTimer = curMillis;
        UniScratchpad.begin(pin,scratchpad);
        UniScratchpad.startMeasure();
      }
      
    } // if

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SensorsUniClient::UpdateStateData(const UniSensorState& states,const UniSensorData* data,bool IsModuleOnline)
{
  if(!(states.State1 || states.State2))
    return; // не найдено ни одного состояния  

  UpdateOneState(states.State1,data,IsModuleOnline);
  UpdateOneState(states.State2,data,IsModuleOnline);  

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SensorsUniClient::UpdateOneState(OneState* os, const UniSensorData* dataPacket, bool IsModuleOnline)
{
    if(!os)
      return;

   uint8_t sensorIndex = dataPacket->index;
   uint8_t sensorType = dataPacket->type;
   uint8_t dataIndex = 0;

   if(sensorIndex == NO_SENSOR_REGISTERED || sensorType == NO_SENSOR_REGISTERED || sensorType == uniNone)
    return; // нет датчика вообще

   switch(os->GetType())
   {
      case StateTemperature:
      {
        if(sensorType == uniHumidity) // если тип датчика - влажность, значит температура у нас идёт после влажности, в 3-м и 4-м байтах
        {
          dataIndex++; dataIndex++;
        }

        int8_t dt = (int8_t) dataPacket->data[dataIndex++];
        uint8_t dt2 =  dataPacket->data[dataIndex];
        
        int8_t b1 = IsModuleOnline ? dt : NO_TEMPERATURE_DATA;             
        uint8_t b2 = IsModuleOnline ? dt2 : 0;

        Temperature t(b1, b2);

        // convert to Fahrenheit if needed
        #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
         t = Temperature::ConvertToFahrenheit(t);
        #endif      
        
        os->Update(&t);
        
      }
      break;

      case StateHumidity:
      case StateSoilMoisture:
      case StatePH:
      {
        int8_t dt = (int8_t)  dataPacket->data[dataIndex++];
        uint8_t dt2 =  dataPacket->data[dataIndex];
        
        int8_t b1 = IsModuleOnline ? dt : NO_TEMPERATURE_DATA;    
        uint8_t b2 = IsModuleOnline ? dt2 : 0;
        
        Humidity h(b1, b2);
        os->Update(&h);        

      }
      break;

      case StateLuminosity:
      {
        unsigned long lum = NO_LUMINOSITY_DATA;
        
        if(IsModuleOnline)
          memcpy(&lum, dataPacket->data, 4);

        os->Update(&lum);
        
      }
      break;

      case StateWaterFlowInstant:
      case StateWaterFlowIncremental:
      case StateUnknown:
      case StateCO2:
      
      break;
      
    
   } // switch
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// SunControllerClient
//-------------------------------------------------------------------------------------------------------------------------------------------------------
SunControllerClient::SunControllerClient() : AbstractUniClient()
{
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SunControllerClient::Register(UniRawScratchpad* scratchpad)
{
  // регистрируем модуль тут, добавляя нужные индексы датчиков в контроллер
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void SunControllerClient::Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough)
{
  
    // тут обновляем данные, полученный по проводу с модуля. 
    // нам передали адрес скратчпада, куда можно писать данные, полученные
    // с клиента, при необходимости.

    // нас дёргают после вычитки скратчпада из модуля, всё, что мы должны сделать - 
    // это обновить данные в контроллере.

    if(ssRadio == receivedThrough)
    {
      #ifdef USE_UNI_SUN_CONTROLLER_MODULE
          // получили данные по радио
          SunControllerDataPacket* dt = (SunControllerDataPacket*) scratchpad->data;
          
          #if defined(NRF_DEBUG) || defined(LORA_DEBUG)
            DEBUG_LOGLN(F("RECEIVED SUN CONTROLLER DATA VIA RADIO!"));
            
            DEBUG_LOG(F("T1: "));
            DEBUG_LOGLN(String(dt->T1whole) + String(".") + String(dt->T1fract));
    
            DEBUG_LOG(F("T2: "));
            DEBUG_LOGLN(String(dt->T2whole) + String(".") + String(dt->T2fract));
    
            DEBUG_LOG(F("T3: "));
            DEBUG_LOGLN(String(dt->T3whole) + String(".") + String(dt->T3fract));
    
            DEBUG_LOG(F("T4: "));
            DEBUG_LOGLN(String(dt->T4whole) + String(".") + String(dt->T4fract));
    
            DEBUG_LOG(F("T5: "));
            DEBUG_LOGLN(String(dt->T5whole) + String(".") + String(dt->T5fract));
    
            DEBUG_LOG(F("Angle: "));
            DEBUG_LOGLN(String(dt->angle));
    
            DEBUG_LOG(F("Direction: "));
            DEBUG_LOGLN(String(dt->direction));
    
            DEBUG_LOG(F("Luminosity: "));
            DEBUG_LOGLN(String(dt->luminosity));          

            DEBUG_LOG(F("Heading: "));
            DEBUG_LOGLN(String(dt->heading));          
            
         #endif
    
          //LogicManageModule->SetWindSpeed(windRainData->windSpeed);
          //LogicManageModule->SetWindDirection((CompassPoints)windRainData->windDirection);
          //LogicManageModule->SetHasRain(windRainData->hasRain);

      #endif // USE_UNI_SUN_CONTROLLER_MODULE
    } // if(ssRadio == receivedThrough)


}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATER_TANK_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// WaterTankClient
//-------------------------------------------------------------------------------------------------------------------------------------------------------
WaterTankClient::WaterTankClient() : AbstractUniClient()
{
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void WaterTankClient::Register(UniRawScratchpad* scratchpad)
{
  // регистрируем модуль тут, добавляя нужные индексы датчиков в контроллер
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void WaterTankClient::Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough)
{
  
    // тут обновляем данные, полученный с модуля контроля бака по радиоканалу
    if(ssRadio == receivedThrough)
    {
      // получили данные по радио
      WaterTankDataPacket* waterTankData = (WaterTankDataPacket*) scratchpad->data;
      
      #if defined(WATER_TANK_MODULE_DEBUG)
        Serial.println(F("RECEIVED WATER TANK DATA VIA RADIO:"));
        Serial.print(F("\tValve state: ")); Serial.println(waterTankData->valveState);
        Serial.print(F("\tFill status, %: ")); Serial.println(waterTankData->fillStatus);
        Serial.print(F("\tErrors status - ")); Serial.println(waterTankData->errorFlag); //добавил еще вывод флага ошибки в дебаг-сообщение - КМВ
        Serial.print(F("\tErrors type - ")); Serial.println(waterTankData->errorType); //добавил еще вывод флага ошибки в дебаг-сообщение - КМВ
        
     #endif

     // ТУТ ОБНОВЛЕНИЕ ДАННЫХ В КОНТРОЛЛЕРЕ
     WaterTank->UpdateState(waterTankData->valveState,waterTankData->fillStatus,waterTankData->errorFlag,waterTankData->errorType);

    } // if(ssRadio == receivedThrough)


}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATER_TANK_MODULE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// WindRainClient
//-------------------------------------------------------------------------------------------------------------------------------------------------------
WindRainClient::WindRainClient() : AbstractUniClient()
{
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void WindRainClient::Register(UniRawScratchpad* scratchpad)
{
  // регистрируем модуль тут, добавляя нужные индексы датчиков в контроллер
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void WindRainClient::Update(UniRawScratchpad* scratchpad, bool isModuleOnline, UniScratchpadSource receivedThrough)
{
  
    // тут обновляем данные, полученный по проводу с модуля. 
    // нам передали адрес скратчпада, куда можно писать данные, полученные
    // с клиента, при необходимости.

    // нас дёргают после вычитки скратчпада из модуля, всё, что мы должны сделать - 
    // это обновить данные в контроллере.

    if(ssRadio == receivedThrough)
    {
      // получили данные по радио
      WindRainDataPacket* windRainData = (WindRainDataPacket*) scratchpad->data;
      
      #if defined(NRF_DEBUG) || defined(LORA_DEBUG)
        DEBUG_LOGLN(F("RECEIVED WIND AND RAIN DATA VIA RADIO!"));
        
        DEBUG_LOG(F("Wind speed: "));
        DEBUG_LOGLN(String(windRainData->windSpeed));

        DEBUG_LOG(F("Wind direction: "));
        DEBUG_LOGLN(String(windRainData->windDirection));

        DEBUG_LOG(F("Has rain: "));
        DEBUG_LOGLN(String(windRainData->hasRain));
     #endif

      LogicManageModule->SetWindSpeed(windRainData->windSpeed);
      LogicManageModule->SetWindDirection((CompassPoints)windRainData->windDirection);
      LogicManageModule->SetHasRain(windRainData->hasRain);
    } // if(ssRadio == receivedThrough)


}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniPermanentLine::UniPermanentLine(uint8_t pinNumber)
{
  pin = pinNumber;
  timer = random(0,UNI_MODULE_UPDATE_INTERVAL); // разнесём опрос датчиков по времени
  lastClient = NULL;

  _timer = 0;

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniPermanentLine::IsRegistered()
{
  if(SHARED_SCRATCHPAD.head.packet_type == uniNextionClient) // для дисплея Nextion не требуется регистрация 
    return true;
    
  return ( SHARED_SCRATCHPAD.head.controller_id == UniDispatcher.GetControllerID() );
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniPermanentLine::Update()
{
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;
    
  timer += dt;

  if(timer < UNI_MODULE_UPDATE_INTERVAL) // рано обновлять
    return;

  timer -= UNI_MODULE_UPDATE_INTERVAL; // сбрасываем таймер

  // теперь обновляем последнего клиента, если он был.
  // говорим ему, чтобы обновился, как будто модуля нет на линии.
  if(lastClient)
  {
    lastClient->Update(&SHARED_SCRATCHPAD,false, ssOneWire);
    lastClient = NULL; // сбрасываем клиента, поскольку его может больше не быть на линии
  }

  // теперь пытаемся прочитать скратчпад
  UniScratchpad.begin(pin,&SHARED_SCRATCHPAD);
  
  if(UniScratchpad.read())
  {
    // прочитали, значит, датчик есть на линии.
   #ifdef UNI_DEBUG
    DEBUG_LOG(F("Module found on 1-Wire pin "));
    DEBUG_LOGLN(String(pin));
   #endif    
   
    // проверяем, зарегистрирован ли модуль у нас?
    if(!IsRegistered()) // модуль не зарегистрирован у нас
      return;
      
    // получаем клиента для прочитанного скратчпада
    lastClient = UniFactory.GetClient(&SHARED_SCRATCHPAD);
    lastClient->SetPin(pin); // назначаем тот же самый пин, что у нас    
    lastClient->Update(&SHARED_SCRATCHPAD,true, ssOneWire);

    // вот здесь получается следующая ситуация - может отправиться команда на старт измерений, и, одновременно - 
    // команда на чтение скратчпада. команда на старт измерений требует времени, поэтому, если
    // сразу после запуска конвертации пытаться читать - получится бяка.
    
  } // if
  else
  {
    // на линии никого нет
   #ifdef UNI_DEBUG
    DEBUG_LOG(F("NO MODULES FOUND ON 1-Wire pin "));
    DEBUG_LOGLN(String(pin));
   #endif

  }
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// UniRegDispatcher
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniRegDispatcher::UniRegDispatcher()
{
  temperatureModule = NULL;
  humidityModule = NULL;
  luminosityModule = NULL;
  soilMoistureModule = NULL;
  phModule = NULL;  

  currentTemperatureCount = 0;
  currentHumidityCount = 0;
  currentLuminosityCount = 0;
  currentSoilMoistureCount = 0;
  currentPHCount = 0;
  currentPinsMapCount = 0;

  hardCodedTemperatureCount = 0;
  hardCodedHumidityCount = 0;
  hardCodedLuminosityCount = 0;
  hardCodedSoilMoistureCount = 0;
  hardCodedPHCount = 0;

  rfChannel = UNI_DEFAULT_RF_CHANNEL;
    
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRegDispatcher::AddUniSensor(UniSensorType type, uint8_t sensorIndex)
{
  // добавляем состояние для датчика в систему. Состояние надо добавлять только тогда,
  // когда переданный индекс датчика не укладывается в уже выданный диапазон.
  // например, переданный индекс - 0, и кол-во выданных до этого индексов - 0, следовательно,
  // мы не попадаем в выданный диапазон. Или - переданный индекс - 1, кол-во ранее выданных - 0,
  // значит, мы должны добавить 2 новых состояния.

  // если sensorIndex == 0xFF - ничего делать не надо
  if(sensorIndex == NO_SENSOR_REGISTERED) // попросили зарегистрировать датчик без назначенного ранее индекса, ошибка.
    return false;
  
   switch(type)
  {
    case uniNone:  // нет датчика
      return false;

    case uniPinsMap:
    {
      if(sensorIndex < currentPinsMapCount) // попали в диапазон уже выданных
        return false;

      // добавляем ещё один датчик "карта пинов"
      currentPinsMapCount++;
      return true;
    }
    
    case uniTemp:  // температурный датчик
      if(temperatureModule)
      {
          if(sensorIndex < currentTemperatureCount) // попадаем в диапазон уже выданных
            return false;

          // здесь sensorIndex больше либо равен currentTemperatureCount, следовательно, мы не попадаем в диапазон
          uint8_t to_add = (sensorIndex - currentTemperatureCount) + 1;

          for(uint8_t cntr = 0; cntr < to_add; cntr++)
          {
            temperatureModule->State.AddState(StateTemperature,hardCodedTemperatureCount + currentTemperatureCount + cntr);
          } // for

          // сохраняем кол-во добавленных
          currentTemperatureCount += to_add;
          
        return true;
      } // if(temperatureModule)
      else
        return false;
    
    case uniHumidity: 
    if(humidityModule)
      {

          if(sensorIndex < currentHumidityCount) // попадаем в диапазон уже выданных
            return false;

          // здесь sensorIndex больше либо равен currentHumidityCount, следовательно, мы не попадаем в диапазон
          uint8_t to_add = (sensorIndex - currentHumidityCount) + 1;

          for(uint8_t cntr = 0; cntr < to_add; cntr++)
          {
            humidityModule->State.AddState(StateTemperature,hardCodedHumidityCount + currentHumidityCount + cntr);
            humidityModule->State.AddState(StateHumidity,hardCodedHumidityCount + currentHumidityCount + cntr);
          } // for

          // сохраняем кол-во добавленных
          currentHumidityCount += to_add;
          
        return true;
        
      }
      else
        return false;
    
    case uniLuminosity: 
    if(luminosityModule)
      {

          if(sensorIndex < currentLuminosityCount) // попадаем в диапазон уже выданных
            return false;

          // здесь sensorIndex больше либо равен currentLuminosityCount, следовательно, мы не попадаем в диапазон
          uint8_t to_add = (sensorIndex - currentLuminosityCount) + 1;

          for(uint8_t cntr = 0; cntr < to_add; cntr++)
          {
            luminosityModule->State.AddState(StateLuminosity,hardCodedLuminosityCount + currentLuminosityCount + cntr);
          } // for

          // сохраняем кол-во добавленных
          currentLuminosityCount += to_add;
          
        return true;
      }    
      else
        return false;
    
    case uniSoilMoisture: 
     if(soilMoistureModule)
      {
     
          if(sensorIndex < currentSoilMoistureCount) // попадаем в диапазон уже выданных
            return false;

          // здесь sensorIndex больше либо равен currentSoilMoistureCount, следовательно, мы не попадаем в диапазон
          uint8_t to_add = (sensorIndex - currentSoilMoistureCount) + 1;

          for(uint8_t cntr = 0; cntr < to_add; cntr++)
          {
            soilMoistureModule->State.AddState(StateSoilMoisture,hardCodedSoilMoistureCount + currentSoilMoistureCount + cntr);
          } // for

          // сохраняем кол-во добавленных
          currentSoilMoistureCount += to_add;
          
        return true;
      } 
      else
        return false;


    case uniPH:
     if(phModule)
      {
     
          if(sensorIndex < currentPHCount) // попадаем в диапазон уже выданных
            return false;

          // здесь sensorIndex больше либо равен currentPHCount, следовательно, мы не попадаем в диапазон
          uint8_t to_add = (sensorIndex - currentPHCount) + 1;

          for(uint8_t cntr = 0; cntr < to_add; cntr++)
          {
            phModule->State.AddState(StatePH,hardCodedPHCount + currentPHCount + cntr);
          } // for

          // сохраняем кол-во добавленных
          currentPHCount += to_add;
          
        return true;
      } 
      else
        return false;
  } 

  return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t UniRegDispatcher::GetUniSensorsCount(UniSensorType type)
{
  switch(type)
  {
    case uniNone: return 0;
    case uniTemp: return currentTemperatureCount;
    case uniHumidity: return currentHumidityCount;
    case uniLuminosity: return currentLuminosityCount;
    case uniSoilMoisture: return currentSoilMoistureCount;
    case uniPH: return currentPHCount;
    case uniPinsMap: return currentPinsMapCount;
  }

  return 0;  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t UniRegDispatcher::GetHardCodedSensorsCount(UniSensorType type)
{
  switch(type)
  {
    case uniNone: return 0;
    case uniTemp: return hardCodedTemperatureCount;
    case uniHumidity: return hardCodedHumidityCount;
    case uniLuminosity: return hardCodedLuminosityCount;
    case uniSoilMoisture: return hardCodedSoilMoistureCount;
    case uniPH: return hardCodedPHCount;
    case uniPinsMap: return 0;
  }

  return 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegDispatcher::Setup()
{
    temperatureModule = MainController->GetModuleByID("STATE");
    if(temperatureModule)
      hardCodedTemperatureCount = temperatureModule->State.GetStateCount(StateTemperature);
    
    humidityModule = MainController->GetModuleByID("HUMIDITY");
    if(humidityModule)
      hardCodedHumidityCount = humidityModule->State.GetStateCount(StateHumidity);
    
    luminosityModule = MainController->GetModuleByID("LIGHT");
    if(luminosityModule)
      hardCodedLuminosityCount = luminosityModule->State.GetStateCount(StateLuminosity);

    soilMoistureModule = MainController->GetModuleByID("SOIL");
    if(soilMoistureModule)
      hardCodedSoilMoistureCount = soilMoistureModule->State.GetStateCount(StateSoilMoisture);

    phModule = MainController->GetModuleByID("PH");
    if(phModule)
      hardCodedPHCount = phModule->State.GetStateCount(StatePH);


    ReadState(); // читаем последнее запомненное состояние
    RestoreState(); // восстанавливаем последнее запомненное состояние
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t UniRegDispatcher::GetRFChannel() // возвращает текущий канал для nRF
{
  rfChannel = MemRead(UNI_SENSOR_INDICIES_EEPROM_ADDR + 4);
  
  if(rfChannel == 0xFF)
    rfChannel = UNI_DEFAULT_RF_CHANNEL;
    
  return rfChannel;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegDispatcher::SetRFChannel(uint8_t channel) // устанавливает канал для nRF
{
  if(rfChannel != channel)
  {
    rfChannel = channel;
    SaveState();
  }
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegDispatcher::ReadState()
{
  //Тут читаем последнее запомненное состояние по индексам сенсоров
  uint32_t addr = UNI_SENSOR_INDICIES_EEPROM_ADDR;
  uint8_t val = MemRead(addr++);
  if(val != 0xFF)
    currentTemperatureCount = val;

  val = MemRead(addr++);
  if(val != 0xFF)
    currentHumidityCount = val;

  val = MemRead(addr++);
  if(val != 0xFF)
    currentLuminosityCount = val;

  val = MemRead(addr++);
  if(val != 0xFF)
    currentSoilMoistureCount = val;

  val = MemRead(addr++);
  if(val != 0xFF)
    rfChannel = val;
  else
    rfChannel = UNI_DEFAULT_RF_CHANNEL;

  val = MemRead(addr++);
  if(val != 0xFF)
    currentPHCount = val;

  // читаем выданные индексы для карт пинов
  val = MemRead(addr++);
  if(val != 0xFF)
    currentPinsMapCount = val;
   
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegDispatcher::RestoreState()
{
  //Тут восстанавливаем последнее запомненное состояние индексов сенсоров.
  // добавляем новые датчики в нужный модуль до тех пор, пока
  // их кол-во не сравняется с сохранённым последним выданным индексом.
  // индексы универсальным датчикам выдаются, начиная с 0, при этом данный индекс является
  // виртуальным, поэтому нам всегда надо добавить датчик в конец
  // списка, после жёстко указанных в прошивке датчиков. Такой подход
  // обеспечит нормальную работу универсальных датчиков вне зависимости
  // от настроек прошивки.
  
  if(temperatureModule)
  {
    uint8_t cntr = 0;    
    while(cntr < currentTemperatureCount)
    {
      temperatureModule->State.AddState(StateTemperature, hardCodedTemperatureCount + cntr);
      cntr++;
    }
    
  } // if(temperatureModule)

  if(humidityModule)
  {
    uint8_t cntr = 0;
    while(cntr < currentHumidityCount)
    {
      humidityModule->State.AddState(StateTemperature, hardCodedHumidityCount + cntr);
      humidityModule->State.AddState(StateHumidity, hardCodedHumidityCount + cntr);
      cntr++;
    }
    
  } // if(humidityModule)

 if(luminosityModule)
  {
    uint8_t cntr = 0;
    while(cntr < currentLuminosityCount)
    {
      luminosityModule->State.AddState(StateLuminosity, hardCodedLuminosityCount + cntr);
      cntr++;
    }
    
  } // if(luminosityModule)  

if(soilMoistureModule)
  {
    uint8_t cntr = 0;
    while(cntr < currentSoilMoistureCount)
    {
      soilMoistureModule->State.AddState(StateSoilMoisture, hardCodedSoilMoistureCount + cntr);
      cntr++;
    }
    
  } // if(soilMoistureModule) 

 if(phModule)
  {
    uint8_t cntr = 0;
    while(cntr < currentPHCount)
    {
      phModule->State.AddState(StatePH, hardCodedPHCount + cntr);
      cntr++;
    }
    
  } // if(phModule)  

 // Что мы сделали? Мы добавили N виртуальных датчиков в каждый модуль, основываясь на ранее сохранённой информации.
 // в результате в контроллере появились датчики с показаниями <нет данных>, и показания с них обновятся, как только
 // поступит информация от них с универсальных модулей.
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegDispatcher::SaveState()
{
  //Тут сохранение текущего состояния в EEPROM
  uint32_t addr = UNI_SENSOR_INDICIES_EEPROM_ADDR;  
  MemWrite(addr++,currentTemperatureCount);
  MemWrite(addr++,currentHumidityCount);
  MemWrite(addr++,currentLuminosityCount);
  MemWrite(addr++,currentSoilMoistureCount);
  MemWrite(addr++,rfChannel);
  MemWrite(addr++,currentPHCount);
  MemWrite(addr++,currentPinsMapCount);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRegDispatcher::GetRegisteredStates(UniSensorType type, uint8_t sensorIndex, UniSensorState& resultStates)
{
  resultStates.State1 = NULL;
  resultStates.State2 = NULL;
  
   // смотрим тип сенсора, получаем состояния
   switch(type)
   {
    case uniNone: return false;
    case uniPinsMap: return false;
    
    case uniTemp: 
    {
        if(!temperatureModule)
          return false; // нет модуля температур в прошивке

       // получаем состояние. Поскольку индексы виртуальных датчиков у нас относительные, то прибавляем
       // к индексу датчика кол-во жёстко прописанных в прошивке. В результате получаем абсолютный индекс датчика в системе.
       resultStates.State1 = temperatureModule->State.GetState(StateTemperature,hardCodedTemperatureCount + sensorIndex);

       return (resultStates.State1 != NULL);
       
    }
    break;
    
    case uniHumidity: 
    {
        if(!humidityModule)
          return false; // нет модуля влажности в прошивке

       resultStates.State1 = humidityModule->State.GetState(StateHumidity,hardCodedHumidityCount + sensorIndex);
       resultStates.State2 = humidityModule->State.GetState(StateTemperature,hardCodedHumidityCount + sensorIndex);

       return (resultStates.State1 != NULL && resultStates.State2 != NULL);

    }
    break;
    
    case uniLuminosity: 
    {
        if(!luminosityModule)
          return false; // нет модуля освещенности в прошивке

       resultStates.State1 = luminosityModule->State.GetState(StateLuminosity,hardCodedLuminosityCount + sensorIndex);
       return (resultStates.State1 != NULL);      
    }
    break;
    
    case uniSoilMoisture: 
    {
        if(!soilMoistureModule)
          return false; // нет модуля влажности почвы в прошивке

       resultStates.State1 = soilMoistureModule->State.GetState(StateSoilMoisture,hardCodedSoilMoistureCount + sensorIndex);
       return (resultStates.State1 != NULL);
      
    }
    break;

    case uniPH:
    {
        if(!phModule)
          return false; // нет модуля pH в прошивке

       resultStates.State1 = phModule->State.GetState(StatePH,hardCodedPHCount + sensorIndex);
       return (resultStates.State1 != NULL);
      
    }
    break;
   } // switch

  return false;    
 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t UniRegDispatcher::GetControllerID()
{
  return MainController->GetSettings()->GetControllerID(); 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// UniScratchpadClass
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniScratchpadClass::UniScratchpadClass()
{
  pin = 0;
  scratchpad = NULL;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniScratchpadClass::canWork()
{
  return (pin > 0 && scratchpad != NULL);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniScratchpadClass::begin(byte _pin,UniRawScratchpad* scratch)
{
  pin = _pin;
  scratchpad = scratch;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniScratchpadClass::read()
{
  if(!canWork())
    return false;
    
    OneWire ow(pin);
    WORK_STATUS.PinMode(pin,INPUT,false);
    
    if(!ow.reset()) { // нет датчика на линии 
      
     #ifdef UNI_DEBUG
      DEBUG_LOG(F("NO PRESENCE FOUND ON 1-Wire pin "));
      DEBUG_LOGLN(String(pin));
     #endif
      return false; 
}

    // теперь читаем скратчпад
    ow.write(0xCC, 1);
    ow.write(UNI_READ_SCRATCHPAD,1); // посылаем команду на чтение скратчпада

    byte* raw = (byte*) scratchpad;
    // читаем скратчпад
    for(uint8_t i=0;i<sizeof(UniRawScratchpad);i++)
      raw[i] = ow.read();
      
    // проверяем контрольную сумму
    bool isCrcGood =  OneWire::crc8(raw, sizeof(UniRawScratchpad)-1) == raw[sizeof(UniRawScratchpad)-1];

   #ifdef UNI_DEBUG
    if(isCrcGood) {
      DEBUG_LOG(F("Checksum OK on 1-Wire pin "));
      DEBUG_LOGLN(String(pin));
    } else {
      DEBUG_LOG(F("BAD scratchpad checksum on 1-Wire pin "));
      DEBUG_LOGLN(String(pin));
      
    }
   #endif

    return isCrcGood;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniScratchpadClass::startMeasure()
{
  if(!canWork())
    return false;
    
    OneWire ow(pin);
    WORK_STATUS.PinMode(pin,INPUT,false);
    
    if(!ow.reset()) // нет датчика на линии
      return false; 

    ow.write(0xCC, 1);
    ow.write(UNI_START_MEASURE,1); // посылаем команду на старт измерений
    
    return ow.reset();
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniScratchpadClass::write()
{
  if(!canWork())
    return false;
    
    OneWire ow(pin);
    WORK_STATUS.PinMode(pin,INPUT,false);
    
  // выставляем ID нашего контроллера
  scratchpad->head.controller_id = UniDispatcher.GetControllerID();
  
  // подсчитываем контрольную сумму и записываем её в последний байт скратчпада
  scratchpad->crc8 = OneWire::crc8((byte*) scratchpad, sizeof(UniRawScratchpad)-1);

  if(!ow.reset()) // нет датчика на линии
    return false; 

  ow.write(0xCC, 1);
  ow.write(UNI_WRITE_SCRATCHPAD,1); // говорим, что хотим записать скратчпад

  byte* raw = (byte*) scratchpad;
  // теперь пишем данные
   for(uint8_t i=0;i<sizeof(UniRawScratchpad);i++)
    ow.write(raw[i]);

   return ow.reset();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniScratchpadClass::save()
{
  if(!canWork())
    return false;
    
  OneWire ow(pin);
  WORK_STATUS.PinMode(pin,INPUT,false);

  if(!ow.reset())
    return false;
    
  // записываем всё в EEPROM
  ow.write(0xCC, 1);
  ow.write(UNI_SAVE_EEPROM,1);
  delay(100);
   
  return ow.reset();   
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// UniRegistrationLine
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniRegistrationLine::UniRegistrationLine(byte _pin)
{
  pin = _pin;

  memset(&scratchpad,0xFF,sizeof(scratchpad));
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRegistrationLine::IsModulePresent()
{
  // проверяем, есть ли модуль на линии, простой вычиткой скратчпада
  UniScratchpad.begin(pin,&scratchpad);

   return UniScratchpad.read();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegistrationLine::CopyScratchpad(UniRawScratchpad* dest)
{
  memcpy(dest,&scratchpad,sizeof(UniRawScratchpad));
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniRegistrationLine::Register()
{
  // регистрируем модуль в системе. Чего там творится в скратчпаде - нас не колышет, это делает конфигуратор: назначает индексы и т.п.

  // однако, в зависимости от типа пакета, нам надо обновить состояние контроллера (например, добавить индексы виртуальных датчиков в систему).
  // это делается всегда, вне зависимости от того, был ранее зарегистрирован модуль или нет - индексы всегда поддерживаются в актуальном
  // состоянии - переназначили мы их или нет. Считаем, что в случаем универсального модуля с датчиками конфигуратор сам правильно расставил
  // все индексы, и нам достаточно только поддержать актуальное состояние индексов у контроллера.

  // подобная настройка при регистрации разных типов модулей может иметь различное поведение, поэтому мы должны работать с разными субъектами
  // такой настройки.

  // получаем клиента
  AbstractUniClient* client = UniFactory.GetClient(&scratchpad);
  
  // просим клиента зарегистрировать модуль в системе, чего он там будет делать - дело десятое.
  client->Register(&scratchpad);

  // теперь мы смело можем писать скратчпад обратно в модуль
  UniScratchpad.begin(pin,&scratchpad);
  
  if(UniScratchpad.write())
    UniScratchpad.save();

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRegistrationLine::IsSameScratchpadType(UniRawScratchpad* src)
{
  if(!src)
    return false;

  return (scratchpad.head.packet_type == src->head.packet_type);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniRegistrationLine::SetScratchpadData(UniRawScratchpad* src)
{
  if(!IsSameScratchpadType(src)) // разные типы пакетов в переданном скратчпаде и вычитанном, нельзя копировать
    return false;

  memcpy(&scratchpad,src,sizeof(UniRawScratchpad));
  return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#define RADIO_PAYLOAD_SIZE 30 // размер нашего пакета для передачи по радиоканалам
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_NRF_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#include <RF24.h>
//-------------------------------------------------------------------------------------------------------------------------------------------------------
RF24* radio = NULL;//(NRF_CE_PIN,NRF_CSN_PIN);
uint64_t controllerStatePipe = 0xF0F0F0F0E0LL; // труба, в которую  мы пишем состояние контроллера
// трубы, которые мы слушаем на предмет показаний с датчиков
const uint64_t readingPipes[5] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0E2LL, 0xF0F0F0F0E3LL, 0xF0F0F0F0E4LL, 0xF0F0F0F0E5LL };
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef NRF_DEBUG
int serial_putc( char c, FILE * ) {
  Serial.write( c );
  return c;
}

void printf_begin(void) {
  #if TARGET_BOARD != DUE_BOARD
  fdevopen( &serial_putc, 0 );
  #endif
  Serial.println(F("Init nRF..."));
}
#endif // NRF_DEBUG
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniNRFGate::UniNRFGate()
{
  bFirstCall = true;
  nRFInited = false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniNRFGate::isInOnlineQueue(byte sensorType,byte sensorIndex, byte& result_index)
{
  for(size_t i=0;i<sensorsOnlineQueue.size();i++)
    if(sensorsOnlineQueue[i].sensorType == sensorType && sensorsOnlineQueue[i].sensorIndex == sensorIndex)
    {
      result_index = i;
      return true;
    }
  return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniNRFGate::Setup()
{
  NrfBinding bnd = HardwareBinding->GetNrfBinding();

  if(bnd.LinkType != linkUnbinded)
  {
    if(bnd.LinkType == linkDirect)
    {
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          #ifndef DISABLE_NRF_CONFIGURE
          if(EEPROMSettingsModule::SafePin(bnd.PowerPin))
          #else
          if(bnd.PowerPin > 1) // prevent Serial locking
          #endif
          {
            WORK_STATUS.PinMode(bnd.PowerPin,OUTPUT);
            WORK_STATUS.PinWrite(bnd.PowerPin,bnd.Level);
          }
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.PowerPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.PowerPin,bnd.Level);      
        }
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
     #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.PowerPin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.PowerPin,bnd.Level);      
        }
     #endif 
    }
    
  } // if(bnd.LinkType != linkUnbinded)
  
  initNRF();
  memset(&packet,0,sizeof(packet));
  ControllerState st = WORK_STATUS.GetState();
  // копируем состояние контроллера к нам
  memcpy(&(packet.state),&st,sizeof(ControllerState));
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniNRFGate::readFromPipes()
{
  if(!nRFInited)
    return;
    
  // открываем все пять труб на прослушку
  for(byte i=0;i<5;i++)
    radio->openReadingPipe(i+1,readingPipes[i]);  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniNRFGate::Update()
{
  if(!nRFInited)
    return;

  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;    

  static uint16_t onlineCheckTimer = 0;
  onlineCheckTimer += dt;
  if(onlineCheckTimer > 5000)
  {
    onlineCheckTimer = 0;

    //Тут, раз в пять секунд - мы должны проверять, не истёк ли интервал
    // получения показаний с датчиков, показания с которых были получены ранее.
    // если интервал истёк - мы должны выставить датчику показания "нет данных"
    // и удалить его из очереди.

    byte count_passes = sensorsOnlineQueue.size();
    byte cur_idx = count_passes-1;

    unsigned long nowTime = millis();

    // проходим от хвоста до головы
    while(count_passes)
    {
      NRFQueueItem* qi = &(sensorsOnlineQueue[cur_idx]);

      // вычисляем интервал в миллисекундах
      unsigned long query_interval = qi->queryInterval*1000;
      
      // смотрим, не истёк ли интервал с момента последнего опроса
      if((nowTime - qi->gotLastDataAt) > (query_interval+3000) )
      {
        
        // датчик не откликался дольше, чем интервал между опросами плюс дельта в 3 секунды,
        // надо ему выставить показания "нет данных"
          byte sType = qi->sensorType;
          byte sIndex = qi->sensorIndex;
        
          UniDispatcher.AddUniSensor((UniSensorType)sType,sIndex);

            // проверяем тип датчика, которому надо выставить "нет данных"
            switch(qi->sensorType)
            {
              case uniTemp:
              {
                // температура
                Temperature t;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&t);
                } // if
              }
              break;

              case uniHumidity:
              {
                // влажность
                Humidity h;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&h);

                  if(states.State2)
                    states.State2->Update(&h);
                } // if                        
              }
              break;

              case uniLuminosity:
              {
                // освещённость
                long lum = NO_LUMINOSITY_DATA;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&lum);
                } // if                        
                
                
              }
              break;

              case uniSoilMoisture: // влажность почвы
              case uniPH: // показания pH
              {
                
                Humidity h;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&h);
                } // if                        
                
              }
              break;

              case uniPinsMap:
              break;
              
            } // switch

        // теперь удаляем оффлайн-датчик из очереди
        sensorsOnlineQueue.pop();
        
      } // if((nowTime
      
      count_passes--;
      cur_idx--;
    } // while
    
   
  } // if onlineCheckTimer

  static uint16_t controllerStateTimer = 0;
  controllerStateTimer += dt;

  // чтобы часто не проверять состояние контроллера
  if(controllerStateTimer > NRF_CONTROLLER_STATE_CHECK_FREQUENCY)
  {
    controllerStateTimer = 0;
    
      // получаем текущее состояние контроллера
      ControllerState st = WORK_STATUS.GetState();
      if(bFirstCall || memcmp(&st,&(packet.state),sizeof(ControllerState)))
      {
        bFirstCall = false;
        // состояние контроллера изменилось, посылаем его в эфир
         memcpy(&(packet.state),&st,sizeof(ControllerState));
         packet.controller_id = UniDispatcher.GetControllerID();
         packet.packetType = RS485ControllerStatePacket;
         packet.crc8 = /*OneWire::*/crc8((const byte*) &packet,sizeof(packet)-1);
    
         #ifdef NRF_DEBUG
         DEBUG_LOG(F("Controller state changed, send it, payload size: "));
         DEBUG_LOGLN(String(sizeof(packet)));
         #endif // NRF_DEBUG
      
        // останавливаем прослушку
        radio->stopListening();
    
        // пишем наш скратч в эфир
        radio->write(&packet,sizeof(packet));
    
        // включаем прослушку
        radio->startListening();
    
        #ifdef NRF_DEBUG
        DEBUG_LOGLN(F("Controller state sent."));
        #endif // NRF_DEBUG
            
      } // if
      
  } // if(controllerStateTimer > NRF_CONTROLLER_STATE_CHECK_FREQUENCY

  // тут читаем данные из труб
  uint8_t pipe_num = 0; // из какой трубы пришло
  if(radio->available(&pipe_num))
  {
     static UniRawScratchpad nrfScratch;
     memset(&nrfScratch,0,sizeof(nrfScratch));
     
     // читаем скратч
     radio->read(&nrfScratch,sizeof(nrfScratch));

     #ifdef NRF_DEBUG
      DEBUG_LOG(F("Received the scratch via radio, payload size: "));
      DEBUG_LOGLN(String(sizeof(nrfScratch)));
     #endif

     byte checksum = /*OneWire::*/crc8((const byte*)&nrfScratch,sizeof(UniRawScratchpad)-1);
     if(checksum == nrfScratch.crc8)
     {
      #ifdef NRF_DEBUG
      DEBUG_LOGLN(F("Checksum OK"));
     #endif

      // проверяем, наш ли пакет
      if((nrfScratch.head.controller_id == BROADCAST_PACKET_ID || nrfScratch.head.controller_id == UniDispatcher.GetControllerID()) && (nrfScratch.head.packet_type >= uniSensorsClient && nrfScratch.head.packet_type <= uniSunControllerClient)
       && nrfScratch.head.rf_id == UniDispatcher.GetRFChannel())
      {
      #ifdef NRF_DEBUG
      DEBUG_LOGLN(F("Packet for us :)"));
      #endif  
        
          // наш пакет, продолжаем
          AbstractUniClient* client = UniFactory.GetClient(&nrfScratch);
          client->Register(&nrfScratch);
          client->Update(&nrfScratch,true,ssRadio);


          if(nrfScratch.head.packet_type == uniSensorsClient)
          {

          //Тут мы должны для всех датчиков модуля добавить в онлайн-очередь
          // время последнего получения значений и интервал между опросами модуля,
          // если таких данных ещё нету у нас.

              UniSensorsScratchpad* ourScrath = (UniSensorsScratchpad*) &(nrfScratch.data);       
                   
              for(byte i=0;i<MAX_UNI_SENSORS;i++)
              {
                byte type = ourScrath->sensors[i].type;
                if(type == NO_SENSOR_REGISTERED) // нет типа датчика 
                  continue;
            
                UniSensorType ut = (UniSensorType) type;
                
                if(ut == uniNone || ourScrath->sensors[i].index == NO_SENSOR_REGISTERED) // нет типа датчика
                  continue;
            
                // имеем тип датчика, можем проверять, есть ли он у нас в онлайновых
                byte result_index = 0;
                if(isInOnlineQueue(type,ourScrath->sensors[i].index,result_index))
                {
                  // он уже был онлайн, надо сбросить таймер опроса
                  NRFQueueItem* qi = &(sensorsOnlineQueue[result_index]);
                  qi->gotLastDataAt = millis();
                }
                else
                {
                  // датчик не был в онлайн очереди, надо его туда добавить
                  NRFQueueItem qi;
                  qi.sensorType = type;
                  qi.sensorIndex = ourScrath->sensors[i].index;
                  qi.queryInterval = ourScrath->query_interval_min*60 + ourScrath->query_interval_sec;
                  qi.gotLastDataAt = millis();

                  sensorsOnlineQueue.push_back(qi);
                } // else
                
              } // for

        } // nrfScratch.head.packet_type == uniSensorsClient

      #ifdef NRF_DEBUG
      DEBUG_LOGLN(F("Controller data updated."));
      #endif  

      }
      #ifdef NRF_DEBUG
      else 
      {
        DEBUG_LOG(F("Unknown packet for controller #"));
        DEBUG_LOGLN(String(nrfScratch.head.controller_id));
      }
      #endif       
       
      
     } // checksum
      #ifdef NRF_DEBUG
      else
      {
        DEBUG_LOGLN(F("Checksum FAIL"));
      }
     #endif
    
    
  } // available
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniNRFGate::SetChannel(byte channel)
{
  if(!nRFInited)
    return;
    
  radio->stopListening();
  radio->setChannel(channel);
  radio->startListening();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
int UniNRFGate::ScanChannel(byte channel)
{
  
  if(!nRFInited)
    return -1;

    NrfBinding bnd = HardwareBinding->GetNrfBinding();

    int level = 0;

    radio->stopListening();
    radio->setAutoAck(bnd.AutoAckInverted == 1 ? true : false);
    radio->setChannel(channel);   
    radio->startListening();

    for(int i=0;i<1000;i++)
    {
        if(radio->testRPD())
          level++;

         delayMicroseconds(50);
    }

    radio->stopListening();

    radio->setAutoAck(bnd.AutoAckInverted == 1 ? false : true);
    radio->setChannel(UniDispatcher.GetRFChannel());   
    radio->startListening();

    return level;
    
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniNRFGate::initNRF()
{

  NrfBinding bnd = HardwareBinding->GetNrfBinding();
  if(bnd.CEPin == UNBINDED_PIN || bnd.CSNPin == UNBINDED_PIN) // нет привязки к пинам контроллера
  {
    return;
  }

  // проверяем пины на безопасность
  #ifndef DISABLE_NRF_CONFIGURE
  if(EEPROMSettingsModule::SafePin(bnd.CEPin) && EEPROMSettingsModule::SafePin(bnd.CSNPin))
  #endif
  {
    radio = new RF24(bnd.CEPin,bnd.CSNPin);
  }

  if(!radio) // ничего не получилось
  {
    return;
  }
  
  #ifdef NRF_DEBUG
  printf_begin();
  #endif

  
  // инициализируем nRF
  nRFInited = radio->begin();

  if(nRFInited)
  {

    WORK_STATUS.PinMode(bnd.CSNPin,OUTPUT,false);
    WORK_STATUS.PinMode(bnd.CEPin,OUTPUT,false);
    WORK_STATUS.PinMode(MOSI,OUTPUT,false);
    WORK_STATUS.PinMode(MISO,INPUT,false);
    WORK_STATUS.PinMode(SCK,OUTPUT,false);


    delay(200); // чуть-чуть подождём
  
    radio->setDataRate(RF24_1MBPS);
    radio->setPALevel(RF24_PA_MAX);
    radio->setChannel(UniDispatcher.GetRFChannel());
    radio->setRetries(15,15);
    radio->setPayloadSize(RADIO_PAYLOAD_SIZE); // у нас 30 байт на пакет
    radio->setCRCLength(RF24_CRC_16);

    radio->setAutoAck(bnd.AutoAckInverted ? false : true);

  
    // открываем трубу, в которую будем писать состояние контроллера
    radio->openWritingPipe(controllerStatePipe);
  
    // открываем все пять труб на прослушку
    readFromPipes();
  
    radio->startListening(); // начинаем слушать
    
    #ifdef NRF_DEBUG
      radio->printDetails();
    #endif

  } // nRFInited
  else
  {
  #ifdef NRF_DEBUG
    DEBUG_LOGLN(F("UNABLE TO INIT NRF!!!"));
  #endif
  }
  

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_NRF_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#include "LoRa.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------
UniLoRaGate::UniLoRaGate()
{
  bFirstCall = true;
  loRaInited = false;
  rssi = -120;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool UniLoRaGate::isInOnlineQueue(byte sensorType,byte sensorIndex, byte& result_index)
{
  for(size_t i=0;i<sensorsOnlineQueue.size();i++)
    if(sensorsOnlineQueue[i].sensorType == sensorType && sensorsOnlineQueue[i].sensorIndex == sensorIndex)
    {
      result_index = i;
      return true;
    }
  return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniLoRaGate::Setup()
{  

  LoraBinding bnd = HardwareBinding->GetLoraBinding();

  if(bnd.LinkType != linkUnbinded)
  {
    if(bnd.LinkType == linkDirect)
    {
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          #ifndef DISABLE_LORA_CONFIGURE
          if(EEPROMSettingsModule::SafePin(bnd.PowerPin))
          #else
          if(bnd.PowerPin > 1) // prevent Serial locking
          #endif
          {
            WORK_STATUS.PinMode(bnd.PowerPin,OUTPUT);
            WORK_STATUS.PinWrite(bnd.PowerPin,bnd.Level);
          }
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.PowerPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.PowerPin,bnd.Level);      
        }
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
     #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        if(bnd.PowerPin != UNBINDED_PIN)
        {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.PowerPin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.PowerPin,bnd.Level);      
        }
     #endif 
    }
    
  } // if(bnd.LinkType != linkUnbinded)

  initLoRa();
  
  memset(&packet,0,sizeof(packet));
  ControllerState st = WORK_STATUS.GetState();
  // копируем состояние контроллера к нам
  memcpy(&(packet.state),&st,sizeof(ControllerState));

  #ifdef LORA_DEBUG
  DEBUG_LOGLN(F("LoRa: setup done."));
  #endif // LORA_DEBUG  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniLoRaGate::Update()
{
  if(!loRaInited)
  {
    return;
  }

  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;    

  static uint16_t onlineCheckTimer = 0;
  onlineCheckTimer += dt;
  if(onlineCheckTimer > 5000)
  {
    onlineCheckTimer = 0;

    //Тут, раз в пять секунд - мы должны проверять, не истёк ли интервал
    // получения показаний с датчиков, показания с которых были получены ранее.
    // если интервал истёк - мы должны выставить датчику показания "нет данных"
    // и удалить его из очереди.

    byte count_passes = sensorsOnlineQueue.size();
    byte cur_idx = count_passes-1;

    unsigned long nowTime = millis();

    // проходим от хвоста до головы
    while(count_passes)
    {
      LoRaQueueItem* qi = &(sensorsOnlineQueue[cur_idx]);

      // вычисляем интервал в миллисекундах
      unsigned long query_interval = qi->queryInterval*1000;
      
      // смотрим, не истёк ли интервал с момента последнего опроса
      if((nowTime - qi->gotLastDataAt) > (query_interval+3000) )
      {
        
        // датчик не откликался дольше, чем интервал между опросами плюс дельта в 3 секунды,
        // надо ему выставить показания "нет данных"
          byte sType = qi->sensorType;
          byte sIndex = qi->sensorIndex;
        
          UniDispatcher.AddUniSensor((UniSensorType)sType,sIndex);

            // проверяем тип датчика, которому надо выставить "нет данных"
            switch(qi->sensorType)
            {
              case uniTemp:
              {
                // температура
                Temperature t;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&t);
                } // if
              }
              break;

              case uniHumidity:
              {
                // влажность
                Humidity h;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&h);

                  if(states.State2)
                    states.State2->Update(&h);
                } // if                        
              }
              break;

              case uniLuminosity:
              {
                // освещённость
                long lum = NO_LUMINOSITY_DATA;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&lum);
                } // if                        
                
                
              }
              break;

              case uniSoilMoisture: // влажность почвы
              case uniPH: // показания pH
              {
                
                Humidity h;
                // получаем состояния
                UniSensorState states;
                if(UniDispatcher.GetRegisteredStates((UniSensorType)sType,sIndex,states))
                {
                  if(states.State1)
                    states.State1->Update(&h);
                } // if                        
                
              }
              break;

              case uniPinsMap:
              break;
              
            } // switch

        // теперь удаляем оффлайн-датчик из очереди
        sensorsOnlineQueue.pop();
        
      } // if((nowTime
      
      count_passes--;
      cur_idx--;
    } // while
    
   
  } // if onlineCheckTimer



  static uint32_t lastPacketSeenAt = 0;
  if(millis() - lastPacketSeenAt >= 60000ul) // если в течение минуты не получали пакетов - сбрасываем уровень сигнала
  {
    rssi = -120;
	lastPacketSeenAt = millis();
  }

  // тут читаем данные из LoRa
  int packetSize = LoRa.parsePacket();
  if(packetSize >= RADIO_PAYLOAD_SIZE)
  {
     static UniRawScratchpad loRaScratch;
     // читаем скратч
     LoRa.readBytes((uint8_t*) &loRaScratch,RADIO_PAYLOAD_SIZE);

     rssi = LoRa.packetRssi();

     #ifdef LORA_DEBUG
      DEBUG_LOG(F("LoRa: Received the scratch via radio with RSSI: "));
      DEBUG_LOGLN(String(rssi));
     #endif

     byte checksum = /*OneWire::*/crc8((const byte*)&loRaScratch,sizeof(UniRawScratchpad)-1);
     if(checksum == loRaScratch.crc8)
     {
      #ifdef LORA_DEBUG
      DEBUG_LOGLN(F("LoRa: Checksum OK"));
     #endif

      // проверяем, наш ли пакет
      if((loRaScratch.head.controller_id == BROADCAST_PACKET_ID || loRaScratch.head.controller_id == UniDispatcher.GetControllerID()) && (loRaScratch.head.packet_type >= uniSensorsClient && loRaScratch.head.packet_type <= uniWaterTankClient))
      {
      #ifdef LORA_DEBUG
      DEBUG_LOGLN(F("LoRa: Packet for us :)"));
      #endif  
	  
			lastPacketSeenAt = millis();
	  
          // наш пакет, продолжаем
          AbstractUniClient* client = UniFactory.GetClient(&loRaScratch);
          client->Register(&loRaScratch);
          client->Update(&loRaScratch,true,ssRadio);

          //Тут мы должны для всех датчиков модуля добавить в онлайн-очередь
          // время последнего получения значений и интервал между опросами модуля,
          // если таких данных ещё нету у нас.

          if(loRaScratch.head.packet_type == uniSensorsClient)
          {

              UniSensorsScratchpad* ourScrath = (UniSensorsScratchpad*) &(loRaScratch.data);       
                   
              for(byte i=0;i<MAX_UNI_SENSORS;i++)
              {
                byte type = ourScrath->sensors[i].type;
                if(type == NO_SENSOR_REGISTERED) // нет типа датчика 
                  continue;
            
                UniSensorType ut = (UniSensorType) type;
                
                if(ut == uniNone || ourScrath->sensors[i].index == NO_SENSOR_REGISTERED) // нет типа датчика
                  continue;
            
                // имеем тип датчика, можем проверять, есть ли он у нас в онлайновых
                byte result_index = 0;
                if(isInOnlineQueue(type,ourScrath->sensors[i].index,result_index))
                {
                  // он уже был онлайн, надо сбросить таймер опроса
                  LoRaQueueItem* qi = &(sensorsOnlineQueue[result_index]);
                  qi->gotLastDataAt = millis();
                }
                else
                {
                  // датчик не был в онлайн очереди, надо его туда добавить
                  LoRaQueueItem qi;
                  qi.sensorType = type;
                  qi.sensorIndex = ourScrath->sensors[i].index;
                  qi.queryInterval = ourScrath->query_interval_min*60 + ourScrath->query_interval_sec;
                  qi.gotLastDataAt = millis();

                  sensorsOnlineQueue.push_back(qi);
                } // else
                
              } // for


        } // loRaScratch.head.packet_type == uniSensorsClient

      #ifdef LORA_DEBUG
      DEBUG_LOGLN(F("LoRa: Controller data updated."));
      #endif  

      }
      #ifdef LORA_DEBUG
      else 
      {
        DEBUG_LOG(F("LoRa: Unknown packet for controller #"));
        DEBUG_LOGLN(String(loRaScratch.head.controller_id));
      }
      #endif       
       
      
     } // checksum
      #ifdef LORA_DEBUG
      else
      {
        DEBUG_LOGLN(F("LoRa: Checksum FAIL"));
      }
     #endif
    
    
  } // available
  #ifdef LORA_DEBUG
    else if(packetSize > 0)
    {
      DEBUG_LOG(F("LoRa: packet too small: "));
      DEBUG_LOGLN(String(packetSize));
    }
   #endif  
  
  static uint16_t controllerStateTimer = 0;
  controllerStateTimer += dt;

  // чтобы часто не проверять состояние контроллера
  if(controllerStateTimer > LORA_CONTROLLER_STATE_CHECK_FREQUENCY)
  {
    controllerStateTimer = 0;
    
      // получаем текущее состояние контроллера
      ControllerState st = WORK_STATUS.GetState();
      if(bFirstCall || memcmp(&st,&(packet.state),sizeof(ControllerState)))
      {
        bFirstCall = false;
        // состояние контроллера изменилось, посылаем его в эфир
         memcpy(&(packet.state),&st,sizeof(ControllerState));
         packet.controller_id = UniDispatcher.GetControllerID();
         packet.packetType = RS485ControllerStatePacket;         
         packet.crc8 = /*OneWire::*/crc8((const byte*) &packet,sizeof(packet)-1);
    
         #ifdef LORA_DEBUG
         DEBUG_LOGLN(F("LoRa: Controller state changed, send it..."));
         #endif // LORA_DEBUG
      
        LoRa.beginPacket();
    
        // пишем наш скратч в эфир
        LoRa.write((uint8_t*) &packet,RADIO_PAYLOAD_SIZE);
    
        // включаем прослушку
        LoRa.endPacket();
        LoRa.receive();
    
        #ifdef LORA_DEBUG
        DEBUG_LOGLN(F("LoRa: Controller state sent."));
        #endif // LORA_DEBUG
            
      } // if
      
  } // if(controllerStateTimer > LORA_CONTROLLER_STATE_CHECK_FREQUENCY
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniLoRaGate::sendWaterTankSettingsPacket()
{
  #ifdef USE_WATER_TANK_MODULE

   if(!loRaInited)
   {

        #ifdef WATER_TANK_MODULE_DEBUG
         Serial.println(F("LoRa: Send WATER TANK settings packet, LORA NOT INITED!!!"));
         #endif // WATER_TANK_MODULE_DEBUG    
      return;
   }
  
    NRFWaterTankSettingsPacket waterTankSettingsPacket;
    memset(&waterTankSettingsPacket,0,sizeof(waterTankSettingsPacket));
    waterTankSettingsPacket.controller_id = UniDispatcher.GetControllerID();
    waterTankSettingsPacket.packetType = RS485WaterTankSettings;     

    //TODO: ТУТ ЗАПОЛНЕНИЕ НАСТРОЕК !!!
    WaterTankBinding bnd = HardwareBinding->GetWaterTankBinding();
    
    waterTankSettingsPacket.level = bnd.Level; // ТУТ ИЗ НАСТРОЕК НАДО БРАТЬ УРОВЕНЬ СРАБАТЫВАНИЯ ДАТЧИКА !!!
    waterTankSettingsPacket.maxWorkTime = bnd.MaxWorkTime;
    waterTankSettingsPacket.distanceEmpty = bnd.DistanceEmpty;
    waterTankSettingsPacket.distanceFull = bnd.DistanceFull;

    waterTankSettingsPacket.crc8 = /*OneWire::*/crc8((const byte*) &waterTankSettingsPacket,sizeof(waterTankSettingsPacket)-1);

        #ifdef WATER_TANK_MODULE_DEBUG
         Serial.println(F("LoRa: Send WATER TANK settings packet..."));
         #endif // WATER_TANK_MODULE_DEBUG
      
        LoRa.beginPacket();
    
        // пишем наш скратч в эфир
        LoRa.write((uint8_t*) &waterTankSettingsPacket,RADIO_PAYLOAD_SIZE);
    
        // включаем прослушку
        LoRa.endPacket();
        LoRa.receive();
    
        #ifdef WATER_TANK_MODULE_DEBUG
        Serial.println(F("LoRa: WATER TANK settings packet sent."));
        #endif // WATER_TANK_MODULE_DEBUG       
                    
  
  #endif // USE_WATER_TANK_MODULE
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniLoRaGate::sendFillTankCommand(bool on)
{
  #ifdef USE_WATER_TANK_MODULE

   if(!loRaInited)
   {
        #ifdef WATER_TANK_MODULE_DEBUG
         Serial.println(F("LoRa: Send WATER TANK COMMAND packet, LORA NOT INITED!!!"));
         #endif // WATER_TANK_MODULE_DEBUG    
      return;
   }
  
  
    NRFWaterTankExecutionPacket waterTankExecutionPacket;
    memset(&waterTankExecutionPacket,0,sizeof(waterTankExecutionPacket));

    waterTankExecutionPacket.controller_id = UniDispatcher.GetControllerID();
    waterTankExecutionPacket.packetType = RS485WaterTankCommands;         
         
    waterTankExecutionPacket.valveCommand = on ? 1 : 0;

    waterTankExecutionPacket.crc8 = /*OneWire::*/crc8((const byte*) &waterTankExecutionPacket,sizeof(waterTankExecutionPacket)-1);

        #ifdef WATER_TANK_MODULE_DEBUG
         Serial.print(F("LoRa: Send WATER TANK command packet with valve command: "));
         Serial.println(on);
         #endif // WATER_TANK_MODULE_DEBUG
      
        LoRa.beginPacket();
    
        // пишем наш скратч в эфир
        LoRa.write((uint8_t*) &waterTankExecutionPacket,RADIO_PAYLOAD_SIZE);
    
        // включаем прослушку
        LoRa.endPacket();
        LoRa.receive();
    
        #ifdef WATER_TANK_MODULE_DEBUG
        Serial.println(F("LoRa: WATER TANK command packet sent."));
        #endif // WATER_TANK_MODULE_DEBUG       
                
  #endif // USE_WATER_TANK_MODULE
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void UniLoRaGate::initLoRa()
{

  LoraBinding bnd = HardwareBinding->GetLoraBinding();
  if(bnd.SSPin == UNBINDED_PIN || bnd.ResetPin == UNBINDED_PIN) // нет привязки к пинам контроллера
  {
    return; 
  }

  #ifndef DISABLE_LORA_CONFIGURE
  if(!(EEPROMSettingsModule::SafePin(bnd.SSPin) && EEPROMSettingsModule::SafePin(bnd.ResetPin))) // пины небезопасны
  {
    return;     
  }
  #endif

    #ifdef LORA_DEBUG
    DEBUG_LOGLN(F("LoRa: init..."));
    #endif  
  
  // инициализируем LoRa
  LoRa.setPins(bnd.SSPin, bnd.ResetPin,-1);
  loRaInited = LoRa.begin(1000000ul*bnd.Frequency);

  if(loRaInited)
  {

    WORK_STATUS.PinMode(bnd.SSPin,OUTPUT,false);
    WORK_STATUS.PinMode(bnd.ResetPin,OUTPUT,false);
    WORK_STATUS.PinMode(MOSI,OUTPUT,false);
    WORK_STATUS.PinMode(MISO,INPUT,false);
    WORK_STATUS.PinMode(SCK,OUTPUT,false);
  
    LoRa.setTxPower(bnd.TXPower);
    LoRa.receive(); // начинаем слушать
  
    #ifdef LORA_DEBUG
    DEBUG_LOGLN(F("LoRa: inited."));
    #endif    

  } // loRaInited
  #ifdef LORA_DEBUG
  else
  {
    DEBUG_LOGLN(F("LoRa: INIT FAIL !!!"));
  }
  #endif
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LORA_GATE
//-------------------------------------------------------------------------------------------------------------------------------------------------------

