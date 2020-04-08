#include "DoorModule.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
#include "PowerManager.h"
//--------------------------------------------------------------------------------------------------------------------------------------
//#define DOOR_DEBUG // закомментировать для выключения отладки дверей
//--------------------------------------------------------------------------------------------------------------------------------------
// Door
//--------------------------------------------------------------------------------------------------------------------------------------
Door::Door()
{
  workMode = doorModeAutomatic;
  channel = 0;
  openFlag = false;
  onMyWay = false;
  timer = 0;
  hops = 0;
  tempSensorsCount = 0;
  canWorkWithSensor = false;
  controlTimer = 0;
  powerTimerDelay = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool Door::isBusy()
{
  return onMyWay;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::setSettings(DoorSettings* sett)
{
  settings = sett;
  canWorkWithSensor = false; // настройки изменились, в следующем вызове update мы обновим кол-во датчиков температуры, известное контроллеру
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::setup(uint8_t idx)
{
  // настраиваем выхода
  channel = idx;
  if(channel > 1)
  {
    channel = 0;
  }
  
  DoorBinding bnd = HardwareBinding->GetDoorBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin1 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin1))
      {
        WORK_STATUS.PinMode(bnd.Pin1,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin1,!bnd.Level);
      }
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin2))
      {
        WORK_STATUS.PinMode(bnd.Pin2,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin2,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin1,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin1,!bnd.Level);      
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin2,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin2,!bnd.Level);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin1,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin1,!bnd.Level);
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin2,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin2,!bnd.Level);
    }
    #endif
  } 

   // закрываемся при старте
   if(settings->active)
   {
    openFlag = true;
    close();
   }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::update()
{

 if(!canWorkWithSensor)
 {
    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
    {
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      tempSensorsCount = 0;  
    }

   canWorkWithSensor = true;
   
 } // !canWorkWithSensor
  
  if(onMyWay) // куда-то движемся
  {
    // проверяем, есть ли у нас задержка по включению питания?
    
    if(powerTimerDelay > 0)  // есть задержка по включению питания
    { 
      if(millis() - timer >= powerTimerDelay) // задержка по включению питания прошла
      { 
        setOutputs(pinState1,pinState2); // можем подавать на выходы питание        
        powerTimerDelay = 0;
        timer = millis(); // начинаем двигаться
      }
      else // задержка ещё не прошла, возвращаемся
      { 
          return;
      }
      
    } // if(powerTimerDelay > 0)

    
    if(millis() - timer >= 1000ul*settings->driveWorkTime)
    {
      timer = millis();
      hops--;
      if(!hops)
      {
        // приехали!
        onMyWay = false;

        // выключаем выхода
        DoorBinding bnd = HardwareBinding->GetDoorBinding(channel);
        setOutputs(!bnd.Level,!bnd.Level);

        // сообщаем менеджеру питания, что дверь закончила движение
        PowerManager.DoorMoveDone(channel);
        
#ifdef DOOR_DEBUG
        Serial.print("DOOR #");
        Serial.print(channel);
        Serial.println(" END MOVE.");
#endif
      }
      else
      {
        // ещё есть куда двигаться, меняем направление на противоположное
        pinState1 = !pinState1;
        pinState2 = !pinState2;

        setOutputs(pinState1,pinState2);
        
      } // else
      
    } // if(millis() - timer >= 1000ul*settings->driveWorkTime)
    
  } // if(onMyWay)


  if(doorModeManual == workMode || !settings->active) // в ручном режиме не контролируем параметры. Также не контролируем параметры, если модуль выключен.
  {
    return;
  }

  if(!onMyWay) // Тут контролируем параметры, но только тогда, когда дверь не движется
  {
    #ifdef USE_DS3231_REALTIME_CLOCK

    if(millis() - controlTimer >= 10000ul) // обновляем состояние не чаще, чем раз в 10 секунд
    {
      controlTimer = millis(); // сохраняем время последней проверки
#ifdef DOOR_DEBUG      
      Serial.print("DOOR #");
      Serial.print(channel);
      Serial.println(" WANT TO CHECK PARAMETERS...");
#endif          
    // можем контролировать параметры, есть часы реального времени в прошивке
    
    // получаем текущее время
      RealtimeClock rtc = MainController->GetClock();
      RTCTime currentTime = rtc.getTime();

      // формируем диапазоны, в минутах
      uint32_t startDia = settings->startWorkTime;
      uint32_t endDia = settings->endWorkTime;
      uint32_t nowMins = 60ul*currentTime.hour + currentTime.minute;

      if((nowMins >= startDia && nowMins < endDia)) // мы попадаем в рабочий диапазон, когда дверь надо держать открытой
      {
        // мы в рабочем диапазоне, и можем проверять температуру. При этом - по достижению порога температуры - мы должны открыть дверь.
        // если температура ниже порога на 5 градусов - закрываем дверь.
        // если датчик не привязан или с него нет данных - открываем дверь.
#ifdef DOOR_DEBUG  
            Serial.print("DOOR #");
            Serial.print(channel);
            Serial.println(" IN WORKING TIME!");
#endif        
                
        if(canWorkWithSensor) // можно работать с дверью на открытие. Надо проверить датчик температуры
        {
          bool shouldOpen = true; // флаг, что мы можем открывать двери. они открываются только по времени, если датчик температуры не привязан, или - с него нет данных.
          
          // уже можно работать с датчиком температуры
          const char* moduleName = "STATE";
          int8_t sensorIndex = settings->sensorIndex;

          if(sensorIndex >= 0) // если есть привязка к датчику
          {
              if(sensorIndex >= tempSensorsCount)
              {
                sensorIndex -= tempSensorsCount;
                moduleName = "HUMIDITY";
              }

              // получаем информацию с датчика
              AbstractModule* module = MainController->GetModuleByID(moduleName);
              
              if(module) // есть модуль
              {
                  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
                  
                  if(sensorState) // есть такой датчик в модуле
                  {
                    if(sensorState->HasData()) // есть данные на датчике
                    {
                        TemperaturePair tmp = *sensorState;
                        Temperature t = tmp.Current;

                        if(t.Value >= settings->minOpenTemp) // если текущая температура больше либо равна уставке - можем открывать
                        {
#ifdef DOOR_DEBUG                          
                              Serial.print("DOOR #");
                              Serial.print(channel);
                              Serial.print(", GOOD TEMP: ");              
                              Serial.print(t.Value);
                              Serial.print(", BORDER: ");
                              Serial.println(settings->minOpenTemp);
#endif
                          shouldOpen = true;
                        }
                        else // проверяем на нижний порог, он должен быть меньше на 5 градусов, чем уставка - и тогда мы закроем дверь
                        {
                          if(t.Value <= (settings->minOpenTemp - 5)) // текущая температура на 5 градусов ниже уставки
                          {
#ifdef DOOR_DEBUG                            
                              Serial.print("DOOR #");
                              Serial.print(channel);
                              Serial.print(", TOO SMALL TEMP: ");              
                              Serial.println(t.Value);
#endif                            
                            // надо закрыть, слишком холодно
                            shouldOpen = false;
                          }
                        }
                        
                    } // if(sensorState->HasData())
                    
                  } // if(sensorState)
              } // if(module)
          } // if(sensorIndex >= 0)

          // всё, получили все флаги, теперь можно открывать или закрывать дверь
          if(shouldOpen)
          {
             if(!openFlag)
             {
#ifdef DOOR_DEBUG              
                Serial.print("DOOR #");
                Serial.print(channel);
                Serial.println(" CAN BE OPEN!");  
#endif
                open();
             }
          }
          else
          {
            if(openFlag)
            {
#ifdef DOOR_DEBUG              
              Serial.print("DOOR #");
              Serial.print(channel);
              Serial.println(" SHOULD BE CLOSED!"); 
#endif
              close();
            }
          }
          
        } // canWorkWithSensor
        
      } // if((nowMins >= startDia && nowMins < endDia))
      else
      {
        // однозначно закрываем, т.к. не попадаем в рабочий диапазон по времени
        if(openFlag) // если дверь открыта, то закрываем
        {
#ifdef DOOR_DEBUG          
              Serial.print("DOOR #");
              Serial.print(channel);
              Serial.println(" SHOULD BE CLOSED BECAUSE OUT OF DIA"); 
#endif
          close();
        }
      }

    } // if(millis() - controlTimer >= 10000ul)
    
    #endif // USE_DS3231_REALTIME_CLOCK   
    
  } // if(!onMyWay)
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::switchMode(DoorWorkMode mode)
{
  workMode = mode;
}
//--------------------------------------------------------------------------------------------------------------------------------------
DoorWorkMode Door::getMode()
{
  return workMode;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::setOutputs(uint8_t state1, uint8_t state2)
{
#ifdef DOOR_DEBUG
  Serial.print("SET OUTPUTS FOR DOOR #");
  Serial.print(channel);
  Serial.print(": ");  
  Serial.print(state1);
  Serial.print(",");
  Serial.println(state2);
#endif

 DoorBinding bnd = HardwareBinding->GetDoorBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin1 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin1))
      {
        WORK_STATUS.PinWrite(bnd.Pin1,state1);
      }
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin2))
      {
        WORK_STATUS.PinWrite(bnd.Pin2,state2);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin1,state1);      
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin2,state2);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin1,state1);
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin2,state2);
    }
    #endif
  } 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::close()
{
  if(!openFlag)
  {
    return;
  }
  
#ifdef DOOR_DEBUG
  Serial.print("DOOR #");
  Serial.print(channel);
  Serial.println(" CLOSING...");
#endif
  
  DoorBinding bnd = HardwareBinding->GetDoorBinding(channel);
  
  if(bnd.DriveType == 0) // двухступенчатое управление
  {
    hops = 2; // надо сначала открыться, потом - закрыться
    pinState1 = bnd.Level;
    pinState2 = !bnd.Level;
  }
  else // прямое управление
  {
    hops = 1; // надо закрыться
    pinState1 = !bnd.Level;
    pinState2 = bnd.Level;    
  }
  
  ///setOutputs(pinState1,pinState2);
  
  openFlag = false;
  onMyWay = true;
  timer = millis();
  
  // спрашиваем менеджера питания, сколько нам ждать до подачи питания на выходы
  powerTimerDelay = PowerManager.DoorWantMove(channel);
  
  if(!powerTimerDelay) // нет задержки, сразу можем подавать сигналы на выходы
  {
    setOutputs(pinState1,pinState2);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Door::open()
{
  if(openFlag)
  {
    return;
  }
#ifdef DOOR_DEBUG
  Serial.print("DOOR #");
  Serial.print(channel);
  Serial.println(" OPENING...");
#endif

  DoorBinding bnd = HardwareBinding->GetDoorBinding(channel);

  if(bnd.DriveType == 0) // двухступенчатое управление
  {
    hops = 2; // надо сначала закрыться, потом - открыться
    pinState1 = !bnd.Level;
    pinState2 = bnd.Level;
  }
  else // прямое управление
  {
    hops = 1; // надо открыться
    pinState1 = bnd.Level;
    pinState2 = !bnd.Level;    
  }
  
  ///setOutputs(pinState1,pinState2);
    
  openFlag = true;
  onMyWay = true;  
  timer = millis();
  
  // спрашиваем менеджера питания, сколько нам ждать до подачи питания на выходы
  powerTimerDelay = PowerManager.DoorWantMove(channel);

  if(!powerTimerDelay) // нет задержки, сразу можем подавать сигналы на выходы
  {
    setOutputs(pinState1,pinState2);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool Door::isOpen()
{
  return openFlag;
}
//--------------------------------------------------------------------------------------------------------------------------------------
DoorModule* Doors = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
DoorModule::DoorModule() : AbstractModule("DOOR") 
{
  Doors = this;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::Setup()
{
  // настройка модуля тут
  ReloadDoorSettings(); // загружаем настройки дверей

  // тут настройки дверей
  const int cnt = sizeof(doors)/sizeof(doors[0]);
  for(size_t i=0;i<cnt;i++)
  {
    doors[i].setup(i);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::Update()
{ 
  // обновление модуля тут
  const int cnt = sizeof(doors)/sizeof(doors[0]);  
  for(size_t i=0;i<cnt;i++)
  {
    doors[i].update();
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DoorModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = PARAMS_MISSED;
        
  uint8_t argsCount = command.GetArgsCount();

  
  if(command.GetType() == ctSET) 
  {
    // установка свойств
    if(argsCount > 13) // CTSET=DOOR|drive time 1|min T 1|reserved 1|start Time 1|stop Time 1|sensor 1|active 1|drive time 2|min T 2|reserved 2|start Time 2|stop Time 2|sensor 2|active 2
    {
      // достаточно параметров
      int argCntr = 0;
      
      door1_Settings.driveWorkTime = atoi(command.GetArg(argCntr++));
      door1_Settings.minOpenTemp = atoi(command.GetArg(argCntr++));
      door1_Settings.reservedU16 = atoi(command.GetArg(argCntr++));
      door1_Settings.startWorkTime = atoi(command.GetArg(argCntr++));
      door1_Settings.endWorkTime = atoi(command.GetArg(argCntr++));
      door1_Settings.sensorIndex = atoi(command.GetArg(argCntr++));
      door1_Settings.active = atoi(command.GetArg(argCntr++)) ? true : false;

      door2_Settings.driveWorkTime = atoi(command.GetArg(argCntr++));
      door2_Settings.minOpenTemp = atoi(command.GetArg(argCntr++));
      door2_Settings.reservedU16 = atoi(command.GetArg(argCntr++));
      door2_Settings.startWorkTime = atoi(command.GetArg(argCntr++));
      door2_Settings.endWorkTime = atoi(command.GetArg(argCntr++));
      door2_Settings.sensorIndex = atoi(command.GetArg(argCntr++));
      door2_Settings.active = atoi(command.GetArg(argCntr++)) ? true : false;

      GlobalSettings* settings = MainController->GetSettings();
      settings->SetDoorSettings(0,door1_Settings);
      settings->SetDoorSettings(1,door2_Settings);

      PublishSingleton.Flags.Status = true;
      PublishSingleton = REG_SUCC;
      
    } // if(argsCount > 13)
    else
    if(argsCount > 1)
    {
        // установка других свойств
        String cmd = command.GetArg(0);
        
        if(cmd == F("MODE")) // установить режим работы - автоматический или ручной: CTSET=DOOR|MODE|idx|AUTO, CTSET=DOOR|MODE|idx|MANUAL
        {
          if(argsCount > 2)
          {
           int doorIdx = atoi(command.GetArg(1));
           if(doorIdx < 0)
           {
            doorIdx = 0;
           }
            const int cnt = sizeof(doors)/sizeof(doors[0]); 
            if(doorIdx > cnt-1)
            {
              doorIdx = cnt-1;
            }
           
           cmd = command.GetArg(2);
           
           if(cmd == WM_AUTOMATIC)
           {
            //В АВТОМАТИЧЕСКИЙ РЕЖИМ !!!
            doors[doorIdx].switchMode(doorModeAutomatic);
           }
           else
           {
            //В РУЧНОЙ РЕЖИМ !!!
            doors[doorIdx].switchMode(doorModeManual);
           }

            PublishSingleton.Flags.Status = true;
            PublishSingleton = command.GetArg(0);
            PublishSingleton << PARAM_DELIMITER << command.GetArg(0) << PARAM_DELIMITER << doorIdx << PARAM_DELIMITER << command.GetArg(2);
            
          } // if(argsCount > 2)
            
        } // if(cmd == F("MODE"))
        else
        if(cmd == F("OPEN") || cmd == F("CLOSE")) // открыть/закрыть дверь, CTSET=DOOR|OPEN|0, CTSET=DOOR|OPEN|1, CTSET=DOOR|OPEN|ALL, CTSET=DOOR|CLOSE|0, CTSET=DOOR|CLOSE|1, CTSET=DOOR|CLOSE|ALL
        {
            // открыть или закрыть дверь
          
            bool isOpen = cmd == F("OPEN");
            cmd = command.GetArg(1);
            bool all = cmd == F("ALL");
            uint8_t idx = cmd.toInt();

            if(idx < 0)
            {
              idx = 0;
            }

            const int cnt = sizeof(doors)/sizeof(doors[0]); 
            if(idx > cnt-1)
            {
              idx = cnt-1;
            }

            if(all) // все двери
            {
              //открыть-закрыть ВСЕ двери!
              for(size_t kk=0;kk<cnt;kk++)
              {
                doors[kk].switchMode(doorModeManual);
                
                if(isOpen)
                {
                  doors[kk].open();
                }
                else
                {
                  doors[kk].close();
                }
              }
              
            }
            else
            {
              //открыть-закрыть одну дверь!
              doors[idx].switchMode(doorModeManual);
                
              if(isOpen)
              {
                doors[idx].open();
              }
              else
              {
                doors[idx].close();
              }
            }

            PublishSingleton.Flags.Status = true;
            PublishSingleton = command.GetArg(0);
            PublishSingleton << PARAM_DELIMITER << command.GetArg(1);
            
        } // if(cmd == F("OPEN"))
        
      
    } // if(argsCount > 1)
  } // ctSET
  else
  if(command.GetType() == ctGET) 
  {
    if(argsCount > 0) // получение информации по режимам и т.п.
    {
       String cmd = command.GetArg(0);
       
       if(cmd == F("MODE")) // получение текущего режима работы, CTGET=DOOR|MODE|idx
       {
          if(argsCount > 1)
          { 
            String workMode;
            int idx = atoi(command.GetArg(1));
             if(idx < 0)
             {
              idx = 0;
             }
            const int cnt = sizeof(doors)/sizeof(doors[0]); 
            if(idx > cnt-1)
            {
              idx = cnt-1;
            }
            
            //ПОЛУЧЕНИЕ РЕЖИМА РАБОТЫ ДВЕРИ !!!
              if(doors[idx].getMode() == doorModeAutomatic)
              {
                workMode = WM_AUTOMATIC;
              }
              else
              {
                workMode = WM_MANUAL;
              }
             
            PublishSingleton.Flags.Status = true;
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER
            << idx
            << PARAM_DELIMITER
            << workMode
            ;
          } // if(argsCount > 1)
       }
       else
       if(cmd == F("STATE")) // получение статуса двери, CTGET=DOOR|STATE|idx
       {
          if(argsCount > 1)
          {
            int idx = atoi(command.GetArg(1));
             if(idx < 0)
             {
              idx = 0;
             }
             
            const int cnt = sizeof(doors)/sizeof(doors[0]); 
            if(idx > cnt-1)
            {
              idx = cnt-1;
            }
                  
            String doorState;

            //получение статуса двери!
            if(!doors[idx].isBusy())
            {
              if(doors[idx].isOpen())
              {
                doorState = F("OPEN");
              }
              else
              {
                doorState = F("CLOSE");
              }
            }
            else // дверь занята передвижением
            {
              if(doors[idx].isOpen())
              {
                doorState = F("OPENING");
              }
              else
              {
                doorState = F("CLOSING");
              }
            }
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER
             << idx
             << PARAM_DELIMITER
             << doorState
            ;
            
          } // if(argsCount > 1)
       } // STATE
    }
    else // CTGET=DOOR
    {
      // получение настроек
      PublishSingleton.Flags.Status = true;
      PublishSingleton = door1_Settings.driveWorkTime;
      PublishSingleton << PARAM_DELIMITER << door1_Settings.minOpenTemp
      << PARAM_DELIMITER << door1_Settings.reservedU16
      << PARAM_DELIMITER << door1_Settings.startWorkTime
      << PARAM_DELIMITER << door1_Settings.endWorkTime
      << PARAM_DELIMITER << door1_Settings.sensorIndex
      << PARAM_DELIMITER << door1_Settings.active
  
      << PARAM_DELIMITER << door2_Settings.driveWorkTime
      << PARAM_DELIMITER << door2_Settings.minOpenTemp
      << PARAM_DELIMITER << door2_Settings.reservedU16
      << PARAM_DELIMITER << door2_Settings.startWorkTime
      << PARAM_DELIMITER << door2_Settings.endWorkTime
      << PARAM_DELIMITER << door2_Settings.sensorIndex
      << PARAM_DELIMITER << door2_Settings.active
  
      ;
    } // !argsCount
    
  } // ctGET
  
  MainController->Publish(this,command);
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::ReloadDoorSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  door1_Settings = settings->GetDoorSettings(0);
  door2_Settings = settings->GetDoorSettings(1);

  doors[0].setSettings(&door1_Settings);
  doors[1].setSettings(&door2_Settings);
}
//--------------------------------------------------------------------------------------------------------------------------------------
DoorWorkMode DoorModule::GetDoorMode(uint8_t idx)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }
  
  return doors[idx].getMode();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::SetDoorMode(uint8_t idx, DoorWorkMode mode)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }

   doors[idx].switchMode(mode);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DoorModule::IsDoorOpen(uint8_t idx)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }

  return doors[idx].isOpen();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::CloseDoor(uint8_t idx)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }

  doors[idx].close();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DoorModule::OpenDoor(uint8_t idx)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }

  doors[idx].open();
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DoorModule::IsDoorBusy(uint8_t idx)
{
 const int cnt = sizeof(doors)/sizeof(doors[0]);
 
 if(idx > cnt-1)
  {
    idx = cnt-1;
  }

  return doors[idx].isBusy();  
}
//--------------------------------------------------------------------------------------------------------------------------------------


