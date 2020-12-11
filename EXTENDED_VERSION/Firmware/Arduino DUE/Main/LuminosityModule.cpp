#include "LuminosityModule.h"
#include "ModuleController.h"
#include "Max44009.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
BH1750Support::BH1750Support()
{

}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::begin(BH1750Address addr, BH1750Mode mode)
{
  deviceAddress = addr;
  writeByte(BH1750PowerOn); // включаем датчик
  delay(10);
  ChangeMode(mode); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::ChangeMode(BH1750Mode mode) // смена режима работы
{
   currentMode = mode; // сохраняем текущий режим опроса
   writeByte((uint8_t)currentMode);
  delay(10);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::ChangeAddress(BH1750Address newAddr)
{
  if(newAddr != deviceAddress) // только при смене адреса включаем датчик
  { 
    deviceAddress = newAddr;
    
    writeByte(BH1750PowerOn); // включаем датчик
    delay(10);
    ChangeMode(currentMode); // меняем режим опроса на текущий
  } // if
}
//--------------------------------------------------------------------------------------------------------------------------------------
void BH1750Support::writeByte(uint8_t toWrite) 
{
  Wire.beginTransmission(deviceAddress);
  BH1750_WIRE_WRITE(toWrite);
  Wire.endTransmission(true);
}
//--------------------------------------------------------------------------------------------------------------------------------------
long BH1750Support::GetCurrentLuminosity() 
{
  long curLuminosity = NO_LUMINOSITY_DATA;

 if(Wire.requestFrom(deviceAddress, 2) == 2)// ждём два байта
 {
  // читаем два байта
  curLuminosity = BH1750_WIRE_READ();
  curLuminosity <<= 8;
  curLuminosity |= BH1750_WIRE_READ();
  curLuminosity = curLuminosity/1.2; // конвертируем в люксы
 }


  return curLuminosity;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LuminosityModule::Setup()
{

  LightBinding bnd = HardwareBinding->GetLightBinding();

   bool bh1750NotFirst = false;
   bool max44009NotFirst = false;

   uint8_t sensorCounter = 0;
 
  for(size_t i=0;i<sizeof(bnd.Sensors)/sizeof(bnd.Sensors[0]);i++)
  {
    if(sensorCounter >= sizeof(lightSensors)/sizeof(lightSensors[0]))
    {
      break; // достигли конца нашего списка датчиков
    }
    
    if(bnd.Sensors[i] != NO_LIGHT_SENSOR) // если в привязках указано, что есть датчик
    {
        switch(bnd.Sensors[i])
        {
           case BH1750_SENSOR:
           {
              State.AddState(StateLuminosity,sensorCounter); // добавляем в состояние нужные индексы датчиков
            
              BH1750Support* bh = new BH1750Support;
              if(bh1750NotFirst)
                bh->begin(BH1750Address2);
              else
                bh->begin();
    
              bh1750NotFirst = true;
    
              lightSensors[sensorCounter] = bh;              
              sensorCounter++;
           }
           break;
    
           case MAX44009_SENSOR:
           {
              State.AddState(StateLuminosity,sensorCounter); // добавляем в состояние нужные индексы датчиков
                          
              Max44009* bh = new Max44009;
              if(max44009NotFirst)
                bh->begin(MAX44009_ADDRESS2);
              else
                bh->begin(MAX44009_ADDRESS1);
    
              max44009NotFirst = true;
    
              lightSensors[sensorCounter] = bh;              
              sensorCounter++;
           }
           break;
          
        } // switch

    } // if(bnd.Sensors[i] != NO_LIGHT_SENSOR)
    
  } // for

  
  
  // настройка модуля тут

  flags.workMode = lightAutomatic; // автоматический режим работы
  flags.bRelaysIsOn = false; // все реле выключены
  flags.bLastRelaysIsOn = false; // состояние не изменилось
  
  SAVE_STATUS(LIGHT_STATUS_BIT,0); // сохраняем, что досветка выключена
  SAVE_STATUS(LIGHT_MODE_BIT,1); // сохраняем, что мы в автоматическом режиме работы

// если указано - использовать диод индикации ручного режима работы - настраиваем его
  canUseBlinker = false;
  DiodesBinding dbnd = HardwareBinding->GetDiodesBinding();
  if(dbnd.LinkType != linkUnbinded)
  {
    if(dbnd.LinkType == linkDirect)
    {
        if(dbnd.LightManualModeDiodePin != UNBINDED_PIN && dbnd.LightManualModeDiodePin > 1
        #ifndef DISABLE_DIODES_CONFIGURE
        && EEPROMSettingsModule::SafePin(dbnd.LightManualModeDiodePin)
        #endif
        )
        {
          canUseBlinker = true;
        }
    }
    else
    if(dbnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
         if(dbnd.LightManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }
    else
    if(dbnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
         if(dbnd.LightManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }    
    
  } // if(bnd.LinkType != linkUnbinded)
  
  if(canUseBlinker)
  {
      blinker.begin(dbnd.LightManualModeDiodePin);  // настраиваем блинкер на нужный пин
      blinker.blink(); // и гасим его по умолчанию
  }


  // настраиваем выхода
  if(bnd.LinkType != linkUnbinded) // есть режим управления через что-то
  {      
      for(size_t i=0;i<sizeof(bnd.LampRelays)/sizeof(bnd.LampRelays[0]);i++)
      {
          if(bnd.LampRelays[i] != UNBINDED_PIN) // есть привязка к пину контроллера
          {              
              if(bnd.LinkType == linkDirect)
              {
                    if(EEPROMSettingsModule::SafePin(bnd.LampRelays[i]))
                    {
                      WORK_STATUS.PinMode(bnd.LampRelays[i],OUTPUT);
                      WORK_STATUS.PinWrite(bnd.LampRelays[i],!bnd.Level);                
                    }
              }
              else
              if(bnd.LinkType == linkMCP23S17)
              {
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                
                  WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.LampRelays[i],OUTPUT);
                  WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.LampRelays[i],!bnd.Level);
                
                #endif
              }
              else
              if(bnd.LinkType == linkMCP23017)
              {
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                
                  WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.LampRelays[i],OUTPUT);
                  WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.LampRelays[i],!bnd.Level);
                
                #endif
              }

          } // if(bnd.LampRelays[i] != UNBINDED_PIN)
      } // for
  } // if(bnd.LinkType != linkUnbinded)

  // тут - мы должны для всех 8 реле установить в статусе контроллера режим ВЫКЛ, поскольку у нас - привязки к железу контроллера,
  // однако не стоит забывать о выносных модулях. Канал реле может быть не подключен к железу контроллера, однако - использоваться
  // в универсальном модуле.
  for(size_t i=0;i<sizeof(bnd.LampRelays)/sizeof(bnd.LampRelays[0]);i++)
  {
     WORK_STATUS.SaveLightChannelState(i,!LIGHT_RELAY_ON);
  }
  
       
 }
//--------------------------------------------------------------------------------------------------------------------------------------
void LuminosityModule::Update()
{ 
  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;


  // обновление модуля тут
  if(canUseBlinker)
  {
    blinker.update();
  }

 // обновляем состояние всех реле управления досветкой
 if(flags.bLastRelaysIsOn != flags.bRelaysIsOn) // только если состояние с момента последнего опроса изменилось
 {
    flags.bLastRelaysIsOn = flags.bRelaysIsOn; // сохраняем текущее

    LightBinding bnd = HardwareBinding->GetLightBinding();
    if(bnd.LinkType != linkUnbinded) // есть режим управления через что-то
    {
      for(size_t i=0;i<sizeof(bnd.LampRelays)/sizeof(bnd.LampRelays[0]);i++)
      {
          if(bnd.LampRelays[i] != UNBINDED_PIN)
          {

              if(bnd.LinkType == linkDirect)
              {
                    if(EEPROMSettingsModule::SafePin(bnd.LampRelays[i]))
                    {
                      WORK_STATUS.PinWrite(bnd.LampRelays[i],flags.bRelaysIsOn ? bnd.Level : !bnd.Level);                
                    }
              }
              else
              if(bnd.LinkType == linkMCP23S17)
              {
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                
                  WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.LampRelays[i],flags.bRelaysIsOn ? bnd.Level : !bnd.Level);
                
                #endif
              }
              else
              if(bnd.LinkType == linkMCP23017)
              {
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                
                  WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.LampRelays[i],flags.bRelaysIsOn ? bnd.Level : !bnd.Level);
                
                #endif
              }
          } // if(bnd.LampRelays[i] != UNBINDED_PIN)
      } // for
      
    } // if(bnd.LinkType != linkUnbinded)

    // тут - мы должны для всех 8 реле установить в статусе контроллера режим ВКЛ или ВЫКЛ, поскольку у нас - привязки к железу контроллера,
    // однако не стоит забывать о выносных модулях. Канал реле может быть не подключен к железу контроллера, однако - использоваться
    // в универсальном модуле.
    // тут опять - константа, поскольку, основываясь на LIGHT_RELAY_ON - внутри функции SaveLightChannelState делается вывод - включен канал
    // или выключен. Однако, через конфигуратор можно поменять уровень срабатывания, поэтому мы тут оперируем константой.
    for(size_t i=0;i<sizeof(bnd.LampRelays)/sizeof(bnd.LampRelays[0]);i++)
    {
       WORK_STATUS.SaveLightChannelState(i,flags.bRelaysIsOn ? LIGHT_RELAY_ON : !LIGHT_RELAY_ON);
    }    
       
 } // if


  lastUpdateCall += dt;
  if(lastUpdateCall < LUMINOSITY_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
  {
    return;
  }
  else
  {
    lastUpdateCall = 0;
  }


    LightBinding bnd = HardwareBinding->GetLightBinding();
    uint8_t sensorCounter = 0;
    
    for(size_t i=0;i<sizeof(bnd.Sensors)/sizeof(bnd.Sensors[0]);i++)
    {
        if(sensorCounter >= sizeof(lightSensors)/sizeof(lightSensors[0]))
        {
          break; // достигли конца нашего списка датчиков
        }      
        
        if(bnd.Sensors[i] != NO_LIGHT_SENSOR)
        {
            long lum = NO_LUMINOSITY_DATA;
            bool knownSensor = false;

            switch(bnd.Sensors[i])
            {
               case BH1750_SENSOR:
               {
                  BH1750Support* bh = (BH1750Support*) lightSensors[sensorCounter];
                  lum = bh->GetCurrentLuminosity();
                  knownSensor = true;
               }
               break;
        
               case MAX44009_SENSOR:
               {
                  Max44009* bh = (Max44009*) lightSensors[sensorCounter];
                  float curLum = bh->readLuminosity();
                  knownSensor = true;
                  
                 if(curLum < 0)
                  lum = NO_LUMINOSITY_DATA;
                else
                {
                  unsigned long ulLum = (unsigned long) curLum;
                  if(ulLum > 65535)
                    ulLum = 65535;
    
                  lum = ulLum;
                }
            
               }
               break;
      
            } // switch 

              if(knownSensor)
              {
                  if(bnd.AveragingEnabled == 1) // усреднение включено
                  {
                    lum = GetAverageValue(lum,sensorCounter);
                  }
                  
                  if(bnd.HarboringEnabled == 1) // загрубление включено
                  {
                    lum = ApplyHarboring(lum);
                  }
                  
                  State.UpdateState(StateLuminosity,sensorCounter,(void*)&lum);            
    
                  sensorCounter++;
                  
              } // if(knownSensor)
              
        } // if(bnd.Sensors[i] != NO_LIGHT_SENSOR)
    } // for
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
long LuminosityModule::GetAverageValue(long lum,uint8_t sensorIndex)
{
  if(lum != NO_LUMINOSITY_DATA) // помещаем в список, только если есть показания с датчика
  {
    averageLists[sensorIndex].push_back(lum);
  }
  else
  {
     // с датчика нет показаний, надо очистить список усреднения, поскольку мы не можем усреднять
     // актуальные значения с отсутствием показаний с датчика: как только показания с него пропадут,
     // мы тут же должны сигнализировать об этом, и не применять никакого усреднения.
     averageLists[sensorIndex].clear();
     return lum;
  }

  // мы отбрасываем 2 граничных показания, а среднее можно искать только минимум по двум показаниям.
  // также, если с датчика постоянно не будет данных - список будет пуст, и мы просто вернём переданное значение.
  if(averageLists[sensorIndex].size() < 4)
    return lum;


  LightBinding bnd = HardwareBinding->GetLightBinding();

  // удаляем элементы, пока кол-во не будет равно кол-ву шагов
  if(averageLists[sensorIndex].size() > bnd.AveragingSamples)
  {
    // у нас на один элемент больше, чем надо, удаляем его из списка
    for(size_t i=1;i<averageLists[sensorIndex].size();i++)
    {
      averageLists[sensorIndex][i-1] = averageLists[sensorIndex][i];
    }
    averageLists[sensorIndex].pop();
  } // if

  // у нас есть список из N элементов, и их минимум 4. Надо найти там минимальное и максимальное значение,
  // и не учитывать их при расчётах.
  long minVal = 0;
  long maxVal = 0;
  uint32_t avg = 0;
  size_t steps = averageLists[sensorIndex].size();

  for(size_t i=0;i<steps;i++)
  {
    long curVal = averageLists[sensorIndex][i];
    
    maxVal = max(maxVal,curVal);
    
    if(i == 0)
      minVal = curVal;
    else
      minVal = min(minVal,curVal);

    avg += curVal;
  } // for

  // отнимаем минимальное и максимальное значение
  avg -= minVal;
  avg -= maxVal;

  // считаем среднее
  long result = avg/(steps-2); // отнимаем 2, поскольку мы отбросили граничные значения

  // готово, возвращаем
  return result;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
long LuminosityModule::ApplyHarboring(long lum)
{
  if(lum == NO_LUMINOSITY_DATA) // нет показаний
    return lum;

  LightBinding bnd = HardwareBinding->GetLightBinding();

  if(bnd.HarboringStep < 2)
    return lum;

  // приводим lum к дискретному шагу
  uint16_t halfStep = bnd.HarboringStep/2;
  uint16_t steps = lum/bnd.HarboringStep;
  uint16_t remain = lum%bnd.HarboringStep;

  if(remain >= halfStep)
    steps++;

  return (steps*bnd.HarboringStep);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  LuminosityModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) 
    PublishSingleton = UNKNOWN_COMMAND;
  
  uint8_t argsCnt = command.GetArgsCount();
  
  if(command.GetType() == ctSET) 
  {
      if(wantAnswer) 
        PublishSingleton = PARAMS_MISSED;

      if(argsCnt > 0)
      {
         String s = command.GetArg(0);
         if(s == STATE_ON) // CTSET=LIGHT|ON
         {
          
          // попросили включить досветку
          if(command.IsInternal() // если команда пришла от другого модуля
          && flags.workMode == lightManual)  // и мы в ручном режиме, то
          {
            // просто игнорируем команду, потому что нами управляют в ручном режиме
          } // if
           else
           {
              if(!command.IsInternal()) // пришла команда от пользователя,
              {
                flags.workMode = lightManual; // переходим на ручной режим работы

                if(canUseBlinker)
                {
                  blinker.blink(WORK_MODE_BLINK_INTERVAL); 
                }
              }

            if(!flags.bRelaysIsOn)
            {
              // значит - досветка была выключена и будет включена, надо записать в лог событие
              MainController->Log(this,s); 
            }

            flags.bRelaysIsOn = true; // включаем реле досветки
            
            PublishSingleton.Flags.Status = true;
            if(wantAnswer) 
              PublishSingleton = STATE_ON;


            
           } // else
 
            SAVE_STATUS(LIGHT_STATUS_BIT,flags.bRelaysIsOn ? 1 : 0); // сохраняем состояние досветки
            SAVE_STATUS(LIGHT_MODE_BIT,flags.workMode == lightAutomatic ? 1 : 0); // сохраняем режим работы досветки
          
         } // STATE_ON
         else
         if(s == STATE_OFF) // CTSET=LIGHT|OFF
         {
          // попросили выключить досветку
          if(command.IsInternal() // если команда пришла от другого модуля
          && flags.workMode == lightManual)  // и мы в ручном режиме, то
          {
            // просто игнорируем команду, потому что нами управляют в ручном режиме
           } // if
           else
           {
              if(!command.IsInternal()) // пришла команда от пользователя,
              {
                flags.workMode = lightManual; // переходим на ручной режим работы

                if(canUseBlinker)
                {
                  blinker.blink(WORK_MODE_BLINK_INTERVAL);
                }
              }

            if(flags.bRelaysIsOn)
            {
              // значит - досветка была включена и будет выключена, надо записать в лог событие
              MainController->Log(this,s); 
            }

            flags.bRelaysIsOn = false; // выключаем реле досветки
            
            PublishSingleton.Flags.Status = true;
            if(wantAnswer) 
              PublishSingleton = STATE_OFF;

            
           } // else

            SAVE_STATUS(LIGHT_STATUS_BIT,flags.bRelaysIsOn ? 1 : 0); // сохраняем состояние досветки
            SAVE_STATUS(LIGHT_MODE_BIT,flags.workMode == lightAutomatic ? 1 : 0); // сохраняем режим работы досветки

         } // STATE_OFF
         else
         if(s == WORK_MODE) // CTSET=LIGHT|MODE|AUTO, CTSET=LIGHT|MODE|MANUAL
         {
           // попросили установить режим работы
           if(argsCnt > 1)
           {
              s = command.GetArg(1);
              if(s == WM_MANUAL)
              {
                // попросили перейти в ручной режим работы
                flags.workMode = lightManual; // переходим на ручной режим работы

               if(canUseBlinker)
               {
                blinker.blink(WORK_MODE_BLINK_INTERVAL); 
               }
              }
              else
              if(s == WM_AUTOMATIC)
              {
                // попросили перейти в автоматический режим работы
                flags.workMode = lightAutomatic; // переходим на автоматический режим работы

                if(canUseBlinker)
                {
                  blinker.blink();
                }
              }

              PublishSingleton.Flags.Status = true;
              if(wantAnswer)
              {
                PublishSingleton = WORK_MODE; 
                PublishSingleton << PARAM_DELIMITER << (flags.workMode == lightAutomatic ? WM_AUTOMATIC : WM_MANUAL);
              }

            SAVE_STATUS(LIGHT_STATUS_BIT,flags.bRelaysIsOn ? 1 : 0); // сохраняем состояние досветки
            SAVE_STATUS(LIGHT_MODE_BIT,flags.workMode == lightAutomatic ? 1 : 0); // сохраняем режим работы досветки
              
           } // if (argsCnt > 1)
         } // WORK_MODE
         else
         if(s == F("DATA")) // установить значение на датчике, CTSET=LIGHT|DATA|idx|value
         {
            if (argsCnt > 2)
            {
                uint8_t sensorIndex = (uint8_t) atoi(command.GetArg(1));
                long sensorValue =  (long) atol(command.GetArg(2));
        
                uint8_t _lightCnt = State.GetStateCount(StateLuminosity);
                
                if(sensorIndex >= _lightCnt)
                {
                  uint8_t toAdd = (sensorIndex - _lightCnt) + 1;
        
                    for(uint8_t qa = 0; qa < toAdd; qa++)
                    {
                      State.AddState(StateLuminosity,_lightCnt + qa);
                    }
                }              
                     
                State.UpdateState(StateLuminosity,sensorIndex,(void*)&sensorValue);

                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
                  #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
                  OneState* dynReset = State.GetState(StateLuminosity,sensorIndex);
                  if(dynReset)
                  {
                    DynamicSensorsReset->Observe(dynReset);
                  }
                  #endif
                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
        
                PublishSingleton.Flags.Status = true;
                PublishSingleton = s;
                PublishSingleton << PARAM_DELIMITER << sensorIndex << PARAM_DELIMITER << REG_SUCC;              
              
            } // if (argsCnt > 2)
         } // DATA
         
      } // if(argsCnt > 0)
  }
  else
  if(command.GetType() == ctGET) //получить данные
  {

    if(!argsCnt) // нет аргументов, попросили дать показания с датчика
    {
      
      PublishSingleton.Flags.Status = true;
      if(wantAnswer) 
      {
         PublishSingleton = "";
        // запросили показания с датчиков. У нас должно выводиться минимум 2 показания,
        // для обеспечения нормальной работы конфигуратора. Поэтому мы добавляем недостающие показания
        // как показания NO_LUMINOSITY_DATA для тех датчиков, которых нет в прошивке.

         uint8_t _cnt = State.GetStateCount(StateLuminosity);
         uint8_t _written = 0;
         
         for(uint8_t i=0;i<_cnt;i++)
         {
            OneState* os = State.GetStateByOrder(StateLuminosity,i);
            if(os)
            {
              LuminosityPair lp = *os;

              if(_written > 0)
                PublishSingleton << PARAM_DELIMITER;

              PublishSingleton << lp.Current;
                
              _written++;
              
            } // if(os)
         } // for
/*
         // добиваем до двух датчиков минимум
         for(uint8_t i=_written; i<2;i++)
         {
           if(_written > 0)
            PublishSingleton << PARAM_DELIMITER;
            
           PublishSingleton << NO_LUMINOSITY_DATA;
           _written++;
         } // for
*/        
      }
    }
    else // есть аргументы
    {
       String s = command.GetArg(0);
       if(s == WORK_MODE) // запросили режим работы
       {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = WORK_MODE;
            PublishSingleton << PARAM_DELIMITER << (flags.workMode == lightAutomatic ? WM_AUTOMATIC : WM_MANUAL);
          }
          
       } // if(s == WORK_MODE)
       else
       if(s == LIGHT_STATE_COMMAND) // CTGET=LIGHT|STATE
       {
          if(wantAnswer)
          {
            PublishSingleton = LIGHT_STATE_COMMAND;
            PublishSingleton << PARAM_DELIMITER << (flags.bRelaysIsOn ? STATE_ON : STATE_OFF) << PARAM_DELIMITER << (flags.workMode == lightAutomatic ? WM_AUTOMATIC : WM_MANUAL);
          }
          
          PublishSingleton.Flags.Status = true;
             
       } // LIGHT_STATE_COMMAND
        
      // разбор других команд

      
    } // if(argsCnt > 0)
    
  } // if
 
 // отвечаем на команду
    MainController->Publish(this,command);
    
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LUMINOSITY_MODULE
