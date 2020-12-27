#include "CO2Module.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CO2Module* CO2Control = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
volatile bool waitForLow = false;
volatile uint8_t co2Pin = 0;
volatile uint32_t co2Timer = 0;
volatile bool interruptAttached = false;
uint32_t requestByInterruptTimer = 0;
//--------------------------------------------------------------------------------------------------------------------------------------
void co2Measure() // обработчик прерывания для подсчёта CO2 через PWM
{
    if(!co2Pin)
    {
      return;
    }
    uint8_t level = digitalRead(co2Pin);
    
    if(level == HIGH) // смена с низкого на высокий уровень
    {
        if(!waitForLow) // ещё не ждём низкого уровня, надо взвести таймер
        {
          waitForLow = true;
          co2Timer = millis();
        }
    }
    else // смена с высокого на низкий уровень, надо проверять длительность импульса
    {
       
       if(waitForLow) // ждали низкого уровня, можно считать, сколько длился высокий
       {         
         uint32_t pwmDuty = millis() - co2Timer; // длительность высокого уровня, миллисекунд
         
         uint16_t ppm = pwmDuty*2; // умножаем на 2, чтобы получить PPM
         
         CO2Binding bnd = HardwareBinding->GetCO2Binding();
         if(ppm > bnd.MaxPPM) // проверяем выход за диапазон
         {
           ppm = bnd.MaxPPM;
         }

         // обновляем значение ppm
         CO2Control->State.UpdateState(StateCO2,0,(void*)&ppm);

         // снимаем обработчик прерывания, он будет установлен заново в Update, по истечению интервала опроса датчика
         detachInterrupt(digitalPinToInterrupt(co2Pin));
         interruptAttached = false;
         
         waitForLow = false;
         
       } // if(waitForLow) 
       
    } // else
}
//--------------------------------------------------------------------------------------------------------------------------------------
CO2Module::CO2Module() : AbstractModule("CO2")
{
  CO2Control = this; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::setupOutputs()
{
       CO2Binding bnd = HardwareBinding->GetCO2Binding();

       if(bnd.LinkType == linkUnbinded) // не используется
       {
        return;
       }

        if(bnd.LinkType == linkDirect)
        {
            if(bnd.RelayPin != UNBINDED_PIN)
            {
              if(EEPROMSettingsModule::SafePin(bnd.RelayPin))
              {
                WORK_STATUS.PinMode(bnd.RelayPin,OUTPUT);
                WORK_STATUS.PinWrite(bnd.RelayPin,!bnd.Level);
              }
            }

            if(bnd.AlertPin != UNBINDED_PIN)
            {
              if(EEPROMSettingsModule::SafePin(bnd.AlertPin))
              {
                WORK_STATUS.PinMode(bnd.AlertPin,OUTPUT);
                WORK_STATUS.PinWrite(bnd.AlertPin,!bnd.Level);              
              }
            }

            if(bnd.VentPin != UNBINDED_PIN)
            {
              if(EEPROMSettingsModule::SafePin(bnd.VentPin))
              {
                WORK_STATUS.PinMode(bnd.VentPin,OUTPUT);
                WORK_STATUS.PinWrite(bnd.VentPin,!bnd.Level);              
              }
            }
            
        }
        else
        if(bnd.LinkType == linkMCP23S17)
        {
          #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
              if(bnd.RelayPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.RelayPin,OUTPUT);
                WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.RelayPin,!bnd.Level);                
              }
              if(bnd.AlertPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.AlertPin,OUTPUT);
                WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.AlertPin,!bnd.Level);                
              }
              if(bnd.VentPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.VentPin,OUTPUT);
                WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.VentPin,!bnd.Level);                
              }
          #endif
        }
        else
        if(bnd.LinkType == linkMCP23017)
        {
          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              if(bnd.RelayPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.RelayPin,OUTPUT);
                WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.RelayPin,!bnd.Level);                
              }
              if(bnd.AlertPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.AlertPin,OUTPUT);
                WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.AlertPin,!bnd.Level);                
              }
              if(bnd.VentPin != UNBINDED_PIN)
              {
                WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.VentPin,OUTPUT);
                WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.VentPin,!bnd.Level);                
              }
          #endif
        }
        
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::out(uint8_t pin,uint8_t level)
{
    CO2Binding bnd = HardwareBinding->GetCO2Binding();

     if(bnd.LinkType == linkUnbinded) // не используется
     {
      return;
     }

      if(bnd.LinkType == linkDirect)
      {
          if(pin != UNBINDED_PIN)
          {
              if(EEPROMSettingsModule::SafePin(pin))
              {
                WORK_STATUS.PinWrite(pin,level);
              }
          }
      }
      else
      if(bnd.LinkType == linkMCP23S17)
      {
          #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
            if(pin != UNBINDED_PIN)
            {
              WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,pin,level);
            }
          #endif
      }
      else
      if(bnd.LinkType == linkMCP23017)
      {
          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
            if(pin != UNBINDED_PIN)
            {
              WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,pin,level);
            }
          #endif
      }
      
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::ReloadSettings()
{
  LoadSettings();

  if(!settings.active) // если выключили - выключаем выходы, и всё
  {
    CO2Binding bnd = HardwareBinding->GetCO2Binding();
    
    out(bnd.RelayPin,!bnd.Level);
    out(bnd.AlertPin,!bnd.Level);
    out(bnd.VentPin,!bnd.Level);
    
    alertOn = co2On = false;
    alertActive = false; // выключаем сигнализатор аварии, т.к. модуль выключен
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::LoadSettings()
{
  GlobalSettings* s = MainController->GetSettings();
  
  settings = s->GetCO2Settings();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::control()
{
  if(!settings.active) // выключены
  {
    return;
  }

  OneState* os = State.GetStateByOrder(StateCO2,0);
  if(!os)
  {
    return; // нету состояния
  }

  CO2Binding bnd = HardwareBinding->GetCO2Binding();

  if(!os->HasData()) // нет данных с датчика, нечего контролировать!
  {
    // выключаем подачу CO2
     if(co2On) // если включена подача CO2 - выключаем
      {
        out(bnd.RelayPin,!bnd.Level);
        co2On = false;
      }

      if(alertActive) // была авария, выключаем
      {
        alertActive = false;
        alertOn = false;
        out(bnd.AlertPin,!bnd.Level);        
        out(bnd.VentPin,!bnd.Level);        
      }      
      
    return;
  }

  bool canWork = false;

 #ifdef USE_DS3231_REALTIME_CLOCK
    // получаем текущее время
    RealtimeClock rtc = MainController->GetClock();
    RTCTime currentTime = rtc.getTime();        

    // формируем диапазоны, в минутах
    uint32_t startDia = settings.startTime;
    uint32_t endDia = settings.endTime;
    uint32_t nowMins = 60ul*currentTime.hour + currentTime.minute;

    if(bitRead(settings.weekdays,currentTime.dayOfWeek-1) && (nowMins >= startDia && nowMins < endDia))
    {
      canWork = true;
    }
  #endif // USE_DS3231_REALTIME_CLOCK 

  if(!canWork) // не можем работать в эти дни недели
  {    
    if(co2On) // если включена подача CO2 - выключаем
    {
      out(bnd.RelayPin,!bnd.Level);
      co2On = false;
    }
    
    return;
  }

  // тут можем работать, проверяя CO2

  CO2Pair co2 = *os;
  uint32_t curPPM = co2.Current;

  

  // тут контроль CO2
  if(curPPM <= uint32_t(settings.ppm - settings.histeresis))
  {
    // текущее значение СО2 меньше, чем уставка минус гистерезис - включаем подачу CO2
    if(!co2On)
    {
        co2On = true;
        out(bnd.RelayPin,bnd.Level);
    }

    if(alertActive) // была авария, выключаем
      {
        alertActive = false;
        alertOn = false;
        out(bnd.AlertPin,!bnd.Level);        
        out(bnd.VentPin,!bnd.Level);        
      }
  }
  else
  if(curPPM >= uint32_t(settings.ppm + settings.histeresis))
  {
    // значение СО2 превысило уставку с учётом гистерезиса - выключаем подачу CO2

    if(co2On)
    {
        co2On = false;
        out(bnd.RelayPin,!bnd.Level);
    }

    // поскольку у нас значение превысило уставку - прибавляем N процентов, и смотрим - превысило ли это значение, авария ли это?
    uint32_t alertPPM = (settings.ppm*bnd.AlertPercents)/100 + settings.ppm + settings.histeresis;


    if(curPPM >= alertPPM)
    {
      // это однозначно авария!
      if(!alertActive)
      {
        // если не был взведён флаг аварии до этого, то - взводим
        alertActive = true;
        alertTimer = millis();
        alertOn = true;
        out(bnd.AlertPin,bnd.Level);
        out(bnd.VentPin,bnd.Level);
      }
      
    } // if
    else
    {
      // аварии нет, выключаем
      
      if(alertActive)
      {
        alertActive = false;
        alertOn = false;
        out(bnd.AlertPin,!bnd.Level);        
        out(bnd.VentPin,!bnd.Level);        
      }

    } // else
    
  } // else if(curPPM >= uint32_t(settings.ppm + settings.histeresis))
  else
  {
    // всё в пределах нормы, если есть авария - выключаем
     if(alertActive)
      {
        alertActive = false;
        alertOn = false;
        out(bnd.AlertPin,!bnd.Level);        
        out(bnd.VentPin,!bnd.Level);        
      }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::Setup()
{
  alertActive = false;
  alertOn = co2On = false;
  alertTimer = 0;
  
  setupOutputs();  
  LoadSettings();
  
  updateTimer = 0;
  
  CO2Binding bnd = HardwareBinding->GetCO2Binding();
  
  if(bnd.SensorPin != UNBINDED_PIN)
  {
    State.AddState(StateCO2,0);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CO2Module::Update()
{ 

#if defined(USE_CO2_MODULE)

  control();


  if(alertActive)
  {
    // авария активна, мигаем выходом
    if(millis() - alertTimer >= CO2_ALERT_DURATION)
    {
      alertOn = !alertOn;

      CO2Binding bnd = HardwareBinding->GetCO2Binding();
      out(bnd.AlertPin,alertOn ? bnd.Level : !bnd.Level);

      alertTimer = millis();
      
    }
    
  } // if(alertActive)


  if(millis() - updateTimer >= CO2_UPDATE_INTERVAL)
  {

    CO2Binding bnd = HardwareBinding->GetCO2Binding();
    if(bnd.SensorPin != UNBINDED_PIN)
    {

        CO2Binding bnd = HardwareBinding->GetCO2Binding();

        if(bnd.MeasureMode == 0) // аналоговое измерение
        {      
            int val = analogRead(bnd.SensorPin);
    
            if(val < bnd.MinADCPPMValue)
              val = bnd.MinADCPPMValue;
    
            if(val > bnd.MaxADCPPMValue)
              val = bnd.MaxADCPPMValue;
    
            // вогнали val в диапазон допустимых значений АЦП
    
           // в MAX_CO2_SENSOR_PPM - у нас каксимальное измеряемое значение датчика, лежащее в диапазоне [0,MAX_CO2_SENSOR_PPM]
           // и отображающееся на диапазон АЦП [CO2_PPM_MIN_ADC_VALUE,CO2_PPM_MAX_ADC_VALUE]
    
           // пересчитываем в ppm
            uint16_t ppm = map(val,bnd.MinADCPPMValue,bnd.MaxADCPPMValue,0,bnd.MaxPPM);
    
            State.UpdateState(StateCO2,0,(void*)&ppm);
            
        } // if(bnd.MeasureMode == 0)
        else // режим PWM, устанавливаем обработчик прерывания
        {
            if(!interruptAttached) // обработчик не назначен, назначаем
            {
              interruptAttached = true;
              co2Pin = bnd.SensorPin;

              requestByInterruptTimer = millis(); // запоминаем время, когда запросили данные по прерыванию
              attachInterrupt(digitalPinToInterrupt(co2Pin), co2Measure, CHANGE);
                          
            } // if(!interruptAttached)
            else
            {
              // прерывание назначено, проверяем время последнего получения данных.
              // если данных не было очень долго - снимаем прерывание, и обновляем показания в вид "нет данных".
              // у нас с датчика в режиме PPM расчёт - 1 мс == 2 ppm, максимум ppm - 5000, ставим 10 секунд про запас.
              if(millis() - requestByInterruptTimer >= 10000)
              {
                // очень долго не было данных, сбрасываем
                detachInterrupt(digitalPinToInterrupt(co2Pin));
                
                interruptAttached = false;
                waitForLow = false;

                uint16_t ppm = NO_CO2_DATA;
                State.UpdateState(StateCO2,0,(void*)&ppm);
              }
            } // interrupt attached
          
        } // else PWM mode
    }

    updateTimer = millis();

  } // if interval reached
#endif // USE_CO2_MODULE
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  CO2Module::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = PARAMS_MISSED;
    
  
  uint8_t argsCount = command.GetArgsCount();
  if(argsCount < 1)
  {
    if(command.GetType() == ctGET)
    {
    // попросили отдать все показания с датчиков
    
      PublishSingleton.Flags.Status = true;
      if(wantAnswer) 
      {
         PublishSingleton = "";

         uint8_t _cnt = State.GetStateCount(StateCO2);

         PublishSingleton << _cnt;
         
         for(uint8_t i=0;i<_cnt;i++)
         {
            OneState* os = State.GetStateByOrder(StateCO2,i);
            if(os)
            {
              CO2Pair lp = *os;

              PublishSingleton << PARAM_DELIMITER;
              PublishSingleton << lp.Current;
                              
            } // if(os)
         } // for
     
      }

    } // if(command.GetType() == ctGET)
   
    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
  } // if(argsCount < 1)
  
  if(command.GetType() == ctGET)
  {
      String which = command.GetArg(0);
      if(which == F("SETTINGS")) // CTGET=CO2|SETTINGS, returns OK=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {  
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        << PARAM_DELIMITER << settings.weekdays
        << PARAM_DELIMITER << settings.startTime
        << PARAM_DELIMITER << settings.endTime
        << PARAM_DELIMITER << settings.ppm
        << PARAM_DELIMITER << settings.histeresis
        ;
      } // if(which == F("SETTINGS"))
      else
      if(which == F("ACTIVE")) // CTGET=CO2|ACTIVE
      {
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        ;
        
      } // if(which == F("ACTIVE"))
      else // возможно, запросили показания датчика, CTGET=CO2|0, CTGET=CO2|1 и т.п.
      {
         uint8_t idx = which.toInt();
          uint8_t _cnt = State.GetStateCount(StateCO2);
          
          if(idx >= _cnt)
          {
            // плохой индекс
            if(wantAnswer)
            {
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << NOT_SUPPORTED;
            }
          } // плохой индекс
          else
          {
             if(wantAnswer)
             {
              PublishSingleton = which;
             }
              
             OneState* stateCO2 = State.GetStateByOrder(StateCO2,idx);
             if(stateCO2)
             {
                PublishSingleton.Flags.Status = true;
                CO2Pair co2p = *stateCO2;
                
                if(wantAnswer)
                {
                    PublishSingleton << PARAM_DELIMITER << (co2p.Current);
                }
             } // if
            
          } // else нормальный индекс        
        
      } // else
  } // ctGET
  else
  if(command.GetType() == ctSET)
  {
      String which = command.GetArg(0);

      if(which == F("SETTINGS")) // CTSET=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {
          if(argsCount > 6)
          {
            settings.active = atoi(command.GetArg(1));
            settings.weekdays = atoi(command.GetArg(2));
            settings.startTime = atol(command.GetArg(3));
            settings.endTime = atol(command.GetArg(4));
            settings.ppm = atoi(command.GetArg(5));
            settings.histeresis = atoi(command.GetArg(6));

            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
      } // if(which == F("SETTINGS")) 
      else
      if(which == F("ACTIVE")) // CTSET=CO2|ACTIVE|active flag
      {
          if(argsCount > 1)
          {
            settings.active = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.active;
          }
      } // if(which == F("ACTIVE"))
       else
      if(which == F("PPM")) // CTSET=CO2|PPM|ppm value
      {
          if(argsCount > 1)
          {
            settings.ppm = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.ppm;
          }
      } // if(which == F("PPM"))
      
  } // ctSET

    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------


