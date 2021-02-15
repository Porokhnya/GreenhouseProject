#include "LogicManageModule.h"
#include "ModuleController.h"
#include "InteropStream.h"
#include "TempSensors.h"
#include "EEPROMSettingsModule.h"
#include "WeatherStation.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
Thermostat::Thermostat()
{
  channel = 0;
  settings.active = false;
  onFlag = false;
  tempTimer = 0;
  workMode = twmAuto;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Thermostat::setup(uint8_t _channel)
{
  #ifdef THERMOSTAT_DEBUG
  Serial.print(F("THERMOSTAT SETUP: CHANNEL #"));
  Serial.println(_channel);
  #endif

  channel = _channel;

// настраиваем выхода
  ThermostatBinding bnd = HardwareBinding->GetThermostatBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);
    }
    #endif
  }

  reloadSettings();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Thermostat::setState(bool on)
{
  if(on && onFlag) // попросили включиться, но мы уже включены
    return;

  if(!on && !onFlag) // попросили выключиться, но мы уже выключены
    return;

  #ifdef THERMOSTAT_DEBUG
  Serial.print(F("THERMOSTAT SET STATE, CHANNEL #"));
  Serial.print(channel);
  Serial.print(F("; STATE = "));
  Serial.println(on);
  #endif

  onFlag = on;
  ThermostatBinding bnd = HardwareBinding->GetThermostatBinding(channel);
  
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  uint8_t level = on ? bnd.Level : !bnd.Level;
  

if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinWrite(bnd.Pin,level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,level);   
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,level);
    }
    #endif
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
void Thermostat::reloadSettings()
{
  #ifdef THERMOSTAT_DEBUG
  Serial.print(F("THERMOSTAT RELOAD SETTINGS: CHANNEL #"));
  Serial.println(channel);
  #endif
  
  GlobalSettings* sett = MainController->GetSettings();
  settings = sett->GetThermostatSettings(channel);

  if(!settings.active) // выключаем, если неактивны
  {
    if(workMode == twmAuto)
    {
      setState(false);      
    }
    
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Thermostat::update()
{
  if(!settings.active || workMode == twmManual) // неактивны, ничего не надо делать  
    return;

  // ожидаем, можно проверять температуру
  if(millis() - tempTimer > THERMOSTAT_UPDATE_INTERVAL)
  {
    check(); // проверяем, надо ли изменять состояние
    tempTimer = millis();
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Thermostat::check()
{
  #ifdef THERMOSTAT_DEBUG
  Serial.print(F("THERMOSTAT CHECK WANT ON: CHANNEL #"));
  Serial.println(channel);
  #endif

  Temperature temp = LogicManageModule->getTemperature(settings.sensorIndex);
  if(temp.Value == NO_TEMPERATURE_DATA)
  {
      #ifdef THERMOSTAT_DEBUG
        Serial.print(F("THERMOSTAT: NO SENSOR TEMPERATURE, CHANNEL #"));
        Serial.println(channel);
      #endif

      return;
  }

  // получили температуру, проверяем уставки
  int32_t tVal = 100l*temp.Value;
  if(tVal < 0)
    tVal -= temp.Fract;
  else
    tVal += temp.Fract;

  int32_t tOnBorder = 100l*settings.temp;

  if(tVal >= (tOnBorder + 10l*settings.histeresis))
  {
    // мы превысили порог выключения, надо выключать !!!
    #ifdef THERMOSTAT_DEBUG
    Serial.print(F("THERMOSTAT: CHANNEL #"));
    Serial.print(channel);
    Serial.println(F(" WANTS OFF!"));
    #endif

    // выключаем вывод МК
    setState(false);    
  } // if
  else
  if(tVal <= (tOnBorder - 10l*settings.histeresis))
  {
    // мы превысили порог включения, надо включать !!!
    #ifdef THERMOSTAT_DEBUG
    Serial.print(F("THERMOSTAT: CHANNEL #"));
    Serial.print(channel);
    Serial.println(F(" WANTS ON!"));
    #endif

    // включаем вывод МК
    setState(true);       
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
Vent::Vent()
{
  channel = 0;
  settings.active = false;
  onFlag = false;
  machineState = ventIdle;
  tempTimer = 0;
  workStartedAt = 0;
  workMode = vwmAuto;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Vent::setup(uint8_t _channel)
{
  #ifdef VENT_DEBUG
  Serial.print(F("VENT SETUP: CHANNEL #"));
  Serial.println(_channel);
  #endif

  channel = _channel;

// настраиваем выхода
  VentBinding bnd = HardwareBinding->GetVentBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);
    }
    #endif
  }
  reloadSettings();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Vent::setState(bool on)
{
  if(on && onFlag) // попросили включиться, но мы уже включены
    return;

  if(!on && !onFlag) // попросили выключиться, но мы уже выключены
    return;

  #ifdef VENT_DEBUG
  Serial.print(F("VENT SET STATE, CHANNEL #"));
  Serial.print(channel);
  Serial.print(F("; STATE = "));
  Serial.println(on);
  #endif

  onFlag = on;
  VentBinding bnd = HardwareBinding->GetVentBinding(channel);
  
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  uint8_t level = on ? bnd.Level : !bnd.Level;

  if(bnd.LinkType == linkDirect)
    {
      if(bnd.Pin != UNBINDED_PIN)
      {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          WORK_STATUS.PinWrite(bnd.Pin,level);
        }
      }
    }
    else if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      if(bnd.Pin != UNBINDED_PIN)
      {
            WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,level);   
      }
      #endif
    }
    else if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      if(bnd.Pin != UNBINDED_PIN)
      {
             WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,level);
      }
      #endif
    }

}
//--------------------------------------------------------------------------------------------------------------------------------------
void Vent::reloadSettings()
{
  #ifdef VENT_DEBUG
  Serial.print(F("VENT RELOAD SETTINGS: CHANNEL #"));
  Serial.println(channel);
  #endif
  
  GlobalSettings* sett = MainController->GetSettings();
  settings = sett->GetVentSettings(channel);

  if(!settings.active) // выключаем, если неактивны
  {
    if(workMode == vwmAuto)
    {
      setState(false);      
    }
    
    machineState = ventIdle;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Vent::update()
{
  if(!settings.active || workMode == vwmManual) // неактивны, ничего не надо делать  
    return;

  switch(machineState)
  {
    case ventIdle:
    {
      // ожидаем, можно проверять температуру
      if(millis() - tempTimer > VENT_UPDATE_INTERVAL)
      {
        checkWantOn();
        tempTimer = millis();
      }
    };
    break; // ventIdle

    case ventCheckOff:
    {
      // проверяем, не достигли ли мы максимального времени работы?
      
      // время работы у нас хранится в секундах
      uint32_t maxWorkTime = 1000ul*settings.maxWorkTime;
      if(millis() - workStartedAt > maxWorkTime)
      {
         // достигли максимального времени работы, надо отдохнуть !!!
        #ifdef VENT_DEBUG
        Serial.print(F("VENT WANTS REST: CHANNEL #"));
        Serial.println(channel);
        #endif

        // выключаем
        setState(false);

        // сохраняем время начала отдыха
        workStartedAt = millis();

        // переходим в режим отдыха
        machineState = ventRest;

        return;
      }
      
      // проверяем на порог выключения, с учётом гистерезиса
      if(millis() - tempTimer > VENT_UPDATE_INTERVAL)
      {
        tempTimer = millis();
        
        // гистерезис у нас хранится в десятых долях
        Temperature temp = LogicManageModule->getTemperature(settings.sensorIndex);
        if(temp.Value == NO_TEMPERATURE_DATA)
        {
          // нет температуры, переключаемся на режим ожидания, и выключаем всё
          #ifdef VENT_DEBUG
            Serial.print(F("VENT: NO SENSOR TEMPERATURE, CHANNEL #"));
            Serial.println(channel);
          #endif

          // выключаем
          setState(false);
  
          machineState = ventIdle;
        } // if
        else
        {
          // есть температура, получаем её, формируем уставку выключения и проверяем
          int32_t tVal = 100l*temp.Value;
          if(tVal < 0)
            tVal -= temp.Fract;
          else
            tVal += temp.Fract;
        
          int32_t tOffBorder = 100l*settings.temp;
          tOffBorder -= 10l*settings.histeresis;
  
          if(tVal <= tOffBorder)
          {
            // достигли нижнего порога, надо выключать!
            #ifdef VENT_DEBUG
            Serial.print(F("VENT WANT OFF: CHANNEL #"));
            Serial.println(channel);
            #endif
  
            // сначала проверяем - если мы превысили порог минимального времени работы - просто выключаем,
            // иначе - дорабатываем определённое кол-во времени
            uint32_t minWorkTime = 1000ul*settings.minWorkTime;
  
            if(millis() - workStartedAt > minWorkTime)
            {
              // проработали больше, чем минимальное время
              #ifdef VENT_DEBUG
              Serial.print(F("VENT OFF: CHANNEL #"));
              Serial.println(channel);
              #endif
              
              // выключаем
              setState(false);
              // и переключаемся на ожидание
              tempTimer = millis();
              machineState = ventIdle;
            } // if
            else
            {
              // проработали меньше, чем минимальное время, дорабатываем
              // вычисляем интервал
              addInterval = minWorkTime - (millis() - workStartedAt);
  
              #ifdef VENT_DEBUG
              Serial.print(F("VENT NEED ADDITIONAL TIME: CHANNEL #"));
              Serial.print(channel);
              Serial.print(F(", TIME = "));
              Serial.println(addInterval);
              #endif            
              
              // запоминаем время дорабатывания
              workStartedAt = millis();
              // и переключаемся в нужную ветку
              machineState = ventWaitAddInterval;
            } // else
            
          } // if
          
        } // else
      } // if
    }
    break; // ventCheckOff

    case ventWaitAddInterval:
    {
      // ждём срабатывания интервала доработки до минимального времени
      if(millis() - workStartedAt > addInterval)
      {
        // интервал прошёл
        #ifdef VENT_DEBUG
        Serial.print(F("VENT ADDITIONAL TIME DONE: CHANNEL #"));
        Serial.println(channel);
        #endif

        // выключаем
        setState(false);
        // и переключаемся на ожидание
        machineState = ventIdle;
        
      }
    }
    break; // ventWaitAddInterval

    case ventRest:
    {
      // тут мы в режиме отдыха
      uint32_t restTime = 1000ul*settings.restTime;
      if(millis() - workStartedAt > restTime)
      {
         // отдохнули, можно начинать опрашивать датчик температуры
        #ifdef VENT_DEBUG
        Serial.print(F("VENT REST DONE: CHANNEL #"));
        Serial.println(channel);
        #endif

        machineState = ventIdle;
      }
    }
    break; // ventRest
  } // switch
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Vent::checkWantOn()
{
  #ifdef VENT_DEBUG
  Serial.print(F("VENT CHECK WANT ON: CHANNEL #"));
  Serial.println(channel);
  #endif

  Temperature temp = LogicManageModule->getTemperature(settings.sensorIndex);
  if(temp.Value == NO_TEMPERATURE_DATA)
  {
      #ifdef VENT_DEBUG
        Serial.print(F("VENT: NO SENSOR TEMPERATURE, CHANNEL #"));
        Serial.println(channel);
      #endif

      return;
  }

  // получили температуру, проверяем уставки
  int32_t tVal = 100l*temp.Value;
  if(tVal < 0)
    tVal -= temp.Fract;
  else
    tVal += temp.Fract;

  int32_t tOnBorder = 100l*settings.temp;

  if(tVal >= tOnBorder)
  {
    // мы превысили порог включения, надо включать !!!
    #ifdef VENT_DEBUG
    Serial.print(F("VENT: CHANNEL #"));
    Serial.print(channel);
    Serial.println(F(" WANTS ON!"));
    #endif

    // включаем вывод МК
    setState(true);

    // запоминаем время начала работы
    workStartedAt = millis();

    // переходим в режим проверки порога на выключение
    machineState = ventCheckOff;
    
  } // if
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
HumiditySpray::HumiditySpray()
{
  channel = 0;
  settings.active = false;
  onFlag = false;
  machineState = hsmIdle;
  workMode = hsmAuto;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void HumiditySpray::setup(uint8_t _channel)
{
  #ifdef HUMIDITY_SPRAY_DEBUG
  Serial.print(F("SPRAY SETUP: CHANNEL #"));
  Serial.println(_channel);
  #endif

  channel = _channel;

// настраиваем выхода
  HumiditySprayBinding bnd = HardwareBinding->GetHumiditySprayBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);
    }
    #endif
  }
  reloadSettings();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void HumiditySpray::setState(bool on)
{
  if(on && onFlag) // попросили включиться, но мы уже включены
    return;

  if(!on && !onFlag) // попросили выключиться, но мы уже выключены
    return;

  #ifdef HUMIDITY_SPRAY_DEBUG
  Serial.print(F("SPRAY SET STATE, CHANNEL #"));
  Serial.print(channel);
  Serial.print(F("; STATE = "));
  Serial.println(on);
  #endif

  onFlag = on;
  HumiditySprayBinding bnd = HardwareBinding->GetHumiditySprayBinding(channel);
  
  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  uint8_t level = on ? bnd.Level : !bnd.Level;

  if(bnd.LinkType == linkDirect)
    {
      if(bnd.Pin != UNBINDED_PIN)
      {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          WORK_STATUS.PinWrite(bnd.Pin,level);
        }
      }
    }
    else if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      if(bnd.Pin != UNBINDED_PIN)
      {
            WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,level);   
      }
      #endif
    }
    else if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      if(bnd.Pin != UNBINDED_PIN)
      {
             WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,level);
      }
      #endif
    }

}
//--------------------------------------------------------------------------------------------------------------------------------------
void HumiditySpray::reloadSettings()
{
  #ifdef HUMIDITY_SPRAY_DEBUG
  Serial.print(F("SPRAY RELOAD SETTINGS: CHANNEL #"));
  Serial.println(channel);
  #endif
  
  GlobalSettings* sett = MainController->GetSettings();
  settings = sett->GetHumiditySpraySettings(channel);

  if(!settings.active) // выключаем, если неактивны
  {
    if(workMode == hsmAuto)
    {
      setState(false);      
    }
    
    machineState = hsmIdle;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void HumiditySpray::update()
{
  if(workMode == hsmManual) // ручной режим работы, ничего не надо делать
  {
    return;
  }

  if(!settings.active) // выключены в настройках
  {
    // выключаем выход
    setState(false);
    return;
  }

  switch(machineState)
  {
    case hsmIdle: // в режиме ожидания
    {
      if(canWork()) // можем работать в этом диапазоне
      {
          // проверяем влажность
          Humidity hum = LogicManageModule->getHumidity(settings.sensorIndex);
          if(hum.Value == NO_TEMPERATURE_DATA)
          {
              #ifdef HUMIDITY_SPRAY_DEBUG
                Serial.print(F("SPRAY: NO SENSOR DATA, CHANNEL #"));
                Serial.println(channel);
              #endif

              setState(false); // выключаем, т.к. нет показаний с датчика влажности
              
              return;
          }

           // получили влажность, проверяем уставки
            int32_t tVal = 100l*hum.Value;
            if(tVal < 0)
              tVal -= hum.Fract;
            else
              tVal += hum.Fract;
          
            int32_t tOnBorder = 100l*settings.sprayOnValue;
            tOnBorder -= 10l*settings.histeresis;
          
            if(tVal <= tOnBorder)
            {
              // влажность ниже уставки, надо включать !!!
              #ifdef HUMIDITY_SPRAY_DEBUG
              Serial.print(F("SPRAY: CHANNEL #"));
              Serial.print(channel);
              Serial.println(F(" WANTS ON!"));
              #endif
          
              // включаем вывод МК
              setState(true);
          
              // переходим в режим проверки порога на выключение
              machineState = hsmCheckOff;
              
            } // if                    
      }
      else
      {
        // не можем работать в этом диапазоне, выключаемся
        setState(false);
      }
    }
    break; // hsmIdle

    case hsmCheckOff:
    {
      // проверяем выключение
      
      if(!canWork()) // если не можем работать - выключаем выход
      {
        setState(false);
        machineState = hsmIdle;
        return;
      }

      // тут проверяем уставку на выключение
      
        // гистерезис у нас хранится в десятых долях
        Humidity hum = LogicManageModule->getHumidity(settings.sensorIndex);
        if(hum.Value == NO_TEMPERATURE_DATA)
        {
          // нет влажности, переключаемся на режим ожидания, и выключаем всё
          #ifdef HUMIDITY_SPRAY_DEBUG
            Serial.print(F("SPRAY: NO SENSOR DATA, CHANNEL #"));
            Serial.println(channel);
          #endif

          // выключаем
          setState(false);
  
          machineState = hsmIdle;
        } // if
        else
        {
          // есть влажность, получаем её, формируем уставку выключения и проверяем
          int32_t tVal = 100l*hum.Value;
          if(tVal < 0)
            tVal -= hum.Fract;
          else
            tVal += hum.Fract;
        
          int32_t tOffBorder = 100l*settings.sprayOffValue;
          tOffBorder += 10l*settings.histeresis;
  
          if(tVal >= tOffBorder)
          {
            // достигли нижнего порога, надо выключать!
            #ifdef HUMIDITY_SPRAY_DEBUG
            Serial.print(F("SPRAY WANT OFF: CHANNEL #"));
            Serial.println(channel);
            #endif
   
            // выключаем
            setState(false);
            // и переключаемся на ожидание
            machineState = hsmIdle;
          } // if
          
        } // else

      
      
    }
    break; // hsmCheckOff
    
  } // switch(machineState)
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool HumiditySpray::canWork()
{
   // получаем текущее время
      RealtimeClock rtc = MainController->GetClock();
      RTCTime currentTime = rtc.getTime();        

      // формируем диапазоны, в минутах
      uint32_t startDia = settings.startWorkTime;
      uint32_t endDia = settings.endWorkTime;
      uint32_t nowMins = 60ul*currentTime.hour + currentTime.minute;

      if((nowMins >= startDia && nowMins < endDia))
      {
        return true;
      }

     return false;

   
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
CycleVent::CycleVent()
{
  channel = 0;
  activeFlag = false;
  onFlag = false;
  machineState = cvmIdle;
  workMode = cvwmAuto;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::setup(uint8_t ch)
{
  channel = ch;
  activeFlag = true;

// настраиваем выхода
  CycleVentBinding bnd = HardwareBinding->GetCycleVentBinding(channel);

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinMode(bnd.Pin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);      
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);
    }
    #endif
  }
  
  #ifdef CYCLE_VENT_DEBUG
  Serial.print("CYCLE VENT CHANNEL #");
  Serial.print(channel);
  Serial.println(" CycleVent::setup()");
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::on()
{
  if(onFlag)
    return;   

  onFlag = true;

  #ifdef CYCLE_VENT_DEBUG
  Serial.print("CYCLE VENT CHANNEL #");
  Serial.print(channel);
  Serial.println(" CycleVent::on()");
  #endif

  CycleVentBinding bnd = HardwareBinding->GetCycleVentBinding(channel);

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinWrite(bnd.Pin,bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,bnd.Level);   
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,bnd.Level);
    }
    #endif
  }  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::off()
{
  if(!onFlag)
    return;

  onFlag = false;

  #ifdef CYCLE_VENT_DEBUG
  Serial.print("CYCLE VENT CHANNEL #");
  Serial.print(channel);
  Serial.println(" CycleVent::off()");
  #endif

 CycleVentBinding bnd = HardwareBinding->GetCycleVentBinding(channel);

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin))
      {
        WORK_STATUS.PinWrite(bnd.Pin,!bnd.Level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);   
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin,!bnd.Level);
    }
    #endif
  }    
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::disable()
{
  if(!activeFlag)
    return;

  #ifdef CYCLE_VENT_DEBUG
  Serial.print("CYCLE VENT CHANNEL #");
  Serial.print(channel);
  Serial.println(" CycleVent::disable()");
  #endif
  
  activeFlag = false;
  if(workMode == cvwmAuto)
  {
    off(); // выключаем, поскольку нас попросили выключится
  }

  // переключаемся на ожидание
  machineState = cvmIdle;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::enable()
{
  if(activeFlag)
    return;

  #ifdef CYCLE_VENT_DEBUG
  Serial.print("CYCLE VENT CHANNEL #");
  Serial.print(channel);
  Serial.println(" CycleVent::enable()");  
  #endif
  
  activeFlag = true;
  // переключаемся на ожидание
  machineState = cvmIdle;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CycleVent::canWork(CycleVentSettings& sett)
{
      // получаем текущее время
      RealtimeClock rtc = MainController->GetClock();
      RTCTime currentTime = rtc.getTime();        

      // формируем диапазоны, в минутах
      uint32_t startDia = sett.startTime;
      uint32_t endDia = sett.endTime;
      uint32_t nowMins = 60ul*currentTime.hour + currentTime.minute;

      if(bitRead(sett.weekdays,currentTime.dayOfWeek-1) && (nowMins >= startDia && nowMins < endDia))
      {
        return true;
      }

     return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CycleVent::update(CycleVentSettings& sett)
{  
  if(!activeFlag || workMode == cvwmManual) // выключены
    return;

  if(!sett.active)
  {
    // выключены в настройках
    off();
    return;
  }

  // проверяем состояние конечного автомата
  switch(machineState)
  {
    case cvmIdle:
    {
      // ничего не делаем, можно проверять на попадание в рабочий диапазон.
      // если попали в рабочий диапазон - однозначно включаем канал, взводим таймер
      // на ожидание выключения и выходим

      if(canWork(sett))
      {
        // надо включать!

        #ifdef CYCLE_VENT_DEBUG
        Serial.print("CYCLE VENT CHANNEL #");
        Serial.print(channel);
        Serial.println(" WANTS ON FROM  cvmIdle!");
        #endif

        on();
        machineState = cvmWaitOff;
        waitInterval = 60000ul*sett.workTime;
        waitTimer = millis();
      }
      else
      {
        // выключаем однозначно, поскольку в диапазон не попадаем
        off();
      }
      
    }
    break;

    case cvmWaitOff:
    {
      // ждём выключения канала
      if(millis() - waitTimer >= waitInterval)
      {
        #ifdef CYCLE_VENT_DEBUG
        Serial.print("CYCLE VENT CHANNEL #");
        Serial.print(channel);
        Serial.println(" WANTS OFF !");  
        #endif
          
        // пора выключать
        off();
        machineState = cvmWaitOn;
        waitInterval = 60000ul*sett.idleTime;
        waitTimer = millis();
      }
    }
    break;

    case cvmWaitOn:
    {
      // ждём включение канала
      if(millis() - waitTimer >= waitInterval)
      {

        #ifdef CYCLE_VENT_DEBUG
        Serial.print("CYCLE VENT CHANNEL #");
        Serial.print(channel);
        Serial.println(" WANTS ON !");  
        #endif

        if(canWork(sett)) // попадаем в диапазон
        {
          // пора включать
          on();
          machineState = cvmWaitOff;
          waitInterval = 60000ul*sett.workTime;
          waitTimer = millis();
        }
        else
        {
          off();
          // переключаемся на ожидание
          machineState = cvmIdle;
        }
      }
    }
    break;
    
  } // switch
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
// RainSensor
//--------------------------------------------------------------------------------------------------------------------------------------
RainSensor::RainSensor()
{
	rainFlag = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
RainSensor::~RainSensor()
{
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RainSensor::setHasRain(bool b)
{ 
  // установка флага дождя извне. Доступна только тогда, когда режим работы - выносной модуль или метеостанция типа Misol!!!

  RainSensorBinding bnd = HardwareBinding->GetRainSensorBinding();
  
  if(!(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol)) // режим работы - не выносной модуль или метеостанция типа Misol
  {
    lastDataAt = millis(); 
    return;
  }
  
  rainFlag = b; 
  lastDataAt = millis(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RainSensor::setup()
{
  RainSensorBinding bnd = HardwareBinding->GetRainSensorBinding();
  
  if(bnd.WorkMode == wrsDirect) // датчик, подключенный напрямую к контроллеру
  {
      // можем тут настраивать датчик, подключенный напрямую к контроллеру
      
      if(bnd.LinkType != linkUnbinded && bnd.Pin != UNBINDED_PIN) // есть привязка управления
      {
          if(bnd.LinkType == linkDirect)
          {
              if(EEPROMSettingsModule::SafePin(bnd.Pin))
              {
                WORK_STATUS.PinMode(bnd.Pin,INPUT);
              }
          }
          else
          if(bnd.LinkType == linkMCP23S17)
          {
              #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                  WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.Pin,INPUT);
              #endif
          }
          else
          if(bnd.LinkType == linkMCP23017)
          {
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.Pin,INPUT); 
            #endif
          }
        
      } // if(bnd.LinkType != linkUnbinded)    
    
  } // if(bnd.WorkMode == wrsDirect)
  
	lastDataAt = millis();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void RainSensor::update()
{

  RainSensorBinding bnd = HardwareBinding->GetRainSensorBinding();

  if(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol) // датчик подключен через выносной модуль или метеостанцию типа Misol
  {
    if((millis() - lastDataAt) > 2ul*WIND_RAIN_UPDATE_INTERVAL)
    {
      // очень долго не обновлялись данные по RS-485 или радиоканалу, сбрасываем
      lastDataAt = millis();
      rainFlag = false;
    }
    
  } // if(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol)
  else 
  if(bnd.WorkMode == wrsDirect) // датчик подключен напрямую к контроллеру
  {
    // тут проверяем на наличие дождя. Наличием дождя считается срабатывание датчика, при котором
    // он выставляет определённый уровень на пине. При этом, если уровень не менялся очень долго - сбрасываем флаг.

     // обновляем достаточно редко, часто - незачем
     static uint32_t upd = 0;
     if(millis() - upd >= WIND_RAIN_UPDATE_INTERVAL)
     {
         if(bnd.LinkType != linkUnbinded && bnd.Pin != UNBINDED_PIN)
         {
            // читаем с датчика уровень
            bool thisRainFlag = false;
            
            if(bnd.LinkType == linkDirect)
              {
                  if(EEPROMSettingsModule::SafePin(bnd.Pin))
                  {
                    thisRainFlag =  WORK_STATUS.PinRead(bnd.Pin) == bnd.Level;
                  }
              }
              else
              if(bnd.LinkType == linkMCP23S17)
              {
                  #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                      thisRainFlag = WORK_STATUS.MCP_SPI_PinRead(bnd.MCPAddress,bnd.Pin) == bnd.Level;
                  #endif
              }
              else
              if(bnd.LinkType == linkMCP23017)
              {
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                  thisRainFlag = WORK_STATUS.MCP_I2C_PinRead(bnd.MCPAddress,bnd.Pin) == bnd.Level;
                #endif
              }
        
              // если поймали срабатывание датчика, то запоминаем время срабатывания
              if(thisRainFlag)
              {
                lastDataAt = millis();
              }
        
              // если датчик срабатывал ооочень давно - сбрасываем флаг дождя
              if(millis() -  lastDataAt >= 1000ul*bnd.ResetInterval)
              {
                thisRainFlag = false;
              }
        
            // запоминаем флаг дождя
            rainFlag = thisRainFlag;      
            
         } // if(bnd.LinkType != linkUnbinded)
    
        upd = millis();
     } // if(millis() - upd >= WIND_RAIN_UPDATE_INTERVAL)
      
  } // else bnd.WorkMode == wrsDirect
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool RainSensor::hasRain()
{
  return rainFlag;	
}
//--------------------------------------------------------------------------------------------------------------------------------------
LogicManageModuleClass* LogicManageModule = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
LogicManageModuleClass::LogicManageModuleClass() : AbstractModule("LOGIC")
{
	LogicManageModule = this;
	inited = false;
	_can_work = false;

  heatSection1DecisionPercents = 0;
  heatSection1DecisionPercents = 0;
  heatSection2DecisionPercents = 0;

  heatSection1LastI = 0;
  heatSection2LastI = 0;
  heatSection3LastI = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadHumiditySpraySettings()
{
  spray1.reloadSettings();
  spray2.reloadSettings();
  spray3.reloadSettings();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeSprayDecision()
{
  spray1.update();
  spray2.update();
  spray3.update();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadVentSettings()
{
  vent1.reloadSettings();
  vent2.reloadSettings();
  vent3.reloadSettings();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeVentDecision()
{
  vent1.update();
  vent2.update();
  vent3.update();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadThermostatSettings()
{
  thermostat1.reloadSettings();
  thermostat2.reloadSettings();
  thermostat3.reloadSettings();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeThermostatDecision()
{
  thermostat1.update();
  thermostat2.update();
  thermostat3.update();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadCycleVentSettings()
{
  // ПЕРЕЗАГРУЗКА НАСТРОЕК ЦИКЛИЧЕСКОЙ ВЕНТИЛЯЦИИ

  
  GlobalSettings* settings = MainController->GetSettings();  
  cycleVentSettings1 = settings->GetCycleVentSettings(0);
  cycleVentSettings2 = settings->GetCycleVentSettings(1);
  cycleVentSettings3 = settings->GetCycleVentSettings(2);

  if(!cycleVentSettings1.active)
    cycleVent1.disable();
  else
    cycleVent1.enable();
  
  if(!cycleVentSettings2.active)
    cycleVent2.disable();
  else
    cycleVent2.enable();

  if(!cycleVentSettings3.active)
    cycleVent3.disable();
  else
    cycleVent3.enable();

}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeCycleVentDecision(CycleVent& vent,CycleVentSettings& sett, CycleVentDriveCommand& decision)
{
  // обновляем внутреннее состояние класса вентиляции
  vent.update(sett);
  
  if(!sett.active)
  {
    decision = cvDisable;
    return;
  }

  decision = cvEnable;

}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeCycleVentDecision()
{
  // принимаем решение по циклической вентиляции
  
  makeCycleVentDecision(cycleVent1,cycleVentSettings1,cycleVent1Decision);
  makeCycleVentDecision(cycleVent2,cycleVentSettings2,cycleVent2Decision);
  makeCycleVentDecision(cycleVent3,cycleVentSettings3,cycleVent3Decision);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WINDOW_MANAGE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadWindowsSettings()
{
	
	GlobalSettings* settings = MainController->GetSettings();
	
	windowsSection1.openTemp25percents = settings->Get25PercentsOpenTemp(0);
	windowsSection1.openTemp50percents = settings->Get50PercentsOpenTemp(0);
	windowsSection1.openTemp75percents = settings->Get75PercentsOpenTemp(0);
	windowsSection1.openTemp100percents = settings->Get100PercentsOpenTemp(0);
	windowsSection1.histeresis = settings->GetWMHisteresis(0);
	windowsSection1.sensorIndex = settings->GetWMSensor(0);
	windowsSection1.active = settings->GetWMActive(0);

	windowsSection2.openTemp25percents = settings->Get25PercentsOpenTemp(1);
	windowsSection2.openTemp50percents = settings->Get50PercentsOpenTemp(1);
	windowsSection2.openTemp75percents = settings->Get75PercentsOpenTemp(1);
	windowsSection2.openTemp100percents = settings->Get100PercentsOpenTemp(1);
	windowsSection2.histeresis = settings->GetWMHisteresis(1);
	windowsSection2.sensorIndex = settings->GetWMSensor(1);
	windowsSection2.active = settings->GetWMActive(1);

	windowsSection3.openTemp25percents = settings->Get25PercentsOpenTemp(2);
	windowsSection3.openTemp50percents = settings->Get50PercentsOpenTemp(2);
	windowsSection3.openTemp75percents = settings->Get75PercentsOpenTemp(2);
	windowsSection3.openTemp100percents = settings->Get100PercentsOpenTemp(2);
	windowsSection3.histeresis = settings->GetWMHisteresis(2);
	windowsSection3.sensorIndex = settings->GetWMSensor(2);
	windowsSection3.active = settings->GetWMActive(2);

	windowsSection4.openTemp25percents = settings->Get25PercentsOpenTemp(3);
	windowsSection4.openTemp50percents = settings->Get50PercentsOpenTemp(3);
	windowsSection4.openTemp75percents = settings->Get75PercentsOpenTemp(3);
	windowsSection4.openTemp100percents = settings->Get100PercentsOpenTemp(3);
	windowsSection4.histeresis = settings->GetWMHisteresis(3);
	windowsSection4.sensorIndex = settings->GetWMSensor(3);
	windowsSection4.active = settings->GetWMActive(3);
	
	windowsBySectionsBinding = settings->GetWMBinding();
	windowsRainBinding = settings->GetRainBinding();	
	windowsOrientationBinding = settings->GetOrientationBinding(); 
	
	windSpeed = settings->GetWindSpeed();
	hurricaneSpeed = settings->GetHurricaneSpeed();
	
	// поскольку наши настройки изменились, считаем, что по окнам никаких решений не принималось
	initWindowsDecisionList();
		
	if(!_can_work) // незарегистрированная версия
  {
		return;
  }
	
	// тут можно принимать какие-либо решения
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WINDOW_MANAGE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SHADOW_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
// Shadow
//--------------------------------------------------------------------------------------------------------------------------------------
Shadow::Shadow()
{
	workTime = 0;
	channel = 0xFF;
	valveTimer = 0;
	valveState = hvsUnknown;
	timerActive = false;
	timerInterval = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool Shadow::isBusy()
{
	return (valveState == hvsOpening || valveState == hvsClosing);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::on()
{
	if(valveState == hvsOpen || valveState == hvsOpening)
	{
		return; // уже открыты или открываемся
	}
	
	// тут принимаем решение о интервале, в том случае, если нас дёрнули по центру открытия/закрытия
	uint32_t diff = 0;
	if(timerActive)
	{
		diff = millis() - valveTimer;
	}
	
	timerActive = true;
	
	if(diff < workTime)
		timerInterval = workTime - diff;
	else
		timerInterval = workTime;
	
	
	#ifdef SHADOW_MODULE_DEBUG	
		Serial.print("SHADOW wants ON, TIMER interval: drive #");
		Serial.print(channel);
		Serial.print(", interval=");
		Serial.println(timerInterval);
	#endif // SHADOW_MODULE_DEBUG
	
	valveState = hvsOpening;
	valveTimer = millis();
	turnValve(vtmOpen);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::turnValve(ValveTurnMode mode)
{

  ShadowBinding bnd = HardwareBinding->GetShadowBinding(channel);
  
	uint8_t leftChannelLevel = !bnd.Level;
	uint8_t rightChannelLevel = !bnd.Level;
	
	switch(mode)
	{
		case vtmOff:
		{
			leftChannelLevel = !bnd.Level;
			rightChannelLevel = !bnd.Level;
		}
		break;
		
		case vtmOpen:
		{
			leftChannelLevel = bnd.Level;
			rightChannelLevel = !bnd.Level;
		}
		break;
		
		case vtmClose:
		{
			leftChannelLevel = !bnd.Level;
			rightChannelLevel = bnd.Level;			
		}
		break;
					
	} // switch

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin1 != UNBINDED_PIN)
    {
        if(EEPROMSettingsModule::SafePin(bnd.Pin1))
        {
          WORK_STATUS.PinWrite(bnd.Pin1,leftChannelLevel);
        }
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
        if(EEPROMSettingsModule::SafePin(bnd.Pin2))
        {
          WORK_STATUS.PinWrite(bnd.Pin2,rightChannelLevel);
        }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin1,leftChannelLevel);
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin2,rightChannelLevel);
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin1,leftChannelLevel);
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin2,rightChannelLevel);
    }
    #endif
  }  

}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::off()
{
	if(valveState == hvsClosed || valveState == hvsClosing)
	{
		return; // уже закрыты или закрываемся
	}
	
	// тут принимаем решение о интервале, в том случае, если нас дёрнули по центру открытия/закрытия
	uint32_t diff = 0;
	if(timerActive)
	{
		diff = millis() - valveTimer;
	}	
	
	timerActive = true;
	if(diff < workTime)
		timerInterval = workTime - diff;
	else
		timerInterval = workTime;
	
	#ifdef SHADOW_MODULE_DEBUG	
		Serial.print("Shadow wants OFF, TIMER interval: drive #");
		Serial.print(channel);
		Serial.print(", interval=");
		Serial.println(timerInterval);
	#endif // SHADOW_MODULE_DEBUG
	
	valveState = hvsClosing;
	valveTimer = millis();
	turnValve(vtmClose);
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::update()
{
	
	if(timerActive)
	{		
		if(millis() - valveTimer > timerInterval)
		{
			// интервал прошёл, можно менять внутреннее состояние
			
			timerActive = false;
			if(valveState == hvsOpening)
			{
				valveState = hvsOpen;				
				turnValve(vtmOff);
				
				#ifdef SHADOW_MODULE_DEBUG
					Serial.print("Shadow OPENED: #");
					Serial.println(channel);
				#endif // SHADOW_MODULE_DEBUG
			}
			else
			if(valveState == hvsClosing)
			{
				valveState = hvsClosed;				
				turnValve(vtmOff);
				
				#ifdef SHADOW_MODULE_DEBUG
					Serial.print("Shadow CLOSED: #");
					Serial.println(channel);
				#endif // SHADOW_MODULE_DEBUG
				
			}
		}
		
	} // if(timerActive)
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::setup(uint8_t _channel, int16_t _driveWorkTime)
{
	channel = _channel;
	
	#ifdef SHADOW_MODULE_DEBUG
		Serial.print("Shadow SETUP: #");
		Serial.println(channel);
	#endif // SHADOW_MODULE_DEBUG
	
	// настраиваем выхода насосов и контуров
  ShadowBinding bnd = HardwareBinding->GetShadowBinding(channel);

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
		
	setSettings(_driveWorkTime);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Shadow::setSettings(int16_t _driveWorkTime)
{
	workTime = 1000ul*abs(_driveWorkTime); // у нас время работы привода - в секундах, переводим в миллисекунды
}
//--------------------------------------------------------------------------------------------------------------------------------------
// END OF Shadow CLASS
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeShadowDecision(Shadow& shadow, ShadowSettings& section, ShadowDriveCommand& decision)
{
	if(!section.active) // секция неактивна, решение не принято
	{
		decision = sdUnknown;
		return;
	}
	
	// получаем освещённость
	int32_t sectionLux = getLux(section.sensorIndex);
	
	#ifdef SHADOW_MODULE_DEBUG
		Serial.print("Section lux: ");
		Serial.println(sectionLux);
	#endif // SHADOW_MODULE_DEBUG
	
	// если сломался датчик секции - выключаем и выходим
	if(sectionLux == NO_LUMINOSITY_DATA)
	{
		#ifdef SHADOW_MODULE_DEBUG
			Serial.println("NO SECTION LUX!!!");
		#endif // SHADOW_MODULE_DEBUG
		decision = sdShadowOff;
		return;
	}
	
	// формируем гистерезис.
	uint16_t histeresis = section.histeresis;
	
	//проверяем условия
	if(sectionLux >= section.lux)
	{
		#ifdef SHADOW_MODULE_DEBUG
			Serial.println("WANT SHADOW ON!!!");
		#endif // SHADOW_MODULE_DEBUG

		decision = sdShadowOn;
		return;
	}
	

	if(sectionLux <= (section.lux - histeresis))
	{
		#ifdef SHADOW_MODULE_DEBUG
			Serial.println("WANT SHADOW OFF!!!");
		#endif // SHADOW_MODULE_DEBUG
		
		decision = sdShadowOff;
		return;
	}
		


	
	// тут можно проверять другие условия
	
	decision = sdUnknown;
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeShadowDecision()
{	
	if(millis() - shadowDecisionTimer < SHADOW_DECISION_UPDATE_INTERVAL)
	{
		return;
	}
	
	#ifdef SHADOW_MODULE_DEBUG
		Serial.println("MAKE SHADOW DECISION!");
	#endif // SHADOW_MODULE_DEBUG
	
	// Тут принимаем решения по отоплению
	if(shadowWorkMode1 == swmAutomatic)
		makeShadowDecision(shadow1, shadowSection1, shadowSection1Decision);
	
	if(shadowWorkMode2 == swmAutomatic)
		makeShadowDecision(shadow2, shadowSection2, shadowSection2Decision);
	
	
	if(shadowWorkMode3 == swmAutomatic)
		makeShadowDecision(shadow3, shadowSection3, shadowSection3Decision);
	
	shadowDecisionTimer = millis();

}
//--------------------------------------------------------------------------------------------------------------------------------------
ShadowWorkMode LogicManageModuleClass::GetShadowMode(uint8_t _section)
{
	switch(_section)
	{
		case 0:
			return shadowWorkMode1;
		case 1:
			return shadowWorkMode2;
		case 2:
			return shadowWorkMode3;
	}
	
	return swmAutomatic;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::TurnShadowMode(uint8_t _section, ShadowWorkMode m)
{
	ShadowWorkMode* hwm = NULL;
	ShadowDriveCommand* decision = NULL;
	
	switch(_section)
	{
		case 0:
		{
			hwm = &shadowWorkMode1;
			decision = &shadowSection1Decision;
		}
		break;
		
		case 1:
		{
			hwm = &shadowWorkMode2;
			decision = &shadowSection2Decision;
		}
		break;
		
		case 2:
		{
			hwm = &shadowWorkMode3;
			decision = &shadowSection3Decision;
		}
		break;
	}
	
	if(!hwm || !decision)
		return;
	
	*decision = sdUnknown;
	*hwm = m;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::TurnShadow(uint8_t _section, bool _on)
{
	// тут надо убедиться, что все настройки загружены, т.к. нас могут вызвать до первого вызова Update!
	if(!inited)
	{
		return;
	}


	ShadowSettings* section = NULL;
	Shadow* shadow = NULL;
	ShadowDriveCommand* decision = NULL;
	ShadowWorkMode* hwm = NULL;
	
	switch(_section)
	{
		case 0:
		{
			section = &shadowSection1;
			shadow = &shadow1;
			decision = &shadowSection1Decision;
			hwm = &shadowWorkMode1;
		}
		break;
		
		case 1:
		{
			section = &shadowSection2;
			shadow = &shadow2;
			decision = &shadowSection2Decision;
			hwm = &shadowWorkMode2;
		}
		break;
		
		case 2:
		{
			section = &shadowSection3;
			shadow = &shadow3;
			decision = &shadowSection3Decision;
			hwm = &shadowWorkMode3;
		}
		break;
	}
	
	if(!section || !shadow || !decision || !hwm)
		return;

	*hwm = swmManual;
	
	if(_on)
	{
		*decision  = sdShadowOn;
		shadow->on();
	}
	else
	{
		*decision  = sdShadowOff;
		shadow->off();
	}
}	
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::GetShadowStatus(uint8_t _section, bool& active, bool& on)
{
	active = false;
	on = false;
	// тут надо убедиться, что все настройки загружены, т.к. нас могут вызвать до первого вызова Update!
	if(!inited)
	{
		return;
	}
	
	ShadowSettings* section = NULL;
	Shadow* shadow = NULL;
	
	switch(_section)
	{
		case 0:
		{
			section = &shadowSection1;
			shadow = &shadow1;
		}
		break;
		
		case 1:
		{
			section = &shadowSection2;
			shadow = &shadow2;			
		}
		break;
		
		case 2:
		{
			section = &shadowSection3;
			shadow = &shadow3;
		}
		break;
	}
	
	if(!section || !shadow)
		return;

	active = section->active;
	on = shadow->isOn();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadShadowSettings()
{	
	GlobalSettings* settings = MainController->GetSettings();
		
	shadowSection1 = settings->GetShadowSettings(0);
	shadowSection2 = settings->GetShadowSettings(1);
	shadowSection3 = settings->GetShadowSettings(2);
	
	shadow1.setSettings(settings->GetShadowDriveWorkTime(0));
	shadow2.setSettings(settings->GetShadowDriveWorkTime(1));
	shadow3.setSettings(settings->GetShadowDriveWorkTime(2));

	if(inited)
	{	

	if(!_can_work)
		return;
	
	// тут можно принимать какие-либо решения
	
		if(!shadowSection1.active)
		{
			shadow1.off();
		}
		if(!shadowSection2.active)
		{
			shadow2.off();
		}
		if(!shadowSection3.active)
		{
			shadow3.off();
		}
	}
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SHADOW_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HEAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
// Heater
//--------------------------------------------------------------------------------------------------------------------------------------
Heater::Heater()
{
	workTime = 0;
	channel = 0xFF;
  currentPosition = 0;
  timer = 0;
  onMyWay = false;
  wayDirection = wdNop;
  timerCounter = 0;
  pumpIsOn = false;
  lastWayDir = wdNop;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::turnPump(bool _on)
{
	// включение или выключение насоса
  if(pumpIsOn == _on)
  {
    return;
  }

  pumpIsOn = _on;

  HeatBinding bnd = HardwareBinding->GetHeatBinding(channel);

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }  
  
  uint8_t level;

	if(_on)
	{
		level = bnd.Level;
	}
	else
	{
		level = !bnd.Level;
	}

  if(bnd.LinkType == linkDirect)
  {
    if(bnd.PumpPin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.PumpPin))
      {
        WORK_STATUS.PinWrite(bnd.PumpPin,level);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.PumpPin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.PumpPin,level);   
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.PumpPin != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.PumpPin,level);
    }
    #endif
  }
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::turnValve(WayDirection dir)
{
  if(lastWayDir == dir)
  {
    return;
  }

  lastWayDir = dir;
  
  HeatBinding bnd = HardwareBinding->GetHeatBinding(channel);

  if(bnd.LinkType == linkUnbinded) // нет привязки
  {
    return;
  }
  
  uint8_t leftChannelLevel = !bnd.Level;
  uint8_t rightChannelLevel = !bnd.Level;
  
  switch(dir)
  {
    case wdNop:
    {
      leftChannelLevel = !bnd.Level;
      rightChannelLevel = !bnd.Level;
    }
    break;
    
    case wdOpen:
    {
      leftChannelLevel = bnd.Level;
      rightChannelLevel = !bnd.Level;
    }
    break;
    
    case wdClose:
    {
      leftChannelLevel = !bnd.Level;
      rightChannelLevel = bnd.Level;      
    }
    break;
          
  } // switch


  if(bnd.LinkType == linkDirect)
  {
    if(bnd.Pin1 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin1))
      {
        WORK_STATUS.PinWrite(bnd.Pin1,leftChannelLevel);
      }
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.Pin2))
      {
        WORK_STATUS.PinWrite(bnd.Pin2,rightChannelLevel);
      }
    }
  }
  else if(bnd.LinkType == linkMCP23S17)
  {
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin1,leftChannelLevel);   
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.Pin2,rightChannelLevel);   
    }
    #endif
  }
  else if(bnd.LinkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    if(bnd.Pin1 != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin1,leftChannelLevel);
    }
    if(bnd.Pin2 != UNBINDED_PIN)
    {
           WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.Pin2,rightChannelLevel);
    }
    #endif
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::update()
{

  if(onMyWay)
  {
      uint32_t dt = millis() - timer;

      if(timerCounter >= dt)
      {
        timerCounter -= dt;
      }
      else
      {
        dt = timerCounter; // поскольку у нас значение счётчика таймера меньше, чем дельта прошедшего времени - именно его и следует брать в дальнейший расчёт для установки позиции
        timerCounter = 0;
      }

      if(wayDirection == wdOpen) // открываемся
      {
        currentPosition += dt;
      }
      else // закрываемся
      {
        if(currentPosition >= dt)
        {
          currentPosition -= dt;
        }
        else
        {
          currentPosition = 0;
        }
      } // закрываемся

      if(!timerCounter)
      {
        // закончили движение за установленное время
        
        #ifdef HEAT_MODULE_DEBUG
          Serial.print("Heater #");
          Serial.print(channel);
          Serial.print(" VALVE DONE, position is: ");
          Serial.println(currentPosition);
        #endif // HEAT_MODULE_DEBUG

        // смотрим - если заслонка открыта - то включаем насос, иначе - выключаем
        if(!currentPosition)
        {
          #ifdef HEAT_MODULE_DEBUG
            Serial.println("Heater::update(), valve closed, turn pump OFF.");
          #endif // HEAT_MODULE_DEBUG   
               
          turnPump(false);
        }
        else
        {
          #ifdef HEAT_MODULE_DEBUG
            Serial.println("Heater::update(), valve opened, turn pump ON.");
          #endif // HEAT_MODULE_DEBUG
                
          turnPump(true);
        }

        // раз закончили движение - выключаем выходы клапана
        turnValve(wdNop);

        // сбрасываем флаг движения
        onMyWay = false;
        // запоминаем, что уже никуда не двигаемся
        wayDirection = wdNop;

        // всё, закончили передвижение, можно выдыхать
        
      } // if(!timerCounter)
      
      timer = millis();
  } // if(onMyWay)
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::setup(uint8_t _channel, int16_t _driveWorkTime)
{
	channel = _channel;
	
	#ifdef HEAT_MODULE_DEBUG
		Serial.print("Heater SETUP: #");
		Serial.println(channel);
	#endif // HEAT_MODULE_DEBUG
	
	// настраиваем выхода насосов и контуров
  HeatBinding bnd = HardwareBinding->GetHeatBinding(channel);

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
    if(bnd.PumpPin != UNBINDED_PIN)
    {
      if(EEPROMSettingsModule::SafePin(bnd.PumpPin))
      {
        WORK_STATUS.PinMode(bnd.PumpPin,OUTPUT);
        WORK_STATUS.PinWrite(bnd.PumpPin,!bnd.Level);
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
    if(bnd.PumpPin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,bnd.PumpPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,bnd.PumpPin,!bnd.Level);      
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
    if(bnd.PumpPin != UNBINDED_PIN)
    {
          WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,bnd.PumpPin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,bnd.PumpPin,!bnd.Level);
    }
    
    #endif
  }  
		
	setSettings(_driveWorkTime);

}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::reset()
{
  // закрываем заслонку с максимального положения на 0
  currentPosition = workTime;
  open(0); // просим закрыться на 0%
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::open(uint8_t percents)
{
  /*
  #ifdef HEAT_MODULE_DEBUG
    Serial.print(channel);
    Serial.print(" Heater::open(), requested position: %");
    Serial.println(percents);
  #endif // HEAT_MODULE_DEBUG
  */
  
  // рассчитываем новую позицию
  uint32_t msOfRequestedPosition = onePercentMS*percents;

  if(percents == 0) // запросили полное закрытие
  {
    turnPump(false); // выключаем помпу
  }

  if(currentPosition == msOfRequestedPosition)
  {
/*
  #ifdef HEAT_MODULE_DEBUG
    Serial.print(channel);
    Serial.println(" Heater::open(), SAME POSITION!");
  #endif // HEAT_MODULE_DEBUG
*/
    // выключаем выходы заслонки
    wayDirection = wdNop;
    turnValve(wayDirection);
    onMyWay = false;

    // смотрим - если заслонка открыта - то включаем насос, иначе - выключаем
    if(!currentPosition)
    {
      /*
      #ifdef HEAT_MODULE_DEBUG
        Serial.print(channel);
        Serial.println(" Heater::open(), same position, valve closed, turn pump OFF.");
      #endif // HEAT_MODULE_DEBUG   
      */
           
      turnPump(false); // выключаем помпу
    }
    else
    {
      /*
      #ifdef HEAT_MODULE_DEBUG
        Serial.print(channel);
        Serial.println(" Heater::open(), same position, valve opened, turn pump ON.");
      #endif // HEAT_MODULE_DEBUG
      */
            
      if(percents > 0)
      {
        turnPump(true); // включаем помпу        
      }
      else
      {
        turnPump(false); // выключаем помпу
      }
    }

    return;
  }

  // теперь рассчитываем время работы и направление, в зависимости от разницы между текущей позицией и запрошенной
  int32_t diff = currentPosition - msOfRequestedPosition;
  timerCounter = abs(diff); // запоминаем, сколько нам осталось двигаться, будем уменьшать это дело, пока не закончим движение

  if(diff > 0)
  {
    // текущая позиция больше запрошенной, надо закрыться
    
    #ifdef HEAT_MODULE_DEBUG
      if(!onMyWay) // чтобы не спамило в Serial
      {
        Serial.print(channel);
        Serial.print(" Heater::open(), want to CLOSE for ms: ");
        Serial.println(timerCounter);
      }
    #endif // HEAT_MODULE_DEBUG  

      wayDirection = wdClose; // закрываемся
  }
  else
  {
    // текущая позиция меньше запрошенной, надо открыться
    
    #ifdef HEAT_MODULE_DEBUG
      if(!onMyWay) // чтобы не спамило в Serial
      {
        Serial.print(channel);
        Serial.print(" Heater::open(), want to OPEN for ms: ");
        Serial.println(timerCounter);
      }
    #endif // HEAT_MODULE_DEBUG

    // проверяем - не откроемся ли мы чрезмерно?
    if(currentPosition + timerCounter > workTime)
    {
      timerCounter = workTime - currentPosition; // чтобы не открыться за 100%
    }

      wayDirection = wdOpen; // открываемся
  }

  turnValve(wayDirection); // выдаём на выходы заслонки нужные уровни
  onMyWay = true; // говорим, что двигаемся
  timer = millis(); // запоминаем время начала движения
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Heater::setSettings(int16_t _driveWorkTime)
{
	workTime = 1000ul*abs(_driveWorkTime); // у нас время работы привода - в секундах, переводим в миллисекунды
  onePercentMS = workTime/100; // рассчитываем, сколько миллисекунд нам надо крутить для открытия/закрытия на 1%
}
//--------------------------------------------------------------------------------------------------------------------------------------
// END OF Heater CLASS
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeHeatDecision(Heater& heater, HeatSettings& section, HeatDriveCommand& decision, uint8_t& decisionPercents, float& lastI)
{
	if(!section.active) // секция неактивна, решение не принято
	{
		decision = hdUnknown;
    decisionPercents = 0;
		return;
	}

  // получаем температуры
  Temperature sectionTemperature = getTemperature(section.sensorIndex);
  Temperature ambientTemperature = getTemperature(section.airSensorIndex);
  
  #ifdef HEAT_MODULE_DEBUG
    Serial.print("Section temperature: ");
    Serial.println(sectionTemperature);
    Serial.print("Ambient temperature: ");
    Serial.println(ambientTemperature);
  #endif // HEAT_MODULE_DEBUG
  
  // если сломался датчик секции - выключаем и выходим
  if(!sectionTemperature.HasData())
  {
    #ifdef HEAT_MODULE_DEBUG
      Serial.println("NO SECTION TEMPERATURE!!!");
    #endif // HEAT_MODULE_DEBUG
    decision = hdHeatOff;
    decisionPercents = 0; // полностью закрыли клапан отопления
    lastI = 0; // сбрасываем коэффициент I
    return;
  }
  
  // формируем гистерезис. Гистерезис у нас хранится в десятых долях, чтобы перевести в сотые доли - его надо разделить на 10
  uint16_t histeresis = 5ul*section.histeresis; // поскольку у нас всё в сотых долях - умножение на 5 делит гистерезис на 2

  
  // проверяем на максимальную температуру контура
  int32_t formattedTemperature = 100l*sectionTemperature.Value;
  if(formattedTemperature < 0)
  {
    formattedTemperature -= sectionTemperature.Fract;
  }
  else
  {
    formattedTemperature += sectionTemperature.Fract;
  }
  
  // формируем уставку
  int32_t border = 100l*section.maxTemp;
  
  if(formattedTemperature >= border)
  {
    #ifdef HEAT_MODULE_DEBUG
      Serial.println("SECTION OVERHEAT!!!");
    #endif // HEAT_MODULE_DEBUG
    // температура контура больше максимальной - однозначно выключаем!
    decision = hdHeatOff;
    decisionPercents = 0; // полностью закрыли клапан отопления
    lastI = 0; // сбрасываем коэффициент I
    return;
  }
  
  // проверяем на минимальную температуру контура 
  border = 100l*section.minTemp;
  
  if(formattedTemperature <= border)
  {
    // температура на контуре меньше минимальной, включаем отопление
    decision = hdHeatOn;
    decisionPercents = ComputePI(formattedTemperature,border,lastI, HEAT_kP, HEAT_P_MIN, HEAT_P_MAX, HEAT_kI, HEAT_I_MIN, HEAT_I_MAX, HEAT_K_PROP);

    #ifdef HEAT_MODULE_DEBUG
      Serial.println("SECTION FROSEN!!!");
      Serial.print("New percents computed (SECTION FROZEN): ");
      Serial.println(decisionPercents);
    #endif // HEAT_MODULE_DEBUG
    
    return;
  }
  
  // если данных с датчика климата нет - выходим
  if(!ambientTemperature.HasData())
  {
    decision = hdHeatOff;
    decisionPercents = 0; // полностью закрыли клапан отопления
    lastI = 0; // сбрасываем коэффициент I
    return;
  }
  
  // формируем температуру
  formattedTemperature = 100l*ambientTemperature.Value;
  if(formattedTemperature < 0)
  {
    formattedTemperature -= ambientTemperature.Fract;
  }
  else
  {
    formattedTemperature += ambientTemperature.Fract;
  }


  
  // формируем уставку верхнего порога
  border = 100ul*section.ethalonTemp + histeresis;
  if(formattedTemperature >= border)
  {

   #ifdef HEAT_MODULE_DEBUG
      Serial.println("AMBIENT TEMPERATURE IS GOOD :)");
      Serial.print("Percents not changed (AMBIENT IS GOOD): ");
      Serial.println(decisionPercents);
    #endif // HEAT_MODULE_DEBUG

    decision = hdHeatOff;
    decisionPercents = 0;
    
    return;
  }

  
  // формируем уставку нижнего порога
  border = 100l*section.ethalonTemp - histeresis;
  
  // проверяем нижнюю границу
  //if(formattedTemperature <= border)
//  {
    decision = hdHeatOn;
    decisionPercents = ComputePI(formattedTemperature,100l*section.ethalonTemp + histeresis,lastI, HEAT_kP, HEAT_P_MIN, HEAT_P_MAX, HEAT_kI, HEAT_I_MIN, HEAT_I_MAX, HEAT_K_PROP);

    #ifdef HEAT_MODULE_DEBUG
      Serial.println("NEED TO HEAT AMBIENT!");
      Serial.print("New percents computed (HEAT AMBIENT): ");
      Serial.println(decisionPercents);
    #endif // HEAT_MODULE_DEBUG
        
    return;
//  }

	// тут можно проверять другие условия
	
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeHeatDecision()
{	
	if(inHeaterResetMode || (millis() - heatDecisionTimer < HEAT_DECISION_UPDATE_INTERVAL) )
	{
		return;
	}
	
	#ifdef HEAT_MODULE_DEBUG
		Serial.println("MAKE HEAT DECISION!");
	#endif // HEAT_MODULE_DEBUG
	
	// Тут принимаем решения по отоплению
	if(heatWorkMode1 == hwmAutomatic)
 {
		makeHeatDecision(heater1, heatSection1, heatSection1Decision, heatSection1DecisionPercents, heatSection1LastI);
 }
	
	if(heatWorkMode2 == hwmAutomatic)
 {
		makeHeatDecision(heater2, heatSection2, heatSection2Decision, heatSection2DecisionPercents, heatSection2LastI);
 }
	
	
	if(heatWorkMode3 == hwmAutomatic)
 {
		makeHeatDecision(heater3, heatSection3, heatSection3Decision, heatSection3DecisionPercents, heatSection3LastI);
 }
	
	
	heatDecisionTimer = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
HeatWorkMode LogicManageModuleClass::GetHeatMode(uint8_t _section)
{
	switch(_section)
	{
		case 0:
			return heatWorkMode1;
		case 1:
			return heatWorkMode2;
		case 2:
			return heatWorkMode3;
	}
	
	return hwmAutomatic;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::TurnHeatMode(uint8_t _section, HeatWorkMode m)
{
	HeatWorkMode* hwm = NULL;
	HeatDriveCommand* decision = NULL;
	
	switch(_section)
	{
		case 0:
		{
			hwm = &heatWorkMode1;
			decision = &heatSection1Decision;
		}
		break;
		
		case 1:
		{
			hwm = &heatWorkMode2;
			decision = &heatSection2Decision;
		}
		break;
		
		case 2:
		{
			hwm = &heatWorkMode3;
			decision = &heatSection3Decision;
		}
		break;
	}
	
	if(!hwm || !decision)
  {
		return;
  }
	
	*decision = hdUnknown;
	*hwm = m;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::TurnHeat(uint8_t _section, bool _on)
{
	// тут надо убедиться, что все настройки загружены, т.к. нас могут вызвать до первого вызова Update!
	if(!inited || inHeaterResetMode)
	{
		return;
	}


	HeatSettings* section = NULL;
	Heater* heater = NULL;
	HeatDriveCommand* decision = NULL;
	HeatWorkMode* hwm = NULL;
  uint8_t* decisionPercents = NULL;
	
	switch(_section)
	{
		case 0:
		{
			section = &heatSection1;
			heater = &heater1;
			decision = &heatSection1Decision;
			hwm = &heatWorkMode1;
      decisionPercents = &heatSection1DecisionPercents;
		}
		break;
		
		case 1:
		{
			section = &heatSection2;
			heater = &heater2;
			decision = &heatSection2Decision;
			hwm = &heatWorkMode2;
     decisionPercents = &heatSection2DecisionPercents;
		}
		break;
		
		case 2:
		{
			section = &heatSection3;
			heater = &heater3;
			decision = &heatSection3Decision;
			hwm = &heatWorkMode3;
     decisionPercents = &heatSection3DecisionPercents;
		}
		break;
	}
	
	if(!section || !heater || !decision || !hwm || !decisionPercents)
  {
		return;
  }

	*hwm = hwmManual; // переводим в ручной режим отопления
	
	if(_on) // включаем отопление
	{
		*decision  = hdHeatOn;    
    *decisionPercents = 100; // открываем на 100%
		heater->open(*decisionPercents);
	}
	else // выключаем отопление
	{
		*decision  = hdHeatOff;
    *decisionPercents = 0; // закрыты
		heater->open(0);
	}
}	
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::GetHeatStatus(uint8_t _section, bool& active, bool& on)
{
	active = false;
	on = false;
	// тут надо убедиться, что все настройки загружены, т.к. нас могут вызвать до первого вызова Update!
	if(!inited)
	{
		return;
	}
	
	HeatSettings* section = NULL;
	Heater* heater = NULL;
	
	switch(_section)
	{
		case 0:
		{
			section = &heatSection1;
			heater = &heater1;
		}
		break;
		
		case 1:
		{
			section = &heatSection2;
			heater = &heater2;			
		}
		break;
		
		case 2:
		{
			section = &heatSection3;
			heater = &heater3;
		}
		break;
	}
	
	if(!section || !heater)
		return;

	active = section->active;
	on = heater->isOn();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::GetHeatTemperature(uint8_t _section,Temperature& sensorTemperature, Temperature& airTemperature)
{
	
	sensorTemperature = Temperature();
	airTemperature = Temperature();

	// тут надо убедиться, что все настройки загружены, т.к. нас могут вызвать до первого вызова Update!
	if(!inited)
	{
		return;
	}
  
	HeatSettings* section = NULL;
	
	switch(_section)
	{
		case 0:
		{
			section = &heatSection1;			
		}
		break;
		
		case 1:
		{
			section = &heatSection2;			
		}
		break;
		
		case 2:
		{
			section = &heatSection3;			
		}
		break;
	}
	
	if(!section)
		return;
	
	sensorTemperature = getTemperature(section->sensorIndex);
	airTemperature = getTemperature(section->airSensorIndex);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadHeatSettings()
{

	
	GlobalSettings* settings = MainController->GetSettings();
	
	heatSection1 = settings->GetHeatSettings(0);
	heatSection2 = settings->GetHeatSettings(1);
	heatSection3 = settings->GetHeatSettings(2);
	
	// обновляем настройки обогревателей
	heater1.setSettings(settings->GetHeatDriveWorkTime(0));
	heater2.setSettings(settings->GetHeatDriveWorkTime(1));
	heater3.setSettings(settings->GetHeatDriveWorkTime(2));

  // сбрасываем последние расчёты PI-регулятора
  heatSection1DecisionPercents = 0;
  heatSection1DecisionPercents = 0;
  heatSection2DecisionPercents = 0;

  heatSection1LastI = 0;
  heatSection2LastI = 0;
  heatSection3LastI = 0;

	
	// если модуль инициализирован и модуль выключили  - надо принудительно отключить отопление
	if(inited && !inHeaterResetMode) // если заслонки уже закрываются - ничего делать не надо
	{	

    	if(!_can_work) // незарегистрированная версия
      {
    		return;
      }
    	
    	// тут можно принимать какие-либо решения
    	
    		if(!heatSection1.active)
    		{
          heater1.open(0);
    		}
    		if(!heatSection2.active)
    		{
          heater2.open(0);
    		}
    		if(!heatSection3.active)
    		{    
          heater3.open(0);
    		}
	} // inited
		
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HEAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::ReloadLightSettings()
{
	
	GlobalSettings* settings = MainController->GetSettings();
	
	lightSettings = settings->GetLightSettings();	
	
	// если настройки перезагрузили, но при этом перезагрузили
	// после инициализации, то мы должны проверить - если модуль выключен,
	// и мы отправляли какую-либо команду модулю света - 
	// то нам надо выключить свет. 
	if(inited && !lightSettings.active)
	{
		if(!_can_work) // незарегистрированная копия
    {
			return;
    }

		// тут можно принимать какие-либо решения
		
		if(ldUnknown != lastLightCommand)
		{
			lightOff();
		}
	}
	
	// поскольку настройки перезагрузили - считаем, что мы ничего не отправляли модулю света
	lastLightCommand = ldUnknown;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeLightDecision()
{
	if(!lightSettings.active) // мы неактивны, никакого решения принимать не надо
	{
		lightDecision = ldUnknown;
		lastLightCommand = ldUnknown;
		lightDecisionTimer = millis();
		return;
	}
	
	if(millis() - lightDecisionTimer < LIGHT_DECISION_UPDATE_INTERVAL)
  {
		return;
  }
	
	lightDecisionTimer = millis();
	
	#ifdef USE_DS3231_REALTIME_CLOCK
	// обновление слежения за освещенностью
	
		
	RealtimeClock rtc = MainController->GetClock();
	RTCTime currentTime = rtc.getTime();
	
	// тут проверяем, можем ли работать по времени
	
	// формируем диапазон проверки, в минутах
	uint16_t startDia = 60ul*lightSettings.hour;
	uint16_t endDia = startDia + 60ul*lightSettings.durationHour;
	
	// если мы находимся между этим диапазоном, то мы можем работать в это время,
	// иначе - не можем, и просто выставляем флаг работы в false.
	// надо отразить текущее время в этот диапазон. Существует одна особенность диапазона:
	// если он полностью попадает в текущие сутки, то мы просто смотрим, попадает ли
	// текущее время в этот диапазон. Иначе (например, час начала 23, продолжительность - 
	// 120 минут, т.е. работа закончится на следующие сутки, в час ночи) нам надо отразить
	// текущий час на этот диапазон, т.е. виртуально продлить сутки. Для этого к текущему 
	// времени прибавляем кол-во минут в сутках.

	const uint16_t mins_in_day = 1440; // кол-во минут в сутках
	uint16_t checkMinutes = currentTime.hour*60 + currentTime.minute; // текущее время в минутах
	/////bool haveOverflow = false; // флаг переноса работы на следующие сутки

	if(endDia >= mins_in_day)
	{
	// правая граница диапазона перешагнула на следующие сутки,
	// отражаем диапазон текущего часа на следующие сутки
	// только в том случае, если текущее кол-во минут от начала суток меньше, чем время начала работы
	if(checkMinutes < startDia)
	{
	  checkMinutes += mins_in_day;
	  /////haveOverflow = true;
	}
	}

	// проверяем попадание в диапазон
	bool inDia = (checkMinutes >= startDia && checkMinutes <= endDia);
	
	/*
	if(inDia)
	{
	  // в диапазон попали, надо проверить попадание в дни недели.
	  // считаем, что мы попали в день недели, если он выставлен
	  // в флагах или у нас был перенос работы на следующие сутки.
	  inDia = haveOverflow || bitRead(Settings.DayMask,currentDOW-1);
	}
	*/

	if(!inDia)
	{
		// мы НЕ в диапазоне активности, надо выключить свет
		lightDecision = ldLightOff;
		return;
	}
		
	AbstractModule* lightModule = MainController->GetModuleByID("LIGHT");
	if(!lightModule)
	{
		lightDecision = ldLightOff;
		return;
	}

  LightBinding lb = HardwareBinding->GetLightBinding();
	
	OneState* os = lightModule->State.GetState(StateLuminosity,lb.SensorIndex);
	if(!os)
	{
		// не срослось, выключаем свет и выходим
		lightDecision = ldLightOff;
		return;
	}
	
	LuminosityPair lp = *os; 
    long currentLuminosity = lp.Current;
	
	if(currentLuminosity == NO_LUMINOSITY_DATA) // нет данных с датчика
	{
		lightDecision = ldLightOff;
		return;
	}
	
	// проверяем порог включения
	if(currentLuminosity <= lightSettings.lux)
	{
		// освещённость меньше порога, включаем свет
		lightDecision = ldLightOn;
		return;
	}
	
	// проверяем порог выключения
	if(currentLuminosity >= (lightSettings.lux + lightSettings.histeresis))
	{
		// надо выключить свет
		lightDecision = ldLightOff;
	}
	
	#endif // USE_DS3231_REALTIME_CLOCK
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LUMINOSITY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::applyLightDecision()
{
	if(ldUnknown == lightDecision)
		return;
	
	if(ldLightOff == lightDecision)
	{
		lightOff();
	}
	else
	{
		lightOn();
	}
	
	// строчка закомментирована, потому что нам надо сохранять последнее принятое решение
	// между вызовами функции принятия решений. Это позволит другим модулям всегда
	// знать последнее принятое решение, и действовать в зависимости от.
	// при этом неповтор отсыла одних и тех же команд должен разруливаться на уровне
	// логики.
	//lightDecision = ldUnknown;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::lightOff()
{
	if(ldLightOff == lastLightCommand)
		return;
		
	lastLightCommand = ldLightOff;
	// выключить досветку
	ModuleInterop.QueryCommand(ctSET,F("LIGHT|OFF"),true);
	yield();	
}	
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::lightOn()
{
	if(ldLightOn == lastLightCommand)
		return;
	
	lastLightCommand = ldLightOn;
	// включить досветку
	ModuleInterop.QueryCommand(ctSET,F("LIGHT|ON"),true);
	yield();	
}	
//--------------------------------------------------------------------------------------------------------------------------------------
bool LogicManageModuleClass::isLightOn()
{
	return WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int32_t LogicManageModuleClass::getLux(uint8_t sensorIndex)
{
	AbstractModule* module = MainController->GetModuleByID("LIGHT");
	if(!module)
		return NO_LUMINOSITY_DATA;
	
	OneState* sensorState = module->State.GetState(StateLuminosity,sensorIndex);
	if(!sensorState)
		return NO_LUMINOSITY_DATA;
	
	LuminosityPair lp = *sensorState;
	
	return lp.Current;
}	
//--------------------------------------------------------------------------------------------------------------------------------------
Temperature LogicManageModuleClass::getTemperature(uint8_t sensor)
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

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
    {
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      humiditySensorsCount = 0;   
    }
  
	Temperature result;
	
	const char* moduleName = "STATE";
	uint16_t sensorIndex = sensor;


/*
	#if defined(HEAT_MODULE_DEBUG)
	Serial.print("Want temperature, sensor=");
	Serial.println(sensor);
	#endif
*/
	if(sensorIndex >= tempSensorsCount)
	{
		sensorIndex -= tempSensorsCount;
		moduleName = "HUMIDITY";
	}
/*
	#if defined(HEAT_MODULE_DEBUG)
	Serial.print("Want temperature, sensorIndex=");
	Serial.print(sensorIndex);
	Serial.print(", module=");
	Serial.println(moduleName);
	#endif
*/
	
	module = MainController->GetModuleByID(moduleName);
	if(!module)
 {
		return result;
 }

	OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
	if(!sensorState)
  {
		return result;
  }

	if(sensorState->HasData())
	{
		TemperaturePair tmp = *sensorState;
		result = tmp.Current;
	}
	
	return result;
   	
}	
//--------------------------------------------------------------------------------------------------------------------------------------
Humidity LogicManageModuleClass::getHumidity(uint8_t sensorIndex)
{
  Humidity result;
  
    AbstractModule* module = MainController->GetModuleByID("HUMIDITY");
    if(module)
    {
      humiditySensorsCount = module->State.GetStateCount(StateHumidity);

      OneState* sensorState = module->State.GetState(StateHumidity,sensorIndex);
      if(!sensorState)
      {
        return result;
      }
    
      if(sensorState->HasData())
      {
        HumidityPair tmp = *sensorState;
        result = tmp.Current;
      }
      
    } // if(module)
    else
    {
      humiditySensorsCount = 0;   
    }

  return result;   
} 
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::applyCycleVentDecision()
{
  #ifdef USE_CYCLE_VENT_MODULE
  
  // применяем решения для циклической вентиляции
  if(cycleVent1Decision == cvEnable)
    cycleVent1.enable();
  else
    cycleVent1.disable();

  if(cycleVent2Decision == cvEnable)
    cycleVent2.enable();
  else
    cycleVent2.disable();
    
  if(cycleVent3Decision == cvEnable)
    cycleVent3.enable();
  else
    cycleVent3.disable();
    
  #endif // USE_CYCLE_VENT_MODULE
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::applyShadowDecision()
{
	#ifdef USE_SHADOW_MODULE
	
	
	// применяем решения для затенения
	
	if(sdUnknown != shadowSection1Decision && shadowWorkMode1 == swmAutomatic)
	{
		if(sdShadowOn == shadowSection1Decision)
		{
			shadow1.on();
		}
		else
		{
			shadow1.off();
		}
	}

	if(sdUnknown != shadowSection2Decision && shadowWorkMode2 == swmAutomatic)
	{
		if(sdShadowOn == shadowSection2Decision)
		{
			shadow2.on();
		}
		else
		{
			shadow2.off();
		}
		
	}

	if(sdUnknown != shadowSection3Decision && shadowWorkMode3 == swmAutomatic)
	{
		if(sdShadowOn == shadowSection3Decision)
		{
			shadow3.on();
		}
		else
		{
			shadow3.off();
		}
		
	}
	
	#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t LogicManageModuleClass::ComputePI(int32_t cT, int32_t wT, float& i, uint8_t kP, float pMin, float pMax, float kI, float iMin, float iMax, float kProp)
{

  const uint8_t out_min = 0; //минимальный выходной %
  const uint8_t out_max = 100; //максимальный выходной % 

  // у нас температура передаётся в сотых долях градусов
  float curTemp = cT;
  float wantedTemp = wT;

  curTemp /= 100.;
  wantedTemp /= 100.;

  uint8_t out = 0;
  float e, p;
  e = (wantedTemp - curTemp);//ошибка регулирования

//  Serial.println(e);
  
  //расчет p
  p = (curTemp < wantedTemp - kProp) ? pMax : (curTemp > wantedTemp) ? pMin : (kP*e);

//  Serial.println(p);
  
  //расчет i
   i = (i < iMin) ? iMin : (i > iMax) ? iMax : i+(kI*e);

//   Serial.println(i);
   
  out = (p+i < out_min) ? out_min : (p+i > out_max) ? out_max : p+i;
  
  return out;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::applyHeatDecision()
{
	#ifdef USE_HEAT_MODULE
	
	
	// применяем решения для отопления
  
  if(inHeaterResetMode) // заслонки сбрасываются в закрытое положение - пока ничего делать не надо
  {
    return;
  }
	
	if(hdUnknown != heatSection1Decision && heatWorkMode1 == hwmAutomatic)
	{
		if(hdHeatOn == heatSection1Decision)
		{
			//heater1.on();
     heater1.open(heatSection1DecisionPercents);
		}
		else
		{
			//heater1.off();
      heater1.open(0);
		}
	}

	if(hdUnknown != heatSection2Decision && heatWorkMode2 == hwmAutomatic)
	{
		if(hdHeatOn == heatSection2Decision)
		{
			//heater2.on();
     heater2.open(heatSection2DecisionPercents);
		}
		else
		{
			//heater2.off();
      heater2.open(0);
		}
		
	}

	if(hdUnknown != heatSection3Decision && heatWorkMode3 == hwmAutomatic)
	{
		if(hdHeatOn == heatSection3Decision)
		{
			//heater3.on();
     heater3.open(heatSection3DecisionPercents);
		}
		else
		{
			//heater3.off();
      heater3.open(0);
		}
		
	}
	
	#endif // USE_HEAT_MODULE
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::Setup()
{
  // настройка модуля тут
  _can_work = MainController->checkReg();
  
  
  #ifdef USE_WINDOW_MANAGE_MODULE
	windowDecisionTimer = 0;
	ReloadWindowsSettings();
  #endif // USE_WINDOW_MANAGE_MODULE
  
  #ifdef USE_HEAT_MODULE
	heatDecisionTimer = 0;
	heatWorkMode1 = hwmAutomatic;
	heatWorkMode2 = hwmAutomatic;
	heatWorkMode3 = hwmAutomatic;
	ReloadHeatSettings();
  #endif // USE_HEAT_MODULE
  
  #ifdef USE_SHADOW_MODULE
	shadowDecisionTimer = 0;
	shadowWorkMode1 = swmAutomatic;
	shadowWorkMode2 = swmAutomatic;
	shadowWorkMode3 = swmAutomatic;
	ReloadShadowSettings();
  #endif // USE_HEAT_MODULE
  
  #if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
	lightDecisionTimer = 0;
	ReloadLightSettings();
  #endif // USE_LUMINOSITY_MODULE


  #ifdef USE_CYCLE_VENT_MODULE
    ReloadCycleVentSettings();
  #endif

  #ifdef USE_VENT_MODULE
    ReloadVentSettings();
  #endif

  #ifdef USE_THERMOSTAT_MODULE
    ReloadThermostatSettings();
  #endif  

  #ifdef USE_HUMIDITY_SPRAY_MODULE
    ReloadHumiditySpraySettings();
  #endif
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::Update()
{ 
  
  if(!inited)
  {
	  firstCallSetup();
	  inited = true;
	  return;
  }

	// обновляем датчик скорости ветра
	windSensor.update();
	// обновляем датчик дождя
	rainSensor.update();
  
	// обновляем приводы отопления
	#ifdef USE_HEAT_MODULE
		heater1.update();
		heater2.update();
		heater3.update();

   // если закрываемся при старте и закрылись - сбросить флаг закрывания
   if(inHeaterResetMode && (!heater1.isBusy() && !heater2.isBusy() && !heater3.isBusy()))
   {
    inHeaterResetMode = false;
   }
	#endif
		
	// обновляем приводы затенения
	#ifdef USE_SHADOW_MODULE
		shadow1.update();
		shadow2.update();
		shadow3.update();
	#endif	  
  
  if(!_can_work)
  {
	  return;
  }

	// тут можем работать в полной версии, т.е. принимать решения в автоматическом режиме
  
	// проверяем на изменение режима работы досветки
  	bool lightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
	if(lightAutoMode != lastLightAutoMode)
	{
		// режим работы досветки менялся, считаем, что в этом случае мы не посылали ни одной команды модулю досветки
		lastLightAutoMode = lightAutoMode;
		lastLightCommand = ldUnknown;
	}
	
	bool windowAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
	if(windowAutoMode != lastWindowAutoMode)
	{
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.println(F("Windows mode changed, clear windows commands!"));
		#endif // WINDOW_MANAGE_DEBUG
			
		// режим работы окон изменился, считаем, что мы ничего не посылали окнам
		lastWindowAutoMode = windowAutoMode;
		initWindowsDecisionList();
	}
	
  
  // принимаем решения
  
  // окна
  #ifdef USE_WINDOW_MANAGE_MODULE
	makeWindowsDecision();
  #endif // USE_WINDOW_MANAGE_MODULE
  
  // досветка
  #if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
	makeLightDecision();
  #endif // USE_LIGHT_MANAGE_MODULE
  
  // отопление
  #ifdef USE_HEAT_MODULE
	makeHeatDecision();
  #endif
  
  #ifdef USE_SHADOW_MODULE
	makeShadowDecision();
  #endif

   #ifdef USE_CYCLE_VENT_MODULE
    makeCycleVentDecision();
   #endif

   #ifdef USE_VENT_MODULE
    makeVentDecision();
   #endif

   #ifdef USE_THERMOSTAT_MODULE
    makeThermostatDecision();
   #endif

   #ifdef USE_HUMIDITY_SPRAY_MODULE
    makeSprayDecision();
   #endif
   
      
  // тут принятие других решений
  
  // тут разруливание зависимостей
	solveConflicts();
  
  // применяем решения  
	applyLightDecision(); // досветка
	applyHeatDecision(); // отопление
	applyShadowDecision(); // затенение
	applyWindowsDecision(); // окна
  applyCycleVentDecision(); // циркуляционная вентиляция
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::solveConflicts()
{
	// ТУТ РАЗРУЛИВАЕМ РАЗЛИЧНЫЕ КОНФЛИКТЫ

  //TODO: ЗАКОММЕНТИРОВАЛ ЗАКРЫТИЕ ОКОН ПРИ ВКЛЮЧЁННОМ ОТОПЛЕНИИ
  /*
	// при включенном отоплении на любом из каналов - закрываем фрамуги
	#if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0

   if(WindowModule->CanDriveWindows()) // только если можем управлять окнами
   {
  		#ifdef USE_HEAT_MODULE
  			
  			bool _nowHeater1On = !heater1.isBusy() && heater1.isOn();
  			bool _nowHeater2On = !heater2.isBusy() && heater2.isOn();
  			bool _nowHeater3On = !heater3.isBusy() && heater3.isOn();
  						
  			bool wantToCloseWindowsBecauseOfHeat = !inHeaterResetMode && (_nowHeater1On || _nowHeater2On || _nowHeater3On);			
  			
  		if(wantToCloseWindowsBecauseOfHeat)
  		{
  			// одна из секций отопления будет включена, незачем греть воздух
  			#ifdef WINDOW_MANAGE_DEBUG
  			//	Serial.println(F("Want to close ALL windows because of HEAT ON!"));
  			#endif // WINDOW_MANAGE_DEBUG
  			
  			// пробегаем по всем окнам, и закрываем их
  			size_t to = windowsDecision.size();
  			for(size_t i=0;i<to;i++)
  			{
  				windowsDecision[i] = wdOpen0;
  			}
  		}
		  #endif // USE_HEAT_MODULE
   
   } // if(WindowModule->CanDriveWindows()) // только если можем управлять окнами
		
	
	#endif // defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
	*/
	// тут можно разруливать другие зависимости
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::initWindowsDecisionList()
{
	windowsDecision.empty();
	lastWindowsCommand.empty();
	
	#if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
		for(uint8_t i=0;i<SUPPORTED_WINDOWS;i++)
		{
			windowsDecision.push_back(wdUnknown);
			lastWindowsCommand.push_back(wdUnknown);
		}
	#endif // USE_TEMP_SENSORS
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WINDOW_MANAGE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeWindowsSectionDecision(uint8_t sectionIndex, WManageChannel& section)
{
	if(!section.active) // секция неактивна, не надо принимать решения
		return;
		
	// получаем привязку окон к нашей секции
	// если в привязке 0 - окно не привязано никак
	Vector<size_t> ourWindowsNumbers;
	
	for(size_t i=0;i<SUPPORTED_WINDOWS;i++)
	{
		if(windowsBySectionsBinding.binding[i] == 0) // не привязано
			continue;
		
		if(windowsBySectionsBinding.binding[i] == (sectionIndex + 1))
		{
			// наша секция, сохраняем индекс окна
			ourWindowsNumbers.push_back(i);
		}
		
	} // for
	
	if(!ourWindowsNumbers.size()) // нет ни одного окна по секции
	{
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Section #"));
			Serial.print(sectionIndex);
			Serial.println(F(" has no windows linked, can't make decision!"));
		#endif // WINDOW_MANAGE_DEBUG
			
		return;
	}
	
	#ifdef WINDOW_MANAGE_DEBUG
		Serial.print(F("Windows for section #"));
		Serial.print(sectionIndex);
		Serial.print(F(" are: "));
		
		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			if(i > 0)
				Serial.print(F(", "));
			
			Serial.print(ourWindowsNumbers[i]);
		}
		Serial.println();
	#endif // WINDOW_MANAGE_DEBUG
	
	// теперь можно делать проверки
	
	// проверяем на температуры
	// получаем температуру секции
	Temperature sectionTemperature = getTemperature(section.sensorIndex);
	if(sectionTemperature.HasData())
	{
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Section #"));
			Serial.print(sectionIndex);
			Serial.print(F(" temperature is: "));
			Serial.println(sectionTemperature);
		#endif // WINDOW_MANAGE_DEBUG
		
		// гистерезис у нас хранится в десятых долях, его надо перевести в сотые
		int16_t histeresis = section.histeresis*10;
		
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Section #"));
			Serial.print(sectionIndex);
			Serial.print(F(" histeresis are: "));
			Serial.println(histeresis);
		#endif // WINDOW_MANAGE_DEBUG
		
		// проверяем на уставки. У нас температуры хранятся в целых, а с датчика - есть с сотыми долями
		int16_t normalizedTemperature = sectionTemperature.Value;
		normalizedTemperature *= 100;
		
		if(normalizedTemperature < 0)
			normalizedTemperature -= sectionTemperature.Fract;
		else
			normalizedTemperature += sectionTemperature.Fract;


    #ifdef WINDOW_MANAGE_DEBUG
      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" normalized temperature are: "));
      Serial.println(normalizedTemperature);
    #endif // WINDOW_MANAGE_DEBUG      
		
		// теперь последовательно проверяем на уставки
		WindowDriveCommand temperatureDecision = wdUnknown;
    WindowDriveCommand lastTemperatureDecision = wdUnknown;

    // получаем последнее решение по окнам нашей секции
    for(size_t i=0;i<ourWindowsNumbers.size();i++)
    {
      size_t windowNumber = ourWindowsNumbers[i];
      if(windowNumber < lastWindowsCommand.size())
      {
        lastTemperatureDecision = lastWindowsCommand[windowNumber];
        break;
      }
    } // for 

    #ifdef WINDOW_MANAGE_DEBUG
    
      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" lastTemperatureDecision are: "));
      Serial.println(lastTemperatureDecision);
      
    #endif // WINDOW_MANAGE_DEBUG
		
		// нормализуем уставки
		int16_t t25percents = section.openTemp25percents*100;
		int16_t t50percents = section.openTemp50percents*100;
		int16_t t75percents = section.openTemp75percents*100;
		int16_t t100percents = section.openTemp100percents*100;

    #ifdef WINDOW_MANAGE_DEBUG
    
      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" t25percents are: "));
      Serial.println(t25percents);

      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" t50percents are: "));
      Serial.println(t50percents);

      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" t75percents are: "));
      Serial.println(t75percents);

      Serial.print(F("Section #"));
      Serial.print(sectionIndex);
      Serial.print(F(" t100percents are: "));
      Serial.println(t100percents);

    #endif // WINDOW_MANAGE_DEBUG   


    // здесь нам надо открываться по интервалам, с учётом гистерезиса
    // открывать мы должны по превышению порога, а прикрывать - по гистерезису

   /*
    пример алгоритма:
      <= 19 - 0%
      > 19 < 20 - 25% if last decision == 25% else 0%
      >= 20 - 25%
      > 22 < 23 - 50% if last decision == 50% else 25%
      >= 23 - 50%
      > 25 < 26 - 75% if last decision == 75% else 50%
      >= 26 - 75%
      > 29 < 30 - 100% if last decision == 100% else 75%
      >= 30 - 100%
    
    */

    // применяем алгоритм выше

    // <= 19 - 0%
    if(normalizedTemperature <= (t25percents - histeresis))
      temperatureDecision = wdOpen0;

    // > 19 < 20 - 25% if last decision == 25% else 0%
    if(normalizedTemperature > (t25percents - histeresis) && normalizedTemperature < t25percents)
    {
        if(lastTemperatureDecision == wdOpen25)
          temperatureDecision = wdOpen25;
        else
          temperatureDecision = wdOpen0;
    }

    // >= 20 - 25%
    if(normalizedTemperature >= t25percents)
      temperatureDecision = wdOpen25;

    // > 22 < 23 - 50% if last decision == 50% else 25%
    if(normalizedTemperature > (t50percents - histeresis) && normalizedTemperature < t50percents)
    {
      if(lastTemperatureDecision == wdOpen50)
        temperatureDecision = wdOpen50;
      else
        temperatureDecision = wdOpen25;
    }

    // >= 23 - 50%
    if(normalizedTemperature >= t50percents)
      temperatureDecision = wdOpen50;

    // > 25 < 26 - 75% if last decision == 75% else 50%
    if(normalizedTemperature > (t75percents - histeresis) && normalizedTemperature < t75percents)
    {
      if(lastTemperatureDecision == wdOpen75)
        temperatureDecision = wdOpen75;
      else
        temperatureDecision = wdOpen50;
    }

    // >= 26 - 75%
    if(normalizedTemperature >= t75percents)
      temperatureDecision = wdOpen75;    

    // > 29 < 30 - 100% if last decision == 100% else 75%
    if(normalizedTemperature > (t100percents - histeresis) && normalizedTemperature < t100percents)
    {
      if(lastTemperatureDecision == wdOpen100)
        temperatureDecision = wdOpen100;
      else
        temperatureDecision = wdOpen75;
    } 

    // >= 30 - 100%
    if(normalizedTemperature >= t100percents)
      temperatureDecision = wdOpen100;               

  		
		// решение по температуре принято, применяем его
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Temperature decision for section #"));
			Serial.print(sectionIndex);
			Serial.print(F(" are: "));
			Serial.println(temperatureDecision);
		#endif
	
		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			size_t windowNumber = ourWindowsNumbers[i];
			if(windowNumber < windowsDecision.size())
			{
				windowsDecision[windowNumber] = temperatureDecision;
			}
		} // for
	
		
	} // if(sectionTemperature.HasData())
	
	// проверяем на ветер
 // ситуация такая, что если ветер часто меняет силу - не надо хлопать окнами, поэтому нижний порог должен быть с гистерезисом.
 // поэтому мы держим список закрытых по порогам ветра окон, и очищаем его только тогда, когда порог - меньше уставки с гистерезисом.
 // при этом направление ветра - будет неважно: раз закрыли по порогу и ориентации, то надо ждать снижения ветра.
 
	CompassPoints windOrientation = windSensor.getWindDirection();
	int16_t windSpeedNow = windSensor.getWindSpeed();

  const int16_t windSpeedHisteresis = 3*100; // 3 метра в секунду - гистерезис
  int16_t windSpeedHighBorder = (windSpeed*100);
  int16_t windSpeedLowBorder = windSpeedHighBorder - windSpeedHisteresis;
  if(windSpeedLowBorder < 0)
    windSpeedLowBorder = 0;


  static Vector<size_t> previouslyClosedByWindWindows; // список окон, закрытых по уставке ветра
  static Vector<size_t> previouslyClosedByHurricaneWindows; // список окон, закрытых по уставке урагана
  
	// проверяем на уставки
	if(windSpeedNow >= windSpeedHighBorder) // уставки у нас хранятся в целых, а скорость ветра - измеряется в сотых
	{
		// превышен порог закрытия, надо проверять направление ветра
		// ориентация у нас хранится так: 1 - восток, 2 - юг, 3 - запад, 4 - север
		Vector<size_t> orientationCloseWindows;
		
		// берём наши окна, и проверяем на совпадение ориентации окна с направлением ветра
		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			size_t windowNumber = ourWindowsNumbers[i];
			
			if(windowNumber < SUPPORTED_WINDOWS)
			{
				uint8_t orientation = windowsOrientationBinding.binding[windowNumber];
				switch(orientation)
				{
					case 1: // восток
					{
						if(windOrientation == cpEast)
           {
							orientationCloseWindows.push_back(windowNumber);
           }
					}
					break;
					
					case 2: // юг
					{
						if(windOrientation == cpSouth)
            {
							orientationCloseWindows.push_back(windowNumber);
            }
					}
					break;
					
					case 3: // запад
					{
						if(windOrientation == cpWest)
           {
							orientationCloseWindows.push_back(windowNumber);
           }
					}
					break;
					
					case 4: // север
					{
						if(windOrientation == cpNorth)
            {
							orientationCloseWindows.push_back(windowNumber);
            }
					}
					break;
				} // switch
			} // if
		}
		
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Wind speed border detected, close section #"));
			Serial.print(sectionIndex);
			Serial.print(F(" linked windows, windows are: "));
			
			for(size_t i=0;i<orientationCloseWindows.size();i++)
			{
				if(i > 0)
					Serial.print(F(", "));
				
				Serial.print(orientationCloseWindows[i]);
			}
			Serial.println();
		#endif // WINDOW_MANAGE_DEBUG
		
		// теперь закрываем эти окна
		for(size_t i=0;i<orientationCloseWindows.size();i++)
		{
			size_t windowNumber = orientationCloseWindows[i];
			if(windowNumber < windowsDecision.size())
			{
				windowsDecision[windowNumber] = wdOpen0;
        addToWindList(previouslyClosedByWindWindows,windowNumber); // добавляем в список окон, закрытых по уставке ветра
			}
		}
	} // if(windSpeedNow >= windSpeedHighBorder)
  else
  if(windSpeedNow <= windSpeedLowBorder)
  {
     // очищаем список ранее закрытых по уставке ветра окон
     previouslyClosedByWindWindows.clear();
  }

  // проверяем на ураган
  windSpeedHighBorder = (hurricaneSpeed*100);
  windSpeedLowBorder = windSpeedHighBorder - windSpeedHisteresis;
  if(windSpeedLowBorder < 0)
    windSpeedLowBorder = 0;  
	
	if(windSpeedNow >= windSpeedHighBorder)  // уставки у нас хранятся в целых, а скорость ветра - измеряется в сотых
	{
		// превышен порог по урагану, надо закрыть все окна секции!!!
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Hurricane detected, close ALL section #"));
			Serial.print(sectionIndex);
			Serial.println(F(" linked windows!"));
		#endif // WINDOW_MANAGE_DEBUG

		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			size_t windowNumber = ourWindowsNumbers[i];
			if(windowNumber < windowsDecision.size())
			{
				windowsDecision[windowNumber] = wdOpen0;
        addToWindList(previouslyClosedByHurricaneWindows,windowNumber); // добавляем в список окон, закрытых по уставке урагана
			}
		} // for		
	} // if(windSpeedNow >= windSpeedHighBorder)
  else
  if(windSpeedNow <= windSpeedLowBorder)
  {
    // чистим список закрытых по уставке урагана окон
    previouslyClosedByHurricaneWindows.clear();
  }

  // теперь пробегаем список закрытых окон на всех предыдущих итерациях,
  // и не даём им открыться, даже если ветер чуть стих.
  // они не будут открываться до тех пор, пока ветер не упадёт ниже уставки
  // с учётом гистерезиса.
  for(size_t p=0;p<previouslyClosedByWindWindows.size();p++)
  {
      size_t windowNumber = previouslyClosedByWindWindows[p];
      if(windowNumber < windowsDecision.size())
      {
        windowsDecision[windowNumber] = wdOpen0;
      }    
  }

  for(size_t p=0;p<previouslyClosedByHurricaneWindows.size();p++)
  {
      size_t windowNumber = previouslyClosedByHurricaneWindows[p];
      if(windowNumber < windowsDecision.size())
      {
        windowsDecision[windowNumber] = wdOpen0;
      }    
  }
      
		
	// проверяем на дождь
	bool hasRain = rainSensor.hasRain();
	if(hasRain)
	{
		// получаем привязку наших окон к датчику дождя
		// если в привязке 0 - то ничего не привязано
		Vector<size_t> rainLinkedWindows;
		// проходим по нашим окнам, смотрим, есть ли для них привязка в списке привязок к датчику дождя
		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			size_t windowNumber = ourWindowsNumbers[i];
			// берём привязку
			if((windowNumber < SUPPORTED_WINDOWS) && windowsRainBinding.binding[windowNumber] == 1)//== (windowNumber + 1))
			{
				// наше окно, и его надо закрыть при дожде
				rainLinkedWindows.push_back(windowNumber);
			}
		}
		#ifdef WINDOW_MANAGE_DEBUG
			Serial.print(F("Rain detected, want to close windows in section #"));
			Serial.print(sectionIndex);
			Serial.print(F(", windows are: "));
			
			
			for(size_t i=0;i<rainLinkedWindows.size();i++)
			{
				if(i > 0)
					Serial.print(F(", "));
				
				Serial.print(rainLinkedWindows[i]);
			}
			Serial.println();
		#endif // WINDOW_MANAGE_DEBUG

		// теперь проходим по всем окнам, привязанным к датчику дождя, и говорим им закрыться
		for(size_t i=0;i<rainLinkedWindows.size();i++)
		{
			size_t windowNumber = rainLinkedWindows[i];
			if(windowNumber < windowsDecision.size())
			{
				windowsDecision[windowNumber] = wdOpen0;
			}
			
		} // for
	} // hasRain
	
	// решение по окнам принято, выводим его для отладки
	#ifdef WINDOW_MANAGE_DEBUG
		for(size_t i=0;i<ourWindowsNumbers.size();i++)
		{
			size_t windowNumber = ourWindowsNumbers[i];
			if(windowNumber < windowsDecision.size())
			{
				Serial.print(F("Decision for section #"));
				Serial.print(sectionIndex);
				Serial.print(F(" and window #"));
				Serial.print(windowNumber);
				Serial.print(F(" are: "));
				Serial.println(windowsDecision[windowNumber]);
			}
		} // for	
	#endif // WINDOW_MANAGE_DEBUG
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::addToWindList(Vector<size_t>& list, size_t windowNumber)
{
  for(size_t i=0;i<list.size();i++)
  {
    if(list[i] == windowNumber)
      return;
  }

  list.push_back(windowNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::makeWindowsDecision()
{
	if(millis() - windowDecisionTimer < WINDOW_DECISION_UPDATE_INTERVAL)
	{
		return;
	}
	
	#ifdef WINDOW_MANAGE_DEBUG
		Serial.println("MAKE WINDOWS DECISION!");
	#endif // WINDOW_MANAGE_DEBUG	
	
	#if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
	
		//ПРИНИМАЕМ РЕШЕНИЯ ПО ОКНАМ
	if(WindowModule->CanDriveWindows()) // только если можем управлять окнами
  {
		// обрабатываем посекционно
		makeWindowsSectionDecision(0,windowsSection1);
		makeWindowsSectionDecision(1,windowsSection2);
		makeWindowsSectionDecision(2,windowsSection3);
		makeWindowsSectionDecision(3,windowsSection4);		
  } 
	
	#endif // USE_TEMP_SENSORS
	
	windowDecisionTimer = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WINDOW_MANAGE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::applyWindowsDecision()
{
#if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
	
	// не забыть, что применять решения можно только тогда, когда окнами можно управлять !!!
	if(!WindowModule->CanDriveWindows())
	{
		// окнами пока нельзя управлять, оставляем команды прежними, и выходим.
		return;
	}
	
	// ПРИМЕНЯЕМ РЕШЕНИЯ ПО ОКНАМ
	for(size_t i=0;i<windowsDecision.size();i++)
	{
		WindowDriveCommand curWindowCommand = windowsDecision[i];
   
		if(curWindowCommand != wdUnknown)
		{
			if(lastWindowsCommand[i] != curWindowCommand)
			{		
				// состояние окна изменилось, надо послать новую команду

        // но перед этим - проверить, может ли окно принимать команды!!!
        
        WindowState* window = WindowModule->GetWindow(i);
        
        if(window && !window->IsBusy() && !window->IsInUninterruptedWay())
        {
        
      				lastWindowsCommand[i] = curWindowCommand;
      				
      				// при этом после отсыла мы считаем, что ещё раз эту команду посылать не надо
      				windowsDecision[i] = wdUnknown;
      				
      				String command = F("STATE|WINDOW|");
      				command += i;
      				command += F("|OPEN|");
      				
      				switch(curWindowCommand)
      				{
      					case wdUnknown:
      					break;
      					
      					case wdOpen0: command += 0; break;
      					case wdOpen25: command += 25; break;
      					case wdOpen50: command += 50; break;
      					case wdOpen75: command += 75; break;
      					case wdOpen100: command += 100; break;
      					
      				} // switch
      				
      				command += '%';
      				
      				#ifdef WINDOW_MANAGE_DEBUG
      					Serial.print(F("Send command: "));
      					Serial.println(command);
      				#endif // WINDOW_MANAGE_DEBUG
      				
      				ModuleInterop.QueryCommand(ctSET,command,true);
      				yield();
        } // if(window
        
			}
		}
	} // for
	
	#endif // defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LogicManageModuleClass::firstCallSetup()
{
	
	lastWindowAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
	lastLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
	
	AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
    {
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      tempSensorsCount = 0;
    }

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
    {
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      humiditySensorsCount = 0;   
    }
  
	GlobalSettings* settings = MainController->GetSettings();
	
	initWindowsDecisionList();
	
	// выключаем досветку
	#if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
		lightOff();
	#endif
	
	// настраиваем отопление при старте
	#ifdef USE_HEAT_MODULE
  
		heater1.setup(0,settings->GetHeatDriveWorkTime(0));
		heater2.setup(1,settings->GetHeatDriveWorkTime(1));
		heater3.setup(2,settings->GetHeatDriveWorkTime(2));

    // закрываем заслонки при старте
    inHeaterResetMode = true;
    
    if(heatSection1.active)
    {
      heater1.reset();
    }
    if(heatSection2.active)
    {
      heater2.reset();
    }
    if(heatSection3.active)
    {
      heater3.reset();
    }

    
	#endif

	// настраиваем затенение при старте
	#ifdef USE_SHADOW_MODULE
		shadow1.setup(0,settings->GetShadowDriveWorkTime(0));
		shadow2.setup(1,settings->GetShadowDriveWorkTime(1));
		shadow3.setup(2,settings->GetShadowDriveWorkTime(2));
	#endif
	
	  // настраиваем датчик дождя
		rainSensor.setup();
    // настраиваем датчик ветра
		windSensor.setup();


   #ifdef USE_CYCLE_VENT_MODULE
    cycleVent1.setup(0);
    cycleVent2.setup(1);
    cycleVent3.setup(2);
   #endif

   #ifdef USE_VENT_MODULE
    vent1.setup(0);
    vent2.setup(1);
    vent3.setup(2);
   #endif

   #ifdef USE_THERMOSTAT_MODULE
    thermostat1.setup(0);
    thermostat2.setup(1);
    thermostat3.setup(2);
   #endif 


   #ifdef USE_HUMIDITY_SPRAY_MODULE
    spray1.setup(0);
    spray2.setup(1);
    spray3.setup(2);
   #endif     
  
}	
//--------------------------------------------------------------------------------------------------------------------------------------
bool  LogicManageModuleClass::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = PARAMS_MISSED;
  
	if(!_can_work) // незарегистрированная копия
  {
    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
  }
		
  
  uint8_t argsCount = command.GetArgsCount();
  if(argsCount < 1)
  {
	  MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
  }
  
  if(command.GetType() == ctSET)
  {
    	String which = command.GetArg(0);
    	if(which == F("HEAT"))
    	{
    		// управление отоплением
    		#ifdef USE_HEAT_MODULE
    			if(argsCount > 1)
    			{
    				String param = command.GetArg(1);
           
    				if(param == STATE_ON) // CTSET=LOGIC|HEAT|ON|channel (optional)
    				{
    					// включить отопление
              if(argsCount < 3) // все каналы
              {
    					  TurnHeat(0,true);
    					  TurnHeat(1,true);
    					  TurnHeat(2,true);
              }
              else
              {
                TurnHeat(atoi(command.GetArg(2)),true);
              }
              
    					
    					PublishSingleton.Flags.Status = true;
    					PublishSingleton = which;
    					PublishSingleton << PARAM_DELIMITER << param;
    				}
    				else 
    				if(param == STATE_OFF) // CTSET=LOGIC|HEAT|OFF|channel (optional)
    				{
    					// выключить отопление
              if(argsCount < 3) // все каналы
              {
    					  TurnHeat(0,false);
    					  TurnHeat(1,false);
    					  TurnHeat(2,false);
              }
              else
              {
                TurnHeat(atoi(command.GetArg(2)),false);
              }
    					
    					PublishSingleton.Flags.Status = true;
    					PublishSingleton = which;
    					PublishSingleton << PARAM_DELIMITER << param;
    				}
    		else
    		if(param == WORK_MODE) // CTSET=LOGIC|HEAT|MODE|(AUTO or MANUAL)
    		{
    			if(argsCount > 2)
    			{
    				String mode = command.GetArg(2);

					if(argsCount > 3) // для конкретного канала
					{
						int channel = atoi(command.GetArg(3));
						if(mode == WM_AUTOMATIC)
						{
							TurnHeatMode(channel,hwmAutomatic);
						}
						else
						{
							TurnHeatMode(channel,hwmManual);
						}
						
					  PublishSingleton.Flags.Status = true;
					  PublishSingleton = which;
					  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;						
					}
    				else
					{
        						if(mode == WM_AUTOMATIC) // CTSET=LOGIC|HEAT|MODE|AUTO
        						{
        							TurnHeatMode(0,hwmAutomatic);
        							TurnHeatMode(1,hwmAutomatic);
        							TurnHeatMode(2,hwmAutomatic);
        							
        							PublishSingleton.Flags.Status = true;
        							PublishSingleton = which;
        							PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
        						}
        						else
        						if(mode == WM_MANUAL) // CTSET=LOGIC|HEAT|MODE|MANUAL
        						{
        							TurnHeatMode(0,hwmManual);
        							TurnHeatMode(1,hwmManual);
        							TurnHeatMode(2,hwmManual);
        							
        							PublishSingleton.Flags.Status = true;
        							PublishSingleton = which;
        							PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
        						}
					} // else
               
    			} // argsCount > 2
    		} // if(param == WORK_MODE)
           else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала отопления, CTSET=LOGIC|HEAT|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HeatSettings* h = channelNum == 0 ? &heatSection1 : channelNum == 1 ? &heatSection2 : &heatSection3;
                  h->active = atoi(command.GetArg(3));

                  MainController->GetSettings()->SetHeatSettings(channelNum,*h);

                  // перезагружаем настройки отопления
                  ReloadHeatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала отопления, СTSET=LOGIC|HEAT|CHANNEL|num|minTemp|maxTemp|ethalonTemp|histeresis|sensorIndex|airSensorIndex|active|drive work time
           {
              if(argsCount > 10)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HeatSettings* h = channelNum == 0 ? &heatSection1 : channelNum == 1 ? &heatSection2 : &heatSection3;
                  
                  h->minTemp = atoi(command.GetArg(3));
                  h->maxTemp = atoi(command.GetArg(4));
                  h->ethalonTemp = atoi(command.GetArg(5));
                  h->histeresis = atoi(command.GetArg(6));
                  h->sensorIndex = atoi(command.GetArg(7));
                  h->airSensorIndex = atoi(command.GetArg(8));
                  h->active = atoi(command.GetArg(9));

                  MainController->GetSettings()->SetHeatDriveWorkTime(channelNum,atoi(command.GetArg(10)));
                  MainController->GetSettings()->SetHeatSettings(channelNum,*h);

                  // перезагружаем настройки отопления
                  ReloadHeatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 10)
              else
              if(argsCount > 5) // настройки канала отопления, СTSET=LOGIC|HEAT|CHANNEL|num|minTemp|maxTemp|ethalonTemp
              {
                
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HeatSettings* h = channelNum == 0 ? &heatSection1 : channelNum == 1 ? &heatSection2 : &heatSection3;
                  
                  h->minTemp = atoi(command.GetArg(3));
                  h->maxTemp = atoi(command.GetArg(4));
                  h->ethalonTemp = atoi(command.GetArg(5));

                  MainController->GetSettings()->SetHeatSettings(channelNum,*h);

                  // перезагружаем настройки отопления
                  ReloadHeatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                }
                  
              } // if(argsCount > 5)
            
           } // if(param == F("CHANNEL"))
           
    			} // argsCount > 1
          
    		#endif // USE_HEAT_MODULE
    	} // if(which == F("HEAT"))
     else
     if(which == F("WBND")) // установить привязку окон по секциям: CTSET=LOGIC|WBND|bnd1|...|bnd16
     {
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 16)
        {
           uint8_t cntr = 0;
           for(size_t i=0;i<sizeof(windowsBySectionsBinding.binding)/sizeof(windowsBySectionsBinding.binding[0]);i++)
           {
            uint8_t bnd = atoi(command.GetArg(++cntr));
            if(bnd > 4)
            {
              bnd = 0;
            }
            windowsBySectionsBinding.binding[i] = bnd;
           }

           GlobalSettings* settings = MainController->GetSettings();
           settings->SetWMBinding(windowsBySectionsBinding);
		   
           ReloadWindowsSettings();
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
       #endif // #ifdef USE_WINDOW_MANAGE_MODULE
     } // if(which == F("WBND"))
     else
     if(which == F("RAINBND")) // установить привязку окон к датчику дождя: CTSET=LOGIC|RAINBND|bnd1|...|bnd16
     {
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 16)
        {
           uint8_t cntr = 0;
           for(size_t i=0;i<sizeof(windowsRainBinding.binding)/sizeof(windowsRainBinding.binding[0]);i++)
           {
            uint8_t bnd = atoi(command.GetArg(++cntr));
            if(bnd > 1)
            {
              bnd = 0;
            }
            windowsRainBinding.binding[i] = bnd;
           }

           GlobalSettings* settings = MainController->GetSettings();
           settings->SetRainBinding(windowsRainBinding);

           ReloadWindowsSettings();
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
       #endif // #ifdef USE_WINDOW_MANAGE_MODULE
     } // if(which == F("RAINBND"))
     else
     if(which == F("WORIENT")) // установить привязку окон по сторонам света: CTSET=LOGIC|WORIENT|bnd1|...|bnd16
     {
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 16)
        {
           uint8_t cntr = 0;
           for(size_t i=0;i<sizeof(windowsOrientationBinding.binding)/sizeof(windowsOrientationBinding.binding[0]);i++)
           {
            uint8_t bnd = atoi(command.GetArg(++cntr));
            if(bnd > 4)
            {
              bnd = 0;
            }
            windowsOrientationBinding.binding[i] = bnd;
           }

           GlobalSettings* settings = MainController->GetSettings();
           settings->SetOrientationBinding(windowsOrientationBinding);

           ReloadWindowsSettings();
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
       #endif // #ifdef USE_WINDOW_MANAGE_MODULE
     } // if(which == F("WORIENT"))
     else
     if(which == F("TSETT")) // установить температуры открытия для секций фрамуг: CTSET=LOGIC|TSETT|section number|25% temp|50% temp|75% temp|100% temp|histeresis|sensor|active
     {
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 8)
        {
          uint8_t sectionNum = atoi(command.GetArg(1));
          if(sectionNum > 3)
          {
            sectionNum = 3;
          }

          uint8_t t25 = atoi(command.GetArg(2));
          uint8_t t50 = atoi(command.GetArg(3));
          uint8_t t75 = atoi(command.GetArg(4));
          uint8_t t100 = atoi(command.GetArg(5));
          int16_t histeresis = atoi(command.GetArg(6));
          int16_t sensorIndex = atoi(command.GetArg(7));
		      bool active  = atoi(command.GetArg(8)) != 0;

          GlobalSettings* settings = MainController->GetSettings();
          settings->Set25PercentsOpenTemp(sectionNum,t25);
          settings->Set50PercentsOpenTemp(sectionNum,t50);
          settings->Set75PercentsOpenTemp(sectionNum,t75);
          settings->Set100PercentsOpenTemp(sectionNum,t100);
		  
		      settings->SetWMHisteresis(sectionNum,histeresis);
          settings->SetWMSensor(sectionNum,sensorIndex);
          settings->SetWMActive(sectionNum,active);


          ReloadWindowsSettings();

          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << sectionNum << PARAM_DELIMITER << REG_SUCC;
        } // if(argsCount > 8)
        
      #endif // #ifdef USE_WINDOW_MANAGE_MODULE
      
     } // if(which == F("TSETT"))
     else
     if(which == F("TTEMP")) // установить температуры открытия для секций фрамуг: CTSET=LOGIC|TTEMP|section number|25% temp|50% temp|75% temp|100% temp
     {
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 5)
        {
          uint8_t sectionNum = atoi(command.GetArg(1));
          if(sectionNum > 3)
          {
            sectionNum = 3;
          }

          uint8_t t25 = atoi(command.GetArg(2));
          uint8_t t50 = atoi(command.GetArg(3));
          uint8_t t75 = atoi(command.GetArg(4));
          uint8_t t100 = atoi(command.GetArg(5));

          GlobalSettings* settings = MainController->GetSettings();
          settings->Set25PercentsOpenTemp(sectionNum,t25);
          settings->Set50PercentsOpenTemp(sectionNum,t50);
          settings->Set75PercentsOpenTemp(sectionNum,t75);
          settings->Set100PercentsOpenTemp(sectionNum,t100);
      
          ReloadWindowsSettings();

          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << sectionNum << PARAM_DELIMITER << REG_SUCC;
        } // if(argsCount > 5)
        
      #endif // #ifdef USE_WINDOW_MANAGE_MODULE
      
     } // if(which == F("TTEMP"))     
     else
     if(which == F("TACTIVE")) // вкл/выкл модуль управления окнами, CTSET=LOGIC|TACTIVE|section number|active
     {
      
      #ifdef USE_WINDOW_MANAGE_MODULE
        if(argsCount > 2)
        {
          uint8_t sectionNum = atoi(command.GetArg(1));
          if(sectionNum > 3)
          {
            sectionNum = 3;
          }

          bool active  = atoi(command.GetArg(2)) != 0;
          
          GlobalSettings* settings = MainController->GetSettings();
          settings->SetWMActive(sectionNum,active);

          ReloadWindowsSettings();

          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << sectionNum << PARAM_DELIMITER << REG_SUCC;
        }
      #endif // #ifdef USE_WINDOW_MANAGE_MODULE
      
     } // if(which == F("TACTIVE"))
	  else
      if(which == F("WINDSETT")) // CTSET=LOGIC|WINDSETT|wind speed close border|hurricane close border
      {
        if(argsCount > 2)
        {

            GlobalSettings* settings = MainController->GetSettings();
  
            settings->SetWindSpeed(atoi(command.GetArg(1)));
            settings->SetHurricaneSpeed(atoi(command.GetArg(2)));
            
#ifdef USE_WINDOW_MANAGE_MODULE
            ReloadWindowsSettings();
#endif            
          
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
        
      } // if(which == F("WINDSETT"))
     else
     if(which == F("SHADOW"))
     {
        #ifdef USE_SHADOW_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
           
            if(param == STATE_ON) // CTSET=LOGIC|SHADOW|ON|channel (optional)
            {
              // включить затенение
              if(argsCount < 3) // все каналы
              {
                TurnShadow(0,true);
                TurnShadow(1,true);
                TurnShadow(2,true);
              }
              else
              {
                TurnShadow(atoi(command.GetArg(2)),true);
              }
              
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else 
            if(param == STATE_OFF) // CTSET=LOGIC|SHADOW|OFF|channel (optional)
            {
              // выключить затенение
              if(argsCount < 3) // все каналы
              {
                TurnShadow(0,false);
                TurnShadow(1,false);
                TurnShadow(2,false);
              }
              else
              {
                TurnShadow(atoi(command.GetArg(2)),false);
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else
            if(param == WORK_MODE) // CTSET=LOGIC|SHADOW|MODE|(AUTO or MANUAL)
            {
              if(argsCount > 2)
              {
                String mode = command.GetArg(2);

                if(argsCount > 3) // для конкретного канала
                {
                  int channel = atoi(command.GetArg(3));
                  if(mode == WM_AUTOMATIC)
                  {
                    TurnShadowMode(channel,swmAutomatic);
                  }
                  else
                  {
                    TurnShadowMode(channel,swmManual);
                  }
                  
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;            
                }
                else
                {
                    if(mode == WM_AUTOMATIC) // CTSET=LOGIC|SHADOW|MODE|AUTO
                    {
                      TurnShadowMode(0,swmAutomatic);
                      TurnShadowMode(1,swmAutomatic);
                      TurnShadowMode(2,swmAutomatic);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                    else
                    if(mode == WM_MANUAL) // CTSET=LOGIC|SHADOW|MODE|MANUAL
                    {
                      TurnShadowMode(0,swmManual);
                      TurnShadowMode(1,swmManual);
                      TurnShadowMode(2,swmManual);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                } // else
              } // argsCount > 2
            } // if(param == WORK_MODE)
           else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала затенения, CTSET=LOGIC|SHADOW|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  ShadowSettings* h = channelNum == 0 ? &shadowSection1 : channelNum == 1 ? &shadowSection2 : &shadowSection3;
                  h->active = atoi(command.GetArg(3));

                  MainController->GetSettings()->SetShadowSettings(channelNum,*h);

                  // перезагружаем настройки затенения
                  ReloadShadowSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала затенения, СTSET=LOGIC|SHADOW|CHANNEL|num|lux|histeresis|sensorIndex|active|drive work time
           {
              if(argsCount > 7)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  ShadowSettings* h = channelNum == 0 ? &shadowSection1 : channelNum == 1 ? &shadowSection2 : &shadowSection3;
                  
                  h->lux = atol(command.GetArg(3));
                  h->histeresis = atoi(command.GetArg(4));
                  h->sensorIndex = atoi(command.GetArg(5));
                  h->active = atoi(command.GetArg(6));

                  MainController->GetSettings()->SetShadowDriveWorkTime(channelNum,atoi(command.GetArg(7)));
                  MainController->GetSettings()->SetShadowSettings(channelNum,*h);

                  // перезагружаем настройки затенения
                  ReloadShadowSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 7)
              else
              if(argsCount > 3) // настройки канала затенения, СTSET=LOGIC|SHADOW|CHANNEL|num|lux
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  
                  ShadowSettings* h = channelNum == 0 ? &shadowSection1 : channelNum == 1 ? &shadowSection2 : &shadowSection3;
                  
                  h->lux = atol(command.GetArg(3));

                  MainController->GetSettings()->SetShadowSettings(channelNum,*h);

                  // перезагружаем настройки затенения
                  ReloadShadowSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 3)
            
           } // if(param == F("CHANNEL"))
           
          } // argsCount > 1
        #endif // USE_SHADOW_MODULE
     } // if(which == F("SHADOW"))
     else
     if(which == F("CVENT"))
     {
        #ifdef USE_CYCLE_VENT_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
           
            if(param == STATE_ON) // CTSET=LOGIC|CVENT|ON|channel (optional)
            {
              // включить воздухообмен
              if(argsCount < 3) // все каналы
              {
                cycleVent1.turn(true);
                cycleVent2.turn(true);
                cycleVent3.turn(true);

                cycleVent1.switchToMode(cvwmManual);
                cycleVent2.switchToMode(cvwmManual);
                cycleVent3.switchToMode(cvwmManual);
              }
              else
              {
                CycleVent* cv = getCycleVent(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(true);
                  cv->switchToMode(cvwmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else 
            if(param == STATE_OFF) // CTSET=LOGIC|CVENT|OFF|channel (optional)
            {
              // выключить воздухообмен
              if(argsCount < 3) // все каналы
              {
                cycleVent1.turn(false);
                cycleVent2.turn(false);
                cycleVent3.turn(false);

                cycleVent1.switchToMode(cvwmManual);
                cycleVent2.switchToMode(cvwmManual);
                cycleVent3.switchToMode(cvwmManual);
                
              }
              else
              {
                CycleVent* cv = getCycleVent(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(false);
                  cv->switchToMode(cvwmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else
            if(param == WORK_MODE) // CTSET=LOGIC|CVENT|MODE|(AUTO or MANUAL)
            {
              if(argsCount > 2)
              {
                String mode = command.GetArg(2);

                if(argsCount > 3)
                {
                  int channel = atoi(command.GetArg(3));
                  CycleVent* cv = getCycleVent(channel);
                  
                  if(mode == WM_AUTOMATIC)
                    cv->switchToMode(cvwmAuto);
                  else
                   cv->switchToMode(cvwmManual);

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;   
                }
                else
                {
                  if(mode == WM_AUTOMATIC) // CTSET=LOGIC|CVENT|MODE|AUTO
                  {
                    cycleVent1.switchToMode(cvwmAuto);
                    cycleVent2.switchToMode(cvwmAuto);
                    cycleVent3.switchToMode(cvwmAuto);
                    
                    PublishSingleton.Flags.Status = true;
                    PublishSingleton = which;
                    PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                  }
                  else
                  if(mode == WM_MANUAL) // CTSET=LOGIC|CVENT|MODE|MANUAL
                  {
                    cycleVent1.switchToMode(cvwmManual);
                    cycleVent2.switchToMode(cvwmManual);
                    cycleVent3.switchToMode(cvwmManual);
                    
                    PublishSingleton.Flags.Status = true;
                    PublishSingleton = which;
                    PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                  }
                }
              } // argsCount > 2
            } // if(param == WORK_MODE)
            else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала воздухообмена, CTSET=LOGIC|CVENT|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  CycleVentSettings* h = channelNum == 0 ? &cycleVentSettings1 : channelNum == 1 ? &cycleVentSettings2 : &cycleVentSettings3;
                  h->active = atoi(command.GetArg(3));

                  MainController->GetSettings()->SetCycleVentSettings(channelNum,*h);

                  // перезагружаем настройки воздухообмена
                  ReloadCycleVentSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала воздухообмена, СTSET=LOGIC|CVENT|CHANNEL|num|active|weekdays|startTime|endTime|workTime|idleTime
           {
              if(argsCount > 8)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  CycleVentSettings* h = channelNum == 0 ? &cycleVentSettings1 : channelNum == 1 ? &cycleVentSettings2 : &cycleVentSettings3;
                  
                  h->active = atoi(command.GetArg(3));
                  h->weekdays = atoi(command.GetArg(4));
                  h->startTime = atol(command.GetArg(5));
                  h->endTime = atol(command.GetArg(6));
                  h->workTime = atoi(command.GetArg(7));
                  h->idleTime = atoi(command.GetArg(8));

                  MainController->GetSettings()->SetCycleVentSettings(channelNum,*h);

                  // перезагружаем настройки воздухообмена
                  ReloadCycleVentSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 8)
            
           } // if(param == F("CHANNEL"))
           
          } // argsCount > 1
        #endif // USE_CYCLE_VENT_MODULE
      
     } // if(which == F("CVENT"))
     else
     if(which == F("VENT"))
     {
        #ifdef USE_VENT_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
           
            if(param == STATE_ON) // CTSET=LOGIC|VENT|ON|channel (optional)
            {
              // включить вентиляцию
              if(argsCount < 3) // все каналы
              {
                vent1.turn(true);
                vent2.turn(true);
                vent3.turn(true);

                vent1.switchToMode(vwmManual);
                vent2.switchToMode(vwmManual);
                vent3.switchToMode(vwmManual);
              }
              else
              {
                Vent* cv = getVent(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(true);
                  cv->switchToMode(vwmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else 
            if(param == STATE_OFF) // CTSET=LOGIC|VENT|OFF|channel (optional)
            {
              // выключить вентиляцию
              if(argsCount < 3) // все каналы
              {
                vent1.turn(false);
                vent2.turn(false);
                vent3.turn(false);

                vent1.switchToMode(vwmManual);
                vent2.switchToMode(vwmManual);
                vent3.switchToMode(vwmManual);
              }
              else
              {
                Vent* cv = getVent(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(false);
                  cv->switchToMode(vwmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else
            if(param == WORK_MODE) // CTSET=LOGIC|VENT|MODE|(AUTO or MANUAL)
            {
              if(argsCount > 2)
              {
                String mode = command.GetArg(2);

                if(argsCount > 3)
                {
                  int channel = atoi(command.GetArg(3));
                  Vent* cv = channel == 0 ? &vent1 : channel == 1 ? &vent2 : &vent3;
                  
                  if(mode == WM_AUTOMATIC)
                    cv->switchToMode(vwmAuto);
                  else
                   cv->switchToMode(vwmManual);

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;   
                }
                else
                {
                    if(mode == WM_AUTOMATIC) // CTSET=LOGIC|VENT|MODE|AUTO
                    {
                      vent1.switchToMode(vwmAuto);
                      vent2.switchToMode(vwmAuto);
                      vent3.switchToMode(vwmAuto);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                    else
                    if(mode == WM_MANUAL) // CTSET=LOGIC|VENT|MODE|MANUAL
                    {
                      vent1.switchToMode(vwmManual);
                      vent2.switchToMode(vwmManual);
                      vent3.switchToMode(vwmManual);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                }
              } // argsCount > 2
            } // if(param == WORK_MODE)
            else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала вентиляции, CTSET=LOGIC|VENT|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  Vent* v = channelNum == 0 ? &vent1 : channelNum == 1 ? &vent2 : &vent3;
                  VentSettings vs = v->getSettings();
                  vs.active = atoi(command.GetArg(3));
                  
                  MainController->GetSettings()->SetVentSettings(channelNum,vs);

                  // перезагружаем настройки вентиляции
                  ReloadVentSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала вентиляции, СTSET=LOGIC|VENT|CHANNEL|num|active|sensorIndex|temp|histeresis|minWorkTime|maxWorkTime|restTime
           {
              if(argsCount > 9)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  Vent* v = channelNum == 0 ? &vent1 : channelNum == 1 ? &vent2 : &vent3;
                  VentSettings vs = v->getSettings();
                  
                  vs.active = atoi(command.GetArg(3));
                  vs.sensorIndex = atoi(command.GetArg(4));
                  vs.temp = atoi(command.GetArg(5));
                  vs.histeresis = atoi(command.GetArg(6));
                  vs.minWorkTime = atoi(command.GetArg(7));
                  vs.maxWorkTime = atoi(command.GetArg(8));
                  vs.restTime = atoi(command.GetArg(9));

                  MainController->GetSettings()->SetVentSettings(channelNum,vs);

                  // перезагружаем настройки вентиляции
                  ReloadVentSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 9)
            
           } // if(param == F("CHANNEL"))
           
          } // argsCount > 1
        #endif // USE_VENT_MODULE
     } // if(which == F("VENT"))
////////////////////////////////////////////////////////////////////////////
     else
     if(which == F("HSPRAY"))
     {
        #ifdef USE_HUMIDITY_SPRAY_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
           
            if(param == STATE_ON) // CTSET=LOGIC|HSPRAY|ON|channel (optional)
            {
              // включить распрыскивание по каналу
              if(argsCount < 3) // все каналы
              {
                spray1.turn(true);
                spray2.turn(true);
                spray3.turn(true);

                spray1.switchToMode(hsmManual);
                spray2.switchToMode(hsmManual);
                spray3.switchToMode(hsmManual);
              }
              else
              {
                HumiditySpray* cv = getSpray(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(true);
                  cv->switchToMode(hsmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else 
            if(param == STATE_OFF) // CTSET=LOGIC|HSPRAY|OFF|channel (optional)
            {
              // выключить распрыскивание по каналу
              if(argsCount < 3) // все каналы
              {
                spray1.turn(false);
                spray2.turn(false);
                spray3.turn(false);

                spray1.switchToMode(hsmManual);
                spray2.switchToMode(hsmManual);
                spray3.switchToMode(hsmManual);
              }
              else
              {
                HumiditySpray* cv = getSpray(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(false);
                  cv->switchToMode(hsmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else
            if(param == WORK_MODE) // CTSET=LOGIC|HSPRAY|MODE|(AUTO or MANUAL)
            {
              if(argsCount > 2)
              {
                String mode = command.GetArg(2);

                if(argsCount > 3)
                {
                  int channel = atoi(command.GetArg(3));
                  HumiditySpray* cv = channel == 0 ? &spray1 : channel == 1 ? &spray2 : &spray3;
                  
                  if(mode == WM_AUTOMATIC)
                    cv->switchToMode(hsmAuto);
                  else
                   cv->switchToMode(hsmManual);

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;   
                }
                else
                {
                    if(mode == WM_AUTOMATIC) // CTSET=LOGIC|HSPRAY|MODE|AUTO
                    {
                      spray1.switchToMode(hsmAuto);
                      spray2.switchToMode(hsmAuto);
                      spray3.switchToMode(hsmAuto);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                    else
                    if(mode == WM_MANUAL) // CTSET=LOGIC|HSPRAY|MODE|MANUAL
                    {
                      spray1.switchToMode(hsmManual);
                      spray2.switchToMode(hsmManual);
                      spray3.switchToMode(hsmManual);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                }
              } // argsCount > 2
            } // if(param == WORK_MODE)
            else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала распрыскивания, CTSET=LOGIC|HSPRAY|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HumiditySpray* v = channelNum == 0 ? &spray1 : channelNum == 1 ? &spray2 : &spray3;
                  HumiditySpraySettings vs = v->getSettings();
                  vs.active = atoi(command.GetArg(3));
                  
                  MainController->GetSettings()->SetHumiditySpraySettings(channelNum,vs);

                  // перезагружаем настройки распрыскивания
                  ReloadHumiditySpraySettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала распрыскивания, СTSET=LOGIC|HSPRAY|CHANNEL|num|active|sensorIndex|sprayOnValue|sprayOffValue|histeresis|startWorkTime|endWorkTime
           {
              if(argsCount > 9)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HumiditySpray* v = channelNum == 0 ? &spray1 : channelNum == 1 ? &spray2 : &spray3;
                  HumiditySpraySettings vs = v->getSettings();
                  
                  vs.active = atoi(command.GetArg(3));
                  vs.sensorIndex = atoi(command.GetArg(4));
                  vs.sprayOnValue = atoi(command.GetArg(5));
                  vs.sprayOffValue = atoi(command.GetArg(6));
                  vs.histeresis = atoi(command.GetArg(7));
                  vs.startWorkTime = atoi(command.GetArg(8));
                  vs.endWorkTime = atoi(command.GetArg(9));

                  MainController->GetSettings()->SetHumiditySpraySettings(channelNum,vs);

                  // перезагружаем настройки вентиляции
                  ReloadHumiditySpraySettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 9)
              else
              if(argsCount > 4) // настройки канала распрыскивания, СTSET=LOGIC|HSPRAY|CHANNEL|num|sprayOnValue|sprayOffValue
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  HumiditySpray* v = channelNum == 0 ? &spray1 : channelNum == 1 ? &spray2 : &spray3;
                  HumiditySpraySettings vs = v->getSettings();
                  
                  vs.sprayOnValue = atoi(command.GetArg(3));
                  vs.sprayOffValue = atoi(command.GetArg(4));

                  MainController->GetSettings()->SetHumiditySpraySettings(channelNum,vs);

                  // перезагружаем настройки вентиляции
                  ReloadHumiditySpraySettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)                
              } // if(argsCount > 4)
            
           } // if(param == F("CHANNEL"))
           
          } // argsCount > 1
        #endif // USE_HUMIDITY_SPRAY_MODULE
     } // if(which == F("HSPRAY"))
////////////////////////////////////////////////////////////////////////////     
     else
     if(which == F("THERMOSTAT"))
     {
        #ifdef USE_THERMOSTAT_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
           
            if(param == STATE_ON) // CTSET=LOGIC|THERMOSTAT|ON|channel (optional)
            {
              // включить термостат
              if(argsCount < 3) // все каналы
              {
                thermostat1.turn(true);
                thermostat2.turn(true);
                thermostat3.turn(true);

                thermostat1.switchToMode(twmManual);
                thermostat2.switchToMode(twmManual);
                thermostat3.switchToMode(twmManual);
              }
              else
              {
                Thermostat* cv = getThermostat(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(true);
                  cv->switchToMode(twmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else 
            if(param == STATE_OFF) // CTSET=LOGIC|THERMOSTAT|OFF|channel (optional)
            {
              // выключить термостат
              if(argsCount < 3) // все каналы
              {
                thermostat1.turn(false);
                thermostat2.turn(false);
                thermostat3.turn(false);

                thermostat1.switchToMode(twmManual);
                thermostat2.switchToMode(twmManual);
                thermostat3.switchToMode(twmManual);
                
              }
              else
              {
                Thermostat* cv = getThermostat(atoi(command.GetArg(2)));
                if(cv)
                {
                  cv->turn(false);
                  cv->switchToMode(twmManual);
                }
              }
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << param;
            }
            else
            if(param == WORK_MODE) // CTSET=LOGIC|THERMOSTAT|MODE|(AUTO or MANUAL)
            {
              if(argsCount > 2)
              {
                String mode = command.GetArg(2);

                if(argsCount > 3) // для конкретного канала
                {
                  int channel = atoi(command.GetArg(3));
                  Thermostat* ts = channel == 2 ? &thermostat3 : channel == 1 ? &thermostat2 : &thermostat1;
                  if(mode == WM_AUTOMATIC)
                  {
                    ts->switchToMode(twmAuto);
                  }
                  else
                  {
                    ts->switchToMode(twmManual);
                  }

                   PublishSingleton.Flags.Status = true;
                   PublishSingleton = which;
                   PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode << PARAM_DELIMITER << channel;                  
                }
                else // для всех каналов
                {
                    if(mode == WM_AUTOMATIC) // CTSET=LOGIC|THERMOSTAT|MODE|AUTO
                    {
                      thermostat1.switchToMode(twmAuto);
                      thermostat2.switchToMode(twmAuto);
                      thermostat3.switchToMode(twmAuto);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                    else
                    if(mode == WM_MANUAL) // CTSET=LOGIC|THERMOSTAT|MODE|MANUAL
                    {
                      thermostat1.switchToMode(twmManual);
                      thermostat2.switchToMode(twmManual);
                      thermostat3.switchToMode(twmManual);
                      
                      PublishSingleton.Flags.Status = true;
                      PublishSingleton = which;
                      PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << mode;
                    }
                }
              } // argsCount > 2
            } // if(param == WORK_MODE)
            else
           if(param == F("ACTIVE")) // вкл/выкл модуль канала термостата, CTSET=LOGIC|THERMOSTAT|ACTIVE|num|active flag
           {
              if(argsCount > 3)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  Thermostat* v = channelNum == 0 ? &thermostat1 : channelNum == 1 ? &thermostat2 : &thermostat3;
                  ThermostatSettings vs = v->getSettings();
                  
                  vs.active = atoi(command.GetArg(3));
                  
                   MainController->GetSettings()->SetThermostatSettings(channelNum,vs);

                  // перезагружаем настройки термостатов
                  ReloadThermostatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << command.GetArg(3);

                }
              }
           } // if(param == F("ACTIVE"))
           else
           if(param == F("CHANNEL")) // настройки канала термостата, СTSET=LOGIC|THERMOSTAT|CHANNEL|num|active|sensorIndex|temp|histeresis
           {
              if(argsCount > 6)
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  Thermostat* v = channelNum == 0 ? &thermostat1 : channelNum == 1 ? &thermostat2 : &thermostat3;
                  ThermostatSettings vs = v->getSettings();
                  
                  vs.active = atoi(command.GetArg(3));
                  vs.sensorIndex = atoi(command.GetArg(4));
                  vs.temp = atoi(command.GetArg(5));
                  vs.histeresis = atoi(command.GetArg(6));

                  MainController->GetSettings()->SetThermostatSettings(channelNum,vs);

                  // перезагружаем настройки термостатов
                  ReloadThermostatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC;
                  
                } // if(channelNum >= 0 && channelNum <= 2)
                
              } // if(argsCount > 6)
              else
              if(argsCount > 3) // настройки канала термостата, СTSET=LOGIC|THERMOSTAT|CHANNEL|num|temp
              {
                int channelNum = atoi(command.GetArg(2));
                if(channelNum >= 0 && channelNum <= 2)
                {
                  Thermostat* v = channelNum == 0 ? &thermostat1 : channelNum == 1 ? &thermostat2 : &thermostat3;
                  ThermostatSettings vs = v->getSettings();
                  
                  vs.temp = atoi(command.GetArg(3));

                  MainController->GetSettings()->SetThermostatSettings(channelNum,vs);

                  // перезагружаем настройки термостатов
                  ReloadThermostatSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum << PARAM_DELIMITER << REG_SUCC; 
                }               
              } // if(argsCount > 3)
            
           } // if(param == F("CHANNEL"))
           
          } // argsCount > 1
        #endif // USE_THERMOSTAT_MODULE
     } // if(which == F("THERMOSTAT"))
     else
     if(which == F("LIGHT"))
     {
        #if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
          if(argsCount > 1)
          {
            String param = command.GetArg(1);           
            if(param == F("SETTINGS")) // CTSET=LOGIC|LIGHT|SETTINGS|hour|histeresis|durationHour|lux|active
            {
              if(argsCount > 6)
              {
                  lightSettings.hour = atoi(command.GetArg(2));
                  lightSettings.histeresis = atoi(command.GetArg(3));
                  lightSettings.durationHour = atoi(command.GetArg(4));
                  lightSettings.lux = atol(command.GetArg(5));
                  lightSettings.active = atoi(command.GetArg(6));

                  MainController->GetSettings()->SetLightSettings(lightSettings);

                  // перезагружаем настройки по контролю света
                  ReloadLightSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param  << PARAM_DELIMITER << REG_SUCC;
              } // if(argsCount > 6)
              else
              if(argsCount > 2) // CTSET=LOGIC|LIGHT|SETTINGS|lux
              {
                  lightSettings.lux = atol(command.GetArg(2));

                  MainController->GetSettings()->SetLightSettings(lightSettings);

                  // перезагружаем настройки по контролю света
                  ReloadLightSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param  << PARAM_DELIMITER << REG_SUCC;                
              } // if(argsCount > 2)
              
            } // if(param == F("SETTINGS"))
             else
           if(param == F("ACTIVE")) // вкл/выкл модуль управления досветкой, CTSET=LOGIC|LIGHT|ACTIVE|active flag
           {
              if(argsCount > 2)
              {
                  lightSettings.active = atoi(command.GetArg(2));

                  MainController->GetSettings()->SetLightSettings(lightSettings);

                  // перезагружаем настройки по контролю света
                  ReloadLightSettings();

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << command.GetArg(2);

              }
           } // if(param == F("ACTIVE"))
            
          } // if(argsCount > 1)
        #endif // #if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
     } // if(which == F("LIGHT"))
		
  } // ctSET
  else
  if(command.GetType() == ctGET)
  {
      
      String which = command.GetArg(0);
      if(which == F("HEAT"))
      {
        // управление отоплением
        #ifdef USE_HEAT_MODULE
          if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|HEAT|STATUS, returns OK=LOGIC|HEAT|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetHeatStatus(0,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
                  GetHeatStatus(1,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;

                  GetHeatStatus(2,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
            } //if(param == F("STATUS"))
			else
            if(param == F("STATE")) // CTGET=LOGIC|HEAT|STATE, returns OK=LOGIC|HEAT|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetHeatStatus(0,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (heatWorkMode1 == hwmAutomatic ? F("AUTO") : F("MANUAL"));

                  GetHeatStatus(1,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (heatWorkMode2 == hwmAutomatic ? F("AUTO") : F("MANUAL"));

                  GetHeatStatus(2,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (heatWorkMode3 == hwmAutomatic ? F("AUTO") : F("MANUAL"));

            }
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|HEAT|ACTIVE, returns OK=LOGIC|HEAT|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetHeatStatus(0,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
                  GetHeatStatus(1,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;

                  GetHeatStatus(2,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|HEAT|CHANNEL|num, returns OK=LOGIC|HEAT|CHANNEL|num|minTemp|maxTemp|ethalonTemp|histeresis|sensorIndex|airSensorIndex|active|drive work time
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        HeatSettings* h = channelNum == 0 ? &heatSection1 : channelNum == 1 ? &heatSection2 : &heatSection3;
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << h->minTemp 
                        << PARAM_DELIMITER << h->maxTemp 
                        << PARAM_DELIMITER << h->ethalonTemp 
                        << PARAM_DELIMITER << h->histeresis
                        << PARAM_DELIMITER << h->sensorIndex
                        << PARAM_DELIMITER << h->airSensorIndex
                        << PARAM_DELIMITER << h->active
                        << PARAM_DELIMITER << MainController->GetSettings()->GetHeatDriveWorkTime(channelNum)
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
          } // if(argsCount > 1)
        #endif // USE_HEAT_MODULE
      } // if(which == F("HEAT"))
      else
      if(which == F("TSETT")) // получить настройки температур для секции окон: CTGET=LOGIC|TSETT|section number, returns OK=LOGIC|TSETT|section number|25% temp|50% temp|75% temp|100% temp|histeresis|sensor|active
      {
        #ifdef USE_WINDOW_MANAGE_MODULE
          if(argsCount > 1)
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;

            uint8_t sectionNum = atoi(command.GetArg(1));
            if(sectionNum > 3)
            {
              sectionNum = 3;
            }

              WManageChannel* ch = &windowsSection1;
              if(sectionNum == 1)
              {
                ch = &windowsSection2;
              }
              else
              if(sectionNum == 2)
              {
                ch = &windowsSection3;
              }
              else
              if(sectionNum == 3)
              {
                ch = &windowsSection4;
              }              
            
            PublishSingleton << PARAM_DELIMITER << sectionNum
            << PARAM_DELIMITER << ch->openTemp25percents 
            << PARAM_DELIMITER << ch->openTemp50percents 
            << PARAM_DELIMITER << ch->openTemp75percents 
            << PARAM_DELIMITER << ch->openTemp100percents 
            << PARAM_DELIMITER << ch->histeresis 
            << PARAM_DELIMITER << ch->sensorIndex 
            << PARAM_DELIMITER << ch->active 
            ;
          } // if(argsCount > 1)
        #endif // #ifdef USE_WINDOW_MANAGE_MODULE
        
      } // if(which == F("TSETT"))
      else
      if(which == F("WBND")) // получить привязку окон по секциям, CTGET=LOGIC|WBND
      {
          #ifdef USE_WINDOW_MANAGE_MODULE
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;

          for(size_t i=0;i<sizeof(windowsBySectionsBinding.binding)/sizeof(windowsBySectionsBinding.binding[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << windowsBySectionsBinding.binding[i];
          }
          #endif // #ifdef USE_WINDOW_MANAGE_MODULE
          
      } // if(which == F("WBND"))
      else
      if(which == F("RAINBND")) // получить привязку окон к датчику дождя, CTGET=LOGIC|RAINBND
      {
          #ifdef USE_WINDOW_MANAGE_MODULE
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;

          for(size_t i=0;i<sizeof(windowsRainBinding.binding)/sizeof(windowsRainBinding.binding[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << windowsRainBinding.binding[i];
          }
          #endif // #ifdef USE_WINDOW_MANAGE_MODULE
          
      } // if(which == F("RAINBND"))
      else
      if(which == F("WORIENT")) // получить привязку окон по сторонам света, CTGET=LOGIC|WORIENT
      {
          #ifdef USE_WINDOW_MANAGE_MODULE
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;

          for(size_t i=0;i<sizeof(windowsOrientationBinding.binding)/sizeof(windowsOrientationBinding.binding[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << windowsOrientationBinding.binding[i];
          }
          #endif // #ifdef USE_WINDOW_MANAGE_MODULE
          
      } // if(which == F("WORIENT"))
      else
      if(which == F("SHADOW"))
      {
          #ifdef USE_SHADOW_MODULE
          if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|SHADOW|STATUS, returns OK=LOGIC|SHADOW|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetShadowStatus(0,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
                  GetShadowStatus(1,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;

                  GetShadowStatus(2,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
            } //if(param == F("STATUS"))
			else
            if(param == F("STATE")) // CTGET=LOGIC|SHADOW|STATE, returns OK=LOGIC|SHADOW|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetShadowStatus(0,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (shadowWorkMode1 == swmAutomatic ? F("AUTO") : F("MANUAL"));

                  GetShadowStatus(1,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (shadowWorkMode2 == swmAutomatic ? F("AUTO") : F("MANUAL"));

                  GetShadowStatus(2,dummy,bIsOn);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  PublishSingleton << PARAM_DELIMITER << (shadowWorkMode3 == swmAutomatic ? F("AUTO") : F("MANUAL"));

            }				
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|SHADOW|ACTIVE, returns OK=LOGIC|SHADOW|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  bool dummy, bIsOn;
                  GetShadowStatus(0,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
                  GetShadowStatus(1,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;

                  GetShadowStatus(2,bIsOn,dummy);
                  PublishSingleton << PARAM_DELIMITER << bIsOn;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|SHADOW|CHANNEL|num, returns OK=LOGIC|SHADOW|CHANNEL|num|lux|histeresis|sensorIndex|active|drive work time
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        ShadowSettings* h = channelNum == 0 ? &shadowSection1 : channelNum == 1 ? &shadowSection2 : &shadowSection3;
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << h->lux 
                        << PARAM_DELIMITER << h->histeresis 
                        << PARAM_DELIMITER << h->sensorIndex 
                        << PARAM_DELIMITER << h->active
                        << PARAM_DELIMITER << MainController->GetSettings()->GetShadowDriveWorkTime(channelNum)
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
            
          } // if(argsCount > 1)
          #endif // USE_SHADOW_MODULE
      } // if(which == F("SHADOW"))
      else
      if(which == F("CVENT"))
      {
        #ifdef USE_CYCLE_VENT_MODULE
          if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|CVENT|STATUS, returns OK=LOGIC|CVENT|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << cycleVent1.isOn();                  
                  PublishSingleton << PARAM_DELIMITER << cycleVent2.isOn();
                  PublishSingleton << PARAM_DELIMITER << cycleVent3.isOn();
                  
            } //if(param == F("STATUS"))
			else
            if(param == F("STATE")) // CTGET=LOGIC|CVENT|STATE, returns OK=LOGIC|CVENT|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << cycleVent1.isOn();
                  PublishSingleton << PARAM_DELIMITER << (cycleVent1.getWorkMode() == cvwmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << cycleVent2.isOn();
                  PublishSingleton << PARAM_DELIMITER << (cycleVent2.getWorkMode() == cvwmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << cycleVent3.isOn();
                  PublishSingleton << PARAM_DELIMITER << (cycleVent3.getWorkMode() == cvwmAuto ? F("AUTO") : F("MANUAL"));

            }
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|CVENT|ACTIVE, returns OK=LOGIC|CVENT|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << cycleVentSettings1.active;
                  PublishSingleton << PARAM_DELIMITER << cycleVentSettings2.active;
                  PublishSingleton << PARAM_DELIMITER << cycleVentSettings3.active;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|CVENT|CHANNEL|num, returns OK=LOGIC|CVENT|CHANNEL|num|active|weekdays|startTime|endTime|workTime|idleTime
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        CycleVentSettings* h = channelNum == 0 ? &cycleVentSettings1 : channelNum == 1 ? &cycleVentSettings2 : &cycleVentSettings3;
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << h->active 
                        << PARAM_DELIMITER << h->weekdays 
                        << PARAM_DELIMITER << h->startTime 
                        << PARAM_DELIMITER << h->endTime
                        << PARAM_DELIMITER << h->workTime
                        << PARAM_DELIMITER << h->idleTime
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
            
          } // if(argsCount > 1)        
        #endif // USE_CYCLE_VENT_MODULE
      } // if(which == F("CVENT"))
      else
      if(which == F("VENT"))
      {
        #ifdef USE_VENT_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|VENT|STATUS, returns OK=LOGIC|VENT|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << vent1.isOn();                  
                  PublishSingleton << PARAM_DELIMITER << vent2.isOn();
                  PublishSingleton << PARAM_DELIMITER << vent3.isOn();
                  
            } //if(param == F("STATUS"))
            else
            if(param == F("STATE")) // CTGET=LOGIC|VENT|STATE, returns OK=LOGIC|VENT|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << vent1.isOn();
                  PublishSingleton << PARAM_DELIMITER << (vent1.getWorkMode() == vwmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << vent2.isOn();
                  PublishSingleton << PARAM_DELIMITER << (vent2.getWorkMode() == vwmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << vent3.isOn();
                  PublishSingleton << PARAM_DELIMITER << (vent3.getWorkMode() == vwmAuto ? F("AUTO") : F("MANUAL"));

            }            
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|VENT|ACTIVE, returns OK=LOGIC|VENT|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << vent1.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << vent2.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << vent3.getSettings().active;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|VENT|CHANNEL|num, returns OK=LOGIC|VENT|CHANNEL|num|active|sensorIndex|temp|histeresis|minWorkTime|maxWorkTime|restTime
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        Vent* v = channelNum == 0 ? &vent1 : channelNum == 1 ? &vent2 : &vent3;
                        VentSettings vs = v->getSettings();
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << vs.active 
                        << PARAM_DELIMITER << vs.sensorIndex 
                        << PARAM_DELIMITER << vs.temp 
                        << PARAM_DELIMITER << vs.histeresis
                        << PARAM_DELIMITER << vs.minWorkTime
                        << PARAM_DELIMITER << vs.maxWorkTime
                        << PARAM_DELIMITER << vs.restTime
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
            
          } // if(argsCount > 1)        
        #endif // USE_VENT_MODULE
      } // if(which == F("VENT"))
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
      else
      if(which == F("HSPRAY"))
      {
        #ifdef USE_HUMIDITY_SPRAY_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|HSPRAY|STATUS, returns OK=LOGIC|HSPRAY|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << spray1.isOn();                  
                  PublishSingleton << PARAM_DELIMITER << spray2.isOn();
                  PublishSingleton << PARAM_DELIMITER << spray3.isOn();
                  
            } //if(param == F("STATUS"))
            else
            if(param == F("STATE")) // CTGET=LOGIC|HSPRAY|STATE, returns OK=LOGIC|HSPRAY|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << spray1.isOn();
                  PublishSingleton << PARAM_DELIMITER << (spray1.getWorkMode() == hsmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << spray2.isOn();
                  PublishSingleton << PARAM_DELIMITER << (spray2.getWorkMode() == hsmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << spray3.isOn();
                  PublishSingleton << PARAM_DELIMITER << (spray3.getWorkMode() == hsmAuto ? F("AUTO") : F("MANUAL"));

            }            
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|HSPRAY|ACTIVE, returns OK=LOGIC|HSPRAY|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << spray1.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << spray2.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << spray3.getSettings().active;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|HSPRAY|CHANNEL|num, returns OK=LOGIC|HSPRAY|CHANNEL|num|active|sensorIndex|sprayOnValue|sprayOffValue|histeresis|startWorkTime|endWorkTime
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        HumiditySpray* v = channelNum == 0 ? &spray1 : channelNum == 1 ? &spray2 : &spray3;
                        HumiditySpraySettings vs = v->getSettings();
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << vs.active 
                        << PARAM_DELIMITER << vs.sensorIndex 
                        << PARAM_DELIMITER << vs.sprayOnValue 
                        << PARAM_DELIMITER << vs.sprayOffValue
                        << PARAM_DELIMITER << vs.histeresis
                        << PARAM_DELIMITER << vs.startWorkTime
                        << PARAM_DELIMITER << vs.endWorkTime
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
            
          } // if(argsCount > 1)        
        #endif // USE_HUMIDITY_SPRAY_MODULE
      } // if(which == F("HSPRAY"))
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////      
      else
      if(which == F("THERMOSTAT"))
      {
        #ifdef USE_THERMOSTAT_MODULE
        if(argsCount > 1)
          {
            String param = command.GetArg(1);
            
            if(param == F("STATUS")) // CTGET=LOGIC|THERMOSTAT|STATUS, returns OK=LOGIC|THERMOSTAT|STATUS|section1 on| section2 on|section3 on
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << thermostat1.isOn();                  
                  PublishSingleton << PARAM_DELIMITER << thermostat2.isOn();
                  PublishSingleton << PARAM_DELIMITER << thermostat3.isOn();
                  
            } //if(param == F("STATUS"))
			else
            if(param == F("STATE")) // CTGET=LOGIC|THERMOSTAT|STATE, returns OK=LOGIC|THERMOSTAT|STATE|section1 on|section1 work mode|section2 on|section2 work mode|section3 on|section3 work mode
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << thermostat1.isOn();
                  PublishSingleton << PARAM_DELIMITER << (thermostat1.getWorkMode() == twmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << thermostat2.isOn();
                  PublishSingleton << PARAM_DELIMITER << (thermostat2.getWorkMode() == twmAuto ? F("AUTO") : F("MANUAL"));

                  PublishSingleton << PARAM_DELIMITER << thermostat3.isOn();
                  PublishSingleton << PARAM_DELIMITER << (thermostat3.getWorkMode() == twmAuto ? F("AUTO") : F("MANUAL"));

            }
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|THERMOSTAT|ACTIVE, returns OK=LOGIC|THERMOSTAT|ACTIVE|section1 active| section2 active|section3 active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << thermostat1.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << thermostat2.getSettings().active;
                  PublishSingleton << PARAM_DELIMITER << thermostat3.getSettings().active;
                  
            } //if(param == F("ACTIVE"))
            else
            if(param == F("CHANNEL")) // CTGET=LOGIC|THERMOSTAT|CHANNEL|num, returns OK=LOGIC|THERMOSTAT|CHANNEL|num|active|sensorIndex|temp|histeresis
            {
                  if(argsCount > 2)
                  {
                    int channelNum = atoi(command.GetArg(2));
                    if(channelNum >= 0 && channelNum <= 2)
                    {
                        Thermostat* v = channelNum == 0 ? &thermostat1 : channelNum == 1 ? &thermostat2 : &thermostat3;
                        ThermostatSettings vs = v->getSettings();
                  
                        PublishSingleton.Flags.Status = true;
                        PublishSingleton = which;
                        PublishSingleton << PARAM_DELIMITER << param << PARAM_DELIMITER << channelNum;

                        PublishSingleton << PARAM_DELIMITER << vs.active 
                        << PARAM_DELIMITER << vs.sensorIndex 
                        << PARAM_DELIMITER << vs.temp 
                        << PARAM_DELIMITER << vs.histeresis
                        ;
                    }
                  }
                  
            } // if(param == F("CHANNEL"))
            
          } // if(argsCount > 1)
        #endif //  USE_THERMOSTAT_MODULE
      } // if(which == F("THERMOSTAT"))
      else
      if(which == F("LIGHT"))
      {
        #if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
        if(argsCount > 1)
        {
          String param = command.GetArg(1);
            
            if(param == F("SETTINGS")) // CTGET=LOGIC|LIGHT|SETTINGS, returns OK=LOGIC|LIGHT|SETTINGS|hour|histeresis|durationHour|lux|active
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << lightSettings.hour
                   << PARAM_DELIMITER << lightSettings.histeresis
                   << PARAM_DELIMITER << lightSettings.durationHour
                   << PARAM_DELIMITER << lightSettings.lux
                   << PARAM_DELIMITER << lightSettings.active
                   ;
                  
            } //if(param == F("SETTINGS"))
            else
            if(param == F("ACTIVE")) // CTGET=LOGIC|LIGHT|ACTIVE
            {
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which;
                  PublishSingleton << PARAM_DELIMITER << param;

                  PublishSingleton << PARAM_DELIMITER << lightSettings.active
                   ;

            } // if(param == F("ACTIVE"))
        } // if(argsCount > 1)
        #endif
        
      } // if(which == F("LIGHT"))
	  #ifdef USE_WINDOW_MANAGE_MODULE
      else
      if(which == F("RAIN")) // CTGET=LOGIC|RAIN, returns OK=LOGIC|RAIN|rain flag
      {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << ( HasRain() ? 1 : 0 );
      } // if(which == F("RAIN"))
      else
      if(which == F("WIND")) // CTGET=LOGIC|WIND, returns OK=LOGIC|WIND|wind speed|wind direction
      {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << GetWindSpeed() << PARAM_DELIMITER << (uint8_t) GetWindDirection();
      } // if(which == F("WIND"))
      else
      if(which == F("WINDSPEED")) // CTGET=LOGIC|WINDSPEED, returns OK=LOGIC|WINDSPEED|wind speed
      {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << GetWindSpeed();
      } // if(which == F("WINDSPEED"))
      else
      if(which == F("WINDDIR")) // CTGET=LOGIC|WINDDIR, returns OK=LOGIC|WINDDIR|wind direction
      {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER << (uint8_t) GetWindDirection();
      } // if(which == F("WINDDIR"))
	  else
      if(which == F("WINDSETT")) // CTGET=LOGIC|WINDSETT, returns OK=LOGIC|WINDSETT|wind speed close border|hurricane close border
      {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          PublishSingleton << PARAM_DELIMITER <<windSpeed << PARAM_DELIMITER << hurricaneSpeed;
      } // if(which == F("WIND"))
	#endif
      
  } // ctGET
  
	MainController->Publish(this,command);
	return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// WindSensor
//--------------------------------------------------------------------------------------------------------------------------------------
WindSensor* _wind_sensor = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
CompassPoints* compassSamples = NULL;;
uint16_t compassSamplesWriteIterator = 0;
#include <stdlib.h>
//--------------------------------------------------------------------------------------------------------------------------------------
int sort_desc(const void *cmp1, const void *cmp2)
{
  CompassCompare a = *((CompassCompare *)cmp1);
  CompassCompare b = *((CompassCompare *)cmp2);

  return a > b ? -1 : (a < b ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
CompassPoints computeWindDirection(CompassPoints dir, uint8_t numSamples)
{
  if(dir == cpUnknown)
  {
    return dir;
  }

  if(!compassSamples)
    return cpUnknown;

  compassSamples[compassSamplesWriteIterator] = dir;
  compassSamplesWriteIterator++;
  
  if(compassSamplesWriteIterator >= numSamples)
    compassSamplesWriteIterator = 0;

  CompassPoints result = cpUnknown;
  
  // тут ищем в массиве значение направления, у которого наибольшее кол-во вхождений
  uint16_t eastCount = 0;
  uint16_t westCount = 0;
  uint16_t northCount = 0;
  uint16_t southCount = 0;

  for(uint16_t i=0;i<numSamples; i++)
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
//--------------------------------------------------------------------------------------------------------------------------------------
bool DirectionFound(uint16_t adcVal, uint16_t* buff, size_t buffSz, uint8_t histeresis)
{
  if(!buff || !buffSz)
    return false;

  for(size_t i=0;i<buffSz;i++)
  {
      int32_t ethalonVal = buff[i];
      int32_t lowBorder = ethalonVal - histeresis;
      int32_t highBorder = ethalonVal + histeresis;

      if(adcVal >= lowBorder && adcVal <= highBorder)
        return true;
      
  } // for

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindSensor::WindSensor()
{
	_wind_sensor = this;
	windDirection = cpUnknown;
	pulses = 0;
	timer = 0;
	windSpeed = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindSensor::~WindSensor()
{	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindSensor::setup()
{

  WindSensorBinding bnd = HardwareBinding->GetWindSensorBinding();

  if(bnd.WorkMode == wrsDirect) // датчик прикреплён напрямую к контроллеру
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          attachInterrupt(digitalPinToInterrupt(bnd.Pin),pulse,WIND_SENSOR_INT_LEVEL);
        }
    }

    compassSamples = new CompassPoints[bnd.NumSamples];
    for(uint8_t i=0;i<bnd.NumSamples;i++)
    {
      compassSamples[i] = cpUnknown;
    }
    
  } // if(bnd.WorkMode == wrsDirect)
  else
  if(bnd.WorkMode == wrsMisol) // работаем через метеостанцию типа Misol
  {
    if(bnd.Pin != UNBINDED_PIN)
    {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          WeatherStation.setup(bnd.Pin);
        }
    }    
  } // if(bnd.WorkMode == wrsMisol)
  

	lastDataAt = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindSensor::pulse()
{
	_wind_sensor->pulses++;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindSensor::setWindSpeed(uint32_t ws) 
{ 
  // установка скорости ветра извне, может применяться только тогда, когда режим работы - внешний модуль!!!
  
  WindSensorBinding bnd = HardwareBinding->GetWindSensorBinding();
  if(!(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol)) // не через внешний модуль работаем и не верез метеостанцию типа Misol
  {
    lastDataAt = millis(); 
    return;
  }
  
  windSpeed = ws; 
  lastDataAt = millis(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindSensor::setWindDirection(CompassPoints cp) 
{ 
  // установка направления ветра извне, может применяться только тогда, когда режим работы - внешний модуль!!!
  
  WindSensorBinding bnd = HardwareBinding->GetWindSensorBinding();
  if(!(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol)) // не через внешний модуль работаем и не верез метеостанцию типа Misol
  {
    lastDataAt = millis(); 
    return;
  }
  
  windDirection = cp; 
  lastDataAt = millis(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindSensor::update()
{

 WindSensorBinding bnd = HardwareBinding->GetWindSensorBinding();

 if(bnd.WorkMode == wrsDirect) // датчик прикреплён к контроллеру
 {
    
    // используем прикреплённый к контроллеру датчик
    uint32_t elapsed = millis() - timer;    
    
    if(elapsed >= WIND_SENSOR_UPDATE_INTERVAL)
    {    
      if(bnd.Pin != UNBINDED_PIN)
      {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          detachInterrupt(digitalPinToInterrupt(bnd.Pin));
        }
      }
      
      uint32_t thisPulses = pulses;
      pulses = 0;
      timer = millis();

     if(bnd.Pin != UNBINDED_PIN)
     {
        if(EEPROMSettingsModule::SafePin(bnd.Pin))
        {
          attachInterrupt(digitalPinToInterrupt(bnd.Pin),pulse,WIND_SENSOR_INT_LEVEL);
        }
     }
      
      // теперь считаем герцы. у нас есть кол-во импульсов за интервал elapsed.
      // пересчитываем всё в импульсы в секунду, т.е. в герцы.
      
      float herz = float(thisPulses*1000ul)/(1.0f*elapsed);

      //Serial.print("Pulses: ");
      //Serial.println(thisPulses);
      
      //Serial.print("Herz: ");
      //Serial.println(herz);
      
      // теперь считаем по формуле китайцев
      float ws = herz*((1.*bnd.SpeedCoeff)/100);
      
      // теперь переводим в сотые доли
      windSpeed = ws*100;
      
      //Serial.print("Wind speed: ");
      //Serial.println(windSpeed);


      // получаем направление ветра
      uint16_t adcVal = 0;
      
      if(bnd.DirectionPin != UNBINDED_PIN)
      {
        if(EEPROMSettingsModule::SafePin(bnd.DirectionPin))
        {
          adcVal = analogRead(bnd.DirectionPin);
        }
      }

      CompassPoints thisDirection = cpUnknown;

      uint16_t east_ADC[] = {bnd.EastAdcFrom, bnd.EastAdcTo};
      uint16_t south_ADC[] = {bnd.SouthAdcFrom,bnd.SouthAdcTo};
      uint16_t west_ADC[] = {bnd.WestAdcFrom,bnd.WestAdcTo};
      uint16_t north_ADC[] = {bnd.NorthAdcFrom,bnd.NorthAdcTo};
  
      if(DirectionFound(adcVal,east_ADC,sizeof(east_ADC)/sizeof(east_ADC[0]),bnd.Histeresis))
      {
        thisDirection = cpEast;
      }
      else
      if(DirectionFound(adcVal,south_ADC,sizeof(south_ADC)/sizeof(south_ADC[0]),bnd.Histeresis))
      {
        thisDirection = cpSouth;
      }
      else
      if(DirectionFound(adcVal,west_ADC,sizeof(west_ADC)/sizeof(west_ADC[0]),bnd.Histeresis))
      {
        thisDirection = cpWest;
      }
      else
      if(DirectionFound(adcVal,north_ADC,sizeof(north_ADC)/sizeof(north_ADC[0]),bnd.Histeresis))
      {
        thisDirection = cpNorth;
      }

      windDirection = computeWindDirection(thisDirection,bnd.NumSamples);
      
      lastDataAt = millis();
    }

 } // if(bnd.WorkMode == wrsDirect)
 else
 if(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol) // датчик прикреплён на выносной модуль или работаем через метеостанцию типа Misol
 {
    if(bnd.WorkMode == wrsMisol) // если работаем через метеостанцию - обновляем её
    {
      WeatherStation.update();
    }
  
    if((millis() - lastDataAt) > 2ul*WIND_RAIN_UPDATE_INTERVAL)
    {
      // очень долго не было данных, сбрасываем
      windSpeed = 0;
      windDirection = cpUnknown;
      lastDataAt = millis();
    }
 } // if(bnd.WorkMode == wrsExternalModule || bnd.WorkMode == wrsMisol)
     
}
//--------------------------------------------------------------------------------------------------------------------------------------
