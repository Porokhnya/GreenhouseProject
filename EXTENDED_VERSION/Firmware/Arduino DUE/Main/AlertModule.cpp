#include "AlertModule.h"
#include "ModuleController.h"
#include "Memory.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AlertModule* RulesDispatcher = NULL;
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AlertModule::AlertModule() : AbstractModule("ALERT") 
{
      RulesDispatcher = this;
      InitRules();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AlertRule::~AlertRule()
{
  delete[] rawCommand;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AlertRule::AlertRule()
{
  rawCommand = NULL;
  linkedModule = NULL;
  
  Settings.StartTime = 0;
  Settings.WorkTime = 0;
  Settings.DayMask = 0xFF; // все дни недели работаем
  Settings.DataAlert = 0;
  Settings.Reserved = 0;

  Settings.Enabled = 1;
  Settings.CanWork = 1;
  Settings.TargetModuleNameIndex = 0;

  Settings.IsAlarm = 0;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t AlertRule::GetKnownModuleID(const char* moduleName)
{
  if(!strcmp_P(moduleName,(const char*) F("STATE")))
      return moduleState;
      
  if(!strcmp_P(moduleName,(const char*) F("PIN")))
    return modulePin;

  if(!strcmp_P(moduleName,(const char*) F("LIGHT")))
    return moduleLight;

  if(!strcmp_P(moduleName,(const char*) F("CC")))
    return moduleCompositeCommands;

  if(!strcmp_P(moduleName,(const char*) F("HUMIDITY")))
    return moduleHumidity;

  if(!strcmp_P(moduleName,(const char*) F("DELTA")))
    return moduleDelta;

  if(!strcmp_P(moduleName,(const char*) F("SOIL")))
    return moduleSoil;
   
  if(!strcmp_P(moduleName,(const char*) F("PH")))
    return modulePH;

  if(!strcmp_P(moduleName,(const char*) F("0")))
    return moduleZero;

  if(!strcmp_P(moduleName,(const char*) F("SCN")))
    return moduleScene;

  if(!strcmp_P(moduleName,(const char*) F("MCP")))
    return moduleMCP;

return 0;

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetLinkedModuleName()
{
  return GetKnownModuleName(Settings.LinkedModuleNameIndex);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetKnownModuleName(uint8_t type)
{
  SD_BUFFER[0] = 0;
  // возвращаем имя известного модуля по типу
  switch(type)
  {
    case moduleZero:
      strcpy_P(SD_BUFFER, (const char*) F("0"));
    break;
    
    case moduleState:
      strcpy_P(SD_BUFFER, (const char*) F("STATE"));
    break;

    case modulePin:
      strcpy_P(SD_BUFFER, (const char*) F("PIN"));
    break;

    case moduleLight:
      strcpy_P(SD_BUFFER, (const char*) F("LIGHT"));
    break;

    case moduleCompositeCommands:
      strcpy_P(SD_BUFFER, (const char*) F("CC"));
    break;

    case moduleHumidity:
      strcpy_P(SD_BUFFER, (const char*) F("HUMIDITY"));
    break;
    
    case moduleDelta:
      strcpy_P(SD_BUFFER, (const char*) F("DELTA"));
    break;
    
    case moduleSoil:
      strcpy_P(SD_BUFFER, (const char*) F("SOIL"));
    break;
    
    case modulePH:
      strcpy_P(SD_BUFFER, (const char*) F("PH"));
    break;

    case moduleScene:
      strcpy_P(SD_BUFFER, (const char*) F("SCN"));
    break;

    case moduleMCP:
      strcpy_P(SD_BUFFER, (const char*) F("MCP"));
    break;
    
    
  }

  return SD_BUFFER;
    
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetTargetCommandModuleName()
{
  return GetKnownModuleName(Settings.TargetModuleNameIndex);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertRule::HasTargetCommand()
{
  if(Settings.TargetCommandType == commandUnparsed)
    return (rawCommand != NULL);

  return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetTargetCommand()
{
  // возвращаем команду на выполнение, БЕЗ имени связанного модуля
  SD_BUFFER[0] = 0;
  switch(Settings.TargetCommandType)
  {
    case commandUnparsed:
      return rawCommand;

    case commandOpenAllWindows:
    {
      strcpy_P(SD_BUFFER,(const char*) F("WINDOW|ALL|OPEN"));
      return SD_BUFFER;
    }

    case commandCloseAllWindows:
    {
      strcpy_P(SD_BUFFER,(const char*) F("WINDOW|ALL|CLOSE"));
      return SD_BUFFER;
    }

    case commandLightOn:
    {
      strcpy_P(SD_BUFFER,(const char*) STATE_ON);
      return SD_BUFFER;
    }
    
    case commandLightOff:
    {
      strcpy_P(SD_BUFFER,(const char*) STATE_OFF);
      return SD_BUFFER;
    }

    case commandExecCompositeCommand:
    {
      strcpy_P(SD_BUFFER,(const char*) F("EXEC"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      String helper = String(Settings.TargetCommandParam);
      strcat(SD_BUFFER,helper.c_str());
      
      return SD_BUFFER;
    }

    case commandExecScene:
    {
      strcpy_P(SD_BUFFER,(const char*) F("EXEC"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      String helper = String(Settings.TargetCommandParam);
      strcat(SD_BUFFER,helper.c_str());
      
      return SD_BUFFER;
    }

    case commandStopScene:
    {
      strcpy_P(SD_BUFFER,(const char*) F("STOP"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      String helper = String(Settings.TargetCommandParam);
      strcat(SD_BUFFER,helper.c_str());
      
      return SD_BUFFER;
    }            

    case commandSetOnePinHigh:
    {
      String helper = String(Settings.TargetCommandParam);
      strcpy(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcat_P(SD_BUFFER,(const char*) STATE_ON);
      
      return SD_BUFFER;
    } 

    case commandSetOnePinLow:
    {
      String helper = String(Settings.TargetCommandParam);
      strcpy(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcat_P(SD_BUFFER,(const char*) F("OFF"));
      
      return SD_BUFFER;
    } 

    case commandMCP23S17PinOn: // CTSET=MCP|SPI|WRITE|mcpNumber|mcpChannel|ON
    {
      strcpy_P(SD_BUFFER,(const char*) F("SPI"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcpy_P(SD_BUFFER,(const char*) F("WRITE"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      String helper = String(Settings.TargetCommandParam); // mcpNumber
      strcat(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      helper = String(Settings.Reserved); // mcpChannel
      strcat(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcat_P(SD_BUFFER,(const char*) STATE_ON);
      return SD_BUFFER;
    } 

    case commandMCP23S17PinOff: // CTSET=MCP|SPI|WRITE|mcpNumber|mcpChannel|OFF
    {
      strcpy_P(SD_BUFFER,(const char*) F("SPI"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcpy_P(SD_BUFFER,(const char*) F("WRITE"));
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      String helper = String(Settings.TargetCommandParam); // mcpNumber
      strcat(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      helper = String(Settings.Reserved); // mcpChannel
      strcat(SD_BUFFER,helper.c_str());
      strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
      strcat_P(SD_BUFFER,(const char*) STATE_OFF);
      return SD_BUFFER;
    }                      
  } // switch

  return rawCommand;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetName()
{
  return RulesDispatcher->GetParam(Settings.RuleNameIndex);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertRule::Update(uint16_t dt
  #ifdef USE_DS3231_REALTIME_CLOCK 
     ,uint8_t currentHour // текущий час
    , uint8_t currentMinute // текущая минута
    ,uint8_t currentDOW // текущий день недели
  #endif
 )
{

  UNUSED(dt);
  
  // считаем, что мы можем работать, если попадаем в текущий день недели
  #ifdef USE_DS3231_REALTIME_CLOCK 
    Settings.CanWork = bitRead(Settings.DayMask,currentDOW-1);
  #else
    Settings.CanWork = 1;
  #endif  

  if(Settings.StartTime == 0  && Settings.WorkTime == 0) // работаем всегда
  {
     return;
  }


  #ifdef USE_DS3231_REALTIME_CLOCK

  // создаём диапазон для проверки
  uint16_t startDia = Settings.StartTime;
  uint16_t stopDia = startDia + Settings.WorkTime;

  // если мы находимся между этим диапазоном, то мы можем работать в это время,
  // иначе - не можем, и просто выставляем флаг работы в false.
  // надо отразить текущее время в этот диапазон. Существует одна особенность диапазона:
  // если он полностью попадает в текущие сутки, то мы просто смотрим, попадает ли
  // текущее время в этот диапазон. Иначе (например, час начала 23, продолжительность - 
  // 120 минут, т.е. работа закончится на следующие сутки, в час ночи) нам надо отразить
  // текущий час на этот диапазон, т.е. виртуально продлить сутки. Для этого к текущему 
  // времени прибавляем кол-во минут в сутках.

  const uint16_t mins_in_day = 1440; // кол-во минут в сутках
  uint16_t checkMinutes = currentHour*60 + currentMinute; // текущее время в минутах
  bool haveOverflow = false; // флаг переноса работы на следующие сутки

  if(stopDia >= mins_in_day)
  {
    // правая граница диапазона перешагнула на следующие сутки,
    // отражаем диапазон текущего часа на следующие сутки
    // только в том случае, если текущее кол-во минут от начала суток меньше, чем время начала работы
    if(checkMinutes < Settings.StartTime)
    {
      checkMinutes += mins_in_day;
      haveOverflow = true;
    }
  }

    // проверяем попадание в диапазон
    bool canWeWork = (checkMinutes >= startDia && checkMinutes <= stopDia);
    if(canWeWork)
    {
      // в диапазон попали, надо проверить попадание в дни недели.
      // считаем, что мы попали в день недели, если он выставлен
      // в флагах или у нас был перенос работы на следующие сутки.
      canWeWork = haveOverflow || bitRead(Settings.DayMask,currentDOW-1);
    }

    Settings.CanWork = canWeWork ? 1 : 0;


  #endif  
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertRule::HasAlert()
{
  if(!linkedModule || !Settings.Enabled || !Settings.CanWork)
    return false;


  switch(Settings.Target)
  {
    case rtTemp: // проверяем температуру
    {
     if(!linkedModule->State.HasState(StateTemperature))  // не поддерживаем температуру
        return false;

     OneState* os = linkedModule->State.GetState(StateTemperature,Settings.SensorIndex);
       
     if(!os) // не срослось
      return false;

       TemperaturePair tp = *os;

       #ifdef ALERT_INCLUDE_COMMA_VALUES
       int8_t sign = tp.Current.Value < 0 ? -1 : 1;
       int curTemp = (abs(tp.Current.Value)*100 + tp.Current.Fract)*sign;
       int tAlert = (int) Settings.DataAlert*100; // следим за переданной температурой
       #else 
       int8_t curTemp = tp.Current.Value;
       int8_t tAlert = (int8_t) Settings.DataAlert; // следим за переданной температурой
       #endif

       if(curTemp == NO_TEMPERATURE_DATA) // нет датчика на линии
       {
        // пытаемся найти резервирование
        OneState* reservedState = MainController->GetReservedState(linkedModule,StateTemperature,Settings.SensorIndex);
        if(!reservedState)
          return (curTemp == tAlert); // на случай, если правило следит за отсутствием показаний с датчика
        else // есть зарезервированный датчик с показаниями
        {
          TemperaturePair tp = *reservedState; 
         
          #ifdef ALERT_INCLUDE_COMMA_VALUES
          int8_t sign = tp.Current.Value < 0 ? -1 : 1;
          curTemp = (abs(tp.Current.Value)*100 + tp.Current.Fract)*sign;
          #else
          curTemp = tp.Current.Value;
          #endif
        } // else
       }

       //------- NEW CODE BEGIN ---------------//
        switch(Settings.DataSource)
       {
          case tsOpenTemperature: // попросили подставить температуру открытия из настроек, делаем это с учётом возможных добавок вида %TO%+5
            tAlert = MainController->GetSettings()->GetOpenTemp() + tAlert;
          break;

          case tsCloseTemperature: // попросили подставить температуру закрытия из настроек, делаем это с учётом возможных добавок вида %TC%-12
            tAlert = MainController->GetSettings()->GetCloseTemp() + tAlert;
          break;

          case tsPassed:
          break;
       }
       //------- NEW CODE END ---------------//

       /*         
       // OLD CODE
       switch(Settings.DataSource)
       {
          case tsOpenTemperature: // попросили подставить температуру открытия из настроек
            tAlert = MainController->GetSettings()->GetOpenTemp();
          break;

          case tsCloseTemperature: // попросили подставить температуру закрытия из настроек
            tAlert = MainController->GetSettings()->GetCloseTemp();
          break;

          case tsPassed:
          break;
       }
       */

       switch(Settings.Operand)
       {
          case roLessThan: return curTemp < tAlert;
          case roLessOrEqual: return curTemp <= tAlert;
          case roGreaterThan: return curTemp > tAlert;
          case roGreaterOrEqual: return curTemp >= tAlert;
          default: return false;
       } // switch
    }  
    break; // rtTemp

    case rtLuminosity: // следим за освещенностью
    {
      if(Settings.DataAlert == -2) 
      {
        // специальное значение, означающее "работать без датчика освещённости"
        return true; // в этом случае считаем, что работать мы можем при любом раскладе
      } // if
      
     if(!linkedModule->State.HasState(StateLuminosity))  // не поддерживаем освещенность
        return false;

       OneState* os = linkedModule->State.GetState(StateLuminosity,Settings.SensorIndex);
       
       if(!os) // не срослось
        return false;

       LuminosityPair lp = *os; 
       long lum = lp.Current;

       if(lum == NO_LUMINOSITY_DATA) // нет датчика на линии
       {
        // пытаемся найти резервирование
        OneState* reservedState = MainController->GetReservedState(linkedModule,StateLuminosity,Settings.SensorIndex);
        if(!reservedState)
          return (lum == Settings.DataAlert); // на случай, если правило следит за отсутствием показаний с датчика
        else // есть зарезервированный датчик с показаниями
          {
            LuminosityPair lp = *reservedState; 
            lum = lp.Current;
          } // else
        }

       switch(Settings.Operand)
       {
          case roLessThan: return lum < Settings.DataAlert;
          case roLessOrEqual: return lum <= Settings.DataAlert;
          case roGreaterThan: return lum > Settings.DataAlert;
          case roGreaterOrEqual: return lum >= Settings.DataAlert;
          default: return false;
       } // switch
      
    }
    break;

    case rtHumidity: // следим за влажностью
    {
     if(!linkedModule->State.HasState(StateHumidity))  // не поддерживаем влажность
        return false;

       OneState* os = linkedModule->State.GetState(StateHumidity,Settings.SensorIndex);
       if(!os) // не срослось
        return false;

       HumidityPair hp = *os;
       #ifdef ALERT_INCLUDE_COMMA_VALUES
       int8_t sign = hp.Current.Value < 0 ? -1 : 1;
       int curHumidity = (abs(hp.Current.Value)*100 + hp.Current.Fract)*sign;
       int humidityAlert = Settings.DataAlert*100;
       #else
       int8_t curHumidity = hp.Current.Value;
       int8_t humidityAlert = Settings.DataAlert;
       #endif

       if(curHumidity == NO_TEMPERATURE_DATA) // нет датчика на линии
       {
           // пытаемся найти резервирование
          OneState* reservedState = MainController->GetReservedState(linkedModule,StateHumidity,Settings.SensorIndex);
          if(!reservedState)
            return (curHumidity == Settings.DataAlert); // на случай, если правило следит за отсутствием показаний с датчика
          else // есть зарезервированный датчик с показаниями
          {
            HumidityPair tp = *reservedState;
            
            #ifdef ALERT_INCLUDE_COMMA_VALUES
            int8_t sign = tp.Current.Value < 0 ? -1 : 1;
            curHumidity = (abs(tp.Current.Value)*100 + tp.Current.Fract)*sign;
            #else 
            curHumidity = tp.Current.Value;
            #endif
          } // else
             
       }

       switch(Settings.Operand)
       {
          case roLessThan: return curHumidity < humidityAlert;
          case roLessOrEqual: return curHumidity <= humidityAlert;
          case roGreaterThan: return curHumidity > humidityAlert;
          case roGreaterOrEqual: return curHumidity >= humidityAlert;
          default: return false;
       } // switch
      
    }
    break;

   case rtSoilMoisture: // следим за влажностью почвы
    {
     if(!linkedModule->State.HasState(StateSoilMoisture))  // не поддерживаем влажность
        return false;

       OneState* os = linkedModule->State.GetState(StateSoilMoisture,Settings.SensorIndex);
       if(!os) // не срослось
        return false;
       
       HumidityPair hp = *os;

       #ifdef ALERT_INCLUDE_COMMA_VALUES
       int8_t sign = hp.Current.Value < 0 ? -1 : 1;
       int curHumidity = (abs(hp.Current.Value)*100 + hp.Current.Fract)*sign;
       int humidityAlert = Settings.DataAlert*100;
       #else
       int8_t curHumidity = hp.Current.Value;
       int8_t humidityAlert = Settings.DataAlert;
       #endif

       if(curHumidity == NO_TEMPERATURE_DATA) // нет датчика на линии
       {
          // пытаемся найти резервирование
          OneState* reservedState = MainController->GetReservedState(linkedModule,StateSoilMoisture,Settings.SensorIndex);
          if(!reservedState)
            return (curHumidity == Settings.DataAlert); // на случай, если правило следит за отсутствием показаний с датчика
          else // есть зарезервированный датчик с показаниями
          {
            HumidityPair tp = *reservedState;
            #ifdef ALERT_INCLUDE_COMMA_VALUES
            int8_t sign = tp.Current.Value < 0 ? -1 : 1;
            curHumidity = (abs(tp.Current.Value)*100 + tp.Current.Fract)*sign;
            #else 
            curHumidity = tp.Current.Value;
            #endif
          } // else
        }

       switch(Settings.Operand)
       {
          case roLessThan: return curHumidity < humidityAlert;
          case roLessOrEqual: return curHumidity <= humidityAlert;
          case roGreaterThan: return curHumidity > humidityAlert;
          case roGreaterOrEqual: return curHumidity >= humidityAlert;
          default: return false;
       } // switch
      
    }
    break;  

    case rtPH: // следим за pH
    {
     if(!linkedModule->State.HasState(StatePH))  // не поддерживаем pH
        return false;

       OneState* os = linkedModule->State.GetState(StatePH,Settings.SensorIndex);
       if(!os) // не срослось
        return false;

       HumidityPair hp = *os;
       #ifdef ALERT_INCLUDE_COMMA_VALUES
       int8_t sign = hp.Current.Value < 0 ? -1 : 1;
       int curHumidity = (abs(hp.Current.Value)*100 + hp.Current.Fract)*sign;
       int phAlert = Settings.DataAlert*100;
       #else
       int8_t curHumidity = hp.Current.Value;
       int8_t phAlert = Settings.DataAlert;
       #endif

       if(curHumidity == NO_TEMPERATURE_DATA) // нет датчика на линии
       {
         // пытаемся найти резервирование
          OneState* reservedState = MainController->GetReservedState(linkedModule,StatePH,Settings.SensorIndex);
          if(!reservedState)
            return (curHumidity == Settings.DataAlert); // на случай, если правило следит за отсутствием показаний с датчика
         else // есть зарезервированный датчик с показаниями
          {
            HumidityPair tp = *reservedState;
            #ifdef ALERT_INCLUDE_COMMA_VALUES
            int8_t sign = tp.Current.Value < 0 ? -1 : 1;
            curHumidity = (abs(tp.Current.Value)*100 + tp.Current.Fract)*sign;
            #else 
            curHumidity = tp.Current.Value;
            #endif
          } // else
            
       }

       switch(Settings.Operand)
       {
          case roLessThan: return curHumidity < phAlert;
          case roLessOrEqual: return curHumidity <= phAlert;
          case roGreaterThan: return curHumidity > phAlert;
          case roGreaterOrEqual: return curHumidity >= phAlert;
          default: return false;
       } // switch
      
    }
    break;      

    case rtPinState: // следим за статусом пина
    {
       #ifndef ALERT_DONT_READ_PIN_DIRECT
        WORK_STATUS.PinMode(Settings.SensorIndex,INPUT);
       #endif
       
       int pinState = WORK_STATUS.PinRead(Settings.SensorIndex); // читаем из пина его значение
       // dataAlertLong у нас может принимать одно значение: 1, поскольку мы сравниваем
       // это значение с HIGH и LOW на пине. Поэтому не имеют смысла операнды > и <=,
       // вместо них мы принудительно используем операнды >= и <.
     
       switch(Settings.Operand)
       {
          case roLessThan: return pinState < Settings.DataAlert; 
          case roLessOrEqual: return pinState < Settings.DataAlert;
          case roGreaterThan: return pinState >= Settings.DataAlert;
          case roGreaterOrEqual: return pinState >= Settings.DataAlert;
          default: return false;
         
       } // switch
    }
    break;

    case rtUnknown:
     // нет того, за чем следим, считаем, что мы сработали по времени
     return true;
  } // switch

  return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetAlertRule() // конструируем правило, когда запрашивают его просмотр
{
    SD_BUFFER[0] = 0;
    strcpy(SD_BUFFER,GetName());
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

    if(linkedModule)
      strcat(SD_BUFFER,linkedModule->GetID());
      
     strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

    switch(Settings.Target)
    {
      case rtTemp:
         strcat_P(SD_BUFFER,(const char*) PROP_TEMP);
      break;
      
      case rtLuminosity:
        strcat_P(SD_BUFFER,(const char*) PROP_LIGHT);
      break;

      case rtHumidity:
        strcat_P(SD_BUFFER,(const char*) PROP_HUMIDITY);
      break;

      case rtPinState:
        strcat_P(SD_BUFFER,(const char*) PROP_PIN);
      break;

      case rtSoilMoisture:
        strcat_P(SD_BUFFER,(const char*) PROP_SOIL);
      break;

      case rtPH:
        strcat_P(SD_BUFFER,(const char*) PROP_PH);
      break;

      case rtUnknown:
        strcat_P(SD_BUFFER,(const char*) PROP_NONE);
      break;
      
    }
    
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);
    String helper = String(Settings.SensorIndex);
    strcat(SD_BUFFER,helper.c_str());
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);
    
    switch(Settings.Operand)
    {
      case roLessThan:
        strcat_P(SD_BUFFER,(const char*) LESS_THAN);
      break;
      case roGreaterThan:
        strcat_P(SD_BUFFER,(const char*) GREATER_THAN);
      break;
      case roLessOrEqual:
        strcat_P(SD_BUFFER,(const char*) LESS_OR_EQUAL_THAN);
      break;
      case roGreaterOrEqual:
        strcat_P(SD_BUFFER,(const char*) GREATER_OR_EQUAL_THAN);
      break;
    }
    
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);


    //------- NEW CODE BEGIN ---------------//
     switch(Settings.DataSource)
    {
      case tsOpenTemperature:
      {
        strcat_P(SD_BUFFER,(const char*) T_OPEN_MACRO);

        // у нас может быть добавка вида %TO%-10
        if(Settings.DataAlert != 0)
        {
          helper = "";
          if(Settings.DataAlert > 0)
          {
            // добавляем +, знак минуса добавится автоматически чуть ниже
            helper += '+';
          }
          
          helper += Settings.DataAlert;
          strcat(SD_BUFFER,helper.c_str());
        }
      }
      break;
      
      case tsCloseTemperature:
      {
        strcat_P(SD_BUFFER,(const char*) T_CLOSE_MACRO);

        // у нас может быть добавка вида %TС%+12
        if(Settings.DataAlert != 0)
        {
          helper = "";
          if(Settings.DataAlert > 0)
          {
            // добавляем +, знак минуса добавится автоматически чуть ниже
            helper += '+';
          }
          
          helper += Settings.DataAlert;
          strcat(SD_BUFFER,helper.c_str());
        }
      }
      break;
      
      case tsPassed:
      {
        helper = Settings.DataAlert;
        strcat(SD_BUFFER,helper.c_str());
      }
      break;
    }
    //------- NEW CODE END ---------------//

    /*
    // OLD CODE
    switch(Settings.DataSource)
    {
      case tsOpenTemperature:
        strcat_P(SD_BUFFER,(const char*) T_OPEN_MACRO);
      break;
      case tsCloseTemperature:
        strcat_P(SD_BUFFER,(const char*) T_CLOSE_MACRO);
      break;
      case tsPassed:
        helper = Settings.DataAlert;
        strcat(SD_BUFFER,helper.c_str());
      break;
    }
    */
    
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

    helper = Settings.StartTime;
    strcat(SD_BUFFER,helper.c_str());
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

    helper = Settings.WorkTime;
    strcat(SD_BUFFER,helper.c_str());
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

    helper = Settings.DayMask;
    strcat(SD_BUFFER,helper.c_str());
    strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);

  size_t sz = linkedRulesIndices.size();
    
  if(!sz)
    strcat_P(SD_BUFFER,(const char*) F("_"));
  else
  {
    for(size_t i=0;i<sz;i++)
    {
      if(i > 0)
        strcat_P(SD_BUFFER,(const char*) F(","));
        
      strcat(SD_BUFFER,RulesDispatcher->GetParam(linkedRulesIndices[i]));
      
    } // for
  } // else

  // теперь копируем в строку правила признак - тревожное оно или нет
  strcat_P(SD_BUFFER,(const char*) PARAM_DELIMITER);
  strcat_P(SD_BUFFER,(const char*) (Settings.IsAlarm == 1 ? F("1") : F("0") ));
  
  return SD_BUFFER;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t AlertRule::Save(uint32_t writeAddr) // сохраняем себя в EEPROM, возвращаем кол-во записанных байт
{
  uint32_t curWriteAddr = writeAddr;

  // сохраняем настройки
  byte* bPtr = (byte*) &Settings;
  for(size_t i=0;i<sizeof(Settings);i++)
    MemWrite(curWriteAddr++,*bPtr++);
    
  //EEPROM.put(curWriteAddr,Settings);
  //curWriteAddr += sizeof(Settings);

  // затем пишем индексы связанных правил
  uint8_t cnt = linkedRulesIndices.size();
  MemWrite(curWriteAddr++,cnt);

  for(uint8_t i=0;i<cnt;i++)
  {
    MemWrite(curWriteAddr++,linkedRulesIndices[i]);
  } // for

  // затем смотрим: если у нас команда commandUnparsed и есть сама команда - то пишем её
  if(Settings.TargetCommandType == commandUnparsed)
  {
      if(rawCommand)
      {
        uint8_t len = strlen(rawCommand);
        MemWrite(curWriteAddr++,len);
        for(uint8_t i=0;i<len;i++)
        {
          MemWrite(curWriteAddr++,rawCommand[i]);
        }
      }
  } // if
  

  return (curWriteAddr - writeAddr) + 4;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t AlertRule::Load(uint32_t readAddr)
{
  // загружаем правило из EEPROM
  uint32_t curReadAddr = readAddr;
  linkedRulesIndices.clear();
  delete[] rawCommand; rawCommand = NULL;

  // сначала читаем настройки
  byte* bSettPtr = (byte*) &Settings;
  for(size_t i=0;i<sizeof(Settings);i++)
  {
    *bSettPtr = MemRead(curReadAddr++);
    bSettPtr++;
  }
    
  //EEPROM.get(curReadAddr,Settings);
  //curReadAddr += sizeof(Settings);

  // потом читаем индексы связанных правил
  uint8_t cnt = MemRead(curReadAddr++);
  for(uint8_t i=0;i<cnt;i++)
    linkedRulesIndices.push_back(MemRead(curReadAddr++));

  // затем смотрим: если у нас команда commandUnparsed и есть сама команда - то читаем её
  if(Settings.TargetCommandType == commandUnparsed)
  {
      uint8_t len = MemRead(curReadAddr++);
      rawCommand = new char[len+1];
      for(uint8_t i=0;i<len;i++)
        rawCommand[i] = MemRead(curReadAddr++);

      rawCommand[len] = 0;
  } // if

  // ищем связанный модуль
  linkedModule = MainController->GetModuleByID(GetLinkedModuleName());

  return (curReadAddr - readAddr) + 4;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const char* AlertRule::GetLinkedRuleName(uint8_t idx)
{

 if(idx < linkedRulesIndices.size())
  return RulesDispatcher->GetParam(linkedRulesIndices[idx]);

 return NULL;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
size_t AlertRule::GetLinkedRulesCount()
{
  return linkedRulesIndices.size();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertRule::Construct(AbstractModule* lm, const Command& command)
{
  // конструируем команду
  linkedModule = lm;
  Settings.LinkedModuleNameIndex = GetKnownModuleID(lm->GetID());

  // чистим имена связанных правил, об удалении памяти имён заботится родитель
  linkedRulesIndices.clear();

  uint8_t argsCnt = command.GetArgsCount();
  if(argsCnt < 12) // мало аргументов
    return false;

  delete[] rawCommand; rawCommand = NULL;

  uint8_t curArgIdx = 1;
  
  // ищем имя правила
  String curArg = command.GetArg(curArgIdx++);
  // сохраняем имя у родителя
  char* rName = new char[curArg.length()+1];
  strcpy(rName,curArg.c_str());
  bool added;
  Settings.RuleNameIndex = (uint8_t) RulesDispatcher->AddParam(rName,added);
  if(!added) // имя уже было у родителя
    delete[] rName;
 
  // записываем имя связанного модуля
  curArgIdx++; // пропускаем имя связанного модуля, нам его уже дали в параметрах функции
  
  curArg = command.GetArg(curArgIdx++);

  Settings.Target = rtUnknown; // да ни за чем не следим
  
  if(curArg == PROP_TEMP) // следим за температурой
    Settings.Target = rtTemp;
  else
  if(curArg == PROP_LIGHT) // следим за освещенностью
    Settings.Target = rtLuminosity;
  else
  if(curArg == PROP_HUMIDITY) // следим за влажностью
    Settings.Target = rtHumidity;
  else
  if(curArg == PROP_PIN)
    Settings.Target = rtPinState; // следим за состоянием пина
  else
  if(curArg == PROP_SOIL)
    Settings.Target = rtSoilMoisture; // следим за влажностью почвы
  else
  if(curArg == PROP_PH)
    Settings.Target = rtPH; // следим за pH

  Settings.SensorIndex = (uint8_t) atoi(command.GetArg(curArgIdx++));
  curArg = command.GetArg(curArgIdx++);

  
  if(curArg == GREATER_THAN)
    Settings.Operand = roGreaterThan;
  else if(curArg == LESS_THAN)
    Settings.Operand = roLessThan;
  else if(curArg == LESS_OR_EQUAL_THAN)
    Settings.Operand = roLessOrEqual;
  else if(curArg == GREATER_OR_EQUAL_THAN)
    Settings.Operand = roGreaterOrEqual;


  curArg = command.GetArg(curArgIdx++);

  // выясняем, за какой температурой следим

  //------- NEW CODE BEGIN ---------------//
  // у нас уставка температуры может быть вида %TO%+5, или %TC%-10 - учитываем это
  if(curArg.startsWith(T_OPEN_MACRO))
  {
    Settings.DataSource = tsOpenTemperature;
    // удаляем префикс %TO%, оставляя возможные дополнения
    curArg.remove(0,String(T_OPEN_MACRO).length());
  }
  else if(curArg.startsWith(T_CLOSE_MACRO))
  {
    Settings.DataSource = tsCloseTemperature;
    // удаляем префикс %TC%, оставляя возможные дополнения
    curArg.remove(0,String(T_CLOSE_MACRO).length());
  }
  else
  {
    // как передали, так и сохраняем
    Settings.DataSource = tsPassed;
  }
  //------- NEW CODE END ---------------//

  /*
  // OLD CODE
  if(curArg == T_OPEN_MACRO)
  {
    Settings.DataSource = tsOpenTemperature;
  }
  else if(curArg == T_CLOSE_MACRO)
  {
    Settings.DataSource = tsCloseTemperature;
  }
  else
  {
    Settings.DataSource = tsPassed;
  }
  */
    
  Settings.DataAlert = curArg.toInt();
  
  // следом идёт час начала работы
  Settings.StartTime = (uint16_t) atoi(command.GetArg(curArgIdx++));
 
  // дальше идёт продолжительность работы
  Settings.WorkTime = (unsigned long) atol(command.GetArg(curArgIdx++));

  // дальше идёт маска дней недели
  Settings.DayMask = (uint8_t) atoi(command.GetArg(curArgIdx++));
  
  // далее идут правила, при срабатывании которых данное правило работать не будет
  curArg = command.GetArg(curArgIdx++);

  // парсим имена связанных правил
  if(curArg != F("_")) // есть связанные правила
  {
        int curNameIdx = 0;
 
        while(curNameIdx != -1)
        {
          curNameIdx = curArg.indexOf(F(",")); // парсим по запятой
          if(curNameIdx == -1)
          {
           if(curArg.length() > 0)
           {
              char* lrnm = new char[curArg.length()+1];
              strcpy(lrnm,curArg.c_str());
              bool added;
              linkedRulesIndices.push_back(RulesDispatcher->AddParam(lrnm,added));
              if(!added) // имя правила уже существовало
                delete[] lrnm;
           }
              
            break;
          } // if
          String param = curArg.substring(0,curNameIdx);
          curArg = curArg.substring(curNameIdx+1,curArg.length());
          if(param.length() > 0)
          {
              char* lrnm = new char[param.length()+1];
              strcpy(lrnm,param.c_str());
              bool added;
              linkedRulesIndices.push_back(RulesDispatcher->AddParam(lrnm,added));
              if(!added) // имя правила уже существовало
                delete[] lrnm;
          }
          
        } // while

 
    
  } // if

  // получаем флаг - тревожное или нет
  curArg = command.GetArg(curArgIdx++);
  Settings.IsAlarm = 0;
  if(curArg.toInt() == 1)
    Settings.IsAlarm = 1;

  // сохраняем команду, которую надо передать какому-либо модулю
  Settings.TargetCommandType = commandUnparsed; // неразобранная команда

  // тут парсим команду
   SD_BUFFER[0] = 0;
   for(uint8_t i=curArgIdx;i<argsCnt;i++)
   {
      if(i > curArgIdx)
        strcat_P(SD_BUFFER,(const char*)PARAM_DELIMITER);
        
      strcat(SD_BUFFER,command.GetArg(i));
   } // for

   // собрали всю команду в кучу
   if(strlen(SD_BUFFER) > 0)
   {
    // есть команда на выполнение
    const char* tcBegin = SD_BUFFER;
    const char* tcModuleName = strchr(tcBegin,'=');
    tcModuleName++;
    const char* tcParams = strchr(tcBegin,'|');

    SD_BUFFER[tcParams - tcBegin] = 0;

    Settings.TargetModuleNameIndex = GetKnownModuleID(tcModuleName);

    
    tcParams++;
    
    // в tcParams лежат все параметры, в tcModuleName - имя модуля.
    // начинаем искать известную нам команду.
    
        if(!strcmp_P(tcModuleName,(const char*)F("LIGHT"))) // чего-то делаем с досветкой
        {
          if(strstr_P(tcParams,(const char*)STATE_ON)) // включить
            Settings.TargetCommandType = commandLightOn;
          else
            Settings.TargetCommandType = commandLightOff;
          
        } // LIGHT
        else
        if(!strcmp_P(tcModuleName,(const char*)F("STATE"))) // чего-то делаем с окнами
        {
            if(!strcmp_P(tcParams,(const char*)F("WINDOW|ALL|OPEN")))
              Settings.TargetCommandType = commandOpenAllWindows;
            else
            if(!strcmp_P(tcParams,(const char*)F("WINDOW|ALL|CLOSE")))
              Settings.TargetCommandType = commandCloseAllWindows;
        } // STATE
        else
        if(!strcmp_P(tcModuleName,(const char*)F("CC"))) // чего-то делаем с составными командами
        {
            Settings.TargetCommandType = commandExecCompositeCommand;
            Settings.TargetCommandParam = atoi(command.GetArg(argsCnt-1)); // последним параметром у команды идёт индекс списка
        } // CC
        else
        if(!strcmp_P(tcModuleName,(const char*)F("SCN"))) // чего-то делаем со сценариями
        {
            Settings.TargetCommandType = commandExecScene;
            
            if(!strcmp_P(command.GetArg(argsCnt-2),(const char*) F("STOP")))
              Settings.TargetCommandType = commandStopScene;
              
            Settings.TargetCommandParam = atoi(command.GetArg(argsCnt-1)); // последним параметром у команды идёт номер сценария
        } // SCN
        else
        if(!strcmp_P(tcModuleName,(const char*)F("PIN"))) // чего-то делаем с пинами
        {
            // получаем номера пинов
            const char* pins = command.GetArg(argsCnt-2);
            // проверяем, если там есть запятая - значит пинов несколько, не пойдёт.
            // иначе - это наш случай.
            
            if(!strchr(pins,',')) // номер одного пина, можем сохранить его в параметр
            {
                  Settings.TargetCommandParam = atoi(pins);
                  
                  if(!strcmp_P(command.GetArg(argsCnt-1),(const char*) STATE_ON))
                    Settings.TargetCommandType = commandSetOnePinHigh;
                  else
                    Settings.TargetCommandType = commandSetOnePinLow;
                     
            } // if
            
        } // PIN
        else
        if(!strcmp_P(tcModuleName,(const char*)F("MCP"))) // чего-то делаем с MCP
        {
            // получаем тип MCP, с которой работаем
             if(!strstr_P(tcParams,(const char*)F("SPI")))
             {
                // работаем с MCP23S17
                
                // у нас в command.GetArg(argsCnt-2), т.е. в предпоследнем параметре - номер канала MCP, который сохраняется в Reserved.
                // в command.GetArg(argsCnt-3) - номер микросхемы MCP

                Settings.Reserved = atoi(command.GetArg(argsCnt-2)); // MCP channel
                Settings.TargetCommandParam = atoi(command.GetArg(argsCnt-3)); // MCP number

                if(!strcmp_P(command.GetArg(argsCnt-1),(const char*) STATE_ON))
                    Settings.TargetCommandType = commandMCP23S17PinOn;
                  else
                    Settings.TargetCommandType = commandMCP23S17PinOff;
                
             } // работаем с MCP23S17
        }

       if(Settings.TargetCommandType == commandUnparsed)
       {
        // так и не смогли разобрать команду, просто копируем все её параметры в сырую команду
        uint8_t len = strlen(tcParams);
        delete[] rawCommand;
        rawCommand = new char[len+1];
        strcpy(rawCommand,tcParams);
        rawCommand[len] = 0;
        
       } // if
        
   } // if(strlen(SD_BUFFER) > 0)

  return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::LoadRules() // читаем настройки из EEPROM
{  
  for(uint8_t i=0;i<rulesCnt;i++)
  {
    AlertRule* r = alertRules[i];
    MainController->RemoveAlarm(r);
    delete r;
  }
  InitRules(); // инициализируем массив

 uint32_t readAddr = EEPROM_RULES_START_ADDR; // пишем с этого смещения

  // сначала читаем заголовок
  uint8_t h1, h2;
  h1 = MemRead(readAddr++);
  h2 = MemRead(readAddr++);

  if(!(h1 == RULE_SETT_HEADER1 && h2 == RULE_SETT_HEADER2)) // ничего не записано
    return;

  ClearParams(); // очищаем параметры
  // потом читаем кол-во сохранённых имён правил
  uint8_t namesCnt = MemRead(readAddr++);

  // потом читаем имена правил
  for(uint8_t i=0;i<namesCnt;i++)
  {
      uint8_t len = MemRead(readAddr++);
      char* param = new char[len+1];
      for(uint8_t j=0;j<len;j++)
        param[j] = MemRead(readAddr++);

       param[len] = 0;
       paramsArray.push_back(param);
  } // for
  
  // потом читаем количество правил
 rulesCnt = MemRead(readAddr++);

  if(rulesCnt > MAX_ALERT_RULES)
    rulesCnt = MAX_ALERT_RULES;
 
  // потом читаем правила
  for(uint8_t i=0;i<rulesCnt;i++)
  {
    AlertRule* r = new AlertRule();
    alertRules[i] = r;
    readAddr += r->Load(readAddr); // просим правило прочитать своё внутреннее состояние
  } // for
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::ClearParams()
{
    for(size_t i=0;i<paramsArray.size();i++)
    {
      char* param = paramsArray[i];
      delete[] param;
    }
    paramsArray.clear();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::SaveRules() // сохраняем настройки в EEPROM
{
  uint32_t writeAddr = EEPROM_RULES_START_ADDR; // пишем с этого смещения

  // сначала пишем заголовок
  MemWrite(writeAddr++,RULE_SETT_HEADER1);
  MemWrite(writeAddr++,RULE_SETT_HEADER2);

  // потом пишем кол-во сохранённых имён правил
  MemWrite(writeAddr++,(uint8_t)paramsArray.size());

  // потом пишем все сохранённые имена правил
  for(size_t i=0;i<paramsArray.size();i++)
  {
    char* param = paramsArray[i];
    uint8_t len = strlen(param);
    MemWrite(writeAddr++,len);
    while(*param)
    {
      MemWrite(writeAddr++,*param);
      param++;
    }
  }
  
  // потом пишем количество правил
  MemWrite(writeAddr++,rulesCnt);

  // потом пишем правила
  for(uint8_t i=0;i<rulesCnt;i++)
  {
    AlertRule* r = alertRules[i];
    if(r)
      writeAddr += r->Save(writeAddr); // просим правило записать своё внутреннее состояние
  } // for

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
char* AlertModule::GetParam(size_t idx)
{
  if(idx < paramsArray.size())
    return paramsArray[idx];

  return NULL;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertModule::AddRule(AbstractModule* m, const Command& c)
{

// сперва ищем правило с таким же именем, как и переданное
 String rName = c.GetArg(1);
 for(uint8_t i= 0;i<rulesCnt;i++)
 {
  AlertRule* r = alertRules[i];
  if(r && !strcmp(r->GetName(),rName.c_str()))
  {
     // нашли такое правило, просто модифицируем его
     return r->Construct(m,c);
  }
 } // for
  
  if(rulesCnt >=  MAX_ALERT_RULES)
    return false;

   AlertRule* ar = new AlertRule();
   if(!ar)
    return false;

   if(!ar->Construct(m,c))
   {
    delete ar;
    return false;
   }
   alertRules[rulesCnt] = ar;

    rulesCnt++;
    return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::Setup()
{
  // настройка модуля алертов тут  
  // загружаем правила
  LoadRules();

  lastUpdateCall = 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::InitRules()
{
  rulesCnt = 0; // кол-вo правил
  for(uint8_t i=0;i<MAX_ALERT_RULES;i++)
  {
    alertRules[i] = NULL;
  } // for
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::Update()
{ 
  // обновление модуля алертов тут
  static uint32_t timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - timer;
  timer = now;
  lastUpdateCall += dt;
    
  if(lastUpdateCall < ALERT_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
    return;
     
#ifdef USE_DS3231_REALTIME_CLOCK
  RealtimeClock rtc = MainController->GetClock();
  RTCTime tm = rtc.getTime();
#endif


  RulesVector raisedAlerts;
  
  for(uint8_t i=0;i<rulesCnt;i++)
  {
    AlertRule* r = alertRules[i];
    if(!r)
      break;

      // сначала обновляем состояние правила
      r->Update(lastUpdateCall
#ifdef USE_DS3231_REALTIME_CLOCK
,tm.hour, tm.minute, tm.dayOfWeek
#endif
        );

      
      if(r->HasAlert())
      {
        // помещаем это правило в список сработавших правил
          raisedAlerts.push_back(r);
          
      } // if(r->HasAlert())
  } // for

  // проверяем список сработавших правил, на предмет связи их с другими сработавшими правилами
  RulesVector workRules; // правила, с которыми будем работать после разрешения конфликтов

  // разрешаем конфликты. Для этого надо пройти по всем цепочкам правил и разрешить все зависимости,
  // например: у нас есть три сработавших правила: 1 - просто, второе - не выполнять, если сработало
  // правило 3, 3 - не выполнять, если сработало правило 1. Очевидно, что в конечном списке
  // должны остаться правила 1 и 2, а не только 1, как будет, если смотреть правила поочерёдно,
  // и отбрасывать без учёта цепочек зависимостей.

  SolveConflicts(raisedAlerts,workRules); // разрешаем все конфликты
  
  // тут можем работать со сработавшими правилами спокойно
  size_t sz = workRules.size();



  if(WORK_STATUS.IsModeChanged())
  {
    WORK_STATUS.SetModeUnchanged();
    lastIterationRaisedRules.clear();
  }
  
  for(size_t i=0;i<sz;i++)
  {
    // для каждого правила в списке вызываем связанную команду
    AlertRule* r = workRules[i];

    if(IsRuleRaisedOnLastIteration(r)) // если правило срабатывало на предыдущей итерации - не надо ещё раз посылать эту команду.
    {
      continue;
    }
      
    
    if(r->HasTargetCommand()) // надо отправлять команду
    {
      Command cmd;
      
         // копируем имя модуля в строку, потому что методы GetTargetCommandModuleName и GetTargetCommand пользуют общий буфер,
         // и перезатрут данные друг друга.
         String moduleId = r->GetTargetCommandModuleName();
         cmd.Construct(moduleId.c_str(),r->GetTargetCommand(),ctSET);
         cmd.SetInternal(true); // говорим, что команда - от одного модуля к другому

        // НЕ БУДЕМ НИКУДА ПЛЕВАТЬСЯ ОТВЕТОМ ОТ МОДУЛЯ
        //cmd.SetIncomingStream(&Serial);
        MainController->ProcessModuleCommand(cmd,NULL);

        // дёргаем функцию обновления других вещей - типа, кооперативная работа
        yield();

      
    } // if(r->HasTargetCommand())

    // тут вызываем тревогу
    if(r->IsAlarm())
      MainController->Alarm(r);
 
  } // for

  lastIterationRaisedRules = workRules; // сохраняем список сработавших на этой итерации правил

  lastUpdateCall = lastUpdateCall - ALERT_UPDATE_INTERVAL;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertModule::IsRuleRaisedOnLastIteration(AlertRule* rule)
{
    for(size_t i=0;i<lastIterationRaisedRules.size();i++)
    {
      AlertRule* r = lastIterationRaisedRules[i];
      if(r == rule)
      {
        return true;
      }
    }
    
    return false;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool AlertModule::CanWorkWithRule(RulesVector& checkedRules, AlertRule* rule, RulesVector& raisedAlerts)
{

  yield(); // дёргаем многозадачность за хвост
  
  size_t cnt = rule->GetLinkedRulesCount();
  if(!cnt)
    return true; // нет связанных правил, при срабатывании которых мы должны игнорировать текущее


  // проверяем, есть ли мы уже в списке проверенных правил, если есть - считаем, что с этим правилом работать нельзя,
  // и удаляем его из списка
  size_t sz = checkedRules.size();
  for(size_t i=0;i<sz;i++)
  {
      if(checkedRules[i] == rule) // нашли кольцевую рекурсию
      {
        return false;
      }
  } // for

  for(uint8_t i=0;i<cnt;i++)
  {
    // проходимся по всем именам связанных с нашим правил, и разрешаем конфликты по цепочке
    const char* linkedRuleName = rule->GetLinkedRuleName(i);
    AlertRule* linkedRule = GetLinkedRule(linkedRuleName,raisedAlerts);
        
      if(linkedRule) // нашли сработавшее правило, от которого мы зависим
      {
        if(!linkedRule->GetLinkedRulesCount()) // сразу нашли сработавшее связанное правило без зависимостей
          return false; // с текущим правилом работать нельзя!
          
          checkedRules.push_back(rule); // помещаем текущий проверяемый узел в стек, чтобы не было кольцевой рекурсии

          // К СОЖАЛЕНИЮ, РЕШАЕМ РЕКУРСИВНО :(
          if(CanWorkWithRule(checkedRules,linkedRule,raisedAlerts)) // с правилом, на которое мы завязаны, работать можно, поэтому игнорируем текущее
            return false;
            
          checkedRules.pop(); // извлекаем последний элемент
 
      } // if linkedRule
    
    
  } // for
return true;
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
size_t AlertModule::AddParam(char* nm, bool& added)
{
  added = false;
  size_t sz = paramsArray.size();
  for(size_t i=0;i<sz;i++)
  {
    if(!strcmp(nm,paramsArray[i]))
      return i;
  } // for

  added = true;
  paramsArray.push_back(nm);
  return (paramsArray.size()-1);
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AlertRule* AlertModule::GetLinkedRule(const char* linkedRuleName,RulesVector& raisedAlerts)
{
  size_t sz = raisedAlerts.size();
  for(size_t i=0;i<sz;i++)
  {
      AlertRule* r = raisedAlerts[i];
      if(!strcmp(r->GetName(),linkedRuleName))
        return r;
  }
  return NULL;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void AlertModule::SolveConflicts(RulesVector& raisedAlerts,RulesVector& workRules)
{
  // разрешаем конфликты
  RulesVector checkedRules; // список уже проверенных правил, для разрешения кольцевой рекурсии
  
  size_t sz = raisedAlerts.size();
    for(size_t i=0;i<sz;i++)
    {
        AlertRule* rule = raisedAlerts[i];
        if(CanWorkWithRule(checkedRules, rule, raisedAlerts)) // разрешили все зависимости
            workRules.push_back(rule);
          
    } // for
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool  AlertModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) 
    PublishSingleton = UNKNOWN_COMMAND;

  size_t argsCount = command.GetArgsCount();
    
  if(command.GetType() == ctSET) 
  {
    PublishSingleton = NOT_SUPPORTED;
   
    if(!argsCount) // нет аргументов
    {
      PublishSingleton = PARAMS_MISSED;
    }
    else
    {
      String t = command.GetArg(0);
 //     t.toUpperCase();
 
          if(t == ADD_RULE)
          {
            t =  command.GetArg(2); // имя модуля
            AbstractModule* m = MainController->GetModuleByID(t);
            if(m && m != this && AddRule(m,command))
            {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = REG_SUCC;
            }
          } // ADD_RULE
          else 
          if(t == SAVE_RULES) // запросили сохранение правил
          {
            SaveRules();
            PublishSingleton.Flags.Status = true;
            PublishSingleton = SAVE_RULES;
          }
          else 
          if(t == RULE_STATE) // установить состояние правила - включено или выключено
          {
            if(argsCount < 2)
            {
              PublishSingleton = PARAMS_MISSED;
            } // if
            else
            {
                 String sParam = command.GetArg(1);                 
                 String state = command.GetArg(2);

                 bool bEnabled = (state == STATE_ON) || (state == STATE_ON_ALT);
                        
                if(sParam == ALL)
                 {
                   // все правила
                   for(uint8_t i=0;i<rulesCnt;i++)
                   {
                      AlertRule* rule = alertRules[i];
                      if(rule)
                         rule->SetEnabled(bEnabled);
                   } // for

                   PublishSingleton.Flags.Status = true;
                   PublishSingleton = RULE_STATE; 
                   PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << state;
                 } // if all
                 else // одно правило
                 {
                      // ищем правило по имени
                      String rName = command.GetArg(1);
                      for(uint8_t i=0;i<rulesCnt;i++)
                      {
                         AlertRule* rule = alertRules[i];
                         if(rule && !strcmp(rule->GetName(),rName.c_str()))
                         {
                          rule->SetEnabled(bEnabled);
                          PublishSingleton.Flags.Status = true;
                          PublishSingleton = RULE_STATE; 
                          PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << state;
                          break;
                         }
                      } // for
                
                 } // else
            } // else
          } // else RULE_STATE
          else
         if(t == RULE_ALERT) // установить состояние тревожного правила - включено или выключено
          {
            if(argsCount < 2)
            {
              PublishSingleton = PARAMS_MISSED;
            } // if
            else
            {
                 String sParam = command.GetArg(1);                 
                 String state = command.GetArg(2);

                 bool bEnabled = (state == STATE_ON) || (state == STATE_ON_ALT);
                        
                if(sParam == ALL)
                 {
                   // все правила
                   for(uint8_t i=0;i<rulesCnt;i++)
                   {
                      AlertRule* rule = alertRules[i];
                      if(rule && rule->IsAlarm()) // это правило тревожное
                         rule->SetEnabled(bEnabled);
                   } // for

                   PublishSingleton.Flags.Status = true;
                   PublishSingleton = RULE_ALERT; 
                   PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << state;
                 } // if all
                 else // одно правило
                 {
                      // ищем правило по имени
                      String rName = command.GetArg(1);
                      for(uint8_t i=0;i<rulesCnt;i++)
                      {
                         AlertRule* rule = alertRules[i];
                         if(rule && rule->IsAlarm() && !strcmp(rule->GetName(),rName.c_str()))
                         {
                          rule->SetEnabled(bEnabled);
                          PublishSingleton.Flags.Status = true;
                          PublishSingleton = RULE_ALERT; 
                          PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << state;
                          break;
                         }
                      } // for
                
                 } // else
            } // else
          } // else RULE_ALERT          
          else
         if(t == RULE_DELETE) // удалить правило по индексу
          {
            if(argsCount < 2)
            {
             PublishSingleton = PARAMS_MISSED;
            } // if
            else
            {
                 String sParam = command.GetArg(1);
                 //sParam.toUpperCase();
 
                if(sParam == ALL) // удалить все правила
                {
                  for(uint8_t i=0;i<rulesCnt;i++)
                  {
                     AlertRule* rule = alertRules[i];
                     MainController->RemoveAlarm(rule);
                     delete rule;

                     alertRules[i] = NULL;
                  } // for

                  // чистим все параметры, поскольку у нас больше нет правил
                  ClearParams();

                  // чистим список правил, сработавших на последней итерации
                  lastIterationRaisedRules.clear();

                  rulesCnt = 0;
                  
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = t; 
                  PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << REG_DEL;

                }
                else // только одно правило, удаляем по имени правила
                {
                   uint8_t deletedIdx = 0;
                   bool bDeleted = false;
                   for(uint8_t i=0;i<rulesCnt;i++)
                   {
                      AlertRule* rule = alertRules[i];
                      if(rule && !strcmp(rule->GetName(),sParam.c_str())) // нашли правило
                      {
                        // очищаем из списка сработавших на последней итерации
                        RulesVector thisLastIterationRaisedRules;
                        for(size_t k=0;k<lastIterationRaisedRules.size();k++)
                        {
                          if(lastIterationRaisedRules[k] != rule)
                          {
                            thisLastIterationRaisedRules.push_back(lastIterationRaisedRules[k]);
                          }
                        } // for

                        lastIterationRaisedRules = thisLastIterationRaisedRules;

                        MainController->RemoveAlarm(rule);
                        delete rule;
                        bDeleted = true;
                        deletedIdx = i; 
                        break;
                      }
                   } // for
                   if(bDeleted)
                   {
                      for(uint8_t i=deletedIdx+1;i<rulesCnt;i++) // сдвигаем массив
                      {
                        alertRules[i-1] = alertRules[i];
                      } // for

                    rulesCnt--;

                    //TODO: Удалять из параметров имя правила и у всех связанных правил удалять индекс этого имени!!!
 
                    PublishSingleton.Flags.Status = true;
                    PublishSingleton = t; 
                    PublishSingleton << PARAM_DELIMITER <<  sParam << PARAM_DELIMITER << REG_DEL;
                   } // if(bDeleted)
                } // else not ALL
            } // else
          } // else RULE_DELETE
           

    } // else
  }
  else
  if(command.GetType() == ctGET) //получить алерты
  {
    if(!argsCount) // нет аргументов
    {
      PublishSingleton = PARAMS_MISSED;
    }
    else
    {
        String t = command.GetArg(0);
        
        if(t == RULE_CNT) // запросили данные о количестве правил
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = RULE_CNT; 
          PublishSingleton << PARAM_DELIMITER << rulesCnt;
        }
        else
        {
               
              if(t == RULE_VIEW) // просмотр правила
              {
                    if(argsCount < 2)
                    {
                        PublishSingleton = PARAMS_MISSED;
                    }
                    else
                    {
                        uint8_t idx = (uint8_t) atoi(command.GetArg(1));
                        if(idx < rulesCnt) // норм индекс
                        {
                          AlertRule* rule = alertRules[idx];
                          if(rule) // нашли правило
                          {
                            PublishSingleton.Flags.Status = true;
                            PublishSingleton = RULE_VIEW; 
                            PublishSingleton << PARAM_DELIMITER << (command.GetArg(1)) << PARAM_DELIMITER
                            << (rule->GetAlertRule());

                            if(rule->HasTargetCommand())
                            {
                              PublishSingleton << PARAM_DELIMITER << F("CTSET=") << (rule->GetTargetCommandModuleName())
                              << PARAM_DELIMITER;
                              PublishSingleton << (rule->GetTargetCommand());
                            }
                            
                          }
                        } // if
                    } // else
                
              }
              else if(t == RULE_STATE) // запросили состояние правила
              {
                    if(argsCount < 2)
                    {
                        PublishSingleton = PARAMS_MISSED;
                    }
                    else
                    {
                        uint8_t idx = (uint8_t) atoi(command.GetArg(1));
                        if(idx <= rulesCnt) // норм индекс
                        {
                          AlertRule* rule = alertRules[idx];
                          if(rule) // нашли правило
                          {
                            
                            PublishSingleton.Flags.Status = true;
                            PublishSingleton = RULE_STATE; 
                            PublishSingleton << PARAM_DELIMITER << (command.GetArg(1)) << PARAM_DELIMITER
                             << (rule->GetEnabled() ? STATE_ON : STATE_OFF);
                          }
                        } // if
                    } // else
              
              }
              else
              {
                // неизвестная команда
              } // else
  
        } 

    } // else have args
              
  } // if ctGET
 
 // отвечаем на команду
  MainController->Publish(this,command);
  return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------------------------

