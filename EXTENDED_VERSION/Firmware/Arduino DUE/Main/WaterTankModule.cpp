#include "WaterTankModule.h"
#include "ModuleController.h"
#include "ZeroStreamListener.h" // for loraGate
//--------------------------------------------------------------------------------------------------------------------------------------
WaterTankModule* WaterTank = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
String WaterTankModule::GetErrorText()
{
  
  if(!HasErrors())
  {
    return "";
  }

  switch(errorType)
  {
    case waterTankNoErrors: // нет ошибок
      return F("МОДУЛЬ ИСПРАВЕН");

    case waterTankNoData: // нет внешних данных в течение долгого времени
      return F("НЕТ МОДУЛЯ");

    case waterTankFullSensorError: // не сработал датчик верхнего уровня в процессе наполнения, по превышению максимального времени наполнения
      return F("СБОЙ НАПОЛНЕНИЯ");

    case waterTankBottomSensorFailure: // ошибка нижнего датчика критического уровня
      return F("СБОЙ ДАТЧИКА #1");

    default:
      return F("ОШИБКА");
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterTankModule::UpdateState(bool _valveOnFlag,uint8_t _fillStatus, bool _errorFlag,uint8_t _errorType)
{
  // нам пришёл пакет с внешнего мира, с состоянием по модулю управления баком с водой
  
  #if defined(WATER_TANK_MODULE_DEBUG)
    Serial.println(F("WaterTankModule - update state from external module!"));
  #endif  
  
  valveOnFlag = _valveOnFlag;
  errorFlag = _errorFlag;
  errorType = _errorType;
  fillStatus = _fillStatus; // статус наполнения (0-100%)

  if(!errorFlag)
  {
    errorType = waterTankNoErrors;
  }


    #ifdef USE_LORA_GATE // отправляем через LoRa
   loraGate.sendWaterTankSettingsPacket();   
   #endif // USE_LORA_GATE

   #ifdef USE_RS485_GATE // отправляем через RS-485
   RS485.sendWaterTankSettingsPacket();   
   #endif


  isOnline = true;
  lastDataPacketSeenAt = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterTankModule::Setup()
{
  WaterTank = this;
  
  // настройка модуля тут
  valveOnFlag = false;
  errorFlag = true;
  errorType = waterTankNoData;
  lastDataPacketSeenAt = 0;
  isOnline = false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WaterTankModule::IsModuleOnline()
{
  return isOnline;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterTankModule::Update()
{ 
  // обновление модуля тут
  if(millis() - lastDataPacketSeenAt >= 120000ul)
  {
    // в течение двух минут не было данных с модуля, сбрасываем флаг наличия данных
    isOnline = false;

    //TODO: ТУТ ОТСЫЛ SMS, ЕСЛИ НЕОБХОДИМО !!!

    errorFlag = true; // не было долго данных, выставляем ошибку
    errorType = waterTankNoData; // вида "нет данных"
    valveOnFlag = false; // говорим, что клапан выключен
    lastDataPacketSeenAt = millis();
  } // if

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  WaterTankModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = UNKNOWN_COMMAND;

  uint8_t argsCnt = command.GetArgsCount();

  if(command.GetType() == ctSET) // SET
  {
    if(argsCnt > 0)
    {
        String s = command.GetArg(0);
        if(s == F("FILL")) // CTSET=WTANK|FILL|flag, включаем или выключаем наполнение бочки
        {
            if(argsCnt > 1)
            {
              uint8_t val = atoi(command.GetArg(1));

              FillTank(val);

              PublishSingleton.Flags.Status = true;
              PublishSingleton = command.GetArg(0);
              PublishSingleton << PARAM_DELIMITER << (valveOnFlag ? 1 : 0);
            }
        } // FILL     
         
    } // if(argsCnt > 0)
  } // SET
  else // GET
  {
    if(argsCnt > 0)
    {
          String s = command.GetArg(0);
          if(s == F("FILL")) // CTGET=WTANK|FILL, статус наполнения бочки
          {
              PublishSingleton.Flags.Status = true;
              PublishSingleton = s;
              PublishSingleton << PARAM_DELIMITER << (valveOnFlag ? 1 : 0);
              
          } // FILL
    } // if(argsCnt > 0)
  } // GET


    // отвечаем на команду
    MainController->Publish(this,command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterTankModule::FillTank(bool on)
{
  valveOnFlag = on; // сохраняем статус клапана, пока не придёт команда в ответ.

  
  #ifdef USE_LORA_GATE // отправляем через LoRa
    
    if(loraGate.isLoraInited())
    {
      loraGate.sendFillTankCommand(on);
    }
    #if defined(WATER_TANK_MODULE_DEBUG)
    else
    {
      Serial.println(F("CAN'T FILL TANK, LORA NOT INITED !!!"));
    }
    #endif // WATER_TANK_MODULE_DEBUG
    
  #endif // USE_LORA_GATE

  #ifdef USE_RS485_GATE // отправляем по RS-485
      RS485.sendFillTankCommand(on);    
  #endif // USE_RS485_GATE
  
}
//--------------------------------------------------------------------------------------------------------------------------------------


