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
    case waterTankNoErrors:
      return F("штатная работа");

    case waterTankNoData:
      return F("нет данных");

    case waterTankFullSensorError:
      return F("ошибка датчиков");

    case waterTankNoFill:
      return F("нет наполнения");

    default:
      return F("ошибка");
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

    //TODO: ТУТ ПРОВЕРКА - ЕСЛИ В НАСТРОЙКАХ УКАЗАНА LORA, ТО ОТПРАВЛЯЕМ ЧЕРЕЗ НЕЁ!!!
     // напоследок - отправляем актуальные настройки модулю
   loraGate.sendWaterTankSettingsPacket();



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
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterTankModule::Update()
{ 
  // обновление модуля тут
  if(millis() - lastDataPacketSeenAt >= 120000ul)
  {
    // в течение двух минут не было данных с модуля, сбрасываем флаг наличия данных

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
  
  #ifdef USE_LORA_GATE
    
    //TODO: ТУТ ПРОВЕРКА НАСТРОЕК МОДУЛЯ - ЕСЛИ ПРИВЯЗАНЫ К LORA, ТО ОТСЫЛАТЬ !!!
    
    if(loraGate.isLoraInited())
    {
      valveOnFlag = on; // сохраняем статус клапана, пока не придёт команда в ответ.
      loraGate.sendFillTankCommand(on);
    }
    #if defined(WATER_TANK_MODULE_DEBUG)
    else
    {
      Serial.println(F("CAN'T FILL TANK, LORA NOT INITED !!!"));
    }
    #endif // WATER_TANK_MODULE_DEBUG
    
  #endif // USE_LORA_GATE
}
//--------------------------------------------------------------------------------------------------------------------------------------


