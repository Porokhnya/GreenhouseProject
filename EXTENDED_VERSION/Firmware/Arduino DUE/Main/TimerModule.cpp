#include "TimerModule.h"
#include "ModuleController.h"
#include "Memory.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------
// PeriodicTimer
//--------------------------------------------------------------------------------------------------------------------------------
PeriodicTimer::PeriodicTimer()
{
  flags.isHoldOnTimer = false;
  tTimer = 0;
  flags.lastPinState = 3;
  memset(&Settings,0,sizeof(Settings));
  _timer = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
void PeriodicTimer::On()
{

  if(flags.lastPinState != TIMER_ON)
  {
    flags.isHoldOnTimer = true;
    flags.lastPinState = TIMER_ON;

    if(Settings.Pin == UNBINDED_PIN) // не указан номер пина. Номер пина настраивается через конфигуратор, в окне "Периодические таймеры", поэтому там может быть и "0"!!!
    {
      return;
    }
        
    TimerBinding bnd = HardwareBinding->GetTimerBinding();
    
    if(bnd.LinkType == linkUnbinded) // не указан режим управления
    {
      return;
    }

    if(bnd.LinkType == linkDirect)
    {
        if(EEPROMSettingsModule::SafePin(Settings.Pin))
        {
          WORK_STATUS.PinWrite(Settings.Pin,bnd.Level);
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0

        WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,Settings.Pin,bnd.Level);
      
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0

        WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,Settings.Pin,bnd.Level);
        
      #endif
    }
    
  } // if(flags.lastPinState != TIMER_ON)
     
}
//--------------------------------------------------------------------------------------------------------------------------------
void PeriodicTimer::Off()
{
  if(flags.lastPinState != !TIMER_ON)
  {
    flags.lastPinState = !TIMER_ON;
    flags.isHoldOnTimer = false;

    if(Settings.Pin == UNBINDED_PIN) // не указан номер пина. Номер пина настраивается через конфигуратор, в окне "Периодические таймеры", поэтому там может быть и "0"!!!
    {
      return;
    }

    TimerBinding bnd = HardwareBinding->GetTimerBinding();
    
    if(bnd.LinkType == linkUnbinded) // не указан режим управления
    {
      return;
    }

    if(bnd.LinkType == linkDirect)
    {
        if(EEPROMSettingsModule::SafePin(Settings.Pin))
        {
          WORK_STATUS.PinWrite(Settings.Pin,!bnd.Level);
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0

        WORK_STATUS.MCP_SPI_PinWrite(bnd.MCPAddress,Settings.Pin,!bnd.Level);
      
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0

        WORK_STATUS.MCP_I2C_PinWrite(bnd.MCPAddress,Settings.Pin,!bnd.Level);
        
      #endif
    }
    
  } // if(flags.lastPinState != !TIMER_ON)
     
}
//--------------------------------------------------------------------------------------------------------------------------------
void PeriodicTimer::Init()
{

  // сбрасываем настройки в дефолт
  tTimer = 0;
  flags.lastPinState = 3;

  TimerBinding bnd = HardwareBinding->GetTimerBinding();
  
  if(bnd.LinkType != linkUnbinded) // если указан режим управления
  {
       if(Settings.Pin != UNBINDED_PIN) // если пин - не 0xFF
      {
            if(bnd.LinkType == linkDirect)
            {
                if(EEPROMSettingsModule::SafePin(Settings.Pin))
                {
                  WORK_STATUS.PinMode(Settings.Pin, OUTPUT);
                }
            }
            else
            if(bnd.LinkType == linkMCP23S17)
            {
              #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
    
                WORK_STATUS.MCP_SPI_PinMode(bnd.MCPAddress,Settings.Pin,OUTPUT);
              
              #endif
            }
            else
            if(bnd.LinkType == linkMCP23017)
            {
              #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
    
                WORK_STATUS.MCP_I2C_PinMode(bnd.MCPAddress,Settings.Pin,OUTPUT);
                
              #endif
            }
        
      } // if(Settings.Pin != UNBINDED_PIN
            

  } // if(bnd.LinkType != linkUnbinded)

  if(IsActive())
  {
    On();
  }
  else
  {
    Off();
  }

}
//--------------------------------------------------------------------------------------------------------------------------------
bool PeriodicTimer::IsActive()
{
  bool en = (Settings.DayMaskAndEnable & 128);

  #ifndef USE_DS3231_REALTIME_CLOCK
    // нет часов реального времени в прошивке, работаем просто по флагу "Таймер включён"
    return en;
  #else
    // модуль часов реального времени есть в прошивке, проверяем маску дней недели
    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();
    return en && (Settings.DayMaskAndEnable & (1 << (t.dayOfWeek-1)));
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void PeriodicTimer::Update()
{
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;  
  
  if(!IsActive()) // таймер неактивен, выключаем пин и выходим
  {
    Off();
    tTimer = 0;    
    return;
  }

  // прибавляем дельту
  tTimer += dt;
  unsigned long tCompare = 0;

  // теперь смотрим, какой интервал мы обрабатываем
  if(flags.isHoldOnTimer)
  {
    // ждём истекания интервала включения
    tCompare = Settings.HoldOnTime;
    tCompare *= 1000;
    
    if(tTimer >= tCompare)
    {
      Off();
      tTimer = 0;
    }
  }
  else
  {
    tCompare = Settings.HoldOffTime;
    tCompare *= 1000;
    
      if(tTimer >= tCompare)
      {
        On();
        tTimer = 0;
      }
  } // else
  
    
}
//--------------------------------------------------------------------------------------------------------------------------------
void TimerModule::LoadTimers()
{
  uint32_t addr = TIMERS_EEPROM_ADDR;
  if(MemRead(addr++) != SETT_HEADER1)
    return;

  if(MemRead(addr++) != SETT_HEADER2)
    return;

    // читаем настройки таймеров  
   for(byte i=0;i<NUM_TIMERS;i++)
   {
       uint8_t* pB = (uint8_t*) &(timers[i].Settings);
       for(size_t k=0;k<sizeof(PeriodicTimerSettings);k++)
       {
        *pB = MemRead(addr++);
        pB++;
       }
   } // for  
}
//--------------------------------------------------------------------------------------------------------------------------------
void TimerModule::SaveTimers()
{
  uint32_t addr = TIMERS_EEPROM_ADDR;
  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

   for(byte i=0;i<NUM_TIMERS;i++)
   {
       uint8_t* pB = (uint8_t*) &(timers[i].Settings);
       for(size_t k=0;k<sizeof(PeriodicTimerSettings);k++)
       {
        MemWrite(addr++,*pB++);
       }
   } // for
}
//--------------------------------------------------------------------------------------------------------------------------------
void TimerModule::Setup()
{
  // настройка модуля тут
  LoadTimers();
  
  for(uint8_t i=0;i<NUM_TIMERS;i++)
  {
    timers[i].Init();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
void TimerModule::Update()
{ 
  for(uint8_t i=0;i<NUM_TIMERS;i++)
  {
    timers[i].Update();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
bool  TimerModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  
  uint8_t argsCount = command.GetArgsCount();

  if(command.GetType() == ctGET)
  {
    PublishSingleton.Flags.Status = true;
    PublishSingleton = "";

    for(uint8_t i=0;i<NUM_TIMERS;i++)
    {
      PublishSingleton << timers[i].Settings.DayMaskAndEnable;
      PublishSingleton << PARAM_DELIMITER;

      PublishSingleton << timers[i].Settings.Pin;
      PublishSingleton << PARAM_DELIMITER;

      PublishSingleton << timers[i].Settings.HoldOnTime;
      PublishSingleton << PARAM_DELIMITER;

      PublishSingleton << timers[i].Settings.HoldOffTime;
      PublishSingleton << PARAM_DELIMITER;
      
    } // for
    
  }
  else // ctSET
  {
      if(argsCount < 16)
      {
        PublishSingleton = PARAMS_MISSED;
      }
      else
      {
        uint8_t tmrNum = 0;
        for(uint8_t i=0;i<16;i+=4)
        {
          timers[tmrNum].Settings.DayMaskAndEnable = (uint8_t) atoi(command.GetArg(i));
          timers[tmrNum].Settings.Pin = (uint8_t) atoi(command.GetArg(i+1));
          timers[tmrNum].Settings.HoldOnTime = (uint16_t) atoi(command.GetArg(i+2));
          timers[tmrNum].Settings.HoldOffTime = (uint16_t) atoi(command.GetArg(i+3));

          timers[tmrNum].Init();

          tmrNum++;
        }
        SaveTimers();
        PublishSingleton = REG_SUCC;
        PublishSingleton.Flags.Status = true;
      }
  } // else ctSET
  

  MainController->Publish(this,command);
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------

