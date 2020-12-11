#include "WateringModule.h"
#include "ModuleController.h"
#include "Memory.h"
#include "EEPROMSettingsModule.h"
#include "PowerManager.h"
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef WATER_DEBUG
  #define WTR_LOG(s) DEBUG_LOG((s))
#else
  #define WTR_LOG(s) (void) 0
#endif
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_GUARD
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static uint8_t WGUARD_PINS[] = { WATERING_GUARD_PINS };
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setupWGuard() // настраиваем защиту полива
{
  const size_t cnt = sizeof(WGUARD_PINS)/sizeof(WGUARD_PINS[0]);
  for(size_t i=0;i<cnt;i++)
  {
    WORK_STATUS.PinMode(WGUARD_PINS[i],INPUT);
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool hasWGuardAlert() // тестируем, сработала ли защита
{
  const size_t cnt = sizeof(WGUARD_PINS)/sizeof(WGUARD_PINS[0]);
  for(size_t i=0;i<cnt;i++)
  {
    if(WORK_STATUS.PinRead(WGUARD_PINS[i]) == WATERING_GUARD_ALERT)
      return true;
  }

  return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERING_GUARD
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::ChangeHardwareState()
{
    WateringBinding wbnd = HardwareBinding->GetWateringBinding();

    if(wbnd.ChannelsLinkType != linkUnbinded)
    {

      uint8_t relayPin = wbnd.RelaysPins[flags.index];

      if(relayPin != UNBINDED_PIN)
      {
      
        uint8_t state = flags.wantON ? wbnd.Level : !wbnd.Level;
    
        WTR_LOG(F("[WTR] - channel "));
        WTR_LOG(String(flags.index));
        WTR_LOG(F(" write to pin #"));
        WTR_LOG( String(relayPin) );
        WTR_LOG(F(", state = "));
        WTR_LOG(String(state));
        WTR_LOG(F("\r\n"));

       if(wbnd.ChannelsLinkType == linkDirect)
       {   
          if(EEPROMSettingsModule::SafePin(relayPin))
          {     
            WORK_STATUS.PinWrite(relayPin,state);
          }
       }        
       else
       if(wbnd.ChannelsLinkType == linkMCP23S17)
       {
            #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
              WORK_STATUS.MCP_SPI_PinWrite(wbnd.ChannelsMCPAddress,relayPin,state);
            #endif
       }
       else
       if(wbnd.ChannelsLinkType == linkMCP23017)
       {
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              WORK_STATUS.MCP_I2C_PinWrite(wbnd.ChannelsMCPAddress,relayPin,state);              
            #endif
       }
       
        
      } // if(relayPin != UNBINDED_PIN)

    } // if(wbnd.ChannelsLinkType != linkUnbinded)

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::SignalToHardware()
{
  flags.powerTimerDelay = 0;
  flags.wantON = flags.isON;

  // ЗДЕСЬ НАДО ОПЕРИРОВАТЬ НЕ STATE - а WATER_RELAY_ON, В ЗАВИСИМОСТИ ОТ СОСТОЯНИЯ flags.wantON
  WORK_STATUS.SaveWaterChannelState(flags.index,flags.wantON ? WATER_RELAY_ON : !WATER_RELAY_ON); // сохраняем статус канала полива  
  
  WPowerBinding bnd = HardwareBinding->GetWPowerBinding();
  
  if(bnd.LinkType == linkUnbinded) // нет управления питанием, сразу пишем в выход
  {
    ChangeHardwareState();
    return;
  }
  
  if(flags.wantON)
  {
    flags.powerTimerDelay = PowerManager.WateringOn(flags.index); // получаем задержку, которую нам необходимо выдержать до записи в выход состояния
    
    if(!flags.powerTimerDelay) // можно сразу включать канал
    {
       ChangeHardwareState(); 
    }
  }
  else
  {
    ChangeHardwareState();
    PowerManager.WateringOff(flags.index); // сразу выключаем, менеджер питания разберётся, что к чему
  }
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
WateringChannel::WateringChannel()
{
  flags.isON = flags.lastIsON = flags.wantON = false;
  flags.index = 0;
  flags.wateringTimer = 0;
  _timer = 0;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::Setup(byte index)
{
    flags.index = index;
    flags.isON = flags.lastIsON = false;
    flags.wateringTimer = 0;
    state = moistureWaitLowBorder; // ждём, пока влажность почвы упадёт ниже нижнего порога
  
    WTR_LOG(F("[WTR] - setup channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F("; OFF relay...\r\n"));

    WateringBinding wbnd = HardwareBinding->GetWateringBinding();

    if(wbnd.ChannelsLinkType != linkUnbinded)
    {
      uint8_t relayPin = wbnd.RelaysPins[index];
      if(relayPin != UNBINDED_PIN)
      {
      
        if(wbnd.ChannelsLinkType == linkDirect)
        {
            if(EEPROMSettingsModule::SafePin(relayPin))
            {
              WORK_STATUS.PinMode(relayPin,OUTPUT);
            }
        }
        else
        if(wbnd.ChannelsLinkType == linkMCP23S17)
        {
          #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
            WORK_STATUS.MCP_SPI_PinMode(wbnd.ChannelsMCPAddress,relayPin,OUTPUT);
          #endif
        }
        else
        if(wbnd.ChannelsLinkType == linkMCP23017)
        {
          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
            WORK_STATUS.MCP_I2C_PinMode(wbnd.ChannelsMCPAddress,relayPin,OUTPUT);
          #endif
        }

      } // if(relayPin != UNBINDED_PIN)
      
    } // if(wbnd.ChannelsLinkType != linkUnbinded)
    
    ChangeHardwareState(); // записываем в пины изначально выключенный уровень

    LoadState();
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::LoadState()
{

    WTR_LOG(F("Load state: channel - "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F("\r\n"));
  
    GlobalSettings* settings = MainController->GetSettings();
    uint8_t currentWateringOption = settings->GetWateringOption();

    byte offset = flags.index + 1; // единичку прибавляем потому, что у нас под нулевым индексом - настройки для всех каналов одновременно

    // смотрим - чего там в опции полива
    switch(currentWateringOption)
    {
      case wateringOFF: // автоматическое управление поливом выключено
      break;

      case wateringWeekDays: // управление поливом по дням недели, все каналы одновременно
        offset = 0; // читаем сохранённое время полива для всех каналов одновременно
      break;

      case wateringSeparateChannels: // раздельное управление каналами по дням недели
      break;
    }  

    DoLoadState(offset);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::SaveState(unsigned long wateringTimer)
{
    WTR_LOG(F("Save state: channel - "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F("\r\n"));
  
    GlobalSettings* settings = MainController->GetSettings();
    uint8_t currentWateringOption = settings->GetWateringOption();

    byte offset = flags.index + 1; // единичку прибавляем потому, что у нас под нулевым индексом - настройки для всех каналов одновременно

    // смотрим - чего там в опции полива
    switch(currentWateringOption)
    {
      case wateringOFF: // автоматическое управление поливом выключено
      break;

      case wateringWeekDays: // управление поливом по дням недели, все каналы одновременно
        offset = 0; // пишем сохранённое время полива для всех каналов одновременно
      break;

      case wateringSeparateChannels: // раздельное управление каналами по дням недели
      break;
    }  

    DoSaveState(offset, wateringTimer);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::On(WateringModule* m)
{
  flags.lastIsON = flags.isON;
  flags.isON = true;
  
  if(IsChanged()) // состояние изменилось
  {
    WTR_LOG(F("[WTR] - state for channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F(" changed, relay ON...\r\n"));

    // записываем в файл действий, что состояние канала полива изменилось
    #if defined(USE_LOG_MODULE) && defined(LOG_ACTIONS_ENABLED)
    
      String message;
      message = '#';
      message += flags.index;
      message += ' ';
      if(flags.isON)
      {
        message += STATE_ON;
      }
      else
      {
        message += STATE_OFF;
      }

      MainController->Log(m,message);
      
    #endif
        
    SignalToHardware(); // записываем новое состояние в пин
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::Off(WateringModule* m)
{
  flags.lastIsON = flags.isON;
  flags.isON = false;
  
  if(IsChanged()) // состояние изменилось
  {
    WTR_LOG(F("[WTR] - state for channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F(" changed, relay OFF...\r\n"));

    // записываем в файл действий, что состояние канала полива изменилось
    #if defined(USE_LOG_MODULE) && defined(LOG_ACTIONS_ENABLED)
    
      String message;
      message = '#';
      message += flags.index;
      message += ' ';
      if(flags.isON)
      {
        message += STATE_ON;
      }
      else
      {
        message += STATE_OFF;
      }

      MainController->Log(m,message);
      
    #endif
        
    SignalToHardware(); // записываем новое состояние в пин
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool WateringChannel::IsChanged()
{
  return (flags.lastIsON != flags.isON);
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool WateringChannel::IsActive()
{
  return flags.isON;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::Update(WateringModule* m,WateringWorkMode currentWorkMode, const RTCTime& currentTime, int8_t savedDayOfWeek)
{

  uint32_t now = millis();
  uint32_t _dt = now - _timer;
  _timer = now;

 // проверяем - есть ли задержка перед началом работы?
  if(flags.powerTimerDelay > 0)
  {
    if(_dt >= flags.powerTimerDelay)
    {
      flags.powerTimerDelay -= _dt;
    }
    else
    {
      flags.powerTimerDelay = 0;
    }
    
    if(!flags.powerTimerDelay)
    {
        // время ожидания записи в выход истекло, пишем туда  
        ChangeHardwareState();
    }
  } // if

  if(flags.powerTimerDelay > 0) // ещё есть задержка перед записью в выход, для устаканивания питания
  {
    return;
  }
  
  #ifdef USE_DS3231_REALTIME_CLOCK

    // только если модуль часов есть в прошивке - тогда обновляем состояние

    if(currentWorkMode == wwmManual) // в ручном режиме управления, ничего не делаем
    {
      return;
    }

    GlobalSettings* settings = MainController->GetSettings();
    uint8_t currentWateringOption = settings->GetWateringOption();

    if(currentWateringOption == wateringOFF) // автоматическое управление каналами выключено, не надо обновлять состояние канала
    {
      return;
    }

     uint32_t dt = _dt;
  
     // теперь получаем настойки полива на канале. Они зависят от currentWateringOption: если там wateringWeekDays - рулим всеми каналами одновременно,
     // если там wateringSeparateChannels - рулим каналами по отдельности, если там wateringOFF - мы не попадём в эту ветку кода.
     
     uint8_t weekDays = currentWateringOption == wateringWeekDays ? settings->GetWateringWeekDays() : settings->GetChannelWateringWeekDays(flags.index);
     // получаем время начала полива, в минутах от начала суток
     uint16_t startWateringTime = currentWateringOption == wateringWeekDays ? settings->GetStartWateringTime() : settings->GetChannelStartWateringTime(flags.index);
     uint32_t timeToWatering = currentWateringOption == wateringWeekDays ? settings->GetWateringTime() : settings->GetChannelWateringTime(flags.index); // время полива (в минутах!)
     int8_t sensorIndex = currentWateringOption == wateringWeekDays ? settings->GetWateringSensorIndex() : settings->GetChannelWateringSensorIndex(flags.index);
     int8_t stopBorder = currentWateringOption == wateringWeekDays ? settings->GetWateringStopBorder() : settings->GetChannelWateringStopBorder(flags.index);
     int8_t startBorder = currentWateringOption == wateringWeekDays ? settings->GetWateringStartBorder() : settings->GetChannelWateringStartBorder(flags.index);
     

      // переход через день недели мы фиксируем однократно, поэтому нам важно его не пропустить.
      // можем ли мы работать или нет - неважно, главное - правильно обработать переход через день недели.
      
      if(savedDayOfWeek != currentTime.dayOfWeek)  // сначала проверяем, не другой ли день недели уже?
      {
        flags.wateringTimer = 0; // сбрасываем таймер полива, т.к. начался новый день недели       
      } // if(savedDayOfWeek != currentTime.dayOfWeek)      


    // проверяем, установлен ли у нас день недели для полива, и настала ли минута, с которого можно поливать
    uint16_t currentTimeInMinutes = currentTime.hour*60 + currentTime.minute;
    bool canWork = bitRead(weekDays,currentTime.dayOfWeek-1) && (currentTimeInMinutes >= startWateringTime);

    #ifdef USE_SOIL_MOISTURE_MODULE // только если модуль влажности почвы есть в прошивке
    
    if(canWork)
    {
      // мы можем работать по времени, теперь проверяем, можем ли мы работать по условию с датчика влажности почвы
            
      if(sensorIndex > -1) // есть привязка к датчику влажности почвы
      {

        // по датчику почвы следует работать по следующему алгоритму:
        // 1. если влажность почвы ниже нижнего порога - то включаемся, и работаем, пока влажность почвы не достигнет верхнего порога.
        // 2. По превышению верхнего порога - однозначно выключаемся.
        
        AbstractModule* mod = MainController->GetModuleByID("SOIL");
        if(mod)
        {
           OneState* os = mod->State.GetState(StateSoilMoisture,sensorIndex);
           
           if(os && os->HasData())
           {
              HumidityPair hp = *os;

              // проверяем состояние работы с влажностью почвы
              switch(state)
              {
                case moistureWaitLowBorder: // ждём, пока влажность не опустится ниже нижнего порога
                {                  
                  if(hp.Current.Value <= startBorder)
                  {
                    // влажность упала ниже нижнего порога, можно включать.
                    // переменная canWork в этой ветке - true, значит, мы и так можем работать по времени.
                    // следовательно - достаточно переключиться на другую ветку конечного автомата.
                    state = moistureWaitHighBorder;
                  }
                  else
                  {
                    canWork = false; // влажность выше нижнего порога, не надо работать
                  }
                }
                break; // moistureWaitLowBorder

                case moistureWaitHighBorder: // ждём превышения верхнего порога влажности почвы
                {
                  if(hp.Current.Value >= stopBorder)
                  {
                    // верхний порог превышен, надо выключаться, и переключаться на ожидание падения влажности почвы на минимальный порог
                    canWork = false;
                    state = moistureWaitLowBorder;
                  }
                }
                break; // moistureWaitHighBorder
                
              } // switch

               
           } // if(os && os->HasData())
           else
           {
             canWork = false; // нет данных с датчика влажности почвы, а он прописан - поэтому немедленно останавливаемся !!!
           }
        }
      } // if(sensorIndex > -1)
      
      
    } // if(canWork)
    
   #endif // USE_SOIL_MOISTURE_MODULE    
  
    if(!canWork)
     { 
       Off(m); // выключаем реле
     }
     else
     {
      // можем работать, смотрим, не вышли ли мы за пределы установленного интервала

      flags.wateringTimer += dt; // прибавляем время работы

      WateringBinding wbnd = HardwareBinding->GetWateringBinding();

      // тут смотрим, не надо ли сохранить время полива, по шагам (например, каждую минуту)
      if(flags.lastSavedStateMinute != -1  && flags.lastSavedStateMinute != currentTime.minute && !(currentTime.minute % wbnd.SaveInterval))
      {       
        // надо сохранить время полива
        if(IsActive())
        {
          flags.lastSavedStateMinute = currentTime.minute;
          SaveState(flags.wateringTimer);
        }
      }
  
      // проверяем, можем ли мы ещё работать
      // если полив уже отработал, и юзер прибавит минуту - мы должны поливать ещё минуту,
      // вне зависимости от показания таймера. Поэтому мы при срабатывании условия окончания полива
      // просто отнимаем дельту времени из таймера, таким образом оставляя его застывшим по времени
      // окончания полива

      uint32_t maxTimeToWatering = ((timeToWatering*60000ul)  + dt);
  
      if(flags.wateringTimer > maxTimeToWatering) // приплыли, надо выключать полив
      {
        flags.wateringTimer -=  dt;// оставляем таймер застывшим на окончании полива, плюс маленькая дельта

        if(IsActive()) // если канал был включён, значит, он будет выключен, и мы однократно запишем в EEPROM нужное значение
        {
         SaveState(flags.wateringTimer);
        } // if(IsActive())

        Off(m); // выключаем реле
      }
      else
      {
        On(m); // ещё можем работать, продолжаем поливать
      }
        
     } // else can work
     

  #endif // USE_DS3231_REALTIME_CLOCK
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::DoLoadState(byte addressOffset)
{
   // сперва сбрасываем настройки времени полива и дополива
   flags.wateringTimer = 0;
   flags.lastSavedStateMinute = -1;
  
#ifdef USE_DS3231_REALTIME_CLOCK

  // читать время полива на канале имеет смысл только, когда модуль часов реального времени есть в прошивке
    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();
    uint8_t today = t.dayOfWeek; // текущий день недели

    flags.lastSavedStateMinute = t.minute;
  
    WTR_LOG(F("[WTR] - load state for channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F(" from EEPROM...\r\n"));

    uint32_t savedWorkTime = 0xFFFFFFFF;
    volatile uint8_t* writeAddr = (uint8_t*) &savedWorkTime;
    uint8_t savedDOW = 0xFF;

    // мы читаем 5 байт на канал, поэтому вычисляем адрес очень просто - по смещению addressOffset, в котором находится индекс канала
    volatile uint32_t curReadAddr = WATERING_STATUS_EEPROM_ADDR + addressOffset*5;

    savedDOW = MemRead(curReadAddr++);

    *writeAddr++ = MemRead(curReadAddr++);
    *writeAddr++ = MemRead(curReadAddr++);
    *writeAddr++ = MemRead(curReadAddr++);
    *writeAddr = MemRead(curReadAddr++);

    if(savedDOW != 0xFF && savedWorkTime != 0xFFFFFFFF) // есть сохранённое время работы канала на сегодня
    {
      WTR_LOG(F("[WTR] - data is OK...\r\n"));
      
      if(savedDOW == today) // поливали на этом канале сегодня, выставляем таймер канала так, как будто он уже поливался сколько-то времени
      {
        WTR_LOG(F("[WTR] - Apply watering timer for today: "));
        WTR_LOG(String(savedWorkTime));
        WTR_LOG(F("\r\n"));
        
        flags.wateringTimer = savedWorkTime + 1;
      }
      else
      {
        flags.wateringTimer = 0; // сбрасываем таймер
      }
      
    } // if
    else
    {
      // нет сохранённых настроек, сбрасываем по умолчанию
      flags.wateringTimer = 0;    
    }
 #else
    WTR_LOG(F("[WTR] - NO state for channel - no realtime clock!\r\n"));
 #endif // USE_DS3231_REALTIME_CLOCK

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::DoSaveState(byte addressOffset,uint32_t wateringTimer)
{  
#ifdef USE_DS3231_REALTIME_CLOCK

  // писать время полива на канале имеет смысл только, когда модуль часов реального времени есть в прошивке
    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();
    uint8_t today = t.dayOfWeek; // текущий день недели 
    
    WTR_LOG(F("[WTR] - save state for channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F(" to EEPROM...\r\n"));

     GlobalSettings* settings = MainController->GetSettings();

    // получаем время полива на канале. Логика простая: если addressOffset == 0 - у нас опция полива по дням недели, все каналы одновременно,
    // иначе - раздельное управление каналами по дням недели. Соответственно, мы либо получаем время полива для всех каналов, либо - для нужного.
    uint32_t timeToWatering = 0;
    
    if(wateringTimer > 0) // если передали время полива - сохраняем
    {
      timeToWatering = wateringTimer;
    }
    else // иначе - пишем полное время полива
    {
      timeToWatering = addressOffset == 0 ? settings->GetWateringTime() : settings->GetChannelWateringTime(flags.index); // время полива (в минутах!)
      timeToWatering *= 60000ul;
    }

     //Тут сохранение в EEPROM статуса, что мы на сегодня уже полили сколько-то времени на канале
    uint32_t wrAddr = WATERING_STATUS_EEPROM_ADDR + addressOffset*5; // адрес записи
    
    // сохраняем в EEPROM день недели, для которого запомнили значение таймера
    MemWrite(wrAddr++,today);
    
    // сохраняем в EEPROM значение таймера канала
    byte* readAddr = (byte*) &timeToWatering;
    for(int i=0;i<4;i++)
    {
      MemWrite(wrAddr++,*readAddr++);
    }
    
 #else
    WTR_LOG(F("[WTR] - NO state for channel - no realtime clock!\r\n"));
 #endif // USE_DS3231_REALTIME_CLOCK

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::GetPumpsState(bool& pump1State, bool& pump2State)
{
  pump1State = false;
  pump2State = false;

  GlobalSettings* settings = MainController->GetSettings();
  if(settings->GetTurnOnPump() != 1)
  {
    // не надо включать насосы при поливе на любом из каналов
    return;
  }
  
  #if WATER_RELAYS_COUNT > 0

    WateringBinding wbnd = HardwareBinding->GetWateringBinding();
  
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
        if(wateringChannels[i].IsActive())
        {
          // канал активен, смотрим, к какому насосу он относится
          if(i < wbnd.Pump2StartChannel)
           {
              // канал относится к первому насосу
              pump1State = true;
           }
           
           if(i >= wbnd.Pump2StartChannel)
           {
            // канал относится ко второму насосу
            pump2State = true;
           }
          
        }
    } // for
  #endif // WATER_RELAYS_COUNT > 0
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------  
void WateringModule::UpdatePumps()
{

  bool anyChannelActive = IsAnyChannelActive();
  
  if(!anyChannelActive)
  {
    // нет полива ни на одном из каналов, выключаем насосы
    pump1OnTimerActive = false;
    pump2OnTimerActive = false;
    
    TurnPump1(false);
    TurnPump2(false);

    PowerManager.PumpOff(0);
    PowerManager.PumpOff(1);

    return;
  }

  // здесь проверяем, какой насос включить. Для этого узнаём, есть ли активные каналы для первого и второго насоса
  bool shouldTurnPump1, shouldTurnPump2;

  // проверяем статус, который надо выставить для насосов
  GetPumpsState(shouldTurnPump1, shouldTurnPump2);

  if(shouldTurnPump1)
  {
     // надо включить первый насос
     if(!flags.isPump1On && !pump1OnTimerActive) // был не включен
     {
       pump1OnTimerActive = true;
       pump1Timer = millis();
       pump1PowerDelay = PowerManager.PumpOn(0);
     }
  }
  else
  {
    // должны выключить первый насос
    TurnPump1(false); 
    PowerManager.PumpOff(0);
  }

  if(shouldTurnPump2)
  {
     // надо включить второй насос
     if(!flags.isPump2On && !pump2OnTimerActive) // был не включен
     {
       pump2OnTimerActive = true;
       pump2Timer = millis();
       pump2PowerDelay = PowerManager.PumpOn(1);
     }
  }
  else
  {
    // должны выключить второй насос
    TurnPump2(false); 
    PowerManager.PumpOff(1);
  }  
  
    if(pump1OnTimerActive)
    {
      if(millis() - pump1Timer >= pump1PowerDelay)
      {
          pump1OnTimerActive = false;
          TurnPump1(true);
      }
    }

    if(pump2OnTimerActive)
    {
      if(millis() - pump2Timer >= pump2PowerDelay)
      {
          pump2OnTimerActive = false;
          TurnPump2(true);
      }
    }
      
  /*
  bool anyChannelActive = IsAnyChannelActive();
  if(!anyChannelActive)
  {
    // нет полива ни на одном из каналов, выключаем насосы
    TurnPump1(false);
    TurnPump2(false);
    
    return;
  }

  // здесь проверяем, какой насос включить. Для этого узнаём, есть ли активные каналы для первого и второго насоса
  bool shouldTurnPump1,shouldTurnPump2;

  // проверяем статус, который надо выставить для насосов
  GetPumpsState(shouldTurnPump1,shouldTurnPump2);

  TurnPump1(shouldTurnPump1);  
  TurnPump2(shouldTurnPump2);
  */
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnPump1(bool isOn)
{
  if(flags.isPump1On == isOn) // состояние не изменилось
  {
    return;
  }

  // сохраняем состояние
  flags.isPump1On = isOn;

  WTR_LOG(F("Turn pump #1 to state: "));
  WTR_LOG(isOn ? F("ON\r\n") : F("OFF\r\n"));

  WateringBinding wbnd = HardwareBinding->GetWateringBinding();

  if(wbnd.PumpLinkType != linkUnbinded && wbnd.PumpPin != UNBINDED_PIN)
  {
  
      uint8_t state = isOn ? wbnd.PumpLevel : !wbnd.PumpLevel;
      
      if(wbnd.PumpLinkType == linkDirect)
      {
        if(EEPROMSettingsModule::SafePin(wbnd.PumpPin))
        {
          WORK_STATUS.PinWrite(wbnd.PumpPin,state);
        }
      }
      else
      if(wbnd.PumpLinkType ==  linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinWrite(wbnd.PumpMCPAddress,wbnd.PumpPin,state);
        #endif
      }
      else
      if(wbnd.PumpLinkType ==  linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinWrite(wbnd.PumpMCPAddress,wbnd.PumpPin,state);
        #endif
      }
  } // if(wbnd.PumpLinkType != linkUnbinded)
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnPump2(bool isOn)
{
  if(flags.isPump2On == isOn) // состояние не изменилось
  {
    return;
  }

  // сохраняем состояние
  flags.isPump2On = isOn;

  WTR_LOG(F("Turn pump #2 to state: "));
  WTR_LOG(isOn ? F("ON\r\n") : F("OFF\r\n"));

  WateringBinding wbnd = HardwareBinding->GetWateringBinding();

  if(wbnd.Pump2LinkType != linkUnbinded && wbnd.Pump2Pin != UNBINDED_PIN)
  {  
      uint8_t state = isOn ? wbnd.PumpLevel : !wbnd.PumpLevel;
    
      if(wbnd.Pump2LinkType == linkDirect)
      {
          if(EEPROMSettingsModule::SafePin(wbnd.Pump2Pin))
          {
            WORK_STATUS.PinWrite(wbnd.Pump2Pin,state);
          }
      }
      else
      if(wbnd.Pump2LinkType == linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinWrite(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,state);
        #endif
      }
      else
      if(wbnd.Pump2LinkType == linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinWrite(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,state);
        #endif
      }
    
  } // if(wbnd.Pump2LinkType != linkUnbinded)
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::SetupPumps()
{
    WTR_LOG(F("[WTR] = setup pumps\r\n"));
    WTR_LOG(F("[WTR] - Turn OFF pump 1\r\n"));


    WateringBinding wbnd = HardwareBinding->GetWateringBinding();    

    if(wbnd.PumpLinkType != linkUnbinded && wbnd.PumpPin != UNBINDED_PIN)
    {
  
      if(wbnd.PumpLinkType == linkDirect)
      {
        if(EEPROMSettingsModule::SafePin(wbnd.PumpPin))
        {
          WORK_STATUS.PinMode(wbnd.PumpPin,OUTPUT);
          WORK_STATUS.PinWrite(wbnd.PumpPin,!wbnd.PumpLevel);
        }
      }
      else
      if(wbnd.PumpLinkType ==  linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(wbnd.PumpMCPAddress,wbnd.PumpPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(wbnd.PumpMCPAddress,wbnd.PumpPin,!wbnd.PumpLevel);
        #endif
      }
      else
      if(wbnd.PumpLinkType ==  linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(wbnd.PumpMCPAddress,wbnd.PumpPin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(wbnd.PumpMCPAddress,wbnd.PumpPin,!wbnd.PumpLevel);
        #endif
      }
    } // if(wbnd.PumpLinkType != linkUnbinded)


    WTR_LOG(F("[WTR] - Turn OFF pump 2\r\n"));

    if(wbnd.Pump2LinkType != linkUnbinded && wbnd.Pump2Pin != UNBINDED_PIN)
    {
  
      if(wbnd.Pump2LinkType == linkDirect)
      {
        if(EEPROMSettingsModule::SafePin(wbnd.Pump2Pin))
        {
          WORK_STATUS.PinMode(wbnd.Pump2Pin,OUTPUT);
          WORK_STATUS.PinWrite(wbnd.Pump2Pin,!wbnd.PumpLevel);
        }
      }
      else
      if(wbnd.Pump2LinkType ==  linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,!wbnd.PumpLevel);
        #endif
      }
      else
      if(wbnd.PumpLinkType ==  linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(wbnd.Pump2MCPAddress,wbnd.Pump2Pin,!wbnd.PumpLevel);
        #endif
      }
    } // if(wbnd.Pump2LinkType != linkUnbinded)
    
    
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Setup()
{
  // настройка модуля тут
  WTR_LOG(F("[WTR] - setup...\r\n"));

  #ifdef USE_WATERING_GUARD
    setupWGuard(); // настраиваем защиту полива
  #endif // USE_WATERING_GUARD

  GlobalSettings* settings = MainController->GetSettings();
  switchToAutoAfterMidnight = settings->GetTurnWateringToAutoAfterMidnight();

  flags.workMode = wwmAutomatic; // автоматический режим работы
  flags.isPump1On = false;
  flags.isPump2On = false;

  // настраиваем насосы
  SetupPumps();
  
  #if WATER_RELAYS_COUNT > 0

    // настраиваем каналы
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
        // просим канал настроиться, она загрузит свои настройки и выключит реле
        wateringChannels[i].Setup(i);
    } // for
  
  #endif // WATER_RELAYS_COUNT > 0

  // если указано - использовать диод индикации ручного режима работы - настраиваем его
  canUseBlinker = false;
  DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
  if(bnd.LinkType != linkUnbinded)
  {
    if(bnd.LinkType == linkDirect)
    {
        if(bnd.WateringManualModeDiodePin != UNBINDED_PIN && bnd.WateringManualModeDiodePin > 1
        #ifndef DISABLE_DIODES_CONFIGURE
        && EEPROMSettingsModule::SafePin(bnd.WateringManualModeDiodePin)
        #endif
        )
        {
          canUseBlinker = true;
        }
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
         if(bnd.WateringManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
         if(bnd.WateringManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }    
    
  } // if(bnd.LinkType != linkUnbinded)
  
  if(canUseBlinker)
  {
      blinker.begin(bnd.WateringManualModeDiodePin);  // настраиваем блинкер на нужный пин
      blinker.blink(); // и гасим его по умолчанию
  }


  #ifdef USE_DS3231_REALTIME_CLOCK

    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();

    lastDOW = t.dayOfWeek; // запоминаем прошлый день недели
    currentDOW = t.dayOfWeek; // запоминаем текущий день недели
  
  #else
  
    lastDOW = 0; // день недели с момента предыдущего опроса
    currentDOW = 0; // текущий день недели
    
  #endif // USE_DS3231_REALTIME_CLOCK

  // тут всё настроили, перешли в автоматический режим работы, выключили реле на всех каналах, запомнили текущий час и день недели.
  // можно начинать работать

 
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::SwitchToAutomaticMode()
{
  if(flags.workMode == wwmAutomatic) // уже в автоматическом режиме
  {
    return;
  }

  WTR_LOG(F("[WTR] - switch to automatic mode\r\n"));

    flags.workMode = wwmAutomatic;

  // гасим блинкер, если он используется в прошивке
  if(canUseBlinker)
  {
    blinker.blink();
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::SwitchToManualMode()
{
  if(flags.workMode == wwmManual) // уже в ручном режиме
  {
    return;
  }

  WTR_LOG(F("[WTR] - switch to manual mode\r\n"));

    flags.workMode = wwmManual;

  // зажигаем блинкер, если он используется в прошивке
  if(canUseBlinker)
  {
    blinker.blink(WORK_MODE_BLINK_INTERVAL);
  }

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Skip(bool skipOrReset) // пропускаем полив за сегодня
{
  WTR_LOG(F("[WTR] - skip watering for today\r\n"));

  uint8_t today = 0xFF;
  uint32_t max_work_time =  0xFFFFFFFF;

 #ifdef USE_DS3231_REALTIME_CLOCK

    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();
    today = t.dayOfWeek; // запоминаем текущий день недели
    max_work_time =  skipOrReset ? 0xFFFFF0F0 : 0;
     
  #endif
      
  // выставляем в EEPROM максимальное значение полива по каналам за сегодня
  uint32_t wrAddr = WATERING_STATUS_EEPROM_ADDR;
  uint8_t recordsCount = WATER_RELAYS_COUNT + 1;
  
  for(uint8_t i=0;i<recordsCount;i++)
  {
    MemWrite(wrAddr++,today);
    uint8_t* readAddr = (uint8_t*) &max_work_time;
    
      for(int k=0;k<4;k++)
      {
        MemWrite(wrAddr++,*readAddr++);    
      }
  }
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::ResetChannelsState()
{
  WTR_LOG(F("[WTR] - reset channels state\r\n"));
  //Тут затирание в EEPROM предыдущего сохранённого значения о статусе полива на всех каналах
  uint32_t wrAddr = WATERING_STATUS_EEPROM_ADDR;
  uint8_t bytes_to_write = 5 + WATER_RELAYS_COUNT*5;
  
  for(uint8_t i=0;i<bytes_to_write;i++)
  {
    MemWrite(wrAddr++,0); // для каждого канала по отдельности  
  }
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelsOff() // выключает все каналы
{
  WTR_LOG(F("[WTR] - turn all channels OFF\r\n"));
  
  #if WATER_RELAYS_COUNT > 0
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
      wateringChannels[i].Off(this);
    }
  #endif // WATER_RELAYS_COUNT > 0
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelsOn() // включает все каналы
{
  WTR_LOG(F("[WTR] - turn all channels ON\r\n"));
  
  #if WATER_RELAYS_COUNT > 0
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
      wateringChannels[i].On(this);
    }
  #endif // WATER_RELAYS_COUNT > 0  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelOff(uint8_t channelIndex) // выключает канал
{
  WTR_LOG(F("[WTR] - turn channel "));
  WTR_LOG(String(channelIndex));
  WTR_LOG(F(" OFF\r\n"));
  
  #if WATER_RELAYS_COUNT > 0
    if(channelIndex < WATER_RELAYS_COUNT)
    {
      wateringChannels[channelIndex].Off(this);
    }
  #endif // WATER_RELAYS_COUNT > 0  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelOn(uint8_t channelIndex) // включает канал
{
  WTR_LOG(F("[WTR] - turn channel "));
  WTR_LOG(String(channelIndex));
  WTR_LOG(F(" ON\r\n"));
    
  #if WATER_RELAYS_COUNT > 0
    if(channelIndex < WATER_RELAYS_COUNT)
    {
      wateringChannels[channelIndex].On(this);
    }
  #endif // WATER_RELAYS_COUNT > 0
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool WateringModule::IsAnyChannelActive() // проверяет, активен ли хоть один канал полива
{
  #if WATER_RELAYS_COUNT > 0
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
      if(wateringChannels[i].IsActive())
      {
        return true;
      }
    }
  #endif // WATER_RELAYS_COUNT > 0    

  return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Update()
{ 

  
  // обновляем блинкер, если он используется в прошивке
  if(canUseBlinker)
  {
     blinker.update();
  }

  #if WATER_RELAYS_COUNT > 0

    bool anyChannelActive = IsAnyChannelActive(); // проверяем, включен ли хотя бы один канал
     // теперь сохраняем статус полива
    SAVE_STATUS(WATER_STATUS_BIT, anyChannelActive ? 1 : 0); // сохраняем состояние полива
    SAVE_STATUS(WATER_MODE_BIT,flags.workMode == wwmAutomatic ? 1 : 0); // сохраняем режим работы полива

    RTCTime t;
    
    #ifdef USE_DS3231_REALTIME_CLOCK

      // обновлять каналы имеет смысл только при наличии часов реального времени
      RealtimeClock watch =  MainController->GetClock();
      t =  watch.getTime(); // получаем текущее время

    if(currentDOW != t.dayOfWeek)
    {      
      if(switchToAutoAfterMidnight)
      {
        // начался новый день недели, принудительно переходим в автоматический режим работы
        // даже если до этого был включен полив командой от пользователя
        SwitchToAutomaticMode();
  
        // здесь надо принудительно гасить полив на всех каналах, поскольку у нас может быть выключено автоуправление каналами.
        // в этом случае, если полив был включен пользователем и настали новые сутки - полив не выключится сам,
        // т.к. канал не обновляет своё состояние при выключенном автоуправлении каналами.
        TurnChannelsOff();

      } // if(sett->GetTurnWateringToAutoAfterMidnight())
      
      //Тут затирание в EEPROM предыдущего сохранённого значения о статусе полива на всех каналах
      ResetChannelsState();
      
    }

    currentDOW = t.dayOfWeek; // сохраняем текущий день недели

    #endif // USE_DS3231_REALTIME_CLOCK

    // в этой точке мы должны выяснять - надо ли гасить каналы, если мы используем защиту полива
    bool canUpdateChannels = true;

    #ifdef USE_WATERING_GUARD
      if(hasWGuardAlert()) // сработала защита полива
      {
        canUpdateChannels = false;
        // выключаем все каналы
        TurnChannelsOff();
        
      }
    #endif // USE_WATERING_GUARD

    if(canUpdateChannels)
    {
      // теперь обновляем все каналы
      for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
      {          
          wateringChannels[i].Update(this,(WateringWorkMode) flags.workMode,t,lastDOW);
      } // for
    }

    UpdatePumps();

   // обновили все каналы, теперь можно сбросить флаг перехода через день недели
    lastDOW = currentDOW; // сделаем вид, что мы ничего не знаем о переходе на новый день недели.
    // таким образом, код перехода на новый день недели выполнится всего один раз при каждом переходе
    // через день недели.    
  
  #endif // WATER_RELAYS_COUNT > 0

  
}
//--------------------------------------------------------------------------------------------------------------------------------
bool  WateringModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = UNKNOWN_COMMAND;

  size_t argsCount = command.GetArgsCount();
    
  if(command.GetType() == ctSET) 
  {   
      if(argsCount < 1) // не хватает параметров
      {
        PublishSingleton = PARAMS_MISSED;
      }
      else
      {
        String which = command.GetArg(0);
        //which.toUpperCase();

        if(which == WATER_SETTINGS_COMMAND) // CTSET=WATER|T_SETT|WateringOption|WateringDays|WateringTime|StartTime|TurnOnPump|wateringSensorIndex|wateringStopBorder|switchToAutoAfterMidnight|wateringStartBorder(optional)
        {
          if(argsCount > 5)
          {
              // парсим параметры
              uint8_t wateringOption = (uint8_t) atoi(command.GetArg(1)); 
              uint8_t wateringWeekDays = (uint8_t) atoi(command.GetArg(2)); 
              uint16_t wateringTime = (uint16_t) atoi(command.GetArg(3)); 
              uint16_t startWateringTime = (uint16_t) atoi(command.GetArg(4)); 
              uint8_t turnOnPump = (uint8_t) atoi(command.GetArg(5));

              int8_t wateringSensorIndex = -1;
              if(argsCount > 6)
              {
                wateringSensorIndex = (int8_t) atoi(command.GetArg(6));
              }

             uint8_t wateringStopBorder = 0;
              if(argsCount > 7)
              {
                wateringStopBorder = (uint8_t) atoi(command.GetArg(7));
              }

             switchToAutoAfterMidnight = 1;
             
              if(argsCount > 8)
              {
                switchToAutoAfterMidnight = (uint8_t) atoi(command.GetArg(8));             
              }

              uint8_t wateringStartBorder = 0;
              if(argsCount > 9)
              {
                wateringStartBorder = (uint8_t) atoi(command.GetArg(9));             
              }

              GlobalSettings* settings = MainController->GetSettings();

              uint8_t oldWateringOption = settings->GetWateringOption();
              
              // пишем в настройки
              settings->SetWateringOption(wateringOption);
              settings->SetWateringWeekDays(wateringWeekDays);
              settings->SetWateringTime(wateringTime);
              settings->SetStartWateringTime(startWateringTime);
              settings->SetTurnOnPump(turnOnPump);
              settings->SetWateringSensorIndex(wateringSensorIndex);
              settings->SetWateringStopBorder(wateringStopBorder);
              settings->SetTurnWateringToAutoAfterMidnight(switchToAutoAfterMidnight);
              settings->SetWateringStartBorder(wateringStartBorder);

              if(oldWateringOption != wateringOption)
              {
                 // состояние управления поливом изменилось, мы должны перезагрузить для всех каналов настройки из EEPROM
                 #if WATER_RELAYS_COUNT > 0

                  for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
                  {
                      wateringChannels[i].LoadState();
                  } // for
                 
                 #endif // WATER_RELAYS_COUNT > 0
              }

              // Поскольку пришла команда от юзера, и там среди параметров присутствует опция
              // управления поливом, то мы ВРОДЕ КАК должны переключиться в автоматический режим работы.
              // Если придёт команда из правил - мы выключим автоуправление поливом, и всё.
              // Если автоуправление поливом выключено - то мы в этой точке не вправе гасить
              // полив на всех каналах, т.к. юзер может вручную до этого включить канал полива.
              // Исходя из вышеизложенного - при изменении опции управления поливом делать ничего
              // не надо, кроме как вычитать состояние каналов из EEPROM, т.к. режим работы
              // может быть как ручным, так и автоматическим.              
              
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which; 
              PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          } // argsCount > 3
          else
          {
            // не хватает команд
            PublishSingleton = PARAMS_MISSED;
          }
          
        } // WATER_SETTINGS_COMMAND
        else
        if(which == WATER_CHANNEL_SETTINGS) // настройки канала CTSET=WATER|CH_SETT|IDX|WateringDays|WateringTime|StartTime|wateringSensorIndex|wateringStopBorder|wateringStartBorder(optional)
        {
           if(argsCount > 4)
           {
              #if WATER_RELAYS_COUNT > 0
                uint8_t channelIdx = (uint8_t) atoi(command.GetArg(1));
                if(channelIdx < WATER_RELAYS_COUNT)
                {
                  // нормальный индекс
                  uint8_t wDays = (uint8_t) atoi(command.GetArg(2));
                  uint16_t wTime =(uint16_t) atoi(command.GetArg(3));
                  uint16_t sTime = (uint16_t) atoi(command.GetArg(4));

                  int8_t wateringSensorIndex = -1;
                  if(argsCount > 5)
                  {
                    wateringSensorIndex = (int8_t) atoi(command.GetArg(5));
                  }
    
                  uint8_t wateringStopBorder = 0;
                  if(argsCount > 6)
                  {
                    wateringStopBorder = (uint8_t) atoi(command.GetArg(6));
                  }

                  uint8_t wateringStartBorder = 0;
                  if(argsCount > 7)
                  {
                    wateringStartBorder = (uint8_t) atoi(command.GetArg(7));
                  }
                  

                  GlobalSettings* settings = MainController->GetSettings();
                  
                  settings->SetChannelWateringWeekDays(channelIdx,wDays);
                  settings->SetChannelWateringTime(channelIdx,wTime);
                  settings->SetChannelStartWateringTime(channelIdx,sTime);
                  settings->SetChannelWateringSensorIndex(channelIdx,wateringSensorIndex);
                  settings->SetChannelWateringStopBorder(channelIdx,wateringStopBorder);
                  settings->SetChannelWateringStartBorder(channelIdx,wateringStartBorder);
				  
                  
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = which; 
                  PublishSingleton << PARAM_DELIMITER << (command.GetArg(1)) << PARAM_DELIMITER << REG_SUCC;
                 
                }
                else
                {
                  // плохой индекс
                  PublishSingleton = UNKNOWN_COMMAND;
                }
             #else
              PublishSingleton = UNKNOWN_COMMAND;
             #endif // WATER_RELAYS_COUNT > 0
           }
           else
           {
            // не хватает команд
            PublishSingleton = PARAMS_MISSED;            
           }
        }
        else
        if(which == F("SKIP")) // пропустить полив за сегодня
        {
            Skip(true);

            // состояние управления поливом изменилось, мы должны перезагрузить для всех каналов настройки из EEPROM
             #if WATER_RELAYS_COUNT > 0

              for(byte i=0;i<WATER_RELAYS_COUNT;i++)
              {
                  wateringChannels[i].LoadState();
              } // for
            
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which; 
              PublishSingleton << PARAM_DELIMITER << REG_SUCC;
              
            #else
              PublishSingleton = UNKNOWN_COMMAND;
            #endif
          
        } // SKIP
        else
        if(which == F("RESET")) // сбросить таймеры полива на сегодня
        {
            Skip(false);

            // состояние управления поливом изменилось, мы должны перезагрузить для всех каналов настройки из EEPROM
             #if WATER_RELAYS_COUNT > 0

              for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
              {
                  wateringChannels[i].LoadState();
              } // for
            
              PublishSingleton.Flags.Status = true;
              PublishSingleton = which; 
              PublishSingleton << PARAM_DELIMITER << REG_SUCC;
              
            #else
              PublishSingleton = UNKNOWN_COMMAND;
            #endif
          
        } // RESET
        else
        if(which == F("DURATION_ALL")) // установить продолжительность полива для всех каналов, CTSET=WATER|DURATION_ALL|Minutes (0-65535)
        {
          #if WATER_RELAYS_COUNT > 0
            if(argsCount > 1)
            {
                GlobalSettings* settings = MainController->GetSettings();
                uint16_t wateringTime =(uint16_t) atoi(command.GetArg(1));
                settings->SetWateringTime(wateringTime);
                PublishSingleton.Flags.Status = true;
                PublishSingleton = which; 
                PublishSingleton << PARAM_DELIMITER << REG_SUCC;
            }
            else
            {
              PublishSingleton = PARAMS_MISSED; 
            }
          #else
            PublishSingleton = UNKNOWN_COMMAND;
          #endif
          
        } // DURATION_ALL
        else
        if(which == F("DURATION_CH")) // установить продолжительность полива для канала, CTSET=WATER|DURATION_CH|Idx|Minutes (0-65535)
        {
          #if WATER_RELAYS_COUNT > 0
            if(argsCount > 2)
            {
                GlobalSettings* settings = MainController->GetSettings();
                uint8_t channelIdx = (uint8_t) atoi(command.GetArg(1));
                uint16_t wateringTime =(uint16_t) atoi(command.GetArg(2));
                settings->SetChannelWateringTime(channelIdx,wateringTime);
                PublishSingleton.Flags.Status = true;
                PublishSingleton = which; 
                PublishSingleton << PARAM_DELIMITER << channelIdx << PARAM_DELIMITER << REG_SUCC;
            }
            else
            {
              PublishSingleton = PARAMS_MISSED; 
            }
          #else
            PublishSingleton = UNKNOWN_COMMAND;
          #endif
          
        } // DURATION_CH
        else
        if(which == F("TIME_ALL")) // установить время начала полива для всех каналов, CTSET=WATER|TIME_ALL|Minutes (from 00:00)
        {
          #if WATER_RELAYS_COUNT > 0
            if(argsCount > 1)
            {
                GlobalSettings* settings = MainController->GetSettings();
                uint16_t startWateringTime =(uint16_t) atoi(command.GetArg(1));
                settings->SetStartWateringTime(startWateringTime);
                PublishSingleton.Flags.Status = true;
                PublishSingleton = which; 
                PublishSingleton << PARAM_DELIMITER << REG_SUCC;
            }
            else
            {
              PublishSingleton = PARAMS_MISSED; 
            }
          #else
            PublishSingleton = UNKNOWN_COMMAND;
          #endif
          
        } // TIME_ALL
        else
        if(which == F("TIME_CH")) // установить время начала полива для канала, CTSET=WATER|TIME_CH|Idx|Minutes (from 00:00)
        {
          #if WATER_RELAYS_COUNT > 0
            if(argsCount > 2)
            {
                GlobalSettings* settings = MainController->GetSettings();
                uint8_t channelIdx = (uint8_t) atoi(command.GetArg(1));
                uint16_t startWateringTime =(uint16_t) atoi(command.GetArg(2));
                settings->SetChannelStartWateringTime(channelIdx,startWateringTime);
                PublishSingleton.Flags.Status = true;
                PublishSingleton = which; 
                PublishSingleton << PARAM_DELIMITER << channelIdx << PARAM_DELIMITER << REG_SUCC;
            }
            else
            {
              PublishSingleton = PARAMS_MISSED; 
            }
          #else
            PublishSingleton = UNKNOWN_COMMAND;
          #endif
          
        } // TIME_CH        
        else
        if(which == WORK_MODE) // CTSET=WATER|MODE|AUTO, CTSET=WATER|MODE|MANUAL
        {
           // попросили установить режим работы
           String param = command.GetArg(1);
           
           if(param == WM_AUTOMATIC)
           {
            // переходим в автоматический режим работы
            SwitchToAutomaticMode();
           }
           else
           {
            // переходим на ручной режим работы
             SwitchToManualMode();
           }

              PublishSingleton.Flags.Status = true;
              PublishSingleton = which; 
              PublishSingleton << PARAM_DELIMITER << param;

              
        
        } // WORK_MODE
        else 
        if(which == STATE_ON) // попросили включить полив на всех каналах, CTSET=WATER|ON, или на одном из каналов: CTSET=WATER|ON|2
        {
           // Здесь ситуация интересная: мы можем быть в автоматическом режиме работы или в ручном.
           // если мы в ручном режиме работы и команда пришла не от юзера (а, например, из правил) - то нам надо выключить
           // автоуправление поливом и перейти в автоматический режим работы.
           // если же мы в автоматическом режиме и команда пришла не от юзера - также выключаем автоуправление поливом.
           // если команда пришла от юзера - переходим в ручной режим работы

           bool canProcessCommand = true;

          // если используем защиту полива - то можем включать каналы только, если защита полива не сработала
          #ifdef USE_WATERING_GUARD
            if(hasWGuardAlert())
            {
              canProcessCommand = false;
            }
          #endif // USE_WATERING_GUARD

           if(canProcessCommand)
           {
             if(argsCount < 2) // для всех каналов запросили
             {
                TurnChannelsOn(); // включаем все каналы
             }
             else
             {
               // запросили для одного канала
               uint8_t channelIndex = (uint8_t) atoi(command.GetArg(1));
               #if WATER_RELAYS_COUNT > 0
                if(channelIndex < WATER_RELAYS_COUNT)
                {
                  TurnChannelOn(channelIndex); // включаем полив на канале
                }
               #endif // WATER_RELAYS_COUNT > 0
             }
           } // if(canProcessCommand)

           // потом смотрим - откуда команда
           if(command.IsInternal())
           {
             // внутренняя команда
             GlobalSettings* settings = MainController->GetSettings();
             // выключаем автоуправление поливом
             settings->SetWateringOption(wateringOFF);

             if(flags.workMode == wwmManual)
             {
               // мы в ручном режиме работы, пришла внутренняя команда - надо переключиться в автоматический режим работы
               SwitchToAutomaticMode();
             }
            
           } // internal command
           else
           {
            // команда от пользователя
              SwitchToManualMode(); // переключаемся в ручной режим работы
           } // command from user
        
          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          if(argsCount > 1)
          {
            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << command.GetArg(1);
          }
        } // STATE_ON
        else 
        if(which == STATE_OFF) // попросили выключить полив на всех каналах, CTSET=WATER|OFF, или для одного канала: CTSET=WATER|OFF|3
        { 
           if(argsCount < 2)
           {
            TurnChannelsOff(); // выключаем все каналы
           }
           else
           {
             // запросили для одного канала
             uint8_t channelIndex = (uint8_t) atoi(command.GetArg(1));
             #if WATER_RELAYS_COUNT > 0
              if(channelIndex < WATER_RELAYS_COUNT)
              {
                TurnChannelOff(channelIndex); // выключаем полив на канале
              }
             #endif // WATER_RELAYS_COUNT > 0
           }            

           // потом смотрим - откуда команда
           if(command.IsInternal())
           {
             // внутренняя команда
             GlobalSettings* settings = MainController->GetSettings();
             // выключаем автоуправление поливом
             settings->SetWateringOption(wateringOFF);

             if(flags.workMode == wwmManual)
             {
               // мы в ручном режиме работы, пришла внутренняя команда - надо переключиться в автоматический режим работы
               SwitchToAutomaticMode();
             }
            
           } // internal command
           else
           {
            // команда от пользователя
              SwitchToManualMode(); // переключаемся в ручной режим работы
           } // command from user 

          PublishSingleton.Flags.Status = true;
          PublishSingleton = which;
          if(argsCount > 1)
          {
            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << command.GetArg(1);
          }         
        } // STATE_OFF        

      } // else
  }
  else
  if(command.GetType() == ctGET) //получить данные
  {    
    if(!argsCount) // нет аргументов, попросили вернуть статус полива
    {
      PublishSingleton.Flags.Status = true;
      #if WATER_RELAYS_COUNT > 0
      
        PublishSingleton = (IsAnyChannelActive() ? STATE_ON : STATE_OFF);
        
      #else
        PublishSingleton = STATE_OFF;
      #endif //  WATER_RELAYS_COUNT > 0
      
      PublishSingleton << PARAM_DELIMITER << (flags.workMode == wwmAutomatic ? WM_AUTOMATIC : WM_MANUAL);
    }
    else
    {
      String t = command.GetArg(0);
      
        if(t == WATER_SETTINGS_COMMAND) // запросили данные о настройках полива
        {
          GlobalSettings* settings = MainController->GetSettings();
          
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t; 
          PublishSingleton << PARAM_DELIMITER; 
          PublishSingleton << (settings->GetWateringOption()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetWateringWeekDays()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetWateringTime()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetStartWateringTime()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetTurnOnPump()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetWateringSensorIndex()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetWateringStopBorder()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetTurnWateringToAutoAfterMidnight()) << PARAM_DELIMITER;
          PublishSingleton << (settings->GetWateringStartBorder());
        }
        else
        if(t == WATER_CHANNELS_COUNT_COMMAND)
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t; 
          PublishSingleton << PARAM_DELIMITER << WATER_RELAYS_COUNT;
          
        }
        else
        if(t == WORK_MODE) // получить режим работы
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = WORK_MODE; 
          PublishSingleton << PARAM_DELIMITER << (flags.workMode == wwmAutomatic ? WM_AUTOMATIC : WM_MANUAL);
        }
        else
        if(t == F("STATEMASK")) // запросили маску состояния каналов
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = t;
          PublishSingleton << PARAM_DELIMITER << WATER_RELAYS_COUNT;
          
          #if WATER_RELAYS_COUNT > 0
          
              PublishSingleton << PARAM_DELIMITER;
              ControllerState state = WORK_STATUS.GetState();

              for(byte i=0;i<WATER_RELAYS_COUNT;i++)
              {
                if(state.WaterChannelsState & (1 << i)) // канал включен
                {
                  PublishSingleton << F("1");
                }
                else // канал выключен
                {
                  PublishSingleton << F("0");
                }
              } // for
              
          #endif // WATER_RELAYS_COUNT > 0
        } // STATEMASK
        else
        {
           // команда с аргументами
           if(argsCount > 1)
           {
                t = command.GetArg(0);
    
                if(t == WATER_CHANNEL_SETTINGS)
                {
                  #if WATER_RELAYS_COUNT > 0
                  // запросили настройки канала
                  uint8_t idx = (uint8_t) atoi(command.GetArg(1));
                  
                  if(idx < WATER_RELAYS_COUNT)
                  {
                    PublishSingleton.Flags.Status = true;

                    GlobalSettings* settings = MainController->GetSettings();
                 
                    PublishSingleton = t; 
                    PublishSingleton << PARAM_DELIMITER << (command.GetArg(1)) << PARAM_DELIMITER 
                    << (settings->GetChannelWateringWeekDays(idx)) << PARAM_DELIMITER
                    << (settings->GetChannelWateringTime(idx)) << PARAM_DELIMITER
                    << (settings->GetChannelStartWateringTime(idx)) << PARAM_DELIMITER
                    << (settings->GetChannelWateringSensorIndex(idx)) << PARAM_DELIMITER
                    << (settings->GetChannelWateringStopBorder(idx))  << PARAM_DELIMITER
                    << (settings->GetChannelWateringStartBorder(idx));
                  }
                  else
                  {
                    // плохой индекс
                    PublishSingleton = UNKNOWN_COMMAND;
                  }
                  #else
                    PublishSingleton = UNKNOWN_COMMAND;
                  #endif // WATER_RELAYS_COUNT > 0
                          
                } // if

           } // if(argsCount > 1)
          else // всего один аргумент
          {
            // тут просто получаем состояние канала, команда CTGET=WATER|1, например
            uint8_t idx = (uint8_t) atoi(command.GetArg(0));
            #if WATER_RELAYS_COUNT > 0
              if(idx >= WATER_RELAYS_COUNT)
              {
                idx = WATER_RELAYS_COUNT - 1;
              }

                PublishSingleton.Flags.Status = true;
                PublishSingleton = idx;
                PublishSingleton << PARAM_DELIMITER;
                
                PublishSingleton << (wateringChannels[idx].IsActive() ? STATE_ON : STATE_OFF);
            #endif
          } // one command argument           
          
        } // else command with arbuments
        
    } // else have arguments
  } // if ctGET
 
 // отвечаем на команду
    MainController->Publish(this,command);
    
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERING_MODULE
