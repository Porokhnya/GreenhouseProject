#include "WateringModule.h"
#include "ModuleController.h"
#include "Memory.h"
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE

#ifdef USE_LOG_MODULE
  #if TARGET_BOARD == STM32_BOARD
  #include <SdFatSTM32.h>
  #else
  #include <SdFat.h> // пробуем записать статус полива не только в EEPROM, но и на SD-карту, если LOG-модуль есть в прошивке
  #endif
#endif
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
static uint8_t WATER_RELAYS[] = { WATER_RELAYS_PINS }; // объявляем массив пинов реле
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::SignalToHardware()
{
    byte state = flags.isON ? WATER_RELAY_ON : WATER_RELAY_OFF;

    WTR_LOG(F("[WTR] - channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F(" write to pin #"));
    WTR_LOG( String(WATER_RELAYS[flags.index]) );
    WTR_LOG(F(", state = "));
    WTR_LOG(String(state));
    WTR_LOG(F("\r\n"));

   #if WATER_DRIVE_MODE == DRIVE_DIRECT
    
      WORK_STATUS.PinWrite(WATER_RELAYS[flags.index],state);
      
    #elif WATER_DRIVE_MODE == DRIVE_MCP23S17
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        
          WORK_STATUS.MCP_SPI_PinWrite(WATER_MCP23S17_ADDRESS,WATER_RELAYS[flags.index],state);
          
        #endif
    #elif WATER_DRIVE_MODE == DRIVE_MCP23017
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        
          WORK_STATUS.MCP_I2C_PinWrite(WATER_MCP23017_ADDRESS,WATER_RELAYS[flags.index],state);
          
        #endif
    #endif

  WORK_STATUS.SaveWaterChannelState(flags.index,state); // сохраняем статус канала полива
    
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
WateringChannel::WateringChannel()
{
  flags.isON = flags.lastIsON = false;
  flags.index = 0;
  flags.wateringTimer = /*flags.wateringDelta =*/ 0;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::Setup(byte index)
{
    flags.index = index;
    flags.isON = flags.lastIsON = false;
    flags.wateringTimer = /*flags.wateringDelta =*/ 0;
  
    WTR_LOG(F("[WTR] - setup channel "));
    WTR_LOG(String(flags.index));
    WTR_LOG(F("; OFF relay...\r\n"));

    #if WATER_DRIVE_MODE == DRIVE_DIRECT
      WORK_STATUS.PinMode(WATER_RELAYS[flags.index],OUTPUT);
    #elif WATER_DRIVE_MODE == DRIVE_MCP23S17
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(WATER_MCP23S17_ADDRESS,WATER_RELAYS[flags.index],OUTPUT);
        #endif
    #elif WATER_DRIVE_MODE == DRIVE_MCP23017
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(WATER_MCP23017_ADDRESS,WATER_RELAYS[flags.index],OUTPUT);
        #endif
    #endif    

    SignalToHardware(); // записываем в пины изначально выключенный уровень

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
        message += STATE_ON;
      else
        message += STATE_OFF;

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
        message += STATE_ON;
      else
        message += STATE_OFF;

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
void WateringChannel::Update(WateringModule* m, uint16_t _dt,WateringWorkMode currentWorkMode, const RTCTime& currentTime, int8_t savedDayOfWeek)
{
  #ifdef USE_DS3231_REALTIME_CLOCK

    // только если модуль часов есть в прошивке - тогда обновляем состояние

    if(currentWorkMode == wwmManual) // в ручном режиме управления, ничего не делаем
      return;

    GlobalSettings* settings = MainController->GetSettings();
    uint8_t currentWateringOption = settings->GetWateringOption();

    if(currentWateringOption == wateringOFF) // автоматическое управление каналами выключено, не надо обновлять состояние канала
      return;

     unsigned long dt = _dt;
  
     // теперь получаем настойки полива на канале. Они зависят от currentWateringOption: если там wateringWeekDays - рулим всеми каналами одновременно,
     // если там wateringSeparateChannels - рулим каналами по отдельности, если там wateringOFF - мы не попадём в эту ветку кода.
     
     uint8_t weekDays = currentWateringOption == wateringWeekDays ? settings->GetWateringWeekDays() : settings->GetChannelWateringWeekDays(flags.index);
     // получаем время начала полива, в минутах от начала суток
     uint16_t startWateringTime = currentWateringOption == wateringWeekDays ? settings->GetStartWateringTime() : settings->GetChannelStartWateringTime(flags.index);
     unsigned long timeToWatering = currentWateringOption == wateringWeekDays ? settings->GetWateringTime() : settings->GetChannelWateringTime(flags.index); // время полива (в минутах!)
     int8_t sensorIndex = currentWateringOption == wateringWeekDays ? settings->GetWateringSensorIndex() : settings->GetChannelWateringSensorIndex(flags.index);
     int8_t stopBorder = currentWateringOption == wateringWeekDays ? settings->GetWateringStopBorder() : settings->GetChannelWateringStopBorder(flags.index);
     

      // переход через день недели мы фиксируем однократно, поэтому нам важно его не пропустить.
      // можем ли мы работать или нет - неважно, главное - правильно обработать переход через день недели.
      
      if(savedDayOfWeek != currentTime.dayOfWeek)  // сначала проверяем, не другой ли день недели уже?
      {
        flags.wateringTimer = 0; // сбрасываем таймер полива, т.к. начался новый день недели       
      } // if(savedDayOfWeek != currentTime.dayOfWeek)      


    // проверяем, установлен ли у нас день недели для полива, и настала ли минута, с которого можно поливать
    uint16_t currentTimeInMinutes = currentTime.hour*60 + currentTime.minute;
    bool canWork = bitRead(weekDays,currentTime.dayOfWeek-1) && (currentTimeInMinutes >= startWateringTime);

    if(canWork)
    {
      // проверяем, можем ли мы работать по условию с датчика влажности почвы
      if(sensorIndex > -1 && stopBorder > 0)
      {
        AbstractModule* mod = MainController->GetModuleByID("SOIL");
        if(mod)
        {
           OneState* os = mod->State.GetState(StateSoilMoisture,sensorIndex);
           if(os && os->HasData())
           {
              HumidityPair hp = *os;
              if(hp.Current.Value >= stopBorder) // показания с датчика больше или равны порогу выключения
                canWork = false;
           }
        }
      } // if
    }
  
    if(!canWork)
     { 
       Off(m); // выключаем реле
     }
     else
     {
      // можем работать, смотрим, не вышли ли мы за пределы установленного интервала

      flags.wateringTimer += dt; // прибавляем время работы

      // тут смотрим, не надо ли сохранить время полива, по шагам (например, каждую минуту)
      if(flags.lastSavedStateMinute != -1  && flags.lastSavedStateMinute != currentTime.minute && !(currentTime.minute % WATERING_STATUS_SAVE_INTERVAL))
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

      unsigned long maxTimeToWatering = ((timeToWatering*60000) /*+ flags.wateringDelta*/ + dt);
  
      if(flags.wateringTimer > maxTimeToWatering) // приплыли, надо выключать полив
      {
        //unsigned long diff = flags.wateringDelta + dt;
        flags.wateringTimer -=  dt;// оставляем таймер застывшим на окончании полива, плюс маленькая дельта
//        flags.wateringDelta = 0; // сбросили дельту дополива

        if(IsActive()) // если канал был включён, значит, он будет выключен, и мы однократно запишем в EEPROM нужное значение
        {
         SaveState(flags.wateringTimer);
        } // if(IsActive())

        Off(m); // выключаем реле
      
      }
      else
        On(m); // ещё можем работать, продолжаем поливать
        
     } // else can work
     

  #endif // USE_DS3231_REALTIME_CLOCK
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringChannel::DoLoadState(byte addressOffset)
{
   // сперва сбрасываем настройки времени полива и дополива
   flags.wateringTimer = /*flags.wateringDelta =*/ 0;
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

    unsigned long savedWorkTime = 0xFFFFFFFF;
    volatile byte* writeAddr = (byte*) &savedWorkTime;
    uint8_t savedDOW = 0xFF;

    // мы читаем 5 байт на канал, поэтому вычисляем адрес очень просто - по смещению addressOffset, в котором находится индекс канала
    volatile uint16_t curReadAddr = WATERING_STATUS_EEPROM_ADDR + addressOffset*5;

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
        flags.wateringTimer = 0; // сбрасываем таймер
      
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
void WateringChannel::DoSaveState(byte addressOffset,unsigned long wateringTimer)
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
    unsigned long timeToWatering = 0;
    
    if(wateringTimer > 0) // если передали время полива - сохраняем
      timeToWatering = wateringTimer;
    else // иначе - пишем полное время полива
    {
      timeToWatering = addressOffset == 0 ? settings->GetWateringTime() : settings->GetChannelWateringTime(flags.index); // время полива (в минутах!)
      timeToWatering *= 60000;
    }

     //Тут сохранение в EEPROM статуса, что мы на сегодня уже полили сколько-то времени на канале
    uint16_t wrAddr = WATERING_STATUS_EEPROM_ADDR + addressOffset*5; // адрес записи
    
    // сохраняем в EEPROM день недели, для которого запомнили значение таймера
    MemWrite(wrAddr++,today);
    
    // сохраняем в EEPROM значение таймера канала
    byte* readAddr = (byte*) &timeToWatering;
    for(int i=0;i<4;i++)
      MemWrite(wrAddr++,*readAddr++);
    
 #else
    WTR_LOG(F("[WTR] - NO state for channel - no realtime clock!\r\n"));
 #endif // USE_DS3231_REALTIME_CLOCK

}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // WATER_RELAYS_COUNT > 0
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PUMP_RELAY
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
    for(byte i=0;i<WATER_RELAYS_COUNT;i++)
    {
        if(wateringChannels[i].IsActive())
        {
          // канал активен, смотрим, к какому насосу он относится
          #ifdef USE_SECOND_PUMP
            // два насоса в прошивке
             if(i < SECOND_PUMP_START_CHANNEL)
             {
                // канал относится к первому насосу
                pump1State = true;
             }
             else
             {
              // канал относится ко второму насосу
              pump2State = true;
             }
          #else
           // один насос в прошивке, включаем по-любому
           pump1State = true;
           return;
          #endif // #ifdef USE_SECOND_PUMP
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
    TurnPump1(false);

    #ifdef USE_SECOND_PUMP // второй насос
      TurnPump2(false);
    #endif
    
    return;
  }

  // здесь проверяем, какой насос включить. Для этого узнаём, есть ли активные каналы для первого и второго насоса
  bool shouldTurnPump1,shouldTurnPump2;

  // проверяем статус, который надо выставить для насосов
  GetPumpsState(shouldTurnPump1,shouldTurnPump2);

  TurnPump1(shouldTurnPump1);
  
  #ifdef USE_SECOND_PUMP
    TurnPump2(shouldTurnPump2);
  #endif
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnPump1(bool isOn)
{
  if(flags.isPump1On == isOn) // состояние не изменилось
    return;

  // сохраняем состояние
  flags.isPump1On = isOn;

  WTR_LOG(F("Turn pump #1 to state: "));
  WTR_LOG(isOn ? F("ON\r\n") : F("OFF\r\n"));
  
  byte state = isOn ? WATER_PUMP_RELAY_ON : WATER_PUMP_RELAY_OFF;
  
  #if WATER_PUMP_DRIVE_MODE == DRIVE_DIRECT
    WORK_STATUS.PinWrite(PUMP_RELAY_PIN,state);
  #elif WATER_PUMP_DRIVE_MODE == DRIVE_MCP23S17
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      WORK_STATUS.MCP_SPI_PinWrite(WATER_PUMP_MCP23S17_ADDRESS,PUMP_RELAY_PIN,state);
    #endif
  #elif WATER_PUMP_DRIVE_MODE == DRIVE_MCP23017
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      WORK_STATUS.MCP_I2C_PinWrite(WATER_PUMP_MCP23017_ADDRESS,PUMP_RELAY_PIN,state);
    #endif
  #endif  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SECOND_PUMP
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnPump2(bool isOn)
{
  if(flags.isPump2On == isOn) // состояние не изменилось
    return;

  // сохраняем состояние
  flags.isPump2On = isOn;

  WTR_LOG(F("Turn pump #2 to state: "));
  WTR_LOG(isOn ? F("ON\r\n") : F("OFF\r\n"));
  
  byte state = isOn ? WATER_PUMP_RELAY_ON : WATER_PUMP_RELAY_OFF;
  
    #if WATER_PUMP2_DRIVE_MODE == DRIVE_DIRECT
      WORK_STATUS.PinWrite(SECOND_PUMP_RELAY_PIN,state);
    #elif WATER_PUMP2_DRIVE_MODE == DRIVE_MCP23S17
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinWrite(WATER_PUMP_MCP23S17_ADDRESS,SECOND_PUMP_RELAY_PIN,state);
      #endif
    #elif WATER_PUMP2_DRIVE_MODE == DRIVE_MCP23017
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinWrite(WATER_PUMP_MCP23017_ADDRESS,SECOND_PUMP_RELAY_PIN,state);
      #endif
    #endif
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SECOND_PUMP
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::SetupPumps()
{
    WTR_LOG(F("[WTR] = setup pumps\r\n"));

    WTR_LOG(F("[WTR] - Turn OFF pump 1\r\n"));
  
  #if WATER_PUMP_DRIVE_MODE == DRIVE_DIRECT
    WORK_STATUS.PinMode(PUMP_RELAY_PIN,OUTPUT);
    WORK_STATUS.PinWrite(PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
  #elif WATER_PUMP_DRIVE_MODE == DRIVE_MCP23S17
    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      WORK_STATUS.MCP_SPI_PinMode(WATER_PUMP_MCP23S17_ADDRESS,PUMP_RELAY_PIN,OUTPUT);
      WORK_STATUS.MCP_SPI_PinWrite(WATER_PUMP_MCP23S17_ADDRESS,PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
    #endif
  #elif WATER_PUMP_DRIVE_MODE == DRIVE_MCP23017
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      WORK_STATUS.MCP_I2C_PinMode(WATER_PUMP_MCP23017_ADDRESS,PUMP_RELAY_PIN,OUTPUT);
      WORK_STATUS.MCP_I2C_PinWrite(WATER_PUMP_MCP23017_ADDRESS,PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
    #endif
  #endif

  #ifdef USE_SECOND_PUMP // второй насос

    WTR_LOG(F("[WTR] - Turn OFF pump 2\r\n"));
  
    #if WATER_PUMP2_DRIVE_MODE == DRIVE_DIRECT
      WORK_STATUS.PinMode(SECOND_PUMP_RELAY_PIN,OUTPUT);
      WORK_STATUS.PinWrite(SECOND_PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
    #elif WATER_PUMP2_DRIVE_MODE == DRIVE_MCP23S17
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        WORK_STATUS.MCP_SPI_PinMode(WATER_PUMP_MCP23S17_ADDRESS,SECOND_PUMP_RELAY_PIN,OUTPUT);
        WORK_STATUS.MCP_SPI_PinWrite(WATER_PUMP_MCP23S17_ADDRESS,SECOND_PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
      #endif
    #elif WATER_PUMP2_DRIVE_MODE == DRIVE_MCP23017
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        WORK_STATUS.MCP_I2C_PinMode(WATER_PUMP_MCP23017_ADDRESS,SECOND_PUMP_RELAY_PIN,OUTPUT);
        WORK_STATUS.MCP_I2C_PinWrite(WATER_PUMP_MCP23017_ADDRESS,SECOND_PUMP_RELAY_PIN,WATER_PUMP_RELAY_OFF);
      #endif
    #endif
    
  #endif // USE_SECOND_PUMP      
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PUMP_RELAY
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Setup()
{
  // настройка модуля тут
  WTR_LOG(F("[WTR] - setup...\r\n"));

  #ifdef USE_WATERING_GUARD
    setupWGuard(); // настраиваем защиту полива
  #endif // USE_WATERING_GUARD

 // GlobalSettings* settings = MainController->GetSettings();

  flags.workMode = wwmAutomatic; // автоматический режим работы
  flags.isPump1On = false;
  flags.isPump2On = false;

  // настраиваем насосы
  #ifdef USE_PUMP_RELAY
    SetupPumps();
  #endif
  
  #if WATER_RELAYS_COUNT > 0

    // настраиваем каналы
    for(uint8_t i=0;i<WATER_RELAYS_COUNT;i++)
    {
        // просим канал настроиться, она загрузит свои настройки и выключит реле
        wateringChannels[i].Setup(i);
    } // for
  
  #endif // WATER_RELAYS_COUNT > 0

  // если указано - использовать диод индикации ручного режима работы - настраиваем его
  #ifdef USE_WATERING_MANUAL_MODE_DIODE
    blinker.begin(DIODE_WATERING_MANUAL_MODE_PIN);  // настраиваем блинкер на нужный пин
    blinker.blink(); // и гасим его по умолчанию
  #endif  


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
    return;

  WTR_LOG(F("[WTR] - switch to automatic mode\r\n"));

    flags.workMode = wwmAutomatic;

  // гасим блинкер, если он используется в прошивке
  #ifdef USE_WATERING_MANUAL_MODE_DIODE
     blinker.blink();
  #endif    
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::SwitchToManualMode()
{
  if(flags.workMode == wwmManual) // уже в ручном режиме
    return;

  WTR_LOG(F("[WTR] - switch to manual mode\r\n"));

    flags.workMode = wwmManual;

  // зажигаем блинкер, если он используется в прошивке
  #ifdef USE_WATERING_MANUAL_MODE_DIODE
     blinker.blink(WORK_MODE_BLINK_INTERVAL);
  #endif    
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Skip(bool skipOrReset) // пропускаем полив за сегодня
{
  WTR_LOG(F("[WTR] - skip watering for today\r\n"));

  uint8_t today = 0xFF;
  unsigned long max_work_time =  0xFFFFFFFF;

 #ifdef USE_DS3231_REALTIME_CLOCK

    RealtimeClock watch =  MainController->GetClock();
    RTCTime t =   watch.getTime();
    today = t.dayOfWeek; // запоминаем текущий день недели
    max_work_time =  skipOrReset ? 0xFFFFF0F0 : 0;
     
  #endif
      
  // выставляем в EEPROM максимальное значение полива по каналам за сегодня
  uint16_t wrAddr = WATERING_STATUS_EEPROM_ADDR;
  uint8_t recordsCount = WATER_RELAYS_COUNT + 1;
  
  for(uint8_t i=0;i<recordsCount;i++)
  {
    MemWrite(wrAddr++,today);
    byte* readAddr = (byte*) &max_work_time;
      for(int k=0;k<4;k++)
        MemWrite(wrAddr++,*readAddr++);    
  }
  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::ResetChannelsState()
{
  WTR_LOG(F("[WTR] - reset channels state\r\n"));
  //Тут затирание в EEPROM предыдущего сохранённого значения о статусе полива на всех каналах
  uint16_t wrAddr = WATERING_STATUS_EEPROM_ADDR;
  uint8_t bytes_to_write = 5 + WATER_RELAYS_COUNT*5;
  
  for(uint8_t i=0;i<bytes_to_write;i++)
    MemWrite(wrAddr++,0); // для каждого канала по отдельности  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelsOff() // выключает все каналы
{
  WTR_LOG(F("[WTR] - turn all channels OFF\r\n"));
  
  #if WATER_RELAYS_COUNT > 0
    for(byte i=0;i<WATER_RELAYS_COUNT;i++)
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
    for(byte i=0;i<WATER_RELAYS_COUNT;i++)
    {
      wateringChannels[i].On(this);
    }
  #endif // WATER_RELAYS_COUNT > 0  
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::TurnChannelOff(byte channelIndex) // выключает канал
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
void WateringModule::TurnChannelOn(byte channelIndex) // включает канал
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
    for(byte i=0;i<WATER_RELAYS_COUNT;i++)
    {
      if(wateringChannels[i].IsActive())
        return true;
    }
  #endif // WATER_RELAYS_COUNT > 0    

  return false;
}
//-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void WateringModule::Update(uint16_t dt)
{ 
  
  // обновляем блинкер, если он используется в прошивке
  #ifdef USE_WATERING_MANUAL_MODE_DIODE
     blinker.update(dt);
  #endif

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
      GlobalSettings* sett = MainController->GetSettings();
      
      //#ifdef SWITCH_TO_AUTOMATIC_WATERING_MODE_AFTER_MIDNIGHT
      if(sett->GetTurnWateringToAutoAfterMidnight())
      {
        // начался новый день недели, принудительно переходим в автоматический режим работы
        // даже если до этого был включен полив командой от пользователя
        SwitchToAutomaticMode();
  
        // здесь надо принудительно гасить полив на всех каналах, поскольку у нас может быть выключено автоуправление каналами.
        // в этом случае, если полив был включен пользователем и настали новые сутки - полив не выключится сам,
        // т.к. канал не обновляет своё состояние при выключенном автоуправлении каналами.
        TurnChannelsOff();

      } // if(sett->GetTurnWateringToAutoAfterMidnight())
      
      //#endif // SWITCH_TO_AUTOMATIC_WATERING_MODE_AFTER_MIDNIGHT

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
          wateringChannels[i].Update(this,dt,(WateringWorkMode) flags.workMode,t,lastDOW);
      } // for
    }

    #ifdef USE_PUMP_RELAY
      UpdatePumps();
    #endif

   // обновили все каналы, теперь можно сбросить флаг перехода через день недели
    lastDOW = currentDOW; // сделаем вид, что мы ничего не знаем о переходе на новый день недели.
    // таким образом, код перехода на новый день недели выполнится всего один раз при каждом переходе
    // через день недели.    
  
  #else
    UNUSED(dt);
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

        if(which == WATER_SETTINGS_COMMAND) // CTSET=WATER|T_SETT|WateringOption|WateringDays|WateringTime|StartTime|TurnOnPump|wateringSensorIndex|wateringStopBorder|switchToAutoAfterMidnight
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
                wateringSensorIndex = (int8_t) atoi(command.GetArg(6));

             uint8_t wateringStopBorder = 0;
              if(argsCount > 7)
                wateringStopBorder = (uint8_t) atoi(command.GetArg(7));

             uint8_t switchToAutoAfterMidnight = 
             #ifdef SWITCH_TO_AUTOMATIC_WATERING_MODE_AFTER_MIDNIGHT
             1
             #else
             0
             #endif
             ;
              if(argsCount > 8)
                switchToAutoAfterMidnight = (uint8_t) atoi(command.GetArg(8));             

              GlobalSettings* settings = MainController->GetSettings();

              byte oldWateringOption = settings->GetWateringOption();
              
              // пишем в настройки
              settings->SetWateringOption(wateringOption);
              settings->SetWateringWeekDays(wateringWeekDays);
              settings->SetWateringTime(wateringTime);
              settings->SetStartWateringTime(startWateringTime);
              settings->SetTurnOnPump(turnOnPump);
              settings->SetWateringSensorIndex(wateringSensorIndex);
              settings->SetWateringStopBorder(wateringStopBorder);
              settings->SetTurnWateringToAutoAfterMidnight(switchToAutoAfterMidnight);

              if(oldWateringOption != wateringOption)
              {
                 // состояние управления поливом изменилось, мы должны перезагрузить для всех каналов настройки из EEPROM
                 #if WATER_RELAYS_COUNT > 0

                  for(byte i=0;i<WATER_RELAYS_COUNT;i++)
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
        if(which == WATER_CHANNEL_SETTINGS) // настройки канала CTSET=WATER|CH_SETT|IDX|WateringDays|WateringTime|StartTime|wateringSensorIndex|wateringStopBorder
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
                    wateringSensorIndex = (int8_t) atoi(command.GetArg(5));
    
                  uint8_t wateringStopBorder = 0;
                  if(argsCount > 6)
                    wateringStopBorder = (uint8_t) atoi(command.GetArg(6));

                  GlobalSettings* settings = MainController->GetSettings();
                  
                  settings->SetChannelWateringWeekDays(channelIdx,wDays);
                  settings->SetChannelWateringTime(channelIdx,wTime);
                  settings->SetChannelStartWateringTime(channelIdx,sTime);
                  settings->SetChannelWateringSensorIndex(channelIdx,wateringSensorIndex);
                  settings->SetChannelWateringStopBorder(channelIdx,wateringStopBorder);
                  
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
              canProcessCommand = false;
          #endif // USE_WATERING_GUARD

           if(canProcessCommand)
           {
             if(argsCount < 2) // для всех каналов запросили
                TurnChannelsOn(); // включаем все каналы
             else
             {
               // запросили для одного канала
               byte channelIndex = (byte) atoi(command.GetArg(1));
               #if WATER_RELAYS_COUNT > 0
                if(channelIndex < WATER_RELAYS_COUNT)
                  TurnChannelOn(channelIndex); // включаем полив на канале
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
            TurnChannelsOff(); // выключаем все каналы
           else
           {
             // запросили для одного канала
             byte channelIndex = (byte) atoi(command.GetArg(1));
             #if WATER_RELAYS_COUNT > 0
              if(channelIndex < WATER_RELAYS_COUNT)
                TurnChannelOff(channelIndex); // выключаем полив на канале
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
          PublishSingleton << (settings->GetTurnWateringToAutoAfterMidnight());
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
                    << (settings->GetChannelWateringStopBorder(idx));
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
                idx = WATER_RELAYS_COUNT - 1;

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
