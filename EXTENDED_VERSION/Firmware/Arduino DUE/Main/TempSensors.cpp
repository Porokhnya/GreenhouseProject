#include "TempSensors.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
#include "PowerManager.h"
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS

TempSensors* WindowModule = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t WindowState::GetWorkTime() // возвращает время работы, в миллисекундах
{
   WindowsIntervals intervals = HardwareBinding->GetWindowsIntervals();  
  if(intervals.Interval[flags.Index]  < 1) // использовать общий интервал
  {
    return MainController->GetSettings()->GetOpenInterval(); // тут у нас сразу в миллисекундах
  }
   return (1000ul*intervals.Interval[flags.Index]); // у нас интервал в секундах !!!
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::ResetToMaxPosition() // сбрасывает процент открытия окна на максимальную позицию
{
  CurrentPosition = GetWorkTime(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t WindowState::GetCurrentPositionPercents() // возвращает процент открытия окна от максимальной позиции
{
  uint32_t oI = GetWorkTime();
  return (CurrentPosition*100)/oI;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::Setup(uint8_t index, uint8_t relayChannel1, uint8_t relayChannel2)
{
  _timer = 0;
  _fullCloseTimer = 0;
  _waitFullClose = false;  
  powerTimerDelay = 0; // нет задержки по питанию
  
  #ifdef USE_FEEDBACK_MANAGER
    CurrentPosition = 0;
  #else
  // считаем, что как будто мы открыты, т.к. при старте контроллера надо принудительно закрыть окна
  ResetToMaxPosition();
  #endif
  
  // запоминаем, какие каналы модуля реле мы используем (в случае со сдвиговым регистром - это номера битов)
  RelayChannel1 = relayChannel1;
  RelayChannel2 = relayChannel2;

  // запоминаем индекс окна
  flags.Index = index;

  // настраиваем концевики
  #ifdef USE_WINDOWS_ENDSTOPS

    EndstopsBinding ebnd = HardwareBinding->GetEndstopsBinding();

    if(ebnd.LinkType != linkUnbinded)
    {
          if(ebnd.LinkType == linkDirect)
          {
              if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(ebnd.EndstopOpenPins[flags.Index]))
              {
                WORK_STATUS.PinMode(ebnd.EndstopOpenPins[flags.Index],INPUT);
              }
            
              if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(ebnd.EndstopClosePins[flags.Index]))
              {      
                  WORK_STATUS.PinMode(ebnd.EndstopClosePins[flags.Index],INPUT);
              }

          }
          else
          if(ebnd.LinkType == linkMCP23S17)
          {
            #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
               if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN)
               {
                  WORK_STATUS.MCP_SPI_PinMode(ebnd.MCPAddress1,ebnd.EndstopOpenPins[flags.Index],INPUT);
               }
               if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN)
               {
                  WORK_STATUS.MCP_SPI_PinMode(ebnd.MCPAddress2,ebnd.EndstopClosePins[flags.Index],INPUT);                
               }
            #endif
          }
          else
          if(ebnd.LinkType == linkMCP23017)
          {
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
               if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN)
               {
                  WORK_STATUS.MCP_I2C_PinMode(ebnd.MCPAddress1,ebnd.EndstopOpenPins[flags.Index],INPUT);
               }
               if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN)
               {
                  WORK_STATUS.MCP_I2C_PinMode(ebnd.MCPAddress2,ebnd.EndstopClosePins[flags.Index],INPUT);                
               }
            #endif
          }
      
    } // if(ebnd.LinkType != linkUnbinded)
  
  
  #endif // USE_WINDOWS_ENDSTOPS
  

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WindowState::ChangePosition(uint32_t newPos, bool waitFor)
{
#ifdef WINDOW_MANAGE_DEBUG
  DEBUG_LOG(F("Window #")); DEBUG_LOGLN(String(flags.Index)); 
  DEBUG_LOG(F("POSITION REQUESTED: ")); DEBUG_LOGLN(String(newPos));
  DEBUG_LOG(F("POSITION CURRENT: ")); DEBUG_LOGLN(String(CurrentPosition));
#endif
  
 if(IsBusy() && waitForChangePositionDone)
 {
   // ещё двигаемся, при этом нас попросили подождать, пока мы не достигнем этой позиции
  // Serial.println("Position  not reached!");
   return false;
 }
  
  long currentDifference = 0;
  if(CurrentPosition > newPos)
    currentDifference = CurrentPosition - newPos;
  else
    currentDifference = newPos - CurrentPosition;


  if(CurrentPosition == newPos || currentDifference < WINDOW_POSITION_HISTERESIS) 
  {
    // та же самая позиция запрошена, или разница текущей позиции и запрошеной - в пределах гистерезиса.
    // в этом случае мы ничего не делаем.
    #ifdef WINDOW_MANAGE_DEBUG   
      DEBUG_LOGLN(F("SAME POSITION!"));
   #endif
    return false;
  }

  // тут нам надо смотреть - если мы никак не управляемся - то сразу устанавливать смену позиции, и всё!!!
  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
  
  uint8_t pin1 = bnd.WindowsPins[RelayChannel1];
  uint8_t pin2 = bnd.WindowsPins[RelayChannel2];
  
  if(bnd.LinkType == linkUnbinded || pin1 == UNBINDED_PIN || pin2 == UNBINDED_PIN)
  {
    // либо не настроено управление, либо - один из пинов не привязан - окно нормально управляться не может!!!
    // просто устанавливаем позицию - и выходим.
    
    waitForChangePositionDone = false;
    CurrentPosition = newPos;
    flags.Direction = dirNOTHING;
    flags.OnMyWay = false;
    
    //ВЫКЛЮЧАЕМ РЕЛЕ
    SwitchRelays(!bnd.Level,!bnd.Level);
    
    // говорим, что мы сменили позицию
    SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);
    
    return true;
  }

  // если текущая позиция больше запрошенной - надо закрывать, иначе - открывать
  uint8_t dir = CurrentPosition > newPos ? dirCLOSE : dirOPEN;

  // сохраняем флаг ожидания окончания смены позиции
  waitForChangePositionDone = waitFor;
 
  if(dir == dirOPEN)
  {

       // открываем тут
       TimerInterval = newPos - CurrentPosition;
       flags.Direction = dir;

        #ifdef WINDOW_MANAGE_DEBUG
            DEBUG_LOG(F("OPEN FROM POSITION "));
            DEBUG_LOG(String(CurrentPosition));
            DEBUG_LOG(F(" TO "));
            DEBUG_LOGLN(String(newPos));
        #endif
  }
  else
  if(dir == dirCLOSE)
  {
        TimerInterval = CurrentPosition - newPos;
        flags.Direction = dir;
			
			
		if(!newPos)  // попросили закрыться полностью, увеличиваем время закрытия на 10%
        {
          _fullCloseTimer = bnd.AdditionalCloseTime;
          
          #ifdef WINDOW_MANAGE_DEBUG
            DEBUG_LOG(F("ADD ADDITIONAL CLOSE TIME: "));
            DEBUG_LOGLN(String(_fullCloseTimer));
          #endif
        }
        else 
        {
         _fullCloseTimer = 0;  
        }
		
        #ifdef WINDOW_MANAGE_DEBUG
            DEBUG_LOG(F("CLOSE FROM POSITION "));
            DEBUG_LOG(String(CurrentPosition));
            DEBUG_LOG(F(" TO "));
            DEBUG_LOGLN(String(newPos));
        #endif
  }

  // будем двигаться, надо получить задержку перед началом движения
  powerTimerDelay = PowerManager.WindowWantMove(flags.Index);
  
  flags.OnMyWay = true; // поогнали!
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::SwitchRelays(uint8_t rel1State, uint8_t rel2State)
{
  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();

  // уведомляем родителя, что такой-то канал имеет такое-то состояние, он сам разберётся, что с этим делать
  // в ращем случае он запишет на выходы нужный уровень.
  WindowModule->SaveChannelState(RelayChannel1,rel1State);
  WindowModule->SaveChannelState(RelayChannel2,rel2State);

  // тут говорим слепку состояния, чтобы он запомнил состояние каналов окон
  // проблема последних изменений в том, что состояние RELAY_ON - не является признаком включения канала,
  // поэтому мы должны проверять флаги rel1State и rel2State, и если она равны статусу Level в настройках - то
  // передавать RELAY_ON, иначе - !RELAY_ON. В этом случае всё будет работать корректно.

  WORK_STATUS.SaveWindowState(RelayChannel1,rel1State == bnd.Level ? RELAY_ON : !RELAY_ON);
  WORK_STATUS.SaveWindowState(RelayChannel2,rel2State == bnd.Level ? RELAY_ON : !RELAY_ON);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::Feedback(bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents, bool isFirstFeedback)
{  
  UNUSED(isFirstFeedback);
    
  uint32_t interval = GetWorkTime();

  if(hasPosition)
  {
    // есть информация о позиции, не надо дожидаться смены позиции, даже если попросили при старте (когда окна гонятся в закрытое положение)
    waitForChangePositionDone = false;
  }

  if(isCloseSwitchTriggered || isOpenSwitchTriggered) // если сработал один из концевиков, то это значит, что нам надо выключить моторы, и обновить позицию
  {
    // если мы ждали окончания смены позиции - надо по-любому сбросить этот флаг, т.к. сработал один из концевиков
    waitForChangePositionDone = false;
        
    if(IsBusy())
    {
      // двигаемся, надо останавливаться
      flags.OnMyWay = false;
      WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
      SwitchRelays(!bnd.Level,!bnd.Level); // держим реле выключенными
      flags.Direction = dirNOTHING; // уже никуда не движемся

      // закончили движение - сообщаем об этом менеджеру питания фрамуг
      PowerManager.WindowMoveDone(flags.Index);
    } 
    
      // говорим, что мы сменили позицию, модуль правил при этом очистит очередь обработанных правил, и сможет нами рулить
      SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);  
  
     // теперь смотрим, какой концевик сработал
     if(isCloseSwitchTriggered)
     {
      // концевик на закрытие
      CurrentPosition = 0;

      _waitFullClose = false;
      _fullCloseTimer = 0;
      
     }
     else
     if(isOpenSwitchTriggered)
     {
      // концевик на открытие
      CurrentPosition = interval;
     }
   
   return;  // поскольку сработали концевики - мы установили позицию по ним, и переданную можно игнорировать

  } // if(isCloseSwitchTriggered || isOpenSwitchTriggered)

  if(hasPosition && !IsBusy())
  {
    // есть информация о позиции, и это первая информация с обратной связи - мы должны запомнить, в какой позиции находится окно 
    uint32_t requestedPosition = (interval*positionPercents)/100;
    long currentDifference = 0;
    if(CurrentPosition > requestedPosition)
    {
      currentDifference = CurrentPosition - requestedPosition;
    }
    else
    {
      currentDifference = requestedPosition - CurrentPosition;
    }
      
    if(currentDifference > WINDOW_POSITION_HISTERESIS)
    {
      // разница позиций больше, чем гистерезис - обновляем позицию.
      // само окно, понятное дело, никуда не движется, но мы должны
      // исключить вариант, когда правила открывают окна на 50%,
      // а модуль обратной связи выдаёт позицию в 49% - в таком
      // случае надо исключить дёрганье моторов на короткие промежутки
      // и через равные интервалы времени, равные промежутку опроса
      // моделй обратной связи.
      CurrentPosition = requestedPosition;
      SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);
    }
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::UpdateState()
{

  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;

   WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
   uint8_t windowOnLevel = bnd.Level;


  // проверяем - есть ли задержка перед началом работы?
  if(powerTimerDelay > 0)
  {
    if(dt >= powerTimerDelay)
    {
      powerTimerDelay -= dt;
    }
    else
    {
      powerTimerDelay = 0;
    }

    // тут, если задержка по питанию вышла - мы должны включить реле СРАЗУ, потому что следующий вызов - 
    // уже отнимет от интервала дельту, в результате получим неточное время движения
    if(!powerTimerDelay)
    {
      switch(flags.Direction)
      {
          case dirOPEN:
          {
            SwitchRelays(windowOnLevel,!windowOnLevel);
          }
          break;

          case dirCLOSE:
          {
            SwitchRelays(!windowOnLevel,windowOnLevel);
          }
          break;

          case dirNOTHING:
          {
            SwitchRelays(!windowOnLevel,!windowOnLevel);
          }
          break;
      } // switch
      
    } // if(!powerTimerDelay)
    
  } // if(powerTimerDelay > 0)

  if(powerTimerDelay > 0) // ещё есть задержка перед началом движения, для устаканивания питания
  {
    return;
  }
    
   
  if(_waitFullClose)
  {
    #ifdef WINDOW_MANAGE_DEBUG
      DEBUG_LOGLN(F("WAIT FULL CLOSE..."));
    #endif
     
      if(_fullCloseTimer >= dt)
      {
        _fullCloseTimer -= dt;
      }
      else
      {
        _fullCloseTimer = 0;
      }
                
      if(_fullCloseTimer > 0)
      {
        _waitFullClose = true;
      }
      else
      {
        _waitFullClose = false;
      }

     if(!_waitFullClose) // время вышло, можно выключать реле
     {
      #ifdef WINDOW_MANAGE_DEBUG
        DEBUG_LOGLN(F("ADDITIONAL CLOSE TIME REACHED, STOP MOVE!"));
      #endif
      
       // приехали, останавливаемся
       flags.Direction = dirNOTHING; // уже никуда не движемся

       // сбрасываем флаг ожидания достижения позиции
       waitForChangePositionDone = false;
       
        //ВЫКЛЮЧАЕМ РЕЛЕ
        SwitchRelays(!windowOnLevel,!windowOnLevel);
        
        flags.OnMyWay = false;

        // говорим, что мы сменили позицию
        SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);

        // закончили движение - сообщаем об этом менеджеру питания фрамуг
        PowerManager.WindowMoveDone(flags.Index);       
     }
      return;
  } // if(_waitFullClose)
	  
  
  if(!flags.OnMyWay) // ничего не делаем
  {
    SwitchRelays(!windowOnLevel,!windowOnLevel); // держим реле выключенными
    return;
  }

   uint8_t bRelay1State, bRelay2State; // состояние выходов реле

   if(TimerInterval < dt)
    dt = TimerInterval;

   TimerInterval -= dt;

  // проверяем концевики
  #ifdef USE_WINDOWS_ENDSTOPS

    bool bAnyEndstopTriggered = false;
    
  #endif // USE_WINDOWS_ENDSTOPS   
       
   switch(flags.Direction)
   {
      case dirOPEN:
      {
        bRelay1State = windowOnLevel; // крутимся в одну сторону
        bRelay2State = !windowOnLevel;
        CurrentPosition += dt;

        #ifdef USE_WINDOWS_ENDSTOPS

            EndstopsBinding ebnd = HardwareBinding->GetEndstopsBinding();
            if(ebnd.LinkType != linkUnbinded)
            {
                if(ebnd.LinkType == linkDirect)
                {
                   if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(ebnd.EndstopOpenPins[flags.Index]))
                   {
                      bAnyEndstopTriggered = WORK_STATUS.PinRead(ebnd.EndstopOpenPins[flags.Index]) == ebnd.Level;
                   }
                }
                else
                if(ebnd.LinkType == linkMCP23S17)
                {
                  #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                    if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN)
                    {
                      bAnyEndstopTriggered = WORK_STATUS.MCP_SPI_PinRead(ebnd.MCPAddress1,ebnd.EndstopOpenPins[flags.Index]) == ebnd.Level;
                    }
                  #endif
                }
                else
                if(ebnd.LinkType == linkMCP23017)
                {
                  #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                    if(ebnd.EndstopOpenPins[flags.Index] != UNBINDED_PIN)
                    {
                      bAnyEndstopTriggered = WORK_STATUS.MCP_I2C_PinRead(ebnd.MCPAddress1,ebnd.EndstopOpenPins[flags.Index]) == ebnd.Level;
                    }
                  #endif
                }
              
            } // if(ebnd.LinkType != linkUnbinded)
        

            if(bAnyEndstopTriggered)
            {
              CurrentPosition = GetWorkTime(); // максимальное время открытия
              TimerInterval = 0;
            }
        #endif // USE_WINDOWS_ENDSTOPS
      } 
      break; // dirOPEN

      case dirCLOSE:
      {
        bRelay1State = !windowOnLevel; // или в другую
        bRelay2State = windowOnLevel;
        CurrentPosition -= dt;

        #ifdef USE_WINDOWS_ENDSTOPS

            EndstopsBinding ebnd = HardwareBinding->GetEndstopsBinding();
            if(ebnd.LinkType != linkUnbinded)
            {
                if(ebnd.LinkType == linkDirect)
                {
                   if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(ebnd.EndstopClosePins[flags.Index]))
                   {
                      bAnyEndstopTriggered = WORK_STATUS.PinRead(ebnd.EndstopClosePins[flags.Index]) == ebnd.Level;
                   }
                }
                else
                if(ebnd.LinkType == linkMCP23S17)
                {
                  #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                    if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN)
                    {
                      bAnyEndstopTriggered = WORK_STATUS.MCP_SPI_PinRead(ebnd.MCPAddress2,ebnd.EndstopClosePins[flags.Index]) == ebnd.Level;
                    }
                  #endif
                }
                else
                if(ebnd.LinkType == linkMCP23017)
                {
                  #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                    if(ebnd.EndstopClosePins[flags.Index] != UNBINDED_PIN)
                    {
                      bAnyEndstopTriggered = WORK_STATUS.MCP_I2C_PinRead(ebnd.MCPAddress2,ebnd.EndstopClosePins[flags.Index]) == ebnd.Level;
                    }
                  #endif
                }
              
            } // if(ebnd.LinkType != linkUnbinded)          

            if(bAnyEndstopTriggered)
            {
              CurrentPosition = 0;
              TimerInterval = 0;
            }

                    
        #endif // USE_WINDOWS_ENDSTOPS
        
      } 
      break; // dirCLOSE

      case dirNOTHING:
      default:
      {
        bRelay1State = !windowOnLevel; // накоротко, мотор не крутится
        bRelay2State = !windowOnLevel;
        TimerInterval = 0;
      } 
      break; // dirNOTHING
      
   } // switch


     if(!TimerInterval
#ifdef USE_WINDOWS_ENDSTOPS
        || bAnyEndstopTriggered
#endif     
     )
     {
      _waitFullClose =   (_fullCloseTimer > 0)
      #ifdef USE_WINDOWS_ENDSTOPS
        && (!bAnyEndstopTriggered)
      #endif
      ;
		
		    if(!_waitFullClose) // не надо ждать полного закрытия
		    {
            _fullCloseTimer = 0; 
    	 
    		   // приехали, останавливаемся
    		   flags.Direction = dirNOTHING; // уже никуда не движемся
    
    		   // сбрасываем флаг ожидания достижения позиции
    		   waitForChangePositionDone = false;
    		   
    			//ВЫКЛЮЧАЕМ РЕЛЕ
    			SwitchRelays(!windowOnLevel,!windowOnLevel);
    			
    			flags.OnMyWay = false;
    
    			// говорим, что мы сменили позицию
    			SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);
    
    			// закончили движение - сообщаем об этом менеджеру питания фрамуг
    			PowerManager.WindowMoveDone(flags.Index);        

          #ifdef WINDOW_MANAGE_DEBUG
    		   DEBUG_LOGLN(F("Position changed!"));
          #endif
    
    			return;
		    }
     }

    // продолжаем работу, включаем реле в нужное состояние
    SwitchRelays(bRelay1State,bRelay2State);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
TempSensors::TempSensors() : AbstractModule("STATE")
{   
  WindowModule = this;
  timer = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::WriteToMCP() // ПИШЕМ В MCP
{
  // сперва проверяем, были ли изменения
  bool hasChanges = false;
  for(uint8_t i=0;i<shiftRegisterDataSize;i++)
  {
    if(shiftRegisterData[i] != lastShiftRegisterData[i])
    {
      hasChanges = true;
      break;
    }
  } // for

  if(!hasChanges)
  {
    return;
  }

   if(shiftRegisterDataSize > 0)
   {        
          
        // работаем с MCP
        WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
        
        if(bnd.LinkType != linkUnbinded)
        {
          if(bnd.LinkType == linkMCP23S17 || bnd.LinkType == linkMCP23017)
          {
          
            int iter = 0;
            int addr = bnd.MCPAddress1;
            
            for(int i=0;i<shiftRegisterDataSize;i++)
            {
                uint8_t bVal = shiftRegisterData[i];
                
                for(uint8_t kk=0;kk<8;kk++)
                {
                    if(bnd.WindowsPins[iter] != UNBINDED_PIN)
                    {

                    #ifdef WINDOW_MANAGE_DEBUG
                      DEBUG_LOG(F("Write to MCP #"));
                      DEBUG_LOG(String(addr));
                      DEBUG_LOG(F(", set channel #"));
                      DEBUG_LOG(String(bnd.WindowsPins[iter]));
                      DEBUG_LOG(F(" to LEVEL="));
                      DEBUG_LOGLN(String(bitRead(bVal,kk)));
                    #endif
                      
                        if(bnd.LinkType == linkMCP23S17)
                        {
                          #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                            //WORK_STATUS.MCP_SPI_PinWrite(addr, bnd.WindowsPins[iter] , bVal & (1 << kk) );
                            WORK_STATUS.MCP_SPI_PinWrite(addr, bnd.WindowsPins[iter], bitRead(bVal,kk) );
                          #endif
                        }
                        else
                        if(bnd.LinkType == linkMCP23017)
                        {
                          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                             //WORK_STATUS.MCP_I2C_PinWrite(addr, bnd.WindowsPins[iter] , bVal & (1 << kk) );
                             WORK_STATUS.MCP_I2C_PinWrite(addr, bnd.WindowsPins[iter], bitRead(bVal,kk) );
                          #endif
                        }
                    } // if(bnd.WindowsPins[iter] != UNBINDED_PIN)
                    
                    iter++;
                    
                    if(iter > 15) // кончилась первая микросхема
                    {
                      addr = bnd.MCPAddress2;
                    }
                } // for(uint8_t kk=0;kk<8;kk++)
              
            } // for 
          } // if(bnd.LinkType == linkMCP23S17 || bnd.LinkType == linkMCP23017)
          else
          if(bnd.LinkType == link74HC595)
          {
              if(bnd.ShiftOEPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.ShiftOEPin)
              && bnd.ShiftLatchPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.ShiftLatchPin)
              && bnd.ShiftDataPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.ShiftDataPin)
              && bnd.ShiftClockPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.ShiftClockPin))
              {
                
                      // сначала разрешаем установить состояние на выходах
                      WORK_STATUS.PinWrite(bnd.ShiftOEPin,LOW);
                      
                      // Отключаем вывод на регистре
                      WORK_STATUS.PinWrite(bnd.ShiftLatchPin, LOW);
                  
                      // проталкиваем все байты один за другим, начиная со старшего к младшему
                        uint8_t i=shiftRegisterDataSize;
                      
                        do
                        {    
                          // проталкиваем байт в регистр
                            shiftOut(bnd.ShiftDataPin, bnd.ShiftClockPin, MSBFIRST, shiftRegisterData[--i]);
                        } while(i > 0);
                  
                        // "защелкиваем" регистр, чтобы байт появился на его выходах
                        WORK_STATUS.PinWrite(bnd.ShiftLatchPin, HIGH);
              }
          } // if(bnd.LinkType == link74HC595)
          
        } // if(bnd.LinkType != linkUnbinded)
           
      
    
   } // if(shiftRegisterDataSize > 0)
  

  // теперь сохраняем последнее запомненное состояние
   for(uint8_t i=0;i<shiftRegisterDataSize;i++)
   {
    lastShiftRegisterData[i] = shiftRegisterData[i];
   }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::SaveChannelState(uint8_t channel, uint8_t state)
{
  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();

  if(bnd.LinkType == linkMCP23S17 || bnd.LinkType == linkMCP23017 || bnd.LinkType == link74HC595)
  {
    //Сохраняем состояние каналов для MCP или 74HC595

     uint8_t idx = channel/8; // выясняем, какой индекс в массиве байт
   // теперь мы должны выяснить, в какой бит писать
    uint8_t bitNum = channel % 8;

    // пишем в нужный байт и в нужный бит нужное состояние
    uint8_t bt = shiftRegisterData[idx];
    bitWrite(bt,bitNum, state);
    shiftRegisterData[idx] = bt;
         
  }
  else
  if(bnd.LinkType == linkDirect) // прямое управление пинами
  {
      if(bnd.WindowsPins[channel] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.WindowsPins[channel]))
      {
        // просто управляем пинами, поэтому напрямую пишем в пины
        WORK_STATUS.PinWrite(bnd.WindowsPins[channel],state);
      }
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TempSensors::IsAnyWindowOpen()
{
  for(uint8_t i=0;i<SUPPORTED_WINDOWS;i++)
  {
    if(IsWindowOpen(i))
    {
      return true;
    }
  }
  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TempSensors::IsWindowOpen(uint8_t windowNumber)
{
  if(windowNumber >= SUPPORTED_WINDOWS)
  {
    return false;
  }

  WindowState* ws = &(Windows[windowNumber]);
  
  if(ws->IsBusy()) // окно в движении
  {  
    if(ws->GetDirection() == dirOPEN) // окно открывается
      return true;    
  }
  else // окно никуда не двигается
  {
     if(ws->GetCurrentPosition() > 0) // окно открыто
      return true;
  } 

  return false; // окно закрывается или закрыто
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::SetupWindows()
{
  // настраиваем фрамуги  
  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();

  #ifdef WINDOW_MANAGE_DEBUG
    DEBUG_LOGLN(F("SETUP WINDOWS HARDWARE BEGIN..."));
  #endif

  for(uint8_t i=0, j=0;i<SUPPORTED_WINDOWS;i++, j+=2)
  {
    
      // раздаём каналы реле: первому окну - 0,1, второму - 2,3 и т.д.
      Windows[i].Setup(i, j, j+1);

      // настраиваем выхода      
      if(bnd.LinkType != linkUnbinded)
      {
          if(bnd.LinkType == linkMCP23S17 || bnd.LinkType == linkMCP23017)
          {

            uint8_t pin1 = bnd.WindowsPins[j];
            uint8_t pin2 = bnd.WindowsPins[j+1];
            int addr = j > 15 ? bnd.MCPAddress2 : bnd.MCPAddress1;
              
          #ifdef WINDOW_MANAGE_DEBUG
            DEBUG_LOG(F("Window #"));
            DEBUG_LOG(String(i));
            DEBUG_LOG(F(" has channels ## "));
            DEBUG_LOG(String(j));
            DEBUG_LOG(F(","));
            DEBUG_LOG(String(j+1));
            DEBUG_LOG(F(" and MCP outputs ## "));
            DEBUG_LOG(String(pin1));
            DEBUG_LOG(F(","));
            DEBUG_LOGLN(String(pin2));

            DEBUG_LOG(F("Current MCP address are: "));
            DEBUG_LOGLN(String(addr));
          #endif
            


              if(bnd.LinkType == linkMCP23S17) // MCP23S17
              {
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                  if(pin1 != UNBINDED_PIN)
                  {
                    #ifdef WINDOW_MANAGE_DEBUG
                      DEBUG_LOG(F("Setup MCP #"));
                      DEBUG_LOG(String(addr));
                      DEBUG_LOG(F(", set channel #"));
                      DEBUG_LOG(String(pin1));
                      DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                      DEBUG_LOGLN(String(!bnd.Level));
                    #endif
                    
                    WORK_STATUS.MCP_SPI_PinMode(addr, pin1, OUTPUT);
                    WORK_STATUS.MCP_SPI_PinWrite(addr, pin1, !bnd.Level);
                  }
                  
                  if(pin2 != UNBINDED_PIN)
                  {
                    #ifdef WINDOW_MANAGE_DEBUG
                      DEBUG_LOG(F("Setup MCP #"));
                      DEBUG_LOG(String(addr));
                      DEBUG_LOG(F(", set channel #"));
                      DEBUG_LOG(String(pin1));
                      DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                      DEBUG_LOGLN(String(!bnd.Level));
                    #endif
                    
                    WORK_STATUS.MCP_SPI_PinMode(addr, pin2, OUTPUT);
                    WORK_STATUS.MCP_SPI_PinWrite(addr, pin2, !bnd.Level);
                  }
                #endif 
              }
              else // MCP23017
              {
                  #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                    if(pin1 != UNBINDED_PIN)
                    {

                    #ifdef WINDOW_MANAGE_DEBUG
                      DEBUG_LOG(F("Setup MCP #"));
                      DEBUG_LOG(String(addr));
                      DEBUG_LOG(F(", set channel #"));
                      DEBUG_LOG(String(pin1));
                      DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                      DEBUG_LOGLN(String(!bnd.Level));
                    #endif

                      
                      WORK_STATUS.MCP_I2C_PinMode(addr, pin1, OUTPUT);
                      WORK_STATUS.MCP_I2C_PinWrite(addr, pin1, !bnd.Level);
                    }
                    
                    if(pin2 != UNBINDED_PIN)
                    {
                      
                    #ifdef WINDOW_MANAGE_DEBUG
                      DEBUG_LOG(F("Setup MCP #"));
                      DEBUG_LOG(String(addr));
                      DEBUG_LOG(F(", set channel #"));
                      DEBUG_LOG(String(pin1));
                      DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                      DEBUG_LOGLN(String(!bnd.Level));
                    #endif
                      
                      WORK_STATUS.MCP_I2C_PinMode(addr, pin2, OUTPUT);
                      WORK_STATUS.MCP_I2C_PinWrite(addr, pin2, !bnd.Level);
                    }
                  #endif
              }
          }
          else
          if(bnd.LinkType == linkDirect)
          {
             // просто настраиваем пины
              uint8_t pin1 = bnd.WindowsPins[j];
              uint8_t pin2 = bnd.WindowsPins[j+1];

              #ifdef WINDOW_MANAGE_DEBUG
                DEBUG_LOG(F("Setup PIN #"));
                DEBUG_LOG(String(pin1));
                DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                DEBUG_LOGLN(String(!bnd.Level));

                DEBUG_LOG(F("Setup PIN #"));
                DEBUG_LOG(String(pin2));
                DEBUG_LOG(F(" to OUTPUT with LEVEL="));
                DEBUG_LOGLN(String(!bnd.Level));                      
              #endif
              

              if(pin1 != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin1))
              {
                  WORK_STATUS.PinMode(pin1,OUTPUT);
                  WORK_STATUS.PinWrite(pin1,!bnd.Level);
              }
              
              if(pin2 != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin2))
              {
                  WORK_STATUS.PinMode(pin2,OUTPUT);
                  WORK_STATUS.PinWrite(pin2,!bnd.Level);
              }
     
          } // linkDirect
        
      } // if(bnd.LinkType != linkUnbinded)


    #ifdef USE_FEEDBACK_MANAGER
      // используем менеджер обратной связи, позиция должна придти
    #else

    // не используем мереджер обратной связи
    // просим окна закрыться при старте контроллера
    
      ////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
      if(bnd.ManageMode == 1) // попеременно
      {
        addToSeparateList(&(Windows[i]),0, true,true); // добавляем в список попеременного закрытия
      }
      else // одновременно
      {
         Windows[i].ChangePosition(0,true); // сразу закрываем окно при старте
      }
      ////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////


    #endif
    
  } // for

////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
   if(bnd.ManageMode == 1) // попеременно
   {
      if(separateManagerList.size())
      {
        if(separateManagerList[0].resetToMaxPositionBeforeStart)
        {
          separateManagerList[0].window->ResetToMaxPosition();
        }
        separateManagerList[0].window->ChangePosition(separateManagerList[0].targetPosition,separateManagerList[0].waitFor);
      }
   } // (bnd.ManageMode == 1) // попеременно

////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////  

  #ifdef WINDOW_MANAGE_DEBUG
    DEBUG_LOGLN(F("SETUP WINDOWS HARDWARE DONE."));
  #endif

}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::CloseWindow(uint8_t num)
{
  if(num >= SUPPORTED_WINDOWS)
  {
    return;
  }

  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();

  if(bnd.ManageMode == 1) // попеременно
  {
       bool shouldRunFirstWindow = !separateManagerList.size();
            
       addToSeparateList(&(Windows[num]),0,true,true); // закрываем окно
    
       if(shouldRunFirstWindow)
       {
        separateManagerList[0].window->ResetToMaxPosition();
        separateManagerList[0].window->ChangePosition(0,true); // закрываем окно
       }
  }
  else // одновременно
  {
     Windows[num].ResetToMaxPosition();
     Windows[num].ChangePosition(0,true); // закрываем окно  
  }
   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::CloseAllWindows()
{
////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();

  if(bnd.ManageMode == 1) // попеременно
  {
      bool shouldRunFirstWindow = !separateManagerList.size();
      for(int i=0;i<SUPPORTED_WINDOWS;i++)
      {
         addToSeparateList(&(Windows[i]),0,true,true); // закрываем окно
      }
    
      if(shouldRunFirstWindow)
      {
        separateManagerList[0].window->ResetToMaxPosition();
        separateManagerList[0].window->ChangePosition(0,true); // закрываем окно
      }    
  }
  else // одновременно
  {
      for(int i=0;i<SUPPORTED_WINDOWS;i++)
      {
         Windows[i].ResetToMaxPosition();
         Windows[i].ChangePosition(0,true); // закрываем окно
      }
  }

////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////  

}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::Setup()
{
  
   workMode = wmAutomatic; // автоматический режим работы по умолчанию

  canUseBlinker = false;
  DiodesBinding dbnd = HardwareBinding->GetDiodesBinding();
  if(dbnd.LinkType != linkUnbinded)
  {
    if(dbnd.LinkType == linkDirect)
    {
        if(dbnd.WindowsManualModeDiodePin != UNBINDED_PIN && dbnd.WindowsManualModeDiodePin > 1
        #ifndef DISABLE_DIODES_CONFIGURE
        && EEPROMSettingsModule::SafePin(dbnd.WindowsManualModeDiodePin)
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
         if(dbnd.WindowsManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }
    else
    if(dbnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
         if(dbnd.WindowsManualModeDiodePin != UNBINDED_PIN)
         {
            canUseBlinker = true;
         }
      #endif
    }    
    
  } // if(bnd.LinkType != linkUnbinded)

  if(canUseBlinker)
  {
     blinker.begin(dbnd.WindowsManualModeDiodePin); // настраиваем блинкер на нужный пин
  }

  lastUpdateCall = 0;
  smallSensorsChange = 0;
  
   // добавляем датчики температуры

   DS18B20LineManager.begin(DS18B20_BINDING_ADDRESS);
   DS18B20LineManager.beginConversion();
   DS18B20LineManager.beginSetResolution();

   DallasBinding bnd = HardwareBinding->GetDallasBinding();

   uint8_t sensorCounter = 0;
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // непривязанный пин
      {
        continue;
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }

      // добавляем состояние
      State.AddState(StateTemperature,sensorCounter);
      
       // добавляем привязку
      DS18B20LineManager.addBinding(pin,sensorCounter);
  
      WORK_STATUS.PinMode(pin,INPUT,false);
      
      DS18B20LineManager.setResolution(pin,temp12bit);
  
      // запускаем конвертацию с датчиков при старте, через 2 секунды нам вернётся измеренная температура
      DS18B20LineManager.startConversion(pin);

      sensorCounter++;
   } // for
   

   SetupWindows(); // настраиваем фрамуги

   WindowsBinding wbnd = HardwareBinding->GetWindowsBinding();

   if(wbnd.LinkType == linkMCP23S17 || wbnd.LinkType == linkMCP23017 || wbnd.LinkType == link74HC595)
   {

        if(wbnd.LinkType == link74HC595)
        {
              // настраиваем пины для сдвигового регистра на выход
              if(wbnd.ShiftLatchPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(wbnd.ShiftLatchPin))
              {
                WORK_STATUS.PinMode(wbnd.ShiftLatchPin,OUTPUT);
                WORK_STATUS.PinWrite(wbnd.ShiftLatchPin, LOW);
              }

              if(wbnd.ShiftDataPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(wbnd.ShiftDataPin))
              {
                WORK_STATUS.PinMode(wbnd.ShiftDataPin,OUTPUT);
                WORK_STATUS.PinWrite(wbnd.ShiftDataPin, LOW);
              }

              if(wbnd.ShiftClockPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(wbnd.ShiftClockPin))
              {
                WORK_STATUS.PinMode(wbnd.ShiftClockPin,OUTPUT);
                WORK_STATUS.PinWrite(wbnd.ShiftClockPin, LOW);
              }
          
              // переводим все выводы в High-Z состояние (они и так уже в нём, 
              // поскольку пин, управляющий OE, подтянут к питанию,
              // но мы не будем мелочиться :) ).
              if(wbnd.ShiftOEPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(wbnd.ShiftOEPin))
              {
                WORK_STATUS.PinMode(wbnd.ShiftOEPin,OUTPUT);
                WORK_STATUS.PinWrite(wbnd.ShiftOEPin,HIGH);
              }
              
        } // if(wbnd.LinkType == link74HC595)
    

       // настраиваем кол-во байт, в котором мы будем держать состояние каналов для сдвигового регистра.
        // у нас для каждого окна - два канала, соответственно, общее кол-во бит - это
        // SUPPORTED_WINDOWS*2. Исходя из этого - легко посчитать кол-во байт, необходимых
        // для хранения данных.
        shiftRegisterDataSize =  (SUPPORTED_WINDOWS*2)/8;
        if((SUPPORTED_WINDOWS*2) > 8 && (SUPPORTED_WINDOWS*2) % 8)
        {
          shiftRegisterDataSize++;
        }
    
        shiftRegisterData = new uint8_t[shiftRegisterDataSize];
        lastShiftRegisterData = new uint8_t[shiftRegisterDataSize];
        // теперь в каждый бит этих байт записываем значение !wbnd.Level для shiftRegisterData,
        // и значение wbnd.Level для lastShiftRegisterData.
        // надо именно побитово, т.к. значение !wbnd.Level может быть 1, и в этом случае
        // все биты должны быть установлены в 1.
    
          uint8_t bOff = 0;
          uint8_t bOn = 0;
          for(uint8_t j=0;j<8;j++)
          {
            bOff |= ((!wbnd.Level) << j);
            bOn |= (wbnd.Level << j);
          }
    
        for(uint8_t i=0;i<shiftRegisterDataSize;i++)
        {      
          // сохранили разные значения первоначально, поскольку мы хотим записать их впервые
          shiftRegisterData[i] = bOff;
          lastShiftRegisterData[i] = bOn;
          
        } // for
          
        WriteToMCP(); // пишем первоначальное состояние реле в MCP
        
   } // if(wbnd.LinkType == linkMCP23S17 || wbnd.LinkType == linkMCP23017)


   SAVE_STATUS(WINDOWS_MODE_BIT,1); // сохраняем режим работы окон
   SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,0); // говорим, что окна инициализируются
 

 }
//--------------------------------------------------------------------------------------------------------------------------------------
////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::updateSeparateManager()
{
  if(!separateManagerList.size())
  {
    return;
  }

  if(!separateManagerList[0].window->IsBusy())
  {
    // удаляем первое окно из очереди
    for(size_t i=1;i<separateManagerList.size();i++)
    {
      separateManagerList[i-1] = separateManagerList[i];
    }
    separateManagerList.pop();

    if(separateManagerList.size())
    {
      // ещё есть окна, запускаем первое
      if(separateManagerList[0].resetToMaxPositionBeforeStart)
      {
        separateManagerList[0].window->ResetToMaxPosition();
      }
      separateManagerList[0].window->ChangePosition(separateManagerList[0].targetPosition,separateManagerList[0].waitFor);
    }
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::addToSeparateList(WindowState* window,unsigned long targetPosition, bool waitFor,bool resetToMaxPositionBeforeStart)
{
  for(size_t i=0;i<separateManagerList.size();i++)
  {
    if(separateManagerList[i].window == window)
    {
      separateManagerList[i].targetPosition = targetPosition;
      separateManagerList[i].waitFor = waitFor;
      separateManagerList[i].resetToMaxPositionBeforeStart = resetToMaxPositionBeforeStart;
      return;
    }
  }

  SeparateManageStruct sm;
  sm.window = window;
  sm.targetPosition = targetPosition;
  sm.waitFor = waitFor;
  sm.resetToMaxPositionBeforeStart = resetToMaxPositionBeforeStart;
  separateManagerList.push_back(sm);
}
//--------------------------------------------------------------------------------------------------------------------------------------
////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::Update()
{ 
  
  uint32_t now = millis();
  uint32_t dt = now - timer;
  timer = now;
  
////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

  WindowsBinding wbnd = HardwareBinding->GetWindowsBinding();
  if(wbnd.ManageMode == 1) // попеременно
  {
    updateSeparateManager();
  }

////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

  if(canUseBlinker)
  {
    blinker.update();
  }
  
  for(uint8_t i=0;i<SUPPORTED_WINDOWS;i++) // обновляем каналы управления фрамугами
  {
      Windows[i].UpdateState();
  } // for 

 
 if(wbnd.LinkType == linkMCP23S17 || wbnd.LinkType == linkMCP23017 ||  wbnd.LinkType == link74HC595)
 {
    // пишем в MCP, если есть изменения
    WriteToMCP();  
 }

  lastUpdateCall += dt;
  if(lastUpdateCall < TEMP_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
  {
    return;
  }
  else
  {
    lastUpdateCall = 0;
  }

  // опрашиваем наши датчики
  DallasBinding bnd = HardwareBinding->GetDallasBinding();

  // запускаем конвертацию датчиков, игнорируя повторные вызовы для одной линии
   DS18B20LineManager.beginConversion();

   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      DS18B20LineManager.startConversion(pin);
   } // for

   // теперь сканируем линии
   DS18B20LineManager.beginScan();
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      DS18B20LineManager.scan(pin);
   } // for

   // теперь - читаем датчики
   uint8_t sensorCounter = 0;
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      DS18B20Temperature tempData;
      Temperature t;
        
        if(DS18B20LineManager.getTemperature(sensorCounter,pin, tempData))
        {
          
          t.Value = tempData.Whole;
        
          if(tempData.Negative)
            t.Value = -t.Value;
    
          t.Fract = tempData.Fract + smallSensorsChange;
    
          // convert to Fahrenheit if needed
          #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
           t = Temperature::ConvertToFahrenheit(t);
          #endif      
          
        }      

      State.UpdateState(StateTemperature,sensorCounter,(void*)&t); // обновляем состояние температуры, индексы датчиков у нас идут без дырок, поэтому с итератором цикла вызывать можно
      sensorCounter++;
   } // for


  smallSensorsChange = 0;


}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::WindowFeedback(uint8_t windowNumber, bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents, bool isFirstFeedback)
{
  #if SUPPORTED_WINDOWS > 0
    if(windowNumber >= SUPPORTED_WINDOWS)
    {
      windowNumber = SUPPORTED_WINDOWS-1;
    }

      Windows[windowNumber].Feedback(isCloseSwitchTriggered,isOpenSwitchTriggered,hasPosition,positionPercents,isFirstFeedback);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TempSensors::CanDriveWindows()
{
  // тут проверяем, можем ли мы выполнить команду на смену позиции.
  // если используется менеджер обратной связи - мы не можем ничего делать,
  // пока менеджер ждёт первого пакета обратной связи
  #ifdef USE_FEEDBACK_MANAGER
    if(FeedbackManager.IsWaitingForFirstWindowsFeedback())
    {
      // ничего не делаем, поскольку всё ещё ждём информации по положению окон
      return false;
    }
  #endif // USE_FEEDBACK_MANAGER

  // если хотя бы одно из окон находится в положении непрерываемого движения - считаем, что мы не можем рулить окнами
  // при помощи внешних команд

  for(int i=0;i<SUPPORTED_WINDOWS;i++)
  {
     if(Windows[i].IsInUninterruptedWay()) // одно из окон находится в непрерываемом движении (мы используем это ТОЛЬКО для закрытия при старте)
     {
      return false; // поэтому пока они не закроются - управлять окнами при помощи внешних команд нельзя
     }
  }  

  WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
  if(bnd.ManageMode == 1) // попеременно
  {
      for(size_t i=0;i<separateManagerList.size();i++)
      {
          if(separateManagerList[i].waitFor) // окно ждёт движения, и ждёт движения без прерываний
          {
            return false;
        }
      }
  } // if


  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  TempSensors::ExecCommand(const Command& command, bool wantAnswer)
{
  GlobalSettings* sett = MainController->GetSettings();
  if(wantAnswer) 
    PublishSingleton = PARAMS_MISSED;
      
  String commandRequested;

  if(command.GetType() == ctSET) // напрямую запись в датчики запрещена, пишем только в состояние каналов
  {
    uint8_t argsCnt = command.GetArgsCount();
    
    if(argsCnt > 2)
    {
      commandRequested = command.GetArg(0);
      commandRequested.toUpperCase();
      if(commandRequested == PROP_WINDOW) // надо записать состояние окна, от нас просят что-то сделать
      {

        if(!CanDriveWindows())
        {
          // пока не можем управлять окнами, ждём их полного закрытия
          // отвечаем на команду
          MainController->Publish(this,command);        
          return PublishSingleton.Flags.Status;
        }
        
        
        if(command.IsInternal() // если команда пришла от другого модуля
        && workMode == wmManual) // и мы в ручном режиме, то
        {
          // просто игнорируем команду, потому что нами управляют в ручном режиме
          // мигаем светодиодом на 6 пине
          
         }
        else
        {
          if(!command.IsInternal()) // пришла команда от пользователя,
          {
            workMode = wmManual; // переходим на ручной режим работы

            if(canUseBlinker)
            {
              blinker.blink(WORK_MODE_BLINK_INTERVAL);
            }
            
          }

          String token = command.GetArg(1);
          token.toUpperCase();

          String whichCommand = command.GetArg(2); // какую команду запросили?
          whichCommand.toUpperCase();
          
          bool bOpen = (whichCommand == STATE_OPEN); // запросили открытие фрамуг?          
          bool bAll = (token == ALL); // на все окна распространяется запрос?
          bool bIntervalAsked = token.indexOf("-") != -1; // запросили интервал каналов?
          uint8_t channelIdx = token.toInt(); // номер канала окна
          
         // uint32_t motorsFullWorkTime = sett->GetOpenInterval();
          //uint32_t targetPosition = bOpen ? motorsFullWorkTime : 0; // если не запрошено интервала - будем использовать настройки прощивки, и открываем/закрываем полностью
          
          uint32_t passedPosition = 0; // какую позицию передали в команде? Это может быть процент от открытия, или - интервал для работы.
          bool bPercentsRequested = false; // флаг, что запросили процент открытия
          bool hasIntervalPassed = false; // передали ли нам доп. параметр с интервалом или процентовкой?
                    
          if(command.GetArgsCount() > 3) // запрошен интервал или проценты на позицию
          {
            String strIntervalPassed = command.GetArg(3);
            bPercentsRequested = strIntervalPassed.endsWith("%");
            hasIntervalPassed = true;
            
             if(bPercentsRequested)
             {
                strIntervalPassed.remove(strIntervalPassed.length()-1);
             }

             passedPosition = (uint32_t) atol(strIntervalPassed.c_str()); // получили интервал для работы реле
             
             if(bPercentsRequested && passedPosition > 100) // если переданы проценты - убеждаемся, что их не больше 100
             {
                passedPosition = 100;
             }
            
             
          } // if(command.GetArgsCount() > 3)

          /*                        
          if(command.GetArgsCount() > 3) // запрошен интервал или проценты на позицию
          {
            String strIntervalPassed = command.GetArg(3);
            bool bPercentsRequested = strIntervalPassed.endsWith("%");
            
            if(bPercentsRequested)
              strIntervalPassed.remove(strIntervalPassed.length()-1);
              
            targetPosition = (uint32_t) atol(strIntervalPassed.c_str()); // получили интервал для работы реле

            if(bPercentsRequested)
            {
              // конвертируем запрошенные проценты в актуальный интервал
              targetPosition = (motorsFullWorkTime*targetPosition)/100;              
            }
            else // запросили обычный интервал
            {
              // тут надо проверить - не выходим ли за границы диапазона работы приводов?
              if(targetPosition > motorsFullWorkTime)
              {
                targetPosition = motorsFullWorkTime;
              }
            }
          } // if(command.GetArgsCount() > 3)
          */

 
          PublishSingleton.Flags.Status = true;

          // откуда до куда шаримся
          uint8_t from = 0;
          uint8_t to = SUPPORTED_WINDOWS;

          if(bIntervalAsked)
          {
             // парсим интервал окон, с которыми надо работать
             int delim = token.indexOf("-");
             from = token.substring(0,delim).toInt();
             to = token.substring(delim+1,token.length()).toInt();
             
          }
          else if(!bAll) // если не интервал окон и не все окна - значит, одно окно
          {            
            from = channelIdx;
            to = from;
          }

          // правильно расставляем шаги - от меньшего к большему
          uint8_t tmp = min(from,to);
          to = max(from,to);
          from = tmp;

          to++; // включаем to в интервал, это надо, если пришла команда интервала, например, 2-3, тогда в этом случае опросятся третий и четвертый каналы
           
           if(to >= SUPPORTED_WINDOWS)
           {
              to = SUPPORTED_WINDOWS;
           }


          ////// НАЧАЛО ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

          WindowsBinding bnd = HardwareBinding->GetWindowsBinding();
          WindowsIntervals intervals = HardwareBinding->GetWindowsIntervals();  
          
          if(bnd.ManageMode == 1) // попеременно
          {
              bool shouldChangeFirstWindowPosition = !separateManagerList.size(); // меняем позицию первого окна сразу, если в очереди не было изменений
              for(uint8_t i=from;i<to;i++)
              {
                // выясняем, какой интервал открытия будем применять к окну?
                uint32_t workInterval = Windows[i].GetWorkTime();

                // в passedPosition у нас лежит либо 0, либо - переданный интервал (bPercentsRequested == false), либо - процентовка (bPercentsRequested == true)
                // если доп. параметр не передан - то мы должны сами выяснять, что делать.
                 uint32_t targetPosition;
                 
                if(hasIntervalPassed) // передали доп. параметр
                {
                  // имеем переданный в доп. параметре интервал или процентовку
                  if(bPercentsRequested) // запросили процентовку для этого окна, от времени работы его приводов
                  {
                    // конвертируем запрошенные проценты в актуальный интервал
                    targetPosition = (workInterval*passedPosition)/100;    
                  }
                  else
                  {
                    // передали просто интервал для работы
                    targetPosition = passedPosition;
                  }                  
                }
                else
                {
                   // нет доп. параметра, надо просто открыть или закрыть окно
                   targetPosition = bOpen ? workInterval : 0;
                }
                
                // проверяем, чтобы окно не работало слишком долго
                if(targetPosition > workInterval)
                {
                  targetPosition = workInterval;
                }
                
                // просим окно сменить позицию, последовательно, одно за другим
                addToSeparateList(&(Windows[i]),targetPosition,false,false);
              } // for
    
              if(shouldChangeFirstWindowPosition && separateManagerList.size())
              {
                // тут опять надо выяснить позицию для окна, которое находится первым в списке
                uint32_t workInterval = separateManagerList[0].window->GetWorkTime();                              
                uint32_t targetPosition;

                if(hasIntervalPassed) // передали доп. параметр
                {
                  // имеем переданный в доп. параметре интервал или процентовку
                  if(bPercentsRequested) // запросили процентовку для этого окна, от времени работы его приводов
                  {
                    // конвертируем запрошенные проценты в актуальный интервал
                    targetPosition = (workInterval*passedPosition)/100;    
                  }
                  else
                  {
                    // передали просто интервал для работы
                    targetPosition = passedPosition;
                  }                  
                }
                else
                {
                   // нет доп. параметра, надо просто открыть или закрыть окно
                   targetPosition = bOpen ? workInterval : 0;
                }                

                // проверяем, чтобы окно не работало слишком долго
                if(targetPosition > workInterval)
                {
                  targetPosition = workInterval;
                }
                
                separateManagerList[0].window->ChangePosition(targetPosition);
              }
          } // if(bnd.ManageMode == 1) // попеременно
          else // одновременно
          {
              // простое управление окнами
              for(uint8_t i=from;i<to;i++)
              {

                // выясняем, какой интервал открытия будем применять к окну?
                uint32_t workInterval = Windows[i].GetWorkTime();
                uint32_t targetPosition;

                if(hasIntervalPassed) // передали доп. параметр
                {
                  // имеем переданный в доп. параметре интервал или процентовку
                  if(bPercentsRequested) // запросили процентовку для этого окна, от времени работы его приводов
                  {
                    // конвертируем запрошенные проценты в актуальный интервал
                    targetPosition = (workInterval*passedPosition)/100;    
                  }
                  else
                  {
                    // передали просто интервал для работы
                    targetPosition = passedPosition;
                  }                  
                }
                else
                {
                   // нет доп. параметра, надо просто открыть или закрыть окно
                   targetPosition = bOpen ? workInterval : 0;
                }                


                // проверяем, чтобы окно не работало слишком долго
                if(targetPosition > workInterval)
                {
                  targetPosition = workInterval;
                }      

                // просим окно сменить позицию
                Windows[i].ChangePosition(targetPosition);
              } // for
            
          } // else одновременно

          ////// КОНЕЦ ИЗМЕНЕНИЙ ПО РАЗДЕЛЬНОМУ УПРАВЛЕНИЮ ОКНАМИ //////

          // если запрошенный или рассчитанный интервал больше нуля - окна открыты, иначе - закрыты
          SAVE_STATUS(WINDOWS_STATUS_BIT,IsAnyWindowOpen() ? 1 : 0); // сохраняем состояние окон
          SAVE_STATUS(WINDOWS_MODE_BIT,workMode == wmAutomatic ? 1 : 0); // сохраняем режим работы окон

          // какую команду запросили, такую и возвращаем, всё равно в результате выполнения
          // все запрошенные окна встанут в одну позицию
          PublishSingleton = token;
          PublishSingleton << PARAM_DELIMITER << (bOpen ? STATE_OPENING : STATE_CLOSING);
                

        } // else command from user
        
      } // if PROP_WINDOW
      else
      if(commandRequested == TEMP_SETTINGS) // установить температуры закрытия/открытия
      {
        uint8_t tOpen = (uint8_t) atoi(command.GetArg(1));
        uint8_t tClose = (uint8_t) atoi(command.GetArg(2));

        sett->SetOpenTemp(tOpen);
        sett->SetCloseTemp(tClose);

        
        PublishSingleton.Flags.Status = true;
        if(wantAnswer) 
        {
          PublishSingleton = commandRequested;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
      } // TEMP_SETTINGS
      else
      if(commandRequested == PROP_TEMP) // установить значение датчика, CTSET=STATE|TEMP|index|value
      {
        uint8_t sensorIndex = (uint8_t) atoi(command.GetArg(1));
        int16_t sensorValue =  (int16_t) atoi(command.GetArg(2));

        uint8_t _tempCnt = State.GetStateCount(StateTemperature);
        
        if(sensorIndex >= _tempCnt)
        {
          uint8_t toAdd = (sensorIndex - _tempCnt) + 1;

            for(uint8_t qa = 0; qa < toAdd; qa++)
            {
              State.AddState(StateTemperature,_tempCnt + qa);
            }
        }

          Temperature t;
          t.Value = sensorValue/100;
    
          t.Fract = abs(sensorValue%100);
    
          // convert to Fahrenheit if needed
          #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
           t = Temperature::ConvertToFahrenheit(t);
          #endif      
             
          State.UpdateState(StateTemperature,sensorIndex,(void*)&t);

          ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
          #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
          OneState* dynReset = State.GetState(StateTemperature,sensorIndex);
          if(dynReset)
          {
            DynamicSensorsReset->Observe(dynReset);
          }
          #endif
          ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
        

        PublishSingleton.Flags.Status = true;
        PublishSingleton = commandRequested;
        PublishSingleton << PARAM_DELIMITER << sensorIndex << PARAM_DELIMITER << REG_SUCC;
        
      } // PROP_TEMP
      
    } // if(argsCnt > 2)
    else if(argsCnt > 1)
    {
      commandRequested = command.GetArg(0);
      commandRequested.toUpperCase();

      if (commandRequested == WORK_MODE)
      {
        // запросили установить режим работы
        commandRequested = command.GetArg(1);
        commandRequested.toUpperCase();


        if(commandRequested == WM_AUTOMATIC)
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = WORK_MODE;
            PublishSingleton << PARAM_DELIMITER << commandRequested;
          }
          workMode = wmAutomatic;
          smallSensorsChange = 1;

          if(canUseBlinker)
          {
            blinker.blink(); // выключаем светодиод
          }

        }
        else if(commandRequested == WM_MANUAL)
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = WORK_MODE;
            PublishSingleton << PARAM_DELIMITER << commandRequested;
          }
          workMode = wmManual;
          smallSensorsChange = 1;

              if(canUseBlinker)
              {
                blinker.blink(WORK_MODE_BLINK_INTERVAL);
              }

        }
        
        SAVE_STATUS(WINDOWS_MODE_BIT,workMode == wmAutomatic ? 1 : 0); // сохраняем режим работы окон
        
      } // WORK_MODE
      else if(commandRequested == TOPEN_COMMAND)
      {
        // установка температуры открытия
        uint8_t tOpen = (uint8_t) atoi(command.GetArg(1));
        sett->SetOpenTemp(tOpen);
        
        PublishSingleton.Flags.Status = true;
        if(wantAnswer) 
        {
          PublishSingleton = commandRequested;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
         
      }
      else if(commandRequested == TCLOSE_COMMAND)
      {
        // установка температуры закрытия
        uint8_t tClose = (uint8_t) atoi(command.GetArg(1));
        sett->SetCloseTemp(tClose);
        
        PublishSingleton.Flags.Status = true;
        if(wantAnswer) 
        {
          PublishSingleton = commandRequested;
          PublishSingleton << PARAM_DELIMITER << REG_SUCC;
        }
         
      }
      else if(commandRequested == WM_INTERVAL) // запросили установку интервала
      {
              unsigned long newInt = (unsigned long) atol(command.GetArg(1));
              if(newInt > 0)
              {
                //СОХРАНЕНИЕ ИНТЕРВАЛА В НАСТРОЙКАХ
                sett->SetOpenInterval(newInt);
                
                PublishSingleton.Flags.Status = true;
                if(wantAnswer) 
                {
                  PublishSingleton = commandRequested;
                  PublishSingleton << PARAM_DELIMITER << REG_SUCC;
                }
              } // if
      } // WM_INTERVAL
    } // argsCnt > 1
  } // SET
  else
  if(command.GetType() == ctGET) // запросили показания
  {
      uint8_t argsCnt = command.GetArgsCount();
       
      if(argsCnt > 1)
      {
        // параметров хватает
        // проверяем, есть ли там запрос на кол-во, или просто индекс?
        commandRequested = command.GetArg(0);
        commandRequested.toUpperCase();

          if(commandRequested == PROP_TEMP) // обращение по температуре
          {
              commandRequested = command.GetArg(1);
              commandRequested.toUpperCase();

              if(commandRequested == PROP_TEMP_CNT) // кол-во датчиков
              {
                 PublishSingleton.Flags.Status = true;
                 if(wantAnswer) 
                 {
                  uint8_t _tempCnt = State.GetStateCount(StateTemperature);
                  PublishSingleton = commandRequested;
                  PublishSingleton << PARAM_DELIMITER << _tempCnt;
                 }
              } // if
              else // запросили по индексу или запрос ALL
              {
                if(commandRequested == ALL)
                {
                  // все датчики
                  PublishSingleton.Flags.Status = true;
                  if(wantAnswer)
                  { 
                   PublishSingleton = PROP_TEMP;
                    
                    // получаем значение всех датчиков
                    uint8_t _tempCnt = State.GetStateCount(StateTemperature);
                    
                    for(uint8_t i=0;i<_tempCnt;i++)
                    {
  
                       OneState* os = State.GetStateByOrder(StateTemperature,i);
                       if(os)
                       {
                          TemperaturePair tp = *os;
                          PublishSingleton << PARAM_DELIMITER << (tp.Current);
                       } // if(os)
                    } // for
                  } // want answer
                  
                }
                else
                {
                   // по индексу
                uint8_t sensorIdx = commandRequested.toInt();
                if(sensorIdx >= State.GetStateCount(StateTemperature) )
                {
                   if(wantAnswer)
                      PublishSingleton = NOT_SUPPORTED; // неверный индекс
                }
                 else
                  {
                    // получаем текущее значение датчика
                    PublishSingleton.Flags.Status = true;

                    if(wantAnswer)
                    {
                        OneState* os = State.GetStateByOrder(StateTemperature,sensorIdx);
                        if(os)
                        {
                          TemperaturePair tp = *os;
                          PublishSingleton = PROP_TEMP;
                          PublishSingleton << PARAM_DELIMITER  << sensorIdx << PARAM_DELIMITER << (tp.Current);
                        }
                    } // if(wantAnswer)
                  }
                } // else
              } // else
              
          } // if
          
          else if(commandRequested == PROP_WINDOW) // статус окна
          {
            commandRequested = command.GetArg(1);
           // commandRequested.toUpperCase();

            if(commandRequested == PROP_WINDOW_CNT)
            {
                    PublishSingleton.Flags.Status = true;
                    if(wantAnswer)
                    {
                      PublishSingleton = commandRequested;
                      PublishSingleton << PARAM_DELIMITER  << SUPPORTED_WINDOWS;
                    }

            }
            else
            if(commandRequested == PROP_WINDOW_STATEMASK)
            {
               // получить состояние окон в виде маски, для каждого окна - два бита в маске
               PublishSingleton.Flags.Status = true;
               if(wantAnswer)
               {
                 PublishSingleton = PROP_WINDOW;
                 PublishSingleton << PARAM_DELIMITER << commandRequested;
                 PublishSingleton << PARAM_DELIMITER << SUPPORTED_WINDOWS << PARAM_DELIMITER;

                 // теперь выводим маску. для начала считаем, сколько байт нам нужно вывести.
                 byte bitsCount = SUPPORTED_WINDOWS*2;
                 byte bytesCount = bitsCount/8;
                 if(bitsCount%8 > 0)
                  bytesCount++;

                  // посчитали кол-во байт, теперь в каждый байт мы запишем состояние максимум четырёх окон
                  byte windowIdx = 0;
                  for(byte bCntr = 0; bCntr < bytesCount; bCntr++)
                  {
                    byte workByte = 0; // байт, куда мы будем писать состояние окон
                    byte written = 0; // сколько окон записали в байт
                    byte bitPos = 0; // позиция записи битов в байт
                    
                    for(byte wIter = windowIdx; wIter < SUPPORTED_WINDOWS; wIter++, bitPos+=2)
                    {
                      if(written > 3) // записали байт полностью
                        break;

                      // теперь пишем состояние окна. Индекс окна является стартовой позицией сдвига.
                      WindowState* ws = &(Windows[wIter]);
                      if(ws->IsBusy())
                      {
                        // окно в движении
                        if(ws->GetDirection() == dirOPEN)
                        {
                          // окно открывается
                          // надо записать 01, т.е пишем в младший из двух бит
                          workByte |= (1 << bitPos);
                        }
                        else
                        {
                          // окно закрывается
                          // надо записать 10, т.е. пишем в старший из двух бит
                          workByte |= (1 << (bitPos+1));
                        }
                        
                      } // if(ws->IsBusy())
                      else
                      {
                         // окно никуда не двигается, записываем его текущее состояние
                         if(ws->GetCurrentPosition() > 0)
                         {
                           // окно открыто, надо записать две единички в биты окна
                           workByte |= (1 << bitPos);
                           workByte |= (1 << (bitPos+1));
                           
                         }
                         else
                         {
                           // окно закрыто, ничего в статус писать не надо, там одни нули
                         }
                      } // else

                        
                      written++;
                    } // for

                    // байт готов к отправке, выводим его в монитор
                    PublishSingleton << WorkStatus::ToHex(workByte);

                    windowIdx += 4; // прибавляем четвёрочку, т.к. мы в один байт можем записать информацию о состоянии максимум 4 окон
                    
                  } // for
                 
               } // if wantAnswer
               
            } // if(commandRequested == PROP_WINDOW_STATEMASK)
            else // запросили по индексу
            {
              if(commandRequested == F("ALL"))
              {
                /* 
                 Запросили состояние всех окон. Поскольку у нас окна могут находится в разничных независимых позициях,
                 разрешаем конфликты так:
                  - если хотя бы одно окно открывается - статус "открываются", при этом неважно, закрывается ли другое окно
                  - если хотя бы одно окно закрывается - статус "закрываются"
                  - если хотя бы одно окно открыто - статус "открыты"
                  - иначе - статус "закрыты"
                */
                  bool isAnyOpening = false;
                  bool isAnyClosing = false;
                  bool isAnyOpen = false;

                  for(byte k=0;k<SUPPORTED_WINDOWS;k++)
                  {
                    WindowState* ws = &(Windows[k]);

                    if(ws->IsBusy())
                    {
                      // окно в движении
                      if(ws->GetDirection() == dirOPEN)
                        isAnyOpening = true;
                      else
                        isAnyClosing = true;
                    }
                    else
                    {
                      // окно не двигается
                      if(ws->GetCurrentPosition() > 0)
                        isAnyOpen = true;
                    }
                    
                  } // for

                  // тут мы уже имеем состояние, обобщённое для всех окон
                  PublishSingleton.Flags.Status = true;
                  PublishSingleton = PROP_WINDOW;
                  PublishSingleton << PARAM_DELIMITER << commandRequested << PARAM_DELIMITER;
                  
                  if(isAnyOpening)
                  {
                    PublishSingleton << STATE_OPENING;
                  }
                  else if(isAnyClosing)
                  {
                    PublishSingleton << STATE_CLOSING;
                  }
                  else if(isAnyOpen)
                  {
                    PublishSingleton << STATE_OPEN;
                  }
                  else
                  {
                    PublishSingleton << STATE_CLOSED;
                  }
                  
              }
              else
              {
                // состояние окна по индексу
              
                   uint8_t windowIdx = commandRequested.toInt();
                   if(windowIdx >= SUPPORTED_WINDOWS)
                   {
                      if(wantAnswer)
                        PublishSingleton = NOT_SUPPORTED; // неверный индекс
                   }
                    else
                    {
                      WindowState* ws = &(Windows[windowIdx]);
                      String sAdd;
                      if(ws->IsBusy())
                      {
                        //куда-то едем
                        sAdd = ws->GetDirection() == dirOPEN ? STATE_OPENING : STATE_CLOSING;
                        
                      } // if
                      else
                      {
                          // никуда не едем
                          if(ws->GetCurrentPosition() > 0)
                            sAdd = STATE_OPEN;
                          else
                            sAdd = STATE_CLOSED;
                      } // else
                      
                      
                      PublishSingleton.Flags.Status = true;
                      if(wantAnswer)
                      {
                        PublishSingleton = PROP_WINDOW;
                        PublishSingleton << PARAM_DELIMITER << commandRequested << PARAM_DELIMITER << sAdd << PARAM_DELIMITER;

                        // тут просчитываем положение окна в процентах от максимального
                        uint32_t curWindowPosition = ws->GetCurrentPosition();
                        uint32_t maxOpenPosition = ws->GetWorkTime();

                        uint32_t positionPercents = (curWindowPosition*100)/maxOpenPosition;

                        PublishSingleton << positionPercents;// << '%';
                      }
                    } // else хороший индекс
                                    
                    } // else состояние окна по индексу
              
            } // else запросили статус окна
            
          } // else command == STATE|WINDOW|...
         
        
      } // if
      else if(argsCnt > 0)
      {
        commandRequested = command.GetArg(0);
        commandRequested.toUpperCase();

        if(commandRequested == WORK_MODE) // запросили режим работы
        {
          
          PublishSingleton.Flags.Status = true;
          if(wantAnswer)
          {
            PublishSingleton = commandRequested;
            PublishSingleton << PARAM_DELIMITER << (workMode == wmAutomatic ? WM_AUTOMATIC : WM_MANUAL);
          }
          
        } // if
        else if(commandRequested == F("WINDOWPOS")) // запросили состояние открытости окон
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = commandRequested;
          PublishSingleton << PARAM_DELIMITER << SUPPORTED_WINDOWS;

          

          for(int i=0;i<SUPPORTED_WINDOWS;i++)
          {
              uint32_t maxOpenPosition = Windows[i].GetWorkTime();
              uint32_t curWindowPosition = Windows[i].GetCurrentPosition();
              uint32_t positionPercents = (curWindowPosition*100)/maxOpenPosition;
              PublishSingleton << PARAM_DELIMITER << positionPercents;
          } // for
        }
        else
        if(commandRequested == WM_INTERVAL) // запросили ОБЩИЙ интервал срабатывания форточек
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer)
          {
            PublishSingleton = commandRequested;
            PublishSingleton << PARAM_DELIMITER  << (sett->GetOpenInterval());
          }
        } // WM_INTERVAL
        else
        if(commandRequested == TEMP_SETTINGS) // запросили температуры открытия и закрытия
        {
          PublishSingleton.Flags.Status = true;
          
          if(wantAnswer)
          {
            PublishSingleton = commandRequested;
            PublishSingleton << PARAM_DELIMITER << (sett->GetOpenTemp()) << PARAM_DELIMITER << (sett->GetCloseTemp());
          }
        }
        else
        if(commandRequested == TOPEN_COMMAND) // запросили температуру открытия
        {
          PublishSingleton.Flags.Status = true;
          
          if(wantAnswer)
          {
            PublishSingleton = commandRequested;
            PublishSingleton << PARAM_DELIMITER << (sett->GetOpenTemp());
          }
        }
        else
        if(commandRequested == TCLOSE_COMMAND) // запросили температуру закрытия
        {
          PublishSingleton.Flags.Status = true;
          
          if(wantAnswer)
          {
            PublishSingleton = commandRequested;
            PublishSingleton << PARAM_DELIMITER << (sett->GetCloseTemp());
          }
        }
        
      } // else if(argsCnt > 0)
  } // if GET
  
 // отвечаем на команду
  MainController->Publish(this,command);

  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TEMP_SENSORS

