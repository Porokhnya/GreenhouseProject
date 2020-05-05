#include "TempSensors.h"
#include "ModuleController.h"
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
#if SUPPORTED_SENSORS > 0
static uint8_t TEMP_SENSORS[] = { TEMP_SENSORS_PINS };
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifndef USE_WINDOWS_SHIFT_REGISTER
static uint8_t WINDOWS_RELAYS[] = { WINDOWS_RELAYS_PINS };
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WINDOWS_ENDSTOPS
static uint8_t ENDSTOPS_OPEN[] = { WINDOWS_ENDSTOPS_OPEN_PINS };
static uint8_t ENDSTOPS_CLOSE[] = { WINDOWS_ENDSTOPS_CLOSE_PINS };
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::ResetToMaxPosition()
{
  CurrentPosition = MainController->GetSettings()->GetOpenInterval();
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t WindowState::GetCurrentPositionPercents()
{
  unsigned long oI = MainController->GetSettings()->GetOpenInterval();
  return (CurrentPosition*100)/oI;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::Setup(uint8_t index, uint8_t relayChannel1, uint8_t relayChannel2)
{
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

    #if WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_DIRECT
    
      WORK_STATUS.PinMode(ENDSTOPS_OPEN[flags.Index],INPUT);
      WORK_STATUS.PinMode(ENDSTOPS_CLOSE[flags.Index],INPUT);
      
    #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23S17
    
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
        
          WORK_STATUS.MCP_SPI_PinMode(WINDOWS_ENDSTOPS_OPEN_MCP23S17_ADDRESS,ENDSTOPS_OPEN[flags.Index],INPUT);
          WORK_STATUS.MCP_SPI_PinMode(WINDOWS_ENDSTOPS_CLOSE_MCP23S17_ADDRESS,ENDSTOPS_CLOSE[flags.Index],INPUT);
          
        #endif
        
    #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23017
    
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
        
          WORK_STATUS.MCP_I2C_PinMode(WINDOWS_ENDSTOPS_OPEN_MCP23017_ADDRESS,ENDSTOPS_OPEN[flags.Index],INPUT);
          WORK_STATUS.MCP_I2C_PinMode(WINDOWS_ENDSTOPS_CLOSE_MCP23017_ADDRESS,ENDSTOPS_CLOSE[flags.Index],INPUT);
          
        #endif
        
    #endif    

  
  #endif // USE_WINDOWS_ENDSTOPS
  

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WindowState::ChangePosition(unsigned long newPos, bool waitFor)
{
  /*
  Serial.print(F("Window #")); Serial.println(flags.Index); 
  Serial.print(F("POSITION REQUESTED: ")); Serial.println(newPos);
  Serial.print(F("POSITION CURRENT: ")); Serial.println(CurrentPosition);
  Serial.println();
  */
  
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

  if(CurrentPosition == newPos || currentDifference < FEEDBACK_MANAGER_POSITION_HISTERESIS) 
  {
    // та же самая позиция запрошена, или разница текущей позиции и запрошеной - в пределах гистерезиса.
    // в этом случае мы ничего не делаем.
    
 //   Serial.println(F("SAME POSITION!"));
    // говорим, что мы сменили позицию
    //SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);    
    return false;
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

 //      Serial.println("OPEN FROM POSITION " + String(CurrentPosition) + " TO " + String(newPos));
  }
  else
  if(dir == dirCLOSE)
  {
        TimerInterval = CurrentPosition - newPos;
        flags.Direction = dir;

 //       Serial.println("CLOSE FROM POSITION " + String(CurrentPosition) + " TO " + String(newPos));

  }

// Serial.println();
  
  flags.OnMyWay = true; // поогнали!
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::SwitchRelays(uint8_t rel1State, uint8_t rel2State)
{

  // уведомляем родителя, что такой-то канал имеет такое-то состояние, он сам разберётся, что с этим делать
  WindowModule->SaveChannelState(RelayChannel1,rel1State);
  WindowModule->SaveChannelState(RelayChannel2,rel2State);

  // тут говорим слепку состояния, чтобы он запомнил состояние каналов окон
  WORK_STATUS.SaveWindowState(RelayChannel1,rel1State);
  WORK_STATUS.SaveWindowState(RelayChannel2,rel2State);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowState::Feedback(bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents, bool isFirstFeedback)
{  
  UNUSED(isFirstFeedback);
  
  GlobalSettings* settings = MainController->GetSettings();
  unsigned long interval = settings->GetOpenInterval();

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
      SwitchRelays(); // держим реле выключенными
      flags.Direction = dirNOTHING; // уже никуда не движемся
    } 
    
      // говорим, что мы сменили позицию, модуль правил при этом очистит очередь обработанных правил, и сможет нами рулить
      SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);  
  
     // теперь смотрим, какой концевик сработал
     if(isCloseSwitchTriggered)
     {
      // концевик на закрытие
      CurrentPosition = 0;
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
    unsigned long requestedPosition = (interval*positionPercents)/100;
    long currentDifference = 0;
    if(CurrentPosition > requestedPosition)
      currentDifference = CurrentPosition - requestedPosition;
    else
      currentDifference = requestedPosition - CurrentPosition;
      
    if(currentDifference > FEEDBACK_MANAGER_POSITION_HISTERESIS)
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
void WindowState::UpdateState(uint16_t dt)
{
  
    if(!flags.OnMyWay) // ничего не делаем
    {
      SwitchRelays(); // держим реле выключенными
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
        bRelay1State = RELAY_ON; // крутимся в одну сторону
        bRelay2State = RELAY_OFF;
        CurrentPosition += dt;

        #ifdef USE_WINDOWS_ENDSTOPS

            #if WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_DIRECT

              bAnyEndstopTriggered = WORK_STATUS.PinRead(ENDSTOPS_OPEN[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                            
            #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23S17
            
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                
                  bAnyEndstopTriggered = WORK_STATUS.MCP_SPI_PinRead(WINDOWS_ENDSTOPS_OPEN_MCP23S17_ADDRESS,ENDSTOPS_OPEN[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                  
                #endif
                
            #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23017
            
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                
                  bAnyEndstopTriggered = WORK_STATUS.MCP_I2C_PinRead(WINDOWS_ENDSTOPS_OPEN_MCP23017_ADDRESS,ENDSTOPS_OPEN[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                                    
                #endif
                
            #endif    

            if(bAnyEndstopTriggered)
            {
              CurrentPosition = MainController->GetSettings()->GetOpenInterval();
            }
        
        #endif // USE_WINDOWS_ENDSTOPS
      } 
      break; // dirOPEN

      case dirCLOSE:
      {
        bRelay1State = RELAY_OFF; // или в другую
        bRelay2State = RELAY_ON;
        CurrentPosition -= dt;

        #ifdef USE_WINDOWS_ENDSTOPS

            #if WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_DIRECT

              bAnyEndstopTriggered = WORK_STATUS.PinRead(ENDSTOPS_CLOSE[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                            
            #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23S17
            
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                
                  bAnyEndstopTriggered = WORK_STATUS.MCP_SPI_PinRead(WINDOWS_ENDSTOPS_CLOSE_MCP23S17_ADDRESS,ENDSTOPS_CLOSE[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                  
                #endif
                
            #elif WINDOWS_ENDSTOPS_DRIVE_MODE == DRIVE_MCP23017
            
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                
                  bAnyEndstopTriggered = WORK_STATUS.MCP_I2C_PinRead(WINDOWS_ENDSTOPS_CLOSE_MCP23017_ADDRESS,ENDSTOPS_CLOSE[flags.Index]) == WINDOWS_ENDSTOP_TRIGGERED_LEVEL;
                                    
                #endif
                
            #endif    

            if(bAnyEndstopTriggered)
            {
              CurrentPosition = 0;
            }
                    
        #endif // USE_WINDOWS_ENDSTOPS
        
      } 
      break; // dirCLOSE

      case dirNOTHING:
      default:
      {
        bRelay1State = SHORT_CIRQUIT_STATE; // накоротко, мотор не крутится
        bRelay2State = SHORT_CIRQUIT_STATE;
      } 
      break; // dirNOTHING
      
   } // switch


     if(!TimerInterval
#ifdef USE_WINDOWS_ENDSTOPS
        || bAnyEndstopTriggered
#endif     
     )
     {        
       // приехали, останавливаемся
       flags.Direction = dirNOTHING; // уже никуда не движемся

       // сбрасываем флаг ожидания достижения позиции
       waitForChangePositionDone = false;
       
        //ВЫКЛЮЧАЕМ РЕЛЕ
        SwitchRelays();
        
        flags.OnMyWay = false;

        // говорим, что мы сменили позицию
        SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,1);

       // Serial.println(F("Position changed!"));

        return;     
     }

    // продолжаем работу, включаем реле в нужное состояние
    SwitchRelays(bRelay1State,bRelay2State);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
TempSensors::TempSensors() : AbstractModule("STATE")
{   
  WindowModule = this; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WINDOWS_SHIFT_REGISTER
void TempSensors::WriteToShiftRegister() // ПИШЕМ В СДВИГОВЫЙ РЕГИСТР
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
    return;

   if(shiftRegisterDataSize > 0)
   {
    
    //Тут пишем в сдвиговый регистр

    // сначала разрешаем установить состояние на выходах
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_OE_PIN,LOW);
    
    // Отключаем вывод на регистре
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_LATCH_PIN, LOW);

    // проталкиваем все байты один за другим, начиная со старшего к младшему
      uint8_t i=shiftRegisterDataSize;
    
      #if (WINDOWS_SHIFT_DATA_PIN < VIRTUAL_PIN_START_NUMBER) && (WINDOWS_SHIFT_CLOCK_PIN < VIRTUAL_PIN_START_NUMBER)
      do
      {    
        // проталкиваем байт в регистр
          shiftOut(WINDOWS_SHIFT_DATA_PIN, WINDOWS_SHIFT_CLOCK_PIN, MSBFIRST, shiftRegisterData[--i]);
      } while(i > 0);
      #endif

      // "защелкиваем" регистр, чтобы байт появился на его выходах
      WORK_STATUS.PinWrite(WINDOWS_SHIFT_LATCH_PIN, HIGH);
    
   } // if
  

  // теперь сохраняем последнее запомненное состояние
   for(uint8_t i=0;i<shiftRegisterDataSize;i++)
    lastShiftRegisterData[i] = shiftRegisterData[i];
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::SaveChannelState(uint8_t channel, uint8_t state)
{
  #ifdef USE_WINDOWS_SHIFT_REGISTER
    
    //Сохраняем состояние каналов для сдвигового регистра

     uint8_t idx = channel/8; // выясняем, какой индекс в массиве байт
   // теперь мы должны выяснить, в какой бит писать
    uint8_t bitNum = channel % 8;

    // пишем в нужный байт и в нужный бит нужное состояние
    uint8_t bt = shiftRegisterData[idx];
    bitWrite(bt,bitNum, state);
    shiftRegisterData[idx] = bt;
    
    
  #else
    // просто управляем пинами, поэтому напрямую пишем в пины
    WORK_STATUS.PinWrite(WINDOWS_RELAYS[channel],state);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TempSensors::IsAnyWindowOpen()
{
  for(uint8_t i=0;i<SUPPORTED_WINDOWS;i++)
  {
    if(IsWindowOpen(i))
      return true;
  }
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool TempSensors::IsWindowOpen(uint8_t windowNumber)
{
  if(windowNumber >= SUPPORTED_WINDOWS)
    return false;

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
  for(uint8_t i=0, j=0;i<SUPPORTED_WINDOWS;i++, j+=2)
  {
      // раздаём каналы реле: первому окну - 0,1, второму - 2,3 и т.д.
      Windows[i].Setup(i, j, j+1);

      #ifdef USE_WINDOWS_SHIFT_REGISTER // если используем сдвиговые регистры
        // ничего не делаем, поскольку у нас все реле будут выключены после первоначальной настройки
      #else
        // просто настраиваем пины
          uint8_t pin1 = WINDOWS_RELAYS[j];
          uint8_t pin2 = WINDOWS_RELAYS[j+1];
        
          WORK_STATUS.PinMode(pin1,OUTPUT);
          WORK_STATUS.PinMode(pin2, OUTPUT);
        
          // выключаем реле
          WORK_STATUS.PinWrite(pin1,RELAY_OFF);
          WORK_STATUS.PinWrite(pin2,RELAY_OFF);        
     #endif

    #ifdef USE_FEEDBACK_MANAGER
      // используем менеджер обратной связи, позиция должна придти
    #else
    // просим окна закрыться при старте контроллера
    Windows[i].ChangePosition(0,true);
    #endif
    
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::CloseWindow(uint8_t num)
{
  if(num >= SUPPORTED_WINDOWS)
    return;
    
   Windows[num].ResetToMaxPosition();
   Windows[num].ChangePosition(0,true); // закрываем окно  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::CloseAllWindows()
{
  for(int i=0;i<SUPPORTED_WINDOWS;i++)
  {
     Windows[i].ResetToMaxPosition();
     Windows[i].ChangePosition(0,true); // закрываем окно
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::Setup()
{
  // настройка модуля тут
   workMode = wmAutomatic; // автоматический режим работы по умолчанию
   
#ifdef USE_WINDOWS_MANUAL_MODE_DIODE
  blinker.begin(DIODE_WINDOWS_MANUAL_MODE_PIN); // настраиваем блинкер на нужный пин
#endif  


  lastUpdateCall = 0;
  smallSensorsChange = 0;
  
   // добавляем датчики температуры
   #if SUPPORTED_SENSORS > 0

   DS18B20Temperature tempData;
   tempData.Whole = 0;
   tempData.Fract = 0;
   for(uint8_t i=0;i<SUPPORTED_SENSORS;i++)
   {
    State.AddState(StateTemperature,i);
    // запускаем конвертацию с датчиков при старте, через 2 секунды нам вернётся измеренная температура
    tempSensor.begin(TEMP_SENSORS[i]);

    tempSensor.setResolution(temp12bit); // устанавливаем разрешение датчика
    
    tempSensor.readTemperature(&tempData,0);
   }
   #endif

  
   SetupWindows(); // настраиваем фрамуги

   #ifdef USE_WINDOWS_SHIFT_REGISTER

    // настраиваем пины для сдвигового регистра на выход
    WORK_STATUS.PinMode(WINDOWS_SHIFT_LATCH_PIN,OUTPUT);
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_LATCH_PIN, LOW);
    
    WORK_STATUS.PinMode(WINDOWS_SHIFT_DATA_PIN,OUTPUT);
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_DATA_PIN, LOW);
    
    WORK_STATUS.PinMode(WINDOWS_SHIFT_CLOCK_PIN,OUTPUT);
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_CLOCK_PIN, LOW);

    // переводим все выводы в High-Z состояние (они и так уже в нём, 
    // поскольку пин, управляющий OE, подтянут к питанию,
    // но мы не будем мелочиться :) ).
    WORK_STATUS.PinMode(WINDOWS_SHIFT_OE_PIN,OUTPUT);
    WORK_STATUS.PinWrite(WINDOWS_SHIFT_OE_PIN,HIGH);
    
   
    // настраиваем кол-во байт, в котором мы будем держать состояние каналов для сдвигового регистра.
    // у нас для каждого окна - два канала, соответственно, общее кол-во бит - это
    // SUPPORTED_WINDOWS*2. Исходя из этого - легко посчитать кол-во байт, необходимых
    // для хранения данных.
    shiftRegisterDataSize =  (SUPPORTED_WINDOWS*2)/8;
    if((SUPPORTED_WINDOWS*2) > 8 && (SUPPORTED_WINDOWS*2) % 8)
      shiftRegisterDataSize++;

    shiftRegisterData = new uint8_t[shiftRegisterDataSize];
    lastShiftRegisterData = new uint8_t[shiftRegisterDataSize];
    // теперь в каждый бит этих байт записываем значение RELAY_OFF для shiftRegisterData,
    // и значение RELAY_ON для lastShiftRegisterData.
    // надо именно побитово, т.к. значение RELAY_OFF может быть 1, и в этом случае
    // все биты должны быть установлены в 1.

      uint8_t bOff = 0;
      uint8_t bOn = 0;
      for(uint8_t j=0;j<8;j++)
      {
        bOff |= (RELAY_OFF << j);
        bOn |= (RELAY_ON << j);
      }

    for(uint8_t i=0;i<shiftRegisterDataSize;i++)
    {      
      // сохранили разные значения первоначально, поскольку мы хотим записать их впервые
      shiftRegisterData[i] = bOff;
      lastShiftRegisterData[i] = bOn;
      
    } // for
      
    WriteToShiftRegister(); // пишем первоначальное состояние реле в сдвиговый регистр
    
   #endif // USE_WINDOWS_SHIFT_REGISTER


   SAVE_STATUS(WINDOWS_MODE_BIT,1); // сохраняем режим работы окон
   SAVE_STATUS(WINDOWS_POS_CHANGED_BIT,0); // говорим, что окна инициализируются
 

 }
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::Update(uint16_t dt)
{ 
#ifdef USE_WINDOWS_MANUAL_MODE_DIODE
  blinker.update(dt);
#endif  

  for(uint8_t i=0;i<SUPPORTED_WINDOWS;i++) // обновляем каналы управления фрамугами
  {
      Windows[i].UpdateState(dt);
  } // for 

 #ifdef USE_WINDOWS_SHIFT_REGISTER
  // пишем в сдвиговый регистр, если есть изменения
  WriteToShiftRegister();
 #endif 


  lastUpdateCall += dt;
  if(lastUpdateCall < TEMP_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
    return;
  else
    lastUpdateCall = 0;

  // опрашиваем наши датчики
  #if SUPPORTED_SENSORS > 0
  Temperature t;

  #ifdef MULTIPLE_DS_SENSORS_ON_ONE_PIN
  
    DSSkipList skipList;
    for(uint8_t i=0;i<SUPPORTED_SENSORS;i++)
    {
      uint8_t pinNumber = TEMP_SENSORS[i];
      bool found = false;
      
      for(size_t k=0;k<skipList.size();k++)
      {
        if(skipList[k].pin == pinNumber)
        {
          found = true;
          break;
        }
      } // for
      
      if(!found)
      {
        DSBindingStruct bs;
        bs.pin = pinNumber;
        bs.skipCount = 0;
        skipList.push_back(bs);
      }
    } // for
    
  #endif // MULTIPLE_DS_SENSORS_ON_ONE_PIN
  
  for(uint8_t i=0;i<SUPPORTED_SENSORS;i++)
  {
    t.Value = NO_TEMPERATURE_DATA;
    t.Fract = 0;

    uint8_t pinNumber = TEMP_SENSORS[i];
    tempSensor.begin(pinNumber);

    uint8_t skipCount = 0;

    #ifdef MULTIPLE_DS_SENSORS_ON_ONE_PIN
      // вычисляем, сколько адресов датчиков на текущем пине надо пропустить
      for(size_t k=0;k<skipList.size();k++)
      {
        if(skipList[k].pin == pinNumber)
        {
          skipCount = skipList[k].skipCount;
          skipList[k].skipCount = skipCount+1;
          break;
        }
      } // for
      // читаем с датчика с учётом пропуска адресов
      // запоминаем, что нв этом пине пропущен +1 адрес
      
    #endif // MULTIPLE_DS_SENSORS_ON_ONE_PIN
            
        DS18B20Temperature tempData;
        
        if(tempSensor.readTemperature(&tempData, skipCount))
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
    
    State.UpdateState(StateTemperature,i,(void*)&t); // обновляем состояние температуры, индексы датчиков у нас идут без дырок, поэтому с итератором цикла вызывать можно
  } // for
  #endif

  smallSensorsChange = 0;


}
//--------------------------------------------------------------------------------------------------------------------------------------
void TempSensors::WindowFeedback(uint8_t windowNumber, bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents, bool isFirstFeedback)
{
  #if SUPPORTED_WINDOWS > 0
    if(windowNumber >= SUPPORTED_WINDOWS)
      windowNumber = SUPPORTED_WINDOWS-1;

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
      return false; // поэтому пока они не закроются - управлять окнами при помощи внешних команд нельзя
  }  

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
         /*
        // тут проверяем, можем ли мы выполнить команду на смену позиции.
        // если используется менеджер обратной связи - мы не можем ничего делать,
        // пока менеджер ждёт первого пакета обратной связи
        #ifdef USE_FEEDBACK_MANAGER
          if(FeedbackManager.IsWaitingForFirstWindowsFeedback())
          {
            // ничего не делаем, поскольку всё ещё ждём информации по положению окон
            // отвечаем на команду
              MainController->Publish(this,command);
            
              return PublishSingleton.Flags.Status;
          }
        #endif // USE_FEEDBACK_MANAGER
        */
        
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
            #ifdef USE_WINDOWS_MANUAL_MODE_DIODE
            // мигаем светодиодом на 6 пине
             blinker.blink(WORK_MODE_BLINK_INTERVAL);
            #endif 
          }

          String token = command.GetArg(1);
          token.toUpperCase();

          String whichCommand = command.GetArg(2); // какую команду запросили?
          whichCommand.toUpperCase();
          
          bool bOpen = (whichCommand == STATE_OPEN); // запросили открытие фрамуг?          
          bool bAll = (token == ALL); // на все окна распространяется запрос?
          bool bIntervalAsked = token.indexOf("-") != -1; // запросили интервал каналов?
          uint8_t channelIdx = token.toInt(); // номер канала окна
          
          unsigned long motorsFullWorkTime = sett->GetOpenInterval();
          unsigned long targetPosition = bOpen ? motorsFullWorkTime : 0; // если не запрошено интервала - будем использовать настройки прощивки, и открываем/закрываем полностью

          //Serial.print(F("Motors FULL work time: "));
          //Serial.println(motorsFullWorkTime);
                        
          if(command.GetArgsCount() > 3) // запрошен интервал или проценты на позицию
          {
            String strIntervalPassed = command.GetArg(3);
            bool bPercentsRequested = strIntervalPassed.endsWith("%");
            
            if(bPercentsRequested)
              strIntervalPassed.remove(strIntervalPassed.length()-1);
              
            targetPosition = (unsigned long) atol(strIntervalPassed.c_str()); // получили интервал для работы реле

            if(bPercentsRequested)
            {
             // Serial.print(F("Percents requested: "));
             // Serial.println(targetPosition);
              
              // конвертируем запрошенные проценты в актуальный интервал
              targetPosition = (motorsFullWorkTime*targetPosition)/100;

              //Serial.print(F("Computed interval: "));
              //Serial.println(targetPosition);
              
            }
            else // запросили обычный интервал
            {
              // тут надо проверить - не выходим ли за границы диапазона работы приводов?
              if(targetPosition > motorsFullWorkTime)
                targetPosition = motorsFullWorkTime;
            }
          } // if(command.GetArgsCount() > 3)

 
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
              to = SUPPORTED_WINDOWS;
          
          for(uint8_t i=from;i<to;i++)
          {
            // просим окно сменить позицию
            Windows[i].ChangePosition(targetPosition);
          } // for

          // если запрошенный или рассчитанный интервал больше нуля - окна открыты, иначе - закрыты
          SAVE_STATUS(WINDOWS_STATUS_BIT,/*targetPosition > 0 ? 1 : 0*/IsAnyWindowOpen() ? 1 : 0); // сохраняем состояние окон
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
//        sett->Save();
        
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
#ifdef USE_WINDOWS_MANUAL_MODE_DIODE        
          blinker.blink();
#endif          
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
#ifdef USE_WINDOWS_MANUAL_MODE_DIODE
          blinker.blink(WORK_MODE_BLINK_INTERVAL);
#endif          
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
//                sett->Save();
                
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
                        unsigned long curWindowPosition = ws->GetCurrentPosition();
                        unsigned long maxOpenPosition = MainController->GetSettings()->GetOpenInterval();

                        unsigned long positionPercents = (curWindowPosition*100)/maxOpenPosition;

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

          unsigned long maxOpenPosition = MainController->GetSettings()->GetOpenInterval();

          for(int i=0;i<SUPPORTED_WINDOWS;i++)
          {
              unsigned long curWindowPosition = Windows[i].GetCurrentPosition();
              unsigned long positionPercents = (curWindowPosition*100)/maxOpenPosition;
              PublishSingleton << PARAM_DELIMITER << positionPercents;
          } // for
        }
        else
        if(commandRequested == WM_INTERVAL) // запросили интервал срабатывания форточек
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

