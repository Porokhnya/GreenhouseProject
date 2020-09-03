#include "NextionModule.h"
#include "CoreNextion.h"
//--------------------------------------------------------------------------------------------------------------------------------------
NextionModule display;
//--------------------------------------------------------------------------------------------------------------------------------------
const char* CAPTIONS[] = {SENSORS_CAPTIONS};
#ifdef DISPLAY_SCENE_PAGE
const char* SCENES_TEXTS[] = {SCENES_CAPTIONS};
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifndef USE_NEXTION_HARDWARE_UART
#include <SoftwareSerial.h>
SoftwareSerial sSerial(NEXTION_SOFTWARE_UART_RX_PIN, NEXTION_SOFTWARE_UART_TX_PIN); // RX, TX
#undef NEXTION_SERIAL
#define NEXTION_SERIAL sSerial
#endif  

Nextion nextion(NEXTION_SERIAL);

String convert(const char* in)
{  
    String out;
    if (in == NULL)
        return out;

    uint32_t codepoint = 0;
    while (*in != 0)
    {
       uint8_t ch = (uint8_t) (*in);
        if (ch <= 0x7f)
            codepoint = ch;
        else if (ch <= 0xbf)
            codepoint = (codepoint << 6) | (ch & 0x3f);
        else if (ch <= 0xdf)
            codepoint = ch & 0x1f;
        else if (ch <= 0xef)
            codepoint = ch & 0x0f;
        else
            codepoint = ch & 0x07;
        ++in;
        if (((*in & 0xc0) != 0x80) && (codepoint <= 0x10ffff))
        {
            if (codepoint <= 255)
            {
                out += (char) codepoint;
            }
            else
            {
              if(codepoint > 0x400)
                out += (char) (codepoint - 0x360);
            }
        }
    }
    return out;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_NEXTION_SLEEP(Nextion& sender, bool isSleep)
{
  display.SetSleep(isSleep);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_NEXTION_PAGE_ID_RECEIVED(Nextion& sender, uint8_t pageID)
{
   display.OnPageChanged(pageID);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_NEXTION_STRING_RECEIVED(Nextion& sender, const char* str)
{
  display.StringReceived(str);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::UpdatePageData(uint8_t pageId)
{ 
    switch(pageId)
    {
      case NEXTION_START_PAGE:
      {
      //TODO: Обновляем стартовую страницу!!!
        rotationTimer = 0;
        displayNextSensorData(0);
        //Тут скрываем компонент времени
        NextionText currentTimeField("curTime");
        currentTimeField.bind(nextion);
        currentTimeField.hide();


        #ifndef DISPLAY_SCENE_PAGE 
        // скрываем кнопку сцен
        NextionGUIComponent bt("b1");
        bt.bind(nextion);
        bt.hide();
       #endif
      }
      break;

      case NEXTION_SCENE_PAGE:
      {
        
      }
      break;

      case NEXTION_MENU_PAGE:
      {

      }
      break;

      case NEXTION_WINDOWS_PAGE:
      {
        //TODO: Обновляем страницу окон!!!
        
        flags.isWindowsOpen = settings.isWindowsOpen;
        flags.isWindowAutoMode = settings.isWindowAutoMode;

        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isWindowsOpen ? 1 : 0);
                
        dsb.setName("bt1");
        dsb.value(flags.isWindowAutoMode ? 1 : 0);
        
              
      }
      break;

      case NEXTION_WATER_PAGE:
      {
        //TODO: Обновляем страницу полива!!!
        
        flags.isWaterOn = settings.isWaterOn;
        flags.isWaterAutoMode = settings.isWaterAutoMode;

        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isWaterOn ? 1 : 0);
                
        dsb.setName("bt1");
        dsb.value(flags.isWaterAutoMode ? 1 : 0);
        
        
      }
      break;

      case NEXTION_LIGHT_PAGE:
      {
        //TODO: Обновляем страницу досветки!!!
        
        #ifdef DISPLAY_LIGHT_PAGE
        flags.isLightOn = settings.isLightOn;
        flags.isLightAutoMode = settings.isLightAutoMode;

        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isLightOn ? 1 : 0);
                
        dsb.setName("bt1");
        dsb.value(flags.isLightAutoMode ? 1 : 0);
        #endif
        
      }
      break;

      case NEXTION_OPTIONS_PAGE:
      {
        //TODO: Обновляем страницу опций!!!
        
        openTemp = settings.openTemp;
        closeTemp = settings.closeTemp;
        unsigned long ulI = settings.interval;

        NextionText txt("page5.topen");
        txt.bind(nextion);
        txt.text(String(openTemp).c_str());

        txt.setName("page5.tclose");
        txt.text(String(closeTemp).c_str());

        txt.setName("page5.motors");
        txt.text(String(ulI).c_str());
        
      }
      break;

      case NEXTION_WINDOWS_CHANNELS_PAGE:
      {
        //TODO: Обновляем страницу окон по каналам !!!
        
        #ifdef DISPLAY_WINDOWS_PAGE
          for(int i=0;i<SUPPORTED_WINDOWS;i++)
          {
            bool isWopen = (settings.windowsStatus & (1 << i));
            String nm; nm = "bt"; nm += i;
            NextionDualStateButton dsb(nm.c_str());
            dsb.bind(nextion);
            dsb.value(isWopen ? 1 : 0);

            yield();
          }
        #endif // DISPLAY_WINDOWS_PAGE
        
      }
      break;

      case NEXTION_WATER_CHANNELS_PAGE:
      {
        //TODO: Обновляем страницу полива по каналам !!!
        
        #ifdef DISPLAY_WATERING_PAGE

          for(int i=0;i<WATER_RELAYS_COUNT;i++)
          {
            bool isWopen = (currentState.WaterChannelsState & (1 << i));
            String nm; nm = "bt"; nm += i;
            NextionDualStateButton dsb(nm.c_str());
            dsb.bind(nextion);
            dsb.value(isWopen ? 1 : 0);

            yield();
          }

          waterChannelsState = currentState.WaterChannelsState;
        #endif // DISPLAY_WATERING_PAGE
      }
      break;      
    } // switch


}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::OnPageChanged(uint8_t pageID)
{
  if(pageID != currentPage)
  {
    currentPage = pageID;
    UpdatePageData(pageID);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::StringReceived(const char* str)
{

  #ifdef _DEBUG
    Serial.println(str);
  #endif

  // по-любому кликнули на кнопку, раз пришла команда
  //TODO: Тут можно пищать баззером!!!
  
  
  // Обрабатываем пришедшие команды здесь
  String sPassed = str;

  #ifdef DISPLAY_WINDOWS_PAGE
  if(sPassed.startsWith(F("wnd")))
  {
    // пришла команда управления каналом окна, имеет вид wnd12=1 (для включения), wnd3=0 (для выключения), номера до '=' - каналы
    sPassed.remove(0,3);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);


    CommandToExecute cmd;
    cmd.whichCommand = sPassed.toInt() == 1 ? emCommandOpenWindow : emCommandCloseWindow;
    cmd.param1 = channel.toInt();
    pool.push_back(cmd);
    

    return;
  } // if(sPassed.startsWith(F("wnd")))
  #endif // DISPLAY_WINDOWS_PAGE

  #ifdef DISPLAY_WATERING_PAGE
  if(sPassed.startsWith(F("wtrng")))
  {
    // пришла команда управления каналом полива, имеет вид wtrng12=1 (для включения), wtrng3=0 (для выключения), номера до '=' - каналы
    sPassed.remove(0,5);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);

    CommandToExecute cmd;
    cmd.whichCommand = sPassed.toInt() == 1 ? emCommandWaterChannelOn : emCommandWaterChannelOff;
    cmd.param1 = channel.toInt();
    pool.push_back(cmd);

    return;
  } // if(sPassed.startsWith(F("wtrng")))
  #endif // DISPLAY_WATERING_PAGE

  #ifdef DISPLAY_SCENE_PAGE
  if(sPassed.startsWith(F("scene")))
  {
    // пришла команда управления сценарием, имеет вид scene0=1 (для включения), scene3=0 (для выключения), номера до '=' - номер сцены
    sPassed.remove(0,5);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);

    CommandToExecute cmd;
    cmd.whichCommand = sPassed.toInt() == 1 ? emCommandStartScene : emCommandStopScene;
    cmd.param1 = channel.toInt();
    pool.push_back(cmd);

    return;
  } // if(sPassed.startsWith(F("scene")))
  #endif // DISPLAY_SCENE_PAGE  

  if(sPassed.startsWith(F("topen=")))
  {
    sPassed.remove(0,6);
    CommandToExecute cmd;
    cmd.whichCommand = emCommandSetOpenTemp;
    cmd.param1 = sPassed.toInt();
    pool.push_back(cmd);
    return;    
  }

  if(sPassed.startsWith(F("tclose=")))
  {
    sPassed.remove(0,7);
    CommandToExecute cmd;
    cmd.whichCommand = emCommandSetCloseTemp;
    cmd.param1 = sPassed.toInt();
    pool.push_back(cmd);
    return;    
  }

  if(sPassed.startsWith(F("motors=")))
  {
    sPassed.remove(0,7);

    uint16_t iVal = sPassed.toInt();
    byte b[2]; memcpy(b,&iVal,2);
    
    CommandToExecute cmd;
    cmd.whichCommand = emCommandSetMotorsInterval;
    cmd.param1 = b[0];
    cmd.param2 = b[1];
    pool.push_back(cmd);   
    return;    
  }
  

  #ifdef DISPLAY_WINDOWS_PAGE
  if(!strcmp_P(str,(const char*)F("w_open")))
  {
    // попросили открыть окна    
    CommandToExecute cmd;
    cmd.whichCommand = emCommandOpenWindows;
    cmd.param1 = 0;
    cmd.param2 = 100;
    pool.push_back(cmd);    
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_close")))
  {
    // попросили закрыть окна
    CommandToExecute cmd;
    cmd.whichCommand = emCommandCloseWindows;
    cmd.param1 = 0;
    cmd.param2 = 0;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_auto")))
  {
    // попросили перевести в автоматический режим окон
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWindowsAutoMode;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_manual")))
  {
    // попросили перевести в ручной режим работы окон
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWindowsManualMode;
    pool.push_back(cmd);  
    return;
  }
  #endif // DISPLAY_WINDOWS_PAGE

  #ifdef DISPLAY_WATERING_PAGE

  if(!strcmp_P(str,(const char*)F("wtr_skip")))
  {
    // попросили пропустить полив на сегодня
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWaterSkip;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_on")))
  {
    // попросили включить полив
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWaterOn;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_off")))
  {
    // попросили выключить полив
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWaterOff;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_auto")))
  {
    // попросили перевести в автоматический режим работы полива
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWaterAutoMode;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_manual")))
  {
    // попросили перевести в ручной режим работы полива
    CommandToExecute cmd;
    cmd.whichCommand = emCommandWaterManualMode;
    pool.push_back(cmd);  
    return;
  }
  #endif DISPLAY_WATERING_PAGE

  #ifdef DISPLAY_LIGHT_PAGE
  if(!strcmp_P(str,(const char*)F("lht_on")))
  {
    // попросили включить досветку
    CommandToExecute cmd;
    cmd.whichCommand = emCommandLightOn;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_off")))
  {
    // попросили выключить досветку
    CommandToExecute cmd;
    cmd.whichCommand = emCommandLigntOff;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_auto")))
  {
    // попросили перевести досветку в автоматический режим
    CommandToExecute cmd;
    cmd.whichCommand = emCommandLightAutoMode;
    pool.push_back(cmd);  
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_manual")))
  {
    // попросили перевести досветку в ручной режим
    CommandToExecute cmd;
    cmd.whichCommand = emCommandLightManualMode;
    pool.push_back(cmd);  
    return;
  }
  #endif // DISPLAY_LIGHT_PAGE
 
 if(!strcmp_P(str,(const char*)F("prev")))
  {
    rotationTimer = 0;
    displayNextSensorData(-1);
    return;
  }

 if(!strcmp_P(str,(const char*)F("next")))
  {
    rotationTimer = 0;
    displayNextSensorData(1);
    return;
  }
  
  // тут отрабатываем остальные команды

   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::SetSleep(bool bSleep)
{
  flags.isDisplaySleep = bSleep;
  updateDisplayData(); // обновляем основные данные для дисплея

  if(!bSleep)
  {
   // if(currentPage == NEXTION_START_PAGE)
   //   updateTime();    
  }

  // говорим, что надо бы показать данные с датчиков
  rotationTimer = NEXTION_ROTATION_INTERVAL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::begin()
{
  // настройка модуля тут
  currentPage = 0;
  
  rotationTimer = NEXTION_ROTATION_INTERVAL;
  currentSensorIndex = -1;
  
  flags.isDisplaySleep = false;
  
  flags.windowChanged = true;
  flags.windowModeChanged = true;
  flags.waterChanged = true;
  flags.waterModeChanged = true;
  flags.lightChanged = true;
  flags.lightModeChanged = true;
  flags.openTempChanged = true;
  flags.closeTempChanged = true;

    
  NEXTION_SERIAL.begin(RS485_SPEED);
  nextion.begin();

    nextion.sleepDelay(NEXTION_SLEEP_DELAY);
    nextion.wakeOnTouch(true);

    NextionNumberVariable waitTimer("va0");
    waitTimer.bind(nextion);
    waitTimer.value(NEXTION_WAIT_TIMER);

      
    #ifndef DISPLAY_WINDOWS_PAGE
      // тут скрываем кнопки вызова экранов управления окнами, т.к. модуля нет в прошивке
      NextionNumberVariable wnd("wndvis");
      wnd.bind(nextion);
      wnd.value(0);
    #else
      // сохраняем текущее положение окон
      windowsPositionFlags = 0;
      // тут настраиваем кнопки управления каналами окон, скрывая ненужные из них
      for(int i=SUPPORTED_WINDOWS;i<16;i++)
      {
       
        String nm; nm = "wndch"; nm += i;
        NextionNumberVariable wndch(nm.c_str());
        wndch.bind(nextion);
        wndch.value(0);
        yield();
      }
      
    #endif    

    #ifndef DISPLAY_WATERING_PAGE
      // тут скрываем кнопки вызова экранов управления поливом, т.к. модуля нет в прошивке
      NextionNumberVariable wtr("wtrvis");
      wtr.bind(nextion);
      wtr.value(0);
    #else

       waterChannelsState = currentState.WaterChannelsState;
      // настраиваем кнопки каналов полива, скрывая ненужные из них
      for(int i=WATER_RELAYS_COUNT;i<16;i++)
      {
        String nm; nm = "wtrch"; nm += i;
        NextionNumberVariable wtrch(nm.c_str());
        wtrch.bind(nextion);
        wtrch.value(0);
        yield();        
      }
    #endif

    #ifndef DISPLAY_LIGHT_PAGE
      // тут скрываем кнопки вызова экранов управления досветкой, т.к. модуля нет в прошивке
      NextionNumberVariable lum("lumvis");
      lum.bind(nextion);
      lum.value(0);
    #endif

      #ifdef DISPLAY_SCENE_PAGE
        const size_t cntr = sizeof(SCENES_TEXTS)/sizeof(SCENES_TEXTS[0]);
        for(size_t i=0;i<cntr;i++)
        {
          String nm; nm = "sc";
          nm += (i+1);
          
          NextionNumberVariable nv(nm.c_str());
          nv.bind(nextion);
          nv.value(1);

          String tnm; tnm = 't'; tnm += nm;
          NextionStringVariable sv(tnm.c_str());
          sv.bind(nextion);
          sv.text(convert(SCENES_TEXTS[i]).c_str());
        }
      #endif


    
    updateDisplayData();

 }
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::updateDisplayData()
{
  
  if(flags.isDisplaySleep) // для спящего дисплея нечего обновлять
    return;

    if(flags.windowChanged)
    {
      flags.windowChanged = false;
      if(currentPage == NEXTION_WINDOWS_PAGE)
      {
        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isWindowsOpen ? 1 : 0);        
      }
    }
    
    if(flags.windowModeChanged)
    {
      flags.windowModeChanged = false;
      if(currentPage == NEXTION_WINDOWS_PAGE)
      {
        NextionDualStateButton dsb("bt1");
        dsb.bind(nextion);
        dsb.value(flags.isWindowAutoMode ? 1 : 0);        
      }      
    }
    
    if(flags.waterChanged)
    {
      flags.waterChanged = false;
      if(currentPage == NEXTION_WATER_PAGE)
      {
        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isWaterOn ? 1 : 0);        
      }      
    }
    
    if(flags.waterModeChanged)
    {
      flags.waterModeChanged = false;
      if(currentPage == NEXTION_WATER_PAGE)
      {
        NextionDualStateButton dsb("bt1");
        dsb.bind(nextion);
        dsb.value(flags.isWaterAutoMode ? 1 : 0);        
      }      
    }
    
    if(flags.lightChanged)
    {
      flags.lightChanged = false;
      if(currentPage == NEXTION_LIGHT_PAGE)
      {
        NextionDualStateButton dsb("bt0");
        dsb.bind(nextion);
        dsb.value(flags.isLightOn ? 1 : 0);        
      }   
    }
    
    if(flags.lightModeChanged)
    {
      flags.lightModeChanged = false;
      if(currentPage == NEXTION_LIGHT_PAGE)
      {
        NextionDualStateButton dsb("bt1");
        dsb.bind(nextion);
        dsb.value(flags.isLightAutoMode ? 1 : 0);        
      }   
    }
    
    if(flags.openTempChanged)
    {
      flags.openTempChanged = false;
      NextionText txt("page5.topen");
      txt.bind(nextion);
      txt.text(String(openTemp).c_str());
    }
    
    if(flags.closeTempChanged)
    {
      flags.closeTempChanged = false;
      NextionText txt("page5.tclose");
      txt.bind(nextion);
      txt.text(String(closeTemp).c_str());
    }
    
    
   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::update(uint16_t dt)
{ 
  rotationTimer += dt;
    
  nextion.update(); // обновляем работу с дисплеем

  if(millis() - resetSensorsTimer > RESET_SENSORS_DATA_DELAY)
  {
    for(size_t i=0;i<sensors.size();i++)
    {
      sensors[i].hasData = false;
    }
    resetSensorsTimer = millis();
  }


  // смотрим, не изменилось ли чего в позиции окон?
  #ifdef DISPLAY_WINDOWS_PAGE
  if(currentPage == NEXTION_WINDOWS_CHANNELS_PAGE)
  {
      for(int i=0;i<SUPPORTED_WINDOWS;i++)
      {
        
        uint8_t isWOpen = settings.windowsStatus & (1 << i) ? 1 : 0;
        uint8_t lastWOpen = windowsPositionFlags & (1 << i) ? 1 : 0;
        
        if(lastWOpen != isWOpen)
        {
          windowsPositionFlags &= ~(1 << i);
          windowsPositionFlags |= (isWOpen << i);
          String nm; nm = "bt"; nm += i;
          NextionDualStateButton dsb(nm.c_str());
          dsb.bind(nextion);
          dsb.value(isWOpen);
        }
        yield();
       
      }
  }
  #endif // DISPLAY_WINDOWS_PAGE

  // смотрим, не изменилось ли чего в состоянии каналов полива?
  #ifdef DISPLAY_WATERING_PAGE
  if(currentPage == NEXTION_WATER_CHANNELS_PAGE)
  {      
      for(int i=0;i<WATER_RELAYS_COUNT;i++)
      {
        uint8_t isWOpen = (currentState.WaterChannelsState & (1 << i)) ? 1 : 0;
        uint8_t lastWOpen = (waterChannelsState & (1 << i)) ? 1 : 0;
        if(lastWOpen != isWOpen)
        {
          String nm; nm = "bt"; nm += i;
          NextionDualStateButton dsb(nm.c_str());
          dsb.bind(nextion);
          dsb.value(isWOpen);
        }
        yield();
      }

      waterChannelsState = currentState.WaterChannelsState;
  }
  #endif // DISPLAY_WATERING_PAGE  

  
  // теперь получаем все настройки и смотрим, изменилось ли чего?
  bool curVal = settings.isWindowsOpen;
  if(curVal != flags.isWindowsOpen)
  {
    // состояние окон изменилось
    flags.isWindowsOpen = curVal;
    flags.windowChanged = true;
  }
  
  curVal = settings.isWindowAutoMode;
  if(curVal != flags.isWindowAutoMode)
  {
    // состояние режима окон изменилось
    flags.isWindowAutoMode = curVal;
    flags.windowModeChanged = true;
  }
  
  curVal = settings.isWaterOn;
  if(curVal != flags.isWaterOn)
  {
    // состояние полива изменилось
    flags.isWaterOn = curVal;
    flags.waterChanged = true;
  }
  
  curVal = settings.isWaterAutoMode;
  if(curVal != flags.isWaterAutoMode)
  {
    // состояние режима полива изменилось
    flags.isWaterAutoMode = curVal;
    flags.waterModeChanged = true;
  }
  
  
  curVal = settings.isLightOn;
  if(curVal != flags.isLightOn)
  {
    // состояние досветки изменилось
    flags.isLightOn = curVal;
    flags.lightChanged = true;
  }
  
  curVal = settings.isLightAutoMode;
  if(curVal != flags.isLightAutoMode)
  {
    // состояние режима досветки изменилось
    flags.isLightAutoMode = curVal;
    flags.lightModeChanged = true;
  }
  

  
  uint8_t cTemp = settings.openTemp;
  if(cTemp != openTemp)
  {
    // температура открытия изменилась
    openTemp = cTemp;
    flags.openTempChanged = true;
  }
  
  cTemp = settings.closeTemp;
  if(cTemp != closeTemp)
  {
    // температура закрытия изменилась
    closeTemp = cTemp;
    flags.closeTempChanged = true;
  }
  

  updateDisplayData(); // обновляем дисплей
  
  // обновили дисплей, теперь на нём актуальные данные, можем работать с датчиками
  if(rotationTimer > NEXTION_ROTATION_INTERVAL)
  {
    rotationTimer = 0;
    displayNextSensorData(1);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::displayNextSensorData(int8_t dir)
{
  if(flags.isDisplaySleep)
    return;

  if(currentPage != NEXTION_START_PAGE)    
    return;

  if(!sensors.size()) // нечего отображать
    return;

  currentSensorIndex += dir; // прибавляем направление
  // при старте currentSensorIndex у нас равен -1, следовательно,
  // мы должны обработать эту ситуацию
  if(currentSensorIndex < 0)
  {
     // перемещаемся на последний элемент
     currentSensorIndex = sensors.size() - 1;      
  } // if(currentSensorIndex < 0)

  // заворачиваем в начало, если надо
  if(currentSensorIndex >= sensors.size())
    currentSensorIndex = 0;


  SensorDisplayData dt = sensors[currentSensorIndex];


   //Тут получаем актуальные данные от датчиков
   switch(dt.type)
   {
      case StateTemperature:
      {
        String displayVal = "-";     
        if(dt.hasData)
        {
          // во втором байте - целое
          // в первом - сотые доли
          displayVal = dt.data[1];
          displayVal += FRACT_DELIMITER;
          
          if(dt.data[0] < 10)
            displayVal += '0';

          displayVal += dt.data[0];
        }
        
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(CAPTIONS[currentSensorIndex]).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());

      } 
      break;

      case StateHumidity:
      case StateSoilMoisture:
      {
        String displayVal = "-";     
        if(dt.hasData)
        {
          // во втором байте - целое
          // в первом - сотые доли
          displayVal = dt.data[1];
          displayVal += FRACT_DELIMITER;
          
          if(dt.data[0] < 10)
            displayVal += '0';

          displayVal += dt.data[0];

          displayVal += '%';
        }
        
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(CAPTIONS[currentSensorIndex]).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());     
      }
      break;

      case StateWaterFlowInstant:
      case StateWaterFlowIncremental:
      {
        String displayVal = "-";
        if(dt.hasData)
        {
          // четыре байта расхода воды
          uint32_t val;
          memcpy(&val,dt.data,sizeof(uint32_t));
          
          displayVal = val;
          
          displayVal += " ";
          if(dt.type == StateWaterFlowInstant)
            displayVal += F("мл");
          else
            displayVal += F("л");
        }
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(CAPTIONS[currentSensorIndex]).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());       
      }
      break;      

      case StatePH:
      {
        String displayVal = "-";
        if(dt.hasData)
        {
          // во втором байте - целое
          // в первом - сотые доли
          displayVal = dt.data[1];
          displayVal += FRACT_DELIMITER;
          
          if(dt.data[0] < 10)
            displayVal += '0';

          displayVal += dt.data[0];
          displayVal += " pH";
        }
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(CAPTIONS[currentSensorIndex]).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());       
      }
      break;      

      case StateLuminosity:
      {
        String displayVal = "-";
        if(dt.hasData)
        {
          // два байта - освещенность
          uint16_t val;
          memcpy(&val,dt.data,sizeof(uint16_t));
          displayVal = val;
          displayVal += " lux";
        }
        
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(CAPTIONS[currentSensorIndex]).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str()); 
     }
      break;
    
   } // switch    

}
//--------------------------------------------------------------------------------------------------------------------------------------

