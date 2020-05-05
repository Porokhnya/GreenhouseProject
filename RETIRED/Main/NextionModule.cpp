#include "NextionModule.h"
#include "ModuleController.h"
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_NEXTION_MODULE

#ifdef USE_BUZZER
#include "Buzzer.h"
#endif

#ifdef USE_TEMP_SENSORS
#include "TempSensors.h"
#endif

#ifdef USE_SCENE_MODULE
#include "SceneModule.h"
#endif

#include "CoreNextion.h"

NextionModule* _thisModule = NULL;

#ifndef USE_NEXTION_HARDWARE_UART
#include <SoftwareSerial.h>
SoftwareSerial sSerial(NEXTION_SOFTWARE_UART_RX_PIN, NEXTION_SOFTWARE_UART_TX_PIN); // RX, TX
#undef NEXTION_SERIAL
#define NEXTION_SERIAL sSerial
#endif  

Nextion nextion(NEXTION_SERIAL);

NextionWaitScreenInfo _waitScreenInfos[] = 
{
   NEXTION_WAIT_SCREEN_SENSORS
  ,{0,0,"",""} // последний элемент пустой, заглушка для признака окончания списка
};
//--------------------------------------------------------------------------------------------------------------------------------------
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
  _thisModule->SetSleep(isSleep);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_NEXTION_PAGE_ID_RECEIVED(Nextion& sender, uint8_t pageID)
{
   _thisModule->OnPageChanged(pageID);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_NEXTION_STRING_RECEIVED(Nextion& sender, const char* str)
{
  _thisModule->StringReceived(str);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::UpdatePageData(uint8_t pageId)
{ 
    switch(pageId)
    {
      case NEXTION_START_PAGE:
      {
      //TODO: Обновляем стартовую страницу!!!
        updateTime();
        rotationTimer = 0;
        displayNextSensorData(0);
        #ifndef USE_SCENE_MODULE
        NextionGUIComponent sc("b1");
        sc.bind(nextion);
        sc.hide();
        #endif
      }
      break;

      #ifdef USE_SCENE_MODULE
      case NEXTION_SCENE_PAGE:
      {
        uint16_t scenesCnt = Scenes->GetScenesCount();
        uint16_t to = min(6,scenesCnt);
        for(uint16_t i=0;i<to;i++)
        {
          String nm = "bt";
          nm += i;
          NextionDualStateButton dsb(nm.c_str());
          dsb.bind(nextion);

          bool isA = Scenes->IsSceneActive(i);
          scenesState[i] = isA;
          dsb.value(isA ? 1 : 0);
        }
      }
      break;
      #endif

      case NEXTION_MENU_PAGE:
      {

      }
      break;

      case NEXTION_WINDOWS_PAGE:
      {
        //TODO: Обновляем страницу окон!!!
        flags.isWindowsOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
        flags.isWindowAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);

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
        flags.isWaterOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
        flags.isWaterAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);

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
        #ifdef USE_LUMINOSITY_MODULE
        flags.isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
        flags.isLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);

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
        GlobalSettings* sett = MainController->GetSettings();
        openTemp = sett->GetOpenTemp();
        closeTemp = sett->GetCloseTemp();
        unsigned long ulI = sett->GetOpenInterval()/1000;

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
        #ifdef USE_TEMP_SENSORS
          for(int i=0;i<SUPPORTED_WINDOWS;i++)
          {
            bool isWopen = WindowModule->IsWindowOpen(i);
            String nm; nm = "bt"; nm += i;
            NextionDualStateButton dsb(nm.c_str());
            dsb.bind(nextion);
            dsb.value(isWopen ? 1 : 0);

            yield();
          }
        #endif // USE_TEMP_SENSORS
      }
      break;

      case NEXTION_WATER_CHANNELS_PAGE:
      {
        //TODO: Обновляем страницу полива по каналам !!!
        #ifdef USE_WATERING_MODULE
          ControllerState state = WORK_STATUS.GetState();
          for(int i=0;i<WATER_RELAYS_COUNT;i++)
          {
            bool isWopen = (state.WaterChannelsState & (1 << i));
            String nm; nm = "bt"; nm += i;
            NextionDualStateButton dsb(nm.c_str());
            dsb.bind(nextion);
            dsb.value(isWopen ? 1 : 0);

            yield();
          }

          waterChannelsState = state.WaterChannelsState;
        #endif // USE_WATERING_MODULE
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
  GlobalSettings* sett = MainController->GetSettings();

  // по-любому кликнули на кнопку, раз пришла команда
  #ifdef USE_BUZZER
  Buzzer.buzz();
  #endif
  
  
  // Обрабатываем пришедшие команды здесь
  String sPassed = str;

  #ifdef USE_TEMP_SENSORS
  if(sPassed.startsWith(F("wnd")))
  {
    // пришла команда управления каналом окна, имеет вид wnd12=1 (для включения), wnd3=0 (для выключения), номера до '=' - каналы
    sPassed.remove(0,3);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);

    String cmd; cmd = F("STATE|WINDOW|"); cmd += channel; cmd += '|';
    if(sPassed.toInt() == 1)
      cmd += F("OPEN");
    else
      cmd += F("CLOSE");

     ModuleInterop.QueryCommand(ctSET,cmd,false);

    return;
  } // if(sPassed.startsWith(F("wnd")))
  #endif // USE_TEMP_SENSORS

  #ifdef USE_WATERING_MODULE
  if(sPassed.startsWith(F("wtrng")))
  {
    // пришла команда управления каналом полива, имеет вид wtrng12=1 (для включения), wtrng3=0 (для выключения), номера до '=' - каналы
    sPassed.remove(0,5);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);

    String cmd; cmd = F("WATER|");
    if(sPassed.toInt() == 1)
      cmd += F("ON|");
    else
      cmd += F("OFF|");
    cmd += channel;

     ModuleInterop.QueryCommand(ctSET,cmd,false);

    return;
  } // if(sPassed.startsWith(F("wtrng")))
  #endif // USE_WATERING_MODULE

  #ifdef USE_SCENE_MODULE
if(sPassed.startsWith(F("scene")))
  {
    // пришла команда управления сценой, имеет вид scene1=1 (для включения), scene3=0 (для выключения), номера до '=' - номера сцен
    sPassed.remove(0,5);
    int idx = sPassed.indexOf("=");
    String channel = sPassed.substring(0,idx);
    sPassed.remove(0,idx+1);

    String cmd; cmd = F("SCN|");
    if(sPassed.toInt() == 1)
      cmd += F("EXEC|");
    else
      cmd += F("STOP|");
    cmd += channel;

     ModuleInterop.QueryCommand(ctSET,cmd,false);

     int iCh = channel.toInt();
     scenesState[iCh] = Scenes->IsSceneActive(iCh);

    return;
  } // if(sPassed.startsWith(F("scene")))  
  #endif // USE_SCENE_MODULE

  if(sPassed.startsWith(F("topen=")))
  {
    sPassed.remove(0,6);
    sett->SetOpenTemp(sPassed.toInt());
    return;    
  }

  if(sPassed.startsWith(F("tclose=")))
  {
    sPassed.remove(0,7);
    sett->SetCloseTemp(sPassed.toInt());
    return;    
  }

  if(sPassed.startsWith(F("motors=")))
  {
    sPassed.remove(0,7);
    sett->SetOpenInterval(sPassed.toInt()*1000);
    return;    
  }
  

  #ifdef USE_TEMP_SENSORS
  if(!strcmp_P(str,(const char*)F("w_open")))
  {
    // попросили открыть окна
    ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|OPEN"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_close")))
  {
    // попросили закрыть окна
    ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|CLOSE"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_auto")))
  {
    // попросили перевести в автоматический режим окон
    ModuleInterop.QueryCommand(ctSET,F("STATE|MODE|AUTO"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("w_manual")))
  {
    // попросили перевести в ручной режим работы окон
    ModuleInterop.QueryCommand(ctSET,F("STATE|MODE|MANUAL"),false);
    return;
  }
  #endif // USE_TEMP_SENSORS

  #ifdef USE_WATERING_MODULE

  if(!strcmp_P(str,(const char*)F("wtr_skip")))
  {
    // попросили пропустить полив на сегодня
    ModuleInterop.QueryCommand(ctSET,F("WATER|SKIP"),false);
    return;
  }
    
  if(!strcmp_P(str,(const char*)F("wtr_on")))
  {
    // попросили включить полив
    ModuleInterop.QueryCommand(ctSET,F("WATER|ON"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_off")))
  {
    // попросили выключить полив
    ModuleInterop.QueryCommand(ctSET,F("WATER|OFF"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_auto")))
  {
    // попросили перевести в автоматический режим работы полива
    ModuleInterop.QueryCommand(ctSET,F("WATER|MODE|AUTO"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("wtr_manual")))
  {
    // попросили перевести в ручной режим работы полива
    ModuleInterop.QueryCommand(ctSET,F("WATER|MODE|MANUAL"),false);
    return;
  }
  #endif USE_WATERING_MODULE

  #ifdef USE_LUMINOSITY_MODULE
  if(!strcmp_P(str,(const char*)F("lht_on")))
  {
    // попросили включить досветку
    ModuleInterop.QueryCommand(ctSET,F("LIGHT|ON"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_off")))
  {
    // попросили выключить досветку
    ModuleInterop.QueryCommand(ctSET,F("LIGHT|OFF"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_auto")))
  {
    // попросили перевести досветку в автоматический режим
    ModuleInterop.QueryCommand(ctSET,F("LIGHT|MODE|AUTO"),false);
    return;
  }
  
  if(!strcmp_P(str,(const char*)F("lht_manual")))
  {
    // попросили перевести досветку в ручной режим
    ModuleInterop.QueryCommand(ctSET,F("LIGHT|MODE|MANUAL"),false);
    return;
  }
  #endif // USE_LUMINOSITY_MODULE
 
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
    if(currentPage == NEXTION_START_PAGE)
      updateTime();    
  }

  // говорим, что надо бы показать данные с датчиков
  rotationTimer = NEXTION_ROTATION_INTERVAL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::Setup()
{
  // настройка модуля тут
  _thisModule = this;
  currentPage = 0;
  
 GlobalSettings* sett = MainController->GetSettings();

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

    
  NEXTION_SERIAL.begin(SERIAL_BAUD_RATE);

  #ifdef USE_NEXTION_HARDWARE_UART

      #if TARGET_BOARD == STM32_BOARD
      if((int*)&(NEXTION_SERIAL) == (int*)&Serial) {
           WORK_STATUS.PinMode(0,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(1,OUTPUT,false);
      } else if((int*)&(NEXTION_SERIAL) == (int*)&Serial1) {
           WORK_STATUS.PinMode(19,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(18,OUTPUT,false);
      } else if((int*)&(NEXTION_SERIAL) == (int*)&Serial2) {
           WORK_STATUS.PinMode(17,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(16,OUTPUT,false);
      } else if((int*)&(NEXTION_SERIAL) == (int*)&Serial3) {
           WORK_STATUS.PinMode(15,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(14,OUTPUT,false);
      }
      #else
      if(&(NEXTION_SERIAL) == &Serial) {
           WORK_STATUS.PinMode(0,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(1,OUTPUT,false);
      } else if(&(NEXTION_SERIAL) == &Serial1) {
           WORK_STATUS.PinMode(19,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(18,OUTPUT,false);
      } else if(&(NEXTION_SERIAL) == &Serial2) {
           WORK_STATUS.PinMode(17,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(16,OUTPUT,false);
      } else if(&(NEXTION_SERIAL) == &Serial3) {
           WORK_STATUS.PinMode(15,INPUT_PULLUP,true);
           WORK_STATUS.PinMode(14,OUTPUT,false);
      }
      #endif

  #else
           WORK_STATUS.PinMode(NEXTION_SOFTWARE_UART_RX_PIN,INPUT,false);
           WORK_STATUS.PinMode(NEXTION_SOFTWARE_UART_TX_PIN,OUTPUT,false);
  #endif

  nextion.begin();

    nextion.sleepDelay(NEXTION_SLEEP_DELAY);
    nextion.wakeOnTouch(true);

    NextionNumberVariable waitTimer("va0");
    waitTimer.bind(nextion);
    waitTimer.value(NEXTION_WAIT_TIMER);

    updateTime();

    #ifndef USE_SCENE_MODULE
      NextionGUIComponent bt("b1");
      bt.bind(nextion);
      bt.hide();
    #else
      for(int i=0;i<6;i++)
      {
        scenesState.push_back(false);
      }

      // показываем доступные сцены
      uint16_t scCount = Scenes->GetScenesCount();
      int to = min(6,scCount);
      for(int i=0;i<to;i++)
      {
        String nm = "sc";
        nm += (i+1);
        NextionNumberVariable scVar(nm.c_str());
        scVar.bind(nextion);
        scVar.value(1);

        String tnm;
        tnm = 't';
        tnm += nm;
        NextionStringVariable txt(tnm.c_str());
        txt.bind(nextion);

        SceneSettings scss = Scenes->GetSceneSettings(i);
        txt.text(convert(scss.sceneName.c_str()).c_str());
      }
    #endif

    #ifndef USE_TEMP_SENSORS
      // тут скрываем кнопки вызова экранов управления окнами, т.к. модуля нет в прошивке
      NextionNumberVariable wnd("wndvis");
      wnd.bind(nextion);
      wnd.value(0);
    #else
      // сохраняем текущее положение окон
      windowsPositionFlags = 0;
      for(int i=0;i<SUPPORTED_WINDOWS;i++)
      {
        bool isWOpen = WindowModule->IsWindowOpen(i);
        if(isWOpen)
          windowsPositionFlags |= (1 << i);
      }
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

    #ifndef USE_WATERING_MODULE
      // тут скрываем кнопки вызова экранов управления поливом, т.к. модуля нет в прошивке
      NextionNumberVariable wtr("wtrvis");
      wtr.bind(nextion);
      wtr.value(0);
    #else
       ControllerState state = WORK_STATUS.GetState();
       waterChannelsState = state.WaterChannelsState;
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

    #ifndef USE_LUMINOSITY_MODULE
      // тут скрываем кнопки вызова экранов управления досветкой, т.к. модуля нет в прошивке
      NextionNumberVariable lum("lumvis");
      lum.bind(nextion);
      lum.value(0);
    #endif          
        
    flags.isWindowsOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
    flags.isWindowAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
    
    flags.isWaterOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
    flags.isWaterAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);

    flags.isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
    flags.isLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);

    openTemp = sett->GetOpenTemp();
    closeTemp = sett->GetCloseTemp();
    
    updateDisplayData();

    unsigned long ulI = sett->GetOpenInterval()/1000;

    NextionText txt("page5.motors");
    txt.bind(nextion);
    txt.text(String(ulI).c_str());

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
void NextionModule::updateTime()
{
    #ifdef USE_DS3231_REALTIME_CLOCK
      RealtimeClock rtc = MainController->GetClock();
      RTCTime controllerTime = rtc.getTime();
      
      NextionText currentTimeField("curTime");
      currentTimeField.bind(nextion);
      String ct;
      ct = rtc.getDateStr(controllerTime);
      ct += ' ';
      ct += rtc.getTimeStr(controllerTime);
      
      ct.remove(ct.lastIndexOf(":"));
      currentTimeField.text(ct.c_str());     
    #endif      
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void NextionModule::Update(uint16_t dt)
{ 
  rotationTimer += dt;
  // обновление модуля тут
  GlobalSettings* sett = MainController->GetSettings();
  
  nextion.update(); // обновляем работу с дисплеем


  static uint16_t timeCntr = 0;
  timeCntr += dt;
  if(timeCntr > 60000)
  {
    timeCntr = 0;
    if(currentPage == NEXTION_START_PAGE)
      updateTime();
  }

  // смотрим, не изменилось ли чего в позиции окон?
  #ifdef USE_TEMP_SENSORS
  if(currentPage == NEXTION_WINDOWS_CHANNELS_PAGE)
  {
      for(int i=0;i<SUPPORTED_WINDOWS;i++)
      {
        uint8_t isWOpen = WindowModule->IsWindowOpen(i) ? 1 : 0;
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
  #endif // USE_TEMP_SENSORS

  #ifdef USE_SCENE_MODULE
    if(currentPage == NEXTION_SCENE_PAGE)
    {
      for(uint16_t i=0;i<scenesState.size();i++)
      {
        bool isA = Scenes->IsSceneActive(i);
        if(isA != scenesState[i])
        {
          scenesState[i] = isA;
          String nm; nm = "bt"; nm += i;
          NextionDualStateButton dsb(nm.c_str());
          dsb.bind(nextion);
          dsb.value(isA);          
        }
      }
    }
  #endif

  // смотрим, не изменилось ли чего в состоянии каналов полива?
  #ifdef USE_WATERING_MODULE
  if(currentPage == NEXTION_WATER_CHANNELS_PAGE)
  {
      ControllerState state = WORK_STATUS.GetState();
      
      for(int i=0;i<WATER_RELAYS_COUNT;i++)
      {
        uint8_t isWOpen = (state.WaterChannelsState & (1 << i)) ? 1 : 0;
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

      waterChannelsState = state.WaterChannelsState;
  }
  #endif // USE_WATERING_MODULE  
  
  // теперь получаем все настройки и смотрим, изменилось ли чего?
  bool curVal = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
  if(curVal != flags.isWindowsOpen)
  {
    // состояние окон изменилось
    flags.isWindowsOpen = curVal;
    flags.windowChanged = true;
  }
  
  curVal = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
  if(curVal != flags.isWindowAutoMode)
  {
    // состояние режима окон изменилось
    flags.isWindowAutoMode = curVal;
    flags.windowModeChanged = true;
  }
  
  curVal = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
  if(curVal != flags.isWaterOn)
  {
    // состояние полива изменилось
    flags.isWaterOn = curVal;
    flags.waterChanged = true;
  }
  
  curVal = WORK_STATUS.GetStatus(WATER_MODE_BIT);
  if(curVal != flags.isWaterAutoMode)
  {
    // состояние режима полива изменилось
    flags.isWaterAutoMode = curVal;
    flags.waterModeChanged = true;
  }
  
  
  curVal = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
  if(curVal != flags.isLightOn)
  {
    // состояние досветки изменилось
    flags.isLightOn = curVal;
    flags.lightChanged = true;
  }
  
  curVal = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
  if(curVal != flags.isLightAutoMode)
  {
    // состояние режима досветки изменилось
    flags.isLightAutoMode = curVal;
    flags.lightModeChanged = true;
  }
  
  uint8_t cTemp = sett->GetOpenTemp();
  if(cTemp != openTemp)
  {
    // температура открытия изменилась
    openTemp = cTemp;
    flags.openTempChanged = true;
  }
  
  cTemp = sett->GetCloseTemp();
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

  currentSensorIndex += dir; // прибавляем направление
  // при старте currentSensorIndex у нас равен -1, следовательно,
  // мы должны обработать эту ситуацию
  if(currentSensorIndex < 0)
  {
     // перемещаемся на последний элемент
     currentSensorIndex = 0;
     int8_t i = 0;
     while(_waitScreenInfos[i].sensorType) // идём до конца массива, как только встретим пустой элемент - выходим
     {
      i++;
     }
     currentSensorIndex = i; // запомнили последний валидный элемент в массиве
     if(currentSensorIndex > 0)
      currentSensorIndex--;
  } // if(currentSensorIndex < 0)
  

  NextionWaitScreenInfo wsi = _waitScreenInfos[currentSensorIndex];
  if(!wsi.sensorType)
  {
    // ничего нет в текущем элементе списка.
    // перемещаемся в начало
    currentSensorIndex = 0;
    wsi = _waitScreenInfos[currentSensorIndex];
  }

  if(!wsi.sensorType)
  {
    return; // так ничего и не нашли
  }


 // теперь получаем показания от модулей
  AbstractModule* mod = MainController->GetModuleByID(wsi.moduleName);

  if(!mod) // не нашли такой модуль
  {
    rotationTimer = NEXTION_ROTATION_INTERVAL; // просим выбрать следующий модуль
    return;
  }
  OneState* os = mod->State.GetState((ModuleStates)wsi.sensorType,wsi.sensorIndex);
  if(!os)
  {
    // нет такого датчика, просим показать следующие данные
    rotationTimer = NEXTION_ROTATION_INTERVAL;
    return;
  }


   //Тут получаем актуальные данные от датчиков
   switch(wsi.sensorType)
   {
      case StateTemperature:
      {
        String displayVal = "-";
        
        if(os->HasData())
        {
          TemperaturePair tp = *os;
          displayVal = tp.Current;
        }
        
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(wsi.caption).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());

      } 
      break;

      case StateHumidity:
      case StateSoilMoisture:
      {
        String displayVal = "-";
        if(os->HasData())
        {
          HumidityPair hp = *os;
          displayVal = hp.Current;
          displayVal += "%";
        }
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(wsi.caption).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());       
      }
      break;

      case StateWaterFlowInstant:
      case StateWaterFlowIncremental:
      {
        String displayVal = "-";
        if(os->HasData())
        {
          WaterFlowPair hp = *os;
          displayVal = hp.Current;
          displayVal += " ";
          if(wsi.sensorType == StateWaterFlowInstant)
            displayVal += F("мл");
          else
            displayVal += F("л");
        }
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(wsi.caption).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());       
      }
      break;      

      case StatePH:
      {
        String displayVal = "-";
        if(os->HasData())
        {
          HumidityPair hp = *os;
          displayVal = hp.Current;
          displayVal += " pH";
        }
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(wsi.caption).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str());       
      }
      break;      

      case StateLuminosity:
      {
        String displayVal = "-";
        if(os->HasData())
        {
          LuminosityPair lp = *os;
          displayVal = lp.Current;
          displayVal += " lux";
        }
        
        NextionText txt("scapt");
        txt.bind(nextion);
        txt.text(convert(wsi.caption).c_str());

        txt.setName("sval");
        txt.text(displayVal.c_str()); 
     }
      break;
    
   } // switch    

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  NextionModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  UNUSED(command);
 
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_NEXTION_MODULE

