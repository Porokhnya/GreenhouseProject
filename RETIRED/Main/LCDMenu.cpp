#include "LCDMenu.h"
#include "InteropStream.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LCD_MODULE

#ifdef USE_BUZZER
#include "Buzzer.h"
#endif

#ifdef USE_SCENE_MODULE
#include "SceneModule.h"
#endif

#if defined(USE_TEMP_SENSORS) && defined(WINDOWS_CHANNELS_SCREEN_ENABLED)
#include "TempSensors.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
WaitScreenInfo WaitScreenInfos[] = 
{
   WAIT_SCREEN_SENSORS
  ,{0,0,"",""} // последний элемент пустой, заглушка для признака окончания списка
};
//--------------------------------------------------------------------------------------------------------------------------------------
PushButton button(MENU_BUTTON_PIN); // кнопка для управления меню
//--------------------------------------------------------------------------------------------------------------------------------------
void ButtonOnClick(const PushButton& Sender, void* UserData) // пришло событие от кнопки - кликнута
{
  UNUSED(Sender);

  #ifdef USE_BUZZER
  Buzzer.buzz();
  #endif
  
  LCDMenu* menu = (LCDMenu*) UserData;
  menu->enterSubMenu(); // просим войти в подменю
}
//--------------------------------------------------------------------------------------------------------------------------------------
IdlePageMenuItem IdleScreen; // экран ожидания
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
SceneMenuItem SceneManageScreen; // экран управления сценариями
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS
WindowMenuItem WindowManageScreen; // экран управления окнами
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE
WateringMenuItem WateringManageScreen; // экран управления поливом
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_WATERING_MODULE) && defined(WATER_CHANNELS_SCREEN_ENABLED)
WateringChannelsMenuItem WateringChannelsManageScreen; // экран управления каналами полива
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_TEMP_SENSORS) && defined(WINDOWS_CHANNELS_SCREEN_ENABLED)
WindowsChannelsMenuItem WindowsChannelsManageScreen; // экран управления каналами полива
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
LuminosityMenuItem LuminosityManageScreen; // экран управления досветкой
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
SettingsMenuItem SettingsManageScreen; // экран настроек
//--------------------------------------------------------------------------------------------------------------------------------------
AbstractLCDMenuItem::AbstractLCDMenuItem(const unsigned char* i, const char* c) :
icon(i), caption(c), flags(0),/*focused(false), needToDrawCursor(false),*/cursorPos(-1), itemsCount(0)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void AbstractLCDMenuItem::init(LCDMenu* parent)
{
  parentMenu = parent;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void AbstractLCDMenuItem::setFocus(bool f)
{
  //focused = f;
  flags &= ~1;
  if(f)
    flags |= 1;
    
 if(!f)
  {
   // needToDrawCursor = false;
   flags &= ~2;
    cursorPos = -1;
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void AbstractLCDMenuItem::OnButtonClicked(LCDMenu* menu)
{
  // кликнули по кнопке, когда наше меню на экране.
  // фокус ввода может быть ещё не установлен (первое нажатие на кнопку),
  // или - установлен (повторные нажатия на кнопку)
  
  bool lastNDC = (flags & 2);//needToDrawCursor;
  //needToDrawCursor = true;
  flags |= 2;
  int8_t lastCP = cursorPos;

  // увеличиваем позицию курсора
  ++cursorPos;
  if(!itemsCount || cursorPos > (itemsCount-1)) // значит, достигли края, и надо выйти из фокуса
  {
    //needToDrawCursor = false;
    flags &= ~2;
    cursorPos = -1;
    flags &= ~1; //focused = false; // не вызываем setFocus напрямую, т.к. он может быть переопределён
  }

  if(lastNDC != /*needToDrawCursor*/(bool)(flags & 2) || lastCP != cursorPos) // сообщаем, что надо перерисовать экран, т.к. позиция курсора изменилась
    menu->wantRedraw(); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
IdlePageMenuItem::IdlePageMenuItem() : AbstractLCDMenuItem(MONITOR_ICON,LCD_MONITOR_CAPTION)
{
  rotationTimer = ROTATION_INTERVAL; // получаем данные с сенсора сразу в первом вызове update
  currentSensorIndex = 0; 

#ifdef SENSORS_SETTINGS_ON_SD_ENABLED
  idleFlags.linkedToSD = false;
  idleFlags.sdSettingsInited = false;
  idleFlags.currentSensorsDirectory = -1;
#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef SENSORS_SETTINGS_ON_SD_ENABLED
//--------------------------------------------------------------------------------------------------------------------------------------
bool IdlePageMenuItem::SelectNextDirectory(LCDMenu* menu)
{    

  // тут надо переместится на первую папку, в которой есть файлы.
  // причём переместится, учитывая зацикливание и тот вариант,
  // что файлов вообще может не быть
  byte attempts = 0;

  while(true)
  {
      attempts++;
      if(attempts >= DIR_DUMMY_LAST_DIR*2) // пробежали по кругу однозначно
      {
        // если ничего не нашли - деградируем в прошитые настройки
        idleFlags.linkedToSD = false;
        return false;
      }

    idleFlags.currentSensorsDirectory++;
    if(idleFlags.currentSensorsDirectory >= DIR_DUMMY_LAST_DIR)
      idleFlags.currentSensorsDirectory = DIR_TEMP;

      if(menu->GetFilesCount(idleFlags.currentSensorsDirectory) > 0) // есть файлы в папке
        return true;
        
  } // while

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::SelectNextSDSensor(LCDMenu* menu)
{
    if(!workDir.isOpen()) // нет открытой текущей папки
    {
      if(!SelectNextDirectory(menu))
        return;

      OpenCurrentSDDirectory(menu);
    }

    if(workDir.isOpen())
    {
      if(workFile.isOpen())
        workFile.close();

        workFile.openNext(&workDir,O_READ);

        if(!workFile.isOpen()) 
        {
           // дошли до конца, надо выбрать следующую папку
           workDir.close(); // закрываем текущую папку, чтобы перейти на новую папку
           SelectNextSDSensor(menu);
           return;
        }

        // файл открыли, можно работать
      
    } // if
   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::OpenCurrentSDDirectory(LCDMenu* menu)
{
  UNUSED(menu);
  
  if(workDir.isOpen())
    workDir.close();

    String folderName = "LCD";
    folderName += F("/");
    
    switch(idleFlags.currentSensorsDirectory)
    {
        case DIR_TEMP:
          folderName += F("TEMP");
        break;
  
        case DIR_HUMIDITY:
          folderName += "HUMIDITY";
        break;
  
        case DIR_LUMINOSITY:
          folderName += "LIGHT";
        break;
  
        case DIR_SOIL:
          folderName += "SOIL";
        break;
  
        case DIR_PH:
          folderName += "PH";
        break;
      
    } // switch

    workDir.open(folderName.c_str(),FILE_READ);
    
    if(workDir.isOpen())
      workDir.rewind();
}
//--------------------------------------------------------------------------------------------------------------------------------------
String IdlePageMenuItem::ReadCurrentFile()
{
  String result;
  
    if(!workFile.isOpen())
    {
      return result;
    }

    int32_t sz = workFile.fileSize();

    if(sz > 0)
    {
      char* buff = new char[sz+1];
      buff[sz] = '\0';
      workFile.read(buff,sz);

      result = buff;
      delete [] buff;
      
      return result;
    }

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::RequestSDSensorData(LCDMenu* menu)
{
  UNUSED(menu);

  if(!idleFlags.linkedToSD)
    return;
  
  sensorData = "";
  displayString = "";

  
    if(!workFile.isOpen())
      return;

   // получаем имя файла
    String idx;
    String strFileName = FileUtils::GetFileName(workFile);
    const char* fName = strFileName.c_str();

    while(*fName && *fName != '.')
    {
      // выцепляем индекс датчика
      idx += *fName;
      fName++;
    }

    // получаем модуль в системе
    AbstractModule* module = NULL;
    ModuleStates sensorType;

    switch(idleFlags.currentSensorsDirectory)
    {
        case DIR_TEMP:
          module = MainController->GetModuleByID("STATE");
          sensorType = StateTemperature;
        break;
  
        case DIR_HUMIDITY:
          module = MainController->GetModuleByID("HUMIDITY");
          sensorType = StateHumidity;
        break;
  
        case DIR_LUMINOSITY:
          module = MainController->GetModuleByID("LIGHT");
          sensorType = StateLuminosity;
        break;
  
        case DIR_SOIL:
          module = MainController->GetModuleByID("SOIL");
          sensorType = StateSoilMoisture;
        break;
  
        case DIR_PH:
          module = MainController->GetModuleByID("PH");
          sensorType = StatePH;
        break;
      
    } // switch    

    if(module)
    {
      // получаем состояние для датчика
      OneState* os = module->State.GetState(sensorType,idx.toInt());
      if(!os)
      {
        // нет такого датчика, просим показать следующие данные
        rotationTimer = ROTATION_INTERVAL;
        return;
      }

      // датчик есть, можно получать его данные
        if(os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
        else
           sensorData = NO_DATA;

       // в displayString надо прочитать содержимое файла
       displayString = ReadCurrentFile();
      
    } // if(module)
      
   workFile.close(); // не забываем закрывать файл за собой
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);
  // инициализируем экран ожидания
  
  // получаем данные с сенсора
  RequestSensorData(WaitScreenInfos[currentSensorIndex]);

}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::OnButtonClicked(LCDMenu* menu)
{
  AbstractLCDMenuItem::OnButtonClicked(menu);

    rotationTimer = 0; // сбрасываем таймер ротации
    
  #ifdef SENSORS_SETTINGS_ON_SD_ENABLED

      if(idleFlags.linkedToSD)
      {
        // работаем с SD

        // выбираем следующий файл на SD
        SelectNextSDSensor(menu);

        // и запрашиваем с него показания
        RequestSDSensorData(menu);
      }
      else
      {
        // деградируем на прошитые жёстко настройки
        // выбираем следующий сенсор
        SelectNextSensor();
        // получаем данные с сенсора
        RequestSensorData(WaitScreenInfos[currentSensorIndex]);
      }
  
  #else
      // выбираем следующий сенсор
      SelectNextSensor();
      // получаем данные с сенсора
      RequestSensorData(WaitScreenInfos[currentSensorIndex]);
  
  #endif

    // говорим, что мы хотим перерисоваться
    menu->notifyMenuUpdated(this);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::SelectNextSensor()
{
    currentSensorIndex++;
    if(!WaitScreenInfos[currentSensorIndex].sensorType)
    {
      // достигли конца списка, возвращаемся в начало
      currentSensorIndex = 0;
    }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::update(uint16_t dt, LCDMenu* menu)
{
#ifdef SENSORS_SETTINGS_ON_SD_ENABLED
  if(!idleFlags.sdSettingsInited)
  {
    idleFlags.sdSettingsInited = true;
    idleFlags.linkedToSD = menu->HasSensorsSettingsOnSD();
  }
#endif  
  

  rotationTimer += dt;

  if(rotationTimer >= ROTATION_INTERVAL) // пришла пора крутить показания
  {
    rotationTimer = 0;

    #ifdef SENSORS_SETTINGS_ON_SD_ENABLED

      if(idleFlags.linkedToSD)
      {
        // есть файлы на SD
        // выбираем следующий датчик на SD
        SelectNextSDSensor(menu);
        
        // и запрашиваем для него данные
        RequestSDSensorData(menu);
      }
      else
      {
        // нет файлов на SD
        // выбираем следующий сенсор
        SelectNextSensor();
    
        // получаем данные с сенсора
        RequestSensorData(WaitScreenInfos[currentSensorIndex]);
      }

    #else
      // работа с SD выключена в прошивке

      // выбираем следующий сенсор
      SelectNextSensor();
  
      // получаем данные с сенсора
      RequestSensorData(WaitScreenInfos[currentSensorIndex]);

    #endif

    // говорим, что мы хотим перерисоваться
    menu->notifyMenuUpdated(this);
    
  } // if(rotationTimer >= ROTATION_INTERVAL)

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::RequestSensorData(const WaitScreenInfo& info)
{
  // обновляем показания с датчиков
  sensorData = "";
  displayString = "";

  if(!info.sensorType) // нечего показывать
    return;

  AbstractModule* mod = MainController->GetModuleByID(info.moduleName);

  if(!mod) // не нашли такой модуль
  {
    rotationTimer = ROTATION_INTERVAL; // просим выбрать следующий модуль
    return;
  }
  OneState* os = mod->State.GetState((ModuleStates)info.sensorType,info.sensorIndex);
  if(!os)
  {
    // нет такого датчика, просим показать следующие данные
    rotationTimer = ROTATION_INTERVAL;
    return;
  }

  displayString = info.displayName; // запоминаем, чего выводить на экране


   //Тут получаем актуальные данные от датчиков
    if(os->HasData())
    {
      sensorData = *os;
      sensorData += os->GetUnit();
    }
    else
       sensorData = NO_DATA;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void IdlePageMenuItem::draw(DrawContext* dc)
{
  // рисуем показания с датчиков
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  
  if(!sensorData.length() || !displayString) // нечего рисовать
    return;

  // рисуем показания с датчика по центру экрана
  int cur_top = 14 + MENU_BITMAP_SIZE;
  u8g_uint_t strW = dc->getStrWidth(sensorData.c_str());
  int left = (frame_width - strW)/2 + CONTENT_PADDING;

  dc->drawStr(left, cur_top, sensorData.c_str());
  yield();

  // теперь рисуем строку подписи
  cur_top += HINT_FONT_HEIGHT;
  if(displayString.length())
  {
    strW = dc->getStrWidth(displayString.c_str());
    left = (frame_width - strW)/2 + CONTENT_PADDING;
    dc->drawStr(left, cur_top, displayString.c_str());
  }
  yield();

     #ifdef USE_DS3231_REALTIME_CLOCK

        cur_top += HINT_FONT_HEIGHT + 6;
        
        RealtimeClock rtc = MainController->GetClock();
        RTCTime tm = rtc.getTime();

        /*static */char dt_buff[20] = {0};
        sprintf_P(dt_buff,(const char*) F("%02d.%02d.%d %02d:%02d"), tm.dayOfMonth, tm.month, tm.year, tm.hour, tm.minute);
        
        strW = dc->getStrWidth(dt_buff);
        left = (frame_width - strW)/2 + CONTENT_PADDING;
        
        dc->drawStr(left, cur_top, dt_buff);
        yield();
        
      #endif // USE_DS3231_REALTIME_CLOCK 

}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS
WindowMenuItem::WindowMenuItem() : AbstractLCDMenuItem(WINDOW_ICON,LCD_WINDOWS_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);
  
  windowsFlags.isWindowsOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
  windowsFlags.isWindowsAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);

  itemsCount = 3;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WindowMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;

    bool lastWO = windowsFlags.isWindowsOpen;
    bool lastWAM = windowsFlags.isWindowsAutoMode;

    if(dir != 0)
    {
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // открыть окна
          {
            windowsFlags.isWindowsOpen = true;
            //Тут посылаем команду на открытие окон
            ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|OPEN"),false);
            yield();
          }
          break;
          
          case 1: // закрыть окна
          {
            windowsFlags.isWindowsOpen = false;
            //Тут посылаем команду на закрытие окон
            ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|CLOSE"),false);
            yield();
          }
          break;
          
          case 2: // поменять режим
          {
            windowsFlags.isWindowsAutoMode = !windowsFlags.isWindowsAutoMode;
            //Тут посылаем команду на смену режима окон
            if(windowsFlags.isWindowsAutoMode)
              ModuleInterop.QueryCommand(ctSET,F("STATE|MODE|AUTO"),false);
            else
              ModuleInterop.QueryCommand(ctSET,F("STATE|MODE|MANUAL"),false);

            yield();
          }
          break;
        
       } // switch
    }

    if(lastWO != windowsFlags.isWindowsOpen || lastWAM != windowsFlags.isWindowsAutoMode) // состояние изменилось, просим меню перерисоваться
      menu->wantRedraw();

    return true; // сами обработали смену позиции энкодера
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowMenuItem::update(uint16_t dt, LCDMenu* menu)
{
  UNUSED(dt);
  
  // вызывать метод wantRedraw родительского меню можно только, если на экране
  // текущее меню, иначе - слишком частые отрисовки могут быть.
  // поэтому вызываем notifyMenuUpdated только тогда, когда были изменения.
 
  bool lastWO = windowsFlags.isWindowsOpen;
  bool lastWAM = windowsFlags.isWindowsAutoMode;
  
  windowsFlags.isWindowsOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
  windowsFlags.isWindowsAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);

  bool anyChangesFound = (windowsFlags.isWindowsOpen != lastWO) || (lastWAM != windowsFlags.isWindowsAutoMode);
  
  
  if(anyChangesFound)
    menu->notifyMenuUpdated(this);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowMenuItem::draw(DrawContext* dc)
{
  // вычисляем, с каких позициях нам рисовать наши иконки
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-MENU_BITMAP_SIZE)/2;

  static const __FlashStringHelper* captions[] = 
  {
     LCD_OPEN_LABEL
    ,LCD_CLOSE_LABEL
    ,LCD_AUTO_LABEL    
  };

 // рисуем три иконки невыбранных чекбоксов  - пока
 for(int i=0;i<itemsCount;i++)
 {
 int cur_top = 20;
  const unsigned char* cur_icon = UNCHECK_ICON;
    if(i == 0)
    {
      if(windowsFlags.isWindowsOpen)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 1)
    {
      if(!windowsFlags.isWindowsOpen)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 2)
    {
      if(windowsFlags.isWindowsAutoMode)
         cur_icon = CHECK_ICON;
    }
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;
  dc->drawXBMP(left, cur_top, MENU_BITMAP_SIZE, MENU_BITMAP_SIZE, cur_icon);
  yield();

  // теперь рисуем текст иконки
  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  // вычисляем позицию шрифта слева
  left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;

  // рисуем заголовок
  cur_top += MENU_BITMAP_SIZE + HINT_FONT_HEIGHT;
  dc->drawStr(left, cur_top, captions[i]);
  yield();

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;
    dc->drawHLine(left,cur_top,strW);
  }
  yield();
 } // for

}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
SceneMenuItem::SceneMenuItem() : AbstractLCDMenuItem(SCENE_ICON,LCD_SCENE_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);
  
  sceneFlags.inited = false;
  currentScene = 0;
  scenesCount = 0;
  
  itemsCount = 2;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool SceneMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2)) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;

  if(!scenesCount) // нет сценариев
    return false;


    if(dir != 0)
    {
      if(cursorPos == 0) // листают сценарии
      {
        currentScene += dir;
        if(currentScene < 0)
          currentScene = scenesCount - 1;
          
        if(currentScene  >= scenesCount)
          currentScene = 0;
  
        SceneSettings ss = Scenes->GetSceneSettings(currentScene);
        displayName = ss.sceneName;
      }
      else
      if(cursorPos == 1) // крутят на выделенной кнопке пуск или стоп
      {        
          if(Scenes->IsSceneActive(currentScene))
          {
            Scenes->StopScene(currentScene);
          }
          else
          {
            Scenes->ExecuteScene(currentScene);
          }        
      }
      
    }

    menu->wantRedraw();

    return true; // сами обработали смену позиции энкодера
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneMenuItem::update(uint16_t dt, LCDMenu* menu)
{
  UNUSED(dt);

  if(!sceneFlags.inited)
  {
    sceneFlags.inited = true;
    scenesCount = Scenes->GetScenesCount();
    currentScene = 0;
    if(scenesCount > 0)
    {
      SceneSettings ss = Scenes->GetSceneSettings(currentScene);
      displayName = ss.sceneName;
    }
    else
      itemsCount = 0;
    
  } 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneMenuItem::draw(DrawContext* dc)
{
  if(!scenesCount) // нечего рисовать
    return;
    
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  int cur_top = 14 + MENU_BITMAP_SIZE;
  u8g_uint_t strW = dc->getStrWidth(displayName.c_str());
  int left = (frame_width - strW)/2 + CONTENT_PADDING;

  dc->drawStr(left, cur_top, displayName.c_str());
  if((flags & 2) && 0 == cursorPos)
  {
    // рисуем курсор в текущей позиции
    dc->drawHLine(left,cur_top + HINT_FONT_BOX_PADDING,strW);
  }

  // рисуем кнопку RUN или STOP

  cur_top += 14;
  String btn;
  if(Scenes->IsSceneActive(currentScene))
    btn = LCD_STOP_SCENE;
  else
    btn = LCD_RUN_SCENE;

  strW = dc->getStrWidth(btn.c_str());
  left = (frame_width - strW)/2 + CONTENT_PADDING;
  dc->drawStr(left, cur_top, btn.c_str());

  if((flags & 2) && 1 == cursorPos)
  {
    // рисуем курсор в текущей позиции
    dc->drawHLine(left,cur_top + HINT_FONT_BOX_PADDING,strW);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SCENE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_WATERING_MODULE) && defined(WATER_CHANNELS_SCREEN_ENABLED)
//--------------------------------------------------------------------------------------------------------------------------------------
WateringChannelsMenuItem::WateringChannelsMenuItem() : AbstractLCDMenuItem(WATERING_CHANNELS_ICON,LCD_WATERING_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringChannelsMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);

  currentSelectedChannel = 0; // выбран первый канал полива
  itemsCount = 3; // у нас три блока, которые могут получать фокус ввода
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringChannelsMenuItem::draw(DrawContext* dc)
{

  // вычисляем, с каких позициях нам рисовать наши иконки
  const int text_field_width = HINT_FONT_HEIGHT*3 + HINT_FONT_BOX_PADDING*2;
  const int text_field_height = HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING*3;
  
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-text_field_width)/2;

  static const __FlashStringHelper* captions[3] = 
  {
     LCD_CHANNEL_LABEL
    ,LCD_ON_LABEL
    ,LCD_OFF_LABEL
   
  };

  ControllerState state = WORK_STATUS.GetState();

 // рисуем наши поля ввода
 for(int i=0;i<itemsCount;i++)
 {
  int cur_top = 24;
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;

  bool isChannelOn = state.WaterChannelsState & (1 << currentSelectedChannel);
  
  bool canDrawFrame = (i < 1) || (i == 1 && isChannelOn) || (i == 2 && !isChannelOn);  // выясняем, надо ли рисовать рамку вокруг поля ввода


  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  u8g_uint_t frameWidth = i == 0 ? text_field_width : (strW + HINT_FONT_BOX_PADDING*3);
  
  if(canDrawFrame)
    dc->drawFrame(left, cur_top, frameWidth, text_field_height);
    
  yield();

  // теперь рисуем текст в полях ввода
  String tmp;
  if(i == 0)
   tmp = currentSelectedChannel + 1;
  else
    tmp = captions[i];
    
  cur_top += HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING;
  left += HINT_FONT_BOX_PADDING*2;
  dc->drawStr(left,cur_top,tmp.c_str());
  yield();

  if(i < 1)
  {
    // теперь рисуем текст под полем ввода, только для первого итема
  
    // вычисляем позицию шрифта слева
    left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;
  
    // рисуем заголовок
    cur_top += text_field_height;
    dc->drawStr(left, cur_top, captions[i]);
    yield();
  }

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;

    // если мы на состоянии канала - то в зависимости от наличия бокса вокруг текущего состояния мы сдвигаем курсор ниже, чтобы бокс его не перекрывал
    if(i > 0 && ((i == 1 && isChannelOn) || ((i == 2 && !isChannelOn))) )
      cur_top += HINT_FONT_BOX_PADDING*2;
    
    dc->drawHLine(left,cur_top,strW);
    yield();
  }
  
 } // for
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WateringChannelsMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;
    
    if(dir != 0)
    {
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // меняют номер канала
          {
            currentSelectedChannel += dir;

            if(currentSelectedChannel < 0)
              currentSelectedChannel = WATER_RELAYS_COUNT-1;

            if(currentSelectedChannel >= WATER_RELAYS_COUNT)
              currentSelectedChannel = 0;

            menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
              
          }
          break;
          
          case 1: // включить полив на канале
          {
            //Тут посылаем команду на включение полива
            String cmd = F("WATER|ON|");
            cmd += currentSelectedChannel;
            ModuleInterop.QueryCommand(ctSET,cmd,false);
            yield();

             menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
          }
          break;
          
          case 2: // выключить полив на канале
          {
            //Тут посылаем команду на выключение полива
            String cmd = F("WATER|OFF|");
            cmd += currentSelectedChannel;
            ModuleInterop.QueryCommand(ctSET,cmd,false);
            yield();

             menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
          }
          break;
        
       } // switch
    }
       
    return true; // сами обработали смену позиции энкодера
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringChannelsMenuItem::update(uint16_t dt, LCDMenu* menu)
{
  UNUSED(dt);
  UNUSED(menu);

}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // WateringChannelsMenuItem
//--------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_TEMP_SENSORS) && defined(WINDOWS_CHANNELS_SCREEN_ENABLED)
//--------------------------------------------------------------------------------------------------------------------------------------
WindowsChannelsMenuItem::WindowsChannelsMenuItem() : AbstractLCDMenuItem(WINDOWS_CHANNELS_ICON,LCD_WINDOWS_CHANNELS_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowsChannelsMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);

  currentSelectedChannel = 0; // выбран первый канал окон
  itemsCount = 3; // у нас три блока, которые могут получать фокус ввода
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowsChannelsMenuItem::draw(DrawContext* dc)
{

  // вычисляем, с каких позициях нам рисовать наши иконки
  const int text_field_width = HINT_FONT_HEIGHT*3 + HINT_FONT_BOX_PADDING*2;
  const int text_field_height = HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING*3;
  
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-text_field_width)/2;

  static const __FlashStringHelper* captions[3] = 
  {
     LCD_WINDOW_LABEL
    ,LCD_OPEN_LABEL
    ,LCD_CLOSE_LABEL
   
  };

  //ControllerState state = WORK_STATUS.GetState();

 // рисуем наши поля ввода
 for(int i=0;i<itemsCount;i++)
 {
  int cur_top = 24;
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;

  bool isChannelOn = WindowModule->IsWindowOpen(currentSelectedChannel); // спрашиваем, открыто ли окно (или открывается)
  
  bool canDrawFrame = (i < 1) || (i == 1 && isChannelOn) || (i == 2 && !isChannelOn);  // выясняем, надо ли рисовать рамку вокруг поля ввода

  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  u8g_uint_t frameWidth = i == 0 ? text_field_width : (strW + HINT_FONT_BOX_PADDING*3);
  
  if(canDrawFrame)
    dc->drawFrame(left, cur_top, frameWidth, text_field_height);
    
  yield();

  // теперь рисуем текст в полях ввода
  String tmp;
  if(i == 0)
   tmp = currentSelectedChannel + 1;
  else
    tmp = captions[i];
    
  cur_top += HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING;
  left += HINT_FONT_BOX_PADDING*2;
  dc->drawStr(left,cur_top,tmp.c_str());
  yield();
  
  if(i < 1)
  {
    // теперь рисуем текст под полем ввода, только для первого итема
  
    // вычисляем позицию шрифта слева
    left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;
  
    // рисуем заголовок
    cur_top += text_field_height;
    dc->drawStr(left, cur_top, captions[i]);
    yield();
  }

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;

    // если мы на состоянии канала - то в зависимости от наличия бокса вокруг текущего состояния мы сдвигаем курсор ниже, чтобы бокс его не перекрывал
    if(i > 0 && ((i == 1 && isChannelOn) || ((i == 2 && !isChannelOn))) )
      cur_top += HINT_FONT_BOX_PADDING*2;
    
    dc->drawHLine(left,cur_top,strW);
    yield();
  }
  
 } // for
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WindowsChannelsMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;
    
    if(dir != 0)
    {
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // меняют номер канала
          {
            currentSelectedChannel += dir;

            if(currentSelectedChannel < 0)
              currentSelectedChannel = SUPPORTED_WINDOWS-1;

            if(currentSelectedChannel >= SUPPORTED_WINDOWS)
              currentSelectedChannel = 0;

            menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
              
          }
          break;
          
          case 1: // открыть окно
          {
            //Тут посылаем команду на открытие окна
            String cmd = F("STATE|WINDOW|");
            cmd += currentSelectedChannel;
            cmd += F("|OPEN");
            
            ModuleInterop.QueryCommand(ctSET,cmd,false);
            yield();

             menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
          }
          break;
          
          case 2: // закрыть окно
          {
            //Тут посылаем команду на закрытие окна
            String cmd = F("STATE|WINDOW|");
            cmd += currentSelectedChannel;
            cmd += F("|CLOSE");
            ModuleInterop.QueryCommand(ctSET,cmd,false);
            yield();

             menu->wantRedraw(); // изменили внутреннее состояние, просим перерисоваться
          }
          break;
        
       } // switch
    }
       
    return true; // сами обработали смену позиции энкодера
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WindowsChannelsMenuItem::update(uint16_t dt, LCDMenu* menu)
{
  UNUSED(dt);
  UNUSED(menu);
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // WindowsChannelsMenuItem
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE
WateringMenuItem::WateringMenuItem() : AbstractLCDMenuItem(WATERING_ICON,LCD_ALLWATERING_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);
 
  waterFlags.isWateringOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
  waterFlags.isWateringAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);
  
  itemsCount = 3;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringMenuItem::draw(DrawContext* dc)
{
  // вычисляем, с каких позициях нам рисовать наши иконки
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-MENU_BITMAP_SIZE)/2;

  static const __FlashStringHelper* captions[] = 
  {
     LCD_ON_LABEL
    ,LCD_OFF_LABEL
    ,LCD_AUTO_LABEL
  };

 // рисуем три иконки невыбранных чекбоксов  - пока
 for(int i=0;i<itemsCount;i++)
 {
 int cur_top = 20;
  const unsigned char* cur_icon = UNCHECK_ICON;
    if(i == 0)
    {
      if(waterFlags.isWateringOn)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 1)
    {
      if(!waterFlags.isWateringOn)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 2)
    {
      if(waterFlags.isWateringAutoMode)
         cur_icon = CHECK_ICON;
    }
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;
  dc->drawXBMP(left, cur_top, MENU_BITMAP_SIZE, MENU_BITMAP_SIZE, cur_icon);
  yield();

  // теперь рисуем текст иконки
  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  // вычисляем позицию шрифта слева
  left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;

  // рисуем заголовок
  cur_top += MENU_BITMAP_SIZE + HINT_FONT_HEIGHT;
  dc->drawStr(left, cur_top, captions[i]);
  yield();

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;
    dc->drawHLine(left,cur_top,strW);
    yield();
  }
 } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WateringMenuItem::update(uint16_t dt, LCDMenu* menu)
{
  UNUSED(dt);
 
  // вызывать метод wantRedraw родительского меню можно только, если на экране
  // текущее меню, иначе - слишком частые отрисовки могут быть.
  // поэтому вызываем notifyMenuUpdated только тогда, когда были изменения.

  bool lastWO = waterFlags.isWateringOn;
  bool lastWAM = waterFlags.isWateringAutoMode;

  waterFlags.isWateringOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
  waterFlags.isWateringAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);

  
  bool anyChangesFound = (waterFlags.isWateringOn != lastWO) || (waterFlags.isWateringAutoMode != lastWAM);
 
  if(anyChangesFound)
    menu->notifyMenuUpdated(this);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool WateringMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;

    bool lastWO = waterFlags.isWateringOn;
    bool lastWAM = waterFlags.isWateringAutoMode;

    if(dir != 0)
    {
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // включить полив
          {
            waterFlags.isWateringOn = true;
            //Тут посылаем команду на включение полива
            ModuleInterop.QueryCommand(ctSET,F("WATER|ON"),false);
            yield();
          }
          break;
          
          case 1: // выключить полив
          {
            waterFlags.isWateringOn = false;
            //Тут посылаем команду на выключение полива
            ModuleInterop.QueryCommand(ctSET,F("WATER|OFF"),false);
            yield();
          }
          break;
          
          case 2: // поменять режим
          {
            waterFlags.isWateringAutoMode = !waterFlags.isWateringAutoMode;
            //Тут посылаем команду на смену режима полива
            if(waterFlags.isWateringAutoMode)
              ModuleInterop.QueryCommand(ctSET,F("WATER|MODE|AUTO"),false);
            else
              ModuleInterop.QueryCommand(ctSET,F("WATER|MODE|MANUAL"),false);

            yield();
          }
          break;
        
       } // switch
    }

    if(lastWO != waterFlags.isWateringOn || lastWAM != waterFlags.isWateringAutoMode) // состояние изменилось, просим меню перерисоваться
      menu->wantRedraw();

    return true; // сами обработали смену позиции энкодера
  
}

#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
LuminosityMenuItem::LuminosityMenuItem() : AbstractLCDMenuItem(LUMINOSITY_ICON,LCD_LIGHT_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LuminosityMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);

  lumFlags.isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
  lumFlags.isLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
  
  itemsCount = 3;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LuminosityMenuItem::draw(DrawContext* dc)
{
  // вычисляем, с каких позициях нам рисовать наши иконки
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-MENU_BITMAP_SIZE)/2;

  static const __FlashStringHelper* captions[] = 
  {
     LCD_ON_LABEL
    ,LCD_OFF_LABEL
    ,LCD_AUTO_LABEL   
  };

 // рисуем три иконки невыбранных чекбоксов  - пока
 for(int i=0;i<itemsCount;i++)
 {
 int cur_top = 20;
  const unsigned char* cur_icon = UNCHECK_ICON;
    if(i == 0)
    {
      if(lumFlags.isLightOn)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 1)
    {
      if(!lumFlags.isLightOn)
        cur_icon = RADIO_CHECK_ICON;
      else
        cur_icon = RADIO_UNCHECK_ICON;
    }
    else
    if(i == 2)
    {
      if(lumFlags.isLightAutoMode)
         cur_icon = CHECK_ICON;
    }
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;
  dc->drawXBMP(left, cur_top, MENU_BITMAP_SIZE, MENU_BITMAP_SIZE, cur_icon);
  yield();

  // теперь рисуем текст иконки
  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  // вычисляем позицию шрифта слева
  left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;

  // рисуем заголовок
  cur_top += MENU_BITMAP_SIZE + HINT_FONT_HEIGHT;
  dc->drawStr(left, cur_top, captions[i]);
  yield();

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;
    dc->drawHLine(left,cur_top,strW);
    yield();
  }

 } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LuminosityMenuItem::update(uint16_t dt, LCDMenu* menu)
{
 UNUSED(dt);
 
  // вызывать метод wantRedraw родительского меню можно только, если на экране
  // текущее меню, иначе - слишком частые отрисовки могут быть.
  // поэтому вызываем notifyMenuUpdated только тогда, когда были изменения.

  bool lastLO = lumFlags.isLightOn;
  bool lastLAM = lumFlags.isLightAutoMode;
  
  lumFlags.isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
  lumFlags.isLightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);

  
  bool anyChangesFound = (lumFlags.isLightOn != lastLO) || (lumFlags.isLightAutoMode != lastLAM);
 
  if(anyChangesFound)
    menu->notifyMenuUpdated(this);  
}
bool LuminosityMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;

    bool lastLO = lumFlags.isLightOn;
    bool lastLAM = lumFlags.isLightAutoMode;

    if(dir != 0)
    {
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // включить досветку
          {
            lumFlags.isLightOn = true;
            //Тут посылаем команду на включение досветки
            ModuleInterop.QueryCommand(ctSET,F("LIGHT|ON"),false);
            yield();
          }
          break;
          
          case 1: // выключить досветку
          {
            lumFlags.isLightOn = false;
            //Тут посылаем команду на выключение досветки
            ModuleInterop.QueryCommand(ctSET,F("LIGHT|OFF"),false);
            yield();
          }
          break;
          
          case 2: // поменять режим
          {
            lumFlags.isLightAutoMode = !lumFlags.isLightAutoMode;
            //Тут посылаем команду на смену режима досветки
            if(lumFlags.isLightAutoMode)
              ModuleInterop.QueryCommand(ctSET,F("LIGHT|MODE|AUTO"),false);
            else
              ModuleInterop.QueryCommand(ctSET,F("LIGHT|MODE|MANUAL"),false);

            yield();
          }
          break;
        
       } // switch
    }

    if(lastLO != lumFlags.isLightOn || lastLAM != lumFlags.isLightAutoMode) // состояние изменилось, просим меню перерисоваться
      menu->wantRedraw();

    return true; // сами обработали смену позиции энкодера
  
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
SettingsMenuItem::SettingsMenuItem() : AbstractLCDMenuItem(SETTINGS_ICON,LCD_SETTINGS_CAPTION)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsMenuItem::init(LCDMenu* parent)
{
  AbstractLCDMenuItem::init(parent);

  GlobalSettings* s = MainController->GetSettings();
  
  openTemp = s->GetOpenTemp();
  closeTemp = s->GetCloseTemp();
  openInterval = s->GetOpenInterval();
  
  itemsCount = 3;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsMenuItem::draw(DrawContext* dc)
{
  // вычисляем, с каких позициях нам рисовать наши иконки
  const int text_field_width = HINT_FONT_HEIGHT*3 + HINT_FONT_BOX_PADDING*2;
  const int text_field_height = HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING*3;
  
  const int frame_width = FRAME_WIDTH - CONTENT_PADDING*2;
  const int one_icon_box_width = frame_width/itemsCount;
  const int one_icon_left_spacing = (one_icon_box_width-text_field_width)/2;

  static const __FlashStringHelper* captions[] = 
  {
     LCD_TOPEN_CAPTION
    ,LCD_TCLOSE_CAPTION
    ,LCD_INTERVAL_CAPTION
   
  };

 // рисуем наши поля ввода
 for(int i=0;i<itemsCount;i++)
 {
 int cur_top = 24;
  int left = i*CONTENT_PADDING + i*one_icon_box_width + one_icon_left_spacing;
  dc->drawFrame(left, cur_top, text_field_width, text_field_height);
  yield();

  // теперь рисуем текст в полях ввода
  String tmp;
  if(i == 1)
    tmp = String(closeTemp);
  else if(i==0) 
    tmp = String(openTemp);
 else
 {
   unsigned long tmpInterval = openInterval/1000;
   tmp = tmpInterval;
 }
    
  cur_top += HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING;
  left += HINT_FONT_BOX_PADDING*2;
  dc->drawStr(left,cur_top,tmp.c_str());
  yield();

  // теперь рисуем текст под полем ввода
  u8g_uint_t strW = dc->getStrWidth(captions[i]);

  // вычисляем позицию шрифта слева
  left =  i*CONTENT_PADDING + i*one_icon_box_width + (one_icon_box_width - strW)/2;

  // рисуем заголовок
  cur_top += text_field_height;
  dc->drawStr(left, cur_top, captions[i]);
  yield();

  if(/*needToDrawCursor*/ (flags & 2) && i == cursorPos)
  {
    // рисуем курсор в текущей позиции
    cur_top += HINT_FONT_BOX_PADDING;
    dc->drawHLine(left,cur_top,strW);
    yield();
  }
  
 } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsMenuItem::update(uint16_t dt, LCDMenu* menu)
{
 UNUSED(dt);
 
  // вызывать метод wantRedraw родительского меню можно только, если на экране
  // текущее меню, иначе - слишком частые отрисовки могут быть.
  // поэтому вызываем notifyMenuUpdated только тогда, когда были изменения.
  GlobalSettings* s = MainController->GetSettings();

 uint8_t lastOT = openTemp;
 uint8_t lastCT = closeTemp;
 unsigned long lastOpenInterval = openInterval; 
  
  openTemp = s->GetOpenTemp();
  closeTemp = s->GetCloseTemp();
  openInterval = s->GetOpenInterval();
  
  bool anyChangesFound = (lastOT != openTemp) || (lastCT != closeTemp) || (lastOpenInterval != openInterval);

  if(anyChangesFound)
    menu->notifyMenuUpdated(this);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsMenuItem::setFocus(bool f)
{
  bool lastFocus = flags & 1;//focused;
  AbstractLCDMenuItem::setFocus(f);
  
  if(lastFocus && !f)
  {
    // был фокус и мы его потеряли, значит, надо сохранить настройки
    GlobalSettings* s = MainController->GetSettings();
    s->SetOpenTemp(openTemp);
    s->SetCloseTemp(closeTemp);
    s->SetOpenInterval(openInterval);
    //s->Save();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool SettingsMenuItem::OnEncoderPositionChanged(int dir, LCDMenu* menu)
{
  if(!(flags & 2))//needToDrawCursor) // курсор не нарисован, значит, нам не надо обрабатывать смену настройки с помощью энкодера
    return false;

    uint8_t lastOT = openTemp;
    uint8_t lastCT = closeTemp;
    unsigned long lastOpenInterval = openInterval;

    if(dir != 0)
    {
      GlobalSettings* s = MainController->GetSettings();
      
       // есть смена позиции энкодера, смотрим, какой пункт у нас выбран
       switch(cursorPos)
       {
          case 0: // поменять температуру открытия
          {
            openTemp += dir;
            if(openTemp > SCREEN_MAX_TEMP_VALUE)
            {
              openTemp = dir > 0 ? 0 : SCREEN_MAX_TEMP_VALUE;
            }
            s->SetOpenTemp(openTemp);
          }
          break;
          
          case 1: // поменять температуру закрытия
          {
            closeTemp += dir;
            if(closeTemp > SCREEN_MAX_TEMP_VALUE)
            {
              closeTemp = dir > 0 ? 0 : SCREEN_MAX_TEMP_VALUE;
            }
            s->SetCloseTemp(closeTemp);
          }
          break;
          
          case 2: // поменять время работы моторов
          {
            long change = dir;
            change *=1000;
            
            openInterval += change;
            
            if(openInterval >= 999000)
            {
              openInterval = dir > 0 ? 0 : 999000;
            }
              
            s->SetOpenInterval(openInterval);
          }
          break;        
       } // switch
    }

    if(lastOT != openTemp || lastCT != closeTemp || openInterval != lastOpenInterval) // состояние изменилось, просим меню перерисоваться
      menu->wantRedraw();

    return true; // сами обработали смену позиции энкодера
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
LCDMenu::LCDMenu(uint8_t sck, uint8_t mosi, uint8_t cs) :
#ifdef SCREEN_USE_SOFT_SPI
DrawContext(sck,mosi,cs)
#else
DrawContext(cs)
#endif
{ 
#ifndef SCREEN_USE_SOFT_SPI
  UNUSED(sck);
  UNUSED(mosi);
#else
  WORK_STATUS.PinMode(sck,OUTPUT,false);
  WORK_STATUS.PinMode(mosi,OUTPUT,false);
#endif

  WORK_STATUS.PinMode(cs,OUTPUT,false);
  
  wantRedraw(); // говорим, что мы хотим перерисоваться

#ifdef FLIP_SCREEN  
  setRot180(); // переворачиваем экран, если нас попросили в настройках
#endif  

  // добавляем экран ожидания
  items.push_back(&IdleScreen);
  
#ifdef USE_TEMP_SENSORS
  // добавляем экран управления окнами
  items.push_back(&WindowManageScreen);
#endif

#if defined(USE_TEMP_SENSORS) && defined(WINDOWS_CHANNELS_SCREEN_ENABLED)
  // добавляем экран управления каналами окон
  items.push_back(&WindowsChannelsManageScreen);
#endif
  
 #ifdef USE_WATERING_MODULE
  // добавляем экран управления поливом
  items.push_back(&WateringManageScreen);
#endif

 #if defined(USE_WATERING_MODULE) && defined(WATER_CHANNELS_SCREEN_ENABLED)
  // добавляем экран управления каналами полива
  items.push_back(&WateringChannelsManageScreen);
#endif

#ifdef USE_LUMINOSITY_MODULE  
  // добавляем экран управления досветкой
  items.push_back(&LuminosityManageScreen);
#endif

#ifdef USE_SCENE_MODULE
  items.push_back(&SceneManageScreen);
#endif
  
  // добавляем экран управления настройками
  items.push_back(&SettingsManageScreen);

  resetTimer(); // сбрасываем таймер ничегонеделания
  selectedMenuItem = 0; // говорим, что выбран первый пункт меню

}
//--------------------------------------------------------------------------------------------------------------------------------------
LCDMenu::~LCDMenu()
{
  //чистим за собой
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::wantRedraw()
{
  flags.needRedraw = true; // выставляем флаг необходимости перерисовки
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::resetTimer()
{
  gotLastCommmandAt = 0; // сбрасываем таймер простоя
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::selectNextMenu(int encoderDirection)
{
  if(!encoderDirection) // не было изменений позиции энкодера
    return;

  resetTimer();   // сбрасываем таймер простоя
  backlight(); // включаем подсветку

  uint8_t lastSelMenu = selectedMenuItem; // запоминаем текущий экран

  //Тут проверяем - не захватил ли экран управление (для изменения своих настроек, например)
  AbstractLCDMenuItem* mi = items[selectedMenuItem];
  if(mi->hasFocus())
  {
    // подменю имеет фокус, значит, оно само должно обработать новое положение энкодера.
    // если оно обработало новое положение энкодера - значит, мы ничего обрабатывать не должны.
    // если экран не хочет обрабатывать положение энкодера - то он возвращает false,
    // и мы обрабатываем положение энкодера сами

    if(mi->OnEncoderPositionChanged(encoderDirection,this))
      return;
      
  } // if(mi->hasFocus())
    
  if(encoderDirection > 0) // двигаемся вперёд
  {
    selectedMenuItem++; // двигаемся к следующему пункту меню
    
    if(selectedMenuItem >= items.size()) // если дошли до конца - заворачиваем в начало
      selectedMenuItem = 0;
  }
  else // двигаемся назад
  {
    if(selectedMenuItem) // если мы не дошли до нулевого пункта
      selectedMenuItem--; // доходим до него
    else
      selectedMenuItem = items.size() - 1; // иначе заворачиваем в самый конец меню
  }

  if(lastSelMenu != selectedMenuItem) // если пункт меню на экране изменился
  {
    items[lastSelMenu]->setFocus(false); // сбрасываем фокус с последнего выбранного меню
    wantRedraw(); // просим перерисоваться
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::backlight(bool en)
{
  flags.backlightIsOn = en;
  analogWrite(SCREEN_BACKLIGHT_PIN, en ? SCREEN_BACKLIGHT_INTENSITY : 0);
 
  flags.backlightCheckingEnabled = false;
  backlightCounter = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::init()
{
  // устанавливаем выбранный шрифт
  setFont(SELECTED_FONT);

  // инициализируем пин подсветки
  WORK_STATUS.PinMode(SCREEN_BACKLIGHT_PIN,OUTPUT);
  backlight(); // включаем подсветку экрана
  
  // инициализируем кнопку
  button.init(this,ButtonOnClick);

  // инициализируем пункты меню
  size_t cnt = items.size();
  for(size_t i=0;i<cnt;i++)
    items[i]->init(this);

}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::notifyMenuUpdated(AbstractLCDMenuItem* miUpd)
{
  AbstractLCDMenuItem* mi = items[selectedMenuItem];
  if(mi == miUpd)
    wantRedraw(); // пункт меню, который изменился - находится на экране, надо перерисовать его состояние
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::enterSubMenu() // переходим в подменю по клику на кнопке
{
  resetTimer(); // сбрасываем таймер ничегонеделания
  backlight(); // включаем подсветку экрана
  
  AbstractLCDMenuItem* mi = items[selectedMenuItem];
  mi->OnButtonClicked(this); // говорим, что кликнута кнопка, затем устанавливаем фокус на окне.
  // если фокуса раньше не было - окно поймёт, что это первый клик на кнопке,
  // в противном случае - повторный.
  mi->setFocus(); // устанавливаем фокус на текущем экране, после этого все позиции на этом экране 
  // могут листаться энкодером, помимо кнопки
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::update(uint16_t dt)
{
  // обновляем кнопку
  button.update();

  // обновляем все экраны
  size_t cnt = items.size();
  for(size_t i=0;i<cnt;i++)
    items[i]->update(dt,this);
  
  // обновляем внутренний таймер ничегонеделания
  gotLastCommmandAt += dt;

  // обновляем таймер выключения подсветки
  if(flags.backlightCheckingEnabled)
  {
    backlightCounter += dt;
    if(backlightCounter >= SCREEN_BACKLIGHT_OFF_DELAY) // надо выключить подсветку
      backlight(false);
  }
  
  if(gotLastCommmandAt >= MENU_RESET_DELAY)
  {
     // ничего не делали какое-то время, надо перейти в экран ожидания
     resetTimer(); // сбрасываем таймер простоя

     //Убираем захват фокуса предыдущим выбранным пунктом меню
     items[selectedMenuItem]->setFocus(false); // сбрасываем фокус с окна
     
     if(selectedMenuItem != 0) // если до этого был выбран не первый пункт меню - просим перерисоваться
      wantRedraw();
      
     selectedMenuItem = 0; // выбираем первый пункт меню
     flags.backlightCheckingEnabled = true; // включаем таймер выключения досветки

  } // if
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool LCDMenu::HasSensorsSettingsOnSD()
{
   for(byte i=DIR_TEMP;i<DIR_DUMMY_LAST_DIR;i++)
   {
    if(GetFilesCount(i) > 0)
      return true;
   }

   return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String LCDMenu::GetFileContent(byte directory,byte fileIndex, int& resultSensorIndex)
{
  String result;
  resultSensorIndex = -1;
  
  #ifdef SENSORS_SETTINGS_ON_SD_ENABLED

  
  String folderName = GetFolderName(directory);
  const char* dirP = folderName.c_str();
    
    SdFile dir;
    if(dir.open(dirP, O_READ)) 
    {
      
        dir.rewind();
        SdFile workFile;
        for(int i=0;i<fileIndex;i++)
        {
          if(workFile.isOpen())
            workFile.close();

          workFile.openNext(&dir,O_READ);
            
          if(!workFile.isOpen())
            break;
        } // for

        if(workFile.isOpen())
        {
          // получаем индекс датчика (он является именем файла до расширения)
          String idx;
          String strFileName = FileUtils::GetFileName(workFile);
          const char* fName = strFileName.c_str();
          while(*fName && *fName != '.')
            idx += *fName++;

          resultSensorIndex = idx.toInt();
                      
          uint32_t sz = workFile.fileSize();
          if(sz > 0)
          {
              char* toRead = new char[sz+1];
              toRead[sz] = '\0';
              workFile.read(toRead,sz);
              result = toRead;
          
              delete [] toRead;

          }
          workFile.close();
        } // if(workFile)


        dir.close();
    }


  #endif

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::DoRemoveFiles(const String& dirName)
{
  FileUtils::RemoveFiles(dirName);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::ClearSDSensors()
{
  #ifdef SENSORS_SETTINGS_ON_SD_ENABLED
    if(MainController->HasSDCard())
      DoRemoveFiles("LCD");
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::AddSDSensor(byte folder,byte sensorIndex,const String& strCaption)
{
 #ifdef SENSORS_SETTINGS_ON_SD_ENABLED
  if(MainController->HasSDCard())
  {
    String folderName = GetFolderName(folder);
    SDFat.mkdir(folderName.c_str());
   
      folderName += F("/");
      folderName += String(sensorIndex);
      folderName += F(".INF");

      SdFile outFile;

      if(outFile.open(folderName.c_str(),FILE_WRITE | O_TRUNC))
      {
        outFile.write((byte*)strCaption.c_str(),strCaption.length());
        outFile.close();
      }
        
  }
 #endif 
}
//--------------------------------------------------------------------------------------------------------------------------------------
String LCDMenu::GetFolderName(byte directory)
{
  String folderName = "LCD"; // эта строка уже в оперативке, т.к. является именем модуля
  folderName += F("/");

  switch(directory)
  {
      case DIR_TEMP:
        folderName += F("TEMP");
      break;

      case DIR_HUMIDITY:
        folderName += "HUMIDITY";
      break;

      case DIR_LUMINOSITY:
        folderName += "LIGHT";
      break;

      case DIR_SOIL:
        folderName += "SOIL";
      break;

      case DIR_PH:
        folderName += "PH";
      break;

  } // switch

  return folderName;
}
//--------------------------------------------------------------------------------------------------------------------------------------
byte LCDMenu::GetFilesCount(byte directory)
{
    #ifdef SENSORS_SETTINGS_ON_SD_ENABLED

      if(!MainController->HasSDCard()) // нет SD-карты, деградируем в жёстко прошитые настройки
        return 0;
      else
      {
          // подсчитываем кол-во файлов в папке
          String folderName = GetFolderName(directory); // эта строка уже в оперативке, т.к. является именем модуля
          return FileUtils::CountFiles(folderName,false);
        
      } // else
      
    
    #else
      UNUSED(directory);
      return 0;
    #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDMenu::draw()
{
if(!flags.needRedraw || !flags.backlightIsOn) // не надо ничего перерисовывать
  return;
    
 size_t sz = items.size();
 AbstractLCDMenuItem* selItem = items[selectedMenuItem];
 const char* capt = selItem->GetCaption();
 
 firstPage();  
  do 
  {
   yield();
    // рисуем бокс
    drawFrame(0,MENU_BITMAP_SIZE-1,FRAME_WIDTH,FRAME_HEIGHT+1);
    
    // рисуем пункты меню верхнего уровня
    for(size_t i=0;i<sz;i++)
    {
      yield();
      drawXBMP( i*MENU_BITMAP_SIZE, 0, MENU_BITMAP_SIZE, MENU_BITMAP_SIZE, items[i]->GetIcon());
    }
    
    // теперь рисуем фрейм вокруг выбранного пункта меню
    drawFrame(selectedMenuItem*MENU_BITMAP_SIZE,0,MENU_BITMAP_SIZE,MENU_BITMAP_SIZE);
    yield();
    
    // теперь рисуем прямоугольник с заливкой внизу от контента
    drawBox(0,FRAME_HEIGHT + MENU_BITMAP_SIZE - (HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING),FRAME_WIDTH,HINT_FONT_HEIGHT + HINT_FONT_BOX_PADDING);
    yield();
    
    setColorIndex(0);

    // теперь убираем линию под выбранным пунктом меню
    drawLine(selectedMenuItem*MENU_BITMAP_SIZE+1,MENU_BITMAP_SIZE-1,selectedMenuItem*MENU_BITMAP_SIZE+MENU_BITMAP_SIZE-2,MENU_BITMAP_SIZE-1);
    yield();
    
    // теперь рисуем название пункта меню
    
    #ifdef SCREEN_HINT_AT_RIGHT
          
    // рисуем подсказку, выровненную по правому краю
    u8g_uint_t strW = getStrWidth(capt);    
    drawStr(FRAME_WIDTH - HINT_FONT_BOX_PADDING - strW,FRAME_HEIGHT + MENU_BITMAP_SIZE - HINT_FONT_BOX_PADDING,capt);
    yield();
    
    #else
    // рисуем подсказку, выровненную по левому краю
    drawStr(HINT_FONT_BOX_PADDING,FRAME_HEIGHT + MENU_BITMAP_SIZE - HINT_FONT_BOX_PADDING,capt);
    yield();
    
    #endif

    setColorIndex(1);

    // теперь просим пункт меню отрисоваться на экране
    selItem->draw(this);
    yield();  
  
  
  } while( nextPage() ); 

   flags.needRedraw = false; // отрисовали всё, что нам надо - и сбросили флаг необходимости отрисовки
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif

