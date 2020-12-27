#include "UTFTMenu.h"
#include "AbstractModule.h"
#include "ModuleController.h"
#include "TempSensors.h"
#include "InteropStream.h"
#include "Memory.h"
#include "EEPROMSettingsModule.h"
#include "UniversalSensors.h"
#include "ZeroStreamListener.h"
// USE_WATER_TANK_MODULE
#include "WaterTankModule.h"
// USE_WATER_TANK_MODULE

#ifdef USE_TFT_MODULE

#ifdef USE_BUZZER
#include "Buzzer.h"
#endif

#if defined(USE_SMS_MODULE) || defined(USE_WIFI_MODULE)
  #include "CoreTransport.h"
#endif

#ifdef USE_PH_MODULE
#include "PHModule.h"
#endif

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// This code block is only needed to support multiple
// MCU architectures in a single sketch.
#if defined(__AVR__)
  #define imagedatatype  unsigned int
#elif defined(__PIC32MX__)
  #define imagedatatype  unsigned short
#elif defined(__arm__)
  #define imagedatatype  unsigned short
#endif
// End of multi-architecture block
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
static const char* WEEKDAYS[] = {

  "ПН",
  "ВТ",
  "СР",
  "ЧТ",
  "ПТ",
  "СБ",
  "ВС"

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
unsigned int utf8GetCharSize(unsigned char bt) 
{ 
  if (bt < 128) 
  return 1; 
  else if ((bt & 0xE0) == 0xC0) 
  return 2; 
  else if ((bt & 0xF0) == 0xE0) 
  return 3; 
  else if ((bt & 0xF8) == 0xF0) 
  return 4; 
  else if ((bt & 0xFC) == 0xF8) 
  return 5; 
  else if ((bt & 0xFE) == 0xFC) 
  return 6; 

 
  return 1; 
} 
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ButtonPressed(int btn)
{
  if(btn != -1)
  {
    #ifdef USE_BUZZER
    Buzzer.buzz();
    #endif
  }

  TFTScreen->onButtonPressed(btn);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void ButtonReleased(int btn)
{
  TFTScreen->onButtonReleased(btn);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawButtonsYield() // вызывается после отрисовки каждой кнопки
{
  yield();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawScreenCaption(TFTMenu* hal, const String& str) // рисуем заголовок экрана
{
  UTFT* dc = hal->getDC();

  if(!dc)
  {
    return;
  }
  
  UTFTRus* rusPrinter = hal->getRusPrinter();
  
  int screenWidth = dc->getDisplayXSize();
  uint8_t* oldFont = dc->getFont();
  word oldBackColor = dc->getBackColor();
  word oldColor = dc->getColor();
  
  dc->setFont(BigRusFont);
  
  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();
  int top = 10;

  // подложка под заголовок
  dc->setColor(VGA_NAVY);
  dc->fillRect(0, 0, screenWidth-1, top*2 + fontHeight);
  
  dc->setBackColor(VGA_NAVY);
  dc->setColor(VGA_WHITE); 
   
  int strLen = rusPrinter->utf8_strlen(str.c_str());

  int left = (screenWidth - fontWidth*strLen)/2;

  rusPrinter->print(str.c_str(),left,top);  

  dc->setFont(oldFont);
  dc->setBackColor(oldBackColor);
  dc->setColor(oldColor);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawTimeInBox(TFTInfoBox* box, uint32_t val)
{
  UTFT* dc = TFTScreen->getDC();

  if(!dc)
  {
    return;
  }
    
  UTFTRus* rusPrinter = TFTScreen->getRusPrinter();
  
  TFTInfoBoxContentRect rc =  box->getContentRect(TFTScreen);
  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRect(rc.x,rc.y,rc.x+rc.w,rc.y+rc.h);
  yield();

  uint8_t* oldFont = dc->getFont();
  dc->setFont(SevenSegNumFontPlus);
  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);

  uint8_t hours = val/60;
  uint8_t mins = val%60;

  String strVal;
  if(hours < 10)
    strVal += '0';

  strVal += hours;
  strVal += ':';

  if(mins < 10)
    strVal += '0';

  strVal += mins;

  
  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();

  int strLen = rusPrinter->utf8_strlen(strVal.c_str());

  int leftPos = rc.x + (rc.w - (strLen*fontWidth))/2;
  int topPos = rc.y + (rc.h - fontHeight)/2;
  rusPrinter->print(strVal.c_str(),leftPos,topPos);
  yield();
  dc->setFont(oldFont);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, const String& strVal, uint8_t* font)
{
  UTFT* dc = TFTScreen->getDC();

  if(!dc)
  {
    return;
  }
    
  UTFTRus* rusPrinter = TFTScreen->getRusPrinter();
  
  TFTInfoBoxContentRect rc =  box->getContentRect(TFTScreen);
  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRect(rc.x,rc.y,rc.x+rc.w,rc.y+rc.h);
  yield();

  uint8_t* oldFont = dc->getFont();
  dc->setFont(font);
  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);

  
  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();

  int strLen = rusPrinter->utf8_strlen(strVal.c_str());

  int leftPos = rc.x + (rc.w - (strLen*fontWidth))/2;
  int topPos = rc.y + (rc.h - fontHeight)/2;
  rusPrinter->print(strVal.c_str(),leftPos,topPos);
  yield();
  dc->setFont(oldFont);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, int val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, int16_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, uint16_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, int8_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, uint8_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, int32_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, uint32_t val)
{
  return drawValueInBox(box,String(val));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTInfoBox
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTInfoBox::TFTInfoBox(const char* caption, int width, int height, int x, int y, int cxo)
{
  boxCaption = caption;
  boxWidth = width;
  boxHeight = height;
  posX = x;
  posY = y;
  captionXOffset = cxo;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTInfoBox::~TFTInfoBox()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTInfoBox::drawCaption(TFTMenu* menuManager, const char* caption)
{
  UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
  uint8_t* oldFont = dc->getFont();
  dc->setFont(BigRusFont);	
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  menuManager->getRusPrinter()->print(caption,posX+captionXOffset,posY);
  dc->setFont(oldFont);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTInfoBox::draw(TFTMenu* menuManager)
{
  drawCaption(menuManager,boxCaption);
  
  int curTop = posY;

  UTFT* dc = menuManager->getDC();
  if(!dc)
  {
    return;
  } 

  uint8_t* oldFont = dc->getFont();
  dc->setFont(BigRusFont);	
  
  int fontHeight = dc->getFontYsize();
  
  curTop += fontHeight + INFO_BOX_CONTENT_PADDING;

  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRoundRect(posX, curTop, posX+boxWidth, curTop + (boxHeight - fontHeight - INFO_BOX_CONTENT_PADDING));

  yield();

  dc->setColor(INFO_BOX_BORDER_COLOR);
  dc->drawRoundRect(posX, curTop, posX+boxWidth, curTop + (boxHeight - fontHeight - INFO_BOX_CONTENT_PADDING));

  yield();
  
  dc->setFont(oldFont);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTInfoBoxContentRect TFTInfoBox::getContentRect(TFTMenu* menuManager)
{
    TFTInfoBoxContentRect result;
    UTFT* dc = menuManager->getDC();
    if(!dc)
    {
      return result;
    }    
	
	uint8_t* oldFont = dc->getFont();
	dc->setFont(BigRusFont);

    int fontHeight = dc->getFontYsize();
	
	dc->setFont(oldFont);

    result.x = posX + INFO_BOX_CONTENT_PADDING;
    result.y = posY + fontHeight + INFO_BOX_CONTENT_PADDING*2;

    result.w = boxWidth - INFO_BOX_CONTENT_PADDING*2;
    result.h = boxHeight - (fontHeight + INFO_BOX_CONTENT_PADDING*3);

    return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AbstractTFTScreen::AbstractTFTScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AbstractTFTScreen::~AbstractTFTScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CO2_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "CO2Module.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTCO2SettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCO2SettingsScreen::TFTCO2SettingsScreen()
{
  startTimeBox = NULL;
  endTimeBox = NULL;
  ppmBox = NULL;
  histeresisBox = NULL;

  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCO2SettingsScreen::~TFTCO2SettingsScreen()
{
 delete screenButtons;
 delete startTimeBox;
 delete endTimeBox;
 delete ppmBox;
 delete histeresisBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* s = MainController->GetSettings();
    
    settings = s->GetCO2Settings();

    if(settings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    // теперь смотрим, какие дни недели установлены для канала
    for(uint8_t i=0;i<7;i++)
    {
      uint8_t mask = (1 << i);
      int btnID = weekdaysButtons[i];
        
      if((settings.weekdays & mask))
      {
        screenButtons->setButtonBackColor(btnID,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      } // if
      else
      {
        screenButtons->setButtonBackColor(btnID,MODE_OFF_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      }
        
    } // for    

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

    if(!dc)
    {
      return;
    }
        
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
  //  int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);


    int weekDaysButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*6)/7;
    int weekDayLeft = initialLeftPos;

    for(int i=0;i<7;i++)
    {
      int addedID = screenButtons->addButton( weekDayLeft ,  topPos, weekDaysButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, WEEKDAYS[i]);
      weekDayLeft += weekDaysButtonWidth + INFO_BOX_V_SPACING;

      weekdaysButtons.push_back(addedID);
    }  
    

    topPos = secondRowTopPos; // со второй строки
    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки    
    decStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   startTimeBox = new TFTInfoBox("НАЧАЛО:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   endTimeBox = new TFTInfoBox("КОНЕЦ:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decPPMButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   ppmBox = new TFTInfoBox("ДЕРЖИМ PPM:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incPPMButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

  
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox("ГИСТЕРЕЗИС:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::saveSettings()
{
  GlobalSettings* s = MainController->GetSettings();
  
  s->SetCO2Settings(settings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  CO2Control->ReloadSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decStartTimeButton || buttonID == incStartTimeButton || buttonID == decEndTimeButton
  || buttonID == incEndTimeButton || buttonID == decPPMButton || buttonID == incPPMButton
  || buttonID == decHisteresisButton || buttonID == incHisteresisButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::onTick()
{
  if(tickerButton == decStartTimeButton)
    incStartTime(-7);
  else
  if(tickerButton == incStartTimeButton)
    incStartTime(7);
  else
  if(tickerButton == decEndTimeButton)
    incEndTime(-7);
  else
  if(tickerButton == incEndTimeButton)
    incEndTime(7);
  else
  if(tickerButton == decPPMButton)
    incPPM(-7);
  else
  if(tickerButton == incPPMButton)
    incPPM(7);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-7);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(7);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::incStartTime(int val)
{
  uint32_t old = settings.startTime;
  
  settings.startTime+=val;

  if(settings.startTime < 0)
    settings.startTime = 0;
  
  if(settings.startTime > 1440)
    settings.startTime = 1440;

  if(settings.startTime != old)
    drawTimeInBox(startTimeBox,settings.startTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::incEndTime(int val)
{
  uint32_t old = settings.endTime;
  
  settings.endTime+=val;

  if(settings.endTime < 0)
    settings.endTime = 0;
  
  if(settings.endTime > 1440)
    settings.endTime = 1440;

  if(settings.endTime != old)
    drawTimeInBox(endTimeBox,settings.endTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::incPPM(int val)
{
  uint32_t old = settings.ppm;
  
  settings.ppm+=val;

  if(settings.ppm < 0)
    settings.ppm = 0;
  
  if(settings.ppm > MAX_CO2_SENSOR_PPM)
    settings.ppm = MAX_CO2_SENSOR_PPM;

  if(settings.ppm != old)
    drawValueInBox(ppmBox,String(settings.ppm), SevenSegNumFontPlus);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::incHisteresis(int val)
{
  uint32_t old = settings.histeresis;
  
  settings.histeresis+=val;

  if(settings.histeresis < 0)
    settings.histeresis = 0;
  
  if(settings.histeresis > MAX_CO2_SENSOR_PPM)
    settings.histeresis = MAX_CO2_SENSOR_PPM;

  if(settings.histeresis != old)
    drawValueInBox(histeresisBox,String(settings.histeresis), SevenSegNumFontPlus);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::update(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("DRIVE");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(settings.active)
                settings.active = false;
              else
                settings.active = true;
                
              if(settings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decStartTimeButton)
          {
            incStartTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incStartTimeButton)
          {
            incStartTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decEndTimeButton)
          {
            incEndTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incEndTimeButton)
          {
            incEndTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decPPMButton)
          {
            incPPM(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incPPMButton)
          {
            incPPM(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else
          {
            // нажата одна из кнопок дней недели
            for(int i=0;i<7;i++)
            {
              if(pressed_button == weekdaysButtons[i])
              {
                word backColor = screenButtons->getButtonBackColor(pressed_button);
                if(backColor == MODE_ON_COLOR)
                {
                  backColor = MODE_OFF_COLOR;
                  settings.weekdays &= ~(1 << i);
                }
                else
                {
                  backColor = MODE_ON_COLOR;
                  settings.weekdays |= (1 << i);
                }
      
                screenButtons->setButtonBackColor(pressed_button,backColor);
                screenButtons->drawButton(pressed_button);
                
                screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
                blinkSaveSettingsButton(true);
                
                break;
              } // if
            } // for
          } // else
          
          

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCO2SettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  startTimeBox->draw(menuManager);
  drawTimeInBox(startTimeBox,settings.startTime);

  endTimeBox->draw(menuManager);
  drawTimeInBox(endTimeBox,settings.endTime);

  ppmBox->draw(menuManager);
  drawValueInBox(ppmBox,String(settings.ppm), SevenSegNumFontPlus);

  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,String(settings.histeresis), SevenSegNumFontPlus);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CO2_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWateringScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringScreen::TFTWateringScreen()
{
  inited = false;
  lastWaterChannelsState = 0;
  screenButtons = NULL;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringScreen::~TFTWateringScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringScreen::setup(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  

  #if WATER_RELAYS_COUNT > 0
  
    screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter(),25);
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

    int buttonsTop = INFO_BOX_V_SPACING;
    int screenWidth = menuManager->getDC()->getDisplayXSize();
    int screenHeight = menuManager->getDC()->getDisplayYSize();
    
    int x_spacing = (screenWidth - (ALL_CHANNELS_BUTTON_WIDTH*3) - INFO_BOX_V_SPACING*2)/2;
    int allChannelsLeft = x_spacing;

    int controlsButtonsWidth = (screenWidth - x_spacing*2 - 20)/3;

    // первая - кнопка назад
    int backButtonHeight = ALL_CHANNELS_BUTTON_HEIGHT;
    backButton = screenButtons->addButton( allChannelsLeft ,  screenHeight - backButtonHeight - INFO_BOX_V_SPACING, controlsButtonsWidth,  backButtonHeight, WM_BACK_CAPTION);
 
    // добавляем кнопки для управления всеми каналами
    screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, TURN_ON_ALL_WATER_LABEL);
    allChannelsLeft += ALL_CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;

    screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, TURN_OFF_ALL_WATER_LABEL);
    allChannelsLeft += ALL_CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;

    int addedId = screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(addedId,CHANNELS_BUTTONS_TEXT_COLOR);

    // добавляем кнопку "Пропустить полив за сегодня"
    int skipButtonTop = screenHeight - backButtonHeight - INFO_BOX_V_SPACING; // координата Y для кнопки "Назад"
    skipButton = screenButtons->addButton(x_spacing + controlsButtonsWidth + 10, skipButtonTop, controlsButtonsWidth, ALL_CHANNELS_BUTTON_HEIGHT, SKIP_WATERING_LABEL);
    channelsButton = screenButtons->addButton(x_spacing + controlsButtonsWidth*2 + 20, skipButtonTop, controlsButtonsWidth, ALL_CHANNELS_BUTTON_HEIGHT, TFT_WATERING_SETT_CAPTION);    

    buttonsTop += ALL_CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;

    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<WATER_RELAYS_COUNT;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
        curButtonLeft = computedButtonLeft;
       }
       
       String* label = new String('#');
       *label += (i+1);
       labels.push_back(label);
       
       addedId = screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  CHANNELS_BUTTON_HEIGHT, label->c_str());
       screenButtons->setButtonFontColor(addedId,CHANNELS_BUTTONS_TEXT_COLOR);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for

//    buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;


  
    #endif // WATER_RELAYS_COUNT > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

 #if WATER_RELAYS_COUNT > 0

  const byte BUTTONS_OFFSET = 6; // с какого индекса начинаются наши кнопки
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);
  
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("DRIVE");
      return;
    }

    if(pressed_button == 3)
    {
      // Кнопка смены режима
      bool waterAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);
      waterAutoMode = !waterAutoMode;
      String command = waterAutoMode ? F("WATER|MODE|AUTO") : F("WATER|MODE|MANUAL");
      yield();
      ModuleInterop.QueryCommand(ctSET,command,false);
      yield();

      menuManager->resetIdleTimer();

      return;
    }

    if(pressed_button == 1)
    {
      // включить все каналы
      ModuleInterop.QueryCommand(ctSET,F("WATER|ON"),false);
      menuManager->resetIdleTimer();
      yield();
      return;
    }
    
    if(pressed_button == 2)
    {
      // выключить все каналы
      ModuleInterop.QueryCommand(ctSET,F("WATER|OFF"),false);
      menuManager->resetIdleTimer();
      yield();
      return;
    }

    if(pressed_button == skipButton)
    {
      // пропустить полив сегодня
      ModuleInterop.QueryCommand(ctSET,F("WATER|SKIP"),false);
      menuManager->resetIdleTimer();
      yield();
      return;      
    }

    if(pressed_button == channelsButton)
    {
      menuManager->switchToScreen("WTRCH");
      return;
    }

    if(pressed_button > (BUTTONS_OFFSET-1))
    {
      // кнопки управления каналами
      int channelNum = pressed_button - BUTTONS_OFFSET;

      ControllerState state = WORK_STATUS.GetState();

      bool isWaterOn = state.WaterChannelsState & (1 << channelNum);
      String command = F("WATER|");

      command += isWaterOn ? F("OFF|") : F("ON|");
      command += channelNum;
      
      ModuleInterop.QueryCommand(ctSET,command,false);
      menuManager->resetIdleTimer();
      yield();

      return;
    }

    
    // тут нам надо обновить состояние кнопок для каналов
    
     ControllerState state = WORK_STATUS.GetState();

     // проходимся по всем каналам
     for(int i=0;i<WATER_RELAYS_COUNT;i++)
     {
       int buttonID = i+BUTTONS_OFFSET; // у нас первые 6 кнопок - не для каналов

       bool isChannelActive = (state.WaterChannelsState & (1 << i));
       bool savedIsChannelActive =  (lastWaterChannelsState & (1 << i));
       
       bool wantRedrawChannel = !inited || (isChannelActive != savedIsChannelActive);

        if(wantRedrawChannel)
        {
            if(isChannelActive)
            {
               screenButtons->setButtonBackColor(buttonID,MODE_ON_COLOR);
            }
            else
            {
              screenButtons->setButtonBackColor(buttonID,MODE_OFF_COLOR);
            }

          screenButtons->drawButton(buttonID);
        }
       
     } // for


     bool wateringAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);
     if(wateringAutoMode)
     {
        screenButtons->setButtonBackColor(3,MODE_ON_COLOR);
        screenButtons->relabelButton(3,AUTO_MODE_LABEL,!inited || (wateringAutoMode != lastWateringAutoMode));
     }
     else
     {
        screenButtons->setButtonBackColor(3,MODE_OFF_COLOR);      
        screenButtons->relabelButton(3,MANUAL_MODE_LABEL,!inited || (wateringAutoMode != lastWateringAutoMode));
     }

      // сохраняем состояние каналов полива
     lastWaterChannelsState = state.WaterChannelsState;
     lastWateringAutoMode = wateringAutoMode;
     inited = true;
     
 } // if(screenButtons)
 
 #endif // WATER_RELAYS_COUNT > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 #if WATER_RELAYS_COUNT > 0
  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }
 #endif 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWateringChannelsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringChannelsScreen::TFTWateringChannelsScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringChannelsScreen::~TFTWateringChannelsScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelsScreen::setup(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }  

  #if WATER_RELAYS_COUNT > 0
  
    screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

    int buttonsTop = INFO_BOX_V_SPACING + 40;
    int screenWidth = menuManager->getDC()->getDisplayXSize();
    int screenHeight = menuManager->getDC()->getDisplayYSize();
    
    int x_spacing = (screenWidth - (ALL_CHANNELS_BUTTON_WIDTH*3) - INFO_BOX_V_SPACING*2)/2;
    int allChannelsLeft = x_spacing;

    // первая - кнопка назад
    int backButtonHeight = ALL_CHANNELS_BUTTON_HEIGHT;
    backButton = screenButtons->addButton( allChannelsLeft ,  screenHeight - backButtonHeight - INFO_BOX_V_SPACING, TFT_BACK_BUTTON_WIDTH,  backButtonHeight, WM_BACK_CAPTION);
 
    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<WATER_RELAYS_COUNT;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
        curButtonLeft = computedButtonLeft;
       }
       
       String* label = new String('#');
       *label += (i+1);
       labels.push_back(label);
       
       screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  CHANNELS_BUTTON_HEIGHT, label->c_str());
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for    
  
    #endif // WATER_RELAYS_COUNT > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

 #if WATER_RELAYS_COUNT > 0

  const byte BUTTONS_OFFSET = 1; // с какого индекса начинаются наши кнопки
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);
  
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("WATER");
      return;
    }

 
    if(pressed_button >= BUTTONS_OFFSET)
    {
      // кнопки управления каналами
      int channelNum = pressed_button - BUTTONS_OFFSET;

      WateringChannelSettingsScreen->editChannel(channelNum);

      return;
    }
    
 } // if(screenButtons)
 
 #endif // WATER_RELAYS_COUNT > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


 #if WATER_RELAYS_COUNT > 0
  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }
  drawScreenCaption(menuManager,TFT_WATERING_SETT_SCREEN_CAPTION);
 #endif 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWateringChannelSettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringChannelSettingsScreen* WateringChannelSettingsScreen = NULL;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringChannelSettingsScreen::TFTWateringChannelSettingsScreen()
{
  WateringChannelSettingsScreen = this;
  tickerButton = -1;

  hourBox = NULL;
  durationBox = NULL;
  humidityBorderBox = NULL;
  humidityBorderMinBox = NULL;
  sensorIndexBox = NULL;

  sensorDataLeft = sensorDataTop = 0;
   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWateringChannelSettingsScreen::~TFTWateringChannelSettingsScreen()
{
 delete screenButtons;

 delete hourBox;
 delete durationBox;
 delete humidityBorderBox;
 delete humidityBorderMinBox;
 delete sensorIndexBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decHourButton || buttonID == incHourButton || buttonID == decDurationButton || buttonID == incDurationButton
  || buttonID == decHumidityBorderButton || buttonID == incHumidityBorderButton
  || buttonID == decHumidityBorderMinButton || buttonID == incHumidityBorderMinButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::onTick()
{
  
  if(tickerButton == decHourButton)
    incHour(-1);
  else
  if(tickerButton == incHourButton)
    incHour(1);
  else
  if(tickerButton == decDurationButton)
    incDuration(-11);
  else
  if(tickerButton == incDurationButton)
    incDuration(11);
  else
  if(tickerButton == decHumidityBorderButton)
    incHumidityBorder(-1);
  else
  if(tickerButton == incHumidityBorderButton)
    incHumidityBorder(1);
  else
  if(tickerButton == decHumidityBorderMinButton)
    incHumidityBorderMin(-1);
  else
  if(tickerButton == incHumidityBorderMinButton)
    incHumidityBorderMin(1);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }  
    GlobalSettings* settings = MainController->GetSettings();

    humiditySensorsCount = 0;

    #ifdef USE_SOIL_MOISTURE_MODULE
      AbstractModule* mod = MainController->GetModuleByID("SOIL");
      if(mod)
      {
        humiditySensorsCount = mod->State.GetStateCount(StateSoilMoisture);
      }
    #endif

    // загружаем настройки канала полива
    wateringDays = settings->GetChannelWateringWeekDays(channelNumber);
    wateringTime = settings->GetChannelWateringTime(channelNumber);
    startWateringTime = settings->GetChannelStartWateringTime(channelNumber) / 60;
    wateringSensorIndex = settings->GetChannelWateringSensorIndex(channelNumber);
    wateringStopBorder = settings->GetChannelWateringStopBorder(channelNumber);
    wateringStartBorder = settings->GetChannelWateringStartBorder(channelNumber);

    // теперь смотрим, какие дни недели установлены для канала
    for(uint8_t i=0;i<7;i++)
    {
      uint8_t mask = (1 << i);
      int btnID = weekdaysButtons[i];
        
      if((wateringDays & mask))
      {
        screenButtons->setButtonBackColor(btnID,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      } // if
      else
      {
        screenButtons->setButtonBackColor(btnID,MODE_OFF_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      }
        
    } // for

    screenButtons->disableButton(saveButton);
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR); 

    getSensorData();
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::setup(TFTMenu* menuManager)
{
    
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int fourthRowTopPos = thirdRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = 170;//(screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = fourthRowTopPos;//screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);

    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    // первая строка

    int weekDaysButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*6)/7;
    int weekDayLeft = initialLeftPos;

    for(int i=0;i<7;i++)
    {
      int addedID = screenButtons->addButton( weekDayLeft ,  topPos, weekDaysButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, WEEKDAYS[i]);
      weekDayLeft += weekDaysButtonWidth + INFO_BOX_V_SPACING;

      weekdaysButtons.push_back(addedID);
    }


   // вторая строка
   int textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
   
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   hourBox = new TFTInfoBox(TFT_WATERING_SETT_START,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decDurationButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   durationBox = new TFTInfoBox(TFT_WATERING_SETT_DURATION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incDurationButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


 // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;


   decHumidityBorderMinButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   humidityBorderMinBox = new TFTInfoBox(TFT_WATERING_SETT_HUM_BORDER_MIN,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHumidityBorderMinButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHumidityBorderButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   humidityBorderBox = new TFTInfoBox(TFT_WATERING_SETT_HUM_BORDER,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHumidityBorderButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


// ЧЕТВЁРТАЯ СТРОКА
   textBoxTopPos = fourthRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = fourthRowTopPos;
   leftPos = initialLeftPos;
   
   leftPos += INFO_BOX_V_SPACING*4 + TFT_ARROW_BUTTON_WIDTH*2 + TFT_TEXT_INPUT_WIDTH;

   decSensorIndexButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorIndexBox = new TFTInfoBox(TFT_WATERING_SETT_SENSOR,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(TFT_WATERING_SETT_SENSOR) + textFontWidth*2 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);

   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorIndexButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   #ifndef USE_SOIL_MOISTURE_MODULE
    screenButtons->disableButton(decHumidityBorderButton);
    screenButtons->disableButton(incHumidityBorderButton);
    screenButtons->disableButton(decHumidityBorderMinButton);
    screenButtons->disableButton(incHumidityBorderMinButton);
    screenButtons->disableButton(decSensorIndexButton);
    screenButtons->disableButton(incSensorIndexButton);
   #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();

  // сохраняем настройки канала полива
  if(settings->GetChannelWateringWeekDays(channelNumber) != wateringDays)
  {
    settings->SetChannelWateringWeekDays(channelNumber, wateringDays);
  }

  if(settings->GetChannelWateringTime(channelNumber) != wateringTime)
  {
    settings->SetChannelWateringTime(channelNumber, wateringTime);
  }

  if(settings->GetChannelStartWateringTime(channelNumber) != (startWateringTime*60))
  {
    settings->SetChannelStartWateringTime(channelNumber, startWateringTime*60);
  }

  if(settings->GetChannelWateringSensorIndex(channelNumber) != wateringSensorIndex)
  {
    settings->SetChannelWateringSensorIndex(channelNumber, wateringSensorIndex);
  }

  if(settings->GetChannelWateringStopBorder(channelNumber) != wateringStopBorder)
  {
    settings->SetChannelWateringStopBorder(channelNumber, wateringStopBorder);
  }

  if(settings->GetChannelWateringStartBorder(channelNumber) != wateringStartBorder)
  {
    settings->SetChannelWateringStartBorder(channelNumber, wateringStartBorder);
  }

  // ну и, напоследок - если опция полива отлична от "раздельное управление каналами" - то включаем раздельное управление каналами
  if(settings->GetWateringOption() != wateringSeparateChannels)
  {
    settings->SetWateringOption(wateringSeparateChannels); 
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::incHour(int val)
{

  uint16_t oldW = startWateringTime;
  startWateringTime+=val;

  if(startWateringTime > 23)
    startWateringTime = 23;
  
  if(startWateringTime != oldW)
    drawValueInBox(hourBox,startWateringTime);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::incDuration(int val)
{
 
  uint16_t oldW = wateringTime;
  wateringTime+=val;
  
  if(wateringTime > 1440)
    wateringTime = 1440;
  
  if(wateringTime != oldW)
    drawValueInBox(durationBox,wateringTime);
     
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::incHumidityBorder(int val)
{
 
  uint8_t oldW = wateringStopBorder;
  wateringStopBorder+=val;
    
  if(wateringStopBorder > 100)
    wateringStopBorder = 100;
  
  if(wateringStopBorder != oldW)
    drawValueInBox(humidityBorderBox,wateringStopBorder); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::incHumidityBorderMin(int val)
{
 
  uint8_t oldW = wateringStartBorder;
  wateringStartBorder+=val;
    
  if(wateringStartBorder > 100)
    wateringStartBorder = 100;
  
  if(wateringStartBorder != oldW)
    drawValueInBox(humidityBorderMinBox,wateringStartBorder); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::incSensorIndex(int val)
{
  
  int8_t oldW = wateringSensorIndex;
  wateringSensorIndex+=val;

  if(wateringSensorIndex < -1)
    wateringSensorIndex = -1;
    
  if(wateringSensorIndex >= humiditySensorsCount)
    wateringSensorIndex = humiditySensorsCount - 1;
  
  if(wateringSensorIndex != oldW)
  {
    String wsi = String(wateringSensorIndex);
    if(wateringSensorIndex < 0)
      wsi = '-';
    drawValueInBox(sensorIndexBox,wsi);  
  }
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData();
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager);
    }

    sensorUpdateTimer = millis();
  }      

 if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
            menuManager->resetIdleTimer();

   
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("WTRCH");
      return;
    }
    else
    if(pressed_button == saveButton)
    {
      saveSettings();
      blinkSaveSettingsButton(false);
      screenButtons->disableButton(saveButton,true);
      return;
    }
    else if(pressed_button == decHourButton)
    {
      incHour(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incHourButton)
    {
      incHour(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decDurationButton)
    {
      incDuration(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incDurationButton)
    {
      incDuration(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decHumidityBorderButton)
    {
      incHumidityBorder(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incHumidityBorderButton)
    {
      incHumidityBorder(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decHumidityBorderMinButton)
    {
      incHumidityBorderMin(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incHumidityBorderMinButton)
    {
      incHumidityBorderMin(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decSensorIndexButton)
    {
      incSensorIndex(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);

      getSensorData();
      drawSensorData(menuManager);      
    }
    else if(pressed_button == incSensorIndexButton)
    {
      incSensorIndex(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
      
      getSensorData();
      drawSensorData(menuManager);      

    }
    else
    {
      // нажата одна из кнопок дней недели
      for(int i=0;i<7;i++)
      {
        if(pressed_button == weekdaysButtons[i])
        {
          word backColor = screenButtons->getButtonBackColor(pressed_button);
          if(backColor == MODE_ON_COLOR)
          {
            backColor = MODE_OFF_COLOR;
            wateringDays &= ~(1 << i);
          }
          else
          {
            backColor = MODE_ON_COLOR;
            wateringDays |= (1 << i);
          }

          screenButtons->setButtonBackColor(pressed_button,backColor);
          screenButtons->drawButton(pressed_button);
          
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
          
          break;
        } // if
      } // for
    }
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::editChannel(int num)
{
  channelNumber = num;
  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::getSensorData()
{
  sensorDataString = F("   -   ");

  if(wateringSensorIndex < 0)
    return;
  
  AbstractModule* module = MainController->GetModuleByID("SOIL");
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateSoilMoisture,wateringSensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   HumidityPair tmp = *sensorState;
   sensorDataString = "";

   sensorDataString += tmp.Current;
   sensorDataString += '%'; 
  }

   while(sensorDataString.length() < 7)
    sensorDataString += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::drawSensorData(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(sensorDataString.c_str(), sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWateringChannelSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }


  hourBox->draw(menuManager);
  drawValueInBox(hourBox,startWateringTime);  

  durationBox->draw(menuManager);
  drawValueInBox(durationBox,wateringTime);  

  humidityBorderMinBox->draw(menuManager);
  drawValueInBox(humidityBorderMinBox,wateringStartBorder);  

  humidityBorderBox->draw(menuManager);
  drawValueInBox(humidityBorderBox,wateringStopBorder);  

  sensorIndexBox->draw(menuManager);
  String wsi = String(wateringSensorIndex);
  if(wateringSensorIndex < 0)
    wsi = '-';
  drawValueInBox(sensorIndexBox,wsi);  

  drawSensorData(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERING_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "SceneModule.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTSceneScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSceneScreen::TFTSceneScreen()
{
  inited = false;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSceneScreen::~TFTSceneScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSceneScreen::initScenes(TFTMenu* menuManager)
{
  if(inited)
  {
    return;
  }

  if(!menuManager->getDC())
  {
    return;
  }
    

    int screenWidth = menuManager->getDC()->getDisplayXSize();
 //   int screenHeight = menuManager->getDC()->getDisplayYSize();
    
    // init scenes here
    uint16_t scenesCount = Scenes->GetScenesCount();

    const uint8_t BUTTONS_PER_ROW = 2;
    const uint8_t MAX_ROWS = 4;
    const int  SCENE_BUTTON_WIDTH = 370;
    const int SCENE_BUTTON_HEIGHT = ALL_CHANNELS_BUTTON_HEIGHT;

    int buttonsTop = INFO_BOX_V_SPACING;
    // по три кнопки на строку
    int buttonsLeft = (screenWidth - (SCENE_BUTTON_WIDTH*BUTTONS_PER_ROW) - INFO_BOX_V_SPACING*(BUTTONS_PER_ROW-1))/2;

    int curLeft = buttonsLeft;
    int curTop = buttonsTop;
    
    int rows = scenesCount/BUTTONS_PER_ROW;
    if(scenesCount%BUTTONS_PER_ROW)
      rows++;

    if(rows > MAX_ROWS)
      rows = MAX_ROWS;


    // add scene buttons
    int counter = 0;
    for(int i=0;i<rows;i++)
    {
       for(int k=0;k<BUTTONS_PER_ROW;k++)
       {
          if(counter >= scenesCount)
            break;

          SceneSettings ss = Scenes->GetSceneSettings(counter);

          if(ss.isSceneExists)
          {
            SceneButton sb;
            sb.sceneNumber = counter;
            sb.sceneName = new char[ss.sceneName.length()+1];
            sb.sceneRunning = Scenes->IsSceneActive(sb.sceneNumber);
            strcpy(sb.sceneName,ss.sceneName.c_str());

            // создаём кнопку одной сцены
            int addedId = screenButtons->addButton( curLeft ,  curTop, SCENE_BUTTON_WIDTH,  SCENE_BUTTON_HEIGHT, sb.sceneName);
            screenButtons->setButtonBackColor(addedId,sb.sceneRunning ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(addedId,sb.sceneRunning ? CHANNELS_BUTTONS_TEXT_COLOR : CHANNEL_BUTTONS_TEXT_COLOR);

            curLeft += SCENE_BUTTON_WIDTH + INFO_BOX_V_SPACING;

            sb.buttonId = addedId;
            
            sceneButtons.push_back(sb);
          }
            
          counter++;
       }

       curLeft = buttonsLeft;
       curTop += SCENE_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
       
          if(counter >= scenesCount)
            break;
    }

    inited = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSceneScreen::setup(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  
    screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

    //int screenWidth = menuManager->getDC()->getDisplayXSize();
    int screenHeight = menuManager->getDC()->getDisplayYSize();
    int leftPos = INFO_BOX_V_SPACING + INFO_BOX_V_SPACING/2 ;
    // первая - кнопка назад
    int backButtonHeight = ALL_CHANNELS_BUTTON_HEIGHT;
    backButton = screenButtons->addButton( leftPos ,  screenHeight - backButtonHeight - INFO_BOX_V_SPACING, TFT_BACK_BUTTON_WIDTH,  backButtonHeight, WM_BACK_CAPTION);
    
    initScenes(menuManager);  
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSceneScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);
   
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("SENSORS");
      return;
    }

    if(pressed_button > backButton)
    {
      int listIndex = -1;

      for(size_t i=0;i<sceneButtons.size();i++)
      {
        if(sceneButtons[i].buttonId == pressed_button)
        {
          listIndex = i;
          break;
        }
      }

      if(listIndex != -1)
      {
        sceneButtons[listIndex].sceneRunning = !sceneButtons[listIndex].sceneRunning;
        screenButtons->setButtonBackColor(sceneButtons[listIndex].buttonId,sceneButtons[listIndex].sceneRunning ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(sceneButtons[listIndex].buttonId,sceneButtons[listIndex].sceneRunning ? CHANNELS_BUTTONS_TEXT_COLOR : CHANNEL_BUTTONS_TEXT_COLOR);
        screenButtons->drawButton(sceneButtons[listIndex].buttonId); 
        
        if(sceneButtons[listIndex].sceneRunning)
        {
          Scenes->ExecuteScene(sceneButtons[listIndex].sceneNumber);
        }
        else
        {
          Scenes->StopScene(sceneButtons[listIndex].sceneNumber);          
        }
      }
      

      return;
    }

     updateSceneButtons();

     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSceneScreen::updateSceneButtons()
{
    // обновляем состояние кнопок
    static uint32_t sceneUpdateTimer = 0;
    uint32_t now = millis();
    if(now - sceneUpdateTimer > 500)
    {
      for(size_t i=0;i<sceneButtons.size();i++)
      {
        bool nowRunning = Scenes->IsSceneActive(sceneButtons[i].sceneNumber);
        if(nowRunning != sceneButtons[i].sceneRunning)
        {
          sceneButtons[i].sceneRunning = nowRunning;
          screenButtons->setButtonBackColor(sceneButtons[i].buttonId,sceneButtons[i].sceneRunning ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);
          screenButtons->setButtonFontColor(sceneButtons[i].buttonId,sceneButtons[i].sceneRunning ? CHANNELS_BUTTONS_TEXT_COLOR : CHANNEL_BUTTONS_TEXT_COLOR);
          screenButtons->drawButton(sceneButtons[i].buttonId);          
        }
        
      } // for

      sceneUpdateTimer = millis();
    }  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSceneScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    updateSceneButtons();
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SCENE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTLightScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTLightScreen::TFTLightScreen()
{
  inited = false;
  tickerButton = -1;

  #ifdef USE_LIGHT_MANAGE_MODULE
  hourBox = NULL;
  histeresisBox = NULL;
  durationHourBox = NULL;
  luxBox = NULL;
 
  memset(&lightSettings,0,sizeof(LightSettings));
  #endif
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTLightScreen::~TFTLightScreen()
{
 delete screenButtons;

 #ifdef USE_LIGHT_MANAGE_MODULE
 delete hourBox;
 delete histeresisBox;
 delete durationHourBox;
 delete luxBox;
 #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  #ifdef USE_LIGHT_MANAGE_MODULE
  if(buttonID == decHourButton || buttonID == incHourButton || buttonID == decHisteresisButton || buttonID == incHisteresisButton
  || buttonID == decDurationHourButton || buttonID == incDurationHourButton || buttonID == decLuxButton || buttonID == incLuxButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::onTick()
{
  #ifdef USE_LIGHT_MANAGE_MODULE
  
  if(tickerButton == decHourButton)
    incHour(-1);
  else
  if(tickerButton == incHourButton)
    incHour(1);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-11);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(11);

  else
  if(tickerButton == decDurationHourButton)
    incDurationHour(-1);
  else
  if(tickerButton == incDurationHourButton)
    incDurationHour(1);
  else
  if(tickerButton == decLuxButton)
    incLux(-11);
  else
  if(tickerButton == incLuxButton)
    incLux(11);

  #endif // #ifdef USE_LIGHT_MANAGE_MODULE

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::onActivate(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();

    #ifdef USE_LIGHT_MANAGE_MODULE
    
    lightSettings = settings->GetLightSettings();

    if(lightSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }
    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR); 
    
    #endif // #ifdef USE_LIGHT_MANAGE_MODULE   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::setup(TFTMenu* menuManager)
{

    bool isLightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
    
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);

    #ifdef USE_LIGHT_MANAGE_MODULE
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    #endif // #ifdef USE_LIGHT_MANAGE_MODULE


    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/3;
    int channelsButtonLeft = initialLeftPos;

    lightOnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, TURN_ON_ALL_LIGHT_LABEL);
    screenButtons->setButtonBackColor(lightOnButton,isLightOn ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    lightOffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, TURN_OFF_ALL_LIGHT_LABEL);
    screenButtons->setButtonBackColor(lightOffButton,!isLightOn ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);

    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    lightModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(lightModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    


  #ifdef USE_LIGHT_MANAGE_MODULE
   // вторая строка
   int textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
   
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   hourBox = new TFTInfoBox(WM_LIGHT_BEGIN_H_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decDurationHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   durationHourBox = new TFTInfoBox(WM_LIGHT_BEGIN_DH_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incDurationHourButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


 // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   luxBox = new TFTInfoBox(WM_LIGHT_LUM_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox(WM_LIGHT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

  #endif // #ifdef USE_LIGHT_MANAGE_MODULE
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LIGHT_MANAGE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
 settings->SetLightSettings(lightSettings);

 LogicManageModule->ReloadLightSettings();
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::incHour(int val)
{
   int16_t oldW = lightSettings.hour;
  lightSettings.hour+=val;

  if(lightSettings.hour < 0)
    lightSettings.hour = 0;
    
  if(lightSettings.hour > 23)
    lightSettings.hour = 23;
  
  if(lightSettings.hour != oldW)
    drawValueInBox(hourBox,lightSettings.hour);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::incHisteresis(int val)
{
   int16_t oldW = lightSettings.histeresis;
  lightSettings.histeresis+=val;

  if(lightSettings.histeresis < 0)
  lightSettings.histeresis = 0;
  
  if(lightSettings.histeresis > 9999)
    lightSettings.histeresis = 9999;
  
  if(lightSettings.histeresis != oldW)
    drawValueInBox(histeresisBox,lightSettings.histeresis);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::incDurationHour(int val)
{
   int16_t oldW = lightSettings.durationHour;
  lightSettings.durationHour+=val;

  if(lightSettings.durationHour < 0)
    lightSettings.durationHour = 0;
    
  if(lightSettings.durationHour > 23)
    lightSettings.durationHour = 23;
  
  if(lightSettings.durationHour != oldW)
    drawValueInBox(durationHourBox,lightSettings.durationHour);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::incLux(int val)
{
   int16_t oldW = lightSettings.lux;
  lightSettings.lux+=val;

  if(lightSettings.lux < 0)
  lightSettings.lux = 0;
  
  if(lightSettings.lux > 65000)
    lightSettings.lux = 65000;
  
  if(lightSettings.lux != oldW)
    drawValueInBox(luxBox,lightSettings.lux);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // #ifdef USE_LIGHT_MANAGE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

 #ifdef USE_LIGHT_MANAGE_MODULE
 if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)
  #endif // #ifdef USE_LIGHT_MANAGE_MODULE
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
            menuManager->resetIdleTimer();

   
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("DRIVE");
      return;
    }
    #ifdef USE_LIGHT_MANAGE_MODULE
    else
    if(pressed_button == saveButton)
    {
      saveSettings();
      blinkSaveSettingsButton(false);
      screenButtons->disableButton(saveButton,true);
      return;
    }
    else if(pressed_button == decHourButton)
    {
      incHour(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incHourButton)
    {
      incHour(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decHisteresisButton)
    {
      incHisteresis(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incHisteresisButton)
    {
      incHisteresis(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decDurationHourButton)
    {
      incDurationHour(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incDurationHourButton)
    {
      incDurationHour(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == decLuxButton)
    {
      incLux(-1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else if(pressed_button == incLuxButton)
    {
      incLux(1);
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);
    }
    else
    if(pressed_button == onOffButton)
    {
        if(lightSettings.active)
          lightSettings.active = false;
       else
          lightSettings.active = true;
        
        if(lightSettings.active)
        {
          screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
          screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
        }
        else
        {
          screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
          screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
        }
      screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
      blinkSaveSettingsButton(true);     
    }
    #endif // #ifdef USE_LIGHT_MANAGE_MODULE
    else
    if(pressed_button == lightModeButton)
    {
      // Кнопка смены режима
      bool lightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);
      lightAutoMode = !lightAutoMode;
      String command = lightAutoMode ? F("LIGHT|MODE|AUTO") : F("LIGHT|MODE|MANUAL");
      ModuleInterop.QueryCommand(ctSET,command,false);
      yield();

      return;
    }
    else
    if(pressed_button == lightOnButton)
    {
      // включить досветку
      ModuleInterop.QueryCommand(ctSET,F("LIGHT|ON"),false);
      yield();
      return;
    }
    
    if(pressed_button == lightOffButton)
    {
      // выключить досветку
      ModuleInterop.QueryCommand(ctSET,F("LIGHT|OFF"),false);
      yield();
      return;
    }

    bool lightIsOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
    bool lightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);

    if(!inited || (lastLightIsOn != lightIsOn))
    {
        screenButtons->setButtonBackColor(lightOnButton,lightIsOn ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonBackColor(lightOffButton,!lightIsOn ? MODE_ON_COLOR : CHANNELS_BUTTONS_BG_COLOR);

        screenButtons->setButtonFontColor(lightOnButton,lightIsOn ? CHANNELS_BUTTONS_TEXT_COLOR : CHANNEL_BUTTONS_TEXT_COLOR);
        screenButtons->setButtonFontColor(lightOffButton,!lightIsOn ? CHANNELS_BUTTONS_TEXT_COLOR : CHANNEL_BUTTONS_TEXT_COLOR);
        
        screenButtons->drawButton(lightOnButton);
        screenButtons->drawButton(lightOffButton);
    }
     
     if(lightAutoMode)
     {
        screenButtons->setButtonBackColor(lightModeButton,MODE_ON_COLOR);
        screenButtons->relabelButton(lightModeButton,AUTO_MODE_LABEL,!inited || (lightAutoMode != lastLightAutoMode));
     }
     else
     {
        screenButtons->setButtonBackColor(lightModeButton,MODE_OFF_COLOR);      
        screenButtons->relabelButton(lightModeButton,MANUAL_MODE_LABEL,!inited || (lightAutoMode != lastLightAutoMode));
     }

      // сохраняем состояние досветки
     lastLightIsOn = lightIsOn;
     lastLightAutoMode = lightAutoMode;
     inited = true; 
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTLightScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  #ifdef USE_LIGHT_MANAGE_MODULE
  hourBox->draw(menuManager);
  drawValueInBox(hourBox,lightSettings.hour);  

  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,lightSettings.histeresis);  

  durationHourBox->draw(menuManager);
  drawValueInBox(durationHourBox,lightSettings.durationHour);  

  luxBox->draw(menuManager);
  drawValueInBox(luxBox,lightSettings.lux);  
  #endif // #ifdef USE_LIGHT_MANAGE_MODULE
  

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LUMINOSITY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWindowScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWindowScreen::TFTWindowScreen()
{
  inited = false;
  lastWindowsState = 0;
  percentsCounter = 50;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWindowScreen::~TFTWindowScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindowScreen::onActivate(TFTMenu* menuManager)
{
  inited = false;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindowScreen::setup(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  

  #if SUPPORTED_WINDOWS > 0
  
    screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter(),22);
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

 
    int buttonsTop = INFO_BOX_V_SPACING;
    int screenWidth = menuManager->getDC()->getDisplayXSize();
    int screenHeight = menuManager->getDC()->getDisplayYSize();

    // добавляем кнопки для управления всеми каналами
    int allChannelsLeft = (screenWidth - (ALL_CHANNELS_BUTTON_WIDTH*3) - INFO_BOX_V_SPACING*2)/2;

    // первая - кнопка назад
    int backButtonHeight = ALL_CHANNELS_BUTTON_HEIGHT;
    int backButtonWidth = CHANNELS_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING;
    int backButtonTop = screenHeight - backButtonHeight - INFO_BOX_V_SPACING;
    backButton = screenButtons->addButton( allChannelsLeft ,  backButtonTop , backButtonWidth,  backButtonHeight, WM_BACK_CAPTION);
    percentsButton = screenButtons->addButton( allChannelsLeft +  backButtonWidth + INFO_BOX_V_SPACING,  backButtonTop, backButtonWidth,  backButtonHeight, WS_50_CAPTION);
    
    openAllButton = screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, OPEN_ALL_LABEL);
    allChannelsLeft += ALL_CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;

    closeAllButton = screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, CLOSE_ALL_LABEL);
    allChannelsLeft += ALL_CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;

    modeButton = screenButtons->addButton( allChannelsLeft ,  buttonsTop, ALL_CHANNELS_BUTTON_WIDTH,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(modeButton,CHANNELS_BUTTONS_TEXT_COLOR);

    buttonsTop += ALL_CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;

    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
        curButtonLeft = computedButtonLeft;
       }
       
       String* label = new String();
       *label += (i+1);
       *label += F(" - ");
       uint8_t curPercents = WindowModule->GetWindow(i)->GetCurrentPositionPercents(); 
       windowsPercents.push_back(curPercents);
       *label += curPercents;
       *label += '%';
       
       labels.push_back(label);
       
       int addedId = screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  CHANNELS_BUTTON_HEIGHT, label->c_str());
       screenButtons->setButtonFontColor(addedId,CHANNELS_BUTTONS_TEXT_COLOR);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
  
    #endif // SUPPORTED_WINDOWS > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindowScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

 #if SUPPORTED_WINDOWS > 0

  byte BUTTONS_OFFSET = 5; // с какого индекса начинаются наши кнопки
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    }
 
    if(pressed_button == backButton)
    {
      menuManager->switchToScreen("DRIVE");
      return;
    }

    if(pressed_button == percentsButton)
    {
      percentsCounter += 25;
      if(percentsCounter > 100)
        percentsCounter -= 100;

      switch(percentsCounter)
      {
        case 25:
          screenButtons->relabelButton(percentsButton,WS_25_CAPTION,true);
        break;
        case 50:
          screenButtons->relabelButton(percentsButton,WS_50_CAPTION,true);
        break;
        case 75:
          screenButtons->relabelButton(percentsButton,WS_75_CAPTION,true);
        break;
        case 100:
          screenButtons->relabelButton(percentsButton,WS_100_CAPTION,true);
        break;
      }
      
    }

    if(pressed_button == modeButton)
    {
      // Кнопка смены режима
      bool windowsAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
      windowsAutoMode = !windowsAutoMode;
      String command = windowsAutoMode ? F("STATE|MODE|AUTO") : F("STATE|MODE|MANUAL");
      ModuleInterop.QueryCommand(ctSET,command,false);
      
      yield();

      return;
    }

    if(pressed_button == openAllButton)
    {
      // открыть все окна
      String command = F("STATE|WINDOW|ALL|OPEN|");
      command += percentsCounter;
      command += '%';
      ModuleInterop.QueryCommand(ctSET,command,false);
      yield();
      return;
    }
    
    if(pressed_button == closeAllButton)
    {
      // закрыть все окна
      ModuleInterop.QueryCommand(ctSET,F("STATE|WINDOW|ALL|CLOSE"),false);
      yield();
      return;
    }

    if(pressed_button >= BUTTONS_OFFSET)
    {
      // кнопки управления каналами
      int channelNum = pressed_button - BUTTONS_OFFSET;

      bool isWindowOpen = WindowModule->IsWindowOpen(channelNum);
      String command = F("STATE|WINDOW|");
      command += channelNum;
      command += '|';

      command += isWindowOpen ? F("CLOSE") : F("OPEN");

      if(!isWindowOpen)
      {
        command += '|';
        command += percentsCounter;
        command += '%';
      }

      // выключаем кнопку принудительно!
      screenButtons->disableButton(pressed_button, screenButtons->buttonEnabled(pressed_button));
      
      ModuleInterop.QueryCommand(ctSET,command,false);
      yield();

      return;
    }
    // тут нам надо обновить состояние кнопок для каналов.
    // если канал окна в движении - надо кнопку выключать,
    // иначе - в зависимости от состояния канала
    
     ControllerState state = WORK_STATUS.GetState();

     // проходимся по всем каналам
     bool anyChannelActive = false;

     for(int i=0, channelNum=0;i<SUPPORTED_WINDOWS;i++, channelNum+=2)
     {
       int buttonID = i+BUTTONS_OFFSET; // у нас первые 4 кнопки - не для каналов
       
       // если канал включен - в бите единичка, иначе - нолик. оба выключены - окно не движется
       bool firstChannelBit = (state.WindowsState & (1 << channelNum));
       bool secondChannelBit = (state.WindowsState & (1 << (channelNum+1)));

       bool savedFirstChannelBit =  (lastWindowsState & (1 << channelNum));
       bool savedSecondChannelBit = (lastWindowsState & (1 << (channelNum+1)));
       
       bool isChannelOnIdle = !firstChannelBit && !secondChannelBit;


       // теперь смотрим, в какую сторону движется окно
       // если оно открывается - первый бит канала единичка, второй - нолик
       // если закрывается - наоборот
       // тут выяснение необходимости перерисовать кнопку. 
       // перерисовываем только тогда, когда состояние сменилось
       // или мы ещё не сохранили у себя первое состояние
       bool wantRedrawChannel = !inited || (firstChannelBit != savedFirstChannelBit || secondChannelBit != savedSecondChannelBit);
       bool backColorChanged = false;

       if(isChannelOnIdle)
       {
          // смотрим - окно открыто или закрыто?
          bool isWindowOpen = WindowModule->IsWindowOpen(i);

          uint16_t oldColor = screenButtons->getButtonBackColor(buttonID);
          
          if(isWindowOpen)
          {
             backColorChanged = oldColor == MODE_OFF_COLOR;
             wantRedrawChannel = wantRedrawChannel || backColorChanged;
             screenButtons->setButtonBackColor(buttonID,MODE_ON_COLOR);             
          }
          else
          {
             backColorChanged = oldColor == MODE_ON_COLOR;
             wantRedrawChannel = wantRedrawChannel || backColorChanged;
             screenButtons->setButtonBackColor(buttonID,MODE_OFF_COLOR);
          }

          uint8_t lastPercents = windowsPercents[i];
          uint8_t currentPercents = WindowModule->GetWindow(i)->GetCurrentPositionPercents();
          if(lastPercents != currentPercents)
          {
            windowsPercents[i] = currentPercents;
            
              String* label = labels[i];
             *label = "";
             *label += (i+1);
             *label += F(" - ");
             *label += currentPercents;
             *label += '%';

             screenButtons->relabelButton(buttonID,label->c_str());
       
          }

          // включаем кнопку
          screenButtons->enableButton(buttonID, wantRedrawChannel && (!screenButtons->buttonEnabled(buttonID) || backColorChanged));
          
       }
       else
       {
          anyChannelActive = true;
         // окно движется, блокируем кнопку
         screenButtons->disableButton(buttonID, wantRedrawChannel && screenButtons->buttonEnabled(buttonID));
       }
       
     } // for

     // теперь проверяем, надо ли вкл/выкл кнопки управления всеми окнами
     bool wantRedrawAllChannelsButtons = !inited || (lastAnyChannelActive != anyChannelActive);

     if(anyChannelActive)
     {
      screenButtons->disableButton(openAllButton, wantRedrawAllChannelsButtons && screenButtons->buttonEnabled(openAllButton));
      screenButtons->disableButton(closeAllButton, wantRedrawAllChannelsButtons && screenButtons->buttonEnabled(closeAllButton));
     }
     else
     {
      screenButtons->enableButton(openAllButton, wantRedrawAllChannelsButtons && !screenButtons->buttonEnabled(openAllButton));
      screenButtons->enableButton(closeAllButton, wantRedrawAllChannelsButtons && !screenButtons->buttonEnabled(closeAllButton));      
     }

     bool windowsAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);
     if(windowsAutoMode)
     {
        screenButtons->setButtonBackColor(modeButton,MODE_ON_COLOR);
        screenButtons->relabelButton(modeButton,AUTO_MODE_LABEL,(!inited || (windowsAutoMode != lastWindowsAutoMode)) && strcmp(AUTO_MODE_LABEL, screenButtons->getLabel(modeButton)) );
     }
     else
     {
        screenButtons->setButtonBackColor(modeButton,MODE_OFF_COLOR);      
        screenButtons->relabelButton(modeButton,MANUAL_MODE_LABEL,(!inited || (windowsAutoMode != lastWindowsAutoMode)) && strcmp(MANUAL_MODE_LABEL, screenButtons->getLabel(modeButton)) );
     }

      // сохраняем состояние окон
     lastWindowsState = state.WindowsState;
     lastAnyChannelActive = anyChannelActive;
     lastWindowsAutoMode = windowsAutoMode;
     inited = true;
     
 } // if(screenButtons)
 #endif // SUPPORTED_WINDOWS > 0
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindowScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 #if SUPPORTED_WINDOWS > 0
  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }
 #endif 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TEMP_SENSORS
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTTimeSyncScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTTimeSyncScreen::TFTTimeSyncScreen()
{
  tickerButton = -1;

  gsmIntervalBox = NULL;
  wifiIntervalBox = NULL;
  timezoneBox = NULL;

  gsmActive = false;
  wifiActive = false;

  wifiInterval = 1;
  gsmInterval = 1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTTimeSyncScreen::~TFTTimeSyncScreen()
{
 delete screenButtons;
 delete gsmIntervalBox;
 delete wifiIntervalBox;
 delete timezoneBox;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;

  if(buttonID == decGsmIntervalButton || buttonID == incGsmIntervalButton || buttonID == decWifiIntervalButton || buttonID == incWifiIntervalButton ||
  buttonID == decTimezoneButton || buttonID == incTimezoneButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::onTick()
{
  
  if(tickerButton == decGsmIntervalButton)
    incGsmInterval(-3);
  else
  if(tickerButton == incGsmIntervalButton)
    incGsmInterval(3);
  else
  if(tickerButton == decWifiIntervalButton)
    incWifiInterval(-3);
  else
  if(tickerButton == incWifiIntervalButton)
    incWifiInterval(3);
  else
  if(tickerButton == decTimezoneButton)
    incTimezone(-7);
  else
  if(tickerButton == incTimezoneButton)
    incTimezone(7);


}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);


    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);

    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);


    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/2;
    int channelsButtonLeft = initialLeftPos;

    gsmActiveButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "");
    gsmBtnLeft = channelsButtonLeft;
    
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING*2;
    wifiActiveButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "");
    wifiBtnLeft = channelsButtonLeft;
    controlButtonsTop = topPos;
    

   // вторая строка
   int textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
   
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decGsmIntervalButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   gsmIntervalBox = new TFTInfoBox(TFT_TIME_SYNC_GSM_INTERVAL_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incGsmIntervalButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decWifiIntervalButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   wifiIntervalBox = new TFTInfoBox(TFT_TIME_SYNC_WIFI_INTERVAL_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWifiIntervalButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


 // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decTimezoneButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   timezoneBox = new TFTInfoBox(TFT_TIMEZONE_BOX_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incTimezoneButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

  /*
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox(WM_LIGHT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

  */
      
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::saveSettings()
{
 //сохраняем настройки

 GlobalSettings* settings = MainController->GetSettings();
 TimeSyncSettings* ts = settings->getTimeSyncSettings();

 TimeSyncSettings thisTS;
 memcpy(&thisTS,ts,sizeof(TimeSyncSettings));

 thisTS.gsmActive = gsmActive;
 thisTS.wifiActive = wifiActive;

 thisTS.gsmInterval = gsmInterval;
 thisTS.wifiInterval = wifiInterval;

 thisTS.ntpTimezone = ntpTimezone;

 settings->setTimeSyncSettings(thisTS);

// Serial.print(F("NTP SERVER: ")); Serial.println(ts->ntpServer);
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::incTimezone(int val)
{
   int16_t oldW = ntpTimezone;
  ntpTimezone+=val;

  if(ntpTimezone < -500)
    ntpTimezone = -500;
    
  if(ntpTimezone > 1440)
    ntpTimezone = 1440;
  
  if(ntpTimezone != oldW)
    drawValueInBox(timezoneBox,ntpTimezone);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::incGsmInterval(int val)
{
   int16_t oldW = gsmInterval;
  gsmInterval+=val;

  if(gsmInterval < 1)
    gsmInterval = 1;
    
  if(gsmInterval > 50)
    gsmInterval = 50;
  
  if(gsmInterval != oldW)
    drawValueInBox(gsmIntervalBox,gsmInterval);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::incWifiInterval(int val)
{
   int16_t oldW = wifiInterval;
  wifiInterval+=val;

  if(wifiInterval < 1)
    wifiInterval = 1;
    
  if(wifiInterval > 50)
    wifiInterval = 50;
  
  if(wifiInterval != oldW)
    drawValueInBox(wifiIntervalBox,wifiInterval);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)  
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
         menuManager->resetIdleTimer();

        if(pressed_button == backButton)
        {
          menuManager->switchToScreen("OPTIONS");
          return;
        }
        else
        if(pressed_button == saveButton)
        {
          saveSettings();
          blinkSaveSettingsButton(false);
          screenButtons->disableButton(saveButton,true);
          return;
        }
        else if(pressed_button == decGsmIntervalButton)
        {
          incGsmInterval(-1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == incGsmIntervalButton)
        {
          incGsmInterval(1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == decWifiIntervalButton)
        {
          incWifiInterval(-1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == incWifiIntervalButton)
        {
          incWifiInterval(1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == decTimezoneButton)
        {
          incTimezone(-1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == incTimezoneButton)
        {
          incTimezone(1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else
        if(pressed_button == gsmActiveButton)
        {
          gsmActive = !gsmActive;
          if(gsmActive)
          {
            screenButtons->setButtonBackColor(gsmActiveButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(gsmActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
            screenButtons->relabelButton(gsmActiveButton,TFT_SYNC_ENABLED_CAPTION,true);
            
          }
          else
          {
            screenButtons->setButtonBackColor(gsmActiveButton,MODE_OFF_COLOR);
            screenButtons->setButtonFontColor(gsmActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
            screenButtons->relabelButton(gsmActiveButton,TFT_SYNC_DISABLED_CAPTION,true);            
          }

          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
          
          return;
        }
        else
        if(pressed_button == wifiActiveButton)
        {
          wifiActive = !wifiActive;
          if(wifiActive)
          {
            screenButtons->setButtonBackColor(wifiActiveButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(wifiActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
            screenButtons->relabelButton(wifiActiveButton,TFT_SYNC_ENABLED_CAPTION,true);
            
          }
          else
          {
            screenButtons->setButtonBackColor(wifiActiveButton,MODE_OFF_COLOR);
            screenButtons->setButtonFontColor(wifiActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
            screenButtons->relabelButton(wifiActiveButton,TFT_SYNC_DISABLED_CAPTION,true);            
          }

          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
          
          return;
        }
        
    } // if(pressed_button != -1)
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  GlobalSettings* settings = MainController->GetSettings();
  TimeSyncSettings* ts = settings->getTimeSyncSettings();

  gsmActive = ts->gsmActive;
  gsmInterval = ts->gsmInterval;
  ntpTimezone = ts->ntpTimezone;

  wifiActive = ts->wifiActive;
  wifiInterval = ts->wifiInterval;
  
  if(gsmActive)
  {
    screenButtons->setButtonBackColor(gsmActiveButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(gsmActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->relabelButton(gsmActiveButton,TFT_SYNC_ENABLED_CAPTION);
    
  }
  else
  {
    screenButtons->setButtonBackColor(gsmActiveButton,MODE_OFF_COLOR);
    screenButtons->setButtonFontColor(gsmActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->relabelButton(gsmActiveButton,TFT_SYNC_DISABLED_CAPTION);            
  }

  if(wifiActive)
  {
    screenButtons->setButtonBackColor(wifiActiveButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(wifiActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->relabelButton(wifiActiveButton,TFT_SYNC_ENABLED_CAPTION);
    
  }
  else
  {
    screenButtons->setButtonBackColor(wifiActiveButton,MODE_OFF_COLOR);
    screenButtons->setButtonFontColor(wifiActiveButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->relabelButton(wifiActiveButton,TFT_SYNC_DISABLED_CAPTION);            
  }
  screenButtons->disableButton(saveButton);

  blinkActive = false;
  screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
  screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTTimeSyncScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  UTFT* dc = menuManager->getDC();
  UTFTRus* rusPrinter = menuManager->getRusPrinter();
  uint8_t* oldFont = dc->getFont();
  word oldColor = dc->getColor();
  word oldBackColor = dc->getBackColor();

  dc->setFont(BigRusFont);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  dc->setBackColor(TFT_BACK_COLOR);

  int fontHeight = dc->getFontYsize();
  int top = controlButtonsTop - fontHeight - 6;

  rusPrinter->print("СИНХРОНИЗАЦИЯ GSM:",gsmBtnLeft,top);
  rusPrinter->print("СИНХРОНИЗАЦИЯ WI-FI:",wifiBtnLeft,top);



  dc->setFont(oldFont);
  dc->setColor(oldColor);
  dc->setBackColor(oldBackColor);


  gsmIntervalBox->draw(menuManager);
  drawValueInBox(gsmIntervalBox,gsmInterval); 

  wifiIntervalBox->draw(menuManager);
  drawValueInBox(wifiIntervalBox,wifiInterval); 

  timezoneBox->draw(menuManager);
  drawValueInBox(timezoneBox,ntpTimezone); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTSettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSettingsScreen::TFTSettingsScreen()
{
  inited = false;
  tickerButton = -1;
  
  #ifdef USE_DS3231_REALTIME_CLOCK
  selectedTimePartButton = -1;
  controllerTimeChangedTimer = 0;  
  #endif

    sealevelBox = NULL;
    closeTempBox = NULL;
    openTempBox = NULL;
    intervalBox = NULL;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSettingsScreen::~TFTSettingsScreen()
{
 delete screenButtons;
 delete sealevelBox;
 delete closeTempBox;
 delete openTempBox;
 delete intervalBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::setup(TFTMenu* menuManager)
{

    // получаем температуры
    GlobalSettings* s = MainController->GetSettings();
    openTemp = s->GetOpenTemp();
    closeTemp = s->GetCloseTemp();
    unsigned long ulI = s->GetOpenInterval()/1000;
    interval = ulI;

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
  
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setTextFont(SevenSegNumFontMDS);
  
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);


 
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    // добавляем кнопки для управления температурой

    // у нас - 4 кнопки руления температурой, и два бокса для её вывода. Между кнопками и боксом - просвет, между двумя параметрами - двойной просвет.


    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;


    // первая - кнопка назад
    int backButtonHeight = ALL_CHANNELS_BUTTON_HEIGHT;
    backButton = screenButtons->addButton( leftPos ,  screenHeight - backButtonHeight - INFO_BOX_V_SPACING, TFT_BACK_BUTTON_WIDTH,  backButtonHeight, WM_BACK_CAPTION);
    screenButtons->setButtonFont(backButton,BigRusFont);

    timeSyncButton = screenButtons->addButton( leftPos + TFT_BACK_BUTTON_WIDTH + INFO_BOX_V_SPACING,  screenHeight - backButtonHeight - INFO_BOX_V_SPACING, widthOccupied - TFT_BACK_BUTTON_WIDTH - INFO_BOX_V_SPACING,  backButtonHeight, TFT_TIME_SYNC_BUTTON_CAPTION);
    screenButtons->setButtonFont(timeSyncButton,BigRusFont);
    

    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;//(screenHeight - TFT_ARROW_BUTTON_HEIGHT*2 - INFO_BOX_V_SPACING*5)/2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;

    UTFTRus* rusPrinter = menuManager->getRusPrinter();

    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
    int secondRowTextBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;

    // теперь добавляем наши кнопки
    decCloseTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
    decIntervalButton = screenButtons->addButton( leftPos ,  secondRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);

    #ifdef USE_DS3231_REALTIME_CLOCK
    
    decTimePartButton = screenButtons->addButton( leftPos ,  thirdRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
    screenButtons->disableButton(decTimePartButton);
    
    dc->setFont(SevenSegNumFontMDS);
    int fontWidth = dc->getFontXsize();
    int curTimePartLeftPos = leftPos + TFT_ARROW_BUTTON_WIDTH + INFO_BOX_V_SPACING;
    
    const int timeButtonPadding = 9;
    const int timeButtonHSpacing = INFO_BOX_V_SPACING;

    int timeButtonWidth = fontWidth*2+timeButtonPadding*2;
    int timeButtonWidth2 = fontWidth*4+timeButtonPadding*2;    
    
    dayButton = screenButtons->addButton( curTimePartLeftPos ,  thirdRowTopPos, timeButtonWidth,  TFT_ARROW_BUTTON_HEIGHT, strDay.c_str());
    screenButtons->setButtonFontColor(dayButton,TIME_PART_FONT_COLOR);
    screenButtons->setButtonBackColor(dayButton,TIME_PART_BG_COLOR);
    curTimePartLeftPos += timeButtonWidth + timeButtonHSpacing;

    monthButton = screenButtons->addButton( curTimePartLeftPos ,  thirdRowTopPos, timeButtonWidth,  TFT_ARROW_BUTTON_HEIGHT, strMonth.c_str());
    screenButtons->setButtonFontColor(monthButton,TIME_PART_FONT_COLOR);
    screenButtons->setButtonBackColor(monthButton,TIME_PART_BG_COLOR);
    curTimePartLeftPos += timeButtonWidth + timeButtonHSpacing;
    
    yearButton = screenButtons->addButton( curTimePartLeftPos ,  thirdRowTopPos, timeButtonWidth2,  TFT_ARROW_BUTTON_HEIGHT, strYear.c_str());
    screenButtons->setButtonFontColor(yearButton,TIME_PART_FONT_COLOR);
    screenButtons->setButtonBackColor(yearButton,TIME_PART_BG_COLOR);
    curTimePartLeftPos += timeButtonWidth2 + timeButtonHSpacing;

    hourButton = screenButtons->addButton( curTimePartLeftPos ,  thirdRowTopPos, timeButtonWidth,  TFT_ARROW_BUTTON_HEIGHT, strHour.c_str());
    screenButtons->setButtonFontColor(hourButton,TIME_PART_FONT_COLOR);
    screenButtons->setButtonBackColor(hourButton,TIME_PART_BG_COLOR);
    curTimePartLeftPos += timeButtonWidth + timeButtonHSpacing;

    minuteButton = screenButtons->addButton( curTimePartLeftPos ,  thirdRowTopPos, timeButtonWidth,  TFT_ARROW_BUTTON_HEIGHT, strMinute.c_str());
    screenButtons->setButtonFontColor(minuteButton,TIME_PART_FONT_COLOR);
    screenButtons->setButtonBackColor(minuteButton,TIME_PART_BG_COLOR);

    
    dc->setFont(BigRusFont);
    #endif
    
    leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

    closeTempBox = new TFTInfoBox(TFT_TCLOSE_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
    intervalBox = new TFTInfoBox(TFT_INTERVAL_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,secondRowTextBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
    leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
        
    incCloseTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
    incIntervalButton = screenButtons->addButton( leftPos ,  secondRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
    leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

    
    decOpenTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
    decSealevelButton = screenButtons->addButton( leftPos ,  secondRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
    leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

    openTempBox = new TFTInfoBox(TFT_TOPEN_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
    sealevelBox = new TFTInfoBox(TFT_SEALEVEL_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,secondRowTextBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
    leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
    incOpenTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
    incSealevelButton = screenButtons->addButton( leftPos ,  secondRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

    #ifdef USE_DS3231_REALTIME_CLOCK
      incTimePartButton = screenButtons->addButton( leftPos ,  thirdRowTopPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
      screenButtons->disableButton(incTimePartButton);    
    #endif
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DS3231_REALTIME_CLOCK
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t TFTSettingsScreen::stepVal(int8_t dir, uint16_t minVal,uint16_t maxVal, int16_t val)
{
  val += dir;

  if(val < minVal)
    val = maxVal;

  if(val > maxVal)
    val = minVal;

  return val;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTSettingsScreen::addLeadingZero(int val)
{
  String result;

  if(val < 0)
    result += '-';
    
  if(abs(val) < 10)
    result += '0';

  result += abs(val);

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::updateTimeButtons(RTCTime& tm, bool redraw)
{
  
    strDay = addLeadingZero(tm.dayOfMonth);
    screenButtons->relabelButton(dayButton,strDay.c_str(),redraw);

    strMonth = addLeadingZero(tm.month);
    screenButtons->relabelButton(monthButton,strMonth.c_str(),redraw);

    strYear = tm.year;
    screenButtons->relabelButton(yearButton,strYear.c_str(),redraw);

    strHour = addLeadingZero(tm.hour);
    screenButtons->relabelButton(hourButton,strHour.c_str(),redraw);

    strMinute = addLeadingZero(tm.minute);
    screenButtons->relabelButton(minuteButton,strMinute.c_str(),redraw);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_DS3231_REALTIME_CLOCK
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::onActivate(TFTMenu* menuManager)
{
  #ifdef USE_DS3231_REALTIME_CLOCK
  RealtimeClock rtc = MainController->GetClock();
  controllerTime = rtc.getTime();  
  updateTimeButtons(controllerTime,false);
  controllerTimeChanged = false;
  #endif

  GlobalSettings* settings = MainController->GetSettings();
  sealevel = settings->GetSealevel();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == incSealevelButton || buttonID == decSealevelButton || buttonID == incIntervalButton || buttonID == decIntervalButton
  || buttonID == incOpenTempButton || buttonID == decOpenTempButton || buttonID == incCloseTempButton || buttonID == decCloseTempButton

  #ifdef USE_DS3231_REALTIME_CLOCK
  || buttonID == decTimePartButton || buttonID == incTimePartButton
  #endif // USE_DS3231_REALTIME_CLOCK
  )
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::onTick()
{
  if(tickerButton == decSealevelButton)
    incSealevel(-9);
  else
  if(tickerButton == incSealevelButton)
    incSealevel(9);
  else
  if(tickerButton == incIntervalButton)
    incInterval(3);    
  else
  if(tickerButton == decIntervalButton)
    incInterval(-3);    
  else
  if(tickerButton == incOpenTempButton)
    incOpenTemp(3);    
  else
  if(tickerButton == decOpenTempButton)
    incOpenTemp(-3);    
  else
  if(tickerButton == incCloseTempButton)
    incCloseTemp(3);    
  else
  if(tickerButton == decCloseTempButton)
    incCloseTemp(-3);

  #ifdef USE_DS3231_REALTIME_CLOCK
  else
  if(tickerButton == decTimePartButton || tickerButton == incTimePartButton)
  {
    incDateTimePart(tickerButton);  
  }
  
  #endif // USE_DS3231_REALTIME_CLOCK
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::incSealevel(int val)
{
  TFTScreen->resetIdleTimer();

  int16_t oldSealevel = sealevel;
  
  sealevel+=val;
  
  if(sealevel > MAX_SEALEVEL_VAL)
    sealevel = MAX_SEALEVEL_VAL;

  if(sealevel < MIN_SEALEVEL_VAL)
    sealevel = MIN_SEALEVEL_VAL;

  if(sealevel != oldSealevel)
    drawValueInBox(sealevelBox,sealevel);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::incInterval(int val)
{
  if(val > 0)
  {
    if(uint32_t(interval+val) >= UINT_MAX)
        return;
  }
  else
  {
    if(interval < (val+2))
        return;    
  }
  
  interval+=val;
  
  unsigned long ulInterval = interval;
  ulInterval *= 1000;
  MainController->GetSettings()->SetOpenInterval(ulInterval);
  drawValueInBox(intervalBox,interval);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::incOpenTemp(int val)
{
  if(val > 0)
  {
    if((openTemp + val) > 50)
        return;
  }
  else
  {
    if(openTemp < (val+1))
        return;    
  }

    openTemp+=val;
    MainController->GetSettings()->SetOpenTemp(openTemp);
    drawValueInBox(openTempBox,openTemp);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::incCloseTemp(int val)
{
  if(val > 0)
  {
    if((closeTemp+val) > 50)
      return;
  }
  else
  {
    if(closeTemp < (val+1))
        return;    
  }

  closeTemp+=val;
  MainController->GetSettings()->SetCloseTemp(closeTemp);
  drawValueInBox(closeTempBox,closeTemp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::incDateTimePart(int btn)
{
  #ifdef USE_DS3231_REALTIME_CLOCK

      if(selectedTimePartButton == -1)
      {
        return;
      }

       int dir = btn == decTimePartButton ? -1 : 1;
       
      if(selectedTimePartButton == dayButton)
      {
        controllerTime.dayOfMonth = stepVal(dir, 0,31, controllerTime.dayOfMonth);
        strDay = addLeadingZero(controllerTime.dayOfMonth);
        screenButtons->relabelButton(dayButton,strDay.c_str(),true);
      }
      else
      if(selectedTimePartButton == monthButton)
      {
        controllerTime.month = stepVal(dir, 0,12, controllerTime.month);
        strMonth = addLeadingZero(controllerTime.month);
        screenButtons->relabelButton(monthButton,strMonth.c_str(),true);
      }
      else 
      if(selectedTimePartButton == yearButton)
      {
       if(controllerTime.year == 2000)
       {
          dir = 0;
          controllerTime.year = 2018;
       }
                  
        controllerTime.year = stepVal(dir, 2018,2100, controllerTime.year);
        strYear = controllerTime.year;
        screenButtons->relabelButton(yearButton,strYear.c_str(),true);
      }
     else
     if(selectedTimePartButton == hourButton)
     {
        controllerTime.hour = stepVal(dir, 0,23, controllerTime.hour);
        strHour = addLeadingZero(controllerTime.hour);
        screenButtons->relabelButton(hourButton,strHour.c_str(),true);
     }
     else
     if(selectedTimePartButton == minuteButton)
     {
        controllerTime.minute = stepVal(dir, 0,59, controllerTime.minute);
        strMinute = addLeadingZero(controllerTime.minute);
        screenButtons->relabelButton(minuteButton,strMinute.c_str(),true);
    }
        
      // вычисляем день недели
       int32_t dow;
       byte mArr[12] = {6,2,2,5,0,3,5,1,4,6,2,4};
       dow = (controllerTime.year % 100);
       dow = dow*1.25;
       dow += controllerTime.dayOfMonth;
       dow += mArr[controllerTime.month-1];
       
       if (((controllerTime.year % 4)==0) && (controllerTime.month<3))
         dow -= 1;
         
       while (dow>7)
         dow -= 7;     

      controllerTime.dayOfWeek = dow;
      controllerTimeChanged = true;
      controllerTimeChangedTimer = millis();
        
  #endif // USE_DS3231_REALTIME_CLOCK
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


 if(screenButtons)
 {
  
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
      menuManager->resetIdleTimer();
    
    if(pressed_button == backButton)
    {
      #ifdef USE_DS3231_REALTIME_CLOCK
      if(controllerTimeChanged)
      {
        controllerTimeChanged = false;
        RealtimeClock rtc = MainController->GetClock();
        rtc.setTime(controllerTime);
      }
      #endif
      GlobalSettings* settings = MainController->GetSettings();
      settings->SetSealevel(sealevel);
      menuManager->switchToScreen("SENSORS");
      return;
    }

    if(pressed_button == timeSyncButton)
    {
      menuManager->switchToScreen("TIMESYNC");
    }

    if(pressed_button == decOpenTempButton)
    {
      incOpenTemp(-1);
      return;
    }
    
    if(pressed_button == incOpenTempButton)
    {
      incOpenTemp(1);
      return;
    }
    
    if(pressed_button == decCloseTempButton)
    {
      incCloseTemp(-1);
      return;
    }

    if(pressed_button == incCloseTempButton)
    {
      incCloseTemp(1);  
      return;
    }

    if(pressed_button == decIntervalButton)
    {
      incInterval(-1);
      return;
    }
    
    if(pressed_button == incIntervalButton)
    {
      incInterval(1);
      return;
    }

    if(pressed_button == decSealevelButton)
    {
      incSealevel(-1);      
      return;
    }
    
    if(pressed_button == incSealevelButton)
    {
      incSealevel(1);
      return;
    }
    
    #ifdef USE_DS3231_REALTIME_CLOCK
    
    if(pressed_button == dayButton || pressed_button == monthButton || pressed_button == yearButton
    || pressed_button == hourButton || pressed_button == minuteButton)
    {
      if(selectedTimePartButton != -1)
      {
        if(selectedTimePartButton == pressed_button)
        {
          // та же самая кнопка, что была уже выделена, снимаем выделение
          screenButtons->setButtonBackColor(selectedTimePartButton,TIME_PART_BG_COLOR);
          screenButtons->setButtonFontColor(selectedTimePartButton,TIME_PART_FONT_COLOR);
          screenButtons->drawButton(selectedTimePartButton);
          selectedTimePartButton = -1;
          
          screenButtons->disableButton(incTimePartButton,screenButtons->buttonEnabled(incTimePartButton));
          screenButtons->disableButton(decTimePartButton,screenButtons->buttonEnabled(decTimePartButton));
        }
        else
        {
          // выделили другую кнопку, поэтому для предыдущей надо убрать выделение
          screenButtons->setButtonBackColor(selectedTimePartButton,TIME_PART_BG_COLOR);
          screenButtons->setButtonFontColor(selectedTimePartButton,TIME_PART_FONT_COLOR);
          screenButtons->drawButton(selectedTimePartButton);
          selectedTimePartButton = pressed_button;
          screenButtons->setButtonBackColor(selectedTimePartButton,TIME_PART_SELECTED_BG_COLOR);
          screenButtons->setButtonFontColor(selectedTimePartButton,TIME_PART_SELECTED_FONT_COLOR);
          screenButtons->drawButton(selectedTimePartButton);

          screenButtons->enableButton(incTimePartButton,!screenButtons->buttonEnabled(incTimePartButton));
          screenButtons->enableButton(decTimePartButton,!screenButtons->buttonEnabled(decTimePartButton));
          
        }
      }
      else
      {
        // ничего не выделено, выделяем нажатую кнопку
        selectedTimePartButton = pressed_button; 
        screenButtons->setButtonBackColor(selectedTimePartButton,TIME_PART_SELECTED_BG_COLOR);
        screenButtons->setButtonFontColor(selectedTimePartButton,TIME_PART_SELECTED_FONT_COLOR);
        screenButtons->drawButton(selectedTimePartButton);

        screenButtons->enableButton(incTimePartButton,!screenButtons->buttonEnabled(incTimePartButton));
        screenButtons->enableButton(decTimePartButton,!screenButtons->buttonEnabled(decTimePartButton));
      }

      return;
    }

    if(pressed_button == decTimePartButton || pressed_button == incTimePartButton)
    {
      incDateTimePart(pressed_button);     
      return;
    }

     
     if(controllerTimeChanged)
     {
        if(millis() - controllerTimeChangedTimer > 5000)
        {
          controllerTimeChangedTimer = millis();
          controllerTimeChanged = false;
          RealtimeClock rtc = MainController->GetClock();
          rtc.setTime(controllerTime);
        }
     }
     else
     {
        controllerTimeChangedTimer = millis();
     }

    
    #endif // USE_DS3231_REALTIME_CLOCK        
       
    inited = true;
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  closeTempBox->draw(menuManager);
  openTempBox->draw(menuManager);
  intervalBox->draw(menuManager);
  sealevelBox->draw(menuManager);

  #ifdef USE_DS3231_REALTIME_CLOCK

  UTFT* dc = menuManager->getDC();
  dc->setFont(BigRusFont);
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  menuManager->getRusPrinter()->print(TFT_CURRENTTIME_CAPTION,30,255);  

  RealtimeClock rtc = MainController->GetClock();
  controllerTime = rtc.getTime();  
  updateTimeButtons(controllerTime,true);
    
  #endif

  drawValueInBox(closeTempBox,closeTemp);
  drawValueInBox(openTempBox,openTemp);
  drawValueInBox(intervalBox,interval);
  drawValueInBox(sealevelBox,sealevel);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_SENSORS_PER_SCREEN 9
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTSensorsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSensorsScreen::TFTSensorsScreen()
{
    offset = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSensorsScreen::~TFTSensorsScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  initSensors();
  
  if(sensors.size() <= TFT_SENSORS_PER_SCREEN)
  {
    screenButtons->disableButton(forwardButton);
  }
  else
  {
    if(size_t(offset + TFT_SENSORS_PER_SCREEN) < sensors.size())
    {
      screenButtons->enableButton(forwardButton);
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::initSensors()
{
  sensors.clear();

  AbstractModule* zeroMod = MainController->GetModuleByID("0");
  if(zeroMod)
  {
    size_t tempCount = zeroMod->State.GetStateCount(StateTemperature);
    if(tempCount > 0)
    {
        TFTSensorData dt;
        dt.sensorIndex = 0;
        dt.type = tftSensorSystemTemperature;
        sensors.push_back(dt);
    }
  }
  
  #ifdef USE_TEMP_SENSORS
    AbstractModule* temps = MainController->GetModuleByID("STATE");
    if(temps)
    {
      size_t tempCount = temps->State.GetStateCount(StateTemperature);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorTemperature;
        sensors.push_back(dt);
      }
    }
  #endif // USE_TEMP_SENSORS

  #ifdef USE_HUMIDITY_MODULE
    AbstractModule* hums = MainController->GetModuleByID("HUMIDITY");
    if(hums)
    {
      size_t tempCount = hums->State.GetStateCount(StateTemperature);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorHumidity;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_HUMIDITY_MODULE

  #ifdef USE_LUMINOSITY_MODULE
    AbstractModule* light = MainController->GetModuleByID("LIGHT");
    if(light)
    {
      size_t tempCount = light->State.GetStateCount(StateLuminosity);

      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorLuminosity;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_LUMINOSITY_MODULE

  #ifdef USE_SOIL_MOISTURE_MODULE
    AbstractModule* soil = MainController->GetModuleByID("SOIL");
    if(soil)
    {
      size_t tempCount = soil->State.GetStateCount(StateSoilMoisture);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorSoilMoisture;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_SOIL_MOISTURE_MODULE

  #ifdef USE_PH_MODULE
    if(PHModule)
    {
      size_t tempCount = PHModule->State.GetStateCount(StatePH);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorPH;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_PH_MODULE



  #ifdef USE_CO2_MODULE
    AbstractModule* co2Mod = MainController->GetModuleByID("CO2");
    if(co2Mod)
    {
      size_t tempCount = co2Mod->State.GetStateCount(StateCO2);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorCO2;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_CO2_MODULE  

  #ifdef USE_EC_MODULE
    AbstractModule* ecMod = MainController->GetModuleByID("EC");
    if(ecMod)
    {
      size_t tempCount = ecMod->State.GetStateCount(StateEC);
      for(size_t i=0;i<tempCount;i++)
      {
        TFTSensorData dt;
        dt.sensorIndex = i;
        dt.type = tftSensorEC;
        sensors.push_back(dt);
      }
    }  
  #endif // USE_EC_MODULE  
  
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::setup(TFTMenu* menuManager)
{

    enterKeyButton = -1;
    
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setIconFont(IconsFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

    int screenWidth = dc->getDisplayXSize();
//    int screenHeight = dc->getDisplayYSize();

    const int spacing = 10;

    int buttonWidth = screenWidth - 280;
    int buttonHeight = 55;
    int leftPos = screenWidth - buttonWidth - spacing;
    int backwardTopPos = spacing;
    int forwardTopPos = backwardTopPos + 340 + buttonHeight;

    const int controlsButtonsWidth = 245;
    int controlsButtonsTop = spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( spacing ,  forwardTopPos, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);

    #if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
    windowsSettingsButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_WINDOWS_CAPTION);
    screenButtons->setButtonHasIcon(windowsSettingsButton);
    controlsButtonsTop += buttonHeight + spacing;
    #endif

    #ifdef USE_HEAT_MODULE
    heatSettingsButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, HEAT_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(heatSettingsButton);
    controlsButtonsTop += buttonHeight + spacing;
    #endif

    #ifdef USE_SHADOW_MODULE
    shadowButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SHADOW_CAPTION);
    screenButtons->setButtonHasIcon(shadowButton);
    controlsButtonsTop += buttonHeight + spacing;
    #endif

    #ifdef USE_SCENE_MODULE
    sceneButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SCENE_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(sceneButton);
    controlsButtonsTop += buttonHeight + spacing;
    #endif


    optionsButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_CLOCK_CAPTION);
    screenButtons->setButtonHasIcon(optionsButton);
    controlsButtonsTop += buttonHeight + spacing;

    #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)

    if(!MainController->checkReg())
    {
      enterKeyButton = screenButtons->addButton( spacing ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ENTERKEY_CAPTION);
      controlsButtonsTop += buttonHeight + spacing;
      screenButtons->setButtonFontColor(enterKeyButton,VGA_RED);
    }
    #endif

    

    // теперь добавляем наши кнопки листания списка
    backwardButton = screenButtons->addButton( leftPos,  backwardTopPos, buttonWidth,  buttonHeight, TFT_LIST_SCROLL_UP_CAPTION);
    forwardButton = screenButtons->addButton( leftPos,  forwardTopPos, buttonWidth,  buttonHeight, TFT_LIST_SCROLL_DOWN_CAPTION);

    screenButtons->disableButton(backwardButton);

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::onKeyboardInputResult(const String& input, bool okPressed)
{
  if(okPressed && input.length())
  {
    // ключ введён, сохраняем!
    MainController->GetSettings()->setKey(input);
    
    Vector<const char*> lines;
    
    lines.push_back("ДЛЯ ЗАВЕРШЕНИЯ ПРОЦЕССА РЕГИСТРАЦИИ");
    lines.push_back("ПЕРЕЗАГРУЗИТЕ КОНТРОЛЛЕР.");

    MessageBox->halt("СПАСИБО ЗА РЕГИСТРАЦИЮ!", lines, false);
  }
  else
  {
    TFTScreen->switchToScreen(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;
  
  static uint16_t updateTimer = 0;
  updateTimer += dt;

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
        if(pressed_button == backButton)
        {
          menuManager->switchToScreen("IDLE");
          updateTimer = 0;
          return;
        }
        #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
        else
        if(pressed_button == enterKeyButton)
        {
          Keyboard->show("",10,this, ktFull, true); // показываем полную клавиатуру со стартовой английской раскладкой
          return;
        }    
        #endif
        else
        if(pressed_button == optionsButton)
        {
          menuManager->switchToScreen("OPTIONS");
          return;
        }    
        else
        if(pressed_button == forwardButton)
        {
          offset++;
          if(size_t(offset + TFT_SENSORS_PER_SCREEN) >= sensors.size())
          {
            if(size_t(offset + TFT_SENSORS_PER_SCREEN) > sensors.size())
              offset--;
              
            screenButtons->disableButton(forwardButton,true);
          }
          bool backEnabled = screenButtons->buttonEnabled(backwardButton);
          if(!backEnabled)
          {
            screenButtons->enableButton(backwardButton,true);
          }
          drawSensors(menuManager,false);
          updateTimer = 0;
          return;
        }
        else
        if(pressed_button == backwardButton)
        {
          offset--;
          if(offset < 1)
          {
            screenButtons->disableButton(backwardButton,true);
          }
    
          bool forEnabled = screenButtons->buttonEnabled(forwardButton);
          if(!forEnabled)
          {
            screenButtons->enableButton(forwardButton,true);
          }
    
          drawSensors(menuManager,false);
          updateTimer = 0;
          return;
        }
        #if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
        else
        if(pressed_button == windowsSettingsButton)
        {
          menuManager->switchToScreen("WMS");
        }
        #endif // USE_WINDOW_MANAGE_MODULE
        
        #ifdef USE_HEAT_MODULE
        else
        if(pressed_button == heatSettingsButton)
        {
          menuManager->switchToScreen("HEAT");
        }
        #endif // USE_HEAT_MODULE     
    
        #ifdef USE_SCENE_MODULE
          if(pressed_button == sceneButton)
          {
            menuManager->switchToScreen("SCN");
            return;
          }
        #endif   
    
        #ifdef USE_SHADOW_MODULE
          if(pressed_button == shadowButton)
          {
            menuManager->switchToScreen("SHADOW");
            return;
          }
        #endif  

   } // if(pressed_button != -1)

 } // if(screenButtons)

 if(updateTimer > TFT_SENSORS_UPDATE_INTERVAL)
 {
  updateTimer = 0;
  drawSensors(menuManager,false,true);
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::drawSensors(TFTMenu* menuManager, bool isBGCleared, bool onlyData)
{
  if(!sensors.size())
    return;
    
  size_t from = offset;
  size_t to = from + TFT_SENSORS_PER_SCREEN;
  if(to > sensors.size())
    to = sensors.size();

  uint8_t row = 0;
  for(size_t i=from;i<to;i++)
  {
    drawSensor(menuManager,row,&(sensors[i]),isBGCleared, onlyData);
    row++;
    yield();
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::drawSensor(TFTMenu* menuManager,uint8_t row, TFTSensorData* data, bool isBGCleared, bool onlyData)
{
  UTFT* dc = menuManager->getDC();
  //int screenWidth = dc->getDisplayXSize();
  dc->setFont(BigRusFont);
  int fontHeight = dc->getFontYsize();
  int fontWidth = dc->getFontXsize();
    
  int rowLeft = 280;
  int rowWidth = 500;
  int rowHeight = fontHeight + 16;
  int rowTop = 90 + row*rowHeight;

  String caption, sensorData;
  sensorData = '-';
  
  switch(data->type)
  {
    case tftSensorSystemTemperature:
    {
      if(!onlyData)
        caption = F("Т-ра системы:");
        
      AbstractModule* mod = MainController->GetModuleByID("0");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateTemperature,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)
    }
    break;
    
    case tftSensorTemperature:
    {
      if(!onlyData)
        caption = TFT_SENSOR_TEMPERATURE_CAPTION;
        
      AbstractModule* mod = MainController->GetModuleByID("STATE");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateTemperature,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)
    }
    break;

    case tftSensorHumidity:
    {
      if(!onlyData)
        caption = TFT_SENSOR_HUMIDITY_CAPTION;

      AbstractModule* mod = MainController->GetModuleByID("HUMIDITY");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateTemperature,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }

        sensorData += '/';

        os = mod->State.GetStateByOrder(StateHumidity,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData += *os;
          sensorData += os->GetUnit();
        }
        else
        {
           sensorData += '-';       
        }
        
      } // if(mod)        
    }   
    break;

    case tftSensorLuminosity:
    {
      if(!onlyData)
        caption = TFT_SENSOR_LUMINOSITY_CAPTION;

      AbstractModule* mod = MainController->GetModuleByID("LIGHT");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateLuminosity,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)        
    }
    break;

    case tftSensorSoilMoisture:
    {
      if(!onlyData)
        caption = TFT_SENSOR_SOIL_CAPTION;

      AbstractModule* mod = MainController->GetModuleByID("SOIL");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateSoilMoisture,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)          
    }
    break;

    case tftSensorPH:
    {
      if(!onlyData)
        caption = F("Датчик pH #");

      AbstractModule* mod = MainController->GetModuleByID("PH");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StatePH,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)          
    }
    break;    

    case tftSensorCO2:
    {
      if(!onlyData)
        caption = F("Датчик CO2 #");

      AbstractModule* mod = MainController->GetModuleByID("CO2");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateCO2,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)          
    }
    break;

    case tftSensorEC:
    {
      if(!onlyData)
        caption = F("Датчик EC #");

      AbstractModule* mod = MainController->GetModuleByID("EC");
      if(mod)
      {
        OneState* os = mod->State.GetStateByOrder(StateEC,data->sensorIndex);
        if(os && os->HasData())
        {
          sensorData = *os;
          sensorData += os->GetUnit();
        }
      } // if(mod)          
    }
    break;    
        
  } // switch

  if(data->type != tftSensorSystemTemperature)
    caption += (data->sensorIndex);
  
  int maxDataWidth = fontWidth*15;
  int dataLen = menuManager->getRusPrinter()->utf8_strlen(sensorData.c_str());
  int dataLeft = (rowLeft + rowWidth) - (fontWidth*dataLen);

  if(!onlyData)
  {
    if(!isBGCleared)
    {
      dc->setColor(INFO_BOX_BACK_COLOR);
      dc->fillRect(rowLeft, rowTop + 2, rowLeft + rowWidth, rowTop + rowHeight - 2);    
      yield();
      dc->setBackColor(INFO_BOX_BACK_COLOR);
      dc->setColor(SENSOR_BOX_FONT_COLOR);
    }
    
    menuManager->getRusPrinter()->print(caption.c_str(), rowLeft,rowTop + (rowHeight-fontHeight)/2);
  }
  else
  {
    if(!isBGCleared)
    {
      int maxDataLeft = (rowLeft + rowWidth) - maxDataWidth;
      dc->setColor(INFO_BOX_BACK_COLOR);
      dc->fillRect(maxDataLeft, rowTop + 2, maxDataLeft + maxDataWidth, rowTop + rowHeight - 2);
      yield();        
    }
  }

  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);
  
  menuManager->getRusPrinter()->print(sensorData.c_str(), dataLeft,rowTop + (rowHeight-fontHeight)/2);
  yield();
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::drawListFrame(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  int screenWidth = dc->getDisplayXSize();
  int fontHeight = dc->getFontYsize();

  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRoundRect(270,80,screenWidth - 10, 390);

  int rowLeft = 280;
  int rowWidth = 500;
  int rowHeight = fontHeight + 16;
  int rowTop = 90;

  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);

  for(uint8_t i=0;i<TFT_SENSORS_PER_SCREEN;i++)
  {
    dc->drawLine(rowLeft, rowTop, rowLeft + rowWidth, rowTop);
    rowTop += rowHeight;
    yield();
    
  }
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSensorsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }
  drawListFrame(menuManager);
  drawSensors(menuManager, true);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWChannelsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWChannelsScreen::TFTWChannelsScreen()
{

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWChannelsScreen::~TFTWChannelsScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  GlobalSettings* settings = MainController->GetSettings();
  binding = settings->GetWMBinding();  
  
  blinkActive = false;
  screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
  screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  for(size_t i=0;i<SUPPORTED_WINDOWS;i++)
  {
    markButton(i,binding.binding[i],false);
  }
  #endif // SUPPORTED_WINDOWS > 0
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::markButton(int button, uint8_t bindTo, bool redraw)
{
  if(!screenButtons)
    return;

   const char* label = WM_SECTION_NOLINK_CAPTION;
   word backColor = VGA_SILVER;
   word fontColor = CHANNEL_BUTTONS_TEXT_COLOR;
   switch(bindTo)
   {
    case 1: 
    {
      label = WM_SECTION_LINK1_CAPTION;
      backColor = WM_SECTION_1_BGCOLOR;
      fontColor = WM_SECTION_1_FONT_COLOR;
    }
    break;

    case 2: 
    {
      label = WM_SECTION_LINK2_CAPTION;
      backColor = WM_SECTION_2_BGCOLOR;
      fontColor = WM_SECTION_2_FONT_COLOR;
    }
    break;

    case 3: 
    {
      label = WM_SECTION_LINK3_CAPTION;
      backColor = WM_SECTION_3_BGCOLOR;
      fontColor = WM_SECTION_3_FONT_COLOR;
    }
    break;

    case 4: 
    {
      label = WM_SECTION_LINK4_CAPTION;
      backColor = WM_SECTION_4_BGCOLOR;
      fontColor = WM_SECTION_4_FONT_COLOR;
    }
    break;

   } // switch

   screenButtons->setButtonBackColor(button, backColor);
   screenButtons->setButtonFontColor(button, fontColor);
   screenButtons->relabelButton(button, label, redraw);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter());
  screenButtons->setTextFont(BigRusFont);
  screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

  int screenWidth = menuManager->getDC()->getDisplayXSize();
  int screenHeight = menuManager->getDC()->getDisplayYSize();
  int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
  const int spacing = 10;
  const int controlButtonHeight = 60;
  const int controlsButtonsWidth = 350;
  int controlsButtonsTop = screenHeight - controlButtonHeight - spacing;

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  
 
    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;    

    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }

       String* sLabel = new String(WM_SECTION_LABEL_PREFIX);
       *sLabel += (i+1);

       labels.push_back(sLabel);
       
       int addedId = screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  controlButtonHeight, WM_SECTION_NOLINK_CAPTION);       
       screenButtons->setButtonBackColor(addedId, VGA_SILVER);
       screenButtons->setButtonFontColor(addedId, CHANNEL_BUTTONS_TEXT_COLOR);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
  
    #endif // SUPPORTED_WINDOWS > 0

     // кнопка назад
     backButton = screenButtons->addButton( computedButtonLeft ,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_BACK_CAPTION);    
     saveButton = screenButtons->addButton( computedButtonLeft + INFO_BOX_V_SPACING +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_SAVE_CAPTION);
     screenButtons->disableButton(saveButton);
   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  settings->SetWMBinding(binding);

  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadWindowsSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)


 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      
      menuManager->resetIdleTimer();
 
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("WMS");
        return;
      }
      else
      if(pressed_button == saveButton)
      {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;        
      }    
      #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
      else
      {
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true); 

          int16_t bind = binding.binding[pressed_button];
          if(bind < 0)
            bind = 0;
          
          bind++;
          
          if(bind > 4)
            bind = 0;

         binding.binding[pressed_button] = bind;

          markButton(pressed_button, bind, true);
      }
      #endif // SUPPORTED_WINDOWS > 0


    } // if(pressed_button != -1)
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::drawLabels(TFTMenu* menuManager)
{
 #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0


    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;

    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    UTFT* dc = menuManager->getDC();
    int screenWidth = dc->getDisplayXSize();
    dc->setFont(BigRusFont);
    dc->setColor(TFT_FONT_COLOR);
    dc->setBackColor(TFT_BACK_COLOR);
   
    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и рисуем подписи к кнопкам
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }


        String* label = labels[i];
        rusPrinter->print(label->c_str(),curButtonLeft,buttonsTop - rowsSpacing);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
 
 #endif  // SUPPORTED_WINDOWS > 0

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWChannelsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  drawLabels(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTOrientationScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTOrientationScreen::TFTOrientationScreen()
{

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTOrientationScreen::~TFTOrientationScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  GlobalSettings* settings = MainController->GetSettings();
  binding = settings->GetOrientationBinding();  
  
  blinkActive = false;
  screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
  screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  for(size_t i=0;i<SUPPORTED_WINDOWS;i++)
  {
    markButton(i,binding.binding[i],false);
  }
  #endif // SUPPORTED_WINDOWS > 0
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::markButton(int button, uint8_t bindTo, bool redraw)
{
  if(!screenButtons)
    return;

   const char* label = WM_SECTION_NOLINK_CAPTION;
   word backColor = VGA_SILVER;
   word fontColor = CHANNEL_BUTTONS_TEXT_COLOR;
   switch(bindTo)
   {
    case 1: 
    {
      label = WM_EAST_CAPTION;
      backColor = WM_SECTION_1_BGCOLOR;
      fontColor = WM_SECTION_1_FONT_COLOR;
    }
    break;

    case 2: 
    {
      label = WM_SOUTH_CAPTION;
      backColor = WM_SECTION_2_BGCOLOR;
      fontColor = WM_SECTION_2_FONT_COLOR;
    }
    break;

    case 3: 
    {
      label = WM_WEST_CAPTION;
      backColor = WM_SECTION_3_BGCOLOR;
      fontColor = WM_SECTION_3_FONT_COLOR;
    }
    break;

    case 4: 
    {
      label = WM_NORTH_CAPTION;
      backColor = WM_SECTION_4_BGCOLOR;
      fontColor = WM_SECTION_4_FONT_COLOR;
    }
    break;

   } // switch

   screenButtons->setButtonBackColor(button, backColor);
   screenButtons->setButtonFontColor(button, fontColor);
   screenButtons->relabelButton(button, label, redraw);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter());
  screenButtons->setTextFont(BigRusFont);
  screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

  int screenWidth = menuManager->getDC()->getDisplayXSize();
  int screenHeight = menuManager->getDC()->getDisplayYSize();
  int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
  const int spacing = 10;
  const int controlButtonHeight = 60;
  const int controlsButtonsWidth = 350;
  int controlsButtonsTop = screenHeight - controlButtonHeight - spacing;

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  
 
    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;    

    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }

       String* sLabel = new String(WM_SECTION_LABEL_PREFIX);
       *sLabel += (i+1);

       labels.push_back(sLabel);
       
       int addedId = screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  controlButtonHeight, WM_SECTION_NOLINK_CAPTION);       
       screenButtons->setButtonBackColor(addedId, VGA_SILVER);
       screenButtons->setButtonFontColor(addedId, CHANNEL_BUTTONS_TEXT_COLOR);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
  
    #endif // SUPPORTED_WINDOWS > 0

     // кнопка назад
     backButton = screenButtons->addButton( computedButtonLeft ,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_BACK_CAPTION);    
     saveButton = screenButtons->addButton( computedButtonLeft + INFO_BOX_V_SPACING +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_SAVE_CAPTION);
     screenButtons->disableButton(saveButton);
   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  settings->SetOrientationBinding(binding);

    //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadWindowsSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)


 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      
      menuManager->resetIdleTimer();
 
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("WMS");
        return;
      }
      else
      if(pressed_button == saveButton)
      {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;        
      }    
      #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
      else
      {
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true); 

          int16_t bind = binding.binding[pressed_button];
          if(bind < 0 || bind > 4)
            bind = 0;
          
          bind++;
          
          if(bind > 4)
            bind = 0;

         binding.binding[pressed_button] = bind;

          markButton(pressed_button, bind, true);
      }
      #endif // SUPPORTED_WINDOWS > 0


    } // if(pressed_button != -1)
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::drawLabels(TFTMenu* menuManager)
{
 #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0


    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;

    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    UTFT* dc = menuManager->getDC();
    int screenWidth = dc->getDisplayXSize();
    dc->setFont(BigRusFont);
    dc->setColor(TFT_FONT_COLOR);
    dc->setBackColor(TFT_BACK_COLOR);
   
    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и рисуем подписи к кнопкам
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }


        String* label = labels[i];
        rusPrinter->print(label->c_str(),curButtonLeft,buttonsTop - rowsSpacing);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
 
 #endif  // SUPPORTED_WINDOWS > 0

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTOrientationScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  drawLabels(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTRainScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTRainScreen::TFTRainScreen()
{

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTRainScreen::~TFTRainScreen()
{
 delete screenButtons;
 for(size_t i=0;i<labels.size();i++)
 {
  delete labels[i];
 }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  GlobalSettings* settings = MainController->GetSettings();
  binding = settings->GetRainBinding();  
  
  blinkActive = false;
  screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
  screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  for(size_t i=0;i<SUPPORTED_WINDOWS;i++)
  {
    markButton(i,binding.binding[i],false);
  }
  #endif // SUPPORTED_WINDOWS > 0
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::markButton(int button, uint8_t bindTo, bool redraw)
{
  if(!screenButtons)
    return;

   const char* label = WM_SECTION_NOLINK_CAPTION;
   word backColor = VGA_SILVER;
   word fontColor = CHANNEL_BUTTONS_TEXT_COLOR;
   switch(bindTo)
   {
    case 1: 
    {
      label = WM_RAIN_CLOSE_CAPTION;
      backColor = WM_RAIN_BGCOLOR;
      fontColor = WM_RAIN_FONT_COLOR;
    }
    break;

   } // switch

   screenButtons->setButtonBackColor(button, backColor);
   screenButtons->setButtonFontColor(button, fontColor);
   screenButtons->relabelButton(button, label, redraw);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),menuManager->getRusPrinter());
  screenButtons->setTextFont(BigRusFont);
  screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

  int screenWidth = menuManager->getDC()->getDisplayXSize();
  int screenHeight = menuManager->getDC()->getDisplayYSize();
  int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
  const int spacing = 10;
  const int controlButtonHeight = 60;
  const int controlsButtonsWidth = 350;
  int controlsButtonsTop = screenHeight - controlButtonHeight - spacing;

  #if defined(USE_TEMP_SENSORS) &&  SUPPORTED_WINDOWS > 0
  
 
    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;
       
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и добавляем наши кнопки - дя каждого канала - по кнопке
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }

       String* sLabel = new String(WM_SECTION_LABEL_PREFIX);
       *sLabel += (i+1);

       labels.push_back(sLabel);
       
       int addedId = screenButtons->addButton(curButtonLeft ,  buttonsTop, CHANNELS_BUTTON_WIDTH,  controlButtonHeight, WM_SECTION_NOLINK_CAPTION);       
       screenButtons->setButtonBackColor(addedId, VGA_SILVER);
       screenButtons->setButtonFontColor(addedId, CHANNEL_BUTTONS_TEXT_COLOR);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
  
    #endif // SUPPORTED_WINDOWS > 0

     // кнопка назад
     backButton = screenButtons->addButton( computedButtonLeft ,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_BACK_CAPTION);    
     saveButton = screenButtons->addButton( computedButtonLeft + INFO_BOX_V_SPACING +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_SAVE_CAPTION);
     screenButtons->disableButton(saveButton);
   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  settings->SetRainBinding(binding);

    //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadWindowsSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)


 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      
      menuManager->resetIdleTimer();
 
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("WMS");
        return;
      }
      else
      if(pressed_button == saveButton)
      {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;        
      }    
      #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0
      else
      {
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true); 

          uint8_t bind = binding.binding[pressed_button];
          if(bind == 0xFF)
            bind = 0;
          
          bind++;
          
          if(bind > 1)
            bind = 0;

         binding.binding[pressed_button] = bind;

          markButton(pressed_button, bind, true);
      }
      #endif // SUPPORTED_WINDOWS > 0


    } // if(pressed_button != -1)
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::drawLabels(TFTMenu* menuManager)
{
 #if defined(USE_TEMP_SENSORS) && SUPPORTED_WINDOWS > 0


    int rowsSpacing = 20;
    int buttonsTop = INFO_BOX_V_SPACING + rowsSpacing;

    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    UTFT* dc = menuManager->getDC();
    int screenWidth = dc->getDisplayXSize();
    dc->setFont(BigRusFont);
    dc->setColor(TFT_FONT_COLOR);
    dc->setBackColor(TFT_BACK_COLOR);
   
    int computedButtonLeft = (screenWidth - (CHANNELS_BUTTON_WIDTH*CHANNELS_BUTTONS_PER_LINE) - ((CHANNELS_BUTTONS_PER_LINE-1)*INFO_BOX_V_SPACING))/2;
    int curButtonLeft = computedButtonLeft;
  
    // теперь проходимся по кол-ву каналов и рисуем подписи к кнопкам
    for(int i=0;i<SUPPORTED_WINDOWS;i++)
    {
       if( i > 0 && !(i%CHANNELS_BUTTONS_PER_LINE))
       {
        buttonsTop += CHANNELS_BUTTON_HEIGHT + INFO_BOX_V_SPACING + rowsSpacing;
        curButtonLeft = computedButtonLeft;
       }


        String* label = labels[i];
        rusPrinter->print(label->c_str(),curButtonLeft,buttonsTop - rowsSpacing);
       
       curButtonLeft += CHANNELS_BUTTON_WIDTH + INFO_BOX_V_SPACING;


    } // for
    
 
 #endif  // SUPPORTED_WINDOWS > 0

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTRainScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  drawLabels(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWMScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWMScreen::TFTWMScreen(uint8_t _channel)
{
  percents25Box = NULL;
  percents50Box = NULL;
  percents75Box = NULL;
  percents100Box = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;

  percents25Temp = WM_25PERCENTS_OPEN_TEMP;
  percents50Temp = WM_50PERCENTS_OPEN_TEMP;
  percents75Temp = WM_75PERCENTS_OPEN_TEMP;
  percents100Temp = WM_100PERCENTS_OPEN_TEMP;
  histeresis = WM_DEFAULT_HISTERESIS;
  sensor = 0;
  isActive = WM_ACTIVE_FLAG;

  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWMScreen::~TFTWMScreen()
{
 delete screenButtons;
 delete percents25Box;
 delete percents50Box;
 delete percents75Box;
 delete percents100Box;
 delete histeresisBox;
 delete sensorBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    percents25Temp = settings->Get25PercentsOpenTemp(channel);
    percents50Temp = settings->Get50PercentsOpenTemp(channel);
    percents75Temp = settings->Get75PercentsOpenTemp(channel);
    percents100Temp = settings->Get100PercentsOpenTemp(channel);
    histeresis = settings->GetWMHisteresis(channel);
    sensor = settings->GetWMSensor(channel);  
    isActive = settings->GetWMActive(channel);

    if(isActive)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    else
      tempSensorsCount = 0;

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    else
      humiditySensorsCount = 0; 


    getSensorData();

    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::getSensorData()
{
  sensorDataString = F("^ - ");
  sensorDataString += char('~' + 1);
  sensorDataString += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = sensor;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   sensorDataString = F("^");

    sensorDataString += tmp.Current;

   sensorDataString += char('~' + 1);
   sensorDataString += 'C';
   
  }

   while(sensorDataString.length() < 10)
    sensorDataString += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::drawSensorData(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(sensorDataString.c_str(), sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::setup(TFTMenu* menuManager)
{


    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    dec25PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   percents25Box = new TFTInfoBox(WM_T25_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   inc25PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   dec50PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   percents50Box = new TFTInfoBox(WM_T50_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   inc50PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   dec75PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   percents75Box = new TFTInfoBox(WM_T75_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   inc75PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   dec100PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   percents100Box = new TFTInfoBox(WM_T100_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   inc100PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   histeresisBox = new TFTInfoBox(WM_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox(WM_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(WM_SENSOR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
   

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->Set25PercentsOpenTemp(channel,percents25Temp);
  settings->Set50PercentsOpenTemp(channel,percents50Temp);
  settings->Set75PercentsOpenTemp(channel,percents75Temp);
  settings->Set100PercentsOpenTemp(channel,percents100Temp);
  settings->SetWMHisteresis(channel,histeresis);
  settings->SetWMSensor(channel,sensor);
  settings->SetWMActive(channel,isActive);

  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadWindowsSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == dec25PercentsButton || buttonID == inc25PercentsButton || buttonID == dec50PercentsButton
  || buttonID == inc50PercentsButton || buttonID == dec75PercentsButton || buttonID == inc75PercentsButton
  || buttonID == dec100PercentsButton || buttonID == inc100PercentsButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::onTick()
{
  if(tickerButton == dec25PercentsButton)
    inc25Temp(-3);
  else
  if(tickerButton == inc25PercentsButton)
    inc25Temp(3);
  else
  if(tickerButton == dec50PercentsButton)
    inc50Temp(-3);
  else
  if(tickerButton == inc50PercentsButton)
    inc50Temp(3);
  else
  if(tickerButton == dec75PercentsButton)
    inc75Temp(-3);
  else
  if(tickerButton == inc75PercentsButton)
    inc75Temp(3);
  else
  if(tickerButton == dec100PercentsButton)
    inc100Temp(-3);
  else
  if(tickerButton == inc100PercentsButton)
    inc100Temp(3);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::inc25Temp(int val)
{
  int16_t old = percents25Temp;
  
  percents25Temp+=val;

  if(percents25Temp < 0)
    percents25Temp = 0;
      
  if(percents25Temp > 99)
    percents25Temp = 99;

  if(percents25Temp != old)
    drawValueInBox(percents25Box,percents25Temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::inc50Temp(int val)
{
  int16_t old = percents50Temp;
  
  percents50Temp+=val;

  if(percents50Temp < 0)
    percents50Temp = 0;
  
  if(percents50Temp > 99)
    percents50Temp = 99;

  if(percents50Temp != old)
    drawValueInBox(percents50Box,percents50Temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::inc75Temp(int val)
{
  int16_t old = percents75Temp;
  
  percents75Temp+=val;

  if(percents75Temp < 0)
    percents75Temp = 0;
  
  if(percents75Temp > 99)
    percents75Temp = 99;

  if(percents75Temp != old)
    drawValueInBox(percents75Box,percents75Temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::inc100Temp(int val)
{
  int16_t old = percents100Temp;
  
  percents100Temp+=val;

  if(percents100Temp < 0)
    percents100Temp = 0;
  
  if(percents100Temp > 99)
    percents100Temp = 99;

  if(percents100Temp != old)
    drawValueInBox(percents100Box,percents100Temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData();
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager);
    }

    sensorUpdateTimer = millis();
  }    

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("WMS");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {

              if(isActive)
                isActive = false;
              else
                isActive = true;

              if(isActive)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == dec25PercentsButton)
          {
            inc25Temp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == inc25PercentsButton)
          {
            inc25Temp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == dec50PercentsButton)
          {
            inc50Temp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == inc50PercentsButton)
          {
            inc50Temp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == dec75PercentsButton)
          {
            inc75Temp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == inc75PercentsButton)
          {
            inc75Temp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == dec100PercentsButton)
          {
            inc100Temp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == inc100PercentsButton)
          {
            inc100Temp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHisteresisButton)
          {
            histeresis-=5;
            if(histeresis < 0)
              histeresis = 0;
      
            drawValueInBox(histeresisBox,formatHisteresis());
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            histeresis+=5;
            if(histeresis > 50)
              histeresis = 50;
      
            drawValueInBox(histeresisBox,formatHisteresis());
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            sensor--;
            if(sensor < 0)
              sensor = 0;
      
            drawValueInBox(sensorBox,sensor);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData();
            drawSensorData(menuManager);
          }
          else if(pressed_button == incSensorButton)
          {
            sensor++;
            if(sensor >= (humiditySensorsCount + tempSensorsCount))
              sensor = (humiditySensorsCount + tempSensorsCount) - 1;
		  
			if(sensor < 0)
              sensor = 0;
      
            drawValueInBox(sensorBox,sensor);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            getSensorData();
            drawSensorData(menuManager);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  percents25Box->draw(menuManager);
  drawValueInBox(percents25Box,percents25Temp);

  percents50Box->draw(menuManager);
  drawValueInBox(percents50Box,percents50Temp);

  percents75Box->draw(menuManager);
  drawValueInBox(percents75Box,percents75Temp);

  percents100Box->draw(menuManager);
  drawValueInBox(percents100Box,percents100Temp);
  
  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,sensor);

  drawSensorData(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTWMScreen::formatHisteresis()
{
  int16_t val = histeresis/10;
  int16_t fract = histeresis%10;

  String result;
  result += val;
  result += '.';
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWindScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWindScreen::TFTWindScreen()
{
  windSpeedBox = NULL;
  hurricaneSpeedBox = NULL;

  windSpeed = WM_DEFAULT_WIND_SPEED;
  hurricaneSpeed = WM_DEFAULT_HURRICANE_SPEED;

  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWindScreen::~TFTWindScreen()
{
 delete screenButtons;
 delete windSpeedBox;
 delete hurricaneSpeedBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    windSpeed = settings->GetWindSpeed();
    hurricaneSpeed = settings->GetHurricaneSpeed();

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::setup(TFTMenu* menuManager)
{


    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    //int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    //int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decWindSpeedButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   windSpeedBox = new TFTInfoBox(WM_WIND_SPEED_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incWindSpeedButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHurricaneSpeedButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   hurricaneSpeedBox = new TFTInfoBox(WM_HURRICANE_SPEED_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHurricaneSpeedButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

/*
   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   dec75PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   percents75Box = new TFTInfoBox(WM_T75_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   inc75PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   dec100PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   percents100Box = new TFTInfoBox(WM_T100_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   inc100PercentsButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   histeresisBox = new TFTInfoBox(WM_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox(WM_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(WM_SENSOR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
  */ 

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetWindSpeed(windSpeed);
  settings->SetHurricaneSpeed(hurricaneSpeed);

  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadWindowsSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decWindSpeedButton || buttonID == incWindSpeedButton || buttonID == decHurricaneSpeedButton
  || buttonID == incHurricaneSpeedButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::onTick()
{
  if(tickerButton == decWindSpeedButton)
    incWindSpeed(-3);
  else
  if(tickerButton == incWindSpeedButton)
    incWindSpeed(3);
  else
  if(tickerButton == decHurricaneSpeedButton)
    incHurricaneSpeed(-3);
  else
  if(tickerButton == incHurricaneSpeedButton)
    incHurricaneSpeed(3);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::incWindSpeed(int val)
{
  int16_t old = windSpeed;
  windSpeed+=val;

  if(windSpeed < 0)
    windSpeed = 0;
  
  if(windSpeed > 150)
    windSpeed = 150;

  if(windSpeed != old)
    drawValueInBox(windSpeedBox,windSpeed);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::incHurricaneSpeed(int val)
{
  int16_t old = hurricaneSpeed;
  hurricaneSpeed+=val;

  if(hurricaneSpeed < 0)
    hurricaneSpeed = 0;
  
  if(hurricaneSpeed > 150)
    hurricaneSpeed = 150;

  if(hurricaneSpeed != old)
    drawValueInBox(hurricaneSpeedBox,hurricaneSpeed);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("WMS");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }          
          else if(pressed_button == decWindSpeedButton)
          {
            incWindSpeed(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incWindSpeedButton)
          {
            incWindSpeed(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHurricaneSpeedButton)
          {
            incHurricaneSpeed(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHurricaneSpeedButton)
          {
            incHurricaneSpeed(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          
    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWindScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  windSpeedBox->draw(menuManager);
  drawValueInBox(windSpeedBox,windSpeed);

  hurricaneSpeedBox->draw(menuManager);
  drawValueInBox(hurricaneSpeedBox,hurricaneSpeed);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTWMenuScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWMenuScreen::TFTWMenuScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTWMenuScreen::~TFTWMenuScreen()
{
 delete screenButtons;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMenuScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMenuScreen::setup(TFTMenu* menuManager)
{


    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
  
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();


    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    
    const int spacing = 10;

    int controlButtonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - leftPos*2)/3;
    int controlsButtonsTop = screenHeight - controlButtonHeight - spacing;
    
    int channelsButtonsWidth = (screenWidth - leftPos*2 - spacing)/2;
    int channelsButtonsTop = 20;
    int channelButtonHeight = TFT_ARROW_BUTTON_HEIGHT;
    
   // первая - кнопка назад
    backButton = screenButtons->addButton( leftPos ,  controlsButtonsTop, controlsButtonsWidth,  controlButtonHeight, WM_BACK_CAPTION);


    // первая строка
    temp1Button = screenButtons->addButton( leftPos ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_TSECTION_1_CAPTION);
    screenButtons->setButtonBackColor(temp1Button,WM_SECTION_1_BGCOLOR);
    screenButtons->setButtonFontColor(temp1Button,WM_SECTION_1_FONT_COLOR);
    
    windowsButton = screenButtons->addButton( leftPos + channelsButtonsWidth + spacing ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_WLIST_CAPTION);

    channelsButtonsTop += channelButtonHeight + spacing;

    // вторая строка
    temp2Button = screenButtons->addButton( leftPos ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_TSECTION_2_CAPTION);
    screenButtons->setButtonBackColor(temp2Button,WM_SECTION_2_BGCOLOR);
    screenButtons->setButtonFontColor(temp2Button,WM_SECTION_2_FONT_COLOR);

    orientationButton = screenButtons->addButton( leftPos + channelsButtonsWidth + spacing ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_ORIENTATION_CAPTION);

    channelsButtonsTop += channelButtonHeight + spacing;

    // третья строка
    temp3Button = screenButtons->addButton( leftPos ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_TSECTION_3_CAPTION);
    screenButtons->setButtonBackColor(temp3Button,WM_SECTION_3_BGCOLOR);
    screenButtons->setButtonFontColor(temp3Button,WM_SECTION_3_FONT_COLOR);

    rainButton = screenButtons->addButton( leftPos + channelsButtonsWidth + spacing ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_RAIN_CAPTION);

    channelsButtonsTop += channelButtonHeight + spacing;

    // четвертая строка
    temp4Button = screenButtons->addButton( leftPos ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_TSECTION_4_CAPTION);
    screenButtons->setButtonBackColor(temp4Button,WM_SECTION_4_BGCOLOR);
    screenButtons->setButtonFontColor(temp4Button,WM_SECTION_4_FONT_COLOR);

    windButton = screenButtons->addButton( leftPos + channelsButtonsWidth + spacing ,  channelsButtonsTop, channelsButtonsWidth,  channelButtonHeight, WM_WIND_CAPTION);

    channelsButtonsTop += channelButtonHeight + spacing;

   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMenuScreen::update(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("SENSORS");
        return;
      }
      else
      if(pressed_button == temp1Button)
      {
        menuManager->switchToScreen("WMS1");
        return;        
      }
      else
      if(pressed_button == temp2Button)
      {
        menuManager->switchToScreen("WMS2");
        return;        
      }
      else
      if(pressed_button == temp3Button)
      {
        menuManager->switchToScreen("WMS3");
        return;        
      }
      else
      if(pressed_button == temp4Button)
      {
        menuManager->switchToScreen("WMS4");
        return;        
      }
      else 
      if(pressed_button == windowsButton)
      {
        menuManager->switchToScreen("WMW");
        return;         
      }
      else 
      if(pressed_button == rainButton)
      {
        menuManager->switchToScreen("RAIN");
        return;         
      }   
      else 
      if(pressed_button == orientationButton)
      {
        menuManager->switchToScreen("ORIENT");
        return;         
      }
      else 
      if(pressed_button == windButton)
      {
        menuManager->switchToScreen("WIND");
        return;         
      }
      
      
    
    } // if(pressed_button != -1)
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTWMenuScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WINDOW_MANAGE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SHADOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTShadowSettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowSettingsScreen::TFTShadowSettingsScreen(uint8_t _channel)
{
  luxBox = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;
  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowSettingsScreen::~TFTShadowSettingsScreen()
{
 delete screenButtons;
 delete luxBox;
 delete histeresisBox;
 delete sensorBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    shadowSettings = settings->GetShadowSettings(channel);

    if(shadowSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("LIGHT");
    if(module)
      lumSensorsCount = module->State.GetStateCount(StateLuminosity);
    else
      lumSensorsCount = 0;

    getSensorData(0, sensorDataString);

    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::getSensorData(uint8_t which, String& result)
{

  result = F("- ");
  result += F("lux     ");

  const char* moduleName = "LIGHT";
  uint16_t sensorIndex = shadowSettings.sensorIndex;
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateLuminosity,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   LuminosityPair tmp = *sensorState;

   result = tmp.Current;
   result += F(" lux");
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    //int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   luxBox = new TFTInfoBox(SHADOW_BORDER_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox(SHADOW_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   sensorBox = new TFTInfoBox(HEAT_SENSORINDEX_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_SENSORINDEX_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   /*
  
   decAirSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   airSensorBox = new TFTInfoBox(HEAT_AIR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop2 = textBoxTopPos;
   sensorDataLeft2 = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_AIR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incAirSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   histeresisBox = new TFTInfoBox(HEAT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox(HEAT_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_SENSOR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
   */

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetShadowSettings(channel,shadowSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadShadowSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decLuxButton || buttonID == incLuxButton || buttonID == decHisteresisButton
  || buttonID == incHisteresisButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::onTick()
{
  if(tickerButton == decLuxButton)
    incLux(-200);
  else
  if(tickerButton == incLuxButton)
    incLux(200);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-7);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(7);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::incLux(int val)
{
  int16_t old = shadowSettings.lux;
  
  shadowSettings.lux+=val;

  if(shadowSettings.lux < 0)
    shadowSettings.lux = 0;
  
  if(shadowSettings.lux > 65000)
    shadowSettings.lux = 65000;

  if(shadowSettings.lux != old)
    drawValueInBox(luxBox,shadowSettings.lux);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::incHisteresis(int val)
{
  int16_t old = shadowSettings.histeresis;
  
  shadowSettings.histeresis+=val;

  if(shadowSettings.histeresis < 0)
    shadowSettings.histeresis = 0;
  
  if(shadowSettings.histeresis > 30000)
    shadowSettings.histeresis = 30000;

  if(shadowSettings.histeresis != old)
    drawValueInBox(histeresisBox,shadowSettings.histeresis);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(0,sensorDataString);
                
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }     
  

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("SHADOW");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(shadowSettings.active)
                shadowSettings.active = false;
              else
                shadowSettings.active = true;
                
              if(shadowSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decLuxButton)
          {
            incLux(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incLuxButton)
          {
            incLux(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            shadowSettings.sensorIndex--;
            if(shadowSettings.sensorIndex < 0)
              shadowSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,shadowSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(0,sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            shadowSettings.sensorIndex++;
            if(shadowSettings.sensorIndex >= lumSensorsCount)
              shadowSettings.sensorIndex = lumSensorsCount - 1;
		  
			if(shadowSettings.sensorIndex < 0 || shadowSettings.sensorIndex >= lumSensorsCount)
              shadowSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,shadowSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(0,sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  luxBox->draw(menuManager);
  drawValueInBox(luxBox,shadowSettings.lux);

  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,shadowSettings.histeresis);

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,shadowSettings.sensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTShadowScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowScreen::TFTShadowScreen()
{
  workTimeBox1 = NULL;
  workTimeBox2 = NULL;
  workTimeBox3 = NULL;

  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowScreen::~TFTShadowScreen()
{
 delete screenButtons;
 delete workTimeBox1;
 delete workTimeBox2;
 delete workTimeBox3;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decWorkTimeButton1 || buttonID == decWorkTimeButton2 || buttonID == decWorkTimeButton3
  || buttonID == incWorkTimeButton1 || buttonID == incWorkTimeButton2 || buttonID == incWorkTimeButton3)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::onTick()
{
  if(tickerButton == decWorkTimeButton1)
    incWorkTime1(-3);
  else
  if(tickerButton == decWorkTimeButton2)
    incWorkTime2(-3);
  else
  if(tickerButton == decWorkTimeButton3)
    incWorkTime3(-3);
  else
  if(tickerButton == incWorkTimeButton1)
    incWorkTime1(3);
  else
  if(tickerButton == incWorkTimeButton2)
    incWorkTime2(3);
  else
  if(tickerButton == incWorkTimeButton3)
    incWorkTime3(3);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    shadowDriveWorkTime1 = settings->GetShadowDriveWorkTime(0);
    shadowDriveWorkTime2 = settings->GetShadowDriveWorkTime(1);
    shadowDriveWorkTime3 = settings->GetShadowDriveWorkTime(2);

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;

    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

    shadow1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, SHADOW_BUTTON_CAPTION1);
 
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING;

   decWorkTimeButton1 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox1 = new TFTInfoBox(SHADOW_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton1 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

    shadow2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, SHADOW_BUTTON_CAPTION2);
    
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING;

   decWorkTimeButton2 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox2 = new TFTInfoBox(SHADOW_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton2 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

    shadow3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, SHADOW_BUTTON_CAPTION3);
 
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING;

   decWorkTimeButton3 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox3 = new TFTInfoBox(SHADOW_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton3 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetShadowDriveWorkTime(0,shadowDriveWorkTime1);
  settings->SetShadowDriveWorkTime(1,shadowDriveWorkTime2);
  settings->SetShadowDriveWorkTime(2,shadowDriveWorkTime3);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadShadowSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::incWorkTime1(int val)
{
   int16_t oldW = shadowDriveWorkTime1;
  shadowDriveWorkTime1+=val;

  if(shadowDriveWorkTime1 < 1 || shadowDriveWorkTime1 > SHADOW_MAX_WORK_TIME)
    shadowDriveWorkTime1 = 1;
  
  if(shadowDriveWorkTime1 != oldW)
    drawValueInBox(workTimeBox1,shadowDriveWorkTime1);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::incWorkTime2(int val)
{
   int16_t oldW = shadowDriveWorkTime2;
  shadowDriveWorkTime2+=val;

  if(shadowDriveWorkTime2 < 1 || shadowDriveWorkTime2 > SHADOW_MAX_WORK_TIME)
    shadowDriveWorkTime2 = 1;

  if(shadowDriveWorkTime2 != oldW)
    drawValueInBox(workTimeBox2,shadowDriveWorkTime2);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::incWorkTime3(int val)
{
   int16_t oldW = shadowDriveWorkTime3;
  shadowDriveWorkTime3+=val;

  if(shadowDriveWorkTime3 < 1 || shadowDriveWorkTime3 > SHADOW_MAX_WORK_TIME)
    shadowDriveWorkTime3 = 1;

  if(shadowDriveWorkTime3 != oldW)
    drawValueInBox(workTimeBox3,shadowDriveWorkTime3);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("SENSORS");
        return;
      }
     else
      if(pressed_button == saveButton)
      {
        saveSettings();
        blinkSaveSettingsButton(false);
        screenButtons->disableButton(saveButton,true);
        return;
      }
      else
      if(pressed_button == shadow1Button)
      {
        menuManager->switchToScreen("SHADOW1");
        return;        
      }
      else
      if(pressed_button == shadow2Button)
      {
        menuManager->switchToScreen("SHADOW2");
        return;        
      }
      else
      if(pressed_button == shadow3Button)
      {
        menuManager->switchToScreen("SHADOW3");
        return;        
      }
      else 
      if(pressed_button == decWorkTimeButton1)
      {
        incWorkTime1(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton1)
      {
        incWorkTime1(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == decWorkTimeButton2)
      {
        incWorkTime2(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton2)
      {
        incWorkTime2(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == decWorkTimeButton3)
      {
        incWorkTime3(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton3)
      {
        incWorkTime3(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
    } // if(pressed_button != -1)
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  workTimeBox1->draw(menuManager);
  drawValueInBox(workTimeBox1,shadowDriveWorkTime1); 

  workTimeBox2->draw(menuManager);
  drawValueInBox(workTimeBox2,shadowDriveWorkTime2); 

  workTimeBox3->draw(menuManager);
  drawValueInBox(workTimeBox3,shadowDriveWorkTime3); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTShadowDriveScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowDriveScreen::TFTShadowDriveScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTShadowDriveScreen::~TFTShadowDriveScreen()
{
 delete screenButtons;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowDriveScreen::onActivate(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  
  LogicManageModule->GetShadowStatus(0,shadow1Active,shadow1On);
  LogicManageModule->GetShadowStatus(1,shadow2Active,shadow2On);
  LogicManageModule->GetShadowStatus(2,shadow3Active,shadow3On);

  shadowMode1 = LogicManageModule->GetShadowMode(0) == swmAutomatic;
  shadowMode2 = LogicManageModule->GetShadowMode(1) == swmAutomatic;
  shadowMode3 = LogicManageModule->GetShadowMode(2) == swmAutomatic;

  if(shadow1On)
  {
    screenButtons->setButtonBackColor(shadow1OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(shadow1OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow1OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow1OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow1OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow1OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow1OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(shadow1OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }


  if(shadow2On)
  {
    screenButtons->setButtonBackColor(shadow2OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(shadow2OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow2OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow2OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow2OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow2OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow2OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(shadow2OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }


  if(shadow3On)
  {
    screenButtons->setButtonBackColor(shadow3OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(shadow3OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow3OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow3OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow3OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(shadow3OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(shadow3OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(shadow3OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }
  
  if(shadowMode1)
  {
    screenButtons->setButtonBackColor(shadow1ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(shadow1ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow1ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(shadow1ModeButton,MANUAL_MODE_LABEL);    
  }

  if(shadowMode2)
  {
    screenButtons->setButtonBackColor(shadow2ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(shadow2ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow2ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(shadow2ModeButton,MANUAL_MODE_LABEL);    
  }

  if(shadowMode3)
  {
    screenButtons->setButtonBackColor(shadow3ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(shadow3ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(shadow3ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(shadow3ModeButton,MANUAL_MODE_LABEL);    
  }

  if(!shadow1Active)
  {
    screenButtons->disableButton(shadow1OnButton);
    screenButtons->disableButton(shadow1OffButton);
    screenButtons->disableButton(shadow1ModeButton);
  }
  else
  {
    screenButtons->enableButton(shadow1OnButton);
    screenButtons->enableButton(shadow1OffButton);
    screenButtons->enableButton(shadow1ModeButton);    
  }

  if(!shadow2Active)
  {
    screenButtons->disableButton(shadow2OnButton);
    screenButtons->disableButton(shadow2OffButton);
    screenButtons->disableButton(shadow2ModeButton);
  }
  else
  {
    screenButtons->enableButton(shadow2OnButton);
    screenButtons->enableButton(shadow2OffButton);
    screenButtons->enableButton(shadow2ModeButton);    
  }

  if(!shadow3Active)
  {
    screenButtons->disableButton(shadow3OnButton);
    screenButtons->disableButton(shadow3OffButton);
    screenButtons->disableButton(shadow3ModeButton);
  }
  else
  {
    screenButtons->enableButton(shadow3OnButton);
    screenButtons->enableButton(shadow3OffButton);
    screenButtons->enableButton(shadow3ModeButton);    
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowDriveScreen::setup(TFTMenu* menuManager)
{
    
    UTFT* dc = menuManager->getDC();
    
  if(!dc)
  {
    return;
  }
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter(),10);
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

 
    //UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    //int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/3;
    int channelsButtonLeft = initialLeftPos;
    const int v_spacing = 50;

    shadow1OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow1OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow1ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(shadow1ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    topPos += ALL_CHANNELS_BUTTON_HEIGHT + v_spacing;
    channelsButtonLeft = initialLeftPos;

    // вторая строка
    shadow2OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow2OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow2ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(shadow2ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    topPos += ALL_CHANNELS_BUTTON_HEIGHT + v_spacing;
    channelsButtonLeft = initialLeftPos;

    // третья строка
    shadow3OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow3OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, SHADOW_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    shadow3ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(shadow3ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowDriveScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
            menuManager->resetIdleTimer();

   
      if(pressed_button == backButton)
      {
        drawCalled = false;
        menuManager->switchToScreen("DRIVE");
        return;
      }
      else
      if(pressed_button == shadow1ModeButton)
      {
          bool cmd = !shadowMode1;
          LogicManageModule->TurnShadowMode(0, cmd ? swmAutomatic : swmManual);
          return;
      }
      else
      if(pressed_button == shadow2ModeButton)
      {
          bool cmd = !shadowMode2;
          LogicManageModule->TurnShadowMode(1, cmd ? swmAutomatic : swmManual);
          return;
      }
      else
      if(pressed_button == shadow3ModeButton)
      {
          bool cmd = !shadowMode3;
          LogicManageModule->TurnShadowMode(2, cmd ? swmAutomatic : swmManual);
          return;
      }
      else
      if(pressed_button == shadow1OnButton)
      {
        LogicManageModule->TurnShadow(0,true);
        return;
      }
      else
      if(pressed_button == shadow1OffButton)
      {
        LogicManageModule->TurnShadow(0,false);
        return;
      }
      else
      if(pressed_button == shadow2OnButton)
      {
        LogicManageModule->TurnShadow(1,true);
        return;
      }
      else
      if(pressed_button == shadow2OffButton)
      {
        LogicManageModule->TurnShadow(1,false);
        return;
      }
      else
      if(pressed_button == shadow3OnButton)
      {
        LogicManageModule->TurnShadow(2,true);
        return;
      }
      else
      if(pressed_button == shadow3OffButton)
      {
        LogicManageModule->TurnShadow(2,false);
        return;
      }
      
    }

    static uint32_t updTimer = millis();
    if(millis() - updTimer > 250)
    {

        bool thisShadow1Active,thisShadow1On;
        bool thisShadow2Active,thisShadow2On;
        bool thisShadow3Active,thisShadow3On;

        bool thisShadowMode1;
        bool thisShadowMode2;
        bool thisShadowMode3;
  
        LogicManageModule->GetShadowStatus(0,thisShadow1Active,thisShadow1On);
        LogicManageModule->GetShadowStatus(1,thisShadow2Active,thisShadow2On);
        LogicManageModule->GetShadowStatus(2,thisShadow3Active,thisShadow3On);
      
        thisShadowMode1 = LogicManageModule->GetShadowMode(0) == swmAutomatic;
        thisShadowMode2 = LogicManageModule->GetShadowMode(1) == swmAutomatic;
        thisShadowMode3 = LogicManageModule->GetShadowMode(2) == swmAutomatic;

        bool wantRedrawShadowStatus1 = false;
        bool wantRedrawShadowStatus2 = false;
        bool wantRedrawShadowStatus3 = false;

        bool wantRedrawShadowMode1 = false;
        bool wantRedrawShadowMode2 = false;
        bool wantRedrawShadowMode3 = false;

        if(thisShadow1On != shadow1On)
        {
          shadow1On = thisShadow1On;
          if(shadow1On)
          {
            screenButtons->setButtonBackColor(shadow1OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(shadow1OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow1OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow1OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow1OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow1OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow1OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(shadow1OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawShadowStatus1 = true;
          }
        }
      
        if(thisShadow2On != shadow2On)
        {
          shadow2On = thisShadow2On;
          if(shadow2On)
          {
            screenButtons->setButtonBackColor(shadow2OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(shadow2OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow2OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow2OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow2OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow2OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow2OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(shadow2OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawShadowStatus2 = true;
          }
        }
      
        if(thisShadow3On != shadow3On)
        {
          shadow3On = thisShadow3On;
          if(shadow3On)
          {
            screenButtons->setButtonBackColor(shadow3OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(shadow3OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow3OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow3OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow3OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(shadow3OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(shadow3OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(shadow3OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawShadowStatus3 = true;
          }
        }

         if(wantRedrawShadowStatus1)
         {
          screenButtons->drawButton(shadow1OnButton);
          screenButtons->drawButton(shadow1OffButton);
         }
         
         if(wantRedrawShadowStatus2)
         {
          screenButtons->drawButton(shadow2OnButton);
          screenButtons->drawButton(shadow2OffButton);
         }
         
         if(wantRedrawShadowStatus3)
         {
          screenButtons->drawButton(shadow3OnButton);
          screenButtons->drawButton(shadow3OffButton);
         }
               
        if(thisShadowMode1 != shadowMode1)
        {
          shadowMode1 = thisShadowMode1;
          if(shadowMode1)
          {
            screenButtons->setButtonBackColor(shadow1ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(shadow1ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow1ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(shadow1ModeButton,MANUAL_MODE_LABEL);    
          }

          if(drawCalled)
          {
            wantRedrawShadowMode1 = true;
          }
        }

        if(thisShadowMode2 != shadowMode2)
        {
          shadowMode2 = thisShadowMode2;
          if(shadowMode2)
          {
            screenButtons->setButtonBackColor(shadow2ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(shadow2ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow2ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(shadow2ModeButton,MANUAL_MODE_LABEL);    
          }
          if(drawCalled)
          {
            wantRedrawShadowMode2 = true;
          }
        }

        if(thisShadowMode3 != shadowMode3)
        {
          shadowMode3 = thisShadowMode3;
          
          if(shadowMode3)
          {
            screenButtons->setButtonBackColor(shadow3ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(shadow3ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(shadow3ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(shadow3ModeButton,MANUAL_MODE_LABEL);    
          }
          if(drawCalled)
          {
            wantRedrawShadowMode3 = true;
          }
        }

        if(wantRedrawShadowMode1)
        {
          screenButtons->drawButton(shadow1ModeButton);
        }

        if(wantRedrawShadowMode2)
        {
          screenButtons->drawButton(shadow2ModeButton);
        }

        if(wantRedrawShadowMode3)
        {
          screenButtons->drawButton(shadow3ModeButton);
        }

      updTimer = millis();
    }
    
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTShadowDriveScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  // рисуем заголовки
  const int left = 30;
  int top = 15;
  const int v_spacing = 120;

  UTFT* dc = menuManager->getDC();
  UTFTRus* rusPrinter = menuManager->getRusPrinter();
  dc->setFont(BigRusFont);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  dc->setBackColor(TFT_BACK_COLOR);

  if(shadow1Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);
  
  rusPrinter->print(SHADOW_BUTTON_CAPTION1,left,top);
  top += v_spacing;

  if(shadow2Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);
  
  
  rusPrinter->print(SHADOW_BUTTON_CAPTION2,left,top);
  top += v_spacing;


  if(shadow3Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);

  
  rusPrinter->print(SHADOW_BUTTON_CAPTION3,left,top);

  drawCalled = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SHADOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTVentSettingsScreen::TFTVentSettingsScreen(uint8_t _channel)
{
  tempBox = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;
  minWorkTimeBox = NULL;
  maxWorkTimeBox = NULL;
  restTimeBox = NULL;

  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTVentSettingsScreen::~TFTVentSettingsScreen()
{
 delete screenButtons;
 delete tempBox;
 delete histeresisBox;
 delete sensorBox;
 delete minWorkTimeBox;
 delete maxWorkTimeBox;
 delete restTimeBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    ventSettings = settings->GetVentSettings(channel);

    if(ventSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    else
      tempSensorsCount = 0;

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    else
      humiditySensorsCount = 0; 


    getSensorData(sensorDataString);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::getSensorData(String& result)
{

  result = F("^ - ");
  result += char('~' + 1);
  result += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = ventSettings.sensorIndex;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   result = F("^");

   result += tmp.Current;

   result += char('~' + 1);
   result += 'C';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decMinWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   minWorkTimeBox = new TFTInfoBox(VENT_MIN_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incMinWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decMaxWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   maxWorkTimeBox = new TFTInfoBox(VENT_MAX_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incMaxWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decRestTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   restTimeBox = new TFTInfoBox(VENT_REST_TIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incRestTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox(VENT_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_AIR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   decTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   tempBox = new TFTInfoBox(VENT_TEMP_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;   
   incTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;
   
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   histeresisBox = new TFTInfoBox(VENT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetVentSettings(channel,ventSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadVentSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decTempButton || buttonID == incTempButton || buttonID == decHisteresisButton
  || buttonID == incHisteresisButton || buttonID == decMinWorkTimeButton || buttonID == incMinWorkTimeButton
  || buttonID == decMaxWorkTimeButton || buttonID == incMaxWorkTimeButton || buttonID == decRestTimeButton
  || buttonID == incRestTimeButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::onTick()
{
  if(tickerButton == decTempButton)
    incTemp(-3);
  else
  if(tickerButton == incTempButton)
    incTemp(3);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-5);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(5);
  else
  if(tickerButton == decMinWorkTimeButton)
    incMinWorkTime(-30);
  else
  if(tickerButton == incMinWorkTimeButton)
    incMinWorkTime(30);
  else
  if(tickerButton == decMaxWorkTimeButton)
    incMaxWorkTime(-30);
  else
  if(tickerButton == incMaxWorkTimeButton)
    incMaxWorkTime(30);
  else
  if(tickerButton == decRestTimeButton)
    incRestTime(-30);
  else
  if(tickerButton == incRestTimeButton)
    incRestTime(30);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::incTemp(int val)
{
  int16_t old = ventSettings.temp;
  
  ventSettings.temp+=val;

  if(ventSettings.temp < 0)
    ventSettings.temp = 0;
  
  if(ventSettings.temp > 127)
    ventSettings.temp = 127;

  if(ventSettings.temp != old)
    drawValueInBox(tempBox,ventSettings.temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::incHisteresis(int val)
{
    int16_t old = ventSettings.histeresis;
    
    ventSettings.histeresis += val;
    
    if(ventSettings.histeresis < 0)
        ventSettings.histeresis = 0;

    if(ventSettings.histeresis > 100)
        ventSettings.histeresis = 100;
      
    if(ventSettings.histeresis != old)
      drawValueInBox(histeresisBox,formatHisteresis());  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::incMinWorkTime(int val)
{
  int32_t old = ventSettings.minWorkTime;
  
  ventSettings.minWorkTime+=val;

  if(ventSettings.minWorkTime < 0)
    ventSettings.minWorkTime = 0;
  
  if(ventSettings.minWorkTime > VENT_MAX_WORK_TIME)
    ventSettings.minWorkTime = VENT_MAX_WORK_TIME;

  if(ventSettings.minWorkTime != old)
    drawTimeInBox(minWorkTimeBox,ventSettings.minWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::incMaxWorkTime(int val)
{
  int32_t old = ventSettings.maxWorkTime;
  
  ventSettings.maxWorkTime+=val;

  if(ventSettings.maxWorkTime < 0)
    ventSettings.maxWorkTime = 0;
  
  if(ventSettings.maxWorkTime > VENT_MAX_WORK_TIME)
    ventSettings.maxWorkTime = VENT_MAX_WORK_TIME;

  if(ventSettings.maxWorkTime != old)
    drawTimeInBox(maxWorkTimeBox,ventSettings.maxWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::incRestTime(int val)
{
  int32_t old = ventSettings.restTime;
  
  ventSettings.restTime+=val;

  if(ventSettings.restTime < 0)
    ventSettings.restTime = 0;
  
  if(ventSettings.restTime > VENT_MAX_WORK_TIME)
    ventSettings.restTime = VENT_MAX_WORK_TIME;

  if(ventSettings.restTime != old)
    drawTimeInBox(restTimeBox,ventSettings.restTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(sensorDataString);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }     

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("VENT");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(ventSettings.active)
                ventSettings.active = false;
              else
                ventSettings.active = true;
                
              if(ventSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decTempButton)
          {
            incTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incTempButton)
          {
            incTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decMinWorkTimeButton)
          {
            incMinWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incMinWorkTimeButton)
          {
            incMinWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decMaxWorkTimeButton)
          {
            incMaxWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incMaxWorkTimeButton)
          {
            incMaxWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decRestTimeButton)
          {
            incRestTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incRestTimeButton)
          {
            incRestTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            ventSettings.sensorIndex--;
            if(ventSettings.sensorIndex < 0)
              ventSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,ventSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            ventSettings.sensorIndex++;
            if(ventSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              ventSettings.sensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;
		  
		  if(ventSettings.sensorIndex < 0 || ventSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
                ventSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,ventSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  tempBox->draw(menuManager);
  drawValueInBox(tempBox,ventSettings.temp);

  minWorkTimeBox->draw(menuManager);
  drawTimeInBox(minWorkTimeBox,ventSettings.minWorkTime);

  maxWorkTimeBox->draw(menuManager);
  drawTimeInBox(maxWorkTimeBox,ventSettings.maxWorkTime);

  restTimeBox->draw(menuManager);
  drawTimeInBox(restTimeBox,ventSettings.restTime);
  
  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,ventSettings.sensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTVentSettingsScreen::formatHisteresis()
{
  int16_t val = ventSettings.histeresis/10;
  int16_t fract = ventSettings.histeresis%10;

  String result;
  result += val;
  result += '.';
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTVentScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTVentScreen::TFTVentScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTVentScreen::~TFTVentScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton);
  updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton);
  updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*3;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

   channel1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_1_CAPTION);

   int channelsControlsButtonsLeft = leftPos + leftRowButtonsWidth + spacing;
   int channelsControleButtonsWidth = 185;
   int channelsModeButtonsLeft = channelsControlsButtonsLeft + channelsControleButtonsWidth + spacing;   

   channel1OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel1AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);
    
 
   // вторая строка
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

   channel2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_2_CAPTION);

   channel2OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel2AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
    
   // третья строка
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   channel3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_3_CAPTION);
  
   channel3OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel3AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentScreen::updateVentButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw)
{
  Vent* vent = LogicManageModule->getVent(channel);
  if(vent->isOn())
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_ON_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВЫКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВЫКЛ") !=0);
  }
  else
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВКЛ") !=0);    
  }

  VentWorkMode wm = vent->getWorkMode();
  if(wm == vwmAuto)
  {
    screenButtons->setButtonBackColor(btnMode,MODE_ON_COLOR);
    screenButtons->relabelButton(btnMode,"АВТО",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"АВТО") !=0);    
  }
  else
  {
    screenButtons->setButtonBackColor(btnMode,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnMode,"РУЧНОЙ",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"РУЧНОЙ") !=0);        
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {

  Vent* vent1 = LogicManageModule->getVent(0);
  Vent* vent2 = LogicManageModule->getVent(1);
  Vent* vent3 = LogicManageModule->getVent(2);
  
  static bool ventIsOn1 = vent1->isOn();
  static bool ventIsOn2 = vent2->isOn();
  static bool ventIsOn3 = vent3->isOn();

  static VentWorkMode ventMode1 = vent1->getWorkMode();
  static VentWorkMode ventMode2 = vent2->getWorkMode();
  static VentWorkMode ventMode3 = vent3->getWorkMode();
    
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("DRIVE");
        return;
      }

      else
      if(pressed_button == channel1Button)
      {
        menuManager->switchToScreen("VENT1");
        return;        
      }
      else
      if(pressed_button == channel2Button)
      {
        menuManager->switchToScreen("VENT2");
        return;        
      }
      else
      if(pressed_button == channel3Button)
      {
        menuManager->switchToScreen("VENT3");
        return;        
      }
      else
      if(pressed_button == channel1AutoManualButton)
      {
        Vent* vent = LogicManageModule->getVent(0);
        if(vent->getWorkMode() == vwmAuto)
          vent->switchToMode(vwmManual);
        else
          vent->switchToMode(vwmAuto);

        ventMode1 = vent->getWorkMode();

        updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel2AutoManualButton)
      {
        Vent* vent = LogicManageModule->getVent(1);
        if(vent->getWorkMode() == vwmAuto)
          vent->switchToMode(vwmManual);
        else
          vent->switchToMode(vwmAuto);

        ventMode2 = vent->getWorkMode();

        updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel3AutoManualButton)
      {
        Vent* vent = LogicManageModule->getVent(2);
        if(vent->getWorkMode() == vwmAuto)
          vent->switchToMode(vwmManual);
        else
          vent->switchToMode(vwmAuto);

        ventMode3 = vent->getWorkMode();

        updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel1OnOffButton)
      {
        Vent* vent = LogicManageModule->getVent(0);
        bool onFlag = !vent->isOn();
        vent->switchToMode(vwmManual);
        vent->turn(onFlag);

        ventMode1 = vent->getWorkMode();
        ventIsOn1 = vent->isOn();
        
        updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;        
      }
      else
      if(pressed_button == channel2OnOffButton)
      {
        Vent* vent = LogicManageModule->getVent(1);
        bool onFlag = !vent->isOn();
        vent->switchToMode(vwmManual);
        vent->turn(onFlag);

        ventMode2 = vent->getWorkMode();
        ventIsOn2 = vent->isOn();
        
        updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;      
      }
      else
      if(pressed_button == channel3OnOffButton)
      {
        Vent* vent = LogicManageModule->getVent(2);
        bool onFlag = !vent->isOn();
        vent->switchToMode(vwmManual);
        vent->turn(onFlag);

        ventMode3 = vent->getWorkMode();
        ventIsOn3 = vent->isOn();
        
        updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;       
      }
      
    } // if(pressed_button != -1)

    if(ventIsOn1 != vent1->isOn() || ventMode1 != vent1->getWorkMode())
    {
      ventIsOn1 = vent1->isOn();
      ventMode1 = vent1->getWorkMode();
      updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
    }

    if(ventIsOn2 != vent2->isOn() || ventMode2 != vent2->getWorkMode())
    {
      ventIsOn2 = vent2->isOn();
      ventMode2 = vent2->getWorkMode();
      updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
    }

    if(ventIsOn3 != vent3->isOn() || ventMode3 != vent3->getWorkMode())
    {
      ventIsOn3 = vent3->isOn();
      ventMode3 = vent3->getWorkMode();
      updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
    }
    
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTVentScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  drawScreenCaption(menuManager,VENT_SCREEN_CAPTION);

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HUMIDITY_SPRAY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSpraySettingsScreen::TFTSpraySettingsScreen(uint8_t _channel)
{
  sprayOnBox = NULL;
  sprayOffBox = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;
  startTimeBox = NULL;
  endTimeBox = NULL;

  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSpraySettingsScreen::~TFTSpraySettingsScreen()
{
 delete screenButtons;
 delete sprayOnBox;
 delete sprayOffBox;
 delete histeresisBox;
 delete sensorBox;
 delete startTimeBox;
 delete endTimeBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    spraySettings = settings->GetHumiditySpraySettings(channel);

    if(spraySettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);


    AbstractModule* module = MainController->GetModuleByID("HUMIDITY");
    if(module)
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    else
      humiditySensorsCount = 0; 


    getSensorData(sensorDataString);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::getSensorData(String& result)
{

  result = F("h - ");
  result += F("%    ");

  
  AbstractModule* module = MainController->GetModuleByID("HUMIDITY");
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateHumidity,spraySettings.sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   HumidityPair tmp = *sensorState;
   result = F("h");

   result += tmp.Current;

   result += '%';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   startTimeBox = new TFTInfoBox("Начало работы:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   endTimeBox = new TFTInfoBox("Конец работы:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decSprayOnButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   sprayOnBox = new TFTInfoBox("Влажность вкл, %:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incSprayOnButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSprayOffButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sprayOffBox = new TFTInfoBox("Влажность выкл, %:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSprayOffButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox("Гистерезис:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;
   
   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   const char* SBOX_CAPTION = "Датчик:";
   sensorBox = new TFTInfoBox(SBOX_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(SBOX_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetHumiditySpraySettings(channel,spraySettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadHumiditySpraySettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decSprayOnButton || buttonID == incSprayOnButton || buttonID == decSprayOffButton
  || buttonID == incSprayOffButton || buttonID == decHisteresisButton || buttonID == incHisteresisButton
  || buttonID == decStartTimeButton || buttonID == incStartTimeButton || buttonID == decEndTimeButton
  || buttonID == incEndTimeButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::onTick()
{
  if(tickerButton == decSprayOnButton)
    incSprayOn(-3);
  else
  if(tickerButton == incSprayOnButton)
    incSprayOn(3);
  else
    if(tickerButton == decSprayOffButton)
    incSprayOff(-3);
  else
  if(tickerButton == incSprayOffButton)
    incSprayOff(3);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-5);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(5);
  else
  if(tickerButton == decStartTimeButton)
    incStartTime(-30);
  else
  if(tickerButton == incStartTimeButton)
    incStartTime(30);
  else
  if(tickerButton == decEndTimeButton)
    incEndTime(-30);
  else
  if(tickerButton == incEndTimeButton)
    incEndTime(30);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::incSprayOn(int val)
{
  int16_t old = spraySettings.sprayOnValue;
  
  spraySettings.sprayOnValue+=val;

  if(spraySettings.sprayOnValue < 0)
    spraySettings.sprayOnValue = 0;
  
  if(spraySettings.sprayOnValue > 100)
    spraySettings.sprayOnValue = 100;

  if(spraySettings.sprayOnValue != old)
    drawValueInBox(sprayOnBox,spraySettings.sprayOnValue);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::incSprayOff(int val)
{
  int16_t old = spraySettings.sprayOffValue;
  
  spraySettings.sprayOffValue+=val;

  if(spraySettings.sprayOffValue < 0)
    spraySettings.sprayOffValue = 0;
  
  if(spraySettings.sprayOffValue > 100)
    spraySettings.sprayOffValue = 100;

  if(spraySettings.sprayOffValue != old)
    drawValueInBox(sprayOffBox,spraySettings.sprayOffValue);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::incHisteresis(int val)
{
    int16_t old = spraySettings.histeresis;
    
    spraySettings.histeresis += val;
    
    if(spraySettings.histeresis < 0)
        spraySettings.histeresis = 0;

    if(spraySettings.histeresis > 100)
        spraySettings.histeresis = 100;
      
    if(spraySettings.histeresis != old)
      drawValueInBox(histeresisBox,formatHisteresis());  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::incStartTime(int val)
{
  uint32_t old = spraySettings.startWorkTime;
  
  spraySettings.startWorkTime+=val;

  if(spraySettings.startWorkTime < 0)
    spraySettings.startWorkTime = 0;
  
  if(spraySettings.startWorkTime > 1440)
    spraySettings.startWorkTime = 1440;

  if(spraySettings.startWorkTime != old)
    drawTimeInBox(startTimeBox,spraySettings.startWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::incEndTime(int val)
{
  uint32_t old = spraySettings.endWorkTime;
  
  spraySettings.endWorkTime+=val;

  if(spraySettings.endWorkTime < 0)
    spraySettings.endWorkTime = 0;
  
  if(spraySettings.endWorkTime > 1440)
    spraySettings.endWorkTime = 1440;

  if(spraySettings.endWorkTime != old)
    drawTimeInBox(endTimeBox,spraySettings.endWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(sensorDataString);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }     

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("SPRAY");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(spraySettings.active)
                spraySettings.active = false;
              else
                spraySettings.active = true;
                
              if(spraySettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decSprayOnButton)
          {
            incSprayOn(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incSprayOnButton)
          {
            incSprayOn(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSprayOffButton)
          {
            incSprayOff(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incSprayOffButton)
          {
            incSprayOff(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decStartTimeButton)
          {
            incStartTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incStartTimeButton)
          {
            incStartTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decEndTimeButton)
          {
            incEndTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incEndTimeButton)
          {
            incEndTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            spraySettings.sensorIndex--;
            if(spraySettings.sensorIndex < 0)
              spraySettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,spraySettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            spraySettings.sensorIndex++;
            if(spraySettings.sensorIndex >= humiditySensorsCount)
              spraySettings.sensorIndex = humiditySensorsCount - 1;
      
            if(spraySettings.sensorIndex < 0 || spraySettings.sensorIndex >= humiditySensorsCount)
                spraySettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,spraySettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSpraySettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  sprayOnBox->draw(menuManager);
  drawValueInBox(sprayOnBox,spraySettings.sprayOnValue);

  sprayOffBox->draw(menuManager);
  drawValueInBox(sprayOffBox,spraySettings.sprayOffValue);

  startTimeBox->draw(menuManager);
  drawTimeInBox(startTimeBox,spraySettings.startWorkTime);

  endTimeBox->draw(menuManager);
  drawTimeInBox(endTimeBox,spraySettings.endWorkTime);
  
  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,spraySettings.sensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTSpraySettingsScreen::formatHisteresis()
{
  int16_t val = spraySettings.histeresis/10;
  int16_t fract = spraySettings.histeresis%10;

  String result;
  result += val;
  result += '.';
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTSprayScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSprayScreen::TFTSprayScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTSprayScreen::~TFTSprayScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSprayScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  updateSprayButtons(0, channel1OnOffButton, channel1AutoManualButton);
  updateSprayButtons(1, channel2OnOffButton, channel2AutoManualButton);
  updateSprayButtons(2, channel3OnOffButton, channel3AutoManualButton);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSprayScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*3;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

   channel1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_1_CAPTION);

   int channelsControlsButtonsLeft = leftPos + leftRowButtonsWidth + spacing;
   int channelsControleButtonsWidth = 185;
   int channelsModeButtonsLeft = channelsControlsButtonsLeft + channelsControleButtonsWidth + spacing;   

   channel1OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel1AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);
    
 
   // вторая строка
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

   channel2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_2_CAPTION);

   channel2OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel2AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
    
   // третья строка
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   channel3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_3_CAPTION);
  
   channel3OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel3AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSprayScreen::updateSprayButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw)
{
  HumiditySpray* hs = LogicManageModule->getSpray(channel);
  if(hs->isOn())
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_ON_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВЫКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВЫКЛ") !=0);
  }
  else
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВКЛ") !=0);    
  }

  HSMWorkMode wm = hs->getWorkMode();
  if(wm == hsmAuto)
  {
    screenButtons->setButtonBackColor(btnMode,MODE_ON_COLOR);
    screenButtons->relabelButton(btnMode,"АВТО",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"АВТО") !=0);    
  }
  else
  {
    screenButtons->setButtonBackColor(btnMode,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnMode,"РУЧНОЙ",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"РУЧНОЙ") !=0);        
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSprayScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {

  HumiditySpray* spray1 = LogicManageModule->getSpray(0);
  HumiditySpray* spray2 = LogicManageModule->getSpray(1);
  HumiditySpray* spray3 = LogicManageModule->getSpray(2);
  
  static bool sprayIsOn1 = spray1->isOn();
  static bool sprayIsOn2 = spray2->isOn();
  static bool sprayIsOn3 = spray3->isOn();

  static HSMWorkMode sprayMode1 = spray1->getWorkMode();
  static HSMWorkMode sprayMode2 = spray2->getWorkMode();
  static HSMWorkMode sprayMode3 = spray3->getWorkMode();
    
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("DRIVE");
        return;
      }

      else
      if(pressed_button == channel1Button)
      {
        menuManager->switchToScreen("SPRAY1");
        return;        
      }
      else
      if(pressed_button == channel2Button)
      {
        menuManager->switchToScreen("SPRAY2");
        return;        
      }
      else
      if(pressed_button == channel3Button)
      {
        menuManager->switchToScreen("SPRAY3");
        return;        
      }
      else
      if(pressed_button == channel1AutoManualButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(0);
        if(hs->getWorkMode() == hsmAuto)
          hs->switchToMode(hsmManual);
        else
          hs->switchToMode(hsmAuto);

        sprayMode1 = hs->getWorkMode();

        updateSprayButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel2AutoManualButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(1);
        if(hs->getWorkMode() == hsmAuto)
          hs->switchToMode(hsmManual);
        else
          hs->switchToMode(hsmAuto);

        sprayMode2 = hs->getWorkMode();

        updateSprayButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel3AutoManualButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(2);
        if(hs->getWorkMode() == hsmAuto)
          hs->switchToMode(hsmManual);
        else
          hs->switchToMode(hsmAuto);

        sprayMode3 = hs->getWorkMode();

        updateSprayButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel1OnOffButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(0);
        bool onFlag = !hs->isOn();
        hs->switchToMode(hsmManual);
        hs->turn(onFlag);

        sprayMode1 = hs->getWorkMode();
        sprayIsOn1 = hs->isOn();
        
        updateSprayButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;        
      }
      else
      if(pressed_button == channel2OnOffButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(1);
        bool onFlag = !hs->isOn();
        hs->switchToMode(hsmManual);
        hs->turn(onFlag);

        sprayMode2 = hs->getWorkMode();
        sprayIsOn2 = hs->isOn();
        
        updateSprayButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;      
      }
      else
      if(pressed_button == channel3OnOffButton)
      {
        HumiditySpray* hs = LogicManageModule->getSpray(2);
        bool onFlag = !hs->isOn();
        hs->switchToMode(hsmManual);
        hs->turn(onFlag);

        sprayMode3 = hs->getWorkMode();
        sprayIsOn3 = hs->isOn();
        
        updateSprayButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;       
      }
      
    } // if(pressed_button != -1)

    if(sprayIsOn1 != spray1->isOn() || sprayMode1 != spray1->getWorkMode())
    {
      sprayIsOn1 = spray1->isOn();
      sprayMode1 = spray1->getWorkMode();
      updateSprayButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
    }

    if(sprayIsOn2 != spray2->isOn() || sprayMode2 != spray2->getWorkMode())
    {
      sprayIsOn2 = spray2->isOn();
      sprayMode2 = spray2->getWorkMode();
      updateSprayButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
    }

    if(sprayIsOn3 != spray3->isOn() || sprayMode3 != spray3->getWorkMode())
    {
      sprayIsOn3 = spray3->isOn();
      sprayMode3 = spray3->getWorkMode();
      updateSprayButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
    }
    
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTSprayScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  drawScreenCaption(menuManager,"УВЛАЖНЕНИЕ ВОЗДУХА");

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HUMIDITY_SPRAY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PH_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTPHSettingsScreen::TFTPHSettingsScreen()
{
  tempBox = NULL;
  calibrationBox = NULL;
  sensorBox = NULL;
  ph4VoltageBox = NULL;
  ph7VoltageBox = NULL;
  ph10VoltageBox = NULL;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTPHSettingsScreen::~TFTPHSettingsScreen()
{
 delete screenButtons;
 delete tempBox;
 delete calibrationBox;
 delete sensorBox;
 delete ph4VoltageBox;
 delete ph7VoltageBox;
 delete ph10VoltageBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

    screenButtons->disableButton(saveButton);

  calibration = PHModule->GetCalibration();
  ph4Voltage = PHModule->GetPh4Voltage();
  ph7Voltage = PHModule->GetPh7Voltage();
  ph10Voltage = PHModule->GetPh10Voltage();
  phTemperatureSensorIndex = PHModule->GetTemperatureSensorIndex();
  phSamplesTemperature = PHModule->GetTemperature();      

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    else
      tempSensorsCount = 0;

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    else
      humiditySensorsCount = 0; 


    getSensorData(sensorDataString);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::getSensorData(String& result)
{

  result = F("^ - ");
  result += char('~' + 1);
  result += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = phTemperatureSensorIndex;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   result = F("^");

   result += tmp.Current;

   result += char('~' + 1);
   result += 'C';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decPh4VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   ph4VoltageBox = new TFTInfoBox("4 pH, милливольт:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incPh4VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decPh7VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   ph7VoltageBox = new TFTInfoBox("7 pH, милливольт:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incPh7VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decPh10VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   ph10VoltageBox = new TFTInfoBox("10 pH, милливольт:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incPh10VoltageBoxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox("Датчик:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen("Датчик:") + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   decTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   tempBox = new TFTInfoBox("Т калибровки:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;   
   incTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;
   
   decCalibrationButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   calibrationBox = new TFTInfoBox("Поправочное число:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incCalibrationButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::saveSettings()
{
  PHModule->SetCalibration(calibration);
  PHModule->SetPh4Voltage(ph4Voltage);
  PHModule->SetPh7Voltage(ph7Voltage);
  PHModule->SetPh10Voltage(ph10Voltage);
  PHModule->SetTemperatureSensorIndex(phTemperatureSensorIndex);
  PHModule->SetTemperature(phSamplesTemperature);

  PHModule->SaveSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decTempButton || buttonID == incTempButton || buttonID == decCalibrationButton
  || buttonID == incCalibrationButton || buttonID == decPh4VoltageBoxButton || buttonID == incPh4VoltageBoxButton
  || buttonID == decPh7VoltageBoxButton || buttonID == incPh7VoltageBoxButton || buttonID == decPh10VoltageBoxButton
  || buttonID == incPh10VoltageBoxButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::onTick()
{
  if(tickerButton == decTempButton)
    incTemp(-3);
  else
  if(tickerButton == incTempButton)
    incTemp(3);
  else
  if(tickerButton == decCalibrationButton)
    incCalibration(-5);
  else
  if(tickerButton == incCalibrationButton)
    incCalibration(5);
  else
  if(tickerButton == decPh4VoltageBoxButton)
    incPh4VoltageBox(-10);
  else
  if(tickerButton == incPh4VoltageBoxButton)
    incPh4VoltageBox(10);
  else
  if(tickerButton == decPh7VoltageBoxButton)
    incPh7VoltageBox(-10);
  else
  if(tickerButton == incPh7VoltageBoxButton)
    incPh7VoltageBox(10);
  else
  if(tickerButton == decPh10VoltageBoxButton)
    incPh10VoltageBox(-10);
  else
  if(tickerButton == incPh10VoltageBoxButton)
    incPh10VoltageBox(10);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::incTemp(int val)
{
  int16_t old = phSamplesTemperature;
  
  phSamplesTemperature+=val;

  if(phSamplesTemperature < 0)
    phSamplesTemperature = 0;
  
  if(phSamplesTemperature > 127)
    phSamplesTemperature = 127;

  if(phSamplesTemperature != old)
    drawValueInBox(tempBox,phSamplesTemperature);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::incCalibration(int val)
{
    int16_t old = calibration;
    
    calibration += val;
    
    if(calibration < 0)
        calibration = 0;

      
    if(calibration != old)
      drawValueInBox(calibrationBox,calibration);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::incPh4VoltageBox(int val)
{
  int16_t old = ph4Voltage;
  
  ph4Voltage+=val;

  if(ph4Voltage < 0)
    ph4Voltage = 0;
  
  if(ph4Voltage != old)
    drawValueInBox(ph4VoltageBox,ph4Voltage);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::incPh7VoltageBox(int val)
{
  int16_t old = ph7Voltage;
  
  ph7Voltage+=val;

  if(ph7Voltage < 0)
    ph7Voltage = 0;
  
  if(ph7Voltage != old)
    drawValueInBox(ph7VoltageBox,ph7Voltage);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::incPh10VoltageBox(int val)
{
  int16_t old = ph10Voltage;
  
  ph10Voltage+=val;

  if(ph10Voltage < 0)
    ph10Voltage = 0;
  
  if(ph10Voltage != old)
    drawValueInBox(ph10VoltageBox,ph10Voltage);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(sensorDataString);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }    

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("PHControl");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else if(pressed_button == decTempButton)
          {
            incTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incTempButton)
          {
            incTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decPh4VoltageBoxButton)
          {
            incPh4VoltageBox(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incPh4VoltageBoxButton)
          {
            incPh4VoltageBox(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decPh7VoltageBoxButton)
          {
            incPh7VoltageBox(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incPh7VoltageBoxButton)
          {
            incPh7VoltageBox(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decPh10VoltageBoxButton)
          {
            incPh10VoltageBox(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incPh10VoltageBoxButton)
          {
            incPh10VoltageBox(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decCalibrationButton)
          {
            incCalibration(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incCalibrationButton)
          {
            incCalibration(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            phTemperatureSensorIndex--;
            if(phTemperatureSensorIndex < 0)
              phTemperatureSensorIndex = 0;
      
            drawValueInBox(sensorBox,phTemperatureSensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            phTemperatureSensorIndex++;
            if(phTemperatureSensorIndex >= (humiditySensorsCount + tempSensorsCount))
              phTemperatureSensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;
		  
            if(phTemperatureSensorIndex < 0 || phTemperatureSensorIndex >= (humiditySensorsCount + tempSensorsCount))
              phTemperatureSensorIndex = 0;
		  
      
            drawValueInBox(sensorBox,phTemperatureSensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  tempBox->draw(menuManager);
  drawValueInBox(tempBox,phSamplesTemperature);

  ph4VoltageBox->draw(menuManager);
  drawValueInBox(ph4VoltageBox,ph4Voltage);

  ph7VoltageBox->draw(menuManager);
  drawValueInBox(ph7VoltageBox,ph7Voltage);

  ph10VoltageBox->draw(menuManager);
  drawValueInBox(ph10VoltageBox,ph10Voltage);
  
  calibrationBox->draw(menuManager);
  drawValueInBox(calibrationBox,calibration);

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,phTemperatureSensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTPHControlScreen::TFTPHControlScreen()
{
  phBox = NULL;
  histeresisBox = NULL;
  mixWorkTimeBox = NULL;
  reagentWorkTimeBox = NULL;
  tickerButton = -1;  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTPHControlScreen::~TFTPHControlScreen()
{
 delete screenButtons;
 delete phBox;
 delete histeresisBox;
 delete mixWorkTimeBox;  
 delete reagentWorkTimeBox;  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

    phValue = PHModule->GetPHValue();
    histeresis = PHModule->GetHisteresis();
    mixWorkTime = PHModule->GetMixPumpTime();
    reagentWorkTime = PHModule->GetReagentPumpTime();

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*4;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    phSettingsButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, "НАСТРОЙКИ");
    screenButtons->setButtonBackColor(phSettingsButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(phSettingsButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decPHButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   phBox = new TFTInfoBox("Держим pH:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incPHButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox("Гистерезис:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decMixWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

      
   mixWorkTimeBox = new TFTInfoBox("Перемешивание, с:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incMixWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


  leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decReagentWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   reagentWorkTimeBox = new TFTInfoBox("Подача реагента, с:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incReagentWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::saveSettings()
{
  PHModule->SetPHValue(phValue);
  PHModule->SetHisteresis(histeresis);
  PHModule->SetMixPumpTime(mixWorkTime);
  PHModule->SetReagentPumpTime(reagentWorkTime);

  PHModule->SaveSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decPHButton || buttonID == incPHButton || buttonID == decHisteresisButton
  || buttonID == incHisteresisButton || buttonID == incMixWorkTimeButton || buttonID == decMixWorkTimeButton
  || buttonID == incReagentWorkTimeButton || buttonID == decReagentWorkTimeButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::onTick()
{
  if(tickerButton == decPHButton)
    incPH(-50);
  else
  if(tickerButton == incPHButton)
    incPH(50);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-10);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(10);
  else
  if(tickerButton == incMixWorkTimeButton)
    incMixWorkTime(5);
  else
  if(tickerButton == decMixWorkTimeButton)
    incMixWorkTime(-5);
  else
  if(tickerButton == incReagentWorkTimeButton)
    incReagentWorkTime(5);
  else
  if(tickerButton == decReagentWorkTimeButton)
    incReagentWorkTime(-5);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::incPH(int val)
{
  int16_t old = phValue;
  
  phValue+=val;

  if(phValue < 0)
    phValue = 0;
  
  if(phValue != old)
    drawValueInBox(phBox,formatPH());  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::incHisteresis(int val)
{
    int16_t old = histeresis;
    
    histeresis += val;
    
    if(histeresis < 0)
        histeresis = 0;

    if(histeresis > 500)
        histeresis = 500;
      
    if(histeresis != old)
      drawValueInBox(histeresisBox,formatHisteresis());  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::incMixWorkTime(int val)
{
    int16_t old = mixWorkTime;
    
    mixWorkTime += val;
    
    if(mixWorkTime < 0)
        mixWorkTime = 0;

      
    if(mixWorkTime != old)
      drawValueInBox(mixWorkTimeBox,mixWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::incReagentWorkTime(int val)
{
    int16_t old = reagentWorkTime;
    
    reagentWorkTime += val;
    
    if(reagentWorkTime < 0)
        reagentWorkTime = 0;
    
    if(reagentWorkTime != old)
      drawValueInBox(reagentWorkTimeBox,reagentWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("DRIVE");
            return;
          }
          else
          if(pressed_button == phSettingsButton)
          {
            menuManager->switchToScreen("PHSettings");
            return;            
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else if(pressed_button == decPHButton)
          {
            incPH(-10);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incPHButton)
          {
            incPH(10);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decMixWorkTimeButton)
          {
            incMixWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incMixWorkTimeButton)
          {
            incMixWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decReagentWorkTimeButton)
          {
            incReagentWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incReagentWorkTimeButton)
          {
            incReagentWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTPHControlScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  drawScreenCaption(menuManager,CONTROL_PH_SCREEN_CAPTION);  

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  phBox->draw(menuManager);
  drawValueInBox(phBox,formatPH());

  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  mixWorkTimeBox->draw(menuManager);
  drawValueInBox(mixWorkTimeBox,mixWorkTime);

  reagentWorkTimeBox->draw(menuManager);
  drawValueInBox(reagentWorkTimeBox,reagentWorkTime);


}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTPHControlScreen::formatHisteresis()
{
  // гистерезис у нас хранится в сотых долях, т.е. 100 == 1.0 !!!
  
  int16_t val = histeresis/100;
  int16_t fract = histeresis%100;

  String result;
  result += val;
  result += '.';

  if(fract < 10)
    result += '0';
    
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTPHControlScreen::formatPH()
{
  // pH у нас хранится в сотых долях, т.е. 700 == 7.0 !!!
  
  int16_t val = phValue/100;
  int16_t fract = phValue%100;

  String result;
  result += val;
  result += '.';

  if(fract < 10)
    result += '0';
  
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PH_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_THERMOSTAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTThermostatSettingsScreen::TFTThermostatSettingsScreen(uint8_t _channel)
{
  tempBox = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;

  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTThermostatSettingsScreen::~TFTThermostatSettingsScreen()
{
 delete screenButtons;
 delete tempBox;
 delete histeresisBox;
 delete sensorBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    thermostatSettings = settings->GetThermostatSettings(channel);

    if(thermostatSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    else
      tempSensorsCount = 0;

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    else
      humiditySensorsCount = 0; 


    getSensorData(sensorDataString);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::getSensorData(String& result)
{

  result = F("^ - ");
  result += char('~' + 1);
  result += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = thermostatSettings.sensorIndex;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   result = F("^");

   result += tmp.Current;

   result += char('~' + 1);
   result += 'C';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   tempBox = new TFTInfoBox(THERMOSTAT_TEMP_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox(THERMOSTAT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   sensorBox = new TFTInfoBox(THERMOSTAT_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(THERMOSTAT_SENSOR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetThermostatSettings(channel,thermostatSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadThermostatSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decTempButton || buttonID == incTempButton || buttonID == decHisteresisButton
  || buttonID == incHisteresisButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::onTick()
{
  if(tickerButton == decTempButton)
    incTemp(-3);
  else
  if(tickerButton == incTempButton)
    incTemp(3);
  else
  if(tickerButton == decHisteresisButton)
    incHisteresis(-5);
  else
  if(tickerButton == incHisteresisButton)
    incHisteresis(5);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::incTemp(int val)
{
  int16_t old = thermostatSettings.temp;
  
  thermostatSettings.temp+=val;

  if(thermostatSettings.temp < 0)
    thermostatSettings.temp = 0;
  
  if(thermostatSettings.temp > 127)
    thermostatSettings.temp = 127;

  if(thermostatSettings.temp != old)
    drawValueInBox(tempBox,thermostatSettings.temp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::incHisteresis(int val)
{
    int16_t old = thermostatSettings.histeresis;
    
    thermostatSettings.histeresis += val;
    
    if(thermostatSettings.histeresis < 0)
        thermostatSettings.histeresis = 0;

    if(thermostatSettings.histeresis > 100)
        thermostatSettings.histeresis = 100;
      
    if(thermostatSettings.histeresis != old)
      drawValueInBox(histeresisBox,formatHisteresis());  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(sensorDataString);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }  

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("THERMOSTAT");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(thermostatSettings.active)
                thermostatSettings.active = false;
              else
                thermostatSettings.active = true;
                
              if(thermostatSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decTempButton)
          {
            incTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incTempButton)
          {
            incTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decHisteresisButton)
          {
            incHisteresis(-5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            incHisteresis(5);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            thermostatSettings.sensorIndex--;
            if(thermostatSettings.sensorIndex < 0)
              thermostatSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,thermostatSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            thermostatSettings.sensorIndex++;
            if(thermostatSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              thermostatSettings.sensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;
		  
			if(thermostatSettings.sensorIndex < 0 || thermostatSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              thermostatSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,thermostatSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  tempBox->draw(menuManager);
  drawValueInBox(tempBox,thermostatSettings.temp);

  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,thermostatSettings.sensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTThermostatSettingsScreen::formatHisteresis()
{
  int16_t val = thermostatSettings.histeresis/10;
  int16_t fract = thermostatSettings.histeresis%10;

  String result;
  result += val;
  result += '.';
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTThermostatScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTThermostatScreen::TFTThermostatScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTThermostatScreen::~TFTThermostatScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  updateThermostatButtons(0, channel1OnOffButton, channel1AutoManualButton);
  updateThermostatButtons(1, channel2OnOffButton, channel2AutoManualButton);
  updateThermostatButtons(2, channel3OnOffButton, channel3AutoManualButton);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*3;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

   channel1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_1_CAPTION);

   int channelsControlsButtonsLeft = leftPos + leftRowButtonsWidth + spacing;
   int channelsControleButtonsWidth = 185;
   int channelsModeButtonsLeft = channelsControlsButtonsLeft + channelsControleButtonsWidth + spacing;   

   channel1OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel1AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);
    
 
   // вторая строка
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

   channel2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_2_CAPTION);

   channel2OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel2AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
    
   // третья строка
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   channel3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_3_CAPTION);
  
   channel3OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel3AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatScreen::updateThermostatButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw)
{
  Thermostat* thermostat = LogicManageModule->getThermostat(channel);
  if(thermostat->isOn())
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_ON_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВЫКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВЫКЛ") !=0);
  }
  else
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВКЛ") !=0);    
  }

  ThermostatWorkMode wm = thermostat->getWorkMode();
  if(wm == twmAuto)
  {
    screenButtons->setButtonBackColor(btnMode,MODE_ON_COLOR);
    screenButtons->relabelButton(btnMode,"АВТО",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"АВТО") !=0);    
  }
  else
  {
    screenButtons->setButtonBackColor(btnMode,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnMode,"РУЧНОЙ",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"РУЧНОЙ") !=0);        
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {

  Thermostat* thermostat1 = LogicManageModule->getThermostat(0);
  Thermostat* thermostat2 = LogicManageModule->getThermostat(1);
  Thermostat* thermostat3 = LogicManageModule->getThermostat(2);
  
  static bool thermostatIsOn1 = thermostat1->isOn();
  static bool thermostatIsOn2 = thermostat2->isOn();
  static bool thermostatIsOn3 = thermostat3->isOn();

  static ThermostatWorkMode thermostatMode1 = thermostat1->getWorkMode();
  static ThermostatWorkMode thermostatMode2 = thermostat2->getWorkMode();
  static ThermostatWorkMode thermostatMode3 = thermostat3->getWorkMode();
    
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("DRIVE");
        return;
      }

      else
      if(pressed_button == channel1Button)
      {
        menuManager->switchToScreen("THERMOSTAT1");
        return;        
      }
      else
      if(pressed_button == channel2Button)
      {
        menuManager->switchToScreen("THERMOSTAT2");
        return;        
      }
      else
      if(pressed_button == channel3Button)
      {
        menuManager->switchToScreen("THERMOSTAT3");
        return;        
      }
      else
      if(pressed_button == channel1AutoManualButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(0);
        if(thermostat->getWorkMode() == twmAuto)
          thermostat->switchToMode(twmManual);
        else
          thermostat->switchToMode(twmAuto);

        thermostatMode1 = thermostat->getWorkMode();

        updateThermostatButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel2AutoManualButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(1);
        if(thermostat->getWorkMode() == twmAuto)
          thermostat->switchToMode(twmManual);
        else
          thermostat->switchToMode(twmAuto);

        thermostatMode2 = thermostat->getWorkMode();

        updateThermostatButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel3AutoManualButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(2);
        if(thermostat->getWorkMode() == twmAuto)
          thermostat->switchToMode(twmManual);
        else
          thermostat->switchToMode(twmAuto);

       thermostatMode3 = thermostat->getWorkMode();

        updateThermostatButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel1OnOffButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(0);
        bool onFlag = !thermostat->isOn();
        thermostat->switchToMode(twmManual);
        thermostat->turn(onFlag);

        thermostatMode1 = thermostat->getWorkMode();
        thermostatIsOn1 = thermostat->isOn();
        
        updateThermostatButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;        
      }
      else
      if(pressed_button == channel2OnOffButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(1);
        bool onFlag = !thermostat->isOn();
        thermostat->switchToMode(twmManual);
        thermostat->turn(onFlag);

        thermostatMode2 = thermostat->getWorkMode();
        thermostatIsOn2 = thermostat->isOn();
        
        updateThermostatButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;      
      }
      else
      if(pressed_button == channel3OnOffButton)
      {
        Thermostat* thermostat = LogicManageModule->getThermostat(2);
        bool onFlag = !thermostat->isOn();
        thermostat->switchToMode(twmManual);
        thermostat->turn(onFlag);

        thermostatMode3 = thermostat->getWorkMode();
        thermostatIsOn3 = thermostat->isOn();
        
        updateThermostatButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;       
      }
      
    } // if(pressed_button != -1)

    if(thermostatIsOn1 != thermostat1->isOn() || thermostatMode1 != thermostat1->getWorkMode())
    {
      thermostatIsOn1 = thermostat1->isOn();
      thermostatMode1 = thermostat1->getWorkMode();
      updateThermostatButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
    }

    if(thermostatIsOn2 != thermostat2->isOn() || thermostatMode2 != thermostat2->getWorkMode())
    {
      thermostatIsOn2 = thermostat2->isOn();
      thermostatMode2 = thermostat2->getWorkMode();
      updateThermostatButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
    }

    if(thermostatIsOn3 != thermostat3->isOn() || thermostatMode3 != thermostat3->getWorkMode())
    {
      thermostatIsOn3 = thermostat3->isOn();
      thermostatMode3 = thermostat3->getWorkMode();
      updateThermostatButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
    }
    
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTThermostatScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  drawScreenCaption(menuManager,THERMOSTAT_SCREEN_CAPTION);

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_THERMOSTAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CYCLE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTCycleVentSettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCycleVentSettingsScreen::TFTCycleVentSettingsScreen(uint8_t _channel)
{
  startTimeBox = NULL;
  endTimeBox = NULL;
  workTimeBox = NULL;
  idleTimeBox = NULL;
  channel = _channel;

  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCycleVentSettingsScreen::~TFTCycleVentSettingsScreen()
{
 delete screenButtons;
 delete startTimeBox;
 delete endTimeBox;
 delete workTimeBox;
 delete idleTimeBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    ventSettings = settings->GetCycleVentSettings(channel);

    if(ventSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    // теперь смотрим, какие дни недели установлены для канала
    for(uint8_t i=0;i<7;i++)
    {
      uint8_t mask = (1 << i);
      int btnID = weekdaysButtons[i];
        
      if((ventSettings.weekdays & mask))
      {
        screenButtons->setButtonBackColor(btnID,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      } // if
      else
      {
        screenButtons->setButtonBackColor(btnID,MODE_OFF_COLOR);
        screenButtons->setButtonFontColor(btnID,CHANNELS_BUTTONS_TEXT_COLOR);        
      }
        
    } // for    

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
  //  int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);


    int weekDaysButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*6)/7;
    int weekDayLeft = initialLeftPos;

    for(int i=0;i<7;i++)
    {
      int addedID = screenButtons->addButton( weekDayLeft ,  topPos, weekDaysButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, WEEKDAYS[i]);
      weekDayLeft += weekDaysButtonWidth + INFO_BOX_V_SPACING;

      weekdaysButtons.push_back(addedID);
    }  
    

    topPos = secondRowTopPos; // со второй строки
    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки    
    decStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   startTimeBox = new TFTInfoBox(CYCLE_VENT_STIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incStartTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   endTimeBox = new TFTInfoBox(CYCLE_VENT_ETIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incEndTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   workTimeBox = new TFTInfoBox(CYCLE_VENT_WTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

  
   decIdleTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   idleTimeBox = new TFTInfoBox(CYCLE_VENT_ITIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incIdleTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetCycleVentSettings(channel,ventSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadCycleVentSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decStartTimeButton || buttonID == incStartTimeButton || buttonID == decEndTimeButton
  || buttonID == incEndTimeButton || buttonID == decWorkTimeButton || buttonID == incWorkTimeButton
  || buttonID == decIdleTimeButton || buttonID == incIdleTimeButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::onTick()
{
  if(tickerButton == decStartTimeButton)
    incStartTime(-7);
  else
  if(tickerButton == incStartTimeButton)
    incStartTime(7);
  else
  if(tickerButton == decEndTimeButton)
    incEndTime(-7);
  else
  if(tickerButton == incEndTimeButton)
    incEndTime(7);
  else
  if(tickerButton == decWorkTimeButton)
    incWorkTime(-7);
  else
  if(tickerButton == incWorkTimeButton)
    incWorkTime(7);
  else
  if(tickerButton == decIdleTimeButton)
    incIdleTime(-7);
  else
  if(tickerButton == incIdleTimeButton)
    incIdleTime(7);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::incStartTime(int val)
{
  uint32_t old = ventSettings.startTime;
  
  ventSettings.startTime+=val;

  if(ventSettings.startTime < 0)
    ventSettings.startTime = 0;
  
  if(ventSettings.startTime > 1440)
    ventSettings.startTime = 1440;

  if(ventSettings.startTime != old)
    drawTimeInBox(startTimeBox,ventSettings.startTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::incEndTime(int val)
{
  uint32_t old = ventSettings.endTime;
  
  ventSettings.endTime+=val;

  if(ventSettings.endTime < 0)
    ventSettings.endTime = 0;
  
  if(ventSettings.endTime > 1440)
    ventSettings.endTime = 1440;

  if(ventSettings.endTime != old)
    drawTimeInBox(endTimeBox,ventSettings.endTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::incWorkTime(int val)
{
  uint32_t old = ventSettings.workTime;
  
  ventSettings.workTime+=val;

  if(ventSettings.workTime < 0)
    ventSettings.workTime = 0;
  
  if(ventSettings.workTime > 1440)
    ventSettings.workTime = 1440;

  if(ventSettings.workTime != old)
    drawValueInBox(workTimeBox,String(ventSettings.workTime), SevenSegNumFontPlus);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::incIdleTime(int val)
{
  uint32_t old = ventSettings.idleTime;
  
  ventSettings.idleTime+=val;

  if(ventSettings.idleTime < 0)
    ventSettings.idleTime = 0;
  
  if(ventSettings.idleTime > 1440)
    ventSettings.idleTime = 1440;

  if(ventSettings.idleTime != old)
    drawValueInBox(idleTimeBox,String(ventSettings.idleTime), SevenSegNumFontPlus);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("CYCLE_VENT");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(ventSettings.active)
                ventSettings.active = false;
              else
                ventSettings.active = true;
                
              if(ventSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decStartTimeButton)
          {
            incStartTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incStartTimeButton)
          {
            incStartTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decEndTimeButton)
          {
            incEndTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incEndTimeButton)
          {
            incEndTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decWorkTimeButton)
          {
            incWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incWorkTimeButton)
          {
            incWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decIdleTimeButton)
          {
            incIdleTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incIdleTimeButton)
          {
            incIdleTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else
          {
            // нажата одна из кнопок дней недели
            for(int i=0;i<7;i++)
            {
              if(pressed_button == weekdaysButtons[i])
              {
                word backColor = screenButtons->getButtonBackColor(pressed_button);
                if(backColor == MODE_ON_COLOR)
                {
                  backColor = MODE_OFF_COLOR;
                  ventSettings.weekdays &= ~(1 << i);
                }
                else
                {
                  backColor = MODE_ON_COLOR;
                  ventSettings.weekdays |= (1 << i);
                }
      
                screenButtons->setButtonBackColor(pressed_button,backColor);
                screenButtons->drawButton(pressed_button);
                
                screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
                blinkSaveSettingsButton(true);
                
                break;
              } // if
            } // for
          } // else
          
          

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  startTimeBox->draw(menuManager);
  drawTimeInBox(startTimeBox,ventSettings.startTime);

  endTimeBox->draw(menuManager);
  drawTimeInBox(endTimeBox,ventSettings.endTime);

  workTimeBox->draw(menuManager);
  drawValueInBox(workTimeBox,String(ventSettings.workTime), SevenSegNumFontPlus);

  idleTimeBox->draw(menuManager);
  drawValueInBox(idleTimeBox,String(ventSettings.idleTime), SevenSegNumFontPlus);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTCycleVentScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCycleVentScreen::TFTCycleVentScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTCycleVentScreen::~TFTCycleVentScreen()
{
 delete screenButtons;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentScreen::updateVentButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw)
{
  CycleVent* vent = LogicManageModule->getCycleVent(channel);
  if(vent->isOn())
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_ON_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВЫКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВЫКЛ") !=0);
  }
  else
  {
    screenButtons->setButtonBackColor(btnOnOff,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnOnOff,"ВКЛ",forceRedraw && strcmp(screenButtons->getLabel(btnOnOff),"ВКЛ") !=0);    
  }

  CycleVentWorkMode wm = vent->getWorkMode();
  if(wm == cvwmAuto)
  {
    screenButtons->setButtonBackColor(btnMode,MODE_ON_COLOR);
    screenButtons->relabelButton(btnMode,"АВТО",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"АВТО") !=0);    
  }
  else
  {
    screenButtons->setButtonBackColor(btnMode,MODE_OFF_COLOR);
    screenButtons->relabelButton(btnMode,"РУЧНОЙ",forceRedraw && strcmp(screenButtons->getLabel(btnMode),"РУЧНОЙ") !=0);        
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton);
  updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton);
  updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*3;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

   channel1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_1_CAPTION);

   int channelsControlsButtonsLeft = leftPos + leftRowButtonsWidth + spacing;
   int channelsControleButtonsWidth = 185;
   int channelsModeButtonsLeft = channelsControlsButtonsLeft + channelsControleButtonsWidth + spacing;
   
   channel1OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel1AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel1AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);
 
   // вторая строка
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

   channel2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_2_CAPTION);

   channel2OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel2AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel2AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);
   
   // третья строка
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   channel3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, CHANNEL_N_3_CAPTION);

   channel3OnOffButton = screenButtons->addButton( channelsControlsButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3OnOffButton,CHANNELS_BUTTONS_TEXT_COLOR);

   channel3AutoManualButton = screenButtons->addButton( channelsModeButtonsLeft ,  topPos, channelsControleButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, "");
   screenButtons->setButtonFontColor(channel3AutoManualButton, CHANNELS_BUTTONS_TEXT_COLOR);

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {

    CycleVent* vent1 = LogicManageModule->getCycleVent(0);
    CycleVent* vent2 = LogicManageModule->getCycleVent(1);
    CycleVent* vent3 = LogicManageModule->getCycleVent(2);
    
    static bool ventIsOn1 = vent1->isOn();
    static bool ventIsOn2 = vent2->isOn();
    static bool ventIsOn3 = vent3->isOn();
  
    static CycleVentWorkMode ventMode1 = vent1->getWorkMode();
    static CycleVentWorkMode ventMode2 = vent2->getWorkMode();
    static CycleVentWorkMode ventMode3 = vent3->getWorkMode();
  
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("DRIVE");
        return;
      }

      else
      if(pressed_button == channel1Button)
      {
        menuManager->switchToScreen("CYCLE_VENT1");
        return;        
      }
      else
      if(pressed_button == channel2Button)
      {
        menuManager->switchToScreen("CYCLE_VENT2");
        return;        
      }
      else
      if(pressed_button == channel3Button)
      {
        menuManager->switchToScreen("CYCLE_VENT3");
        return;        
      }
      else
      if(pressed_button == channel1AutoManualButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(0);
        if(vent->getWorkMode() == cvwmAuto)
          vent->switchToMode(cvwmManual);
        else
          vent->switchToMode(cvwmAuto);

        ventMode1 = vent1->getWorkMode();

        updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel2AutoManualButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(1);
        if(vent->getWorkMode() == cvwmAuto)
          vent->switchToMode(cvwmManual);
        else
          vent->switchToMode(cvwmAuto);

        ventMode2 = vent2->getWorkMode();

        updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel3AutoManualButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(2);
        if(vent->getWorkMode() == cvwmAuto)
          vent->switchToMode(cvwmManual);
        else
          vent->switchToMode(cvwmAuto);

        ventMode3 = vent3->getWorkMode();

        updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;
      }
      else
      if(pressed_button == channel1OnOffButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(0);
        bool onFlag = !vent->isOn();
        vent->switchToMode(cvwmManual);
        vent->turn(onFlag);

        ventIsOn1 = vent1->isOn();
        ventMode1 = vent1->getWorkMode();
      
        updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
        return;       
      }
      else
      if(pressed_button == channel2OnOffButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(1);
        bool onFlag = !vent->isOn();
        vent->switchToMode(cvwmManual);
        vent->turn(onFlag);

        ventIsOn2 = vent2->isOn();
        ventMode2 = vent2->getWorkMode();
      
        updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
        return;        
      }
      else
      if(pressed_button == channel3OnOffButton)
      {
        CycleVent* vent = LogicManageModule->getCycleVent(2);
        bool onFlag = !vent->isOn();
        vent->switchToMode(cvwmManual);
        vent->turn(onFlag);

        ventIsOn3 = vent3->isOn();
        ventMode3 = vent3->getWorkMode();
      
        updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
        return;        
      }
      
    } // if(pressed_button != -1)

    if(ventIsOn1 != vent1->isOn() || ventMode1 != vent1->getWorkMode())
    {
      ventIsOn1 = vent1->isOn();
      ventMode1 = vent1->getWorkMode();
      updateVentButtons(0, channel1OnOffButton, channel1AutoManualButton,true);
    }

    if(ventIsOn2 != vent2->isOn() || ventMode2 != vent2->getWorkMode())
    {
      ventIsOn2 = vent2->isOn();
      ventMode2 = vent2->getWorkMode();
      updateVentButtons(1, channel2OnOffButton, channel2AutoManualButton,true);
    }

    if(ventIsOn3 != vent3->isOn() || ventMode3 != vent3->getWorkMode())
    {
      ventIsOn3 = vent3->isOn();
      ventMode3 = vent3->getWorkMode();
      updateVentButtons(2, channel3OnOffButton, channel3AutoManualButton,true);
    }    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTCycleVentScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  drawScreenCaption(menuManager,CYCLE_VENT_SCREEN_CAPTION);

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CYCLE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HEAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTHeatDriveScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatDriveScreen::TFTHeatDriveScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatDriveScreen::~TFTHeatDriveScreen()
{
 delete screenButtons;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatDriveScreen::onActivate(TFTMenu* menuManager)
{

  if(!menuManager->getDC())
  {
    return;
  }
  
  LogicManageModule->GetHeatStatus(0,heat1Active,heat1On);
  LogicManageModule->GetHeatStatus(1,heat2Active,heat2On);
  LogicManageModule->GetHeatStatus(2,heat3Active,heat3On);

  heatMode1 = LogicManageModule->GetHeatMode(0) == hwmAutomatic;
  heatMode2 = LogicManageModule->GetHeatMode(1) == hwmAutomatic;
  heatMode3 = LogicManageModule->GetHeatMode(2) == hwmAutomatic;

  if(heat1On)
  {
    screenButtons->setButtonBackColor(heat1OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(heat1OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat1OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat1OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(heat1OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat1OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat1OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(heat1OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }


  if(heat2On)
  {
    screenButtons->setButtonBackColor(heat2OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(heat2OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat2OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat2OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(heat2OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat2OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat2OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(heat2OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }


  if(heat3On)
  {
    screenButtons->setButtonBackColor(heat3OnButton,MODE_ON_COLOR);
    screenButtons->setButtonFontColor(heat3OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat3OffButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat3OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
  }
  else
  {
    screenButtons->setButtonBackColor(heat3OnButton,CHANNELS_BUTTONS_BG_COLOR);
    screenButtons->setButtonFontColor(heat3OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
    
    screenButtons->setButtonBackColor(heat3OffButton,MODE_ON_COLOR);    
    screenButtons->setButtonFontColor(heat3OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
  }
  
  if(heatMode1)
  {
    screenButtons->setButtonBackColor(heat1ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(heat1ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(heat1ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(heat1ModeButton,MANUAL_MODE_LABEL);    
  }

  if(heatMode2)
  {
    screenButtons->setButtonBackColor(heat2ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(heat2ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(heat2ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(heat2ModeButton,MANUAL_MODE_LABEL);    
  }

  if(heatMode3)
  {
    screenButtons->setButtonBackColor(heat3ModeButton,MODE_ON_COLOR);
    screenButtons->relabelButton(heat3ModeButton,AUTO_MODE_LABEL);
  }
  else
  {
    screenButtons->setButtonBackColor(heat3ModeButton,MODE_OFF_COLOR);
    screenButtons->relabelButton(heat3ModeButton,MANUAL_MODE_LABEL);    
  }

  if(!heat1Active)
  {
    screenButtons->disableButton(heat1OnButton);
    screenButtons->disableButton(heat1OffButton);
    screenButtons->disableButton(heat1ModeButton);
  }
  else
  {
    screenButtons->enableButton(heat1OnButton);
    screenButtons->enableButton(heat1OffButton);
    screenButtons->enableButton(heat1ModeButton);    
  }

  if(!heat2Active)
  {
    screenButtons->disableButton(heat2OnButton);
    screenButtons->disableButton(heat2OffButton);
    screenButtons->disableButton(heat2ModeButton);
  }
  else
  {
    screenButtons->enableButton(heat2OnButton);
    screenButtons->enableButton(heat2OffButton);
    screenButtons->enableButton(heat2ModeButton);    
  }

  if(!heat3Active)
  {
    screenButtons->disableButton(heat3OnButton);
    screenButtons->disableButton(heat3OffButton);
    screenButtons->disableButton(heat3ModeButton);
  }
  else
  {
    screenButtons->enableButton(heat3OnButton);
    screenButtons->enableButton(heat3OffButton);
    screenButtons->enableButton(heat3ModeButton);    
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatDriveScreen::setup(TFTMenu* menuManager)
{
    
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter(),10);
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);

 
    //UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    //int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/3;
    int channelsButtonLeft = initialLeftPos;
    const int v_spacing = 50;

    heat1OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat1OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat1ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(heat1ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    topPos += ALL_CHANNELS_BUTTON_HEIGHT + v_spacing;
    channelsButtonLeft = initialLeftPos;

    // вторая строка
    heat2OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat2OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat2ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(heat2ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
    topPos += ALL_CHANNELS_BUTTON_HEIGHT + v_spacing;
    channelsButtonLeft = initialLeftPos;

    // третья строка
    heat3OnButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_ON);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat3OffButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, HEAT_DRIVE_CHANNEL_OFF);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING;

    heat3ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(heat3ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);
    
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatDriveScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
            menuManager->resetIdleTimer();

   
      if(pressed_button == backButton)
      {
        drawCalled = false;
        menuManager->switchToScreen("DRIVE");
        return;
      }
      else
      if(pressed_button == heat1ModeButton)
      {
          bool cmd = !heatMode1;
          LogicManageModule->TurnHeatMode(0, cmd ? hwmAutomatic : hwmManual);
          return;
      }
      else
      if(pressed_button == heat2ModeButton)
      {
          bool cmd = !heatMode2;
          LogicManageModule->TurnHeatMode(1, cmd ? hwmAutomatic : hwmManual);
          return;
      }
      else
      if(pressed_button == heat3ModeButton)
      {
          bool cmd = !heatMode3;
          LogicManageModule->TurnHeatMode(2, cmd ? hwmAutomatic : hwmManual);
          return;
      }
      else
      if(pressed_button == heat1OnButton)
      {
        LogicManageModule->TurnHeat(0,true);
        screenButtons->setButtonBackColor(heat1OnButton,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(heat1OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat1OffButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat1OffButton,CHANNEL_BUTTONS_TEXT_COLOR);

        screenButtons->drawButton(heat1OnButton);
        screenButtons->drawButton(heat1OffButton);        
        return;
      }
      else
      if(pressed_button == heat1OffButton)
      {
        LogicManageModule->TurnHeat(0,false);
        screenButtons->setButtonBackColor(heat1OnButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat1OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat1OffButton,MODE_ON_COLOR);    
        screenButtons->setButtonFontColor(heat1OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
        
        screenButtons->drawButton(heat1OnButton);
        screenButtons->drawButton(heat1OffButton);

        return;
      }
      else
      if(pressed_button == heat2OnButton)
      {
        LogicManageModule->TurnHeat(1,true);
        screenButtons->setButtonBackColor(heat2OnButton,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(heat2OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat2OffButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat2OffButton,CHANNEL_BUTTONS_TEXT_COLOR);

        screenButtons->drawButton(heat2OnButton);
        screenButtons->drawButton(heat2OffButton);
        
        return;
      }
      else
      if(pressed_button == heat2OffButton)
      {
        LogicManageModule->TurnHeat(1,false);
        screenButtons->setButtonBackColor(heat2OnButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat2OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat2OffButton,MODE_ON_COLOR);    
        screenButtons->setButtonFontColor(heat2OffButton,CHANNELS_BUTTONS_TEXT_COLOR);

        screenButtons->drawButton(heat2OnButton);
        screenButtons->drawButton(heat2OffButton);
        
        return;
      }
      else
      if(pressed_button == heat3OnButton)
      {
        LogicManageModule->TurnHeat(2,true);
        screenButtons->setButtonBackColor(heat3OnButton,MODE_ON_COLOR);
        screenButtons->setButtonFontColor(heat3OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat3OffButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat3OffButton,CHANNEL_BUTTONS_TEXT_COLOR);

        screenButtons->drawButton(heat3OnButton);
        screenButtons->drawButton(heat3OffButton);
        
        return;
      }
      else
      if(pressed_button == heat3OffButton)
      {
        LogicManageModule->TurnHeat(2,false);
        screenButtons->setButtonBackColor(heat3OnButton,CHANNELS_BUTTONS_BG_COLOR);
        screenButtons->setButtonFontColor(heat3OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
        
        screenButtons->setButtonBackColor(heat3OffButton,MODE_ON_COLOR);    
        screenButtons->setButtonFontColor(heat3OffButton,CHANNELS_BUTTONS_TEXT_COLOR);

        screenButtons->drawButton(heat3OnButton);
        screenButtons->drawButton(heat3OffButton);
        
        return;
      }
      
    }

    static uint32_t updTimer = millis();
    if(millis() - updTimer > 250)
    {

        bool thisHeat1Active,thisHeat1On;
        bool thisHeat2Active,thisHeat2On;
        bool thisHeat3Active,thisHeat3On;

        bool thisHeatMode1;
        bool thisHeatMode2;
        bool thisHeatMode3;
  
        LogicManageModule->GetHeatStatus(0,thisHeat1Active,thisHeat1On);
        LogicManageModule->GetHeatStatus(1,thisHeat2Active,thisHeat2On);
        LogicManageModule->GetHeatStatus(2,thisHeat3Active,thisHeat3On);
      
        thisHeatMode1 = LogicManageModule->GetHeatMode(0) == hwmAutomatic;
        thisHeatMode2 = LogicManageModule->GetHeatMode(1) == hwmAutomatic;
        thisHeatMode3 = LogicManageModule->GetHeatMode(2) == hwmAutomatic;

        bool wantRedrawHeatStatus1 = false;
        bool wantRedrawHeatStatus2 = false;
        bool wantRedrawHeatStatus3 = false;

        bool wantRedrawHeatMode1 = false;
        bool wantRedrawHeatMode2 = false;
        bool wantRedrawHeatMode3 = false;

        if(thisHeat1On != heat1On)
        {
          heat1On = thisHeat1On;
          if(heat1On)
          {
            screenButtons->setButtonBackColor(heat1OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(heat1OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat1OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat1OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(heat1OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat1OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat1OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(heat1OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawHeatStatus1 = true;
          }
        }
      
        if(thisHeat2On != heat2On)
        {
          heat2On = thisHeat2On;
          if(heat2On)
          {
            screenButtons->setButtonBackColor(heat2OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(heat2OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat2OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat2OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(heat2OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat2OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat2OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(heat2OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawHeatStatus2 = true;
          }
        }
      
        if(thisHeat3On != heat3On)
        {
          heat3On = thisHeat3On;
          if(heat3On)
          {
            screenButtons->setButtonBackColor(heat3OnButton,MODE_ON_COLOR);
            screenButtons->setButtonFontColor(heat3OnButton,CHANNELS_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat3OffButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat3OffButton,CHANNEL_BUTTONS_TEXT_COLOR);
          }
          else
          {
            screenButtons->setButtonBackColor(heat3OnButton,CHANNELS_BUTTONS_BG_COLOR);
            screenButtons->setButtonFontColor(heat3OnButton,CHANNEL_BUTTONS_TEXT_COLOR);
            
            screenButtons->setButtonBackColor(heat3OffButton,MODE_ON_COLOR);    
            screenButtons->setButtonFontColor(heat3OffButton,CHANNELS_BUTTONS_TEXT_COLOR);
          }

          if(drawCalled)
          {
            wantRedrawHeatStatus3 = true;
          }
        }

         if(wantRedrawHeatStatus1)
         {
          screenButtons->drawButton(heat1OnButton);
          screenButtons->drawButton(heat1OffButton);
         }
         
         if(wantRedrawHeatStatus2)
         {
          screenButtons->drawButton(heat2OnButton);
          screenButtons->drawButton(heat2OffButton);
         }
         
         if(wantRedrawHeatStatus3)
         {
          screenButtons->drawButton(heat3OnButton);
          screenButtons->drawButton(heat3OffButton);
         }
               
        if(thisHeatMode1 != heatMode1)
        {
          heatMode1 = thisHeatMode1;
          if(heatMode1)
          {
            screenButtons->setButtonBackColor(heat1ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(heat1ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(heat1ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(heat1ModeButton,MANUAL_MODE_LABEL);    
          }

          if(drawCalled)
          {
            wantRedrawHeatMode1 = true;
          }
        }

        if(thisHeatMode2 != heatMode2)
        {
          heatMode2 = thisHeatMode2;
          if(heatMode2)
          {
            screenButtons->setButtonBackColor(heat2ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(heat2ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(heat2ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(heat2ModeButton,MANUAL_MODE_LABEL);    
          }
          if(drawCalled)
          {
            wantRedrawHeatMode2 = true;
          }
        }

        if(thisHeatMode3 != heatMode3)
        {
          heatMode3 = thisHeatMode3;
          
          if(heatMode3)
          {
            screenButtons->setButtonBackColor(heat3ModeButton,MODE_ON_COLOR);
            screenButtons->relabelButton(heat3ModeButton,AUTO_MODE_LABEL);
          }
          else
          {
            screenButtons->setButtonBackColor(heat3ModeButton,MODE_OFF_COLOR);
            screenButtons->relabelButton(heat3ModeButton,MANUAL_MODE_LABEL);    
          }
          if(drawCalled)
          {
            wantRedrawHeatMode3 = true;
          }
        }

        if(wantRedrawHeatMode1)
        {
          screenButtons->drawButton(heat1ModeButton);
        }

        if(wantRedrawHeatMode2)
        {
          screenButtons->drawButton(heat2ModeButton);
        }

        if(wantRedrawHeatMode3)
        {
          screenButtons->drawButton(heat3ModeButton);
        }

      updTimer = millis();
    }
    
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatDriveScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  // рисуем заголовки
  const int left = 30;
  int top = 15;
  const int v_spacing = 120;

  UTFT* dc = menuManager->getDC();
  UTFTRus* rusPrinter = menuManager->getRusPrinter();
  dc->setFont(BigRusFont);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  dc->setBackColor(TFT_BACK_COLOR);

  if(heat1Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);
  
  rusPrinter->print(HEAT_BUTTON_CAPTION1,left,top);
  top += v_spacing;

  if(heat2Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);
  
  
  rusPrinter->print(HEAT_BUTTON_CAPTION2,left,top);
  top += v_spacing;


  if(heat3Active)
    dc->setColor(INFO_BOX_CAPTION_COLOR);
  else
    dc->setColor(VGA_SILVER);

  
  rusPrinter->print(HEAT_BUTTON_CAPTION3,left,top);

  drawCalled = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTHeatSettingsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatSettingsScreen::TFTHeatSettingsScreen(uint8_t _channel)
{
  minTempBox = NULL;
  maxTempBox = NULL;
  ethalonTempBox = NULL;
  histeresisBox = NULL;
  sensorBox = NULL;
  airSensorBox = NULL;

  channel = _channel;

  sensorDataLeft = sensorDataTop = 0;
  sensorDataLeft2 = sensorDataTop2 = 0;
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatSettingsScreen::~TFTHeatSettingsScreen()
{
 delete screenButtons;
 delete minTempBox;
 delete maxTempBox;
 delete ethalonTempBox;
 delete histeresisBox;
 delete sensorBox;
 delete airSensorBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    heatSettings = settings->GetHeatSettings(channel);

    if(heatSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
    {
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      tempSensorsCount = 0;
    }

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
    {
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      humiditySensorsCount = 0; 
    }


    getSensorData(0, sensorDataString);
    getSensorData(1, sensorDataString2);

    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::getSensorData(uint8_t which, String& result)
{

  result = F("^ - ");
  result += char('~' + 1);
  result += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = which == 0 ? heatSettings.sensorIndex : heatSettings.airSensorIndex;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
  {
    return;
  }

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
  {
    return;
  }

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   result = F("^");

   result += tmp.Current;

   result += char('~' + 1);
   result += 'C';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::setup(TFTMenu* menuManager)
{
    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decMinTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   minTempBox = new TFTInfoBox(HEAT_MIN_TEMP_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incMinTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decMaxTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   maxTempBox = new TFTInfoBox(HEAT_MAX_TEMP_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incMaxTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decEthalonTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   ethalonTempBox = new TFTInfoBox(HEAT_ETHALON_TEMP_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incEthalonTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decAirSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   airSensorBox = new TFTInfoBox(HEAT_AIR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop2 = textBoxTopPos;
   sensorDataLeft2 = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_AIR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incAirSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   histeresisBox = new TFTInfoBox(HEAT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   sensorBox = new TFTInfoBox(HEAT_SENSOR_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(HEAT_SENSOR_CAPTION) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
   

 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetHeatSettings(channel,heatSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadHeatSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decMinTempButton || buttonID == incMinTempButton || buttonID == decMaxTempButton
  || buttonID == incMaxTempButton || buttonID == decEthalonTempButton || buttonID == incEthalonTempButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::onTick()
{
  if(tickerButton == decMinTempButton)
    incMinTemp(-3);
  else
  if(tickerButton == incMinTempButton)
    incMinTemp(3);
  else
  if(tickerButton == decMaxTempButton)
    incMaxTemp(-3);
  else
  if(tickerButton == incMaxTempButton)
    incMaxTemp(3);
  else
  if(tickerButton == decEthalonTempButton)
    incEthalonTemp(-3);
  else
  if(tickerButton == incEthalonTempButton)
    incEthalonTemp(3);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::incMinTemp(int val)
{
  int16_t old = heatSettings.minTemp;
  
  heatSettings.minTemp+=val;

  if(heatSettings.minTemp < 0)
    heatSettings.minTemp = 0;
  
  if(heatSettings.minTemp > HEAT_MAX_ALLOWED_TEMP)
    heatSettings.minTemp = HEAT_MAX_ALLOWED_TEMP;

  if(heatSettings.minTemp != old)
    drawValueInBox(minTempBox,heatSettings.minTemp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::incMaxTemp(int val)
{
  int16_t old = heatSettings.maxTemp;
  
  heatSettings.maxTemp+=val;

  if(heatSettings.maxTemp < 0)
    heatSettings.maxTemp = 0;
  
  if(heatSettings.maxTemp > HEAT_MAX_ALLOWED_TEMP)
    heatSettings.maxTemp = HEAT_MAX_ALLOWED_TEMP;

  if(heatSettings.maxTemp != old)
    drawValueInBox(maxTempBox,heatSettings.maxTemp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::incEthalonTemp(int val)
{
  int16_t old = heatSettings.ethalonTemp;
  
  heatSettings.ethalonTemp+=val;

  if(heatSettings.ethalonTemp < 0)
    heatSettings.ethalonTemp = 0;
  
  if(heatSettings.ethalonTemp > HEAT_MAX_ALLOWED_TEMP)
    heatSettings.ethalonTemp = HEAT_MAX_ALLOWED_TEMP;

  if(heatSettings.ethalonTemp != old)
    drawValueInBox(ethalonTempBox,heatSettings.ethalonTemp);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString, old2 = sensorDataString2;
    
    getSensorData(0, sensorDataString);
    getSensorData(1, sensorDataString2);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    if(sensorDataString2 != old2)
    {
      drawSensorData(menuManager, sensorDataString2,sensorDataLeft2,sensorDataTop2);  
    }
    sensorUpdateTimer = millis();
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("HEAT");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(heatSettings.active)
                heatSettings.active = false;
              else
                heatSettings.active = true;
                
              if(heatSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decMinTempButton)
          {
            incMinTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incMinTempButton)
          {
            incMinTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decMaxTempButton)
          {
            incMaxTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incMaxTempButton)
          {
            incMaxTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decEthalonTempButton)
          {
            incEthalonTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incEthalonTempButton)
          {
            incEthalonTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decHisteresisButton)
          {
            heatSettings.histeresis-=5;
            if(heatSettings.histeresis < 0)
              heatSettings.histeresis = 0;
      
            drawValueInBox(histeresisBox,formatHisteresis());
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incHisteresisButton)
          {
            heatSettings.histeresis+=5;
            if(heatSettings.histeresis > 50)
              heatSettings.histeresis = 50;
      
            drawValueInBox(histeresisBox,formatHisteresis());
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decSensorButton)
          {
            heatSettings.sensorIndex--;
            if(heatSettings.sensorIndex < 0)
              heatSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,heatSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(0,sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            heatSettings.sensorIndex++;
            if(heatSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              heatSettings.sensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;
		  
			if(heatSettings.sensorIndex < 0 || heatSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              heatSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,heatSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(0,sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == decAirSensorButton)
          {
            heatSettings.airSensorIndex--;
            if(heatSettings.airSensorIndex < 0)
              heatSettings.airSensorIndex = 0;
      
            drawValueInBox(airSensorBox,heatSettings.airSensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(1,sensorDataString2);
            drawSensorData(menuManager, sensorDataString2,sensorDataLeft2,sensorDataTop2);
          }
          else if(pressed_button == incAirSensorButton)
          {
            heatSettings.airSensorIndex++;
            if(heatSettings.airSensorIndex >= (humiditySensorsCount + tempSensorsCount))
              heatSettings.airSensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;
		  
			if(heatSettings.airSensorIndex < 0 || heatSettings.airSensorIndex >= (humiditySensorsCount + tempSensorsCount))
              heatSettings.airSensorIndex = 0;
      
            drawValueInBox(airSensorBox,heatSettings.airSensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(1,sensorDataString2);
            drawSensorData(menuManager, sensorDataString2,sensorDataLeft2,sensorDataTop2);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatSettingsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  minTempBox->draw(menuManager);
  drawValueInBox(minTempBox,heatSettings.minTemp);

  maxTempBox->draw(menuManager);
  drawValueInBox(maxTempBox,heatSettings.maxTemp);

  ethalonTempBox->draw(menuManager);
  drawValueInBox(ethalonTempBox,heatSettings.ethalonTemp);

  airSensorBox->draw(menuManager);
  drawValueInBox(airSensorBox,heatSettings.airSensorIndex);
  
  histeresisBox->draw(menuManager);
  drawValueInBox(histeresisBox,formatHisteresis());

  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,heatSettings.sensorIndex);


  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
  drawSensorData(menuManager, sensorDataString2,sensorDataLeft2,sensorDataTop2);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String TFTHeatSettingsScreen::formatHisteresis()
{
  int16_t val = heatSettings.histeresis/10;
  int16_t fract = heatSettings.histeresis%10;

  String result;
  result += val;
  result += '.';
  result += fract;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTHeatScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatScreen::TFTHeatScreen()
{
  workTimeBox1 = NULL;
  workTimeBox2 = NULL;
  workTimeBox3 = NULL;

  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTHeatScreen::~TFTHeatScreen()
{
 delete screenButtons;
 delete workTimeBox1;
 delete workTimeBox2;
 delete workTimeBox3;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decWorkTimeButton1 || buttonID == decWorkTimeButton2 || buttonID == decWorkTimeButton3
  || buttonID == incWorkTimeButton1 || buttonID == incWorkTimeButton2 || buttonID == incWorkTimeButton3)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::onTick()
{
  if(tickerButton == decWorkTimeButton1)
    incWorkTime1(-3);
  else
  if(tickerButton == decWorkTimeButton2)
    incWorkTime2(-3);
  else
  if(tickerButton == decWorkTimeButton3)
    incWorkTime3(-3);
  else
  if(tickerButton == incWorkTimeButton1)
    incWorkTime1(3);
  else
  if(tickerButton == incWorkTimeButton2)
    incWorkTime2(3);
  else
  if(tickerButton == incWorkTimeButton3)
    incWorkTime3(3);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    heatDriveWorkTime1 = settings->GetHeatDriveWorkTime(0);
    heatDriveWorkTime2 = settings->GetHeatDriveWorkTime(1);
    heatDriveWorkTime3 = settings->GetHeatDriveWorkTime(2);

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;

    int leftRowButtonsWidth = TFT_ARROW_BUTTON_WIDTH*2 + INFO_BOX_V_SPACING*2 + TFT_TEXT_INPUT_WIDTH;
   
    // теперь добавляем наши кнопки

    heat1Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, HEAT_BUTTON_CAPTION1);
 
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING; //INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decWorkTimeButton1 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox1 = new TFTInfoBox(HEAT_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton1 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;

    heat2Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, HEAT_BUTTON_CAPTION2);
    
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING; //INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decWorkTimeButton2 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox2 = new TFTInfoBox(HEAT_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton2 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

    heat3Button = screenButtons->addButton( leftPos ,  topPos, leftRowButtonsWidth,  TFT_ARROW_BUTTON_HEIGHT, HEAT_BUTTON_CAPTION3);
 
   leftPos += leftRowButtonsWidth + INFO_BOX_V_SPACING; //INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decWorkTimeButton3 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   workTimeBox3 = new TFTInfoBox(HEAT_WORKTIME_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incWorkTimeButton3 = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
   
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetHeatDriveWorkTime(0,heatDriveWorkTime1);
  settings->SetHeatDriveWorkTime(1,heatDriveWorkTime2);
  settings->SetHeatDriveWorkTime(2,heatDriveWorkTime3);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  LogicManageModule->ReloadHeatSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::incWorkTime1(int val)
{
   int16_t oldW = heatDriveWorkTime1;
  heatDriveWorkTime1+=val;

  if(heatDriveWorkTime1 < 0 || heatDriveWorkTime1 > HEAT_MAX_WORK_TIME)
    heatDriveWorkTime1 = 0;
  
  if(heatDriveWorkTime1 != oldW)
    drawValueInBox(workTimeBox1,heatDriveWorkTime1);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::incWorkTime2(int val)
{
   int16_t oldW = heatDriveWorkTime2;
  heatDriveWorkTime2+=val;

  if(heatDriveWorkTime2 < 0 || heatDriveWorkTime2 > HEAT_MAX_WORK_TIME)
    heatDriveWorkTime2 = 0;

  if(heatDriveWorkTime2 != oldW)
    drawValueInBox(workTimeBox2,heatDriveWorkTime2);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::incWorkTime3(int val)
{
   int16_t oldW = heatDriveWorkTime3;
  heatDriveWorkTime3+=val;

  if(heatDriveWorkTime3 < 0 || heatDriveWorkTime3 > HEAT_MAX_WORK_TIME)
    heatDriveWorkTime3 = 0;

  if(heatDriveWorkTime3 != oldW)
    drawValueInBox(workTimeBox3,heatDriveWorkTime3);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
   
      if(pressed_button == backButton)
      {
        menuManager->switchToScreen("SENSORS");
        return;
      }
     else
      if(pressed_button == saveButton)
      {
        saveSettings();
        blinkSaveSettingsButton(false);
        screenButtons->disableButton(saveButton,true);
        return;
      }
      else
      if(pressed_button == heat1Button)
      {
        menuManager->switchToScreen("HEAT1");
        return;        
      }
      else
      if(pressed_button == heat2Button)
      {
        menuManager->switchToScreen("HEAT2");
        return;        
      }
      else
      if(pressed_button == heat3Button)
      {
        menuManager->switchToScreen("HEAT3");
        return;        
      }
      else 
      if(pressed_button == decWorkTimeButton1)
      {
        incWorkTime1(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton1)
      {
        incWorkTime1(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == decWorkTimeButton2)
      {
        incWorkTime2(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton2)
      {
        incWorkTime2(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == decWorkTimeButton3)
      {
        incWorkTime3(-1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
      else 
      if(pressed_button == incWorkTimeButton3)
      {
        incWorkTime3(1);
        screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
        blinkSaveSettingsButton(true);
      }
    } // if(pressed_button != -1)
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTHeatScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  workTimeBox1->draw(menuManager);
  drawValueInBox(workTimeBox1,heatDriveWorkTime1); 

  workTimeBox2->draw(menuManager);
  drawValueInBox(workTimeBox2,heatDriveWorkTime2); 

  workTimeBox3->draw(menuManager);
  drawValueInBox(workTimeBox3,heatDriveWorkTime3); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HEAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTDriveScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDriveScreen::TFTDriveScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDriveScreen::~TFTDriveScreen()
{
 delete screenButtons;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDriveScreen::setup(TFTMenu* menuManager)
{

    UTFT* dc = menuManager->getDC();

  if(!dc)
  {
    return;
  }
    
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setIconFont(IconsFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
    
    int screenWidth = dc->getDisplayXSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int initialLeftPos = (screenWidth - widthOccupied)/2;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING;
   
    const int spacing = 10;

    const int buttonHeight = 73;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;

   int leftPos = initialLeftPos;
    
    
    // кнопки управления

    // первый ряд
    windowsButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, WM_WINDOWS_CAPTION);
    screenButtons->setButtonHasIcon(windowsButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_TEMP_SENSORS
    screenButtons->disableButton(windowsButton);
    #endif

    waterButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, WM_WATERING_CAPTION);
    screenButtons->setButtonHasIcon(waterButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_WATERING_MODULE
    screenButtons->disableButton(waterButton);
    #endif
    
    lightButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, WM_LIGHT_CAPTION);
    screenButtons->setButtonHasIcon(lightButton);
    leftPos = initialLeftPos;
    topPos += buttonHeight + INFO_BOX_V_SPACING;
    #ifndef USE_LUMINOSITY_MODULE
    screenButtons->disableButton(lightButton);
    #endif

    // вторая строка
    heatButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, HEAT_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(heatButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_HEAT_MODULE
    screenButtons->disableButton(heatButton);
    #endif

    shadowButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, WM_SHADOW_CAPTION);
    screenButtons->setButtonHasIcon(shadowButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_SHADOW_MODULE  
    screenButtons->disableButton(shadowButton);
    #endif

    flowButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, FLOW_CONTROL_BUTTON);
    screenButtons->setButtonHasIcon(flowButton);
    leftPos = initialLeftPos;
    topPos += buttonHeight + INFO_BOX_V_SPACING;
    #ifndef USE_WATERFLOW_MODULE
    screenButtons->disableButton(flowButton);
    #endif

    // третья строка
    cycleVentButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, CYCLE_VENT_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(cycleVentButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_CYCLE_VENT_MODULE
    screenButtons->disableButton(cycleVentButton);
    #endif

    ventButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, VENT_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(ventButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_VENT_MODULE
    screenButtons->disableButton(ventButton);
    #endif

    thermostatButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, THERMOSTAT_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(thermostatButton);
    #ifndef USE_THERMOSTAT_MODULE
    screenButtons->disableButton(thermostatButton);
    #endif


    // четвёртая строка
    topPos += buttonHeight + INFO_BOX_V_SPACING;
    leftPos = initialLeftPos;

    co2Button = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, CONTROL_CO2_CAPTION);
    screenButtons->setButtonHasIcon(co2Button);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_CO2_MODULE
    screenButtons->disableButton(co2Button);
    #endif

    
    phButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, CONTROL_PH_CAPTION);
    leftPos += controlsButtonsWidth + spacing;
    screenButtons->setButtonHasIcon(phButton);
    #ifndef USE_PH_MODULE
    screenButtons->disableButton(phButton);
    #endif

    doorButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, DOORS_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(doorButton);
    leftPos += controlsButtonsWidth + spacing;
     #ifndef USE_DOOR_MODULE
    screenButtons->disableButton(doorButton);
    #endif    


    // пятый ряд
    topPos += buttonHeight + INFO_BOX_V_SPACING;
    leftPos = initialLeftPos;
    
   // первая - кнопка назад
    backButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    leftPos += controlsButtonsWidth + spacing;

    // кнопка распрыскивания
    sprayButton = screenButtons->addButton( leftPos ,  topPos, controlsButtonsWidth,  buttonHeight, SPRAY_BUTTON_CAPTION);
    screenButtons->setButtonHasIcon(sprayButton);
    leftPos += controlsButtonsWidth + spacing;
    #ifndef USE_HUMIDITY_SPRAY_MODULE
    screenButtons->disableButton(sprayButton);
    #endif
   
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDriveScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
         menuManager->resetIdleTimer();

        if(pressed_button == backButton)
        {
          menuManager->switchToScreen("IDLE");
          return;
        }
        else
        if(pressed_button == windowsButton)
        {
          menuManager->switchToScreen("WINDOW");
          return;
        }
        else
        if(pressed_button == waterButton)
        {
          menuManager->switchToScreen("WATER");
          return;
        }
        else
        if(pressed_button == lightButton)
        {
          menuManager->switchToScreen("LIGHT");
          return;
        }
        else
        if(pressed_button == heatButton)
        {
          menuManager->switchToScreen("HDRIVE");
          return;
        }
        else
        if(pressed_button == shadowButton)
        {
          menuManager->switchToScreen("SDRIVE");
          return;
        }
        else
        if(pressed_button == flowButton)
        {
          menuManager->switchToScreen("FLOW");
          return;
        }
        else
        if(pressed_button == cycleVentButton)
        {
          menuManager->switchToScreen("CYCLE_VENT");
          return;
        }
        else
        if(pressed_button == ventButton)
        {
          menuManager->switchToScreen("VENT");
        }
        else
        if(pressed_button == sprayButton)
        {
          menuManager->switchToScreen("SPRAY");
        }
        else
        if(pressed_button == thermostatButton)
        {
          menuManager->switchToScreen("THERMOSTAT");
        }
        else
        if(pressed_button == phButton)
        {
          menuManager->switchToScreen("PHControl");
        }
        else
        if(pressed_button == co2Button)
        {
          menuManager->switchToScreen("CO2Control");
        }
        else // USE_DOOR_MODULE
        if(pressed_button == doorButton)
        {
          menuManager->switchToScreen("DOORS");
        }
    } // if(pressed_button != -1)
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDriveScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DOOR_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDoorScreen* DoorScreen = NULL;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDoorScreen::TFTDoorScreen()
{
  tempBox = NULL;
  sensorBox = NULL;
  startWorkTimeBox = NULL;
  endWorkTimeBox = NULL;
  driveWorkTimeBox = NULL;

  channel = 0;

  sensorDataLeft = sensorDataTop = 0;
  tickerButton = -1;

  DoorScreen = this;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDoorScreen::~TFTDoorScreen()
{
 delete screenButtons;
 delete tempBox;
 delete sensorBox;
 delete startWorkTimeBox;
 delete endWorkTimeBox;
 delete driveWorkTimeBox;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::show(uint8_t _channel)
{
  channel = _channel;
  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
    GlobalSettings* settings = MainController->GetSettings();
    
    doorSettings = settings->GetDoorSettings(channel);

    if(doorSettings.active)
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
      screenButtons->relabelButton(onOffButton,WM_ON_CAPTION);
    }
    else
    {
      screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION);
    }

    screenButtons->disableButton(saveButton);

    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);

    AbstractModule* module = MainController->GetModuleByID("STATE");
    if(module)
    {
      tempSensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      tempSensorsCount = 0;
    }

    module = MainController->GetModuleByID("HUMIDITY");
    if(module)
    {
      humiditySensorsCount = module->State.GetStateCount(StateTemperature);
    }
    else
    {
      humiditySensorsCount = 0; 
    }

    getSensorData(sensorDataString);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::getSensorData(String& result)
{

  result = F("^ - ");
  result += char('~' + 1);
  result += F("C    ");

  const char* moduleName = "STATE";
  uint16_t sensorIndex = doorSettings.sensorIndex;

  if(sensorIndex >= tempSensorsCount)
  {
    sensorIndex -= tempSensorsCount;
    moduleName = "HUMIDITY";
  }
  
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
    return;

  OneState* sensorState = module->State.GetState(StateTemperature,sensorIndex);
  if(!sensorState)
    return;

  if(sensorState->HasData())
  {
   TemperaturePair tmp = *sensorState;
   result = F("^");

   result += tmp.Current;

   result += char('~' + 1);
   result += 'C';
   
  }

   while(result.length() < 10)
    result += ' ';
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::drawSensorData(TFTMenu* menuManager, String& which, int left, int top)
{
    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);

    dc->setColor(VGA_RED);
    dc->setBackColor(TFT_BACK_COLOR);

    dc->print(which.c_str(), left,top);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::setup(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  if(!dc)
  {
    return;
  }
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();
    
    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);
    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    onOffButton = screenButtons->addButton( initialLeftPos + spacing*2 +  controlsButtonsWidth*2,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_ON_CAPTION);
    
    screenButtons->setButtonFontColor(onOffButton,CHANNELS_BUTTONS_TEXT_COLOR);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

    int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
    int textBoxTopPos = topPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   
    // теперь добавляем наши кнопки
    decStartWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   startWorkTimeBox = new TFTInfoBox("Открыть двери в:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incStartWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decEndWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   endWorkTimeBox = new TFTInfoBox("Закрыть двери в:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incEndWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);


   // вторая строка
   textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decDriveWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   driveWorkTimeBox = new TFTInfoBox("Работа привода, с:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incDriveWorkTimeButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   const char* sensCaption = "Датчик:";
   sensorBox = new TFTInfoBox(sensCaption,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   sensorDataTop = textBoxTopPos;
   sensorDataLeft = leftPos + textFontWidth*rusPrinter->utf8_strlen(sensCaption) + textFontWidth*3 - (TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING);

   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incSensorButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

   // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;

   decTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   tempBox = new TFTInfoBox("Порог температуры:",TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;   
   incTempButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
  
 
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::saveSettings()
{
  GlobalSettings* settings = MainController->GetSettings();
  
  settings->SetDoorSettings(channel,doorSettings);
 
  //УВЕДОМЛЯЕМ МОДУЛЬ КОНТРОЛЯ, ЧТО ЕМУ НЕОБХОДИМО ПЕРЕЗАГРУЗИТЬ НАСТРОЙКИ
  Doors->ReloadDoorSettings();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;
  if(buttonID == decTempButton || buttonID == incTempButton || buttonID == decStartWorkTimeButton || buttonID == incStartWorkTimeButton
  || buttonID == decEndWorkTimeButton || buttonID == incEndWorkTimeButton || buttonID == decDriveWorkTimeButton
  || buttonID == incDriveWorkTimeButton)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::onTick()
{
  if(tickerButton == decTempButton)
    incTemp(-3);
  else
  if(tickerButton == incTempButton)
    incTemp(3);
  else
  if(tickerButton == decStartWorkTimeButton)
    incStartWorkTime(-30);
  else
  if(tickerButton == incStartWorkTimeButton)
    incStartWorkTime(30);
  else
  if(tickerButton == decEndWorkTimeButton)
    incEndWorkTime(-30);
  else
  if(tickerButton == incEndWorkTimeButton)
    incEndWorkTime(30);
  else
  if(tickerButton == decDriveWorkTimeButton)
    incDriveWorkTime(-5);
  else
  if(tickerButton == incDriveWorkTimeButton)
    incDriveWorkTime(5);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::incTemp(int val)
{
  int16_t old = doorSettings.minOpenTemp;
  
  doorSettings.minOpenTemp+=val;

  if(doorSettings.minOpenTemp < 0)
    doorSettings.minOpenTemp = 0;
  
  if(doorSettings.minOpenTemp > 127)
    doorSettings.minOpenTemp = 127;

  if(doorSettings.minOpenTemp != old)
    drawValueInBox(tempBox,doorSettings.minOpenTemp);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::incStartWorkTime(int val)
{
  int32_t old = doorSettings.startWorkTime;
  
  doorSettings.startWorkTime+=val;

  if(doorSettings.startWorkTime < 0)
    doorSettings.startWorkTime = 0;
  
  if(doorSettings.startWorkTime > 1440)
    doorSettings.startWorkTime = 1440;

  if(doorSettings.startWorkTime != old)
    drawTimeInBox(startWorkTimeBox,doorSettings.startWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::incEndWorkTime(int val)
{
  int32_t old = doorSettings.endWorkTime;
  
  doorSettings.endWorkTime+=val;

  if(doorSettings.endWorkTime < 0)
    doorSettings.endWorkTime = 0;
  
  if(doorSettings.endWorkTime > 1440)
    doorSettings.endWorkTime = 1440;

  if(doorSettings.endWorkTime != old)
    drawTimeInBox(endWorkTimeBox,doorSettings.endWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::incDriveWorkTime(int val)
{
  int32_t old = doorSettings.driveWorkTime;
  
  doorSettings.driveWorkTime+=val;

  if(doorSettings.driveWorkTime < 0)
    doorSettings.driveWorkTime = 0;
  
  if(doorSettings.driveWorkTime > 1440)
    doorSettings.driveWorkTime = 1440;

  if(doorSettings.driveWorkTime != old)
    drawValueInBox(driveWorkTimeBox,doorSettings.driveWorkTime);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  static uint32_t sensorUpdateTimer = millis();
  if(millis() - sensorUpdateTimer > 2000)
  {
    String old1 = sensorDataString;
    
    getSensorData(sensorDataString);
    
    if(sensorDataString != old1)
    {
      drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
    }

    sensorUpdateTimer = millis();
  }     


  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)

 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
      menuManager->resetIdleTimer();
    
   
          if(pressed_button == backButton)
          {
            menuManager->switchToScreen("DOORS");
            return;
          }
          else
          if(pressed_button == saveButton)
          {
            saveSettings();
            blinkSaveSettingsButton(false);
            screenButtons->disableButton(saveButton,true);
            return;
          }
          else
          if(pressed_button == onOffButton)
          {
              if(doorSettings.active)
                doorSettings.active = false;
              else
                doorSettings.active = true;
                
              if(doorSettings.active)
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_ON_COLOR);
                screenButtons->relabelButton(onOffButton,WM_ON_CAPTION,true);
              }
              else
              {
                screenButtons->setButtonBackColor(onOffButton,MODE_OFF_COLOR);
                screenButtons->relabelButton(onOffButton,WM_OFF_CAPTION,true);
              }
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);     
          }
          else if(pressed_button == decTempButton)
          {
            incTemp(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incTempButton)
          {
            incTemp(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decStartWorkTimeButton)
          {
            incStartWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incStartWorkTimeButton)
          {
            incStartWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == decEndWorkTimeButton)
          {
            incEndWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incEndWorkTimeButton)
          {
            incEndWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }  
          else if(pressed_button == decDriveWorkTimeButton)
          {
            incDriveWorkTime(-1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }
          else if(pressed_button == incDriveWorkTimeButton)
          {
            incDriveWorkTime(1);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
          }           
          else if(pressed_button == decSensorButton)
          {
            doorSettings.sensorIndex--;
            if(doorSettings.sensorIndex < 0)
              doorSettings.sensorIndex = 0;
      
            drawValueInBox(sensorBox,doorSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
      
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }
          else if(pressed_button == incSensorButton)
          {
            doorSettings.sensorIndex++;
            if(doorSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
              doorSettings.sensorIndex = (humiditySensorsCount + tempSensorsCount) - 1;

            if(doorSettings.sensorIndex < 0 || doorSettings.sensorIndex >= (humiditySensorsCount + tempSensorsCount))
                doorSettings.sensorIndex = 0;        
      
            drawValueInBox(sensorBox,doorSettings.sensorIndex);
            screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
            blinkSaveSettingsButton(true);
            
            getSensorData(sensorDataString);
            drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);
          }

    } // if(pressed_button != -1)
    
 } // if(screenButtons)

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  tempBox->draw(menuManager);
  drawValueInBox(tempBox,doorSettings.minOpenTemp);

  startWorkTimeBox->draw(menuManager);
  drawTimeInBox(startWorkTimeBox,doorSettings.startWorkTime);

  endWorkTimeBox->draw(menuManager);
  drawTimeInBox(endWorkTimeBox,doorSettings.endWorkTime);

  driveWorkTimeBox->draw(menuManager);
  drawValueInBox(driveWorkTimeBox,doorSettings.driveWorkTime);
  
  sensorBox->draw(menuManager);
  drawValueInBox(sensorBox,doorSettings.sensorIndex);

  drawSensorData(menuManager, sensorDataString,sensorDataLeft,sensorDataTop);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTDoorsScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDoorsScreen::TFTDoorsScreen()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTDoorsScreen::~TFTDoorsScreen()
{
 delete screenButtons;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorsScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

    UTFT* dc = menuManager->getDC();
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
 
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);


    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/2;
    int channelsButtonLeft = initialLeftPos;

    door1Button = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "ДВЕРЬ #1");
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING*2;
    door2Button = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "ДВЕРЬ #2");
    
    topPos += INFO_BOX_V_SPACING + ALL_CHANNELS_BUTTON_HEIGHT;
    channelsButtonLeft = initialLeftPos;

    door1ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(door1ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);

    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING*2;
    door2ModeButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, AUTO_MODE_LABEL);
    screenButtons->setButtonFontColor(door2ModeButton,CHANNELS_BUTTONS_TEXT_COLOR);   

    topPos += INFO_BOX_V_SPACING + ALL_CHANNELS_BUTTON_HEIGHT;
    channelsButtonLeft = initialLeftPos;

    door1OpenCloseButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "");
    screenButtons->setButtonFontColor(door1OpenCloseButton,CHANNELS_BUTTONS_TEXT_COLOR);

    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING*2;
    door2OpenCloseButton = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, "");
    screenButtons->setButtonFontColor(door2OpenCloseButton,CHANNELS_BUTTONS_TEXT_COLOR);       
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorsScreen::onActivate(TFTMenu* menuManager)
{
  door1Mode = Doors->GetDoorMode(0);
  door2Mode = Doors->GetDoorMode(1);

  door1OpenFlag = Doors->IsDoorOpen(0);
  door2OpenFlag = Doors->IsDoorOpen(1);

  bool door1Busy = Doors->IsDoorBusy(0);
  bool door2Busy = Doors->IsDoorBusy(1);

 if(screenButtons)
 {
  
   if(door1Mode == doorModeAutomatic)
   {
      screenButtons->setButtonBackColor(door1ModeButton,MODE_ON_COLOR);
      screenButtons->relabelButton(door1ModeButton,AUTO_MODE_LABEL,false);
   }
   else
   {
      screenButtons->setButtonBackColor(door1ModeButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(door1ModeButton,MANUAL_MODE_LABEL,false);
   }    

   if(door2Mode == doorModeAutomatic)
   {
      screenButtons->setButtonBackColor(door2ModeButton,MODE_ON_COLOR);
      screenButtons->relabelButton(door2ModeButton,AUTO_MODE_LABEL,false);
   }
   else
   {
      screenButtons->setButtonBackColor(door2ModeButton,MODE_OFF_COLOR);      
      screenButtons->relabelButton(door2ModeButton,MANUAL_MODE_LABEL,false);
   }    

  
  if(door1OpenFlag)
   {
      screenButtons->setButtonBackColor(door1OpenCloseButton,MODE_ON_COLOR);
      if(door1Busy)
      {
        screenButtons->disableButton(door1OpenCloseButton,false);
        screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ОТКРЫВАЕТСЯ",false);
      }
      else
      {
        screenButtons->enableButton(door1OpenCloseButton,false);
        screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ОТКРЫТА",false);        
      }
   }
   else
   {
      screenButtons->setButtonBackColor(door1OpenCloseButton,MODE_OFF_COLOR);      
      if(door1Busy)
      {
        screenButtons->disableButton(door1OpenCloseButton,false);
        screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ЗАКРЫВАЕТСЯ",false);
      }
      else
      {
        screenButtons->enableButton(door1OpenCloseButton,false);
        screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ЗАКРЫТА",false);        
      }
   }       

  if(door2OpenFlag)
   {
      screenButtons->setButtonBackColor(door2OpenCloseButton,MODE_ON_COLOR);
      if(door2Busy)
      {
        screenButtons->disableButton(door2OpenCloseButton,false);
        screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ОТКРЫВАЕТСЯ",false);
      }
      else
      {
        screenButtons->enableButton(door2OpenCloseButton,false);
        screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ОТКРЫТА",false);
      }
   }
   else
   {
      screenButtons->setButtonBackColor(door2OpenCloseButton,MODE_OFF_COLOR);
      if(door2Busy)
      { 
        screenButtons->disableButton(door2OpenCloseButton,false);    
        screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ЗАКРЫВАЕТСЯ",false);
      }
      else
      {
        screenButtons->enableButton(door2OpenCloseButton,false);
        screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ЗАКРЫТА",false);
      }
   } 
 }   
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorsScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

 if(screenButtons)
 {

  DoorWorkMode nowMode = Doors->GetDoorMode(0);
  if(nowMode != door1Mode)
  {
    door1Mode = nowMode;

    if(nowMode == doorModeAutomatic)
     {
        screenButtons->setButtonBackColor(door1ModeButton,MODE_ON_COLOR);
        screenButtons->relabelButton(door1ModeButton,AUTO_MODE_LABEL,true);
     }
     else
     {
        screenButtons->setButtonBackColor(door1ModeButton,MODE_OFF_COLOR);      
        screenButtons->relabelButton(door1ModeButton,MANUAL_MODE_LABEL,true);
     }      
  }

  nowMode = Doors->GetDoorMode(1);
  if(nowMode != door2Mode)
  {
    door2Mode = nowMode;

    if(nowMode == doorModeAutomatic)
     {
        screenButtons->setButtonBackColor(door2ModeButton,MODE_ON_COLOR);
        screenButtons->relabelButton(door2ModeButton,AUTO_MODE_LABEL,true);
     }
     else
     {
        screenButtons->setButtonBackColor(door2ModeButton,MODE_OFF_COLOR);      
        screenButtons->relabelButton(door2ModeButton,MANUAL_MODE_LABEL,true);
     }      
  }

  bool nowDoorFlag = Doors->IsDoorOpen(0);
  if(door1OpenFlag != nowDoorFlag)
  {
    door1OpenFlag = nowDoorFlag;
    if(nowDoorFlag)
     {
        screenButtons->setButtonBackColor(door1OpenCloseButton,MODE_ON_COLOR);
        if(Doors->IsDoorBusy(0))
        {
          screenButtons->disableButton(door1OpenCloseButton,false);
          screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ОТКРЫВАЕТСЯ",true);
        }
        else
        {
          screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ОТКРЫТА",true);
        }
     }
     else
     {
        screenButtons->setButtonBackColor(door1OpenCloseButton,MODE_OFF_COLOR);      
        if(Doors->IsDoorBusy(0))
        {
          screenButtons->disableButton(door1OpenCloseButton,false);
          screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ЗАКРЫВАЕТСЯ",true);
        }
        else
        {
          screenButtons->relabelButton(door1OpenCloseButton,"ДВЕРЬ ЗАКРЫТА",true);
        }
     }    
  }

  nowDoorFlag = Doors->IsDoorOpen(1);
  if(door2OpenFlag != nowDoorFlag)
  {
    door2OpenFlag = nowDoorFlag;
    if(nowDoorFlag)
     {
        screenButtons->setButtonBackColor(door2OpenCloseButton,MODE_ON_COLOR);
        if(Doors->IsDoorBusy(1))
        {
          screenButtons->disableButton(door2OpenCloseButton,false);
          screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ОТКРЫВАЕТСЯ",true);
        }
        else
        {
          screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ОТКРЫТА",true);
        }
     }
     else
     {
        screenButtons->setButtonBackColor(door2OpenCloseButton,MODE_OFF_COLOR);      
        if(Doors->IsDoorBusy(1))
        {
          screenButtons->disableButton(door2OpenCloseButton,false);
          screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ЗАКРЫВАЕТСЯ",true);
        }
        else
        {
          screenButtons->relabelButton(door2OpenCloseButton,"ДВЕРЬ ЗАКРЫТА",true);
        }
     }    
  }

  if(!Doors->IsDoorBusy(0) && !screenButtons->buttonEnabled(door1OpenCloseButton))
  {
    screenButtons->relabelButton(door1OpenCloseButton,Doors->IsDoorOpen(0) ? "ДВЕРЬ ОТКРЫТА" : "ДВЕРЬ ЗАКРЫТА",false);
    screenButtons->enableButton(door1OpenCloseButton,true);
  }

  if(!Doors->IsDoorBusy(1) && !screenButtons->buttonEnabled(door2OpenCloseButton))
  {
    screenButtons->relabelButton(door2OpenCloseButton,Doors->IsDoorOpen(1) ? "ДВЕРЬ ОТКРЫТА" : "ДВЕРЬ ЗАКРЫТА",false);
    screenButtons->enableButton(door2OpenCloseButton,true);
  }

 
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
         menuManager->resetIdleTimer();

        if(pressed_button == backButton)
        {
          menuManager->switchToScreen("DRIVE");
          return;
        }
        else
        if(pressed_button == door1Button)
        {
          DoorScreen->show(0);
          return;
        }
        else 
        if(pressed_button == door2Button)
        {
           DoorScreen->show(1);
           return;
        }
        else
        if(pressed_button == door1ModeButton)
        {
          DoorWorkMode requestedMode;          
          if(door1Mode == doorModeAutomatic)
          {
            requestedMode = doorModeManual;
          }
          else
          {
            requestedMode = doorModeAutomatic;
          }

          Doors->SetDoorMode(0,requestedMode);

          return;
        }
        else
        if(pressed_button == door2ModeButton)
        {
          DoorWorkMode requestedMode;          
          if(door2Mode == doorModeAutomatic)
          {
            requestedMode = doorModeManual;
          }
          else
          {
            requestedMode = doorModeAutomatic;
          }

          Doors->SetDoorMode(1,requestedMode);

          return;
        }
        else
        if(pressed_button == door1OpenCloseButton)
        {

          Doors->SetDoorMode(0,doorModeManual);
          if(door1OpenFlag)
          {
            Doors->CloseDoor(0);
          }
          else
          {
            Doors->OpenDoor(0);
          }

          screenButtons->disableButton(door1OpenCloseButton,!screenButtons->buttonEnabled(door1OpenCloseButton));

          return;
        }
        else
        if(pressed_button == door2OpenCloseButton)
        {

          Doors->SetDoorMode(1,doorModeManual);
          if(door2OpenFlag)
          {
            Doors->CloseDoor(1);
          }
          else
          {
            Doors->OpenDoor(1);
          }

          screenButtons->disableButton(door2OpenCloseButton,!screenButtons->buttonEnabled(door2OpenCloseButton));

          return;
        }
       
        
    } // if(pressed_button != -1)
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTDoorsScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }


  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }


}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_DOOR_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERFLOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTFlowScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTFlowScreen::TFTFlowScreen()
{
  confirmMode = false;
  messageBoxOkPressed = false;
  wantShowOkMessage = false;
  flowToReset = 0;
  tickerButton = -1;

  cal1Box = NULL;
  cal2Box = NULL;

  cal1 = 0;
  cal2 = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTFlowScreen::~TFTFlowScreen()
{
 delete screenButtons;
 delete cal1Box;
 delete cal2Box;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::onButtonPressed(TFTMenu* menuManager,int buttonID)
{
  tickerButton = -1;

  if(buttonID == decCal1Button || buttonID == incCal1Button || buttonID == decCal2Button || buttonID == incCal2Button)
  {
    tickerButton = buttonID;
    Ticker.start(this);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::onButtonReleased(TFTMenu* menuManager,int buttonID)
{
  Ticker.stop();
  tickerButton = -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::onTick()
{
  
  if(tickerButton == decCal1Button)
    incCal1(-3);
  else
  if(tickerButton == incCal1Button)
    incCal1(3);
  else
  if(tickerButton == decCal2Button)
    incCal2(-3);
  else
  if(tickerButton == incCal2Button)
    incCal2(3);


}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

   //загружаем факторы калибровки
  uint32_t addr = WATERFLOW_EEPROM_ADDR + sizeof(unsigned long)*2;

  //Serial.println("TFT");
 // Serial.println(addr);
  
  cal1 = MemRead(addr++);
  cal2 = MemRead(addr); 

 //Serial.println(cal1);
// Serial.println(cal2);

  if(cal1 == 0xFF || cal1 == 0)
    cal1 = WATERFLOW_CALIBRATION_FACTOR;

  if(cal2 == 0xFF || cal2 == 0)
    cal2 = WATERFLOW_CALIBRATION_FACTOR;

    UTFT* dc = menuManager->getDC();
    
    screenButtons = new UTFT_Buttons_Rus(dc, menuManager->getTouch(),menuManager->getRusPrinter());
    screenButtons->setTextFont(BigRusFont);
    screenButtons->setSymbolFont(SensorFont);
    screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);


    UTFTRus* rusPrinter = menuManager->getRusPrinter();
    
    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();

    dc->setFont(BigRusFont);
    int textFontHeight = dc->getFontYsize();
    //int textFontWidth = dc->getFontXsize();

    // вычисляем ширину всего занятого пространства
    int widthOccupied = TFT_TEXT_INPUT_WIDTH*2 + TFT_ARROW_BUTTON_WIDTH*4 + INFO_BOX_V_SPACING*6;    
    
    // теперь вычисляем левую границу для начала рисования
    int leftPos = (screenWidth - widthOccupied)/2;
    int initialLeftPos = leftPos;
    
    // теперь вычисляем верхнюю границу для отрисовки кнопок
    int topPos = INFO_BOX_V_SPACING*2;
    int secondRowTopPos = topPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
   // int thirdRowTopPos = secondRowTopPos + TFT_ARROW_BUTTON_HEIGHT + INFO_BOX_V_SPACING*2;
    
    const int spacing = 10;

    int buttonHeight = TFT_ARROW_BUTTON_HEIGHT;

    int controlsButtonsWidth = (screenWidth - spacing*2 - initialLeftPos*2)/3;
    int controlsButtonsTop = screenHeight - buttonHeight - spacing;
   // первая - кнопка назад
    backButton = screenButtons->addButton( initialLeftPos ,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_BACK_CAPTION);

    saveButton = screenButtons->addButton( initialLeftPos + spacing +  controlsButtonsWidth,  controlsButtonsTop, controlsButtonsWidth,  buttonHeight, WM_SAVE_CAPTION);
    screenButtons->disableButton(saveButton);


    static char leftArrowCaption[2] = {0};
    static char rightArrowCaption[2] = {0};

    leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
    rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);



    // первая строка
    int channelsButtonWidth = (widthOccupied - INFO_BOX_V_SPACING*2)/2;
    int channelsButtonLeft = initialLeftPos;

    resetFlow1Button = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, FLOW_RESET_BUTTON1);
    channelsButtonLeft += channelsButtonWidth + INFO_BOX_V_SPACING*2;
    resetFlow2Button = screenButtons->addButton( channelsButtonLeft ,  topPos, channelsButtonWidth,  ALL_CHANNELS_BUTTON_HEIGHT, FLOW_RESET_BUTTON2);
    

   // вторая строка
   int textBoxTopPos = secondRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   int textBoxHeightWithCaption =  TFT_TEXT_INPUT_HEIGHT + textFontHeight + INFO_BOX_CONTENT_PADDING;
   
   topPos = secondRowTopPos;
   leftPos = initialLeftPos;
   
   decCal1Button = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   cal1Box = new TFTInfoBox(FLOW_CAL1_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incCal1Button = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decCal2Button = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   cal2Box = new TFTInfoBox(FLOW_CAL2_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incCal2Button = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

/*
 // третья строка
   textBoxTopPos = thirdRowTopPos - textFontHeight - INFO_BOX_CONTENT_PADDING;
   topPos = thirdRowTopPos;
   leftPos = initialLeftPos;
   
   decLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;
   
   luxBox = new TFTInfoBox(WM_LIGHT_LUM_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
 
   incLuxButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);
 
   leftPos += INFO_BOX_V_SPACING*2 + TFT_ARROW_BUTTON_WIDTH;

   decHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, leftArrowCaption, BUTTON_SYMBOL);
   leftPos += INFO_BOX_V_SPACING + TFT_ARROW_BUTTON_WIDTH;

   histeresisBox = new TFTInfoBox(WM_LIGHT_HISTERESIS_CAPTION,TFT_TEXT_INPUT_WIDTH,textBoxHeightWithCaption,leftPos,textBoxTopPos,-(TFT_ARROW_BUTTON_WIDTH+INFO_BOX_V_SPACING));
   leftPos += INFO_BOX_V_SPACING + TFT_TEXT_INPUT_WIDTH;
   
   incHisteresisButton = screenButtons->addButton( leftPos ,  topPos, TFT_ARROW_BUTTON_WIDTH,  TFT_ARROW_BUTTON_HEIGHT, rightArrowCaption, BUTTON_SYMBOL);

  */
      
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::saveSettings()
{
 //сохраняем настройки калибровки
  String cmd = F("FLOW|T_SETT|");
  cmd += cal1;
  cmd += '|';
  cmd += cal2;
  
  ModuleInterop.QueryCommand(ctSET,cmd,false);
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::blinkSaveSettingsButton(bool bOn)
{
  
  if(bOn)
  {
    if(blinkActive != bOn)
    {
      blinkActive = bOn;
      blinkOn = true;
      blinkTimer = millis();
      screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
      screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
      screenButtons->drawButton(saveButton);
    }
  }
  else
  {
    blinkOn = false;
    blinkActive = false;
    screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
    screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
    screenButtons->drawButton(saveButton);    
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::incCal1(int val)
{
   int16_t oldW = cal1;
  cal1+=val;

  if(cal1 < 1)
    cal1 = 1;
    
  if(cal1 > 250)
    cal1 = 250;
  
  if(cal1 != oldW)
    drawValueInBox(cal1Box,cal1);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::incCal2(int val)
{
   int16_t oldW = cal2;
  cal2+=val;

  if(cal2 < 1)
    cal2 = 1;
    
  if(cal2 > 250)
    cal2 = 250;
  
  if(cal2 != oldW)
    drawValueInBox(cal2Box,cal2);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(confirmMode) // в режиме подтверждения, не надо ничего обновлять
  {
    return;
  }

  if(wantShowOkMessage) // попросили показать окно результатов
  {
    wantShowOkMessage = false;
    Vector<const char*> lines;
    lines.push_back("ДАННЫЕ ДАТЧИКА РАСХОДА СБРОШЕНЫ.");
    MessageBox->show("СООБЩЕНИЕ",lines,this);
    
  }

  if(blinkActive && screenButtons)
  {
    if(millis() - blinkTimer > 500)
    {
      blinkOn = !blinkOn;

      if(blinkOn)
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_ON_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_ON_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);
      }
      else
      {
        screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
        screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR);
        screenButtons->drawButton(saveButton);   
      }

      blinkTimer = millis();
    }
  } // if(blinkActive)  
 
 if(screenButtons)
 {
    int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

    if(pressed_button != -1)
    {
         menuManager->resetIdleTimer();

        if(pressed_button == backButton)
        {
          menuManager->switchToScreen("DRIVE");
          return;
        }
        else
        if(pressed_button == saveButton)
        {
          saveSettings();
          blinkSaveSettingsButton(false);
          screenButtons->disableButton(saveButton,true);
          return;
        }
        else if(pressed_button == decCal1Button)
        {
          incCal1(-1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == incCal1Button)
        {
          incCal1(1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == decCal2Button)
        {
          incCal2(-1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else if(pressed_button == incCal2Button)
        {
          incCal2(1);
          screenButtons->enableButton(saveButton,!screenButtons->buttonEnabled(saveButton));
          blinkSaveSettingsButton(true);
        }
        else
        if(pressed_button == resetFlow1Button)
        {
          confirm(0);
          return;
        }
        else
        if(pressed_button == resetFlow2Button)
        {
          confirm(1);
          return;
        }
        
    } // if(pressed_button != -1)
     
 } // if(screenButtons)
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::confirm(uint8_t num)
{
  flowToReset = num;
  confirmMode = true;
  messageBoxOkPressed = false;
  
  Vector<const char*> lines;
  lines.push_back("ВЫ УВЕРЕНЫ, ЧТО ХОТИТЕ ОБНУЛИТЬ ПОКАЗАНИЯ?");
  MessageBox->confirm("ПОДТВЕРЖДЕНИЕ",lines,this,this,this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  if(confirmMode)
  {    
    // ждали подтверждения
    if(messageBoxOkPressed)
    {
      String command = F("FLOW|ZERO|");
      command += flowToReset;      
      // тут обнуление показаний выбранного датчика
      ModuleInterop.QueryCommand(ctSET,command,false);
    }
    else
    {
      confirmMode = false;
    }
  }

  screenButtons->disableButton(saveButton);

  blinkActive = false;
  screenButtons->setButtonFontColor(saveButton,WM_BLINK_OFF_TEXT_COLOR);
  screenButtons->setButtonBackColor(saveButton,WM_OFF_BLINK_BGCOLOR); 
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::onMessageBoxResult(bool okPressed)
{
  if(confirmMode)
    messageBoxOkPressed = okPressed;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTFlowScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  if(confirmMode)
  {    
    confirmMode = false;
    if(messageBoxOkPressed)
    {
       // обнулили, можно показать сообщение
       wantShowOkMessage = true;
    }
    return;
  }

  if(screenButtons)
  {
    screenButtons->drawButtons(drawButtonsYield);
  }

  cal1Box->draw(menuManager);
  drawValueInBox(cal1Box,cal1); 

  cal2Box->draw(menuManager);
  drawValueInBox(cal2Box,cal2); 

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERFLOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTIdleScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTIdleScreen::TFTIdleScreen() : AbstractTFTScreen()
{

  currentScreen = 0;
  #ifdef USE_DS3231_REALTIME_CLOCK
      lastMinute = -1;
  #endif  

  #ifdef USE_SMS_MODULE
    gsmSignalQuality = 0; // нет сигнала
    gprsAvailable = false;
    providerNamePrinted = false;
    providerNameLength = 0;
  #endif

  #ifdef USE_WIFI_MODULE
    connectedToRouter = false;
    wifiSignalQuality = 0;
  #endif

  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
  unrTimer = 0;
  rCopy = false;
  #endif

  #ifdef USE_LORA_GATE
    lastLoraRSSI = -120;
  #endif


  // USE_WATER_TANK_MODULE
  fillTankButton = 0xFF;
  // USE_WATER_TANK_MODULE

  drawCalled = false;
  uptimeMinutes = 0;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTIdleScreen::~TFTIdleScreen()
{
  delete screenButtons;
  
  delete windowStatusBox;
  delete waterStatusBox;
  delete lightStatusBox;
  delete heatBox1;
  delete heatBox2;
  delete heatBox3;

  delete heatBox1Sensor;
  delete heatBox1Air;

  delete heatBox2Sensor;
  delete heatBox2Air;

  delete heatBox3Sensor;
  delete heatBox3Air;

  delete flow1Box;
  delete flow2Box;

  delete windSpeedBox;
  delete windDirectionBox;
  delete rainStatusBox;

  delete phFlowMixBox;
  delete phPlusMinusBox;

  // USE_WATER_TANK_MODULE
  delete waterTankStatusBox;
  delete waterTankCommandsBox;
  // USE_WATER_TANK_MODULE

  for(int i=0;i<6;i++)
  {
    delete sensors[i].box;
  }


  for(int i=0;i<6;i++)
  {
    delete sensors2[i].box;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawWindowStatus(TFTMenu* menuManager)
{
  #ifdef USE_TEMP_SENSORS
    drawStatusesInBox(menuManager, windowStatusBox, flags.isWindowsOpen, flags.windowsAutoMode, TFT_WINDOWS_OPEN_CAPTION, TFT_WINDOWS_CLOSED_CAPTION, TFT_AUTO_MODE_CAPTION, TFT_MANUAL_MODE_CAPTION);
  #else
    drawValueInBox(windowStatusBox,UNAVAIL_FEATURE,BigRusFont);
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawWaterStatus(TFTMenu* menuManager)
{
  #ifdef USE_WATERING_MODULE
    drawStatusesInBox(menuManager, waterStatusBox, flags.isWaterOn, flags.waterAutoMode, TFT_WATER_ON_CAPTION, TFT_WATER_OFF_CAPTION, TFT_AUTO_MODE_CAPTION, TFT_MANUAL_MODE_CAPTION);
  #else
    drawValueInBox(waterStatusBox,UNAVAIL_FEATURE,BigRusFont);
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawLightStatus(TFTMenu* menuManager)
{
  #ifdef USE_LUMINOSITY_MODULE
    drawStatusesInBox(menuManager, lightStatusBox, flags.isLightOn, flags.lightAutoMode, TFT_LIGHT_ON_CAPTION, TFT_LIGHT_OFF_CAPTION, TFT_AUTO_MODE_CAPTION, TFT_MANUAL_MODE_CAPTION);  
  #else
    drawValueInBox(lightStatusBox,UNAVAIL_FEATURE,BigRusFont);
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawHeatStatus(TFTMenu* menuManager, uint8_t _heatSectionIndex, TFTInfoBox* _box, bool _active, bool _on)
{
  #ifdef USE_HEAT_MODULE 
    drawStatusesInBox(menuManager, _box,_on, _active, HEAT_INFO_BOX_CONTOUR_ON, HEAT_INFO_BOX_CONTOUR_OFF, HEAT_INFO_BOX_MODULE_ON, HEAT_INFO_BOX_MODULE_OFF, HEAT_INFO_BOX_CONTOUR, HEAT_INFO_BOX_MODULE);  
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawHeatStatus(TFTMenu* menuManager)
{
  #ifdef USE_HEAT_MODULE
    drawHeatStatus(menuManager,0,heatBox1, heat1Active, heat1On);
    drawHeatStatus(menuManager,1,heatBox2, heat2Active, heat2On);
    drawHeatStatus(menuManager,2,heatBox3, heat3Active, heat3On);
  #else
    drawValueInBox(heatBox1,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(heatBox2,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(heatBox3,UNAVAIL_FEATURE,BigRusFont);
  #endif // USE_HEAT_MODULE


}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawPHStatus(TFTMenu* menuManager, uint8_t whichSection, TFTInfoBox* _box, bool on1, bool on2)
{
  #ifdef USE_PH_MODULE 
    if(whichSection == 1) // flow add, mix on
    {
      drawStatusesInBox(menuManager, _box,on1, on2, "ВКЛ", "ВЫКЛ", "ВКЛ", "ВЫКЛ", "Подача воды:", "Перемешивание:");  
    }
    else // plus on, minus on
    {
      drawStatusesInBox(menuManager, _box,on1, on2, "ВКЛ", "ВЫКЛ", "ВКЛ", "ВЫКЛ", "Повышение pH:", "Понижение pH:");        
    }
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawPHStatus(TFTMenu* menuManager)
{

  phFlowMixBox->draw(menuManager);
  phPlusMinusBox->draw(menuManager);
  
  #ifdef USE_PH_MODULE
    drawPHStatus(menuManager,1,phFlowMixBox, phFlowAddOn, phMixOn);
    drawPHStatus(menuManager,2,phPlusMinusBox, phPlusOn, phMinusOn);
  #else
    drawValueInBox(phFlowMixBox,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(phPlusMinusBox,UNAVAIL_FEATURE,BigRusFont);
  #endif // USE_PH_MODULE
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// USE_WATER_TANK_MODULE
void TFTIdleScreen::drawWaterTankStatus(TFTMenu* menuManager)
{

  //Serial.println("drawWaterTankStatus");
  
  waterTankStatusBox->draw(menuManager);
  waterTankCommandsBox->draw(menuManager);

  
  #ifdef USE_WATER_TANK_MODULE

  #ifndef USE_LORA_GATE
     drawValueInBox(waterTankCommandsBox,UNAVAIL_FEATURE,BigRusFont); 
  #endif 

  wTankFillStatus =  WaterTank->GetFillStatus();
  wTankErrorType = WaterTank->GetErrorType();
  wTankIsValveOn = WaterTank->IsValveOn();

    // тут рисуем статус в первом боксе
    String fStatus;
    int waterDropCount = wTankFillStatus;
    fStatus = wTankFillStatus;
    //fStatus += "%";

    if (wTankFillStatus == 100) 
    {
      fStatus += "% ";
    }
    else 
    {
      fStatus += "%  ";
    }
    
    for (uint8_t i=0; i<=4;i++) 
    {
        if(waterDropCount > 0) 
        {
          fStatus += "*";
          waterDropCount -= 20;
        }
        else 
        {
          fStatus += "-";
        }  
    }
        

    String errText = WaterTank->GetErrorText();
    wTankHasErrors = WaterTank->HasErrors();
    
    drawStatusesInBox(menuManager, waterTankStatusBox,!wTankHasErrors, !wTankHasErrors, fStatus.c_str(), fStatus.c_str(), errText.c_str(), errText.c_str(), "Заполнение:", "Статус:");

    

    // TODO: ТУТ ОТРИСОВКА КНОПКИ ДЛЯ НАПОЛНЕНИЯ БАКА !!!
    if(fillTankButton != 0xFF)
    {
      if(wTankIsValveOn)
      {
        screenButtons->relabelButton(fillTankButton,"НАПОЛНЯЕТСЯ...");        
      }
      else
      {
        screenButtons->relabelButton(fillTankButton,"НАПОЛНИТЬ БАК");        
      }

      if(!WaterTank->IsModuleOnline())
      {
        screenButtons->disableButton(fillTankButton);
      }
      else
      {
        screenButtons->enableButton(fillTankButton);
      }

      //Serial.println("draw fill tank button 3");
      screenButtons->drawButton(fillTankButton);
    }
    else // кнопки нет, рисуем заглушку
    {
      drawValueInBox(waterTankCommandsBox,UNAVAIL_FEATURE,BigRusFont); 
    }
  #else
    drawValueInBox(waterTankStatusBox,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(waterTankCommandsBox,UNAVAIL_FEATURE,BigRusFont);
  #endif // USE_WATER_TANK_MODULE
}
// USE_WATER_TANK_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawFlowData(TFTMenu* menuManager)
{

  flow1Box->draw(menuManager);
  flow2Box->draw(menuManager);
  windSpeedBox->draw(menuManager);
  windDirectionBox->draw(menuManager);
  rainStatusBox->draw(menuManager);
  
  #ifdef USE_WATERFLOW_MODULE  
    drawFlowData(menuManager,flow1Box,flow1Data);
    drawFlowData(menuManager,flow2Box,flow2Data);
  #else // !USE_WATERFLOW_MODULE
    drawValueInBox(flow1Box,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(flow2Box,UNAVAIL_FEATURE,BigRusFont);
  #endif // USE_WATERFLOW_MODULE

  #ifdef USE_WINDOW_MANAGE_MODULE
    String val,fract;
    
    val = windSpeed/100;

    int vf = windSpeed%100;
    if(vf < 10)
      fract = '0';
    fract += vf; // у нас всё хранится в сотых долях

    drawSensorData(menuManager,windSpeedBox,charWindSpeed,true,false,val,fract);
    
    TFTSpecialSimbol unitChar = charUnknown;
    val = fract = "";
    
    switch(windDirection)
    {
      case cpEast: { unitChar = charEast; } break;
      case cpWest: { unitChar = charWest; } break;
      case cpSouth:{ unitChar = charSouth; } break;
      case cpNorth: { unitChar = charNorth; } break;
      case cpUnknown: unitChar = charUnknown; break;
    }
    drawSensorData(menuManager,windDirectionBox,unitChar,windDirection != cpUnknown, false,val,fract);

    val = fract = "";
    if(hasRain)
      val = RAIN_BOX_YES;
    else
      val = RAIN_BOX_NO;

    drawValueInBox(rainStatusBox,val,BigRusFont);    
    
  #else // !USE_WINDOW_MANAGE_MODULE
  
    drawValueInBox(windSpeedBox,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(windDirectionBox,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(rainStatusBox,UNAVAIL_FEATURE,BigRusFont);
    
  #endif // USE_WINDOW_MANAGE_MODULE

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawHeatSensorData(TFTMenu* menuManager, TFTInfoBox* box, Temperature& t)
{
  String val,fract;
  if(t.HasData())
  {
    val = abs(t.Value);
    
    if(t.Fract < 10)
      fract = '0';

    fract += t.Fract;
  }
  drawSensorData(menuManager,box,charDegree,t.HasData(),t.Value < 0,val,fract);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawHeatSensorsData(TFTMenu* menuManager)
{
  #ifdef USE_HEAT_MODULE

      drawHeatSensorData(menuManager,heatBox1Sensor,heatBox1SensorData);
      drawHeatSensorData(menuManager,heatBox1Air,heatBox1AirData);

      drawHeatSensorData(menuManager,heatBox2Sensor,heatBox2SensorData);
      drawHeatSensorData(menuManager,heatBox2Air,heatBox2AirData);

      drawHeatSensorData(menuManager,heatBox3Sensor,heatBox3SensorData);
      drawHeatSensorData(menuManager,heatBox3Air,heatBox3AirData);
      
  #else
    drawValueInBox(heatBox1Sensor,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(heatBox1Air,UNAVAIL_FEATURE,BigRusFont);
    
    drawValueInBox(heatBox2Sensor,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(heatBox2Air,UNAVAIL_FEATURE,BigRusFont);
    
    drawValueInBox(heatBox3Sensor,UNAVAIL_FEATURE,BigRusFont);
    drawValueInBox(heatBox3Air,UNAVAIL_FEATURE,BigRusFont);
    
  #endif // USE_HEAT_MODULE
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawHeatSensors(TFTMenu* menuManager)
{

  heatBox1Sensor->draw(menuManager);
  heatBox1Air->draw(menuManager);

  heatBox2Sensor->draw(menuManager);
  heatBox2Air->draw(menuManager);

  heatBox3Sensor->draw(menuManager);
  heatBox3Air->draw(menuManager);

  drawHeatSensorsData(menuManager);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawFlowData(TFTMenu* menuManager, TFTInfoBox* box, unsigned long fl)
{
  TFTSpecialSimbol unitChar = charLitres;
  String val, fract;
  if(fl < 1000ul)
  {
    val = fl;
  }
  else if(fl < 100000ul)
  {
    unitChar = charM3;
    val = fl/1000;
    unsigned long modVal = fl%1000;
    modVal /= 10;

    if(modVal < 10)
      fract = '0';

    fract += modVal;
  }
  else
  {
    // больше 100 кубометров, показываем целыми
    unitChar = charM3;
    val = fl/1000;
  }
  drawSensorData(menuManager,box,unitChar,true,false,val,fract); 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
unsigned long TFTIdleScreen::GetFlowData(uint8_t idx)
{
  #ifdef USE_WATERFLOW_MODULE
    AbstractModule* mod = MainController->GetModuleByID("FLOW");
    if(!mod)
      return 0;

    OneState* os = mod->State.GetState(StateWaterFlowIncremental,idx);
    if(!os)
      return 0;

      WaterFlowPair wp = *os;
      return wp.Current;
      
  #else // !USE_WATERFLOW_MODULE
    return 0;
  #endif // USE_WATERFLOW_MODULE
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateFlowData(TFTMenu* menuManager)
{
  #if defined(USE_WATERFLOW_MODULE) || defined(USE_WINDOW_MANAGE_MODULE)
  static uint32_t tmr = millis();

  if(millis() - tmr > 3875)
  {
    #ifdef USE_WATERFLOW_MODULE
      unsigned long thisFlow1Data = GetFlowData(0);
      unsigned long thisFlow2Data = GetFlowData(1);

      if(flow1Data != thisFlow1Data)
      {
        flow1Data = thisFlow1Data;
        if(currentScreen == TFT_IDLE_FLOW_SCREEN_NUMBER)
          drawFlowData(menuManager,flow1Box,flow1Data);
      }
      
      if(flow2Data != thisFlow2Data)
      {
        flow2Data = thisFlow2Data;
        if(currentScreen == TFT_IDLE_FLOW_SCREEN_NUMBER)
          drawFlowData(menuManager,flow2Box,flow2Data);
      }
      
    #endif // USE_WATERFLOW_MODULE

    #ifdef USE_WINDOW_MANAGE_MODULE
    
    uint32_t thisWindSpeed = LogicManageModule->GetWindSpeed();
    if(thisWindSpeed > 30000) // явно что-то не так
      thisWindSpeed = 0;
    CompassPoints thisDirection = LogicManageModule->GetWindDirection();

    bool thisHasRain = LogicManageModule->HasRain();
    String val, fract;

    if(thisWindSpeed != windSpeed)
    {
      windSpeed = thisWindSpeed;
      
      val = windSpeed/100;
      
      int vf = windSpeed%100;
      if(vf < 10)
        fract = '0';
      fract += vf; // у нас всё хранится в сотых долях
      
      if(currentScreen == TFT_IDLE_FLOW_SCREEN_NUMBER)
        drawSensorData(menuManager,windSpeedBox,charWindSpeed,true,false,val,fract);
    }


    if(windDirection != thisDirection)
    {
      windDirection = thisDirection;
      if(currentScreen == TFT_IDLE_FLOW_SCREEN_NUMBER)
      {
          TFTSpecialSimbol unitChar = charUnknown;
          val = fract = "";
          switch(windDirection)
          {
            case cpEast: { unitChar = charEast; } break;
            case cpWest: { unitChar = charWest; } break;
            case cpSouth:{ unitChar = charSouth; } break;
            case cpNorth: { unitChar = charNorth; } break;
            case cpUnknown: unitChar = charUnknown; break;
          }
          drawSensorData(menuManager,windDirectionBox,unitChar,windDirection != cpUnknown, false,val,fract);
      }
    }

    if(thisHasRain != hasRain)
    {
      hasRain = thisHasRain;
      val = fract = "";
      if(hasRain)
        val = RAIN_BOX_YES;
      else
        val = RAIN_BOX_NO;
  
        drawValueInBox(rainStatusBox,val,BigRusFont); 
      
    }
    #endif // USE_WINDOW_MANAGE_MODULE

    tmr = millis();
  }
  
  #endif // USE_WINDOW_MANAGE_MODULE
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateHeatSensors(TFTMenu* menuManager)
{
  #ifdef USE_HEAT_MODULE
    static uint32_t heatSensorsTimer = millis();
    if(millis() - heatSensorsTimer > 1000)
    {
      Temperature tSensor, tAir;
      
      LogicManageModule->GetHeatTemperature(0,tSensor,tAir);

      if(tSensor != heatBox1SensorData)
      {
        heatBox1SensorData = tSensor;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox1Sensor,heatBox1SensorData);
      }
      if(tAir != heatBox1AirData)
      {
        heatBox1AirData = tAir;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox1Air,heatBox1AirData);
      }

      LogicManageModule->GetHeatTemperature(1,tSensor,tAir);

      if(tSensor != heatBox2SensorData)
      {
        heatBox2SensorData = tSensor;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox2Sensor,heatBox2SensorData);
      }
      if(tAir != heatBox2AirData)
      {
        heatBox2AirData = tAir;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox2Air,heatBox2AirData);
      }


      LogicManageModule->GetHeatTemperature(2,tSensor,tAir);

      if(tSensor != heatBox3SensorData)
      {
        heatBox3SensorData = tSensor;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox3Sensor,heatBox3SensorData);
      }
      if(tAir != heatBox3AirData)
      {
        heatBox3AirData = tAir;
        if(currentScreen == TFT_IDLE_HEAT_SCREEN_NUMBER)
          drawHeatSensorData(menuManager,heatBox3Air,heatBox1AirData);
      }

      
      heatSensorsTimer = millis();
    }
  #endif
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DS3231_REALTIME_CLOCK
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::DrawDateTime(TFTMenu* menuManager)
{
    RealtimeClock rtc = MainController->GetClock();
    RTCTime tm = rtc.getTime();

  if(lastMinute == tm.minute)
    return;

    static char dt_buff[20] = {0};
    sprintf_P(dt_buff,(const char*) F("%02d.%02d.%d %02d:%02d"), tm.dayOfMonth, tm.month, tm.year, tm.hour, tm.minute);

    lastMinute = tm.minute;

    UTFT* dc = menuManager->getDC();
    dc->setFont(BigRusFont);
    dc->setBackColor(TFT_BACK_COLOR);
    dc->setColor(INFO_BOX_CAPTION_COLOR); 

    int screenWidth = dc->getDisplayXSize();
    int screenHeight = dc->getDisplayYSize();
    int fontWidth = dc->getFontXsize();
    int fontHeight = dc->getFontYsize();
    int textLen = menuManager->getRusPrinter()->utf8_strlen(dt_buff);

    int left = 20;//(screenWidth - (textLen*fontWidth))/2;
    int top = screenHeight - (fontHeight + 4);
    dc->print(dt_buff,left,top);
    yield();
      
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_DS3231_REALTIME_CLOCK
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawUptimeMinutes(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  dc->setFont(BigRusFont);
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR); 

  uint32_t minutes = uptimeMinutes;
  uint32_t hours = minutes/60;
  minutes -= hours*60;
  uint32_t days = hours/24;
  hours -= days*24;
  
  static char dt_buff[30] = {0};
  sprintf_P(dt_buff,(const char*) F("%02dд:%02dч:%02dм"), days, hours, minutes);


  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();  

  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();
  
  int textLen = menuManager->getRusPrinter()->utf8_strlen(dt_buff);

  int left = (screenWidth - (textLen*fontWidth)) - 20;
  int top = screenHeight - (fontHeight + 4);
  menuManager->getRusPrinter()->print(dt_buff,left,top);
  yield();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawSensorData(TFTMenu* menuManager,TFTInfoBox* box, int dataIndex, bool forceDraw)
{
  
  if(!box) // нет переданного бокса для отрисовки
  {
    return;
  }

  TFTSensorBinding bnd = HardwareBinding->GetTFTSensorBinding(dataIndex);
  ModuleStates sensorType = StateUnknown;
  const char* moduleName = "_";

  switch(bnd.Type)
  {
    case tftSensor_NoModule:
    {
      
    }
    break;
    
    case tftSensorModule_State_Temperature:
    {
      moduleName = "STATE";
      sensorType = StateTemperature;
    }
    break;

    case tftSensorModule_Humidity_Temperature:
    {
      moduleName = "HUMIDITY";
      sensorType = StateTemperature;      
    }
    break;

    case tftSensorModule_Humidity_Humidity:
    {
      moduleName = "HUMIDITY";
      sensorType = StateHumidity;            
    }
    break;

    case tftSensorModule_Luminosity:
    {
      moduleName = "LIGHT";
      sensorType = StateLuminosity;                  
    }
    break;

    case tftSensorModule_CO2:
    {
      moduleName = "CO2";
      sensorType = StateCO2;                        
    }
    break;

    case tftSensorModule_EC:
    {
      moduleName = "EC";
      sensorType = StateEC;                        
    }
    break;

    case tftSensorModule_Soil:
    {
      moduleName = "SOIL";
      sensorType = StateSoilMoisture;                              
    }
    break;

    case tftSensorModule_PH:
    {
      moduleName = "PH";
      sensorType = StatePH;                                    
    }
    break;
  } // switch

  // тут сами данные с датчика, в двух частях
  String sensorValue;
  String sensorFract;

  //Тут получение OneState для датчика, и выход, если не получили
  AbstractModule* module = MainController->GetModuleByID(moduleName);
  if(!module)
  {
    if(forceDraw)
    {
      drawSensorData(menuManager,box,charUnknown,false,false,sensorValue,sensorFract);
    }
    return;
  }
    
  OneState* sensorState = module->State.GetState(sensorType,bnd.Index);
  if(!sensorState)
  {
    if(forceDraw)
    {
      drawSensorData(menuManager,box,charUnknown,false,false,sensorValue,sensorFract);
    }
    return;
  }

  //Тут проверка на то, что данные с датчика изменились. И если не изменились - не надо рисовать, только если forceDraw не true.
  bool sensorDataChanged = sensorState->IsChanged();

  if(!sensorDataChanged && !forceDraw)
  {
    // ничего не надо перерисовывать, выходим
    return;
  }

  //Если данных с датчика нет - выводим два минуса, не забыв поменять шрифт на  SensorFont.
  bool hasSensorData = sensorState->HasData();
  bool minusVisible = false;
  TFTSpecialSimbol unitChar = charUnknown;

  if(hasSensorData)
  {
      switch(sensorType)
      {
        case StateTemperature:
        {
        
          unitChar = charDegree;
          
          //Тут получение данных с датчика
          TemperaturePair tp = *sensorState;
          
          sensorValue = (byte) abs(tp.Current.Value);
          
          int fract = tp.Current.Fract;
          if(fract < 10)
            sensorFract += '0';
            
          sensorFract += fract;
         
          //Тут проверяем на отрицательную температуру
          minusVisible = tp.Current.Value < 0;    
        }
        break;
    
        case StateHumidity:
        case StateSoilMoisture:
        {
          unitChar = charPercent;

          //Тут получение данных с датчика

          HumidityPair hp = *sensorState;
          sensorValue = hp.Current.Value;
          
          int fract = hp.Current.Fract;
          if(fract < 10)
            sensorFract += '0';
            
          sensorFract += fract;
        }
        break;
    
        case StateLuminosity:
        {
          unitChar = charLux;
        
          //Тут получение данных с датчика
          LuminosityPair lp = *sensorState;
          long lum = lp.Current;
          sensorValue = lum;
        }
        break;

        case StateCO2:
        {
          unitChar = charCO2;
        
          //Тут получение данных с датчика
          CO2Pair lp = *sensorState;
          uint16_t lum = lp.Current;
          sensorValue = lum;
        }
        break;

        case StateEC:
        {
          unitChar = charCO2;
        
          //Тут получение данных с датчика
          ECPair lp = *sensorState;
          uint16_t lum = lp.Current;
          sensorValue = lum;
        }
        break;
    
        case StateWaterFlowInstant:
        case StateWaterFlowIncremental:
        {
          //Тут получение данных с датчика
          WaterFlowPair wp = *sensorState;
          unsigned long flow = wp.Current;
          sensorValue = flow;
        }
        break;
        
        case StatePH:
        {
          //Тут получение данных с датчика
          HumidityPair ph = *sensorState;
          sensorValue = (byte) ph.Current.Value;
          int fract = ph.Current.Fract;
          if(fract < 10)
            sensorFract += '0';
            
          sensorFract += fract;
        } 
        break;
        
        case StateUnknown:
        break;
        
      } // switch
    
  } // hasSensorData
  else
  {
    minusVisible = false;
    unitChar = charUnknown;
  }

  drawSensorData(menuManager,box,unitChar,hasSensorData,minusVisible,sensorValue,sensorFract);

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawSensorData(TFTMenu* menuManager,TFTInfoBox* box, TFTSpecialSimbol unitChar, bool hasData, bool negative, String& value, const String& fract)
{

  TFTInfoBoxContentRect rc = box->getContentRect(menuManager);
  UTFT* dc = menuManager->getDC();
  UTFTRus* rusPrinter = menuManager->getRusPrinter();

   if(!hasData)
   {
    value = rusPrinter->mapChar(charMinus);
    value += rusPrinter->mapChar(charMinus);
   }

  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRect(rc.x,rc.y,rc.x+rc.w,rc.y+rc.h);
  yield();
  
  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);  

  dc->setFont(SensorFont);
  
  bool dotAvailable = fract.length() > 0;
  
  // у нас длина строки будет - длина показаний + точка + единицы измерения + опциональный минус
  int totalStringLength =  dotAvailable ? 1 : 0; // place for dot
  
  if(hasData && negative) // минус у нас просчитывается, только если есть показания с датчика
    totalStringLength++;

   if(unitChar != charUnknown && hasData) // единицы измерения у нас просчитываются, только если есть показания с датчика
    totalStringLength++; // есть единицы измерения

  int curTop = rc.y + (rc.h - dc->getFontYsize())/2;
  int fontWidth = dc->getFontXsize();

  int valueLen = value.length();
  int fractLen = fract.length();

  totalStringLength += valueLen + fractLen;

  // рисуем данные с датчика
  int curLeft = rc.x + (rc.w - totalStringLength*fontWidth)/2;

  if(hasData && negative)
  {
    rusPrinter->printSpecialChar(charMinus,curLeft,curTop);
    curLeft += fontWidth;
  }

  // теперь рисуем целое значение
  if(hasData)
   dc->setFont(SevenSegNumFontMDS); // есть данные, рисуем числа    
  else
   dc->setFont(SensorFont); // нет данных, рисуем два минуса
  
  dc->print(value.c_str(),curLeft,curTop);
  yield();
  curLeft += fontWidth*valueLen;

  if(hasData)
  {
      // теперь рисуем запятую, если надо
      // признаком служит доступность разделителя к рисованию
      // также не надо рисовать, если длина дробной части - 0
      if(dotAvailable)
      {
          dc->setFont(SensorFont);
          rusPrinter->printSpecialChar(TFT_SENSOR_DECIMAL_SEPARATOR,curLeft,curTop);
          curLeft += fontWidth;
      
          // теперь рисуем дробную часть
          dc->setFont(SevenSegNumFontMDS);
          dc->print(fract.c_str(),curLeft,curTop);
          yield();
          curLeft += fontWidth*fractLen;
    
      } // if(dotAvailable)
      
  } // if(hasData)

 

  if(unitChar != charUnknown && hasData) // если надо рисовать единицы измерений - рисуем
  {
    // теперь рисуем единицы измерения
    if(unitChar >= charLitres)
      dc->setFont(SensorFont2);  
    else
      dc->setFont(SensorFont);
        
    dc->setColor(SENSOR_BOX_UNIT_COLOR);
    rusPrinter->printSpecialChar(unitChar,curLeft,curTop);
  }


  // сбрасываем на шрифт по умолчанию
  dc->setFont(BigRusFont);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawStatusesInBox(TFTMenu* menuManager,TFTInfoBox* box, bool status, bool mode, const char* onStatusString, const char* offStatusString, const char* autoModeString, const char* manualModeString, const char* status1Caption, const char* status2Caption)
{
  TFTInfoBoxContentRect rc = box->getContentRect(menuManager);
  UTFT* dc = menuManager->getDC();

  dc->setColor(INFO_BOX_BACK_COLOR);
  dc->fillRect(rc.x,rc.y,rc.x+rc.w,rc.y+rc.h);
  yield();

  dc->setBackColor(INFO_BOX_BACK_COLOR);
  dc->setColor(SENSOR_BOX_FONT_COLOR);

  const int offset = 10;
  int curTop = rc.y + offset;
  int curLeft = rc.x;
  int fontHeight = dc->getFontYsize();
  int fontWidth = dc->getFontXsize();

  // рисуем заголовки режимов
  menuManager->getRusPrinter()->print(status1Caption,curLeft,curTop);
  curTop += fontHeight + INFO_BOX_CONTENT_PADDING + offset;
  menuManager->getRusPrinter()->print(status2Caption,curLeft,curTop);

  // теперь рисуем статусы режимов

  curTop = rc.y + offset;
  const char* toDraw;
  
  if(status)
  {
    dc->setColor(STATUS_ON_COLOR);
    toDraw = onStatusString;
  }
  else
  {
    dc->setColor(SENSOR_BOX_FONT_COLOR);
    toDraw = offStatusString;
  }

  int captionLen = menuManager->getRusPrinter()->utf8_strlen(toDraw);
  
  curLeft = (rc.x + rc.w) - (captionLen*fontWidth);
  menuManager->getRusPrinter()->print(toDraw,curLeft,curTop);

  curTop += fontHeight + INFO_BOX_CONTENT_PADDING + offset;

  if(mode)
  {
    dc->setColor(STATUS_ON_COLOR);
    toDraw = autoModeString;
  }
  else
  {
    dc->setColor(TFT_MANUAL_MODE_CAPTION_COLOR); // ручной режим будет отрисован другим цветом = КМВ
    toDraw = manualModeString;
  }

  captionLen = menuManager->getRusPrinter()->utf8_strlen(toDraw);
  
  curLeft = (rc.x + rc.w) - (captionLen*fontWidth);
  menuManager->getRusPrinter()->print(toDraw,curLeft,curTop);

  yield();
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateStatuses(TFTMenu* menuManager)
{
  
  bool wOpen = WORK_STATUS.GetStatus(WINDOWS_STATUS_BIT);
  bool wAutoMode = WORK_STATUS.GetStatus(WINDOWS_MODE_BIT);

  bool waterOn = WORK_STATUS.GetStatus(WATER_STATUS_BIT);
  bool waterAutoMode = WORK_STATUS.GetStatus(WATER_MODE_BIT);

  bool lightOn = WORK_STATUS.GetStatus(LIGHT_STATUS_BIT);
  bool lightAutoMode = WORK_STATUS.GetStatus(LIGHT_MODE_BIT);

  bool windowChanges = flags.isWindowsOpen != wOpen || flags.windowsAutoMode != wAutoMode;
  bool waterChanges = flags.isWaterOn != waterOn || flags.waterAutoMode != waterAutoMode;
  bool lightChanges = flags.isLightOn != lightOn || flags.lightAutoMode != lightAutoMode;

  flags.isWindowsOpen = wOpen;
  flags.windowsAutoMode = wAutoMode;
  
  flags.isWaterOn = waterOn;
  flags.waterAutoMode = waterAutoMode;
  
  flags.isLightOn = lightOn;
  flags.lightAutoMode = lightAutoMode;

  #ifdef USE_TEMP_SENSORS
    if(windowChanges && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
    {
      drawWindowStatus(menuManager);
    }
  #endif

  #ifdef USE_WATERING_MODULE
    if(waterChanges && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
    {
      drawWaterStatus(menuManager);
    }
  #endif

  #ifdef USE_LUMINOSITY_MODULE
    if(lightChanges && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
    {
      drawLightStatus(menuManager);
    }
  #endif

  #ifdef USE_HEAT_MODULE 

    if(millis() - heatTimer > HEAT_STATUS_UPDATE_INTERVAL)
    {
      bool nowHeat1Active, nowHeat1On;
      bool nowHeat2Active, nowHeat2On;
      bool nowHeat3Active, nowHeat3On;
      
      LogicManageModule->GetHeatStatus(0,nowHeat1Active,nowHeat1On);
      LogicManageModule->GetHeatStatus(1,nowHeat2Active,nowHeat2On);
      LogicManageModule->GetHeatStatus(2,nowHeat3Active,nowHeat3On);

      if(nowHeat1Active != heat1Active || nowHeat1On != heat1On)
      {
        heat1Active = nowHeat1Active;
        heat1On = nowHeat1On;

        if(drawCalled && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
          drawHeatStatus(menuManager,0, heatBox1, heat1Active, heat1On);
      }
  
      if(nowHeat2Active != heat2Active || nowHeat2On != heat2On)
      {
        heat2Active = nowHeat2Active;
        heat2On = nowHeat2On;

        if(drawCalled && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
          drawHeatStatus(menuManager,0, heatBox2, heat2Active, heat2On);
      }

      if(nowHeat3Active != heat3Active || nowHeat3On != heat3On)
      {
        heat3Active = nowHeat3Active;
        heat3On = nowHeat3On;

        if(drawCalled && currentScreen == TFT_IDLE_STATUSES_SCREEN_NUMBER)
          drawHeatStatus(menuManager,0, heatBox3, heat3Active, heat3On);
      }
      
  
      heatTimer = millis();
    }
  #endif   // USE_HEAT_MODULE

   #ifdef USE_PH_MODULE
      bool _phFlowAddOn = WORK_STATUS.GetStatus(PH_FLOW_ADD_BIT);
      bool _phMixOn = WORK_STATUS.GetStatus(PH_MIX_PUMP_BIT);

      bool flowMixChanges = _phFlowAddOn != phFlowAddOn || _phMixOn != phMixOn;
      if(flowMixChanges)
      {
        phFlowAddOn = _phFlowAddOn;
        phMixOn = _phMixOn;

        if(drawCalled && currentScreen == TFT_IDLE_PH_SCREEN_NUMBER)
          drawPHStatus(menuManager,1, phFlowMixBox, phFlowAddOn, phMixOn);
      }

      bool _phPlusOn = WORK_STATUS.GetStatus(PH_PLUS_PUMP_BIT);
      bool _phMinusOn = WORK_STATUS.GetStatus(PH_MINUS_PUMP_BIT);

      bool plusMinusChanges = _phPlusOn != phPlusOn || _phMinusOn != phMinusOn;
      if(plusMinusChanges)
      {
        phPlusOn = _phPlusOn;
        phMinusOn = _phMinusOn;

         if(drawCalled && currentScreen == TFT_IDLE_PH_SCREEN_NUMBER)
          drawPHStatus(menuManager,2, phPlusMinusBox, phPlusOn, phMinusOn);
      }
   #endif // USE_PH_MODULE


   #ifdef USE_WATER_TANK_MODULE
   
    //TODO: ТУТ ОБНОВЛЯЕМ СТАТУС МОДУЛЯ ЗАПОЛНЕНИЯ ТАНКА С ВОДОЙ !!!
    uint8_t _wTankErrorType = WaterTank->GetErrorType();
    uint8_t _wTankHasErrors = WaterTank->HasErrors();
    uint8_t _wTankFillStatus = WaterTank->GetFillStatus();
    bool _wTankIsValveOn = WaterTank->IsValveOn();

    if(wTankErrorType != _wTankErrorType || wTankHasErrors != _wTankHasErrors || wTankFillStatus != _wTankFillStatus || wTankIsValveOn != _wTankIsValveOn)
    {
      
         if(currentScreen == TFT_IDLE_PH_SCREEN_NUMBER)
         {          
          drawWaterTankStatus(menuManager);
         }
      
    }
    
   #endif // USE_WATER_TANK_MODULE
    
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  UTFTRus* rusPrinter = menuManager->getRusPrinter();

  screenButtons = new UTFT_Buttons_Rus(menuManager->getDC(), menuManager->getTouch(),rusPrinter);
  screenButtons->setTextFont(BigRusFont);
  screenButtons->setSymbolFont(SensorFont);
  screenButtons->setIconFont(IconsFont);
  screenButtons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);


  static char leftArrowCaption[2] = {0};
  static char rightArrowCaption[2] = {0};

  leftArrowCaption[0] = rusPrinter->mapChar(charLeftArrow);
  rightArrowCaption[0] = rusPrinter->mapChar(charRightArrow);

  int buttonsTop = menuManager->getDC()->getDisplayYSize() - TFT_IDLE_SCREEN_BUTTON_HEIGHT - 30; // координата Y для кнопок стартового экрана
  int screenWidth = menuManager->getDC()->getDisplayXSize();

  // вычисляем, сколько кнопок доступно
  const int CONTENT_TOP = 60;

  int initialLeft = 20;
  int curInfoBoxLeft = initialLeft;
  int statusBoxTop = CONTENT_TOP;


  // первая строка
  
  windowStatusBox = new TFTInfoBox(TFT_WINDOW_STATUS_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox1Sensor = new TFTInfoBox(HEAT_BUTTON_CAPTION1,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  flow1Box = new TFTInfoBox(WTRFLOW_CAPTION1,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  phFlowMixBox = new TFTInfoBox("НАСОСЫ pH",(screenWidth - initialLeft*2 -  SENSOR_BOX_H_SPACING)/2,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;

  waterStatusBox = new TFTInfoBox(TFT_WATER_STATUS_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox2Sensor = new TFTInfoBox(HEAT_BUTTON_CAPTION2,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  flow2Box = new TFTInfoBox(WTRFLOW_CAPTION2,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  phPlusMinusBox = new TFTInfoBox("ИЗМЕНЕНИЕ pH",(screenWidth - initialLeft*2 - SENSOR_BOX_H_SPACING)/2,SENSOR_BOX_HEIGHT,phFlowMixBox->getX() + phFlowMixBox->getWidth() + SENSOR_BOX_H_SPACING,statusBoxTop);
  curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;

  lightStatusBox = new TFTInfoBox(TFT_LIGHT_STATUS_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox3Sensor = new TFTInfoBox(HEAT_BUTTON_CAPTION3,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  windSpeedBox = new TFTInfoBox(WINDSPEED_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  curInfoBoxLeft = initialLeft;
  statusBoxTop += SENSOR_BOX_HEIGHT + SENSOR_BOX_V_SPACING;

  // вторая строка

  heatBox1 = new TFTInfoBox(HEAT_BUTTON_CAPTION1,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox1Air = new TFTInfoBox(HEAT_AIR,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  windDirectionBox = new TFTInfoBox(WINDSPEED_DIRECTION_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);

  //USE_WATER_TANK_MODULE
  waterTankStatusBox = new TFTInfoBox("БАК С ВОДОЙ",(screenWidth - initialLeft*2 -  SENSOR_BOX_H_SPACING)/2,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  // USE_WATER_TANK_MODULE
  
  curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;

  heatBox2 = new TFTInfoBox(HEAT_BUTTON_CAPTION2,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox2Air = new TFTInfoBox(HEAT_AIR,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  rainStatusBox = new TFTInfoBox(RAIN_BOX_CAPTION,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);

  //USE_WATER_TANK_MODULE
  waterTankCommandsBox = new TFTInfoBox("УПРАВЛЕНИЕ БАКОМ",(screenWidth - initialLeft*2 - SENSOR_BOX_H_SPACING)/2,SENSOR_BOX_HEIGHT,waterTankStatusBox->getX() + waterTankStatusBox->getWidth() + SENSOR_BOX_H_SPACING,statusBoxTop);
  // USE_WATER_TANK_MODULE
  
  curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;

  heatBox3 = new TFTInfoBox(HEAT_BUTTON_CAPTION3,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  heatBox3Air = new TFTInfoBox(HEAT_AIR,SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,statusBoxTop);
  

  // теперь добавляем боксы для датчиков
   for(int i=0;i<6;i++)
   {
    sensors[i].box = NULL;
   }

  
  int startLeft = (screenWidth - (SENSOR_BOXES_PER_LINE*SENSOR_BOX_WIDTH + (SENSOR_BOXES_PER_LINE-1)*SENSOR_BOX_H_SPACING))/2;
  curInfoBoxLeft = startLeft;
  int sensorsTop = CONTENT_TOP;
  int sensorBoxesPlacedInLine = 0;
  int createdSensorIndex = 0;
  
  for(int i=0;i<6;i++)
  {
        
    if(sensorBoxesPlacedInLine == SENSOR_BOXES_PER_LINE)
    {
      sensorBoxesPlacedInLine = 0;
      curInfoBoxLeft = startLeft;
      sensorsTop += SENSOR_BOX_HEIGHT + SENSOR_BOX_V_SPACING;
    }


    sensors[createdSensorIndex].box = new TFTInfoBox("",SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,sensorsTop);
    curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;
    sensorBoxesPlacedInLine++;
    createdSensorIndex++;
  }




    curInfoBoxLeft = startLeft;
    sensorsTop = CONTENT_TOP;
    sensorBoxesPlacedInLine = 0;
    createdSensorIndex = 0;

   for(int i=0;i<6;i++)
   {
    sensors2[i].box = NULL;
   }

  for(int i=0;i<6;i++)
  {
    
    if(sensorBoxesPlacedInLine == SENSOR_BOXES_PER_LINE)
    {
      sensorBoxesPlacedInLine = 0;
      curInfoBoxLeft = startLeft;
      sensorsTop += SENSOR_BOX_HEIGHT + SENSOR_BOX_V_SPACING;
    }

    sensors2[createdSensorIndex].box = new TFTInfoBox("",SENSOR_BOX_WIDTH,SENSOR_BOX_HEIGHT,curInfoBoxLeft,sensorsTop);
    curInfoBoxLeft += SENSOR_BOX_WIDTH + SENSOR_BOX_H_SPACING;
    sensorBoxesPlacedInLine++;
    createdSensorIndex++;
  }   
  

  const int LIST_BUTTON_WIDTH = 110;
  const int options_btn_width = 200;
    
    int driveButtonWidth = (screenWidth - startLeft*2 - LIST_BUTTON_WIDTH*2 - options_btn_width - TFT_IDLE_SCREEN_BUTTON_SPACING*3);
    // у нас есть availButtons кнопок, между ними - availButtons-1 пробел, вычисляем левую координату для первой кнопки
    int curButtonLeft = initialLeft;//(screenWidth + (TFT_IDLE_SCREEN_BUTTON_WIDTH - LIST_BUTTON_WIDTH)*2 - ( availButtons*TFT_IDLE_SCREEN_BUTTON_WIDTH + (availButtons-1)*TFT_IDLE_SCREEN_BUTTON_SPACING ))/2;

    prevScreenButton = screenButtons->addButton( curButtonLeft ,  buttonsTop, LIST_BUTTON_WIDTH,  TFT_IDLE_SCREEN_BUTTON_HEIGHT, leftArrowCaption ,BUTTON_SYMBOL);
    curButtonLeft += LIST_BUTTON_WIDTH + TFT_IDLE_SCREEN_BUTTON_SPACING;

    tftSensorsButton = screenButtons->addButton( curButtonLeft ,  buttonsTop, options_btn_width,  TFT_IDLE_SCREEN_BUTTON_HEIGHT, WM_OPTIONS_CAPTION);
    screenButtons->setButtonHasIcon(tftSensorsButton);
    curButtonLeft += options_btn_width + TFT_IDLE_SCREEN_BUTTON_SPACING;

    driveButton = screenButtons->addButton( curButtonLeft ,  buttonsTop, driveButtonWidth,  TFT_IDLE_SCREEN_BUTTON_HEIGHT, WM_DRIVE_CAPTION);
    screenButtons->setButtonHasIcon(driveButton);
    curButtonLeft += driveButtonWidth + TFT_IDLE_SCREEN_BUTTON_SPACING;

    nextScreenButton = screenButtons->addButton( curButtonLeft ,  buttonsTop, LIST_BUTTON_WIDTH,  TFT_IDLE_SCREEN_BUTTON_HEIGHT, rightArrowCaption ,BUTTON_SYMBOL);
    curButtonLeft += LIST_BUTTON_WIDTH + TFT_IDLE_SCREEN_BUTTON_SPACING;



  #ifdef USE_HEAT_MODULE
    LogicManageModule->GetHeatStatus(0,heat1Active,heat1On);
    LogicManageModule->GetHeatStatus(1,heat2Active,heat2On);
    LogicManageModule->GetHeatStatus(2,heat3Active,heat3On);

    GlobalSettings* settings = MainController->GetSettings();
    HeatSettings hs = settings->GetHeatSettings(0);
    heat1Active = hs.active;

    hs = settings->GetHeatSettings(1);
    heat2Active = hs.active;

    hs = settings->GetHeatSettings(2);
    heat3Active = hs.active;
    
    heatTimer = 0;
  #endif    
  

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::onActivate(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
  
  #ifdef USE_DS3231_REALTIME_CLOCK
  lastMinute = -1;
  #endif

  #ifdef USE_SMS_MODULE
    providerNamePrinted = false;
    providerNameLength = 0;
  #endif

  #if defined(USE_PRESSURE_MODULE) && defined(DRAW_PRESSURE_ON_SCREEN)
    lastPressure = 0;
  #endif


  #ifdef USE_HEAT_MODULE

    GlobalSettings* settings = MainController->GetSettings();
    HeatSettings hs = settings->GetHeatSettings(0);
    heat1Active = hs.active;

    hs = settings->GetHeatSettings(1);
    heat2Active = hs.active;

    hs = settings->GetHeatSettings(2);
    heat3Active = hs.active;

    LogicManageModule->GetHeatTemperature(0,heatBox1SensorData,heatBox1AirData);
    LogicManageModule->GetHeatTemperature(1,heatBox2SensorData,heatBox2AirData);
    LogicManageModule->GetHeatTemperature(2,heatBox3SensorData,heatBox3AirData);
        
  #endif

  #ifdef USE_WINDOW_MANAGE_MODULE
    windSpeed = LogicManageModule->GetWindSpeed();
    if(windSpeed > 30000) // дичь
      windSpeed = 0;
    windDirection = LogicManageModule->GetWindDirection();
    hasRain = LogicManageModule->HasRain();
  #endif // USE_WINDOW_MANAGE_MODULE

  #ifdef USE_WATERFLOW_MODULE
    flow1Data = GetFlowData(0);
    flow2Data = GetFlowData(1);
  #endif // USE_WATERFLOW_MODULE

  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
  rCopy = MainController->checkReg();
  #endif

  uptimeMinutes = millis() / 60000ul;

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LORA_GATE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int TFTIdleScreen::drawLoRaIcons(TFTMenu* menuManager, int curIconRightMargin, bool rssiChanged, bool drawCaption)
{
   UTFT* dc = menuManager->getDC();
    dc->setFont(SmallRusFont);
    int fontWidth = dc->getFontXsize();
   
   int totalSegments = 4; // рисуем 4 сегмента
   int segmentWidth = 8; // ширина сегмента
   int signalHeight = 20; // высота сегмента
   int segmentSpacing = 2; // пробелы между сегментами

   bool fillSegment1 = false;
   bool fillSegment2 = false;
   bool fillSegment3 = false;
   bool fillSegment4 = false;

   int loraSigQuality = 0;
   static String loraRSSICaption;
  loraRSSICaption = lastLoraRSSI;
  loraRSSICaption += F("dBm");

   if(lastLoraRSSI >= -50)
   {
      loraSigQuality = 4;
   }
   else
   if(lastLoraRSSI >= -75)
   {
      loraSigQuality = 3;
   }
   else
   if(lastLoraRSSI >= -70)
   {
      loraSigQuality = 2;
   }
   else
   if(lastLoraRSSI >= -90)
   {
      loraSigQuality = 1;
   }
   else
   {
      loraSigQuality = 0;
      loraRSSICaption = F("- dBm");     
   }

   if(rssiChanged)
   {

     switch(loraSigQuality)
      {
        case 1: 
          fillSegment1 = true; // 25%
        break;
  
        case 2: 
          fillSegment1 = true; // 50%
          fillSegment2 = true;
        break;
  
        case 3: 
          fillSegment1 = true; // 75%
          fillSegment2 = true;
          fillSegment3 = true;
        break;
  
        case 4:
          fillSegment1 = true; // 100%
          fillSegment2 = true;
          fillSegment3 = true;
          fillSegment4 = true;
        break;
        
        case 0:
        default:
          // нет сигнала
        break;
       
      } // switch
   } // qualityChanged


    static int maxRSSICaptionWidth = -1;
    if(maxRSSICaptionWidth == -1)
    {
      maxRSSICaptionWidth = menuManager->getRusPrinter()->print("-999dBm",0,0,0,true)*fontWidth;
    }

    int initialLeft = curIconRightMargin - (totalSegments*segmentWidth) - (totalSegments-1)*segmentSpacing - maxRSSICaptionWidth - 10; 
    int initialTop = 10;

    if(rssiChanged)
    {
        // рисуем первый сегмент
        int curLeft = initialLeft;      
    
        int curHeight = signalHeight/4;
        int curTop = initialTop + (signalHeight - curHeight);    
    
        if(fillSegment1)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();           
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
    
        // рисуем второй сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*2;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment2)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();            
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем третий сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*3;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment3)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();          
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем четвертый сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = signalHeight;
        curTop = initialTop;
    
        if(fillSegment4)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();         
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }

        curLeft += 12;
        dc->setBackColor(TFT_BACK_COLOR);
        dc->setColor(TFT_BACK_COLOR);
        dc->fillRect(curLeft,initialTop,curLeft+maxRSSICaptionWidth,initialTop+signalHeight);
        
        dc->setColor(INFO_BOX_CAPTION_COLOR);
        int curRSSICaptionWidth = menuManager->getRusPrinter()->print(loraRSSICaption.c_str(),0,0,0,true)*fontWidth;
        menuManager->getRusPrinter()->print(loraRSSICaption.c_str(),curLeft+(maxRSSICaptionWidth - curRSSICaptionWidth)/2,initialTop+4);   
    
        yield();
    } // rssiChanged

    

    // теперь рисуем иконку LORA
    initialLeft -= 14;
    initialLeft -= fontWidth*4;
    
    if(drawCaption)
    {   
        String strToDraw;
        strToDraw = F("LORA");
        
        dc->setColor(INFO_BOX_CAPTION_COLOR);
        dc->setBackColor(TFT_BACK_COLOR);
    
        if(loraGate.isLoraInited())
        {
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          dc->setColor(SENSOR_BOX_FONT_COLOR);
          dc->setBackColor(INFO_BOX_CAPTION_COLOR);
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->setBackColor(TFT_BACK_COLOR);
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          yield();
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
        }
        
        menuManager->getRusPrinter()->print(strToDraw.c_str(),initialLeft+5,initialTop+5);

      yield();
    
    
    } // drawCaption

    initialLeft -= 4;
    
    dc->setFont(BigRusFont);
    return initialLeft;  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LORA_GATE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SMS_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int TFTIdleScreen::drawGSMIcons(TFTMenu* menuManager, int curIconRightMargin, bool qualityChanged, bool gprsChanged)
{
   UTFT* dc = menuManager->getDC();
   int totalSegments = 4; // рисуем 4 сегмента
   int segmentWidth = 8; // ширина сегмента
   int signalHeight = 20; // высота сегмента
   int segmentSpacing = 2; // пробелы между сегментами

   bool fillSegment1 = false;
   bool fillSegment2 = false;
   bool fillSegment3 = false;
   bool fillSegment4 = false;

   if(qualityChanged)
   {

     switch(gsmSignalQuality)
      {
        case 1: 
          fillSegment1 = true; // 25%
        break;
  
        case 2: 
          fillSegment1 = true; // 50%
          fillSegment2 = true;
        break;
  
        case 3: 
          fillSegment1 = true; // 75%
          fillSegment2 = true;
          fillSegment3 = true;
        break;
  
        case 4:
          fillSegment1 = true; // 100%
          fillSegment2 = true;
          fillSegment3 = true;
          fillSegment4 = true;
        break;
        
        case 0:
        default:
          // нет сигнала
        break;
       
      } // switch
   } // qualityChanged

    int initialLeft = curIconRightMargin - (totalSegments*segmentWidth) - (totalSegments-1)*segmentSpacing; 
    int initialTop = 10;

    if(qualityChanged)
    {
        // рисуем первый сегмент
        int curLeft = initialLeft;      
    
        int curHeight = signalHeight/4;
        int curTop = initialTop + (signalHeight - curHeight);    
    
        if(fillSegment1)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();           
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
    
        // рисуем второй сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*2;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment2)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();            
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем третий сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*3;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment3)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();          
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем четвертый сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = signalHeight;
        curTop = initialTop;
    
        if(fillSegment4)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();         
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
    } // qualityChanged

    

    // теперь рисуем иконку GPRS
    dc->setFont(SmallRusFont);
    int fontWidth = dc->getFontXsize();
    initialLeft -= 14;
    initialLeft -= fontWidth*4;
    
    if(gprsChanged)
    {   
        String strToDraw;
        strToDraw = F("GPRS");
        
        dc->setColor(INFO_BOX_CAPTION_COLOR);
        dc->setBackColor(TFT_BACK_COLOR);
    
        if(gprsAvailable)
        {
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          dc->setColor(SENSOR_BOX_FONT_COLOR);
          dc->setBackColor(INFO_BOX_CAPTION_COLOR);
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->setBackColor(TFT_BACK_COLOR);
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          yield();
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
        }
        
        menuManager->getRusPrinter()->print(strToDraw.c_str(),initialLeft+5,initialTop+5);

      yield();
    
    
    } // gprsChanged

    initialLeft -= 8;

    if(!providerNamePrinted)
    {
      providerNamePrinted = true;
      String gsmProv = MainController->GetSettings()->GetGSMProviderName();
      providerNameLength = menuManager->getRusPrinter()->utf8_strlen(gsmProv.c_str());
      int lft = initialLeft - fontWidth*providerNameLength;

      dc->setBackColor(TFT_BACK_COLOR);
      dc->setColor(INFO_BOX_CAPTION_COLOR);
      
      menuManager->getRusPrinter()->print(gsmProv.c_str(),lft+5,initialTop+5);

    }

    initialLeft -= fontWidth*providerNameLength;
    
    dc->setFont(BigRusFont);
    return initialLeft;
 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SMS_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_PRESSURE_MODULE) && defined(DRAW_PRESSURE_ON_SCREEN)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "PressureModule.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int TFTIdleScreen::drawPressure(TFTMenu* menuManager, int curIconRightMargin)
{
  const int MAX_PRESSURE_LEN = 18;
  const int MAX_PRESSURE_VALUE_LEN = 3;
  const int SPACING = 20;

  UTFT* dc = menuManager->getDC();
  uint8_t* oldFont = dc->getFont();
  
  dc->setFont(SmallRusFont);
  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();
  int left = curIconRightMargin - MAX_PRESSURE_LEN*fontWidth - SPACING;

  PressureData nowPressure = Pressure->GetPressure();

  if(nowPressure.Value == lastPressure)
  {
    dc->setFont(oldFont);
    return left; // ничего не изменилось
  }

  word oldColor = dc->getColor();
  word oldBackColor = dc->getBackColor();

  lastPressure = nowPressure.Value;

  // рисуем надпись "давление"

  UTFTRus* rusPrinter = menuManager->getRusPrinter();
  String displayString = F("ДАВЛЕНИЕ: ");
  int strWidth = rusPrinter->utf8_strlen(displayString.c_str()) * fontWidth;

  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR);

  int curLeft = left;
  int curTop = 14;
  rusPrinter->print(displayString.c_str(),curLeft,curTop);
  
  curLeft += strWidth;
  
  // рисуем бокс с подложкой
  int boxRight = curLeft  + fontWidth*MAX_PRESSURE_VALUE_LEN;
  dc->fillRect(curLeft - 8,curTop - 4 ,boxRight+4,curTop+fontHeight + 4);


  // рисуем значение давления
  displayString = nowPressure.HasData() ? String(nowPressure.Value)  : "-";
  strWidth = rusPrinter->utf8_strlen(displayString.c_str()) * fontWidth;

  curLeft += (MAX_PRESSURE_VALUE_LEN*fontWidth - strWidth)/2;

  dc->setColor(TFT_BACK_COLOR);
  dc->setBackColor(INFO_BOX_CAPTION_COLOR);

  rusPrinter->print(displayString.c_str(),curLeft,curTop);

  // теперь рисуем "мм"
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR);

  displayString = F("мм рт");
  curLeft = boxRight + 8;
  rusPrinter->print(displayString.c_str(),curLeft,curTop);  

  dc->setFont(oldFont);
  dc->setColor(oldColor);
  dc->setBackColor(oldBackColor);
    
  return left;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PRESSURE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int TFTIdleScreen::drawWiFiIcons(TFTMenu* menuManager, int curIconRightMargin, bool connectChanged, bool qualityChanged)
{
   UTFT* dc = menuManager->getDC();
   int totalSegments = 4; // рисуем 4 сегмента
   int segmentWidth = 8; // ширина сегмента
   int signalHeight = 20; // высота сегмента
   int segmentSpacing = 2; // пробелы между сегментами

   bool fillSegment1 = false;
   bool fillSegment2 = false;
   bool fillSegment3 = false;
   bool fillSegment4 = false;

   if(qualityChanged)
   {

     switch(wifiSignalQuality)
      {
        case 1: 
          fillSegment1 = true; // 25%
        break;
  
        case 2: 
          fillSegment1 = true; // 50%
          fillSegment2 = true;
        break;
  
        case 3: 
          fillSegment1 = true; // 75%
          fillSegment2 = true;
          fillSegment3 = true;
        break;
  
        case 4:
          fillSegment1 = true; // 100%
          fillSegment2 = true;
          fillSegment3 = true;
          fillSegment4 = true;
        break;
        
        case 0:
        default:
          // нет сигнала
        break;
       
      } // switch
   } // qualityChanged

    int initialLeft = curIconRightMargin - (totalSegments*segmentWidth) - (totalSegments-1)*segmentSpacing - 20; 
    int initialTop = 10;

    if(qualityChanged)
    {
        // рисуем первый сегмент
        int curLeft = initialLeft;      
    
        int curHeight = signalHeight/4;
        int curTop = initialTop + (signalHeight - curHeight);    
    
        if(fillSegment1)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();           
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
    
        // рисуем второй сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*2;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment2)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();            
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем третий сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = (signalHeight/4)*3;
        curTop = initialTop + (signalHeight - curHeight);
    
        if(fillSegment3)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();          
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
        
        // рисуем четвертый сегмент
        curLeft += segmentWidth + segmentSpacing;
        curHeight = signalHeight;
        curTop = initialTop;
    
        if(fillSegment4)
        {
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);            
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->fillRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);
          yield();         
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(curLeft,curTop,curLeft+segmentWidth,curTop+curHeight);      
        }
    
        yield();
    } // qualityChanged

    // теперь рисуем иконку WIFI
    dc->setFont(SmallRusFont);
    int fontWidth = dc->getFontXsize();
    initialLeft -= 14;
    initialLeft -= fontWidth*4;
    
    if(connectChanged)
    {   
        String strToDraw;
        strToDraw = F("WIFI");
        
        dc->setColor(INFO_BOX_CAPTION_COLOR);
        dc->setBackColor(TFT_BACK_COLOR);
    
        if(connectedToRouter)
        {
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          dc->setColor(SENSOR_BOX_FONT_COLOR);
          dc->setBackColor(INFO_BOX_CAPTION_COLOR);
        }
        else
        {
          dc->setColor(TFT_BACK_COLOR);
          dc->setBackColor(TFT_BACK_COLOR);
          dc->fillRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
          yield();
          dc->setColor(INFO_BOX_CAPTION_COLOR);
          dc->drawRect(initialLeft,initialTop,initialLeft+fontWidth*4+6,initialTop+signalHeight);
        }
        
        menuManager->getRusPrinter()->print(strToDraw.c_str(),initialLeft+5,initialTop+5);
    
      yield();
    
    } // connectChanged

    dc->setFont(BigRusFont);
    return initialLeft - 10;
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WIFI_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC() || !drawCalled)
  {
    return;
  }
  
  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)

  static bool unrVisible = false;
  const uint16_t UNR_TMR_INTERVAL = 1000;

  if(millis() - unrTimer > UNR_TMR_INTERVAL)
  {
      unrVisible = !unrVisible;
    
      if(!rCopy)
      {
        UTFT* dc = menuManager->getDC();
        UTFTRus* rusPrinter = menuManager->getRusPrinter();
        uint8_t* oldFont = dc->getFont();
        dc->setFont(BigRusFont);
        word oldBackColor = dc->getBackColor();
        dc->setBackColor(TFT_BACK_COLOR);
        word oldColor = dc->getColor();
        dc->setColor(VGA_RED);
    
        String unrMessage = RESTRICTED_FEATURES;
        
        if(!unrVisible)
        {
          int len = rusPrinter->utf8_strlen(unrMessage.c_str());
          unrMessage = "";
          
          for(int i=0;i<len;i++)
            unrMessage += ' ';
        }
        
        rusPrinter->print(unrMessage.c_str(),10,10);
    
        dc->setFont(oldFont);
        dc->setBackColor(oldBackColor);
        dc->setColor(oldColor);
        
      } // if(!MainController->checkReg())

    unrTimer = millis();

  } // if(millis() - unrTimer > UNR_TMR_INTERVAL)

  
  
  #endif // (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)


  #ifdef USE_DS3231_REALTIME_CLOCK    
    DrawDateTime(menuManager);
  #endif

  uint32_t thisUptimeMinutes = millis() / 60000ul;
  if(thisUptimeMinutes != uptimeMinutes)
  {
    uptimeMinutes = thisUptimeMinutes;
    drawUptimeMinutes(menuManager);
  }

  UTFT* dc = menuManager->getDC();
  int screenWidth = dc->getDisplayXSize();   
  int curIconRightMargin = screenWidth - 16;
  

  #ifdef USE_SMS_MODULE
    uint8_t q = SIM800.getSignalQuality();
    bool hasGprs = SIM800.hasGPRSConnection();
    bool sigChanges = q != gsmSignalQuality;
    bool gprsChanges = gprsAvailable != hasGprs;
    gsmSignalQuality = q;
    gprsAvailable = hasGprs;
    
    curIconRightMargin = drawGSMIcons(menuManager,curIconRightMargin,sigChanges, gprsChanges);
  #endif  

  #ifdef USE_WIFI_MODULE
    bool conToRouter = ESP.isConnectedToRouter();
    bool conChanges = connectedToRouter != conToRouter;
    connectedToRouter = conToRouter;

    uint8_t wifiQ = ESP.getSignalQuality();
    bool wifiQChanges = wifiQ != wifiSignalQuality;
    wifiSignalQuality = wifiQ;
    curIconRightMargin = drawWiFiIcons(menuManager,curIconRightMargin,conChanges, wifiQChanges);
  #endif

  #ifdef USE_LORA_GATE
    int curRSSI = loraGate.getRSSI();
    bool rssiChanged = curRSSI != lastLoraRSSI;
    lastLoraRSSI = curRSSI;
    curIconRightMargin = drawLoRaIcons(menuManager,curIconRightMargin,rssiChanged,false);
  #endif

  #if defined(USE_PRESSURE_MODULE) && defined(DRAW_PRESSURE_ON_SCREEN)
    curIconRightMargin = drawPressure(menuManager,curIconRightMargin);
  #endif

  // Смотрим, какая кнопка нажата
  int pressed_button = screenButtons->checkButtons(ButtonPressed,ButtonReleased);

  if(pressed_button != -1)
  {
    menuManager->resetIdleTimer();
  }

  if(pressed_button == tftSensorsButton)
  {
    drawCalled = false;
    menuManager->switchToScreen("SENSORS");
    return;
  }

  if(pressed_button == driveButton)
  {
    drawCalled = false;
    menuManager->switchToScreen("DRIVE");
    return;    
  }

  if(pressed_button == prevScreenButton)
  {
    currentScreen--;
    if(currentScreen < 0)
      currentScreen = TFT_IDLE_SCREENS - 1;

    eraseScreenArea(menuManager);
    drawCurrentScreen(menuManager);
    
    return;
  }

  if(pressed_button == nextScreenButton)
  {
    currentScreen++;
    if(currentScreen >= TFT_IDLE_SCREENS)
      currentScreen = 0;

    eraseScreenArea(menuManager);
    drawCurrentScreen(menuManager);
  }

  #ifdef USE_WATER_TANK_MODULE
  if(pressed_button == fillTankButton)
  {
    WaterTank->FillTank(!WaterTank->IsValveOn());
  }
  #endif // USE_WATER_TANK_MODULE

  // обновляем текущий экран
  updateCurrentScreen(menuManager);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateSensors(TFTMenu* menuManager)
{
  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;  
  
    sensorsTimer += dt;
        
    if(sensorsTimer > TFT_SENSORS_UPDATE_INTERVAL)
    {
      sensorsTimer = 0;

      for(int i=0;i<6;i++)
      {
        drawSensorData(menuManager,sensors[i].box,i);
      }
      
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateSensors2(TFTMenu* menuManager)
{  
  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;
    
    sensorsTimer2 += dt;
        
    if(sensorsTimer2 > TFT_SENSORS_UPDATE_INTERVAL)
    {
      sensorsTimer2 = 0;

      for(int i=0;i<6;i++)
      {
        drawSensorData(menuManager,sensors2[i].box,6+i);
      }
      
    }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::updateCurrentScreen(TFTMenu* menuManager)
{
  // USE_WATER_TANK_MODULE
  if(currentScreen != TFT_IDLE_PH_SCREEN_NUMBER)
  {
    if(fillTankButton != 0xFF)
    {
      screenButtons->deleteButton(fillTankButton);
      fillTankButton = 0xFF;
    }
  }
  // USE_WATER_TANK_MODULE
  
  switch(currentScreen)
  {

    case TFT_IDLE_SENSORS_SCREEN_NUMBER:
    {
      updateSensors(menuManager);
    }    
    break;

      case TFT_IDLE_SENSORS_SCREEN_NUMBER2:
      {
        updateSensors2(menuManager);
      }    
      break;

      #ifdef USE_WATER_TANK_MODULE
      case TFT_IDLE_PH_SCREEN_NUMBER:
      {
        if(fillTankButton == 0xFF) // ещё нет кнопки
        {          

          #ifdef USE_LORA_GATE

          // тут проверяем, активна ли LoRa
          if(loraGate.isLoraInited())
          {
          
              TFTInfoBoxContentRect rc = waterTankCommandsBox->getContentRect(menuManager);
              fillTankButton = screenButtons->addButton( rc.x + 10 , rc.y + 10, rc.w - 20,  rc.h - 20, "НАПОЛНИТЬ БАК");
              //screenButtons->setButtonBackColor(fillTankButton, VGA_MAROON);
              //screenButtons->setButtonFontColor(fillTankButton, VGA_WHITE);

              if(!WaterTank->IsModuleOnline())
              {
                screenButtons->disableButton(fillTankButton);
              }
    
              //Serial.println("draw fill tank button 2");
              screenButtons->drawButton(fillTankButton);
              
          } // if(loraGate.isLoraInited())

         #endif // USE_LORA_GATE

        } // if(fillTankButton == 0xFF)
        else
        {
          // кнопка заполнения бака - уже есть

            if(!WaterTank->IsModuleOnline())
            {
              screenButtons->disableButton(fillTankButton,screenButtons->buttonEnabled(fillTankButton));
            }
            else
            {
              screenButtons->enableButton(fillTankButton,!screenButtons->buttonEnabled(fillTankButton));
            }
                     
          
        } // else
      }
      break;
     #endif // USE_WATER_TANK_MODULE

  }

  updateStatuses(menuManager);
  updateHeatSensors(menuManager);
  updateFlowData(menuManager);

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawCurrentScreen(TFTMenu* menuManager)
{
  switch(currentScreen)
  {
    case TFT_IDLE_STATUSES_SCREEN_NUMBER:
    {
      drawStatuses(menuManager);
    }
    break;

    case TFT_IDLE_SENSORS_SCREEN_NUMBER:
    {
      drawSensors(menuManager);
    }
    break;

      case TFT_IDLE_SENSORS_SCREEN_NUMBER2:
      {
        drawSensors2(menuManager);
      }
      break;

    case TFT_IDLE_HEAT_SCREEN_NUMBER:
    {
      drawHeatSensors(menuManager);
    }
    break;
    
    case TFT_IDLE_FLOW_SCREEN_NUMBER:
    {
      drawFlowData(menuManager);
    }
    break;

    case TFT_IDLE_PH_SCREEN_NUMBER:
    {      
      drawPHStatus(menuManager);
      drawWaterTankStatus(menuManager);
    }
    break;

  } // switch
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawStatuses(TFTMenu* menuManager)
{
    windowStatusBox->draw(menuManager);
    drawWindowStatus(menuManager);

    waterStatusBox->draw(menuManager);
    drawWaterStatus(menuManager);

    lightStatusBox->draw(menuManager);
    drawLightStatus(menuManager);

    heatBox1->draw(menuManager);
    heatBox2->draw(menuManager);
    heatBox3->draw(menuManager);
    drawHeatStatus(menuManager);
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawSensors(TFTMenu* menuManager)
{
 
  //Тут отрисовка боксов с показаниями датчиков
  for(int i=0;i<6;i++)
  {
    if(!sensors[i].box)
      continue;   

     TFTSensorBinding bnd = HardwareBinding->GetTFTSensorBinding(i);
    
    sensors[i].box->draw(menuManager);
    sensors[i].box->drawCaption(menuManager,bnd.Caption);
    drawSensorData(menuManager,sensors[i].box,i,true); // тут перерисовываем показания по-любому
    sensorsTimer = 0; // сбрасываем таймер перерисовки показаний датчиков

  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::drawSensors2(TFTMenu* menuManager)
{  
  //Тут отрисовка боксов с показаниями датчиков
  for(int i=0;i<6;i++)
  {
    if(!sensors2[i].box)
      continue;
      
    TFTSensorBinding bnd = HardwareBinding->GetTFTSensorBinding(6+i);
    
    sensors2[i].box->draw(menuManager);
    sensors2[i].box->drawCaption(menuManager,bnd.Caption);
    drawSensorData(menuManager,sensors2[i].box,6+i,true); // тут перерисовываем показания по-любому
    sensorsTimer2 = 0; // сбрасываем таймер перерисовки показаний датчиков

  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::eraseScreenArea(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  dc->setColor(TFT_BACK_COLOR);

  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();
  int left = 10;
  int top = 45;
  int right =  screenWidth - left;
  int bottom = screenHeight - TFT_IDLE_SCREEN_BUTTON_HEIGHT - 40;

  dc->fillRect(left,top,right,bottom);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTIdleScreen::draw(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }

  screenButtons->drawButtons(drawButtonsYield); // рисуем наши кнопки

  // рисуем линию под информационными иконками
  UTFT* dc = menuManager->getDC();
  dc->setColor(TFT_FONT_COLOR);
  int screenWidth = dc->getDisplayXSize();
  dc->drawLine(10,40,screenWidth-10,40);

  // рисуем номер версии
  uint8_t* oldFont = dc->getFont();
  dc->setFont(SmallRusFont);
  dc->setColor(INFO_BOX_CAPTION_COLOR);
  dc->setBackColor(TFT_BACK_COLOR);
  menuManager->getRusPrinter()->print(PROGRAM_VERSION,10,14);
  dc->setFont(oldFont);    


  drawCurrentScreen(menuManager);

  int curIconRightMargin = screenWidth - 16;

  #ifdef USE_SMS_MODULE
    curIconRightMargin = drawGSMIcons(menuManager,curIconRightMargin,true,true);
  #endif

  #ifdef USE_WIFI_MODULE
    curIconRightMargin = drawWiFiIcons(menuManager,curIconRightMargin,true,true);
  #endif

  #ifdef USE_LORA_GATE
    curIconRightMargin = drawLoRaIcons(menuManager,curIconRightMargin,true,true);
  #endif

  #if defined(USE_PRESSURE_MODULE) && defined(DRAW_PRESSURE_ON_SCREEN)
    curIconRightMargin = drawPressure(menuManager,curIconRightMargin);
  #endif

  drawUptimeMinutes(menuManager);

  drawCalled = true;
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTMenu
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTMenu* TFTScreen = NULL;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTMenu::TFTMenu()
{
  TFTScreen = this;
  currentScreenIndex = -1;
  flags.isLCDOn = true;
  switchTo = NULL;
  switchToIndex = -1;
  tftDC = NULL;
  tftTouch = NULL;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::setup()
{
  //Serial.println("TFTMenu::setup()");

  TFTBinding bnd = HardwareBinding->GetTFTBinding();
  if(!(bnd.DisplayType == 1 || bnd.DisplayType == 2))
  {
    return; // нет привязки
  }

  //Serial.println("Turn backlight ON !!!");
  
  lcdOn(); // включаем подсветку

  //Serial.println("Create TFT instance...");
  
  tftDC = new UTFT(TFT_MODEL,TFT_RS_PIN,TFT_WR_PIN,TFT_CS_PIN,TFT_RST_PIN);
  tftTouch = new URTouch(TFT_TOUCH_CLK_PIN,TFT_TOUCH_CS_PIN,TFT_TOUCH_DIN_PIN,TFT_TOUCH_DOUT_PIN,TFT_TOUCH_IRQ_PIN);

  #ifdef TFT_EXTRA_RESET
    WORK_STATUS.PinMode(TFT_RST_PIN,OUTPUT);
    WORK_STATUS.PinWrite(TFT_RST_PIN,HIGH);
    delay(10);
    WORK_STATUS.PinWrite(TFT_RST_PIN,LOW);
    delay(10);
    WORK_STATUS.PinWrite(TFT_RST_PIN,HIGH);        
  #endif

  if(bnd.InitDelay > 0)
  {
    delay(bnd.InitDelay);
  }
  
  tftDC->InitLCD(LANDSCAPE);
  tftDC->fillScr(TFT_BACK_COLOR);
  yield();
  tftDC->setFont(BigRusFont);

  tftTouch->InitTouch(LANDSCAPE);
  tftTouch->setPrecision(PREC_HI);
  
  rusPrint.init(tftDC);

  resetIdleTimer();

  // добавляем служебные экраны

  // окно сообщения
  TFTScreenInfo mbscrif;
  mbscrif.screen = MessageBoxScreen::create();
  mbscrif.screen->setup(this);
  mbscrif.screenName = "MB";
  screens.push_back(mbscrif);

  // клавиатура
  mbscrif.screen = KeyboardScreen::create();
  mbscrif.screen->setup(this);
  mbscrif.screenName = "KBD";
  screens.push_back(mbscrif);

  // экран управления  
  mbscrif.screen = new TFTDriveScreen();
  mbscrif.screen->setup(this);
  mbscrif.screenName = "DRIVE";
  screens.push_back(mbscrif);

  // добавляем экран ожидания
  AbstractTFTScreen* idleScreen = new TFTIdleScreen();
  idleScreen->setup(this);
  TFTScreenInfo si; 
  si.screenName = "IDLE"; 
  si.screen = idleScreen;  
  screens.push_back(si);


  // добавляем экран датчиков
    AbstractTFTScreen* sensorsScreen = new TFTSensorsScreen();
    sensorsScreen->setup(this);
    TFTScreenInfo sdtscr; 
    sdtscr.screenName = "SENSORS"; 
    sdtscr.screen = sensorsScreen;  
    screens.push_back(sdtscr);

  #if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
    AbstractTFTScreen* wmScreen = new TFTWMScreen(0);
    wmScreen->setup(this);
    TFTScreenInfo wmss; 
    wmss.screenName = "WMS1"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTWMScreen(1);
    wmScreen->setup(this);
    wmss.screenName = "WMS2"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTWMScreen(2);
    wmScreen->setup(this);
    wmss.screenName = "WMS3"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTWMScreen(3);
    wmScreen->setup(this);
    wmss.screenName = "WMS4"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTWMenuScreen();
    wmScreen->setup(this);
    wmss.screenName = "WMS"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTWChannelsScreen();
    wmScreen->setup(this);
    wmss.screenName = "WMW"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);
    
    wmScreen = new TFTRainScreen();
    wmScreen->setup(this);
    wmss.screenName = "RAIN"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);

    wmScreen = new TFTOrientationScreen();
    wmScreen->setup(this);
    wmss.screenName = "ORIENT"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss); 

    wmScreen = new TFTWindScreen();
    wmScreen->setup(this);
    wmss.screenName = "WIND"; 
    wmss.screen = wmScreen;  
    screens.push_back(wmss);       
    
  #endif // USE_WINDOW_MANAGE_MODULE    

  #ifdef USE_HEAT_MODULE
  
    AbstractTFTScreen* heatScreen = new TFTHeatScreen();
    heatScreen->setup(this);
    TFTScreenInfo hsis; 
    hsis.screenName = "HEAT"; 
    hsis.screen = heatScreen;  
    screens.push_back(hsis);

    heatScreen = new TFTHeatSettingsScreen(0);
    heatScreen->setup(this);
    hsis.screenName = "HEAT1"; 
    hsis.screen = heatScreen;  
    screens.push_back(hsis);

    heatScreen = new TFTHeatSettingsScreen(1);
    heatScreen->setup(this);
    hsis.screenName = "HEAT2"; 
    hsis.screen = heatScreen;  
    screens.push_back(hsis);
    
    heatScreen = new TFTHeatSettingsScreen(2);
    heatScreen->setup(this);
    hsis.screenName = "HEAT3"; 
    hsis.screen = heatScreen;  
    screens.push_back(hsis);

    heatScreen = new TFTHeatDriveScreen();
    heatScreen->setup(this);
    hsis.screenName = "HDRIVE"; 
    hsis.screen = heatScreen;  
    screens.push_back(hsis);

  #endif // USE_HEAT_MODULE

  #ifdef USE_SHADOW_MODULE
    AbstractTFTScreen* shadowScreen = new TFTShadowScreen();
    shadowScreen->setup(this);
    TFTScreenInfo shsis; 
    shsis.screenName = "SHADOW"; 
    shsis.screen = shadowScreen;  
    screens.push_back(shsis);

    shadowScreen = new TFTShadowSettingsScreen(0);
    shadowScreen->setup(this);
    shsis.screenName = "SHADOW1"; 
    shsis.screen = shadowScreen;  
    screens.push_back(shsis);

    shadowScreen = new TFTShadowSettingsScreen(1);
    shadowScreen->setup(this);
    shsis.screenName = "SHADOW2"; 
    shsis.screen = shadowScreen;  
    screens.push_back(shsis);

    shadowScreen = new TFTShadowSettingsScreen(2);
    shadowScreen->setup(this);
    shsis.screenName = "SHADOW3"; 
    shsis.screen = shadowScreen;  
    screens.push_back(shsis);

    shadowScreen = new TFTShadowDriveScreen();
    shadowScreen->setup(this);
    shsis.screenName = "SDRIVE"; 
    shsis.screen = shadowScreen;  
    screens.push_back(shsis);
    
  #endif // USE_SHADOW_MODULE


  #ifdef USE_WATERFLOW_MODULE
    AbstractTFTScreen* flowScreen = new TFTFlowScreen();
    flowScreen->setup(this);
    TFTScreenInfo fff; 
    fff.screenName = "FLOW"; 
    fff.screen = flowScreen;  
    screens.push_back(fff);
  #endif  


  // добавляем экран управления фрамугами
  #ifdef USE_TEMP_SENSORS
    AbstractTFTScreen* windowScreen = new TFTWindowScreen();
    windowScreen->setup(this);
    TFTScreenInfo wsi; 
    wsi.screenName = "WINDOW"; 
    wsi.screen = windowScreen;  
    screens.push_back(wsi);
  #endif

  // добавляем экран управления каналами полива
  #ifdef USE_WATERING_MODULE
    AbstractTFTScreen* wateringScreen = new TFTWateringScreen();
    wateringScreen->setup(this);
    TFTScreenInfo watersi; 
    watersi.screenName = "WATER"; 
    watersi.screen = wateringScreen;  
    screens.push_back(watersi);

    wateringScreen = new TFTWateringChannelsScreen();
    wateringScreen->setup(this);
    watersi.screenName = "WTRCH"; 
    watersi.screen = wateringScreen;  
    screens.push_back(watersi);

    wateringScreen = new TFTWateringChannelSettingsScreen();
    wateringScreen->setup(this);
    watersi.screenName = "WTRCHS"; 
    watersi.screen = wateringScreen;  
    screens.push_back(watersi);

    
  #endif // USE_WATERING_MODULE


  // добавляем экран управления досветкой
  #ifdef USE_LUMINOSITY_MODULE
    AbstractTFTScreen* lightScreen = new TFTLightScreen();
    lightScreen->setup(this);
    TFTScreenInfo lsi; 
    lsi.screenName = "LIGHT"; 
    lsi.screen = lightScreen;  
    screens.push_back(lsi);
  #endif

  // добавляем экран управления сценариями
  #ifdef USE_SCENE_MODULE
    AbstractTFTScreen* sceneScreen = new TFTSceneScreen();
    sceneScreen->setup(this);
    TFTScreenInfo scsi; 
    scsi.screenName = "SCN"; 
    scsi.screen = sceneScreen;  
    screens.push_back(scsi);
  #endif  


    #ifdef USE_CYCLE_VENT_MODULE
    
    AbstractTFTScreen* cvScreen = new TFTCycleVentScreen();
    cvScreen->setup(this);
    TFTScreenInfo cventS; 
    cventS.screenName = "CYCLE_VENT"; 
    cventS.screen = cvScreen;  
    screens.push_back(cventS);

    cvScreen = new TFTCycleVentSettingsScreen(0);
    cvScreen->setup(this);
    cventS.screenName = "CYCLE_VENT1"; 
    cventS.screen = cvScreen;  
    screens.push_back(cventS);

    cvScreen = new TFTCycleVentSettingsScreen(1);
    cvScreen->setup(this);
    cventS.screenName = "CYCLE_VENT2"; 
    cventS.screen = cvScreen;  
    screens.push_back(cventS);

    cvScreen = new TFTCycleVentSettingsScreen(2);
    cvScreen->setup(this);
    cventS.screenName = "CYCLE_VENT3"; 
    cventS.screen = cvScreen;  
    screens.push_back(cventS);
    
    #endif // USE_CYCLE_VENT_MODULE


    #ifdef USE_VENT_MODULE
    
    AbstractTFTScreen* ventScreen = new TFTVentScreen();
    ventScreen->setup(this);
    TFTScreenInfo ventS; 
    ventS.screenName = "VENT"; 
    ventS.screen = ventScreen;  
    screens.push_back(ventS);

    ventScreen = new TFTVentSettingsScreen(0);
    ventScreen->setup(this);
    ventS.screenName = "VENT1"; 
    ventS.screen = ventScreen;  
    screens.push_back(ventS);

    ventScreen = new TFTVentSettingsScreen(1);
    ventScreen->setup(this);
    ventS.screenName = "VENT2"; 
    ventS.screen = ventScreen;  
    screens.push_back(ventS);

    ventScreen = new TFTVentSettingsScreen(2);
    ventScreen->setup(this);
    ventS.screenName = "VENT3"; 
    ventS.screen = ventScreen;  
    screens.push_back(ventS);
    
    #endif // USE_VENT_MODULE    


 #ifdef USE_HUMIDITY_SPRAY_MODULE
    
    AbstractTFTScreen* sprayScreen = new TFTSprayScreen();
    sprayScreen->setup(this);
    TFTScreenInfo sprayS; 
    sprayS.screenName = "SPRAY"; 
    sprayS.screen = sprayScreen;  
    screens.push_back(sprayS);

    sprayScreen = new TFTSpraySettingsScreen(0);
    sprayScreen->setup(this);
    sprayS.screenName = "SPRAY1"; 
    sprayS.screen = sprayScreen;  
    screens.push_back(sprayS);

    sprayScreen = new TFTSpraySettingsScreen(1);
    sprayScreen->setup(this);
    sprayS.screenName = "SPRAY2"; 
    sprayS.screen = sprayScreen;  
    screens.push_back(sprayS);

    sprayScreen = new TFTSpraySettingsScreen(2);
    sprayScreen->setup(this);
    sprayS.screenName = "SPRAY3"; 
    sprayS.screen = sprayScreen;  
    screens.push_back(sprayS);
    
    #endif // USE_HUMIDITY_SPRAY_MODULE        


  #ifdef USE_THERMOSTAT_MODULE
    
    AbstractTFTScreen* thermostatScreen = new TFTThermostatScreen();
    thermostatScreen->setup(this);
    TFTScreenInfo thermostatS; 
    thermostatS.screenName = "THERMOSTAT"; 
    thermostatS.screen = thermostatScreen;  
    screens.push_back(thermostatS);

    thermostatScreen = new TFTThermostatSettingsScreen(0);
    thermostatScreen->setup(this);
    thermostatS.screenName = "THERMOSTAT1"; 
    thermostatS.screen = thermostatScreen;  
    screens.push_back(thermostatS);

    thermostatScreen = new TFTThermostatSettingsScreen(1);
    thermostatScreen->setup(this);
    thermostatS.screenName = "THERMOSTAT2"; 
    thermostatS.screen = thermostatScreen;  
    screens.push_back(thermostatS);

    thermostatScreen = new TFTThermostatSettingsScreen(2);
    thermostatScreen->setup(this);
    thermostatS.screenName = "THERMOSTAT3"; 
    thermostatS.screen = thermostatScreen;  
    screens.push_back(thermostatS);
    
    #endif // USE_THERMOSTAT_MODULE    


    #ifdef USE_PH_MODULE
      AbstractTFTScreen* phScreen = new TFTPHControlScreen();
      phScreen->setup(this);
      TFTScreenInfo phInfo;

      phInfo.screenName = "PHControl"; 
      phInfo.screen = phScreen;  
      screens.push_back(phInfo);


      phScreen = new TFTPHSettingsScreen();
      phScreen->setup(this);
      phInfo.screenName = "PHSettings"; 
      phInfo.screen = phScreen;  
      screens.push_back(phInfo);
      
    #endif // USE_PH_MODULE

    
    
#ifdef USE_CO2_MODULE
      AbstractTFTScreen* co2Screen = new TFTCO2SettingsScreen();
      co2Screen->setup(this);
      TFTScreenInfo co2Info;

      co2Info.screenName = "CO2Control"; 
      co2Info.screen = co2Screen;  
      screens.push_back(co2Info);
      
    #endif // USE_CO2_MODULE

#ifdef USE_DOOR_MODULE
    AbstractTFTScreen* doorsScreen = new TFTDoorsScreen();
    doorsScreen->setup(this);
    TFTScreenInfo doorsS; 
    doorsS.screenName = "DOORS"; 
    doorsS.screen = doorsScreen;  
    screens.push_back(doorsS);

    doorsScreen = new TFTDoorScreen();
    doorsScreen->setup(this);
    doorsS.screenName = "DOOR"; 
    doorsS.screen = doorsScreen;  
    screens.push_back(doorsS);
  #endif // USE_DOOR_MODULE    

  // добавляем экран настроек
    AbstractTFTScreen* settingsScreen = new TFTSettingsScreen();
    settingsScreen->setup(this);
    TFTScreenInfo ssi; 
    ssi.screenName = "OPTIONS"; 
    ssi.screen = settingsScreen;  
    screens.push_back(ssi);

    ssi.screen = new TFTTimeSyncScreen();
    ssi.screen->setup(this);
    ssi.screenName = "TIMESYNC";
    screens.push_back(ssi);
    


  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
    AbstractTFTScreen* unrkScreen = new TFTUnrKScreen();
    unrkScreen->setup(this);
    TFTScreenInfo unrkSS; 
    unrkSS.screenName = "UNRK"; 
    unrkSS.screen = unrkScreen;  
    screens.push_back(unrkSS);
  #endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)


  AbstractTFTScreen* mimscr = new TFTMemInitScreen();
  mimscr->setup(this);
  TFTScreenInfo mimscrInf;
  mimscrInf.screenName = "MEMINIT"; 
  mimscrInf.screen = mimscr;  
  screens.push_back(mimscrInf);

 // Serial.println("TFT ready!!!");
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::switchBacklight(uint8_t level)
{

  TFTBinding bnd = HardwareBinding->GetTFTBinding();

  
  if(bnd.BacklightLinkType != linkUnbinded && bnd.BacklightPin != UNBINDED_PIN)
  {
      if(bnd.BacklightLinkType == linkDirect)
      {

         #ifndef DISABLE_TFT_CONFIGURE
          if(EEPROMSettingsModule::SafePin(bnd.BacklightPin))
         #else
          if(bnd.BacklightPin > 1) // не даём блокировать Serial
         #endif 
          {
              
  
//    Serial.println(bnd.BacklightLinkType);
//    Serial.println(bnd.BacklightPin);
//    Serial.println(bnd.BacklightOnLevel);
//    Serial.println(level);
   
              WORK_STATUS.PinMode(bnd.BacklightPin,OUTPUT);
              WORK_STATUS.PinWrite(bnd.BacklightPin,level ? bnd.BacklightOnLevel : !bnd.BacklightOnLevel);
            
          }
      }
      else
      if(bnd.BacklightLinkType == linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(bnd.BacklightMCPAddress,bnd.BacklightPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.BacklightMCPAddress,bnd.BacklightPin, level ? bnd.BacklightOnLevel : !bnd.BacklightOnLevel);
        #endif
      }
      else
      if(bnd.BacklightLinkType == linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(bnd.BacklightMCPAddress,bnd.BacklightPin,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(bnd.BacklightMCPAddress,bnd.BacklightPin, level ? bnd.BacklightOnLevel : !bnd.BacklightOnLevel);
        #endif
      }
    
  } // if(bnd.BacklightLinkType != linkUnbinded && bnd.BacklightPin != UNBINDED_PIN)  
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::lcdOn()
{
    switchBacklight(1);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::lcdOff()
{
    switchBacklight(0);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::onButtonPressed(int button)
{
  if(currentScreenIndex == -1)
    return;

  TFTScreenInfo* currentScreenInfo = &(screens[currentScreenIndex]);
  currentScreenInfo->screen->onButtonPressed(this, button);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::onButtonReleased(int button)
{
  if(currentScreenIndex == -1)
    return;

  TFTScreenInfo* currentScreenInfo = &(screens[currentScreenIndex]);
  currentScreenInfo->screen->onButtonReleased(this, button);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::update()
{
  if(!tftDC)
  {
    return;
  }
  
  TFTBinding bnd = HardwareBinding->GetTFTBinding();
  if(!(bnd.DisplayType == 1 || bnd.DisplayType == 2))
  {
    return; // нет привязки
  }  
  
  if(currentScreenIndex == -1 && !switchTo) // ни разу не рисовали ещё ничего, исправляемся
  {
     switchToScreen("IDLE"); // переключаемся на стартовый экран, если ещё ни разу не показали ничего     
  }

  if(switchTo != NULL)
  {
      tftDC->fillScr(TFT_BACK_COLOR); // clear screen first      
      yield();
      currentScreenIndex = switchToIndex;
      switchTo->onActivate(this);
      switchTo->update(this);
      yield();
      switchTo->draw(this);
      yield();
      resetIdleTimer(); // сбрасываем таймер ничегонеделанья

      switchTo = NULL;
      switchToIndex = -1;
    return;
  }


  if(flags.isLCDOn)
  {
    if(millis() - idleTimer > bnd.OffDelay)
    {
      flags.isLCDOn = false;
      lcdOff();
    }
  }
  else
  {
    // LCD currently off, check the touch on screen
    if(tftTouch->dataAvailable())
    {
      tftTouch->read();
      while(tftTouch->dataAvailable())
      {
        yield();
      }
      lcdOn();
      resetIdleTimer();
      flags.isLCDOn = true;
    }
  }

  // обновляем текущий экран
  TFTScreenInfo* currentScreenInfo = &(screens[currentScreenIndex]);
  currentScreenInfo->screen->update(this);
  yield();
  
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::switchToScreen(AbstractTFTScreen* to)
{
  if(!tftDC)
  {
    return;  
  }
   // переключаемся на запрошенный экран
  for(size_t i=0;i<screens.size();i++)
  {
    TFTScreenInfo* si = &(screens[i]);
    if(si->screen == to)
    {
      switchTo = si->screen;
      switchToIndex = i;
      break;

    }
  } 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMenu::switchToScreen(const char* screenName)
{
  if(!tftDC)
  {
    return;
  }
  
  // переключаемся на запрошенный экран
  for(size_t i=0;i<screens.size();i++)
  {
    TFTScreenInfo* si = &(screens[i]);
    if(!strcmp(si->screenName,screenName))
    {
      switchTo = si->screen;
      switchToIndex = i;
      break;

    }
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
 void TFTMenu::resetIdleTimer()
{
  idleTimer = millis();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
KeyboardScreen* Keyboard;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
KeyboardScreen::KeyboardScreen() : AbstractTFTScreen()
{
  inputTarget = NULL;
  maxLen = 20;
  isRusInput = true;

  if(!TFTScreen->getDC())
  {
    return;
  }
  

  buttons = new UTFT_Buttons_Rus(TFTScreen->getDC(), TFTScreen->getTouch(),TFTScreen->getRusPrinter(),60);
  
  buttons->setTextFont(BigRusFont);
  buttons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
  buttons->setSymbolFont(Various_Symbols_32x32);

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
KeyboardScreen::~KeyboardScreen()
{
  for(size_t i=0;i<captions.size();i++)
  {
    delete captions[i];
  }
  for(size_t i=0;i<altCaptions.size();i++)
  {
    delete altCaptions[i];
  }
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::switchInput(bool redraw)
{
  isRusInput = !isRusInput;
  Vector<String*>* pVec = isRusInput ? &captions : &altCaptions;

  // у нас кнопки изменяемой клавиатуры начинаются с индекса 10
  size_t startIdx = 10;

  for(size_t i=startIdx;i<pVec->size();i++)
  {
    buttons->relabelButton(i,(*pVec)[i]->c_str(),redraw);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::setup(TFTMenu* dc)
{
  if(!dc->getDC())
  {
    return;
  }
  
  createKeyboard(dc);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::update(TFTMenu* menu)
{
  if(!menu->getDC())
  {
    return;
  }
  
    // тут обновляем внутреннее состояние
    // раз нас вызвали, то пока не нажмут кнопки - мы не выйдем, поэтому всегда сообщаем, что на экране что-то происходит
    menu->resetIdleTimer();

    // мигаем курсором
    static uint32_t cursorTimer = millis();
    if(millis() - cursorTimer > 500)
    {
      static bool cursorVisible = true;
      cursorVisible = !cursorVisible;

      redrawCursor(menu,cursorVisible);

      cursorTimer = millis();
    }
    
    // проверяем на перемещение курсора
    URTouch* touch = menu->getTouch();
    
    if(touch->dataAvailable())
    {
      touch->read();
      int   touch_x = touch->getX();
      int   touch_y = touch->getY();

      // проверяем на попадание в прямоугольную область ввода текста
      UTFT* dc = menu->getDC();
      dc->setFont(BigRusFont);
      
      int screenWidth = dc->getDisplayXSize();
      
      if(touch_x >= KBD_SPACING && touch_x <= (screenWidth - KBD_SPACING) && touch_y >= KBD_SPACING && touch_y <= (KBD_SPACING + KBD_BUTTON_HEIGHT))
      {
        #ifdef USE_BUZZER
          Buzzer.buzz();
        #endif
        // кликнули на области ввода, ждём отпускания тача
        while (touch->dataAvailable()) { yield(); }
        
        int fontWidth = dc->getFontXsize();

        // вычисляем, на какой символ приходится клик тачем
        int symbolNum = touch_x/fontWidth - 1;
        
        if(symbolNum < 0)
          symbolNum = 0;
          
        int valLen = menu->getRusPrinter()->utf8_strlen(inputVal.c_str());

        if(symbolNum > valLen)
          symbolNum = valLen;

        redrawCursor(menu,true);
        cursorPos = symbolNum;
        redrawCursor(menu,false);
      }
    } // if (touch->dataAvailable())
  
    int pressed_button = buttons->checkButtons(ButtonPressed, ButtonReleased);
    if(pressed_button != -1)
    {
      
       if(pressed_button == backspaceButton)
       {
        // удалить последний введённый символ
        drawValue(menu,true);
       }
       else
       if(pressed_button == okButton)
       {
          // закрыть всё нафик
          if(inputTarget)
          {
            inputTarget->onKeyboardInputResult(inputVal,true);
            inputVal = "";
          }
       }
        else
       if(pressed_button == switchButton)
       {
          // переключить раскладку
          switchInput(true);
       }
       else
       if(pressed_button == cancelButton)
       {
          // закрыть всё нафик
          if(inputTarget)
          {
            inputTarget->onKeyboardInputResult(inputVal,false);
            inputVal = "";
          }
       }
       else
       {
         // одна из кнопок клавиатуры, добавляем её текст к буферу, но - в позицию курсора!!!
         int oldLen = menu->getRusPrinter()->utf8_strlen(inputVal.c_str());
         const char* lbl = buttons->getLabel(pressed_button);
         
         if(!oldLen) // пустая строка
         {
          inputVal = lbl;
         }
         else
         if(oldLen < maxLen)
         {
            
            String buff;            
            const char* ptr = inputVal.c_str();
            
            for(int i=0;i<oldLen;i++)
            {
              unsigned char curChar = (unsigned char) *ptr;
              unsigned int charSz = utf8GetCharSize(curChar);
              for(byte k=0;k<charSz;k++) 
              {
                utf8Bytes[k] = *ptr++;
              }
              utf8Bytes[charSz] = '\0'; // добавляем завершающий 0
              
              if(i == cursorPos)
              {
                buff += lbl;
              }
              
              buff += utf8Bytes;
              
            } // for

            if(cursorPos >= oldLen)
              buff += lbl;

          inputVal = buff;
          
         } // if(oldLen < maxLen)
         

          int newLen = menu->getRusPrinter()->utf8_strlen(inputVal.c_str());

          if(newLen <= maxLen)
          {
            drawValue(menu);
                     
            if(newLen != oldLen)
            {
              redrawCursor(menu,true);
              cursorPos++;
              redrawCursor(menu,false);
            }
            
          }
          

         
       } // else одна из кнопок клавиатуры
    
    } // if(pressed_button != -1)
    
    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::draw(TFTMenu* menu)
{
  if(!menu->getDC())
  {
    return;
  }

  buttons->drawButtons(drawButtonsYield);

  UTFT* dc = menu->getDC();
  int screenWidth = dc->getDisplayXSize();
  dc->setColor(VGA_GRAY);
  dc->drawRoundRect(KBD_SPACING, KBD_SPACING, screenWidth-KBD_SPACING, KBD_SPACING + KBD_BUTTON_HEIGHT);

  drawValue(menu);
  redrawCursor(menu,false);
}
//--------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::redrawCursor(TFTMenu* menu, bool erase)
{
  UTFT* dc = menu->getDC();

  dc->setFont(BigRusFont);
  uint8_t fontHeight = dc->getFontYsize();
  uint8_t fontWidth = dc->getFontXsize();

  int top = KBD_SPACING + (KBD_BUTTON_HEIGHT - fontHeight)/2;
  int left = KBD_SPACING*2 + fontWidth*cursorPos;

  if(erase)
    dc->setColor(TFT_BACK_COLOR);
  else
    dc->setColor(TFT_FONT_COLOR);
  
  dc->fillRect(left,top,left+1,top+fontHeight);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::drawValue(TFTMenu* menu, bool deleteCharAtCursor)
{
  if(!inputVal.length())
    return;


   UTFT* dc = menu->getDC();

  if(deleteCharAtCursor)
  {
    // надо удалить символ слева от позиции курсора.

    String buff;
    int len = menu->getRusPrinter()->utf8_strlen(inputVal.c_str());
    const char* ptr = inputVal.c_str();
    
    for(int i=0;i<len;i++)
    {
      unsigned char curChar = (unsigned char) *ptr;
      unsigned int charSz = utf8GetCharSize(curChar);
      for(byte k=0;k<charSz;k++) 
      {
        utf8Bytes[k] = *ptr++;
      }
      utf8Bytes[charSz] = '\0'; // добавляем завершающий 0
      
      if(i != (cursorPos-1)) // игнорируем удаляемый символ
      {
        buff += utf8Bytes;
      }
      
    } // for
    
    buff += ' '; // маскируем последний символ для корректной перерисовки на экране
    inputVal = buff;

  }

  dc->setFont(BigRusFont);
  
  uint8_t fontHeight = dc->getFontYsize();

  dc->setColor(TFT_FONT_COLOR);
  dc->setBackColor(TFT_BACK_COLOR);

  int top = KBD_SPACING + (KBD_BUTTON_HEIGHT - fontHeight)/2;
  int left = KBD_SPACING*2;

  menu->getRusPrinter()->print(inputVal.c_str(),left,top);

  if(deleteCharAtCursor)
  {
    // если надо удалить символ слева от позиции курсора, то в этом случае у нас последний символ - пробел, и мы его удаляем
    inputVal.remove(inputVal.length()-1,1);

    redrawCursor(menu,true);

    cursorPos--;
    if(cursorPos < 0)
      cursorPos = 0;

    redrawCursor(menu,false);
  }
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::createKeyboard(TFTMenu* menu)
{
  buttons->deleteAllButtons();

  UTFT* dc = menu->getDC();
  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();  

  // создаём клавиатуру

  int colCounter = 0;
  int left = KBD_SPACING;
  int top = KBD_SPACING*2 + KBD_BUTTON_HEIGHT;

  // сперва у нас кнопки 0-9
  for(uint8_t i=0;i<10;i++)
  {
    char c = '0' + i;
    String* s = new String(c);
    captions.push_back(s);

    String* altS = new String(c);
    altCaptions.push_back(altS);    

    /*int addedBtn = */buttons->addButton(left, top, KBD_BUTTON_WIDTH, KBD_BUTTON_HEIGHT, s->c_str());
   // buttons->setButtonBackColor(addedBtn, VGA_GRAY);
   // buttons->setButtonFontColor(addedBtn, VGA_BLACK);
    
    left += KBD_BUTTON_WIDTH + KBD_SPACING;
    colCounter++;
    if(colCounter >= KBD_BUTTONS_IN_ROW)
    {
      colCounter = 0;
      left = KBD_SPACING;
      top += KBD_SPACING + KBD_BUTTON_HEIGHT;
    }
  }
  // затем - А-Я
  const char* letters[] = {
    "А", "Б", "В", "Г", "Д", "Е",
    "Ж", "З", "И", "Й", "К", "Л",
    "М", "Н", "О", "П", "Р", "С",
    "Т", "У", "Ф", "Х", "Ц", "Ч",
    "Ш", "Щ", "Ъ", "Ы", "Ь", "Э",
    "Ю", "Я", NULL
  };

  const char* altLetters[] = {
    "A", "B", "C", "D", "E", "F",
    "G", "H", "I", "J", "K", "L",
    "M", "N", "O", "P", "Q", "R",
    "S", "T", "U", "V", "W", "X",
    "Y", "Z", ".", ",", ":", ";",
    "!", "?", NULL
  };  

  int lettersIterator = 0;
  while(letters[lettersIterator])
  {
    String* s = new String(letters[lettersIterator]);
    captions.push_back(s);

    String* altS = new String(altLetters[lettersIterator]);
    altCaptions.push_back(altS);

    buttons->addButton(left, top, KBD_BUTTON_WIDTH, KBD_BUTTON_HEIGHT, s->c_str());
    left += KBD_BUTTON_WIDTH + KBD_SPACING;
    colCounter++;
    if(colCounter >= KBD_BUTTONS_IN_ROW)
    {
      colCounter = 0;
      left = KBD_SPACING;
      top += KBD_SPACING + KBD_BUTTON_HEIGHT;
    } 

    lettersIterator++;
  }
  // затем - кнопка переключения ввода
    switchButton = buttons->addButton(left, top, KBD_BUTTON_WIDTH, KBD_BUTTON_HEIGHT, "q", BUTTON_SYMBOL);
    buttons->setButtonBackColor(switchButton, VGA_MAROON);
    buttons->setButtonFontColor(switchButton, VGA_WHITE);

    left += KBD_BUTTON_WIDTH + KBD_SPACING;
  
  // затем - пробел,
    spaceButton = buttons->addButton(left, top, KBD_BUTTON_WIDTH*5 + KBD_SPACING*4, KBD_BUTTON_HEIGHT, " ");
    //buttons->setButtonBackColor(spaceButton, VGA_GRAY);
    //buttons->setButtonFontColor(spaceButton, VGA_BLACK);
       
    left += KBD_BUTTON_WIDTH*5 + KBD_SPACING*5;
   
  // backspace, 
    backspaceButton = buttons->addButton(left, top, KBD_BUTTON_WIDTH*2 + KBD_SPACING, KBD_BUTTON_HEIGHT, ":", BUTTON_SYMBOL);
    buttons->setButtonBackColor(backspaceButton, VGA_MAROON);
    buttons->setButtonFontColor(backspaceButton, VGA_WHITE);

    left = KBD_SPACING;
    top = screenHeight - KDB_BIG_BUTTON_HEIGHT - KBD_SPACING;
   
  // OK,
    int okCancelButtonWidth = (screenWidth - KBD_SPACING*3)/2;
    okButton = buttons->addButton(left, top, okCancelButtonWidth, KDB_BIG_BUTTON_HEIGHT, "OK");
    left += okCancelButtonWidth + KBD_SPACING;
  
  // CANCEL
    cancelButton = buttons->addButton(left, top, okCancelButtonWidth, KDB_BIG_BUTTON_HEIGHT, "ОТМЕНА");

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::applyType(KeyboardType keyboardType)
{
  if(ktFull == keyboardType)
  {
    buttons->enableButton(spaceButton);
    buttons->enableButton(switchButton);

    // включаем все кнопки
    // у нас кнопки изменяемой клавиатуры начинаются с индекса 10
    size_t startIdx = 10;
  
    for(size_t i=startIdx;i<altCaptions.size();i++)
    {
      buttons->enableButton(i);
    }    

    isRusInput = false;
    switchInput(false);

    return;
  }

  if(ktNumbers == keyboardType)
  {
    buttons->disableButton(spaceButton);
    buttons->disableButton(switchButton);

    // выключаем все кнопки, кроме номеров и точки
    // у нас кнопки изменяемой клавиатуры начинаются с индекса 10
    size_t startIdx = 10;
  
    for(size_t i=startIdx;i<altCaptions.size();i++)
    {
      if(*(altCaptions[i]) != ".")
        buttons->disableButton(i);
    }        

    isRusInput = true;
    switchInput(false);

    return;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void KeyboardScreen::show(const String& val, int ml, KeyboardInputTarget* it, KeyboardType keyboardType, bool eng)
{
  if(!TFTScreen->getDC())
  {
    return;
  }
  
  inputVal = val;
  inputTarget = it;
  maxLen = ml;

  cursorPos = TFTScreen->getRusPrinter()->utf8_strlen(inputVal.c_str());

  applyType(keyboardType);

  if(eng && isRusInput)
    switchInput(false);

  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AbstractTFTScreen* KeyboardScreen::create()
{
    Keyboard = new KeyboardScreen();
    return Keyboard;  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
MessageBoxScreen* MessageBox;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
MessageBoxScreen::MessageBoxScreen() : AbstractTFTScreen()
{
  targetOkScreen = NULL;
  targetCancelScreen = NULL;
  resultSubscriber = NULL;
  caption = NULL;

  if(!TFTScreen->getDC())
  {
    return;
  }
  

  buttons = new UTFT_Buttons_Rus(TFTScreen->getDC(), TFTScreen->getTouch(),TFTScreen->getRusPrinter());
  
  buttons->setTextFont(BigRusFont);
  buttons->setButtonColors(TFT_CHANNELS_BUTTON_COLORS);
   
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::setup(TFTMenu* dc)
{
  if(!dc->getDC())
  {
    return;
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::update(TFTMenu* dc)
{
  if(!dc->getDC())
  {
    return;
  }
  
    // тут обновляем внутреннее состояние    
 
    int pressed_button = buttons->checkButtons(ButtonPressed, ButtonReleased);
    if(pressed_button != -1)
    {
      // сообщаем, что у нас нажата кнопка
      dc->resetIdleTimer();
      
       if(pressed_button == noButton && targetCancelScreen)
       {
        if(resultSubscriber)
          resultSubscriber->onMessageBoxResult(false);
          
        dc->switchToScreen(targetCancelScreen);
       }
       else
       if(pressed_button == yesButton && targetOkScreen)
       {
          if(resultSubscriber)
            resultSubscriber->onMessageBoxResult(true);
            
            dc->switchToScreen(targetOkScreen);
       }
    
    } // if(pressed_button != -1)

    
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::draw(TFTMenu* hal)
{
  UTFT* dc = hal->getDC();

  if(!dc)
  {
    return;
  }
  
  
  dc->setFont(BigRusFont);
  UTFTRus* rusPrinter = hal->getRusPrinter();
  
  uint8_t fontHeight = dc->getFontYsize();
  uint8_t fontWidth = dc->getFontXsize();
  
  int displayWidth = dc->getDisplayXSize();
  int displayHeight = dc->getDisplayYSize();
  
  int lineSpacing = 6; 
  int topOffset = 10;
  int curX = 0;
  int curY = topOffset;

  int lineLength = 0;

  // подложка под заголовок
  if(boxType == mbHalt && errorColors)
  {
    dc->setColor(VGA_RED);
  }
  else
  {
    dc->setColor(VGA_NAVY);
  }
    
  dc->fillRect(0, 0, displayWidth-1, topOffset*2 + fontHeight);
  
  if(caption)
  {
    if(boxType == mbHalt && errorColors)
    {
      dc->setBackColor(VGA_RED);
      dc->setColor(VGA_WHITE);
    }
    else
    {
      dc->setBackColor(VGA_NAVY);
      dc->setColor(VGA_WHITE);      
    }
    lineLength = rusPrinter->utf8_strlen(caption);
    curX = (displayWidth - lineLength*fontWidth)/2; 
    rusPrinter->print(caption,curX,curY);
  }

  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(TFT_FONT_COLOR);

  curY = (displayHeight - ALL_CHANNELS_BUTTON_HEIGHT - (lines.size()*fontHeight + (lines.size()-1)*lineSpacing))/2;

  for(size_t i=0;i<lines.size();i++)
  {
    lineLength = rusPrinter->utf8_strlen(lines[i]);
    curX = (displayWidth - lineLength*fontWidth)/2;    
    rusPrinter->print(lines[i],curX,curY);
    curY += fontHeight + lineSpacing;
  }

  buttons->drawButtons(drawButtonsYield);

  if(boxType == mbHalt && haltInWhile)
  {
    while(1)
    {
      #ifdef USE_EXTERNAL_WATCHDOG
        updateExternalWatchdog();
      #endif
    }
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::recreateButtons()
{
  buttons->deleteAllButtons();
  yesButton = -1;
  noButton = -1;
  
  UTFT* dc = TFTScreen->getDC();
  
  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();
  int buttonsWidth = 200;

  int numOfButtons = boxType == mbShow ? 1 : 2;

  int top = screenHeight - ALL_CHANNELS_BUTTON_HEIGHT - INFO_BOX_V_SPACING;
  int left = (screenWidth - (buttonsWidth*numOfButtons + INFO_BOX_V_SPACING*(numOfButtons-1)))/2;
  
  yesButton = buttons->addButton(left, top, buttonsWidth, ALL_CHANNELS_BUTTON_HEIGHT, boxType == mbShow ? "OK" : "ДА");

  if(boxType == mbConfirm)
  {
    left += buttonsWidth + INFO_BOX_V_SPACING;
    noButton = buttons->addButton(left, top, buttonsWidth, ALL_CHANNELS_BUTTON_HEIGHT, "НЕТ");  
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::halt(const char* _caption, Vector<const char*>& _lines, bool _errorColors, bool _haltInWhile)
{
  if(!TFTScreen->getDC())
  {
    return;
  }
  
  lines = _lines;
  caption = _caption;
  boxType = mbHalt;
  errorColors = _errorColors;
  haltInWhile = _haltInWhile;

  buttons->deleteAllButtons();
  yesButton = -1;
  noButton = -1;
    
  targetOkScreen = NULL;
  targetCancelScreen = NULL;
  resultSubscriber = NULL;  

  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::show(const char* _caption, Vector<const char*>& _lines, AbstractTFTScreen* okTarget, MessageBoxResultSubscriber* sub)
{
  if(!TFTScreen->getDC())
  {
    return;
  }
  
  lines = _lines;
  caption = _caption;
  errorColors = false;

  boxType = mbShow;
  recreateButtons();
    
  targetOkScreen = okTarget;
  targetCancelScreen = NULL;
  resultSubscriber = sub;

  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void MessageBoxScreen::confirm(const char* _caption, Vector<const char*>& _lines, AbstractTFTScreen* okTarget, AbstractTFTScreen* cancelTarget, MessageBoxResultSubscriber* sub)
{
  if(!TFTScreen->getDC())
  {
    return;
  }
  
  lines = _lines;
  caption = _caption;
  errorColors = false;

  boxType = mbConfirm;
  recreateButtons();
  
  targetOkScreen = okTarget;
  targetCancelScreen = cancelTarget;
  resultSubscriber = sub;

  TFTScreen->switchToScreen(this);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
AbstractTFTScreen* MessageBoxScreen::create()
{
    MessageBox = new MessageBoxScreen();
    return MessageBox;  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTUnrKScreen::TFTUnrKScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTUnrKScreen::~TFTUnrKScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTUnrKScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTUnrKScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTUnrKScreen::draw(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  if(!dc)
  {
    return;
  }
  
  dc->setFont(BigRusFont);
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(VGA_RED);

  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();

  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();


  String line1 = F("ОБНАРУЖЕНА НЕЗАРЕГИСТРИРОВАННАЯ КОПИЯ!");
  String line2 = F("ДАЛЬНЕЙШАЯ РАБОТА НЕВОЗМОЖНА.");
  String spacer_line = F("");
  String line4 = F("ОБРАТИТЕСЬ К РАЗРАБОТЧИКАМ");
  String line5 = F("ДЛЯ РЕГИСТРАЦИИ ВАШЕЙ КОПИИ.");
  String line6 = F("Email: spywarrior@gmail.com");
  String line7 = F("Email: promavto@ntmp.ru");

  Vector<const char*> vec;
  vec.push_back(line1.c_str());
  vec.push_back(line2.c_str());
  vec.push_back(spacer_line.c_str());
  vec.push_back(line4.c_str());
  vec.push_back(line5.c_str());
  vec.push_back(spacer_line.c_str());
  vec.push_back(line6.c_str());
  vec.push_back(line7.c_str());

  const int LINE_V_SPACING = 20;

  int top = (screenHeight - (vec.size()*fontHeight + (vec.size()-1)*LINE_V_SPACING))/2;

  UTFTRus* rusPrinter = menuManager->getRusPrinter();

  for(size_t i=0;i<vec.size();i++)
  {
    if(i > 0)
    {
      dc->setColor(INFO_BOX_CAPTION_COLOR);
    }
    
      int len = rusPrinter->utf8_strlen(vec[i])*fontWidth;
      int left = (screenWidth - len)/2;

      rusPrinter->print(vec[i],left,top); 

      top += fontHeight + LINE_V_SPACING;
  } // for

  // отрисовали, зависаем :)
  while(1);
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFTMemInitScreen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTMemInitScreen::TFTMemInitScreen() : AbstractTFTScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TFTMemInitScreen::~TFTMemInitScreen()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMemInitScreen::setup(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMemInitScreen::update(TFTMenu* menuManager)
{
  if(!menuManager->getDC())
  {
    return;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TFTMemInitScreen::draw(TFTMenu* menuManager)
{
  UTFT* dc = menuManager->getDC();
  if(!dc)
  {
    return;
  }
  
  dc->setFont(BigRusFont);
  dc->setBackColor(TFT_BACK_COLOR);
  dc->setColor(INFO_BOX_CAPTION_COLOR);

  int screenWidth = dc->getDisplayXSize();
  int screenHeight = dc->getDisplayYSize();

  int fontWidth = dc->getFontXsize();
  int fontHeight = dc->getFontYsize();

  String line1 = F("ИДЕТ ИНИЦИАЛИЗАЦИЯ EEPROM!");
  String line2 = F("НЕ ВЫКЛЮЧАЙТЕ КОНТРОЛЛЕР!");
  String spacer_line = F("");
  String line4 = F("ИНИЦИАЛИЗАЦИЯ ЗАВЕРШЕНА.");
  String line5 = F("ПЕРЕЗАГРУЗИТЕ КОНТРОЛЛЕР.");

  Vector<const char*> vec;
  vec.push_back(line1.c_str());
  vec.push_back(line2.c_str());
  vec.push_back(spacer_line.c_str());
  vec.push_back(line4.c_str());
  vec.push_back(line5.c_str());

  size_t callMemClearOn = 3;

  Serial.println(F("Start EEPROM clearance..."));

  const int LINE_V_SPACING = 20;

  int top = (screenHeight - (vec.size()*fontHeight + (vec.size()-1)*LINE_V_SPACING))/2;

  UTFTRus* rusPrinter = menuManager->getRusPrinter();

  for(size_t i=0;i<vec.size();i++)
  { 

       if(callMemClearOn == i)
       {
          MemClear();
       }
    
      int len = rusPrinter->utf8_strlen(vec[i])*fontWidth;
      int left = (screenWidth - len)/2;

      rusPrinter->print(vec[i],left,top); 

      top += fontHeight + LINE_V_SPACING;
  } // for

  Serial.println(F("Please restart controller."));

  // отрисовали, очистили память, зависаем :)
  while(1)
  {
    #ifdef USE_EXTERNAL_WATCHDOG
      updateExternalWatchdog();
    #endif        
  }
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TickerClass
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TickerClass Ticker;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TickerClass::TickerClass()
{
  started = false;
  beforeStartTickInterval = 1000;
  tickInterval = 100;
  waitBefore = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
TickerClass::~TickerClass()
{
  stop();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TickerClass::setIntervals(uint16_t _beforeStartTickInterval,uint16_t _tickInterval)
{
  beforeStartTickInterval = _beforeStartTickInterval;
  tickInterval = _tickInterval;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TickerClass::start(ITickHandler* h)
{
  if(started)
    return;

  handler = h;

  timer = millis();
  waitBefore = true;
  started = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TickerClass::stop()
{
  if(!started)
    return;

  handler = NULL;

  started = false;
  waitBefore = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void TickerClass::tick()
{
  if(!started)
    return;

  uint32_t now = millis();

  if(waitBefore)
  {
    if(now - timer > beforeStartTickInterval)
    {
      waitBefore = false;
      timer = now;
      if(handler)
        handler->onTick();
    }
    return;
  }

  if(now - timer > tickInterval)
  {
    timer = now;
    if(handler)
      handler->onTick();
  }

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TFT_MODULE

