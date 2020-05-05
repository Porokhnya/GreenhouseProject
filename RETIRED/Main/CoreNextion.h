#pragma once
//--------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "Globals.h"
//--------------------------------------------------------------------------------------------------
#ifdef USE_NEXTION_MODULE
//--------------------------------------------------------------------------------------------------
class Nextion; // forward declaration
//--------------------------------------------------------------------------------------------------
#define F2C(x) String(F((x))).c_str()
//--------------------------------------------------------------------------------------------------
extern "C" 
{
  void ON_NEXTION_BUTTON_TOUCH(Nextion& sender, uint8_t pageID,uint8_t buttonID,bool pressed);
  void ON_NEXTION_PAGE_ID_RECEIVED(Nextion& sender, uint8_t pageID);
  void ON_NEXTION_TOUCH(Nextion& sender,uint16_t x, uint16_t y, bool pressed, bool inSleep);
  void ON_NEXTION_STRING_RECEIVED(Nextion& sender, const char* str);
  void ON_NEXTION_NUMBER_RECEIVED(Nextion& sender, uint32_t num);
  void ON_NEXTION_SLEEP(Nextion& sender, bool isSleep);
}
//--------------------------------------------------------------------------------------------------
typedef enum
{
  Left,
  Center,
  Right
  
} NextionTextAlignment;
//--------------------------------------------------------------------------------------------------
typedef enum
{
  Character,
  Password
  
} NextionInputType;
//--------------------------------------------------------------------------------------------------
typedef enum
{
  LeftToRight,
  RightToLeft,
  UpToDown,
  DownToUp
  
} NextionScrollingDirection;
//--------------------------------------------------------------------------------------------------
typedef enum
{
  Decimal,
  Currency,
  Hex
} NextionNumberFormat;
//--------------------------------------------------------------------------------------------------
class NextionComponent
{
 public:
 
  NextionComponent(const char* name); 
  virtual ~NextionComponent();

  void bind(Nextion& parent);
  void bind(Nextion* parent);

  void setName(const char* _name)
  {
    name = _name;
  }

  const char* getName()
  {
    return name;
  }

  protected:
  
    const char* name;


    // pw
    void command_pw(NextionInputType type);
    
    // val
    void command_val(int32_t val);
    
    // format
    void command_format(NextionNumberFormat format);

    // en
    void command_en(uint8_t en);

    // click
    void command_click(bool isReleaseEvent=false); 

    // vis
    void command_vis(uint8_t val);

     // wid
    void command_wid(uint8_t width);

    // pic
    void command_pic(uint16_t number);

    // xcen
    void command_xcen(NextionTextAlignment alignment);

    // ycen
    void command_ycen(NextionTextAlignment alignment); 

    // font
    void command_font(uint8_t fontNumber);

    // isbr
    void command_isbr(bool wrap);

   // bco
    void command_bco(uint16_t color);
  
    // pco
    void command_pco(uint16_t color);
  
  
    // txt
    void command_txt(const char* value);
  
    // picc
    void command_picc(uint16_t number); 


    // bco2
    void command_bco2(uint16_t color);
  
    // pco2
    void command_pco2(uint16_t color);
  
    // picc2
    void command_picc2(uint16_t number);
  
    // pic2
    void command_pic2(uint16_t number);

   // hig
    void command_hig(uint8_t height);
  
    // maxval
    void command_maxval(uint16_t val);
  
    // minval
    void command_minval(uint16_t val);
  

 // bco0
  void command_bco0(uint16_t color);
  
  // bco1
  void command_bco1(uint16_t color);

  // picc0
  void command_picc0(uint16_t num);

  // picc1
  void command_picc1(uint16_t num);

  // pic0
  void command_pic0(uint16_t num);

  // pic1
  void command_pic1(uint16_t num);

  // tim
  void command_tim(uint16_t interval);

  // bpic
  void command_bpic(uint16_t number);

  // ppic
  void command_ppic(uint16_t number);

 // dir
  void command_dir(NextionScrollingDirection direction);

  // dis
  void command_dis(uint8_t distance);

  // waveform related
  void command_gdc(uint16_t val);
  void command_gdw(uint16_t val);
  void command_gdh(uint16_t val);
  void command_pconum(uint8_t val, uint16_t color);
  void command_waveformdata(uint8_t id, uint8_t channel, uint8_t val);
    

  private:
  
    Nextion* parent;

    void send(const char* command);
    void send(const char* command,int32_t param);
    void sendDone();
  
};
//--------------------------------------------------------------------------------------------------
class NextionGUIComponent : public virtual NextionComponent
{
  public:

    NextionGUIComponent(const char* name);

    // click
    void click(bool isReleaseEvent=false); 
  
    // vis
    void show();
  
    // vis
    void hide();    
  
};
//--------------------------------------------------------------------------------------------------
class NextionText : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionText(const char* name);

    void backgroundColor(uint16_t val);
    void fontColor(uint16_t val);
    void font(uint16_t val);
    void horizontalAlignment(NextionTextAlignment alignment);
    void verticalAlignment(NextionTextAlignment alignment);
    void inputType(NextionInputType type);
    void text(const char* val);
    void wordWrap(bool val);

    void backgroundCropImage(uint16_t val);
    void backgroundImage(uint16_t val); 
  
}; 
//--------------------------------------------------------------------------------------------------
class NextionScrollingText : public virtual NextionText
{
  public:
    NextionScrollingText(const char* name);

    void direction(NextionScrollingDirection val);
    void interval(uint16_t val);
    void distance(uint8_t val);
    void enable();
    void disable();
  
}; 
//--------------------------------------------------------------------------------------------------
class NextionNumber : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionNumber(const char* name);

    void backgroundColor(uint16_t val);
    void fontColor(uint16_t val);
    void font(uint16_t val);
    void horizontalAlignment(NextionTextAlignment alignment);
    void verticalAlignment(NextionTextAlignment alignment);
    void value(int32_t val);
    void format(NextionNumberFormat val);
    void wordWrap(bool val);
    void backgroundCropImage(uint16_t val);
    void backgroundImage(uint16_t val);
};
//--------------------------------------------------------------------------------------------------
class NextionButton : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionButton(const char* name);

    void backgroundColor(uint16_t val);
    void backgroundPressedColor(uint16_t val);
    
    void fontColor(uint16_t val);
    void fontPressedColor(uint16_t val);
    
    void font(uint16_t val);
    void horizontalAlignment(NextionTextAlignment val);
    void verticalAlignment(NextionTextAlignment val);
    
    void text(const char* val);

    void wordWrap(bool val);
    
    void backgroundCropImage(uint16_t val);
    void backgroundCropPressedImage(uint16_t val);
    
    void backgroundImage(uint16_t val);
    void backgroundPressedImage(uint16_t val);
};
//--------------------------------------------------------------------------------------------------
class NextionProgressBar : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionProgressBar(const char* name);

    void backgroundColor(uint16_t val);
    void foregroundColor(uint16_t val);

    void value(uint8_t val);

    void backgroundImage(uint16_t val);
    void foregroundImage(uint16_t val);
        
};
//--------------------------------------------------------------------------------------------------
class NextionPicture : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionPicture(const char* name);

    void image(uint16_t val);
        
};
//--------------------------------------------------------------------------------------------------
class NextionCrop : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionCrop(const char* name);

    void image(uint16_t val);
        
};
//--------------------------------------------------------------------------------------------------
class NextionGauge : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionGauge(const char* name);

   void backgroundColor(uint16_t val);
   void value(uint16_t val);
   void thickness(uint8_t val);
   void pointerColor(uint16_t val);
   void backgroundCropImage(uint16_t val);
        
};
//--------------------------------------------------------------------------------------------------
class NextionSlider : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionSlider(const char* name);

   void backgroundColor(uint16_t val);
   
   void cursorColor(uint16_t val);
   void cursorWidth(uint16_t val);
   void cursorHeight(uint16_t val);
   void cursorImage(uint16_t val);

   void value(uint16_t val);
   
   void maxValue(uint16_t val);
   void minValue(uint16_t val);

   void backgroundCropImage(uint16_t val);
   void backgroundImage(uint16_t val);

   
        
};
//--------------------------------------------------------------------------------------------------
class NextionTimer : public NextionComponent
{
  public:
    NextionTimer(const char* name);

   void interval(uint16_t val);
   
   void enable();
   void disable();
        
};
//--------------------------------------------------------------------------------------------------
class NextionNumberVariable : public NextionComponent
{
  public:
    NextionNumberVariable(const char* name);

   void value(uint16_t val);
           
};
//--------------------------------------------------------------------------------------------------
class NextionStringVariable : public NextionComponent
{
  public:
    NextionStringVariable(const char* name);

   void text(const char* val);
           
};
//--------------------------------------------------------------------------------------------------
class NextionDualStateButton : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionDualStateButton(const char* name);

   void state0BackgroundColor(uint16_t val);
   void state1BackgroundColor(uint16_t val);
   
   void fontColor(uint16_t val);
   void font(uint16_t val);

    void horizontalAlignment(NextionTextAlignment val);
    void verticalAlignment(NextionTextAlignment val);

    void value(uint8_t val);

    void text(const char* text);

    void wordWrap(bool val);

    void state0CropImage(uint16_t val);
    void state1CropImage(uint16_t val);

    void state0Image(uint16_t val);
    void state1Image(uint16_t val);
   
        
};
//--------------------------------------------------------------------------------------------------
class NextionCheckboxRadio : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionCheckboxRadio(const char* name);

   void backgroundColor(uint16_t val);
   void foregroundColor(uint16_t val);
   
    void value(uint8_t val);
   
        
};
//--------------------------------------------------------------------------------------------------
class NextionQRCode : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionQRCode(const char* name);

   void backgroundColor(uint16_t val);
   void foregroundColor(uint16_t val);

   void image(uint16_t val);
   
    void text(const char* val);
   
        
};
//--------------------------------------------------------------------------------------------------
class NextionWaveform : public virtual NextionComponent, public virtual NextionGUIComponent
{
  public:
    NextionWaveform(const char* name);

   void backgroundColor(uint16_t val);
   
   void gridColor(uint16_t val);
   void gridWidth(uint16_t val);
   void gridHeight(uint16_t val);

   void channelColor(uint8_t channelNumber,uint16_t val);
   
  void backgroundCropImage(uint16_t val);
  void backgroundImage(uint16_t val);

  void data(uint8_t id,uint8_t channel,uint8_t val);
   
        
};
//--------------------------------------------------------------------------------------------------
// Nextion
//--------------------------------------------------------------------------------------------------
class Nextion
{
  public:
    Nextion(Stream* s);
    Nextion(Stream& s);

    void begin();
    void update();

    // задержка перед уходом в сон
    void sleepDelay(uint8_t seconds);

    // просыпаться по тачу?
    void wakeOnTouch(bool awake);

    // скорость общения
    void baudRate(uint16_t baud, bool setAsDefault=false);

    // яркость дисплея
    void brightness(uint8_t bright, bool setAsDefault=false);

    // шрифты
    void fontXSpacing(uint8_t spacing);
    void fontYSpacing(uint8_t spacing);

    // принимать события тача?
    void receiveTouchEvents(bool receive);

    // уйти в сон или проснуться
    void sleep(bool enterSleep);

    // перейти на страницу
    void page(uint8_t pageNum);

    // установить значение системной переменной
    void sysVariable(uint8_t sysVarNumber,uint32_t val);

    // запретить обновлять экран
    void stopRefresh();

    // разрешить обновлять экран
    void startRefresh();

  protected:
  
    friend class NextionComponent;
    void send(const char* command);
    void sendEndOfPacket();

  private:
  
   Stream* stream;

   String recvBuff; // буфер для приёма команд
   void recvAnswer(); // получает ответ от Nextion
   bool gotCommand(); // проверяет, есть ли полная команда в буфере
   void processCommand(); // обрабатывает команду
    
};
//--------------------------------------------------------------------------------------------------
#endif // USE_NEXTION_MODULE

