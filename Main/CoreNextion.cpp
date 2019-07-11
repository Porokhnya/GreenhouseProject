//--------------------------------------------------------------------------------------------------
#include "CoreNextion.h"
//--------------------------------------------------------------------------------------------------
#ifdef USE_NEXTION_MODULE
//--------------------------------------------------------------------------------------------------
extern "C" {
  static void __no_ON_NEXTION_BUTTON_TOUCH(Nextion& sender, uint8_t pageID,uint8_t buttonID,bool pressed) {}
  static void __no_ON_NEXTION_PAGE_ID_RECEIVED(Nextion& sender, uint8_t pageID) {}
  static void __no_ON_NEXTION_TOUCH(Nextion& sender,uint16_t x, uint16_t y, bool pressed, bool inSleep) {}
  static void __no_ON_NEXTION_STRING_RECEIVED(Nextion& sender, const char* str) {}
  static void __no_ON_NEXTION_NUMBER_RECEIVED(Nextion& sender, uint32_t num) {}
  static void __no_ON_NEXTION_SLEEP(Nextion& sender, bool isSleep) {}
}
//--------------------------------------------------------------------------------------------------
void ON_NEXTION_BUTTON_TOUCH(Nextion& sender, uint8_t pageID,uint8_t buttonID,bool pressed) __attribute__ ((weak, alias("__no_ON_NEXTION_BUTTON_TOUCH")));
void ON_NEXTION_PAGE_ID_RECEIVED(Nextion& sender, uint8_t pageID) __attribute__ ((weak, alias("__no_ON_NEXTION_PAGE_ID_RECEIVED")));
void ON_NEXTION_TOUCH(Nextion& sender,uint16_t x, uint16_t y, bool pressed, bool inSleep) __attribute__ ((weak, alias("__no_ON_NEXTION_TOUCH")));
void ON_NEXTION_STRING_RECEIVED(Nextion& sender, const char* str) __attribute__ ((weak, alias("__no_ON_NEXTION_STRING_RECEIVED")));
void ON_NEXTION_NUMBER_RECEIVED(Nextion& sender, uint32_t num) __attribute__ ((weak, alias("__no_ON_NEXTION_NUMBER_RECEIVED")));
void ON_NEXTION_SLEEP(Nextion& sender, bool isSleep) __attribute__ ((weak, alias("__no_ON_NEXTION_SLEEP")));
//--------------------------------------------------------------------------------------------------
// NextionComponent
//--------------------------------------------------------------------------------------------------
NextionComponent::NextionComponent(const char* _name)
{
  name = _name;
  parent = NULL;

}
//--------------------------------------------------------------------------------------------------
NextionComponent::~NextionComponent()
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::bind(Nextion& _parent)
{
  parent = &_parent;
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::bind(Nextion* _parent)
{
  parent = _parent;
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::send(const char* command)
{
  if(!parent)
    return;

   parent->send(command);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pw(NextionInputType type)
{
  send(F2C("pw"), int32_t(type));  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_val(int32_t val)
{
  send(F2C("val"), val);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_format(NextionNumberFormat format)
{
  send(F2C("format"), int32_t(format));    
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_en(uint8_t en)
{
  send(F2C("en"), en);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_click(bool isReleaseEvent)
{
  send(F2C("click "));
  send(this->name);
  send(",");
  send(isReleaseEvent ? "0" : "1");
  sendDone();
  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_vis(uint8_t val)
{
  send(F2C("vis "));
  send(this->name);
  send(",");
  String s;
  s = val;
  send(s.c_str());
  sendDone();
    
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_gdc(uint16_t val)
{
  send(F2C("gdc"), val);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_gdw(uint16_t val)
{
  send(F2C("gdw"), val);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_gdh(uint16_t val)
{
  send(F2C("gdh"), val);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pconum(uint8_t val, uint16_t color)
{
  String num;
  num = val;
  
  send(this->name);
  send(F2C(".pco"));
  send(num.c_str());
  send("=");
  num = color;
  send(num.c_str());
  sendDone();    
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_waveformdata(uint8_t id, uint8_t channel, uint8_t val)
{
  String num;
  num = id;
  send(F2C("add "));
  send(num.c_str());
  send(",");
  num = channel;
  send(num.c_str());
  send(",");
  num = val;
  send(num.c_str());
  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_wid(uint8_t width)
{
   send(F2C("width"), width);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pic(uint16_t number)
{    
  send(F2C("pic"), number); 
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_xcen(NextionTextAlignment alignment)
{
  send(F2C("xcen"), int32_t(alignment));  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_ycen(NextionTextAlignment alignment)
{
 send(F2C("ycen"), int32_t(alignment));
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_font(uint8_t fontNumber)
{
  send(F2C("font"), fontNumber);  
  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_isbr(bool wrap)
{
  send(F2C("isbr"), wrap ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_bco(uint16_t color)
{
  send(F2C("bco"), color);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pco(uint16_t color)
{
  send(F2C("pco"), color);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_txt(const char* value)
{
  send(this->name);
  send(F2C(".txt=\""));
  send(value);
  send(F2C("\""));
  sendDone();  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_picc(uint16_t number)
{
  send(F2C("picc"), number);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_bco2(uint16_t color)
{
  send(F2C("bc02"), color); 
  
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pco2(uint16_t color)
{
 send(F2C("pco2"), color);   
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_picc2(uint16_t number)
{
 send(F2C("picc2"), number);   
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pic2(uint16_t number)
{
 send(F2C("pic2"), number);    
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_hig(uint8_t height)
{
 send(F2C("hig"), height);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_maxval(uint16_t val)
{
 send(F2C("maxval"), val);   
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_minval(uint16_t val)
{
 send(F2C("minval"), val);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_bco0(uint16_t color)
{
 send(F2C("bco0"), color);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_bco1(uint16_t color)
{
  send(F2C("bco1"), color);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_picc0(uint16_t num)
{
  send(F2C("picc0"), num);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_picc1(uint16_t num)
{
  send(F2C("picc1"), num);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pic0(uint16_t num)
{
  send(F2C("pic0"), num);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_pic1(uint16_t num)
{
  send(F2C("pic1"), num);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_tim(uint16_t interval)
{
  send(F2C("tim"), interval);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_bpic(uint16_t number)
{
  send(F2C("bpic"), number);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_ppic(uint16_t number)
{
  send(F2C("ppic"), number);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_dir(NextionScrollingDirection direction)
{
  send(F2C("dir"), int32_t(direction));
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::command_dis(uint8_t distance)
{
  send(F2C("dis"), distance);
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::send(const char* cmd,int32_t param)
{
  String strParam;
  strParam = param;
  
  send(this->name);
  send(".");
  send(cmd);
  send("=");
  send(strParam.c_str());
  sendDone();
}
//--------------------------------------------------------------------------------------------------
void NextionComponent::sendDone()
{
  if(!parent)
    return;

  parent->sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
// Nextion
//--------------------------------------------------------------------------------------------------
Nextion::Nextion(Stream* s)
{
  stream = s;
}
//--------------------------------------------------------------------------------------------------
Nextion::Nextion(Stream& s)
{
  stream = &s;
}
//--------------------------------------------------------------------------------------------------
void Nextion::sleepDelay(uint8_t seconds)
{
  String num;
  num = seconds;
  send(F2C("thsp="));
  send(num.c_str());
  sendEndOfPacket(); 
}
//--------------------------------------------------------------------------------------------------
void Nextion::wakeOnTouch(bool awake)
{
  send(F2C("thup="));
  send(awake ? "1" : "0");
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::baudRate(uint16_t baud, bool setAsDefault)
{
  send(F2C("baud"));
  if(setAsDefault)
    send("s");

  send("=");
  String num;
  num = baud;
  send(num.c_str());

  sendEndOfPacket();
  
}
//--------------------------------------------------------------------------------------------------
void Nextion::brightness(uint8_t bright, bool setAsDefault)
{
  send(F2C("dim"));
  if(setAsDefault)
    send("s");

  send("=");
  String num;
  num = bright;
  send(num.c_str());

  sendEndOfPacket();  
}
//--------------------------------------------------------------------------------------------------
void Nextion::fontXSpacing(uint8_t spacing)
{
  send(F2C("spax="));
  String num;
  num = spacing;
  send(num.c_str());
  sendEndOfPacket(); 
  
}
//--------------------------------------------------------------------------------------------------
void Nextion::fontYSpacing(uint8_t spacing)
{
  send(F2C("spay="));
  String num;
  num = spacing;
  send(num.c_str());
  sendEndOfPacket(); 
  
}
//--------------------------------------------------------------------------------------------------
void Nextion::receiveTouchEvents(bool receive)
{
  send(F2C("sendxy="));
  send(receive ? "1" : "0");
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::sleep(bool enterSleep)
{
  send(F2C("sleep="));
  send(enterSleep ? "1" : "0");
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::page(uint8_t pageNum)
{
  send(F2C("page "));
  String num;
  num = pageNum;
  send(num.c_str());
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::sysVariable(uint8_t sysVarNumber,uint32_t val)
{
  send(F2C("sys"));
  String num;
  num = sysVarNumber;
  send(num.c_str());
  send("=");
  num = val;
  send(num.c_str());
  sendEndOfPacket();
  
}
//--------------------------------------------------------------------------------------------------
void Nextion::stopRefresh()
{
  send(F2C("ref_stop"));
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::startRefresh()
{
  send(F2C("ref_star"));
  sendEndOfPacket();
}
//--------------------------------------------------------------------------------------------------
void Nextion::begin()
{
  recvBuff = ""; 
}
//--------------------------------------------------------------------------------------------------
void Nextion::update()
{
  recvAnswer(); // вычитываем ответ от Nextion
}
//--------------------------------------------------------------------------------------------------
void Nextion::sendEndOfPacket()
{
  recvAnswer(); // вычитываем ответ от Nextion
  
  if(!stream)
    return; 

  uint8_t packet_end[3] = {0xFF,0xFF,0xFF};  
  stream->write(packet_end,sizeof(packet_end));

}
//--------------------------------------------------------------------------------------------------
void Nextion::recvAnswer()
{
  if(!stream)
    return;

  // получаем ответ от Nextion
  char ch;
  while(stream->available())
  {
    if(gotCommand()) // если уже есть команда
      processCommand(); // обрабатываем её

      ch = stream->read();
      recvBuff += ch;
  }

    if(gotCommand()) // если уже есть команда
      processCommand(); // обрабатываем её
  
}
//--------------------------------------------------------------------------------------------------
bool Nextion::gotCommand()
{
  int len = recvBuff.length();
  return (len > 3
  && 0xFF ==(uint8_t) recvBuff[len-1]
  && 0xFF ==(uint8_t) recvBuff[len-2]
  && 0xFF ==(uint8_t) recvBuff[len-3]
  );
  
}
//--------------------------------------------------------------------------------------------------
void Nextion::processCommand()
{
  uint8_t commandType = (uint8_t) recvBuff[0];
  switch(commandType)
  {
    case 0x00: // ret invalid command
    {
    }
    break;
    
    case 0x01: // ret command finished
    {
    }
    break;

    case 0x02: // ret invalid component id
    {
    }
    break;

    case 0x03: // ret invalid page id
    {
    }
    break;

    case 0x04: // ret invalid picture id
    {
    }
    break;

    case 0x05: // ret invalid font id
    {      
    }
    break;

    case 0x11: // ret invalid baud
    {
    }
    break;

    case 0x65: // ret event touch
    {
      // что лежит в буфере:
      // 0X65+Page ID+Component ID+TouchEvent+End 
      uint8_t pageID = (uint8_t) recvBuff[1];
      uint8_t buttonID = (uint8_t) recvBuff[2];
      bool pressed = 1 == (uint8_t) recvBuff[3];

      recvBuff = ""; // очищаем буфер
      ON_NEXTION_BUTTON_TOUCH(*this,pageID,buttonID,pressed);
    }
    break;

    case 0x66: // ret current page id
    {
      // что лежит в буфере:
      // 0X66+Page ID+End
      uint8_t pageID = (uint8_t) recvBuff[1];

      recvBuff = ""; // очищаем буфер
      ON_NEXTION_PAGE_ID_RECEIVED(*this,pageID);
      
    }
    break;

    case 0x67: // ret event position
    case 0x68: // ret event sleep
    {
      // что лежит в буфере:
      // 0X67+ Coordinate X High-order+Coordinate X Low-order+Coordinate Y High-order+Coordinate Y Low-order+TouchEvent State+End
      uint16_t x = ((uint16_t) recvBuff[1] << 8) | ((uint8_t) recvBuff[2]);
      uint16_t y = ((uint16_t) recvBuff[3] << 8) | ((uint8_t) recvBuff[4]);
      bool pressed = 1 == (uint8_t) recvBuff[5];
      bool inSleep = commandType == 0x68;

      recvBuff = ""; // очищаем буфер
      ON_NEXTION_TOUCH(*this,x,y,pressed,inSleep);
      
    }
    break;

    
    case 0x70: // ret string
    {
      recvBuff[recvBuff.length()-3] = '\0'; // маскируем конец пакета
      const char* ptr = recvBuff.c_str();
      ptr++; // переходим на начало данных

      String cpy = ptr;
      recvBuff = ""; // очищаем буфер
      ON_NEXTION_STRING_RECEIVED(*this,cpy.c_str());
    }
    break;

    case 0x71: // ret number
    {
      // что лежит в буфере:
      // 0x71, b1,b2,b3,b4,0xFF,0xFF,0xFF
      int readIdx = recvBuff.length()-4; // переходим на первый значащий байт
      uint32_t num = ((uint32_t)recvBuff[readIdx] << 24) | ((uint32_t)recvBuff[readIdx-1] << 16) | ((uint32_t)recvBuff[readIdx-2] << 8) | recvBuff[readIdx-3];

      recvBuff = ""; // очищаем буфер
      ON_NEXTION_NUMBER_RECEIVED(*this,num);
      
    }
    break;

    case 0x86: // enter sleep mode
    {
      recvBuff = ""; // очищаем буфер
      ON_NEXTION_SLEEP(*this,true);
    }
    break;

    case 0x87: // wake up
    {
      recvBuff = ""; // очищаем буфер
      ON_NEXTION_SLEEP(*this,false);
    }
    break;

    case 0x88: // ret event launched
    {
    }
    break;

    case 0x89: // ret event upgraded
    {
    }
    break;
    
  } // switch

  recvBuff = ""; // очищаем буфер
}
//--------------------------------------------------------------------------------------------------
void Nextion::send(const char* command)
{
  recvAnswer(); // вычитываем ответ от Nextion
  
  if(!stream)
    return;

  stream->write(command,strlen(command));     

}
//--------------------------------------------------------------------------------------------------
// COMPONENTS
//--------------------------------------------------------------------------------------------------
// NextionGUIComponent
//--------------------------------------------------------------------------------------------------
NextionGUIComponent::NextionGUIComponent(const char* name) : NextionComponent(name)
{
}
//--------------------------------------------------------------------------------------------------
void NextionGUIComponent::click(bool isReleaseEvent)
{
  command_click(isReleaseEvent);
}
//--------------------------------------------------------------------------------------------------
void NextionGUIComponent::show()
{
  command_vis(1);
}
//--------------------------------------------------------------------------------------------------
void NextionGUIComponent::hide()
{
 command_vis(0);
}
//--------------------------------------------------------------------------------------------------
// NextionText
//--------------------------------------------------------------------------------------------------
NextionText::NextionText(const char* name) : NextionComponent(name), NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionText::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::fontColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::font(uint16_t val)
{
  command_font(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::horizontalAlignment(NextionTextAlignment alignment)
{
  command_xcen(alignment);
}
//--------------------------------------------------------------------------------------------------
void NextionText::verticalAlignment(NextionTextAlignment alignment)
{
  command_ycen(alignment);
}
//--------------------------------------------------------------------------------------------------
void NextionText::inputType(NextionInputType type)
{
  command_pw(type);
}
//--------------------------------------------------------------------------------------------------
void NextionText::text(const char* val)
{
  command_txt(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::wordWrap(bool val)
{
  command_isbr(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionText::backgroundImage(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionScrollingText
//--------------------------------------------------------------------------------------------------
NextionScrollingText::NextionScrollingText(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name),
NextionText(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionScrollingText::direction(NextionScrollingDirection val)
{
  command_dir(val);
}
//--------------------------------------------------------------------------------------------------
void NextionScrollingText::interval(uint16_t val)
{
  command_tim(val);
}
//--------------------------------------------------------------------------------------------------
void NextionScrollingText::distance(uint8_t val)
{
  command_dis(val);
}
//--------------------------------------------------------------------------------------------------
void NextionScrollingText::enable()
{
  command_en(1);
}
//--------------------------------------------------------------------------------------------------
void NextionScrollingText::disable()
{
  command_en(0);
}
//--------------------------------------------------------------------------------------------------
// NextionNumber
//--------------------------------------------------------------------------------------------------
NextionNumber::NextionNumber(const char* name) :
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::fontColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::font(uint16_t val)
{
  command_font(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::horizontalAlignment(NextionTextAlignment alignment)
{
  command_xcen(alignment);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::verticalAlignment(NextionTextAlignment alignment)
{
  command_ycen(alignment);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::value(int32_t val)
{
  command_val(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::format(NextionNumberFormat val)
{
  command_format(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::wordWrap(bool val)
{
  command_isbr(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionNumber::backgroundImage(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionButton
//--------------------------------------------------------------------------------------------------
NextionButton::NextionButton(const char* name) :
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundColor(uint16_t val)
{
  command_bco(val); 
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundPressedColor(uint16_t val)
{
  command_bco2(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::fontColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::fontPressedColor(uint16_t val)
{
  command_pco2(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::font(uint16_t val)
{
  command_font(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::horizontalAlignment(NextionTextAlignment val)
{
  command_xcen(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::verticalAlignment(NextionTextAlignment val)
{
  command_ycen(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::text(const char* val)
{
  command_txt(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::wordWrap(bool val)
{
  command_isbr(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundCropPressedImage(uint16_t val)
{
  command_picc2(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundImage(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
void NextionButton::backgroundPressedImage(uint16_t val)
{
  command_pic2(val);
}
//--------------------------------------------------------------------------------------------------
// NextionProgressBar
//--------------------------------------------------------------------------------------------------
NextionProgressBar::NextionProgressBar(const char* name) :
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionProgressBar::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionProgressBar::foregroundColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionProgressBar::value(uint8_t val)
{
  command_val(val);
}
//--------------------------------------------------------------------------------------------------
void NextionProgressBar::backgroundImage(uint16_t val)
{
  command_bpic(val);
}
//--------------------------------------------------------------------------------------------------
void NextionProgressBar::foregroundImage(uint16_t val)
{
  command_ppic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionPicture
//--------------------------------------------------------------------------------------------------
NextionPicture::NextionPicture(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionPicture::image(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionCrop
//--------------------------------------------------------------------------------------------------
NextionCrop::NextionCrop(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionCrop::image(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
// NextionGauge
//--------------------------------------------------------------------------------------------------
NextionGauge::NextionGauge(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionGauge::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionGauge::value(uint16_t val)
{
  if(val > 360)
    val = 360;

  command_val(val);
}
//--------------------------------------------------------------------------------------------------
void NextionGauge::thickness(uint8_t val)
{
  if(val > 5)
    val = 5;

  command_wid(val);
}
//--------------------------------------------------------------------------------------------------
void NextionGauge::pointerColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionGauge::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
// NextionSlider
//--------------------------------------------------------------------------------------------------
NextionSlider::NextionSlider(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::backgroundColor(uint16_t val)
{
  command_bco(val); 
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::cursorColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::cursorWidth(uint16_t val)
{
  command_wid(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::cursorHeight(uint16_t val)
{
  command_hig(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::cursorImage(uint16_t val)
{
  command_pic2(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::value(uint16_t val)
{
  command_val(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::maxValue(uint16_t val)
{
  command_maxval(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::minValue(uint16_t val)
{
  command_minval(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionSlider::backgroundImage(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionTimer
//--------------------------------------------------------------------------------------------------
NextionTimer::NextionTimer(const char* name) :
NextionComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionTimer::interval(uint16_t val)
{
  if(val < 50)
    val = 50;
    
  command_tim(val);
}
//--------------------------------------------------------------------------------------------------
void NextionTimer::enable()
{
  command_en(1);
}
//--------------------------------------------------------------------------------------------------
void NextionTimer::disable()
{
  command_en(0);
}
//--------------------------------------------------------------------------------------------------
// NextionNumberVariable
//--------------------------------------------------------------------------------------------------
NextionNumberVariable::NextionNumberVariable(const char* name) : 
NextionComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionNumberVariable::value(uint16_t val)
{
  command_val(val);
}
//--------------------------------------------------------------------------------------------------
// NextionStringVariable
//--------------------------------------------------------------------------------------------------
NextionStringVariable::NextionStringVariable(const char* name) : 
NextionComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionStringVariable::text(const char* val)
{
  command_txt(val);
}
//--------------------------------------------------------------------------------------------------
// NextionDualStateButton
//--------------------------------------------------------------------------------------------------
NextionDualStateButton::NextionDualStateButton(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state0BackgroundColor(uint16_t val)
{
  command_bco0(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state1BackgroundColor(uint16_t val)
{
  command_bco1(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::fontColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::font(uint16_t val)
{
  command_font(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::horizontalAlignment(NextionTextAlignment val)
{
  command_xcen(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::verticalAlignment(NextionTextAlignment val)
{
  command_ycen(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::value(uint8_t val)
{
  if(!(val == 0 || val == 1))
    val = 0;

    command_val(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::text(const char* text)
{
  command_txt(text);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::wordWrap(bool val)
{
  command_isbr(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state0CropImage(uint16_t val)
{
  command_picc0(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state1CropImage(uint16_t val)
{
  command_picc1(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state0Image(uint16_t val)
{
  command_pic0(val);
}
//--------------------------------------------------------------------------------------------------
void NextionDualStateButton::state1Image(uint16_t val)
{
  command_pic1(val);
}
//--------------------------------------------------------------------------------------------------
// NextionCheckboxRadio
//--------------------------------------------------------------------------------------------------
NextionCheckboxRadio::NextionCheckboxRadio(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionCheckboxRadio::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionCheckboxRadio::foregroundColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionCheckboxRadio::value(uint8_t val)
{
  if(!(val == 0 || val == 1))
    val = 0;

    command_val(val);
}
//--------------------------------------------------------------------------------------------------
// NextionQRCode
//--------------------------------------------------------------------------------------------------
NextionQRCode::NextionQRCode(const char* name) :
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionQRCode::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionQRCode::foregroundColor(uint16_t val)
{
  command_pco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionQRCode::text(const char* val)
{
  command_txt(val);
}
//--------------------------------------------------------------------------------------------------
void NextionQRCode::image(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
// NextionWaveform
//--------------------------------------------------------------------------------------------------
NextionWaveform::NextionWaveform(const char* name) : 
NextionComponent(name),
NextionGUIComponent(name)
{
  
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::backgroundColor(uint16_t val)
{
  command_bco(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::gridColor(uint16_t val)
{
  command_gdc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::gridWidth(uint16_t val)
{
  command_gdw(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::gridHeight(uint16_t val)
{
  command_gdh(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::channelColor(uint8_t channelNumber,uint16_t val)
{
  if(channelNumber > 3)
    channelNumber = 3;
    
  command_pconum(channelNumber,val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::backgroundCropImage(uint16_t val)
{
  command_picc(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::backgroundImage(uint16_t val)
{
  command_pic(val);
}
//--------------------------------------------------------------------------------------------------
void NextionWaveform::data(uint8_t id,uint8_t channel,uint8_t val)
{
  if(channel > 3)
    channel = 3;
    
  command_waveformdata(id,channel,val);
}
//--------------------------------------------------------------------------------------------------
#endif // USE_NEXTION_MODULE
