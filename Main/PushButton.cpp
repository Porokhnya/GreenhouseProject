#include "PushButton.h"
#include "AbstractModule.h"

//--------------------------------------------------------------------------------------------------
PushButton::PushButton(uint8_t _pin)
{
  buttonPin = _pin;
  
  userData = NULL;
  OnClick = NULL;
  OnPress = NULL;
  OnDoubleClick = NULL;
  OnInactive = NULL;
  OnRetention = NULL;
  
  state.atLeastOneStateChangesFound = false;

  state.click_down      = false;
  state.click_up        = false;
  state.doubleclick     = false;
  state.timer           = false;
  state.retention       = false;

  state.clickCounter  =     0;
  
  state.lastBounce  =      false;
  state.lastDoubleClick =  false;
  state.lastTimer  =        false;
  state.lastRetention  =    false;
  
  lastMillis  =      millis();
}
//--------------------------------------------------------------------------------------------------
void PushButton::init(void* _userData  
    , PushButtonEvent _onClick
    , PushButtonEvent _onPress
    , PushButtonEvent _onDoubleClick
    , PushButtonEvent _onInactive
    , PushButtonEvent _onRetention)
{

  WORK_STATUS.PinMode(buttonPin, INPUT_PULLUP); // подтягиваем к питанию
  
  userData = _userData;
  OnClick = _onClick;
  OnPress = _onPress;
  OnDoubleClick = _onDoubleClick;
  OnInactive = _onInactive;
  OnRetention = _onRetention;
  
  state.lastButtonState  = readButtonState(buttonPin);
}
//--------------------------------------------------------------------------------------------------
void PushButton::update()
{

  // обновляем внутреннее состояние
  bool curBounce  = false;
  bool curDoubleClick = false;
  bool curTimer  = false;
  bool curRetention  = false;

  // сбрасываем все флаги
  state.click_down  = false;
  state.click_up    = false;
  state.doubleclick = false;
  state.timer       = false;
  state.retention   = false;


  unsigned long curMillis = millis();
  unsigned long millisDelta = curMillis - lastMillis;
  uint8_t curButtonState = readButtonState(buttonPin); // читаем текущее состояние

 if (curButtonState != state.lastButtonState) // состояние изменилось
  {
    state.atLeastOneStateChangesFound = true; // было хотя бы одно изменение в состоянии (нужно для того, чтобы не было события "clicked", когда кнопку не нажимали ни разу)
    state.lastButtonState = curButtonState; // сохраняем его
    lastMillis = curMillis; // и время последнего обновления
  }

  if (millisDelta > BOUNCE_INTERVAL)  // надо проверить на дребезг
    curBounce = true;

  if (millisDelta > DOUBLECLICK_INTERVAL) // надо проверить на даблклик
    curDoubleClick = true;

  if (curDoubleClick != state.lastDoubleClick) // состояние даблклика с момента последней проверки изменилось
  {
    state.lastDoubleClick = curDoubleClick; // сохраняем текущее
    if (state.lastDoubleClick) // проверяем - если кнопка не нажата, то сбрасываем счётчик нажатий 
      state.clickCounter = 0;
  }

  if (curBounce != state.lastBounce) // состояние проверки дребезга изменилось
  {
    state.lastBounce = curBounce; // сохраняем текущее

    if (!state.lastButtonState && curBounce) // если кнопка была нажата в момент последнего замера и сейчас - значит, дребезг прошёл и мы можем сохранять состояние
    {
      state.click_down = true; // выставляем флаг, что кнопка нажата

      if(OnPress)
        OnPress(*this,userData);
      
      ++state.clickCounter; // увеличиваем счётчик кликов
      
      if (state.clickCounter == 2) // если кликнули два раза
      {
        state.clickCounter = 0;  // сбрасываем счётчик кликов
        state.doubleclick = true; // и выставляем флаг двойного нажатия
        
        if(OnDoubleClick)
          OnDoubleClick(*this,userData);
      }
    }

    state.click_up = state.lastButtonState && state.lastBounce && state.atLeastOneStateChangesFound; // кнопка отпущена тогда, когда последний замер и текущий - равны 1 (пин подтянут к питанию!), и был хотя бы один клик на кнопке
    
    if(state.click_up && OnClick)
      OnClick(*this,userData);

  }

  if (millisDelta > INACTIVITY_INTERVAL) // пора проверять неактивность
    curTimer = true;
    
  if (curTimer != state.lastTimer) // состояние неактивности изменилось с момента последнего замера?
  {
    state.lastTimer = curTimer; // сохраняем текущее
    state.timer = state.lastButtonState && state.lastTimer && state.atLeastOneStateChangesFound; // кнопка неактивна тогда, когда не была нажата с момента последнего опроса этого состояния

    if(state.timer && OnInactive)
      OnInactive(*this,userData);
  }

  if (millisDelta > RETENTION_INTERVAL) // пора проверять удержание
    curRetention = true;

  if (curRetention != state.lastRetention) // если состояние изменилось
  {
    state.lastRetention = curRetention; // сохраняем его
    state.retention = !state.lastButtonState && state.lastRetention && state.atLeastOneStateChangesFound; // и считаем кнопку удерживаемой, когда она нажата сейчас и была нажата до этого
    
    if(state.retention && OnRetention)
      OnRetention(*this,userData); 
  }
  
}
//--------------------------------------------------------------------------------------------------

