#ifndef _PUSHBUTTON_H
#define _PUSHBUTTON_H
//--------------------------------------------------------------------------------------------------
#include <Arduino.h>
//--------------------------------------------------------------------------------------------------
#define BOUNCE_INTERVAL  70 // длительность отслеживания дребезга.
#define DOUBLECLICK_INTERVAL 200 // длительность отслеживания двойного клика.
#define INACTIVITY_INTERVAL 5000 // длительность отслеживания неактивности.
#define RETENTION_INTERVAL 2000 // длительность отслеживания нажатия и удержания.
//--------------------------------------------------------------------------------------------------
class PushButton;
typedef void (*PushButtonEvent)(const PushButton& Sender, void* UserData);
//--------------------------------------------------------------------------------------------------
typedef struct
{
  bool  lastBounce : 1;
  bool  lastTimer : 1;
  bool  lastRetention : 1;
  bool click_down : 1; // нажата?
  bool click_up : 1; // нажата и отпущена?
  bool doubleclick : 1; // два раза нажата и отпущена?
  bool timer : 1; // неактивна в течение установленного интервала?
  bool retention : 1; // нажата и удерживается в течение установленного интервала?  

  uint8_t  lastButtonState : 1;
  bool atLeastOneStateChangesFound : 1;
  bool lastDoubleClick : 1;
  uint8_t clickCounter : 5;

  
} PushButtonState;
//--------------------------------------------------------------------------------------------------

class PushButton
{
 public:
  
    PushButton(uint8_t _pin);

    // инициализируем, в параметрах можем передать указатель на любые пользовательские данные,
    // плюс указатели на функции-обработчики событий, если это необходимо
    void init(void* _userData = NULL
    , PushButtonEvent _onClick = NULL
    , PushButtonEvent _onPress = NULL
    , PushButtonEvent _onDoubleClick = NULL
    , PushButtonEvent _onInactive = NULL
    , PushButtonEvent _onRetention = NULL
    );    
    void update(); // обновляем внутреннее состояние
    
    bool isPressed() { return state.click_down; }
    bool isClicked() { return state.click_up; }
    bool isDoubleClicked() { return state.doubleclick; }
    bool isInactive() { return state.timer; }
    bool isRetention() { return state.retention; }


 private:

  void* userData;
  PushButtonEvent OnClick; // событие - кнопка нажата и отпущена
  PushButtonEvent OnPress; // событие - кнопка нажата
  PushButtonEvent OnDoubleClick; // событие - кнопка нажата дважды
  PushButtonEvent OnInactive; // событие - кнопка неактивна в течение настроенного интервала
  PushButtonEvent OnRetention; // событие - кнопка нажата и удерживается определённое время
  
  byte buttonPin; // пин, на котором висит кнопка
  
  unsigned long lastMillis;

  PushButtonState state;


 protected:   
    
    uint8_t readButtonState(uint8_t pin)
    {
      return digitalRead(pin);
    }
    
 };
//--------------------------------------------------------------------------------------------------
#endif
