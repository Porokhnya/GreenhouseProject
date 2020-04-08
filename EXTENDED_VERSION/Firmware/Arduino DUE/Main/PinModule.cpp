#include "Arduino.h"
#include "PinModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
void PinModule::Setup()
{
  Update();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PinModule::UpdatePinStates()
{
  size_t sz = pinStates.size();
  for(size_t i=0;i<sz;i++)
  {
    PIN_STATE* s = &(pinStates[i]);
    
    if((s->flags & 1) == 1 && (s->flags & 2) == 2)//(s->isActive && s->hasChanges) // если мы управляем пином
    {
      //s->hasChanges = false;
      s->flags &= ~2;
      WORK_STATUS.PinMode(s->pinNumber,OUTPUT); // делаем пин запоминающим значения
      WORK_STATUS.PinWrite(s->pinNumber,/*s->pinState*/ (s->flags & 4) == 4 ? HIGH : LOW); // запоминаем текущее состояние пина
   
    }
  } // for
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
PIN_STATE* PinModule::GetPin(uint8_t pinNumber)
{
  size_t sz = pinStates.size();
  for(size_t i=0;i<sz;i++)
  {
    PIN_STATE* s = &(pinStates[i]);
    if(s->pinNumber == pinNumber)
      return s;
  } // for
  return NULL;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool PinModule::PinExist(uint8_t pinNumber)
{
  return (GetPin(pinNumber) != NULL);
}
//--------------------------------------------------------------------------------------------------------------------------------------
PIN_STATE* PinModule::AddPin(uint8_t pinNumber,uint8_t currentState)
{
  if(!pinNumber) // если номер пина 0 - не надо ничего делать
    return NULL;
    
  PIN_STATE* s = GetPin(pinNumber);
  if(s)
  {
    //s->pinState = currentState;          
    s->flags &= ~4;
    s->flags |= (currentState << 2);
    //s->isActive = true;
    s->flags |= 1;
    //s->hasChanges = true; 
    s->flags |= 2;
    return s;   
  }

  // можем добавлять, т.к. не нашли пин
    PIN_STATE p;
    p.pinNumber = pinNumber;
//    p.pinState = currentState;
    p.flags &= ~ 4;
    p.flags |= (currentState << 2);
//    p.isActive = true;
    p.flags |= 1;
//    p.hasChanges = true;
    p.flags |= 2;
    WORK_STATUS.PinMode(pinNumber,OUTPUT);
    pinStates.push_back(p);

  return &(pinStates[pinStates.size()-1]);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t PinModule::GetPinState(uint8_t pinNumber)
{
  PIN_STATE* s = GetPin(pinNumber);
  if(s)
    return (s->flags & 4) == 4 ? HIGH : LOW;//s->pinState;
        
// не можем читать состояние пина, не зарегистрированного у нас, поскольку
// этим пином может управлять другой модуль, и мы не можем переводить его в режим 
// чтения состояния. Поэтому возвращаем LOW.
  return LOW;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PinModule::Update()
{ 
  UpdatePinStates(); // обновляем состояние пинов
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  PinModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer)
    PublishSingleton = PARAMS_MISSED;
      
  if(command.GetType() == ctGET) //получить состояние пина
  {
    if(command.GetArgsCount() > 0)
    {
       String strNum = command.GetArg(0);
       if(strNum == F("STATUS"))
       {
         // запросили статус пина из карты пинов
         if(command.GetArgsCount() > 1)
         {
            uint8_t pinNum = atoi(command.GetArg(1));
            uint8_t byteNum = pinNum/8;
            
            if(byteNum <= 15)
            {
              uint8_t bitNum = pinNum%8;

              ControllerState state = WORK_STATUS.GetState();
              String pinState = STATE_OFF;
              
              if(state.PinsState[byteNum] & (1 << bitNum))
                pinState = STATE_ON;

              PublishSingleton.Flags.Status = true;
              PublishSingleton = pinNum;
              PublishSingleton  << PARAM_DELIMITER << pinState;
            }
         }
       }
       else
       {
           uint8_t pinNumber = strNum.toInt();
           uint8_t currentState = GetPinState(pinNumber);
           
           if(wantAnswer) // чтобы не работать с памятью, когда от нас не ждут ответа
           {
            PublishSingleton = strNum;
            PublishSingleton << PARAM_DELIMITER << (currentState == HIGH ? STATE_ON : STATE_OFF);
           }
            
           PublishSingleton.Flags.Status = true;
       }
    }
    

  // отвечаем на команду
    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
    
  } // if ctGET
  else
  if(command.GetType() == ctSET) // set
  {

    if(command.GetArgsCount() > 1)
    {
      String strNum = command.GetArg(0); // номер пина
      String state = command.GetArg(1); // статус пина      
      state.toUpperCase();
      
      // берём номер первого пина, на который будем опираться
      uint8_t pinNumber = strNum.toInt();

      byte pinLevel = LOW;
      bool bActive = !(state == PIN_DETACH); // если послана команда DETACH - выключаем слежение
   
      if(state == STATE_ON_ALT || state == STATE_ON)
        pinLevel = HIGH;
      else
      if(state == PIN_TOGGLE)
      {
           if(!PinExist(pinNumber)) // ещё нет такого пина для слежения
           {
              WORK_STATUS.PinMode(pinNumber,INPUT); // читаем из пина его текущее состояние
              pinLevel = WORK_STATUS.PinRead(pinNumber);
              pinLevel = pinLevel == HIGH ? LOW : HIGH;
           }
           else // пин уже существует для слежения
           {
            PIN_STATE* s = GetPin(pinNumber);
            if(s)
            {
              pinLevel = (s->flags & 4) == 4 ? HIGH : LOW;//s->pinState;
              pinLevel = pinLevel == LOW ? HIGH : LOW;
            }
           } // else
                 
      } // toggle state

      if(!bActive)
      {
         // пришла команда DETACH, поэтому менять статус пина - не надо
         PIN_STATE* s = GetPin(pinNumber);
         if(s)
          pinLevel = (s->flags & 4) == 4 ? HIGH : LOW;//s->pinState; // читаем статус из пина
         else
          AddPin(pinNumber,pinLevel); // пина нету, просто добавляем с уровнем LOW
      }

       // добавляем пин
       PIN_STATE* s = AddPin(pinNumber,pinLevel);
       if(s)
       {
            PublishSingleton.Flags.Status = true;
            if(wantAnswer)
            {
              PublishSingleton = strNum;
              PublishSingleton << PARAM_DELIMITER << (/*s->pinState == HIGH*/(s->flags & 4) == 4 ? STATE_ON : STATE_OFF);
            }
       }

       // добавляем все пины, которые нам передали в параметрах через запятую
       int idx;
       while((idx = strNum.indexOf(F(","))) != -1)
       {
         String pinNum = strNum.substring(0,idx);
         strNum = strNum.substring(idx+1);
         PIN_STATE* s = AddPin(pinNum.toInt(),pinLevel);
         if(s)
         {
//          s->isActive = bActive;
//          s->hasChanges = bActive;
          s->flags &= ~1;
          s->flags &= ~2;
          if(bActive) {
            s->flags |= 1;
            s->flags |= 2;
          }
         }
       } // while
       s =  AddPin(strNum.toInt(),pinLevel);
       if(s)
       {
          s->flags &= ~1;
          s->flags &= ~2;
          if(bActive) {
            s->flags |= 1;
            s->flags |= 2;
          }
          //s->isActive = bActive;
          //s->hasChanges = bActive;
       }
      
    } // if

    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;

  } // if ctSET

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------

