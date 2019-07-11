#include "WaterflowModule.h"
#include "ModuleController.h"
#include "Globals.h"
#include "Memory.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#if WATERFLOW_SENSORS_COUNT > 0
volatile unsigned int pin2FlowPulses; // зафиксированные срабатывания датчика Холла на пине 2
int pin2Interrupt;
void pin2FlowFunc() //  регистрируем срабатывания датчика Холла на пине 2
{
   pin2FlowPulses++;
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#if WATERFLOW_SENSORS_COUNT > 1
volatile unsigned int pin3FlowPulses; // зафиксированные срабатывания датчика Холла на пине 3
int pin3Interrupt;
void pin3FlowFunc() //  регистрируем срабатывания датчика Холла на пине 3
{
   pin3FlowPulses++;
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterflowModule::Setup()
{
  // настройка модуля тут
  checkTimer = 0;

  // настраиваем наши датчики
  pin2Flow.flowMilliLitres = 0;
  pin2Flow.totalMilliliters = 0;
  pin2Flow.totalLitres = 0;
  pin2Flow.calibrationFactor = WATERFLOW_CALIBRATION_FACTOR;
  
  pin3Flow.flowMilliLitres = 0;
  pin3Flow.totalMilliliters = 0;
  pin3Flow.totalLitres = 0;
  pin3Flow.calibrationFactor = WATERFLOW_CALIBRATION_FACTOR;


  //читаем из EEPROM сохранённых значений литров для каждого датчика
  unsigned long tmp = 0;
  
  // читаем  сохранённые показания первого датчика
   byte* wrAddr = (byte*) &tmp;
   uint16_t readPtr = WATERFLOW_EEPROM_ADDR;
  
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr = MemRead(readPtr++);

  if(*wrAddr != 0xFF)
  {
    // есть показания, сохраняем в нашу структуру
    pin2Flow.totalLitres = tmp;
  }

  // теперь читаем показания для второго датчика
  tmp = 0;
  wrAddr = (byte*) &tmp;
  
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr++ = MemRead(readPtr++);
  *wrAddr = MemRead(readPtr++);

  if(*wrAddr != 0xFF)
  {
    // есть показания, сохраняем в нашу структуру
    pin3Flow.totalLitres = tmp;
  }

  // теперь читаем факторы калибровки
  pin2Flow.calibrationFactor = MemRead(readPtr++);
  pin3Flow.calibrationFactor = MemRead(readPtr++);

  // если ничего не сохранено - назначаем фактор калибровки по умолчанию
  if(pin2Flow.calibrationFactor == 0xFF)
    pin2Flow.calibrationFactor = WATERFLOW_CALIBRATION_FACTOR;

  if(pin3Flow.calibrationFactor == 0xFF)
    pin3Flow.calibrationFactor = WATERFLOW_CALIBRATION_FACTOR;


  // регистрируем датчики
  #if WATERFLOW_SENSORS_COUNT > 0
  // первый
  WORK_STATUS.PinMode(FIRST_WATERFLOW_PIN,INPUT,false);
  pin2FlowPulses = 0;
  pin2Interrupt = digitalPinToInterrupt(FIRST_WATERFLOW_PIN);
  State.AddState(StateWaterFlowInstant,0);
  State.AddState(StateWaterFlowIncremental,0);
  
  State.UpdateState(StateWaterFlowIncremental,0,(void*)&(pin2Flow.totalLitres));
  attachInterrupt(pin2Interrupt, pin2FlowFunc, FALLING); 
  #endif

  #if WATERFLOW_SENSORS_COUNT > 1
  // второй
  WORK_STATUS.PinMode(SECOND_WATERFLOW_PIN,INPUT,false);
  pin3FlowPulses = 0;
  pin3Interrupt = digitalPinToInterrupt(SECOND_WATERFLOW_PIN);
  State.AddState(StateWaterFlowInstant,1);
  State.AddState(StateWaterFlowIncremental,1);

   State.UpdateState(StateWaterFlowIncremental,1,(void*)&(pin3Flow.totalLitres));
  attachInterrupt(pin3Interrupt, pin3FlowFunc, FALLING);
  #endif

  // датчики зарегистрированы, теперь можно работать
 
 }
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterflowModule::UpdateFlow(WaterflowStruct* wf,unsigned int delta, unsigned int pulses, uint8_t writeOffset)
{
    // за delta миллисекунд у нас произошло pulses пульсаций, пересчитываем в кол-во миллилитров с момента последнего замера
    float flowRate = (((WATERFLOW_CHECK_FREQUENCY / delta) * pulses)*10) / wf->calibrationFactor;
    
    wf->flowMilliLitres = (flowRate / 60) * 1000; // мгновенные показания с датчика
    wf->totalMilliliters += wf->flowMilliLitres; // накапливаем показания тут

    bool litresChanged = wf->totalMilliliters > 1000;

    while(wf->totalMilliliters > 1000)
    {
        wf->totalLitres++; 
        wf->totalMilliliters -= 1000;        
    } // while


    if(litresChanged && !(wf->totalLitres % WATERFLOW_SAVE_DELTA) ) // сохраняем каждые N литров
    {
      //сохраняем в EEPROM данные с датчика, чтобы не потерять при перезагрузке
        uint16_t addr = WATERFLOW_EEPROM_ADDR + writeOffset;
        unsigned long toWrite = wf->totalLitres;
          
        const byte* readAddr = (const byte*) &toWrite;
        MemWrite(addr++,*readAddr++);
        MemWrite(addr++,*readAddr++);
        MemWrite(addr++,*readAddr++);
        MemWrite(addr++,*readAddr);

    }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WaterflowModule::Update(uint16_t dt)
{ 
  checkTimer += dt;

  if(checkTimer >= WATERFLOW_CHECK_FREQUENCY) // настала пора обновить данные с датчиков
  {
    #if WATERFLOW_SENSORS_COUNT > 0 // чтобы не ругалось, когда не объявлено ни одного датчика   
    unsigned int delta = checkTimer; // получаем актуальное кол-во миллисекунд, прошедшее с последнего опроса
    #endif    
    
    checkTimer = 0; // обнуляем таймер


    #if WATERFLOW_SENSORS_COUNT > 0
    
    // первый датчик
    unsigned int pin2CurPulses = pin2FlowPulses;

    UpdateFlow(&pin2Flow,delta,pin2CurPulses,0); // обновляем состояние, при необходимости - пишем его в EEPROM

    // теперь можем обновить внутреннее состояние модуля
    State.UpdateState(StateWaterFlowInstant,0,(void*) &(pin2Flow.flowMilliLitres));
    State.UpdateState(StateWaterFlowIncremental,0,(void*) &(pin2Flow.totalLitres));

    detachInterrupt(pin2Interrupt); // запрещаем прерывание, чтобы сохранить разницу возможных накопленных тиков, пока мы вычисляли значение
    pin2FlowPulses -= pin2CurPulses;
    attachInterrupt(pin2Interrupt, pin2FlowFunc, FALLING); 


    #endif

    #if WATERFLOW_SENSORS_COUNT > 1
    
    // второй датчик
    unsigned int pin3CurPulses = pin3FlowPulses;

    UpdateFlow(&pin3Flow,delta,pin3CurPulses,sizeof(unsigned long)); // обновляем состояние, при необходимости - пишем его в EEPROM

    // теперь можем обновить внутреннее состояние модуля
    State.UpdateState(StateWaterFlowInstant,1,(void*) &(pin3Flow.flowMilliLitres));
    State.UpdateState(StateWaterFlowIncremental,1,(void*) &(pin3Flow.totalLitres));

    detachInterrupt(pin3Interrupt); // запрещаем прерывание, чтобы сохранить разницу возможных накопленных тиков, пока мы вычисляли значение
    pin3FlowPulses -= pin3CurPulses;
    attachInterrupt(pin3Interrupt, pin3FlowFunc, FALLING); 


    #endif
    
  } // if

  // обновили, отдыхаем

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  WaterflowModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) PublishSingleton = UNKNOWN_COMMAND;

  size_t argsCount = command.GetArgsCount();
  
  if(command.GetType() == ctSET) 
  {
       if(argsCount < 1)
       {
          if(wantAnswer) 
            PublishSingleton = PARAMS_MISSED;
       }
       else
       {
          String t = command.GetArg(0);
          if(t == FLOW_CALIBRATION_COMMAND)
          {
              if(argsCount < 3)
              {
                if(wantAnswer) 
                  PublishSingleton = PARAMS_MISSED;                
              }
              else
              {
                  pin2Flow.calibrationFactor = (uint8_t) atoi(command.GetArg(1));
                  pin3Flow.calibrationFactor = (uint8_t) atoi(command.GetArg(2));
                  
                  uint16_t addr = WATERFLOW_EEPROM_ADDR + sizeof(unsigned long)*2;
                  
                  MemWrite(addr++,pin2Flow.calibrationFactor);
                  MemWrite(addr++,pin3Flow.calibrationFactor);

                  PublishSingleton.Flags.Status = true;
                  if(wantAnswer)
                    PublishSingleton = REG_SUCC;
              }
            
          } // FLOW_CALIBRATION_COMMAND
          else
          if(t == RESET_COMMAND)
          {
            // сбросить показания датчиков расхода
            uint16_t addr = WATERFLOW_EEPROM_ADDR;
            for(byte i=0;i<sizeof(unsigned long)*2;i++)
              MemWrite(addr++,0xFF);

              pin2Flow.totalLitres = 0;
              pin3Flow.totalLitres = 0;
            
            
                  PublishSingleton.Flags.Status = true;
                  if(wantAnswer)
                    PublishSingleton = REG_SUCC;
            
          }
       } // else
  }
  else
  if(command.GetType() == ctGET) //получить статистику
  {
    if(!argsCount) // нет аргументов
    {
      if(wantAnswer) PublishSingleton = PARAMS_MISSED;
    }
    else
    {
        String t = command.GetArg(0);

        if(t == FLOW_CALIBRATION_COMMAND) // запросили данные о факторах калибровки
        {
         PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = FLOW_CALIBRATION_COMMAND; 
            PublishSingleton << PARAM_DELIMITER << pin2Flow.calibrationFactor << PARAM_DELIMITER << pin3Flow.calibrationFactor;
          }
        }
        else
        {
          // неизвестная команда
        } // else

    }// have arguments
    
  } // if
 
 // отвечаем на команду
  MainController->Publish(this,command);
    
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------

