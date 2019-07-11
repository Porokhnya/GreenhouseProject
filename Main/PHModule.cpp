#include "PHModule.h"
#include "ModuleController.h"
#include "Memory.h"
#include <Wire.h>
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PH_MODULE

#define PH_DEBUG_OUT(which, value) {DEBUG_LOG((which)); DEBUG_LOGLN((value));}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
PHCalculator PHCalculation;
PhModule* _thisPHModule = NULL;
PCF8574 pcfModule(PCF8574_ADDRESS);
//-------------------------------------------------------------------------------------------------------------------------------------------------------
PHCalculator::PHCalculator()
{
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PHCalculator::ApplyCalculation(Temperature* temp)
{ 
  if(!_thisPHModule)
    return;

  _thisPHModule->ApplyCalculation(temp);  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
PCF8574::PCF8574(int address) 
{
  _address = address;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::begin()
{
  ////////////Wire.begin();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t PCF8574::read8()
{
  if(Wire.requestFrom(_address, 1) == 1)
  {
    
#if (ARDUINO <  100)
   _data = Wire.receive();
#else
   _data = Wire.read();
#endif
  }
  else
  {
    _error = -100;
  }
  return _data;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t PCF8574::value()
{
  return _data;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::write8(uint8_t value)
{
  Wire.beginTransmission(_address);
  _data = value;
  Wire.write(_data);
  _error = Wire.endTransmission();
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t PCF8574::read(uint8_t pin)
{
  PCF8574::read8();
  return (_data & (1<<pin)) > 0;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::write(uint8_t pin, uint8_t value)
{
  PCF8574::read8();
  if (value == LOW) 
  {
    _data &= ~(1<<pin);
  }
  else 
  {
    _data |= (1<<pin);
  }
  PCF8574::write8(_data); 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::toggle(uint8_t pin)
{
  PCF8574::read8();
  _data ^=  (1 << pin);
  PCF8574::write8(_data); 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::shiftRight(uint8_t n)
{
  if (n == 0 || n > 7 ) return;
  PCF8574::read8();
  _data >>= n;
  PCF8574::write8(_data); 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PCF8574::shiftLeft(uint8_t n)
{
  if (n == 0 || n > 7) return;
  PCF8574::read8();
  _data <<= n;
  PCF8574::write8(_data); 
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
int PCF8574::lastError()
{
  int e = _error;
  
  #ifdef PH_DEBUG
    if(_error)
    {
      PH_DEBUG_OUT(F("Error working with PCF8574, code: "), _error);
    }
  #endif    

  _error = 0;
  return e;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PhModule::ApplyCalculation(Temperature* temp)
{
  // Эта функция вызывается при любом обновлении значения с датчика pH, откуда бы это значение
  // ни пришло. Здесь мы можем применить к значению поправочные факторы калибровки, в том числе
  // по температуре, плюс применяем к показаниям поправочное число.
  
  if(!temp)
    return;

  if(!temp->HasData())
    return;

  // теперь проверяем, можем ли мы применить калибровочные поправки?
  // для этого все вольтажи, наличие показаний с датчика температуры
  // и выставленная настройки температуры калибровочных растворов
  // должны быть актуальными.

  if(ph4Voltage < 1 || ph7Voltage < 1 || ph10Voltage < 1 || phTemperatureSensorIndex < 0 
  || !phSamplesTemperature.HasData() || phSamplesTemperature.Value > 100 || phSamplesTemperature.Value < 0)
    return;

  AbstractModule* tempModule = MainController->GetModuleByID("STATE");
  if(!tempModule)
    return;

  OneState* os = tempModule->State.GetState(StateTemperature,phTemperatureSensorIndex);
  if(!os)
    return;

  TemperaturePair tempPair = *os;
  Temperature tempPH = tempPair.Current;

  if(!tempPH.HasData() || tempPH.Value > 100 || tempPH.Value < 0)
    return;

  Temperature tDiff = tempPH - phSamplesTemperature;

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("T diff: "), tDiff);
  #endif

  long ulDiff = tDiff.Value;
  uint8_t sign = tDiff.Value < 0 ? -1 : 1;
  ulDiff *= 100;
  ulDiff += tDiff.Fract*sign;

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("ulDiff: "), ulDiff);
  #endif
  
  float fTempDiff = ulDiff/100.0;

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("fTempDiff: "), fTempDiff);
    PH_DEBUG_OUT(F("source PH: "), *temp);
  #endif 

  // теперь можем применять факторы калибровки.
  // сначала переводим текущие показания в вольтаж, приходится так делать, поскольку
  // они приходят уже нормализованными.
  long curPHVoltage = temp->Value;

  curPHVoltage *= 100;
  curPHVoltage += temp->Fract + calibration; // прибавляем сотые доли показаний, плюс сотые доли поправочного числа
  curPHVoltage *= 100;

 // у нас есть PH_MV_PER_7_PH 2000 - кол-во милливольт, при  которых датчик показывает 7 pH
 // следовательно, в этом месте мы должны получить коэффициент 35 (например), который справедлив для значения 2000 mV при 7 pH
 // путём нехитрой формулы получаем, что коэффициент здесь будет равен 70000/PH_MV_PER_7_PH
 float coeff = 70000/PH_MV_PER_7_PH;
  
  curPHVoltage /= coeff; // например, 7,00 pH  сконвертируется в 2000 милливольт, при значении  PH_MV_PER_7_PH == 2000

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("curPHVoltage: "), curPHVoltage);
    PH_DEBUG_OUT(F("ph4Voltage: "), ph4Voltage);
    PH_DEBUG_OUT(F("ph7Voltage: "), ph7Voltage);
    PH_DEBUG_OUT(F("ph10Voltage: "), ph10Voltage);
  #endif

  long phDiff = ph4Voltage;
  phDiff -= ph10Voltage;

  float sensitivity = phDiff/6.0;
  sensitivity = sensitivity + fTempDiff*0.0001984;

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("sensitivity: "), sensitivity);
  #endif

  phDiff = ph7Voltage;
  phDiff -= curPHVoltage;

  #ifdef PH_REVERSIVE_MEASURE
  float calibratedPH = 7.0 - phDiff/sensitivity; // реверсивное изменение вольтажа при нарастании pH
  #else
  float calibratedPH = 7.0 + phDiff/sensitivity; // прямое изменение вольтажа при нарастании pH
  #endif

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("calibratedPH: "), calibratedPH);
  #endif

  // теперь переводим всё это обратно в понятный всем вид
  uint16_t phVal = calibratedPH*100;

  // и сохраняем это дело в показания датчика
  temp->Value = phVal/100;
  temp->Fract = phVal%100;

  #ifdef PH_DEBUG
    PH_DEBUG_OUT(F("pH result: "), *temp);
  #endif
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PhModule::Setup()
{
  _thisPHModule = this;
  
  // настройка модуля тут
  phSensorPin = PH_SENSOR_PIN;
  measureTimer = 0;
  flags.inMeasure = false;
  samplesDone = 0;
  samplesTimer = 0;
  calibration = 0;
  ph4Voltage = 0;
  ph7Voltage = 0;
  ph10Voltage = 0;
  phTemperatureSensorIndex = -1;
  phSamplesTemperature.Value = 25; // 25 градусов температура калибровочных растворов по умолчанию

  // читаем настройки
  ReadSettings();

  // теперь смотрим - если у нас пин pH не 0 - значит, надо добавить состояние
  if(phSensorPin > 0)
  {
    State.AddState(StatePH,0); // добавляем датчик pH, прикреплённый к меге
    WORK_STATUS.PinMode(phSensorPin,INPUT);
    digitalWrite(phSensorPin,HIGH);
  }

  // настраиваем пины PCF8574
  // выставляем все значения по умолчанию - насос заполнения бака выключен, все помпы и насосы - выключены
  uint8_t defaultData = 0;
  defaultData |= (PH_FLOW_ADD_OFF << PH_FLOW_ADD_CHANNEL);
  defaultData |= (PH_CONTROL_VALVE_OFF << PH_PLUS_CHANNEL);
  defaultData |= (PH_CONTROL_VALVE_OFF << PH_MINUS_CHANNEL);
  defaultData |= (PH_MIX_PUMP_OFF << PH_MIX_PUMP_CHANNEL);

  flags.isMixPumpOn = false; // насос перемешивания выключен
  mixPumpTimer = 0;

  phControlTimer = 0;

  flags.isInAddReagentsMode = false;
  reagentsTimer = 0;
  targetReagentsTimer = 0;
  targetReagentsChannel = 0;
  
  // пишем в микросхему
  pcfModule.begin();
  pcfModule.write8(defaultData);
  pcfModule.lastError();

  updateDelta = 0; // дельта обновления данных, чтобы часто не дёргать микросхему

  // сохраняем статусы работы
  SAVE_STATUS(PH_FLOW_ADD_BIT,0);
  SAVE_STATUS(PH_MIX_PUMP_BIT,0);
  SAVE_STATUS(PH_PLUS_PUMP_BIT,0);
  SAVE_STATUS(PH_MINUS_PUMP_BIT,0);
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool PhModule::isLevelSensorTriggered(byte data)
{
  return (data & (PH_FLOW_LEVEL_TRIGGERED << PH_FLOW_LEVEL_SENSOR_CHANNEL)) == PH_FLOW_LEVEL_TRIGGERED;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PhModule::SaveSettings()
{
  uint16_t addr = PH_SETTINGS_EEPROM_ADDR;

  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

  MemWrite(addr++,phSensorPin);

  // пишем калибровку
  byte cal[2];
  memcpy(cal,&calibration,2);
  MemWrite(addr++,cal[0]);
  MemWrite(addr++,cal[1]);

  // пишем вольтаж раствора 4 pH
  byte* pB = (byte*) &ph4Voltage;
  for(size_t i=0;i<sizeof(ph4Voltage);i++)
    MemWrite(addr++,*pB++);
    

  // пишем вольтаж раствора 7 pH
  pB = (byte*) &ph7Voltage;
  for(size_t i=0;i<sizeof(ph7Voltage);i++)
    MemWrite(addr++,*pB++);
    

  // пишем вольтаж раствора 10 pH
  pB = (byte*) &ph10Voltage;
  for(size_t i=0;i<sizeof(ph10Voltage);i++)
    MemWrite(addr++,*pB++);
      

  // пишем индекс датчика температуры
  MemWrite(addr++,phTemperatureSensorIndex);

  // пишем показания температуры при калибровке
  cal[0] = phSamplesTemperature.Value;
  cal[1] = phSamplesTemperature.Fract;

  MemWrite(addr++,cal[0]);
  MemWrite(addr++,cal[1]);

  pB = (byte*) &phTarget;
  for(size_t i=0;i<sizeof(phTarget);i++)
    MemWrite(addr++,*pB++);
    

  pB = (byte*) &phHisteresis;
  for(size_t i=0;i<sizeof(phHisteresis);i++)
    MemWrite(addr++,*pB++);

  pB = (byte*) &phMixPumpTime;
  for(size_t i=0;i<sizeof(phMixPumpTime);i++)
    MemWrite(addr++,*pB++);

  pB = (byte*) &phReagentPumpTime;
  for(size_t i=0;i<sizeof(phReagentPumpTime);i++)
    MemWrite(addr++,*pB++);

  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PhModule::ReadSettings()
{
  uint16_t addr = PH_SETTINGS_EEPROM_ADDR;
  if(MemRead(addr++) != SETT_HEADER1)
    return;

  if(MemRead(addr++) != SETT_HEADER2)
    return;

  phSensorPin =  MemRead(addr++); 
  if(phSensorPin == 0xFF)
    phSensorPin = PH_SENSOR_PIN;

  byte cal[2];
  cal[0] = MemRead(addr++);
  cal[1] = MemRead(addr++);

  if(cal[0] == 0xFF && cal[1] == 0xFF) // нет калибровки
    calibration = PH_DEFAULT_CALIBRATION;
  else
    memcpy(&calibration,cal,2); // иначе копируем сохранённую калибровку

 // читаем вольтаж раствора 4 pH
 byte* pB = (byte*) &ph4Voltage;
 for(size_t i=0;i<sizeof(ph4Voltage);i++)
 {
    *pB = MemRead(addr++);
     pB++;
 }

 // читаем вольтаж раствора 7 pH
  pB = (byte*) &ph7Voltage;
 for(size_t i=0;i<sizeof(ph7Voltage);i++)
 {
    *pB = MemRead(addr++);
     pB++;
 }


 // читаем вольтаж раствора 10 pH
  pB = (byte*) &ph10Voltage;
 for(size_t i=0;i<sizeof(ph10Voltage);i++)
 {
    *pB = MemRead(addr++);
     pB++;
 }
 
  // читаем индекс датчика температуры
  phTemperatureSensorIndex = MemRead(addr++);

  // читаем значение температуры калибровки
  cal[0] = MemRead(addr++);
  cal[1] = MemRead(addr++);

  // теперь проверяем корректность всех настроек
  if(0xFFFF == (uint16_t) ph4Voltage)
    ph4Voltage = 0;

  if(0xFFFF == (uint16_t) ph7Voltage)
    ph7Voltage = 0;

  if(0xFFFF == (uint16_t) ph10Voltage)
    ph10Voltage = 0;

  if(0xFF == (byte) phTemperatureSensorIndex)
    phTemperatureSensorIndex = -1;

  if(cal[0] == 0xFF)
    phSamplesTemperature.Value = 25; // 25 градусов дефолтная температура
  else
    phSamplesTemperature.Value = cal[0];

  if(cal[1] == 0xFF)
    phSamplesTemperature.Fract = 0;
  else
    phSamplesTemperature.Fract = cal[1];


    pB = (byte*) &phTarget;
    for(size_t i=0;i<sizeof(phTarget);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }
 
  
  if(phTarget == 0xFFFF)
    phTarget = PH_DEFAULT_TARGET;   

    pB = (byte*) &phHisteresis;
    for(size_t i=0;i<sizeof(phHisteresis);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }
  
  if(phHisteresis == 0xFFFF)
    phHisteresis = PH_DEFAULT_HISTERESIS;   

    pB = (byte*) &phMixPumpTime;
    for(size_t i=0;i<sizeof(phMixPumpTime);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }

  
  if(phMixPumpTime == 0xFFFF)
    phMixPumpTime = PH_DEFAULT_MIX_PUMP_TIME;   

    pB = (byte*) &phReagentPumpTime;
    for(size_t i=0;i<sizeof(phReagentPumpTime);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }
 
  if(phReagentPumpTime == 0xFFFF)
    phReagentPumpTime = PH_DEFAULT_REAGENT_PUMP_TIME;   
  
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
void PhModule::Update(uint16_t dt)
{ 
  // обновление модуля тут
  if(phSensorPin > 0)
  {
    // у нас есть датчик, жёстко прикреплённый к меге, можно читать с него данные.
    // если вы сейчас измеряем, то надо проверять, не истёк ли интервал между семплированиями.
    // если истёк - начинаем замерять. если нет - ничего не делаем.

    // как только достигнем нужного кол-ва семплов - подсчитываем значение pH и обновляем его

    // если же мы не измеряем, то проверяем - не истёк ли интервал между замерами.
    // если истёк - начинаем измерения.

    if(flags.inMeasure)
    {
      // в процессе замера
      if(samplesDone >= PH_SAMPLES_PER_MEASURE)
      {
         // набрали нужное кол-во семплов
         samplesTimer = 0;
         flags.inMeasure = false;
         measureTimer = 0;

         // теперь преобразуем полученное значение в среднее
         float avgSample = (dataArray*1.0)/samplesDone;

          // считаем вольтаж
          float voltage = avgSample*5.0/1024;

  
         // теперь получаем значение pH
         //unsigned long phValue = voltage*350;
         // у нас есть PH_MV_PER_7_PH 2000 - кол-во милливольт, при  которых датчик показывает 7 pH
         // следовательно, в этом месте мы должны получить коэффициент 350 (например), который справедлив для значения 2000 mV при 7 pH
         // путём нехитрой формулы получаем, что коэффициент здесь будет равен 700000/PH_MV_PER_7_PH
         float coeff = 700000/PH_MV_PER_7_PH;
         // и применяем этот коэффициент
         unsigned long phValue = voltage*coeff;
         // вышеприведённые подсчёты pH справедливы для случая "больше вольтаж - больше pH",
         // однако нам надо учесть и реверсивный случай, когда "больше вольтаж - меньше pH".
        #ifdef PH_REVERSIVE_MEASURE
          // считаем значение pH в условиях реверсивных измерений
          int16_t rev = phValue - 700; // поскольку у нас 7 pH - это средняя точка, то в условии реверсивных изменений от
          // средней точки pH (7.0) надо отнять разницу между значением 7 pH и полученным значением, что мы и делаем
          phValue = 700 - rev;
         #endif
                  
         
         Humidity h;         

         if(avgSample > 1000)
         {
           // не прочитали ничего из порта
         }
         else
         {
           h.Value = phValue/100;
           h.Fract = phValue%100;          
         }

         // сохраняем состояние с датчика
         State.UpdateState(StatePH,0,(void*)&h);     

         samplesDone = 0;
        
      } // if(samplesDone >= PH_SAMPLES_PER_MEASURE)
      else
      {
        // ещё набираем семплы
        samplesTimer += dt; // обновляем таймер
        if(samplesTimer > PH_SAMPLES_INTERVAL)
        {
          // настало время очередного замера
          samplesTimer = 0; // сбрасываем таймер
          samplesDone++; // увеличиваем кол-во семплов

          // читаем из порта и запоминаем прочитанное
          dataArray += analogRead(phSensorPin);
          
        } // PH_SAMPLES_INTERVAL
        
      } // else
      
    } // inMeasure
    else
    {
      // ничего не измеряем, сохраняем дельту в таймер между замерами
      measureTimer += dt;

      if(measureTimer > PH_UPDATE_INTERVAL)
      {
        // настала пора переключиться на замер
        measureTimer = 0;
        flags.inMeasure = true;
        samplesDone = 0;
        samplesTimer = 0;

        // очищаем массив данных
        dataArray = 0;

        // читаем первый раз, игнорируем значение, чтобы выровнять датчик
        analogRead(phSensorPin);
      }
    } // else
    
  } // if(phSensorPin > 0)

  //Тут контроль pH
  updateDelta += dt;

  if(flags.isMixPumpOn) // если насос перемешивания включен - прибавляем время его работы
  {
    mixPumpTimer += dt; // прибавляем время работы насоса

    if(mixPumpTimer/1000 > phMixPumpTime)
    {
      // насос отработал положенное время, надо выключать

      #ifdef PH_DEBUG
        PH_DEBUG_OUT(F("Mix pump done, turn it OFF."), "");
      #endif

      SAVE_STATUS(PH_MIX_PUMP_BIT,0); // сохраняем статус помпы перемешивания
            
      // надо выключить насос перемешивания
      flags.isMixPumpOn = false; // выключаем помпу
      mixPumpTimer = 0; // сбрасываем таймер помпы перемешивания
      phControlTimer = 0; // сбрасываем таймер обновления pH
      
      // пишем в микросхему
      byte data = pcfModule.read8();
      if(!pcfModule.lastError())
      {
        data &= ~(1 << PH_MIX_PUMP_CHANNEL);
        data |= (PH_MIX_PUMP_OFF << PH_MIX_PUMP_CHANNEL);
        pcfModule.write8(data);
      }
      
    } // if
    
  } // if(flags.isMixPumpOn)

  if(flags.isInAddReagentsMode)
  {
    // запущен таймер добавления реагентов
    reagentsTimer += dt; // прибавляем дельту работы
    
    if(reagentsTimer/1000 > targetReagentsTimer)
    {
      #ifdef PH_DEBUG
        PH_DEBUG_OUT(F("Reagents pump done, turn it OFF. Pump channel: "), targetReagentsChannel);
        PH_DEBUG_OUT(F("Turn mix pump ON..."), "");
      #endif 

      SAVE_STATUS(PH_MIX_PUMP_BIT,1); // сохраняем статус помпы перемешивания
      SAVE_STATUS(PH_PLUS_PUMP_BIT,0);
      SAVE_STATUS(PH_MINUS_PUMP_BIT,0);    
           
      // настала пора выключать реагенты и включать насос перемешивания
      reagentsTimer = 0; // сбрасываем таймер реагентов
      flags.isInAddReagentsMode = false; // выключаем подачу реагентов
      flags.isMixPumpOn = true; // включаем помпу перемешивания
      mixPumpTimer = 0; // сбрасываем таймер работы помпы перемешивания

      byte data = pcfModule.read8();

      if(!pcfModule.lastError())
      {
        // выключаем канал подачи реагента
        data &= ~(1 << targetReagentsChannel);
        data |= (PH_CONTROL_VALVE_OFF << targetReagentsChannel);
  
        // включаем канал перемешивания
        data &= ~(1 << PH_MIX_PUMP_CHANNEL);
        data |= (PH_MIX_PUMP_ON << PH_MIX_PUMP_CHANNEL);
  
        // пишем в микросхему актуальное состояние
        pcfModule.write8(data);
      }

    } // if(reagentsTimer/1000 > targetReagentsTimer)
    
  } // if(isInAddReagentsMode)
  
  if(updateDelta > 1234)
  {
    updateDelta = 0;
    // настала пора проверить, чего у нас там творится?
    byte data = pcfModule.read8();

    if(!pcfModule.lastError())
    {  
      if(isLevelSensorTriggered(data))
      {
        // сработал датчик уровня воды
        #ifdef PH_DEBUG
          PH_DEBUG_OUT(F("Level sensor triggered, OFF pumps, turn ON flow add pump, no pH control..."), "");
        #endif

        SAVE_STATUS(PH_FLOW_ADD_BIT,1); // говорим в статус, что включен насос подачи воды в бак pH

        // остальные статусы сбрасываем, поскольку во время подачи воды мы ничего не делаем
        SAVE_STATUS(PH_MIX_PUMP_BIT,0);
        SAVE_STATUS(PH_PLUS_PUMP_BIT,0);
        SAVE_STATUS(PH_MINUS_PUMP_BIT,0);

        // выключаем все насосы подачи, перемешивания, включаем помпу подачи воды и выходим
        // сначала сбрасываем нужные биты
        data &= ~(1 << PH_FLOW_ADD_CHANNEL);
        data &= ~(1 << PH_PLUS_CHANNEL);
        data &= ~(1 << PH_MINUS_CHANNEL);
        data &= ~(1 << PH_MIX_PUMP_CHANNEL);
  
        // теперь устанавливаем нужные биты
        data |= (PH_FLOW_ADD_ON << PH_FLOW_ADD_CHANNEL);
        data |= (PH_CONTROL_VALVE_OFF << PH_PLUS_CHANNEL);
        data |= (PH_CONTROL_VALVE_OFF << PH_MINUS_CHANNEL);
        data |= (PH_MIX_PUMP_OFF << PH_MIX_PUMP_CHANNEL);
  
        // теперь пишем всё это дело в микросхему
        pcfModule.write8(data);
  
        flags.isMixPumpOn = false; // выключаем помпу
        mixPumpTimer = 0;
  
        flags.isInAddReagentsMode = false; // выключаем насосы добавления реагентов
        reagentsTimer = 0; // сбрасываем таймер добавления реагентов
  
        
        return; // ничего не контролируем, т.к. наполняем бак водой
        
      } // if(isLevelSensorTriggered(data))
      

      // датчик уровня не сработал, очищаем бит контроля насоса, потом - выключаем насос подачи воды
      SAVE_STATUS(PH_FLOW_ADD_BIT,0); // сохраняем статус насоса подачи воды
      data &= ~(1 << PH_FLOW_ADD_CHANNEL);
      data |= (PH_FLOW_ADD_OFF << PH_FLOW_ADD_CHANNEL);
  
      // сразу пишем в микросхему, чтобы поддержать актуальное состояние
      pcfModule.write8(data);
      
    } // if(!pcfModule.lastError())
    
  } // if(updateDelta > 1234)

  if(flags.isInAddReagentsMode)
  {
   // добавляем реагенты, не надо ничего делать
    
  } // if (isInAddReagentsMode)
  else
  {
    // реагенты не добавляем, можем проверять pH, если помпа перемешивания не работает и настал интервал проверки
    
    if(!flags.isMixPumpOn)
    {
      // только если не включен насос перемешивания и насосы подачи реагента - попадаем сюда, на проверку контроля pH
      phControlTimer += dt;
      if(phControlTimer > PH_CONTROL_CHECK_INTERVAL)
      {
        phControlTimer = 0;
        // пора проверить pH
        #ifdef PH_DEBUG
          PH_DEBUG_OUT(F("Start pH checking..."), "");
        #endif    

        // тут собираем данные со всех датчиков pH, берём среднее арифметическое и проверяем
        byte validDataCount = 0;
        unsigned long accumulatedData = 0;
        uint8_t _cnt = State.GetStateCount(StatePH);
          for(uint8_t i=0;i<_cnt;i++)
          {
             OneState* st = State.GetStateByOrder(StatePH,i);
             HumidityPair hp = *st;
             Humidity h = hp.Current;
             if(h.HasData())
             {
              validDataCount++;
              accumulatedData += h.Value*100 + h.Fract;
             } // if
          } // for

          if(validDataCount > 0)
          {
            accumulatedData = accumulatedData/validDataCount;
            #ifdef PH_DEBUG
              PH_DEBUG_OUT(F("AVG current pH: "), accumulatedData);
              PH_DEBUG_OUT(F("Target pH: "), phTarget);
            #endif

            if(accumulatedData >= (unsigned long) (phTarget - phHisteresis) && accumulatedData <= (unsigned long) (phTarget + phHisteresis))
            {
              // находимся в пределах гистерезиса
            #ifdef PH_DEBUG
              PH_DEBUG_OUT(F("pH valid, nothing to control."),"");
            #endif
              
            }
            else
            {
              // находимся за пределами гистерезиса, надо выяснить, какой насос включать и на сколько
              targetReagentsChannel = accumulatedData < phTarget ? PH_PLUS_CHANNEL : PH_MINUS_CHANNEL;
              reagentsTimer = 0;

              // сохраняем статус
              if(targetReagentsChannel == PH_PLUS_CHANNEL)
                SAVE_STATUS(PH_PLUS_PUMP_BIT,1);
              else
                SAVE_STATUS(PH_MINUS_PUMP_BIT,1);

              SAVE_STATUS(PH_MIX_PUMP_BIT,0);

              #ifdef PH_DEBUG
                PH_DEBUG_OUT(F("pH needs to change, target channel: "),targetReagentsChannel);
              #endif

              // теперь вычисляем, сколько времени в секундах надо работать каналу
              uint16_t distance = accumulatedData < phTarget ? (phTarget - accumulatedData) : (accumulatedData - phTarget);
              
              // дистанция у нас в сотых долях, т.е. 50 - это 0.5 десятых. в phReagentPumpTime у нас значение в секундах для дистанции в 0.1 pH.
              // переводим дистанцию в десятые доли
              distance /= 10;
              
              #ifdef PH_DEBUG
                PH_DEBUG_OUT(F("pH distance: "),distance);
              #endif

              // подсчитываем время работы канала подачи
              targetReagentsTimer = phReagentPumpTime*distance;
              
              #ifdef PH_DEBUG
                PH_DEBUG_OUT(F("Reagents pump work time, s: "),targetReagentsTimer);
              #endif

              // переходим в режим подачи реагента
              flags.isInAddReagentsMode = true;
              flags.isMixPumpOn = false;
              mixPumpTimer = 0;

              // читаем из микросхемы и изменяем наши настройки
              byte data = pcfModule.read8();

              if(!pcfModule.lastError())
              {
                data &= ~(1 << targetReagentsChannel); // сбрасываем бит канала подачи реагента
                data |= (PH_CONTROL_VALVE_ON << targetReagentsChannel); // включаем канал подачи реагента
  
                // на всякий случай выключаем помпу перемешивания
                data &= ~(1 << PH_MIX_PUMP_CHANNEL);
                data |= (PH_MIX_PUMP_OFF << PH_MIX_PUMP_CHANNEL);
  
                // пишем в микросхему
                pcfModule.write8(data);
              
              } // if(!pcfModule.lastError())
              
                 #ifdef PH_DEBUG
                PH_DEBUG_OUT(F("Reagents pump ON."),"");
              #endif           

            
            } // else


          } // if(validDataCount > 0)
          else
          {
            // нет данных с датчиков, нечего контролировать
            #ifdef PH_DEBUG
              PH_DEBUG_OUT(F("No pH sensors data found, nothing to control!"), "");
            #endif                
          } // else
        
      } // if(phControlTimer > PH_CONTROL_CHECK_INTERVAL)
      
    } // if(!flags.isMixPumpOn)
    
  } // else не добавляем реагенты

}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
bool  PhModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) 
    PublishSingleton = NOT_SUPPORTED;


  uint8_t argsCnt = command.GetArgsCount();
    
  if(command.GetType() == ctSET) // установка свойств
  {
    if(argsCnt < 1)
     {
        if(wantAnswer) 
          PublishSingleton = PARAMS_MISSED;
     }
     else
     {
       String param = command.GetArg(0);
       if(param == PH_SETTINGS_COMMAND) // установить настройки: CTSET=PH|T_SETT|calibration_factor|ph4Voltage|ph7Voltage|ph10Voltage|temp_sensor_index|samples_temp|ph_target|ph_histeresis|mix_time|reagent_time
       {
          if(argsCnt < 11)
          {
            if(wantAnswer)
              PublishSingleton = PARAMS_MISSED;
          }
          else
          {
             // аргументов хватает
             calibration = atoi(command.GetArg(1));
             ph4Voltage = atoi(command.GetArg(2));
             ph7Voltage = atoi(command.GetArg(3));
             ph10Voltage = atoi(command.GetArg(4));
             phTemperatureSensorIndex = atoi(command.GetArg(5));
             
             int samplesTemp = atoi(command.GetArg(6));
             phSamplesTemperature.Value = samplesTemp/100;
             phSamplesTemperature.Fract = samplesTemp%100;

             phTarget = atoi(command.GetArg(7));
             phHisteresis = atoi(command.GetArg(8));
             phMixPumpTime = atoi(command.GetArg(9));
             phReagentPumpTime = atoi(command.GetArg(10));
             
             
             SaveSettings();

             PublishSingleton.Flags.Status = true;
             PublishSingleton = REG_SUCC;
          }
       } // PH_SETTINGS_COMMAND
       
     } // else argsCount >= 1
     
    
  } // ctSET    
  else
  if(command.GetType() == ctGET) // запрос свойств
  {
      
      if(argsCnt < 1)
      {
        if(wantAnswer) 
          PublishSingleton = PARAMS_MISSED; // не хватает параметров
        
      } // argsCnt < 1 
      else
      {     
        String param = command.GetArg(0);
        
        if(param == ALL) // запросили показания со всех датчиков: CTGET=PH|ALL
        {
          PublishSingleton.Flags.Status = true;
          uint8_t _cnt = State.GetStateCount(StatePH);
          if(wantAnswer) 
            PublishSingleton = _cnt;
          
          for(uint8_t i=0;i<_cnt;i++)
          {

             OneState* stateHumidity = State.GetStateByOrder(StatePH,i);
             if(stateHumidity)
             {
                HumidityPair hp = *stateHumidity;

                // конвертируем текущее значение pH в милливольты
                // в PH_MV_PER_7_PH - mV при 7.00 pH
                // в Х - mV при current hH
                // X = (PH_MV_PER_7_PH*7.00)/current pH
                // или, в целых числах
                // X = (PH_MV_PER_7_PH*700)/current pH

                unsigned long curPH = 0;
                uint16_t phMV = 0;

                if(stateHumidity->HasData())
                {

                  // надо правильно подсчитать милливольты, в зависимости от типа направления измерений
                  // (растёт ли вольтаж при увеличении pH или убывает)

                  #ifdef PH_REVERSIVE_MEASURE
                    // реверсивное измерение pH
                    curPH  = hp.Current.Value*100 + hp.Current.Fract;
                    int16_t diff = (700 - curPH);
                    curPH = 700 + diff;
                  #else
                    // прямое измерение pH 
                    curPH  = hp.Current.Value*100 + hp.Current.Fract;
                  #endif
                  
                  phMV = (PH_MV_PER_7_PH*curPH)/700; // получаем милливольты
                    
                }
              
                if(wantAnswer) 
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current) << PARAM_DELIMITER << phMV;
                }
             } // if
          } // for        
        } // param == ALL
        else
        if(param == PH_SETTINGS_COMMAND) // получить/установить настройки: CTGET=PH|T_SETT, CTSET=PH|T_SETT|calibration_factor|ph4Voltage|ph7Voltage|ph10Voltage|temp_sensor_index|samples_temp|ph_target|ph_histeresis|mix_time|reagent_time
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer)
          {
            PublishSingleton = PH_SETTINGS_COMMAND;
            PublishSingleton << PARAM_DELIMITER << calibration;
            PublishSingleton << PARAM_DELIMITER << ph4Voltage;
            PublishSingleton << PARAM_DELIMITER << ph7Voltage;
            PublishSingleton << PARAM_DELIMITER << ph10Voltage;
            PublishSingleton << PARAM_DELIMITER << phTemperatureSensorIndex;
            PublishSingleton << PARAM_DELIMITER << phSamplesTemperature;             
            PublishSingleton << PARAM_DELIMITER << phTarget;             
            PublishSingleton << PARAM_DELIMITER << phHisteresis;             
            PublishSingleton << PARAM_DELIMITER << phMixPumpTime;             
            PublishSingleton << PARAM_DELIMITER << phReagentPumpTime;             
          }
          
        } // PH_SETTINGS_COMMAND
        else
        if(param == PROP_CNT) // запросили данные о кол-ве датчиков: CTGET=PH|CNT
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = PROP_CNT; 
            uint8_t _cnt = State.GetStateCount(StatePH);
            PublishSingleton << PARAM_DELIMITER << _cnt;
          }
        } // PROP_CNT
        else
        if(param != GetID()) // если только не запросили без параметров
        {
          // запросили показания с датчика по индексу
          uint8_t idx = param.toInt();
          uint8_t _cnt = State.GetStateCount(StatePH);
          
          if(idx >= _cnt)
          {
            // плохой индекс
            if(wantAnswer) 
              PublishSingleton = NOT_SUPPORTED;
          } // плохой индекс
          else
          {
             if(wantAnswer) 
              PublishSingleton = param;
              
             OneState* stateHumidity = State.GetStateByOrder(StatePH,idx);
             if(stateHumidity)
             {
                PublishSingleton.Flags.Status = true;
                HumidityPair hp = *stateHumidity;

                // конвертируем текущее значение pH в милливольты
                // в PH_MV_PER_7_PH - mV при 7.00 pH
                // в Х - mV при current hH
                // X = (PH_MV_PER_7_PH*7.00)/current pH
                // или, в целых числах
                // X = (PH_MV_PER_7_PH*700)/current pH

                unsigned long curPH = 0;
                uint16_t phMV = 0;

                if(stateHumidity->HasData())
                {
                  // надо правильно подсчитать милливольты, в зависимости от типа направления измерений
                  // (растёт ли вольтаж при увеличении pH или убывает)

                  #ifdef PH_REVERSIVE_MEASURE
                    // реверсивное измерение pH
                    curPH  = hp.Current.Value*100 + hp.Current.Fract;
                    int16_t diff = (700 - curPH);
                    curPH = 700 + diff;
                  #else
                    // прямое измерение pH 
                    curPH  = hp.Current.Value*100 + hp.Current.Fract;
                  #endif
                  
                  phMV = (PH_MV_PER_7_PH*curPH)/700; // получаем милливольты
                  
                }
                if(wantAnswer)
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current) << PARAM_DELIMITER << phMV;
                }
             } // if
            
          } // else нормальный индекс        
        } // if param != GetID()
        
      } // else
  }
  
  MainController->Publish(this,command); 
  
  return true;
}
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PH_MODULE

