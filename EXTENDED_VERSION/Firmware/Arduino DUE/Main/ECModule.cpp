#include "ECModule.h"
#include "ModuleController.h"
#include "Memory.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
ECModule* ECControl = NULL;
EC_PHCalculator EC_PH_Calculation;
//--------------------------------------------------------------------------------------------------------------------------------------
EC_PHCalculator::EC_PHCalculator()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void EC_PHCalculator::ApplyCalculation(Temperature* temp)
{ 
  if(!ECControl)
    return;

  ECControl->ApplyCalculation(temp);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
ECModule::ECModule() : AbstractModule("EC") 
{
  ECControl = this;
  phSensorAdded = false;
  phSamplesDone = 0;
  phTimer = 0;
  phMeasureState = phInMeasure;

  ecSamplesDone = 0;
  ecTimer = 0;
  ecMeasureState = ecInMeasure;

  ecControlled = false;
  ecControlTimer = 0;
  ecWorkTimer = 0;
  ecRequestedTimerInterval = 0;
  ecMachineState = ms_Normal;
  ecCheckRetries = 0;
  absDiffPPM = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool ECModule::out(uint8_t linkType, uint8_t MCPAddress, uint8_t pin, uint8_t level, bool setMode)
{
  if(linkType == linkDirect)
  {
    if(pin != UNBINDED_PIN && (EEPROMSettingsModule::SafePin(pin)))
    {
      if(setMode)
      {
         WORK_STATUS.PinMode(pin,OUTPUT);
      }
      WORK_STATUS.PinWrite(pin,level);

      return true;
    }
  }
  else
  if(linkType == linkMCP23S17)
  {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      
        if(pin != UNBINDED_PIN)
        {
          if(setMode)
          {
            WORK_STATUS.MCP_SPI_PinMode(MCPAddress,pin,OUTPUT);
          }
          WORK_STATUS.MCP_SPI_PinWrite(MCPAddress,pin,level);

          return true;
        }

        
      #endif
  }
  else
  if(linkType == linkMCP23017)
  {
    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0

        if(pin != UNBINDED_PIN)
        {
          if(setMode)
          {
            WORK_STATUS.MCP_I2C_PinMode(MCPAddress,pin,OUTPUT);
          }
          WORK_STATUS.MCP_I2C_PinWrite(MCPAddress,pin,level);

          return true;
        }

    #endif
  }  

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::ReadECSensors()
{
   switch(ecMeasureState)
    {
      case ecWaitInterval: // ждём истечения интервала между замерами EC
      {
        if(millis() - ecTimer >= EC_READ_INTERVAL) // если настало время читать с датчика
        {
          ecSamplesDone = 0; // нет собранных семплов
          ecDataArray[0] = ecDataArray[1] = ecDataArray[2] = ecDataArray[3] = 0; // нет данных семплов
          ecMeasureState = ecInMeasure; // переключаемся на ветку измерения          

          ECBinding bnd = HardwareBinding->GetECBinding();
          uint8_t stopCntr = 0;
          for(size_t k=0;k<sizeof(bnd.SensorPins)/sizeof(bnd.SensorPins[0]);k++,stopCntr++)
          {
            if(stopCntr > 3)
            {
              break;
            }

            if(bnd.SensorPins[k] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.SensorPins[k]))
            {
              analogRead(bnd.SensorPins[k]); // читаем впустую, чтобы исключить шум
            }
          }
          
          ecTimer = millis(); // сохраняем значение таймера
        }
      }
      break; // ecWaitInterval

      case ecInMeasure: // собираем семплы
      {
        if(millis() - ecTimer >= EC_SAMPLES_INTERVAL) // настало время собирать очередной семпл
        {
          ecSamplesDone++; // увеличиваем кол-во семплов

          ECBinding bnd = HardwareBinding->GetECBinding();
          uint8_t stopCntr = 0;
          for(size_t k=0;k<sizeof(bnd.SensorPins)/sizeof(bnd.SensorPins[0]);k++,stopCntr++)
          {
            if(stopCntr > 3) // чтобы не вылезти за границы массива, макс 4 датчика
            {
              break;
            }

            if(bnd.SensorPins[k] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.SensorPins[k]))
            {
              ecDataArray[k] += analogRead(bnd.SensorPins[k]); // читаем впустую, чтобы исключить шум
            }
          }          

          ecTimer = millis(); // сохраняем значение таймера до следующего замера

          if(ecSamplesDone >= EC_SAMPLES_PER_MEASURE) // сэмплы собраны, можно усреднять
          {
            
            // преобразуем полученное значение в среднее
            stopCntr = 0;
            uint8_t ecSensorCntr = 0;
            for(size_t k=0;k<sizeof(bnd.SensorPins)/sizeof(bnd.SensorPins[0]);k++,stopCntr++)
            {
              if(stopCntr > 3) // чтобы не вылезти за границы массива, макс 4 датчика
              {
                break;
              }
  
              if(bnd.SensorPins[k] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.SensorPins[k]))
              {
                float avgSample = (ecDataArray[k]*1.0)/ecSamplesDone;

                // делаем перерасчёт значения в PPM
                // формула такая: датчик у нас 5В, есть макс диапазон в PPM. Т.е. простая пропорция:
                // 5V = maxPpm
                // ADC_V = N ppm
                // N = (maxPpm*ADC_V)/5.0;

                // в avgSample - усреднённое значение с АЦП
                avgSample *= bnd.MaxPpm;
                avgSample /= 5.0;

                // теперь преобразовываем в нужную величину
                uint16_t sensorValue = avgSample;

                // сохраняем состояние с датчика
                State.UpdateState(StateEC,ecSensorCntr,(void*)&sensorValue);                    
                ecSensorCntr++;
              }
            }          

            // переключаемся на ветку ожидания интервала до следующего замера
            ecMeasureState = ecWaitInterval;
            ecTimer = millis();
            
          } // if(ecSamplesDone >= EC_SAMPLES_PER_MEASURE)
          
        } // if(millis() - ecTimer >= EC_SAMPLES_INTERVAL)
      }
      break; // ecInMeasure
      
    } // switch  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::ReadPHSensor()
{
    if(!phSensorAdded) // нет датчика, прикреплённого к контроллеру, не надо ничего читать
    {
      return;
    }

    switch(phMeasureState)
    {
      case phWaitInterval: // ждём истечения интервала между замерами pH
      {
        if(millis() - phTimer >= EC_PH_READ_INTERVAL) // если настало время читать с датчика
        {
          phSamplesDone = 0; // нет собранных семплов
          phDataArray = 0; // нет данных семплов
          phMeasureState = phInMeasure; // переключаемся на ветку измерения
          analogRead(phSensorPin); // читаем впустую, чтобы исключить шум
          phTimer = millis(); // сохраняем значение таймера
        }
      }
      break; // phWaitInterval

      case phInMeasure: // собираем семплы
      {
        if(millis() - phTimer >= EC_PH_SAMPLES_INTERVAL) // настало время собирать очередной семпл
        {
          phSamplesDone++; // увеличиваем кол-во семплов
          phDataArray += analogRead(phSensorPin); // читаем с пина
          phTimer = millis(); // сохраняем значение таймера до следующего замера

          if(phSamplesDone >= EC_PH_SAMPLES_PER_MEASURE) // сэмплы собраны, можно усреднять
          {
            
            // преобразуем полученное значение в среднее
             float avgSample = (phDataArray*1.0)/phSamplesDone;
             ECPHBinding bnd = HardwareBinding->GetECPHBinding();

             // считаем вольтаж
            float voltage = avgSample*bnd.VRef/1024000.0; // у нас VRef - в милливольтах

           // теперь получаем значение pH
           // у нас есть EC_PH_MV_PER_7_PH 2000 - кол-во милливольт, при  которых датчик показывает 7 pH
           // следовательно, в этом месте мы должны получить коэффициент 350 (например), который справедлив для значения 2000 mV при 7 pH
           // путём нехитрой формулы получаем, что коэффициент здесь будет равен 700000/EC_PH_MV_PER_7_PH
           float coeff = 700000/bnd.MVPer7Ph;
           // и применяем этот коэффициент
           uint32_t phValue = voltage*coeff;
           // вышеприведённые подсчёты pH справедливы для случая "больше вольтаж - больше pH",
           // однако нам надо учесть и реверсивный случай, когда "больше вольтаж - меньше pH".
           if(bnd.ReversiveMeasure == 1)
           {
            // считаем значение pH в условиях реверсивных измерений
            int16_t rev = phValue - 700; // поскольку у нас 7 pH - это средняя точка, то в условии реверсивных изменений от
            // средней точки pH (7.0) надо отнять разницу между значением 7 pH и полученным значением, что мы и делаем
            phValue = 700 - rev;
           }


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

            // переключаемся на ветку ожидания интервала до следующего замера
            phMeasureState = phWaitInterval;
            phTimer = millis();
            
          } // if(phSamplesDone >= EC_PH_SAMPLES_PER_MEASURE)
          
        } // if(millis() - phTimer >= EC_PH_SAMPLES_INTERVAL)
      }
      break; // phInMeasure
      
    } // switch
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::ApplyCalculation(Temperature* temp)
{


  // Эта функция вызывается при любом обновлении значения с датчика pH, откуда бы это значение
  // ни пришло. Здесь мы можем применить к значению поправочные факторы калибровки, в том числе
  // по температуре, плюс применяем к показаниям поправочное число.
  
  if(!temp)
  {
    return;
  }

  if(!temp->HasData())
  {
    return;
  }

  // теперь проверяем, можем ли мы применить калибровочные поправки?
  // для этого все вольтажи, наличие показаний с датчика температуры
  // и выставленная настройки температуры калибровочных растворов
  // должны быть актуальными.

  if(ph4Voltage < 1 || ph7Voltage < 1 || ph10Voltage < 1 || phTemperatureSensorIndex < 0 
  || !phSamplesTemperature.HasData() || phSamplesTemperature.Value > 100 || phSamplesTemperature.Value < 0)
  {
    return;
  }

  AbstractModule* tempModule = MainController->GetModuleByID("STATE");
  if(!tempModule)
  {
    return;
  }

  OneState* os = tempModule->State.GetState(StateTemperature,phTemperatureSensorIndex);
  if(!os)
  {
    return;
  }

  TemperaturePair tempPair = *os;
  Temperature tempPH = tempPair.Current;

  if(!tempPH.HasData() || tempPH.Value > 100 || tempPH.Value < 0)
  {
    return;
  }

  Temperature tDiff = tempPH - phSamplesTemperature;

  long ulDiff = tDiff.Value;
  uint8_t sign = tDiff.Value < 0 ? -1 : 1;
  ulDiff *= 100;
  ulDiff += tDiff.Fract*sign;
  
  float fTempDiff = ulDiff/100.0;

  // теперь можем применять факторы калибровки.
  // сначала переводим текущие показания в вольтаж, приходится так делать, поскольку
  // они приходят уже нормализованными.
  long curPHVoltage = temp->Value;

  curPHVoltage *= 100;
  curPHVoltage += temp->Fract + phCalibration; // прибавляем сотые доли показаний, плюс сотые доли поправочного числа
  curPHVoltage *= 100;

  ECPHBinding bnd = HardwareBinding->GetECPHBinding();

 // у нас есть EC_PH_MV_PER_7_PH 2000 - кол-во милливольт, при  которых датчик показывает 7 pH
 // следовательно, в этом месте мы должны получить коэффициент 35 (например), который справедлив для значения 2000 mV при 7 pH
 // путём нехитрой формулы получаем, что коэффициент здесь будет равен 70000/EC_PH_MV_PER_7_PH
 float coeff = 70000/bnd.MVPer7Ph;
  
  curPHVoltage /= coeff; // например, 7,00 pH  сконвертируется в 2000 милливольт, при значении  EC_PH_MV_PER_7_PH == 2000

  long phDiff = ph4Voltage;
  phDiff -= ph10Voltage;

  float sensitivity = phDiff/6.0;
  sensitivity = sensitivity + fTempDiff*0.0001984;

  phDiff = ph7Voltage;
  phDiff -= curPHVoltage;

  float calibratedPH;
  if(bnd.ReversiveMeasure == 1)
  {
    calibratedPH = 7.0 - phDiff/sensitivity; // реверсивное изменение вольтажа при нарастании pH
  }
  else
  {
    calibratedPH = 7.0 + phDiff/sensitivity; // прямое изменение вольтажа при нарастании pH
  }

  // теперь переводим всё это обратно в понятный всем вид
  uint16_t phVal = calibratedPH*100;

  // и сохраняем это дело в показания датчика
  temp->Value = phVal/100;
  temp->Fract = phVal%100;

}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::Setup()
{

  // читаем настройки
  ReadSettings();


 // теперь смотрим - если у нас пин pH не 255 - значит, надо добавить состояние
  if(phSensorPin > 0 && phSensorPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(phSensorPin))
  {
    State.AddState(StatePH,0); // добавляем датчик pH, прикреплённый к контроллеру
    WORK_STATUS.PinMode(phSensorPin,INPUT);
    digitalWrite(phSensorPin,HIGH);

    phSensorAdded = true;
  }

  // настраиваем выхода контроля EC
  ECBinding bnd = HardwareBinding->GetECBinding();
  
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinA,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinB,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinC,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinPhPlus,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinPhMinus,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PinWater,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.WaterValve,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.WaterContour,!bnd.Level,true);
  out(bnd.LinkType,bnd.MCPAddress,bnd.PumpPin,!bnd.Level,true);
  
  // добавляем датчики EC в систему

  uint8_t ecSensorsCounter = 0;
  for(size_t k=0;k<sizeof(bnd.SensorPins)/sizeof(bnd.SensorPins[0]);k++)
  {
    if(bnd.SensorPins[k] != UNBINDED_PIN && EEPROMSettingsModule::SafePin(bnd.SensorPins[k]))
    {
      State.AddState(StateEC,ecSensorsCounter); 
      ecSensorsCounter++;
    }
  }

  // сохраняем статусы работы pH
  SAVE_STATUS(PH_FLOW_ADD_BIT,0); // жидкость в бак не поступает
  SAVE_STATUS(PH_MIX_PUMP_BIT,0); // насос перемешивания выключен
  SAVE_STATUS(PH_PLUS_PUMP_BIT,0); // насос + pH выключен
  SAVE_STATUS(PH_MINUS_PUMP_BIT,0); // насос - pH выключен
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::SaveSettings()
{
  uint32_t addr = PH_SETTINGS_EEPROM_ADDR;

  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

  MemWrite(addr++,phSensorPin);

  // пишем калибровку
  byte cal[2];
  memcpy(cal,&phCalibration,2);
  MemWrite(addr++,cal[0]);
  MemWrite(addr++,cal[1]);

  // пишем вольтаж раствора 4 pH
  byte* pB = (byte*) &ph4Voltage;
  for(size_t i=0;i<sizeof(ph4Voltage);i++)
  {
    MemWrite(addr++,*pB++);
  }
    

  // пишем вольтаж раствора 7 pH
  pB = (byte*) &ph7Voltage;
  for(size_t i=0;i<sizeof(ph7Voltage);i++)
  {
    MemWrite(addr++,*pB++);
  }
    

  // пишем вольтаж раствора 10 pH
  pB = (byte*) &ph10Voltage;
  for(size_t i=0;i<sizeof(ph10Voltage);i++)
  {
    MemWrite(addr++,*pB++);
  }
      

  // пишем индекс датчика температуры
  MemWrite(addr++,phTemperatureSensorIndex);

  // пишем показания температуры при калибровке
  cal[0] = phSamplesTemperature.Value;
  cal[1] = phSamplesTemperature.Fract;

  MemWrite(addr++,cal[0]);
  MemWrite(addr++,cal[1]);

  // пишем уставку, за которой следим
  pB = (byte*) &phTarget;
  for(size_t i=0;i<sizeof(phTarget);i++)
  {
    MemWrite(addr++,*pB++);
  }
    

  // пишем гистерезис
  pB = (byte*) &phHisteresis;
  for(size_t i=0;i<sizeof(phHisteresis);i++)
  {
    MemWrite(addr++,*pB++);
  }
  
  pB = (byte*) &phMixPumpTime;
  for(size_t i=0;i<sizeof(phMixPumpTime);i++)
  {
    MemWrite(addr++,*pB++);
  }

  pB = (byte*) &phReagentPumpTime;
  for(size_t i=0;i<sizeof(phReagentPumpTime);i++)
  {
    MemWrite(addr++,*pB++);
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::ReadSettings()
{
  uint32_t addr = PH_SETTINGS_EEPROM_ADDR;
  if(MemRead(addr++) != SETT_HEADER1)
  {
    return;
  }

  if(MemRead(addr++) != SETT_HEADER2)
  {
    return;
  }

  phSensorPin =  MemRead(addr++); 

  ECPHBinding bnd = HardwareBinding->GetECPHBinding();
  phSensorPin = bnd.SensorPin;

  byte cal[2];
  cal[0] = MemRead(addr++);
  cal[1] = MemRead(addr++);

  if(cal[0] == 0xFF && cal[1] == 0xFF) // нет калибровки
  {
    phCalibration = EC_PH_DEFAULT_CALIBRATION;
  }
  else
  {
    memcpy(&phCalibration,cal,2); // иначе копируем сохранённую калибровку
  }

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
    phTarget = EC_PH_DEFAULT_TARGET;   

    pB = (byte*) &phHisteresis;
    for(size_t i=0;i<sizeof(phHisteresis);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }
  
  if(phHisteresis == 0xFFFF)
    phHisteresis = EC_PH_DEFAULT_HISTERESIS;   

    pB = (byte*) &phMixPumpTime;
    for(size_t i=0;i<sizeof(phMixPumpTime);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }

  
  if(phMixPumpTime == 0xFFFF)
    phMixPumpTime = EC_PH_DEFAULT_MIX_PUMP_TIME;   

    pB = (byte*) &phReagentPumpTime;
    for(size_t i=0;i<sizeof(phReagentPumpTime);i++)
    {
      *pB = MemRead(addr++);
       pB++;
    }
 
  if(phReagentPumpTime == 0xFFFF)
    phReagentPumpTime = EC_PH_DEFAULT_REAGENT_PUMP_TIME;  
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::Update()
{ 
  // обновление модуля
  
  ReadPHSensor(); // читаем значение с датчика pH
  ReadECSensors(); // читаем значения с датчиков EC

  /////////////////////////////////////////////////////////////////////////////////////
  //PAID CODE BEGIN
  /////////////////////////////////////////////////////////////////////////////////////
 
  /////////////////////////////////////////////////////////////////////////////////////
  //PAID CODE END
  /////////////////////////////////////////////////////////////////////////////////////
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  ECModule::ExecCommand(const Command& command, bool wantAnswer)
{
 UNUSED(wantAnswer);
  PublishSingleton = PARAMS_MISSED;
    
  
  uint8_t argsCount = command.GetArgsCount();
  if(argsCount < 1)
  {
    if(command.GetType() == ctGET)
    {
    // попросили отдать все показания с датчиков
    
      PublishSingleton.Flags.Status = true;
      if(wantAnswer) 
      {
         PublishSingleton = "";

         uint8_t _cnt = State.GetStateCount(StateEC);

         PublishSingleton << _cnt;
         
         for(uint8_t i=0;i<_cnt;i++)
         {
            OneState* os = State.GetStateByOrder(StateEC,i);
            if(os)
            {
              ECPair lp = *os;

              PublishSingleton << PARAM_DELIMITER;
              PublishSingleton << lp.Current;
                              
            } // if(os)
         } // for
     
      }

    } // if(command.GetType() == ctGET)
   
    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
  } // if(argsCount < 1)
  
  if(command.GetType() == ctGET)
  {
      String which = command.GetArg(0);
      /*
      if(which == F("SETTINGS")) // CTGET=CO2|SETTINGS, returns OK=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {  
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        << PARAM_DELIMITER << settings.weekdays
        << PARAM_DELIMITER << settings.startTime
        << PARAM_DELIMITER << settings.endTime
        << PARAM_DELIMITER << settings.ppm
        << PARAM_DELIMITER << settings.histeresis
        ;
      } // if(which == F("SETTINGS"))
      else
      if(which == F("ACTIVE")) // CTGET=CO2|ACTIVE
      {
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        ;
        
      } // if(which == F("ACTIVE"))
      else // возможно, запросили показания датчика, CTGET=CO2|0, CTGET=CO2|1 и т.п.
      */
      {
         uint8_t idx = which.toInt();
          uint8_t _cnt = State.GetStateCount(StateEC);
          
          if(idx >= _cnt)
          {
            // плохой индекс
            if(wantAnswer)
            {
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << NOT_SUPPORTED;
            }
          } // плохой индекс
          else
          {
             if(wantAnswer)
             {
              PublishSingleton = which;
             }
              
             OneState* stateEC = State.GetStateByOrder(StateEC,idx);
             if(stateEC)
             {
                PublishSingleton.Flags.Status = true;
                ECPair co2p = *stateEC;
                
                if(wantAnswer)
                {
                    PublishSingleton << PARAM_DELIMITER << (co2p.Current);
                }
             } // if
            
          } // else нормальный индекс        
        
      } // else
  } // ctGET
  else
  if(command.GetType() == ctSET)
  {
    /*
      String which = command.GetArg(0);

      if(which == F("SETTINGS")) // CTSET=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {
          if(argsCount > 6)
          {
            settings.active = atoi(command.GetArg(1));
            settings.weekdays = atoi(command.GetArg(2));
            settings.startTime = atol(command.GetArg(3));
            settings.endTime = atol(command.GetArg(4));
            settings.ppm = atoi(command.GetArg(5));
            settings.histeresis = atoi(command.GetArg(6));

            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
      } // if(which == F("SETTINGS")) 
      else
      if(which == F("ACTIVE")) // CTSET=CO2|ACTIVE|active flag
      {
          if(argsCount > 1)
          {
            settings.active = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.active;
          }
      } // if(which == F("ACTIVE"))
       else
      if(which == F("PPM")) // CTSET=CO2|PPM|ppm value
      {
          if(argsCount > 1)
          {
            settings.ppm = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.ppm;
          }
      } // if(which == F("PPM"))
     */ 
  } // ctSET

    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------


