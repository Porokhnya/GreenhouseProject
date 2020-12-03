#include "HumidityModule.h"
#include "ModuleController.h"
#include "SHT1x.h"
#include "EEPROMSettingsModule.h"
#include "WeatherStation.h"
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------------
void HumidityModule::Setup()
{
  // настройка модуля тут

  lastSi7021StrobeBreakPin = 0;

  HumidityBinding hbnd = HardwareBinding->GetHumidityBinding();

  uint8_t sensorCntr = 0;
  for(size_t i=0;i<sizeof(hbnd.Type)/sizeof(hbnd.Type[0]);i++)
  {
    uint8_t t = hbnd.Type[i];
    uint8_t pin1 = hbnd.Pin1[i];
    
    if(t > MISOL || t < DHT11) // неизвестный науке датчик
    {
      continue;
    }

      State.AddState(StateHumidity,sensorCntr); // поддерживаем и влажность,
      State.AddState(StateTemperature,sensorCntr); // и температуру

      sensorCntr++;

      // для Si7021 применяем пин разрыва строба, если указан
      if(t == SI7021 && pin1 != UNBINDED_PIN && pin1 > 0 && EEPROMSettingsModule::SafePin(pin1))
      {
        // есть привязанный пин для разрыва строба - настраиваем его
        WORK_STATUS.PinMode(pin1,OUTPUT);
      }
  } // for
   
 }
//--------------------------------------------------------------------------------------------------------------------------------------
HumidityAnswer HumidityModule::QuerySensor(uint8_t sensorNumber, uint8_t pin, uint8_t pin2, HumiditySensorType type)
{
  UNUSED(sensorNumber);
  
  dummyAnswer.IsOK = false;
  dummyAnswer.Humidity = NO_TEMPERATURE_DATA;
  dummyAnswer.Temperature = NO_TEMPERATURE_DATA;
  
  switch(type)
  {
    case NO_HUMIDITY_SENSOR:
    {
      return dummyAnswer;  
    }
    break;
    
    case DHT11:
    {
      if(pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin))
      {
        DHTSupport dhtQuery;
        return dhtQuery.read(pin,DHT_11);
      }
      else
      {
        return dummyAnswer;
      }
    }
    break;
    
    case DHT2x:
    {      
      if(pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin))
      {
        DHTSupport dhtQuery;
        return dhtQuery.read(pin,DHT_2x);
      }
      else
      {
        return dummyAnswer;
      }
    }
    break;

    case SI7021:
    {
      
      Si7021 si7021;

      // сначала смотрим - не надо ли разорвать строб у предыдущего Si7021 ?

      if(lastSi7021StrobeBreakPin && pin != lastSi7021StrobeBreakPin)
      {
         HumidityBinding hbnd = HardwareBinding->GetHumidityBinding();
       // Serial.println("SI7021 STROBE OFF!!!");
         // предыдущему датчику был назначен пин для разрыва строба - рвём ему строб
         if(lastSi7021StrobeBreakPin != UNBINDED_PIN && lastSi7021StrobeBreakPin > 0 && EEPROMSettingsModule::SafePin(lastSi7021StrobeBreakPin))
         {
            WORK_STATUS.PinWrite(lastSi7021StrobeBreakPin,!hbnd.StrobeLevel);
         }
         lastSi7021StrobeBreakPin = 0; // сбрасываем статус, т.к. мы уже разорвали этот строб
      }

      // тут смотрим - не назначен ли у нас пин для разрыва строба?
      if(pin > 0 && pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin))
      {
         HumidityBinding hbnd = HardwareBinding->GetHumidityBinding();
        //Serial.println("SI7021 STROBE ON!!!");
            // нам назначена линия разрыва строба - мы должны её включить
            lastSi7021StrobeBreakPin = pin; // запоминаем, какую линию включали
            // включаем её
            WORK_STATUS.PinWrite(pin,hbnd.StrobeLevel);
      }

      // теперь смотрим - проинициализирован ли датчик?
      bool si7021Inited = false;
      for(size_t kk=0;kk<si7021InitStatus.size();kk++)
      {
        if(si7021InitStatus[kk] == sensorNumber)
        {
          si7021Inited = true;
          break;
        }
      }
      
      if(!si7021Inited)
      {
        //Serial.println("SI7021 NOT INITED!!!");

         // датчик не проинициализирован         
          si7021InitStatus.push_back(sensorNumber); // запоминаем, что мы проинициализировали датчик

         // и инициализируем его
         si7021.begin();
         
        // Serial.println("SI7021 NOW INITED.");
      }

     // Serial.println("READ FROM SI7021");
      // теперь мы можем читать с датчика - предыдущий строб, если был - разорван, текущий, если есть - включен
      return si7021.read();
    }
    break;

    case SHT10:
    {
        if(pin != UNBINDED_PIN && pin2 != UNBINDED_PIN && EEPROMSettingsModule::SafePin(pin) && EEPROMSettingsModule::SafePin(pin2))
        {
        SHT1x sht(pin,pin2);
        float temp = sht.readTemperatureC();
        float hum = sht.readHumidity();
  
        if(((int)temp) != -40)
        {
          // has temperature
          int conv = temp * 100;
          dummyAnswer.Temperature = conv/100;
          dummyAnswer.TemperatureDecimal = abs(conv%100);
        }
  
        if(!(hum < 0))
        {
          // has humidity
          int conv = hum*100;
          dummyAnswer.Humidity = conv/100;
          dummyAnswer.HumidityDecimal = conv%100;
        }
  
        dummyAnswer.IsOK = (dummyAnswer.Temperature != NO_TEMPERATURE_DATA) && (dummyAnswer.Humidity != NO_TEMPERATURE_DATA);
  
        return dummyAnswer;
      } // if
      else
      {
        return dummyAnswer;
      }
    }
    break;

    case MISOL: // датчики с метеостанции типа MISOL
    {
        dummyAnswer.Temperature = WeatherStation.Temperature;
        dummyAnswer.TemperatureDecimal = WeatherStation.TemperatureDecimal;

        dummyAnswer.Humidity = WeatherStation.Humidity;
        dummyAnswer.HumidityDecimal = WeatherStation.HumidityDecimal;

        dummyAnswer.IsOK = (dummyAnswer.Temperature != NO_TEMPERATURE_DATA) || (dummyAnswer.Humidity != NO_TEMPERATURE_DATA);

      return dummyAnswer;
    }
    break;
  }
  return dummyAnswer;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void HumidityModule::Update()
{ 
  // обновление модуля тут
   static uint32_t timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - timer;
  timer = now;
 
  lastUpdateCall += dt;
  if(lastUpdateCall < HUMIDITY_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
  {
    return;
  }
  else
  {
    lastUpdateCall = 0; 
  }

  // получаем данные с датчиков влажности
  HumidityBinding hbnd = HardwareBinding->GetHumidityBinding();

  uint8_t sensorCntr=0;
  for(size_t i=0;i<sizeof(hbnd.Type)/sizeof(hbnd.Type[0]);i++)
  {
      uint8_t pin1 = hbnd.Pin1[i];
      uint8_t pin2 = hbnd.Pin2[i];
      uint8_t type = hbnd.Type[i];

      if(type > MISOL || type < DHT11) // неизвестный науке датчик
      {
        continue;
      }

      Humidity h;
      Temperature t;
      HumidityAnswer answer = QuerySensor(sensorCntr, pin1, pin2, (HumiditySensorType)type);

        if(answer.IsOK)
        {
          h.Value = answer.Humidity;
          h.Fract = answer.HumidityDecimal;
          
          t.Value = answer.Temperature;
          t.Fract = answer.TemperatureDecimal;
  
          // convert to Fahrenheit if needed
          #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
           t = Temperature::ConvertToFahrenheit(t);
          #endif
          
        } // if

      // сохраняем данные в состоянии модуля - индексы мы назначаем сами, последовательно, поэтому дыр в нумерации датчиков нет
      State.UpdateState(StateTemperature,sensorCntr,(void*)&t);
      State.UpdateState(StateHumidity,sensorCntr,(void*)&h);
        
      sensorCntr++;

  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  HumidityModule::ExecCommand(const Command& command,bool wantAnswer)
{

  if(wantAnswer) 
    PublishSingleton = NOT_SUPPORTED;

  if(command.GetType() == ctSET) // установка свойств
  {
    uint8_t argsCnt = command.GetArgsCount();
    if(argsCnt > 0)
    {
    String param = command.GetArg(0);
    
         if(param == F("DATA")) // установить значение на датчике, CTSET=HUMIDITY|DATA|idx|temperature|humidity
         {
            if (argsCnt > 3)
            {
                uint8_t sensorIndex = (uint8_t) atoi(command.GetArg(1));
                int16_t temperature =  (long) atol(command.GetArg(2));
                int16_t humidity =  (long) atol(command.GetArg(3));
        
                uint8_t _tempCnt = State.GetStateCount(StateTemperature);
                
                if(sensorIndex >= _tempCnt)
                {
                  uint8_t toAdd = (sensorIndex - _tempCnt) + 1;
        
                    for(uint8_t qa = 0; qa < toAdd; qa++)
                    {
                      State.AddState(StateTemperature,_tempCnt + qa);
                    }
                } 

                uint8_t _humCnt = State.GetStateCount(StateHumidity);
                
                if(sensorIndex >= _humCnt)
                {
                  uint8_t toAdd = (sensorIndex - _humCnt) + 1;
        
                    for(uint8_t qa = 0; qa < toAdd; qa++)
                    {
                      State.AddState(StateHumidity,_humCnt + qa);
                    }
                }                                 
                     
                
                    Temperature t;
                    t.Value = temperature/100;
              
                    t.Fract = abs(temperature%100);
              
                    // convert to Fahrenheit if needed
                    #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
                     t = Temperature::ConvertToFahrenheit(t);
                    #endif      
                       
                  State.UpdateState(StateTemperature,sensorIndex,(void*)&t);

                  Humidity h;
                  h.Value = humidity/100;
                  h.Fract = abs(humidity%100);
                  
                  State.UpdateState(StateHumidity,sensorIndex,(void*)&h);

                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
                  #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
                  OneState* dynReset = State.GetState(StateTemperature,sensorIndex);
                  if(dynReset)
                  {
                    DynamicSensorsReset->Observe(dynReset);
                  }

                  dynReset = State.GetState(StateHumidity,sensorIndex);
                  if(dynReset)
                  {
                    DynamicSensorsReset->Observe(dynReset);
                  }
                  #endif
                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
        
                PublishSingleton.Flags.Status = true;
                PublishSingleton = param;
                PublishSingleton << PARAM_DELIMITER << sensorIndex << PARAM_DELIMITER << REG_SUCC;              
              
            } // if (argsCnt > 3)
         } // DATA
    } // argsCount > 0
  } // ctSET
  else
  if(command.GetType() == ctGET) // запрос свойств
  {
      uint8_t argsCnt = command.GetArgsCount();
      if(argsCnt < 1)
      {
        if(wantAnswer) 
          PublishSingleton = PARAMS_MISSED; // не хватает параметров
        
      } // argsCnt < 1
      else
      {
        // argsCnt >= 1
        String param = command.GetArg(0);
        if(param == PROP_CNT) // запросили данные о кол-ве датчиков: CTGET=HUMIDITY|CNT
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = PROP_CNT; 
            uint8_t _cnt = State.GetStateCount(StateHumidity);
            PublishSingleton << PARAM_DELIMITER << _cnt;
          }
        } // PROP_CNT
        else
        if(param == ALL) // запросили показания со всех датчиков
        {
          PublishSingleton.Flags.Status = true;
          uint8_t _cnt = State.GetStateCount(StateHumidity);
          if(wantAnswer) 
            PublishSingleton = _cnt;
          
          for(uint8_t i=0;i<_cnt;i++)
          {

             OneState* stateTemp = State.GetStateByOrder(StateTemperature,i);
             OneState* stateHumidity = State.GetStateByOrder(StateHumidity,i);
             if(stateTemp && stateHumidity)
             {
                TemperaturePair tp = *stateTemp;
                HumidityPair hp = *stateHumidity;
              
                if(wantAnswer) 
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current) << PARAM_DELIMITER << (tp.Current);
                }
             } // if
          } // for
                    
        } // all data
        else
        if(param != GetID()) // если только не запросили без параметров
        {
          // запросили показания с датчика по индексу
          uint8_t idx = param.toInt();
          uint8_t _cnt = State.GetStateCount(StateHumidity);
          
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
              
             OneState* stateTemp = State.GetStateByOrder(StateTemperature,idx);
             OneState* stateHumidity = State.GetStateByOrder(StateHumidity,idx);
             if(stateTemp && stateHumidity)
             {
                PublishSingleton.Flags.Status = true;

                TemperaturePair tp = *stateTemp;
                HumidityPair hp = *stateHumidity;
                
                if(wantAnswer)
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current) << PARAM_DELIMITER << (tp.Current);
                }
             } // if
            
          } // else нормальный индекс
          
        } // else показания по индексу
        
      } // else
    
  } // ctGET
  

  MainController->Publish(this,command);    
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------

