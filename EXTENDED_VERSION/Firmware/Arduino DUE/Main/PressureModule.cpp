#include "PressureModule.h"
#include "ModuleController.h"
#include "ZeroStreamListener.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PRESSURE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
PressureModule* Pressure = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
PressureModule::PressureModule() : AbstractModule("PRESS") 
{
	Pressure = this;
	hasSensor = false;
	measureTimerEnabled = false;
	updateTimer = 0;
  predictTimer = 0;
  predict = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PressureModule::Setup()
{
  // настройка модуля тут

  for(uint8_t i=0;i<PREDICT_MEASURES;i++)
  {
    predictTime.push_back(i);
  }
  
  pressureData.Value = NO_PRESSURE_DATA;
  
  hasSensor = sensor.begin();
  readPressure();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PressureModule::Update()
{ 
  // обновление модуля тут

  if(millis() - predictTimer > PREDICT_INTERVAL)
  {
     // пора пополнять список
     if(pressureData.Value != NO_PRESSURE_DATA)
     {
       // есть данные
       int32_t dt = pressureData.Value*100 + pressureData.Fract;
       
       while(predictList.size() < PREDICT_MEASURES)
       {
        predictList.push_back(dt);
       }

       // сдвигаем в голову
       for(size_t i=1;i<predictList.size();i++)
       {
          predictList[i-1] = predictList[i];
       }

       // сохраняем новое значение в хвосте
       predictList[predictList.size()-1] = dt;

       // считаем вероятность

        int32_t sumX = 0;
        int32_t sumY = 0;
        int32_t sumX2 = 0;
        int32_t sumXY = 0;
        for (size_t i = 0; i < predictList.size(); i++) 
        {
          sumX += predictTime[i];
          sumY += predictList[i];
          sumX2 += predictTime[i] * predictTime[i];
          sumXY += predictTime[i] * predictList[i];
        }
        float a = 0;
        a = predictList.size() * sumXY;             // расчёт коэффициента наклона приямой
        a = a - sumX * sumY;
        a = (float)a / (predictList.size() * sumX2 - sumX * sumX);
        int32_t delta = a * predictList.size();      // расчёт изменения давления
        predict = map(delta, -250, 250, -100, 100);  // пересчитать в проценты

        // у нас есть центральная точка - 0. Всё, что меньше нуля - вероятность осадков
        // 0 - фифти-фифти, больше нуля - хорошая погода.
        // т.е. -100 - вероятность 100%, 100 - 0% вероятность

       // Serial.println(String(predictList[predictList.size()-1]) + " " + String(delta) + " " + String(predict));
     } // if has data

     

     predictTimer = millis();
     
  } // timer
  
  
  if(measureTimerEnabled)
  {
	 updateTimer = millis();
	  
	  if(millis() - measureTimerTime > measureTimerInterval)
	  {
		  
		  measureIterator++;
		  		  
		  if(measureIterator > 1)
		  {
			  // закончилась конвертация давления
			  measureTimerEnabled = false;
			  measureIterator = 0;
			  
			  if(hasSensor)
			  {
				  double P;
				  char status = sensor.getPressure(P,pressureTemperature);
				  
				  if(status !=0)
				  {
					int computedAlt = sensor.altitude(P,1013.25); // получили высоту относительно среднего давления на уровне моря            
					double PAbsSealevel = sensor.sealevel(P, computedAlt);  // получили давление над уровнем моря для измеренной высоты      
					int16_t userSealevel = MainController->GetSettings()->GetSealevel(); // данные высоты, введённые пользователем    
					double PUserSealevel = sensor.sealevel(P, userSealevel);  // получили давление над уровнем моря для высоты пользователя     
					P -= (PUserSealevel - P) + (PAbsSealevel - P); // вычислили давление в точке измерения
			
					//P = sensor.sealevel(P, MainController->GetSettings()->GetSealevel());
					P *= 0.750064; // в мм. рт. ст
					
					int32_t pVal = P*100;

          // смотрим на предмет пополнения списка предсказаний
          if(!predictList.size())
          {
            // список пустой, надо наполнить его исходными данными
            for(int i=0;i<PREDICT_MEASURES;i++)
            {
              predictList.push_back(pVal);
            }
          }
         
					
					pressureData.Value = pVal/100;
					pressureData.Fract = pVal%100;
					
				  }
			  } // hasSensor			  
		  } // if(measureIterator > 1)
		  else
		  {
			  // закончилась конвертация температуры	  
				if(sensor.getTemperature(pressureTemperature))
        {
          // получили температуру системы, смотрим - надо ли её пихнуть в нужное место?
          #if SYSTEM_TEMP_SOURCE == 1 // температура системы определяется как температура с датчика BMP180

          if(ZeroStream)
          {
              int32_t tempI = pressureTemperature*100;
              int8_t tVal = tempI/100;
              uint8_t tFract = abs(tempI)%100;
              Temperature t;
              t.Value = tVal;
              t.Fract = tFract;

              // convert to Fahrenheit if needed
              #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
               t = Temperature::ConvertToFahrenheit(t);
              #endif  

              ZeroStream->State.UpdateState(StateTemperature,0,(void*)&t);
              
          } // if(ZeroStream)
          
          #endif // SYSTEM_TEMP_SOURCE == 1
          
        } // if(sensor.getTemperature(pressureTemperature))
        else
        {
          // температуру получить не удалось
           #if SYSTEM_TEMP_SOURCE == 1 // температура системы определяется как температура с датчика BMP180
           if(ZeroStream)
           {
              Temperature t;
              ZeroStream->State.UpdateState(StateTemperature,0,(void*)&t);
           } // if(ZeroStream)
           #endif  // SYSTEM_TEMP_SOURCE == 1
        } // else
       
			  measureTimerInterval = sensor.startPressure(3);
				measureTimerTime = millis();
        
		  } // else		  		
		  
	  } // if(millis() - measureTimerTime > measureTimerInterval)
	  
	  return;
  }
  
  if(millis() - updateTimer > PRESSURE_UPDATE_INTERVAL)
  {
	  readPressure();
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  PressureModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  PublishSingleton = UNKNOWN_COMMAND;

  size_t argsCount = command.GetArgsCount();
  
  if(command.GetType() == ctSET) 
  {
        PublishSingleton = NOT_SUPPORTED;
  }
  else
  if(command.GetType() == ctGET) //получить статистику
  {

    if(!argsCount) // нет аргументов
    {
      String result = GetPressure();
      PublishSingleton = result;
      PublishSingleton.Flags.Status = true;
    }
    else
    {

    }// have arguments
    
  } // if
 
 // отвечаем на команду
    MainController->Publish(this,command);
    
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PressureModule::readPressure()
{
	if(!hasSensor)
		return;
	
	measureTimerEnabled = true;
	measureTimerInterval = sensor.startTemperature();
	measureIterator = 0;
	measureTimerTime = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PRESSURE_MODULE
