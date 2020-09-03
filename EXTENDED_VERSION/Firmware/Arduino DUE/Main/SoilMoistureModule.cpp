#include "SoilMoistureModule.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif   
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SOIL_MOISTURE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#define PULSE_TIMEOUT 50000 // 50 миллисекунд на чтение фронта максимум
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::Setup()
{
  // настройка модуля тут

  machineState = SOIL_WAIT_INTERVAL;

  SoilBinding sbnd = HardwareBinding->GetSoilBinding();

  // настраиваем пин управления питанием
  if(sbnd.LinkType != linkUnbinded && sbnd.PowerPin != UNBINDED_PIN)
  {
      if(sbnd.LinkType == linkDirect)
      {
          if(EEPROMSettingsModule::SafePin(sbnd.PowerPin))
          {
            WORK_STATUS.PinMode(sbnd.PowerPin,OUTPUT);
            WORK_STATUS.PinWrite(sbnd.PowerPin,!sbnd.Level);            
          }
      }
      else
      if(sbnd.LinkType == linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0        
              WORK_STATUS.MCP_SPI_PinMode(sbnd.MCPAddress,sbnd.PowerPin,OUTPUT);
              WORK_STATUS.MCP_SPI_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,!sbnd.Level);
        #endif
      }
      else
      if(sbnd.LinkType == linkMCP23017)
      {
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              WORK_STATUS.MCP_I2C_PinMode(sbnd.MCPAddress,sbnd.PowerPin,OUTPUT);
              WORK_STATUS.MCP_I2C_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,!sbnd.Level);
        #endif
      }
    
  } // if(sbnd.LinkType != linkUnbinded)

  // теперь настраиваем датчики почвы
  uint8_t sensorCntr = 0;
  for(size_t i=0;i<sizeof(sbnd.SensorType)/sizeof(sbnd.SensorType[0]);i++)
  {
    if(sbnd.SensorPin[i] != UNBINDED_PIN && sbnd.SensorPin[i] > 0 && EEPROMSettingsModule::SafePin(sbnd.SensorPin[i]))
    {
      WORK_STATUS.PinMode(sbnd.SensorPin[i], INPUT, false);

      if(sbnd.SensorType[i] == FREQUENCY_SOIL_MOISTURE) // частотный датчик влажности почвы
      {
        WORK_STATUS.PinMode(sbnd.SensorPin[i],INPUT);
        WORK_STATUS.PinWrite(sbnd.SensorPin[i],HIGH);
      }
      State.AddState(StateSoilMoisture,sensorCntr); // добавляем датчики влажности почвы
      sensorCntr++;
    } // if
    
  } // for

}
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::Update()
{ 

  // обновление модуля тут
  static uint32_t _timer = 0;
  uint32_t now = millis();
  uint32_t dt = now - _timer;
  _timer = now;
  
  SoilBinding sbnd = HardwareBinding->GetSoilBinding();
  
  switch(machineState)
  {
    case SOIL_WAIT_INTERVAL: // ждём истечения интервала между опросами датчиков
    {
      lastUpdateCall += dt;
      if(lastUpdateCall < SOIL_MOISTURE_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
      {
        return;
      }
      else
      {
        // включаем датчики

        if(sbnd.LinkType != linkUnbinded && sbnd.PowerPin != UNBINDED_PIN)
        {
            if(sbnd.LinkType == linkDirect)
            {
                if(EEPROMSettingsModule::SafePin(sbnd.PowerPin))
                {
                  WORK_STATUS.PinWrite(sbnd.PowerPin,sbnd.Level);            
                }
            }
            else
            if(sbnd.LinkType == linkMCP23S17)
            {
              #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0        
                    WORK_STATUS.MCP_SPI_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,sbnd.Level);
              #endif
            }
            else
            if(sbnd.LinkType == linkMCP23017)
            {
              #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                    WORK_STATUS.MCP_I2C_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,sbnd.Level);
              #endif
            }
          
        } // if(sbnd.LinkType != linkUnbinded)
              
        lastUpdateCall = 0;
        machineState = SOIL_WAIT_POWER;
      }
          
    }
    break; // SOIL_WAIT_INTERVAL

    case SOIL_WAIT_POWER: // ждём истечения времени инициализации по питанию
    {
        if(sbnd.LinkType != linkUnbinded && sbnd.PowerPin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(sbnd.PowerPin))
        {
          lastUpdateCall += dt;
          if(lastUpdateCall < sbnd.PowerOnDelay)
          {
            return;
          }
          else
          {
            readFromSensors();
            lastUpdateCall = 0;
            machineState = SOIL_WAIT_INTERVAL;
          }
        } // if(sbnd.LinkType != linkUnbinded && sbnd.PowerPin != UNBINDED_PIN
        else
        {
          readFromSensors();
          lastUpdateCall = 0;
          machineState = SOIL_WAIT_INTERVAL;          
        }
      
    }
    break; // SOIL_WAIT_POWER
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::readFromSensors()
{
  SoilBinding sbnd = HardwareBinding->GetSoilBinding();
  uint8_t sensorCntr = 0;
    
      for(size_t i=0;i<sizeof(sbnd.SensorType)/sizeof(sbnd.SensorType[0]);i++)
      {
        uint8_t pin = sbnd.SensorPin[i];
        uint8_t type = sbnd.SensorType[i];

        if(pin == UNBINDED_PIN || pin < 1 || !EEPROMSettingsModule::SafePin(pin)) // нет привязки датчика
        {
          continue;
        }
        
        switch(type)
        {
          case ANALOG_SOIL_MOISTURE: // аналоговый датчик влажности почвы
          {
              int val = analogRead(pin);
              int rawADC = val;
      
              // теперь нам надо отразить показания между SOIL_MOISTURE_100_PERCENT и SOIL_MOISTURE_0_PERCENT
              int pMin = min(sbnd.Soil0Percent,sbnd.Soil100Percent);
              int pMax = max(sbnd.Soil0Percent,sbnd.Soil100Percent);
              
              val = constrain(val,pMin,pMax);

              int percentsInterval = map(val,pMin,pMax,0,10000);
      
              // теперь, если у нас значение 0% влажности больше, чем значение 100% влажности - надо от 10000 отнять полученное значение
              if(sbnd.Soil0Percent > sbnd.Soil100Percent)
              {
                percentsInterval = 10000 - percentsInterval;
              }
           
              Humidity h;
              h.Value = percentsInterval/100;
              h.Fract  = percentsInterval%100;
              if(h.Value > 99)
              {
                h.Value = 100;
                h.Fract = 0;
              }
      
              if(h.Value < 0)
              {
                h.Value = NO_TEMPERATURE_DATA;
                h.Fract = 0;
              }

              // проверяем на выход за интервал, отсекая минимальный шум
              if(sbnd.Soil0Percent > sbnd.Soil100Percent)
              {
                if(rawADC <= sbnd.NoDataBorder)
                {
                  h.Value = NO_TEMPERATURE_DATA;
                  h.Fract = 0;
                }
              }
              else
              {
                if(rawADC >= sbnd.NoDataBorder)
                {
                  h.Value = NO_TEMPERATURE_DATA;
                  h.Fract = 0;
                }                
              }
      
              
              // обновляем состояние  
              State.UpdateState(StateSoilMoisture,sensorCntr,(void*)&h);
              sensorCntr++;
          } 
          break; // ANALOG_SOIL_MOISTURE

          case FREQUENCY_SOIL_MOISTURE: // частотный датчик влажности почвы
          {
           // Serial.println("Update frequency sensors...");
            
            Humidity h;

            int highTime = pulseIn(pin,HIGH, PULSE_TIMEOUT);

            if(!highTime) // ALWAYS HIGH,  BUS ERROR
            {
             // Serial.println("BUS ERROR, NO HIGH TIME");
              State.UpdateState(StateSoilMoisture,sensorCntr,(void*)&h);
              sensorCntr++;
            }
            else
            {
              // normal
              highTime = pulseIn(pin,HIGH, PULSE_TIMEOUT);
              int lowTime = pulseIn(pin,LOW, PULSE_TIMEOUT);

              if(!lowTime || !highTime)
              {
               // Serial.println("BUS ERROR, NO LOW OR HIGH TIME");
                // BUS ERROR
                State.UpdateState(StateSoilMoisture,sensorCntr,(void*)&h);
                sensorCntr++;
              }
              else
              {
                // normal
                int totalTime = lowTime + highTime;
                float moisture = (highTime*100.0)/totalTime;
                int moistureInt = moisture*100;

                h.Value = moistureInt/100;
                h.Fract = moistureInt%100;

                State.UpdateState(StateSoilMoisture,sensorCntr,(void*)&h);
                sensorCntr++;

                //Serial.print("Moisture is: ");
               // Serial.println(h);

              }

            } // else           
          }
          break; // FREQUENCY_SOIL_MOISTURE

        } // switch
      } // for

      // выключаем датчики после чтения
      if(sbnd.LinkType != linkUnbinded && sbnd.PowerPin != UNBINDED_PIN)
        {
            if(sbnd.LinkType == linkDirect)
            {
                if(EEPROMSettingsModule::SafePin(sbnd.PowerPin))
                {
                  WORK_STATUS.PinWrite(sbnd.PowerPin,!sbnd.Level);            
                }
            }
            else
            if(sbnd.LinkType == linkMCP23S17)
            {
              #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0        
                    WORK_STATUS.MCP_SPI_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,!sbnd.Level);
              #endif
            }
            else
            if(sbnd.LinkType == linkMCP23017)
            {
              #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                    WORK_STATUS.MCP_I2C_PinWrite(sbnd.MCPAddress,sbnd.PowerPin,!sbnd.Level);
              #endif
            }
          
        } // if(sbnd.LinkType != linkUnbinded)  
      
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  SoilMoistureModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer)
  {
    PublishSingleton = NOT_SUPPORTED;
  }
    
  if(command.GetType() == ctSET) // установка свойств
  {
    uint8_t argsCnt = command.GetArgsCount();
    if(argsCnt > 0)
    {
      String param = command.GetArg(0);
      if(param == F("DATA")) // установить значение на датчике, CTSET=SOIL|DATA|idx|value
      {
        if (argsCnt > 2)
        {
          uint8_t sensorIndex = (uint8_t) atoi(command.GetArg(1));
          int16_t humidity =  (int16_t) atoi(command.GetArg(2));
          
          uint8_t _tCnt = State.GetStateCount(StateSoilMoisture);
          if(sensorIndex >= _tCnt)
          {
            uint8_t toAdd = (sensorIndex - _tCnt) + 1;
  
              for(uint8_t qa = 0; qa < toAdd; qa++)
              {
                State.AddState(StateSoilMoisture,_tCnt + qa);
              }
           } 

          Temperature t;
          t.Value = humidity/100;    
          t.Fract = abs(humidity%100);
          State.UpdateState(StateSoilMoisture,sensorIndex,(void*)&t);

                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
                  #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
                  OneState* dynReset = State.GetState(StateSoilMoisture,sensorIndex);
                  if(dynReset)
                  {
                    DynamicSensorsReset->Observe(dynReset);
                  }
                  #endif
                  ///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
        
                PublishSingleton.Flags.Status = true;
                PublishSingleton = param;
                PublishSingleton << PARAM_DELIMITER << sensorIndex << PARAM_DELIMITER << REG_SUCC;                
                    
        } // if (argsCnt > 2)
        
      } // if(param == F("DATA"))
      
    } // if(argsCnt > 0)
    
  } // ctSET    
  else
  if(command.GetType() == ctGET) // запрос свойств
  {
      uint8_t argsCnt = command.GetArgsCount();
      if(argsCnt < 1)
      {
        if(wantAnswer)
        {
          PublishSingleton = PARAMS_MISSED; // не хватает параметров
        }
        
      } // argsCnt < 1 
      else
      {     
        String param = command.GetArg(0);
        
        if(param == ALL) // запросили показания со всех датчиков: CTGET=SOIL|ALL
        {
          PublishSingleton.Flags.Status = true;
          uint8_t _cnt = State.GetStateCount(StateSoilMoisture);
          if(wantAnswer) 
          {
            PublishSingleton = _cnt;
          }
          
          for(uint8_t i=0;i<_cnt;i++)
          {

             OneState* stateHumidity = State.GetStateByOrder(StateSoilMoisture,i);
             if(stateHumidity)
             {
                HumidityPair hp = *stateHumidity;
              
                if(wantAnswer) 
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current);
                }
             } // if
          } // for        
        } // param == ALL
        else
        if(param == PROP_CNT) // запросили данные о кол-ве датчиков: CTGET=SOIL|CNT
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = PROP_CNT; 
            uint8_t _cnt = State.GetStateCount(StateSoilMoisture);
            PublishSingleton << PARAM_DELIMITER << _cnt;
          }
        } // PROP_CNT
        else
        if(param != GetID()) // если только не запросили без параметров
        {
 // запросили показания с датчика по индексу
          uint8_t idx = param.toInt();
          uint8_t _cnt = State.GetStateCount(StateSoilMoisture);
          
          if(idx >= _cnt)
          {
            // плохой индекс
            if(wantAnswer)
            {
              PublishSingleton = NOT_SUPPORTED;
            }
          } // плохой индекс
          else
          {
             if(wantAnswer)
             {
              PublishSingleton = param;
             }
              
             OneState* stateHumidity = State.GetStateByOrder(StateSoilMoisture,idx);
             if(stateHumidity)
             {
                PublishSingleton.Flags.Status = true;
                HumidityPair hp = *stateHumidity;
                
                if(wantAnswer)
                {
                  PublishSingleton << PARAM_DELIMITER << (hp.Current);
                }
             } // if
            
          } // else нормальный индекс        
        } // if param != GetID()
        
      } // else
  }
  
  MainController->Publish(this,command); 
  
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SOIL_MOISTURE_MODULE

