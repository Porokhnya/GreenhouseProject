#include "SoilMoistureModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SOIL_MOISTURE_MODULE

#define PULSE_TIMEOUT 50000 // 50 миллисекунд на чтение фронта максимум
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  int8_t pin;
  int8_t type;
  
} SoilMoistureSensorSettings;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
#if SUPPORTED_SOIL_MOISTURE_SENSORS > 0
static SoilMoistureSensorSettings SOIL_MOISTURE_SENSORS_ARRAY[] = { SOIL_MOISTURE_SENSORS };
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::Setup()
{
  // настройка модуля тут

  machineState = SOIL_WAIT_INTERVAL;
  
  #if SUPPORTED_SOIL_MOISTURE_SENSORS > 0

      #ifdef USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
      
          #if SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_DIRECT
            WORK_STATUS.PinMode(SOIL_MOISTURE_POWER_PIN,OUTPUT);
            WORK_STATUS.PinWrite(SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
          #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23S17
            #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
              WORK_STATUS.MCP_SPI_PinMode(SOIL_MOISTURE_MCP23S17_ADDRESS,SOIL_MOISTURE_POWER_PIN,OUTPUT);
              WORK_STATUS.MCP_SPI_PinWrite(SOIL_MOISTURE_MCP23S17_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
            #endif
          #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23017
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              WORK_STATUS.MCP_I2C_PinMode(SOIL_MOISTURE_MCP23017_ADDRESS,SOIL_MOISTURE_POWER_PIN,OUTPUT);
              WORK_STATUS.MCP_I2C_PinWrite(SOIL_MOISTURE_MCP23017_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
            #endif
          #endif
          
      #endif // USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
  
    for(uint8_t i=0;i<SUPPORTED_SOIL_MOISTURE_SENSORS;i++)
    {
      WORK_STATUS.PinMode(SOIL_MOISTURE_SENSORS_ARRAY[i].pin, INPUT, false);
      if(SOIL_MOISTURE_SENSORS_ARRAY[i].type == FREQUENCY_SOIL_MOISTURE)
      {
        WORK_STATUS.PinMode(SOIL_MOISTURE_SENSORS_ARRAY[i].pin,INPUT);
        WORK_STATUS.PinWrite(SOIL_MOISTURE_SENSORS_ARRAY[i].pin,HIGH);
      }
      State.AddState(StateSoilMoisture,i); // добавляем датчики влажности почвы
    } // for
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::Update(uint16_t dt)
{ 

  // обновление модуля тут
  switch(machineState)
  {
    case SOIL_WAIT_INTERVAL: // ждём истечения интервала между опросами датчиков
    {
      lastUpdateCall += dt;
      if(lastUpdateCall < SOIL_MOISTURE_UPDATE_INTERVAL) // обновляем согласно настроенному интервалу
        return;
      else
      {
        // включаем датчики
        #if SUPPORTED_SOIL_MOISTURE_SENSORS > 0
          #ifdef USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
          
              #if SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_DIRECT
                WORK_STATUS.PinWrite(SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_ON);
              #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23S17
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                  WORK_STATUS.MCP_SPI_PinWrite(SOIL_MOISTURE_MCP23S17_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_ON);
                #endif
              #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23017
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                  WORK_STATUS.MCP_I2C_PinWrite(SOIL_MOISTURE_MCP23017_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_ON);
                #endif
              #endif
              
          #endif // USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
         #endif // SUPPORTED_SOIL_MOISTURE_SENSORS > 0
              
        lastUpdateCall = 0;
        machineState = SOIL_WAIT_POWER;
      }
          
    }
    break; // SOIL_WAIT_INTERVAL

    case SOIL_WAIT_POWER: // ждём истечения времени инициализации по питанию
    {
      #ifdef USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
          lastUpdateCall += dt;
          if(lastUpdateCall < SOIL_MOISTURE_POWER_ON_DELAY)
            return;
          else
          {
            readFromSensors();
            lastUpdateCall = 0;
            machineState = SOIL_WAIT_INTERVAL;
          }
      #else
        readFromSensors();
        lastUpdateCall = 0;
        machineState = SOIL_WAIT_INTERVAL;
      #endif
    }
    break; // SOIL_WAIT_POWER
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
void SoilMoistureModule::readFromSensors()
{
    #if SUPPORTED_SOIL_MOISTURE_SENSORS > 0
    
      for(uint8_t i=0;i<SUPPORTED_SOIL_MOISTURE_SENSORS;i++)
      {
        switch(SOIL_MOISTURE_SENSORS_ARRAY[i].type)
        {
          case ANALOG_SOIL_MOISTURE: // аналоговый датчик влажности почвы
          {
              int val = analogRead(SOIL_MOISTURE_SENSORS_ARRAY[i].pin);
      
              // теперь нам надо отразить показания между SOIL_MOISTURE_100_PERCENT и SOIL_MOISTURE_0_PERCENT
              int pMin = min(SOIL_MOISTURE_0_PERCENT,SOIL_MOISTURE_100_PERCENT);
              int pMax = max(SOIL_MOISTURE_0_PERCENT,SOIL_MOISTURE_100_PERCENT);
              
              val = constrain(val,pMin,pMax);
      
              int percentsInterval = map(val,min(SOIL_MOISTURE_0_PERCENT,SOIL_MOISTURE_100_PERCENT),max(SOIL_MOISTURE_0_PERCENT,SOIL_MOISTURE_100_PERCENT),0,10000);
      
              // теперь, если у нас значение 0% влажности больше, чем значение 100% влажности - надо от 10000 отнять полученное значение
              if(SOIL_MOISTURE_0_PERCENT > SOIL_MOISTURE_100_PERCENT)
                percentsInterval = 10000 - percentsInterval;
           
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
      
              
              // обновляем состояние  
              State.UpdateState(StateSoilMoisture,i,(void*)&h);
          } 
          break; // ANALOG_SOIL_MOISTURE

          case FREQUENCY_SOIL_MOISTURE: // частотный датчик влажности почвы
          {
           // Serial.println("Update frequency sensors...");
            
            int8_t pin = SOIL_MOISTURE_SENSORS_ARRAY[i].pin;
            Humidity h;

            int highTime = pulseIn(pin,HIGH, PULSE_TIMEOUT);

            if(!highTime) // ALWAYS HIGH,  BUS ERROR
            {
             // Serial.println("BUS ERROR, NO HIGH TIME");
              State.UpdateState(StateSoilMoisture,i,(void*)&h);
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
                State.UpdateState(StateSoilMoisture,i,(void*)&h);
              }
              else
              {
                // normal
                int totalTime = lowTime + highTime;
                float moisture = (highTime*100.0)/totalTime;
                int moistureInt = moisture*100;

                h.Value = moistureInt/100;
                h.Fract = moistureInt%100;

                State.UpdateState(StateSoilMoisture,i,(void*)&h);

                //Serial.print("Moisture is: ");
               // Serial.println(h);

              }

            } // else           
          }
          break; // FREQUENCY_SOIL_MOISTURE

        } // switch
      } // for

      // выключаем датчики после чтения
        #ifdef USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT
          
              #if SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_DIRECT
                WORK_STATUS.PinWrite(SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
              #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23S17
                #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                  WORK_STATUS.MCP_SPI_PinWrite(SOIL_MOISTURE_MCP23S17_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
                #endif
              #elif SOIL_MOISTURE_POWER_DRIVE_MODE == DRIVE_MCP23017
                #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                  WORK_STATUS.MCP_I2C_PinWrite(SOIL_MOISTURE_MCP23017_ADDRESS,SOIL_MOISTURE_POWER_PIN,SOIL_MOISTURE_POWER_OFF);
                #endif
              #endif
              
          #endif // USE_SOIL_MOISTURE_SENSORS_POWER_MANAGEMENT      
      
    #endif // SUPPORTED_SOIL_MOISTURE_SENSORS > 0
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  SoilMoistureModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) 
    PublishSingleton = NOT_SUPPORTED;
    
  if(command.GetType() == ctSET) // установка свойств
  {
    
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
        String param = command.GetArg(0);
        
        if(param == ALL) // запросили показания со всех датчиков: CTGET=SOIL|ALL
        {
          PublishSingleton.Flags.Status = true;
          uint8_t _cnt = State.GetStateCount(StateSoilMoisture);
          if(wantAnswer) 
            PublishSingleton = _cnt;
          
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
              PublishSingleton = NOT_SUPPORTED;
          } // плохой индекс
          else
          {
             if(wantAnswer) 
              PublishSingleton = param;
              
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
  
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SOIL_MOISTURE_MODULE

