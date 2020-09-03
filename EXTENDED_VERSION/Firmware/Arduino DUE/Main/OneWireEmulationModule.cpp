#include "OneWireEmulationModule.h"
#include "ModuleController.h"
#include "EEPROMSettingsModule.h"
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
void OneWireEmulationModule::Setup()
{
  // настройка модуля тут
   lineManager.begin(DS18B20_EMULATION_STORE_ADDRESS);
   lineManager.beginConversion();
   lineManager.beginSetResolution();

   DS18B20EmulationBinding bnd = HardwareBinding->GetDS18B20EmulationBinding();

   uint8_t sensorCounter = 0;
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // непривязанный пин
      {
        continue;
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }

      // добавляем состояние
//      State.AddState(StateTemperature,sensorCounter);
      
       // добавляем привязку
      lineManager.addBinding(pin,sensorCounter);
  
      WORK_STATUS.PinMode(pin,INPUT,false);
      
      lineManager.setResolution(pin,temp12bit);
  
      // запускаем конвертацию с датчиков при старте, через 2 секунды нам вернётся измеренная температура
      lineManager.startConversion(pin);

      sensorCounter++;
   } // for  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void OneWireEmulationModule::Update()
{ 
  // обновление модуля тут
  static uint32_t updateTimer = 0;
  if(millis() - updateTimer >= 5000)
  {
    // опрашиваем наши датчики
    DS18B20EmulationBinding bnd = HardwareBinding->GetDS18B20EmulationBinding();

  // запускаем конвертацию датчиков, игнорируя повторные вызовы для одной линии
   lineManager.beginConversion();

   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      lineManager.startConversion(pin);
   } // for

   // теперь сканируем линии
   lineManager.beginScan();
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      lineManager.scan(pin);
   } // for

   // теперь - читаем датчики
   uint8_t sensorCounter = 0;
   
   for(size_t i=0;i<sizeof(bnd.Pin);i++)
   {
      uint8_t pin = bnd.Pin[i];
      
      if(pin == UNBINDED_PIN) // нет привязки к пину
      {
        continue;  
      }

      if(!EEPROMSettingsModule::SafePin(pin)) // небезопасный пин
      {
        continue;
      }
      
      DS18B20Temperature tempData;
      Temperature t;
        
        if(lineManager.getTemperature(sensorCounter,pin, tempData))
        {
          
          t.Value = tempData.Whole;
        
          if(tempData.Negative)
            t.Value = -t.Value;
    
          t.Fract = tempData.Fract;
          
        }      

      //State.UpdateState(StateTemperature,sensorCounter,(void*)&t); // обновляем состояние температуры, индексы датчиков у нас идут без дырок, поэтому с итератором цикла вызывать можно

      String cmd; // команда для скармливания

      if(bnd.Index[i] != 0xFF) // если индекс датчику назначен
      {
          // теперь вычисляем, что за тип датчика у нас, собственно
          switch(bnd.Type[i])
          {
            case 1: // влажность
            {
              int32_t rawVal = abs(t.Value)*100;
              rawVal += t.Fract;

              if(t.Value < 0)
              {
                rawVal = -rawVal;
              }

              cmd = "HUMIDITY";
              cmd += PARAM_DELIMITER;
              cmd += "DATA";
              cmd += PARAM_DELIMITER;
              cmd += bnd.Index[i]; 
              cmd += PARAM_DELIMITER;
              cmd += rawVal;
              cmd += PARAM_DELIMITER;
              cmd += rawVal;
            }
            break;
    
            case 2: // влажность почвы
            {
              int32_t rawVal = abs(t.Value)*100;
              rawVal += t.Fract;

              if(t.Value < 0)
              {
                rawVal = -rawVal;
              }

              cmd = "SOIL";
              cmd += PARAM_DELIMITER;
              cmd += "DATA";
              cmd += PARAM_DELIMITER;
              cmd += bnd.Index[i]; 
              cmd += PARAM_DELIMITER;
              cmd += rawVal;
            }
            break;
    
            case 3: // освещённость
            {
              long rawVal = ((uint32_t)t.Value) << 8;
              rawVal |= t.Fract;
              
              cmd = "LIGHT";
              cmd += PARAM_DELIMITER;
              cmd += "DATA";
              cmd += PARAM_DELIMITER;
              cmd += bnd.Index[i]; 
              cmd += PARAM_DELIMITER;
              cmd += rawVal;
            }
            break; 
            
            case 0: // нет привязки
            default:
            break;
            
          } // switch

          if(cmd.length() > 0)
          {
            ModuleInterop.QueryCommand(ctSET,cmd,false);
          }
          
      } // if(bnd.Index[i] != 0xFF)
      
      sensorCounter++;
   } // for
    

    updateTimer = millis();
  } // if

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  OneWireEmulationModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  UNUSED(command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------


