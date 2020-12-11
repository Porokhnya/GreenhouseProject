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
//   lineManager.beginSetResolution();

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
      
//      lineManager.setResolution(pin,temp12bit);
  
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

      // читаем тип датчика
      ModuleStates sensorType;
      OneWireAddress sensorAddress;
      if(lineManager.getType(sensorCounter,pin,sensorType,sensorAddress))
      {
        if(bnd.Index[i] != 0xFF) // если индекс датчику назначен
        {
            // получили тип датчика на линии, теперь можем читать его скратчпад
            size_t rawDataLength = lineManager.getRawDataLength(sensorType);
            uint8_t* rawData = new uint8_t[rawDataLength];
    
            // теперь читаем сырые данные с датчика
            if(lineManager.getRawData(pin,sensorAddress,sensorType,rawData))
            {
               // прочитали сырые данные с датчика, можно их конвертировать в нужный тип данных
               Temperature temperatureData;
               Humidity    humidityData;
               long        luminosityData = NO_LUMINOSITY_DATA;
               
               
               switch(sensorType) // какой тип датчика у нас на линии?
               {
                  case StateTemperature:
                  case StateSoilMoisture:
                  {
                    // температура и влажность почвы - передаются как показания с DS18B20
                    DS18B20Temperature tempData = lineManager.asTemperature(rawData,rawDataLength,sensorAddress);
                    temperatureData.Value = tempData.Whole;
        
                    if(tempData.Negative)
                      temperatureData.Value = -temperatureData.Value;
              
                    temperatureData.Fract = tempData.Fract;
                  }
                  break;

                  case StateHumidity:
                  {
                    // температура - в TemperatureValue, влажность - в HumidityValue
                    HumidityCompositeData tData = lineManager.asHumidity(rawData,rawDataLength,sensorAddress);
                    temperatureData.Value = tData.TemperatureValue.Whole;
        
                    if(tData.TemperatureValue.Negative)
                      temperatureData.Value = -temperatureData.Value;
              
                    temperatureData.Fract = tData.TemperatureValue.Fract;  

                    humidityData = tData.HumidityValue;
                  }
                  break;

                  case StateLuminosity:
                  {
                      luminosityData = lineManager.asLuminosity(rawData,rawDataLength,sensorAddress);
                  }
                  break;

                  //TODO: Тут другие типы данных !!!
                
               } // switch

               // теперь пытаемся сконвертировать полученные данные в указанный пользователем тип
               String cmd;
               
                  switch(bnd.Type[i])
                  {
                    case 1: // влажность
                    {
                      int32_t rawTemperatureVal = abs(temperatureData.Value)*100;
                      rawTemperatureVal += temperatureData.Fract;
        
                      if(temperatureData.Value < 0)
                      {
                        rawTemperatureVal = -rawTemperatureVal;
                      }

                      int32_t rawHumidityVal = abs(humidityData.Value)*100;
                      rawHumidityVal += humidityData.Fract;
        
                      if(humidityData.Value < 0)
                      {
                        rawHumidityVal = -rawHumidityVal;
                      }
        
                      cmd = "HUMIDITY";
                      cmd += PARAM_DELIMITER;
                      cmd += "DATA";
                      cmd += PARAM_DELIMITER;
                      cmd += bnd.Index[i]; 
                      cmd += PARAM_DELIMITER;
                      cmd += rawTemperatureVal;
                      cmd += PARAM_DELIMITER;
                      cmd += rawHumidityVal;
                    }
                    break;
            
                    case 2: // влажность почвы
                    {
                      int32_t rawVal = abs(temperatureData.Value)*100;
                      rawVal += temperatureData.Fract;
        
                      if(temperatureData.Value < 0)
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
                      cmd = "LIGHT";
                      cmd += PARAM_DELIMITER;
                      cmd += "DATA";
                      cmd += PARAM_DELIMITER;
                      cmd += bnd.Index[i]; 
                      cmd += PARAM_DELIMITER;
                      cmd += luminosityData;
                    }
                    break; 
                    
                    case 0: // нет привязки
                    default:
                    break;
                } // switch               

                // ну и в оконцове - посылаем команду в нужный модуль
                if(cmd.length() > 0)
                {
                  ModuleInterop.QueryCommand(ctSET,cmd,false);
                }
            } // if(lineManager.getRawData(pin,sensorAddress,sensorType,rawData))
    
            delete[] rawData;

        } // if(bnd.Index[i] != 0xFF) // если индекс датчику назначен
        
      } // if(lineManager.getType(sensorCounter,pin,sensorType,sensorAddress))
      
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


