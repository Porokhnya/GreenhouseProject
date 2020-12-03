#include "WeatherStation.h"
#include "LogicManageModule.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
WeatherStationClass WeatherStation;
//--------------------------------------------------------------------------------------------------------------------------------------
#define MAX_DELTA 140
uint8_t count = 0;
volatile uint8_t res[225];
volatile uint8_t flag = 0;
uint8_t head_flag = 0;
volatile unsigned long prevtime;
volatile unsigned int lolen, hilen, state;
uint8_t lastRainPulse = 0xFF;
//--------------------------------------------------------------------------------------------------------------------------------------
int calc_CRC()
{
  uint8_t tmp_byte = 0;
  uint16_t tmp_result = 0;
  for (uint8_t j = 0; j < 12; j++) 
  {
    tmp_byte = 0;
    for (uint8_t i = 0; i < 8; i++) 
    {
      tmp_byte <<= 1; //смещаем влево
      tmp_byte |= res[8 * j + i] & 1; //прибавляем младший бит
    }
    tmp_result = tmp_result + tmp_byte;
  }
  return (tmp_result & 0xFF);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CheckValue(unsigned int base, unsigned int value) 
{
  return ((value == base) || ((value > base) && ((value - base) < MAX_DELTA)) || ((value < base) && ((base - value) < MAX_DELTA)));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WeatherStationClass::read_input()
{
  state = digitalRead(WeatherStation.pin);
  
  if (state == HIGH)
  {
    lolen = micros() - prevtime;
  }
  else
  {
    hilen = micros() - prevtime;
  }
  prevtime = micros();


  if (state == LOW) 
  {
    // по спаду начинаем анализ
    if (CheckValue(240, hilen) && CheckValue(1520, lolen) && head_flag != 2) 
    { //нашли короткий импульс
      head_flag = 1;
    }
    else if (head_flag != 2) 
    {
      head_flag = 0; //если нет - все заново
    }
    
    if (CheckValue(2000, hilen) && CheckValue(1520, lolen) && head_flag != 2) 
    { //нашли длинный импульс
      head_flag = 2;
    }
    else if (head_flag != 2) 
    {
      head_flag = 0; //если нет - все заново
    }
    
   // голова посылки найдена начинаем прием полезных 224 бит данных
    if (CheckValue(960, hilen) && head_flag == 2) //бит 1 - 960 мс
    {
      res[count] = 1;
      count++;
    }
    else if (CheckValue(480, hilen) && head_flag == 2) // бит 0 - 480 мс
    {
      res[count] = 0;
      count++;
    }
    if (count == 223) 
    { 
      // отсчитали нужное число бит, все сбрасываем и запрещаем прерывания
      noInterrupts();
      head_flag = 0;
      flag = 1; //уходим в парсинг пакета
      count = 0;
      lolen = 0;
      hilen = 0;
    }
  } // if (state == LOW) 
}
//--------------------------------------------------------------------------------------------------------------------------------------
WeatherStationClass::WeatherStationClass()
{
  pin = -1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WeatherStationClass::setup(int16_t _pin)
{

  if(_pin > -1 && _pin != UNBINDED_PIN && EEPROMSettingsModule::SafePin(_pin))
  {
      pin = _pin;
      pinMode(pin, INPUT_PULLUP);
      attachInterrupt(pin, read_input, CHANGE);  // DUE    
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void WeatherStationClass::update()
{ 
  if(pin < 0 || pin == UNBINDED_PIN) // нет привязки к пину
  {
    return;
  }


  if (flag == 1) //если были данные
  {


    uint16_t stationID = 0;  // Получить ID станции
    for (uint8_t i = 0; i < 8; i++)
    {
      stationID <<= 1;
      stationID |= res[i] & 1;
    }
/*    
    Serial.print("ID: ");
    Serial.println(tmp,HEX);
*/

    uint8_t crc = 0;  // checksum
    for (uint8_t i = 96; i < 104; i++)
    {
      crc <<= 1;
      crc |= res[i] & 1;
    }
    /*
       Serial.print("calc_CRC: ");
       Serial.println(calc_CRC());
       Serial.print("received CRC: ");
       Serial.println(crc);
    */       

    if (stationID == 0xF5 && crc == calc_CRC())
    { //первый байт должен быть 0xF5 (ID станции, может варьироваться от модели и верный CRC
      //ID станции возможно необходимо настраивать под конкретную?

      /*
      //Температура
      float temp = 0;
      int16_t temp_tmp = 0;
      int16_t temp_tmp1 = 0;
      for (uint8_t i = 36; i < 48; i++)    //биты температуры
      {
        temp_tmp <<= 1;                  //смещаем влево
        temp_tmp |= res[i] & 1;          //прибавляем младший бит
        temp_tmp1 <<= 1;
        temp_tmp1 |= res[i + 112] & 1;   //прибавляем младший бит
      }
      
      //сверяем значение из двух посылок, если не ок, то выставляем в 255
      if (temp_tmp == temp_tmp1) 
      {
        temp = (float)(temp_tmp) / 10.0;
      }
      else 
      {
        temp = 255;
      }

      //Влажность
      uint8_t hum = 0;
      uint8_t hum1 = 0;
      for (uint8_t i = 48; i < 56; i++) //биты влажности наружного воздуха
      {
        hum <<= 1;
        hum |= res[i] & 1;
        hum1 <<= 1;
        hum1 |= res[i + 112] & 1;
      }
      if (hum == hum1) 
      {
        hum = hum1;
      }
      else 
      {
        hum = 255;
      }
*/

      //скорость ветра
      uint16_t ch_wind = 0;
      uint16_t ch_wind1 = 0;
      float wind = 0;
      bool windValid = true;
      for (uint8_t i = 56; i < 72; i++) //скорость ветра
      {
        ch_wind <<= 1;
        ch_wind |= res[i] & 1;
        ch_wind1 <<= 1;
        ch_wind1 |= res[i + 112] & 1;
      }
      if (ch_wind == ch_wind1) 
      {
        wind = (float)ch_wind / 588;
      }
      else 
      {
        windValid = false;
      }

      if(windValid) // если данные по скорости ветра валидны
      {
        // метеостанция передаёт нам скорость ветра в метрах в секунду
        // а у нас скорость ветра хранится в сотых долях м/с
        // поэтому - конвертируем значение в сотые доли
        uint32_t windSpeed = wind*100;

        // просим модуль логики сохранить данные по скорости ветра
        LogicManageModule->SetWindSpeed(windSpeed);
        
      } // if(windValid)

      //направление ветра
      uint8_t wind_dir = 0;
      uint8_t wind_dir1 = 0;
      for (uint8_t i = 32; i < 36; i++)
      {
        wind_dir <<= 1;
        wind_dir |= res[i] & 1;
        wind_dir1 <<= 1;
        wind_dir1 |= res[i + 112] & 1;
      }
      
      if (wind_dir1 == wind_dir) 
      {
        wind_dir = wind_dir1;
      }
      else 
      {
        wind_dir = 0xFF;
      }

      if(wind_dir != 0xFF) // если данные по направлению ветра валидны
      {
         CompassPoints windDirection = cpUnknown;
         switch (wind_dir) 
        {
          case 0: /*Serial.print("N")*/ windDirection = cpNorth; break;
          case 2: /*Serial.print("NE")*/ windDirection = cpNorth; break;
          case 4: /*Serial.print("E")*/ windDirection = cpEast; break;
          case 6: /*Serial.print("SE")*/ windDirection = cpEast; break;
          case 8: /*Serial.print("S")*/ windDirection = cpSouth; break;
          case 10: /*Serial.print("SW")*/ windDirection = cpSouth; break;
          case 12: /*Serial.print("W")*/ windDirection = cpWest; break;
          case 14: /*Serial.print("NW")*/ windDirection = cpWest; break;
          default: windDirection = cpUnknown; break;
        } // switch

        // просим модуль логики сохранить данные по направлению ветра
        LogicManageModule->SetWindDirection(windDirection);
                
      } // if(wind_dir != 0xFF)


      //Дождь. Передает состояние счетчика импульсов коромысла. Для определения наличия дождя нужно сравнивать предыдущие данные и текущие.

      uint8_t rain = 0;
      uint8_t rain1 = 0;
      for (uint8_t i = 88; i < 96; i++)         //биты счетчика импульсов дождя
      {
        rain <<= 1;
        rain |= res[i] & 1;
        rain1 <<= 1;
        rain1 |= res[i + 112] & 1;
      }
      
      if (rain == rain1)                     //сверяем значение из двух посылок, если не ок, то выставляем в 255
      {
        rain = rain1;
      }
      else 
      {
        rain = 0xFF;
      }

      if(rain != 0xFF) // если данные по дождю валидны
      {
        bool hasRain = false;
        if(lastRainPulse == 0xFF) // ещё не сохраняли последнее значение с метеостанции
        {
          lastRainPulse = rain; // сохраняем его
        }

        if(rain != lastRainPulse) // если значения не равны - идёт дождь
        {
          lastRainPulse = rain;
          hasRain = true;
        }
        // просим модуль логики сохранить флаг дождя
        LogicManageModule->SetHasRain(hasRain);
      } // if(rain != 0xFF)


      /*
      Serial.print("\n");

      Serial.print("Temperature: ");
      Serial.print((temp), 1);
      Serial.println("C");
      Serial.print("Humidity: ");
      Serial.print(hum);
      Serial.println("%");
      Serial.print("Wind: ");
      Serial.print(wind);
      Serial.println("m/s");
      Serial.print("Direction: ");
      switch (wind_dir) 
      {
      case 0: Serial.print("N"); break;
      case 2: Serial.print("NE"); break;
      case 4: Serial.print("E"); break;
      case 6: Serial.print("SE"); break;
      case 8: Serial.print("S"); break;
      case 10: Serial.print("SW"); break;
      case 12: Serial.print("W"); break;
      case 14: Serial.print("NW"); break;
      default: Serial.print("Error"); break;
      }
      
      Serial.print("\n"); 

      Serial.print("Rain pulse: ");  // счетчик дождя
      Serial.println(rain);

      Serial.print("\n\n");
      
     */
        
    } // if (stationID == 0xF5 && crc == calc_CRC())
    
    res[0] = 0;
    flag = 0;
    count = 0;
    interrupts();
    
  }  // if (flag == 1) //если были данные

}
//--------------------------------------------------------------------------------------------------------------------------------------

