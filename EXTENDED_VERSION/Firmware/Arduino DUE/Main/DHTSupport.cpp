#include "DHTSupport.h"
#include "AbstractModule.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
DHTSupport::DHTSupport()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
HumidityAnswer DHTSupport::read(uint8_t pin, DHTType sensorType)
{
  HumidityAnswer answer;
  answer.IsOK = false;

  uint32_t wakeup_delay = DHT2x_WAKEUP;
  
  if(sensorType == DHT_11)
  {
    wakeup_delay = DHT11_WAKEUP;
  }

  const uint32_t mstcc = ( F_CPU / 40000 ); // сторож таймаута - 100us
  HumidityBinding hbnd = HardwareBinding->GetHumidityBinding();

  // начинаем читать с датчика
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW); // прижимаем к земле
  delayMicroseconds(wakeup_delay); // и ждём, пока датчик прочухается
  digitalWrite(pin,HIGH); // поднимаем линию
  delayMicroseconds(hbnd.DHTReadDelay); // ждём 40us, как написано в даташите
  WORK_STATUS.PinMode(pin, INPUT_PULLUP); // переводим пин на чтение

  // тут должны проверить последовательность, которую выдал датчик:
  // если линия прижата на 80us, затем поднята на 80us - значит,
  // датчик готов выдавать данные
  
  uint32_t tmout_guard = mstcc;
  
  /*while ((*PIR & bit) == LOW )/*/
  while(digitalRead(pin) == LOW) // читаем, пока низкий уровень на пине
  {
    if(!--tmout_guard)
     return answer; // таймаут поймали
  }
  tmout_guard = mstcc;
  /*while ((*PIR & bit) != LOW )/*/
  while(digitalRead(pin) == HIGH) // читаем, пока высокий уровень на пине
  {
    if(!--tmout_guard)
     return answer; // таймаут поймали
  }

  // считаем, что теперь пойдут данные. нам надо получить 40 бит, т.е. 5 байт.
  uint8_t bytes[5] = {0}; // байты, в которые мы будем принимать данные
  /*
  for(uint8_t i=0;i<5;i++)
    bytes[i] = 0;
  */
  memset(bytes,0,5);
  
  uint8_t idx = 0; // индекс текущего байта
  uint8_t bitmask = 0x80; // старший бит байта установлен в единичку, его будем двигать вниз
  
  for(uint8_t i=0;i<40;i++)
  {
      // сначала ждём 50us, говорящие, что пойдёт следующий бит
      tmout_guard = mstcc;
      /*while ((*PIR & bit) == LOW )/*/
      while(digitalRead(pin) == LOW)
      {
        if(!--tmout_guard)
            return answer; // таймаут поймали
      } // while

      // теперь принимаем бит. Если время подтянутой вверх линии более 40us - это единица, иначе - ноль.

      tmout_guard = mstcc;
      uint32_t tMicros = micros();
      /*while ((*PIR & bit) != LOW )/*/
      while(digitalRead(pin) == HIGH)
      {
        if(!--tmout_guard)
            return answer; // таймаут поймали
      } // while

      if(micros() - tMicros > 40) // единичка
      {
        bytes[idx] |= bitmask;
      }

      // сдвигаем маску вправо
      bitmask >>= 1;
      
      if(!bitmask) // дошли до конца байта
      {
        bitmask = 0x80;
        idx++; // читаем в следующий байт
      }
        
  } // for

 // pinMode(pin, OUTPUT);
//  digitalWrite(pin, HIGH); // поднимаем линию, говоря датчику, что он свободен


  // проверяем принятые данные
  switch(sensorType)
  {
    case DHT_11:
    {
      uint8_t crc = bytes[0] + bytes[2];
      if(crc != bytes[4]) // чексумма не сошлась
        return answer;

     // сохраняем данные
      answer.Humidity = bytes[0];
      answer.Temperature = bytes[2];
    }
    break;

    case DHT_2x:
    {
      uint8_t crc = bytes[0] + bytes[1] + bytes[2] + bytes[3];
      if(crc != bytes[4]) // чексумма не сошлась
        return answer;

     // сохраняем данные
      unsigned long rh = ((bytes[0] << 8) + bytes[1])*10;
      answer.Humidity = rh/100;
      answer.HumidityDecimal = rh%100;

     long temp = (((bytes[2] & 0x7F) << 8) + bytes[3])*10;
      
      answer.Temperature =  temp/100;
      answer.TemperatureDecimal = abs(temp%100);
      
      if(bytes[2] & 0x80) // температура ниже нуля
        answer.Temperature = -answer.Temperature;
     
    }
    break;
  } // switch
 
  if(answer.Humidity < 0 || answer.Humidity > 100)
  {
    answer.Humidity = NO_TEMPERATURE_DATA;
    answer.HumidityDecimal = 0;
  }

  if(answer.Temperature < -40 || answer.Temperature > 80)
  {
    answer.Temperature = NO_TEMPERATURE_DATA;
    answer.TemperatureDecimal = 0;
  }
  
  answer.IsOK = true;
  return answer;
}
//--------------------------------------------------------------------------------------------------------------------------------------


