#include "DHTSupport.h"


DHTSupport::DHTSupport(DHTType sensorType)
{
  type = sensorType;
}
void DHTSupport::read(uint8_t pin, HumidityAnswer& answer)
{
  answer.Humidity = NO_TEMPERATURE_DATA;
  answer.HumidityDecimal = 0;
  answer.Temperature = NO_TEMPERATURE_DATA;
  answer.TemperatureDecimal = 0;

  uint8_t wakeup_delay = DHT2x_WAKEUP;
  
  if(type == DHT_11)
    wakeup_delay = DHT11_WAKEUP;

  const uint32_t mstcc = ( F_CPU / 40000 ); // сторож таймаута - 100us

  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *PIR = portInputRegister(port);

  // начинаем читать с датчика
  pinMode(pin,OUTPUT);
  digitalWrite(pin,LOW); // прижимаем к земле
  delay(wakeup_delay); // и ждём, пока датчик прочухается
  digitalWrite(pin,HIGH); // поднимаем линию
  delayMicroseconds(40); // ждём 40us, как написано в даташите
  pinMode(pin, INPUT_PULLUP); // переводим пин на чтение

  // тут должны проверить последовательность, которую выдал датчик:
  // если линия прижата на 80us, затем поднята на 80us - значит,
  // датчик готов выдавать данные
  
  uint32_t tmout_guard = mstcc;
  
  while ((*PIR & bit) == LOW )//while(digitalRead(pin) == LOW) // читаем, пока низкий уровень на пине
  {
    if(!--tmout_guard)
     return; // таймаут поймали
  }
  tmout_guard = mstcc;
  while ((*PIR & bit) != LOW )//while(digitalRead(pin) == HIGH) // читаем, пока высокий уровень на пине
  {
    if(!--tmout_guard)
     return; // таймаут поймали
  }

  // считаем, что теперь пойдут данные. нам надо получить 40 бит, т.е. 5 байт.
  uint8_t bytes[5] = {0}; // байты, в которые мы будем принимать данные
  for(uint8_t i=0;i<5;i++)
    bytes[i] = 0;
  
  uint8_t idx = 0; // индекс текущего байта
  uint8_t bitmask = 0x80; // старший бит байта установлен в единичку, его будем двигать вниз
  
  for(uint8_t i=0;i<40;i++)
  {
      // сначала ждём 50us, говорящие, что пойдёт следующий бит
      tmout_guard = mstcc;
      while ((*PIR & bit) == LOW )//while(digitalRead(pin) == LOW)
      {
        if(!--tmout_guard)
            return; // таймаут поймали
      } // while

      // теперь принимаем бит. Если время подтянутой вверх линии более 40us - это единица, иначе - ноль.

      tmout_guard = mstcc;
      uint32_t tMicros = micros();
      while ((*PIR & bit) != LOW )//while(digitalRead(pin) == HIGH)
      {
        if(!--tmout_guard)
            return; // таймаут поймали
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

  pinMode(pin, OUTPUT);
  digitalWrite(pin, HIGH); // поднимаем линию, говоря датчику, что он свободен


  // проверяем принятые данные
  switch(type)
  {
    case DHT_11:
    {
      uint8_t crc = bytes[0] + bytes[2];
      if(crc != bytes[4]) // чексумма не сошлась
        return;

     // сохраняем данные
      answer.Humidity = bytes[0];
      answer.Temperature = bytes[2];
    }
    break;

    case DHT_2x:
    {
      uint8_t crc = bytes[0] + bytes[1] + bytes[2] + bytes[3];
      if(crc != bytes[4]) // чексумма не сошлась
        return;

     // сохраняем данные
      unsigned long rh = ((bytes[0] << 8) + bytes[1])*10;
      answer.Humidity = rh/100;
      answer.HumidityDecimal = rh%100;

     long temp = (((bytes[2] & 0x7F) << 8) + bytes[3])*10;
      
      answer.Temperature =  temp/100;
      answer.TemperatureDecimal = temp%100;
      
      if(bytes[2] & 0x80) // температура ниже нуля
        answer.Temperature = -answer.Temperature;
    }
    break;
  } // switch
 
}


