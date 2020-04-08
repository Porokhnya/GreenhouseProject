 #include "RTCSupport.h"
 #include <Arduino.h>
 #include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
// RTCTime
//--------------------------------------------------------------------------------------------------------------------------------------
#include <time.h>
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t RTCTime::unixtime() const
{
  tm t;
  t.tm_hour = hour;
  t.tm_isdst = 0;
  t.tm_mday = dayOfMonth;
  t.tm_min = minute;
  t.tm_mon = month - 1;
  t.tm_sec = second;
  t.tm_wday = dayOfWeek - 1;
  t.tm_year = year - 1900;

  return mktime(&t);
}
//--------------------------------------------------------------------------------------------------------------------------------------
RTCTime RTCTime::maketime(uint32_t time)
{
  RTCTime result;

  time_t tt = time;
  tm* t = localtime(&tt);
  
  result.hour = t->tm_hour;
  //t.tm_isdst = 0;
  result.dayOfMonth = t->tm_mday;
  result.minute = t->tm_min;
  result.month = t->tm_mon + 1;
  result.second = t->tm_sec;
  result.dayOfWeek = t->tm_wday;

  // 0 - воскресенье, 1 - понедельник, ... 6 - суббота
  //Serial.println(result.dayOfWeek);
  
  if(!result.dayOfWeek) // 0 - воскресенье, остальные - соответствуют своим номерам
    result.dayOfWeek = 7;
  //else
 //   result.dayOfWeek++;
    
  if(result.dayOfWeek > 7)
    result.dayOfWeek = result.dayOfWeek - 7;
    
  result.year = t->tm_year + 1900;  

  return result;
}    
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_INTERNAL_CLOCK
//--------------------------------------------------------------------------------------------------------------------------------------
char RealtimeClock::workBuff[12] = {0};
//--------------------------------------------------------------------------------------------------------------------------------------
RealtimeClock::RealtimeClock()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::begin(uint8_t src)
{
  if (src) 
  {
    pmc_switch_sclk_to_32kxtal(0);
  
  while (!pmc_osc_is_ready_32kxtal());
  }

  RTC_SetHourMode(RTC, 0);
  
  NVIC_DisableIRQ(RTC_IRQn);
  NVIC_ClearPendingIRQ(RTC_IRQn);
  NVIC_SetPriority(RTC_IRQn, 0);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
RTCTime RealtimeClock::getTime()
{
  RTCTime result;
  
  RTC_GetTime(RTC, (uint8_t*)&(result.hour), (uint8_t*)&(result.minute), (uint8_t*)&(result.second));
  RTC_GetDate(RTC, (uint16_t*)&(result.year), (uint8_t*)&(result.month), (uint8_t*)&(result.dayOfMonth), (uint8_t*)&(result.dayOfWeek));

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::setTime(const RTCTime& time)
{
  setTime(time.second, time.minute, time.hour, time.dayOfWeek, time.dayOfMonth, time.month,time.year);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year)
{
  RTC_SetTime (RTC, hour, minute, second);  
  RTC_SetDate (RTC, year, month, dayOfMonth, dayOfWeek);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
Temperature RealtimeClock::getTemperature()
{
 Temperature res;
 
 return res;
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getDayOfWeekStr(const RTCTime& t)
{
  static const char* dow[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
  return dow[t.dayOfWeek-1];
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getTimeStr(const RTCTime& t)
{
  char* writePtr = workBuff;
  
  if(t.hour < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.hour/10) + '0';

  *writePtr++ = (t.hour % 10) + '0';

 *writePtr++ = ':';

 if(t.minute < 10)
  *writePtr++ = '0';
 else
  *writePtr++ = (t.minute/10) + '0';

 *writePtr++ = (t.minute % 10) + '0';

 *writePtr++ = ':';

 if(t.second < 10)
  *writePtr++ = '0';
 else
  *writePtr++ = (t.second/10) + '0';

 *writePtr++ = (t.second % 10) + '0';

 *writePtr = 0;

 return workBuff;
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getDateStr(const RTCTime& t)
{
  char* writePtr = workBuff;
  if(t.dayOfMonth < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.dayOfMonth/10) + '0';
  *writePtr++ = (t.dayOfMonth % 10) + '0';

  *writePtr++ = '.';

  if(t.month < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.month/10) + '0';
  *writePtr++ = (t.month % 10) + '0';

  *writePtr++ = '.';

  *writePtr++ = (t.year/1000) + '0';
  *writePtr++ = (t.year % 1000)/100 + '0';
  *writePtr++ = (t.year % 100)/10 + '0';
  *writePtr++ = (t.year % 10) + '0';  

  *writePtr = 0;

  return workBuff;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#else
//--------------------------------------------------------------------------------------------------------------------------------------
char RealtimeClock::workBuff[12] = {0};
//--------------------------------------------------------------------------------------------------------------------------------------
RealtimeClock::RealtimeClock()
{
/*
#if TARGET_BOARD == DUE_BOARD
#else
*/  
    wireInterface = &Wire;
/*    
#endif   
*/
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*
#if TARGET_BOARD == DUE_BOARD
//--------------------------------------------------------------------------------------------------------------------------------------
// DUE board
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::setDOW(uint8_t dow)
{
  if ((dow > 0) && (dow < 8))
    _writeRegister(REG_DOW, dow);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_burstRead()
{
  if (_use_hw)
  {
    // Set slave address and number of internal address bytes.
    twi->TWI_MMR = (1 << 8) | TWI_MMR_MREAD | (DS3231Address << 16);
    // Set internal address bytes
    twi->TWI_IADR = 0;
    // Send START condition
    twi->TWI_CR = TWI_CR_START;

    for (int i=0; i<6; i++)
    {
      while ((twi->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY)
      {
      };
      _burstArray[i] = twi->TWI_RHR;
    }

    twi->TWI_CR = TWI_CR_STOP;
    while ((twi->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY) {};
    _burstArray[6] = twi->TWI_RHR;
    while ((twi->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {};
  }
  else
  {
    _sendStart(DS3231Address_W);
    _waitForAck();
    _writeByte(0);
    _waitForAck();
    _sendStart(DS3231Address_R);
    _waitForAck();

    for (int i=0; i<7; i++)
    {
      _burstArray[i] = _readByte();
      if (i<6)
        _sendAck();
      else
        _sendNack();
    }
    _sendStop();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::_readRegister(uint8_t reg)
{
  uint8_t readValue=0;

  if (_use_hw)
  {
    // Set slave address and number of internal address bytes.
    twi->TWI_MMR = (1 << 8) | TWI_MMR_MREAD | (DS3231Address << 16);
    // Set internal address bytes
    twi->TWI_IADR = reg;
    // Send START and STOP condition to read a single byte
    twi->TWI_CR = TWI_CR_START | TWI_CR_STOP;
    while ((twi->TWI_SR & TWI_SR_RXRDY) != TWI_SR_RXRDY) {};
    readValue = twi->TWI_RHR;
    while ((twi->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {};
  }
  else
  {
    _sendStart(DS3231Address_W);
    _waitForAck();
    _writeByte(reg);
    _waitForAck();
    _sendStart(DS3231Address_R);
    _waitForAck();
    readValue = _readByte();
    _sendNack();
    _sendStop();
  }
  return readValue;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_writeRegister(uint8_t reg, uint8_t value)
{
  if (_use_hw)
  {
    // Set slave address and number of internal address bytes.
    twi->TWI_MMR = (1 << 8) | (DS3231Address << 16);
    // Set internal address bytes
    twi->TWI_IADR = reg;
    // Send a single byte to start transfer
    twi->TWI_THR = value;
    while ((twi->TWI_SR & TWI_SR_TXRDY) != TWI_SR_TXRDY) {};
    // Send STOP condition
    twi->TWI_CR = TWI_CR_STOP;
    while ((twi->TWI_SR & TWI_SR_TXCOMP) != TWI_SR_TXCOMP) {};
  }
  else
  {
    _sendStart(DS3231Address_W);
    _waitForAck();
    _writeByte(reg);
    _waitForAck();
    _writeByte(value);
    _waitForAck();
    _sendStop();
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_sendStart(byte addr)
{
  pinMode(_sda_pin, OUTPUT);
  digitalWrite(_sda_pin, HIGH);
  digitalWrite(_scl_pin, HIGH);
  digitalWrite(_sda_pin, LOW);
  digitalWrite(_scl_pin, LOW);
  shiftOut(_sda_pin, _scl_pin, MSBFIRST, addr);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_sendStop()
{
  pinMode(_sda_pin, OUTPUT);
  digitalWrite(_sda_pin, LOW);
  digitalWrite(_scl_pin, HIGH);
  digitalWrite(_sda_pin, HIGH);
  pinMode(_sda_pin, INPUT);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_sendNack()
{
  pinMode(_sda_pin, OUTPUT);
  digitalWrite(_scl_pin, LOW);
  digitalWrite(_sda_pin, HIGH);
  digitalWrite(_scl_pin, HIGH);
  digitalWrite(_scl_pin, LOW);
  pinMode(_sda_pin, INPUT);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_sendAck()
{
  pinMode(_sda_pin, OUTPUT);
  digitalWrite(_scl_pin, LOW);
  digitalWrite(_sda_pin, LOW);
  digitalWrite(_scl_pin, HIGH);
  digitalWrite(_scl_pin, LOW);
  pinMode(_sda_pin, INPUT);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_waitForAck()
{
  pinMode(_sda_pin, INPUT);
  digitalWrite(_scl_pin, HIGH);
  while (digitalRead(_sda_pin) == HIGH) {}
  digitalWrite(_scl_pin, LOW);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::_readByte()
{
  pinMode(_sda_pin, INPUT);

  uint8_t value = 0;
  uint8_t currentBit = 0;

  for (int i = 0; i < 8; ++i)
  {
    digitalWrite(_scl_pin, HIGH);
    currentBit = digitalRead(_sda_pin);
    value |= (currentBit << (7 - i));
    delayMicroseconds(1);
    digitalWrite(_scl_pin, LOW);
  }
  return value;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::_writeByte(uint8_t value)
{
  pinMode(_sda_pin, OUTPUT);
  shiftOut(_sda_pin, _scl_pin, MSBFIRST, value);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t  RealtimeClock::_decode(uint8_t value)
{
  uint8_t decoded = value & 127;
  decoded = (decoded & 15) + 10 * ((decoded & (15 << 4)) >> 4);
  return decoded;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::_decodeH(uint8_t value)
{
  if (value & 128)
    value = (value & 15) + (12 * ((value & 32) >> 5));
  else
    value = (value & 15) + (10 * ((value & 48) >> 4));
  return value;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t  RealtimeClock::_decodeY(uint8_t value)
{
  uint8_t decoded = (value & 15) + 10 * ((value & (15 << 4)) >> 4);
  return decoded;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::_encode(uint8_t value)
{
  uint8_t encoded = ((value / 10) << 4) + (value % 10);
  return encoded;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#else // NOT DUE BOARD
*/
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::dec2bcd(uint8_t val)
{
  return( (val/10*16) + (val%10) );
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t RealtimeClock::bcd2dec(uint8_t val)
{
  return( (val/16*10) + (val%16) );
}
//--------------------------------------------------------------------------------------------------------------------------------------
/*
#endif // NOT DUE BOARD
*/
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::setTime(const RTCTime& time)
{
  setTime(time.second, time.minute, time.hour, time.dayOfWeek, time.dayOfMonth, time.month,time.year);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year)
{
/*
#if TARGET_BOARD == DUE_BOARD

  if ( (hour < 24) && (minute < 60) && (second < 60) && ((dayOfMonth > 0) && (dayOfMonth <= 31)) && ((month > 0) && (month <= 12)) && ((year >= 2000) && (year < 3000)))
  {
    year -= 2000;
    _writeRegister(REG_HOUR, _encode(hour));
    _writeRegister(REG_MIN, _encode(minute));
    _writeRegister(REG_SEC, _encode(second));
    _writeRegister(REG_YEAR, _encode(year));
    _writeRegister(REG_MON, _encode(month));
    _writeRegister(REG_DATE, _encode(dayOfMonth));
    setDOW(dayOfWeek);
  }


#else
*/
  while(year > 100) // приводим к диапазону 0-99
    year -= 100;
 
  wireInterface->beginTransmission(DS3231Address);
  
  wireInterface->write(0); // указываем, что начинаем писать с регистра секунд
  wireInterface->write(dec2bcd(second)); // пишем секунды
  wireInterface->write(dec2bcd(minute)); // пишем минуты
  wireInterface->write(dec2bcd(hour)); // пишем часы
  wireInterface->write(dec2bcd(dayOfWeek)); // пишем день недели
  wireInterface->write(dec2bcd(dayOfMonth)); // пишем дату
  wireInterface->write(dec2bcd(month)); // пишем месяц
  wireInterface->write(dec2bcd(year)); // пишем год
  
  wireInterface->endTransmission(true);  

  delay(10); // немного подождём для надёжности
/*
#endif
*/
}
//--------------------------------------------------------------------------------------------------------------------------------------
Temperature RealtimeClock::getTemperature()
{
/*
#if TARGET_BOARD == DUE_BOARD

    Temperature res;
    
     union int16_byte {
          int i;
          byte b[2];
      } rtcTemp;
        
    
       rtcTemp.b[1] = _readRegister(REG_TEMPM);
       rtcTemp.b[0] = _readRegister(REG_TEMPL);
    
        long tempC100 = (rtcTemp.i >> 6) * 25;
    
        res.Value = tempC100/100;
        res.Fract = abs(tempC100 % 100);
    
    
      return res;

#else
*/  
     Temperature res;
      
     union int16_byte {
           int i;
           byte b[2];
       } rtcTemp;
         
      wireInterface->beginTransmission(DS3231Address);
      wireInterface->write(0x11);
      if(wireInterface->endTransmission(true) != 0) // ошибка
        return res;
    
      if(wireInterface->requestFrom(DS3231Address, 2) == 2)
      {
        rtcTemp.b[1] = wireInterface->read();
        rtcTemp.b[0] = wireInterface->read();
    
        long tempC100 = (rtcTemp.i >> 6) * 25;
    
        res.Value = tempC100/100;
        res.Fract = abs(tempC100 % 100);
        
      }
      
      return res;
  /*
  #endif
  */
}
//--------------------------------------------------------------------------------------------------------------------------------------
RTCTime RealtimeClock::getTime()
{
  static RTCTime t;
  static uint32_t lastRequestTime = 0;
  
  if(!lastRequestTime)
    memset(&t,0,sizeof(t));
  /*
  #if TARGET_BOARD == DUE_BOARD

      if(!lastRequestTime || ( (millis() - lastRequestTime) > 999) )
      {
        _burstRead();
        t.second = _decode(_burstArray[0]);
        t.minute = _decode(_burstArray[1]);
        t.hour = _decodeH(_burstArray[2]);
        t.dayOfWeek = _burstArray[3];
        t.dayOfMonth = _decode(_burstArray[4]);
        t.month = _decode(_burstArray[5]);
        t.year = _decodeY(_burstArray[6]) + 2000;
        
        lastRequestTime = millis();
      }
      
      return t;
  
  #else
  */
    // NOT Due board

     if(!lastRequestTime || ( (millis() - lastRequestTime) > 999) )
     {
        wireInterface->beginTransmission(DS3231Address);
        wireInterface->write(0); // говорим, что мы собираемся читать с регистра 0
        
        if(wireInterface->endTransmission(true) != 0) // ошибка
          return t;
        
        if(wireInterface->requestFrom(DS3231Address, 7) == 7) // читаем 7 байт, начиная с регистра 0
        {
            t.second = bcd2dec(wireInterface->read() & 0x7F);
            if(t.second > 59)
              t.second = 59;
            
            t.minute = bcd2dec(wireInterface->read());
            if(t.minute > 59)
              t.minute = 59;
            
            t.hour = bcd2dec(wireInterface->read() & 0x3F);
            if(t.hour > 23)
              t.hour = 23;
            
            t.dayOfWeek = bcd2dec(wireInterface->read());
            t.dayOfWeek = constrain(t.dayOfWeek,1,7);
            
            t.dayOfMonth = bcd2dec(wireInterface->read());
            t.dayOfMonth = constrain(t.dayOfMonth,1,31);
            
            t.month = bcd2dec(wireInterface->read());
            t.month = constrain(t.month,1,12);
            
            t.year = bcd2dec(wireInterface->read());     
            t.year += 2000; // приводим время к нормальному формату
        } // if

        lastRequestTime = millis();
     }
      
      return t;
  /*
  #endif
  */
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getDayOfWeekStr(const RTCTime& t)
{
  uint8_t idx = t.dayOfWeek;
  if(idx > 0)
    --idx;

  if(idx > 6)
    idx = 6;
    
  static const char* dow[] = {"Mon", "Tue", "Wed", "Thu", "Fri", "Sat", "Sun"};
  return dow[idx];
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getTimeStr(const RTCTime& t)
{
  char* writePtr = workBuff;
  
  if(t.hour < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.hour/10) + '0';

  *writePtr++ = (t.hour % 10) + '0';

 *writePtr++ = ':';

 if(t.minute < 10)
  *writePtr++ = '0';
 else
  *writePtr++ = (t.minute/10) + '0';

 *writePtr++ = (t.minute % 10) + '0';

 *writePtr++ = ':';

 if(t.second < 10)
  *writePtr++ = '0';
 else
  *writePtr++ = (t.second/10) + '0';

 *writePtr++ = (t.second % 10) + '0';

 *writePtr = 0;

 return workBuff;
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* RealtimeClock::getDateStr(const RTCTime& t)
{
  char* writePtr = workBuff;
  if(t.dayOfMonth < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.dayOfMonth/10) + '0';
  *writePtr++ = (t.dayOfMonth % 10) + '0';

  *writePtr++ = '.';

  if(t.month < 10)
    *writePtr++ = '0';
  else
    *writePtr++ = (t.month/10) + '0';
  *writePtr++ = (t.month % 10) + '0';

  *writePtr++ = '.';

  *writePtr++ = (t.year/1000) + '0';
  *writePtr++ = (t.year % 1000)/100 + '0';
  *writePtr++ = (t.year % 100)/10 + '0';
  *writePtr++ = (t.year % 10) + '0';  

  *writePtr = 0;

  return workBuff;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void RealtimeClock::begin(uint8_t wireNumber)
{
#if TARGET_BOARD == DUE_BOARD

  if(wireNumber == 1)
    wireInterface = &Wire1;
  else
    wireInterface = &Wire;

/*
  if (wireNumber == 1)
  {
    _sda_pin = SDA1;// , SCLdata_pin;
    _scl_pin = SCL1;// sclk_pin;
  }
  else
  {
    _sda_pin = SDA;// , SCLdata_pin;
    _scl_pin = SCL;// sclk_pin;
  }

  _use_hw = false;
  if ((_sda_pin == SDA) and (_scl_pin == SCL))
  {
    _use_hw = true;
    twi = TWI1;
    pmc_enable_periph_clk(WIRE_INTERFACE_ID);
    PIO_Configure(g_APinDescription[PIN_WIRE_SDA].pPort, g_APinDescription[PIN_WIRE_SDA].ulPinType, g_APinDescription[PIN_WIRE_SDA].ulPin, g_APinDescription[PIN_WIRE_SDA].ulPinConfiguration);
    PIO_Configure(g_APinDescription[PIN_WIRE_SCL].pPort, g_APinDescription[PIN_WIRE_SCL].ulPinType, g_APinDescription[PIN_WIRE_SCL].ulPin, g_APinDescription[PIN_WIRE_SCL].ulPinConfiguration);
    NVIC_DisableIRQ(TWI1_IRQn);
    NVIC_ClearPendingIRQ(TWI1_IRQn);
    NVIC_SetPriority(TWI1_IRQn, 0);
    NVIC_EnableIRQ(TWI1_IRQn);

  }
  else if ((_sda_pin == SDA1) and (_scl_pin == SCL1))
  {
    _use_hw = true;
    twi = TWI0;
    pmc_enable_periph_clk(WIRE1_INTERFACE_ID);
    PIO_Configure(g_APinDescription[PIN_WIRE1_SDA].pPort, g_APinDescription[PIN_WIRE1_SDA].ulPinType, g_APinDescription[PIN_WIRE1_SDA].ulPin, g_APinDescription[PIN_WIRE1_SDA].ulPinConfiguration);
    PIO_Configure(g_APinDescription[PIN_WIRE1_SCL].pPort, g_APinDescription[PIN_WIRE1_SCL].ulPinType, g_APinDescription[PIN_WIRE1_SCL].ulPin, g_APinDescription[PIN_WIRE1_SCL].ulPinConfiguration);
    NVIC_DisableIRQ(TWI0_IRQn);
    NVIC_ClearPendingIRQ(TWI0_IRQn);
    NVIC_SetPriority(TWI0_IRQn, 0);
    NVIC_EnableIRQ(TWI0_IRQn);
  }

  if (_use_hw)
  {
    // activate internal pullups for twi.
    digitalWrite(_sda_pin, 1);
    digitalWrite(_scl_pin, 1);

    // Reset the TWI
    twi->TWI_CR = TWI_CR_SWRST;
    // TWI Slave Mode Disabled, TWI Master Mode Disabled.
    twi->TWI_CR = TWI_CR_SVDIS;
    twi->TWI_CR = TWI_CR_MSDIS;
    // Set TWI Speed
    twi->TWI_CWGR = (TWI_DIV << 16) | (TWI_SPEED << 8) | TWI_SPEED;
    // Set master mode
    twi->TWI_CR = TWI_CR_MSEN;
  }
  else
  {
    pinMode(_scl_pin, OUTPUT);
  }
*/  
#elif TARGET_BOARD == MEGA_BOARD

        wireInterface = &Wire;
        
#elif TARGET_BOARD == STM32_BOARD

        if(wireNumber == 1)
          wireInterface = &Wire1;
        else
          wireInterface = &Wire;
#else
  #error "Unknown target board!!!"      
#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
