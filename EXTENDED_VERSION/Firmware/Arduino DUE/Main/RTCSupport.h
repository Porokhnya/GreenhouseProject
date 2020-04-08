#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_INTERNAL_CLOCK

  #pragma message "Use internal clock..."
  // Includes Atmel CMSIS
  #include <chip.h>
  
  #define SUPC_KEY     0xA5u
  #define RESET_VALUE  0x01210720
  
  #define RC           0
  #define  XTAL         1

#else
//#pragma message "Use DS3231 clock..."
#include <Wire.h>
#endif
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
struct RTCTime // данные по текущему времени
{
  uint8_t second; // секунда (0-59)
  uint8_t minute; // минута (0-59)
  uint8_t hour; // час (0-23)
  uint8_t dayOfWeek; // день недели (1 - понедельник и т.д.)
  uint8_t dayOfMonth; // день месяца (1-31)
  uint8_t month; // месяц(1-12)
  uint16_t year; // формат - ХХХХ

  uint32_t unixtime() const;  
  RTCTime maketime(uint32_t unixtime);


}; 
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_INTERNAL_CLOCK
//--------------------------------------------------------------------------------------------------------------------------------------
class RealtimeClock
{
  public:
    RealtimeClock();
    void setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year);
    void setTime(const RTCTime& time);


    const char* getDayOfWeekStr(const RTCTime& t);
    const char* getTimeStr(const RTCTime& t);
    const char* getDateStr(const RTCTime& t);

    RTCTime getTime();

    Temperature getTemperature();
 
    void begin(uint8_t src);
        
  private:
    static char workBuff[12]; // буфер под дату/время
    
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#else
//--------------------------------------------------------------------------------------------------------------------------------------
enum { DS3231Address = 0x68/*, DS3231Address_W = 0xD0, DS3231Address_R = 0xD1*/}; // адрес датчика
//--------------------------------------------------------------------------------------------------------------------------------------
/*
#if TARGET_BOARD == DUE_BOARD

    #define TWI_SPEED    TWI_SPEED_400k  // Set default TWI Speed
    #define TWI_SPEED_100k  208
    #define TWI_SPEED_400k  101
    
    #define TWI_DIV     TWI_DIV_400k  // Set divider for TWI Speed (must match TWI_SPEED setting)
    #define TWI_DIV_100k  1
    #define TWI_DIV_400k  0

    #define REG_SEC    0x00
    #define REG_MIN   0x01
    #define REG_HOUR  0x02
    #define REG_DOW   0x03
    #define REG_DATE  0x04
    #define REG_MON   0x05
    #define REG_YEAR  0x06
    #define REG_CON   0x0e
    #define REG_STATUS  0x0f
    #define REG_AGING 0x10
    #define REG_TEMPM 0x11
    #define REG_TEMPL 0x12
    #define SEC_1970_TO_2000 946684800
      
#endif
*/
//--------------------------------------------------------------------------------------------------------------------------------------
class RealtimeClock
{


public:

    RealtimeClock();

    void setTime(uint8_t second, uint8_t minute, uint8_t hour, uint8_t dayOfWeek, uint8_t dayOfMonth, uint8_t month, uint16_t year);
    void setTime(const RTCTime& time);

    const char* getDayOfWeekStr(const RTCTime& t);
    const char* getTimeStr(const RTCTime& t);
    const char* getDateStr(const RTCTime& t);

    RTCTime getTime();

    Temperature getTemperature();
 
    void begin(uint8_t wireNumber);

private:
/*
#if TARGET_BOARD == DUE_BOARD

  void setDOW(uint8_t dow);

  uint8_t _scl_pin;
  uint8_t _sda_pin;
  uint8_t _burstArray[7];
  boolean _use_hw;

  void  _sendStart(byte addr);
  void  _sendStop();
  void  _sendAck();
  void  _sendNack();
  void  _waitForAck();
  uint8_t _readByte();
  void  _writeByte(uint8_t value);
  void  _burstRead();
  uint8_t _readRegister(uint8_t reg);
  void  _writeRegister(uint8_t reg, uint8_t value);
  uint8_t _decode(uint8_t value);
  uint8_t _decodeH(uint8_t value);
  uint8_t _decodeY(uint8_t value);
  uint8_t _encode(uint8_t vaule);

  Twi   *twi;
      
#else
*/
    uint8_t dec2bcd(uint8_t val);
    uint8_t bcd2dec(uint8_t val);

    TwoWire* wireInterface;
/*  
 #endif
*/
  static char workBuff[12]; // буфер под дату/время

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif

