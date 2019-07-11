#ifndef _LUMINOSITY_MODULE_H
#define _LUMINOSITY_MODULE_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
#include <Wire.h>
#ifdef LIGHT_AVERAGING_ENABLED
  #include "TinyVector.h"
  typedef Vector<long> LightAverageList;
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
enum
{
  BH1750_SENSOR,
  MAX44009_SENSOR
  
}; // тип датчика освещённости
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  lightAutomatic,
  lightManual
} LightWorkMode; // режим управления досветкой
//--------------------------------------------------------------------------------------------------------------------------------------
// смотрим, какие методы Wire актуальны - чтоб не париться с препроцессором в рабочем коде
#if (ARDUINO >= 100)
  #define BH1750_WIRE_READ Wire.read
  #define BH1750_WIRE_WRITE Wire.write
#else
  #define BH1750_WIRE_READ Wire.receive
  #define BH1750_WIRE_WRITE Wire.send
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ContinuousHighResolution = 0x10,
  ContinuousHighResolution2 = 0x11,
  ContinuousLowResolution = 0x13
    
} BH1750Mode;

enum { BH1750PowerOff=0x00, BH1750PowerOn=0x01, BH1750Reset = 0x07 };

typedef enum { BH1750Address1 = 0x23, BH1750Address2 = 0x5C } BH1750Address; // адрес датчика освещенности на шине I2C
//--------------------------------------------------------------------------------------------------------------------------------------
class BH1750Support
{
  private:
    void writeByte(uint8_t toWrite);

    BH1750Address deviceAddress;
    BH1750Mode currentMode;
  
  public:
    BH1750Support();
    
    void begin(BH1750Address addr = BH1750Address1, BH1750Mode mode = ContinuousHighResolution);
    
    void ChangeMode(BH1750Mode newMode);
    void ChangeAddress(BH1750Address newAddr);
     
    long GetCurrentLuminosity();
};
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t workMode : 6;
  bool bRelaysIsOn : 1; // включены ли реле досветки?
  bool bLastRelaysIsOn : 1; // флаг последнего состояния досветки
  
} LuminosityModuleFlags;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
class LuminosityModule : public AbstractModule // модуль управления освещенностью
{
  private:

 // GlobalSettings* settings; // настройки
#ifdef USE_LIGHT_MANUAL_MODE_DIODE
  BlinkModeInterop blinker;
#endif

#ifdef LIGHT_AVERAGING_ENABLED
    LightAverageList averageLists[4];
    long GetAverageValue(long lum,uint8_t sensorIndex);
#endif

#ifdef LIGHT_HARBORING_ENABLED
    long ApplyHarboring(long lum);
#endif    

  void* lightSensors[4]; // массив датчиков

  uint16_t lastUpdateCall;
  LuminosityModuleFlags flags;
    
  public:
    LuminosityModule() : AbstractModule("LIGHT")
    , lastUpdateCall(
      #if LUMINOSITY_UPDATE_INTERVAL > 678
        LUMINOSITY_UPDATE_INTERVAL - 678
      #else
        0
      #endif
      ) // разнесём опросы датчиков по времени
    {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
#endif // USE_LUMINOSITY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
