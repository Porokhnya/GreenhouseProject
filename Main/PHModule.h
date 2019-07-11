#ifndef _PH_MODULE_H
#define _PH_MODULE_H
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PH_MODULE

class PCF8574
{
  public:
  PCF8574(int address); 

  void begin();

  uint8_t read8(); 
  uint8_t read(uint8_t pin); 
  uint8_t value();  

  void write8(uint8_t value); 
  void write(uint8_t pin, uint8_t value); 

  void toggle(uint8_t pin);
  void shiftRight(uint8_t n=1);
  void shiftLeft(uint8_t n=1);

  int lastError();

  private:
  int _address;
  uint8_t _data;
  int8_t _error;
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
    bool inMeasure : 1;
    bool isMixPumpOn : 1;
    bool isInAddReagentsMode : 1;
    byte pad : 5;
  
} PHModuleFlags;
#pragma pack(pop)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class PhModule : public AbstractModule // модуль контроля pH
{
  private:

    PHModuleFlags flags;

    byte phSensorPin;
    unsigned long measureTimer;
    byte samplesDone;
    byte samplesTimer;

    int calibration; // калибровка, в сотых долях
    int16_t ph4Voltage; // показания в милливольтах для тестового раствора 4 pH
    int16_t ph7Voltage; // показания в милливольтах для тестового раствора 7 pH
    int16_t ph10Voltage; // показания в милливольтах для тестового раствора 10 pH
    int8_t phTemperatureSensorIndex; // индекс датчика температуры, который завязан на измерения pH
    Temperature phSamplesTemperature; // температура, при которой производилась калибровка

    uint16_t phTarget; // значение pH, за которым следим
    uint16_t phHisteresis; // гистерезис
    uint16_t phMixPumpTime; // время работы насоса перемешивания, с
    uint16_t phReagentPumpTime; // время работы подачи реагента, с

    unsigned long dataArray;

    void ReadSettings();
    void SaveSettings();

    bool isLevelSensorTriggered(byte data);
    
    uint16_t updateDelta;
    unsigned long mixPumpTimer;
    unsigned long phControlTimer;
    unsigned long reagentsTimer;
    uint16_t targetReagentsTimer;
    byte targetReagentsChannel;
  
  public:
    PhModule() : AbstractModule("PH") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

    void ApplyCalculation(Temperature* temp);

};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
class PHCalculator
{
  public:

    void ApplyCalculation(Temperature* temp);
  
    PHCalculator();
};
//-------------------------------------------------------------------------------------------------------------------------------------------------------
extern PHCalculator PHCalculation;
#endif // USE_PH_MODULE

#endif
