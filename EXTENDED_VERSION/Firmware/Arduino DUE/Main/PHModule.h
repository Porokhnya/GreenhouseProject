#pragma once
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
//-------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PH_MODULE

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
    void out(uint8_t pin,uint8_t level);
    uint8_t in(uint8_t pin);

    uint8_t phSensorPin;
    unsigned long measureTimer;
    uint8_t samplesDone;
    uint8_t samplesTimer;

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

    bool isLevelSensorTriggered(byte data);
    
    uint16_t updateDelta;
    unsigned long mixPumpTimer;
    unsigned long phControlTimer;
    unsigned long reagentsTimer;
    uint16_t targetReagentsTimer;
    uint8_t targetReagentsChannel;
  
  public:
    PhModule() : AbstractModule("PH") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    void ApplyCalculation(Temperature* temp);
    void SaveSettings();

    uint16_t GetPHValue() { return phTarget; }
    void SetPHValue(uint16_t val) {phTarget = val;}

    uint16_t GetHisteresis() {return phHisteresis;}
    void SetHisteresis(uint16_t val) {phHisteresis = val;}

    uint16_t GetMixPumpTime() { return phMixPumpTime; }
    void SetMixPumpTime(uint16_t val) { phMixPumpTime = val; }

    uint16_t GetReagentPumpTime() { return phReagentPumpTime; }
    void SetReagentPumpTime(uint16_t val) { phReagentPumpTime = val; }

    int GetCalibration() { return calibration; }
    void SetCalibration(int val) { calibration = val; }

    int16_t GetPh4Voltage() { return ph4Voltage; }
    void SetPh4Voltage(int16_t val){ ph4Voltage = val; }

    int16_t GetPh7Voltage() { return ph7Voltage; }
    void SetPh7Voltage(int16_t val){ ph7Voltage = val; }

    int16_t GetPh10Voltage() { return ph10Voltage; }
    void SetPh10Voltage(int16_t val){ ph10Voltage = val; }

    int8_t GetTemperatureSensorIndex(){ return phTemperatureSensorIndex; }
    void SetTemperatureSensorIndex(int8_t val){ phTemperatureSensorIndex = val; }

    int GetTemperature(){ return phSamplesTemperature.Value; }
    void SetTemperature(int val)
    {
      phSamplesTemperature.Value = val;
      phSamplesTemperature.Fract = 0;
    }
    
    

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
extern PhModule* PHModule;
#endif // USE_PH_MODULE
