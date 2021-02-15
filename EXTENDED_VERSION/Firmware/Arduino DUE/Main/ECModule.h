#pragma once

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  phWaitInterval,
  phInMeasure
  
} PHMeasureState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ecWaitInterval,
  ecInMeasure
  
} ECMeasureState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ms_Normal, // нормальный режим работы
  ms_CheckEC, // контроль EC
  ms_CheckECRetries, // повторы прочитать с датчика EC
  ms_AddA, // добавление раствора А
  ms_AddACheck, // проверка, что надо выключать подачу А
  ms_AddB, // добавление раствора B
  ms_AddBCheck, // проверка, что надо выключать подачу B
  ms_AddC, // добавление раствора С
  ms_AddCCheck, // проверка, что надо выключать подачу C
  ms_AddWater, // добавление воды (уменьшение EC)
  ms_AddWaterCheck, // проверка, что надо выключать подачу воды
  ms_CheckPH, // контроль pH
  ms_PhPlus, // увеличение pH
  ms_PhPlusCheck, // проверка, что надо выключать подачу pH+
  ms_PhMinus, // уменьшение pH
  ms_PhMinusCheck, // проверка, что надо выключать подачу pH-
  ms_Mix, // перемешивание
  ms_MixCheck, // проверка, что надо выключать перемешивание
  ms_Work, // подача готового раствора
  ms_WorkCheck, // проверка, что надо выключать подачу раствора
  ms_Mix2, // перемешиваем после добавления pH
  ms_Mix2Check, // проверка, что надо закончить перемешивание pH
  
} ECMachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
class ECModule : public AbstractModule // модуль контроля EC
{
  private:

    /////////////////////////////////////////////////////////
    // контроль pH
    /////////////////////////////////////////////////////////
    uint32_t phTimer;
    uint8_t phSamplesDone;
    uint32_t phDataArray;
    PHMeasureState phMeasureState;
    
    void ReadPHSensor(); // читаем значение с датчика pH
    
    bool phSensorAdded;
    uint8_t phSensorPin; // номер пина датчика pH
    int phCalibration; // калибровка, в сотых долях
    int16_t ph4Voltage; // показания в милливольтах для тестового раствора 4 pH
    int16_t ph7Voltage; // показания в милливольтах для тестового раствора 7 pH
    int16_t ph10Voltage; // показания в милливольтах для тестового раствора 10 pH
    int8_t phTemperatureSensorIndex; // индекс датчика температуры, который завязан на измерения pH
    Temperature phSamplesTemperature; // температура, при которой производилась калибровка

    uint16_t phTarget; // значение pH, за которым следим
    uint16_t phHisteresis; // гистерезис
    uint16_t phMixPumpTime; // время работы насоса перемешивания, с
    uint16_t phReagentPumpTime; // время работы подачи реагента, с
    
    /////////////////////////////////////////////////////////
    // контроль pH
    /////////////////////////////////////////////////////////

    // чтение настроек
    void ReadSettings();

    // установка уровня на пине
    bool out(uint8_t linkType, uint8_t MCPAddress, uint8_t pin, uint8_t level, bool setMode=false);

    // чтение датчиков EC
    void ReadECSensors();

    uint32_t ecTimer;
    uint8_t ecSamplesDone;
    uint32_t ecDataArray[4];
    ECMeasureState ecMeasureState;


// контроль EC
    
    bool ecControlled; // флаг, что мы проконтролировали EC перед первой подачей
    uint32_t ecControlTimer; // таймер контроля EC
    uint32_t ecWorkTimer; // таймер подачи раствора в рабочую зону
    uint32_t ecRequestedTimerInterval; // интервал для таймера
    ECMachineState ecMachineState; // состояние конечного автомата
    uint8_t ecCheckRetries; // кол-во попыток прочитать с датчика при контроле ЕС
    uint32_t absDiffPPM; // абсолютная разница PPM с датчика и уставки
    
// контроль EC

  
  public:
    ECModule();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    void ApplyCalculation(Temperature* temp);

    // сохранение настроек
    void SaveSettings();

    /////////////////////////////////////////////////////////
    // контроль pH
    /////////////////////////////////////////////////////////

    // установка чтение уставки значения pH
    uint16_t GetPhTarget() { return phTarget; }
    void SetPhTarget(uint16_t val) {phTarget = val;}

    // установка/чтение значения гистерезиса pH
    uint16_t GetPhHisteresis() {return phHisteresis;}
    void SetPhHisteresis(uint16_t val) {phHisteresis = val;}

    // установка/чтение фактора калибровки
    int GetPhCalibration() { return phCalibration; }
    void SetPhCalibration(int val) { phCalibration = val; }

    // установка/чтение вольтажа при 4 pH
    int16_t GetPh4Voltage() { return ph4Voltage; }
    void SetPh4Voltage(int16_t val){ ph4Voltage = val; }

    // установка/чтение вольтажа при 7 pH
    int16_t GetPh7Voltage() { return ph7Voltage; }
    void SetPh7Voltage(int16_t val){ ph7Voltage = val; }

    // установка/чтение вольтажа при 10 pH
    int16_t GetPh10Voltage() { return ph10Voltage; }
    void SetPh10Voltage(int16_t val){ ph10Voltage = val; }

    // установка/чтение индекса датчика темературы для контроля pH
    int8_t GetPhTemperatureSensorIndex(){ return phTemperatureSensorIndex; }
    void SetPhTemperatureSensorIndex(int8_t val){ phTemperatureSensorIndex = val; }

    // установка/чтение вольтажа температуры, при которой калибровались растворы
    int GetPhSamplesTemperature(){ return phSamplesTemperature.Value; }
    void SetPhSamplesTemperature(int val)
    {
      phSamplesTemperature.Value = val;
      phSamplesTemperature.Fract = 0;
    }

    uint16_t GetPhMixPumpTime() { return phMixPumpTime; }
    void SetPhMixPumpTime(uint16_t val) { phMixPumpTime = val; }

    uint16_t GetPhReagentPumpTime() { return phReagentPumpTime; }
    void SetPhReagentPumpTime(uint16_t val) { phReagentPumpTime = val; }
    
    /////////////////////////////////////////////////////////
    // контроль pH
    /////////////////////////////////////////////////////////
    

};
//--------------------------------------------------------------------------------------------------------------------------------------
class EC_PHCalculator
{
  public:

    void ApplyCalculation(Temperature* temp);
  
    EC_PHCalculator();
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern ECModule* ECControl;
extern EC_PHCalculator EC_PH_Calculation;

