#pragma once

#include "AbstractModule.h"
#include "BMP180.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_PRESSURE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#define NO_PRESSURE_DATA -2147483647
//--------------------------------------------------------------------------------------------------------------------------------------
struct PressureData
{
	int32_t Value;
	uint8_t Fract;
	
	PressureData()
	{
		Value = NO_PRESSURE_DATA;
		Fract = 0;
	}
	
	bool HasData()
	{
		return (Value != NO_PRESSURE_DATA);
	}
	
	operator String() const
	{
		if(Value == NO_PRESSURE_DATA)
		{
			return F("-");
		}
		
		String result;
		result = Value;
		result += '.';
		
		if(Fract < 10)
			result += '0';
		
		result += Fract;
		
		return result;
	}
	
	bool operator==(const PressureData& rhs)
	{
		return (Value == rhs.Value && Fract == rhs.Fract);
	}
	
};
//--------------------------------------------------------------------------------------------------------------------------------------
class PressureModule : public AbstractModule
{
  private:
  
	PressureData pressureData;
	SFE_BMP180 sensor;
	bool hasSensor;
	bool measureTimerEnabled;
	uint8_t measureTimerInterval;
	uint8_t measureIterator;
	uint32_t measureTimerTime;
	void readPressure();
	double pressureTemperature;
	
	uint32_t updateTimer;

  Vector<int32_t> predictList;
  Vector<uint8_t> predictTime;
  
  uint32_t predictTimer;
  int8_t predict;
  
  public:
    PressureModule();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();
	
	PressureData GetPressure() { return pressureData; }

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern PressureModule* Pressure;
#endif
