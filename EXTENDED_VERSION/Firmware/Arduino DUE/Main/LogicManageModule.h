#pragma once

#include "AbstractModule.h"
#include "Settings.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  cvDisable,
  cvEnable
  
} CycleVentDriveCommand;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
	int16_t openTemp25percents;
	int16_t openTemp50percents;
	int16_t openTemp75percents;
	int16_t openTemp100percents;
	int16_t histeresis;
	int16_t sensorIndex;
	bool active;
	
} WManageChannel;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	cpUnknown,	// нет привязки
	cpEast,		// восток
	cpSouth,	// юг
	cpWest,		// запад
	cpNorth		// север
	
} CompassPoints; // стороны света
//--------------------------------------------------------------------------------------------------------------------------------------
struct CompassCompare
{
  uint16_t count;
  CompassPoints point;

  CompassCompare()
  {
    point = cpUnknown;
    count = 0;
  }
  
  CompassCompare(CompassPoints d, uint16_t c)
  {
    point = d;
    count = c;
  }

  bool operator < (const CompassCompare& rhs)
  {
    return count < rhs.count;
  }

  bool operator > (const CompassCompare& rhs)
  {
    return count > rhs.count;
  }

  bool operator == (const CompassCompare& rhs)
  {
    return count == rhs.count;
  }
    
};
//----------------------------------------------------------------------------------------------------------------
typedef enum
{
	wdUnknown,
	wdOpen0, // команда "закрыть"
	wdOpen25, // команда "открыть на 25%"
	wdOpen50, // команда "открыть на 50%"
	wdOpen75, // команда "открыть на 75%"
	wdOpen100 // команда "открыть на 100%"
	
} WindowDriveCommand;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	ldUnknown,
	ldLightOn,
	ldLightOff
	
} LightDriveCommand;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	hdUnknown,
	hdHeatOn,
	hdHeatOff
	
} HeatDriveCommand;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	sdUnknown,
	sdShadowOn,
	sdShadowOff
	
} ShadowDriveCommand;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	hvsUnknown,
	hvsOpening,
	hvsOpen,
	hvsClosing,
	hvsClosed
	
} ValveState;
typedef enum
{
	vtmOff,
	vtmOpen,
	vtmClose
	
} ValveTurnMode;

typedef enum
{
  wdNop,
  wdOpen,
  wdClose
  
} WayDirection;
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HEAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
class Heater
{
	public:
		Heater();

 
	void update();
	void setup(uint8_t channel,int16_t driveWorkTime);
	void setSettings(int16_t driveWorkTime);

  void open(uint8_t percents);
  void reset(); // закрывает заслонки с максимального положения


  bool isBusy() { return wayDirection != wdNop; }
  bool isOn() 
  { 
    //return currentPosition > 0; 
    switch(wayDirection)
    {
      case wdNop: // никуда не движемся, просто проверяем текущую позицию
        return currentPosition > 0;

      case wdOpen: // открываемся, в любом случае отопление включено
        return (currentPosition + timerCounter) > 0;

      case wdClose: // закрываемся, считаем, что мы выключены, если запрошена нулевая позиция
        return (currentPosition - timerCounter) > 0;
    }
  }

	
	private:
	
		uint32_t workTime;
    uint32_t onePercentMS; // кол-во миллисекунд на открытие/закрытие на 1%
    uint32_t currentPosition; // текущая позиция заслонки
    bool onMyWay; // флаг, что мы двигаемся
    WayDirection wayDirection; // направление движения (0 - не двигаемся, 1 - открываемся, 2 - закрываемся)
    uint32_t timer; // таймер для открытия/закрытия
    uint32_t timerCounter; // тут счётчик пройденных миллисекунд
    bool pumpIsOn;
    WayDirection lastWayDir;


    void turnValve(WayDirection dir);
    
		uint8_t channel;
	//	ValveState valveState;
//		uint32_t valveTimer;
//		bool timerActive;
//		uint32_t timerInterval;
		
		void turnPump(bool _on);
//		void turnValve(ValveTurnMode mode);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HEAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SHADOW_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
class Shadow
{
	public:
		Shadow();
		
	void on();
	void off();
	void update();
	void setup(uint8_t channel,int16_t driveWorkTime);
	void setSettings(int16_t driveWorkTime);
	
	bool isOn() { return valveState == hvsOpening || valveState == hvsOpen;}	
	bool isBusy();
	
	private:
	
		uint32_t workTime;
		uint8_t channel;
		ValveState valveState;
		uint32_t valveTimer;
		bool timerActive;
		uint32_t timerInterval;
		
		void turnValve(ValveTurnMode mode);
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SHADOW_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	hwmAutomatic,
	hwmManual
	
} HeatWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
	swmAutomatic,
	swmManual
	
} ShadowWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<WindowDriveCommand> WindowDriveCommandList;
//--------------------------------------------------------------------------------------------------------------------------------------
class RainSensor // датчик дождя
{
	
	public:
		RainSensor();
		~RainSensor();
		
		void setup();
		void update();
		bool hasRain();
		
		void setHasRain(bool b);
		
	private:
		bool rainFlag;
		uint32_t lastDataAt;
};
//--------------------------------------------------------------------------------------------------------------------------------------
class WindSensor // датчик ветра
{
	public:
		WindSensor();
		~WindSensor();
		
		void setup();
		void update();
		
		uint32_t getWindSpeed() { return windSpeed; };	// скорость ветра, в сотых долях м/с
		void setWindSpeed(uint32_t ws);
		
		CompassPoints getWindDirection() { return windDirection; }; // направление ветра		
		void setWindDirection(CompassPoints cp);

	protected:
	
		volatile uint32_t pulses;		
		static void pulse();
		
	private:
	
		uint32_t timer;
		uint32_t windSpeed;
		CompassPoints windDirection;
		
		uint32_t lastDataAt;
	
};
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  cvmIdle,
  cvmWaitOff,
  cvmWaitOn,
  
} CycleVentMachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  cvwmAuto,
  cvwmManual
  
} CycleVentWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
class CycleVent
{
  public:
    CycleVent();
    void setup(uint8_t channel);

    bool isOn() { return onFlag; }
    CycleVentWorkMode getWorkMode() { return workMode; }

    // управление в ручном режиме
    void switchToMode(CycleVentWorkMode mode) { workMode = mode; machineState = cvmIdle; }
    void turn(bool isOn) { if(isOn) { on(); } else { off(); }  machineState = cvmIdle; }

    void disable();
    void enable();
    void update(CycleVentSettings& settings);

  private:
    bool activeFlag, onFlag;
    uint8_t channel;
    CycleVentMachineState machineState;
    uint32_t waitTimer, waitInterval;

    CycleVentWorkMode workMode;

    void on();
    void off();

    bool canWork(CycleVentSettings& sett);
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CYCLE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ventIdle,
  ventCheckOff,
  ventRest,
  ventWaitAddInterval,
  
} VentMachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  vwmAuto,
  vwmManual
  
} VentWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
class Vent
{
  public:
    Vent();

    void setup(uint8_t channel);
    void reloadSettings();
    void update();

    bool isOn() { return onFlag; }
    VentWorkMode getWorkMode() { return workMode; }

    // управление в ручном режиме
    void switchToMode(VentWorkMode mode) { workMode = mode; machineState = ventIdle; }
    void turn(bool isOn) { setState(isOn); machineState = ventIdle; }
    
    VentSettings& getSettings() { return settings; }

  private:
    VentSettings settings;
    uint8_t channel;
    bool onFlag;
    VentMachineState machineState;
    uint32_t tempTimer, workStartedAt, addInterval;

    VentWorkMode workMode;

    void setState(bool on);
    void checkWantOn();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_VENT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  twmAuto,
  twmManual
  
} ThermostatWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
class Thermostat
{
  public:
    Thermostat();

    void setup(uint8_t channel);
    void reloadSettings();
    void update();

    bool isOn() { return onFlag; }
    ThermostatWorkMode getWorkMode() { return workMode; }

    // управление в ручном режиме
    void switchToMode(ThermostatWorkMode mode) { workMode = mode;}
    void turn(bool isOn) { setState(isOn);}

    ThermostatSettings& getSettings() { return settings; }

  private:
    ThermostatSettings settings;
    uint8_t channel;
    bool onFlag;

    uint32_t tempTimer;
    ThermostatWorkMode workMode;

    void setState(bool on);
    void check();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_THERMOSTAT_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  hsmIdle,
  hsmCheckOff,  
} HSMMachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  hsmAuto,
  hsmManual
  
} HSMWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
class HumiditySpray
{
  public:
    HumiditySpray();

    void setup(uint8_t channel);
    void reloadSettings();
    void update();

    bool isOn() { return onFlag; }
    HSMWorkMode getWorkMode() { return workMode; }

    // управление в ручном режиме
    void switchToMode(HSMWorkMode mode) { workMode = mode; machineState = hsmIdle; }
    void turn(bool isOn) { setState(isOn); machineState = hsmIdle; }
    
    HumiditySpraySettings& getSettings() { return settings; }

  private:
    HumiditySpraySettings settings;
    uint8_t channel;
    bool onFlag;
    HSMMachineState machineState;

    HSMWorkMode workMode;

    void setState(bool on);
    bool canWork();
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HUMIDITY_SPRAY_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
class LogicManageModuleClass : public AbstractModule
{
  private:
  
	bool inited;
	bool _can_work;
	
	RainSensor rainSensor; // датчик дождя
	WindSensor windSensor; // датчик ветра
	
  
	#ifdef USE_WINDOW_MANAGE_MODULE // окна
		
		WManageChannel windowsSection1;
		WManageChannel windowsSection2;
		WManageChannel windowsSection3;
		WManageChannel windowsSection4;		
		
		WindowsChannelsBinding windowsBySectionsBinding; // привязки окон к секциям
		WindowsChannelsBinding windowsRainBinding; // привязки окон к датчикам дождя
		WindowsChannelsBinding windowsOrientationBinding; // привязка окон по сторонам света
		
		int16_t windSpeed, hurricaneSpeed; // уставки скорости ветра
    void addToWindList(Vector<size_t>& list, size_t windowNumber);
				
		uint32_t windowDecisionTimer;
		
		void makeWindowsDecision();
		void makeWindowsSectionDecision(uint8_t sectionIndex, WManageChannel& section);
		
	#endif // USE_WINDOW_MANAGE_MODULE

	#ifdef USE_HEAT_MODULE // отопление
	
		HeatWorkMode heatWorkMode1;
		HeatWorkMode heatWorkMode2;
		HeatWorkMode heatWorkMode3;
		
		HeatSettings heatSection1;
		HeatSettings heatSection2;
		HeatSettings heatSection3;
				
		Heater heater1;
		Heater heater2;
		Heater heater3;
    bool inHeaterResetMode;
		
		uint32_t heatDecisionTimer;
		
		void makeHeatDecision();
		void makeHeatDecision(Heater& heater, HeatSettings& section, HeatDriveCommand& decision, uint8_t& decisionPercents, float& lastI);
		
   
	#endif
	
	#ifdef USE_SHADOW_MODULE // затенение
			
		ShadowWorkMode shadowWorkMode1;
		ShadowWorkMode shadowWorkMode2;
		ShadowWorkMode shadowWorkMode3;
		
		ShadowSettings shadowSection1;
		ShadowSettings shadowSection2;
		ShadowSettings shadowSection3;
				
		Shadow shadow1;
		Shadow shadow2;
		Shadow shadow3;
		
		uint32_t shadowDecisionTimer;
		
		void makeShadowDecision();
		void makeShadowDecision(Shadow& shadow, ShadowSettings& section, ShadowDriveCommand& decision);
		
	#endif
	
	#if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE) // досветка
		LightSettings lightSettings;
		uint32_t lightDecisionTimer;
		void makeLightDecision();
	#endif

  #ifdef USE_CYCLE_VENT_MODULE
    CycleVentSettings cycleVentSettings1;
    CycleVentSettings cycleVentSettings2;
    CycleVentSettings cycleVentSettings3;

    CycleVent cycleVent1;
    CycleVent cycleVent2;
    CycleVent cycleVent3;

    void makeCycleVentDecision();
    void makeCycleVentDecision(CycleVent& vent,CycleVentSettings& sett, CycleVentDriveCommand& decision);
  #endif

  #ifdef USE_VENT_MODULE
    Vent vent1;
    Vent vent2;
    Vent vent3;
    void makeVentDecision();
  #endif

  #ifdef USE_HUMIDITY_SPRAY_MODULE
    HumiditySpray spray1;
    HumiditySpray spray2;
    HumiditySpray spray3;
    void makeSprayDecision();
  #endif

  #ifdef USE_THERMOSTAT_MODULE
    Thermostat thermostat1;
    Thermostat thermostat2;
    Thermostat thermostat3;
    void makeThermostatDecision();
  #endif  
	
	// служебные
	void firstCallSetup();
	
	uint8_t tempSensorsCount, humiditySensorsCount;

	// управление

	// досветкой
	LightDriveCommand lastLightCommand;
	bool lastLightAutoMode;
	
	void lightOff();
	void lightOn();
	bool isLightOn();
		
	
	// механизмы принятия решений
	
	// досветка
	LightDriveCommand lightDecision;
	void applyLightDecision();
	
	// отопление

  
	HeatDriveCommand heatSection1Decision;
	HeatDriveCommand heatSection2Decision;
	HeatDriveCommand heatSection3Decision;

  uint8_t heatSection1DecisionPercents;
  uint8_t heatSection2DecisionPercents;
  uint8_t heatSection3DecisionPercents;
  float heatSection1LastI;
  float heatSection2LastI;
  float heatSection3LastI;
  
	void applyHeatDecision();
 uint8_t ComputePI(int32_t cT, int32_t wT, float& i, uint8_t kP, float pMin, float pMax, float kI, float iMin, float iMax, float kProp);
	
	
	// затенение
	ShadowDriveCommand shadowSection1Decision;
	ShadowDriveCommand shadowSection2Decision;
	ShadowDriveCommand shadowSection3Decision;
	void applyShadowDecision();
	
	// фрамуги
	WindowDriveCommandList windowsDecision;
	WindowDriveCommandList lastWindowsCommand;
	bool lastWindowAutoMode;
	void initWindowsDecisionList();
	void applyWindowsDecision();


  // циклическая вентиляция
  CycleVentDriveCommand cycleVent1Decision;
  CycleVentDriveCommand cycleVent2Decision;
  CycleVentDriveCommand cycleVent3Decision;
  void applyCycleVentDecision();

	
	
	// разруливание зависимостей
	void solveConflicts();
  
  public:
    LogicManageModuleClass();

    Temperature getTemperature(uint8_t sensorIndex);
    Humidity getHumidity(uint8_t sensorIndex);
    int32_t getLux(uint8_t sensorIndex);
  
    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();
	
	#ifdef USE_WINDOW_MANAGE_MODULE
		void ReloadWindowsSettings();
	#endif
	
	#ifdef USE_HEAT_MODULE
		void ReloadHeatSettings();
		void GetHeatTemperature(uint8_t section,Temperature& sensor, Temperature& air);
		void GetHeatStatus(uint8_t section,bool& active, bool& on);
		void TurnHeat(uint8_t section, bool on); // управление в ручном режиме
		void TurnHeatMode(uint8_t section, HeatWorkMode m);
		HeatWorkMode GetHeatMode(uint8_t section);
	#endif

	#ifdef USE_SHADOW_MODULE
		void ReloadShadowSettings();
		void GetShadowStatus(uint8_t section,bool& active, bool& on);
		void TurnShadow(uint8_t section, bool on); // управление в ручном режиме
		void TurnShadowMode(uint8_t section, ShadowWorkMode m);
		ShadowWorkMode GetShadowMode(uint8_t section);
	#endif
	
	#if defined(USE_LUMINOSITY_MODULE)  && defined(USE_LIGHT_MANAGE_MODULE)
		void ReloadLightSettings();
	#endif

  #ifdef USE_CYCLE_VENT_MODULE
    void ReloadCycleVentSettings();
    CycleVent* getCycleVent(uint8_t channel) { if(channel == 1) return &cycleVent2; else if(channel == 2) return &cycleVent3; return &cycleVent1; }
  #endif

  #ifdef USE_VENT_MODULE
    void ReloadVentSettings();
    Vent* getVent(uint8_t channel) { if(channel == 1) return &vent2; else if(channel == 2) return &vent3; return &vent1; }
  #endif

  #ifdef USE_HUMIDITY_SPRAY_MODULE
    void ReloadHumiditySpraySettings();
    HumiditySpray* getSpray(uint8_t channel) { if(channel == 1) return &spray2; else if(channel == 2) return &spray3; return &spray1; }
  #endif

  #ifdef USE_THERMOSTAT_MODULE
    void ReloadThermostatSettings();
    Thermostat* getThermostat(uint8_t channel) { if(channel == 1) return &thermostat2; else if(channel == 2) return &thermostat3; return &thermostat1; }
  #endif  
  
	bool HasRain() { return rainSensor.hasRain(); }
	void SetHasRain(bool b) { rainSensor.setHasRain(b); }
	
	void SetWindSpeed(uint32_t ws) { windSensor.setWindSpeed(ws); }
	uint32_t GetWindSpeed() { return windSensor.getWindSpeed(); }
	
	void SetWindDirection(CompassPoints cp) { windSensor.setWindDirection(cp); }
	CompassPoints GetWindDirection() { return windSensor.getWindDirection(); }
	
	
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern LogicManageModuleClass* LogicManageModule;
//--------------------------------------------------------------------------------------------------------------------------------------
