#ifndef _NEXTION_MODULE_H
#define _NEXTION_MODULE_H

#include "Common.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
    bool isDisplaySleep : 1;
    bool isWindowsOpen : 1;
    bool isWindowAutoMode : 1;
    bool isWaterOn : 1;
    bool isWaterAutoMode : 1;
    bool isLightOn : 1;
    bool isLightAutoMode : 1;
    
    bool windowChanged : 1;
    bool windowModeChanged : 1;
    bool waterChanged : 1;
    bool waterModeChanged : 1;
    bool lightChanged : 1;
    bool lightModeChanged : 1;
    bool openTempChanged : 1;
    bool closeTempChanged : 1;
  
} NextionModuleFlags;
//--------------------------------------------------------------------------------------------------------------------------------------
#define NEXTION_START_PAGE 0
#define NEXTION_MENU_PAGE 1
#define NEXTION_WINDOWS_PAGE 2
#define NEXTION_WATER_PAGE 3
#define NEXTION_LIGHT_PAGE 4
#define NEXTION_OPTIONS_PAGE 5
#define NEXTION_WINDOWS_CHANNELS_PAGE 6
#define NEXTION_WATER_CHANNELS_PAGE 7
#define NEXTION_SCENE_PAGE 9
//--------------------------------------------------------------------------------------------------------------------------------------
class NextionModule
{
  private:
  
    NextionModuleFlags flags;
    uint8_t openTemp, closeTemp;
    unsigned long rotationTimer;
    
    void updateDisplayData();

    void displayNextSensorData(int8_t dir=1);
    int8_t currentSensorIndex;

    uint8_t currentPage;
    void UpdatePageData(uint8_t pageId);

    #ifdef DISPLAY_WINDOWS_PAGE
    uint16_t windowsPositionFlags;
    #endif

    #ifdef DISPLAY_WATERING_PAGE
    uint16_t waterChannelsState;
    #endif


    CommandsPool pool;
    ControllerState currentState;
    SensorsList sensors;
    RemoteDisplaySettingsPacket settings;
    uint32_t resetSensorsTimer;
  
  public:
    NextionModule() { resetSensorsTimer = 0; }

    void begin();
    void update(uint16_t dt);

    uint8_t getOutgoingCommandsCount() { return pool.size(); }
    CommandToExecute getOutgoingCommand(uint8_t idx) { return pool[idx]; }
    void clearOutgoingCommands(){ pool.clear(); }

    void setControllerState(ControllerState& val) {currentState = val; }
    void setSettings(RemoteDisplaySettingsPacket& s) { settings = s; } 

    void clearSensors() { sensors.clear(); resetSensorsTimer = millis(); }
    void addSensor(SensorDisplayData& dt) { sensors.push_back(dt); resetSensorsTimer = millis(); }
    
    void SetSleep(bool bSleep);
    void StringReceived(const char* str);
    void OnPageChanged(uint8_t pageID);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern NextionModule display;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
