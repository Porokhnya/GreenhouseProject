#ifndef _NEXTION_MODULE_H
#define _NEXTION_MODULE_H

#include "AbstractModule.h"
#include "Settings.h"

#ifdef USE_NEXTION_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
 typedef struct
 {
  uint8_t sensorType;
  uint8_t sensorIndex;
  const char* moduleName;
  const char* caption;
    
 } NextionWaitScreenInfo; // структура для хранения информации, которую необходимо показывать на экране ожидания
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
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
#pragma pack(pop)
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
class NextionModule : public AbstractModule // модуль управления дисплеем Nextion
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

    void updateTime();

    #ifdef USE_TEMP_SENSORS
    uint16_t windowsPositionFlags;
    #endif

    #ifdef USE_WATERING_MODULE
    uint16_t waterChannelsState;
    #endif

    #ifdef USE_SCENE_MODULE
    Vector<bool> scenesState;
    #endif
  
  public:
    NextionModule() : AbstractModule("NXT") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);
    
    void SetSleep(bool bSleep);
    void StringReceived(const char* str);
    void OnPageChanged(uint8_t pageID);

};
#endif // USE_NEXTION_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
