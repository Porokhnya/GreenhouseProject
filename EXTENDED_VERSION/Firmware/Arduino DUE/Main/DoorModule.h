#pragma once

#include "AbstractModule.h"
#include "Settings.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  doorModeAutomatic,
  doorModeManual
  
} DoorWorkMode;
//--------------------------------------------------------------------------------------------------------------------------------------
class Door
{
  public:
    Door();
    void setSettings(DoorSettings* sett);
    
    void setup(uint8_t idx);
    void update();
    
    void switchMode(DoorWorkMode mode);
    DoorWorkMode getMode();

    void close();
    void open();

    bool isOpen();
    bool isBusy();
    

  private:

    uint32_t powerTimerDelay;
  
    DoorSettings* settings;
    DoorWorkMode workMode;
    uint8_t channel;
    bool openFlag, onMyWay;
    uint32_t timer;
    uint8_t hops;
    uint8_t pinState1, pinState2;

    void setOutputs(uint8_t state1, uint8_t state2);

    uint8_t tempSensorsCount;
    bool canWorkWithSensor;
    uint32_t controlTimer;
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
class DoorModule : public AbstractModule // модуль управления входными дверями
{
  private:

    DoorSettings door1_Settings, door2_Settings;

    Door doors[2];
  
  public:
    DoorModule();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    void ReloadDoorSettings();
    
    DoorWorkMode GetDoorMode(uint8_t idx);
    void SetDoorMode(uint8_t idx, DoorWorkMode mode);
    
    bool IsDoorOpen(uint8_t idx);
    void CloseDoor(uint8_t idx);
    void OpenDoor(uint8_t idx);
    bool IsDoorBusy(uint8_t idx);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern DoorModule* Doors;

