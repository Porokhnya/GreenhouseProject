#pragma once

#include "AbstractModule.h"
#include "Settings.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class CO2Module : public AbstractModule // заготовка для модуля
{
  private:

    uint32_t updateTimer;
    CO2Settings settings;
    void LoadSettings();
    void setupOutputs();

    void control();
    void out(uint8_t pin,uint8_t level);
    bool alertActive;
    uint32_t alertTimer;

    bool alertOn, co2On;
  
  public:
    CO2Module();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update();

    void ReloadSettings();

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CO2Module* CO2Control;

