#include "EEPROMSettingsModule.h"
#include "ModuleController.h"
#include "HumidityGlobals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
EEPROMSettingsModule* HardwareBinding = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
const uint8_t UNSAFE_PINS[] = { // список небезопасных пинов
  // Serial
   0
  ,1
  // запрещаем работу с I2C
  ,SDA
  ,SCL

  // запрещаем работу с SPI
  ,MOSI
  ,MISO
  ,SCK

  #ifdef UNSAFE_PIN_LIST
  , UNSAFE_PIN_LIST // добавляем запрещённые пользователем пины
  #endif

  // пин ватчдога
  #ifdef USE_EXTERNAL_WATCHDOG
  ,WATCHDOG_REBOOT_PIN
  #endif
   
   // SD-карта
   #ifdef SDCARD_CS_PIN
    ,SDCARD_CS_PIN
   #endif


   // TFT
   #ifdef TFT_TOUCH_CLK_PIN
    ,TFT_TOUCH_CLK_PIN
   #endif
   
  #ifdef TFT_TOUCH_CS_PIN
    ,TFT_TOUCH_CS_PIN
  #endif

  #ifdef TFT_TOUCH_DIN_PIN
    ,TFT_TOUCH_DIN_PIN
  #endif

  #ifdef TFT_TOUCH_DOUT_PIN
    ,TFT_TOUCH_DOUT_PIN
  #endif

  #ifdef TFT_TOUCH_IRQ_PIN
    ,TFT_TOUCH_IRQ_PIN
  #endif

  #ifdef INCH_4_TOUCH_CS
    ,INCH_4_TOUCH_CS
  #endif

  #ifdef INCH_4_TOUCH_IRQ
    ,INCH_4_TOUCH_IRQ
  #endif

  #ifdef INCH_7_TOUCH_CS
    ,INCH_7_TOUCH_CS
  #endif

  #ifdef INCH_7_TOUCH_IRQ
    ,INCH_7_TOUCH_IRQ
  #endif


  #ifdef TFT_RS_PIN
    ,TFT_RS_PIN
  #endif

  #ifdef TFT_WR_PIN
    ,TFT_WR_PIN
  #endif

  #ifdef TFT_CS_PIN
    ,TFT_CS_PIN
  #endif

  #ifdef TFT_RST_PIN
    ,TFT_RST_PIN
  #endif

  #if defined(DISABLE_TFT_CONFIGURE) && defined(USE_TFT_MODULE)
    #if (TFT_BACKLIGHT_LINK_TYPE == linkDirect) && (TFT_BACKLIGHT_PIN != UNBINDED_PIN)
    , TFT_BACKLIGHT_PIN
    #endif
  #endif

  #if defined(DISABLE_WIFI_CONFIGURE) && defined(USE_WIFI_MODULE)
    #if (WIFI_REBOOT_PIN_LINK_TYPE == linkDirect) && (WIFI_REBOOT_PIN != UNBINDED_PIN)
    , WIFI_REBOOT_PIN
    #endif
  #endif

  #if defined(DISABLE_NRF_CONFIGURE) && defined(USE_NRF_GATE)
    #if (NRF_REBOOT_PIN_LINK_TYPE == linkDirect) && (NRF_REBOOT_PIN != UNBINDED_PIN)
    , NRF_REBOOT_PIN
    #endif

    #if (NRF_CE_PIN != UNBINDED_PIN)
    , NRF_CE_PIN
    #endif
    
    #if (NRF_CSN_PIN != UNBINDED_PIN)
    , NRF_CSN_PIN
    #endif
  #endif

  #if defined(DISABLE_LORA_CONFIGURE) && defined(USE_LORA_GATE)
    #if (LORA_REBOOT_PIN_LINK_TYPE == linkDirect) && (LORA_REBOOT_PIN != UNBINDED_PIN)
    , LORA_REBOOT_PIN
    #endif

    #if (LORA_SS_PIN != UNBINDED_PIN)
      , LORA_SS_PIN
    #endif

    #if (LORA_RESET_PIN != UNBINDED_PIN)
    , LORA_RESET_PIN
    #endif
  #endif

  #if defined(DISABLE_BUZZER_CONFIGURE) && defined(USE_BUZZER)
    #if (BUZZER_LINK_TYPE == linkDirect) && (BUZZER_PIN != UNBINDED_PIN)
    , BUZZER_PIN
    #endif
  #endif

  #if defined(DISABLE_GSM_CONFIGURE) && defined(USE_SMS_MODULE)
    #if (GSM_REBOOT_PIN_LINK_TYPE == linkDirect) && (GSM_REBOOT_PIN != UNBINDED_PIN)
    , GSM_REBOOT_PIN
    #endif

    #if (GSM_POWERKEY_LINK_TYPE == linkDirect) && (GSM_POWERKEY_PIN != UNBINDED_PIN)
    , GSM_POWERKEY_PIN
    #endif
  #endif

  #if defined(DISABLE_RS485_CONFIGURE) && defined (USE_RS485_GATE)
    #if (RS485_LINK_TYPE == linkDirect) && (RS485_DE_PIN != UNBINDED_PIN)
    , RS485_DE_PIN
    #endif
  #endif

  #if defined(USE_INFO_DIODES) && defined(DISABLE_DIODES_CONFIGURE)
    #if (DIODES_LINK_TYPE == linkDirect)
      #if (READY_DIODE_PIN != UNBINDED_PIN)
      , READY_DIODE_PIN
      #endif
      #if (WINDOWS_DIODE_PIN != UNBINDED_PIN)
      , WINDOWS_DIODE_PIN
      #endif
      #if (WATERING_DIODE_PIN != UNBINDED_PIN)
      , WATERING_DIODE_PIN
      #endif
      #if (LIGHT_DIODE_PIN != UNBINDED_PIN)
      , LIGHT_DIODE_PIN
      #endif
    #endif
  #endif
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
bool EEPROMSettingsModule::SafePin(uint8_t pin)
{

  if(pin == UNBINDED_PIN)
  {
    return false;
  }

  for(size_t i=0;i<sizeof(UNSAFE_PINS)/sizeof(UNSAFE_PINS[0]);i++)
  {
      if(pin == UNSAFE_PINS[i])
        return false;
    
  } // for

    return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
EEPROMSettingsModule::EEPROMSettingsModule() : AbstractModule("EES")
{
  HardwareBinding = this;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void EEPROMSettingsModule::Setup()
{
  // читаем настройки CO2
  memset(&co2Binding,0,sizeof(co2Binding));
  if(!read(CO2_BINDING_ADDRESS, co2Binding))
  {
    // применяем настройки по умолчанию
    memset(&co2Binding,0,sizeof(co2Binding));
    
    co2Binding.RelayPin = UNBINDED_PIN;
    co2Binding.AlertPin = UNBINDED_PIN;
    co2Binding.Level = LOW;
    co2Binding.VentPin = UNBINDED_PIN;
    co2Binding.SensorPin = UNBINDED_PIN;
    co2Binding.MaxPPM = MAX_CO2_SENSOR_PPM;
    co2Binding.MinADCPPMValue = CO2_PPM_MIN_ADC_VALUE;
    co2Binding.MaxADCPPMValue = CO2_PPM_MAX_ADC_VALUE;
    co2Binding.AlertPercents = CO2_ALERT_PERCENTS;

    co2Binding.LinkType = linkUnbinded;
  }

  // читаем настройки pH
  #ifdef USE_PH_MODULE
  memset(&phBinding,0,sizeof(phBinding));
  if(!read(PH_BINDING_ADDRESS, phBinding))
  {
      memset(&phBinding,0,sizeof(phBinding));
      
      phBinding.SensorPin = UNBINDED_PIN;
      phBinding.MVPer7Ph = PH_MV_PER_7_PH;
      phBinding.FlowLevelSensorPin = UNBINDED_PIN;
      phBinding.FlowLevelSensorPin2 = UNBINDED_PIN;
      phBinding.FlowAddPin = UNBINDED_PIN;
      phBinding.PhPlusPin = UNBINDED_PIN;
      phBinding.PhMinusPin = UNBINDED_PIN;
      phBinding.PhMixPin = UNBINDED_PIN;
      phBinding.FlowSensorsLevel = PH_FLOW_LEVEL_TRIGGERED;
      phBinding.OutputsLevel = PH_OUTPUTS_LEVEL_TRIGGERED;

      phBinding.LinkTypeOutputs = linkUnbinded;
      phBinding.LinkTypeInputs = linkUnbinded;  

      phBinding.VRef = 2400;
  }
  #else // EC MODULE
  memset(&ecphBinding,0,sizeof(ecphBinding));
  if(!read(PH_BINDING_ADDRESS, ecphBinding))
  {
      memset(&ecphBinding,0,sizeof(ecphBinding));      
      ecphBinding.SensorPin = UNBINDED_PIN;
      ecphBinding.MVPer7Ph = EC_PH_MV_PER_7_PH;
      ecphBinding.VRef = 2400;
  }

  memset(&ecBinding,0,sizeof(ecBinding));
  if(!read(EC_BINDING_ADDRESS, ecBinding))
  {
      memset(&ecBinding,0,sizeof(ecBinding));      
      ecBinding.LinkType = linkUnbinded;
        for(uint8_t k=0;k<sizeof(ecBinding.SensorPins)/sizeof(ecBinding.SensorPins[0]);k++)
        {
          ecBinding.SensorPins[k] = UNBINDED_PIN;
        }

        ecBinding.PinA = UNBINDED_PIN;
        ecBinding.PinB = UNBINDED_PIN;
        ecBinding.PinC = UNBINDED_PIN;

        ecBinding.PinPhPlus = UNBINDED_PIN;
        ecBinding.PinPhMinus = UNBINDED_PIN;
        ecBinding.PinWater = UNBINDED_PIN;
        ecBinding.WaterValve = UNBINDED_PIN;
        ecBinding.WaterContour = UNBINDED_PIN;
        ecBinding.PumpPin = UNBINDED_PIN;
        ecBinding.MaxPpm = 1000;
        
  }
  
  #endif // EC module

  // читаем настройки термостатов
  memset(thermostatBinding,0,sizeof(thermostatBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = THERMOSTAT_BINDING_ADDRESS + i*(sizeof(ThermostatBinding) + 3); // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, thermostatBinding[i]))
      {
        memset(&(thermostatBinding[i]),0,sizeof(ThermostatBinding));
        
        thermostatBinding[i].LinkType = linkUnbinded; // нет привязки
        thermostatBinding[i].Pin = UNBINDED_PIN; // нет привязки
        thermostatBinding[i].Level = LOW;        
      }
    
  } // for

  // читаем настройки вентиляции
  memset(ventBinding,0,sizeof(ventBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = VENT_BINDING_ADDRESS + i*(sizeof(VentBinding) + 3);  // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, ventBinding[i]))
      {
         memset(&(ventBinding[i]),0,sizeof(VentBinding));
         
        ventBinding[i].LinkType = linkUnbinded; // нет привязки
        ventBinding[i].Pin = UNBINDED_PIN; // нет привязки
        ventBinding[i].Level = LOW;        
      }
    
  } // for

  // читаем настройки воздухообмена
  memset(cycleVentBinding,0,sizeof(cycleVentBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = CYCLE_VENT_BINDING_ADDRESS + i*(sizeof(CycleVentBinding) + 3); // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, cycleVentBinding[i]))
      {

        memset(&(cycleVentBinding[i]),0,sizeof(CycleVentBinding));
        
        cycleVentBinding[i].LinkType = linkUnbinded; // нет привязки
        cycleVentBinding[i].Pin = UNBINDED_PIN; // нет привязки
        cycleVentBinding[i].Level = LOW;        
      }
    
  } // for  


  // читаем настройки затенения
  memset(shadowBinding,0,sizeof(shadowBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = SHADOW_BINDING_ADDRESS + i*(sizeof(ShadowBinding) + 3); // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, shadowBinding[i]))
      {
        memset(&(shadowBinding[i]),0,sizeof(ShadowBinding));
        
        shadowBinding[i].LinkType = linkUnbinded; // нет привязки
        shadowBinding[i].Pin1 = UNBINDED_PIN; // нет привязки
        shadowBinding[i].Pin2 = UNBINDED_PIN; // нет привязки
        shadowBinding[i].Level = LOW;        
      }
    
  } // for  

  // читаем настройки отопления
  memset(heatBinding,0,sizeof(heatBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = HEAT_BINDING_ADDRESS + i*(sizeof(HeatBinding) + 3); // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, heatBinding[i]))
      {
        memset(&(heatBinding[i]),0,sizeof(HeatBinding));
        
        heatBinding[i].LinkType = linkUnbinded; // нет привязки
        heatBinding[i].Pin1 = UNBINDED_PIN; // нет привязки
        heatBinding[i].Pin2 = UNBINDED_PIN; // нет привязки
        heatBinding[i].PumpPin = UNBINDED_PIN; // нет привязки
      }
    
  } // for    


  // читаем настройки досветки
  memset(&lightBinding,0,sizeof(lightBinding));
  if(!read(LIGHT_BINDING_ADDRESS, lightBinding))
  {
      memset(&lightBinding,0,sizeof(lightBinding));
      
      lightBinding.LinkType = linkUnbinded;

      for(size_t i=0;i<sizeof(lightBinding.Sensors)/sizeof(lightBinding.Sensors[0]);i++)
      {
        lightBinding.Sensors[i] = NO_LIGHT_SENSOR; // нет привязки
      }

      for(size_t i=0;i<sizeof(lightBinding.LampRelays)/sizeof(lightBinding.LampRelays[0]);i++)
      {
        lightBinding.LampRelays[i] = UNBINDED_PIN; // нет привязки
      }

      lightBinding.Level = LIGHT_RELAY_ON;
      lightBinding.AveragingSamples = 10;
      lightBinding.HarboringStep = 200;

  }  

  // читаем настройки датчика дождя
  memset(&rainSensorBinding,0,sizeof(rainSensorBinding));
  if(!read(RAIN_BINDING_ADDRESS, rainSensorBinding))
  {
      memset(&rainSensorBinding,0,sizeof(rainSensorBinding));
      
      rainSensorBinding.LinkType = linkUnbinded;
      rainSensorBinding.Level = HIGH;
      rainSensorBinding.Pin = UNBINDED_PIN;
      rainSensorBinding.ResetInterval = 120;
  }  

  // читаем настройки датчика ветра
  memset(&windSensorBinding,0,sizeof(windSensorBinding));
  if(!read(WIND_BINDING_ADDRESS, windSensorBinding))
  {
      memset(&windSensorBinding,0,sizeof(windSensorBinding));
      
      windSensorBinding.Pin = UNBINDED_PIN;
      windSensorBinding.DirectionPin = UNBINDED_PIN;
      windSensorBinding.SpeedCoeff = 34; // 0.34
      windSensorBinding.EastAdcFrom = 297;
      windSensorBinding.EastAdcTo = 682;
      windSensorBinding.SouthAdcFrom = 1104;
      windSensorBinding.SouthAdcTo = 2494;
      windSensorBinding.WestAdcFrom = 3798;
      windSensorBinding.WestAdcTo = 3525;
      windSensorBinding.NorthAdcFrom = 3127;
      windSensorBinding.NorthAdcTo = 1799;
      windSensorBinding.Histeresis = 50;
      windSensorBinding.NumSamples = 10;
  }  

  #ifndef DISABLE_BUZZER_CONFIGURE
    // читаем настройки пищалки
    memset(&buzzerBinding,0,sizeof(buzzerBinding));
    if(!read(BUZZER_BINDING_ADDRESS, buzzerBinding))
    {
        memset(&buzzerBinding,0,sizeof(buzzerBinding));
        
        buzzerBinding.LinkType = linkUnbinded;
        buzzerBinding.Level = HIGH;
        buzzerBinding.Pin = UNBINDED_PIN;
        buzzerBinding.Duration = 60;
    }
  #else
    // применяем настройки пищалки по умолчанию
        buzzerBinding.LinkType = BUZZER_LINK_TYPE;
        buzzerBinding.MCPAddress = BUZZER_MCP_ADDRESS;
        buzzerBinding.Level = BUZZER_LEVEL;
        buzzerBinding.Pin = BUZZER_PIN;
        buzzerBinding.Duration = BUZZER_DURATION;
  #endif 

  #ifndef DISABLE_NRF_CONFIGURE
  // читаем настройки nRF
  memset(&nrfBinding,0,sizeof(nrfBinding));
  if(!read(NRF_BINDING_ADDRESS, nrfBinding))
  {
      memset(&nrfBinding,0,sizeof(nrfBinding));
      
      nrfBinding.LinkType = linkUnbinded;
      nrfBinding.CEPin = UNBINDED_PIN;
      nrfBinding.CSNPin = UNBINDED_PIN;
      nrfBinding.PowerPin = UNBINDED_PIN;
      nrfBinding.Level = LOW;
  }
  #else
      // применяем настройки по умолчанию
      nrfBinding.LinkType = NRF_REBOOT_PIN_LINK_TYPE;
      nrfBinding.MCPAddress = NRF_REBOOT_PIN_MCP_ADDRESS;
      nrfBinding.CEPin = NRF_CE_PIN;
      nrfBinding.CSNPin = NRF_CSN_PIN;
      nrfBinding.PowerPin = NRF_REBOOT_PIN;
      nrfBinding.Level = NRF_POWER_ON;
      nrfBinding.AutoAckInverted = NRF_AUTOACK_INVERTED;
  #endif  

  #ifndef DISABLE_LORA_CONFIGURE
    // читаем настройки LoRa
    memset(&loraBinding,0,sizeof(loraBinding));
    if(!read(LORA_BINDING_ADDRESS, loraBinding))
    {
        memset(&loraBinding,0,sizeof(loraBinding));
        
        loraBinding.LinkType = linkUnbinded;
        loraBinding.SSPin = UNBINDED_PIN;
        loraBinding.ResetPin = UNBINDED_PIN;
        loraBinding.PowerPin = UNBINDED_PIN;
        loraBinding.Level = HIGH;
        loraBinding.TXPower = 17;
        loraBinding.Frequency = 868;
    }
  #else
        // применяем настройки по умолчанию
        loraBinding.LinkType = LORA_REBOOT_PIN_LINK_TYPE;
        loraBinding.MCPAddress = LORA_REBOOT_PIN_MCP_ADDRESS;
        loraBinding.SSPin = LORA_SS_PIN;
        loraBinding.ResetPin = LORA_RESET_PIN;
        loraBinding.PowerPin = LORA_REBOOT_PIN;
        loraBinding.Level = LORA_POWER_ON;
        loraBinding.TXPower = LORA_TX_POWER;
        loraBinding.Frequency = LORA_FREQUENCY;
  #endif 

  // читаем настройки таймеров
  memset(&timerBinding,0,sizeof(timerBinding));
  if(!read(TIMER_BINDING_ADDRESS, timerBinding))
  {
      memset(&timerBinding,0,sizeof(timerBinding));
      
      timerBinding.LinkType = linkUnbinded;
      timerBinding.Level = HIGH;
  }  

  // читаем настройки DS18B20
  memset(&dallasBinding,UNBINDED_PIN,sizeof(dallasBinding));
  if(!read(DALLAS_BINDING_ADDRESS, dallasBinding))
  {
      memset(&dallasBinding,UNBINDED_PIN,sizeof(dallasBinding));
  }  

  // читаем настройки эмуляторов DS18B20
  memset(&ds18B20EmulationBinding,UNBINDED_PIN,sizeof(ds18B20EmulationBinding));
  if(!read(DS18B20_EMULATION_BINDING_ADDRESS, ds18B20EmulationBinding))
  {
      memset(&ds18B20EmulationBinding,UNBINDED_PIN,sizeof(ds18B20EmulationBinding));
  }  

  // читаем настройки 1-Wire
  memset(&oneWireBinding,UNBINDED_PIN,sizeof(oneWireBinding));
  if(!read(ONEWIRE_BINDING_ADDRESS, oneWireBinding))
  {
      memset(&oneWireBinding,UNBINDED_PIN,sizeof(oneWireBinding));
  }  

  #ifndef DISABLE_DIODES_CONFIGURE
  // читаем настройки информационных диодов
  memset(&diodesBinding,0,sizeof(diodesBinding));
  if(!read(DIODES_BINDING_ADDRESS, diodesBinding))
  {
      memset(&diodesBinding,0,sizeof(diodesBinding));

      diodesBinding.ReadyDiodePin = UNBINDED_PIN;
      diodesBinding.WindowsManualModeDiodePin = UNBINDED_PIN;
      diodesBinding.WateringManualModeDiodePin = UNBINDED_PIN;
      diodesBinding.LightManualModeDiodePin = UNBINDED_PIN;
  } 
  #else
    // применяем настройки по умолчанию
    diodesBinding.LinkType = DIODES_LINK_TYPE;
    diodesBinding.MCPAddress = DIODES_MCP_ADDRESS;
    diodesBinding.Level = DIODES_ON;
    diodesBinding.ReadyDiodePin = READY_DIODE_PIN;
    diodesBinding.BlinkReadyDiode = BLINK_READY_DIODE;
    diodesBinding.WindowsManualModeDiodePin = WINDOWS_DIODE_PIN;
    diodesBinding.WateringManualModeDiodePin = WATERING_DIODE_PIN;
    diodesBinding.LightManualModeDiodePin = LIGHT_DIODE_PIN;
  #endif 

  // читаем настройки фрамуг
  memset(&windowsBinding,UNBINDED_PIN,sizeof(windowsBinding));
  if(!read(WINDOWS_BINDING_ADDRESS, windowsBinding))
  {
      memset(&windowsBinding,UNBINDED_PIN,sizeof(windowsBinding));

      windowsBinding.LinkType = linkUnbinded;
      windowsBinding.MCPAddress1 = 0;
      windowsBinding.MCPAddress2 = 0;
      windowsBinding.Level = 0;
      windowsBinding.ManageMode = 1; // попеременное управление по умолчанию
	  windowsBinding.AdditionalCloseTime = 0;
  }  

  // читаем настройки концевиков
  memset(&endstopsBinding,UNBINDED_PIN,sizeof(endstopsBinding));
  if(!read(ENDSTOPS_BINDING_ADDRESS, endstopsBinding))
  {
      memset(&endstopsBinding,UNBINDED_PIN,sizeof(endstopsBinding));

      endstopsBinding.LinkType = linkUnbinded;
      endstopsBinding.MCPAddress1 = 0;
      endstopsBinding.MCPAddress2 = 0;
      endstopsBinding.Level = 0;
  }  
  
  // читаем настройки датчиков влажности почвы
  memset(&soilBinding,UNBINDED_PIN,sizeof(soilBinding));
  if(!read(SOIL_BINDING_ADDRESS, soilBinding))
  {
      memset(&soilBinding,UNBINDED_PIN,sizeof(soilBinding));

      soilBinding.LinkType = linkUnbinded;
      soilBinding.MCPAddress = 0;
      soilBinding.PowerOnDelay = 200;
      soilBinding.Soil100Percent = 450;
      soilBinding.Soil0Percent = 1023;
      soilBinding.NoDataBorder = 100;
      soilBinding.Level = 0;

      for(size_t i=0;i<sizeof(soilBinding.SensorType)/sizeof(soilBinding.SensorType[0]);i++)
      {
        soilBinding.SensorType[i] = 0; // нет привязки
      }
      
  }  

  // читаем настройки датчиков влажности
  memset(&humidityBinding,0,sizeof(humidityBinding));
  if(!read(HUMIDITY_BINDING_ADDRESS, humidityBinding))
  {
      memset(&humidityBinding,0,sizeof(humidityBinding));

      humidityBinding.DHTReadDelay = 40;

      for(size_t i=0;i<sizeof(humidityBinding.Type)/sizeof(humidityBinding.Type[0]);i++)
      {
        humidityBinding.Type[i] = NO_HUMIDITY_SENSOR; // нет привязки
      }
      
  }  

  #ifndef DISABLE_RS485_CONFIGURE
  // читаем настройки RS485
  memset(&rs485Binding,0,sizeof(rs485Binding));
  if(!read(RS485_BINDING_ADDRESS, rs485Binding))
  {
      memset(&rs485Binding,0,sizeof(rs485Binding));

      rs485Binding.LinkType = linkUnbinded;
      rs485Binding.DEPin = UNBINDED_PIN;
      rs485Binding.StatePushFrequency = 1000;
      rs485Binding.OneSensorUpdateInterval = 1234;
      rs485Binding.BytesTimeout = 10;
      rs485Binding.ResetSensorBadReadings = 5;
      
  }
  #else
    // применяем настройки по умолчанию
    rs485Binding.LinkType = RS485_LINK_TYPE;
    rs485Binding.MCPAddress = RS485_MCP_ADDRESS;
    rs485Binding.DEPin = RS485_DE_PIN;
    rs485Binding.StatePushFrequency = RS485_PUSH_FREQUENCY;
    rs485Binding.OneSensorUpdateInterval = RS485_ONE_SENSOR_UPDATE_INTERVAL;
    rs485Binding.BytesTimeout = RS485_BYTES_TIMEOUT;
    rs485Binding.ResetSensorBadReadings = RS485_BAD_READINGS_RESET;
    rs485Binding.SerialNumber = RS485_SERIAL;
  #endif  

  // читаем настройки полива
  memset(&wateringBinding,0,sizeof(wateringBinding));
  if(!read(WATERING_BINDING_ADDRESS, wateringBinding))
  {
      memset(&wateringBinding,0,sizeof(wateringBinding));

      wateringBinding.ChannelsLinkType = linkUnbinded;
      wateringBinding.PumpLinkType = linkUnbinded;
      wateringBinding.Pump2LinkType = linkUnbinded;
      wateringBinding.PumpPin = UNBINDED_PIN;
      wateringBinding.Pump2Pin = UNBINDED_PIN;
      wateringBinding.Pump2StartChannel = 7;
      wateringBinding.SaveInterval = 2;
      for(size_t i=0;i<sizeof(wateringBinding.RelaysPins)/sizeof(wateringBinding.RelaysPins[0]);i++)
      {
        wateringBinding.RelaysPins[i] = UNBINDED_PIN;
      }      
  }  

  #ifndef DISABLE_GSM_CONFIGURE
    // читаем настройки SIM800
    memset(&gsmBinding,0,sizeof(gsmBinding));
    if(!read(GSM_BINDING_ADDRESS, gsmBinding))
    {
        memset(&gsmBinding,0,sizeof(gsmBinding));
  
        gsmBinding.AvailableCheckTime = 30000ul;
        gsmBinding.RebootTime = 2000ul;
        gsmBinding.WaitAfterRebootTime = 5000ul;
        gsmBinding.MaxAnswerTime = 60000ul;
        gsmBinding.RebootPinLinkType = linkUnbinded;
        gsmBinding.RebootPin = UNBINDED_PIN;
        gsmBinding.PowerOnLevel = HIGH;
        gsmBinding.PullGPRSByPing = 1;
        gsmBinding.PingInterval = 45000ul;
        gsmBinding.PowerkeyLinkType = linkUnbinded;
        gsmBinding.PowerkeyPin = UNBINDED_PIN;
        gsmBinding.PowerkeyPulseDuration = 300ul;
        gsmBinding.PowerkeyOnLevel = LOW;
        gsmBinding.WaitPowerkeyAfterPowerOn = 500ul;
        gsmBinding.IncomingDataTimeout = 10000ul;
        gsmBinding.CalmWatchdog = 1;
        gsmBinding.SendWindowStateInStatusSMS = 1;
        gsmBinding.SendWaterStateInStatusSMS = 1;
        
    } 
  #else
        // применяем настройки по умолчанию
        gsmBinding.SerialNumber = GSM_SERIAL;
        gsmBinding.AvailableCheckTime = 30000ul;
        gsmBinding.RebootTime = 2000ul;
        gsmBinding.WaitAfterRebootTime = 5000ul;
        gsmBinding.MaxAnswerTime = 60000ul;
        gsmBinding.RebootPinLinkType = GSM_REBOOT_PIN_LINK_TYPE;
        gsmBinding.RebootPinMCPAddress = GSM_REBOOT_PIN_MCP_ADDRESS;
        gsmBinding.RebootPin = GSM_REBOOT_PIN;
        gsmBinding.PowerOnLevel = GSM_POWER_ON;
        gsmBinding.PullGPRSByPing = GSM_PULL_GPRS;
        gsmBinding.PingInterval = 45000ul;
        gsmBinding.PowerkeyLinkType = GSM_POWERKEY_LINK_TYPE;
        gsmBinding.PowerkeyMCPAddress = GSM_POWERKEY_MCP_ADDRESS;
        gsmBinding.PowerkeyPin = GSM_POWERKEY_PIN;
        gsmBinding.PowerkeyPulseDuration = GSM_POWERKEY_PULSE_DURATION;
        gsmBinding.PowerkeyOnLevel = GSM_POWERKEY_ON_LEVEL;
        gsmBinding.WaitPowerkeyAfterPowerOn = 500ul;
        gsmBinding.SendWorkStartedSMS = 1;
        gsmBinding.IncomingDataTimeout = 10000ul;
        gsmBinding.CalmWatchdog = 1;
        gsmBinding.SendWindowStateInStatusSMS = 1;
        gsmBinding.SendWaterStateInStatusSMS = 1;
  #endif   

  #ifndef DISABLE_WIFI_CONFIGURE
    // читаем настройки ESP
    memset(&wifiBinding,0,sizeof(wifiBinding));
    if(!read(WIFI_BINDING_ADDRESS, wifiBinding))
    {
        memset(&wifiBinding,0,sizeof(wifiBinding));
  
        wifiBinding.AvailableCheckTime = 60000ul;
        wifiBinding.RebootTime = 2000ul;
        wifiBinding.WaitAfterRebootTime = 5000ul;
        wifiBinding.MaxAnswerTime = 60000ul;
        wifiBinding.RebootPinLinkType = linkUnbinded;
        wifiBinding.RebootPin = UNBINDED_PIN;
        wifiBinding.PowerOnLevel = LOW;
    }
  #else
    // применяем настройки Wi-Fi по умолчанию
      wifiBinding.SerialNumber = WIFI_SERIAL;
      wifiBinding.AvailableCheckTime = 60000ul;
      wifiBinding.RebootTime = 2000ul;
      wifiBinding.WaitAfterRebootTime = 5000ul;
      wifiBinding.MaxAnswerTime = 60000ul;
      wifiBinding.RebootPinLinkType = WIFI_REBOOT_PIN_LINK_TYPE;
      wifiBinding.RebootPinMCPAddress = WIFI_REBOOT_PIN_MCP_ADDRESS;
      wifiBinding.RebootPin = WIFI_REBOOT_PIN;
      wifiBinding.PowerOnLevel = WIFI_POWER_ON;
  #endif    

  #ifndef DISABLE_TFT_CONFIGURE
  // читаем настройки TFT
  memset(&tftBinding,0,sizeof(tftBinding));
  if(!read(TFT_BINDING_ADDRESS, tftBinding))
  {
      memset(&tftBinding,0,sizeof(tftBinding));

      tftBinding.BacklightLinkType = linkUnbinded;
      tftBinding.BacklightPin = UNBINDED_PIN;
      tftBinding.BacklightOnLevel = HIGH;      
      tftBinding.OffDelay = 300000ul;
      
  }
  #else
      // применяем настройки по умолчанию
      tftBinding.BacklightLinkType = TFT_BACKLIGHT_LINK_TYPE;
      tftBinding.BacklightMCPAddress = TFT_BACKLIGHT_MCP_ADDRESS;
      tftBinding.BacklightPin = TFT_BACKLIGHT_PIN;
      tftBinding.BacklightOnLevel = TFT_BACKLIGHT_ON;      
      tftBinding.InitDelay = TFT_INIT_DELAY;
      tftBinding.OffDelay = TFT_OFF_DELAY;
      tftBinding.DisplayType = TFT_DISPLAY_TYPE;
  #endif    

  // читаем настройки расходомеров
  memset(&flowBinding,UNBINDED_PIN,sizeof(flowBinding));
  if(!read(FLOW_BINDING_ADDRESS, flowBinding))
  {
      memset(&flowBinding,UNBINDED_PIN,sizeof(flowBinding));

      flowBinding.SaveDelta = 10;
      
  }   

  // читаем настройки датчиков на TFT-экране
  memset(tftSensorBinding,0,sizeof(tftSensorBinding));
  for(uint8_t i=0;i<12;i++)
  {
      int addr = TFT_SENSORS_BINDING_ADDRESS + i*(sizeof(TFTSensorBinding) + 3); // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, tftSensorBinding[i]))
      {
        memset(&(tftSensorBinding[i]),0,sizeof(TFTSensorBinding));        
      }
    
  } // for

   // читаем настройки управления питанием фрамуг
   memset(&wPowerBinding,0,sizeof(wPowerBinding));
   if(!read(WPOWER_BINDING_ADDRESS, wPowerBinding))
   {
      memset(&wPowerBinding,0,sizeof(wPowerBinding));
      
      wPowerBinding.LinkType = linkUnbinded;
      wPowerBinding.Pin = UNBINDED_PIN;
      wPowerBinding.Level = LOW;
      wPowerBinding.PowerOnDelay = 1000;
      wPowerBinding.PowerOffDelay = 1000;
   }


   // читаем настройки управления дверями
   memset(&doorBinding,0,sizeof(doorBinding));
   if(!read(DOOR_BINDING_ADDRESS, doorBinding))
   {
      memset(&doorBinding,0,sizeof(doorBinding));

      for(size_t i=0;i<sizeof(doorBinding)/sizeof(doorBinding[0]);i++)
      {
        doorBinding[i].LinkType = linkUnbinded;
        doorBinding[i].Pin1 = UNBINDED_PIN;
        doorBinding[i].Pin2 = UNBINDED_PIN;
      }
      
   }

   // читаем настройки интервалов открытия окон
    for(size_t i=0;i<sizeof(windowsIntervals.Interval)/sizeof(windowsIntervals.Interval[0]);i++)
    {
      windowsIntervals.Interval[i] = -1;   
    }

   if(!read(INTERVALS_BINDING_ADDRESS, windowsIntervals))
   {
      for(size_t i=0;i<sizeof(windowsIntervals.Interval)/sizeof(windowsIntervals.Interval[0]);i++)
      {
        windowsIntervals.Interval[i] = -1;   
      }   
   }

  // читаем настройки распрыскивания
  memset(sprayBinding,0,sizeof(sprayBinding));
  for(uint8_t i=0;i<3;i++)
  {
      int addr = SPRAY_BINDING_ADDRESS + i*(sizeof(HumiditySprayBinding) + 3);  // 2 байта - заголовок, 1 - CRC8
      if(!read(addr, sprayBinding[i]))
      {
         memset(&(sprayBinding[i]),0,sizeof(HumiditySprayBinding));
         
        sprayBinding[i].LinkType = linkUnbinded; // нет привязки
        sprayBinding[i].Pin = UNBINDED_PIN; // нет привязки
        sprayBinding[i].Level = LOW;        
      }
    
  } // for  


   #ifdef USE_WATER_TANK_MODULE
   // читаем настройки модуля бака
   memset(&waterTankBinding,0,sizeof(waterTankBinding));
   if(!read(WATER_TANK_BINDING_ADDRESS, waterTankBinding))
   {
      memset(&waterTankBinding,0,sizeof(waterTankBinding));
      
      waterTankBinding.LinkType = 1; // LoRa
      waterTankBinding.Level = LOW;
      waterTankBinding.MaxWorkTime = 600;
      waterTankBinding.DistanceEmpty = 100;
      waterTankBinding.DistanceFull = 10;
   }
   #endif // USE_WATER_TANK_MODULE  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void EEPROMSettingsModule::Update()
{ 
  // обновление модуля тут

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  EEPROMSettingsModule::ExecCommand(const Command& command, bool wantAnswer)
{
 if(wantAnswer) 
 {
    PublishSingleton = NOT_SUPPORTED;
 }

  uint8_t argsCnt = command.GetArgsCount();
    
  if(command.GetType() == ctSET) // установка свойств
  {
    if(argsCnt < 1)
     {
        if(wantAnswer) 
        {
          PublishSingleton = PARAMS_MISSED;
        }
     }
     else
     {
       String param = command.GetArg(0);
       if(false)
       {
        
       }
       #ifdef USE_DOOR_MODULE
       else
       if(param == F("DOOR")) // установить настройки привязки управления дверями: CTSET=EES|DOOR|link type 1|mcp address 1|pin1 1|pin2 1|level 1|drive 1|link type 2|mcp address 2|pin1 2|pin2 2|level 2|drive 2
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
            int cntr = 0;
            for(size_t z=0;z<sizeof(doorBinding)/sizeof(doorBinding[0]);z++)
            {
              if(cntr >= argsCnt)
                break;

              doorBinding[z].LinkType = atoi(command.GetArg(++cntr));
              doorBinding[z].MCPAddress = atoi(command.GetArg(++cntr));
              doorBinding[z].Pin1 = atoi(command.GetArg(++cntr));
              doorBinding[z].Pin2 = atoi(command.GetArg(++cntr));
              doorBinding[z].Level = atoi(command.GetArg(++cntr));
              doorBinding[z].DriveType = atoi(command.GetArg(++cntr));
            } // for

             write(DOOR_BINDING_ADDRESS, doorBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
       }
       #endif // USE_DOOR_MODULE
       #ifdef USE_WATER_TANK_MODULE
       else
       if(param == F("WTANK")) // установить настройки модуля контроля бака с водой: CTSET=EES|WTANK|link type|level|max work time|empty_cm|full_cm
       {
          if(argsCnt < 6)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             waterTankBinding.LinkType = atoi(command.GetArg(1));
             waterTankBinding.Level = atoi(command.GetArg(2));
             waterTankBinding.MaxWorkTime = atol(command.GetArg(3));
             waterTankBinding.DistanceEmpty = atol(command.GetArg(4));
             waterTankBinding.DistanceFull = atol(command.GetArg(5));

             write(WATER_TANK_BINDING_ADDRESS, waterTankBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
       } // F("WTANK")       
       #endif // USE_WATER_TANK_MODULE      
       #ifdef USE_CO2_MODULE
       else
       if(param == F("CO2")) // установить настройки CO2: CTSET=EES|CO2|link type|mcp address|relay pin|alert pin|level|vent pin|sensor pin|max ppm|min adc|max adc|alert percents|measure mode
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             co2Binding.LinkType = atoi(command.GetArg(1));
             co2Binding.MCPAddress = atoi(command.GetArg(2));
             co2Binding.RelayPin = atoi(command.GetArg(3));
             co2Binding.AlertPin = atoi(command.GetArg(4));
             co2Binding.Level = atoi(command.GetArg(5)); 
             co2Binding.VentPin = atoi(command.GetArg(6)); 
             co2Binding.SensorPin = atoi(command.GetArg(7)); 
             co2Binding.MaxPPM = atoi(command.GetArg(8)); 
             co2Binding.MinADCPPMValue = atoi(command.GetArg(9)); 
             co2Binding.MaxADCPPMValue = atoi(command.GetArg(10)); 
             co2Binding.AlertPercents = atoi(command.GetArg(11)); 
             co2Binding.MeasureMode = atoi(command.GetArg(12)); 

             write(CO2_BINDING_ADDRESS, co2Binding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
       } // F("CO2")
       #endif // #ifdef USE_CO2_MODULE
       #ifdef USE_WATERFLOW_MODULE
       else
       if(param == F("FLOW")) // установить настройки расходомеров: CTSET=EES|FLOW|pin1|pin2|save delta
       {
          if(argsCnt < 4)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             flowBinding.Flow1 = atoi(command.GetArg(1));
             flowBinding.Flow2 = atoi(command.GetArg(2));
             flowBinding.SaveDelta = atoi(command.GetArg(3));

             write(FLOW_BINDING_ADDRESS, flowBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
       } // F("FLOW")
       #endif // USE_WATERFLOW_MODULE
       #ifdef USE_PH_MODULE
       else
       if(param == F("PH")) // установить настройки pH: CTSET=EES|PH|link outputs|link inputs|mcp outputs|mcp inputs|sensor pin|mvPer7pH|reversive flag|flow level pin 1|flow level pin 2|flow add pin|plus pin|minus pin|mix pin|flow level|out level|vref
       {
          if(argsCnt < 17)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             phBinding.LinkTypeOutputs = atoi(command.GetArg(1));
             phBinding.LinkTypeInputs = atoi(command.GetArg(2));
             phBinding.MCPAddressOutputs = atoi(command.GetArg(3));
             phBinding.MCPAddressInputs = atoi(command.GetArg(4));
             phBinding.SensorPin = atoi(command.GetArg(5)); 
             phBinding.MVPer7Ph = atoi(command.GetArg(6)); 
             phBinding.ReversiveMeasure = atoi(command.GetArg(7)); 
             phBinding.FlowLevelSensorPin = atoi(command.GetArg(8)); 
             phBinding.FlowLevelSensorPin2 = atoi(command.GetArg(9)); 
             phBinding.FlowAddPin = atoi(command.GetArg(10)); 
             phBinding.PhPlusPin = atoi(command.GetArg(11)); 
             phBinding.PhMinusPin = atoi(command.GetArg(12)); 
             phBinding.PhMixPin = atoi(command.GetArg(13)); 
             phBinding.FlowSensorsLevel = atoi(command.GetArg(14)); 
             phBinding.OutputsLevel = atoi(command.GetArg(15)); 
             phBinding.VRef = atoi(command.GetArg(16)); 

             write(PH_BINDING_ADDRESS, phBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("PH")
       #endif // #ifdef USE_PH_MODULE
       #ifdef USE_EC_MODULE
       else
       if(param == F("ECPH")) // установить настройки EC/pH: CTSET=EES|ECPH|sensor pin|mvPer7pH|reversive flag|vref
       {
          if(argsCnt < 5)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             ecphBinding.SensorPin = atoi(command.GetArg(1));
             ecphBinding.MVPer7Ph = atoi(command.GetArg(2));
             ecphBinding.ReversiveMeasure = atoi(command.GetArg(3));
             ecphBinding.VRef = atoi(command.GetArg(4));
             
             write(PH_BINDING_ADDRESS, ecphBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("ECPH")
              else
       if(param == F("EC")) // установить настройки EC: CTSET=EES|EC|link type|mcp address|sensor1|sensor2|sensor3|sensor4|pinA|pinB|pinC|ph+|ph-|water|water valve|water contour valve|pump|max ppm|level
       {
          if(argsCnt < 18)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t _cntr = 1;
             ecBinding.LinkType = atoi(command.GetArg(_cntr++));
             ecBinding.MCPAddress = atoi(command.GetArg(_cntr++));
             ecBinding.SensorPins[0] = atoi(command.GetArg(_cntr++));
             ecBinding.SensorPins[1] = atoi(command.GetArg(_cntr++));
             ecBinding.SensorPins[2] = atoi(command.GetArg(_cntr++));
             ecBinding.SensorPins[3] = atoi(command.GetArg(_cntr++));
             ecBinding.PinA = atoi(command.GetArg(_cntr++));
             ecBinding.PinB = atoi(command.GetArg(_cntr++));
             ecBinding.PinC = atoi(command.GetArg(_cntr++));
             ecBinding.PinPhPlus = atoi(command.GetArg(_cntr++));
             ecBinding.PinPhMinus = atoi(command.GetArg(_cntr++));
             ecBinding.PinWater = atoi(command.GetArg(_cntr++));
             ecBinding.WaterValve = atoi(command.GetArg(_cntr++));
             ecBinding.WaterContour = atoi(command.GetArg(_cntr++));
             ecBinding.PumpPin = atoi(command.GetArg(_cntr++));
             ecBinding.MaxPpm = atoi(command.GetArg(_cntr++));
             ecBinding.Level = atoi(command.GetArg(_cntr++));
             
             write(EC_BINDING_ADDRESS, ecBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("EC")
       #endif // #ifdef USE_EC_MODULE
       #ifdef USE_THERMOSTAT_MODULE
       else
       if(param == F("THERMOSTAT")) // установить настройки термостатов: CTSET=EES|THERMOSTAT|link type 1|mcp address 1|pin 1|level 1|link type 2|mcp address 2|pin 2|level 2|link type 3|mcp address 3|pin 3|level 3
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             thermostatBinding[0].LinkType = atoi(command.GetArg(1));
             thermostatBinding[0].MCPAddress = atoi(command.GetArg(2));
             thermostatBinding[0].Pin = atoi(command.GetArg(3));
             thermostatBinding[0].Level = atoi(command.GetArg(4));
             
             thermostatBinding[1].LinkType = atoi(command.GetArg(5)); 
             thermostatBinding[1].MCPAddress = atoi(command.GetArg(6)); 
             thermostatBinding[1].Pin = atoi(command.GetArg(7)); 
             thermostatBinding[1].Level = atoi(command.GetArg(8)); 
             
             thermostatBinding[2].LinkType = atoi(command.GetArg(9)); 
             thermostatBinding[2].MCPAddress = atoi(command.GetArg(10)); 
             thermostatBinding[2].Pin = atoi(command.GetArg(11)); 
             thermostatBinding[2].Level = atoi(command.GetArg(12)); 


             for(uint8_t i=0;i<3;i++)
             {
                int addr = THERMOSTAT_BINDING_ADDRESS + i*(sizeof(ThermostatBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, thermostatBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("THERMOSTAT")
       #endif // #ifdef USE_THERMOSTAT_MODULE
       #ifdef USE_VENT_MODULE
       else
       if(param == F("VENT")) // установить настройки вентиляции: CTSET=EES|VENT|link type 1|mcp address 1|pin 1|level 1|link type 2|mcp address 2|pin 2|level 2|link type 3|mcp address 3|pin 3|level 3
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             ventBinding[0].LinkType = atoi(command.GetArg(1));
             ventBinding[0].MCPAddress = atoi(command.GetArg(2));
             ventBinding[0].Pin = atoi(command.GetArg(3));
             ventBinding[0].Level = atoi(command.GetArg(4));
             
             ventBinding[1].LinkType = atoi(command.GetArg(5)); 
             ventBinding[1].MCPAddress = atoi(command.GetArg(6)); 
             ventBinding[1].Pin = atoi(command.GetArg(7)); 
             ventBinding[1].Level = atoi(command.GetArg(8)); 
             
             ventBinding[2].LinkType = atoi(command.GetArg(9)); 
             ventBinding[2].MCPAddress = atoi(command.GetArg(10)); 
             ventBinding[2].Pin = atoi(command.GetArg(11)); 
             ventBinding[2].Level = atoi(command.GetArg(12)); 


             for(uint8_t i=0;i<3;i++)
             {
                int addr = VENT_BINDING_ADDRESS + i*(sizeof(VentBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, ventBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("VENT")
       #endif // #ifdef USE_VENT_MODULE
///////////////////////////////////////////////////
 #ifdef USE_HUMIDITY_SPRAY_MODULE
       else
       if(param == F("HSPRAY")) // установить настройки распрыскивания: CTSET=EES|HSPRAY|link type 1|mcp address 1|pin 1|level 1|link type 2|mcp address 2|pin 2|level 2|link type 3|mcp address 3|pin 3|level 3
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             sprayBinding[0].LinkType = atoi(command.GetArg(1));
             sprayBinding[0].MCPAddress = atoi(command.GetArg(2));
             sprayBinding[0].Pin = atoi(command.GetArg(3));
             sprayBinding[0].Level = atoi(command.GetArg(4));
             
             sprayBinding[1].LinkType = atoi(command.GetArg(5)); 
             sprayBinding[1].MCPAddress = atoi(command.GetArg(6)); 
             sprayBinding[1].Pin = atoi(command.GetArg(7)); 
             sprayBinding[1].Level = atoi(command.GetArg(8)); 
             
             sprayBinding[2].LinkType = atoi(command.GetArg(9)); 
             sprayBinding[2].MCPAddress = atoi(command.GetArg(10)); 
             sprayBinding[2].Pin = atoi(command.GetArg(11)); 
             sprayBinding[2].Level = atoi(command.GetArg(12)); 


             for(uint8_t i=0;i<3;i++)
             {
                int addr = SPRAY_BINDING_ADDRESS + i*(sizeof(HumiditySprayBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, sprayBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("HSPRAY")
       #endif // #ifdef USE_HUMIDITY_SPRAY_MODULE
///////////////////////////////////////////////////      
       #ifdef USE_CYCLE_VENT_MODULE
       else
       if(param == F("CVENT")) // установить настройки воздухообмена: CTSET=EES|CVENT|link type 1|mcp address 1|pin 1|level 1|link type 2|mcp address 2|pin 2|level 2|link type 3|mcp address 3|pin 3|level 3
       {
          if(argsCnt < 13)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             cycleVentBinding[0].LinkType = atoi(command.GetArg(1));
             cycleVentBinding[0].MCPAddress = atoi(command.GetArg(2));
             cycleVentBinding[0].Pin = atoi(command.GetArg(3));
             cycleVentBinding[0].Level = atoi(command.GetArg(4));
             
             cycleVentBinding[1].LinkType = atoi(command.GetArg(5)); 
             cycleVentBinding[1].MCPAddress = atoi(command.GetArg(6)); 
             cycleVentBinding[1].Pin = atoi(command.GetArg(7)); 
             cycleVentBinding[1].Level = atoi(command.GetArg(8)); 
             
             cycleVentBinding[2].LinkType = atoi(command.GetArg(9)); 
             cycleVentBinding[2].MCPAddress = atoi(command.GetArg(10)); 
             cycleVentBinding[2].Pin = atoi(command.GetArg(11)); 
             cycleVentBinding[2].Level = atoi(command.GetArg(12)); 


             for(uint8_t i=0;i<3;i++)
             {
                int addr = CYCLE_VENT_BINDING_ADDRESS + i*(sizeof(CycleVentBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, cycleVentBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("CVENT")
       #endif // #ifdef USE_CYCLE_VENT_MODULE
       #ifdef USE_SHADOW_MODULE
       else
       if(param == F("SHADOW")) // установить настройки затенения: CTSET=EES|SHADOW|link type 1|mcp address 1|pin 1-1|pin 1-2|level 1|link type 2|mcp address 2|pin 2-1|pin 2-2|level 2|link type 3|mcp address 3|pin 3-1|pin 3-2|level 3
       {
          if(argsCnt < 16)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             shadowBinding[0].LinkType = atoi(command.GetArg(1));
             shadowBinding[0].MCPAddress = atoi(command.GetArg(2));
             shadowBinding[0].Pin1 = atoi(command.GetArg(3));
             shadowBinding[0].Pin2 = atoi(command.GetArg(4));
             shadowBinding[0].Level = atoi(command.GetArg(5));
             
             shadowBinding[1].LinkType = atoi(command.GetArg(6));
             shadowBinding[1].MCPAddress = atoi(command.GetArg(7));
             shadowBinding[1].Pin1 = atoi(command.GetArg(8));
             shadowBinding[1].Pin2 = atoi(command.GetArg(9));
             shadowBinding[1].Level = atoi(command.GetArg(10));

             shadowBinding[2].LinkType = atoi(command.GetArg(11));
             shadowBinding[2].MCPAddress = atoi(command.GetArg(12));
             shadowBinding[2].Pin1 = atoi(command.GetArg(13));
             shadowBinding[2].Pin2 = atoi(command.GetArg(14));
             shadowBinding[2].Level = atoi(command.GetArg(15));
            

             for(uint8_t i=0;i<3;i++)
             {
                int addr = SHADOW_BINDING_ADDRESS + i*(sizeof(ShadowBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, shadowBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("SHADOW")
       #endif // #ifdef USE_SHADOW_MODULE
       #ifdef USE_HEAT_MODULE
       else
       if(param == F("HEAT")) // установить настройки отопления: CTSET=EES|HEAT|link type 1|mcp address 1|pin 1-1|pin 1-2|pump pin 1|level 1|link type 2|mcp address 2|pin 2-1|pin 2-2|pump pin 2|level 2|link type 3|mcp address 3|pin 3-1|pin 3-2|pump pin 3|level 3
       {
          if(argsCnt < 19)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             int argIter = 1;
             heatBinding[0].LinkType = atoi(command.GetArg(argIter++));
             heatBinding[0].MCPAddress = atoi(command.GetArg(argIter++));
             heatBinding[0].Pin1 = atoi(command.GetArg(argIter++));
             heatBinding[0].Pin2 = atoi(command.GetArg(argIter++));
             heatBinding[0].PumpPin = atoi(command.GetArg(argIter++));
             heatBinding[0].Level = atoi(command.GetArg(argIter++));
             
             heatBinding[1].LinkType = atoi(command.GetArg(argIter++));
             heatBinding[1].MCPAddress = atoi(command.GetArg(argIter++));
             heatBinding[1].Pin1 = atoi(command.GetArg(argIter++));
             heatBinding[1].Pin2 = atoi(command.GetArg(argIter++));
             heatBinding[1].PumpPin = atoi(command.GetArg(argIter++));
             heatBinding[1].Level = atoi(command.GetArg(argIter++));

             heatBinding[2].LinkType = atoi(command.GetArg(argIter++));
             heatBinding[2].MCPAddress = atoi(command.GetArg(argIter++));
             heatBinding[2].Pin1 = atoi(command.GetArg(argIter++));
             heatBinding[2].Pin2 = atoi(command.GetArg(argIter++));
             heatBinding[2].PumpPin = atoi(command.GetArg(argIter++));
             heatBinding[2].Level = atoi(command.GetArg(argIter++));
            

             for(uint8_t i=0;i<3;i++)
             {
                int addr = HEAT_BINDING_ADDRESS + i*(sizeof(HeatBinding) + 3); // 2 байта - заголовок, 1 - CRC8
                write(addr, heatBinding[i]);
             }
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("HEAT")
       #endif // #ifdef USE_HEAT_MODULE
       #ifdef USE_LUMINOSITY_MODULE
       else
       if(param == F("LIGHT")) // установить настройки досветки: CTSET=EES|LIGHT|link type|mcp address|sensor1|sensor2|sensor3|sensor4|relay1|relay2|relay3|relay4|relay5|relay6|relay7|relay8|level|avg en|avg samples|harboring en|harboring step|sensor index
       {
          if(argsCnt < 21)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             lightBinding.LinkType = atoi(command.GetArg(++cntr));
             lightBinding.MCPAddress = atoi(command.GetArg(++cntr));
             
             for(size_t i=0;i<sizeof(lightBinding.Sensors)/sizeof(lightBinding.Sensors[0]);i++)
             {
                lightBinding.Sensors[i] = atoi(command.GetArg(++cntr));
             }
             
             for(size_t i=0;i<sizeof(lightBinding.LampRelays)/sizeof(lightBinding.LampRelays[0]);i++)
             {
                lightBinding.LampRelays[i] = atoi(command.GetArg(++cntr));
             }

             lightBinding.Level = atoi(command.GetArg(++cntr));
             lightBinding.AveragingEnabled = atoi(command.GetArg(++cntr));
             lightBinding.AveragingSamples = atoi(command.GetArg(++cntr));
             lightBinding.HarboringEnabled = atoi(command.GetArg(++cntr));
             lightBinding.HarboringStep = atoi(command.GetArg(++cntr));
             lightBinding.SensorIndex = atoi(command.GetArg(++cntr));
            
              write(LIGHT_BINDING_ADDRESS, lightBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("LIGHT")
       #endif // #ifdef USE_LUMINOSITY_MODULE
       #ifdef USE_RAIN_SENSOR
       else
       if(param == F("RAIN")) // установить настройки датчика дождя: CTSET=EES|RAIN|link type|mcp address|pin|level|reset interval|work mode
       {
          if(argsCnt < 7)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             rainSensorBinding.LinkType = atoi(command.GetArg(++cntr));
             rainSensorBinding.MCPAddress = atoi(command.GetArg(++cntr));
             rainSensorBinding.Pin = atoi(command.GetArg(++cntr));
             rainSensorBinding.Level = atoi(command.GetArg(++cntr));
             rainSensorBinding.ResetInterval = atoi(command.GetArg(++cntr));
             rainSensorBinding.WorkMode = atoi(command.GetArg(++cntr));
            
              write(RAIN_BINDING_ADDRESS, rainSensorBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("RAIN")
       #endif // #ifdef USE_RAIN_SENSOR
       #ifdef USE_WIND_SENSOR
       else
       if(param == F("WIND")) // установить настройки датчика ветра: CTSET=EES|WIND|pin|dir pin|speed coeff|east from|east to|south from|south to|west from|west to|north from|north to|histeresis|num samples|work mode
       {
          if(argsCnt < 15)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             windSensorBinding.Pin = atoi(command.GetArg(++cntr));
             windSensorBinding.DirectionPin = atoi(command.GetArg(++cntr));
             windSensorBinding.SpeedCoeff = atoi(command.GetArg(++cntr));
             windSensorBinding.EastAdcFrom = atoi(command.GetArg(++cntr));
             windSensorBinding.EastAdcTo = atoi(command.GetArg(++cntr));
             windSensorBinding.SouthAdcFrom = atoi(command.GetArg(++cntr));
             windSensorBinding.SouthAdcTo = atoi(command.GetArg(++cntr));
             windSensorBinding.WestAdcFrom = atoi(command.GetArg(++cntr));
             windSensorBinding.WestAdcTo = atoi(command.GetArg(++cntr));
             windSensorBinding.NorthAdcFrom = atoi(command.GetArg(++cntr));
             windSensorBinding.NorthAdcTo = atoi(command.GetArg(++cntr));
             windSensorBinding.Histeresis = atoi(command.GetArg(++cntr));
             windSensorBinding.NumSamples = atoi(command.GetArg(++cntr));
             windSensorBinding.WorkMode = atoi(command.GetArg(++cntr));
            
              write(WIND_BINDING_ADDRESS, windSensorBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("WIND")
       #endif // #ifdef USE_WIND_SENSOR             
       #if defined(USE_BUZZER) && !defined(DISABLE_BUZZER_CONFIGURE)
       else
       if(param == F("BUZZER")) // установить настройки пищалки CTSET=EES|BUZZER|link type|mcp address|pin|level|duration
       {
          if(argsCnt < 6)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             buzzerBinding.LinkType = atoi(command.GetArg(++cntr));
             buzzerBinding.MCPAddress = atoi(command.GetArg(++cntr));
             buzzerBinding.Pin = atoi(command.GetArg(++cntr));
             buzzerBinding.Level = atoi(command.GetArg(++cntr));
             buzzerBinding.Duration = atoi(command.GetArg(++cntr));
             
            
              write(BUZZER_BINDING_ADDRESS, buzzerBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("BUZZER")
       #endif // #ifdef USE_BUZZER
       #if defined(USE_NRF_GATE) && !defined(DISABLE_NRF_CONFIGURE)
       else
       if(param == F("NRF")) // установить настройки nRF CTSET=EES|NRF|link type|mcp address|ce pin|csn pin|power pin|level|auto ack inverted flag
       {
          if(argsCnt < 8)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             nrfBinding.LinkType = atoi(command.GetArg(++cntr));
             nrfBinding.MCPAddress = atoi(command.GetArg(++cntr));
             nrfBinding.CEPin = atoi(command.GetArg(++cntr));
             nrfBinding.CSNPin = atoi(command.GetArg(++cntr));
             nrfBinding.PowerPin = atoi(command.GetArg(++cntr));
             nrfBinding.Level = atoi(command.GetArg(++cntr));
             nrfBinding.AutoAckInverted = atoi(command.GetArg(++cntr));             
            
              write(NRF_BINDING_ADDRESS, nrfBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("NRF")
       #endif // #ifdef USE_NRF_GATE
       #if defined(USE_LORA_GATE) && !defined(DISABLE_LORA_CONFIGURE)
       else
       if(param == F("LORA")) // установить настройки LORA CTSET=EES|LORA|link type|mcp address|ss pin|reset pin|power pin|level|tx power|freq
       {
          if(argsCnt < 9)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             loraBinding.LinkType = atoi(command.GetArg(++cntr));
             loraBinding.MCPAddress = atoi(command.GetArg(++cntr));
             loraBinding.SSPin = atoi(command.GetArg(++cntr));
             loraBinding.ResetPin = atoi(command.GetArg(++cntr));
             loraBinding.PowerPin = atoi(command.GetArg(++cntr));
             loraBinding.Level = atoi(command.GetArg(++cntr));
             loraBinding.TXPower = atoi(command.GetArg(++cntr));             
             loraBinding.Frequency = atoi(command.GetArg(++cntr));             
            
              write(LORA_BINDING_ADDRESS, loraBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("LORA")
       #endif // #ifdef USE_LORA_GATE            
       #ifdef USE_TIMER_MODULE
       else
       if(param == F("TIMER")) // установить настройки таймеров CTSET=EES|TIMER|link type|mcp address|level
       {
          if(argsCnt < 4)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             timerBinding.LinkType = atoi(command.GetArg(++cntr));
             timerBinding.MCPAddress = atoi(command.GetArg(++cntr));
             timerBinding.Level = atoi(command.GetArg(++cntr));
            
              write(TIMER_BINDING_ADDRESS, timerBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("TIMER")
       #endif // #ifdef USE_TIMER_MODULE
       else
       if(param == F("WPOWER")) // установить настройки управления питанием: CTSET=EES|WPOWER|linkType|mcp|pin|level|on delay|off delay|windows channels|watering channels|doors
       {
          if(argsCnt < 10)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
            uint8_t cntr = 0;
            wPowerBinding.LinkType = atoi(command.GetArg(++cntr));
            wPowerBinding.MCPAddress = atoi(command.GetArg(++cntr));
            wPowerBinding.Pin = atoi(command.GetArg(++cntr));
            wPowerBinding.Level = atoi(command.GetArg(++cntr));
            wPowerBinding.PowerOnDelay = atoi(command.GetArg(++cntr));
            wPowerBinding.PowerOffDelay = atoi(command.GetArg(++cntr));
            wPowerBinding.LinkedChannels = atoi(command.GetArg(++cntr));
            wPowerBinding.WateringChannels = atoi(command.GetArg(++cntr));
			wPowerBinding.DoorBinding = atoi(command.GetArg(++cntr));
            

            write(WPOWER_BINDING_ADDRESS, wPowerBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
        
       } // if(param == F("WPOWER"))
       #ifdef USE_DS18B20_EMULATION_MODULE
       else
       if(param == F("1WEMUL")) // установить настройки эмуляторов DS18B20: CTSET=EES|1WEMUL|pin1|..|pin20|type1|..|type20|index1|..|index20
       {
          if(argsCnt < 61)
          {
            PublishSingleton = PARAMS_MISSED;
          }
          else
          {
            // аргументов хватает
             uint8_t cntr = 0;

             for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Pin)/sizeof(ds18B20EmulationBinding.Pin[0]);i++)
             {
                ds18B20EmulationBinding.Pin[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Type)/sizeof(ds18B20EmulationBinding.Type[0]);i++)
             {
                ds18B20EmulationBinding.Type[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Index)/sizeof(ds18B20EmulationBinding.Index[0]);i++)
             {
                ds18B20EmulationBinding.Index[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(DS18B20_EMULATION_BINDING_ADDRESS, ds18B20EmulationBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
       } // if(param == F("1WEMUL"))
       #endif // USE_DS18B20_EMULATION_MODULE
       #ifdef USE_TEMP_SENSORS
       else
       if(param == F("DS18B20")) // установить настройки DS18B20 CTSET=EES|DS18B20|pin1|..|pin20
       {
          if(argsCnt < 21)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             for(size_t i=0;i<sizeof(dallasBinding.Pin)/sizeof(dallasBinding.Pin[0]);i++)
             {
                dallasBinding.Pin[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(DALLAS_BINDING_ADDRESS, dallasBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("DS18B20")
        else
       if(param == F("WINDOWS")) // установить настройки фрамуг CTSET=EES|WINDOWS|link type|mcp 1|mcp 2|level|manage mode|pin1|..|pin32
       {
          if(argsCnt < 42)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             windowsBinding.LinkType = atoi(command.GetArg(++cntr));
             windowsBinding.MCPAddress1 = atoi(command.GetArg(++cntr));
             windowsBinding.MCPAddress2 = atoi(command.GetArg(++cntr));
             windowsBinding.Level = atoi(command.GetArg(++cntr));
             windowsBinding.ManageMode = atoi(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(windowsBinding.WindowsPins)/sizeof(windowsBinding.WindowsPins[0]);i++)
             {
                windowsBinding.WindowsPins[i] =  atoi(command.GetArg(++cntr));
             }

             windowsBinding.ShiftOEPin = atoi(command.GetArg(++cntr));
             windowsBinding.ShiftLatchPin = atoi(command.GetArg(++cntr));
             windowsBinding.ShiftDataPin = atoi(command.GetArg(++cntr));
             windowsBinding.ShiftClockPin = atoi(command.GetArg(++cntr));
			       windowsBinding.AdditionalCloseTime = atoi(command.GetArg(++cntr));
            
              write(WINDOWS_BINDING_ADDRESS, windowsBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("WINDOWS")
       else
       if(param == F("WINTERVAL")) // установить настройки интервалов окон: CTSET=EES|WINTERVAL|int1|..|int16
       {
          if(argsCnt < 17)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             for(size_t i=0;i<sizeof(windowsIntervals.Interval)/sizeof(windowsIntervals.Interval[0]);i++)
             {
              windowsIntervals.Interval[i] = atoi(command.GetArg(++cntr));
             }
              write(INTERVALS_BINDING_ADDRESS, windowsIntervals);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("WINTERVAL")       
       #ifdef USE_WINDOWS_ENDSTOPS
        else
       if(param == F("ENDSTOPS")) // установить настройки концевиков CTSET=EES|ENDSTOPS|link type|mcp 1|mcp 2|level|open1|..|open16|close1|..|close16
       {
          if(argsCnt < 37)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             endstopsBinding.LinkType = atoi(command.GetArg(++cntr));
             endstopsBinding.MCPAddress1 = atoi(command.GetArg(++cntr));
             endstopsBinding.MCPAddress2 = atoi(command.GetArg(++cntr));
             endstopsBinding.Level = atoi(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(endstopsBinding.EndstopOpenPins)/sizeof(endstopsBinding.EndstopOpenPins[0]);i++)
             {
                endstopsBinding.EndstopOpenPins[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(endstopsBinding.EndstopClosePins)/sizeof(endstopsBinding.EndstopClosePins[0]);i++)
             {
                endstopsBinding.EndstopClosePins[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(ENDSTOPS_BINDING_ADDRESS, endstopsBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("ENDSTOPS")
       #endif // USE_WINDOWS_ENDSTOPS
       #endif // #ifdef USE_TEMP_SENSORS
       #if defined(USE_UNIVERSAL_MODULES) && defined(USE_UNI_REGISTRATION_LINE)
       else
       if(param == F("ONEWIRE")) // установить настройки 1-Wire CTSET=EES|ONEWIRE|reg line|pin1|..|pin5
       {
          if(argsCnt < 7)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             oneWireBinding.RegistrationLine = atoi(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(oneWireBinding.Pin)/sizeof(oneWireBinding.Pin[0]);i++)
             {
                oneWireBinding.Pin[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(ONEWIRE_BINDING_ADDRESS, oneWireBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("ONEWIRE")
       #endif // #if defined(USE_UNIVERSAL_MODULES) && defined(USE_UNI_REGISTRATION_LINE)
       #if defined(USE_INFO_DIODES) && !defined(DISABLE_DIODES_CONFIGURE)
       else
       if(param == F("DIODES")) // установить настройки информационных диодов CTSET=EES|DIODES|link type|mcp address|level|ready d|blink ready|windows d|watering d|light d
       {
          if(argsCnt < 9)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;
             diodesBinding.LinkType = atoi(command.GetArg(++cntr));
             diodesBinding.MCPAddress = atoi(command.GetArg(++cntr));
             diodesBinding.Level = atoi(command.GetArg(++cntr));
             diodesBinding.ReadyDiodePin = atoi(command.GetArg(++cntr));
             diodesBinding.BlinkReadyDiode = atoi(command.GetArg(++cntr));
             diodesBinding.WindowsManualModeDiodePin = atoi(command.GetArg(++cntr));
             diodesBinding.WateringManualModeDiodePin = atoi(command.GetArg(++cntr));             
             diodesBinding.LightManualModeDiodePin = atoi(command.GetArg(++cntr));             
            
              write(DIODES_BINDING_ADDRESS, diodesBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("DIODES")
       #endif // #ifdef USE_INFO_DIODES
       #ifdef USE_SOIL_MOISTURE_MODULE
       else
       if(param == F("SOIL")) // установить настройки датчиков влажности почвы, CTSET=EES|SOIL|link type|mcp address|power pin|level|power delay|soil100percent|soil0percent|noDataBorder|sensorType1|..|sensorType10|sensorPin1|..|sensorPin10
       {
          if(argsCnt < 29)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             soilBinding.LinkType = atoi(command.GetArg(++cntr));
             soilBinding.MCPAddress = atoi(command.GetArg(++cntr));
             soilBinding.PowerPin = atoi(command.GetArg(++cntr));
             soilBinding.Level = atoi(command.GetArg(++cntr));
             soilBinding.PowerOnDelay = atol(command.GetArg(++cntr));
             soilBinding.Soil100Percent = atol(command.GetArg(++cntr));
             soilBinding.Soil0Percent = atol(command.GetArg(++cntr));
             soilBinding.NoDataBorder = atol(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(soilBinding.SensorType)/sizeof(soilBinding.SensorType[0]);i++)
             {
                soilBinding.SensorType[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(soilBinding.SensorPin)/sizeof(soilBinding.SensorPin[0]);i++)
             {
                soilBinding.SensorPin[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(SOIL_BINDING_ADDRESS, soilBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("SOIL")
       #endif  // USE_SOIL_MOISTURE_MODULE
       #ifdef USE_HUMIDITY_MODULE
       else
       if(param == F("HUMIDITY")) // установить настройки датчиков влажности, CTSET=EES|HUMIDITY|strobe level|dhtReadDelay|pin1_1|..|pin1_10|pin2_1|..|pin2_10|type1|..|type10
       {
          if(argsCnt < 33)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             humidityBinding.StrobeLevel = atoi(command.GetArg(++cntr));
             humidityBinding.DHTReadDelay = atoi(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(humidityBinding.Pin1)/sizeof(humidityBinding.Pin1[0]);i++)
             {
                humidityBinding.Pin1[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(humidityBinding.Pin2)/sizeof(humidityBinding.Pin2[0]);i++)
             {
                humidityBinding.Pin2[i] =  atoi(command.GetArg(++cntr));
             }

             for(size_t i=0;i<sizeof(humidityBinding.Type)/sizeof(humidityBinding.Type[0]);i++)
             {
                humidityBinding.Type[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(HUMIDITY_BINDING_ADDRESS, humidityBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("HUMIDITY")
       #endif  // USE_HUMIDITY_MODULE
       #if defined(USE_RS485_GATE) && !defined(DISABLE_RS485_CONFIGURE)
       else
       if(param == F("RS485")) // установить настройки RS485, CTSET=EES|RS485|link type|mcp|de pin|state push freq|sensor upd interval|bytes timeout|reset readings|serial num
       {
          if(argsCnt < 9)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             rs485Binding.LinkType = atoi(command.GetArg(++cntr));
             rs485Binding.MCPAddress = atoi(command.GetArg(++cntr));
             rs485Binding.DEPin = atoi(command.GetArg(++cntr));
             rs485Binding.StatePushFrequency = atol(command.GetArg(++cntr));
             rs485Binding.OneSensorUpdateInterval = atol(command.GetArg(++cntr));
             rs485Binding.BytesTimeout = atoi(command.GetArg(++cntr));
             rs485Binding.ResetSensorBadReadings = atoi(command.GetArg(++cntr));
             rs485Binding.SerialNumber = atoi(command.GetArg(++cntr));
            
              write(RS485_BINDING_ADDRESS, rs485Binding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("RS485")
       #endif // USE_RS485_GATE
       #ifdef USE_WATERING_MODULE
        else
       if(param == F("WATER")) // установить настройки полива CTSET=EES|WATER|link|mcp|level|pump link|pump mcp|pump pin|pump2 link|pump2 mcp|pump2 pin|pump level|pump2 start channel|save interval|switch flag|relay1|..|relay16
       {
          if(argsCnt < 29)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             wateringBinding.ChannelsLinkType = atoi(command.GetArg(++cntr));
             wateringBinding.ChannelsMCPAddress = atoi(command.GetArg(++cntr));
             wateringBinding.Level = atoi(command.GetArg(++cntr));
             wateringBinding.PumpLinkType = atoi(command.GetArg(++cntr));
             wateringBinding.PumpMCPAddress = atoi(command.GetArg(++cntr));
             wateringBinding.PumpPin = atoi(command.GetArg(++cntr));
             wateringBinding.Pump2LinkType = atoi(command.GetArg(++cntr));
             wateringBinding.Pump2MCPAddress = atoi(command.GetArg(++cntr));
             wateringBinding.Pump2Pin = atoi(command.GetArg(++cntr));
             wateringBinding.PumpLevel = atoi(command.GetArg(++cntr));
             wateringBinding.Pump2StartChannel = atoi(command.GetArg(++cntr));
             wateringBinding.SaveInterval = atoi(command.GetArg(++cntr));

             for(size_t i=0;i<sizeof(wateringBinding.RelaysPins)/sizeof(wateringBinding.RelaysPins[0]);i++)
             {
                wateringBinding.RelaysPins[i] =  atoi(command.GetArg(++cntr));
             }
            
              write(WATERING_BINDING_ADDRESS, wateringBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("WATER") 
       #endif // USE_WATERING_MODULE
       #if defined(USE_SMS_MODULE) && !defined(DISABLE_GSM_CONFIGURE)
       else
       if(param == F("SMS")) // установить настройки SIM800 CTSET=EES|SMS|
       {
          if(argsCnt < 23)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             gsmBinding.SerialNumber = atoi(command.GetArg(++cntr));
             gsmBinding.AvailableCheckTime = atol(command.GetArg(++cntr));
             gsmBinding.RebootTime = atol(command.GetArg(++cntr));
             gsmBinding.WaitAfterRebootTime = atol(command.GetArg(++cntr));
             gsmBinding.MaxAnswerTime = atol(command.GetArg(++cntr));
             gsmBinding.RebootPinLinkType = atoi(command.GetArg(++cntr));
             gsmBinding.RebootPinMCPAddress = atoi(command.GetArg(++cntr));
             gsmBinding.RebootPin = atoi(command.GetArg(++cntr));
             gsmBinding.PowerOnLevel = atoi(command.GetArg(++cntr));
             gsmBinding.PullGPRSByPing = atoi(command.GetArg(++cntr));
             gsmBinding.PingInterval = atol(command.GetArg(++cntr));
             gsmBinding.PowerkeyLinkType = atoi(command.GetArg(++cntr));
             gsmBinding.PowerkeyMCPAddress = atoi(command.GetArg(++cntr));
             gsmBinding.PowerkeyPin = atoi(command.GetArg(++cntr));
             gsmBinding.PowerkeyPulseDuration = atol(command.GetArg(++cntr));
             gsmBinding.PowerkeyOnLevel = atoi(command.GetArg(++cntr));
             gsmBinding.WaitPowerkeyAfterPowerOn = atol(command.GetArg(++cntr));
             gsmBinding.SendWorkStartedSMS = atoi(command.GetArg(++cntr));
             gsmBinding.IncomingDataTimeout = atol(command.GetArg(++cntr));
             gsmBinding.CalmWatchdog = atoi(command.GetArg(++cntr));
             gsmBinding.SendWindowStateInStatusSMS = atoi(command.GetArg(++cntr));
             gsmBinding.SendWaterStateInStatusSMS = atoi(command.GetArg(++cntr));

            
              write(GSM_BINDING_ADDRESS, gsmBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("SMS")
       #endif // USE_SMS_MODULE
       #if defined(USE_WIFI_MODULE) && !defined(DISABLE_WIFI_CONFIGURE)
       else
       if(param == F("WIFI")) // установить настройки ESP CTSET=EES|WIFI|
       {
          if(argsCnt < 10)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             wifiBinding.SerialNumber = atoi(command.GetArg(++cntr));
             wifiBinding.AvailableCheckTime = atol(command.GetArg(++cntr));
             wifiBinding.RebootTime = atol(command.GetArg(++cntr));
             wifiBinding.WaitAfterRebootTime = atol(command.GetArg(++cntr));
             wifiBinding.MaxAnswerTime = atol(command.GetArg(++cntr));
             wifiBinding.RebootPinLinkType = atoi(command.GetArg(++cntr));
             wifiBinding.RebootPinMCPAddress = atoi(command.GetArg(++cntr));
             wifiBinding.RebootPin = atoi(command.GetArg(++cntr));
             wifiBinding.PowerOnLevel = atoi(command.GetArg(++cntr));
          
              write(WIFI_BINDING_ADDRESS, wifiBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("WIFI")
       #endif // USE_WIFI_MODULE
       #if defined(USE_TFT_MODULE) && !defined(DISABLE_TFT_CONFIGURE)
       else
       if(param == F("TFT")) // установить настройки TFT CTSET=EES|TFT|
       {
          if(argsCnt < 8)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             uint8_t cntr = 0;

             tftBinding.BacklightLinkType = atoi(command.GetArg(++cntr));
             tftBinding.BacklightMCPAddress = atoi(command.GetArg(++cntr));
             tftBinding.BacklightPin = atoi(command.GetArg(++cntr));
             tftBinding.BacklightOnLevel = atoi(command.GetArg(++cntr));
             tftBinding.InitDelay = atol(command.GetArg(++cntr));
             tftBinding.OffDelay = atol(command.GetArg(++cntr));
             tftBinding.DisplayType = atoi(command.GetArg(++cntr));
          
              write(TFT_BINDING_ADDRESS, tftBinding);

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }        
       } // F("TFT")
       #endif
       #if defined(USE_TFT_MODULE)
       else
       if(param == F("TFTS")) // установить настройки датчика на TFT-экране: CTSET=EES|TFTS|rec index|type|index|caption
       {
          if(argsCnt < 5)
          {
            if(wantAnswer)
            {
              PublishSingleton = PARAMS_MISSED;
            }
          }
          else
          {
             // аргументов хватает
             int recIdx = atoi(command.GetArg(1));
             if(recIdx < 0)
             {
              recIdx = 0;
             }
             if(recIdx > 11)
             {
              recIdx = 11;
             }
             
             tftSensorBinding[recIdx].Type = atoi(command.GetArg(2));
             tftSensorBinding[recIdx].Index = atoi(command.GetArg(3));

              const char* encodedCaption = command.GetArg(4);
              uint8_t captCntr = 0;
                  
              // переводим закодированный текст в UTF-8
              while(*encodedCaption && captCntr < (sizeof(tftSensorBinding[recIdx].Caption) - 1) )
              {
                tftSensorBinding[recIdx].Caption[captCntr++] = (char) WorkStatus::FromHex(encodedCaption);
                encodedCaption += 2;
              }

              tftSensorBinding[recIdx].Caption[captCntr] = '\0';

              int addr = TFT_SENSORS_BINDING_ADDRESS + recIdx*(sizeof(TFTSensorBinding) + 3); // 2 байта - заголовок, 1 - CRC8
              write(addr, tftSensorBinding[recIdx]);
                         

             PublishSingleton.Flags.Status = true;
             PublishSingleton = param;
             PublishSingleton << PARAM_DELIMITER << recIdx << PARAM_DELIMITER << REG_SUCC;
          }        
       } // "TFTS"
       #endif // USE_TFT_MODULE
     } // else argsCount >= 1
     
    
  } // ctSET    
  else
  if(command.GetType() == ctGET) // запрос свойств
  {
      
      if(argsCnt < 1)
      {
        if(wantAnswer) 
          PublishSingleton = PARAMS_MISSED; // не хватает параметров
        
      } // argsCnt < 1 
      else
      {     
        String param = command.GetArg(0);
        if(param == F("UNSAFE")) // запросили настройки небезопасных пинов, CTGET=EES|UNSAFE
        {
         PublishSingleton.Flags.Status = true;
         PublishSingleton = param; 

         for(size_t i=0;i<sizeof(UNSAFE_PINS)/sizeof(UNSAFE_PINS[0]);i++)
         {
          PublishSingleton << PARAM_DELIMITER << UNSAFE_PINS[i];
         }
         
        } // F("UNSAFE")
        #ifdef USE_DOOR_MODULE
        else
        if(param == F("DOOR")) // запросили настройки управления дверьми: CTGET=EES|DOOR
        {

          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
           PublishSingleton << PARAM_DELIMITER << doorBinding[0].LinkType
          << PARAM_DELIMITER << doorBinding[0].MCPAddress
          << PARAM_DELIMITER << doorBinding[0].Pin1
          << PARAM_DELIMITER << doorBinding[0].Pin2
          << PARAM_DELIMITER << doorBinding[0].Level
          << PARAM_DELIMITER << doorBinding[0].DriveType
          
          << PARAM_DELIMITER << doorBinding[1].LinkType
          << PARAM_DELIMITER << doorBinding[1].MCPAddress
          << PARAM_DELIMITER << doorBinding[1].Pin1
          << PARAM_DELIMITER << doorBinding[1].Pin2
          << PARAM_DELIMITER << doorBinding[1].Level
          << PARAM_DELIMITER << doorBinding[1].DriveType
          ;
          
        }
        #endif // USE_DOOR_MODULE
        #ifdef USE_WATER_TANK_MODULE
        else
        if(param == F("WTANK")) // запросили настройки модуля управления баком: CTGET=EES|WTANK
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << waterTankBinding.LinkType
          << PARAM_DELIMITER << waterTankBinding.Level
          << PARAM_DELIMITER << waterTankBinding.MaxWorkTime
          << PARAM_DELIMITER << waterTankBinding.DistanceEmpty
          << PARAM_DELIMITER << waterTankBinding.DistanceFull
          ;
          
        } // param == F("WTANK")        
        #endif // USE_WATER_TANK_MODULE
        #ifdef USE_CO2_MODULE
        else
        if(param == F("CO2")) // запросили настройки CO2: CTGET=EES|CO2
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << co2Binding.LinkType
          << PARAM_DELIMITER << co2Binding.MCPAddress
          << PARAM_DELIMITER << co2Binding.RelayPin
          << PARAM_DELIMITER << co2Binding.AlertPin
          << PARAM_DELIMITER << co2Binding.Level
          << PARAM_DELIMITER << co2Binding.VentPin
          << PARAM_DELIMITER << co2Binding.SensorPin
          << PARAM_DELIMITER << co2Binding.MaxPPM
          << PARAM_DELIMITER << co2Binding.MinADCPPMValue
          << PARAM_DELIMITER << co2Binding.MaxADCPPMValue
          << PARAM_DELIMITER << co2Binding.AlertPercents
          << PARAM_DELIMITER << co2Binding.MeasureMode
          ;
          
        } // param == F("CO2")
        #endif // #ifdef USE_CO2_MODULE
        #ifdef USE_WATERFLOW_MODULE
        else
        if(param == F("FLOW")) // запросили настройки расходомеров: CTGET=EES|FLOW
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << flowBinding.Flow1
          << PARAM_DELIMITER << flowBinding.Flow2
          << PARAM_DELIMITER << flowBinding.SaveDelta        
          ;
          
        } // param == F("FLOW")
        #endif // #ifdef USE_WATERFLOW_MODULE
        #ifdef USE_PH_MODULE
        else
        if(param == F("PH")) // запросили настройки PH: CTGET=EES|PH
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << phBinding.LinkTypeOutputs
          << PARAM_DELIMITER << phBinding.LinkTypeInputs
          << PARAM_DELIMITER << phBinding.MCPAddressOutputs
          << PARAM_DELIMITER << phBinding.MCPAddressInputs
          << PARAM_DELIMITER << phBinding.SensorPin
          << PARAM_DELIMITER << phBinding.MVPer7Ph
          << PARAM_DELIMITER << phBinding.ReversiveMeasure
          << PARAM_DELIMITER << phBinding.FlowLevelSensorPin
          << PARAM_DELIMITER << phBinding.FlowLevelSensorPin2
          << PARAM_DELIMITER << phBinding.FlowAddPin
          << PARAM_DELIMITER << phBinding.PhPlusPin
          << PARAM_DELIMITER << phBinding.PhMinusPin
          << PARAM_DELIMITER << phBinding.PhMixPin
          << PARAM_DELIMITER << phBinding.FlowSensorsLevel
          << PARAM_DELIMITER << phBinding.OutputsLevel
          << PARAM_DELIMITER << phBinding.VRef          
          ;                  
        } // F("PH")
        #endif // #ifdef USE_PH_MODULE
        #ifdef USE_EC_MODULE
        else
        if(param == F("ECPH")) // запросили настройки EC/PH: CTGET=EES|ECPH
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << ecphBinding.SensorPin
          << PARAM_DELIMITER << ecphBinding.MVPer7Ph
          << PARAM_DELIMITER << ecphBinding.ReversiveMeasure
          << PARAM_DELIMITER << ecphBinding.VRef
          ;                  
        } // F("ECPH")
        else
        if(param == F("EC")) // запросили настройки EC: CTGET=EES|EC
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << ecBinding.LinkType
          << PARAM_DELIMITER << ecBinding.MCPAddress
          << PARAM_DELIMITER << ecBinding.SensorPins[0]
          << PARAM_DELIMITER << ecBinding.SensorPins[1]
          << PARAM_DELIMITER << ecBinding.SensorPins[2]
          << PARAM_DELIMITER << ecBinding.SensorPins[3]
          << PARAM_DELIMITER << ecBinding.PinA
          << PARAM_DELIMITER << ecBinding.PinB
          << PARAM_DELIMITER << ecBinding.PinC
          << PARAM_DELIMITER << ecBinding.PinPhPlus
          << PARAM_DELIMITER << ecBinding.PinPhMinus
          << PARAM_DELIMITER << ecBinding.PinWater
          << PARAM_DELIMITER << ecBinding.WaterValve
          << PARAM_DELIMITER << ecBinding.WaterContour
          << PARAM_DELIMITER << ecBinding.PumpPin
          << PARAM_DELIMITER << ecBinding.MaxPpm
          << PARAM_DELIMITER << ecBinding.Level
          ;                  
        } // F("EC")
        #endif // #ifdef USE_EC_MODULE
        #ifdef USE_THERMOSTAT_MODULE
        else
        if(param == F("THERMOSTAT")) // запросили настройки термостатов: CTGET=EES|THERMOSTAT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << thermostatBinding[0].LinkType
          << PARAM_DELIMITER << thermostatBinding[0].MCPAddress
          << PARAM_DELIMITER << thermostatBinding[0].Pin
          << PARAM_DELIMITER << thermostatBinding[0].Level
          
          << PARAM_DELIMITER << thermostatBinding[1].LinkType
          << PARAM_DELIMITER << thermostatBinding[1].MCPAddress
          << PARAM_DELIMITER << thermostatBinding[1].Pin
          << PARAM_DELIMITER << thermostatBinding[1].Level
          
          << PARAM_DELIMITER << thermostatBinding[2].LinkType
          << PARAM_DELIMITER << thermostatBinding[2].MCPAddress
          << PARAM_DELIMITER << thermostatBinding[2].Pin
          << PARAM_DELIMITER << thermostatBinding[2].Level
          ;                  
        } // F("THERMOSTAT")
        #endif // #ifdef USE_THERMOSTAT_MODULE
        #ifdef USE_VENT_MODULE
        else
        if(param == F("VENT")) // запросили настройки вентиляции: CTGET=EES|VENT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << ventBinding[0].LinkType
          << PARAM_DELIMITER << ventBinding[0].MCPAddress
          << PARAM_DELIMITER << ventBinding[0].Pin
          << PARAM_DELIMITER << ventBinding[0].Level
          
          << PARAM_DELIMITER << ventBinding[1].LinkType
          << PARAM_DELIMITER << ventBinding[1].MCPAddress
          << PARAM_DELIMITER << ventBinding[1].Pin
          << PARAM_DELIMITER << ventBinding[1].Level
          
          << PARAM_DELIMITER << ventBinding[2].LinkType
          << PARAM_DELIMITER << ventBinding[2].MCPAddress
          << PARAM_DELIMITER << ventBinding[2].Pin
          << PARAM_DELIMITER << ventBinding[2].Level
          ;                  
        } // F("VENT")
        #endif // #ifdef USE_VENT_MODULE
///////////////////////////////////////////////////////////////
        #ifdef USE_HUMIDITY_SPRAY_MODULE
        else
        if(param == F("HSPRAY")) // запросили настройки опрыскивания: CTGET=EES|HSPRAY
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << sprayBinding[0].LinkType
          << PARAM_DELIMITER << sprayBinding[0].MCPAddress
          << PARAM_DELIMITER << sprayBinding[0].Pin
          << PARAM_DELIMITER << sprayBinding[0].Level
          
          << PARAM_DELIMITER << sprayBinding[1].LinkType
          << PARAM_DELIMITER << sprayBinding[1].MCPAddress
          << PARAM_DELIMITER << sprayBinding[1].Pin
          << PARAM_DELIMITER << sprayBinding[1].Level
          
          << PARAM_DELIMITER << sprayBinding[2].LinkType
          << PARAM_DELIMITER << sprayBinding[2].MCPAddress
          << PARAM_DELIMITER << sprayBinding[2].Pin
          << PARAM_DELIMITER << sprayBinding[2].Level
          ;                  
        } // F("HSPRAY")
        #endif // #ifdef USE_HUMIDITY_SPRAY_MODULE
//////////////////////////////////////////////////////////////
        #ifdef USE_CYCLE_VENT_MODULE
        else
        if(param == F("CVENT")) // запросили настройки воздухообмена: CTGET=EES|CVENT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << cycleVentBinding[0].LinkType
          << PARAM_DELIMITER << cycleVentBinding[0].MCPAddress
          << PARAM_DELIMITER << cycleVentBinding[0].Pin
          << PARAM_DELIMITER << cycleVentBinding[0].Level
          
          << PARAM_DELIMITER << cycleVentBinding[1].LinkType
          << PARAM_DELIMITER << cycleVentBinding[1].MCPAddress
          << PARAM_DELIMITER << cycleVentBinding[1].Pin
          << PARAM_DELIMITER << cycleVentBinding[1].Level
          
          << PARAM_DELIMITER << cycleVentBinding[2].LinkType
          << PARAM_DELIMITER << cycleVentBinding[2].MCPAddress
          << PARAM_DELIMITER << cycleVentBinding[2].Pin
          << PARAM_DELIMITER << cycleVentBinding[2].Level
          ;                  
        } // F("CVENT")
        #endif // #ifdef USE_CYCLE_VENT_MODULE
        #ifdef USE_SHADOW_MODULE
        else
        if(param == F("SHADOW")) // запросили настройки затенения: CTGET=EES|SHADOW
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << shadowBinding[0].LinkType
          << PARAM_DELIMITER << shadowBinding[0].MCPAddress
          << PARAM_DELIMITER << shadowBinding[0].Pin1
          << PARAM_DELIMITER << shadowBinding[0].Pin2
          << PARAM_DELIMITER << shadowBinding[0].Level
          
          << PARAM_DELIMITER << shadowBinding[1].LinkType
          << PARAM_DELIMITER << shadowBinding[1].MCPAddress
          << PARAM_DELIMITER << shadowBinding[1].Pin1
          << PARAM_DELIMITER << shadowBinding[1].Pin2
          << PARAM_DELIMITER << shadowBinding[1].Level
          
          << PARAM_DELIMITER << shadowBinding[2].LinkType
          << PARAM_DELIMITER << shadowBinding[2].MCPAddress
          << PARAM_DELIMITER << shadowBinding[2].Pin1
          << PARAM_DELIMITER << shadowBinding[2].Pin2
          << PARAM_DELIMITER << shadowBinding[2].Level
          ;                  
        } // F("SHADOW")
        #endif // #ifdef USE_SHADOW_MODULE
        #ifdef USE_HEAT_MODULE
        else
        if(param == F("HEAT")) // запросили настройки отопления: CTGET=EES|HEAT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << heatBinding[0].LinkType
          << PARAM_DELIMITER << heatBinding[0].MCPAddress
          << PARAM_DELIMITER << heatBinding[0].Pin1
          << PARAM_DELIMITER << heatBinding[0].Pin2
          << PARAM_DELIMITER << heatBinding[0].PumpPin
          << PARAM_DELIMITER << heatBinding[0].Level
          
          << PARAM_DELIMITER << heatBinding[1].LinkType
          << PARAM_DELIMITER << heatBinding[1].MCPAddress
          << PARAM_DELIMITER << heatBinding[1].Pin1
          << PARAM_DELIMITER << heatBinding[1].Pin2
          << PARAM_DELIMITER << heatBinding[1].PumpPin
          << PARAM_DELIMITER << heatBinding[1].Level
          
          << PARAM_DELIMITER << heatBinding[2].LinkType
          << PARAM_DELIMITER << heatBinding[2].MCPAddress
          << PARAM_DELIMITER << heatBinding[2].Pin1
          << PARAM_DELIMITER << heatBinding[2].Pin2
          << PARAM_DELIMITER << heatBinding[2].PumpPin
          << PARAM_DELIMITER << heatBinding[2].Level
          ;                  
        } // F("HEAT")
        #endif // #ifdef USE_HEAT_MODULE
        #ifdef USE_LUMINOSITY_MODULE
        else
        if(param == F("LIGHT")) // запросили настройки досветки: CTGET=EES|LIGHT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << lightBinding.LinkType
          << PARAM_DELIMITER << lightBinding.MCPAddress;

          for(size_t i=0;i<sizeof(lightBinding.Sensors)/sizeof(lightBinding.Sensors[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << lightBinding.Sensors[i];
          }

          for(size_t i=0;i<sizeof(lightBinding.LampRelays)/sizeof(lightBinding.LampRelays[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << lightBinding.LampRelays[i];
          }
          
          PublishSingleton << PARAM_DELIMITER << lightBinding.Level
          << PARAM_DELIMITER << lightBinding.AveragingEnabled
          << PARAM_DELIMITER << lightBinding.AveragingSamples
          << PARAM_DELIMITER << lightBinding.HarboringEnabled
          << PARAM_DELIMITER << lightBinding.HarboringStep
          << PARAM_DELIMITER << lightBinding.SensorIndex
          ;                  
        } // F("LIGHT")
        #endif // #ifdef USE_LUMINOSITY_MODULE
        #ifdef USE_RAIN_SENSOR
        else
        if(param == F("RAIN")) // запросили настройки датчика дождя: CTGET=EES|RAIN
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << rainSensorBinding.LinkType
          << PARAM_DELIMITER << rainSensorBinding.MCPAddress
          << PARAM_DELIMITER << rainSensorBinding.Pin
          << PARAM_DELIMITER << rainSensorBinding.Level
          << PARAM_DELIMITER << rainSensorBinding.ResetInterval
          << PARAM_DELIMITER << rainSensorBinding.WorkMode
          ;                  
        } // F("RAIN")
        #endif // #ifdef USE_RAIN_SENSOR
        #ifdef USE_WIND_SENSOR
        else
        if(param == F("WIND")) // запросили настройки датчика ветра: CTGET=EES|WIND
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << windSensorBinding.Pin
          << PARAM_DELIMITER << windSensorBinding.DirectionPin
          << PARAM_DELIMITER << windSensorBinding.SpeedCoeff
          << PARAM_DELIMITER << windSensorBinding.EastAdcFrom
          << PARAM_DELIMITER << windSensorBinding.EastAdcTo
          << PARAM_DELIMITER << windSensorBinding.SouthAdcFrom
          << PARAM_DELIMITER << windSensorBinding.SouthAdcTo
          << PARAM_DELIMITER << windSensorBinding.WestAdcFrom
          << PARAM_DELIMITER << windSensorBinding.WestAdcTo
          << PARAM_DELIMITER << windSensorBinding.NorthAdcFrom
          << PARAM_DELIMITER << windSensorBinding.NorthAdcTo
          << PARAM_DELIMITER << windSensorBinding.Histeresis
          << PARAM_DELIMITER << windSensorBinding.NumSamples
          << PARAM_DELIMITER << windSensorBinding.WorkMode
          ;                  
        } // F("WIND")
        #endif // #ifdef USE_WIND_SENSOR
        #if defined(USE_BUZZER) && !defined(DISABLE_BUZZER_CONFIGURE)
        else
        if(param == F("BUZZER")) // запросили настройки пищалки: CTGET=EES|BUZZER
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << buzzerBinding.LinkType
          << PARAM_DELIMITER << buzzerBinding.MCPAddress
          << PARAM_DELIMITER << buzzerBinding.Pin
          << PARAM_DELIMITER << buzzerBinding.Level
          << PARAM_DELIMITER << buzzerBinding.Duration
          ;                  
        } // F("BUZZER")
        #endif // #ifdef USE_BUZZER
        #if defined(USE_NRF_GATE)  && !defined(DISABLE_NRF_CONFIGURE)
        else
        if(param == F("NRF")) // запросили настройки nRF: CTGET=EES|NRF
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << nrfBinding.LinkType
          << PARAM_DELIMITER << nrfBinding.MCPAddress
          << PARAM_DELIMITER << nrfBinding.CEPin
          << PARAM_DELIMITER << nrfBinding.CSNPin
          << PARAM_DELIMITER << nrfBinding.PowerPin
          << PARAM_DELIMITER << nrfBinding.Level
          << PARAM_DELIMITER << nrfBinding.AutoAckInverted
          
          ;                  
        } // F("NRF")
        #endif // #ifdef USE_NRF_GATE
        #if defined(USE_LORA_GATE)  && !defined(DISABLE_LORA_CONFIGURE)
        else
        if(param == F("LORA")) // запросили настройки LORA: CTGET=EES|LORA
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << loraBinding.LinkType
          << PARAM_DELIMITER << loraBinding.MCPAddress
          << PARAM_DELIMITER << loraBinding.SSPin
          << PARAM_DELIMITER << loraBinding.ResetPin
          << PARAM_DELIMITER << loraBinding.PowerPin
          << PARAM_DELIMITER << loraBinding.Level
          << PARAM_DELIMITER << loraBinding.TXPower
          << PARAM_DELIMITER << loraBinding.Frequency
          
          ;                  
        } // F("LORA")
        #endif // #ifdef USE_LORA_GATE
        #ifdef USE_TIMER_MODULE
        else
        if(param == F("TIMER")) // запросили настройки таймеров: CTGET=EES|TIMER
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << timerBinding.LinkType
          << PARAM_DELIMITER << timerBinding.MCPAddress
          << PARAM_DELIMITER << timerBinding.Level
          
          ;                  
        } // F("TIMER")
        #endif // #ifdef USE_TIMER_MODULE
        else
        if(param == F("WPOWER")) // запросили настройки управления питанием, CTGET=EES|WPOWER
        {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = param;
            PublishSingleton << PARAM_DELIMITER << wPowerBinding.LinkType
            << PARAM_DELIMITER << wPowerBinding.MCPAddress
            << PARAM_DELIMITER << wPowerBinding.Pin
            << PARAM_DELIMITER << wPowerBinding.Level
            << PARAM_DELIMITER << wPowerBinding.PowerOnDelay
            << PARAM_DELIMITER << wPowerBinding.PowerOffDelay
            << PARAM_DELIMITER << wPowerBinding.LinkedChannels
            << PARAM_DELIMITER << wPowerBinding.WateringChannels
			<< PARAM_DELIMITER << wPowerBinding.DoorBinding
            ;
          
        } // if(param == F("WPOWER"))
        #ifdef USE_DS18B20_EMULATION_MODULE
        else
        if(param == F("1WEMUL")) // запросили настройки эмуляторов DS18B20: CTGET=EES|1WEMUL
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Pin)/sizeof(ds18B20EmulationBinding.Pin[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << ds18B20EmulationBinding.Pin[i];
          }

          for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Type)/sizeof(ds18B20EmulationBinding.Type[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << ds18B20EmulationBinding.Type[i];
          }

          for(size_t i=0;i<sizeof(ds18B20EmulationBinding.Index)/sizeof(ds18B20EmulationBinding.Index[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << ds18B20EmulationBinding.Index[i];
          }
          
        } // if(param == F("1WEMUL"))
        #endif // USE_DS18B20_EMULATION_MODULE        
        #ifdef USE_TEMP_SENSORS
        else
        if(param == F("DS18B20")) // запросили настройки DS18B20: CTGET=EES|DS18B20
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          for(size_t i=0;i<sizeof(dallasBinding.Pin)/sizeof(dallasBinding.Pin[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << dallasBinding.Pin[i];
          }
        } // F("DS18B20")
        else
        if(param == F("WINDOWS")) // запросили настройки фрамуг: CTGET=EES|WINDOWS
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << windowsBinding.LinkType
          << PARAM_DELIMITER << windowsBinding.MCPAddress1
          << PARAM_DELIMITER << windowsBinding.MCPAddress2
          << PARAM_DELIMITER << windowsBinding.Level
          << PARAM_DELIMITER << windowsBinding.ManageMode
          ;

          for(size_t i=0;i<sizeof(windowsBinding.WindowsPins)/sizeof(windowsBinding.WindowsPins[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << windowsBinding.WindowsPins[i];
          }

          PublishSingleton << PARAM_DELIMITER << windowsBinding.ShiftOEPin
          << PARAM_DELIMITER << windowsBinding.ShiftLatchPin
          << PARAM_DELIMITER << windowsBinding.ShiftDataPin
          << PARAM_DELIMITER << windowsBinding.ShiftClockPin
          << PARAM_DELIMITER << windowsBinding.AdditionalCloseTime
		  ;
          
        } // F("WINDOWS")
        else
        if(param == F("WINTERVAL")) // запросили настройки интервалов открытия окон: CTGET=EES|WINTERVAL
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          for(size_t i=0;i<sizeof(windowsIntervals.Interval)/sizeof(windowsIntervals.Interval[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << windowsIntervals.Interval[i];
          }
        } // F("DS18B20")
        #ifdef USE_WINDOWS_ENDSTOPS
         else
        if(param == F("ENDSTOPS")) // запросили настройки концевиков: CTGET=EES|ENDSTOPS
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << endstopsBinding.LinkType
          << PARAM_DELIMITER << endstopsBinding.MCPAddress1
          << PARAM_DELIMITER << endstopsBinding.MCPAddress2
          << PARAM_DELIMITER << endstopsBinding.Level
          ;

          for(size_t i=0;i<sizeof(endstopsBinding.EndstopOpenPins)/sizeof(endstopsBinding.EndstopOpenPins[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << endstopsBinding.EndstopOpenPins[i];
          }

          for(size_t i=0;i<sizeof(endstopsBinding.EndstopClosePins)/sizeof(endstopsBinding.EndstopClosePins[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << endstopsBinding.EndstopClosePins[i];
          }
          
        } // F("ENDSTOPS")
        #endif // USE_WINDOWS_ENDSTOPS
        #endif // #ifdef USE_TEMP_SENSORS
        #if defined(USE_UNIVERSAL_MODULES) && defined(USE_UNI_REGISTRATION_LINE)
        else
        if(param == F("ONEWIRE")) // запросили настройки 1-Wire: CTGET=EES|ONEWIRE
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << oneWireBinding.RegistrationLine;

          for(size_t i=0;i<sizeof(oneWireBinding.Pin)/sizeof(oneWireBinding.Pin[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << oneWireBinding.Pin[i];
          }
        } // F("ONEWIRE")
        #endif // #if defined(USE_UNIVERSAL_MODULES) && defined(USE_UNI_REGISTRATION_LINE)
        #if defined(USE_INFO_DIODES) && !defined(DISABLE_DIODES_CONFIGURE)
        else
        if(param == F("DIODES")) // запросили настройки информационных диодов: CTGET=EES|DIODES
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;
          PublishSingleton << PARAM_DELIMITER << diodesBinding.LinkType
          << PARAM_DELIMITER << diodesBinding.MCPAddress
          << PARAM_DELIMITER << diodesBinding.Level
          << PARAM_DELIMITER << diodesBinding.ReadyDiodePin
          << PARAM_DELIMITER << diodesBinding.BlinkReadyDiode
          << PARAM_DELIMITER << diodesBinding.WindowsManualModeDiodePin
          << PARAM_DELIMITER << diodesBinding.WateringManualModeDiodePin
          << PARAM_DELIMITER << diodesBinding.LightManualModeDiodePin
          
          ;                  
        } // F("DIODES")
        #endif // #ifdef USE_INFO_DIODES
        #ifdef USE_SOIL_MOISTURE_MODULE
         else
        if(param == F("SOIL")) // запросили настройки датчиков влажности почвы: CTGET=EES|SOIL
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << soilBinding.LinkType
          << PARAM_DELIMITER << soilBinding.MCPAddress
          << PARAM_DELIMITER << soilBinding.PowerPin
          << PARAM_DELIMITER << soilBinding.Level
          << PARAM_DELIMITER << soilBinding.PowerOnDelay
          << PARAM_DELIMITER << soilBinding.Soil100Percent
          << PARAM_DELIMITER << soilBinding.Soil0Percent
          << PARAM_DELIMITER << soilBinding.NoDataBorder
          ;

          for(size_t i=0;i<sizeof(soilBinding.SensorType)/sizeof(soilBinding.SensorType[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << soilBinding.SensorType[i];
          }
          for(size_t i=0;i<sizeof(soilBinding.SensorPin)/sizeof(soilBinding.SensorPin[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << soilBinding.SensorPin[i];
          }
        } // F("SOIL")
        #endif // USE_SOIL_MOISTURE_MODULE
        #ifdef USE_HUMIDITY_MODULE
         else
        if(param == F("HUMIDITY")) // запросили настройки датчиков влажности: CTGET=EES|HUMIDITY
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << humidityBinding.StrobeLevel
          << PARAM_DELIMITER << humidityBinding.DHTReadDelay
          ;

          for(size_t i=0;i<sizeof(humidityBinding.Pin1)/sizeof(humidityBinding.Pin1[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << humidityBinding.Pin1[i];
          }
          for(size_t i=0;i<sizeof(humidityBinding.Pin2)/sizeof(humidityBinding.Pin2[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << humidityBinding.Pin2[i];
          }
          for(size_t i=0;i<sizeof(humidityBinding.Type)/sizeof(humidityBinding.Type[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << humidityBinding.Type[i];
          }
          
        } // F("HUMIDITY")
        #endif // USE_HUMIDITY_MODULE
        #if defined(USE_RS485_GATE) && !defined(DISABLE_RS485_CONFIGURE)
        else
        if(param == F("RS485")) // запросили настройки RS485: CTGET=EES|RS485
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << rs485Binding.LinkType
          << PARAM_DELIMITER << rs485Binding.MCPAddress
          << PARAM_DELIMITER << rs485Binding.DEPin
          << PARAM_DELIMITER << rs485Binding.StatePushFrequency
          << PARAM_DELIMITER << rs485Binding.OneSensorUpdateInterval
          << PARAM_DELIMITER << rs485Binding.BytesTimeout
          << PARAM_DELIMITER << rs485Binding.ResetSensorBadReadings
          << PARAM_DELIMITER << rs485Binding.SerialNumber
          ;
          
        } // F("RS485")
        #endif // USE_RS485_GATE
        #ifdef USE_WATERING_MODULE
        else
        if(param == F("WATER")) // запросили настройки полива: CTGET=EES|WATER
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << wateringBinding.ChannelsLinkType
          << PARAM_DELIMITER << wateringBinding.ChannelsMCPAddress
          << PARAM_DELIMITER << wateringBinding.Level
          << PARAM_DELIMITER << wateringBinding.PumpLinkType
          << PARAM_DELIMITER << wateringBinding.PumpMCPAddress
          << PARAM_DELIMITER << wateringBinding.PumpPin
          << PARAM_DELIMITER << wateringBinding.Pump2LinkType
          << PARAM_DELIMITER << wateringBinding.Pump2MCPAddress
          << PARAM_DELIMITER << wateringBinding.Pump2Pin
          << PARAM_DELIMITER << wateringBinding.PumpLevel
          << PARAM_DELIMITER << wateringBinding.Pump2StartChannel
          << PARAM_DELIMITER << wateringBinding.SaveInterval
          ;

          for(size_t i=0;i<sizeof(wateringBinding.RelaysPins)/sizeof(wateringBinding.RelaysPins[0]);i++)
          {
            PublishSingleton << PARAM_DELIMITER << wateringBinding.RelaysPins[i];
          }
        } // F("WATER")
        #endif // USE_WATERING_MODULE
        #if defined(USE_SMS_MODULE)  && !defined(DISABLE_GSM_CONFIGURE)
        else
        if(param == F("SMS")) // запросили настройки SIM800: CTGET=EES|SMS
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << gsmBinding.SerialNumber
          << PARAM_DELIMITER << gsmBinding.AvailableCheckTime
          << PARAM_DELIMITER << gsmBinding.RebootTime
          << PARAM_DELIMITER << gsmBinding.WaitAfterRebootTime
          << PARAM_DELIMITER << gsmBinding.MaxAnswerTime
          << PARAM_DELIMITER << gsmBinding.RebootPinLinkType
          << PARAM_DELIMITER << gsmBinding.RebootPinMCPAddress
          << PARAM_DELIMITER << gsmBinding.RebootPin
          << PARAM_DELIMITER << gsmBinding.PowerOnLevel
          << PARAM_DELIMITER << gsmBinding.PullGPRSByPing
          << PARAM_DELIMITER << gsmBinding.PingInterval
          << PARAM_DELIMITER << gsmBinding.PowerkeyLinkType
          << PARAM_DELIMITER << gsmBinding.PowerkeyMCPAddress
          << PARAM_DELIMITER << gsmBinding.PowerkeyPin
          << PARAM_DELIMITER << gsmBinding.PowerkeyPulseDuration
          << PARAM_DELIMITER << gsmBinding.PowerkeyOnLevel
          << PARAM_DELIMITER << gsmBinding.WaitPowerkeyAfterPowerOn
          << PARAM_DELIMITER << gsmBinding.SendWorkStartedSMS
          << PARAM_DELIMITER << gsmBinding.IncomingDataTimeout
          << PARAM_DELIMITER << gsmBinding.CalmWatchdog
          << PARAM_DELIMITER << gsmBinding.SendWindowStateInStatusSMS
          << PARAM_DELIMITER << gsmBinding.SendWaterStateInStatusSMS
          ;

        } // F("SMS")
        #endif // USE_SMS_MODULE
        #if defined(USE_WIFI_MODULE) && !defined(DISABLE_WIFI_CONFIGURE)
        else
        if(param == F("WIFI")) // запросили настройки ESP: CTGET=EES|WIFI
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << wifiBinding.SerialNumber
          << PARAM_DELIMITER << wifiBinding.AvailableCheckTime
          << PARAM_DELIMITER << wifiBinding.RebootTime
          << PARAM_DELIMITER << wifiBinding.WaitAfterRebootTime
          << PARAM_DELIMITER << wifiBinding.MaxAnswerTime
          << PARAM_DELIMITER << wifiBinding.RebootPinLinkType
          << PARAM_DELIMITER << wifiBinding.RebootPinMCPAddress
          << PARAM_DELIMITER << wifiBinding.RebootPin
          << PARAM_DELIMITER << wifiBinding.PowerOnLevel
          ;

        } // F("WIFI")
        #endif // USE_WIFI_MODULE
        #if defined(USE_TFT_MODULE)  && !defined(DISABLE_TFT_CONFIGURE)
        else
        if(param == F("TFT")) // запросили настройки TFT: CTGET=EES|TFT
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = param;

          PublishSingleton << PARAM_DELIMITER << tftBinding.BacklightLinkType
          << PARAM_DELIMITER << tftBinding.BacklightMCPAddress
          << PARAM_DELIMITER << tftBinding.BacklightPin
          << PARAM_DELIMITER << tftBinding.BacklightOnLevel
          << PARAM_DELIMITER << tftBinding.InitDelay
          << PARAM_DELIMITER << tftBinding.OffDelay
          << PARAM_DELIMITER << tftBinding.DisplayType
          ;

        } // F("TFT")
        #endif
        #if defined(USE_TFT_MODULE)
        else
        if(param == F("TFTS")) // получить настройки датчика на TFT-экране, CTGET=EES|TFTS|rec idx
        {
          if(argsCnt > 1)
          {
            int recIdx = atoi(command.GetArg(1));
            if(recIdx < 0)
            {
              recIdx = 0; 
            }
            if(recIdx > 11)
            {
              recIdx = 11;
            }

            PublishSingleton.Flags.Status = true;
            PublishSingleton = param;

             PublishSingleton << PARAM_DELIMITER << recIdx 
             << PARAM_DELIMITER << tftSensorBinding[recIdx].Type
             << PARAM_DELIMITER << tftSensorBinding[recIdx].Index << PARAM_DELIMITER;

                const char* str = tftSensorBinding[recIdx].Caption;
                while(*str)
                {
                  PublishSingleton << WorkStatus::ToHex(*str++);
                }
          } // if(argsCnt > 1)
        } // "TFTS"
        #endif // USE_TFT_MODULE
        
      } // else args > 0
  }
  
  MainController->Publish(this,command); 
  
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------


