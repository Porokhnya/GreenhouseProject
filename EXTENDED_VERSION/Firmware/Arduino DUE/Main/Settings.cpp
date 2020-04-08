#include "Settings.h"
#include "Globals.h"
#include "Memory.h" 
//--------------------------------------------------------------------------------------------------------------------------------------
//  ГЛОБАЛЬНЫЕ НАСТРОЙКИ
//--------------------------------------------------------------------------------------------------------------------------------------
GlobalSettings::GlobalSettings()
{
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::setup()
{
  wateringOption = read8(WATERING_OPTION_EEPROM_ADDR, wateringOFF);
  wateringWeekDays = read8(WATERING_WEEKDAYS_EEPROM_ADDR,0);
  wateringTime = read16(WATERING_TIME_EEPROM_ADDR,0);
  startWateringTime = read16(START_WATERING_TIME_EEPROM_ADDR,0);
  wateringSensorIndex = (int8_t) read8(WATERING_SENSOR_EEPROM_ADDR, -1);
  wateringStopBorder = read8(WATERING_STOP_BORDER_EEPROM_ADDR, 0);
  turnOnPump = read8(TURN_PUMP_EEPROM_ADDR,0);
  turnWateringToAutoAfterMidnight = read8(WATERING_TURN_TO_AUTOMODE_AFTER_MIDNIGHT_ADDRESS,
  #ifdef SWITCH_TO_AUTOMATIC_WATERING_MODE_AFTER_MIDNIGHT
  1
  #else
  0
  #endif
  );

    for(uint8_t idx=0;idx < WATER_RELAYS_COUNT; idx++)
    {
      uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions);
      uint8_t* writeAddr = (uint8_t*) &(wateringChannels[idx]);

      for(size_t i=0;i<sizeof(WateringChannelOptions);i++)
      {
        *writeAddr++ = MemRead(readAddr++);
      }

    }

    // настройки по умолчанию для каналов
    for(uint8_t idx=0;idx < WATER_RELAYS_COUNT; idx++)
    {
      if(wateringChannels[idx].wateringTime == 0xFFFF)
        wateringChannels[idx].wateringTime = 0;

      if(wateringChannels[idx].startWateringTime == 0xFFFF)
        wateringChannels[idx].startWateringTime = 0;

      if((uint8_t)wateringChannels[idx].sensorIndex == 0xFF)
        wateringChannels[idx].sensorIndex = -1;

      if(wateringChannels[idx].stopBorder> 100)
        wateringChannels[idx].stopBorder = 0;
        
    } // for

    uint16_t readPtr = IOT_SETTINGS_EEPROM_ADDR;
    memset(&iotSettings,0,sizeof(IoTSettings));

    byte* writePtr = (byte*) &iotSettings;
    for(size_t i=0;i<sizeof(IoTSettings);i++)
      *writePtr++ = read8(readPtr++,0);


   controllerID = read8(CONTROLLER_ID_EEPROM_ADDR,0);
   openTemp = read8(OPEN_TEMP_EEPROM_ADDR,DEF_OPEN_TEMP);
   closeTemp = read8(CLOSE_TEMP_EEPROM_ADDR,DEF_CLOSE_TEMP);
   openInterval = read32(OPEN_INTERVAL_EEPROM_ADDR,DEF_OPEN_INTERVAL);
   smsPhoneNumber = readString(SMS_NUMBER_EEPROM_ADDR,15);
   wifiState = read8(WIFI_STATE_EEPROM_ADDR,0x01);
   routerID = readString(ROUTER_ID_EEPROM_ADDR,20);
   routerPassword = readString(ROUTER_PASSWORD_EEPROM_ADDR,20);
   stationID = readString(STATION_ID_EEPROM_ADDR,20);
   stationPassword = readString(STATION_PASSWORD_EEPROM_ADDR,20);


  uint16_t addr = HTTP_API_KEY_ADDRESS;
  
  byte header1 = MemRead(addr++);
  byte header2 = MemRead(addr++);

  if(header1 == SETT_HEADER1 && header2 == SETT_HEADER2)
  {
      for(byte i=0;i<32;i++)
      {
        char ch = (char) MemRead(addr++);
        if(ch != '\0' && ch != 0xFF)
          httpApiKey += ch;
        else
          break;
      }
  } // if


  addr = HTTP_API_KEY_ADDRESS + 34;
  byte en = MemRead(addr);
  if(en == 0xFF)
    en = 0; // если ничего не записано - считаем, что API выключено

  isHTTPEnabled = en ? true : false;

  timezone = 0;
  addr = TIMEZONE_ADDRESS;
  
  header1 = MemRead(addr++);
  header2 = MemRead(addr++);

  if(header1 == SETT_HEADER1 && header2 == SETT_HEADER2)
  {
      byte* b = (byte*) &timezone;
      *b++ = MemRead(addr++);
      *b++ = MemRead(addr++);

      if(0xFFFF == (uint16_t)timezone)
        timezone = 0;
  }

  en = MemRead(HTTP_SEND_SENSORS_DATA_ADDRESS);
  if(en == 0xFF)
    en = 1; // если ничего не записано - считаем, что можем отсылать данные

  canSendSensorsToHTTP = en ? true : false; 


  en = MemRead(HTTP_SEND_STATUS_ADDRESS);
  if(en == 0xFF)
    en = 1; // если ничего не записано - считаем, что можем отсылать данные

  canSendControllerStatusToHTTP = en ? true : false; 

  #ifdef WM_KEY_ADDRESS
  key = readString(WM_KEY_ADDRESS,8);
  #endif

  memset(&timeSyncSettings,0,sizeof(TimeSyncSettings));
  
  timeSyncSettings.gsmActive = false;
  timeSyncSettings.wifiActive = false;

  timeSyncSettings.gsmInterval = GSM_SYNC_TIME_INTERVAL;
  timeSyncSettings.wifiInterval = NTP_UPDATE_INTERVAL;

  timeSyncSettings.ntpPort = NTP_PORT;
  timeSyncSettings.ntpTimezone = NTP_TIMEZONE;
  String ns = NTP_SERVER;
  strcpy(timeSyncSettings.ntpServer,ns.c_str());

  
  addr = SYNC_SETTINGS_ADDRESS;
  if(checkHeader(addr))
  {
    addr += 2;
    uint8_t* tsWr = (uint8_t*)&timeSyncSettings;
    for(size_t i=0;i<sizeof(TimeSyncSettings);i++)
    {
      *tsWr++ = MemRead(addr++);
    } // for
  } // checkHeader

}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::setTimeSyncSettings(TimeSyncSettings& val)
{
  memcpy(&timeSyncSettings,&val,sizeof(TimeSyncSettings));
  uint16_t addr = SYNC_SETTINGS_ADDRESS;
  writeHeader(addr);
  addr += 2;

  uint8_t* readPtr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(TimeSyncSettings);i++)
  {
    MemWrite(addr++,*readPtr++);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::getKey()
{
  return key;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::setKey(const String& val)
{
  key = val;
  #ifdef WM_KEY_ADDRESS
  writeString(WM_KEY_ADDRESS,val,8);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::WriteDeltaSettings(DeltaCountFunction OnDeltaGetCount, DeltaReadWriteFunction OnDeltaWrite)
{
  if(!(OnDeltaGetCount && OnDeltaWrite)) // обработчики не заданы
    return;

  uint16_t writeAddr = DELTA_SETTINGS_EEPROM_ADDR;

  // записываем заголовок
  MemWrite(writeAddr++,SETT_HEADER1);
  MemWrite(writeAddr++,SETT_HEADER2);
  

  uint8_t deltaCount = 0;

  // получаем кол-во дельт
  OnDeltaGetCount(deltaCount);

  // записываем кол-во дельт
  MemWrite(writeAddr++,deltaCount);

  //теперь пишем дельты
  for(uint8_t i=0;i<deltaCount;i++)
  {
    String name1,name2;
    uint8_t sensorType = 0,sensorIdx1 = 0,sensorIdx2 = 0;

    // получаем настройки дельт
    OnDeltaWrite(sensorType,name1,sensorIdx1,name2,sensorIdx2);

    // получили, можем сохранять. Каждая запись дельт идёт так:
  
  // 1 байт - тип датчика (температура, влажность, освещенность)
  
  // 1 байт - длина имени модуля 1
  // N байт - имя модуля 1
  // 1 байт - индекс датчика модуля 1
  
  // 1 байт - длина имени модуля 2
  // N байт - имя модуля 2
  // 1 байт - индекс датчика модуля 1

    // пишем тип датчика
     MemWrite(writeAddr++,sensorType);

     // пишем длину имени модуля 1
     uint8_t nameLen = name1.length();
     MemWrite(writeAddr++,nameLen);

     // пишем имя модуля 1
     const char* namePtr = name1.c_str();
     for(uint8_t idx=0;idx<nameLen; idx++)
      MemWrite(writeAddr++,*namePtr++);

     // пишем индекс датчика 1
     MemWrite(writeAddr++,sensorIdx1);


     // пишем длину имени модуля 2
     nameLen = name2.length();
     MemWrite(writeAddr++,nameLen);

     // пишем имя модуля 2
     namePtr = name2.c_str();
     for(uint8_t idx=0;idx<nameLen; idx++)
      MemWrite(writeAddr++,*namePtr++);

     // пишем индекс датчика 2
     MemWrite(writeAddr++,sensorIdx2);
     
    
  } // for

  // записали, отдыхаем
    
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::ReadDeltaSettings(DeltaCountFunction OnDeltaSetCount, DeltaReadWriteFunction OnDeltaRead)
{
  if(!(OnDeltaSetCount && OnDeltaRead)) // обработчики не заданы
    return;

  uint16_t readAddr = DELTA_SETTINGS_EEPROM_ADDR;
  uint8_t h1,h2;
  
  h1 = MemRead(readAddr++);
  h2 = MemRead(readAddr++);

  uint8_t deltaCount = 0;

  if(!(h1 == SETT_HEADER1 && h2 == SETT_HEADER2)) // в памяти нет данных о сохранённых настройках дельт
  {
    
    OnDeltaSetCount(deltaCount); // сообщаем, что мы прочитали 0 настроек
    return; // и выходим
  }

  // читаем кол-во настроек
  deltaCount = MemRead(readAddr++);
  if(deltaCount == 0xFF) // ничего нет
    deltaCount = 0; // сбрасываем в ноль
    
  OnDeltaSetCount(deltaCount); // сообщаем, что мы прочитали N настроек

  // читаем настройки дельт. В памяти каждая запись дельт идёт так:
  
  // 1 байт - тип датчика (температура, влажность, освещенность)
  
  // 1 байт - длина имени модуля 1
  // N байт - имя модуля 1
  // 1 байт - индекс датчика модуля 1
  
  // 1 байт - длина имени модуля 2
  // N байт - имя модуля 2
  // 1 байт - индекс датчика модуля 1
  
  // теперь читаем настройки
  for(uint8_t i=0;i<deltaCount;i++)
  {
    // читаем тип датчика
    uint8_t sensorType = MemRead(readAddr++);

    // читаем длину имени модуля 1
    uint8_t nameLen = MemRead(readAddr++);
    
    // резервируем память
    String name1; name1.reserve(nameLen + 1);
    
    // читаем имя модуля 1
    for(uint8_t idx = 0; idx < nameLen; idx++)
      name1 += (char) MemRead(readAddr++);

    // читаем индекс датчика модуля 1
    uint8_t sensorIdx1 = MemRead(readAddr++);

    // читаем длину имени модуля 2 
    nameLen = MemRead(readAddr++);
    
    // резервируем память
    String name2; name2.reserve(nameLen + 1);

    // читаем имя модуля 2
    for(uint8_t idx = 0; idx < nameLen; idx++)
      name2 += (char) MemRead(readAddr++);

    // читаем индекс датчика модуля 2
    uint8_t sensorIdx2 = MemRead(readAddr++);

    // всё прочитали - можем вызывать функцию, нам переданную
    OnDeltaRead(sensorType,name1,sensorIdx1,name2,sensorIdx2);
    
  } // for

  // всё прочитали, отдыхаем
  
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindowsChannelsBinding GlobalSettings::readWBinding(uint16_t addr)
{
  WindowsChannelsBinding result;
   memset(&result,0,sizeof(WindowsChannelsBinding));
   
  if(!checkHeader(addr))
    return result;

    addr += sizeof(int16_t);

    for(size_t i=0;i<sizeof(result.binding);i++)
    {
      result.binding[i] = MemRead(addr++);
    }

    return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::writeWBinding(uint16_t addr, WindowsChannelsBinding& val)
{
  writeHeader(addr);
  addr += sizeof(int16_t);

  for(size_t i=0;i<sizeof(val.binding);i++)
  {
    MemWrite(addr++,val.binding[i]);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindowsChannelsBinding GlobalSettings::GetOrientationBinding()
{
    return readWBinding(WM_ORIENTATION_BINDING_ADDRESS);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetOrientationBinding(WindowsChannelsBinding& val)
{
  writeWBinding(WM_ORIENTATION_BINDING_ADDRESS, val); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindowsChannelsBinding GlobalSettings::GetRainBinding()
{
    return readWBinding(WM_RAIN_BINDING_ADDRESS);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetRainBinding(WindowsChannelsBinding& val)
{
  writeWBinding(WM_RAIN_BINDING_ADDRESS, val); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
WindowsChannelsBinding GlobalSettings::GetWMBinding()
{
    return readWBinding(WM_BINDING_ADDRESS);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWMBinding(WindowsChannelsBinding& val)
{
  writeWBinding(WM_BINDING_ADDRESS, val); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetWindSpeed(int16_t defVal)
{
   uint16_t addr = WM_WIND_SPEED_ADDRESS;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWindSpeed(int16_t val)
{
  uint16_t addr = WM_WIND_SPEED_ADDRESS;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);  
}
//--------------------------------------------------------------------------------------------------------------------------------------    
int16_t GlobalSettings::GetHurricaneSpeed(int16_t defVal)
{
   uint16_t addr = WM_HURRICANE_SPEED_ADDRESS;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHurricaneSpeed(int16_t val)
{
  uint16_t addr = WM_HURRICANE_SPEED_ADDRESS;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
LightSettings GlobalSettings::GetLightSettings()
{
  LightSettings result;
  memset(&result,0,sizeof(LightSettings));

  uint16_t addr = WM_LIGHT_SETTINGS_ADDRESS;
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(LightSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  if(result.hour <0 )
    result.hour = 0;
    
  if(result.hour > 24)
    result.hour = 0;
    
  if(result.histeresis < 0)
    result.histeresis = 0;
    
  if(result.durationHour < 0)
    result.durationHour = 0;
    
  if(result.lux < 0)
    result.lux = 0;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetLightSettings(LightSettings& val)
{
  uint16_t addr = WM_LIGHT_SETTINGS_ADDRESS;
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(LightSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
DoorSettings GlobalSettings::GetDoorSettings(uint8_t channel)
{
  DoorSettings result;
  memset(&result,0,sizeof(DoorSettings));

  result.active = false;
  
  uint16_t addr = WM_DOOR_SETTINGS_ADDRESS + channel*(sizeof(DoorSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(DoorSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetDoorSettings(uint8_t channel,DoorSettings& val)
{
  uint16_t addr = WM_DOOR_SETTINGS_ADDRESS + channel*(sizeof(DoorSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(DoorSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
ShadowSettings GlobalSettings::GetShadowSettings(uint8_t channel)
{
  ShadowSettings result;
  memset(&result,0,sizeof(ShadowSettings));

  result.active = true;
  
  uint16_t addr = WM_SHADOW_SETTINGS_ADDRESS + channel*(sizeof(ShadowSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(ShadowSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetShadowSettings(uint8_t channel,ShadowSettings& val)
{
  uint16_t addr = WM_SHADOW_SETTINGS_ADDRESS + channel*(sizeof(ShadowSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(ShadowSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
VentSettings GlobalSettings::GetVentSettings(uint8_t channel)
{
  VentSettings result;
  memset(&result,0,sizeof(VentSettings));

  result.active = true;
  result.histeresis = 5;
  result.maxWorkTime = VENT_MAX_WORK_TIME;
  
  uint16_t addr = WM_VENT_SETTINGS_ADDRESS + channel*(sizeof(VentSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(VentSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetVentSettings(uint8_t channel,VentSettings& val)
{
  uint16_t addr = WM_VENT_SETTINGS_ADDRESS + channel*(sizeof(VentSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(VentSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
ThermostatSettings GlobalSettings::GetThermostatSettings(uint8_t channel)
{
  ThermostatSettings result;
  memset(&result,0,sizeof(ThermostatSettings));

  result.active = true;
  result.histeresis = 5;
  
  uint16_t addr = WM_THERMOSTAT_SETTINGS_ADDRESS + channel*(sizeof(ThermostatSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(ThermostatSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetThermostatSettings(uint8_t channel,ThermostatSettings& val)
{
  uint16_t addr = WM_THERMOSTAT_SETTINGS_ADDRESS + channel*(sizeof(ThermostatSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(ThermostatSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
CycleVentSettings GlobalSettings::GetCycleVentSettings(uint8_t channel)
{
  CycleVentSettings result;
  memset(&result,0,sizeof(CycleVentSettings));

  result.active = true;
  
  uint16_t addr = WM_CYCLE_SETTINGS_ADDRESS + channel*(sizeof(CycleVentSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(CycleVentSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetCycleVentSettings(uint8_t channel,CycleVentSettings& val)
{
  uint16_t addr = WM_CYCLE_SETTINGS_ADDRESS + channel*(sizeof(CycleVentSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(CycleVentSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
CO2Settings GlobalSettings::GetCO2Settings()
{
  CO2Settings result;
  memset(&result,0,sizeof(CO2Settings));

  result.active = true;
  
  uint16_t addr = WM_CO2_SETTINGS_ADDRESS;
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(CO2Settings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetCO2Settings(CO2Settings& val)
{
  uint16_t addr = WM_CO2_SETTINGS_ADDRESS;
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(CO2Settings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
HeatSettings GlobalSettings::GetHeatSettings(uint8_t channel)
{
  HeatSettings result;
  memset(&result,0,sizeof(HeatSettings));

  result.active = false;
  result.minTemp = HEAT_DEFAULT_MIN_TEMP;
  result.maxTemp = HEAT_DEFAULT_MAX_TEMP;
  result.ethalonTemp = HEAT_DEFAULT_ETHALON_TEMP;
  result.histeresis = HEAT_DEFAULT_HISTERESIS;
  
  uint16_t addr = WM_HEAT_SETTINGS_ADDRESS + channel*(sizeof(HeatSettings) + sizeof(uint16_t));
  if(!checkHeader(addr))
    return result;

  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&result;
  for(size_t i=0;i<sizeof(HeatSettings);i++)
  {
    *ptr++ = MemRead(addr++);
  }

  result.active = (bool) result.active;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHeatSettings(uint8_t channel,HeatSettings& val)
{
  uint16_t addr = WM_HEAT_SETTINGS_ADDRESS + channel*(sizeof(HeatSettings) + sizeof(uint16_t));
  writeHeader(addr);
  addr += sizeof(int16_t);

  uint8_t* ptr = (uint8_t*)&val;
  for(size_t i=0;i<sizeof(HeatSettings);i++)
  {
    MemWrite(addr++,*ptr++);
  }

}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetSealevel(int16_t defVal)
{
  uint16_t addr = WM_SEALEVEL_ADDRESS;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSealevel(int16_t val)
{
  uint16_t addr = WM_SEALEVEL_ADDRESS;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetHeatDriveWorkTime(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_HEAT_WORKTIME_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHeatDriveWorkTime(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_HEAT_WORKTIME_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetShadowDriveWorkTime(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_SHADOW_WORKTIME_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetShadowDriveWorkTime(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_SHADOW_WORKTIME_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetWMHisteresis(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_HISTERESIS_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWMHisteresis(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_HISTERESIS_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetWMSensor(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_SENSOR_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWMSensor(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_SENSOR_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GlobalSettings::GetWMActive(uint8_t channel,bool defVal)
{
  uint16_t addr = WM_ACTIVE_ADDRESS + channel*3;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return (bool) read8(addr,defVal ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWMActive(uint8_t channel,bool val)
{
  uint16_t addr = WM_ACTIVE_ADDRESS + channel*3;
  writeHeader(addr);
  addr += sizeof(int16_t);
  MemWrite(addr,val ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::Get25PercentsOpenTemp(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_T25_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::Set25PercentsOpenTemp(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_T25_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::Get50PercentsOpenTemp(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_T50_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::Set50PercentsOpenTemp(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_T50_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::Get75PercentsOpenTemp(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_T75_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::Set75PercentsOpenTemp(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_T75_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::Get100PercentsOpenTemp(uint8_t channel,int16_t defVal)
{
  uint16_t addr = WM_T100_ADDRESS + channel*4;
  if(!checkHeader(addr))
    return defVal;

  addr += sizeof(int16_t);

  return read16(addr,defVal);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::Set100PercentsOpenTemp(uint8_t channel,int16_t val)
{
  uint16_t addr = WM_T100_ADDRESS + channel*4;
  writeHeader(addr);
  addr += sizeof(int16_t);
  write16(addr,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GlobalSettings::checkHeader(uint16_t address)
{
  if(MemRead(address++) != SETT_HEADER1)
    return false;

  return MemRead(address) == SETT_HEADER2;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::writeHeader(uint16_t addr)
{
  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr,SETT_HEADER2);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::read8(uint16_t address, uint8_t defaultVal)
{
    uint8_t curVal = MemRead(address);
    if(curVal == 0xFF)
      curVal = defaultVal;

   return curVal;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::read16(uint16_t address, uint16_t defaultVal)
{
    uint16_t val = 0;
    byte* b = (byte*) &val;
    
    for(byte i=0;i<2;i++)
      *b++ = MemRead(address + i);

   if(val == 0xFFFF)
    val = defaultVal;

    return val;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::write16(uint16_t address, uint16_t val)
{
  byte* b = (byte*) &val;

  for(byte i=0;i<2;i++)
    MemWrite(address + i, *b++);
      
}
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned long GlobalSettings::read32(uint16_t address, unsigned long defaultVal)
{
   unsigned long val = 0;
    byte* b = (byte*) &val;
    
    for(byte i=0;i<4;i++)
      *b++ = MemRead(address + i);

   if(val == 0xFFFFFFFF)
    val = defaultVal;

    return val;    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::write32(uint16_t address, unsigned long val)
{
  byte* b = (byte*) &val;

  for(byte i=0;i<4;i++)
    MemWrite(address + i, *b++);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::readString(uint16_t address, byte maxlength)
{
  String result;
  for(byte i=0;i<maxlength;i++)
  {
    byte b = read8(address++,0);
    if(b == 0)
      break;

    result += (char) b;
  }

  return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::writeString(uint16_t address, const String& v, byte maxlength)
{

  for(byte i=0;i<maxlength;i++)
  {
    if(i >= v.length())
      break;
      
    MemWrite(address++,v[i]);
  }

  // пишем завершающий ноль
  MemWrite(address++,'\0');
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetChannelWateringWeekDays(uint8_t idx)
{
    return wateringChannels[idx].wateringWeekDays;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringWeekDays(uint8_t idx, uint8_t val)
{
    // вычисляем начало адреса
    wateringChannels[idx].wateringWeekDays = val;
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions);
    MemWrite(writeAddr, val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetChannelWateringTime(uint8_t idx)
{
    return wateringChannels[idx].wateringTime;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringTime(uint8_t idx,uint16_t val)
{
    wateringChannels[idx].wateringTime = val;
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 1; // со второго байта в структуре идёт время продолжительности полива
    write16(writeAddr,val);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetChannelStartWateringTime(uint8_t idx)
{
    return wateringChannels[idx].startWateringTime;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelStartWateringTime(uint8_t idx,uint16_t val)
{
    wateringChannels[idx].startWateringTime = val;
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 3; // с четвёртого байта в структуре идёт время начала полива
    write16(writeAddr,val);    
}
//--------------------------------------------------------------------------------------------------------------------------------------
int8_t GlobalSettings::GetChannelWateringSensorIndex(uint8_t idx)
{
   return wateringChannels[idx].sensorIndex;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringSensorIndex(uint8_t idx,int8_t val)
{
   wateringChannels[idx].sensorIndex = val;
   uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 5; // с шестого байта в структуре идёт индекс датчика
  MemWrite(writeAddr,val);   
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetChannelWateringStopBorder(uint8_t idx)
{
  return wateringChannels[idx].stopBorder;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringStopBorder(uint8_t idx,uint8_t val)
{
  wateringChannels[idx].stopBorder = val;
  uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 6; // с седьмого байта в структуре идёт значение показаний датчика
  MemWrite(writeAddr,val);     
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetStationPassword()
{
  return stationPassword;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStationPassword(const String& v)
{
  stationPassword = v;
  writeString( STATION_PASSWORD_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetStationID()
{
  return stationID;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStationID(const String& v)
{
  stationID = v;
  writeString( STATION_ID_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetRouterPassword()
{
  return routerPassword;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetRouterPassword(const String& v)
{
  routerPassword = v;
  writeString( ROUTER_PASSWORD_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetRouterID()
{
  return routerID;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetRouterID(const String& v)
{
  routerID = v;
  writeString( ROUTER_ID_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetSmsPhoneNumber()
{
  return smsPhoneNumber;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSmsPhoneNumber(const String& v)
{
    smsPhoneNumber = v;
    writeString( SMS_NUMBER_EEPROM_ADDR, v,  15);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetIoTSettings(IoTSettings& sett)
{
    iotSettings = sett;
    byte writePtr = IOT_SETTINGS_EEPROM_ADDR;
    byte* readPtr = (byte*) &sett;

     for(size_t i=0;i<sizeof(IoTSettings);i++)
        MemWrite(writePtr++, *readPtr++);
}
//--------------------------------------------------------------------------------------------------------------------------------------
IoTSettings GlobalSettings::GetIoTSettings()
{
    return iotSettings;   
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWiFiState()
{
  return wifiState;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWiFiState(uint8_t st)
{
  wifiState = st;
  MemWrite(WIFI_STATE_EEPROM_ADDR,st);
}
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned long GlobalSettings::GetOpenInterval()
{
  return openInterval;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetOpenInterval(unsigned long val)
{
  openInterval = val;
  write32(OPEN_INTERVAL_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetCloseTemp()
{
  return closeTemp;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetCloseTemp(uint8_t val)
{
  closeTemp = val;
  MemWrite(CLOSE_TEMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetOpenTemp()
{
    return openTemp;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetOpenTemp(uint8_t val)
{
  openTemp = val;
  MemWrite(OPEN_TEMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetTurnOnPump()
{
  return turnOnPump;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTurnOnPump(uint8_t val)
{
  turnOnPump = val;
  MemWrite(TURN_PUMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetTurnWateringToAutoAfterMidnight()
{  
  return turnWateringToAutoAfterMidnight;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTurnWateringToAutoAfterMidnight(uint8_t val)
{
  turnWateringToAutoAfterMidnight = val;
  MemWrite(WATERING_TURN_TO_AUTOMODE_AFTER_MIDNIGHT_ADDRESS,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStartWateringTime(uint16_t val)
{
  startWateringTime = val;
  write16(START_WATERING_TIME_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetStartWateringTime()
{
  return startWateringTime;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringTime(uint16_t val)
{
  wateringTime = val;
  write16(WATERING_TIME_EEPROM_ADDR,val);    
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetWateringTime()
{
  return wateringTime;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringWeekDays(uint8_t val)
{
  wateringWeekDays = val;
  MemWrite(WATERING_WEEKDAYS_EEPROM_ADDR, val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringWeekDays()
{
  return wateringWeekDays;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringStopBorder()
{
  return wateringStopBorder;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringStopBorder(uint8_t val)
{
  wateringStopBorder = val;
  MemWrite(WATERING_STOP_BORDER_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int8_t GlobalSettings::GetWateringSensorIndex()
{
  return wateringSensorIndex;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringSensorIndex(int8_t val)
{
  wateringSensorIndex = val;
  MemWrite(WATERING_SENSOR_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringOption(uint8_t val)
{
  wateringOption = val;
  MemWrite(WATERING_OPTION_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringOption()
{
  return wateringOption;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetGSMProviderName(const String& val)
{
  writeString(GSM_PROVIDER_NAME_ADDRESS,val,19);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMProviderName()
{
  String res = readString(GSM_PROVIDER_NAME_ADDRESS,20);
  if(!res.length())
    res = F("MTS");

  return res;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetGSMAPNAddress(const String& val)
{
  writeString(GSM_APN_ADDRESS,val,49);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMAPNAddress()
{
  String res = readString(GSM_APN_ADDRESS,50);
  if(!res.length())
    res = F("mts");

  return res;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetGSMAPNUser(const String& val)
{
  writeString(GSM_APN_USER_ADDRESS,val,19);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMAPNUser()
{
  return readString(GSM_APN_USER_ADDRESS,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetGSMAPNPassword(const String& val)
{
  writeString(GSM_APN_PASS_ADDRESS,val,19);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMAPNPassword()
{
  return readString(GSM_APN_PASS_ADDRESS,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetGSMBalanceCommand(const String& val)
{
  writeString(GSM_BALANCE_CUSD_ADDRESS,val,49);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMBalanceCommand()
{
  String res = readString(GSM_BALANCE_CUSD_ADDRESS,50);
  if(!res.length())
    res = F("*100#");

  return res;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetControllerID()
{
  return controllerID;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetControllerID(uint8_t val)
{
  controllerID = val;
  MemWrite(CONTROLLER_ID_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GlobalSettings::IsHttpApiEnabled()
{
  return isHTTPEnabled;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHttpApiEnabled(bool val)
{
  isHTTPEnabled = val;
  uint16_t addr = HTTP_API_KEY_ADDRESS + 34;
  MemWrite(addr,val ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetTimezone()
{
  return timezone;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTimezone(int16_t val)
{
  timezone = val;
  
  uint16_t addr = TIMEZONE_ADDRESS;
  
  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

  byte* b = (byte*) &val;

  MemWrite(addr++,*b++);
  MemWrite(addr++,*b++);
  
    
}
//--------------------------------------------------------------------------------------------------------------------------------------        
bool GlobalSettings::CanSendSensorsDataToHTTP()
{
  return canSendSensorsToHTTP;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSendSensorsDataFlag(bool val)
{
   canSendSensorsToHTTP = val;
   MemWrite(HTTP_SEND_SENSORS_DATA_ADDRESS, val ? 1 : 0); 
}
//--------------------------------------------------------------------------------------------------------------------------------------        
bool GlobalSettings::CanSendControllerStatusToHTTP()
{
  return canSendControllerStatusToHTTP;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSendControllerStatusFlag(bool val)
{
   canSendControllerStatusToHTTP = val;
   MemWrite(HTTP_SEND_STATUS_ADDRESS, val ? 1 : 0); 
}
//--------------------------------------------------------------------------------------------------------------------------------------        
String GlobalSettings::GetHttpApiKey()
{
  return httpApiKey;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHttpApiKey(const char* val)
{
  if(!*val)
    return;

  httpApiKey = val;

  uint16_t addr = HTTP_API_KEY_ADDRESS;
  
  MemWrite(addr++,SETT_HEADER1);
  MemWrite(addr++,SETT_HEADER2);

  for(byte i=0;i<32;i++)
  {
      if(!*val)
      {
          MemWrite(addr++,'\0');
          break;  
      }

      MemWrite(addr++,*val);
      val++;
  } // for
    
}
//--------------------------------------------------------------------------------------------------------------------------------------


