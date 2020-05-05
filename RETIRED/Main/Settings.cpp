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
    // вычисляем начало адреса
    uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions);
    return read8(readAddr,0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringWeekDays(uint8_t idx, uint8_t val)
{
    // вычисляем начало адреса
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions);
    MemWrite(writeAddr, val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetChannelWateringTime(uint8_t idx)
{
    uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 1; // со второго байта в структуре идёт время продолжительностиала полива
    return read16(readAddr,0);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringTime(uint8_t idx,uint16_t val)
{
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 1; // со второго байта в структуре идёт время продолжительности полива
    write16(writeAddr,val);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetChannelStartWateringTime(uint8_t idx)
{
    uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 3; // с четвёртого байта в структуре идёт время начала полива
    return read16(readAddr,0);    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelStartWateringTime(uint8_t idx,uint16_t val)
{
    uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 3; // с четвёртого байта в структуре идёт время начала полива
    write16(writeAddr,val);    
}
//--------------------------------------------------------------------------------------------------------------------------------------
int8_t GlobalSettings::GetChannelWateringSensorIndex(uint8_t idx)
{
   uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 5; // с шестого байта в структуре идёт индекс датчика
  return (int8_t) read8(readAddr,-1);   
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringSensorIndex(uint8_t idx,int8_t val)
{
   uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 5; // с шестого байта в структуре идёт индекс датчика
  MemWrite(writeAddr,val);   
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetChannelWateringStopBorder(uint8_t idx)
{
  uint16_t readAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 6; // с седьмого байта в структуре идёт значение показаний датчика
  return read8(readAddr,0);     
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetChannelWateringStopBorder(uint8_t idx,uint8_t val)
{
  uint16_t writeAddr = WATERING_CHANNELS_SETTINGS_EEPROM_ADDR + idx*sizeof(WateringChannelOptions) + 6; // с седьмого байта в структуре идёт значение показаний датчика
  MemWrite(writeAddr,val);     
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetStationPassword()
{
  return readString(STATION_PASSWORD_EEPROM_ADDR,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStationPassword(const String& v)
{
  writeString( STATION_PASSWORD_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetStationID()
{
  return readString(STATION_ID_EEPROM_ADDR,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStationID(const String& v)
{
  writeString( STATION_ID_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetRouterPassword()
{
  return readString(ROUTER_PASSWORD_EEPROM_ADDR,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetRouterPassword(const String& v)
{
  writeString( ROUTER_PASSWORD_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetRouterID()
{
  return readString(ROUTER_ID_EEPROM_ADDR,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetRouterID(const String& v)
{
  writeString( ROUTER_ID_EEPROM_ADDR, v,  20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetSmsPhoneNumber()
{
  return readString(SMS_NUMBER_EEPROM_ADDR,15);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSmsPhoneNumber(const String& v)
{
    writeString( SMS_NUMBER_EEPROM_ADDR, v,  15);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetIoTSettings(IoTSettings& sett)
{
    byte writePtr = IOT_SETTINGS_EEPROM_ADDR;
    byte* readPtr = (byte*) &sett;

     for(size_t i=0;i<sizeof(IoTSettings);i++)
        MemWrite(writePtr++, *readPtr++);
}
//--------------------------------------------------------------------------------------------------------------------------------------
IoTSettings GlobalSettings::GetIoTSettings()
{
    uint16_t readPtr = IOT_SETTINGS_EEPROM_ADDR;
    IoTSettings result;
    memset(&result,0,sizeof(IoTSettings));

    byte* writePtr = (byte*) &result;
    for(size_t i=0;i<sizeof(IoTSettings);i++)
      *writePtr++ = read8(readPtr++,0);

   return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWiFiState()
{
  return read8(WIFI_STATE_EEPROM_ADDR,0x01);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWiFiState(uint8_t st)
{
  MemWrite(WIFI_STATE_EEPROM_ADDR,st);
}
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned long GlobalSettings::GetOpenInterval()
{
  return read32(OPEN_INTERVAL_EEPROM_ADDR,DEF_OPEN_INTERVAL);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetOpenInterval(unsigned long val)
{
  write32(OPEN_INTERVAL_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetCloseTemp()
{
  return read8(CLOSE_TEMP_EEPROM_ADDR,DEF_CLOSE_TEMP);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetCloseTemp(uint8_t val)
{
  MemWrite(CLOSE_TEMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetOpenTemp()
{
    return read8(OPEN_TEMP_EEPROM_ADDR,DEF_OPEN_TEMP);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetOpenTemp(uint8_t val)
{
  MemWrite(OPEN_TEMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetTurnOnPump()
{
  return read8(TURN_PUMP_EEPROM_ADDR,0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTurnOnPump(uint8_t val)
{
  MemWrite(TURN_PUMP_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetTurnWateringToAutoAfterMidnight()
{
  return read8(WATERING_TURN_TO_AUTOMODE_AFTER_MIDNIGHT_ADDRESS,
  #ifdef SWITCH_TO_AUTOMATIC_WATERING_MODE_AFTER_MIDNIGHT
  1
  #else
  0
  #endif
  );
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTurnWateringToAutoAfterMidnight(uint8_t val)
{
  MemWrite(WATERING_TURN_TO_AUTOMODE_AFTER_MIDNIGHT_ADDRESS,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetStartWateringTime(uint16_t val)
{
  write16(START_WATERING_TIME_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetStartWateringTime()
{
  return read16(START_WATERING_TIME_EEPROM_ADDR,0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringTime(uint16_t val)
{
  write16(WATERING_TIME_EEPROM_ADDR,val);    
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t GlobalSettings::GetWateringTime()
{
  return read16(WATERING_TIME_EEPROM_ADDR,0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringWeekDays(uint8_t val)
{
  MemWrite(WATERING_WEEKDAYS_EEPROM_ADDR, val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringWeekDays()
{
  return read8(WATERING_WEEKDAYS_EEPROM_ADDR,0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringStopBorder()
{
  return read8(WATERING_STOP_BORDER_EEPROM_ADDR, 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringStopBorder(uint8_t val)
{
  MemWrite(WATERING_STOP_BORDER_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int8_t GlobalSettings::GetWateringSensorIndex()
{
  return (int8_t) read8(WATERING_SENSOR_EEPROM_ADDR, -1);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringSensorIndex(int8_t val)
{
  MemWrite(WATERING_SENSOR_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetWateringOption(uint8_t val)
{
  MemWrite(WATERING_OPTION_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetWateringOption()
{
  return read8(WATERING_OPTION_EEPROM_ADDR, wateringOFF);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String GlobalSettings::GetGSMProviderName()
{
  byte prov = GetGSMProvider();

  switch(prov)
  {
    case MTS: return PROV_NAME_MTS;
    case Beeline: return PROV_NAME_BEELINE;
    case Megafon: return PROV_NAME_MEGAFON;
    case Tele2: return PROV_NAME_TELE2;
    case Yota: return PROV_NAME_YOTA;
    case MTS_Bel: return PROV_NAME_MTS_BEL;
    case Velcom_Bel: return PROV_NAME_VELCOM_BEL;
    case Privet_Bel: return PROV_NAME_PRIVET_BEL;
    case Life_Bel: return PROV_NAME_LIFE_BEL;
    
  } // switch

  return "";
}
//--------------------------------------------------------------------------------------------------------------------------------------
byte GlobalSettings::GetGSMProvider()
{
  return read8(GSM_PROVIDER_EEPROM_ADDR,MTS); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GlobalSettings::SetGSMProvider(byte p)
{
  if(p < Dummy_Last_Op) 
  {
    MemWrite(GSM_PROVIDER_EEPROM_ADDR,p);
    return true;
  }
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t GlobalSettings::GetControllerID()
{
  return read8(CONTROLLER_ID_EEPROM_ADDR,0);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetControllerID(uint8_t val)
{
  //controllerID = val;
  MemWrite(CONTROLLER_ID_EEPROM_ADDR,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool GlobalSettings::IsHttpApiEnabled()
{
  uint16_t addr = HTTP_API_KEY_ADDRESS + 34;
  byte en = MemRead(addr);
  if(en == 0xFF)
    en = 0; // если ничего не записано - считаем, что API выключено

  return en ? true : false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHttpApiEnabled(bool val)
{
  uint16_t addr = HTTP_API_KEY_ADDRESS + 34;
  MemWrite(addr,val ? 1 : 0);
}
//--------------------------------------------------------------------------------------------------------------------------------------
int16_t GlobalSettings::GetTimezone()
{
  int16_t result = 0;
  uint16_t addr = TIMEZONE_ADDRESS;
  
  byte header1 = MemRead(addr++);
  byte header2 = MemRead(addr++);

  if(header1 == SETT_HEADER1 && header2 == SETT_HEADER2)
  {
      byte* b = (byte*) &result;
      *b++ = MemRead(addr++);
      *b++ = MemRead(addr++);

      if(0xFFFF == (uint16_t)result)
        result = 0;
  }

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetTimezone(int16_t val)
{
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
  byte en = MemRead(HTTP_SEND_SENSORS_DATA_ADDRESS);
  if(en == 0xFF)
    en = 1; // если ничего не записано - считаем, что можем отсылать данные

  return en ? true : false;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSendSensorsDataFlag(bool val)
{
   MemWrite(HTTP_SEND_SENSORS_DATA_ADDRESS, val ? 1 : 0); 
}
//--------------------------------------------------------------------------------------------------------------------------------------        
bool GlobalSettings::CanSendControllerStatusToHTTP()
{
  byte en = MemRead(HTTP_SEND_STATUS_ADDRESS);
  if(en == 0xFF)
    en = 1; // если ничего не записано - считаем, что можем отсылать данные

  return en ? true : false;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetSendControllerStatusFlag(bool val)
{
   MemWrite(HTTP_SEND_STATUS_ADDRESS, val ? 1 : 0); 
}
//--------------------------------------------------------------------------------------------------------------------------------------        
String GlobalSettings::GetHttpApiKey()
{
  String result;
  uint16_t addr = HTTP_API_KEY_ADDRESS;
  
  byte header1 = MemRead(addr++);
  byte header2 = MemRead(addr++);

  if(header1 == SETT_HEADER1 && header2 == SETT_HEADER2)
  {
      for(byte i=0;i<32;i++)
      {
        char ch = (char) MemRead(addr++);
        if(ch != '\0')
          result += ch;
        else
          break;
      }
  } // if

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void GlobalSettings::SetHttpApiKey(const char* val)
{
  if(!*val)
    return;

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


