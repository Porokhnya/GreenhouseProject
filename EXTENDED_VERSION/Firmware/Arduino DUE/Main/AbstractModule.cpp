#include "AbstractModule.h"
#include "ModuleController.h"
#ifdef USE_PH_MODULE
#include "PHModule.h"
#endif
#ifdef USE_EC_MODULE
#include "ECModule.h"
#endif
#ifdef USE_TEMP_SENSORS
#include "TempSensors.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(const String& src)
{
  this->Text = src;
  return *this;
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(const char* src)
{
  this->Text = src;
  return *this;
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(char src)
{
  this->Text = src;
  return *this;  
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(const __FlashStringHelper *src)
{
  this->Text = src;
  return *this;    
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(unsigned long src)
{
  this->Text = src;
  return *this;    
  
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(int src)
{
  this->Text = src;
  return *this;      
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator=(long src)
{
  this->Text = src;
  return *this;      
  
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(const String& src)
{
  this->Text += src;
  return *this;
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(const char* src)
{
  this->Text += src;
  return *this;
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(char src)
{
  this->Text += src;
  return *this;  
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(const __FlashStringHelper *src)
{
  this->Text += src;
  return *this;    
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(unsigned long src)
{
  this->Text += src;
  return *this;    
  
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(int src)
{
  this->Text += src;
  return *this;      
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(unsigned int src)
{
  this->Text += src;
  return *this;        
}
//--------------------------------------------------------------------------------------------------------------------------------
PublishStruct& PublishStruct::operator<<(long src)
{
  this->Text += src;
  return *this;      
}
//--------------------------------------------------------------------------------------------------------------------------------
WorkStatus::WorkStatus()
{
  memset(statuses,0,sizeof(uint8_t)*STATUSES_BYTES);
  memset(lastStatuses,0,sizeof(uint8_t)*STATUSES_BYTES);
  memset(&State,0,sizeof(State));
  memset(&UsedPins,0,sizeof(UsedPins));
}
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
void WorkStatus::InitMcpI2CExtenders()
{  
  byte mcp_addresses[] = {MCP23017_ADDRESSES};
  
  for(byte i=0;i<COUNT_OF_MCP23017_EXTENDERS;i++)
  {
    Adafruit_MCP23017* bank = new Adafruit_MCP23017;
    
    bank->begin(mcp_addresses[i]);
    
    mcpI2CExtenders[i] = bank;
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------
Adafruit_MCP23017* WorkStatus::GetMCP_I2C_ByAddress(byte addr)
{
  for(byte i=0;i<COUNT_OF_MCP23017_EXTENDERS;i++)
  {
    Adafruit_MCP23017* bank = mcpI2CExtenders[i];
    if(bank->getAddress() == addr)
      return bank;
  }

  return NULL;  
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::MCP_I2C_PinMode(byte mcpAddress, byte mpcChannel, byte mode)
{
  Adafruit_MCP23017* bank = GetMCP_I2C_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return;

  bank->pinMode(mpcChannel,mode);
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::MCP_I2C_PinWrite(byte mcpAddress, byte mpcChannel, byte level)
{
  Adafruit_MCP23017* bank = GetMCP_I2C_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return; 

  bank->digitalWrite(mpcChannel,level);
}
//--------------------------------------------------------------------------------------------------------------------------------
byte WorkStatus::MCP_I2C_PinRead(byte mcpAddress, byte mpcChannel)
{
  Adafruit_MCP23017* bank = GetMCP_I2C_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return 0xFF; 

  return bank->digitalRead(mpcChannel);
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
void WorkStatus::InitMcpSPIExtenders()
{
  byte mcp_addresses[] = {MCP23S17_ADDRESSES};
  
  for(byte i=0;i<COUNT_OF_MCP23S17_EXTENDERS;i++)
  {
    MCP23S17* bank = new MCP23S17(&SPI,MCP23S17_CS_PIN,mcp_addresses[i]);
    bank->begin();
    mcpSPIExtenders[i] = bank;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::MCP_SPI_PinMode(byte mcpAddress, byte mpcChannel, byte mode)
{
  MCP23S17* bank = GetMCP_SPI_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return;

  bank->pinMode(mpcChannel,mode);
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::MCP_SPI_PinWrite(byte mcpAddress, byte mpcChannel, byte level)
{
  MCP23S17* bank = GetMCP_SPI_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return;  

  bank->digitalWrite(mpcChannel,level);
}
//--------------------------------------------------------------------------------------------------------------------------------
byte WorkStatus::MCP_SPI_PinRead(byte mcpAddress, byte mpcChannel)
{
  MCP23S17* bank = GetMCP_SPI_ByAddress(mcpAddress);
  if(!bank || mpcChannel > 15)
    return 0xFF;  

  return bank->digitalRead(mpcChannel);
}
//--------------------------------------------------------------------------------------------------------------------------------
MCP23S17* WorkStatus::GetMCP_SPI_ByAddress(byte addr)
{
  for(byte i=0;i<COUNT_OF_MCP23S17_EXTENDERS;i++)
  {
    MCP23S17* bank = mcpSPIExtenders[i];
    if(bank->getAddress() == addr)
      return bank;
  }

  return NULL;
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::PinMode(byte pinNumber,byte mode, bool setMode)
{

  uint8_t byte_num = pinNumber/8;
  uint8_t bit_num = pinNumber%8;
  
  if(byte_num < PINS_MAP_SIZE) 
  { // помещаемся
  
      UsedPins.PinsUsed[byte_num] |= (1 << bit_num);  

      byte thisMode = mode;
      if(mode == INPUT_PULLUP)
        thisMode = INPUT;

      UsedPins.PinsMode[byte_num] &= ~(1 << bit_num);
      UsedPins.PinsMode[byte_num] |= (thisMode << bit_num);   
  }
    

   if(setMode && (pinNumber < VIRTUAL_PIN_START_NUMBER))
   {
      pinMode(pinNumber,mode);
   }
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::SaveWindowState(byte channel, byte state)
{
  if(channel > 31)
    return;
    
  // state у нас принимает значения HIGH или LOW, т.е. 0 или 1
  // channel - номер канала, от 0 до 31

  // сперва сбрасываем нужный бит
  State.WindowsState &= ~(1 << channel);

  // теперь, если нам передали не 0 - устанавливаем нужный бит
  if(state == RELAY_ON)
     State.WindowsState |= (1 << channel);
     
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::SaveLightChannelState(byte channel, byte state)
{
  if(channel > 7)
    return;

  // сперва сбрасываем нужный бит
  State.LightChannelsState &= ~(1 << channel);

  // теперь, если нам передали не 0 - устанавливаем нужный бит
  if(state == LIGHT_RELAY_ON)
    State.LightChannelsState |= (1 << channel);  
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::SaveWaterChannelState(byte channel, byte state)
{
  if(channel > 15)
    return;

  // сперва сбрасываем нужный бит
  State.WaterChannelsState &= ~(1 << channel);

  // теперь, если нам передали не 0 - устанавливаем нужный бит
  if(state == WATER_RELAY_ON)
    State.WaterChannelsState |= (1 << channel);
}
//--------------------------------------------------------------------------------------------------------------------------------
byte WorkStatus::PinRead(byte pin)
{
  #ifndef ALERT_DONT_READ_PIN_DIRECT
  
    if(pin < VIRTUAL_PIN_START_NUMBER)
      return digitalRead(pin);
    
  #endif

  uint8_t byte_num = pin/8;
  uint8_t bit_num = pin%8;

  if(byte_num > 15) // не помещаемся
    return LOW;  

  return (State.PinsState[byte_num] & (1 << bit_num)) ? HIGH : LOW;
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::PinWrite(byte pin, byte level)
{
  if(pin < VIRTUAL_PIN_START_NUMBER) // если у нас номер пина меньше, чем номер первого виртуального пина, то - пишем в него
    digitalWrite(pin,level);

  // теперь копируем состояние пина во внутреннюю структуру
  uint8_t byte_num = pin/8;
  uint8_t bit_num = pin%8;
  
  if(byte_num > 15) // не помещаемся
    return;

  // сперва сбрасываем нужный бит
  State.PinsState[byte_num] &= ~(1 << bit_num);

  // теперь, если нам передали не 0 - устанавливаем нужный бит
  if(level)
    State.PinsState[byte_num] |= (1 << bit_num);
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::CopyStatusModes()
{
  CopyStatusMode(WINDOWS_MODE_BIT);
  CopyStatusMode(WATER_MODE_BIT);
  CopyStatusMode(LIGHT_MODE_BIT);
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::CopyStatusMode(uint8_t bitNum)
{
  uint8_t byte_num = bitNum/8;
  uint8_t bit_num = bitNum%8;
  bitWrite(lastStatuses[byte_num],bit_num,bitRead(statuses[byte_num],bit_num));  
}
//--------------------------------------------------------------------------------------------------------------------------------
bool WorkStatus::IsStatusModeChanged(uint8_t bitNum)
{
  uint8_t byte_num = bitNum/8;
  uint8_t bit_num = bitNum%8;

  return bitRead(statuses[byte_num],bit_num) != bitRead(lastStatuses[byte_num],bit_num);
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::SetStatus(uint8_t bitNum, bool bOn)
{    
  uint8_t byte_num = bitNum/8;
  uint8_t bit_num = bitNum%8;

  bitWrite(statuses[byte_num],bit_num,(bOn ? 1 : 0));
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::SetModeUnchanged()
{
  memcpy(lastStatuses,statuses,sizeof(statuses));
  SetStatus(WINDOWS_POS_CHANGED_BIT,0);
}
//--------------------------------------------------------------------------------------------------------------------------------
bool WorkStatus::IsModeChanged()
{
  return IsStatusModeChanged(WINDOWS_MODE_BIT) || IsStatusModeChanged(WATER_MODE_BIT) || IsStatusModeChanged(LIGHT_MODE_BIT) ||
  GetStatus(WINDOWS_POS_CHANGED_BIT);
}
//--------------------------------------------------------------------------------------------------------------------------------
bool WorkStatus::GetStatus(uint8_t bitNum)
{
  uint8_t byte_num = bitNum/8;
  uint8_t bit_num = bitNum%8;
  return bitRead(statuses[byte_num],bit_num) ? true : false; 
}
//--------------------------------------------------------------------------------------------------------------------------------
byte WorkStatus::MakeNum(char ch) 
{
  if((ch >= '0') && (ch <= '9'))
    return ((byte) ch) - '0';
  
  switch(ch) 
  {
    case 'A':
    case 'a': return 10;
    
    case 'B':
    case 'b': return 11;
    
    case 'C':
    case 'c': return 12;
    
    case 'D':
    case 'd': return 13;

    case 'E':
    case 'e': return 14;
    
    case 'F':
    case 'f': return 15;
    
    default: return 16;
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
byte WorkStatus::FromHex(const char* buff)
{
  byte tens = WorkStatus::MakeNum(*buff++);
  byte ones = WorkStatus::MakeNum(*buff);

   if(ones == 'X') 
    return  0;
    
  return  (tens * 16) + ones; 
}
//--------------------------------------------------------------------------------------------------------------------------------
const char HEX_CHARS[]  PROGMEM = {"0123456789ABCDEF"};
//--------------------------------------------------------------------------------------------------------------------------------
const char* WorkStatus::ToHex(int i)
{  

  static char WORK_STATUS_HEX_HOLDER[3] = {0}; // глобальный холдер для шестнадцатеричного представления байта в строковом виде
  
 // String Out;
  int idx = i & 0xF;
  char char1 = (char) pgm_read_byte_near( HEX_CHARS + idx );
  i>>=4;
  idx = i & 0xF;
  char char2 = (char) pgm_read_byte_near( HEX_CHARS + idx );
  //Out = String(char2); Out += String(char1);
  
  WORK_STATUS_HEX_HOLDER[0] = char2;
  WORK_STATUS_HEX_HOLDER[1] = char1;
  
  //return Out; 
  return WORK_STATUS_HEX_HOLDER;
}
//--------------------------------------------------------------------------------------------------------------------------------
void WorkStatus::WriteStatus(Stream* pStream, bool bAsTextHex)
{
  if(!pStream)
    return;
    
  for(uint8_t i=0;i<STATUSES_BYTES;i++)
  {
    if(!bAsTextHex)
      pStream->write(statuses[i]);
    else
    {
      pStream->print(WorkStatus::ToHex(statuses[i]));
    }
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------
WorkStatus WORK_STATUS; // экземпляр класса состояний
//--------------------------------------------------------------------------------------------------------------------------------
void OneState::Update(void* newData) // обновляем внутреннее состояние
{
     switch(Type)
    {
      
      case StateTemperature:
      case StateHumidity: // и для влажности используем структуру температуры
      case StateSoilMoisture: // и для влажности почвы используем структуру температуры
      case StatePH: // и для pH  используем структуру температуры
      {
        Temperature* t1 = (Temperature*) Data;
        Temperature* t2 = (Temperature*) PreviousData;

        *t2 = *t1; // сохраняем предыдущую температуру

        Temperature* tNew = (Temperature*) newData;

        #if defined(USE_PH_MODULE) || defined(USE_EC_MODULE)
          if(Type == StatePH)
          {
            // тут поправляем значение pH в зависимости от калибровочных коэффициентов
            #if defined(USE_PH_MODULE)
              PHCalculation.ApplyCalculation(tNew);
            #else
              EC_PH_Calculation.ApplyCalculation(tNew);
            #endif
              
          }
        #endif // USE_PH_MODULE
        
        *t1 = *tNew; // пишем новую
      } 
      break;

      case StateLuminosity:
      {
        long*  ui1 = (long*) Data;
        long*  ui2 = (long*) PreviousData;

        *ui2 = *ui1; // сохраняем предыдущее состояние освещенности

        long* newState = (long*) newData;
        *ui1 = *newState; // пишем новое состояние освещенности
      } 
      break;

      case StateCO2:
      case StateEC:
      {
        uint16_t*  ui1 = (uint16_t*) Data;
        uint16_t*  ui2 = (uint16_t*) PreviousData;

        *ui2 = *ui1; // сохраняем предыдущее состояние CO2

        uint16_t* newState = (uint16_t*) newData;
        *ui1 = *newState; // пишем новое состояние CO2

      } 
      break;

      case StateWaterFlowInstant: // работаем с датчиками расхода воды
      case StateWaterFlowIncremental:
      {
        unsigned long*  ui1 = (unsigned long*) Data;
        unsigned long*  ui2 = (unsigned long*) PreviousData;

        *ui2 = *ui1; // сохраняем предыдущее состояние расхода воды

        unsigned long* newState = (unsigned long*) newData;
        *ui1 = *newState; // пишем новое состояние расхода воды
        
      }
      break;

      case StateUnknown:
      break;
      
    } // switch
 
}
//--------------------------------------------------------------------------------------------------------------------------------
void OneState::Init(ModuleStates state, uint8_t idx)
{
    Type = state;
    Index = idx;

    switch(state)
    {
      case StateTemperature:
      case StateHumidity: // и для влажности используем структуру температуры
      case StateSoilMoisture: // и для влажности почвы используем структуру температуры
      case StatePH: // и для pH  используем структуру температуры
      {
      
        Temperature* t1 = new Temperature;
        Temperature* t2 = new Temperature;
        
        Data = t1;
        PreviousData = t2;
      }
        
      break;

      case StateLuminosity:
      {
        long*  ui1 = new long;
        long*  ui2 = new long;

        *ui1 = NO_LUMINOSITY_DATA; // нет данных об освещенности
        *ui2 = NO_LUMINOSITY_DATA;
        
        Data = ui1;
        PreviousData = ui2;
      }
      break;

      case StateCO2:
      {
        uint16_t*  ui1 = new uint16_t;
        uint16_t*  ui2 = new uint16_t;

        *ui1 = NO_CO2_DATA; // нет данных о CO2
        *ui2 = NO_CO2_DATA;
        
        Data = ui1;
        PreviousData = ui2;
      }
      break;

      case StateEC:
      {
        uint16_t*  ui1 = new uint16_t;
        uint16_t*  ui2 = new uint16_t;

        *ui1 = NO_EC_DATA; // нет данных о EC
        *ui2 = NO_EC_DATA;
        
        Data = ui1;
        PreviousData = ui2;
      }
      break;

      case StateWaterFlowInstant:
      case StateWaterFlowIncremental:
      {
        unsigned long*  ui1 = new unsigned long;
        unsigned long*  ui2 = new unsigned long;

        *ui1 = 0; // нет данных о расходе воды
        *ui2 = 0;
        
        Data = ui1;
        PreviousData = ui2;
        
      }
      break;

      case StateUnknown:
      break;
    } // switch
  
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator String() // выводим текущие значения как строку
{
    switch(Type)
    {
      case StateTemperature:
      case StateHumidity: // и для влажности используем структуру температуры
      case StateSoilMoisture: // и для влажности почвы используем структуру температуры
      case StatePH: // и для pH  используем структуру температуры
      {
      
        Temperature* t1 = (Temperature*) Data;
        return *t1;
      }
        
      case StateLuminosity:
      {
        long*  ul1 = (long*) Data;
        return String(*ul1);
      }

      case StateCO2:
      case StateEC:
      {
        uint16_t*  ul1 = (uint16_t*) Data;
        return String(*ul1);
      }

      case StateWaterFlowInstant:
      case StateWaterFlowIncremental:
      {
        unsigned long*  ul1 = (unsigned long*) Data;
        return String(*ul1);        
      }

      case StateUnknown:
        return String();
    } // switch

    return String();
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState& OneState::operator=(const OneState& rhs)
{

  if(this == &rhs)
    return *this;

  if(Type != rhs.Type)
  {
    return *this;
  }

      switch(Type)
      {
        case StateTemperature:
        case StateHumidity: // и для влажности используем структуру температуры
        case StateSoilMoisture: // и для влажности почвы используем структуру температуры
        case StatePH: // и для pH  используем структуру температуры
        {
          Temperature* rhs_t1 = (Temperature*) rhs.Data;
          Temperature* rhs_t2 = (Temperature*) rhs.PreviousData;

          Temperature* this_t1 = (Temperature*) Data;
          Temperature* this_t2 = (Temperature*) PreviousData;

          *this_t1 = *rhs_t1;
          *this_t2 = *rhs_t2;
          
        }
        break;

        case StateLuminosity:
        {
          long*  rhs_ui1 = (long*) rhs.Data;
          long*  rhs_ui2 = (long*) rhs.PreviousData;
  
          long*  this_ui1 = (long*) Data;
          long*  this_ui2 = (long*) PreviousData;

          *this_ui1 = *rhs_ui1;
          *this_ui2 = *rhs_ui2;
        }  
        break;

        case StateCO2:
        case StateEC:
        {
          uint16_t*  rhs_ui1 = (uint16_t*) rhs.Data;
          uint16_t*  rhs_ui2 = (uint16_t*) rhs.PreviousData;
  
          uint16_t*  this_ui1 = (uint16_t*) Data;
          uint16_t*  this_ui2 = (uint16_t*) PreviousData;

          *this_ui1 = *rhs_ui1;
          *this_ui2 = *rhs_ui2;
        }  
        break;

        case StateWaterFlowInstant:
        case StateWaterFlowIncremental:
        {
          unsigned long*  rhs_ui1 = (unsigned long*) rhs.Data;
          unsigned long*  rhs_ui2 = (unsigned long*) rhs.PreviousData;
  
          unsigned long*  this_ui1 = (unsigned long*) Data;
          unsigned long*  this_ui2 = (unsigned long*) PreviousData;

          *this_ui1 = *rhs_ui1;
          *this_ui2 = *rhs_ui2;
        }  
        break;

        case StateUnknown:
        break;
      
      } // switch
  

  return *this;
}
//--------------------------------------------------------------------------------------------------------------------------------
bool OneState::IsChanged()
{
      switch(Type)
      {
        case StateTemperature:
        case StateHumidity: // и для влажности используем структуру температуры
        case StateSoilMoisture: // и для влажности почвы используем структуру температуры
        case StatePH: // и для pH  используем структуру температуры
        {
          Temperature* t1 = (Temperature*) Data;
          Temperature* t2 = (Temperature*) PreviousData;

          if(*t1 != *t2)
            return true; // температура изменилась
        }
        break;


        case StateLuminosity:
        {
          long*  ui1 = (long*) Data;
          long*  ui2 = (long*) PreviousData;
  
         if(*ui1 != *ui2)
          return true; // состояние освещенности изменилось
        }  
        break;

        case StateCO2:
        case StateEC:
        {
          uint16_t*  ui1 = (uint16_t*) Data;
          uint16_t*  ui2 = (uint16_t*) PreviousData;
  
         if(*ui1 != *ui2)
          return true; // состояние CO2 изменилось          
        }  
        break;


        case StateWaterFlowInstant:
        case StateWaterFlowIncremental:
        {
          unsigned long*  ui1 = (unsigned long*) Data;
          unsigned long*  ui2 = (unsigned long*) PreviousData;
  
         if(*ui1 != *ui2)
          return true; // состояние освещенности изменилось
        }  
        break;

        case StateUnknown:
          return false;

      
      } // switch

 return false;
  
}
//--------------------------------------------------------------------------------------------------------------------------------
ModuleStates OneState::GetType(const String& stringType)
{
  return GetType(stringType.c_str());
}
//--------------------------------------------------------------------------------------------------------------------------------
String OneState::GetUnit()
{
 switch(Type)
  {
    case StateUnknown:
      return String();

    case StateTemperature:
    #ifdef MEASURE_TEMPERATURES_IN_FAHRENHEIT
      return UNIT_FAHRENHEIT;
    #else
      return UNIT_CELSIUS;
    #endif

    case StateHumidity:
    case StateSoilMoisture:
      return F("%");
    
    case StatePH:
      return UNIT_PH;

    case StateLuminosity:
      return UNIT_LUX;

    case StateCO2:
      return UNIT_PPM;

    case StateEC:
      return UNIT_EC;

    case StateWaterFlowIncremental:
    case StateWaterFlowInstant:
      return  UNIT_LITRES;
      
  } 

    return String();
}
//--------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------
void OneState::Reset()
{
 switch(Type)
    {
      
      case StateTemperature:
      case StateHumidity: // и для влажности используем структуру температуры
      case StateSoilMoisture: // и для влажности почвы используем структуру температуры
      case StatePH: // и для pH  используем структуру температуры
      {
        Temperature* t1 = (Temperature*) Data;
        t1->Value = NO_TEMPERATURE_DATA;
       
      } 
      break;

      case StateLuminosity:
      {
        long*  ui1 = (long*) Data;
        *ui1 = NO_LUMINOSITY_DATA;
      } 
      break;

      case StateCO2:
      {
        uint16_t*  ui1 = (uint16_t*) Data;
        *ui1 = NO_CO2_DATA;
      } 
      break;

      case StateEC:
      {
        uint16_t*  ui1 = (uint16_t*) Data;
        *ui1 = NO_EC_DATA;
      } 
      break;

      case StateWaterFlowInstant: // работаем с датчиками расхода воды
      case StateWaterFlowIncremental:
      {
        unsigned long*  ui1 = (unsigned long*) Data;
        *ui1 = 0;
      }
      break;

      case StateUnknown:
      break;
      
    } // switch  
}
//--------------------------------------------------------------------------------------------------------------------------------
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------
bool OneState::HasData()
{
   switch(Type)
  {
    case StateUnknown:
      return false;

    // для всех структур ниже мы используем одну структуру
    case StateTemperature:
    case StateHumidity:
    case StatePH:
    case StateSoilMoisture:
    {
      Temperature* t = (Temperature*) Data;
      return t->HasData();
    }

    case StateLuminosity:
    {
      long*  ui1 = (long*) Data;
      return *ui1 != NO_LUMINOSITY_DATA;
    }

    case StateCO2:
    {
      uint16_t*  ui1 = (uint16_t*) Data;
      return *ui1 != NO_CO2_DATA;
    }

    case StateEC:
    {
      uint16_t*  ui1 = (uint16_t*) Data;
      return *ui1 != NO_EC_DATA;
    }

    // для датчиков расхода воды считаем,
    // что показания есть всегда.
    case StateWaterFlowIncremental:
    case StateWaterFlowInstant:
        return true;
  } 

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------
uint8_t OneState::GetRawData(byte* outBuffer)
{
  switch(Type)
  {
    case StateUnknown:
      return 0;

    case StateTemperature:
    case StateHumidity:
    case StateSoilMoisture:
    {
        Temperature* t = (Temperature*) Data;
        *outBuffer++ = t->Fract;
        *outBuffer = t->Value;
      return 2;
    }

    case StatePH: // для датчика pH мы теперь возвращаем ещё и подсчитанный вольтаж во вторых двух байтах
    {
        Temperature* t = (Temperature*) Data;

        uint16_t phMV = 0;
        unsigned long curPH = 0;

        if(HasData())
        {
          // есть данные
          #ifdef PH_REVERSIVE_MEASURE
            // реверсивное измерение pH
            curPH  = t->Value*100 + t->Fract;
            int16_t diff = (700 - curPH);
            curPH = 700 + diff;
          #else
            // прямое измерение pH 
            curPH  = t->Value*100 + t->Fract;
          #endif
          
          phMV = (PH_MV_PER_7_PH*curPH)/700; // получаем милливольты          
        }

        // пишем в первые два байта значение вольтажа, поскольку у нас сырой массив - пишем в обратной последовательности всё
        byte* rdPtr = (byte*) &phMV;
        *outBuffer++ = *rdPtr++;
        *outBuffer++ = *rdPtr;

        // потом пишем показания датчика
        *outBuffer++ = t->Fract;
        *outBuffer++ = t->Value;
         
      return 4;
    }

    // для освещённости пишем два байта в сырые данные
    case StateLuminosity:
    {
      long* lum = (long*) Data;
      memcpy(outBuffer,lum,2);
      return 2;
    }

    // для CO2 пишем два байта в сырые данные
    case StateCO2:
    case StateEC:
    {
      uint16_t* lum = (uint16_t*) Data;
      memcpy(outBuffer,lum,2);
      return 2;
    }

    case StateWaterFlowInstant:
    case StateWaterFlowIncremental:
    {
      unsigned long* flow = (unsigned long*) Data;
      memcpy(outBuffer,flow,sizeof(unsigned long));
      return sizeof(unsigned long);
    }
    
    
  }
  return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
String OneState::GetStringType(ModuleStates type)
{
  switch(type)
  {
    case StateUnknown:
      return PROP_NONE;

    case StateTemperature:
      return PROP_TEMP;

    case StateHumidity:
      return PROP_HUMIDITY;

    case StateLuminosity:
      return PROP_LIGHT;

    case StateCO2:
      return PROP_CO2;

    case StateEC:
      return PROP_EC;

    case StateSoilMoisture:
      return PROP_SOIL;

    case StatePH:
      return PROP_PH;

    case StateWaterFlowIncremental:
      return PROP_FLOW_INCREMENTAL;

    case StateWaterFlowInstant:
      return PROP_FLOW_INSTANT;
  }

  return PROP_NONE;
}
//--------------------------------------------------------------------------------------------------------------------------------
ModuleStates OneState::GetType(const char* stringType)
{
  if(!strcmp_P(stringType, (const char*) PROP_TEMP))
    return StateTemperature;
    
  if(!strcmp_P(stringType, (const char*) PROP_HUMIDITY))
    return StateHumidity;

  if(!strcmp_P(stringType, (const char*) PROP_LIGHT))
    return StateLuminosity;

  if(!strcmp_P(stringType, (const char*) PROP_CO2))
    return StateCO2;

  if(!strcmp_P(stringType, (const char*) PROP_EC))
    return StateEC;

  if(!strcmp_P(stringType, (const char*) PROP_SOIL))
    return StateSoilMoisture;

  if(!strcmp_P(stringType, (const char*) PROP_PH))
    return StatePH;

  if(!strcmp_P(stringType, (const char*) PROP_FLOW_INCREMENTAL))
    return StateWaterFlowIncremental;

  if(!strcmp_P(stringType, (const char*) PROP_FLOW_INSTANT))
    return StateWaterFlowInstant;

  return StateUnknown;
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::~OneState()
{
  // подчищаем за собой
  
       switch(Type)
      {
        case StateTemperature:
        case StateHumidity: // и для влажности используем структуру температуры
        case StateSoilMoisture: // и для влажности почвы используем структуру температуры
        case StatePH: // и для pH  используем структуру температуры
        {
          Temperature* t1 = (Temperature*) Data;
          Temperature* t2 = (Temperature*) PreviousData;

          delete t1;
          delete t2;
        }
        break;

        case StateLuminosity:
        {
          long*  ui1 = (long*) Data;
          long*  ui2 = (long*) PreviousData;
  
          delete ui1;
          delete ui2;
        }  
        break;

        case StateCO2:
        case StateEC:
        {
          uint16_t*  ui1 = (uint16_t*) Data;
          uint16_t*  ui2 = (uint16_t*) PreviousData;
  
          delete ui1;
          delete ui2;
        }  
        break;


        case StateWaterFlowInstant:
        case StateWaterFlowIncremental:
        {
          unsigned long*  ui1 = (unsigned long*) Data;
          unsigned long*  ui2 = (unsigned long*) PreviousData;
  
          delete ui1;
          delete ui2;
        }  
        break;

        case StateUnknown:
        break;
      
      } // switch
 
  
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator HumidityPair()
{
  if(!(Type == StateHumidity || Type == StateSoilMoisture || Type == StatePH)) // влажность можно получить только для трёх типов датчиков
  {
    return HumidityPair(Humidity(),Humidity()); // undefined behaviour
  }

    return HumidityPair(*((Humidity*)PreviousData),*((Humidity*)Data));  
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator TemperaturePair()
{
  if(Type != StateTemperature)
  {
    return TemperaturePair(Temperature(),Temperature()); // undefined behaviour
  }

    return TemperaturePair(*((Temperature*)PreviousData),*((Temperature*)Data));
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator LuminosityPair()
{
  if(Type != StateLuminosity)
  {
    return LuminosityPair(0,0); // undefined behaviour
  }
  return LuminosityPair(*((long*)PreviousData),*((long*)Data));   
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator CO2Pair()
{
  if(Type != StateCO2)
  {
    return CO2Pair(0,0); // undefined behaviour
  }
  return CO2Pair(*((uint16_t*)PreviousData),*((uint16_t*)Data));   
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator ECPair()
{
  if(Type != StateEC)
  {
    return ECPair(0,0); // undefined behaviour
  }
  return ECPair(*((uint16_t*)PreviousData),*((uint16_t*)Data));   
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState::operator WaterFlowPair()
{
  if(!(Type == StateWaterFlowInstant || Type == StateWaterFlowIncremental))
  {
    return WaterFlowPair(0,0); // undefined behaviour
  }
  return WaterFlowPair(*((unsigned long*)PreviousData),*((unsigned long*)Data));   
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState operator-(const OneState& left, const OneState& right)
{
  OneState result(left.Type,left.Index); // инициализируем

  if(left.Type != right.Type)
  {
    return result; // undefined behaviour
  }
  
      switch(left.Type)
      {
        case StateTemperature:
        case StateHumidity: // и для влажности используем структуру температуры
        case StateSoilMoisture: // и для влажности почвы используем структуру температуры
        case StatePH: // и для pH  используем структуру температуры
        {
          Temperature* t1 = (Temperature*) left.Data;
          Temperature* t2 = (Temperature*) right.Data;


          Temperature* thisT = (Temperature*) result.Data;
          if(t1->Value != NO_TEMPERATURE_DATA && t2->Value != NO_TEMPERATURE_DATA) // только если есть показания с датчиков
              *thisT = (*t1 - *t2); // получаем дельту текущих изменений
          
          t1 = (Temperature*) left.PreviousData;
          t2 = (Temperature*) right.PreviousData;

          thisT = (Temperature*) result.PreviousData;
          if(t1->Value != NO_TEMPERATURE_DATA && t2->Value != NO_TEMPERATURE_DATA) // только если есть показания с датчиков
              *thisT = (*t1 - *t2); // получаем дельту предыдущих изменений
        
        }
        break;

        case StateLuminosity:
        {
          long*  ui1 = (long*) left.Data;
          long*  ui2 = (long*) right.Data;

          long* thisLong = (long*) result.Data;

          // получаем дельту текущих изменений
          if(*ui1 != NO_LUMINOSITY_DATA && *ui2 != NO_LUMINOSITY_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));

          ui1 = (long*) left.PreviousData;
          ui2 = (long*) right.PreviousData;

          thisLong = (long*) result.PreviousData;

          // получаем дельту предыдущих изменений
          if(*ui1 != NO_LUMINOSITY_DATA && *ui2 != NO_LUMINOSITY_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));   
        }  
        break;

        case StateCO2:
        {
          uint16_t*  ui1 = (uint16_t*) left.Data;
          uint16_t*  ui2 = (uint16_t*) right.Data;

          uint16_t* thisLong = (uint16_t*) result.Data;

          // получаем дельту текущих изменений
          if(*ui1 != NO_CO2_DATA && *ui2 != NO_CO2_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));

          ui1 = (uint16_t*) left.PreviousData;
          ui2 = (uint16_t*) right.PreviousData;

          thisLong = (uint16_t*) result.PreviousData;

          // получаем дельту предыдущих изменений
          if(*ui1 != NO_CO2_DATA && *ui2 != NO_CO2_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));   
        }  
        break;

        case StateEC:
        {
          uint16_t*  ui1 = (uint16_t*) left.Data;
          uint16_t*  ui2 = (uint16_t*) right.Data;

          uint16_t* thisLong = (uint16_t*) result.Data;

          // получаем дельту текущих изменений
          if(*ui1 != NO_EC_DATA && *ui2 != NO_EC_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));

          ui1 = (uint16_t*) left.PreviousData;
          ui2 = (uint16_t*) right.PreviousData;

          thisLong = (uint16_t*) result.PreviousData;

          // получаем дельту предыдущих изменений
          if(*ui1 != NO_EC_DATA && *ui2 != NO_EC_DATA) // только если есть показания с датчиков
            *thisLong = abs((*ui1 - *ui2));   
        }  
        break;

        case StateWaterFlowInstant:
        case StateWaterFlowIncremental:
        {
          unsigned long*  ui1 = (unsigned long*) left.Data;
          unsigned long*  ui2 = (unsigned long*) right.Data;

          unsigned long* thisUi = (unsigned long*) result.Data;

          // получаем дельту текущих изменений
          *thisUi = abs((*ui1 - *ui2));

          ui1 = (unsigned long*) left.PreviousData;
          ui2 = (unsigned long*) right.PreviousData;

          thisUi = (unsigned long*) result.PreviousData;

          // получаем дельту предыдущих изменений
          *thisUi = abs((*ui1 - *ui2));
  
        }  
        break;

        case StateUnknown:
        break;

        
      } // switch
      
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------
Temperature::Temperature()
{
  Value = NO_TEMPERATURE_DATA;
  Fract = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
Temperature::operator String() const
{
    sprintf_P(SD_BUFFER,(const char*) F("%d,%02u"), Value,Fract);
    return SD_BUFFER;
}
//--------------------------------------------------------------------------------------------------------------------------------
Temperature operator-(const Temperature& left, const Temperature& right) 
{
  
  if(left.Value == NO_TEMPERATURE_DATA || right.Value == NO_TEMPERATURE_DATA)
  {
    // разница температур, когда нет показаний на одном из датчиков - всегда имеет значение NO_TEMPERATURE_DATA
    return Temperature(NO_TEMPERATURE_DATA,0);
  }  

// получаем разницу двух температур
  int8_t sign1 = 1;
  int8_t sign2 = 1;
  
  if(left.Value < 0) // первая температура отрицательная
    sign1 = -1;
  
  if(right.Value < 0) // вторая температура отрицательная
    sign2 = -1;

  // получаем абсолютные значения температур, с учётом сотых, и умножаем их на знак. 
  // Знак сбрасываем, чтобы правильно сконвертировать в целое число, поскольку, если температура
  // отрицательная - то Fract её прибавит: (Fract = 5, Value = -10: -10*100 + 5 = -995 (!) вместо -1005.)
  long lVal = (abs(left.Value)*100 + left.Fract)*sign1;
  long rVal = (abs(right.Value*100) + right.Fract)*sign2;


  long res = abs((lVal - rVal));
  
    return Temperature(res/100, res%100); // дельта у нас всегда положительная.
}
//--------------------------------------------------------------------------------------------------------------------------------
ModuleState::ModuleState() : supportedStates(0)
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------
bool ModuleState::HasState(ModuleStates state)
{
  return ( (supportedStates & state) == state);
}
//--------------------------------------------------------------------------------------------------------------------------------
void ModuleState::RemoveState(ModuleStates state, uint8_t idx)
{
  size_t cnt = states.size();
  for(size_t i=0;i<cnt;i++)
  {
    OneState* os = states[i];
    if(os->GetType() == state && os->GetIndex() == idx)
    {
      // нашли нужное состояние, удаляем его
      delete os;
      // теперь сдвигаем на пустое место
      size_t wIdx = i;
      while(wIdx < cnt-1)
      {
       states[wIdx] = states[wIdx+1]; 
       wIdx++;
      } // while
      // удаляем последний элемент (по сути, внутри вектора просто сдвинется указатель записи, и всё).
      states.pop();

      break; // выходим из цикла
    } // if
  } // for

  // теперь проверяем - если больше нет такого состояния - обнуляем его флаг.
  if(!HasState(state)) // нет такого состояния
    supportedStates &= ~state; // инвертируем все биты в state, кроме выставленного, и применяем эту маску к supportedStates. 
    // В результате в supportedStates очистятся только те биты, которые были выставлены в state.
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState* ModuleState::AddState(ModuleStates state, uint8_t idx)
{
    supportedStates |= state;
    OneState* s = new OneState(state,idx);
    states.push_back(s); // сохраняем состояние
    
    return s;
}
//--------------------------------------------------------------------------------------------------------------------------------
bool ModuleState::HasChanges()
{
  size_t sz = states.size();
  for(size_t i=0;i<sz;i++)
  {
      OneState* s = states[i];

      if(s->IsChanged())
        return true;

  } // for

  return false;
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void ModuleState::UpdateState(ModuleStates state, uint8_t idx, void* newData)
{
  size_t sz = states.size();
  for(size_t i=0;i<sz;i++)
  {
      OneState* s = states[i];
      if(s->GetType() == state && s->GetIndex() == idx)
      {
        s->Update(newData);
        return;
      } // if
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------
uint8_t ModuleState::GetStateCount(ModuleStates state)
{
  uint8_t result = 0;
  size_t sz = states.size();
  
  for(size_t i=0;i<sz;i++)
  {
      OneState* s = states[i];
      if(s->GetType() == state)
        result++;
  }
  
  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState* ModuleState::GetStateByOrder(ModuleStates state, uint8_t orderNum)
{
  size_t sz = states.size();
  uint8_t cntr = 0;
  for(size_t i=0;i<sz;i++)
  {
      OneState* s = states[i];
      if(s->GetType() == state)
      {
        if(cntr == orderNum)
          return s;

          cntr++;
      }
  }

    return NULL;  
}
//--------------------------------------------------------------------------------------------------------------------------------
OneState* ModuleState::GetState(ModuleStates state, uint8_t idx)
{
  size_t sz = states.size();
  for(size_t i=0;i<sz;i++)
  {
      OneState* s = states[i];
      if(s->GetType() == state && s->GetIndex() == idx)
        return s;
  }

    return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------
char SD_BUFFER[SD_BUFFER_LENGTH] = {0};
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_FEEDBACK_MANAGER
//--------------------------------------------------------------------------------------------------------------------------------
FeedbacksManager::FeedbacksManager()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void FeedbacksManager::Setup()
{
  #ifdef USE_TEMP_SENSORS
  waitingWindowsFeedbackTimer = 0;
  windowFeedbackReceivedFlags = 0;
  #endif
  
  flags.inWaitingWindowsFeedbackMode = true;
  flags.isAnyWindowsFeedbackReceived = false;
  flags.isFirstCallOfWindowsFeedback = true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void FeedbacksManager::Update()
{
  #ifdef USE_TEMP_SENSORS
  
    static uint32_t _timer = 0;
    uint32_t now = millis();
    uint32_t dt = now - _timer;
    _timer = now;
	
    if(flags.inWaitingWindowsFeedbackMode)
    {
      waitingWindowsFeedbackTimer += dt;
      if(waitingWindowsFeedbackTimer > FEEDBACK_MANAGER_WAIT_TIME)
      {
        // таймаут получения обратной связи
        WindowFeedbackDone();
      }
    }
  #endif // USE_TEMP_SENSORS
}
//--------------------------------------------------------------------------------------------------------------------------------
void FeedbacksManager::WindowFeedbackDone()
{
  // эту функцию вызывают, когда пришла (или не пришла) информация по обратной связи от окон.
  // Здесь мы просто сбрасываем таймер, и вызываем функцию закрытия окон, если ни один пакет
  // по обратной связи канала окна не был обработан
  
    flags.inWaitingWindowsFeedbackMode = false;
    
    if(!flags.isAnyWindowsFeedbackReceived) // если не получили ни одного пакета 
    {
      if(flags.isFirstCallOfWindowsFeedback) // и первый раз вызвали эту функцию
      {
        // то надо принудительно закрыть окна при стартке
        flags.isFirstCallOfWindowsFeedback = false;
        
        #ifdef USE_TEMP_SENSORS
          //Serial.println(F("No feedback, close windows..."));
          WindowModule->CloseAllWindows();
        #endif
      }
    }
    else
    {
       // получили хотя бы один пакет информации по обратной связи. Надо закрыть те окна,
       // которые не получали информацию по обратной связи.
       if(flags.isFirstCallOfWindowsFeedback) // и первый раз вызвали эту функцию
       {
          flags.isFirstCallOfWindowsFeedback = false;
          #ifdef USE_TEMP_SENSORS
            for(int i=0;i<SUPPORTED_WINDOWS;i++)
            {
              if(!bitRead(windowFeedbackReceivedFlags, i))
              {
                // для окна не получена обратная связь
                WindowModule->CloseWindow(i);
              }
            }
          #endif
       }
    }
}
//--------------------------------------------------------------------------------------------------------------------------------
void FeedbacksManager::WindowFeedback(uint8_t windowNumber, bool isCloseSwitchTriggered, bool isOpenSwitchTriggered, bool hasPosition, uint8_t positionPercents)
{
  #ifdef USE_TEMP_SENSORS
  
    flags.isAnyWindowsFeedbackReceived = true; // говорим, что получили инфу по обратной связи по крайней мере для одного окна
    windowFeedbackReceivedFlags |= (1 << windowNumber);
    
    WindowModule->WindowFeedback(windowNumber,isCloseSwitchTriggered,isOpenSwitchTriggered,hasPosition,positionPercents, flags.inWaitingWindowsFeedbackMode);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------
bool FeedbacksManager::IsWaitingForFirstWindowsFeedback()
{
  #ifdef USE_TEMP_SENSORS
    return flags.inWaitingWindowsFeedbackMode;
  #else
    return false;
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------
FeedbacksManager FeedbackManager;
//--------------------------------------------------------------------------------------------------------------------------------
#endif // USE_FEEDBACK_MANAGER
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(_DEBUG) || defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE) || defined(LOGGING_DEBUG_MODE) || defined(WINDOW_MANAGE_DEBUG) || defined(WATER_DEBUG) || defined(NRF_DEBUG) || defined(RS485_DEBUG) || defined(UNI_DEBUG) || defined(PH_DEBUG) || defined(IOT_DEBUG) || defined(HTTP_DEBUG) || defined(MQTT_DEBUG) || defined(LORA_DEBUG)  || defined(DS18B20_DEBUG) || defined(POWER_DEBUG)

  #if defined(USE_WIFI_MODULE) || defined(USE_SMS_MODULE)
  #include "CoreTransport.h"
  #endif

void DebugLog(const String& str)
{
  for(size_t i=0;i<str.length();i++)
  {
    #ifdef USE_WIFI_MODULE
      ESP.readFromStream();
    #endif

   #ifdef USE_SMS_MODULE
    SIM800.readFromStream();
   #endif     
    
    Serial.write(str[i]);
    Serial.flush(); // принудительно передаём все данные
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------
#endif

