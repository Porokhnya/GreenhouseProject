//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include "CONFIG.h"
#include "Settings.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Прошивка универсального исполнительного модуля, получающего состояние контроллера приёмом широковещательных пакетов
// Прошивка предназначена для закачки в ESP8266, например, в плату NodeMCU
// ВСЕ НАСТРОЙКИ - ВВЕРХУ ФАЙЛА CONFIG.H !!!
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// дальше лазить - неосмотрительно :)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// \/
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool wifiConnected = false;
WiFiUDP Udp;
SlotSettings slots[] = { SLOTS };
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setupSlots()
{
  size_t cnt = sizeof(slots)/sizeof(SlotSettings);
  
  for(size_t i=0;i<cnt;i++)
  {
    if(slots[i].type == NO_BINDING) // нет привязки
    {
      continue;
    }

    pinMode(slots[i].pin,OUTPUT);
    digitalWrite(slots[i].pin, !(POWER_ON_LEVEL));
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t MakeNum(char ch) 
{
  if((ch >= '0') && (ch <= '9'))
    return ((uint8_t) ch) - '0';
  
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
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint8_t FromHex(const char* buff)
{
  uint8_t tens = MakeNum(*buff++);
  uint8_t ones = MakeNum(*buff);

   if(ones == 'X') 
    return  0;
    
  return  (tens * 16) + ones; 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void dealWithPacket(const char* packet)
{
  // разбираем пакет, превращая его в структуру ControllerState
  
  ControllerState state;
  uint8_t* pState = (uint8_t*)&state;

  // поскольку у нас строка вида F0DEFE18..., то нам надо каждый байт раскодировать из двух символов, и записать последовательно в структуру
  
  for(size_t i=0;i<sizeof(ControllerState);i++)
  {
    uint8_t bVal = FromHex(packet);
    *pState = bVal;
    pState++;
    packet++; if(!*packet) { break; }  // перемещаемся на следующий байт
    packet++; if(!*packet) { break; }  // перемещаемся на следующий байт
  }

  // структура сформирована, можем с ней работать
  size_t cnt = sizeof(slots)/sizeof(SlotSettings);

  // пробегаем по всем слотам
  for(size_t i=0;i<cnt;i++)
  {
    if(slots[i].type == NO_BINDING) // нет привязки
    {
      continue;
    }

      // смотрим тип привязки
      switch(slots[i].type)
      {
        case WINDOW:  // канал окна
        {
            uint8_t bit_num = slots[i].link*2;
            if(slots[i].param == RIGHT_CHANNEL)
            {
              bit_num++;
            }

            if(state.WindowsState & (1 << bit_num)) // канал включен
            {
              digitalWrite(slots[i].pin, (POWER_ON_LEVEL));
            }
            else // канал выключен
            {
              digitalWrite(slots[i].pin, !(POWER_ON_LEVEL));
            }            
        }
        break;

        case WATER:   // канал полива
        {
            if(state.WaterChannelsState & (1 << slots[i].link)) // канал включен
            {
              digitalWrite(slots[i].pin, (POWER_ON_LEVEL));
            }
            else // канал выключен
            {
              digitalWrite(slots[i].pin, !(POWER_ON_LEVEL));
            }          
        }
        break;

        case LIGHT:   // канал досветки
        {
            if(state.LightChannelsState & (1 << slots[i].link)) // канал включен
            {
              digitalWrite(slots[i].pin, (POWER_ON_LEVEL));
            }
            else // канал выключен
            {
              digitalWrite(slots[i].pin, !(POWER_ON_LEVEL));
            }
        }
        break;

        case VPIN:  // виртуальный пин
        {
            uint8_t byte_num = slots[i].link/8;
            uint8_t bit_num = slots[i].link%8;

            uint8_t bVal = state.PinsState[byte_num];
            
            if(bVal & (1 << bit_num)) // пин включен
            {
              digitalWrite(slots[i].pin, (POWER_ON_LEVEL));
            }
            else // пин выключен
            {
              digitalWrite(slots[i].pin, !(POWER_ON_LEVEL));
            }
        }
        break;
        
      } // switch

  } // for
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void myDelay(uint32_t msec)
{
  while(msec > 0)
  {
    --msec;
    CommandHandler.handleCommands();
    delay(1);
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void wifiOff()
{
  Udp.stop();  
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void makeConnection()
{

  wifiConnected = WiFi.isConnected();
  
  if(wifiConnected)
  {
    return;
  }
  else
  {
    wifiOff(); 
  }

   WiFi.mode(WIFI_STA);

  String ssid = Settings.getRouterID();
  String pass = Settings.getRouterPassword();
    
  #ifdef _DEBUG
    Serial.print(F("Connecting to \""));
    Serial.print(ssid);
    Serial.print(F("\" with password \""));
    Serial.print(pass);
    Serial.print(F("\" "));
  #endif
  
  WiFi.begin(ssid.c_str(), pass.c_str());

  static uint32_t startConnectTimer = 0;
  startConnectTimer = millis();

  while(1)
  {
    CommandHandler.handleCommands();
    
    if(millis() - startConnectTimer > CONNECT_TIMEOUT)
    {
        #ifdef _DEBUG
          Serial.println();
          Serial.println(F("Connect timeout!"));
        #endif

       wifiOff();
      return;
    }
    int status = WiFi.status();
    switch(status)
    {
      case WL_IDLE_STATUS:
      break;

      case WL_NO_SSID_AVAIL:
      {
        #ifdef _DEBUG
          Serial.println();
          Serial.println(F("No SSID found!"));
        #endif

        wifiOff();
      }
      return;

      case WL_CONNECTED:
      {
        #ifdef _DEBUG
          Serial.println();
          Serial.println(F("Connected!"));
        #endif

         wifiConnected = true;
         Udp.begin(BROADCAST_PORT);
            
      }
      return;

      case WL_CONNECT_FAILED:
      {
        #ifdef _DEBUG
          Serial.println();
          Serial.println(F("Password incorrect!"));
        #endif

        wifiOff();    
      }
      return;   
   
    } // switch
    #ifdef _DEBUG
      Serial.print('.');
    #endif    
    myDelay(500);
  } // while

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void parseIncomingUDPPackets()
{
  if(!WiFi.isConnected())
  {
    return;  
  }

  int packetSize = Udp.parsePacket();
  if(packetSize == sizeof(ControllerState)*2) // у нас всё приходит в строковом HEX-виде
  {
    char* packet = new char[packetSize+1];
    memset(packet,0,packetSize+1);

    Udp.read(packet, packetSize);
    
    #ifdef _DEBUG
      Serial.print(F("UDP PACKET DETECTED: "));
      Serial.println(packet);
    #endif

    dealWithPacket(packet);

    delete [] packet;
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(57600);

  #ifdef _DEBUG
    Serial.println();
    Serial.println(F("Starting..."));
    Serial.setDebugOutput(true);
  #endif
  
  WiFi.setAutoConnect(false);
  WiFi.setAutoReconnect(false);

  Settings.begin();

  setupSlots();

  #ifdef _DEBUG
    Serial.println(F("Started."));
  #endif



}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  CommandHandler.handleCommands();

  makeConnection();
  parseIncomingUDPPackets();

  myDelay(1);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

