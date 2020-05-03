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

    //TODO: ТУТ РАЗБОР ПАКЕТА !!!

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

