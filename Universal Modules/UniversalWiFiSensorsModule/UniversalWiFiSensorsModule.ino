//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <ESP8266WiFi.h>
#include <Wire.h>
#include "CONFIG.h"
#include "DS18B20Query.h"
#include "BH1750.h"
#include "HTU21D.h"
#include "Settings.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Прошивка универсального модуля с датчиками, отсылающего данные на контроллер через ESP.
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
uint32_t lastMillis = 0;
uint32_t updateInterval = UPDATE_INTERVAL;
bool isWireInited = false;
bool isDeepSleepAllowed = 
#ifdef USE_DEEP_SLEEP
true;
#else
false;
#endif
SensorSettings sensors[] = {
  SENSORS
};
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
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
String readFromSensor(SensorSettings* sett)
{
  String result;
  switch(sett->type)
  {
      case sensor_NoSensor:
      break;

     case sensor_Si7021:
     {
        HTU21D* htu = (HTU21D*) sett->data;
        float temperature = htu->readTemperature();
        float humidity = htu->readHumidity();
        result = F("CTSET=HUMIDITY|DATA|");
        result += sett->targetIndex;
        result += '|';

        byte humError = (byte) humidity;
        byte tempError = (byte) temperature;

        if(tempError == HTU21D_ERROR)
        {
         result += NO_TEMPERATURE_DATA;
         result += "00";          
        }
        else
        {
            int iTmp = temperature*100;
            result += iTmp;
        }

        result += '|';

        if(humError == HTU21D_ERROR)
        {
         result += NO_TEMPERATURE_DATA;
         result += "00";          
        }
        else
        {
            int iTmp = humidity*100;
            result += iTmp;
        }
        
     }
     return result;

     case sensor_BH1750:
     {
        BH1750Support* bh = (BH1750Support*) sett->data;
        long lum = bh->GetCurrentLuminosity();
        result = F("CTSET=LIGHT|DATA|");
        result += sett->targetIndex;
        result += '|';
        result += lum;
     }
     return result;
    
     case sensor_DS18B20:
     case sensor_DS18S20:
     {
        DS18B20Temperature t;
        DS18B20Support* ds = (DS18B20Support*) sett->data;
        DSSensorType st = DS18B20;
        if(sett->type == sensor_DS18S20)
          st = DS18S20;


        result = F("CTSET=STATE|TEMP|");
        result += sett->targetIndex;
        result += '|';
          
        if(ds->readTemperature(&t,st))
        {
          if(t.Negative)
            result += '-';
          uint16_t tt = abs(t.Whole);
          tt *= 100;
          tt += abs(t.Fract);

          result += tt;
          
        }
        else
        {
          // не удалось прочитать
          result += NO_TEMPERATURE_DATA;
          result += "00";
          
        }
     }
     return result;
  } // switch

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void doSendData()
{
  #ifdef _DEBUG
    Serial.println(F("Start sending data..."));
  #endif 

  const size_t sensorsCount = sizeof(sensors)/sizeof(sensors[0]);

  for(size_t i=0;i<sensorsCount;i++)
  {
      String strToSend = readFromSensor(&(sensors[i]));
      if(!strToSend.length())
      {
        #ifdef _DEBUG
          Serial.println("No data from sensor, continue...");
        #endif        
        continue;
      }

    
      WiFiClient client;
      if (!client.connect(WiFi.gatewayIP(), 1975)) 
      {
        #ifdef _DEBUG
          Serial.println(F("Connection failed!"));
        #endif
        myDelay(50);
        continue;
      }  
        
      #ifdef _DEBUG
        Serial.print(F("Command to send: "));
        Serial.println(strToSend);
      #endif
    
      client.println(strToSend);
      client.flush();
    
      myDelay(SEND_PACKET_DELAY + random(100));
    
      while(client.connected() && client.available())
      {
        char ch = client.read();
        #ifdef _DEBUG
          Serial.print(ch);
        #endif
      }
    
      client.stop();
      myDelay(CLIENT_FLUSH_DELAY + random(100));

  } // for

  #ifdef _DEBUG
    Serial.println(F("All data was sent!"));
  #endif  

}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void sendSensorsData()
{
  #ifdef _DEBUG
    Serial.println(F("Attempting to send sensors data..."));
  #endif

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
          Serial.println(F("Connected, send data!"));
        #endif

        doSendData();

        wifiOff(); 
            
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
void clearSensorsData()
{
  isWireInited = false;
  lastMillis = 0;
  
  const size_t sensorsCount = sizeof(sensors)/sizeof(sensors[0]); 

 for(size_t i=0;i<sensorsCount;i++)
  {
    switch(sensors[i].type)
    {
      case sensor_NoSensor:
      break;

      case sensor_Si7021:
      {
        HTU21D* htu = (HTU21D*) sensors[i].data;
        delete htu;
      }
      break;

      case sensor_BH1750:
      {
        BH1750Support* bh = (BH1750Support*) sensors[i].data;
        delete bh;
      }
      break;
      
      case sensor_DS18B20:
      case sensor_DS18S20:
      {
        DS18B20Support* ds = (DS18B20Support*) sensors[i].data;
        delete ds;
      }
      break;
      
    } // switch

    sensors[i].data = NULL;
  } // for  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void initSensors()
{
  #ifdef _DEBUG
    Serial.println(F("Init sensors..."));
  #endif

  const size_t sensorsCount = sizeof(sensors)/sizeof(sensors[0]);

  #ifdef _DEBUG
    Serial.print(F("Sensors found: "));
    Serial.println(sensorsCount);
  #endif

  for(size_t i=0;i<sensorsCount;i++)
  {
    switch(sensors[i].type)
    {
      case sensor_NoSensor:
      break;

      case sensor_Si7021:
      {
        #ifdef _DEBUG
          Serial.println(F("Si7021 found, init..."));
        #endif        
        if(!isWireInited)
        {
          isWireInited = true;
          Wire.begin();
        }

        HTU21D* htu = new HTU21D();
        sensors[i].data = htu;
        if(htu->begin())
        {
          #ifdef _DEBUG
            Serial.print(F("Si7021 inited"));
            Serial.print(F(" with data="));
            Serial.println((int)sensors[i].data);
            Serial.print(F("TEST READ FROM Si7021: "));
            Serial.println(htu->readHumidity());
          #endif          
        }
        else
        {
          #ifdef _DEBUG
            Serial.println(F("[ERR]: Si7021 NOT FOUND!!!"));
          #endif                
        }
      }
      break;

      case sensor_BH1750:
      {
        #ifdef _DEBUG
          Serial.println(F("BH1750 found, init..."));
        #endif        
        if(!isWireInited)
        {
          isWireInited = true;
          Wire.begin();
        }

        BH1750Support* bh = new BH1750Support();
        sensors[i].data = bh;
        bh->begin((BH1750Address) sensors[i].param1);

        #ifdef _DEBUG
          Serial.print(F("BH1750 inited on address "));
          Serial.print(sensors[i].param1);
          Serial.print(F(" with data="));
          Serial.println((int)sensors[i].data);
        #endif        
      }
      break;
      
      case sensor_DS18B20:
      case sensor_DS18S20:
      {
        #ifdef _DEBUG
          Serial.println(F("DS18*20 found, init..."));
        #endif
        DS18B20Support* ds = new DS18B20Support();
        sensors[i].data = ds;
        ds->begin(sensors[i].param1);
        ds->setResolution((DS18B20Resolution) sensors[i].param2);

         DSSensorType st = DS18B20;
         if(sensors[i].type == sensor_DS18S20)
          st = DS18S20;

         DS18B20Temperature t;
         ds->readTemperature(&t,st);
         myDelay(800);
        
        #ifdef _DEBUG
          Serial.print(F("DS18*20 inited on pin "));
          Serial.print(sensors[i].param1);
          Serial.print(F(" with data="));
          Serial.println((int)sensors[i].data);
        #endif
      }
      break;
      
    } // switch
  } // for

  #ifdef _DEBUG
    Serial.println(F("Sensors inited."));
  #endif  
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

  randomSeed(analogRead(0));
  updateInterval = Settings.getInterval() + random(100);

  initSensors();

  #if defined(DEEP_SLEEP_OFF_PIN) && defined(USE_DEEP_SLEEP)
    pinMode(DEEP_SLEEP_OFF_PIN, INPUT);
    if(digitalRead(DEEP_SLEEP_OFF_PIN) == DEEP_SLEEP_SIGNAL_LEVEL)
    {
      isDeepSleepAllowed = false;
     #ifdef _DEBUG
        Serial.println(F("DEEP SLEEP OFF!"));
      #endif      
    }
  #endif

  #ifdef _DEBUG
    Serial.println(F("Started."));
  #endif



}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  CommandHandler.handleCommands();

  if(isDeepSleepAllowed)
  {
    sendSensorsData();
    
    clearSensorsData();
    uint32_t dsPeriod = Settings.getInterval() + random(100);
    dsPeriod *= 1000; // в микросекунды
    ESP.deepSleep(dsPeriod);
        
  }
  else
  {
       static bool bFirst = true;
       if(bFirst || (millis() - lastMillis > updateInterval))
       {
          bFirst = false;
          sendSensorsData();
          lastMillis = millis();
          updateInterval = Settings.getInterval() + random(100);
       } // if
       
  } // no deep sleep

/*  
#ifndef USE_DEEP_SLEEP

  static bool bFirst = true;
  if(bFirst || (millis() - lastMillis > updateInterval))
  {
    bFirst = false;
#endif    

    sendSensorsData();

#ifndef USE_DEEP_SLEEP    
    lastMillis = millis();
    updateInterval = Settings.getInterval() + random(100);
  }
#endif
*/

  myDelay(1);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

