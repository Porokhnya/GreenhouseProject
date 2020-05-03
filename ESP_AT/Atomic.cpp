//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "Atomic.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
volatile bool __semaphor = false;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
WiFiClient Clients[MAX_CLIENTS];
uint8_t ClientConnectStatus[MAX_CLIENTS] = {0};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CipsendHandler
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CipsendHandler Cipsend;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CipsendHandler::CipsendHandler()
{
  segmentID = 0;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CipsendHandler::~CipsendHandler()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CipsendHandler::add(size_t dataLength,uint8_t linkID)
{
  CipsendData dt;
  dt.dataLength = dataLength;
  dt.linkID = linkID;

  data.push_back(dt);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CipsendHandler::clear()
{
  data.clear();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void CipsendHandler::update()
{
  if(CriticalSection::Triggered() || !data.size())
    return;

  CRITICAL_SECTION;

  CipsendData dt = data[0];
  for(size_t i=1;i<data.size();i++)
    data[i-1] = data[i];

  data.pop();

   Serial << '>'; // выводим приглашение
   Serial.flush();
   
   // читаем данные
   size_t readed = 0;

   uint8_t* data = new uint8_t[dt.dataLength];
   memset(data,0,dt.dataLength);
   
   while(readed < dt.dataLength)
   {
      if(!Serial.available())
      {
        delay(0);
        continue;
      }

      uint8_t ch = Serial.read();
      data[readed] = ch;
      readed++;
   } // while

   String sendFail = ENDLINE;
   sendFail += dt.linkID;
   sendFail += ",SEND FAIL";
   sendFail += ENDLINE;

   // данные получили, можно их писать в клиента
   if(!Clients[dt.linkID].connected())
   {
    Serial << sendFail;
    Serial.flush();
    delete [] data;
    return;    
   }   

   if(!Clients[dt.linkID].write((const uint8_t*)data,dt.dataLength))
   {
     delete [] data;
     Serial << sendFail;
     Serial.flush();
     return;
   }

   String sendOK = ENDLINE;
   sendOK += dt.linkID;
   sendOK += ",";
   sendOK += segmentID;
   segmentID++;
   sendOK += ",SEND OK";
   sendOK += ENDLINE;

   delete [] data;
  
   Serial << sendOK;
   Serial.flush();

  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// CriticalSection
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CriticalSection::CriticalSection()
{
  __semaphor = true;
  DBGLN("Atomic Guard: enter.");
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
CriticalSection::~CriticalSection()
{
  __semaphor = false;
  DBGLN("Atomic Guard: leave.");
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
bool CriticalSection::Triggered()
{
  return __semaphor;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// EventsList
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
EventsList Events;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
WiFiEventHandler onStationModeConnectedHandler;
WiFiEventHandler onStationModeDisconnectedHandler;
WiFiEventHandler onStationModeGotIPHandler;
bool __connected = false;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void onStationConnected(const WiFiEventStationModeConnected& evt) 
{
  if(!__connected)
  {
    ntpUDP.begin(NTP_DEFAULT_LOCAL_PORT);
    broadcastUDP.begin(broadcastPort);
    Events.raise("WIFI CONNECTED\r\n"); 
  }
        
  __connected = true;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void onStationDisconnected(const WiFiEventStationModeDisconnected& evt) 
{
  if(__connected)
  {
    ntpUDP.stop();
    broadcastUDP.stop();
    Events.raise("WIFI DISCONNECT\r\n");     
  }

  __connected = false; 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void onStationGotIP(const WiFiEventStationModeGotIP& evt) 
{
  Events.raise("WIFI GOT IP\r\n");
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
EventsList::EventsList()
{
  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EventsList::update()
{
  if(!CriticalSection::Triggered())
  {
    MessagesList thisMessages = messages;
    messages.empty();

    for(size_t i=0;i<thisMessages.size();i++)
    {
      Serial.write((const uint8_t*) thisMessages[i].data,thisMessages[i].dataLength);
      Serial.flush();
      delete [] thisMessages[i].data;
    }
  }
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EventsList::clear()
{
   for(size_t i=0;i<messages.size();i++)
  {
    delete [] messages[i].data;
  }

  messages.clear(); 
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EventsList::begin()
{
  clear();
  
  onStationModeConnectedHandler = WiFi.onStationModeConnected(&onStationConnected);
  onStationModeDisconnectedHandler = WiFi.onStationModeDisconnected(&onStationDisconnected);
  onStationModeGotIPHandler = WiFi.onStationModeGotIP(&onStationGotIP);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EventsList::raise(const char* evt)
{
  raise(evt, strlen(evt));
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void EventsList::raise(const char* data, size_t dataLength)
{
  if(!CriticalSection::Triggered())
  {
    Serial.write((const uint8_t*)data,dataLength);
    Serial.flush();
    return;
  }
  
  for(size_t i=0;i<messages.size();i++)
  {
    if(messages[i].dataLength == dataLength && !memcmp(messages[i].data,data, dataLength)) // same message
      return;
  }

  EventData dt;
  dt.data = new char[dataLength];
  dt.dataLength = dataLength;
  memcpy(dt.data,data,dataLength);
  
  messages.push_back(dt);
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------

