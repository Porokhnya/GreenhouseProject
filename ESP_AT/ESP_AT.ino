//----------------------------------------------------------------------------------------------------------------------------------------------------------
#define UART_SPEED 57600 // СКОРОСТЬ РАБОТЫ С ПОРТОМ
#define BROADCAST_PORT 8888 // порт для рассылки широковещательных пакетов
//----------------------------------------------------------------------------------------------------------------------------------------------------------
#include <ESP8266WiFi.h>
#include "SerialCommand.h"
#include "Atomic.h"
#include "ESP8266Ping.h"
//----------------------------------------------------------------------------------------------------------------------------------------------------------
#define AT_OK "OK"
#define AT_ERROR "ERROR"
#define AT_FAIL "FAIL"
//----------------------------------------------------------------------------------------------------------------------------------------------------------
// variables
//----------------------------------------------------------------------------------------------------------------------------------------------------------
bool echoOn = true;
String softAPPassword;
String softAPName;
int softAPencMethod;
int softAPchannelNum;
int cipserverMode = 0;
uint16_t cipserverPort = 0;
WiFiServer* server = NULL;
String apName;
String apPassword;
WiFiMode_t cwMode = WIFI_OFF;
uint8_t statusHelper = 0;

// NTP related
WiFiUDP ntpUDP;
int32_t timezoneOffset = 0;
String ntpServer;
int ntpPort = 123;
uint8_t ntpPacket[NTP_PACKET_SIZE];
#define SEVENZYYEARS 2208988800UL

// broadcast related
WiFiUDP broadcastUDP;
int broadcastPort = BROADCAST_PORT;

//----------------------------------------------------------------------------------------------------------------------------------------------------------
SerialCommand* commandStream = NULL;
//----------------------------------------------------------------------------------------------------------------------------------------------------------
template <typename T> inline void echo(const char* command, T last,void (*function)(void) = NULL)
{
  if(echoOn)
    Serial << command << ENDLINE;

  if(function)
    function();

  Serial << last << ENDLINE;
  
  Serial.flush();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void stopServer()
{
  if(!server)
    return;

  server->stop();
  delete server;
  server = NULL;    
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void unQuote(String& str)
{
  if(!str.length())
    return;
    
   if(str.startsWith("\""))
    str.remove(0,1);

  if(str.endsWith("\""))
    str.remove(str.length()-1);  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void printReady(Stream& s)
{
  s << ENDLINE << "ready" << ENDLINE;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void AT(const char* command)
{
  echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void ATE0(const char* command)
{
  echo(command, AT_OK);
  echoOn = false;
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void ATE1(const char* command)
{
  echoOn = true;
  echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void RESTART(const char* command)
{
  echo(command, AT_OK);

  stopServer();

  delete commandStream;
  commandStream = NULL;

  ESP.restart();
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void GMR(const char* command)
{
  CRITICAL_SECTION;
  echo(command, AT_OK,[](){
      Serial << ESP.getCoreVersion() << ENDLINE;
      Serial << ESP.getSdkVersion() << ENDLINE;
      Serial << __DATE__ << " " << __TIME__ << ENDLINE;
    });
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CWMODE_CUR(const char* command)
{
   CRITICAL_SECTION;
   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

    if(*arg == '?')
    {
        echo(command, AT_OK,[](){
        Serial << WiFi.getMode() << ENDLINE;    
      });      

      return;
    }

    int mode = *arg - '0';

    if(mode < WIFI_STA || mode > WIFI_AP_STA )
    {
      echo(command, AT_ERROR);
      return;      
    }

   cwMode = WiFiMode_t(mode);
   WiFi.mode(cwMode);
    
   echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CWSAP_CUR(const char* command)
{
   CRITICAL_SECTION;
   
   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   softAPName = arg;
   arg = commandStream->next();

   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   softAPPassword = arg;
   arg = commandStream->next();

   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   softAPchannelNum = atoi(arg);
   arg = commandStream->next();

   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   softAPencMethod = atoi(arg);

   unQuote(softAPName);
   unQuote(softAPPassword);
      
  if(!WiFi.softAP(softAPName.c_str(), softAPPassword.c_str(), softAPchannelNum))
  {
    echo(command, AT_ERROR);
    return;    
  }

  echo(command, AT_OK,[](){
      Serial << "+CWSAP_CUR:\"" << softAPName << "\",\"" << softAPPassword << "\"," << softAPchannelNum << "," << softAPencMethod << ENDLINE;
  });
    
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CWJAP_TEST(const char* command)
{
  CRITICAL_SECTION;
  
  if(!WiFi.isConnected())
  {
     echo(command, AT_OK,[](){
        Serial << "No AP" << ENDLINE;
      });
    return;
  }

 echo(command, AT_OK,[](){
        Serial << "+CWJAP:\"" << WiFi.SSID() << "\",\"" << WiFi.BSSIDstr() << "\"," << WiFi.channel() << "," << WiFi.RSSI() << ENDLINE;
      });
 
  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void BROADCAST(const char* command)
{
 CRITICAL_SECTION;

   char* arg = commandStream->next(); // packet
   if(!arg || !WiFi.isConnected())
   {
    echo(command, AT_ERROR);
    return;
   }  

   String packet = arg;
   unQuote(packet);

   IPAddress broadcastIp(255,255,255,255);
   broadcastUDP.beginPacket(broadcastIp,broadcastPort);
   broadcastUDP.write(packet.c_str(),packet.length());
   broadcastUDP.endPacket();

   echo(command, AT_OK);  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void sendNTPPacket()
{
 // set all bytes in the buffer to 0
  memset(ntpPacket, 0, NTP_PACKET_SIZE);
  // Initialize values needed to form NTP request
  // (see URL above for details on the packets)
  ntpPacket[0] = 0b11100011;   // LI, Version, Mode
  ntpPacket[1] = 0;     // Stratum, or type of clock
  ntpPacket[2] = 6;     // Polling Interval
  ntpPacket[3] = 0xEC;  // Peer Clock Precision
  // 8 bytes of zero for Root Delay & Root Dispersion
  ntpPacket[12]  = 49;
  ntpPacket[13]  = 0x4E;
  ntpPacket[14]  = 49;
  ntpPacket[15]  = 52;

  // all NTP fields have been given values, now
  // you can send a packet requesting a timestamp:
  ntpUDP.beginPacket(ntpServer.c_str(), ntpPort); //NTP requests are to port 123
  ntpUDP.write(ntpPacket, NTP_PACKET_SIZE);
  ntpUDP.endPacket();  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void NTPTIME(const char* command)
{
  CRITICAL_SECTION;

   char* arg = commandStream->next(); // NTP server
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }  

   ntpServer = arg;
   unQuote(ntpServer);

   arg = commandStream->next(); // NTP port
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   ntpPort = atoi(arg);

   arg = commandStream->next(); // time offset (seconds)
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   timezoneOffset = atoi(arg);

   /*
   Serial.print("ntpServer="); Serial.println(ntpServer);
   Serial.print("ntpPort="); Serial.println(ntpPort);
   Serial.print("timezoneOffset="); Serial.println(timezoneOffset);
   */

   sendNTPPacket();

   echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CWJAP_CUR(const char* command)
{
  CRITICAL_SECTION;

  if(!(cwMode == WIFI_STA || cwMode == WIFI_AP_STA)) // команда подсоединения требует активным режим станции
  {
    DBGLN(F("not configured in STA mode!"));
    echo(command, AT_ERROR);
    return;
  }

   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   apName = arg;
   arg = commandStream->next();

   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }
   
   apPassword = arg;

   unQuote(apName);
   unQuote(apPassword);

   // коннектимся к точке доступа
   /*
   if(WiFi.isConnected())
   {
      DBGLN(F("already connected, disconnect..."));
      WiFi.disconnect();
      DBGLN(F("Disconnected."));
   }
   */

  WiFi.disconnect();
  while (WiFi.status() == WL_CONNECTED)
  {
    delay(10);
  }   

   DBGLN(F("Connect..."));
   WiFi.begin(apName.c_str(),apPassword.c_str());
   DBGLN(F("Connect done, wait for connect result..."));
   uint8_t status = WiFi.waitForConnectResult();

   DBG(F("Connect result done: "));
   DBGLN(status);

   switch(status)
   {
      case WL_CONNECTED: // подсоединились
      {
        DBGLN(F("Connected!"));
        echo(command, AT_OK);
      }
      break;

      case WL_NO_SSID_AVAIL: // не найдена станция
      {
        DBGLN(F("No SSID found!"));
        
        echo(command, AT_FAIL,[](){
            Serial << "+CWJAP_CUR:" << 3 << ENDLINE;
        });        
      }
      break;

      case WL_CONNECT_FAILED: // неправильный пароль
      {
        DBGLN(F("Password incorrect!"));
        
        echo(command, AT_FAIL,[](){
            Serial << "+CWJAP_CUR:" << 2 << ENDLINE;
        });        
      }
      break;

      case WL_IDLE_STATUS:
      {
        DBGLN(F("WL_IDLE_STATUS!"));
        
        echo(command, AT_FAIL,[](){
            Serial << "+CWJAP_CUR:" << 4 << ENDLINE;
        });            
      }
      break;

      case WL_DISCONNECTED: // модуль не сконфигурирован как станция
      {
        DBGLN(F("Not configured as STATION!"));
        
        echo(command, AT_FAIL,[](){
            Serial << "+CWJAP_CUR:" << 4 << ENDLINE;
        });        
        
      }
      break;

      default:
      {
        DBGLN(F("Unknown status!"));
        
        statusHelper = status;
        echo(command, AT_FAIL,[](){
            Serial << "+CWJAP_CUR:" << statusHelper << ENDLINE;
        });   
      }
      break;
    
   } // switch
   
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CWQAP(const char* command)
{
  CRITICAL_SECTION;

  if(WiFi.isConnected())
  {
    WiFi.disconnect(false);
  }

  while (WiFi.status() == WL_CONNECTED)
  {
    delay(10);
  }

   echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPMODE(const char* command)
{
  CRITICAL_SECTION;

   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   echo(command, AT_OK);
  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPMUX(const char* command)
{
  CRITICAL_SECTION;

   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   echo(command, AT_OK);
  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPSERVER(const char* command)
{
  CRITICAL_SECTION;

   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   cipserverMode = atoi(arg);
   
   arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   cipserverPort = atoi(arg);

   switch(cipserverMode)
   {
     case 0:
     {
       stopServer();
     }
     break;

     case 1:
     {
        stopServer();
        server = new WiFiServer(cipserverPort);
        server->begin();
     }
     break;
   }

   echo(command, AT_OK);
  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
int pingAvgTimeHelper = 0;
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void PING(const char* command)
{
   CRITICAL_SECTION;

   char* arg = commandStream->next();
   if(!arg || !WiFi.isConnected())
   {
    echo(command, AT_ERROR);
    return;
   }
   

   String remoteIP = arg;
   unQuote(remoteIP);

   if(Ping.ping(remoteIP.c_str()))
   {
      pingAvgTimeHelper = Ping.averageTime();
      echo(command, AT_OK,[](){

          Serial << "+" << pingAvgTimeHelper << ENDLINE;
        
        });
   }
   else
   {
     echo(command, AT_ERROR,[](){

          Serial << "+timeout" << ENDLINE;
        
        });
   }
   
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void getCIPSTAMAC(const char* command)
{
 CRITICAL_SECTION;

 echo(command, AT_OK,[](){

          Serial << "+CIPSTAMAC:\"" << WiFi.macAddress() << "\"" << ENDLINE;
        
        });  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void getCIPAPMAC(const char* command)
{
 CRITICAL_SECTION;

 echo(command, AT_OK,[](){

          Serial << "+CIPAPMAC:\"" << WiFi.softAPmacAddress() << "\"" << ENDLINE;
        
        });  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CSQ(const char* command)
{
  CRITICAL_SECTION;

 echo(command, AT_OK,[](){
          Serial << "+CSQ: " << WiFi.RSSI() << ENDLINE;        
        });      
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void getCIFSR(const char* command)
{
  CRITICAL_SECTION;

 echo(command, AT_OK,[](){

          Serial << "+CIFSR:APIP,\"" << WiFi.softAPIP() << "\"" << ENDLINE;
          Serial << "+CIFSR:APMAC,\"" << WiFi.softAPmacAddress() << "\"" << ENDLINE;
          Serial << "+CIFSR:STAIP,\"" << WiFi.localIP() << "\"" << ENDLINE;
          Serial << "+CIFSR:STAMAC,\"" << WiFi.macAddress() << "\"" << ENDLINE;
        
        });    
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPCLOSE(const char* command)
{
  CRITICAL_SECTION;

   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_OK);
    return;
   }

   int clientNumber = atoi(arg);
   if(clientNumber < 0)
   {
    echo(command, AT_ERROR);
    return;
   }
    
   if(clientNumber > (MAX_CLIENTS-1))
   {
    echo(command, AT_ERROR);
    return;
   }

   if(Clients[clientNumber].connected())
   {
      DBG(F("Client #"));
      DBG(clientNumber);
      DBGLN(F(" connected, disconnect..."));
      Clients[clientNumber].stop();
      
   }

   echo(command, AT_OK);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPSTART(const char* command)
{
  CRITICAL_SECTION;

  if(!WiFi.isConnected()) // не приконнекчены к роутеру - не можем никуда коннектиться
  {
    echo(command, AT_ERROR);
    return;    
  }
  
   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   int linkID = atoi(arg);
   if( linkID < 0)
   {
    echo(command, AT_ERROR);
    return;
   }

  if(linkID > (MAX_CLIENTS-1))
  {
    echo(command, AT_ERROR);
    return;    
  }

   arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   String connectionType = arg;
   unQuote(connectionType);

   arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   String remoteIP = arg;
   unQuote(remoteIP);

   arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   int remotePort = atoi(arg);

   if(Clients[linkID].connected())
   {
    echo(command, "ALREADY CONNECTED");
    return;    
   }

   if(!Clients[linkID].connect(remoteIP.c_str(),remotePort))
   {
    echo(command, AT_ERROR);
    return;
   }

   echo(command, AT_OK);   
  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void CIPSENDBUF(const char* command)
{
  CRITICAL_SECTION;
  
   char* arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   int linkID = atoi(arg);
   if( linkID < 0)
   {
    echo(command, AT_ERROR);
    return;
   }

  if(linkID > (MAX_CLIENTS-1))
  {
    echo(command, AT_ERROR);
    return;
  }

   arg = commandStream->next();
   if(!arg)
   {
    echo(command, AT_ERROR);
    return;
   }

   size_t dataLength = atoi(arg);
   if(dataLength < 1)
   {
    echo(command, AT_ERROR);
    return;    
   }

   if(!Clients[linkID].connected())
   {
    echo(command, AT_ERROR);
    return;    
   }

   echo(command, AT_OK);

   // вот тут приглашение выводить нельзя, поскольку у нас команда могла дойти
   // только тогда, когда уже Events отработали и выплюнули в поток что-то типа +IPD
   // В этом случае принимающая сторона должна отработать +IPD,
   // а мы, в свою очередь - должны отправить приглашение только тогда, когда
   // уже ничего не делается.
   // для этого надо заводить отдельную очередь для отсыла приглашений на ввод данных, сохраняя там
   // переданного клиента.

   Cipsend.add(dataLength,linkID);


}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void raiseClientStatus(uint8_t clientNumber, bool connected)
{
  String message;
  message = clientNumber;
  if(connected)
    message += ",CONNECT";
  else
    message += ",CLOSED";
    
  message += ENDLINE;
  Events.raise(message.c_str(),message.length());  
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void handleClientConnectStatus()
{
  // обновляем статус клиентов
  for(uint8_t i=0;i<MAX_CLIENTS;i++)
  {
      uint8_t curStatus = Clients[i].connected() ? 1 : 0;
      uint8_t lastStatus = ClientConnectStatus[i];
      ClientConnectStatus[i] = curStatus;
      
      if(curStatus != lastStatus) // статус соединения изменился
      {

          if(!curStatus) // клиент не подконнекчен, надо его остановить
          {
            Clients[i].stop();
          }
       
          // сообщаем статус клиента
          raiseClientStatus(i,!lastStatus);

                  
      } // if(curStatus != lastStatus)
    
  } // for 
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void handleIncomingConnections()
{
  if(!server)
    return;

  // проверяем на входящие подключения.
  // их принимать можно только тогда, когда есть хотя бы один свободный слот
  uint8_t freeSlotNumber = 0xFF;
  for(uint8_t i=0;i<MAX_CLIENTS;i++)
  {
    if(!Clients[i].connected())
    {
      freeSlotNumber = i;
      break;
    }
  } // for    

  WiFiClient client = server->available();
  if(client.connected())
  {
    DBGLN("CATCH CONNECTED CLIENT IN SERVER!!!");
    DBG("Local port: ");
    DBGLN(client.localPort());
    DBG("Local IP: ");
    DBGLN(client.localIP());
    DBG("Remote port: ");
    DBGLN(client.remotePort());
    DBG("Remote IP: ");
    DBGLN(client.remoteIP());
    DBGLN("");

    if(freeSlotNumber != 0xFF)
    {
      DBG("HAVE FREE SLOT #");
      DBG(freeSlotNumber);
      DBGLN(", MOVE INCOMING CLIENT TO SLOT.");
      // есть свободный слот, помещаем туда клиента
      Clients[freeSlotNumber] = client;     
    } // if(freeSlotNumber != 0xFF)
    else
    {
      // свободных слотов нет, отсоединяем клиента
      DBGLN("NO FREE SLOTS, STOP INCOMING CLIENT.");
      client.stop();
    } // else
  }    
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void handleClientData(uint8_t clientNumber, WiFiClient& client)
{
  const uint16_t buf_sz = 2048;
  static uint8_t read_buff[buf_sz];

  if(!client.available())
    return;
  
  memset(read_buff,0,buf_sz);

  int readed = client.read(read_buff,buf_sz);
  if(readed > 0)
  {
    // есть данные, сообщаем о них. Приходится через raise, чтобы не вклиниться между команд
    String toRaise;
    toRaise.reserve(readed + 20); // резервируем буфер сразу
    toRaise = "+IPD,";
    toRaise += clientNumber;
    toRaise += ",";
    toRaise += readed;
    toRaise += ":";
    for(int i=0;i<readed;i++)
      toRaise += (char) read_buff[i];

    toRaise += ENDLINE;

    Events.raise(toRaise.c_str(),toRaise.length()); 
    
  } // if(readed > 0)
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void handleClientsData()
{
  // тут смотрим, есть ли у клиентов входящие данные.
  // проходимся равномерно по всем клиентам, чтобы не было ситуации,
  // когда пришедший в меньший слот всегда обслуживается первым.
  static uint8_t currentClientNumber = 0;

 if(Clients[currentClientNumber].connected())
      handleClientData(currentClientNumber,Clients[currentClientNumber]);

 currentClientNumber++;

 if(currentClientNumber >= MAX_CLIENTS)
    currentClientNumber = 0;

}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void unknownCommand(const char* command)
{
  CRITICAL_SECTION;
  echo(command, AT_ERROR);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  Serial.begin(UART_SPEED);
  #ifdef _DEBUG
    Serial.setDebugOutput(true);
  #endif

  DBGLN(F("Serial ready."));

  server = NULL;
  cwMode = WIFI_OFF;
  echoOn = true;
  memset(ClientConnectStatus,0,sizeof(ClientConnectStatus));

  DBGLN(F("Create Serial listener..."));
  commandStream = new SerialCommand;
  DBGLN(F("Serial listener created."));
  
  DBGLN(F("Begin cipsend handler..."));
  Cipsend.clear();
  DBGLN(F("Cipsend handler inited."));

  DBGLN(F("Begin events..."));
  Events.begin();
  DBGLN(F("Events inited."));
  
 // WiFi.setOutputPower(20);
 DBGLN(F("Turn OFF persistent mode..."));
  WiFi.persistent(false);
  DBGLN(F("Persistent mode OFF."));

  DBGLN(F("Turn Wi-Fi OFF..."));
  WiFi.mode(WIFI_OFF);   // this is a temporary line, to be removed after SDK update to 1.5.4
  DBGLN(F("Wi-Fi turned OFF."));
  //WiFi.setAutoConnect(false);
  //WiFi.setAutoReconnect(false);

  DBGLN(F("Setup known commands..."));
  
  commandStream->setDefaultHandler(unknownCommand);

  commandStream->addCommand("AT",AT);
  commandStream->addCommand("ATE0",ATE0);
  commandStream->addCommand("ATE1",ATE1);
  commandStream->addCommand("AT+RST",RESTART);
  commandStream->addCommand("AT+GMR",GMR);
  commandStream->addCommand("AT+CWMODE_CUR",CWMODE_CUR);
  commandStream->addCommand("AT+CWMODE_DEF",CWMODE_CUR);
  commandStream->addCommand("AT+CWSAP_CUR",CWSAP_CUR);
  commandStream->addCommand("AT+CWSAP_DEF",CWSAP_CUR);
  commandStream->addCommand("AT+CWQAP",CWQAP);
  commandStream->addCommand("AT+CIPMODE",CIPMODE);
  commandStream->addCommand("AT+CIPMUX",CIPMUX);
  commandStream->addCommand("AT+CIPSERVER",CIPSERVER);
  commandStream->addCommand("AT+CWJAP_CUR",CWJAP_CUR);
  commandStream->addCommand("AT+CWJAP_DEF",CWJAP_CUR);
  commandStream->addCommand("AT+CWJAP?",CWJAP_TEST);
  commandStream->addCommand("AT+PING",PING);
  commandStream->addCommand("AT+CIPSTAMAC?",getCIPSTAMAC);
  commandStream->addCommand("AT+CIPAPMAC?",getCIPAPMAC);
  commandStream->addCommand("AT+CIFSR",getCIFSR);
  commandStream->addCommand("AT+CIPCLOSE",CIPCLOSE);
  commandStream->addCommand("AT+CIPSTART",CIPSTART);
  commandStream->addCommand("AT+CIPSENDBUF",CIPSENDBUF);
  commandStream->addCommand("AT+CIPSEND",CIPSENDBUF);
  commandStream->addCommand("AT+CSQ",CSQ);
  commandStream->addCommand("AT+NTPTIME",NTPTIME);
  commandStream->addCommand("AT+BROADCAST",BROADCAST);
  
  DBGLN(F("Known commands inited."));  

  printReady(Serial);
  Serial.flush();


}
//----------------------------------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
  
  // здесь мы уже обработали входящую команду, если это запрос на подсоединение -
  // слот клиента будет занят
  commandStream->readSerial();

  // теперь отрабатываем соединения от сервера
  if(!commandStream->waitingCommand())
    handleIncomingConnections();

  // выводим статус соединений
  handleClientConnectStatus();

  // обрабатываем данные для клиентов
  handleClientsData();

  // и выводим их через события
  if(!commandStream->waitingCommand())
    Events.update();

  Cipsend.update();

  if(ntpUDP.parsePacket() == NTP_PACKET_SIZE)
  {
    // got NTP packet!
     ntpUDP.read(ntpPacket, NTP_PACKET_SIZE);
     unsigned long highWord = word(ntpPacket[40], ntpPacket[41]);
     unsigned long lowWord = word(ntpPacket[42], ntpPacket[43]);
     // combine the four bytes (two words) into a long integer
     // this is NTP time (seconds since Jan 1 1900):
     unsigned long secsSince1900 = highWord << 16 | lowWord;

     unsigned long currentEpoc = secsSince1900 - SEVENZYYEARS;
     currentEpoc += (timezoneOffset*60);

     String message = "+NTPTIME:";
     message += currentEpoc;
     message += ENDLINE;

     Events.raise(message.c_str(),message.length());
  }
    
  delay(0);
}
//----------------------------------------------------------------------------------------------------------------------------------------------------------

