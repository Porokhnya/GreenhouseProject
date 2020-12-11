#include "CoreTransport.h"
#include "Settings.h"
#include "ModuleController.h"
#include "Memory.h"
#include "InteropStream.h"
#include "EEPROMSettingsModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#include <SdFat.h>
//--------------------------------------------------------------------------------------------------------------------------------------
#define CIPSEND_COMMAND F("AT+CIPSENDBUF=")
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreTransportClient
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient::CoreTransportClient()
{
  socket = NO_CLIENT_ID;
  dataBuffer = NULL;
  dataBufferSize = 0;
  parent = NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient::~CoreTransportClient()
{
  clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::accept(CoreTransport* _parent)
{
  parent = _parent;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::clear()
{
    delete [] dataBuffer; 
    dataBuffer = NULL;
    dataBufferSize = 0;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::disconnect()
{
  if(!parent)
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
     DEBUG_LOGLN(F("CLIENT, disconnect(): parent == NULL!"));
   #endif     
    return;
  }
  
  if(!connected())
    return;

  parent->doDisconnect(*this);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransportClient::connect(const char* ip, uint16_t port)
{
  if(!parent)
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
     DEBUG_LOGLN(F("CLIENT, connect(): parent == NULL!"));
   #endif     
    
    return;
  }
  
  if(connected()) // уже присоединены, нельзя коннектится до отсоединения!!!
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
              DEBUG_LOGLN(F("CLIENT, connect(): CLIENT ALREADY CONNECTED!"));
    #endif	  
    return;
  }
          
  parent->doConnect(*this,ip,port);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransportClient::write(uint8_t* buff, size_t sz)
{
  if(!parent)
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
     DEBUG_LOGLN(F("CLIENT, write(): parent == NULL!"));
   #endif     
    
    
    return false;
  }
  
    if(!sz || !buff || !connected() || socket == NO_CLIENT_ID)
    {
      #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
        DEBUG_LOGLN(F("CLIENT, write() - CAN'T WRITE!"));
      #endif
      
      return false;
    }

  clear();
  dataBufferSize = sz; 
  if(dataBufferSize)
  {
      dataBuffer = new  uint8_t[dataBufferSize];
      memcpy(dataBuffer,buff,dataBufferSize);
  }

    parent->doWrite(*this);
    
   return true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransportClient::connected() 
{
  if(!parent || socket == NO_CLIENT_ID)
  {
    return false;
  }
    
  return parent->connected(socket);
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreTransport
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::CoreTransport(uint8_t clientsPoolSize)
{
  for(uint8_t i=0;i<clientsPoolSize;i++)
  {
    CoreTransportClient* client = new CoreTransportClient();
    client->accept(this);
    client->bind(i);
    
    pool.push_back(client);
    status.push_back(false);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransport::~CoreTransport()
{
  for(size_t i=0;i<pool.size();i++)
  {
    delete pool[i];
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::initPool()
{

  #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
    DEBUG_LOGLN(F("TRANSPORT: INIT CLIENTS POOL..."));
  #endif
  
  Vector<CoreTransportClient*> tmp = externalClients;
  for(size_t i=0;i<tmp.size();i++)
  {
    notifyClientConnected(*(tmp[i]),false,CT_ERROR_NONE);
    tmp[i]->release();
    tmp[i]->clear(); // очищаем данные внешнего клиента
  }

  externalClients.clear(); // очищаем список внешних клиентов
  
  for(size_t i=0;i<status.size();i++)
  {
    status[i] = false;
    pool[i]->clear(); // очищаем внутренних клиентов
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransport::connected(uint8_t socket)
{
  return status[socket];
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::doWrite(CoreTransportClient& client)
{
  if(!client.connected())
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
     DEBUG_LOGLN(F("TRANSPORT, doWrite(): client NOT CONNECTED!"));
   #endif       
    client.clear();
    return;
  }

   beginWrite(client); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::doConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
  if(client.connected())
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
              DEBUG_LOG(F("ERROR!!! CLIENT ALREADY CONNECTED: #"));
              DEBUG_LOGLN(String(client.socket));
    #endif	  
    return;
  }

   // запоминаем нашего клиента
   client.accept(this);

  // если внешний клиент - будем следить за его статусом соединения/подсоединения
   if(isExternalClient(client))
   {
      externalClients.push_back(&client);
   }

   beginConnect(client,ip,port); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::doDisconnect(CoreTransportClient& client)
{
  if(!client.connected())
  {
   #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
     DEBUG_LOGLN(F("TRANSPORT, doDisconnect(): client NOT CONNECTED!"));
   #endif         
    return;
  }

    beginDisconnect(client);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::subscribe(IClientEventsSubscriber* subscriber)
{
  for(size_t i=0;i<subscribers.size();i++)
  {
    if(subscribers[i] == subscriber)
    {
      return;
    }
  }

  subscribers.push_back(subscriber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::unsubscribe(IClientEventsSubscriber* subscriber)
{
  for(size_t i=0;i<subscribers.size();i++)
  {
    if(subscribers[i] == subscriber)
    {
      for(size_t k=i+1;k<subscribers.size();k++)
      {
        subscribers[k-1] = subscribers[k];
      }
      subscribers.pop();
      break;
    }
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreTransport::isExternalClient(CoreTransportClient& client)
{
  // если клиент не в нашем пуле - это экземпляр внешнего клиента
  for(size_t i=0;i<pool.size();i++)
  {
    if(pool[i] == &client)
    {
      return false;
    }
  }

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::notifyClientConnected(CoreTransportClient& client, bool connected, int16_t errorCode)
{
   // тут надо синхронизировать с пулом клиентов
   if(client.socket != NO_CLIENT_ID)
   {
      status[client.socket] = connected;
   }
  
    for(size_t i=0;i<subscribers.size();i++)
    {
      subscribers[i]->OnClientConnect(client,connected,errorCode);
    }

      // возможно, это внешний клиент, надо проверить - есть ли он в списке слежения
      if(!connected) // пришло что-то типа 1,CLOSED
      {

        client.clear(); // очищаем данные клиента
                 
        // клиент отсоединился, надо освободить его сокет
        for(size_t i=0;i<externalClients.size();i++)
        {
          if(externalClients[i]->socket == client.socket)
          {
            externalClients[i]->clear();
            
            #if defined(WIFI_DEBUG) || defined(GSM_DEBUG_MODE)
              DEBUG_LOG(F("TRANSPORT: RELEASE SOCKET ON OUTGOING CLIENT #"));
              DEBUG_LOGLN(String(client.socket));
            #endif
            
            externalClients[i]->release(); // освобождаем внешнему клиенту сокет
            
            for(size_t k=i+1;k<externalClients.size();k++)
            {
              externalClients[k-1] = externalClients[k];
            }
            externalClients.pop();
            break;
          }
        } // for
      } // if(!connected)
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::notifyDataWritten(CoreTransportClient& client, int16_t errorCode)
{
    for(size_t i=0;i<subscribers.size();i++)
    {
      subscribers[i]->OnClientDataWritten(client,errorCode);
    } 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreTransport::notifyDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone)
{
    for(size_t i=0;i<subscribers.size();i++)
    {
      subscribers[i]->OnClientDataAvailable(client,data,dataSize,isDone);
    }  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreTransportClient* CoreTransport::getClient(uint8_t socket)
{
  if(socket != NO_CLIENT_ID)
  {
    return pool[socket];
  }

  for(size_t i=0;i<pool.size();i++)
  {
    if(!pool[i]->connected())
    {
      return pool[i];
    }
  }

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreESPTransport
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPTransport ESP;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPTransport::CoreESPTransport() : CoreTransport(ESP_MAX_CLIENTS)
{
  recursionGuard = 0;
  signalQuality = 0;
  flags.waitCipstartConnect = false;
  cipstartConnectClient = NULL;
  workStream = NULL;
  badPingAttempts = 0;
  internalPingTimer = 0;

}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreESPTransport::~CoreESPTransport()
{
  clearSpecialCommandResults();
  clearClientsQueue(false);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::broadcast(const String& packet)
{
  packetToBroadcast = packet;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::readFromStream()
{
  if(!workStream)
  {
    return;
  }
    
  while(workStream->available())
  {
    receiveBuffer.push_back((uint8_t) workStream->read());
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::sendCommand(const String& command, bool addNewLine)
{

  if(!workStream)
  {
    return;
  }
  
  size_t len = command.length();
  for(size_t i=0;i<len;i++)
  {
    // записали байтик
    workStream->write(command[i]);

    // прочитали, что пришло от ESP
    readFromStream();

   #ifdef USE_SMS_MODULE
     // и модуль GSM тоже тут обновим
     SIM800.readFromStream();
   #endif     
  }
    
  if(addNewLine)
  {
    workStream->println();
  }
  
  // прочитали, что пришло от ESP
  readFromStream();

   #ifdef USE_SMS_MODULE
   // и модуль GSM тоже тут обновим
   SIM800.readFromStream();
   #endif   

  #ifdef WIFI_DEBUG
    DEBUG_LOG(F("ESP: ==> "));
    DEBUG_LOGLN(command);
  #endif

  machineState = espWaitAnswer; // говорим, что надо ждать ответа от ESP
  // запоминаем время отсылки последней команды
  timer = millis();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::pingGoogle(bool& result)
{
    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      return false;
    }

    flags.specialCommandDone = false;
    clearSpecialCommandResults();
    initCommandsQueue.push_back(cmdPING);

    while(!flags.specialCommandDone)
    {     
      update();
      yield();
      if(flags.wantReconnect || machineState == espReboot)
      {
        break;
      }
    }

    if(!specialCommandResults.size())
    {
      return false;
    }

    result = *(specialCommandResults[0]) == F("OK");

    clearSpecialCommandResults();

    return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::getMAC(String& staMAC, String& apMAC)
{
    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      return false;
    }

    staMAC = '-';
    apMAC = '-';    

    flags.specialCommandDone = false;
    clearSpecialCommandResults();
    initCommandsQueue.push_back(cmdCIFSR);

    while(!flags.specialCommandDone)
    {     
      update();
      yield();
      if(flags.wantReconnect || machineState == espReboot)
      {
        break;
      }
    }

    if(!specialCommandResults.size())
    {
      return false;
    }

    for(size_t i=0;i<specialCommandResults.size();i++)
    {
      String* s = specialCommandResults[i];
      
      int idx = s->indexOf(F("STAMAC,"));
      if(idx != -1)
      {
        const char* stamacPtr = s->c_str() + idx + 7;
        staMAC = stamacPtr;
      }
      else
      {
        idx = s->indexOf(F("APMAC,"));
        if(idx != -1)
        {
          const char* apmacPtr = s->c_str() + idx + 6;
          apMAC = apmacPtr;
        }
      }
      
      
    } // for

    clearSpecialCommandResults();

    return true;            
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::getIP(String& stationCurrentIP, String& apCurrentIP)
{

    if(machineState != espIdle || !workStream || !ready() || initCommandsQueue.size()) // чего-то делаем, не могём
    {
      return false;
    }

    stationCurrentIP = '-';
    apCurrentIP = '-';    

    flags.specialCommandDone = false;
    clearSpecialCommandResults();
    initCommandsQueue.push_back(cmdCIFSR);

    while(!flags.specialCommandDone)
    {     
      update();
      yield();
      if(flags.wantReconnect || machineState == espReboot)
      {
        break;
      }
    }

    if(!specialCommandResults.size())
    {
      return false;
    }

    for(size_t i=0;i<specialCommandResults.size();i++)
    {
      String* s = specialCommandResults[i];
      
      int idx = s->indexOf(F("STAIP,"));
      if(idx != -1)
      {
        const char* staipPtr = s->c_str() + idx + 6;
        stationCurrentIP = staipPtr;
      }
      else
      {
        idx = s->indexOf(F("APIP,"));
        if(idx != -1)
        {
          const char* apipPtr = s->c_str() + idx + 5;
          apCurrentIP = apipPtr;
        }
      }
      
      
    } // for

    clearSpecialCommandResults();

    return true;

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::sendCommand(ESPCommands command)
{
  currentCommand = command;
  
  // тут посылаем команду в ESP
  switch(command)
  {
    case cmdNone:
    case cmdCIPCLOSE: // ничего тут не надо, эти команды формируем не здесь
    case cmdCIPSTART:
    case cmdCIPSEND:
    case cmdWaitSendDone:
    break;

    case cmdPING:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: PING GOOGLE..."));
      #endif

      sendCommand(F("AT+PING=\"google.com\""));
     
    }
    break;

    case cmdInternalPing:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: PING HOST: "));
        DEBUG_LOGLN(WIFI_PING_HOST);
      #endif

      String cmd = F("AT+PING=\"");
      cmd += WIFI_PING_HOST;
      cmd += F("\"");

      sendCommand(cmd);
     
    }
    break;    

    case cmdNTPTIME:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: REQUEST NTP TIME..."));
      #endif

      GlobalSettings* settings = MainController->GetSettings();
      TimeSyncSettings* ts = settings->getTimeSyncSettings();

      String cmd = F("AT+NTPTIME=\"");
      cmd += ts->ntpServer;
      cmd += F("\",");
      cmd += ts->ntpPort;
      cmd += ',';
      cmd += ts->ntpTimezone;

      sendCommand(cmd);
    }
    break;

    case cmdBROADCAST:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: SEND BROADCAST PACKET..."));
      #endif

      String cmd = F("AT+BROADCAST=\"");
      cmd += packetToBroadcast;
      cmd += "\"";
      
      packetToBroadcast = "";

      sendCommand(cmd);
      
    }
    break;

    case cmdCIFSR:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: REQUEST CIFSR..."));
      #endif

      sendCommand(F("AT+CIFSR"));
      
    }
    break;

    case cmdWantReady:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: reset..."));
      #endif

      // принудительно очищаем очередь клиентов
      clearClientsQueue(true);
      // и говорим, что все слоты свободны
      initPool();
      
      sendCommand(F("AT+RST"));
    }
    break;

    case cmdEchoOff:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: echo OFF..."));
      #endif
      sendCommand(F("ATE0"));
    }
    break;

    case cmdCWMODE:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: softAP mode..."));
      #endif
      sendCommand(F("AT+CWMODE_CUR=3"));
    }
    break;

    case cmdCWSAP:
    {
        #ifdef WIFI_DEBUG
          DEBUG_LOGLN(F("ESP: Creating the access point..."));
        #endif

        GlobalSettings* Settings = MainController->GetSettings();
      
        String com = F("AT+CWSAP_CUR=\"");
        com += Settings->GetStationID();
        com += F("\",\"");
        com += Settings->GetStationPassword();
        com += F("\",8,4");
        
        sendCommand(com);      
    }
    break;

    case cmdCWJAP:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: Connecting to the router..."));
      #endif

        GlobalSettings* Settings = MainController->GetSettings();
        
        String com = F("AT+CWJAP_CUR=\"");
        com += Settings->GetRouterID();
        com += F("\",\"");
        com += Settings->GetRouterPassword();
        com += F("\"");
        sendCommand(com);      
    }
    break;

    case cmdCWQAP:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: Disconnect from router..."));
      #endif
      sendCommand(F("AT+CWQAP"));
    }
    break;

    case cmdCIPMODE:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: Set the TCP server mode to 0..."));
      #endif
      sendCommand(F("AT+CIPMODE=0"));
    }
    break;

    case cmdCIPMUX:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: Allow multiple connections..."));
      #endif
      sendCommand(F("AT+CIPMUX=1"));
    }
    break;

    case cmdCIPSERVER:
    {
      #ifdef WIFI_DEBUG
        Serial.println(F("ESP: Starting TCP-server..."));
      #endif
      sendCommand(F("AT+CIPSERVER=1,1975"));
    }
    break;

    case cmdCheckModemHang:
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: check for ESP available..."));
      #endif
      
      flags.wantReconnect = false;
      sendCommand(F("AT+CWJAP?")); // проверяем, подконнекчены ли к роутеру
    }
    break;

    case cmdCheckCSQ:
    {
    #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: check ESP RSSI..."));
     #endif
      
      sendCommand(F("AT+CSQ")); // получаем уровень сигнала     
    }
    break;
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isESPBootFound(const String& line)
{
  return (line == F("ready")) || line.startsWith(F("Ai-Thinker Technology"));
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isKnownAnswer(const String& line, ESPKnownAnswer& result)
{
  result = kaNone;
  
  if(line == F("OK"))
  {
    result = kaOK;
    return true;
  }
  if(line == F("ERROR"))
  {
    result = kaError;
    return true;
  }
  if(line == F("FAIL"))
  {
    result = kaFail;
    return true;
  }
  if(line.endsWith(F("SEND OK")))
  {
    result = kaSendOk;
    return true;
  }
  if(line.endsWith(F("SEND FAIL")))
  {
    result = kaSendFail;
    return true;
  }
  if(line.endsWith(F("ALREADY CONNECTED")))
  {
    result = kaAlreadyConnected;
    return true;
  }
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::processConnect(const String& line)
{
     // клиент подсоединился
     
    int idx = line.indexOf(F(",CONNECT"));
    
    if(idx == -1)
      return;
    
    String s = line.substring(0,idx);
    int16_t clientID = s.toInt();
    
    if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: client connected - #"));
        DEBUG_LOGLN(String(clientID));
      #endif

      // тут смотрим - посылали ли мы запрос на коннект?
      if(flags.waitCipstartConnect && cipstartConnectClient != NULL && clientID == cipstartConnectClientID)
      {
        #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: WAIT CIPSTART CONNECT, CATCH OUTGOING CLIENT!"));
        #endif
        // есть клиент, для которого надо установить ID.
        // тут у нас может возникнуть коллизия, когда придёт коннект с внешнего адреса.
        // признаком этой коллизии является тот факт, что если мы в этой ветке - мы ОБЯЗАНЫ
        // получить один из известных ответов OK, ERROR, ALREADY CONNECTED
        // ДО ТОГО, как придёт статус ID,CONNECT
        cipstartConnectClient->bind(clientID);
        
        if(!flags.cipstartConnectKnownAnswerFound)
        {
        #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: WAIT CIPSTART CONNECT, NO OK FOUND!"));
        #endif
          
          // не найдено ни одного ответа из известных. Проблема в том, что у внешнего клиента ещё нет слота,
          // но нам надо ему временно выставить слот (мы это сделали выше), потом вызвать событие отсоединения, потом - очистить ему слот
          removeClientFromQueue(cipstartConnectClient);
          notifyClientConnected(*cipstartConnectClient,false,CT_ERROR_CANT_CONNECT);
          cipstartConnectClient->release();

          // при этом, поскольку мы освободили нашего клиента на внешнее соединение и это коллизия,
          // мы должны сообщить, что клиент от ESP подсоединился
          CoreTransportClient* client = getClient(clientID);            
          notifyClientConnected(*client,true,CT_ERROR_NONE);
          
          // поскольку строка ID,CONNECT пришла ДО известного ответа - это коллизия, и мы в ветке cmdCIPSTART,
          // поэтому мы здесь сами должны удалить клиента из очереди и переключиться на ожидание
          machineState = espIdle;
        }
        else
        {
        #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: WAIT CIPSTART CONNECT, CLIENT CONNECTED!"));
        #endif
          
          // если вы здесь - ответ OK получен сразу после команды AT+CIPSTART,
          // клиент из очереди удалён, и, раз мы получили ID,CONNECT - мы можем сообщать, что клиент подсоединён
          CoreTransportClient* client = getClient(clientID);    
          notifyClientConnected(*client,true,CT_ERROR_NONE);          
        }
      
          flags.waitCipstartConnect = false;
          cipstartConnectClient = NULL;
          cipstartConnectClientID = NO_CLIENT_ID;
          flags.cipstartConnectKnownAnswerFound = false;
        
      } // if
      else
      {            
        // если мы здесь - то мы не ждём подсоединения клиента на исходящий адрес
        // просто выставляем клиенту флаг, что он подсоединён
        CoreTransportClient* client = getClient(clientID);            
        notifyClientConnected(*client,true,CT_ERROR_NONE);
      }
      
    } // if
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::processDisconnect(const String& line)
{
  // клиент отсоединился
    int idx = line.indexOf(F(",CLOSED"));
    
    if(idx == -1)
    {
      idx = line.indexOf(F(",CONNECT FAIL"));
    }
      
    if(idx == -1)
    {
      return;
    }
      
    String s = line.substring(0,idx);
    int16_t clientID = s.toInt();
    
    if(clientID >=0 && clientID < ESP_MAX_CLIENTS)
    {
      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: client disconnected - #"));
        DEBUG_LOGLN(String(clientID));
      #endif

      // выставляем клиенту флаг, что он отсоединён
      CoreTransportClient* client = getClient(clientID);            
      notifyClientConnected(*client,false,CT_ERROR_NONE);
      
    }

    // тут смотрим - посылали ли мы запрос на коннект?
    if(flags.waitCipstartConnect && cipstartConnectClient != NULL && clientID == cipstartConnectClientID)
    {

      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: waitCipstartConnect - #"));
        DEBUG_LOGLN(String(clientID));
      #endif
      
      // есть клиент, для которого надо установить ID
      cipstartConnectClient->bind(clientID);
      notifyClientConnected(*cipstartConnectClient,false,CT_ERROR_NONE);
      cipstartConnectClient->release();
      removeClientFromQueue(cipstartConnectClient);
      
      flags.waitCipstartConnect = false;
      cipstartConnectClient = NULL;
      cipstartConnectClientID = NO_CLIENT_ID;
      
    } // if            
          
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::processKnownStatusFromESP(const String& line)
{
   // смотрим, подсоединился ли клиент?
   if(line.endsWith(F(",CONNECT")))
   {
    processConnect(line);
   } // if
   else 
   if(line.endsWith(F(",CLOSED")) || line.endsWith(F(",CONNECT FAIL")))
   {
    processDisconnect(line);
   } // if(idx != -1)
   else
   if(line == F("WIFI CONNECTED"))
   {
      flags.connectedToRouter = true;

     GlobalSettings* settings = MainController->GetSettings();
     TimeSyncSettings* ts = settings->getTimeSyncSettings();

     if(ts->wifiActive)
     {
        initCommandsQueue.push_back(cmdNTPTIME); // добавляем команду на соединение с NTP-сервером и получение с него времени
     }
      
      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: connected to router!"));
      #endif
   }
   else
   if(line == F("WIFI DISCONNECT"))
   {
      flags.connectedToRouter = false;
	  clearClientsQueue (true);
	  initPool();

      #ifdef WIFI_DEBUG
        DEBUG_LOGLN(F("ESP: disconnected from router!"));
      #endif
   }
   else
   if(line.startsWith(F("+NTPTIME:")))
   {
      const char* str = line.c_str();
      str += 9; // переходим за двоеточие

      uint32_t ntpTime = atol(str);

      #ifdef YEAR_30_FIX
        ntpTime -= 946684800ul;
      #endif

      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("ESP: CATCH NTPTIME: "));
        DEBUG_LOGLN(String(ntpTime));
      #endif

      RTCTime tm;
      tm = tm.maketime(ntpTime);   

      RealtimeClock rtc = MainController->GetClock();
      rtc.setTime(tm);     
   }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::checkIPD(const TransportReceiveBuffer& buff)
{
  if(buff.size() < 9) // минимальная длина для IPD, на примере +IPD,1,1:
  {
    return false;
  }

  if(buff[0] == '+' && buff[1] == 'I' && buff[2] == 'P' && buff[3] == 'D')
  {
    size_t to = min(buff.size(),20); // заглядываем вперёд на 20 символов, не больше
    for(size_t i=4;i<to;i++)
    {
      if(buff[i] == ':') // буфер начинается на +IPD и содержит дальше ':', т.е. за ним уже идут данные
      {
        return true;
      }
    }
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::update()
{ 
  if(!workStream) // нет рабочего потока
  {
    return;
  }

  if(flags.onIdleTimer) // попросили подождать определённое время, в течение которого просто ничего не надо делать
  {
      if(millis() - idleTimer > idleTime)
      {
        flags.onIdleTimer = false;
      }
  }

  // читаем из потока всё, что там есть
  readFromStream();

   #ifdef USE_SMS_MODULE
   // и модуль GSM тоже тут обновим
   SIM800.readFromStream();
   #endif   

  RecursionCounter recGuard(&recursionGuard);

  if(recursionGuard > 1) // рекурсивный вызов - просто вычитываем из потока - и всё.
  {
    #ifdef WIFI_DEBUG
      DEBUG_LOGLN(F("ESP: RECURSION!"));
    #endif    
    return;
  }
  
  bool hasAnswer = receiveBuffer.size();

  WiFiBinding bnd = HardwareBinding->GetWiFiBinding();

  if(!hasAnswer)
  {
      // нет ответа от ESP, проверяем, зависла ли она?
      if(millis() - timer > bnd.MaxAnswerTime)
      {
        #ifdef WIFI_DEBUG
          DEBUG_LOGLN(F("ESP: modem not answering, reboot!"));
        #endif

        power(false); // выключаем питание модему
        machineState = espReboot;
        timer = millis();        
      } // if 
  }
  // выставляем флаг, что мы хотя бы раз получили хоть чего-то от ESP
  flags.isAnyAnswerReceived = flags.isAnyAnswerReceived || hasAnswer;

  bool hasAnswerLine = false; // флаг, что мы получили строку ответа от модема    

  String thisCommandLine;

  // тут проверяем, есть ли чего интересующего в буфере?
  if(checkIPD(receiveBuffer))
  {
      
    // в буфере лежит +IPD,ID,DATA_LEN:
      int16_t idx = receiveBuffer.indexOf(','); // ищем первую запятую после +IPD
      uint8_t* ptr = receiveBuffer.pData();
      ptr += idx+1;
      // перешли за запятую, парсим ID клиента
      String connectedClientID;
      while(*ptr != ',')
      {
        connectedClientID += (char) *ptr;
        ptr++;
      }
      ptr++; // за запятую
      String dataLen;
      while(*ptr != ':')
      {
        dataLen += (char) *ptr;
        ptr++;
      }
  
      // получили ID клиента и длину его данных, которые - пока в потоке, и надо их быстро попакетно вычитать
      int ipdClientID = connectedClientID.toInt();
      size_t ipdClientDataLength = dataLen.toInt();

      #ifdef WIFI_DEBUG
        DEBUG_LOG(F("+IPD DETECTED, CLIENT #"));
        DEBUG_LOG(String(ipdClientID));
        DEBUG_LOG(F(", LENGTH="));
        DEBUG_LOGLN(String(ipdClientDataLength));
      #endif

      // удаляем +IPD,ID,DATA_LEN:
      receiveBuffer.remove(0,receiveBuffer.indexOf(':')+1);

      // у нас есть длина данных к вычитке, плюс сколько-то их лежит в буфере уже.
      // читать всё - мы не можем, т.к. данные могут быть гигантскими.
      // следовательно, надо читать по пакетам.
      CoreTransportClient* cl = getClient(ipdClientID);

      if(receiveBuffer.size() >= ipdClientDataLength)
      {
        // на время события мы должны обеспечить неизменность буфера, т.к.
        // в обработчике события может быть вызван yield, у указатель на память станет невалидным!
        
        uint8_t* thisBuffer = new uint8_t[ipdClientDataLength];
        memcpy(thisBuffer,receiveBuffer.pData(),ipdClientDataLength);

        receiveBuffer.remove(0,ipdClientDataLength);

        if(!receiveBuffer.size())
        {
          receiveBuffer.clear();
        }
          
        // весь пакет - уже в буфере
        notifyDataAvailable(*cl, thisBuffer, ipdClientDataLength, true);
        delete [] thisBuffer;
                
      }
      else
      {
        // не хватает части пакета в буфере.
        
        // теперь смотрим, сколько у нас данных ещё не послано клиентам
        size_t remainingDataLength = ipdClientDataLength;

        // нам осталось послать remainingDataLength данных клиентам,
        // побив их на пакеты длиной максимум TRANSPORT_MAX_PACKET_LENGTH

        // вычисляем длину одного пакета
        size_t packetLength = min(TRANSPORT_MAX_PACKET_LENGTH,remainingDataLength);

        uint32_t incomingDataTmr = millis();
        const uint32_t ESP_INCOMING_DATA_TIMEOUT = 60000;

        while(remainingDataLength > 0)
        {

          // NEW CODE //////////////////////////////////////////
          #ifdef USE_EXTERNAL_WATCHDOG
          updateExternalWatchdog();
          #endif
          // NEW CODE //////////////////////////////////////////
                  
            // читаем, пока не хватает данных для одного пакета
            while(receiveBuffer.size() < packetLength)
            {

                if(millis() - incomingDataTmr > ESP_INCOMING_DATA_TIMEOUT)
                {
                  #ifdef WIFI_DEBUG
                    DEBUG_LOGLN(F("[ESP] READING TIMEOUT 1!!!"));            
                  #endif

                  power(false); // выключаем питание модему
                  machineState = espReboot;
                  timer = millis();
                  break;
                }                    

              // NEW CODE //////////////////////////////////////////
              #ifdef USE_EXTERNAL_WATCHDOG
              updateExternalWatchdog();
              #endif
              // NEW CODE //////////////////////////////////////////
              
                #ifdef USE_SMS_MODULE
                  SIM800.readFromStream();
                #endif
              
                // данных не хватает, дочитываем
                if(!workStream->available())
                {
                  continue;
                }
    
                receiveBuffer.push_back((uint8_t) workStream->read());
          
                
            } // while


            if(machineState != espReboot)
            {
                // вычитали один пакет, уведомляем клиентов, при этом может пополниться буфер,
                // поэтому сохраняем пакет так, чтобы указатель на него был всегда валидным.
                uint8_t* thisBuffer = new uint8_t[packetLength];
                memcpy(thisBuffer,receiveBuffer.pData(),packetLength);
    
                receiveBuffer.remove(0,packetLength);
                if(!receiveBuffer.size())
                {
                  receiveBuffer.clear();
                }
    
                notifyDataAvailable(*cl, thisBuffer, packetLength, (remainingDataLength - packetLength) == 0);
                delete [] thisBuffer;
                
                remainingDataLength -= packetLength;
                packetLength = min(TRANSPORT_MAX_PACKET_LENGTH,remainingDataLength);
                
            } // if(machineState != espReboot)

            if(millis() - incomingDataTmr > ESP_INCOMING_DATA_TIMEOUT)
            {
              #ifdef WIFI_DEBUG
                DEBUG_LOGLN(F("[ESP] READING TIMEOUT 2!!!"));            
              #endif

              power(false); // выключаем питание модему
              machineState = espReboot;
              timer = millis();
              break;
            }
                
        } // while

          
      } // else

    
  } // if(checkIPD(receiveBuffer))  
  else if(flags.waitForDataWelcome && receiveBuffer.size() && receiveBuffer[0] == '>')
  {
    flags.waitForDataWelcome = false;
    thisCommandLine = '>';
    hasAnswerLine = true;

    receiveBuffer.remove(0,1);
  }
  else // любые другие ответы от ESP
  {
    // ищем до первого перевода строки
    size_t cntr = 0;
    for(;cntr<receiveBuffer.size();cntr++)
    {
      if(receiveBuffer[cntr] == '\n')
      {          
        hasAnswerLine = true;
        cntr++;
        break;
      }
    } // for

    if(hasAnswerLine) // нашли перевод строки в потоке
    {
      for(size_t i=0;i<cntr;i++)
      {
        if(receiveBuffer[i] != '\r' && receiveBuffer[i] != '\n')
        {
          thisCommandLine += (char) receiveBuffer[i];
        }
      } // for

      receiveBuffer.remove(0,cntr);
      
    } // if(hasAnswerLine)
  } // else

  // если в приёмном буфере ничего нету - просто почистим память
  if(!receiveBuffer.size())
  {
    receiveBuffer.clear();
  }


  if(hasAnswerLine && !thisCommandLine.length()) // пустая строка, не надо обрабатывать
  {
    hasAnswerLine = false;
  }

   #ifdef WIFI_DEBUG
    if(hasAnswerLine)
    {
      DEBUG_LOG(F("<== ESP: "));
      DEBUG_LOGLN(thisCommandLine);
    }
   #endif

    // тут анализируем ответ от ESP, если он есть, на предмет того - соединён ли клиент, отсоединён ли клиент и т.п.
    // это нужно делать именно здесь, поскольку в этот момент в ESP может придти внешний коннект.
    if(hasAnswerLine)
    {
      processKnownStatusFromESP(thisCommandLine);
    }

  // при разборе ответа тут будет лежать тип ответа, чтобы часто не сравнивать со строкой
  ESPKnownAnswer knownAnswer = kaNone;

  if(!flags.onIdleTimer) // только если мы не в режиме простоя
  {
    // анализируем состояние конечного автомата, чтобы понять, что делать
    switch(machineState)
    {
        case espIdle: // ничего не делаем, можем работать с очередью команд и клиентами
        {            
            // смотрим - если есть хоть одна команда в очереди инициализации - значит, мы в процессе инициализации, иначе - можно работать с очередью клиентов
            if(initCommandsQueue.size())
            {
                #ifdef WIFI_DEBUG
                  DEBUG_LOGLN(F("ESP: process next init command..."));
                #endif
                currentCommand = initCommandsQueue[initCommandsQueue.size()-1];
                initCommandsQueue.pop();
                sendCommand(currentCommand);
            } // if
            else
            {
              // в очереди команд инициализации ничего нет, значит, можем выставить флаг, что мы готовы к работе с клиентами
              flags.ready = true;
              
              if(clientsQueue.size())
              {
                  // получаем первого клиента в очереди
                  TransportClientQueueData dt = clientsQueue[0];
                  int clientID = dt.client->socket;
                  
                  // смотрим, чего он хочет от нас
                  switch(dt.action)
                  {
                    case actionDisconnect:
                    {
                      // хочет отсоединиться
                      currentCommand = cmdCIPCLOSE;
                      String cmd = F("AT+CIPCLOSE=");
                      cmd += clientID;
                      sendCommand(cmd);
                      
                    }
                    break; // actionDisconnect

                    case actionConnect:
                    {
                      // мы разрешаем коннектиться только тогда, когда предыдущий коннект клиента уже обработан
                      if(!cipstartConnectClient)
                      {
                          // здесь надо искать первый свободный слот для клиента
                          CoreTransportClient* freeSlot = getClient(NO_CLIENT_ID);
                          clientID = freeSlot ? freeSlot->socket : NO_CLIENT_ID;
                          
                          if(flags.connectedToRouter)
                          {
                            flags.waitCipstartConnect = true;
                            cipstartConnectClient = dt.client;
                            cipstartConnectClientID = clientID;
                            flags.cipstartConnectKnownAnswerFound = false;
      
                            currentCommand = cmdCIPSTART;
                            String comm = F("AT+CIPSTART=");
                            comm += clientID;
                            comm += F(",\"TCP\",\"");
                            comm += dt.ip;
                            comm += F("\",");
                            comm += dt.port;
      
                            delete [] clientsQueue[0].ip;
                            clientsQueue[0].ip = NULL;
                  
                            // и отсылаем её
                            sendCommand(comm);
                          } // flags.connectedToRouter
                          else
                          {
                            // не законнекчены к роутеру, не можем устанавливать внешние соединения!!!
                            removeClientFromQueue(dt.client);
                            dt.client->bind(clientID);
                            notifyClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
                            dt.client->release();
                            
                          }
                      } // if(!cipstartConnectClient)
                    }
                    break; // actionConnect

                    case actionWrite:
                    {
                      // хочет отослать данные

                      currentCommand = cmdCIPSEND;

                      size_t dataSize;
                      uint8_t* buffer = dt.client->getBuffer(dataSize);
                      dt.client->releaseBuffer();

                      clientsQueue[0].data = buffer;
                      clientsQueue[0].dataLength = dataSize;

                      String command = CIPSEND_COMMAND;
                      command += clientID;
                      command += F(",");
                      command += dataSize;
                      flags.waitForDataWelcome = true; // выставляем флаг, что мы ждём >
                      
                      sendCommand(command);
                      
                    }
                    break; // actionWrite
                  } // switch
              }
              else
              {
                timer = millis(); // обновляем таймер в режиме ожидания, поскольку мы не ждём ответа на команды

                static uint32_t csqTimer = 0;
                if(millis() - csqTimer > 20000)
                {
                  csqTimer = millis();
                  sendCommand(cmdCheckCSQ);
                }
                else
                {

                  if(flags.connectedToRouter && (millis() - internalPingTimer >= WIFI_PING_FREQUENCY))
                  {

                    // пора пинговать соединение с интернетом
                    sendCommand(cmdInternalPing);
                                        
                  }
                  else
                  {
                           // у нас прошла инициализация, нет клиентов в очереди на обработку, следовательно - мы можем проверять модем на зависание
                          // тут смотрим - не пора ли послать команду для проверки на зависание. Слишком часто её звать нельзя, что очевидно,
                          // поэтому мы будем звать её минимум раз в N секунд. При этом следует учитывать, что мы всё равно должны звать эту команду
                          // вне зависимости от того, откликается ли ESP или нет, т.к. в этой команде мы проверяем - есть ли соединение с роутером.
                          // эту проверку надо делать периодически, чтобы форсировать переподсоединение, если роутер отвалился.
                          static uint32_t hangTimer = 0;
                          if(millis() - hangTimer > bnd.AvailableCheckTime)
                          {
                            hangTimer = millis();
                            sendCommand(cmdCheckModemHang);
                            
                          } // if
                          else if(packetToBroadcast != "")
                          {
                            sendCommand(cmdBROADCAST);                                    
                          }
                          else
                          {
                            GlobalSettings* settings = MainController->GetSettings();
                            TimeSyncSettings* ts = settings->getTimeSyncSettings();
                            uint32_t tsInterval = ts->wifiInterval;
                            tsInterval *= 3600000ul;
        
                            if(ts->wifiActive)
                            {
                               static uint32_t ntpTimer = 0;
                               if(millis() - ntpTimer > tsInterval)
                               {
                                  ntpTimer = millis();
                                  sendCommand(cmdNTPTIME);
                               }
                            } // if(ts->wifiActive)
                          } // else

                  } // else
                  
                } // else
                
              } // else
            } // else inited
        }
        break; // espIdle

        case espWaitAnswer: // ждём ответа от модема на посланную ранее команду (функция sendCommand переводит конечный автомат в эту ветку)
        {
          // команда, которую послали - лежит в currentCommand, время, когда её послали - лежит в timer.
              if(hasAnswerLine)
              {                
                // есть строка ответа от модема, можем её анализировать, в зависимости от посланной команды (лежит в currentCommand)
                switch(currentCommand)
                {
                  case cmdNone:
                  {
                    // ничего не делаем
                  }
                  break; // cmdNone

                  case cmdInternalPing:
                  {
                     if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(kaError == knownAnswer)
                      {
                        badPingAttempts++;
                      }
                      else
                      if(kaOK == knownAnswer)
                      {
                        badPingAttempts = 0;                        
                      }

                      internalPingTimer = millis();
                      machineState = espIdle; // переходим к следующей команде

                      if(badPingAttempts >= WIFI_BAD_PING_ATTEMPTS)
                      {
                        #ifdef WIFI_DEBUG
                          DEBUG_LOG(F("ESP: BAD "));
                          DEBUG_LOG(String(WIFI_BAD_PING_ATTEMPTS));
                          DEBUG_LOGLN(F(" PING ATTEMPTS, NEED TO RESTART!"));
                        #endif

                        power(false); // выключаем питание модему
                        machineState = espReboot;
                        timer = millis();
                        badPingAttempts = 0;                        
                      }
                    }
                  }
                  break; // cmdInternalPing

                  case cmdPING:
                  {
                    // ждали ответа на пинг
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      flags.specialCommandDone = true;
                      specialCommandResults.push_back(new String(thisCommandLine.c_str()));
                      machineState = espIdle; // переходим к следующей команде
                    }
                    
                  }
                  break; // cmdPING

                  case cmdNTPTIME:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdNTPTIME

                  case cmdBROADCAST:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdBROADCAST                  

                  case cmdCIFSR:
                  {
                    // ждём выполнения команды CIFSR
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      flags.specialCommandDone = true;
                      machineState = espIdle; // переходим к следующей команде
                    }
                    else if(thisCommandLine.startsWith(F("+CIFSR:")))
                    {
                      specialCommandResults.push_back(new String(thisCommandLine.c_str()));
                    }
                  }
                  break; // cmdCIFSR

                  case cmdCIPCLOSE:
                  {
                    // отсоединялись. Здесь не надо ждать известного ответа, т.к. ответ может придти асинхронно
                      if(clientsQueue.size())
                      {
                        // клиент отсоединён, ставим ему соответствующий флаг, освобождаем его и удаляем из очереди
                        TransportClientQueueData dt = clientsQueue[0];

                        CoreTransportClient* thisClient = dt.client;
                        removeClientFromQueue(thisClient);

                      } // if(clientsQueue.size()) 
                      
                        machineState = espIdle; // переходим к следующей команде

                  }
                  break; // cmdCIPCLOSE

                  case cmdCIPSTART:
                  {
                    // соединялись, коннект у нас только с внутреннего соединения, поэтому в очереди лежит по-любому
                    // указатель на связанного с нами клиента, который использует внешний пользователь транспорта
                    
                        if(isKnownAnswer(thisCommandLine,knownAnswer))
                        {
                          if(knownAnswer == kaOK || knownAnswer == kaError || knownAnswer == kaAlreadyConnected)
                          {
                            flags.cipstartConnectKnownAnswerFound = true;
                          }
                            
                          if(knownAnswer == kaOK)
                          {
                            // законнектились удачно, после этого должна придти строка ID,CONNECT
                            if(clientsQueue.size())
                            {
                               TransportClientQueueData dt = clientsQueue[0];
                               removeClientFromQueue(dt.client);                              
                            }
                          }
                          else
                          {
                              
                            if(clientsQueue.size())
                            {
                               #ifdef WIFI_DEBUG
                                DEBUG_LOG(F("ESP: Client connect ERROR, received: "));
                                DEBUG_LOGLN(thisCommandLine);
                               #endif
                               
                               TransportClientQueueData dt = clientsQueue[0];

                               CoreTransportClient* thisClient = dt.client;
                               removeClientFromQueue(thisClient);

                               // если мы здесь, то мы получили ERROR или ALREADY CONNECTED сразу после команды
                               // AT+CIPSTART. Это значит, что пока у внешнего клиента нет ID, мы его должны
                               // временно назначить, сообщить клиенту, и освободить этот ID.
                               thisClient->bind(cipstartConnectClientID);                               
                               notifyClientConnected(*thisClient,false,CT_ERROR_CANT_CONNECT);
                               thisClient->release();
                            }

                            // ошибка соединения, строка ID,CONNECT нас уже не волнует
                            flags.waitCipstartConnect = false;
                            cipstartConnectClient = NULL;
                            
                          } // else
                          machineState = espIdle; // переходим к следующей команде
                        }                    
                    
                  }
                  break; // cmdCIPSTART


                  case cmdWaitSendDone:
                  {
                    // дожидаемся результата отсыла данных
                      
                      if(isKnownAnswer(thisCommandLine,knownAnswer))
                      {
                        if(knownAnswer == kaSendOk)
                        {
                          // send ok
                          if(clientsQueue.size())
                          {
                             TransportClientQueueData dt = clientsQueue[0];
                             
                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);

                             // очищаем данные у клиента
                             thisClient->clear();

                             notifyDataWritten(*thisClient,CT_ERROR_NONE);
                          }                     
                        } // send ok
                        else
                        {
                          // send fail
                          if(clientsQueue.size())
                          {
                             TransportClientQueueData dt = clientsQueue[0];

                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);
                                                          
                             // очищаем данные у клиента
                             thisClient->clear();
                             
                             notifyDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                          }                     
                        } // else send fail
  
                        machineState = espIdle; // переходим к следующей команде
                        
                      } // if(isKnownAnswer(*wiFiReceiveBuff,knownAnswer))
                       

                  }
                  break; // cmdWaitSendDone

                  case cmdCIPSEND:
                  {
                    // тут отсылали запрос на запись данных с клиента
                    if(thisCommandLine == F(">"))
                    {
                       // дождались приглашения, можем писать в ESP
                       // тут пишем напрямую
                       if(clientsQueue.size())
                       {
                          // говорим, что ждём окончания отсыла данных
                          currentCommand = cmdWaitSendDone;                          
                          TransportClientQueueData dt = clientsQueue[0];

                          #ifdef WIFI_DEBUG
                            DEBUG_LOG(F("ESP: > RECEIVED, CLIENT #"));
                            DEBUG_LOG(String(dt.client->socket));
                            DEBUG_LOG(F("; LENGTH="));
                            DEBUG_LOGLN(String(dt.dataLength));
                          #endif

                          for(size_t kk=0;kk<dt.dataLength;kk++)
                          {
                            workStream->write(dt.data[kk]);
                            readFromStream();

                             #ifdef USE_SMS_MODULE
                             // и модуль GSM тоже тут обновим
                             SIM800.readFromStream();
                             #endif                             
                          }
                          
                          delete [] clientsQueue[0].data;
                          delete [] clientsQueue[0].ip;
                          clientsQueue[0].data = NULL;
                          clientsQueue[0].ip = NULL;
                          clientsQueue[0].dataLength = 0;

                          // очищаем данные у клиента сразу после отсыла
                          dt.client->clear();
                       }
                    } // if
                    else
                    if(thisCommandLine.indexOf(F("FAIL")) != -1 || thisCommandLine.indexOf(F("ERROR")) != -1)
                    {
                        #ifdef WIFI_DEBUG
                          DEBUG_LOGLN(F("ESP: WRITE ERROR DETECTED!"));
                        #endif
                                               
                       // всё плохо, не получилось ничего записать
                      if(clientsQueue.size())
                      {
                         
                         TransportClientQueueData dt = clientsQueue[0];

                         CoreTransportClient* thisClient = dt.client;
                         removeClientFromQueue(thisClient);

                         #ifdef WIFI_DEBUG
                          DEBUG_LOG(F("ESP: CLIENT WRITE ERROR #"));
                          DEBUG_LOGLN(String(thisClient->socket));
                         #endif

                         // очищаем данные у клиента
                         thisClient->clear();

                         notifyDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                        
                      }                     
                      
                      machineState = espIdle; // переходим к следующей команде
              
                    } // else can't write
                    
                  }
                  break; // cmdCIPSEND
                  
                  case cmdWantReady: // ждём загрузки модема в ответ на команду AT+RST
                  {
                    if(isESPBootFound(thisCommandLine))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: BOOT FOUND!!!"));
                      #endif
                      
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdWantReady

                  case cmdEchoOff:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: Echo OFF command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdEchoOff

                  case cmdCWMODE:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CWMODE command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }
                  }
                  break; // cmdCWMODE

                  case cmdCWSAP:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CWSAP command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCWSAP

                  case cmdCWJAP:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {

                      machineState = espIdle; // переходим к следующей команде

                      if(knownAnswer != kaOK)
                      {
                        // ошибка подсоединения к роутеру
                        #ifdef WIFI_DEBUG
                          DEBUG_LOGLN(F("ESP: CWJAP command FAIL, RESTART!"));
                        #endif
                        restart();
						
						            flags.onIdleTimer = true;
                        idleTimer = millis();
                        idleTime = 5000;
                      }
                      else
                      {
                        // подсоединились успешно
                        #ifdef WIFI_DEBUG
                          DEBUG_LOGLN(F("ESP: CWJAP command processed."));
                        #endif                        
                      }
                  
                    }  
                  }
                  break; // cmdCWJAP

                  case cmdCWQAP:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CWQAP command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCWQAP

                  case cmdCIPMODE:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CIPMODE command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPMODE

                  case cmdCIPMUX:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CIPMUX command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPMUX
                  
                  case cmdCIPSERVER:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: CIPSERVER command processed."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                    }  
                  }
                  break; // cmdCIPSERVER

                  case cmdCheckCSQ:
                  {
                    if(thisCommandLine.startsWith(F("+CSQ: ")))
                    {
                      signalQuality = 0; // нет сигнала
                      
                      // получили уровень сигнала
                      thisCommandLine.remove(0,6);
                      int dBm = thisCommandLine.toInt();

                        #ifdef WIFI_DEBUG
                          DEBUG_LOG(F("ESP signal quality, dBm: "));
                          DEBUG_LOGLN(String(dBm));
                        #endif

                        if(dBm < 0)
                        {

                          // теперь пересчитываем в значение 0-4, что соответствует:
                          // 0 - нет сигнала
                          // 4 - отличный сигнал
                          if(dBm >= -55)
                          {
                            signalQuality = 4;
                          }
                          else if(dBm >= -66)
                          {
                            signalQuality = 3;
                          }
                          else if(dBm >= -77)
                          {
                            signalQuality = 2;
                          }
                          else if(dBm >= -88)
                          {
                            signalQuality = 1;
                          }
                          else
                          {
                            signalQuality = 0;
                          }

                          #ifdef WIFI_DEBUG
                            DEBUG_LOG(F("ESP signal quality, computed: "));
                            DEBUG_LOGLN(String(signalQuality));
                          #endif                            
                            
                        }
                     
                    }
                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                              DEBUG_LOGLN(F("ESP: CSQ received."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде
                      
                    } // if(isKnownAnswer                    
                  }
                  break; // cmdCheckCSQ

                  case cmdCheckModemHang:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef WIFI_DEBUG
                        DEBUG_LOGLN(F("ESP: ESP answered and available."));
                      #endif
                      machineState = espIdle; // переходим к следующей команде

                       if(flags.wantReconnect)
                       {
                          // требуется переподсоединение к роутеру. Проще всего это сделать вызовом restart - тогда весь цикл пойдёт с начала
                          restart();

                          // чтобы часто не дёргать реконнект - мы говорим, что после рестарта надо подождать 5 секунд перед тем, как обрабатывать следующую команду
                          #ifdef WIFI_DEBUG
                            DEBUG_LOGLN(F("ESP: Wait 5 seconds before reconnect..."));
                          #endif
                          flags.onIdleTimer = true;
                          idleTimer = millis();
                          idleTime = 5000;
                          
                       } // if(flags.wantReconnect)
                      
                    } // if(isKnownAnswer

                     if(thisCommandLine == F("No AP"))
                     {
                        signalQuality = 0;
                        GlobalSettings* Settings = MainController->GetSettings();
                        if(Settings->GetWiFiState() & 0x01)
                        {
                          #ifdef WIFI_DEBUG
                            DEBUG_LOGLN(F("ESP: No connect to router, want to reconnect..."));
                          #endif
                          // нет соединения с роутером, надо переподсоединиться, как только это будет возможно.
                          flags.wantReconnect = true;
                          flags.connectedToRouter = false;
                        }
                      
                     } // if
                      else
                      {
                        // на случай, когда ESP не выдаёт WIFI CONNECTED в порт - проверяем статус коннекта тут,
                        // как признак, что строчка содержит ID нашей сети, проще говоря - не равна No AP
                        if(thisCommandLine.startsWith(F("+CWJAP")))
                        {
                          flags.connectedToRouter = true;
                        }
                        
                      }
                    
                  }
                  break; // cmdCheckModemHang
                                    
                } // switch

                
              } // if(hasAnswerLine)
              
         
        }
        break; // espWaitAnswer

        case espReboot:
        {
          // ждём перезагрузки модема          
          if(millis() - timer > bnd.RebootTime)
          {
            #ifdef WIFI_DEBUG
              DEBUG_LOGLN(F("ESP: turn power ON!"));
            #endif

            power(true); // включаем питание модему
            machineState = espWaitInit;
            timer = millis();
            
          } // if
        }
        break; // espReboot

        case espWaitInit:
        {
          if(millis() - timer > bnd.WaitAfterRebootTime)
          {            
            restart();
            #ifdef WIFI_DEBUG
              DEBUG_LOGLN(F("ESP: inited after reboot!"));
            #endif
          } // 
        }
        break;
      
    } // switch

  } // if(!flags.onIdleTimer)

    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::clearSpecialCommandResults()
{
  for(size_t i=0;i<specialCommandResults.size();i++)
  {
    delete specialCommandResults[i];
  }
  specialCommandResults.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::power(bool on)
{

#ifdef WIFI_DEBUG
  DEBUG_LOG(F("ESP: power("));
  DEBUG_LOG(String(on));
  DEBUG_LOGLN(F(");"));
#endif
  
  WiFiBinding bnd = HardwareBinding->GetWiFiBinding();

  if(bnd.RebootPinLinkType != linkUnbinded && bnd.RebootPin != UNBINDED_PIN)
  {
      if(bnd.RebootPinLinkType == linkDirect)
      {
          #ifndef DISABLE_WIFI_CONFIGURE
          if(EEPROMSettingsModule::SafePin(bnd.RebootPin))
          #else
          if(bnd.RebootPin > 1) // prevent Serial locking
          #endif
          {
             WORK_STATUS.PinMode(bnd.RebootPin,OUTPUT);
             WORK_STATUS.PinWrite(bnd.RebootPin,on ? bnd.PowerOnLevel: !bnd.PowerOnLevel);
          }
      }
      else
      if(bnd.RebootPinLinkType == linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(bnd.RebootPinMCPAddress,bnd.RebootPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(bnd.RebootPinMCPAddress,bnd.RebootPin,on ? bnd.PowerOnLevel: !bnd.PowerOnLevel);
        #endif
      }
      else
      if(bnd.RebootPinLinkType == linkMCP23017)
      {
          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
            WORK_STATUS.MCP_I2C_PinMode(bnd.RebootPinMCPAddress,bnd.RebootPin,OUTPUT);
            WORK_STATUS.MCP_I2C_PinWrite(bnd.RebootPinMCPAddress,bnd.RebootPin, on ? bnd.PowerOnLevel: !bnd.PowerOnLevel);
          #endif
      }
      
  } // if(bnd.RebootPinLinkType != linkUnbinded && bnd.RebootPin != UNBINDED_PIN)    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::begin()
{

  #ifdef WIFI_DEBUG
   DEBUG_LOGLN(F("ESP: begin."));
  #endif


  #if TARGET_BOARD == STM32_BOARD
    SerialUART* hs = NULL;
  #else
    HardwareSerial* hs = NULL;
  #endif  

  WiFiBinding bnd = HardwareBinding->GetWiFiBinding();

  if(bnd.SerialNumber == 1) // Serial1
  {
    hs = &Serial1;
  }
  else
  if(bnd.SerialNumber == 2) // Serial2
  {
    hs = &Serial2;
  }
  else
  if(bnd.SerialNumber == 3) // Serial3
  {
    hs = &Serial3;
  }
  #if TARGET_BOARD == STM32_BOARD
  else
  if(bnd.SerialNumber == 4) // Serial4
  {
    hs = &Serial4;
  }
  else
  if(bnd.SerialNumber == 5) // Serial5
  {
    hs = &Serial5;
  }
  else
  if(bnd.SerialNumber == 6) // Serial6
  {
    hs = &Serial6;
  }
  #endif
    
  workStream = hs;

  if(hs)
  {
    hs->begin(SERIAL_BAUD_RATE);
  }  

  restart();
  power(false);
  machineState = espReboot;

  #ifdef WIFI_DEBUG
    DEBUG_LOGLN(F("ESP: started."));
  #endif


}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::restart()
{
#ifdef WIFI_DEBUG
  DEBUG_LOGLN(F("ESP: restart();"));
#endif
  
  // очищаем входной буфер
  receiveBuffer.clear();

  // очищаем очередь клиентов, заодно им рассылаем события
  clearClientsQueue(true);

  // т.к. мы ничего не инициализировали - говорим, что мы не готовы предоставлять клиентов
  flags.ready = false;
  flags.isAnyAnswerReceived = false;
  flags.waitForDataWelcome = false;
  flags.connectedToRouter = false;
  flags.wantReconnect = false;
  flags.onIdleTimer = false;

  signalQuality = 0;
  
  timer = millis();

  flags.waitCipstartConnect = false; // не ждёт соединения внешнего клиента
  cipstartConnectClient = NULL;

  currentCommand = cmdNone;
  machineState = espIdle;
  badPingAttempts = 0;
  internalPingTimer = millis();

  // инициализируем очередь командами по умолчанию
 createInitCommands(true);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::createInitCommands(bool addResetCommand)
{  
#ifdef WIFI_DEBUG
  DEBUG_LOGLN(F("ESP: createInitCommands();"));
#endif  

  // очищаем очередь команд
  clearInitCommands();


  GlobalSettings* Settings = MainController->GetSettings();
 
  if(Settings->GetWiFiState() & 0x01) // коннектимся к роутеру
  {
    GlobalSettings* Settings = MainController->GetSettings();     
    if(Settings->GetRouterID().length() > 0 &&   Settings->GetRouterPassword().length() > 0)
    {      
      initCommandsQueue.push_back(cmdCWJAP); // коннектимся к роутеру совсем в конце
    }
  }
  else  
  {
    initCommandsQueue.push_back(cmdCWQAP); // отсоединяемся от роутера
  }
    
  initCommandsQueue.push_back(cmdCIPSERVER); // сервер поднимаем в последнюю очередь
  initCommandsQueue.push_back(cmdCIPMUX); // разрешаем множественные подключения
  initCommandsQueue.push_back(cmdCIPMODE); // устанавливаем режим работы
  initCommandsQueue.push_back(cmdCWSAP); // создаём точку доступа
  initCommandsQueue.push_back(cmdCWMODE); // // переводим в смешанный режим
  initCommandsQueue.push_back(cmdEchoOff); // выключаем эхо
  
  if(addResetCommand)
  {
    initCommandsQueue.push_back(cmdWantReady); // надо получить ready от модуля путём его перезагрузки      
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::clearInitCommands()
{
  initCommandsQueue.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::clearClientsQueue(bool raiseEvents)
{  

#ifdef WIFI_DEBUG
  DEBUG_LOGLN(F("ESP: clearClientsQueue();"));
#endif  
  
  // тут попросили освободить очередь клиентов.
  // для этого нам надо выставить каждому клиенту флаг того, что он свободен,
  // плюс - сообщить, что текущее действие над ним не удалось.  

    for(size_t i=0;i<clientsQueue.size();i++)
    {
        TransportClientQueueData dt = clientsQueue[i];
        delete [] dt.data;
        delete [] dt.ip;

        // если здесь в очереди есть хоть один клиент с неназначенным ID (ждёт подсоединения) - то в события он не придёт,
        // т.к. там сравнивается по назначенному ID. Поэтому мы назначаем ID клиенту в первый свободный слот.
        if(dt.client->socket == NO_CLIENT_ID)
        {
          CoreTransportClient* cl = getClient(NO_CLIENT_ID);
          if(cl)
          {
            dt.client->bind(cl->socket);
          }
        }
        
        if(raiseEvents)
        {
          switch(dt.action)
          {
            case actionDisconnect:
                // при дисконнекте всегда считаем, что ошибок нет
                notifyClientConnected(*(dt.client),false,CT_ERROR_NONE);
            break;
  
            case actionConnect:
                // если было запрошено соединение клиента с адресом - говорим, что соединиться не можем
                notifyClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
            break;
  
            case actionWrite:
              // если попросили записать данные - надо сообщить подписчикам, что не можем записать данные
              notifyDataWritten(*(dt.client),CT_ERROR_CANT_WRITE);
              notifyClientConnected(*(dt.client),false,CT_ERROR_NONE);
            break;
          } // switch
          

        } // if(raiseEvents)

        dt.client->clear();
        
    } // for

  clientsQueue.clear();

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::isClientInQueue(CoreTransportClient* client, TransportClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
    {
      return true;
    }
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::addClientToQueue(CoreTransportClient* client, TransportClientAction action, const char* ip, uint16_t port)
{
  while(isClientInQueue(client, action))
  {
    #ifdef WIFI_DEBUG
      DEBUG_LOG(F("ESP: Client #"));
      DEBUG_LOG(String(client->socket));
      DEBUG_LOG(F(" with same action already in queue, ACTION="));
      DEBUG_LOG(String(action));
      DEBUG_LOGLN(F(" - remove that client!"));
    #endif
    removeClientFromQueue(client,action);
  }

    TransportClientQueueData dt;
    dt.client = client;
    dt.action = action;
    
    dt.ip = NULL;
    if(ip)
    {
      dt.ip = new char[strlen(ip)+1];
      strcpy(dt.ip,ip);
    }
    dt.port = port;

    clientsQueue.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::removeClientFromQueue(CoreTransportClient* client, TransportClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
    {
      delete [] clientsQueue[i].ip;
      delete [] clientsQueue[i].data;
      client->clear();
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::removeClientFromQueue(CoreTransportClient* client)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client)
    {
      
      delete [] clientsQueue[i].ip;
      delete [] clientsQueue[i].data;
      client->clear();
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginWrite(CoreTransportClient& client)
{
  // добавляем клиента в очередь на запись
  addClientToQueue(&client, actionWrite);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с записью в поток с этого клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
  if(client.connected())
  {
    
    #ifdef WIFI_DEBUG
      DEBUG_LOGLN(F("ESP: client already connected!"));
    #endif
    return;
    
  }
  
  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionConnect, ip, port);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с соединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreESPTransport::beginDisconnect(CoreTransportClient& client)
{
  if(!client.connected())
  {
    #ifdef WIFI_DEBUG
      DEBUG_LOGLN(F("ESP: beginDisconnect(), client NOT CONNECTED!"));
    #endif     
    return;
  }

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionDisconnect);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с отсоединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreESPTransport::ready()
{
  return workStream != NULL && flags.ready && flags.isAnyAnswerReceived; // если мы полностью инициализировали ESP - значит, можем работать
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
// CoreMQTT
//--------------------------------------------------------------------------------------------------------------------------------------
CoreMQTT::CoreMQTT()
{
  timer = 0;
  machineState = mqttWaitClient;
  currentTransport = NULL;
  mqttMessageId = 0;
  streamBuffer = new String();
  currentTopicNumber = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::AddTopic(const char* topicIndex, const char* topicName, const char* moduleName, const char* sensorType, const char* sensorIndex, const char* topicType)
{

  #ifdef MQTT_DEBUG
    DEBUG_LOG(F("Add topic: "));
    DEBUG_LOGLN(topicName);
    DEBUG_LOGLN(moduleName);
    DEBUG_LOGLN(sensorType);
    DEBUG_LOGLN(sensorIndex);
    DEBUG_LOGLN(topicType);
  #endif
    
  // добавляем новый топик
  String fName = MQTT_FILENAME_PATTERN;
  fName += topicIndex;

  String dirName = F("MQTT");
  SDFat.mkdir(dirName.c_str()); // create directory
  yield();
  
  SdFile f;
  if(f.open(fName.c_str(),FILE_WRITE | O_TRUNC))
  {
    yield();
    f.println(topicName); // имя топика
    yield();
    f.println(moduleName); // имя модуля
    yield();
    f.println(sensorType); // тип датчика
    yield();
    f.println(sensorIndex); // индекс датчика
    yield();
    f.println(topicType); // тип топика
    yield();
    f.close();
    yield();
  }
  #ifdef MQTT_DEBUG
  else
  {
    DEBUG_LOG(F("Unable to create topic file: "));
    DEBUG_LOGLN(fName);
  }
  #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::DeleteAllTopics()
{
  // удаляем все топики
  FileUtils::RemoveFiles(F("MQTT"));
  currentTopicNumber = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------
uint16_t CoreMQTT::GetSavedTopicsCount()
{
  if(!MainController->HasSDCard()) // нет SD-карты, деградируем в жёстко прошитые настройки
    return 0;

    String folderName = F("MQTT");
    return FileUtils::CountFiles(folderName);
        
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::reset()
{

  currentClient.disconnect();
  timer = 0;
  machineState = mqttWaitClient;
  mqttMessageId = 0;

  clearReportsQueue();
  clearPublishQueue();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::clearPublishQueue()
{
  for(size_t i=0;i<publishList.size();i++)
  {
    delete [] publishList[i].payload;
    delete [] publishList[i].topic;
  }

  publishList.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::processIncomingPacket(CoreTransportClient* client, uint8_t* packet, size_t dataLen)
{
  UNUSED(client);
  
  if(!dataLen)
    return;

  if(dataLen > 0)
  {

    uint8_t bCommand = packet[0];
    if((bCommand & MQTT_PUBLISH_COMMAND) == MQTT_PUBLISH_COMMAND)
    {
      // это к нам опубликовали топик
      #ifdef MQTT_DEBUG
        DEBUG_LOGLN(F("MQTT: PUBLISH topic found!!!"));
      #endif

      bool isQoS1 = (bCommand & 6) == MQTT_QOS1;

      // декодируем длину сообщения
      
        uint32_t multiplier = 1;
        int16_t remainingLength = 0;
        uint16_t curReadPos = 1;
        uint8_t encodedByte;
        
        do
        {
          encodedByte =  packet[curReadPos];
          curReadPos++;
          
          remainingLength += (encodedByte & 127) * multiplier;
          multiplier *= 128;
          
        if (multiplier > 0x200000)
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: malformed 1."));
          #endif          
          break; // malformed
        }
          
        } while ((encodedByte & 128) != 0);


      if(curReadPos >= dataLen) // malformed
      {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: malformed 2."));
          #endif          
        return;
      }

      // теперь получаем имя топика
      uint8_t topicLengthMSB = packet[curReadPos];    
      curReadPos++;

      if(curReadPos >= dataLen) // malformed
      {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: malformed 3."));
          #endif          
        return;
      }
            
      uint8_t topicLengthLSB = packet[curReadPos];
      curReadPos++;

      uint16_t topicLength = (topicLengthMSB<<8)+topicLengthLSB;
      
      // теперь собираем топик
      String topic;
      for(uint16_t j=0;j<topicLength;j++)
      {
        if(curReadPos >= dataLen) // malformed
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: malformed 4."));
          #endif          
          return;
        }        
        topic += (char) packet[curReadPos];
        curReadPos++;
      }

      // тут работаем с payload, склеивая его с топиком
      if(isQoS1)
      {
       // игнорируем ID сообщения
       curReadPos += 2; // два байта на ID сообщения
      }

      String* payload = new String();

      for(size_t p=curReadPos;p<dataLen;p++)
      {
        (*payload) += (char) packet[p];
      }

      if(payload->length())
      {
            #ifdef MQTT_DEBUG
              DEBUG_LOG(F("MQTT: Payload are: "));
              DEBUG_LOGLN(*payload);
            #endif

          // теперь склеиваем payload с топиком
          if(topic.length() && topic[topic.length()-1] != '/')
          {
            if((*payload)[0] != '/')
              topic += '/';
          }

          topic += *payload;
      }
      
      delete payload;
      
      if(topic.length())
      {
            #ifdef MQTT_DEBUG
              DEBUG_LOG(F("MQTT: Topic are: "));
              DEBUG_LOGLN(topic);
            #endif

          const char* setCommandPtr = strstr_P(topic.c_str(),(const char*) F("SET/") );
          const char* getCommandPtr = strstr_P(topic.c_str(),(const char*) F("GET/") );
          bool isSetCommand = setCommandPtr != NULL;
          bool isGetCommand = getCommandPtr != NULL;

          if(isSetCommand || isGetCommand)
          {
            const char* normalizedTopic = isSetCommand ? setCommandPtr : getCommandPtr;

            // нашли команду SET или GET, перемещаемся за неё

            // удаляем ненужные префиксы
            topic.remove(0,(normalizedTopic - topic.c_str()) + 4 );

            for(uint16_t k=0;k<topic.length();k++)
            {
              if(topic[k] == '/')
              {
                  topic[k] = '|';             
              }
            } // for

              #ifdef MQTT_DEBUG
                DEBUG_LOG(F("Normalized topic are: "));
                DEBUG_LOGLN(topic);
              #endif

              delete streamBuffer;
              streamBuffer = new String();

              yield();
              ModuleInterop.QueryCommand(isSetCommand ? ctSET : ctGET , topic, false);
              yield();
              
              if(PublishSingleton.Flags.Status)
                *streamBuffer = OK_ANSWER;
              else
                *streamBuffer = ERR_ANSWER;

             *streamBuffer += '=';
  
            int idx = topic.indexOf(PARAM_DELIMITER);
            if(idx == -1)
              *streamBuffer += topic;
            else
              *streamBuffer += topic.substring(0,idx);
            
            if(PublishSingleton.Text.length())
            {
              *streamBuffer += "|";
              *streamBuffer += PublishSingleton.Text;
            }                

              pushToReportQueue(streamBuffer);
            
          } // if(isSetCommand || isGetCommand)
          else // unsupported topic
          {
              #ifdef MQTT_DEBUG
                DEBUG_LOG(F("Unsupported topic: "));
                DEBUG_LOGLN(topic);
              #endif
          } // else
          
      } // if(topic.length())
      else
      {
        #ifdef MQTT_DEBUG
          DEBUG_LOGLN(F("Malformed topic name!!!"));
        #endif
      }

    } // if((bCommand & MQTT_PUBLISH_COMMAND) == MQTT_PUBLISH_COMMAND)
    
  } // if(dataLen > 0)    
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::pushToReportQueue(String* toReport)
{
  
  String* newReport = new String();
  *newReport = *toReport;

#ifdef MQTT_DEBUG
  DEBUG_LOG(F("MQTT: Want to report - "));
  DEBUG_LOGLN(*newReport);
#endif  

  reportQueue.push_back(newReport);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone)
{
  
  if(!currentClient || client != currentClient) // не наш клиент
  {
    return;
  }

  timer = millis();

  if(machineState == mqttWaitSendConnectPacketDone)
  {
    if(isDone)
    {
      machineState = mqttSendSubscribePacket;
    }
  }
  else
  if(machineState == mqttWaitSendSubscribePacketDone)
  {
    if(isDone)
    {
      machineState = mqttSendPublishPacket;
    }
  }
  else
  if(machineState == mqttWaitSendPublishPacketDone)
  {
    // отсылали пакет публикации, тут к нам пришла обратка,
    // поскольку мы подписались на все топики для нашего клиента, на будущее
    
      for(size_t i=0;i<dataSize;i++)
      {
        packetBuffer.push_back(data[i]);
      }
          
      if(!isDone) // ещё не все данные получены
      {
        return;
      }

     machineState = mqttSendPublishPacket;

     #ifdef MQTT_DEBUG
      DEBUG_LOGLN(F("MQTT: process incoming packet..."));
     #endif

    // по-любому обрабатываем обратку
    processIncomingPacket(&currentClient, packetBuffer.pData(), packetBuffer.size());

    packetBuffer.clear();
  }
  else
  {
     #ifdef MQTT_DEBUG
      DEBUG_LOGLN(F("MQTT: process incoming packet on idle mode..."));
     #endif    
      // тут разбираем, что пришло от брокера. Если мы здесь, значит данные от брокера
      // пришли в необрабатываемую ветку, т.е. это публикация прямо с брокера
      
      
      for(size_t i=0;i<dataSize;i++)
      {
        packetBuffer.push_back(data[i]);
      }
          
      if(!isDone) // ещё не все данные получены
      {
        return;
      }
      
      processIncomingPacket(&currentClient, packetBuffer.pData(), packetBuffer.size());

      packetBuffer.clear();
  }

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientDataWritten(CoreTransportClient& client, int16_t errorCode)
{

  UNUSED(errorCode);
  
  if(!currentClient || client != currentClient) // не наш клиент
  {
    return;
  }
  
  timer = millis();
   
  if(errorCode != CT_ERROR_NONE)
  {
    #ifdef MQTT_DEBUG
      DEBUG_LOGLN(F("MQTT: Can't write to client!"));
    #endif
    clearReportsQueue();
    clearPublishQueue();
    packetBuffer.clear();
    machineState = mqttWaitReconnect;

    return;
  }
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode)
{
  UNUSED(errorCode);
  
  if(!currentClient || client != currentClient) // не наш клиент
  {
    return;
  }

  if(!connected)
  {
    // клиент не подсоединился, сбрасываем текущего клиента и вываливаемся в ожидание переподсоединения.
    #ifdef MQTT_DEBUG
      DEBUG_LOGLN(F("MQTT: Disconnected from broker, try to reconnect..."));
    #endif
    
    clearReportsQueue();
    clearPublishQueue();
    packetBuffer.clear();
    machineState = mqttWaitReconnect;
    timer = millis();    
  }
  else
  {
    // клиент подсоединён, переходим на отсыл пакета с авторизацией
    machineState = mqttSendConnectPacket;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::convertAnswerToJSON(const String& answer, String* resultBuffer)
{
  // тут мы должны сформировать объект JSON из ответа, для этого надо разбить ответ по разделителям, и для каждого параметра создать именованное поле
  // в анонимном JSON-объекте
  // прикинем, сколько нам памяти надо резервировать, чтобы вместиться
  int16_t neededJsonLen = 3; // {} - под скобки и завершающий ноль
  // считаем кол-во параметров ответа
  int16_t jsonParamsCount=1; // всегда есть один ответ
  int16_t answerLen = answer.length();
  
  for(int16_t j=0;j<answerLen;j++)
  {
    if(answer[j] == '|') // разделитель
      jsonParamsCount++;
  }
  // у нас есть количество параметров, под каждый параметр нужно минимум 6 символов ("p":""), плюс длина числа, которое будет как имя
  // параметра, плюс длина самого параметра, плюс запятые между параметрами
  int16_t paramNameCharsCount = jsonParamsCount > 9 ? 2 : 1;

   neededJsonLen += (6 + paramNameCharsCount)*jsonParamsCount + (jsonParamsCount-1) + answer.length();

   // теперь можем резервировать память
   resultBuffer->reserve(neededJsonLen);

   // теперь формируем наш JSON-объект
   *resultBuffer = '{'; // начали объект

    if(answerLen > 0)
    {
       int16_t currentParamNumber = 1;

       *resultBuffer += F("\"p");
       *resultBuffer += currentParamNumber;
       *resultBuffer += F("\":\"");
       
       for(int16_t j=0;j<answerLen;j++)
       {
         if(answer[j] == '|')
         {
           // достигли нового параметра, закрываем предыдущий и формируем новый
           currentParamNumber++;
           *resultBuffer += F("\",\"p");
           *resultBuffer += currentParamNumber;
           *resultBuffer += F("\":\"");
         }
         else
         {
            char ch = answer[j];
            
            if(ch == '"' || ch == '\\')
              *resultBuffer += '\\'; // экранируем двойные кавычки и обратный слеш
              
            *resultBuffer += ch;
         }
       } // for

       // закрываем последний параметр
       *resultBuffer += '"';
    } // answerLen > 0

   *resultBuffer += '}'; // закончили объект

}
//--------------------------------------------------------------------------------------------------------------------------------
bool CoreMQTT::publish(const char* topicName, const char* payload)
{
  
  if(!enabled() || !currentTransport || !currentClient || !topicName) // выключены
    return false; 
    
  MQTTPublishQueue pq;
  int16_t tnLen = strlen(topicName);
  pq.topic = new char[tnLen+1];
  memset(pq.topic,0,tnLen+1);
  strcpy(pq.topic,topicName);

  pq.payload = NULL;
  if(payload)
  {
    int16_t pllen = strlen(payload);
    pq.payload = new char[pllen+1];
    memset(pq.payload,0,pllen+1);
    strcpy(pq.payload,payload);    
  }

  publishList.push_back(pq);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------
bool CoreMQTT::enabled()
{
  if(!MainController->HasSDCard()) // нет SD
  {
    return false;
  }

  // проверяем, выключен ли клиент MQTT в настройках
  byte en = MemRead(MQTT_ENABLED_FLAG_ADDRESS);
  if(en == 0xFF)
    en = 0;

  if(!en)
  {
    return false; // выключены в настройках, не надо ничего делать
  }
    
  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------
MQTTSettings CoreMQTT::getSettings()
{
    MQTTSettings result;
    // Тут читаем настройки с SD
    String mqttSettingsFileName = F("mqtt.ini");

    SdFile f;
    if(f.open(mqttSettingsFileName.c_str(),FILE_READ))
    {
      yield();
      // первые две строки - адрес сервера и порт        
      FileUtils::readLine(f,result.serverAddress);
      String dummy;
      FileUtils::readLine(f,dummy);
      result.port = dummy.toInt();

      // в третьей строке - ID клиента
      FileUtils::readLine(f,result.clientID);

      // в четвёртой - пользователь
      FileUtils::readLine(f,result.userName);
      
      // в пятой - пароль
      FileUtils::readLine(f,result.password);
      
      f.close();
    } // if(f) 

    if(!result.clientID.length())
      result.clientID = DEFAULT_MQTT_CLIENT;


  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::reloadSettings()
{
  currentSettings = getSettings();
  intervalBetweenTopics = MemRead(MQTT_INTERVAL_BETWEEN_TOPICS_ADDRESS);
  
  if(!intervalBetweenTopics || intervalBetweenTopics == 0xFF)
    intervalBetweenTopics = 10; // 10 секунд по умолчанию на публикацию между топиками

  intervalBetweenTopics *= 1000;

  if(currentClient.connected())
    currentClient.disconnect();
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::update()
{
  if(!enabled() || !currentTransport) // выключены
  {
    return; 
  }
  
  switch(machineState)
  {
    
      case mqttWaitClient:
      {
        if(currentTransport->ready())
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: Start connect!"));
          #endif
            currentClient.connect(currentSettings.serverAddress.c_str(), currentSettings.port);
            machineState = mqttWaitConnection; 
            timer = millis();
        } // if(currentTransport->ready())
      }
      break; // mqttWaitClient

      case mqttWaitConnection:
      {
        uint32_t toWait = 20000;        
        if(millis() - timer > toWait)
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOG(F("MQTT: unable to connect within "));
            DEBUG_LOG(String(toWait/1000));
            DEBUG_LOGLN(F(" seconds, try to reconnect..."));
          #endif
          
          // долго ждали, переподсоединяемся
          clearReportsQueue();
          clearPublishQueue();
          machineState = mqttWaitReconnect;
          timer = millis();
        }
      }
      break; // mqttWaitConnection

      case mqttWaitReconnect:
      {
        if(millis() - timer > 10000)
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: start reconnect!"));
          #endif
          clearReportsQueue();
          clearPublishQueue();
          machineState = mqttWaitClient;
        }
      }
      break; // mqttWaitReconnect

      case mqttSendConnectPacket:
      {
        if(currentClient.connected())
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: start send connect packet!"));
          #endif  
  
          String mqttBuffer;
          int16_t mqttBufferLength;
          
          constructConnectPacket(mqttBuffer,mqttBufferLength,
            currentSettings.clientID.c_str() // client id
          , currentSettings.userName.length() ? currentSettings.userName.c_str() : NULL // user
          , currentSettings.password.length() ? currentSettings.password.c_str() : NULL // pass
          , NULL // will topic
          , 0 // willQoS
          , 0 // willRetain
          , NULL // will message
          );

          // переключаемся на ожидание результата отсылки пакета
          machineState = mqttWaitSendConnectPacketDone;

          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: WRITE CONNECT PACKET TO CLIENT!"));
          #endif
          
          // сформировали пакет CONNECT, теперь отсылаем его брокеру
          currentClient.write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);
         
          timer = millis();
        }  // if(currentClient)
        else
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: client not connected in construct CONNECT packet mode!"));
          #endif
          machineState = mqttWaitReconnect;
          timer = millis();          
        } // no client
        
      }
      break; // mqttSendConnectPacket

      case mqttSendSubscribePacket:
      {

        #ifdef MQTT_DEBUG
          DEBUG_LOGLN(F("MQTT: Subscribe to topics!"));
        #endif

        if(currentClient.connected())
        {
          String mqttBuffer;
          int16_t mqttBufferLength;

          // конструируем пакет подписки
          String topic = currentSettings.clientID;
          topic +=  F("/#");
          constructSubscribePacket(mqttBuffer,mqttBufferLength, topic.c_str());
  
          // переключаемся на ожидание результата отсылки пакета
          machineState = mqttWaitSendSubscribePacketDone;

          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: WRITE SUBSCRIBE PACKET TO CLIENT!"));
          #endif
          
          // сформировали пакет SUBSCRIBE, теперь отсылаем его брокеру
          currentClient.write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);
          timer = millis();
        }
        else
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: client not connected in construct SUBSCRIBE packet mode!"));
          #endif
          machineState = mqttWaitReconnect;
          timer = millis();          
        } // no client
      
      }
      break; // mqttSendSubscribePacket

      case mqttSendPublishPacket:
      {
        // тут мы находимся в процессе публикации, поэтому можем проверять - есть ли топики для репорта
        bool hasReportTopics = reportQueue.size() > 0;
        bool hasPublishTopics = publishList.size() > 0;
        
        uint32_t interval = intervalBetweenTopics;
        if(hasReportTopics || hasPublishTopics || millis() - timer > interval)
        {
          if(currentClient.connected())
          {
            String mqttBuffer;
            int16_t mqttBufferLength;
  
            String topicName, data;
            bool retain = false;

            if(hasReportTopics)
            {
              // у нас есть топик для репорта
              topicName =  currentSettings.clientID + REPORT_TOPIC_NAME;

              // удаляем перевод строки
              reportQueue[0]->trim();

              // тут в имя топика надо добавить запрошенную команду, чтобы в клиенте можно было ориентироваться
              // на конкретные топики отчёта
              int16_t idx = reportQueue[0]->indexOf("=");
              String commandStatus = reportQueue[0]->substring(0,idx);
              reportQueue[0]->remove(0,idx+1);

              // теперь в reportQueue[0] у нас лежит ответ после OK= или ER=
              String delim = PARAM_DELIMITER;
              idx = reportQueue[0]->indexOf(delim);
              if(idx != -1)
              {
                // есть ответ с параметрами, выцепляем первый - это и будет дополнением к имени топика
                topicName += reportQueue[0]->substring(0,idx);
                reportQueue[0]->remove(0,idx);
                *reportQueue[0] = commandStatus + *reportQueue[0];
              }
              else
              {
                // только один ответ - имя команды, без возвращённых параметров
                topicName += *reportQueue[0];
                *reportQueue[0] = commandStatus;
              }
              

              #ifdef MQTT_REPORT_AS_JSON
                convertAnswerToJSON(*(reportQueue[0]),&data);
              #else
                data = *(reportQueue[0]);            
              #endif

              // тут удаляем из очереди первое вхождение отчёта
              if(reportQueue.size() < 2)
                clearReportsQueue();
              else
              {
                  delete reportQueue[0];
                  for(size_t k=1;k<reportQueue.size();k++)
                  {
                    reportQueue[k-1] = reportQueue[k];
                  }
                  reportQueue.pop();
              }
            } // hasReportTopics
            else
            if(hasPublishTopics)
            {

              #ifdef MQTT_RETAIN_TOPICS
                      retain = true;
              #else
                      retain = false;
              #endif
              // есть пакеты для публикации
              MQTTPublishQueue pq = publishList[0];

              // тут публикуем из пакета для публикации
              topicName =  currentSettings.clientID + "/";
              topicName += pq.topic;

              if(pq.payload)
                data = pq.payload;

              // чистим память
              delete [] pq.topic;
              delete [] pq.payload;
              
              // и удаляем из списка
              if(publishList.size() < 2)
                publishList.clear();
              else
              {
                for(size_t kk=1;kk<publishList.size();kk++)
                {
                  publishList[kk-1] = publishList[kk];  
                }
                publishList.pop();
              }
            } // hasPublishTopics
            else
            {
                retain = true;
                // обычный режим работы, отсылаем показания с хранилища
                getNextTopic(topicName,data);

            } // else send topics

              if(data.length() && topicName.length())
              {
                 // конструируем пакет публикации
                 constructPublishPacket(mqttBuffer,mqttBufferLength,topicName.c_str(), data.c_str(), retain); 
      
                // переключаемся на ожидание результата отсылки пакета
                machineState = mqttWaitSendPublishPacketDone;

                #ifdef MQTT_DEBUG
                  DEBUG_LOGLN(F("MQTT: WRITE PUBLISH PACKET TO CLIENT!"));
                #endif

                // сформировали пакет PUBLISH, теперь отсылаем его брокеру
                currentClient.write((uint8_t*) mqttBuffer.c_str(),mqttBufferLength);
                timer = millis();
              }
          }
          else
          {
            #ifdef MQTT_DEBUG
              DEBUG_LOGLN(F("MQTT: client not connected in construct PUBLISH packet mode!"));
            #endif
            machineState = mqttWaitReconnect;
            timer = millis();          
          } // no client          
           
        }
      }
      break; // mqttSendPublishPacket

      case mqttWaitSendConnectPacketDone:
      case mqttWaitSendSubscribePacketDone:
      case mqttWaitSendPublishPacketDone:
      {
        if(millis() - timer > 20000)
        {
          #ifdef MQTT_DEBUG
            DEBUG_LOGLN(F("MQTT: wait for send results timeout, reconnect!"));
          #endif
          // долго ждали результата записи в клиента, переподсоединяемся
          clearReportsQueue();
          clearPublishQueue();
          machineState = mqttWaitReconnect;
          timer = millis();
        }        
      }
      break;
      
    
  } // switch

  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::getNextTopic(String& topicName, String& data)
{
  topicName = "";
  data = "";

  String topicFileName = MQTT_FILENAME_PATTERN;
  topicFileName += String(currentTopicNumber);

  if(!SDFat.exists(topicFileName.c_str())) // нет топика
  {
    yield();
    currentTopicNumber = 0; // переключаемся на первый топик
    return;
  }

  // тут можем читать из файла настроек топика
  SdFile f;
  
  if(!f.open(topicFileName.c_str(),FILE_READ)) // не получилось открыть файл
  {
    yield();
    switchToNextTopic();
    return;          
  }

  // теперь читаем настройки топика
  // первой строкой идёт имя топика
  FileUtils::readLine(f,topicName);

  // добавляем ID клиента перед именем топика
  topicName = currentSettings.clientID + "/" + topicName;  

  // второй строкой - идёт имя модуля, в котором взять нужные показания
  String moduleName;
  FileUtils::readLine(f,moduleName);

  // в третьей строке - тип датчика, числовое значение соответствует перечислению ModuleStates
  String sensorTypeString;
  FileUtils::readLine(f,sensorTypeString);
  ModuleStates sensorType = (ModuleStates) sensorTypeString.toInt();

  // в четвёртой строке - индекс датчика в модуле
  String sensorIndexString;
  FileUtils::readLine(f,sensorIndexString);
  int sensorIndex = sensorIndexString.toInt();

  // в пятой строке - тип топика: показания с датчиков (0), или статус контроллера (1).
  // в случае статуса контроллера во второй строке - команда, которую надо запросить у контроллера
  String topicType;
  FileUtils::readLine(f,topicType);
  
  
  // не забываем закрыть файл
  f.close();
  yield();

  if(topicType == F("1")) // топик со статусом контроллера
  {
   
    #ifdef MQTT_DEBUG
      DEBUG_LOGLN(F("Status topic found - process command..."));
    #endif
        
     //Тут работаем с топиком статуса контроллера

      // тут тонкость - команда у нас с изменёнными параметрами, где все разделители заменены на символ @
      // поэтому перед выполнением - меняем назад
      moduleName.replace('@','|');

      yield();
      ModuleInterop.QueryCommand(ctGET, moduleName, true);
      yield();

      #ifdef MQTT_REPORT_AS_JSON
        convertAnswerToJSON(PublishSingleton.Text,&data);
      #else // ответ как есть, в виде RAW
        data = PublishSingleton.Text;
      #endif

       switchToNextTopic();
      
      return; // нашли и отослали показания

    
  } // if
  else // топик с показаниями датчика
  {
      // теперь получаем модуль у контроллера
      AbstractModule* mod = MainController->GetModuleByID(moduleName.c_str());

      if(!mod) // не нашли такой модуль
      {
        switchToNextTopic();
        return;
      }

      // получаем состояние
      OneState* os = mod->State.GetState(sensorType,sensorIndex);

      if(!os) // нет такого состояния
      {
        switchToNextTopic();
        return;          
      }

      // теперь получаем данные состояния
      if(os->HasData()) // данные с датчика есть, можем читать
        data = *os;
      else
        data = "-"; // нет данных с датчика  

       switchToNextTopic();
      
      return; // нашли и отослали показания
       
  } // sensor data topic    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::switchToNextTopic()
{
    // переключаемся на следующий топик
    currentTopicNumber++;
    
    // проверим - не надо ли завернуть на старт?
    String topicFileName = MQTT_FILENAME_PATTERN;
    topicFileName += String(currentTopicNumber);
    if(!SDFat.exists(topicFileName.c_str()))
    {
      yield();
      currentTopicNumber = 0; // следующего файла нет, начинаем сначала
    }
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::clearReportsQueue()
{
  for(size_t i=0;i<reportQueue.size();i++)
  {
    delete reportQueue[i];
  }

  reportQueue.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructPublishPacket(String& mqttBuffer,int16_t& mqttBufferLength, const char* topic, const char* payload, bool retain)
{
  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // тут формируем пакет

  // кодируем топик
  encode(byteBuffer,topic);

  // теперь пишем данные топика
  int16_t sz = strlen(payload);
  const char* readPtr = payload;
  for(int i=0;i<sz;i++)
  {
    byteBuffer.push_back(*readPtr++);
  }   

  size_t payloadSize = byteBuffer.size();

  MQTTBuffer fixedHeader;
  
  uint8_t command = MQTT_PUBLISH_COMMAND;
  
  if(retain)
    command |= 1;

  #ifdef MQTT_DEBUG
    if(retain)
    {
      DEBUG_LOG(F("MQTT: RETAIN topic detected, byte #0 is: "));
      DEBUG_LOG(String(command));
    }
  #endif
  
  constructFixedHeader(command,fixedHeader,payloadSize);

  writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);
  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructSubscribePacket(String& mqttBuffer,int16_t& mqttBufferLength, const char* topic)
{
  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // тут формируем пакет подписки

  // сначала записываем ID сообщения
  mqttMessageId++;
  
  if(!mqttMessageId)
    mqttMessageId = 1;
    
  byteBuffer.push_back((mqttMessageId >> 8));
  byteBuffer.push_back((mqttMessageId & 0xFF));

  // кодируем топик, на который подписываемся
  encode(byteBuffer,topic);

  // теперь пишем байт QoS
  byteBuffer.push_back(1);

  size_t payloadSize = byteBuffer.size();

  MQTTBuffer fixedHeader;
  
  constructFixedHeader(MQTT_SUBSCRIBE_COMMAND | MQTT_QOS1, fixedHeader, payloadSize);

  writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructConnectPacket(String& mqttBuffer,int16_t& mqttBufferLength,const char* id, const char* user, const char* pass
,const char* willTopic,uint8_t willQoS, uint8_t willRetain, const char* willMessage)
{
  mqttBuffer = "";

  MQTTBuffer byteBuffer; // наш буфер из байт, в котором будет содержаться пакет

  // теперь формируем переменный заголовок

  // переменный заголовок, для команды CONNECT
  byteBuffer.push_back(0);
  byteBuffer.push_back(6); // длина версии протокола MQTT
  byteBuffer.push_back('M');
  byteBuffer.push_back('Q');
  byteBuffer.push_back('I');
  byteBuffer.push_back('s');
  byteBuffer.push_back('d');
  byteBuffer.push_back('p');

  byteBuffer.push_back(3); // версия протокола - 3

  // теперь рассчитываем флаги
  byte flags = 0;

  if(willTopic)
    flags = 0x06 | (willQoS << 3) | (willRetain << 5);
  else
    flags = 0x02;

  if(user) // есть имя пользователя
    flags |= (1 << 7);

  if(pass) // есть пароль
    flags |= (1 << 6);
  
   byteBuffer.push_back(flags);

   // теперь смотрим настройки keep-alive
   int keepAlive = 60; // 60 секунд
   byteBuffer.push_back((keepAlive >> 8));
   byteBuffer.push_back((keepAlive & 0xFF));

   // теперь записываем payload, для этого каждую строку надо закодировать
   encode(byteBuffer,id);
   encode(byteBuffer,willTopic);
   encode(byteBuffer,willMessage);
   encode(byteBuffer,user);
   encode(byteBuffer,pass);

   // теперь мы имеем буфер переменной длины, нам надо подсчитать его длину, сворфировать фиксированный заголовок,
   // и сохранить всё в буфере
   size_t payloadSize = byteBuffer.size();
   MQTTBuffer fixedHeader;
   constructFixedHeader(MQTT_CONNECT_COMMAND,fixedHeader,payloadSize);

   writePacket(fixedHeader,byteBuffer,mqttBuffer,mqttBufferLength);


   // всё, пакет сформирован
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::writePacket(MQTTBuffer& fixedHeader, MQTTBuffer& payload, String& mqttBuffer,int16_t& mqttBufferLength)
{
  mqttBuffer = "";
  
// запомнили, сколько байт надо послать в ESP
   mqttBufferLength = fixedHeader.size() + payload.size();

   // теперь записываем это в строку, перед этим зарезервировав память, и заполнив строку пробелами
   mqttBuffer.reserve(mqttBufferLength);
   for(int16_t i=0;i<mqttBufferLength;i++)
    mqttBuffer += ' ';

  // теперь можем копировать данные в строку побайтово
  int16_t writePos = 0;

  // пишем фиксированный заголовок
  for(size_t i=0;i<fixedHeader.size();i++)
  {
    mqttBuffer[writePos++] = fixedHeader[i];
  }
  
  // и переменный
  for(size_t i=0;i<payload.size();i++)
  {
    mqttBuffer[writePos++] = payload[i];
  }  
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::constructFixedHeader(uint8_t command, MQTTBuffer& fixedHeader, size_t payloadSize)
{
    fixedHeader.push_back(command); // пишем тип команды
  
    uint8_t remainingLength[4];
    uint8_t digit;
    uint8_t written = 0;
    uint16_t len = payloadSize;
    
    do 
    {
        digit = len % 128;
        len = len / 128;
        if (len > 0) 
        {
            digit |= 0x80;
        }
        
        remainingLength[written++] = digit;
        
    } while(len > 0);

    // мы записали written символов, как длину переменного заголовка - теперь пишем эти байты в фиксированный
    
    for(uint8_t i=0;i<written;i++)
    {
      fixedHeader.push_back(remainingLength[i]);
    }

}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::encode(MQTTBuffer& buff,const char* str)
{
  if(!str)
    return;

    size_t sz = buff.size(); // запоминаем текущий размер

    // записываем нули, как длину строки, потом мы это поправим
    buff.push_back(0);
    buff.push_back(0);

    const char* ptr = str;
    int16_t strLen = 0;
    while(*ptr)
    {
      buff.push_back(*ptr++);
      strLen++;
    }

    // теперь записываем актуальную длину
    buff[sz] = (strLen >> 8);
    buff[sz+1] = (strLen & 0xFF);
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreMQTT::begin(CoreTransport* transport)
{
  // попросили начать работу
  // для начала - освободим клиента
  machineState = mqttWaitClient;
  currentTransport = transport;
  currentClient.accept(transport);
  mqttMessageId = 0;

  // подписываемся на события клиентов
  if(currentTransport)
  {
    currentTransport->subscribe(this);  
  }

  reloadSettings();
    
  // ну и запомним, когда вызвали начало работы
  timer = millis();
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SMS_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#include "PDUClasses.h"
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
static void __noincomingcall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp) {}
static void __nosmsreceived(const String& phoneNumber, const String& message, bool isKnownNumber) {}
static void __nocusdreceived(const String& cusd){}
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp) __attribute__ ((weak, alias("__noincomingcall")));
void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber) __attribute__ ((weak, alias("__nosmsreceived")));
void ON_CUSD_RECEIVED(const String& cusd) __attribute__ ((weak, alias("__nocusdreceived")));
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSIM800Transport SIM800;
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSIM800Transport::CoreSIM800Transport() : CoreTransport(SIM800_MAX_CLIENTS)
{

  recursionGuard = 0;
  flags.waitCipstartConnect = false;
  cipstartConnectClient = NULL;
  workStream = NULL;
  signalQuality = 0;
  smsToSend = NULL;
  inWaitingGPRS_CIICR = false;
  waitGprsReconnect = false;

  PSUTTZreceived = false;
  syncTimeTimerEnabled = false;

  flags.isSIMCardPresent = false;

}
//--------------------------------------------------------------------------------------------------------------------------------------
CoreSIM800Transport::~CoreSIM800Transport()
{
  delete smsToSend;
  for(size_t i=0;i<cusdList.size();i++)
  {
    delete cusdList[i];
  }

  clearClientsQueue(false);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::readFromStream()
{
  if(!workStream)
  {
    return;
  }
    
  while(workStream->available())
  {
    receiveBuffer.push_back((uint8_t) workStream->read()); 
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendCommand(const String& command, bool addNewLine)
{
 if(!workStream)
 {
  return;
 }
 
  size_t len = command.length();
  for(size_t i=0;i<len;i++)
  {
    // записали байтик
    workStream->write(command[i]);

    // прочитали, что пришло от SIM800
    readFromStream();

   #ifdef USE_WIFI_MODULE
     // и модуль ESP тоже тут обновим
     ESP.readFromStream();
   #endif     
  }
    
  if(addNewLine)
  {
    workStream->println();
  }
  
  // прочитали, что пришло от SIM800
  readFromStream();

   #ifdef USE_WIFI_MODULE
   // и модуль ESP тоже тут обновим
   ESP.readFromStream();
   #endif   

  #ifdef GSM_DEBUG_MODE
    DEBUG_LOG(F("SIM800: ==> "));
    DEBUG_LOGLN(command);
  #endif

  machineState = sim800WaitAnswer; // говорим, что надо ждать ответа от SIM800
  // запоминаем время отсылки последней команды
  timer = millis();
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::GetAPNUserPass(String& user, String& pass)
{
  user = MainController->GetSettings()->GetGSMAPNUser();
  pass = MainController->GetSettings()->GetGSMAPNPassword();
}
//--------------------------------------------------------------------------------------------------------------------------------
String CoreSIM800Transport::GetAPN()
{
   return MainController->GetSettings()->GetGSMAPNAddress();
}
//--------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendCommand(SIM800Commands command)
{
  currentCommand = command;
  
  // тут посылаем команду в SIM800
  switch(command)
  {
    case smaNone:
    case smaCIPSTART:
    case smaCIPSEND:
    case smaWaitSendDone:
    case smaCIPCLOSE:
    case smaCMGS:
    case smaWaitForSMSClearance:
    case smaWaitSMSSendDone:
    case smaCUSD:
    break;

    case smaEnableSimDetection:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Enable SIM detection..."));
      #endif
      sendCommand(F("AT+CSDT=1"));
    }
    break;

    case smaCheckSimCardPresent:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Check for SIM CARD presence..."));
      #endif
      sendCommand(F("AT+CSMINS?"));
    }
    break;

    case smaCheckReady:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Check for modem READY..."));
      #endif
      sendCommand(F("AT+CPAS"));
    }
    break;

    case smaCIPHEAD:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set IPD setting..."));
      #endif
      sendCommand(F("AT+CIPHEAD=1"));      
    }
    break;

    case smaCIPSHUT:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Deactivate PDP context..."));
      #endif
      sendCommand(F("AT+CIPSHUT"));      
    }
    break;

    case smaCGATT:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Check GPRS availability..."));
      #endif
      sendCommand(F("AT+CGATT?"));      
      
    }
    break;

    case smaCGATTach:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Attach to GPRS..."));
      #endif
      sendCommand(F("AT+CGATT=1"));            
    }
    break;

    case smaCIICR:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Activate GPRS connection..."));
      #endif

      inWaitingGPRS_CIICR = true;
      sendCommand(F("AT+CIICR"));            
    }
    break;

    case smaCIFSR:
    {
      flags.gprsAvailable = false;
      sendCommand(F("AT+CIFSR"));      
    }
    break;
    
  //////////////////////////////
  // синхронизация времени
  //////////////////////////////
    case smaStartTimeSync:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Enable time sync..."));
      #endif  

      sendCommand(F("AT+CLTS=1"));
    }
    break;

    case smaSaveSyncSettings:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Save sync settings..."));
      #endif  

      sendCommand(F("AT&W"));
      
    }
    break;


    case smaGetSyncTime:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Get time..."));
      #endif  

      sendCommand(F("AT+CCLK?"));          
    }
    break;
  
    case smaCSTT:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Setup GPRS connection..."));
      #endif

      String apnUser, apnPass;
      GetAPNUserPass(apnUser, apnPass);
      
      String comm = F("AT+CSTT=\"");
      comm += GetAPN();
      comm += F("\",\"");
      comm += apnUser;
      comm += F("\",\"");
      comm += apnPass;
      comm += F("\"");

      sendCommand(comm);
    }
    break;

    case smaCIPMODE:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set CIPMODE..."));
      #endif
      sendCommand(F("AT+CIPMODE=0"));            
    }
    break;

    case smaCIPMUX:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set CIPMUX..."));
      #endif
      sendCommand(F("AT+CIPMUX=1"));            
    }
    break;

    case smaEchoOff:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: echo OFF..."));
      #endif
      sendCommand(F("ATE0"));
    }
    break;

    case smaDisableCellBroadcastMessages:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: disable cell broadcast messagess..."));
      #endif
      sendCommand(F("AT+CSCB=1"));
    }
    break;

    case smaAON:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Turn AON ON..."));
      #endif    
        sendCommand(F("AT+CLIP=1"));  
    }
    break;

    case smaPDUEncoding:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set PDU format..."));
      #endif
      sendCommand(F("AT+CMGF=0"));
    }
    break;

    case smaUCS2Encoding:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set UCS2 format..."));
      #endif
      sendCommand(F("AT+CSCS=\"UCS2\""));
    }
    break;

    case smaSMSSettings:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set SMS output mode..."));
      #endif
      sendCommand(F("AT+CNMI=2,2"));
    }
    break;

    case smaGPRSMultislotClass:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Set GPRS multislot class..."));
      #endif
      sendCommand(F("AT+CGMSCLASS=4"));
    }
    break;    

    case smaWaitReg:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Check registration status..."));
      #endif
      sendCommand(F("AT+CREG?"));
    }
    break;

    case smaCheckModemHang:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Check if modem available..."));
      #endif

      csqReceived = false;
      sendCommand(F("AT+CSQ"));
    }
    break;

    case smaPING:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Start PING..."));
      #endif
      String cmd = F("AT+CIPPING=\"");
      cmd += GSM_PING_HOST;
      cmd += F("\",2,32,2,64");
      sendCommand(cmd);
    }
    break;
    

    case smaHangUp:
    {
      #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: Hang up...")); 
      #endif     
      sendCommand(F("ATH"));      
    }
    break;
    
  } // switch

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::isKnownAnswer(const String& line, SIM800KnownAnswer& result)
{
  result = gsmNone;
  
  if(line == F("OK"))
  {
    result = gsmOK;
    return true;
  }
  if(line == F("ERROR"))
  {
    result = gsmError;
    return true;
  }
  if(line == F("FAIL"))
  {
    result = gsmFail;
    return true;
  }
  if(line.endsWith(F("SEND OK")))
  {
    result = gsmSendOk;
    return true;
  }
  if(line.endsWith(F("SEND FAIL")))
  {
    result = gsmSendFail;
    return true;
  }
  if(line.endsWith(F("CONNECT OK")))
  {
    result = gsmConnectOk;
    return true;
  }
  if(line.endsWith(F("CONNECT FAIL")))
  {
    result = gsmConnectFail;
    return true;
  }
  if(line.endsWith(F("ALREADY CONNECT")))
  {
    result = gsmAlreadyConnect;
    return true;
  }
  if(line.endsWith(F("CLOSE OK")))
  {
    result = gsmCloseOk;
    return true;
  }
   if(line.endsWith(F("SHUT OK")))
  {
    result = gsmShutOk;
    return true;
  }  
  
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processIncomingCall(const String& line)
{
 // приходит строка вида
  // +CLIP: "79182900063",145,,,"",0
  
   // входящий звонок, проверяем, приняли ли мы конец строки?
    String ring = line.substring(8); // пропускаем команду +CLIP:, пробел и открывающую кавычку "

    int16_t idx = ring.indexOf("\"");
    if(idx != -1)
      ring = ring.substring(0,idx);

    if(ring.length() && ring[0] != '+')
      ring = String(F("+")) + ring;

  // ищем - есть ли у нас этот номер среди известных
  bool isKnownNumber = false;

  String knownNumber = MainController->GetSettings()->GetSmsPhoneNumber();

  if(knownNumber.startsWith(ring))
  {
    isKnownNumber = true;
  }
  else
  {
    // тут проверяем на SD-карточке
    if(MainController->HasSDCard())
    {
       SdFile f;
       String fName;
       fName = F("PHONES.TXT");
       if(f.open(fName.c_str(),FILE_READ))
       {
         while(f.available())
         {
           String line;
           FileUtils::readLine(f,line);
           if(!line.length())
            break;

            if(line.startsWith(ring))
            {
              isKnownNumber = true;
              break;
            }
         }
         f.close();
       }
    } // if(MainController->HasSDCard())
  } // else

  bool shouldHangUp = true;
  
  // вызываем событие
  ON_INCOMING_CALL(ring,isKnownNumber,shouldHangUp);
  
 // добавляем команду "положить трубку" - она выполнится первой, поскольку очередь инициализации у нас имеет приоритет
  if(shouldHangUp)
  {
    initCommandsQueue.push_back(smaHangUp);  
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processCMT(const String& pdu)
{
  if(pdu.length())
  {
    
    PDUIncomingMessage sms = PDU.Decode(pdu);

    if(sms.IsDecodingSucceed)
    {
      // СМС пришло, вызываем событие
      String knownNumber = MainController->GetSettings()->GetSmsPhoneNumber();
      
      bool anyKnownNumbersFound = false;
      if(knownNumber.startsWith(sms.SenderNumber))
      {
        anyKnownNumbersFound = true;
      }
      ON_SMS_RECEIVED(sms.SenderNumber,sms.Message, anyKnownNumbersFound);
    }

    
  } // if(pdu.length())
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendQueuedCUSD()
{
  if(!cusdList.size())
  {
    return;
  }

  String* cusdToSend = cusdList[0];

  for(size_t i=1;i<cusdList.size();i++)
  {
    cusdList[i-1] = cusdList[i];
  }

  cusdList.pop();

  sendCommand(*cusdToSend);
  delete cusdToSend;
  currentCommand = smaCUSD;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendQueuedSMS()
{
  if(!outgoingSMSList.size())
  {
    return;
  }

  delete smsToSend;
  smsToSend = new String();
  
  int16_t messageLength = 0;

  SIM800OutgoingSMS* sms = &(outgoingSMSList[0]);  
 
  PDUOutgoingMessage encodedMessage = PDU.Encode(*(sms->phone),*(sms->message),sms->isFlash, smsToSend);
  messageLength = encodedMessage.MessageLength;
    
  delete sms->phone;
  delete sms->message;

  if(outgoingSMSList.size() < 2)
  {
    outgoingSMSList.clear();
  }
  else
  {
    for(size_t i=1;i<outgoingSMSList.size();i++)
    {
      outgoingSMSList[i-1] = outgoingSMSList[i];
    }
    outgoingSMSList.pop();
  }
    
  // тут отсылаем СМС
  String command = F("AT+CMGS=");
  command += String(messageLength);

  flags.waitForDataWelcome = true;
  sendCommand(command);
  currentCommand = smaCMGS;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::processKnownStatusFromSIM800(const String& line)
{
  if(flags.pduInNextLine) // в прошлой строке пришло +CMT, поэтому в текущей - содержится PDU
  {
      flags.pduInNextLine = false;

      // разбираем, чего там пришло
      processCMT(line);

      timer = millis();
      return; 
  }

  // смотрим, подсоединился ли клиент?
   int16_t idx = line.indexOf(F(", CONNECT OK"));
   if(idx != -1)
   {
      // клиент подсоединился
      String s = line.substring(0,idx);
      int16_t clientID = s.toInt();
      if(clientID >=0 && clientID < SIM800_MAX_CLIENTS)
      {          
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOG(F("SIM800: client connected - #"));
          DEBUG_LOGLN(String(clientID));
        #endif

        // тут смотрим - посылали ли мы запрос на коннект?
        if(flags.waitCipstartConnect && cipstartConnectClient != NULL && clientID == cipstartConnectClientID)
        {                
          // есть клиент, для которого надо установить ID
          cipstartConnectClient->bind(clientID);
          flags.waitCipstartConnect = false;
          cipstartConnectClient = NULL;
          cipstartConnectClientID = NO_CLIENT_ID;              
        } // if                 

        // выставляем клиенту флаг, что он подсоединён
        CoreTransportClient* client = getClient(clientID);              
        notifyClientConnected(*client,true,CT_ERROR_NONE);
      }
   } // if

   idx = line.indexOf(F(", CLOSE OK"));
   if(idx == -1)
   {
    idx = line.indexOf(F(", CLOSED"));
   }
   if(idx == -1)
   {
    idx = line.indexOf(F("CONNECT FAIL"));
   }
    
   if(idx != -1)
   {
    // клиент отсоединился
      String s = line.substring(0,idx);
      int16_t clientID = s.toInt();
      if(clientID >=0 && clientID < SIM800_MAX_CLIENTS)
      {
        
        if(line.indexOf(F("CONNECT FAIL")) != -1) // CONNECT FAIL приходит без ID клиента!!!
        {
          clientID = cipstartConnectClientID;
        }
        
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOG(F("SIM800: client disconnected - #"));
          DEBUG_LOGLN(String(clientID));
        #endif

          // выставляем клиенту флаг, что он отсоединён
          CoreTransportClient* client = getClient(clientID);
          notifyClientConnected(*client,false,CT_ERROR_NONE);

        if(flags.waitCipstartConnect && cipstartConnectClient != NULL && clientID == cipstartConnectClientID)
        {            
          // есть клиент, для которого надо установить ID
          cipstartConnectClient->bind(clientID);
          notifyClientConnected(*cipstartConnectClient,false,CT_ERROR_NONE);
          cipstartConnectClient->bind(NO_CLIENT_ID);
          flags.waitCipstartConnect = false;
          cipstartConnectClient = NULL;
          cipstartConnectClientID = NO_CLIENT_ID;
                  
        } // if                           
      }        
    
   } // if(idx != -1)

  if(line.startsWith(F("+CLIP:")))
  {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: +CLIP detected, parse!"));
    #endif
   
    processIncomingCall(line);
    
    timer = millis();        
    return; // поскольку мы сами отработали входящий звонок - выходим
  }
  else
  if(line.startsWith(F("+CMT:")))
  {
      flags.pduInNextLine = true;
      return;
  }
  else
  if(line.startsWith(F("+CUSD:")))
  {
    // пришёл ответ на запрос CUSD
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: +CUSD detected, parse!"));
    #endif
      
    // дождались ответа, парсим
    int quotePos = line.indexOf('"');
    int lastQuotePos = line.lastIndexOf('"');
    
    if(quotePos != -1 && lastQuotePos != -1 && quotePos != lastQuotePos) 
    {
   
      String cusdSMS = line.substring(quotePos+1,lastQuotePos);
      
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOG(F("ENCODED CUSD IS: ")); 
        DEBUG_LOGLN(cusdSMS);
      #endif

      // тут декодируем CUSD
      String decodedCUSD = PDU.getUTF8From16BitEncoding(cusdSMS);

      #ifdef GSM_DEBUG_MODE
        DEBUG_LOG(F("DECODED CUSD IS: ")); 
        DEBUG_LOGLN(decodedCUSD);
      #endif

      ON_CUSD_RECEIVED(decodedCUSD);
    
    }
    return;   
  } // if(line.startsWith(F("+CUSD:")))
  else
  if(line.startsWith(F("+PDP: DEACT")))
  {
    flags.gprsAvailable = false;
    
    if(!inWaitingGPRS_CIICR) // если мы не ждём коннекта по GPRS
    {
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOGLN(F("SIM800: GPRS connection broken, restart!")); 
      #endif

      rebootModem();
    }
      
  } // if(line.startsWith(F("+PDP:DEACT")))
  else
  if(line.startsWith(F("+CSQ: ")))
  {
    signalQuality = 0; // нет сигнала
    csqReceived = true;

    String thisCommandLine = line;
    
    // получили уровень сигнала
    thisCommandLine.remove(0,6);
    int commaIdx = thisCommandLine.indexOf(',');
    if(commaIdx != -1)
    {
      thisCommandLine.remove(commaIdx);
      int quality = thisCommandLine.toInt();

      #ifdef GSM_DEBUG_MODE
        DEBUG_LOG(F("GSM signal quality, raw: "));
        DEBUG_LOGLN(String(quality));
      #endif

      if(quality != 99)
      {
        // если есть сигнал, пересчитываем в dBm
        int dBm = -115 + quality*2;

        #ifdef GSM_DEBUG_MODE
          DEBUG_LOG(F("GSM signal dBm: "));
          DEBUG_LOGLN(String(dBm));
        #endif

        // теперь пересчитываем в значение 0-4, что соответствует:
        // 0 - нет сигнала
        // 4 - отличный сигнал
        if(dBm >= -73)
        {
          signalQuality = 4;
        }
        else if(dBm >= -83)
        {
          signalQuality = 3;
        }
        else if(dBm >= -93)
        {
          signalQuality = 2;
        }
        else
        {
          signalQuality = 1;
        }

        #ifdef GSM_DEBUG_MODE
          DEBUG_LOG(F("GSM signal quality, computed: "));
          DEBUG_LOGLN(String(signalQuality));
        #endif                            
          
      }
    }
  } // CSQ
  else
  if(line.startsWith(F("+CGATT: ")))
  {
    cgattStatusReceived = true;
    cgattStatus = false;
    String tl = line;
    tl.remove(0,8);
    if(tl.toInt() == 1)
    {
      cgattStatus = true;
    }

    if(cgattKnownAnswerFound) // получили +CGATT после ответа ОК
    {
      cgattKnownAnswerFound = false;
      machineState = sim800Idle;
      if(cgattStatus)
      {
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOGLN(F("SIM800: CGATT, start GPRS..."));
      #endif
        
        initCommandsQueue.push_back(smaCSTT);
        initCommandsQueue.push_back(smaCIPSHUT);
      }
      else
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("SIM800: GPRS not attached, try to attach..."));
        #endif
        
        initCommandsQueue.push_back(smaCGATTach);
      }
    }
  } // if(line.startsWith(F("+CGATT: ")))
  else
  if(line.startsWith(F("*PSUTTZ: ")))
  {
    PSUTTZreceived = true;

    GlobalSettings* settings = MainController->GetSettings();
    TimeSyncSettings* ts = settings->getTimeSyncSettings();
    
    if(ts->gsmActive)
    {
    
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("SIM800: CATCH OPERATOR TIME!"));
        #endif    
                
        // пришло время оператора, можно выставлять часы
        String timeline = line;
        timeline.remove(0,9);
        
        // теперь там строка вида 2019,1,21,18,31,58,"+12",0
        // и её надо разбить на лексемы
        Vector<const char*> params;
    
        const char* p = timeline.c_str();
        
        for(size_t i=0;i<timeline.length();i++)
        {
            if(timeline[i] == ',')
            {
              timeline[i] = 0;
              params.push_back(p);
              p = timeline.c_str() + i + 1;
            }
          if(!*p)
          {
            break;
          }
        }
    
        if(*p)
        {
          params.push_back(p);
        }
    
        if(params.size() > 7)
        {
          // разобрали норм
          RTCTime tm;
    
          tm.year = atoi(params[0]);
          tm.month = atoi(params[1]);
          tm.dayOfMonth = atoi(params[2]);
          tm.hour = atoi(params[3]);
          tm.minute = atoi(params[4]);
          tm.second = atoi(params[5]);
          
          String offset = params[6];
          if(offset[0] == '"')
          {
            offset.remove(0,1);
          }
    
          if(offset[offset.length()-1] == '"')
          {
            offset.remove(offset.length()-1,1);
          }
    
          int32_t offset_sec = offset.toInt();
          offset_sec *= 15*60;
    
          uint32_t ut = tm.unixtime();
          ut += offset_sec;
    
          tm = tm.maketime(ut);
    
          RealtimeClock rtc = MainController->GetClock();
    
          rtc.setTime(tm);    
          
        }

    } // if(ts.gsmActive)
    
  } // if(line.startsWith(F("*PSUTTZ: ")))
  else
  if(line.startsWith(F("+CSMINS:")))
  {
    flags.isSIMCardPresent = line.endsWith(F(",1"));

        #ifdef GSM_DEBUG_MODE
          if(!flags.isSIMCardPresent)
          {
            DEBUG_LOGLN(F("SIM800: NO SIM CARD FOUND!"));
          }
          else
          {
            DEBUG_LOGLN(F("SIM800: SIM CARD PRESENCE DETECTED!"));
          }
        #endif  
  } // if(line.startsWith(F("+CSMINS:")))
  else
  if(line.startsWith(F("+CME ERROR:")) || line.startsWith(F("ERROR")))  
  {

    if(currentCommand == smaPING) // посылали пинг, он неудачен
    {
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOGLN(F("SIM800: PING FAIL DETECTED!"));
      #endif
      
      badPingAttempts++;
    }
    
  } // if(line.startsWith(F("+CME ERROR:")))
  else
  if(line.startsWith(F("+CIPPING:")))
  {
    if(currentCommand == smaPING) // посылали пинг, он удачен
    {
      badPingAttempts = 0; 
    }    
  } // if(line.startsWith(F("+CIPPING:")))
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::checkIPD(const TransportReceiveBuffer& buff)
{

  // +RECEIVE BUG FIX /////////////////////////////////////////////
  if(buff.size() < 15) // минимальная длина для RECEIVE, на примере +RECEIVE,1,1: С УЧЁТОМ ПЕРЕВОДА СТРОКИ \r\n, КОТОРЫЙ НАДО ИГНОРИРОВАТЬ !!!
  {
    return false;
  }

  if(buff[0] == '+' && buff[1] == 'R' && buff[2] == 'E' && buff[3] == 'C'  && buff[4] == 'E'
   && buff[5] == 'I'  && buff[6] == 'V'  && buff[7] == 'E')
  {
    size_t to = min(buff.size(),30); // заглядываем вперёд на 30 символов, не больше
    for(size_t i=8;i<to;i++)
    {
      if(buff[i] == ':' && ((i+1) < to && buff[i+1] == '\r') && ((i+2) < to && buff[i+2] == '\n') ) // буфер начинается на +RECEIVE и содержит дальше ':', затем '\r', затем '\n', т.е. за ними уже идут данные
      {
        return true;
      }
    }
  }
  // +RECEIVE BUG FIX /////////////////////////////////////////////


  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef GSM_DEBUG_MODE
void CoreSIM800Transport::dumpReceiveBuffer()
{
  Serial.print(F("SIM800 RECEIVE BUFFER, SIZE="));
  Serial.println(receiveBuffer.size());
  
  for(size_t i=0;i<receiveBuffer.size();i++)
  {
    Serial.print(WorkStatus::ToHex(receiveBuffer[i]));
    Serial.print(" ");
  }

 Serial.println();
  
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::power(bool on)
{
  GSMBinding gbnd = HardwareBinding->GetGSMBinding();

  if(gbnd.RebootPinLinkType != linkUnbinded && gbnd.RebootPin != UNBINDED_PIN)
  {
      if(gbnd.RebootPinLinkType == linkDirect)
      {
          #ifndef DISABLE_GSM_CONFIGURE
          if(EEPROMSettingsModule::SafePin(gbnd.RebootPin))
          #else
          if(gbnd.RebootPin > 1) // prevent Serial locking
          #endif
          {
            WORK_STATUS.PinMode(gbnd.RebootPin,OUTPUT);
            WORK_STATUS.PinWrite(gbnd.RebootPin,on ? gbnd.PowerOnLevel : !gbnd.PowerOnLevel);
          }
      }
      else
      if(gbnd.RebootPinLinkType == linkMCP23S17)
      {
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(gbnd.RebootPinMCPAddress,gbnd.RebootPin,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(gbnd.RebootPinMCPAddress,gbnd.RebootPin,on ? gbnd.PowerOnLevel : !gbnd.PowerOnLevel);
        #endif
      }
      else
      if(gbnd.RebootPinLinkType == linkMCP23017)
      {
          #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
            WORK_STATUS.MCP_I2C_PinMode(gbnd.RebootPinMCPAddress,gbnd.RebootPin,OUTPUT);
            WORK_STATUS.MCP_I2C_PinWrite(gbnd.RebootPinMCPAddress,gbnd.RebootPin,on ? gbnd.PowerOnLevel : !gbnd.PowerOnLevel);
          #endif
      }
    
  } // if(gbnd.RebootPinLinkType != linkUnbinded && gbnd.RebootPin != UNBINDED_PIN)  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::rebootModem()
{
    flags.ready = false; // чтобы никто не тыркался извне
    power(false); // выключаем питание модему
    machineState = sim800Reboot;
    timer = millis();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::update()
{  

  if(!workStream) // нет рабочего потока
  {
    return;
  }

  GSMBinding gbnd = HardwareBinding->GetGSMBinding();

  if(flags.onIdleTimer) // попросили подождать определённое время, в течение которого просто ничего не надо делать
  {
      if(millis() - idleTimer > idleTime)
      {
        flags.onIdleTimer = false;
      }
  }

  // читаем из потока всё, что там есть
  readFromStream();

   #ifdef USE_WIFI_MODULE
   // и модуль ESP тоже тут обновим
   ESP.readFromStream();
   #endif   

  RecursionCounter recGuard(&recursionGuard);

  if(recursionGuard > 1) // рекурсивный вызов - просто вычитываем из потока - и всё.
  {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: RECURSION!"));
    #endif    
    return;
  }
  
  bool hasAnswer = receiveBuffer.size();

  if(!hasAnswer)
  {
      // нет ответа от SIM800, проверяем, завис ли он?

      if(millis() - timer > gbnd.MaxAnswerTime)
      {
        #ifdef GSM_DEBUG_MODE
          DEBUG_LOGLN(F("SIM800: modem not answering, reboot!"));
        #endif

        rebootModem();        
      } // if   
  }

  // выставляем флаг, что мы хотя бы раз получили хоть чего-то от ESP
  flags.isAnyAnswerReceived = flags.isAnyAnswerReceived || hasAnswer;

  bool hasAnswerLine = false; // флаг, что мы получили строку ответа от модема    

  String thisCommandLine;

  // тут проверяем, есть ли чего интересующего в буфере?
  if(checkIPD(receiveBuffer))
  {
      
    // в буфере лежит +RECEIVE,ID,DATA_LEN:
      int16_t idx = receiveBuffer.indexOf(','); // ищем первую запятую после +RECEIVE
      uint8_t* ptr = receiveBuffer.pData();
      ptr += idx+1;
      // перешли за запятую, парсим ID клиента
      String connectedClientID;
      while(*ptr != ',')
      {
        connectedClientID += (char) *ptr;
        ptr++;
      }
      ptr++; // за запятую
      String dataLen;
      while(*ptr != ':')
      {
        dataLen += (char) *ptr;
        ptr++;
      }
  
      // получили ID клиента и длину его данных, которые - пока в потоке, и надо их быстро попакетно вычитать
      int ipdClientID = connectedClientID.toInt();
      size_t ipdClientDataLength = dataLen.toInt();

      #ifdef GSM_DEBUG_MODE
        DEBUG_LOG(F("+RECEIVE DETECTED, CLIENT #"));
        DEBUG_LOG(String(ipdClientID));
        DEBUG_LOG(F(", LENGTH="));
        DEBUG_LOGLN(String(ipdClientDataLength));
      #endif

  // +RECEIVE BUG FIX /////////////////////////////////////////////
      // удаляем +RECEIVE,ID,DATA_LEN:, ВКЛЮЧАЯ ПЕРЕВОД СТРОКИ \r\n !!!
      receiveBuffer.remove(0,receiveBuffer.indexOf(':')+3);
  // +RECEIVE BUG FIX /////////////////////////////////////////////
  
  
      // у нас есть длина данных к вычитке, плюс сколько-то их лежит в буфере уже.
      // читать всё - мы не можем, т.к. данные могут быть гигантскими.
      // следовательно, надо читать по пакетам.
      CoreTransportClient* cl = getClient(ipdClientID);

      if(receiveBuffer.size() >= ipdClientDataLength)
      {
        // на время события мы должны обеспечить неизменность буфера, т.к.
        // в обработчике события может быть вызван yield, у указатель на память станет невалидным!
        
        uint8_t* thisBuffer = new uint8_t[ipdClientDataLength];
        memcpy(thisBuffer,receiveBuffer.pData(),ipdClientDataLength);

        receiveBuffer.remove(0,ipdClientDataLength);

        if(!receiveBuffer.size())
        {
          receiveBuffer.clear();
        }
          
        // весь пакет - уже в буфере
        notifyDataAvailable(*cl, thisBuffer, ipdClientDataLength, true);
        delete [] thisBuffer;
                
      }
      else
      {
        // не хватает части пакета в буфере.
        
        // теперь смотрим, сколько у нас данных ещё не послано клиентам
        size_t remainingDataLength = ipdClientDataLength;

        #ifdef GSM_DEBUG_MODE
            DEBUG_LOG(F("[SIM800] REMAINING BYTES: "));
            DEBUG_LOGLN(String(remainingDataLength));
        #endif         

        // нам осталось послать remainingDataLength данных клиентам,
        // побив их на пакеты длиной максимум TRANSPORT_MAX_PACKET_LENGTH

        // вычисляем длину одного пакета
        size_t packetLength = min(TRANSPORT_MAX_PACKET_LENGTH,remainingDataLength);

        uint32_t incomingDataTmr = millis();

        while(remainingDataLength > 0)
        {

          // NEW CODE //////////////////////////////////////////
          #if defined(USE_EXTERNAL_WATCHDOG)
            if(gbnd.CalmWatchdog == 1)
            {
              updateExternalWatchdog();
            }
          #endif

          
          if(millis() - incomingDataTmr > gbnd.IncomingDataTimeout)
          {
            DEBUG_LOGLN(F("SIM800: READING TIMEOUT #0!!!"));
            rebootModem();
            return;
          }
          
          // NEW CODE //////////////////////////////////////////
                  
            // читаем, пока не хватает данных для одного пакета
            while(receiveBuffer.size() < packetLength)
            {

              // NEW CODE //////////////////////////////////////////
              #if defined(USE_EXTERNAL_WATCHDOG)
                if(gbnd.CalmWatchdog == 1)
                {
                  updateExternalWatchdog();
                }
              #endif

              if(millis() - incomingDataTmr > gbnd.IncomingDataTimeout)
              {
                #ifdef GSM_DEBUG_MODE
                DEBUG_LOGLN(F("SIM800: READING TIMEOUT #1!!!"));                                 
                #endif
                rebootModem();
                return;
              }
              // NEW CODE //////////////////////////////////////////
              
                #ifdef USE_WIFI_MODULE
                  ESP.readFromStream();
                #endif
              
                // данных не хватает, дочитываем
                if(!workStream->available())
                {
                  continue;
                }
    
                receiveBuffer.push_back((uint8_t) workStream->read());
                
            } // while

            // вычитали один пакет, уведомляем клиентов, при этом может пополниться буфер,
            // поэтому сохраняем пакет так, чтобы указатель на него был всегда валидным.
            uint8_t* thisBuffer = new uint8_t[packetLength];
            memcpy(thisBuffer,receiveBuffer.pData(),packetLength);

            receiveBuffer.remove(0,packetLength);
            if(!receiveBuffer.size())
            {
              receiveBuffer.clear();
            }

            notifyDataAvailable(*cl, thisBuffer, packetLength, (remainingDataLength - packetLength) == 0);
            delete [] thisBuffer;
            
            remainingDataLength -= packetLength;
            packetLength = min(TRANSPORT_MAX_PACKET_LENGTH,remainingDataLength);
        } // while

  #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("[SIM800] REMAINING BYTES READED."));
  #endif  
            
      } // else
    
  } // if(checkIPD(receiveBuffer))
  else if(flags.waitForDataWelcome && receiveBuffer.size() && receiveBuffer[0] == '>')
  {
    flags.waitForDataWelcome = false;
    thisCommandLine = '>';
    hasAnswerLine = true;

    receiveBuffer.remove(0,1);
  }
  else // любые другие ответы от SIM800
  {
    // ищем до первого перевода строки
    size_t cntr = 0;
    for(;cntr<receiveBuffer.size();cntr++)
    {
      if(receiveBuffer[cntr] == '\n')
      {          
        hasAnswerLine = true;
        cntr++;
        break;
      }
    } // for

    if(hasAnswerLine) // нашли перевод строки в потоке
    {
      for(size_t i=0;i<cntr;i++)
      {
        if(receiveBuffer[i] != '\r' && receiveBuffer[i] != '\n')
        {
          thisCommandLine += (char) receiveBuffer[i];
        }
      } // for

      receiveBuffer.remove(0,cntr);
      
    } // if(hasAnswerLine)
  } // else

  // если в приёмном буфере ничего нету - просто почистим память
  if(!receiveBuffer.size())
  {
    receiveBuffer.clear();
  }

  if(hasAnswerLine && thisCommandLine.startsWith(F("AT+")))
   {
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOG(F("Ignored echo: "));
      DEBUG_LOGLN(thisCommandLine);    
    #endif    
    // это эхо, игнорируем
      thisCommandLine = "";
      hasAnswerLine = false;
   }

   if(hasAnswerLine && flags.ignoreNextEmptyLine)
   {
      flags.ignoreNextEmptyLine = false;
      if(thisCommandLine == " ")
      {
        hasAnswerLine = false;
        thisCommandLine = "";
      }   
   }


  if(hasAnswerLine && !thisCommandLine.length()) // пустая строка, не надо обрабатывать
  {
    hasAnswerLine = false;
  }

   #ifdef GSM_DEBUG_MODE
    if(hasAnswerLine)
    {
      DEBUG_LOG(F("<== SIM800: "));
      DEBUG_LOGLN(thisCommandLine);
    }
   #endif

    // тут анализируем ответ от ESP, если он есть, на предмет того - соединён ли клиент, отсоединён ли клиент и т.п.
    // это нужно делать именно здесь, поскольку в этот момент в SIM800 может придти внешний коннект.
    if(hasAnswerLine)
    {
      processKnownStatusFromSIM800(thisCommandLine);
    } 

    // тут проверяем - возможно, кучу неудачных попыток пропинговать соединение
    if(badPingAttempts >= 10)
    {
      #ifdef GSM_DEBUG_MODE
        DEBUG_LOGLN(F("SIM800: 10 BAD PING ATTEMPTS, REBOOT MODEM!"));
      #endif

      badPingAttempts = 0;
      rebootModem();
      return;
    }

    // при разборе ответа тут будет лежать тип ответа, чтобы часто не сравнивать со строкой
    SIM800KnownAnswer knownAnswer = gsmNone;

    if(!flags.onIdleTimer)
    {
    // анализируем состояние конечного автомата, чтобы понять, что делать
    switch(machineState)
    {
        case sim800Idle: // ничего не делаем, можем работать с очередью команд и клиентами
        {            
            // смотрим - если есть хоть одна команда в очереди инициализации - значит, мы в процессе инициализации, иначе - можно работать с очередью клиентов
            if(initCommandsQueue.size())
            {
                #ifdef GSM_DEBUG_MODE
                  DEBUG_LOGLN(F("SIM800: process next command..."));
                #endif
                
                currentCommand = initCommandsQueue[initCommandsQueue.size()-1];
                initCommandsQueue.pop();

                if(currentCommand == smaCGATT && !flags.isSIMCardPresent) // не стоит посылать команду коннекта к GPRS, если нет SIM-карты
                {
                    #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CAN'T DO GPRS WITHOUT SIM-card!"));
                    #endif                     
                  return;
                }
                
                sendCommand(currentCommand);
            } // if
            else
            {
              // в очереди команд инициализации ничего нет, значит, можем выставить флаг, что мы готовы к работе с клиентами
              flags.ready = true;

              if(outgoingSMSList.size())
              {
                sendQueuedSMS();
              }
              else
              if(cusdList.size())
              {
                sendQueuedCUSD();
              }
              else
              if(clientsQueue.size())
              {
                  // получаем первого клиента в очереди
                  TransportClientQueueData dt = clientsQueue[0];
                  int16_t clientID = dt.client->socket;
                  
                  // смотрим, чего он хочет от нас
                  switch(dt.action)
                  {
                    case actionDisconnect:
                    {
                      // хочет отсоединиться

                      currentCommand = smaCIPCLOSE;
                      String cmd = F("AT+CIPCLOSE=");
                      cmd += clientID;
                      sendCommand(cmd);                      
                      
                    }
                    break; // actionDisconnect

                    case actionConnect:
                    {
                        // мы разрешаем коннектиться только тогда, когда предыдущий коннект клиента уже обработан
                        if(!cipstartConnectClient)
                        {
                          // хочет подсоединиться
                          if(flags.gprsAvailable)
                          {
                            // здесь надо искать первый свободный слот для клиента
                            CoreTransportClient* freeSlot = getClient(NO_CLIENT_ID);
                            clientID = freeSlot ? freeSlot->socket : NO_CLIENT_ID;
      
                            flags.waitCipstartConnect = true;
                            cipstartConnectClient = dt.client;
                            cipstartConnectClientID = clientID;
                            
                            currentCommand = smaCIPSTART;
                            String comm = F("AT+CIPSTART=");
                            comm += clientID;
                            comm += F(",\"TCP\",\"");
                            comm += dt.ip;
                            comm += F("\",");
                            comm += dt.port;
      
                            delete [] clientsQueue[0].ip;
                            clientsQueue[0].ip = NULL;
                                            
                            // и отсылаем её
                            sendCommand(comm);
                            
                          } // gprsAvailable
                          else
                          {
                            // нет GPRS, не можем устанавливать внешние соединения!!!
                            ////// NEW CODE //////
                            // ТУТ СИТУАЦИЯ: НАС ПОПРОСИЛИ ЗАКОННЕКТИТЬСЯ, КОГДА НЕТ GPRS СОЕДИНЕНИЯ.
                            // ПРИ ЭТОМ У НАС В ОЧЕРЕДИ КЛИЕНТ НА КОННЕКТ, ЕЩЁ БЕЗ ПРИВЯЗКИ К СЛОТУ, которому мы должны сказать, что коннект - не удался.
                            // поэтому его НЕОБХОДИМО забиндить на первый свободный слот, на время отработки события.
                            
                            // здесь надо искать первый свободный слот для клиента
                            CoreTransportClient* freeSlot = getClient(NO_CLIENT_ID);
                            clientID = freeSlot ? freeSlot->socket : NO_CLIENT_ID;                            
                            ////// NEW CODE //////
                                                        
                            removeClientFromQueue(dt.client);
                            dt.client->bind(clientID);
                            notifyClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
                            dt.client->release();                        
                          }
                          
                       } // if(!cipstartConnectClient)
                    }
                    break; // actionConnect

                    case actionWrite:
                    {
                      // хочет отослать данные

                      currentCommand = smaCIPSEND;

                      size_t dataSize;
                      uint8_t* buffer = dt.client->getBuffer(dataSize);
                      clientsQueue[0].data = buffer;
                      clientsQueue[0].dataLength = dataSize;
                      dt.client->releaseBuffer();
                      
                      String command = F("AT+CIPSEND=");
                      command += clientID;
                      command += F(",");
                      command += dataSize;
                      flags.waitForDataWelcome = true; // выставляем флаг, что мы ждём >
                      
                      sendCommand(command);                
                    }
                    break; // actionWrite
                  } // switch
              }
              else
              {
                timer = millis(); // обновляем таймер в режиме ожидания, поскольку мы не ждём ответа на команды

                if(waitGprsReconnect)
                {
                  if(millis() - gprsReconnectTimer > GPRS_RECONNECT_TIME)
                  {
                    waitGprsReconnect = false;

                      if(flags.isSIMCardPresent)
                      {
                            #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: TRY TO RECONNECT GPRS!"));
                            #endif         
                          
                            String apn = GetAPN();
                            if(apn.length())
                            {
                                cgattStatusReceived = false;
                                cgattKnownAnswerFound = false;
                                initCommandsQueue.push_back(smaCGATT);
    
                            }
                      
                      } // flags.isSIMCardPresent
                  }
                } // waitGprsReconnect
                
                GlobalSettings* settings = MainController->GetSettings();
                TimeSyncSettings* ts = settings->getTimeSyncSettings();
                uint32_t gsmSyncInterval = ts->gsmInterval;
                gsmSyncInterval *= 3600000ul;
                
                if(ts->gsmActive && syncTimeTimerEnabled && (millis() - syncTimeTimer > gsmSyncInterval))
                {
                  syncTimeTimerEnabled = false;
                  
                  if(flags.isSIMCardPresent)
                  {
                    #ifdef GSM_DEBUG_MODE
                      DEBUG_LOGLN(F("SIM800: want to SYNC TIME !!!"));
                    #endif
                    
                    initCommandsQueue.push_back(smaGetSyncTime);
                  }
                }
                

                static uint32_t hangTimer = 0;
                if(millis() - hangTimer > gbnd.AvailableCheckTime)
                {
                  #ifdef GSM_DEBUG_MODE
                    DEBUG_LOGLN(F("SIM800: want to check modem availability..."));
                  #endif
                  hangTimer = millis();
                  sendCommand(smaCheckModemHang);
                  
                } // if
                else
                {
                  if(gbnd.PullGPRSByPing == 1)
                  {
                    static uint32_t pingTimer = 0;
                    if(millis() - pingTimer > gbnd.PingInterval)
                    {
                        pingTimer = millis();
                        if(flags.gprsAvailable)
                        {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: PING GPRS!"));
                          #endif
                          sendCommand(smaPING);                      
                        }
                    }
                  } // if(gbnd.PullGPRSByPing == 1)
                } // else
                
              } // else
            } // else inited
        }
        break; // sim800Idle

        case sim800WaitAnswer: // ждём ответа от модема на посланную ранее команду (функция sendCommand переводит конечный автомат в эту ветку)
        {
          // команда, которую послали - лежит в currentCommand, время, когда её послали - лежит в timer.
              if(hasAnswerLine)
              {                
                // есть строка ответа от модема, можем её анализировать, в зависимости от посланной команды (лежит в currentCommand)
                switch(currentCommand)
                {
                  case smaNone:
                  {
                    // ничего не делаем
                  }
                  break; // cmdNone

                  case smaCUSD:
                  {
                    // отсылали запрос CUSD
                    if(isKnownAnswer(thisCommandLine,knownAnswer) || thisCommandLine == F("+CUSD: 4"))
                    {
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CUSD WAS SENT."));
                      #endif
                      machineState = sim800Idle;
                    }                     
                  }
                  break; // smaCUSD

                  case smaCheckSimCardPresent:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                       #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CSMINS command processed."));
                      #endif
                      
                      machineState = sim800Idle;
                    }
                  }
                  break;

                  case smaEnableSimDetection:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                       #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CSDT command processed."));
                      #endif
                      
                      machineState = sim800Idle;
                    }
                  }
                  break;

                  case smaWaitSMSSendDone:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: SMS was sent."));
                      #endif
                      sendCommand(F("AT+CMGD=1,4"));
                      currentCommand = smaWaitForSMSClearance;
                    }                       
                  }
                  break; // smaWaitSMSSendDone

                  case smaWaitForSMSClearance:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: SMS cleared."));
                      #endif
                      machineState = sim800Idle;
                    }                       
                    
                  }
                  break; // smaWaitForSMSClearance

                  case smaCMGS:
                  {
                    // отсылаем SMS
                    
                          if(thisCommandLine == F(">")) 
                          {
                            
                            // дождались приглашения, можно посылать    
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Welcome received, continue sending..."));
                            #endif

                            sendCommand(*smsToSend,false);
                            workStream->write(0x1A); // посылаем символ окончания посыла

                            delete smsToSend;
                            smsToSend = new String();

                            currentCommand = smaWaitSMSSendDone;
                          } 
                          else 
                          {
                  
                            #ifdef GSM_DEBUG_MODE
                              DEBUG_LOG(F("SIM800: BAD ANWER TO SMS COMMAND: WANT '>', RECEIVED: "));
                              DEBUG_LOGLN(thisCommandLine);
                           #endif

                            delete smsToSend;
                            smsToSend = new String();
                            
                            // пришло не то, что ждали - просто игнорируем отсыл СМС
                             machineState = sim800Idle;
                              
                          }
                  }
                  break; // smaCMGS

                  case smaCIPCLOSE:
                  {
                    // отсоединялись. Ответа не ждём, т.к. может вклиниться всё, что угодно, пока мы ждём ответа
                    
                      if(clientsQueue.size())
                      {
                        #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: Client disconnected."));
                        #endif

                        TransportClientQueueData dt = clientsQueue[0];                        
                        CoreTransportClient* thisClient = dt.client;
                        removeClientFromQueue(thisClient);
                                               
                      } // if(clientsQueue.size()) 
                      
                        machineState = sim800Idle; // переходим к следующей команде

                  }
                  break; // smaCIPCLOSE

                  case smaCIPSTART:
                  {
                    // соединялись
                    
                        if(isKnownAnswer(thisCommandLine,knownAnswer))
                        {                                                                                                        
                          if(knownAnswer == gsmConnectOk)
                          {
                            // законнектились удачно
                            if(clientsQueue.size())
                            {
                               #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Client connected."));
                               #endif
                               
                               TransportClientQueueData dt = clientsQueue[0];
                               removeClientFromQueue(dt.client);       
                            }
                            machineState = sim800Idle; // переходим к следующей команде
                          } // gsmConnectOk
                          else if(knownAnswer == gsmConnectFail)
                          {
                            // ошибка соединения
                            if(clientsQueue.size())
                            {
                               #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Client connect ERROR!"));
                               #endif

                              flags.waitCipstartConnect = false;
                              cipstartConnectClient = NULL;                               
                               
                              TransportClientQueueData dt = clientsQueue[0];

                              CoreTransportClient* thisClient = dt.client;
                              removeClientFromQueue(thisClient);
                               
                              notifyClientConnected(*thisClient,false,CT_ERROR_CANT_CONNECT);
                            }
                            machineState = sim800Idle; // переходим к следующей команде
                          } // gsmConnectFail
                        } // isKnownAnswer                   
                    
                  }
                  break; // cmdCIPSTART


                  case smaWaitSendDone:
                  {
                    // дожидаемся конца отсыла данных от клиента в SIM800
                      
                      if(isKnownAnswer(thisCommandLine,knownAnswer))
                      {                                                
                        if(knownAnswer == gsmSendOk)
                        {
                          // send ok
                          if(clientsQueue.size())
                          {
                             #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: data was sent."));
                             #endif
                             
                             TransportClientQueueData dt = clientsQueue[0];
                             
                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);

                             // очищаем данные у клиента
                             thisClient->clear();

                             notifyDataWritten(*thisClient,CT_ERROR_NONE);
                          }                     
                        } // send ok
                        else
                        {
                          // send fail
                          if(clientsQueue.size())
                          {
                             #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: send data fail!"));
                             #endif
                             
                             TransportClientQueueData dt = clientsQueue[0];

                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);
                                                          
                             // очищаем данные у клиента
                             thisClient->clear();
                             
                             notifyDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                             
                          }                     
                        } // else send fail
  
                        machineState = sim800Idle; // переходим к следующей команде
                        
                      } // if(isKnownAnswer(thisCommandLine,knownAnswer))
                       

                  }
                  break; // smaWaitSendDone

                  case smaCIPSEND:
                  {
                    // тут отсылали запрос на запись данных с клиента
                    if(thisCommandLine == F(">"))
                    {
                       // дождались приглашения, можем писать в SIM800
                       // тут пишем напрямую
                       if(clientsQueue.size())
                       {
                          // говорим, что ждём окончания отсыла данных
                          currentCommand = smaWaitSendDone;                          
                          TransportClientQueueData dt = clientsQueue[0];                     

                          #ifdef GSM_DEBUG_MODE
                              DEBUG_LOG(F("SIM800: > received, start write from client to SIM800, DATA LENGTH: "));
                              DEBUG_LOGLN(String(dt.dataLength));
                          #endif

                          flags.ignoreNextEmptyLine = true;
                          
                          for(size_t kk=0;kk<dt.dataLength;kk++)
                          {
                            workStream->write(dt.data[kk]);
                            readFromStream();

                             #ifdef USE_WIFI_MODULE
                             // и модуль ESP тоже тут обновим
                             ESP.readFromStream();
                             #endif                             
                          }
                          
                          delete [] clientsQueue[0].data;
                          delete [] clientsQueue[0].ip;
                          clientsQueue[0].data = NULL;
                          clientsQueue[0].ip = NULL;
                          clientsQueue[0].dataLength = 0;

                          // очищаем данные у клиента сразу после отсыла
                           dt.client->clear();
                       }
                    } // if
                    else
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(knownAnswer == gsmError || knownAnswer == gsmSendFail)
                      {
                           // всё плохо, не получилось ничего записать
                          if(clientsQueue.size())
                          {
                             #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Client write ERROR!"));
                             #endif
                             
                             TransportClientQueueData dt = clientsQueue[0];
    
                             CoreTransportClient* thisClient = dt.client;
                             removeClientFromQueue(thisClient);
    
                             // очищаем данные у клиента
                             thisClient->clear();
    
                             notifyDataWritten(*thisClient,CT_ERROR_CANT_WRITE);
                            
                          }
                          
                        machineState = sim800Idle; // переходим к следующей команде
                      }                                           
              
                    } // else can't write
                    
                  }
                  break; // smaCIPSEND
                 
                  case smaCheckReady: // ждём готовности модема, ответ на команду AT+CPAS?
                  {
                      if( thisCommandLine.startsWith( F("+CPAS:") ) ) 
                      {
                          // это ответ на команду AT+CPAS, можем его разбирать
                          if(thisCommandLine == F("+CPAS: 0")) 
                          {
                            // модем готов, можем убирать команду из очереди и переходить к следующей
                            #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Modem ready."));
                            #endif
                            
                            machineState = sim800Idle; // и переходим на следующую
                        }
                        else 
                        {
                           #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Modem NOT ready, try again later..."));
                           #endif
                           
                           idleTime = 2000; // повторим через 2 секунды
                           flags.onIdleTimer = true;
                           idleTimer = millis();
                           // добавляем ещё раз эту команду
                           initCommandsQueue.push_back(smaCheckReady);
                           machineState = sim800Idle; // и пошлём ещё раз команду проверки готовности           
                        }
                      }                    
                  }
                  break; // cmdWantReady


                  case smaHangUp:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: Call dropped."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде
                    }                    
                  }
                  break; // smaHangUp

                  case smaCIPHEAD:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CIPHEAD command processed."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPHEAD

                  case smaCGATT:
                  {
                    // ждём ответа на команду проверки доступностим GPRS.
                    // ответ, теоретически, может придти позже или раньше статуса (ОК и т.п.)
                    // поэтому мы должны грамотно обработать эту ситуацию.
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      cgattKnownAnswerFound = true;
                      
                      if(knownAnswer != gsmOK)
                      {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: CGATT command processed, CAN'T start GPRS!"));
                          #endif
                        
                        machineState = sim800Idle; // переходим к следующей команде
                      }
                      else
                      {
                        // проверяем - если до OK мы получили ответ, то вываливаемся
                        if(cgattStatusReceived)
                        {
                          if(cgattStatus)
                          {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: CGATT processed, START GPRS!"));
                          #endif
                            
                            initCommandsQueue.push_back(smaCSTT);
                            initCommandsQueue.push_back(smaCIPSHUT);
                          }
                          else
                          {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: Try to attach GPRS..."));
                          #endif
                            
                            //waitGprsReconnect = true;
                            //gprsReconnectTimer = millis();
                            initCommandsQueue.push_back(smaCGATTach);
                          }                          
                          machineState = sim800Idle; // переходим к следующей команде 
                        }
                      }
                      
                    }

                    
                  }
                  break; // smaCGATT

                  case smaCGATTach:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(knownAnswer == gsmOK)
                      {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: GPRS attached, continue..."));
                          #endif
                                                  
                            initCommandsQueue.push_back(smaCSTT);
                            initCommandsQueue.push_back(smaCIPSHUT);

                      }
                      else
                      {
                          #ifdef GSM_DEBUG_MODE
                            DEBUG_LOGLN(F("SIM800: Can't attach GPRS, try later..."));
                          #endif
                            
                          waitGprsReconnect = true;
                          gprsReconnectTimer = millis();                      
                      }

                      machineState = sim800Idle;
                    }
                  }
                  break; // smaCGATTach

                  case smaCIPSHUT:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(knownAnswer == gsmShutOk || knownAnswer == gsmError)
                      {
                        #ifdef GSM_DEBUG_MODE
                          DEBUG_LOGLN(F("SIM800: CIPSHUT command processed."));
                        #endif
                        machineState = sim800Idle; // переходим к следующей команде
                      }
                    }
                  }
                  break; // smaCIPSHUT                  

                  case smaCIICR:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      inWaitingGPRS_CIICR = false;
                      
                      #ifdef GSM_DEBUG_MODE
                        DEBUG_LOGLN(F("SIM800: CIICR command processed."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде

                      if(knownAnswer == gsmOK)
                      {
                        // тут можем добавлять новые команды для GPRS
                        idleTime = 1000; // обработаем ответ через 1 секунду
                        flags.onIdleTimer = true;
                        idleTimer = millis();
                                                   
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: start checking GPRS connection..."));
                        #endif                        
                        initCommandsQueue.push_back(smaCIFSR);
                        gprsCheckingAttempts = 0;
                      }
                      else
                      {
                        // ошибка соединения, попробуем позже
                        waitGprsReconnect = true;
                        gprsReconnectTimer = millis();
                      }
                    }
                  }
                  break; // smaCIICR

                  case smaCIFSR:
                  {
                    if(thisCommandLine.length() && thisCommandLine.indexOf(".") != -1)
                    {
                        // известный ответ получен, мы можем вываливаться
                          #ifdef GSM_DEBUG_MODE
                              DEBUG_LOG(F("SIM800: GPRS IP address found - "));                      
                              DEBUG_LOGLN(thisCommandLine);
                          #endif
                        
                        flags.gprsAvailable = true;
                       // cifsrIPfound = true;

                      // всё хорошо, получили IP-адрес, можем вываливаться
                      machineState = sim800Idle; // переходим к следующей команде
                        
                    }
                    else
                      if(isKnownAnswer(thisCommandLine,knownAnswer))
                      {
                          // пытаемся переполучить
                          if(++gprsCheckingAttempts <=5)
                          {
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: try to get GPRS IP address a little bit later..."));
                            #endif
                            
                            idleTime = 5000; // обработаем ответ через 5 секунд
                            flags.onIdleTimer = true;
                            idleTimer = millis();
                            initCommandsQueue.push_back(smaCIFSR);
                            machineState = sim800Idle; // переходим к следующей команде  
                          }
                          else
                          {
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Unable to get GPRS IP address!"));
                            #endif
                            // всё, исчерпали лимит на попытки получить IP-адрес
                            machineState = sim800Idle; // переходим к следующей команде
                            flags.gprsAvailable = false;
                          }                                   
                   
                      } // else if isKnownAnswer
                       
                  }
                  break; // smaCIFSR

                      //////////////////////////////
                      // синхронизация времени
                      //////////////////////////////
                      case smaStartTimeSync:
                      {
                        if(isKnownAnswer(thisCommandLine,knownAnswer))
                        {
                          if(knownAnswer == gsmOK)
                          {
                            initCommandsQueue.push_back(smaSaveSyncSettings);
                          }
                          else
                          {
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Unable to sync time!"));
                            #endif
                          }

                          machineState = sim800Idle; // переходим к следующей команде
                        }
                      }
                      break; // smaStartTimeSync

                      case smaSaveSyncSettings:
                      {
                        if(isKnownAnswer(thisCommandLine,knownAnswer))
                        {
                          if(knownAnswer == gsmOK)
                          {
                            initCommandsQueue.push_back(smaGetSyncTime);
                          }
                          else
                          {
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: Unable to save sync settings!"));
                            #endif
                          }

                          machineState = sim800Idle; // переходим к следующей команде
                        }
                      }
                      break; // smaTimeSyncConnectToServer                    

                      case smaGetSyncTime:
                      {
                        if(thisCommandLine.startsWith(F("+CCLK: \"")))
                        {
                          // тут приходит время. Его можно использовать, только если приходил PSUTTZ, в противном случае - там будет неправильное время
                          if(PSUTTZreceived)
                          {
                            #ifdef GSM_DEBUG_MODE
                                DEBUG_LOGLN(F("SIM800: TIME RECEIVED!"));
                            #endif

                             // тут разбираем время, строка имеет вид
                             // +CCLK: "19/01/21,22:22:30+12"
                             String timeline = thisCommandLine;
                             timeline.remove(0,8);

                             if(timeline[timeline.length()-1] == '"')
                             {
                              timeline.remove(timeline.length()-1,1);
                             }

                             if(timeline.length() >= 18)
                             {
                               // разбиваем на части
                               timeline[2] = 0;
                               timeline[5] = 0;
                               timeline[8] = 0;
                               timeline[11] = 0;
                               timeline[14] = 0;
                               timeline[17] = 0;
  
  
                               RTCTime tm;
                               // выцепляем компоненты
                               tm.year = atoi(timeline.c_str()) + 2000;
                               tm.month = atoi(&timeline[3]);
                               tm.dayOfMonth = atoi(&timeline[6]);
                               tm.hour = atoi(&timeline[9]);
                               tm.minute = atoi(&timeline[12]);
                               tm.second = atoi(&timeline[15]);
  
                               uint32_t ut = tm.unixtime();
                               tm = tm.maketime(ut);
  
                               RealtimeClock rtc = MainController->GetClock();
  
                               rtc.setTime(tm);
                               
                             } // if(timeline.length() >= 18)

                             syncTimeTimerEnabled = true;
                             syncTimeTimer = millis();
                          } // if(PSUTTZreceived)
                        }

                        
                        if(isKnownAnswer(thisCommandLine,knownAnswer))
                        {                        
                          machineState = sim800Idle; // переходим к следующей команде
                        }
                      }
                      break; // smaGetSyncTime

                  case smaCSTT:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: CSTT command processed."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде

                      if(knownAnswer == gsmOK)
                      {
                        // тут можем добавлять новые команды для GPRS
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: start GPRS connection..."));
                        #endif
                        initCommandsQueue.push_back(smaCIICR);
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Can't start GPRS connection!"));
                        #endif
                      }
                    }                    
                  }
                  break; // smaCSTT

                  case smaCIPMODE:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: CIPMODE command processed."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPMODE
                  
                  case smaCIPMUX:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: CIPMUX command processed."));
                      #endif
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaCIPMUX


                  case smaEchoOff:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Echo OFF command processed."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Echo OFF command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaEchoOff

                  case smaDisableCellBroadcastMessages:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Broadcast SMS disabled."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Broadcast SMS command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaDisableCellBroadcastMessages

                  case smaAON:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: AON is ON."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: AON command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }
                  }
                  break; // smaAON

                  case smaPDUEncoding:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: PDU format is set."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: PDU format command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                    
                  }
                  break; // smaPDUEncoding

                  case smaUCS2Encoding:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: UCS2 encoding is set."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: UCS2 encoding command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                                        
                  }
                  break; // smaUCS2Encoding

                  case smaSMSSettings:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: SMS settings is set."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: SMS settings command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                                        
                  }
                  break; // smaSMSSettings

                  case smaGPRSMultislotClass:
                  {
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      if(gsmOK == knownAnswer)
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: GPRS multislot class is set."));
                        #endif
                      }
                      else
                      {
                        #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: GPRS multislot class command FAIL!"));
                        #endif
                      }
                      machineState = sim800Idle; // переходим к следующей команде
                    }                                        
                  }
                  break; // smaGPRSMultislotClass                  

                  case smaWaitReg:
                  {
                     if(thisCommandLine.indexOf(F("+CREG: 0,1")) != -1)
                        {
                          // зарегистрированы в GSM-сети
                             flags.isModuleRegistered = true;
                             #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: Modem registered in GSM!"));
                             #endif
                             machineState = sim800Idle;
                        } // if
                        else
                        {
                          // ещё не зарегистрированы
                            flags.isModuleRegistered = false;
                            idleTime = 5000; // повторим через 5 секунд
                            flags.onIdleTimer = true;
                            idleTimer = millis();
                            // добавляем ещё раз эту команду
                            initCommandsQueue.push_back(smaWaitReg);
                            machineState = sim800Idle;
                        } // else                    
                  }
                  break; // smaWaitReg
                  
                  case smaCheckModemHang:
                  {                                 
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: modem answered and available."));
                      #endif

                      if(knownAnswer == gsmOK)
                      {
                        if(csqReceived)                     
                          machineState = sim800Idle; // переходим к следующей команде
                      }
                      else
                      {
                         machineState = sim800Idle; // переходим к следующей команде
                      }
                      
                    } // if(isKnownAnswer

                  }
                  break; // smaCheckModemHang

                  case smaPING:
                  {                    
                    if(isKnownAnswer(thisCommandLine,knownAnswer))
                    {
                      #ifdef GSM_DEBUG_MODE
                              DEBUG_LOGLN(F("SIM800: PING done."));
                      #endif
                      
                      machineState = sim800Idle; // переходим к следующей команде
                      
                    } // if(isKnownAnswer

                  }
                  break; // smaPING
                                    
                } // switch

                
              } // if(hasAnswerLine)
              
         
        }
        break; // sim800WaitAnswer

        case sim800Reboot:
        {          
          // ждём перезагрузки модема
          uint32_t powerOffTime = 0;
          if(gbnd.RebootPinLinkType != linkUnbinded && gbnd.RebootPin != UNBINDED_PIN)
          {
            powerOffTime = gbnd.RebootTime;
          }
          
          if(millis() - timer > powerOffTime)
          {            
              #ifdef GSM_DEBUG_MODE
                DEBUG_LOGLN(F("SIM800: turn power ON!"));
              #endif

              power(true); // включаем питание модема

            // работаем с пином POWERKEY
            if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)
            {
              if(gbnd.PowerkeyLinkType == linkDirect)
                  {
                      #ifndef DISABLE_GSM_CONFIGURE
                      if(EEPROMSettingsModule::SafePin(gbnd.PowerkeyPin))
                      #else
                      if(gbnd.PowerkeyPin > 1) // prevent Serial locking
                      #endif
                      {
                        WORK_STATUS.PinWrite(gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);
                      }
                  }
                  else
                  if(gbnd.PowerkeyLinkType == linkMCP23S17)
                  {
                    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                       WORK_STATUS.MCP_SPI_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);
                    #endif 
                  }
                  else
                  if(gbnd.PowerkeyLinkType == linkMCP23017)
                  {
                    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                      WORK_STATUS.MCP_I2C_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);
                    #endif
                  }
            } // if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)

            machineState = sim800WaitInit;
            timer = millis();
            
          } // if
        }
        break; // smaReboot

        case sim800WaitInit:
        {

          uint32_t waitTime = 0;
          if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)
          {
            waitTime = gbnd.WaitPowerkeyAfterPowerOn;
          }
            
          if(millis() - timer > waitTime)
          { 
            #ifdef GSM_DEBUG_MODE
              DEBUG_LOGLN(F("SIM800: Power ON completed!"));
            #endif

            if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)
            {
              #ifdef GSM_DEBUG_MODE
                  DEBUG_LOGLN(F("SIM800: use POWERKEY!"));
               #endif
              
                  if(gbnd.PowerkeyLinkType == linkDirect)
                  {
                      #ifndef DISABLE_GSM_CONFIGURE
                      if(EEPROMSettingsModule::SafePin(gbnd.PowerkeyPin))
                      #else
                      if(gbnd.PowerkeyPin > 1) // prevent Serial locking
                      #endif
                      {
                        WORK_STATUS.PinWrite(gbnd.PowerkeyPin,gbnd.PowerkeyOnLevel);
                        delay(gbnd.PowerkeyPulseDuration);        
                        WORK_STATUS.PinWrite(gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);

                      }
                  }
                  else
                  if(gbnd.PowerkeyLinkType == linkMCP23S17)
                  {
                    #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
                       WORK_STATUS.MCP_SPI_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,gbnd.PowerkeyOnLevel);
                       delay(gbnd.PowerkeyPulseDuration);
                       WORK_STATUS.MCP_SPI_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);
                    #endif 
                  }
                  else
                  if(gbnd.PowerkeyLinkType == linkMCP23017)
                  {
                    #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
                      WORK_STATUS.MCP_I2C_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,gbnd.PowerkeyOnLevel);
                      delay(gbnd.PowerkeyPulseDuration);
                      WORK_STATUS.MCP_I2C_PinWrite(gbnd.RebootPinMCPAddress,gbnd.PowerkeyPin,!gbnd.PowerkeyOnLevel);
                    #endif
                  }
            } // if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)  

            // теперь ждём загрузки модема
            idleTime = gbnd.WaitAfterRebootTime; // подождём чуть-чуть...
            flags.onIdleTimer = true;
           
            machineState = sim800WaitBootBegin;
            idleTimer = millis();
                

          } // 
        }
        break; // sim800WaitInit

        case sim800WaitBootBegin:
        {
          #ifdef GSM_DEBUG_MODE
                DEBUG_LOGLN(F("SIM800: inited after reboot!"));
          #endif
          
          sendCommand(F("AT"));
          machineState = sim800WaitBoot;          
        }
        break; // sim800WaitBootBegin

        case sim800WaitBoot:
        {
          if(hasAnswerLine)
          {
            if(gbnd.RebootPinLinkType != linkUnbinded && gbnd.RebootPin != UNBINDED_PIN)
            {
              // используем управление питанием, ждём загрузки модема
              if(thisCommandLine == F("Call Ready") || thisCommandLine == F("SMS Ready"))
              {
                #ifdef GSM_DEBUG_MODE
                  DEBUG_LOGLN(F("SIM800: BOOT FOUND, INIT!"));
                #endif
                restart();
              }
              
            } // if(gbnd.RebootPinLinkType != linkUnbinded && gbnd.RebootPin != UNBINDED_PIN)
            else
            {
              // управление питанием не используем, здесь не надо ждать загрузки модема - достаточно дождаться ответа на команду
              if(isKnownAnswer(thisCommandLine,knownAnswer))
              {
                #ifdef GSM_DEBUG_MODE
                  DEBUG_LOGLN(F("SIM800: ANSWERED, INIT!"));
                #endif
                restart();
              }
            } // else
            
          } // if(hasAnswerLine)
             
        }
        break; // sim800WaitBoot
      
      } // switch

    } // if(!flags.onIdleTimer)


}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::powerkeyInit()
{
  GSMBinding gbnd = HardwareBinding->GetGSMBinding();
  
  if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)
  {
    if(gbnd.PowerkeyLinkType == linkDirect)
    {
        #ifndef DISABLE_GSM_CONFIGURE
        if(EEPROMSettingsModule::SafePin(gbnd.PowerkeyPin))
        #else
        if(gbnd.PowerkeyPin > 1) // prevent Serial locking
        #endif
        {
          WORK_STATUS.PinMode(gbnd.PowerkeyPin,OUTPUT);
        }
    }
    else
    if(gbnd.PowerkeyLinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(gbnd.PowerkeyMCPAddress,gbnd.PowerkeyPin,OUTPUT);
      #endif
    }
    else
    if(gbnd.PowerkeyLinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(gbnd.PowerkeyMCPAddress,gbnd.PowerkeyPin,OUTPUT);
      #endif
    }    
  } // if(gbnd.PowerkeyLinkType != linkUnbinded && gbnd.PowerkeyPin != UNBINDED_PIN)
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::begin()
{

  #ifdef GSM_DEBUG_MODE
   DEBUG_LOGLN(F("SIM800: begin."));
  #endif


  GSMBinding gbnd = HardwareBinding->GetGSMBinding();

  #if TARGET_BOARD == STM32_BOARD
    SerialUART* hs = NULL;
  #else
    HardwareSerial* hs = NULL;
  #endif  

 if(gbnd.SerialNumber == 1) // Serial1
  {
    hs = &Serial1;
  }
  else
  if(gbnd.SerialNumber == 2) // Serial2
  {
    hs = &Serial2;
  }
  else
  if(gbnd.SerialNumber == 3) // Serial3
  {
    hs = &Serial3;
  }
  #if TARGET_BOARD == STM32_BOARD
  else
  if(gbnd.SerialNumber == 4) // Serial4
  {
    hs = &Serial4;
  }
  else
  if(gbnd.SerialNumber == 5) // Serial5
  {
    hs = &Serial5;
  }
  else
  if(gbnd.SerialNumber == 6) // Serial6
  {
    hs = &Serial6;
  }
  #endif

  
  workStream = hs;

  if(hs)
  {
    hs->begin(SERIAL_BAUD_RATE);
  }  

  restart();

  power(false); // выключаем питание модему
  powerkeyInit();  
  
  machineState = sim800Reboot;

  #ifdef GSM_DEBUG_MODE
    DEBUG_LOGLN(F("SIM800: started."));
  #endif

}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::sendCUSD(const String& cusd)
{
  if(!ready())
  {
    return;
  }

  #ifdef GSM_DEBUG_MODE
    DEBUG_LOG(F("SIM800: CUSD REQUESTED: "));
    DEBUG_LOGLN(cusd);
  #endif


  unsigned int bp = 0;
  String out;
  
  PDU.UTF8ToUCS2(cusd, bp, &out);

  String* completeCommand = new String(F("AT+CUSD=1,\""));
  *completeCommand += out.c_str();
  *completeCommand += F("\"");

   cusdList.push_back(completeCommand);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::sendSMS(const String& phoneNumber, const String& message, bool isFlash)
{
  if(!ready())
  {
    return false;
  }
    
    SIM800OutgoingSMS queuedSMS;
    
    queuedSMS.isFlash = isFlash;   
    queuedSMS.phone = new String(phoneNumber.c_str());    
    queuedSMS.message = new String(message.c_str());    

    outgoingSMSList.push_back(queuedSMS);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::restart()
{
  // очищаем входной буфер
  receiveBuffer.clear();

  delete smsToSend;
  smsToSend = new String();

  // очищаем очередь клиентов, заодно им рассылаем события
  clearClientsQueue(true);  
  // и говорим, что все слоты свободны
  initPool();
  
  // т.к. мы ничего не инициализировали - говорим, что мы не готовы предоставлять клиентов
  flags.ready = false;
  flags.isAnyAnswerReceived = false;
  flags.waitForDataWelcome = false;
  flags.onIdleTimer = false;
  flags.isModuleRegistered = false;
  flags.gprsAvailable = false;
  flags.ignoreNextEmptyLine = false;
  flags.isSIMCardPresent = false;

  PSUTTZreceived = false;
  syncTimeTimerEnabled = false;
  
  signalQuality = 0;
  badPingAttempts = 0;
  
  timer = millis();

  flags.waitCipstartConnect = false; // не ждёт соединения внешнего клиента
  cipstartConnectClient = NULL;  

  currentCommand = smaNone;
  machineState = sim800Idle;

  // инициализируем очередь командами по умолчанию
 createInitCommands(true);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::createInitCommands(bool addResetCommand)
{  
  UNUSED(addResetCommand);
  
  // очищаем очередь команд
  clearInitCommands();

  // если указаны параметры APN - при старте поднимаем GPRS
  String apn = GetAPN();
  if(apn.length())
  {
    cgattStatusReceived = false;
    cgattKnownAnswerFound = false;
    initCommandsQueue.push_back(smaCGATT);
  }



    GlobalSettings* settings = MainController->GetSettings();
    TimeSyncSettings* ts = settings->getTimeSyncSettings();

    if(ts->gsmActive)
    {
      //////////////////////////////
      // синхронизация времени
      //////////////////////////////
      initCommandsQueue.push_back(smaStartTimeSync);
    } // gsmActive

  
  // получим уровень сигнала при старте
  initCommandsQueue.push_back(smaCheckModemHang);
  

  initCommandsQueue.push_back(smaCIPMUX);
  initCommandsQueue.push_back(smaCIPMODE);
  initCommandsQueue.push_back(smaWaitReg); // ждём регистрации
  
  initCommandsQueue.push_back(smaCIPHEAD);
  
  initCommandsQueue.push_back(smaGPRSMultislotClass); // настройки вывода SMS
  initCommandsQueue.push_back(smaSMSSettings); // настройки вывода SMS
  initCommandsQueue.push_back(smaUCS2Encoding); // кодировка сообщений
  initCommandsQueue.push_back(smaPDUEncoding); // формат сообщений
  initCommandsQueue.push_back(smaAON); // включение АОН
  initCommandsQueue.push_back(smaDisableCellBroadcastMessages); // выключение броадкастовых SMS
  initCommandsQueue.push_back(smaCheckReady); // проверка готовности    
  initCommandsQueue.push_back(smaCheckSimCardPresent); // проверка наличия SIM-карты
  initCommandsQueue.push_back(smaEnableSimDetection); // включение проверки наличия SIM-карты
  initCommandsQueue.push_back(smaEchoOff); // выключение эха
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::clearInitCommands()
{
  initCommandsQueue.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::clearClientsQueue(bool raiseEvents)
{
 // тут попросили освободить очередь клиентов.
  // для этого нам надо выставить каждому клиенту флаг того, что он свободен,
  // плюс - сообщить, что текущее действие над ним не удалось.  

    for(size_t i=0;i<clientsQueue.size();i++)
    {
        TransportClientQueueData dt = clientsQueue[i];
        delete [] dt.data;
        delete [] dt.ip;

        // если здесь в очереди есть хоть один клиент с неназначенным ID (ждёт подсоединения) - то в события он не придёт,
        // т.к. там сравнивается по назначенному ID. Поэтому мы назначаем ID клиенту в первый свободный слот.
        if(dt.client->socket == NO_CLIENT_ID)
        {
          CoreTransportClient* cl = getClient(NO_CLIENT_ID);
          if(cl)
            dt.client->bind(cl->socket);
        }
        
        if(raiseEvents)
        {
          switch(dt.action)
          {
            case actionDisconnect:
                // при дисконнекте всегда считаем, что ошибок нет
                notifyClientConnected(*(dt.client),false,CT_ERROR_NONE);
            break;
  
            case actionConnect:
                // если было запрошено соединение клиента с адресом - говорим, что соединиться не можем
                notifyClientConnected(*(dt.client),false,CT_ERROR_CANT_CONNECT);
            break;
  
            case actionWrite:
              // если попросили записать данные - надо сообщить подписчикам, что не можем записать данные
              notifyDataWritten(*(dt.client),CT_ERROR_CANT_WRITE);
              notifyClientConnected(*(dt.client),false,CT_ERROR_NONE);
            break;
          } // switch
          

        } // if(raiseEvents)
        
    } // for

  clientsQueue.clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::isClientInQueue(CoreTransportClient* client, TransportClientAction action)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
    {
      return true;
    }
  }

  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::addClientToQueue(CoreTransportClient* client, TransportClientAction action, const char* ip, uint16_t port)
{
  while(isClientInQueue(client, action))
  {
    removeClientFromQueue(client,action);
  }

    TransportClientQueueData dt;
    dt.client = client;
    dt.action = action;
    
    dt.ip = NULL;
    if(ip)
    {
      dt.ip = new char[strlen(ip)+1];
      strcpy(dt.ip,ip);
    }
    dt.port = port;

    clientsQueue.push_back(dt);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::removeClientFromQueue(CoreTransportClient* client, TransportClientAction action)
{
  
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client && clientsQueue[i].action == action)
    {
      delete [] clientsQueue[i].ip;
      delete [] clientsQueue[i].data;
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::removeClientFromQueue(CoreTransportClient* client)
{
  for(size_t i=0;i<clientsQueue.size();i++)
  {
    if(clientsQueue[i].client == client)
    {
      delete [] clientsQueue[i].ip;
      delete [] clientsQueue[i].data;
      
        for(size_t j=i+1;j<clientsQueue.size();j++)
        {
          clientsQueue[j-1] = clientsQueue[j];
        }
        
        clientsQueue.pop();
        break;
    } // if
    
  } // for
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginWrite(CoreTransportClient& client)
{

  // добавляем клиента в очередь на запись
  addClientToQueue(&client, actionWrite);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с записью в поток с этого клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginConnect(CoreTransportClient& client, const char* ip, uint16_t port)
{
  if(client.connected())
  {
    
    #ifdef GSM_DEBUG_MODE
      DEBUG_LOGLN(F("SIM800: client already connected!"));
    #endif
    return;
    
  }
  
  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionConnect, ip, port);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с соединением клиента
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CoreSIM800Transport::beginDisconnect(CoreTransportClient& client)
{
  if(!client.connected())
  {
    return;
  }

  // добавляем клиента в очередь на соединение
  addClientToQueue(&client, actionDisconnect);

  // клиент добавлен, теперь при обновлении транспорта мы начнём работать с отсоединением клиента
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreSIM800Transport::ready()
{
  return workStream != NULL && flags.ready && flags.isAnyAnswerReceived && flags.isModuleRegistered && flags.isSIMCardPresent; // если мы полностью инициализировали SIM800 - значит, можем работать
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SMS_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------

