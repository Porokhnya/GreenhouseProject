#pragma once

#include "AbstractModule.h"
#include "TinyVector.h"
#include "Settings.h"
#include "CoreTransport.h"

#if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
#include "IoT.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------
#include "HTTPInterfaces.h" // подключаем интерфейсы для работы с HTTP-запросами
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
class WiFiModule : public AbstractModule // модуль поддержки WI-FI
#if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
, public IoTGate
#endif
#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
, public HTTPQueryProvider
#endif
, public IClientEventsSubscriber
, public Stream
{
  private:
  
    String* streamBuffer;

    #ifdef USE_WIFI_MODULE_AS_MQTT_CLIENT
      CoreMQTT mqtt;
    #endif

    TransportReceiveBuffer externalClientData;
    void ProcessUnknownClientQuery(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone);

#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER

    HTTPRequestHandler* httpHandler; // интерфейс перехватчика работы с HTTP-запросами
    CoreTransportClient httpClient;
    bool canCallHTTPEvent;
    bool httpDataWritten;
    String* httpData;
    void EnsureHTTPProcessed(uint16_t statusCode); // убеждаемся, что мы сообщили вызывающей стороне результат запроса по HTTP
    void sendDataToGardenbossRu();
    void processGardenbossData(uint8_t* data, size_t dataSize, bool isLastData);
    
#endif
    
    #if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
    
      IOT_OnWriteToStream iotWriter;
      IOT_OnSendDataDone iotDone;
      IoTService iotService;

      uint16_t thingSpeakDataLength;
      bool thingSpeakDataWritten;
      CoreTransportClient thingSpeakClient;
      void sendDataToThingSpeak();
      void EnsureIoTProcessed(bool success=false);
      
    #endif

  
  public:
    WiFiModule() : AbstractModule("WIFI") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

  // IClientEventsSubscriber
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode); // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int16_t errorCode); // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone); // событие "Для клиента поступили данные", флаг - все ли данные приняты
    
   // Stream
  virtual void flush(){}
  virtual int peek() {return 0;}
  virtual int read() {return 0;}
  virtual int available() {return 0;}
  virtual size_t write(uint8_t ch) { *streamBuffer += (char) ch; return 1;}
  

#if defined(USE_IOT_MODULE) && defined(USE_WIFI_MODULE_AS_IOT_GATE)
    virtual void SendData(IoTService service,uint16_t dataLength, IOT_OnWriteToStream writer, IOT_OnSendDataDone onDone);
#endif 

#ifdef USE_WIFI_MODULE_AS_HTTP_PROVIDER
  virtual bool CanMakeQuery(); // тестирует, может ли модуль сейчас сделать запрос
  virtual void MakeQuery(HTTPRequestHandler* handler); // начинаем запрос по HTTP
#endif

};
#endif // USE_WIFI_MODULE
