#pragma once
//--------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "Settings.h"
#include "TinyVector.h"
#include "CoreTransport.h"
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SMS_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
#include "IoT.h"
#endif
//--------------------------------------------------------------------------------------------------------------------------------
#include "HTTPInterfaces.h" // подключаем интерфейсы для работы с HTTP-запросами
//--------------------------------------------------------------------------------------------------------------------------------
class SMSModule : public AbstractModule // модуль поддержки управления по SMS
#if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
, public IoTGate
#endif
#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
, public HTTPQueryProvider
#endif
, public IClientEventsSubscriber
{
  private:

#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
    HTTPRequestHandler* httpHandler; // интерфейс перехватчика работы с HTTP-запросами
    CoreTransportClient httpClient;
    bool canCallHTTPEvent;
    bool httpDataWritten;
    String* httpData;
    void EnsureHTTPProcessed(uint16_t statusCode); // убеждаемся, что мы сообщили вызывающей стороне результат запроса по HTTP
    void sendDataToGardenbossRu();
    void processGardenbossData(uint8_t* data, size_t dataSize, bool isLastData);
#endif

    #if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
      IOT_OnWriteToStream iotWriter;
      IOT_OnSendDataDone iotDone;
      IoTService iotService;

      uint16_t thingSpeakDataLength;
      bool thingSpeakDataWritten;
      CoreTransportClient thingSpeakClient;
      void sendDataToThingSpeak();
      void EnsureIoTProcessed(bool success=false);
    #endif



    const char* GetKnownModuleName(int moduleIndex);
    String RequestDataFromKnownModule(const char* knownModule, int moduleIndex, int sensorIndex, const String& label);
    void SendStatToCaller(const String& phoneNum);
    void RequestBalance();

    bool requestBalanceAsked;


    #if defined(USE_ALARM_DISPATCHER) && defined(CLEAR_ALARM_STATUS)
      unsigned long processedAlarmsClearTimer;
    #endif
    
  public:
    SMSModule() : AbstractModule("SMS") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);


#if defined(USE_IOT_MODULE) && defined(USE_GSM_MODULE_AS_IOT_GATE)
    virtual void SendData(IoTService service,uint16_t dataLength, IOT_OnWriteToStream writer, IOT_OnSendDataDone onDone);
#endif 

#ifdef USE_GSM_MODULE_AS_HTTP_PROVIDER
  virtual bool CanMakeQuery(); // тестирует, может ли модуль сейчас сделать запрос
  virtual void MakeQuery(HTTPRequestHandler* handler); // начинаем запрос по HTTP
#endif

  // IClientEventsSubscriber
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode); // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int16_t errorCode); // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone); // событие "Для клиента поступили данные", флаг - все ли данные приняты

  void IncomingCall(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp);
  void IncomingSMS(const String& phoneNumber,const String& message, bool isKnownNumber);
  void IncomingCUSD(const String& cusd);

};

#endif // #ifdef USE_SMS_MODULE
