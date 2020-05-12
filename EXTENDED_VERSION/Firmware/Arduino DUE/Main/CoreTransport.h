#pragma once
//--------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "TinyVector.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------
#define MQTT_FILENAME_PATTERN F("MQTT/MQTT.")
#define DEFAULT_MQTT_CLIENT F("greenhouse")
#define REPORT_TOPIC_NAME F("/REPORT/")
// максимальная длина одного пакета к вычитке прежде, чем подписчику придёт уведомление о пакете данных
#define TRANSPORT_MAX_PACKET_LENGTH 128
//--------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient;
#ifdef USE_WIFI_MODULE
class CoreESPTransport;
#endif
#ifdef USE_SMS_MODULE
class CoreSIM800Transport;
#endif
//--------------------------------------------------------------------------------------------------------------------------------
// Коды ошибок транспорта
//--------------------------------------------------------------------------------------------------------------------------------
#define CT_ERROR_NONE             0 // нет ошибки
#define CT_ERROR_CANT_CONNECT     1 // не удалось установить соединение
#define CT_ERROR_CANT_WRITE       2 // ошибка записи данных из клиента в поток
//--------------------------------------------------------------------------------------------------------------------------------
// типы событий
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  etConnect,      // соединено
  etDataWritten,    // данные записаны
  etDataAvailable   // доступны входящие данные
  
} ClientEventType;
//---------------------------------------------------------------------------------------------------------------------------------
struct IClientEventsSubscriber
{
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode) = 0; // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int16_t errorCode) = 0; // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone) = 0; // событие "Для клиента поступили данные", флаг - все ли данные приняты
};
//--------------------------------------------------------------------------------------------------------------------------------
class RecursionCounter
{
  private:
    uint16_t* thisCntr;
  public:
    RecursionCounter(uint16_t* cntr)
    {
      thisCntr = cntr;
      (*thisCntr)++;
    }
   ~RecursionCounter()
   {
     (*thisCntr)--;
   }
  
};
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<IClientEventsSubscriber*> ClientSubscribers;
#define NO_CLIENT_ID 0xFF
//--------------------------------------------------------------------------------------------------------------------------------
class CoreTransport
{
  public:
  
    CoreTransport(uint8_t clientsPoolSize);
    virtual ~CoreTransport();
    
    // обновляем состояние транспорта
    virtual void update() = 0; 

    // начинаем работу
    virtual void begin() = 0; 

    // проверяет, готов ли транспорт к работе (например, проведена ли первичная инициализация)
    virtual bool ready() = 0; 

   // подписка на события клиентов
   void subscribe(IClientEventsSubscriber* subscriber);
   
   // отписка от событий клиентов
   void unsubscribe(IClientEventsSubscriber* subscriber);


private:

    ClientSubscribers subscribers;
    Vector<CoreTransportClient*> pool;
    Vector<bool> status;

    Vector<CoreTransportClient*> externalClients;
    bool isExternalClient(CoreTransportClient& client);

protected:

    void initPool();

  friend class CoreTransportClient;

  bool connected(uint8_t socket);

  void doWrite(CoreTransportClient& client); // начинаем писать в транспорт с клиента
  void doConnect(CoreTransportClient& client, const char* ip, uint16_t port); // начинаем коннектиться к адресу
  void doDisconnect(CoreTransportClient& client); // начинаем отсоединение от адреса
  
  CoreTransportClient* getClient(uint8_t socket);

  // вызов событий для клиента
  void notifyClientConnected(CoreTransportClient& client, bool connected, int16_t errorCode);  
  void notifyDataWritten(CoreTransportClient& client, int16_t errorCode);
  void notifyDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone);
      
    
  virtual void beginWrite(CoreTransportClient& client) = 0; // начинаем писать в транспорт с клиента
  virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port) = 0; // начинаем коннектиться к адресу
  virtual void beginDisconnect(CoreTransportClient& client) = 0; // начинаем отсоединение от адреса
  
};
//--------------------------------------------------------------------------------------------------------------------------------
class CoreTransportClient
{
  public:

   CoreTransportClient();
   ~CoreTransportClient();
  
  bool connected();
  void accept(CoreTransport* _parent);
     
  operator bool()
  {
    return  (socket != NO_CLIENT_ID); 
  }

  bool operator==(const CoreTransportClient& rhs)
  {
    return (rhs.socket == socket);
  }

  bool operator!=(const CoreTransportClient& rhs)
  {
    return !(operator==(rhs));
  }

  void connect(const char* ip, uint16_t port);
  void disconnect();
  
  bool write(uint8_t* buff, size_t sz);  


 protected:

    friend class CoreTransport;
    
    #ifdef USE_WIFI_MODULE
    friend class CoreESPTransport;
    #endif

    #ifdef USE_SMS_MODULE
    friend class CoreSIM800Transport;
    #endif    
    
    CoreTransport* parent;

    void releaseBuffer()
    {
      dataBuffer = NULL;
      dataBufferSize = 0;
    }

    // установка ID клиента транспортом
    void bind(uint8_t _socket)
    {
      socket = _socket;
    }
    void release()
    {
      socket = NO_CLIENT_ID;
    }

    void clear();
    uint8_t* getBuffer(size_t& sz)
    {
      sz =  dataBufferSize;
      return dataBuffer;
    }

 private:

    CoreTransportClient(const CoreTransportClient& rhs);
    CoreTransportClient& operator=(const CoreTransportClient& rhs);

    uint8_t* dataBuffer;
    size_t dataBufferSize;
    uint8_t socket;
    
};
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  
  actionDisconnect, // запрошено отсоединение клиента
  actionConnect, // запрошено подсоединение клиента
  actionWrite, // запрошена запись из клиента в транспорт
  
} TransportClientAction;
//--------------------------------------------------------------------------------------------------------------------------------
struct TransportClientQueueData // данные по клиенту в очереди
{
  TransportClientAction action; // действие, которое надо выполнить с клиентом
  CoreTransportClient* client; // ссылка на клиента
  char* ip; // IP для подсоединения
  uint16_t port; // порт для подсоединения
  size_t dataLength;
  uint8_t* data;

  TransportClientQueueData()
  {
   client = NULL;
   ip = NULL;
   data = NULL;
  }
   
};
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<TransportClientQueueData> TransportClientsQueue; // очередь клиентов на совершение какой-либо исходящей операции (коннект, дисконнект, запись)
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<uint8_t> TransportReceiveBuffer;
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#define ESP_MAX_CLIENTS 4 // наш пул клиентов
//--------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool ready                : 1; // флаг готовности
  bool connectedToRouter    : 1; // флаг того, что заокннекчены к роутеру
  bool isAnyAnswerReceived  : 1; // флаг, что мы получили хотя бы один ответ от модема
  bool waitForDataWelcome   : 1; // флаг, что ждём приглашения на отсыл данных
  bool wantReconnect        : 1; // флаг, что мы должны переподсоединиться к роутеру
  bool onIdleTimer          : 1; // флаг, что мы в режиме простоя
  bool waitCipstartConnect  : 1;
  bool cipstartConnectKnownAnswerFound : 1;

  bool specialCommandDone : 1;
  bool pad : 7;
  
} CoreESPTransportFlags;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  cmdNone, // ничего не делаем
  cmdWantReady, // надо получить ready от модуля
  cmdEchoOff, // выключаем эхо
  cmdCWMODE, // переводим в смешанный режим
  cmdCWSAP, // создаём точку доступа
  cmdCWJAP, // коннектимся к роутеру
  cmdCWQAP, // отсоединяемся от роутера
  cmdCIPMODE, // устанавливаем режим работы
  cmdCIPMUX, // разрешаем множественные подключения
  cmdCIPSERVER, // запускаем сервер
  cmdCheckModemHang, // проверяем на зависание модема 
  cmdCIPCLOSE, // отсоединямся
  cmdCIPSTART, // соединяемся
  cmdCIPSEND, // начинаем слать данные
  cmdWaitSendDone, // ждём окончания отсылки данных
  cmdPING, // команда пингования
  cmdCIFSR, // команда получения MAC-адресов и IP
  cmdCheckCSQ, // уровень сигнала
  cmdNTPTIME, // получить время с NTP-сервера
  cmdBROADCAST, // отправить пакет броадкаста
  cmdInternalPing, // команда внутреннего пингования
  
} ESPCommands;
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<ESPCommands> ESPCommandsList;
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  espIdle,        // состояние "ничего не делаем"
  espWaitAnswer,  // состояние "ждём ответа на команду, посланную ESP"
  espReboot,      // состояние "ESP в процессе перезагрузки"
  espWaitInit,    // ждём инициализации после подачи питания
  
} ESPMachineState;
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  kaNone,
  kaOK,             // OK
  kaError,          // ERROR
  kaFail,           // FAIL
  kaSendOk,         // SEND OK
  kaSendFail,       // SEND FAIL
  kaAlreadyConnected, // ALREADY CONNECTED
  
} ESPKnownAnswer;
//--------------------------------------------------------------------------------------------------------------------------------
class CoreESPTransport : public CoreTransport
{
  public:
    CoreESPTransport();
    ~CoreESPTransport();


    void broadcast(const String& packet); // запись пакета броадкаста в ESP

    virtual void update(); // обновляем состояние транспорта
    virtual void begin(); // начинаем работу

    // возвращает IP клиента и станции
    bool getIP(String& staIP, String& apIP);
    
    // возвращает MAC клиента и станции
    bool getMAC(String& staMAC, String& apMAC);

    // пропинговать гугл
    bool pingGoogle(bool& result);

    virtual bool ready(); // проверяем на готовность к работе
       
    void restart();
    void readFromStream();

    bool isConnectedToRouter() { return flags.connectedToRouter; }
    uint8_t getSignalQuality() { return signalQuality; }

  protected:

    virtual void beginWrite(CoreTransportClient& client); // начинаем писать в транспорт с клиента
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port); // начинаем коннектиться к адресу
    virtual void beginDisconnect(CoreTransportClient& client); // начинаем отсоединение от адреса

  private:

      uint8_t badPingAttempts;
      uint32_t internalPingTimer;

      void power(bool on);

      String packetToBroadcast;
      
      uint8_t signalQuality;

      // буфер для приёма команд от ESP
      TransportReceiveBuffer receiveBuffer;
      uint16_t recursionGuard;
      
      CoreTransportClient* cipstartConnectClient;
      uint8_t cipstartConnectClientID;

      bool checkIPD(const TransportReceiveBuffer& buff);
      void processKnownStatusFromESP(const String& line);
      void processConnect(const String& line);
      void processDisconnect(const String& line);

      bool isESPBootFound(const String& line);
      bool isKnownAnswer(const String& line, ESPKnownAnswer& result);

      CoreESPTransportFlags flags; // флаги состояния
      ESPMachineState machineState; // состояние конечного автомата

      ESPCommands currentCommand;
      ESPCommandsList initCommandsQueue; // очередь команд на инициализацию
      uint32_t timer, idleTimer; // общий таймер
      uint32_t idleTime; // время, которое нам надо подождать, ничего не делая
      
      void clearInitCommands();
      void createInitCommands(bool addResetCommand);
      
      void sendCommand(const String& command, bool addNewLine=true);
      void sendCommand(ESPCommands command);
      
      Stream* workStream; // поток, с которым мы работаем (читаем/пишем в/из него)

      TransportClientsQueue clientsQueue; // очередь действий с клиентами

      Vector<String*> specialCommandResults;
      void clearSpecialCommandResults();

      void clearClientsQueue(bool raiseEvents);
      bool isClientInQueue(CoreTransportClient* client, TransportClientAction action); // тестирует - не в очереди ли уже клиент?
      void addClientToQueue(CoreTransportClient* client, TransportClientAction action, const char* ip=NULL, uint16_t port=0); // добавляет клиента в очередь
      void removeClientFromQueue(CoreTransportClient* client); // удаляет клиента из очереди  
      void removeClientFromQueue(CoreTransportClient* client, TransportClientAction action); // удаляет клиента из очереди  
      
      void initClients();
    
};
//--------------------------------------------------------------------------------------------------------------------------------
extern CoreESPTransport ESP;
//--------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WIFI_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SMS_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool ready                : 1; // флаг готовности
  bool isModuleRegistered   : 1; // флаг регистрации в сети
  bool isAnyAnswerReceived  : 1; // флаг, что мы получили хотя бы один ответ от модема
  bool waitForDataWelcome   : 1; // флаг, что ждём приглашения на отсыл данных
  bool onIdleTimer          : 1; // флаг, что мы в режиме простоя
  bool gprsAvailable        : 1; //
  bool pduInNextLine        : 1; //
  bool waitCipstartConnect  : 1;
  bool isSIMCardPresent     : 1; // флаг наличия SIM-карты

  bool ignoreNextEmptyLine  : 1;
  
} CoreSIM800TransportFlags;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  smaNone, // ничего не делаем
  smaCheckReady, // надо получить ready от модуля
  smaEchoOff, // выключаем эхо
  smaDisableCellBroadcastMessages, 
  smaAON, 
  smaPDUEncoding,
  smaUCS2Encoding,
  smaSMSSettings,
  smaWaitReg,
  smaCheckModemHang, // проверяем на зависание модема 
  smaCIPHEAD,
  smaCIPMODE,
  smaCIPMUX,
  smaCIICR, // activate GPRS connection
  smaCIFSR, // check GPRS connection
  smaCSTT, // setup GPRS
  smaHangUp, // кидаем трубку
  smaCIPSTART, // начинаем коннектиться к адресу
  smaCIPSEND,
  smaWaitSendDone,
  smaCIPCLOSE,
  smaCMGS,
  smaWaitSMSSendDone,
  smaWaitForSMSClearance,
  smaCUSD,
  smaGPRSMultislotClass,
  smaPING,
  smaCIPSHUT,
  smaCGATT,
  smaCGATTach,

  //////////////////////////////
  // синхронизация времени
  //////////////////////////////
  smaStartTimeSync,
  smaSaveSyncSettings,
  smaGetSyncTime,

  smaCheckSimCardPresent,
  smaEnableSimDetection,
    
} SIM800Commands;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<SIM800Commands> SIM800CommandsList;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  sim800Idle,        // состояние "ничего не делаем"
  sim800WaitAnswer,  // состояние "ждём ответа на команду, посланную SIM800"
  sim800Reboot,      // состояние "SIM800 в процессе перезагрузки"
  sim800WaitInit,    // ждём инициализации после подачи питания
  sim800WaitBootBegin, // отправляем первую команду AT, чтобы модем знал, что с ним общаются
  sim800WaitBoot,    // ждём в порту строчки "Call Ready или SMS Ready"
  
} SIM800MachineState;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  gsmNone,
  gsmOK,             // OK
  gsmError,          // ERROR
  gsmFail,           // FAIL
  gsmSendOk,         // SEND OK
  gsmSendFail,       // SEND FAIL
  gsmConnectOk,
  gsmConnectFail,
  gsmAlreadyConnect,
  gsmCloseOk,
  gsmShutOk,
  
} SIM800KnownAnswer;
//--------------------------------------------------------------------------------------------------------------------------------------
#define SIM800_MAX_CLIENTS 5
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool isFlash;
  String* phone;
  String* message;
  
} SIM800OutgoingSMS;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<SIM800OutgoingSMS> SIM800OutgoingSMSList;
//--------------------------------------------------------------------------------------------------------------------------------------
extern "C" {
  void ON_INCOMING_CALL(const String& phoneNumber, bool isKnownNumber, bool& shouldHangUp);
  void ON_SMS_RECEIVED(const String& phoneNumber,const String& message, bool isKnownNumber);
  void ON_CUSD_RECEIVED(const String& cusd);
}
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreSIM800Transport : public CoreTransport
{
  public:
    CoreSIM800Transport();
    ~CoreSIM800Transport();


    virtual void update(); // обновляем состояние транспорта
    virtual void begin(); // начинаем работу

    virtual bool ready(); // проверяем на готовность к работе

    bool sendSMS(const String& phoneNumber, const String& message, bool isFlash);
    void sendCUSD(const String& cusd);

    void restart();
    void readFromStream();

    #ifdef GSM_DEBUG_MODE
    void dumpReceiveBuffer();
    #endif

    uint8_t getSignalQuality() { return signalQuality; }
    bool hasGPRSConnection() {return flags.gprsAvailable; }

  protected:

    virtual void beginWrite(CoreTransportClient& client); // начинаем писать в транспорт с клиента
    virtual void beginConnect(CoreTransportClient& client, const char* ip, uint16_t port); // начинаем коннектиться к адресу
    virtual void beginDisconnect(CoreTransportClient& client); // начинаем отсоединение от адреса

  private:

      void power(bool on);
      void powerkeyInit();

      bool PSUTTZreceived;
      bool syncTimeTimerEnabled;
      uint32_t syncTimeTimer;

      bool inWaitingGPRS_CIICR;
      uint32_t gprsReconnectTimer;
      bool waitGprsReconnect;
      
      bool csqReceived;
      bool cgattStatusReceived, cgattStatus, cgattKnownAnswerFound;
      uint8_t signalQuality;

      uint8_t badPingAttempts;
      
      void rebootModem();

      Vector<String*> cusdList;
      void sendQueuedCUSD();

      SIM800OutgoingSMSList outgoingSMSList;
      String* smsToSend;
      void sendQueuedSMS();

      // буфер для приёма команд от SIM800
      TransportReceiveBuffer receiveBuffer;
      uint16_t recursionGuard;
  
      CoreTransportClient* cipstartConnectClient;
      uint8_t cipstartConnectClientID;

      void GetAPNUserPass(String& user, String& pass);
      String GetAPN();
      
      bool checkIPD(const TransportReceiveBuffer& buff);
      void processCMT(const String& cmtInfo);
      void processIncomingCall(const String& call);
      void processKnownStatusFromSIM800(const String& line);

      bool isKnownAnswer(const String& line, SIM800KnownAnswer& result);

      CoreSIM800TransportFlags flags; // флаги состояния
      SIM800MachineState machineState; // состояние конечного автомата

      SIM800Commands currentCommand;
      SIM800CommandsList initCommandsQueue; // очередь команд на инициализацию
      
      uint32_t timer, idleTimer; // общий таймер
      uint32_t idleTime; // время, которое нам надо подождать, ничего не делая
      
      void clearInitCommands();
      void createInitCommands(bool addResetCommand);
      
      void sendCommand(const String& command, bool addNewLine=true);
      void sendCommand(SIM800Commands command);
      
      Stream* workStream; // поток, с которым мы работаем (читаем/пишем в/из него)

      int16_t gprsCheckingAttempts;

      void clearClientsQueue(bool raiseEvents);

      TransportClientsQueue clientsQueue; // очередь действий с клиентами

      bool isClientInQueue(CoreTransportClient* client, TransportClientAction action); // тестирует - не в очереди ли уже клиент?
      void addClientToQueue(CoreTransportClient* client, TransportClientAction action, const char* ip=NULL, uint16_t port=0); // добавляет клиента в очередь
      void removeClientFromQueue(CoreTransportClient* client); // удаляет клиента из очереди
      void removeClientFromQueue(CoreTransportClient* client, TransportClientAction action);
          
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreSIM800Transport SIM800;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SMS_MODULE
//--------------------------------------------------------------------------------------------------------------------------------
#define MQTT_CONNECT_COMMAND (1 << 4)
#define MQTT_PUBLISH_COMMAND (3 << 4)
#define MQTT_SUBSCRIBE_COMMAND (8 << 4)
#define MQTT_QOS1 (1 << 1)
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<uint8_t> MQTTBuffer;
//--------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  mqttWaitClient, // ожидаем свободного клиента
  mqttWaitConnection, // ожидаем подсоединения к брокеру
  mqttWaitReconnect, // ожидаем переподсоединения к брокеру (в случае неуспешного соединения)
  mqttSendConnectPacket, // отсылаем пакет с информацией о подсоединении к брокеру
  mqttWaitSendConnectPacketDone,
  mqttSendSubscribePacket, // отсылаем пакет с информацией о подписке
  mqttWaitSendSubscribePacketDone,
  mqttSendPublishPacket, // отсылаем пакет публикации
  mqttWaitSendPublishPacketDone,
  
} MQTTState;
//--------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  char* topic;
  char* payload;
  
} MQTTPublishQueue;
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<MQTTPublishQueue> MQTTPublishList;
//--------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  String serverAddress;
  int port;
  String clientID;
  String userName;
  String password;
  
} MQTTSettings;
//--------------------------------------------------------------------------------------------------------------------------------
class CoreMQTT : public IClientEventsSubscriber//, public Stream
{
  public:
    CoreMQTT();
    void reset();
    void update();
    void begin(CoreTransport* transport);
    bool enabled();

  // IClientEventsSubscriber
  virtual void OnClientConnect(CoreTransportClient& client, bool connected, int16_t errorCode); // событие "Статус соединения клиента"
  virtual void OnClientDataWritten(CoreTransportClient& client, int16_t errorCode); // событие "Данные из клиента записаны в поток"
  virtual void OnClientDataAvailable(CoreTransportClient& client, uint8_t* data, size_t dataSize, bool isDone); // событие "Для клиента поступили данные", флаг - все ли данные приняты

  // для публикации любого стороннего топика
  bool publish(const char* topicName, const char* payload);

  void reloadSettings();

  void AddTopic(const char* topicIndex, const char* topicName, const char* moduleName, const char* sensorType, const char* sensorIndex, const char* topicType);
  void DeleteAllTopics();
  uint16_t GetSavedTopicsCount();


private:

  TransportReceiveBuffer packetBuffer;

  void getNextTopic(String& topicName, String& data);
  void switchToNextTopic();

  MQTTSettings currentSettings;
  MQTTSettings getSettings();

  MQTTPublishList publishList;
  void clearPublishQueue();

  CoreTransportClient currentClient;
  CoreTransport* currentTransport;
  uint32_t timer;

  MQTTState machineState;
  uint16_t mqttMessageId;
  String* streamBuffer;

  uint32_t intervalBetweenTopics;
  uint16_t currentTopicNumber;

  void pushToReportQueue(String* toReport);
  Vector<String*> reportQueue;
  void clearReportsQueue();

  void constructFixedHeader(byte command, MQTTBuffer& fixedHeader,size_t payloadSize);

  void constructConnectPacket(String& mqttBuffer,int16_t& mqttBufferLength,const char* id, const char* user, const char* pass,const char* willTopic,uint8_t willQoS, uint8_t willRetain, const char* willMessage);
  void constructSubscribePacket(String& mqttBuffer,int16_t& mqttBufferLength, const char* topic);
  void constructPublishPacket(String& mqttBuffer,int16_t& mqttBufferLength, const char* topic, const char* payload, bool retain);
  
  void encode(MQTTBuffer& buff,const char* str);

  void writePacket(MQTTBuffer& fixedHeader,MQTTBuffer& payload, String& mqttBuffer,int16_t& mqttBufferLength);

  void processIncomingPacket(CoreTransportClient* client, uint8_t* data, size_t dataLength);

  void convertAnswerToJSON(const String& answer, String* resultBuffer);
};
//--------------------------------------------------------------------------------------------------------------------------------

