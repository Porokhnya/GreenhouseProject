#ifndef _HTTP_MODULE_H
#define _HTTP_MODULE_H

#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------
#include "HTTPInterfaces.h" // подключаем интерфейсы для работы с HTTP-запросами
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
struct HttpModuleFlags
{
  bool inProcessQuery: 1;
  byte currentAction: 2;
  byte isEnabled: 1;
  bool isFirstUpdateCall: 1;
  byte currentProviderNumber: 3;
};
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------
enum
{
    HTTP_ASK_FOR_COMMANDS,
    HTTP_REPORT_TO_SERVER
};
//--------------------------------------------------------------------------------------------------------------------------------
typedef Vector<String*> HTTPReportList;
//--------------------------------------------------------------------------------------------------------------------------------
class HttpModule : public AbstractModule, public HTTPRequestHandler
{
  private:


   HTTPQueryProvider* providers[2]; // наши провайдеры - Wi-Fi и GSM
  
   long waitTimer;
   unsigned long commandsCheckTimer;
   HttpModuleFlags flags;

   HTTPReportList commandsToReport;
   
   void CheckForIncomingCommands(byte wantedAction);
   void CollectSensorsData(String* data);
   void CollectControllerStatus(String* data);
   uint8_t MapFraction(uint8_t fraction);
  
  public:
    HttpModule() : AbstractModule("HTTP") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);
    
  virtual void OnAskForHost(String& host, int& port); // вызывается для запроса имени хоста
  virtual void OnAskForData(String* data); // вызывается для запроса данных, которые надо отправить HTTP-запросом
  virtual void OnAnswerLineReceived(String& line, bool& enough); // вызывается по приходу строки ответа от сервера, вызываемая сторона должна сама определить, когда достаточно данных.
  virtual void OnHTTPResult(uint16_t statusCode); // вызывается по завершению HTTP-запроса и получению ответа от сервера    

};
//--------------------------------------------------------------------------------------------------------------------------------
#endif
