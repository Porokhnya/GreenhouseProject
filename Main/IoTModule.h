#ifndef _IOT_MODULE_H
#define _IOT_MODULE_H

#include "AbstractModule.h"
#include "IoT.h"
//--------------------------------------------------------------------------------------------------------------------------------------
class IoTModule : public AbstractModule // модуль отсылки данных в IoT-хранилища
{
  private:

#if defined(USE_IOT_MODULE)

  void CollectDataForThingSpeak();

  void SwitchToWaitMode();
  void SwitchToNextService();

  String* dataToSend; // данные для отсылки
  unsigned long updateTimer;
  bool inSendData;

  IoTServices services;
  IoTService currentService;
  int currentGateIndex;

  void SendDataToIoT();
  void ProcessNextGate();

  AbstractModule* FindModule(byte index);
  
#endif
  
  public:
    IoTModule() : AbstractModule("IOT") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

#if defined(USE_IOT_MODULE)
    void Write(Stream* writeTo);
    void Done(const IoTCallResult& result);
 #endif

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
