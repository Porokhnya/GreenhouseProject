#include "EthernetModule.h"
#include "ModuleController.h"

#ifdef USE_W5100_MODULE

#include <Ethernet.h>
//--------------------------------------------------------------------------------------------------------------------------------------
// наш локальный мак-адрес
//--------------------------------------------------------------------------------------------------------------------------------------
byte local_mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
//--------------------------------------------------------------------------------------------------------------------------------------
// IP-адрес по умолчанию
//--------------------------------------------------------------------------------------------------------------------------------------
IPAddress default_ip(192, 168, 0, 177);
//--------------------------------------------------------------------------------------------------------------------------------------
// наш сервер, который будет обработывать клиентов
//--------------------------------------------------------------------------------------------------------------------------------------
EthernetServer lanServer(1975);
//--------------------------------------------------------------------------------------------------------------------------------------
void EthernetModule::Setup()
{
  // настраиваем всё необходимое добро тут
  #ifdef USE_W5100_REBOOT_PIN
    WORK_STATUS.PinMode(W5100_REBOOT_PIN,OUTPUT);
    WORK_STATUS.PinWrite(W5100_REBOOT_PIN,W5100_POWER_ON);
  #endif
  
  bInited = false;
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void EthernetModule::Update(uint16_t dt)
{ 
  UNUSED(dt);
  // обновление модуля тут

  if(!bInited) // не было инициализации, инициализируемся
  {

   #ifdef ETHERNET_DEBUG
    DEBUG_LOGLN(F("[LAN] Start server using DHCP... "));
   #endif 

    // пытаемся по DHCP получить конфигурацию
    if(!Ethernet.begin(local_mac))
    {

     #ifdef ETHERNET_DEBUG
      DEBUG_LOGLN(F("[LAN] DHCP failed, start with defailt IP 192.168.0.177 "));
     #endif      
      // стартуем пока с настройками по умолчанию
      Ethernet.begin(local_mac, default_ip);
    }
    
    lanServer.begin();    

  #ifdef ETHERNET_DEBUG
    DEBUG_LOG(F("[LAN] server started at "));
    DEBUG_LOGLN(String(Ethernet.localIP()));
  #endif

  WORK_STATUS.PinMode(10,OUTPUT,false);
  WORK_STATUS.PinMode(MOSI,OUTPUT,false);
  WORK_STATUS.PinMode(MISO,INPUT,false);
  WORK_STATUS.PinMode(SCK,OUTPUT,false);  

    bInited = true;
    return;
    
  } // if(!bInited)

  EthernetClient client = lanServer.available();
  if(client)
  {
    // есть активный клиент
    uint8_t sockNumber = client.getSocketNumber(); // получили номер сокета клиента

    while(client.available()) // пока есть данные с клиента
    {
      char c = client.read(); // читаем символ
      
      if(c == '\r') // этот символ нам не нужен, мы ждём '\n'
        continue;

      if(c == '\n') // дождались перевода строки
      {
        // пытаемся распарсить команду
        Command cmd;
        CommandParser* cParser = MainController->GetCommandParser();

        if(cParser->ParseCommand(clientCommands[sockNumber], cmd))
        {
          // команду разобрали, выполняем
          
          cmd.SetIncomingStream(&client); // назначаем команде поток, куда выводить данные

          // запустили команду в обработку
          MainController->ProcessModuleCommand(cmd);
        }

        // останавливаем клиента, т.к. все данные ему уже посланы.
        // даже если команда неправильная - считаем, что раз мы
        // получили строку, значит, имеем полное право с ней работать,
        // и каждый ССЗБ, если пришло что-то не то.
        client.stop();

        // очищаем внутренний буфер, подготавливая его к приёму следующей команды
        clientCommands[sockNumber] = ""; 
        
        break; // выходим из цикла
        
      } // if(c == '\n')

      // если символ не '\r' и не '\n' -
      // запоминаем его во внутренний буфер, 
      // привязанный к номеру клиента
      clientCommands[sockNumber] += c; 
      
    } // while

    Ethernet.maintain(); // обновляем состояние Ethernet
    
  } // if(client)

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool EthernetModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);
  UNUSED(command);

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_W5100_MODULE

