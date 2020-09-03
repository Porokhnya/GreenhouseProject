#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define ROUTER_ID_ADDRESS 0
#define ROUTER_PASSWORD_ADDRESS 30
#define INTERVAL_ADDRESS 60
//--------------------------------------------------------------------------------------------------------------------------------------
#define STX1 0xAB
#define STX2 0xBA
//--------------------------------------------------------------------------------------------------------------------------------------
#define CORE_COMMAND_GET F("GET=") // префикс для команды получения данных из ядра
#define CORE_COMMAND_SET F("SET=") // префикс для команды сохранения данных в ядро
#define CORE_COMMAND_ANSWER_OK F("OK=") // какой префикс будет посылаться в ответ на команду получения данных и её успешной отработке
#define CORE_COMMAND_ANSWER_ERROR F("ER=") // какой префикс будет посылаться в ответ на команду получения данных и её неуспешной отработке
#define CORE_COMMAND_PARAM_DELIMITER '|' // разделитель параметров
//--------------------------------------------------------------------------------------------------------------------------------------
// класс для накопления команды из потока
//--------------------------------------------------------------------------------------------------------------------------------------
class CoreCommandBuffer
{
private:
  Stream* pStream;
  String* strBuff;
public:
  CoreCommandBuffer(Stream* s);

  bool hasCommand();
  const String& getCommand() {return *strBuff;}
  void clearCommand() {delete strBuff; strBuff = new String(); }
  Stream* getStream() {return pStream;}

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CoreCommandBuffer Commands;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<char*> CommandArgsVec;
//--------------------------------------------------------------------------------------------------------------------------------------
class CommandParser
{
  private:
    CommandArgsVec arguments;
  public:
    CommandParser();
    ~CommandParser();

    void clear();
    bool parse(const String& command, bool isSetCommand);
    const char* getArg(size_t idx) const;
    size_t argsCount() const {return arguments.size();}
};
//--------------------------------------------------------------------------------------------------------------------------------------
class CommandHandlerClass
{
  public:
  
    CommandHandlerClass();
    
    void handleCommands();
    void processCommand(const String& command,Stream* outStream);


 private:
  void onUnknownCommand(const String& command, Stream* outStream);
  
  bool getROUTER(const char* commandPassed, const CommandParser& parser, Stream* pStream);
  bool setROUTER(CommandParser& parser, Stream* pStream);

  bool getINTERVAL(const char* commandPassed, const CommandParser& parser, Stream* pStream);
  bool setINTERVAL(CommandParser& parser, Stream* pStream);

  bool printBackSETResult(bool isOK, const char* command, Stream* pStream);
    
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern CommandHandlerClass CommandHandler;
//--------------------------------------------------------------------------------------------------------------------------------------
class SettingsClass
{

    uint8_t read8(uint16_t address, uint8_t defaultVal = 0);
    void write8(uint16_t address, uint8_t val);

    uint16_t read16(uint16_t address, uint16_t defaultVal = 0);
    void write16(uint16_t address, uint16_t val);

    uint32_t read32(uint16_t address, uint32_t defaultVal = 0);
    void write32(uint16_t address, uint32_t val);
    
    String readString(uint16_t address, byte maxlength);
    void writeString(uint16_t address, const String& v, byte maxlength);

  
  public:
    SettingsClass();

    void begin();
    void commit();

    String getRouterID();
    void setRouterID(const String& val);

    String getRouterPassword();
    void setRouterPassword(const String& val);

    uint32_t getInterval();
    void setInterval(uint32_t val);
  
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern SettingsClass Settings;
//--------------------------------------------------------------------------------------------------------------------------------------

