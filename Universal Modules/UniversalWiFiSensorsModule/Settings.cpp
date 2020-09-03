#include "Settings.h"
#include "CONFIG.h"
#include <EEPROM.h>
//--------------------------------------------------------------------------------------------------------------------------------------
// список поддерживаемых команд
//--------------------------------------------------------------------------------------------------------------------------------------
const char ROUTER_COMMAND[] PROGMEM = "ROUTER"; // получить/установить идентификатор станции и пароль для ESP
const char INTERVAL_COMMAND[] PROGMEM = "INTERVAL"; // получить/установить интервал между отсылом показаний датчиков
//--------------------------------------------------------------------------------------------------------------------------------------
CoreCommandBuffer Commands(&Serial);
//--------------------------------------------------------------------------------------------------------------------------------------
CoreCommandBuffer::CoreCommandBuffer(Stream* s) : pStream(s)
{
    strBuff = new String();
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CoreCommandBuffer::hasCommand()
{
  if(!(pStream && pStream->available()))
    return false;

    char ch;
    while(pStream->available())
    {
      ch = (char) pStream->read();
            
      if(ch == '\r' || ch == '\n')
      {
        return strBuff->length() > 0; // вдруг лишние управляющие символы придут в начале строки?
      } // if

      *strBuff += ch;
      // не даём вычитать больше символов, чем надо - иначе нас можно заспамить
      if(strBuff->length() >= 200)
      {
         clearCommand();
         return false;
      } // if
    } // while

    return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
CommandParser::CommandParser()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
CommandParser::~CommandParser()
{
  clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandParser::clear()
{
  for(size_t i=0;i<arguments.size();i++)
  {
    delete [] arguments[i];  
  }

  arguments.empty();
  //while(arguments.size())
  //  arguments.pop();
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* CommandParser::getArg(size_t idx) const
{
  if(arguments.size() && idx < arguments.size())
    return arguments[idx];

  return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandParser::parse(const String& command, bool isSetCommand)
{
  clear();
    // разбиваем на аргументы
    const char* startPtr = command.c_str() + strlen_P(isSetCommand ? (const char* )CORE_COMMAND_SET : (const char*) CORE_COMMAND_GET);
    size_t len = 0;

    while(*startPtr)
    {
      const char* delimPtr = strchr(startPtr,CORE_COMMAND_PARAM_DELIMITER);
            
      if(!delimPtr)
      {
        len = strlen(startPtr);
        char* newArg = new char[len + 1];
        memset(newArg,0,len+1);
        strncpy(newArg,startPtr,len);
        arguments.push_back(newArg);        

        return arguments.size();
      } // if(!delimPtr)

      size_t len = delimPtr - startPtr;

     
      char* newArg = new char[len + 1];
      memset(newArg,0,len+1);
      strncpy(newArg,startPtr,len);
      arguments.push_back(newArg);

      startPtr = delimPtr + 1;
      
    } // while      

  return arguments.size();
    
}
//--------------------------------------------------------------------------------------------------------------------------------------
// CommandHandlerClass
//--------------------------------------------------------------------------------------------------------------------------------------
CommandHandlerClass CommandHandler;
//--------------------------------------------------------------------------------------------------------------------------------------
CommandHandlerClass::CommandHandlerClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandHandlerClass::handleCommands()
{
  if(Commands.hasCommand())
  {    

    String command = Commands.getCommand();

    if(command.startsWith(CORE_COMMAND_GET) || command.startsWith(CORE_COMMAND_SET))
    {
      Stream* pStream = Commands.getStream();
      processCommand(command,pStream);
    }
    

    Commands.clearCommand(); // очищаем буфер команд
  
  } // if(Commands.hasCommand())  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandHandlerClass::processCommand(const String& command,Stream* pStream)
{
    bool commandHandled = false;

    if(command.startsWith(CORE_COMMAND_SET))
    {
      // команда на установку свойств

      CommandParser cParser;
      if(cParser.parse(command,true))
      {
        const char* commandName = cParser.getArg(0);

       
        if(!strcmp_P(commandName, ROUTER_COMMAND))
        {
            // установить параметры станции SET=ROUTER|id|password
            if(cParser.argsCount() > 2)
            {
              commandHandled = setROUTER(cParser, pStream);
            }
            else
            {
              // недостаточно параметров
              commandHandled = printBackSETResult(false,commandName,pStream);
            }
        } // ROUTER_COMMAND
        else
        if(!strcmp_P(commandName, INTERVAL_COMMAND))
        {
            // установить параметры станции SET=INTERVAL|value
            if(cParser.argsCount() > 1)
            {
              commandHandled = setINTERVAL(cParser, pStream);
            }
            else
            {
              // недостаточно параметров
              commandHandled = printBackSETResult(false,commandName,pStream);
            }
        } // INTERVAL_COMMAND        
      
      //TODO: тут разбор команды !!!
      
      } // if(cParser.parse(command,true))
      
    } // SET COMMAND
    else
    if(command.startsWith(CORE_COMMAND_GET))
    {
      // команда на получение свойств
      CommandParser cParser;
      
      if(cParser.parse(command,false))
      {
        const char* commandName = cParser.getArg(0);
        
        if(!strcmp_P(commandName, ROUTER_COMMAND))
        {
            commandHandled = getROUTER(commandName,cParser,pStream);                    
          
        } // ROUTER_COMMAND
        else
        if(!strcmp_P(commandName, INTERVAL_COMMAND))
        {
            commandHandled = getINTERVAL(commandName,cParser,pStream);                    
          
        } // INTERVAL_COMMAND
        
                
        //TODO: тут разбор команды !!!
        
      } // if(cParser.parse(command,false))
      
    } // GET COMMAND
    
    if(!commandHandled)
      onUnknownCommand(command, pStream);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandHandlerClass::onUnknownCommand(const String& command, Stream* outStream)
{
    outStream->print(CORE_COMMAND_ANSWER_ERROR);
    outStream->println(F("UNKNOWN_COMMAND"));  
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandHandlerClass::getROUTER(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  if(parser.argsCount() < 1)
    return false;  


  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(commandPassed);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->print( Settings.getRouterID());
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(Settings.getRouterPassword());   

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandHandlerClass::setROUTER(CommandParser& parser, Stream* pStream)
{

  if(parser.argsCount() < 3)
    return false;
  
  String routerId = parser.getArg(1);
  String routerPass = parser.getArg(2);

  Settings.setRouterID(routerId);
  Settings.setRouterPassword(routerPass);

  Settings.commit();

  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(parser.getArg(0));
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->print(routerId);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(routerPass);


  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandHandlerClass::getINTERVAL(const char* commandPassed, const CommandParser& parser, Stream* pStream)
{
  if(parser.argsCount() < 1)
    return false;  


  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(commandPassed);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(Settings.getInterval());

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandHandlerClass::setINTERVAL(CommandParser& parser, Stream* pStream)
{

  if(parser.argsCount() < 2)
    return false;
  
  uint32_t interval = atol(parser.getArg(1));

  Settings.setInterval(interval);

  Settings.commit();

  pStream->print(CORE_COMMAND_ANSWER_OK);

  pStream->print(parser.getArg(0));
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);
  pStream->println(interval);


  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandHandlerClass::printBackSETResult(bool isOK, const char* command, Stream* pStream)
{
  if(isOK)
    pStream->print(CORE_COMMAND_ANSWER_OK);
  else
    pStream->print(CORE_COMMAND_ANSWER_ERROR);

  pStream->print(command);
  pStream->print(CORE_COMMAND_PARAM_DELIMITER);

  if(isOK)
    pStream->println(F("OK"));
  else
    pStream->println(F("BAD_PARAMS"));

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
// SettingsClass
//--------------------------------------------------------------------------------------------------------------------------------------
SettingsClass Settings;
//--------------------------------------------------------------------------------------------------------------------------------------
SettingsClass::SettingsClass()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::begin()
{
  EEPROM.begin(1024);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::commit()
{
  EEPROM.commit();
}
//--------------------------------------------------------------------------------------------------------------------------------------
String SettingsClass::getRouterID()
{
  String result = readString(ROUTER_ID_ADDRESS,20);
  if(!result.length())
    result = CONTROLLER_SSID;

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::setRouterID(const String& val)
{
  writeString(ROUTER_ID_ADDRESS,val,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String SettingsClass::getRouterPassword()
{
  String result = readString(ROUTER_PASSWORD_ADDRESS,20);
  if(!result.length())
    result = CONTROLLER_PASSWORD;

  return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::setRouterPassword(const String& val)
{
  writeString(ROUTER_PASSWORD_ADDRESS,val,20);
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t SettingsClass::getInterval()
{
  uint32_t address = INTERVAL_ADDRESS;
  uint8_t header1 = read8(address++);
  uint8_t header2 = read8(address++);

  if(!(header1 == STX1 && header2 == STX2 ))
    return UPDATE_INTERVAL;
  
  uint32_t val = read32(address,UPDATE_INTERVAL); 
  if(val < 1000)
    val = 1000;

    return val;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::setInterval(uint32_t val)
{
  uint32_t address = INTERVAL_ADDRESS;
  
  write8(address++,STX1);
  write8(address++,STX2);
  
  write32(address,val);
}
//--------------------------------------------------------------------------------------------------------------------------------------
String SettingsClass::readString(uint16_t address, byte maxlength)
{
  String result;
  
  uint8_t header1 = read8(address++);
  uint8_t header2 = read8(address++);
  
  if(!(header1 == STX1 && header2 == STX2 ))
    return result;
  
  for(byte i=0;i<maxlength;i++)
  {
    byte b = read8(address++,0);
    if(b == 0)
      break;

    result += (char) b;
  }

  return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::writeString(uint16_t address, const String& v, byte maxlength)
{

  write8(address++,STX1);
  write8(address++,STX2);
  
  for(byte i=0;i<maxlength;i++)
  {
    if(i >= v.length())
      break;
      
    write8(address++,v[i]);
  }

  // пишем завершающий ноль
  write8(address++,'\0');
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::write8(uint16_t address, uint8_t val)
{
  EEPROM.write(address,val); 
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t SettingsClass::read8(uint16_t address, uint8_t defaultVal)
{
  uint8_t val = EEPROM.read(address);
  if(val == 0xFF)
    val = defaultVal;

 return val;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint32_t SettingsClass::read32(uint16_t address, uint32_t defaultVal)
{
  uint32_t result = 0;

  EEPROM.get(address,result);

  if(result == 0xFFFFFFFF)
    result = defaultVal;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::write32(uint16_t address, uint32_t val)
{
    EEPROM.put(address,val);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
uint16_t SettingsClass::read16(uint16_t address, uint16_t defaultVal)
{
  uint16_t result = 0;
  EEPROM.get(address,result);

  if(result == 0xFFFF)
    result = defaultVal;

  return result;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SettingsClass::write16(uint16_t address, uint16_t val)
{
    EEPROM.put(address,val);  
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------



