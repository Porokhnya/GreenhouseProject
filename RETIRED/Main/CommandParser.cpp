#include <Arduino.h>
#include "CommandParser.h"
//--------------------------------------------------------------------------------------------------------------------------------------
Command::Command()
{

  Clear();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
Command::~Command()
{
  Clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t Command::GetArgsCount() const
{ 
  return arguments.size();
}
//--------------------------------------------------------------------------------------------------------------------------------------
const char* Command::GetArg(size_t idx) const
{
  if(idx < arguments.size())
    return arguments[idx];

 return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Command::Construct(const char* moduleID,const char* rawArgs, const char* ct)
{
  uint8_t commandType = ctGET;
  if(!strcmp_P(ct,(const char*) CMD_SET))
    commandType = ctSET;

  Construct(moduleID,rawArgs,commandType);
 
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Command::Construct(const char* id, const char* rawArgs, uint8_t ct)
{
  Clear(); // сбрасываем все настройки
  
    Type = ct;
    ModuleID = id;

    if(!rawArgs) // нет аргументов
      return;
       
    // разбиваем на аргументы
    const char* startPtr = rawArgs;
    size_t len = 0;

    while(*startPtr)
    {
      const char* delimPtr = strchr(startPtr,'|');
            
      if(!delimPtr)
      {
        len = strlen(startPtr);
        char* newArg = new char[len + 1];
        memset(newArg,0,len+1);
        strncpy(newArg,startPtr,len);
        arguments.push_back(newArg);        

        return;
      } // if(!delimPtr)

      size_t len = delimPtr - startPtr;

     
      char* newArg = new char[len + 1];
      memset(newArg,0,len+1);
      strncpy(newArg,startPtr,len);
      arguments.push_back(newArg);


      startPtr = delimPtr + 1;
      
    } // while    
         
}
//--------------------------------------------------------------------------------------------------------------------------------------
void Command::Clear()
{
  Type = ctUNKNOWN;
  ModuleID = F("");
  IncomingStream = NULL;
  bIsInternal = false;

  size_t sz = arguments.size();
  for(size_t i=0;i<sz;i++)
  {
    char* ch = arguments[i];
    delete[] ch;
  } // for
  arguments.clear();

}
//--------------------------------------------------------------------------------------------------------------------------------------  
CommandParser::CommandParser()
{  
  Clear();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void CommandParser::Clear()
{

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandParser::ParseCommand(const String& command, Command& outCommand)
{
  Clear(); // clear first

  if(command.length() < MIN_COMMAND_LENGTH)
    return false;

  const char* readPtr = command.c_str();
  
  bool rightPrefix = !strncmp_P(readPtr,(const char*)CMD_PREFIX,CMD_PREFIX_LEN);

  if(!rightPrefix)
    return false;


  // перемещаемся за префикс (CT или CD)
  readPtr += CMD_PREFIX_LEN;

  // проверяем, GET или SET должно быть передано
  bool isGet = !strncmp_P(readPtr,(const char*)CMD_GET,CMD_TYPE_LEN);
  bool rightType =  isGet || !strncmp_P(readPtr,(const char*)CMD_SET,CMD_TYPE_LEN);
  if(!rightType)
    return false;


  uint8_t commandType = isGet ? ctGET : ctSET;

  // перемещаемся за тип команды и знак '='
  readPtr += CMD_TYPE_LEN + 1;

  // ищем, есть ли разделитель в строке. Если он есть, значит, передали ещё и параметры помимо просто имени модуля
  const char* delimPtr = strchr(readPtr,'|');
  if(!delimPtr)
  {
    // без параметров, тупо конструируем и выходим
     outCommand.Construct(readPtr,NULL,commandType);
     return true;
  }

  // есть параметры, надо выцепить имя модуля
  size_t len = (delimPtr - readPtr);

  
  char* moduleName = new char[len+1];
  memset(moduleName,0,len+1);
  strncpy(moduleName,readPtr,len);
  
  delimPtr++;

  outCommand.Construct(moduleName,delimPtr,commandType);
  delete[] moduleName;
  return true;
   
}
//--------------------------------------------------------------------------------------------------------------------------------------

