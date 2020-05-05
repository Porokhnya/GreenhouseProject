#ifndef _COMPOSITE_COMMANDS_MODULE_H
#define _COMPOSITE_COMMANDS_MODULE_H

#include "AbstractModule.h"
#include "TinyVector.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ccCloseWindows = 0, // закрыть форточки
  ccOpenWindows = 1, // открыть форточки
  ccLightOff = 2, // выключить досветку
  ccLightOn = 3, // включить досветку
  ccPinOff = 4, // выставить на пине низкий уровень
  ccPinOn = 5 // выставить на пине высокий уровень
  
} CompositeCommandAction; // действие, которое может выполнять составная команда
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t command; // команда на выполнение
  uint8_t data; // дополнительные данные
  
} CompositeCommand; // составная команда
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CompositeCommand*> CompositeCommandVector;
//--------------------------------------------------------------------------------------------------------------------------------------
struct CompositeCommands // класс одной составной команды
{
  CompositeCommandVector Commands; // список команд на выполнение
}; 
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<CompositeCommands*> CompositeCommandsVector;
//--------------------------------------------------------------------------------------------------------------------------------------
class CompositeCommandsModule : public AbstractModule // модуль обработки составных команд
{
  private:
    CompositeCommandsVector commands; // наши команды на выполнение
    void LoadCommands(); // загружаем команды
    void SaveCommands(); // сохраняем команды
    void Clear(); // очищаем все команды
    
    void AddCommand(uint8_t listIdx, uint8_t action, uint8_t param); // добавляем команду в список
    void ProcessCommand(uint8_t idx); // выполняем составную команду
    
  public:
    CompositeCommandsModule() : AbstractModule("CC") {}

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
