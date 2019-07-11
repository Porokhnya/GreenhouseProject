#include "StatModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == DUE_BOARD)
    #include <malloc.h>
    #include <stdlib.h>
    #include <stdio.h>
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == STM32_BOARD)
extern "C" char* sbrk(int i);
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
// выводит свободную память
//--------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == MEGA_BOARD)
struct __freelist 
{
  size_t sz;
  struct __freelist *nx;
};
extern struct __freelist *__flp;
int freeListSize() 
{
  struct __freelist* current;
  int total = 0;
  for (current = __flp; current; current = current->nx) 
  {
    total += 2; /* Add two bytes for the memory block's header  */
    total += (int) current->sz;
  }
  return total;
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
int freeRam() 
{
  #if (TARGET_BOARD == MEGA_BOARD)
   extern int __heap_start, *__brkval;
   int free_memory;
    if ((int)__brkval == 0) 
    {
      free_memory = ((int)&free_memory) - ((int)&__heap_start);
    } 
    else 
    {
      free_memory = ((int)&free_memory) - ((int)__brkval);
      free_memory += freeListSize();
    }
    return free_memory;    

 #elif (TARGET_BOARD == DUE_BOARD)

    struct mallinfo mi = mallinfo();
    char* heapend = _sbrk(0);
    register char* stack_ptr asm("sp");

    return (stack_ptr - heapend + mi.fordblks);

#elif (TARGET_BOARD == STM32_BOARD)

    char top = 't';
    return &top - reinterpret_cast<char*>(sbrk(0));
        
 #else
  #error "Unknown target board!"
 #endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
void StatModule::Setup()
{
  // настройка модуля статистики тут
  uptime = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void StatModule::Update(uint16_t dt)
{ 
  // обновление модуля статистики тут
  uptime += dt;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  StatModule::ExecCommand(const Command& command, bool wantAnswer)
{
  if(wantAnswer) PublishSingleton = UNKNOWN_COMMAND;
  PublishSingleton.Flags.AddModuleIDToAnswer = false;

  size_t argsCount = command.GetArgsCount();
  
  if(command.GetType() == ctSET) 
  {
      if(wantAnswer) 
        PublishSingleton = NOT_SUPPORTED;
  }
  else
  if(command.GetType() == ctGET) //получить статистику
  {

    if(!argsCount) // нет аргументов
    {
      if(wantAnswer) PublishSingleton = PARAMS_MISSED;
    }
    else
    {
        String t = command.GetArg(0);

        if(t == FREERAM_COMMAND) // запросили данные о свободной памяти
        {
         PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = FREERAM_COMMAND; 
            PublishSingleton << PARAM_DELIMITER << freeRam();
          }
        }
        else
        if(t == UPTIME_COMMAND) // запросили данные об аптайме
        {
          PublishSingleton.Flags.Status = true;
          if(wantAnswer) 
          {
            PublishSingleton = UPTIME_COMMAND; 
            PublishSingleton << PARAM_DELIMITER <<  (unsigned long) uptime/1000;
          }
        }
     #ifdef USE_DS3231_REALTIME_CLOCK   
        else if(t == CURDATETIME_COMMAND)
        {
           RealtimeClock rtc = MainController->GetClock();
           RTCTime tm = rtc.getTime();
           if(wantAnswer) 
           {
             PublishSingleton = rtc.getDayOfWeekStr(tm);
             PublishSingleton << F(" ") << (rtc.getDateStr(tm)) << F(" ");
             // Глюк компилятора? Если поставить все команды в одну строку - вместо времени ещё раз выведется дата! 
             PublishSingleton << rtc.getTimeStr(tm);
           }
          PublishSingleton.Flags.Status = true;
        }
      #endif  
        else
        {
          // неизвестная команда
        } // else

    }// have arguments
    
  } // if
 
 // отвечаем на команду
    MainController->Publish(this,command);
    
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------


