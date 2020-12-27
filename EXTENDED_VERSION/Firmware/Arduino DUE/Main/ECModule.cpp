#include "ECModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::Setup()
{
  // добавляем датчик EC в систему
  State.AddState(StateEC,0);

  // пока делаем ему фейковые показания
  uint16_t fakeEC = 1234;
  State.UpdateState(StateEC,0,(void*)&fakeEC);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void ECModule::Update()
{ 
  // обновление модуля тут

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  ECModule::ExecCommand(const Command& command, bool wantAnswer)
{
 UNUSED(wantAnswer);
  PublishSingleton = PARAMS_MISSED;
    
  
  uint8_t argsCount = command.GetArgsCount();
  if(argsCount < 1)
  {
    if(command.GetType() == ctGET)
    {
    // попросили отдать все показания с датчиков
    
      PublishSingleton.Flags.Status = true;
      if(wantAnswer) 
      {
         PublishSingleton = "";

         uint8_t _cnt = State.GetStateCount(StateEC);

         PublishSingleton << _cnt;
         
         for(uint8_t i=0;i<_cnt;i++)
         {
            OneState* os = State.GetStateByOrder(StateEC,i);
            if(os)
            {
              ECPair lp = *os;

              PublishSingleton << PARAM_DELIMITER;
              PublishSingleton << lp.Current;
                              
            } // if(os)
         } // for
     
      }

    } // if(command.GetType() == ctGET)
   
    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
  } // if(argsCount < 1)
  
  if(command.GetType() == ctGET)
  {
      String which = command.GetArg(0);
      /*
      if(which == F("SETTINGS")) // CTGET=CO2|SETTINGS, returns OK=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {  
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        << PARAM_DELIMITER << settings.weekdays
        << PARAM_DELIMITER << settings.startTime
        << PARAM_DELIMITER << settings.endTime
        << PARAM_DELIMITER << settings.ppm
        << PARAM_DELIMITER << settings.histeresis
        ;
      } // if(which == F("SETTINGS"))
      else
      if(which == F("ACTIVE")) // CTGET=CO2|ACTIVE
      {
        PublishSingleton.Flags.Status = true;
        PublishSingleton = which;
        PublishSingleton << PARAM_DELIMITER << settings.active
        ;
        
      } // if(which == F("ACTIVE"))
      else // возможно, запросили показания датчика, CTGET=CO2|0, CTGET=CO2|1 и т.п.
      */
      {
         uint8_t idx = which.toInt();
          uint8_t _cnt = State.GetStateCount(StateEC);
          
          if(idx >= _cnt)
          {
            // плохой индекс
            if(wantAnswer)
            {
              PublishSingleton = which;
              PublishSingleton << PARAM_DELIMITER << NOT_SUPPORTED;
            }
          } // плохой индекс
          else
          {
             if(wantAnswer)
             {
              PublishSingleton = which;
             }
              
             OneState* stateEC = State.GetStateByOrder(StateEC,idx);
             if(stateEC)
             {
                PublishSingleton.Flags.Status = true;
                ECPair co2p = *stateEC;
                
                if(wantAnswer)
                {
                    PublishSingleton << PARAM_DELIMITER << (co2p.Current);
                }
             } // if
            
          } // else нормальный индекс        
        
      } // else
  } // ctGET
  else
  if(command.GetType() == ctSET)
  {
    /*
      String which = command.GetArg(0);

      if(which == F("SETTINGS")) // CTSET=CO2|SETTINGS|active|weekdays|startTime|endTime|ppm|histeresis
      {
          if(argsCount > 6)
          {
            settings.active = atoi(command.GetArg(1));
            settings.weekdays = atoi(command.GetArg(2));
            settings.startTime = atol(command.GetArg(3));
            settings.endTime = atol(command.GetArg(4));
            settings.ppm = atoi(command.GetArg(5));
            settings.histeresis = atoi(command.GetArg(6));

            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << REG_SUCC;
          }
      } // if(which == F("SETTINGS")) 
      else
      if(which == F("ACTIVE")) // CTSET=CO2|ACTIVE|active flag
      {
          if(argsCount > 1)
          {
            settings.active = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.active;
          }
      } // if(which == F("ACTIVE"))
       else
      if(which == F("PPM")) // CTSET=CO2|PPM|ppm value
      {
          if(argsCount > 1)
          {
            settings.ppm = atoi(command.GetArg(1));
            MainController->GetSettings()->SetCO2Settings(settings);
            ReloadSettings();
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = which;
            PublishSingleton << PARAM_DELIMITER << settings.ppm;
          }
      } // if(which == F("PPM"))
     */ 
  } // ctSET

    MainController->Publish(this,command);
    return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------


