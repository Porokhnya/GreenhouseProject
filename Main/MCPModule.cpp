#include "MCPModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
MCPModule::MCPModule() : AbstractModule("MCP")
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void MCPModule::Setup()
{
  // настройка модуля тут
}
//--------------------------------------------------------------------------------------------------------------------------------------
void MCPModule::Update(uint16_t dt)
{ 
  UNUSED(dt);
  // обновление модуля тут

}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  MCPModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);

  size_t argsCount = command.GetArgsCount();
  
  if(command.GetType() == ctGET)
  {
    if(argsCount < 3)
    {
      // params missed
      PublishSingleton = PARAMS_MISSED;
    }
    else
    {
      String cmd = command.GetArg(0);
      if(cmd == F("SPI")) // CTGET=MCP|SPI|mcpNumber|mcpChannel
      {
        PublishSingleton.Flags.Status = true;
        PublishSingleton = cmd;

        byte mcpNumber = atoi(command.GetArg(1));
        byte mcpChannel = atoi(command.GetArg(2));

        int status = -1;

        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          status = WORK_STATUS.MCP_SPI_PinRead(mcpNumber,mcpChannel);
        #endif

        PublishSingleton << PARAM_DELIMITER << mcpNumber << PARAM_DELIMITER << mcpChannel << PARAM_DELIMITER;
        if(status != -1)
        {
          PublishSingleton << (status == HIGH ? STATE_ON : STATE_OFF);
        }
        else
          PublishSingleton << status;
      } // if(cmd == F("SPI"))
      else
      if(cmd == F("I2C")) // CTGET=MCP|I2C|mcpNumber|mcpChannel
      {
        PublishSingleton.Flags.Status = true;
        PublishSingleton = cmd;

        byte mcpNumber = atoi(command.GetArg(1));
        byte mcpChannel = atoi(command.GetArg(2));

        int status = -1;

        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          status = WORK_STATUS.MCP_I2C_PinRead(mcpNumber,mcpChannel);
        #endif

        PublishSingleton << PARAM_DELIMITER << mcpNumber << PARAM_DELIMITER << mcpChannel << PARAM_DELIMITER;
        if(status != -1)
        {
          PublishSingleton << (status == HIGH ? STATE_ON : STATE_OFF);
        }
        else
          PublishSingleton << status;        
      } // if(cmd == F("I2C"))
    }
  }
  else
  if(command.GetType() == ctSET)
  {
    if(argsCount < 5)
    {
      PublishSingleton = PARAMS_MISSED;            
    }
    else
    {
        String cmd = command.GetArg(0);
        String operation = command.GetArg(1);
        byte mcpNumber = atoi(command.GetArg(2));
        byte mcpChannel = atoi(command.GetArg(3));
        String levelOrMode = command.GetArg(4);
        byte level = (levelOrMode == STATE_ON || levelOrMode == STATE_ON_ALT) ? HIGH : LOW;

        if(cmd == F("SPI")) // CTSET=MCP|SPI|operation|mcpNumber|mcpChannel|levelOrMode, where "operation" == MODE or WRITE
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = cmd;
          PublishSingleton << PARAM_DELIMITER << operation;

          if(operation == F("MODE")) // CTSET=MCP|SPI|MODE|mcpNumber|mcpChannel|pinMode, for example  CTSET=MCP|SPI|MODE|0|7|OUT, CTSET=MCP|SPI|MODE|1|2|IN
          {
            #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
            
            byte mode = INPUT;
            if(levelOrMode == F("OUT"))
              mode = OUTPUT;
              
              WORK_STATUS.MCP_SPI_PinMode(mcpNumber,mcpChannel,mode);
              
            #endif            
          }
          else // CTSET=MCP|SPI|WRITE|mcpNumber|mcpChannel|level
          {
            #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
              WORK_STATUS.MCP_SPI_PinWrite(mcpNumber,mcpChannel,level);
            #endif
          }

          PublishSingleton << PARAM_DELIMITER << mcpNumber << PARAM_DELIMITER << mcpChannel << PARAM_DELIMITER << levelOrMode << PARAM_DELIMITER << REG_SUCC;
                  
        } // if(cmd == F("SPI"))
        else
       if(cmd == F("I2C")) // CTSET=MCP|I2C|operation|mcpNumber|mcpChannel|levelOrMode, where "operation" == MODE or WRITE
        {
          PublishSingleton.Flags.Status = true;
          PublishSingleton = cmd;
          PublishSingleton << PARAM_DELIMITER << operation;

          if(operation == F("MODE")) // CTSET=MCP|I2C|MODE|mcpNumber|mcpChannel|pinMode, for example  CTSET=MCP|I2C|MODE|0|7|OUT, CTSET=MCP|I2C|MODE|1|2|IN
          {
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
            
            byte mode = INPUT;
            if(levelOrMode == F("OUT"))
              mode = OUTPUT;
              
              WORK_STATUS.MCP_I2C_PinMode(mcpNumber,mcpChannel,mode);
            #endif            
          }
          else // CTSET=MCP|I2C|WRITE|mcpNumber|mcpChannel|level
          {
            #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
              WORK_STATUS.MCP_I2C_PinWrite(mcpNumber,mcpChannel,level);
            #endif
          }

          PublishSingleton << PARAM_DELIMITER << mcpNumber << PARAM_DELIMITER << mcpChannel << PARAM_DELIMITER << levelOrMode << PARAM_DELIMITER << REG_SUCC;
                  
        } // if(cmd == F("I2C"))   
      
    }
  }
  // отвечаем на команду
  MainController->Publish(this,command);
   
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------


