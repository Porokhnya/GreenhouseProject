#include "LCDModule.h"
#include "ModuleController.h"
#include "LCDMenu.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LCD_MODULE
// наше меню
LCDMenu lcdMenu(SCREEN_SCK_PIN, SCREEN_MOSI_PIN, SCREEN_CS_PIN);
// наш энкодер
RotaryEncoder rotaryEncoder(ENCODER_A_PIN,ENCODER_B_PIN,ENCODER_PULSES_PER_CLICK);
#endif
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDModule::Setup()
{
waitInitCounter = 0;
inited = false;
  
#ifdef USE_LCD_MODULE  
  rotaryEncoder.begin(); // инициализируем энкодер

  // инициализируем меню
  lcdMenu.init();
#endif  
 }
//--------------------------------------------------------------------------------------------------------------------------------------
void LCDModule::Update(uint16_t dt)
{ 
  // ждём три секунды до начала обновления, чтобы дать прочухаться всему остальному
  if(!inited)
  {
     waitInitCounter += dt;
     if(waitInitCounter > 3000)
     {
      waitInitCounter = 0;
      inited = true;
     }
     return;
  }
  
#ifdef USE_LCD_MODULE  
  rotaryEncoder.update(); // обновляем энкодер
  lcdMenu.update(dt);

  int ch = rotaryEncoder.getChange();
   if(ch)
   {
    // было движение энкодера
    // выбираем следующее меню LCD
    lcdMenu.selectNextMenu(ch);
   }

    lcdMenu.draw(); // рисуем меню
#else
  UNUSED(dt);    
#endif
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  LCDModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);

  uint8_t argsCnt = command.GetArgsCount();
 
  
  if(command.GetType() == ctSET) // установка свойств
  {

      if(argsCnt < 1)
      {
        PublishSingleton << PARAMS_MISSED;
      }
      else
      {
            String whichCommand = command.GetArg(0);
            
            if(whichCommand == F("DEL")) // удалить все данные датчиков
            {
               PublishSingleton.Flags.Status = true;
               PublishSingleton << whichCommand << PARAM_DELIMITER << REG_SUCC;

               #ifdef USE_LCD_MODULE
                lcdMenu.ClearSDSensors();
               #endif
            } // if(whichCommand == F("DEL"))
            else if(whichCommand == F("ADD"))
            {
              if(argsCnt < 4)
              {
                PublishSingleton << PARAMS_MISSED;
              }
              else
              {
                #ifdef USE_LCD_MODULE
                  byte folder = (byte) atoi(command.GetArg(1));
                  byte sensorIndex = (byte) atoi(command.GetArg(2));

                  const char* encodedCaption = command.GetArg(3);
                  String strCaption;
                  
                  // переводим закодированный текст в UTF-8
                  while(*encodedCaption)
                  {
                    strCaption += (char) WorkStatus::FromHex(encodedCaption);
                    encodedCaption += 2;
                  }

                    lcdMenu.AddSDSensor(folder,sensorIndex,strCaption);
                    
                  #endif

                  PublishSingleton.Flags.Status = true;
                  PublishSingleton << whichCommand << PARAM_DELIMITER << REG_SUCC;
              }
            } // else if(whichCommand == F("ADD"))
      }
    
  } // ctSET
  else
  {
      // чтение свойств
      if(argsCnt < 1)
      {
        PublishSingleton << PARAMS_MISSED;
      }
      else
      {
        // есть параметры
        String whichCommand = command.GetArg(0);
        if(whichCommand == F("T_SETT")) // CTGET=LCD|T_SETT
        {
            // получить данные о кол-ве файлов в каждой директории настроек датчиков экрана.
            // данные идут так: DIR_TEMP|DIR_HUMIDITY|DIR_LUMINOSITY|DIR_SOIL|DIR_PH

            PublishSingleton.Flags.Status = true;
            PublishSingleton << whichCommand;

            for(byte i=DIR_TEMP;i<DIR_DUMMY_LAST_DIR;i++)
            {
              PublishSingleton << PARAM_DELIMITER;
              
              #ifdef USE_LCD_MODULE
                PublishSingleton << lcdMenu.GetFilesCount(i);
              #else
                PublishSingleton << 0;
              #endif
            } // for
          
        } // if(whichCommand == F("T_SETT"))
        else if(whichCommand == F("VIEW")) // CTGET=LCD|VIEW|folder|index
        {
          // запросили просмотр содержимого файла
          if(argsCnt < 3)
          {
            // недостаточно параметров
            PublishSingleton << PARAMS_MISSED;
          }
          else
          {
            byte folder = (byte) atoi(command.GetArg(1));
            byte index = (byte)  atoi(command.GetArg(2));

            PublishSingleton.Flags.Status = true;
            PublishSingleton << whichCommand << PARAM_DELIMITER << folder << PARAM_DELIMITER;

            #ifdef USE_LCD_MODULE
                int sensorIndex = 0;
                String content = lcdMenu.GetFileContent(folder,index,sensorIndex);

                PublishSingleton << sensorIndex << PARAM_DELIMITER;
                
                const char* str = content.c_str();
                while(*str)
                {
                  PublishSingleton << WorkStatus::ToHex(*str++);
                }
             #else
                PublishSingleton << index << PARAM_DELIMITER;
             #endif

            
          }
          
        } // if(whichCommand == F("VIEW"))
      } // else
  } // ctGET

  MainController->Publish(this,command); 

  return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------


