#include "SceneModule.h"
#include "ModuleController.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
#include "InteropStream.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#define SCENES_FOLDER F("SCENES/")
#define SCENE_SETTINGS_FILENAME F("SCENE.TXT")
#define SCENE_INIT_FILENAME F("INIT.TXT")
#define SCENE_STOP_FILENAME F("STOP.TXT")
#define SCENE_STEPS_FOLDER F("STEPS/")
//--------------------------------------------------------------------------------------------------------------------------------------
SceneModule* Scenes = NULL;
//--------------------------------------------------------------------------------------------------------------------------------------
SceneModule::SceneModule() : AbstractModule("SCN")
{
  Scenes = this;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::Setup()
{
  // настройка модуля тут
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::Update(uint16_t dt)
{ 
  UNUSED(dt);
  // обновление модуля тут

}
//--------------------------------------------------------------------------------------------------------------------------------------
SceneSettings SceneModule::GetSceneSettings(uint16_t sceneNumber)
{
    SceneSettings result;
    result.isSceneExists = false;

  if(!MainController->HasSDCard())
    return result;  

  String dirName = SCENES_FOLDER;
  dirName += sceneNumber;
  dirName += '/';
  dirName += SCENE_SETTINGS_FILENAME;
  
  const char* dirP = dirName.c_str();
  
  if(!SDFat.exists(dirP))
    return result;

  SdFile f;
  if(f.open(dirP,FILE_READ))
  {
    result.isSceneExists = true;
    String sceneName, mode;
    FileUtils::readLine(f,sceneName);
    FileUtils::readLine(f,mode);
    result.sceneName = sceneName;
    result.continuousMode = mode.toInt() > 0 ? 1 : 0;
    f.close();
  }

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool SceneModule::IsSceneExists(uint16_t sceneNumber)
{
  if(!MainController->HasSDCard())
    return false;

  String dirName = SCENES_FOLDER;
  dirName += sceneNumber;
  dirName += '/';
  dirName += SCENE_SETTINGS_FILENAME;
  
  const char* dirP = dirName.c_str(); 
  return SDFat.exists(dirP);
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::ExecuteScene(uint16_t sceneNumber)
{
  if(IsSceneActive(sceneNumber))
    return;
    
  StopScene(sceneNumber);
  ExecuteSceneFile(sceneNumber,initFile);
  AddSceneToList(sceneNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::StopScene(uint16_t sceneNumber)
{
  if(!IsSceneActive(sceneNumber))
    return;
  
  RemoveSceneFromList(sceneNumber);
  ExecuteSceneFile(sceneNumber,stopFile);  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::AddSceneToList(uint16_t sceneNumber)
{
  RemoveSceneFromList(sceneNumber);
  SceneSettings ss = GetSceneSettings(sceneNumber);
  if(!ss.isSceneExists)
  {
    #ifdef SCENE_DEBUG
    Serial.println(F("Scene not found!"));
    #endif
    return;
  }

  #ifdef SCENE_DEBUG
  Serial.println(F("Add scene to list and execute first step..."));
  #endif

  SceneWork sw;
  sw.sceneNumber = sceneNumber;
  sw.continuousMode = ss.continuousMode > 0;
  sw.stepNumber = 0xFFFF;
  list.push_back(sw);

  ExecuteStep(sceneNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::NextSceneStep(void* param)
{
  Scenes->ExecuteStep(*((uint16_t*) &param));
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::ExecuteStep(uint16_t sceneNumber)
{
  size_t sceneIndex = 0;
  bool sceneFound = false;
  for(size_t i=0;i<list.size();i++)
  {
    if(list[i].sceneNumber == sceneNumber)
    {
      sceneFound = true;
      sceneIndex = i;
      break;
    }
  }

  if(!sceneFound)
  {
    #ifdef SCENE_DEBUG
    Serial.println(F("Scene not found in list!"));
    #endif
    return;
  }

  // исполняем текущий шаг, добавляем событие, прибавляем номер шага.
  list[sceneIndex].stepNumber++;
  // если файла текущего шага нет - удаляем сцену, если она однократная или шаг был - 0.
  // иначе - сбрасываем шаг на 0.
  String dirName = SCENES_FOLDER;
  dirName += sceneNumber;
  dirName += '/';
  dirName += SCENE_STEPS_FOLDER;
  dirName += list[sceneIndex].stepNumber;
  dirName += F(".TXT");

  #ifdef SCENE_DEBUG
  Serial.print(F("CHECK STEP EXISTS - "));
  Serial.println(dirName);
  #endif

  const char* dirP = dirName.c_str(); 
  if(!SDFat.exists(dirP))
  {
    #ifdef SCENE_DEBUG
    Serial.println(F("Step NOT EXISTS!"));
    #endif
    
    if(list[sceneIndex].stepNumber == 0 || !list[sceneIndex].continuousMode)
    {
      #ifdef SCENE_DEBUG
      Serial.println(F("SCENE DONE, remove from list!"));
      #endif
      ExecuteSceneFile(sceneNumber, stopFile);
      RemoveSceneFromList(sceneNumber);
    }
    else
    {
      #ifdef SCENE_DEBUG
      Serial.println(F("Wrap to scene beginning!"));
      #endif
      list[sceneIndex].stepNumber = 0xFFFF;
      ExecuteStep(sceneNumber);
    }
    return;
  }

  // файл шага существует
  #ifdef SCENE_DEBUG
  Serial.print(F("EXECUTE STEP - "));
  Serial.println(dirName);
  #endif

  SdFile f;
  if(f.open(dirP,FILE_READ))
  {
    String duration, command;
    FileUtils::readLine(f,duration);
    FileUtils::readLine(f,command);
    ExecuteSceneCommand(command);

    uint32_t dur = duration.toInt();
    dur *= 1000;    
    f.close();

    #ifdef SCENE_DEBUG
    Serial.print(F("Step duration, s = "));
    Serial.println(duration);
    #endif

    CoreDelayedEvent.raise(dur, NextSceneStep, (void*) sceneNumber);
    
  }
  else
  {
    #ifdef SCENE_DEBUG
    Serial.println(F("Unable to open step file, skip it!"));
    #endif
    
    ExecuteStep(sceneNumber);
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::RemoveSceneFromList(uint16_t sceneNumber)
{
  ActiveScenesList lst;
  for(size_t i=0;i<list.size();i++)
  {
    if(list[i].sceneNumber != sceneNumber)
    {
      lst.push_back(list[i]);
    }
  }

    list = lst;
    // удаляем ждущие события
    CoreDelayedEvent.removeByParam(NextSceneStep, (void*) sceneNumber);
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool SceneModule::IsSceneActive(uint16_t sceneNumber)
{
  for(size_t i=0;i<list.size();i++)
  {
    if(list[i].sceneNumber == sceneNumber)
      return true;
  }
  return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::ExecuteSceneCommand(String& line)
{
  #ifdef SCENE_DEBUG
    Serial.println(line);
  #endif
    
    COMMAND_TYPE ct = ctSET;
    if(line.startsWith(F("CTGET=")))
      ct = ctGET;
    line.remove(0,6); // remove CTSET= or CTGET= prefix
    
    ModuleInterop.QueryCommand(ct,line,false); 
    yield();  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void SceneModule::ExecuteSceneFile(uint16_t sceneNumber, SceneFile whichFile)
{
  String dirName = SCENES_FOLDER;
  dirName += sceneNumber;
  dirName += '/';

  if(whichFile == initFile)
    dirName += SCENE_INIT_FILENAME;
  else
  if(whichFile == stopFile)
    dirName += SCENE_STOP_FILENAME;

  const char* dirP = dirName.c_str(); 
  if(SDFat.exists(dirP))
  {
    #ifdef SCENE_DEBUG
    Serial.print(F("Execute scene file - "));
    Serial.println(dirName);
    #endif

    SdFile f;
    if(f.open(dirP,FILE_READ))
    {
      String line;
      // тут читаем последовательно все строчки
      uint32_t sz = f.fileSize();
      do
      {
        line = "";
        FileUtils::readLine(f,line);
        yield();
        
        if(line.length())
        {
          ExecuteSceneCommand(line);
        }
        
      } while(f.curPosition() < sz);

      #ifdef SCENE_DEBUG
      Serial.println(F("EXECUTED!"));
      #endif
      
      f.close();
    }
    
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint16_t SceneModule::GetScenesCount()
{
  uint16_t result = 0;
  
  if(!MainController->HasSDCard())
    return result;

  String dirName = SCENES_FOLDER;
  const char* dirP = dirName.c_str();
  
  if(!SDFat.exists(dirP))
    return result;

  SdFile root;
  if(!root.open(dirP,O_READ))
    return result;
        
  root.rewind();

  SdFile entry;
  while(entry.openNext(&root,O_READ))
  {
    yield();
    if(entry.isDir())
    {
      result++;
    }
    entry.close();
  } // while


  root.close();
  return result;  
      
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool  SceneModule::ExecCommand(const Command& command, bool wantAnswer)
{
  UNUSED(wantAnswer);

  size_t argsCount = command.GetArgsCount();
  
  if(command.GetType() == ctGET)
  {
    if(!argsCount) // запросили кол-во сцен, CTGET=SCN
    {
      PublishSingleton.Flags.Status = true;
      PublishSingleton = "";
      PublishSingleton << GetScenesCount();
        
    } // !argsCount
    else
    {
      String cmd = command.GetArg(0);
      if(cmd == F("SETT")) // получить настройки сцены, CTGET=SCN|SETT|sceneNumber
      {
        if(argsCount > 1)
        {
          SceneSettings ss = GetSceneSettings(atoi(command.GetArg(1)));
          if(ss.isSceneExists)
          {
            PublishSingleton.Flags.Status = true;
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER << command.GetArg(1) << PARAM_DELIMITER << ss.continuousMode << PARAM_DELIMITER;
            for(size_t i=0;i< ss.sceneName.length();i++)
            {
              PublishSingleton << WorkStatus::ToHex(ss.sceneName[i]);
            }
          }
          else
          {
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << command.GetArg(1);
            PublishSingleton << PARAM_DELIMITER;
            PublishSingleton << F("NO_SCENE");
          }
        }
        else
        {
          PublishSingleton = PARAMS_MISSED;
        }
        
      } // if(cmd == F("SETT"))
      else
      {
        PublishSingleton = F("UNKNOWN_COMMAND");
      }      
      
    } // else has argsCount

  }
  else
  if(command.GetType() == ctSET)
  {
    if(argsCount < 2)
    {
      PublishSingleton = PARAMS_MISSED; 
    }
    else
    {
      String cmd = command.GetArg(0);
      if(cmd == F("EXEC")) // выполнить сценарий, CTSET=SCN|EXEC|sceneNumber
      {
          uint16_t sceneNumber = atoi(command.GetArg(1));
          if(!IsSceneExists(sceneNumber))
          {
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER << sceneNumber << PARAM_DELIMITER << F("NO_SCENE");
          }
          else
          {
            ExecuteScene(sceneNumber);
            
            PublishSingleton.Flags.Status = true;
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER << sceneNumber << PARAM_DELIMITER << F("DONE");
          }

      } // cmd == F("EXEC")
      else
      if(cmd == F("STOP")) // остановить сценарий, CTSET=SCN|STOP|sceneNumber
      {
          uint16_t sceneNumber = atoi(command.GetArg(1));
          if(!IsSceneExists(sceneNumber))
          {
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER << sceneNumber << PARAM_DELIMITER << F("NO_SCENE");            
          }
          else
          {
            StopScene(sceneNumber);
  
            PublishSingleton.Flags.Status = true;
            PublishSingleton = cmd;
            PublishSingleton << PARAM_DELIMITER << sceneNumber << PARAM_DELIMITER << F("DONE");
          }

      } // cmd == F("STOP")
      else
      {
        PublishSingleton = F("UNKNOWN_COMMAND");
      }
    } // else args >= 2
  }
  // отвечаем на команду
  MainController->Publish(this,command);
   
  return PublishSingleton.Flags.Status;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SCENE_MODULE

