#pragma once
//--------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "TinyVector.h"
#include "DelayedEvents.h"
//--------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool isSceneExists;
  uint8_t continuousMode;
  String sceneName;
  
} SceneSettings;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  initFile,
  stopFile
  
} SceneFile;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint16_t sceneNumber;
  bool continuousMode;
  uint16_t stepNumber;
  
} SceneWork;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<SceneWork> ActiveScenesList;
//--------------------------------------------------------------------------------------------------------------------------------------
class SceneModule : public AbstractModule
{
  private:

    void AddSceneToList(uint16_t sceneNumber);
    void RemoveSceneFromList(uint16_t sceneNumber);

    void ExecuteSceneCommand(String& line);

    void ExecuteStep(uint16_t sceneNumber);
    static void NextSceneStep(void* param);

    ActiveScenesList list;
  
  public:
    SceneModule();

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

    uint16_t GetScenesCount();
    SceneSettings GetSceneSettings(uint16_t sceneNumber);
    bool IsSceneExists(uint16_t sceneNumber);
    void ExecuteSceneFile(uint16_t sceneNumber, SceneFile whichFile);
    void ExecuteScene(uint16_t sceneNumber);
    void StopScene(uint16_t sceneNumber);

    bool IsSceneActive(uint16_t sceneNumber);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern SceneModule* Scenes;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SCENE_MODULE
