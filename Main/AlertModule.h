#ifndef _ALERT_MODULE_H
#define _ALERT_MODULE_H
//--------------------------------------------------------------------------------------------------------------------------------------
#include "AbstractModule.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  rtUnknown,
  rtTemp, // за температурой следим
  rtLuminosity, // за освещенностью следим
  rtHumidity, // за влажностью следим
  rtPinState, // следим за статусом пина
  rtSoilMoisture, // следим за влажностью почвы
  rtPH // следим за pH
  
} RuleTarget; // за чем следит правило
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  tsPassed, // следим четко за переданной
  tsOpenTemperature, // берём температуру открытия из настроек
  tsCloseTemperature // берём температуру закрытия из настроек
  
} RuleDataSource; // откуда брать настройку для слежения?
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  roLessThan, // < меньше чем
  roGreaterThan, // > больше чем
  roLessOrEqual, // <=
  roGreaterOrEqual // >=
  
} RuleOperand; // какой операнд использовать
//--------------------------------------------------------------------------------------------------------------------------------------
class AlertModule; // forward declaration
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<uint8_t> LinkedRulesToIdxVector;
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  uint8_t SensorIndex; // индекс датчика, за которым следим
  uint8_t RuleNameIndex; // индекс имени правила у родителя // 64 правила, с запасом
  
  uint8_t Operand : 2; // операнд, которым проверяем
  uint8_t IsAlarm : 1; // это правило - тревожное?
  uint8_t reserved : 5; // выравнивание по границе байта

  uint8_t DataSource  : 2; // источник, с которого получаем установку значения для правила
  uint8_t Enabled : 1;
  uint8_t CanWork : 1;
  uint8_t Target  : 4; // за чем следит правило
  
  uint8_t TargetModuleNameIndex : 4; // индекс имени модуля, для которого посылается команда
  uint8_t LinkedModuleNameIndex : 4; // индекс имени модуля, с которым мы связаны

  uint8_t DayMask; // маска дней недели, когда работает правило
  uint16_t StartTime; // начало работы (минут от начала суток)
  uint16_t WorkTime; // продолжительность работы, минут
  int32_t DataAlert; // настройка, за которой следим (4 байта)

  uint8_t TargetCommandType; // тип команды на выполнение
  uint8_t TargetCommandParam; // дополнительный параметр для команды
      
} RuleSettings; // структура настроек правила
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  moduleState = 1, // STATE - получаем температуру, открываем/закрываем окна
  modulePin, // PIN - выставляем уровни на пинах
  moduleLight, // LIGHT - получаем освещённость, включаем/выключаем досветку
  moduleCompositeCommands, // CC - выполняем составные команды
  moduleHumidity, // HUMIDITY - получаем влажность
  moduleDelta, // DELTA - получаем дельты показаний с датчиков
  moduleSoil, // SOIL - получаем влажность почвы
  modulePH, // PH - получаем значение pH
  moduleZero, // модуль "0"
  moduleScene, // модуль сценариев
  
} RuleKnownModules; // список модулей, с которых мы можем получать показания и для которых можем выполнять команды
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  commandUnparsed,              // команду не разобрали как известную, тупо копируем её всю
  commandOpenAllWindows,        // открыть все окна
  commandCloseAllWindows,       // закрыть все окна
  commandLightOn,               // включить досветку
  commandLightOff,              // выключить досветку
  commandExecCompositeCommand,   // выполнить составную команду
  commandSetOnePinHigh,         // выставить на одном пине высокий уровень
  commandSetOnePinLow,           // выставить на одном пине низкий уровень 
  commandExecScene,              // выполнить сценарий
  commandStopScene,              // остановить сценарий
  
} RuleKnownCommands; // известные правилу команды, которые оно может перевести в краткую форму
//--------------------------------------------------------------------------------------------------------------------------------------
#define RULE_SETT_HEADER1 0x21
#define RULE_SETT_HEADER2 0x17
//--------------------------------------------------------------------------------------------------------------------------------------
class AlertRule
{
  private:

    RuleSettings Settings; // наши настройки

    char* rawCommand; // сырая команда, если Settings.TargetCommandType == commandUnparsed, то вся команда будет здесь    
    AbstractModule* linkedModule; // модуль, показания которого надо отслеживать
    LinkedRulesToIdxVector linkedRulesIndices; // привязка имён связанных правил к их индексу у родителя
    const char* GetKnownModuleName(uint8_t type);
    
  public:
    AlertRule();
    ~AlertRule();

    bool IsAlarm() {return Settings.IsAlarm == 1; }

    bool GetEnabled() {return  Settings.Enabled; }
    void SetEnabled(bool e)  { Settings.Enabled = e ? 1 : 0; }
    
    const char* GetName();
    AbstractModule* GetModule() {return linkedModule;}
    
    bool Construct(AbstractModule* linkedModule, const Command& command);
    
    const char* GetTargetCommand();
    bool HasTargetCommand();
    
    const char* GetAlertRule();

    const char* GetTargetCommandModuleName();
    const char* GetLinkedModuleName();
    uint8_t GetKnownModuleID(const char* moduleName);

    size_t GetLinkedRulesCount();
    const char* GetLinkedRuleName(uint8_t idx);

    uint8_t Save(uint16_t writeAddr); // сохраняем себя в EEPROM, возвращаем кол-во записанных байт
    uint8_t Load(uint16_t readAddr); // читаем себя из EEPROM, возвращаем кол-во прочитанных байт

    void Update(uint16_t dt
  #ifdef USE_DS3231_REALTIME_CLOCK 
     ,uint8_t currentHour // текущий час
     ,uint8_t currentMinute // текущая минута
     ,uint8_t currentDOW // текущий день недели
  #endif
 );

    bool HasAlert(); // проверяем, есть ли алерт?
};
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<AlertRule*> RulesVector;
typedef Vector<char*> NamesVector;
//--------------------------------------------------------------------------------------------------------------------------------------
class AlertModule : public AbstractModule
{
  private:
  
    RulesVector lastIterationRaisedRules; // список правил, сработавших на текущей итерации
    bool IsRuleRaisedOnLastIteration(AlertRule* rule);

    NamesVector paramsArray; // всякие общие имена храним здесь
    void ClearParams();

    unsigned long lastUpdateCall;

    uint8_t rulesCnt;
    AlertRule* alertRules[MAX_ALERT_RULES];
    void InitRules();
    bool AddRule(AbstractModule* m, const Command& c);

    void SolveConflicts(RulesVector& raisedAlerts,RulesVector& workRules);
    bool CanWorkWithRule(RulesVector& checkedRules, AlertRule* rule,RulesVector& raisedAlerts);
    AlertRule* GetLinkedRule(const char* linkedRuleName,RulesVector& raisedAlerts);

    void LoadRules();
    void SaveRules();
    
  public:
    AlertModule();

    size_t AddParam(char* nm, bool& added); // добавляем строку в общий массив
    char* GetParam(size_t idx); // возвращает строку из массива по индексу

    bool ExecCommand(const Command& command, bool wantAnswer);
    void Setup();
    void Update(uint16_t dt);

};
//--------------------------------------------------------------------------------------------------------------------------------------
extern AlertModule* RulesDispatcher;
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
