//-----------------------------------------------------------------------------------------------------
// Классы для работы с правилами
//-----------------------------------------------------------------------------------------------------
var MAX_RULES = 30; // максимальное кол-во правил
RULE_TARGETS = {'_': 'нет слежения', 'TEMP': 'температурой', 'LIGHT' : 'освещенностью', 'HUMIDITY': 'влажностью', 'PIN': 'пином', 'SOIL': 'влажностью почвы', 'PH' : 'значением pH'};
RULE_COMMANDS = {
  '.+STATE\\|WINDOW\\|(.*)\\|OPEN' : [0, 'открываем окна']
, '.+STATE\\|WINDOW\\|(.*)\\|CLOSE': [1, 'закрываем окна']
, '.+LIGHT\\|ON' : [2, 'включаем досветку']
, '.+LIGHT\\|OFF' : [3, 'выключаем досветку']
, '.+PIN\\|(.*)\\|ON' : [4, 'включаем пины']
, '.+PIN\\|(.*)\\|OFF' : [5, 'выключаем пины']
, '.+CC\\|EXEC\\|(.*)' : [6, 'выполняем составную команду']
, '.+SCN\\|EXEC\\|(.*)' : [7, 'запускаем сценарий']
, '.+SCN\\|STOP\\|(.*)' : [8, 'останавливаем сценарий']
, '.+MCP\\|SPI\\|WRITE\\|(.*)\\|(.*)\\|ON' : [9, 'включаем канал MCP23S17']
, '.+MCP\\|SPI\\|WRITE\\|(.*)\\|(.*)\\|OFF' : [10, 'выключаем канал MCP23S17']
};

RULE_BUILD_COMMANDS = [
  'CTSET=STATE|WINDOW|{0}|OPEN'
, 'CTSET=STATE|WINDOW|{0}|CLOSE'
, 'CTSET=LIGHT|ON'
, 'CTSET=LIGHT|OFF'
, 'CTSET=PIN|{0}|ON'
, 'CTSET=PIN|{0}|OFF'
, 'CTSET=CC|EXEC|{0}'
, 'CTSET=SCN|EXEC|{0}'
, 'CTSET=SCN|STOP|{0}'
, 'CTSET=MCP|SPI|WRITE|{0}|{1}|ON'
, 'CTSET=MCP|SPI|WRITE|{0}|{1}|OFF'

];
//-----------------------------------------------------------------------------------------------------
// конструктор нового правила
var AlertRule = function()
{
  return this;
}
//-----------------------------------------------------------------------------------------------------
// конструирует правило из параметров, полученных с контроллера
AlertRule.prototype.Construct = function(params)
{
  var startIdx = 3;
  this.Name = params[startIdx++];
  this.ModuleName = params[startIdx++];
  this.Target = params[startIdx++];
  this.SensorIndex = parseInt(params[startIdx++]);
  this.Operand = params[startIdx++];
  this.AlertCondition = params[startIdx++];
  this.StartTime = parseInt(params[startIdx++]);
  this.WorkTime = parseInt(params[startIdx++]);
  this.DayMask = parseInt(params[startIdx++]);
  this.LinkedRules = params[startIdx++].split(',');
  this.IsAlarm = parseInt(params[startIdx++]);
  
  this.TargetCommand = '';
  
  for(var i=startIdx;i<params.length;i++)
  {
    if(this.TargetCommand != '')
      this.TargetCommand += '|';
      
    this.TargetCommand += params[i];
  }
  
  
}
//-----------------------------------------------------------------------------------------------------
// возвращает строку с правилом
AlertRule.prototype.getAlertRule = function()
{

  var result = '' + this.Name + '|' + this.ModuleName + '|' + this.Target + '|' + this.SensorIndex +
  '|' + this.Operand + '|' + this.AlertCondition + '|' + this.StartTime + '|' + this.WorkTime + '|' +
  this.DayMask + '|' + this.LinkedRules.join(',') + '|' + this.IsAlarm + '|' + this.TargetCommand;
  
  return result;
}
//-----------------------------------------------------------------------------------------------------
AlertRule.prototype.getTargetDescription = function()
{
  return RULE_TARGETS[this.Target];
}
//-----------------------------------------------------------------------------------------------------
AlertRule.prototype.getTargetCommandIndex = function()
{
  for(var propName in RULE_COMMANDS)
  {
    
    var propVal = RULE_COMMANDS[propName];

    var reg = new RegExp(propName,'i');

    if(reg.test(this.TargetCommand))
    {
      return propVal[0];
    }
  }
  
  return this.TargetCommand;
}
//-----------------------------------------------------------------------------------------------------
AlertRule.prototype.getAdditionalParam = function()
{
  for(var propName in RULE_COMMANDS)
  {
    
    var propVal = RULE_COMMANDS[propName];

    var reg = new RegExp(propName,'i');

    if(reg.test(this.TargetCommand))
    {
      var res = reg.exec(this.TargetCommand);
      if(res.length > 1)
        return res[1];
    }
  }
  
  return '';
}
//-----------------------------------------------------------------------------------------------------
AlertRule.prototype.getAdditionalParam2 = function()
{
  for(var propName in RULE_COMMANDS)
  {
    
    var propVal = RULE_COMMANDS[propName];

    var reg = new RegExp(propName,'i');

    if(reg.test(this.TargetCommand))
    {
      var res = reg.exec(this.TargetCommand);
      if(res.length > 2)
        return res[2];
    }
  }
  
  return '';
}
//-----------------------------------------------------------------------------------------------------
AlertRule.prototype.getTargetCommandDescription = function()
{
  for(var propName in RULE_COMMANDS)
  {
    
    var propVal = RULE_COMMANDS[propName];

    var reg = new RegExp(propName,'i');

    if(reg.test(this.TargetCommand))
    {
      return propVal[1];
    }
  }
  
  return this.TargetCommand;
}
//-----------------------------------------------------------------------------------------------------
// список правил
//-----------------------------------------------------------------------------------------------------
var RulesList = function()
{
  this.Rules = new Array();
  return this;
}
//-----------------------------------------------------------------------------------------------------
// добавляет правило в список
RulesList.prototype.Add = function()
{
  var rule = new AlertRule();
  this.Rules.push(rule);
  
  return rule;
}
//-----------------------------------------------------------------------------------------------------
// удаляет все правила
RulesList.prototype.Clear = function()
{
  this.Rules = new Array();
}
//-----------------------------------------------------------------------------------------------------
RulesList.prototype.buildTargetCommand = function(idx,param, param2)
{
  var result = '';
  var pattern = RULE_BUILD_COMMANDS[idx];
    
  result = pattern.replace("{0}",param);
  
  if(param2 != undefined)
  {
	  result = result.replace("{1}",param2);
  }
  return result;
}
