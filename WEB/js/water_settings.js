//-----------------------------------------------------------------------------------------------------
// классы для работы с настройками полива
//-----------------------------------------------------------------------------------------------------
SHORT_WEEKDAYS = ['Пн','Вт','Ср','Чт','Пт','Сб','Вс'];
LONG_WEEKDAYS = ['Понедельник','Вторник','Среда','Четверг','Пятница','Суббота','Воскресенье'];
//-----------------------------------------------------------------------------------------------------
var WaterChannelSettings = function(wateringDays,wateringTime,startTime, sensorIndex, stopBorder, startBorder)
{
  this.Parent = null;
  
  this.WateringDays = parseInt(wateringDays);
  this.WateringTime = parseInt(wateringTime);
  this.StartTime = parseInt(startTime);
  this.WateringSensorIndex = parseInt(sensorIndex);
  this.WateringStopBorder = parseInt(stopBorder);
  this.WateringStartBorder = parseInt(startBorder);
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
WaterChannelSettings.prototype.getWateringDaysString = function(shortFormat)
{
  var arr = shortFormat ? SHORT_WEEKDAYS : LONG_WEEKDAYS;
  var result = '';
  for(var i=0;i<7;i++)
  {
    if(BitIsSet(this.WateringDays,i))
    {
      if(result.length)
        result += ',';
        
      result += arr[i];
    }
  } // for
  
  if(result == '')
    result = '&lt;не задано&gt;'
  
  return result;
}
//-----------------------------------------------------------------------------------------------------
var WateringSettings = function(wateringOption, wateringDays, wateringTime, startTime, turnOnPump, sensorIndex, stopBorder, toAutoAAM, startBorder)
{
  this.WateringOption = parseInt(wateringOption);
  this.WateringDays = parseInt(wateringDays);
  this.WateringTime = parseInt(wateringTime);
  this.StartTime = parseInt(startTime);
  this.TurnOnPump = parseInt(turnOnPump);
  this.WateringSensorIndex = parseInt(sensorIndex);
  this.WateringStopBorder = parseInt(stopBorder);
  this.SwitchToAutoAfterMidnight = parseInt(toAutoAAM);
  this.WateringStartBorder = parseInt(startBorder);

  this.Channels = new Array();
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
WateringSettings.prototype.Add = function(channel)
{
  channel.Parent = this;
  this.Channels.push(channel);
}
//-----------------------------------------------------------------------------------------------------
WateringSettings.prototype.Clear = function()
{
  this.Channels = new Array();
}
//-----------------------------------------------------------------------------------------------------
