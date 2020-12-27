//-----------------------------------------------------------------------------------------------------
// отображаем данные контроллера
//-----------------------------------------------------------------------------------------------------
var ModuleNamesBindings = {'STATE' : 'Модуль температур', 'HUMIDITY' : 'Модуль влажности', 'LIGHT' : 'Модуль освещенности', 'DELTA' : 'Модуль дельт', 'SOIL' : 'Модуль влажности почвы', 'PH' : 'Модуль контроля pH', '0' : 'Температура прибора'}
var NO_DATA = "<span class='no_data'>&lt;нет данных&gt;</span>";
//-----------------------------------------------------------------------------------------------------
// функция-хелпер для просмотра объекта
//-----------------------------------------------------------------------------------------------------
function print_r(printthis, returnoutput) 
{
    var output = '';

    if($.isArray(printthis) || typeof(printthis) == 'object') 
    {
        for(var i in printthis) 
        {
            output += i + ' : ' + print_r(printthis[i], true) + '\n';
        }
    }
    else 
    {
        output += printthis;
    }
    if(returnoutput && returnoutput == true) 
    {
        return output;
    }
    else 
    {
        alert(output);
    }
}
//-----------------------------------------------------------------------------------------------------
// проверяем бит
//-----------------------------------------------------------------------------------------------------
function BitIsSet(val, pos)
{
  return (val & (1 << pos)) != 0;
}
//-----------------------------------------------------------------------------------------------------
// прототип для удобного удаления значения из массива
//-----------------------------------------------------------------------------------------------------
Array.prototype.remove = function() {
    var what, a = arguments, L = a.length, ax;
    while (L && this.length) {
        what = a[--L];
        while ((ax = this.indexOf(what)) !== -1) {
            this.splice(ax, 1);
        }
    }
    return this;
};
//-----------------------------------------------------------------------------------------------------
// класс перепривязки индексов датчиков к мнемоническим именам
//-----------------------------------------------------------------------------------------------------
var SensorMnemonicName = function(sensorIndex,module_name,mnemonic_name)
{
  this.Controller = null;
  this.Index = sensorIndex;
  this.ModuleName = module_name;
  this.DisplayName = mnemonic_name;
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
// обновляет имя
SensorMnemonicName.prototype.setName = function(new_name)
{
  if(new_name != this.DisplayName)
  {
    this.DisplayName = new_name;
    this.Controller.saveSensorName(this); // сохраняем данные в БД
  }
}
//-----------------------------------------------------------------------------------------------------

// проверяет на совпадение с датчиком
SensorMnemonicName.prototype.IsMatch = function(sensor)
{
  return sensor.Index == this.Index && sensor.ModuleName == this.ModuleName;
}
//-----------------------------------------------------------------------------------------------------
// класс списка мнемонических имён для датчиков
//-----------------------------------------------------------------------------------------------------
var SensorMnemonics = function(controller)
{
  this.Controller = controller;
  this.List = new Array();
  return this;
}
//-----------------------------------------------------------------------------------------------------
// добавляет новое мнемоническое имя при загрузке из БД
SensorMnemonics.prototype.Add = function(mnemonic)
{
  var canAdd = true;
  for(var i=0;i<this.List.length;i++)
  {
    var obj = this.List[i];
    if(obj.IsMatch(mnemonic))
    {
      canAdd = false;
      obj.DisplayName = mnemonic.DisplayName;
      break;
    }
    
  } // for
  if(canAdd)
  {
    mnemonic.Controller = this.Controller; // сохраняем ссылку на контроллер
    this.List.push(mnemonic);
  }
}
//-----------------------------------------------------------------------------------------------------
// возвращает мнемоническое имя для датчика
SensorMnemonics.prototype.getMnemonicName = function(sensor)
{
  for(var i=0;i<this.List.length;i++)
  {
    var obj = this.List[i];
    if(obj.IsMatch(sensor))
      return obj.DisplayName;
    
  } // for
  
  return sensor.Index; // если ничего не нашли - возвращаем индекс датчика
}
//-----------------------------------------------------------------------------------------------------
// возвращает объект имени для датчика
SensorMnemonics.prototype.getMnemonicObject = function(sensor)
{
  for(var i=0;i<this.List.length;i++)
  {
    var obj = this.List[i];
    if(obj.IsMatch(sensor))
      return obj;
    
  } // for
  
  return null;
}
//-----------------------------------------------------------------------------------------------------
// класс для показаний с датчика
//-----------------------------------------------------------------------------------------------------
var Sensor = function(idx,module,data,hasData, additionalData)
{
  this.Index = idx; // индекс датчика
  this.ModuleName = module; // имя модуля, в котором датчик
  this.Data = data; // данные с датчика
  this.HasData = hasData; // флаг, что данные есть
  this.AdditionalData = additionalData; // дополнительные показания, например, вольтаж с датчика pH
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
Sensor.prototype.normalizedData = function()
{
  if(!this.HasData)
    return 0.0;
    
  return parseFloat(this.Data.toString().replace(",","."));
}
//-----------------------------------------------------------------------------------------------------
// класс списка показаний с датчиков
//-----------------------------------------------------------------------------------------------------
var SensorsList = function()
{
  this.List = new Array();
}
//-----------------------------------------------------------------------------------------------------
SensorsList.prototype.find = function(sensorIdx, moduleName)
{
  for(var i=0;i<this.List.length;i++)
  {
    var t = this.List[i];
    if(t.ModuleName == moduleName && t.Index == sensorIdx)
      return t;
  }
  
  return null;
}
//-----------------------------------------------------------------------------------------------------
SensorsList.prototype.Add = function(sensorIdx, moduleName, data, hasData, additionalData)
{
  var canAdd = true;
  for(var i=0;i<this.List.length;i++)
  {
    var t = this.List[i];
    if(t.ModuleName == moduleName && t.Index == sensorIdx)
    {
      canAdd = false;
      t.Data = data;
      t.HasData = hasData;
      t.AdditionalData = additionalData;
      break;
    }
  } // for
  if(canAdd)
  {
    var t = new Sensor(sensorIdx,moduleName, data, hasData, additionalData);
    this.List.push(t);
  }
}
//-----------------------------------------------------------------------------------------------------
// класс ответа на запрос Ajax
//-----------------------------------------------------------------------------------------------------
var Answer = function(data) 
{  
  this.Parse(data);
  return this;
}
//-----------------------------------------------------------------------------------------------------
Answer.prototype.Parse = function(data)
{
  data = data.toString();
  
  this.RawData = data;
  this.IsOK = data.indexOf("OK=") == 0;
  this.Params = new Array();
  
  if(this.IsOK)
  {
    data = data.substring(3);
    this.Params = data.split("|");
  }
}
//-----------------------------------------------------------------------------------------------------
// класс текущего действия
//-----------------------------------------------------------------------------------------------------
var Action = function(prepareFunc,doneFunc, userFunc)
{
 this.prepareFunc = prepareFunc;
 this.doneFunc = doneFunc;
 this.userFunc = userFunc;
 
return this;
}
//-----------------------------------------------------------------------------------------------------
// Класс контроллера, предназначенный для обмена данными с железкой
//-----------------------------------------------------------------------------------------------------
// конструктор
var Controller = function(id, name, address, silent)
{
  this.OnStatus = null; // обработчик события "онлайн/оффлайн"
  this.OnUpdate = null; // обработчик события "Получен слепок состояния с контроллера", вызывается как результат вызова метода queryState
  this.OnGetModulesList = null; // обработчик события получения списка модулей с контроллера
  this.OnQueryMotorWorkInterval = null; // обработчик события получения интервала работы моторов с контроллера
  this.OnQueryTemperatureSettings = null; // обработчик события получения настроек температур с контроллера
  
  // список имён датчиков
  this.SensorsNames = new SensorMnemonics(this);

  // список модулей, зарегистрированных в контроллере
  this.Modules = new Array();
 
  // списки показаний с датчиков
  this.TemperatureList = new SensorsList(); // список температур
  this.HumidityList = new SensorsList(); // список влажностей
  this.LuminosityList = new SensorsList(); // список освещённостей
  this.SoilMoistureList = new SensorsList(); // список влажностей почвы 
  this.PHList = new SensorsList(); // список датчиков pH
  
  this.Flow1Present = false; // доступен ли первый расходомер в прошивке
  this.FlowIncrementalLitres = 0; // кол-во литров расхода воды всего
  this.FlowInstantLitres = 0; // мгновенный расход воды

  this.Flow2Present = false; // доступен ли второй расходомер в прошивке
  this.FlowIncrementalLitres2 = 0; // кол-во литров расхода воды всего, второй расходомер
  this.FlowInstantLitres2 = 0; // мгновенный расход воды, второй расходомер

  
  this.IsWindowsOpen = false; // первый бит выставлен - окна открыты
  this.IsWindowsAutoMode = true; // второй бит выставлен - автоматический режим работы окон
  this.IsWaterOn = false; // третий бит выставлен - полив включен
  this.IsWaterAutoMode = true; // четвертый бит выставлен - автоматический режим работы полива
  this.IsLightOn = false; // пятый бит выставлен - включена досветка
  this.IsLightAutoMode = true; // шестой бит выставлен - автоматический режим работы досветки
  
  this.OpenTemperature = 0; // температура открытия
  this.CloseTemperature = 0; // температура закрытия
  this.MotorWorkInterval = 0; // время работы моторов
  
  this.IsPHFlowAddOn = false; // включен ли насос подачи воды в бак pH
  this.IsPHMixPumpOn = false; // включена ли помпа перемешивания раствора pH
  this.IsPHPlusPumpOn = false; // включена ли помпа подачи раствора для увеличения pH
  this.IsPHMinusPumpOn = false; // включена ли помпа подачи раствора для уменьшения pH
  

  this._id = id;
  this._name = name;
  this._address = address;
  this._isOnline = false;
  
  this._queue = new Array();
  this._currentAction = null;

  if(!silent) // только если попросили обновляться
  {
    this._statusTimer = window.setInterval(
    (function(self) {         
           return function() {
               self.updateStatus(); 
           }
       })(this)
    ,10000);
    
    this._queueTimer = window.setInterval(
    (function(self) {         
           return function() {   
               self.processQueue(); 
           }
       })(this)
    ,50);
    
    
    this.updateStatus();
  } // if(!silent)
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
// смотрит, нет ли в очереди такой же команды?
Controller.prototype.inQueue = function(prepareFunc,doneFunc)
{
  var objPassed = prepareFunc(this);
  
  for(var i=0;i<this._queue.length;i++)
  {
    var objInQueue = this._queue[i].prepareFunc(this);
    
    if(objPassed.url == objInQueue.url)
    {
        if(objPassed.data.query != undefined && objInQueue.data.query != undefined)
        {
          if(objPassed.data.query == objInQueue.data.query)
          {
//            console.log("SAME QUERY FOUND: " + objPassed.url + "; query=" + objInQueue.data.query);
            return true;
          }
        }
    }

  }
return false;
}
//-----------------------------------------------------------------------------------------------------
// помещает команду в очередь
Controller.prototype.addToQueue = function(prepareFunc,doneFunc, userFunc)
{
  if(this.inQueue(prepareFunc,doneFunc))
  { 
    return this;
  }

  this._queue.push(new Action(prepareFunc,doneFunc, userFunc));
  
  return this;
}
//-----------------------------------------------------------------------------------------------------
// обрабатывает очередь запросов
Controller.prototype.processQueue = function()
{
if(this._currentAction != null || !this._queue.length)
    return this;
    
    this._currentAction = this._queue.shift();
    
    
    var preparedData = this._currentAction.prepareFunc(this);
    
    var _this = this;
    
 $.ajax( 
 { 
    method: preparedData.method
  , url: preparedData.url
  , dataType: "json" 
  , data: preparedData.data
 }  ).done(function(data) 
        {
   
          _this.processRequestResult(data);
     
        }).fail(function(){
          _this.processRequestResult({query_result : 'ER=FAIL', online: 0});
        });    
    

  return this;
}
//-----------------------------------------------------------------------------------------------------
// обрабатываем результат запроса к серверу
Controller.prototype.processRequestResult = function(data)
{
  this._currentAction.doneFunc(this, data);
  this._currentAction = null;
}
//-----------------------------------------------------------------------------------------------------
// онлайн ли контроллер?
Controller.prototype.IsOnline = function()
{
  return this._isOnline;
}
//-----------------------------------------------------------------------------------------------------
// обновляем статус
Controller.prototype.updateStatus = function()
{

  this.addToQueue(
    function(obj)
    {
      return { method: "GET", url: "/x_check_controller.php", data: {posted : 1, controller_id : obj.getId(), query : "_check" } };
    },
    function(obj,result)
    {
      var lastIsOnline = obj._isOnline;
      
      obj._isOnline = result.online == 1 ? true : false;

      if(!lastIsOnline && obj._isOnline)
      {
        // надо получить список модулей в контроллере
        obj.queryModules();
        
      }

      if(obj.OnStatus != null)
        obj.OnStatus(obj);
    }
    , null
    
  );
}
//-----------------------------------------------------------------------------------------------------
// получаем настройки температур
Controller.prototype.queryTemperatureSettings = function()
{  
  this.queryCommand(true,"STATE|T_SETT", (function(self) {         
         return function(obj, answer) { 
         
            if(answer.IsOK)
            {
              self.OpenTemperature = answer.Params[2];
              self.CloseTemperature = answer.Params[3];
              
              if(self.OnQueryTemperatureSettings != null)
                self.OnQueryTemperatureSettings(self);
            }
             
         }
     })(this)
     );  
}
//-----------------------------------------------------------------------------------------------------
// сохраняет настройки температур
Controller.prototype.saveTemperatureSettings = function(t_open, t_close)
{
  this.OpenTemperature = t_open;
  this.CloseTemperature = t_close;

  var cmd = "STATE|T_SETT|" + t_open + "|" + t_close;
  
    
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// получаем интервал работы мотора
Controller.prototype.queryMotorInterval = function()
{
  this.queryCommand(true,"STATE|INTERVAL", (function(self) {         
         return function(obj, answer) {   
             if(answer.IsOK)
                self.MotorWorkInterval = answer.Params[2];
                
              if(self.OnQueryMotorWorkInterval != null)
                self.OnQueryMotorWorkInterval(self);
                
             
         }
     })(this)
     );  
}
//-----------------------------------------------------------------------------------------------------
// сохраняет настройку времени работы моторов
Controller.prototype.saveMotorInterval = function(new_interval)
{
  this.MotorWorkInterval = new_interval*1000;

  var cmd = "STATE|INTERVAL|" + this.MotorWorkInterval;
     
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// получаем список модулей в прошивке
Controller.prototype.queryModules = function()
{
  this.Modules = new Array(); // очищаем список
  
  this.queryCommand(true,"0|LIST", (function(self) {         
         return function(obj, answer) {   
             self.Modules = answer.Params;
             
             if(self.OnGetModulesList != null)
              self.OnGetModulesList(self);
             
         }
     })(this)
     );  
}
//-----------------------------------------------------------------------------------------------------
// посылаем команду контроллеру
Controller.prototype.queryCommand = function(isGetQuery, query, userFunc)
{
  var _url = isGetQuery ? "/x_query_controller.php" : "/x_set_controller.php";
  this.addToQueue(
    function(obj)
    {
      return { method: "GET", url: _url, data: {posted : 1, controller_id : obj.getId(), query: query } };
    },
    function(obj,result)
    {
      if(obj._currentAction.userFunc)
        obj._currentAction.userFunc(obj,new Answer(result.query_result));
    }
    , userFunc
  );
  
}
//-----------------------------------------------------------------------------------------------------
// парсим пришедшие с контроллера данные
Controller.prototype.parseControllerState = function(answer)
{

  // тут парсим данные, пришедшие с контроллера
  if(answer.IsOK)
  {
  
   var line = answer.Params[0].toString().trim();
   var firstByte = '0x' + line.substring(0, 2);
   var secondByte = '0x' + line.substring(2, 4);

    // переводим это в номер всё
    var num = parseInt(firstByte);
    // теперь смотрим, чего там в состояниях
    this.IsWindowsOpen = BitIsSet(num, 0); // первый бит выставлен - окна открыты
    this.IsWindowsAutoMode = BitIsSet(num, 1); // второй бит выставлен - автоматический режим работы окон
    this.IsWaterOn = BitIsSet(num, 2); // третий бит выставлен - полив включен
    this.IsWaterAutoMode = BitIsSet(num, 3); // четвертый бит выставлен - автоматический режим работы полива
    this.IsLightOn = BitIsSet(num, 4); // пятый бит выставлен - включена досветка
    this.IsLightAutoMode = BitIsSet(num, 5); // шестой бит выставлен - автоматический режим работы досветки
    
    this.IsPHFlowAddOn = BitIsSet(num,7);
    num = parseInt(secondByte);
    this.IsPHMixPumpOn = BitIsSet(num,0);
    this.IsPHPlusPumpOn = BitIsSet(num,1);
    this.IsPHMinusPumpOn = BitIsSet(num,2);
    
    
    this.Flow1Present = false;
    this.Flow2Present = false;


   line = line.substring(4);
   
   while (line.length > 0)
   {
     // читаем байт флагов модуля
      var f = "0x" + line.substring(0, 4);
      line = line.substring(4);

      // смотрим, чего там во флагах модуля задано
      var flags = parseInt(f);
      var tempPresent = (flags & 1) == 1;
      var luminosityPresent = (flags & 4) == 4;
      var humidityPresent = (flags & 8) == 8;
      var waterFlowInstantPresent = (flags & 16) == 16;
      var waterFlowIncrementalPresent = (flags & 32) == 32;
      var soilMoisturePresent = (flags & 64) == 64;
      var phPresent = (flags & 128) == 128;

      // читаем байт имени модуля
      var s = "0x" + line.substring(0, 2);
      var namelen = parseInt(s);
      
      // дальше идёт имя модуля, читаем его
      var moduleName = line.substring(2, 2+namelen);
      
      line = line.substring(2 + namelen);

      var cnt = 0;

      // прочитали имя модуля, можем работать с датчиками
      // сперва идут температурные, первый байт - их количество
      if (tempPresent)
      {
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // теперь читаем данные с датчиков
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);
              // затем два байта - его показания
              var val = line.substring(2, 4);
              var fract = line.substring(4, 6);
              line = line.substring(6);

              // теперь смотрим, есть ли показания с датчика
              var haveSensorData = !(val == "FF" && fract == "FF");
              var temp = "";
              if (haveSensorData)
              {
                  // имеем показания, надо сконвертировать
                  var hexTemp = parseInt("0x" + val);
                  
                  // конвертируем в знаковое представление
                  if((hexTemp & 0x80) > 0)
                      hexTemp = hexTemp - 0x100;
                  
                  temp = "" + hexTemp + ",";
                  var fractVal = parseInt("0x" + fract);
                  if (fractVal < 10)
                      temp += "0";
                  temp += '' + fractVal;
              }
              // получили показания с датчика, надо их сохранить в список
              this.TemperatureList.Add(sensorIdx,moduleName,temp,haveSensorData);
              
          } // for
      } // if(tempPresent)
      // опрос температурных датчиков окончен, переходим на датчики влажности

      if (humidityPresent)
      {
          // переходим на чтение данных с датчиков влажности
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // обрабатываем их
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);
              // затем два байта - его показания
              var val = line.substring(2, 4);
              var fract = line.substring(4, 6);
              line = line.substring(6);

              // теперь смотрим, есть ли показания с датчика
              var haveSensorData = !(val == "FF" && fract == "FF");
              var humidity = "";
              if (haveSensorData)
              {
                  // имеем показания, надо сконвертировать
                  humidity = '' + parseInt("0x" + val) + ",";
                  var fractVal = parseInt("0x" + fract);
                  if (fractVal < 10)
                      humidity += "0";
                  humidity += '' + fractVal;
              }

              // получили показания с датчика, надо их сохранить в список
              this.HumidityList.Add(sensorIdx, moduleName, humidity, haveSensorData);

          } // for
      } // if(humidityPresent)

      if (luminosityPresent)
      {
          // далее идут показания датчиков освещенности
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // обрабатываем их
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);
              // затем два байта - его показания
              var val = line.substring(2, 4);
              var fract = line.substring(4, 6);
              line = line.substring(6);

              // теперь смотрим, есть ли показания с датчика
              var haveSensorData = !(val == "FF" && fract == "FF");
              var luminosity = 0;
              if (haveSensorData)
              {
                  // имеем показания, надо сконвертировать
                  luminosity = parseInt("0x" + val + fract);

              }

              // получили показания с датчика, надо их  сохранить в список
              this.LuminosityList.Add(sensorIdx, moduleName, luminosity, haveSensorData);

          } // for
      } // luminosityPresent

      if (waterFlowInstantPresent)
      {
          // далее идут показания датчиков мгновенного расхода воды
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // обрабатываем их
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);

              // затем 4 байта - его показания
              var dt = "0x" + line.substring(2, 10);
              line = line.substring(10);
              var flow = parseInt(dt);

              //сохраняем показания с датчика мгновенного расхода воды
              if(sensorIdx == 0)
              {
                this.Flow1Present = true;
                this.FlowInstantLitres = flow;
              }
              else
              {
                this.Flow2Present = true;
                this.FlowInstantLitres2 = flow;
              }

          } // for
      } // waterFlowInstantPresent

      if (waterFlowIncrementalPresent)
      {
          // далее идут показания датчиков накопительного расхода воды
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // обрабатываем их
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);

              // затем 4 байта - его показания
              var dt = "0x" + line.substring(2, 10);
              line = line.substring(10);
              var flow = parseInt(dt);

              //сохраняем показания с датчика накопительного расхода воды
              if(sensorIdx == 0)
              {
                this.Flow1Present = true;
                this.FlowIncrementalLitres = flow;
              }
              else
              {
                this.Flow2Present = true;
                this.FlowIncrementalLitres2 = flow;
              }

          } // for
      } // waterFlowIncrementalPresent

      // разбираем показания с датчиков влажности почвы
      if (soilMoisturePresent)
      {
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);

          // теперь читаем данные с датчиков
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);
              // затем два байта - его показания
              var val = line.substring(2, 4);
              var fract = line.substring(4, 6);
              line = line.substring(6);

              // теперь смотрим, есть ли показания с датчика
              var haveSensorData = !(val == "FF" && fract == "FF");
              var temp = "";
              if (haveSensorData)
              {
                  // имеем показания, надо сконвертировать
                  temp = '' + parseInt("0x" + val) + ",";
                  var fractVal = parseInt("0x" + fract);
                  if (fractVal < 10)
                      temp += "0";
                  temp += '' + fractVal;
              }

              // получили показания с датчика, надо их  сохранить в список
              this.SoilMoistureList.Add(sensorIdx, moduleName, temp, haveSensorData);
          } // for
      } // if(soilMoisturePresent)
      
      if (phPresent)
      {
          // переходим на чтение данных с датчиков PH
          s = "0x" + line.substring(0, 2);
          cnt = parseInt(s);
          line = line.substring(2);
          
          // обрабатываем их
          for (var i = 0; i < cnt; i++)
          {
              // первым байтом идёт индекс датчика
              s = "0x" + line.substring(0, 2);
              var sensorIdx = parseInt(s);
              // затем два байта - его показания
              var val = line.substring(2, 4);
              var fract = line.substring(4, 6);
              
              // затем 2 байта - вольтаж в порту
              var phMV = line.substring(6,10);
              
              line = line.substring(10);

              // теперь смотрим, есть ли показания с датчика
              var haveSensorData = !(val == "FF" && fract == "FF");
              var ph = "";
              var phVoltage = "";
              
              if (haveSensorData)
              {
                  // имеем показания, надо сконвертировать
                  ph = '' + parseInt("0x" + val) + ",";
                  var fractVal = parseInt("0x" + fract);
                  if (fractVal < 10)
                      ph += "0";
                  ph += '' + fractVal;
                  
                  phVoltage = '' + parseInt("0x" + phMV);
              }

              // получили показания с датчика, надо их сохранить в список
              this.PHList.Add(sensorIdx, moduleName, ph, haveSensorData,phVoltage);
              

           } // for
           
        } // if(phPresent)
      

      // все датчики обработали, переходим к следующему модулю

   
   } // while
  
  
  } // is ok

  if(this.OnUpdate != null)
    this.OnUpdate(this,answer);
    
}
//-----------------------------------------------------------------------------------------------------
// пустой обработчик ответа
Controller.prototype.dummyAnswer = function(answer)
{
}
//-----------------------------------------------------------------------------------------------------
// переключает состояние досветки
Controller.prototype.toggleLight = function()
{
 var cmd = "LIGHT|";
  if(!this.IsLightOn)
    cmd += "ON";
  else
    cmd += "OFF";
    
  this.IsLightOn = !this.IsLightOn;
  this.IsLightAutoMode = false;    
  
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// переключает режим работы досветки
Controller.prototype.toggleLightMode = function()
{
 var cmd = "LIGHT|MODE|";
  if(!this.IsLightAutoMode)
    cmd += "AUTO";
  else
    cmd += "MANUAL";
    
  this.IsLightAutoMode = !this.IsLightAutoMode;
      
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// вкл/выкл полив
Controller.prototype.toggleWater = function()
{
 var cmd = "WATER|";
  if(!this.IsWaterOn)
    cmd += "ON";
  else
    cmd += "OFF";
    
  this.IsWaterOn = !this.IsWaterOn;
  this.IsWaterAutoMode = false;    
  
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// переключаем режим полива
Controller.prototype.toggleWaterMode = function()
{
var cmd = "WATER|MODE|";
  if(!this.IsWaterAutoMode)
    cmd += "AUTO";
  else
    cmd += "MANUAL";
    
  this.IsWaterAutoMode = !this.IsWaterAutoMode;
      
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// откр/закр окна
Controller.prototype.toggleWindows = function()
{
var cmd = "STATE|WINDOW|ALL|";
  if(!this.IsWindowsOpen)
    cmd += "OPEN";
  else
    cmd += "CLOSE";
    
  this.IsWindowsOpen = !this.IsWindowsOpen;
  this.IsWindowsAutoMode = false;
      
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// переключить режим работы окон
Controller.prototype.toggleWindowsMode = function()
{
var cmd = "STATE|MODE|";
  if(!this.IsWindowsAutoMode)
    cmd += "AUTO";
  else
    cmd += "MANUAL";
    
  this.IsWindowsAutoMode = !this.IsWindowsAutoMode;
      
  this.queryCommand(false,cmd, (function(self) {         
         return function(obj, answer) {   
             self.dummyAnswer(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
// получает слепок состояния контроллера
Controller.prototype.queryState = function()
{
  this.queryCommand(true,"0|STAT", (function(self) {         
         return function(obj, answer) {   
             self.parseControllerState(answer); 
         }
     })(this)
     );
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.getName = function()
{
  return this._name;
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.setName = function(newName)
{
  this._name = newName;
  return this;
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.getId = function()
{
  return this._id;
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.setId = function(newId)
{
  this._id = newId;
  return this;
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.getAddress = function()
{
  return this._address;
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.setAddress = function(newAddress)
{
  this._address = newAddress;
  return this;
}
//-----------------------------------------------------------------------------------------------------
// получает список имён датчиков
Controller.prototype.querySensorNames = function()
{
  var _this = this;
   $.ajax( 
 { 
    method: "GET"
  , url: "/x_get_sensor_names.php"
  , dataType: "json" 
  , data: 
    {
      posted : 1
      , controller_id : _this._id
    }
 }  ).done(function(data) 
        {
          
           for(var i=0;i<data.names.length;i++)
           {
            var dt = data.names[i];
            _this.SensorsNames.Add(new SensorMnemonicName(dt.sensor_idx,dt.module_name,dt.display_name));
           } 
           
          if(_this.OnGetSensorNames != null)
            _this.OnGetSensorNames(_this);
     
        });  
}
//-----------------------------------------------------------------------------------------------------
// сохраняет новое имя для датчика
Controller.prototype.saveSensorName = function(mnemonic)
{
  var _this = this;
   $.ajax( 
 { 
    method: "GET"
  , url: "/x_save_sensor_name.php"
  , dataType: "json" 
  , data: 
    {
      posted : 1
      , controller_id : _this._id
      , sensor_idx: mnemonic.Index
      , module_name: mnemonic.ModuleName
      , display_name: mnemonic.DisplayName  
    }
 }  ).done(function(data) 
        {
   
          
     
        });  
}
//-----------------------------------------------------------------------------------------------------
// запрашивает серверный скрипт и возвращает результат запроса
Controller.prototype.queryServerScript = function(script_name,params,doneFunc)
{  
  params.controller_id = this._id;
  params.posted = 1;
  
  var _this = this;
  
  $.ajax( 
 { 
    method: "GET"
  , url: script_name
  , dataType: "json" 
  , data: params
    
 }  ).done(function(data) 
        {
          doneFunc(_this,data);
        });      
       
  
}
//-----------------------------------------------------------------------------------------------------
Controller.prototype.deleteController = function(doneCallback,failCallback) {
 var that = this; 
 $.ajax( 
 { 
    method: "GET"
  , url: "/x_delete_controller.php"
  , dataType: "json" 
  , data: 
    {
      posted : 1
      , controller_id : that._id
    }
 }  ).done(function(data) 
        {
   
          if(doneCallback)
            doneCallback(data);
     
        }).fail(function(){
        
          if(failCallback)
            failCallback();
        
        });  

}
//-----------------------------------------------------------------------------------------------------
// редактирует данные контроллера и отправляет их на сервер в обход очереди
Controller.prototype.edit = function(new_id, onAjaxDone)
{
  var this_id = this._id;
  this._id = new_id;
  var _this = this;
    
  $.ajax( 
 { 
    method: "GET"
  , url: "/x_edit_controller.php"
  , dataType: "json" 
  , data: 
    {
      posted : 1
      , controller_id : this_id
      //, new_id: new_id
      , controller_name: _this._name
      , c_addr: _this._address  
    }
 }  ).done(function(data) 
        {
   
          onAjaxDone(_this, new Answer(data.authorized.toString()));
     
        });      
     
}
//-----------------------------------------------------------------------------------------------------