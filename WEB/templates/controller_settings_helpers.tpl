{* Smarty *}


<script type="text/javascript" src="js/deltas.js"></script>
<script type="text/javascript" src="js/deltas_view.js"></script>
<script type="text/javascript" src="js/cc.js"></script>
<script type="text/javascript" src="js/water_settings.js"></script>
<script type="text/javascript" src="js/rules.js"></script>
<script type="text/javascript" src="js/sms.js"></script>
<script type="text/javascript" src="js/reservation.js"></script>


<script type='text/javascript'>
//-----------------------------------------------------------------------------------------------------
// наш контроллер
var controller = new Controller({$selected_controller.controller_id},'{$selected_controller.controller_name}','{$selected_controller.controller_address}');

var deltaList = new DeltaList();
var deltaView = new DeltaView(controller, deltaList);

var compositeCommands = new CompositeCommands(); // список составных команд
var wateringSettings = new WateringSettings(); // настройки полива
var rulesList = new RulesList(); // список правил из контроллера
var smsList = new SMSList(); // список СМС
var reservations = new Reservations(); // список резервирования

{literal}
//-----------------------------------------------------------------------------------------------------
// запрошено редактирование дельты
deltaView.OnEditDelta = function(controllerObject, delta, row)
{

}
//-----------------------------------------------------------------------------------------------------
// запрошено удаление дельты
deltaView.OnDeleteDelta = function(controllerObject, delta, row)
{
  deltaList.List.remove(delta); // удаляем дельту из списка
  row.remove(); // удаляем строку из таблицы  
}
//-----------------------------------------------------------------------------------------------------
function showWaitDialog()
{
  $("#data_requested_dialog" ).dialog({
                dialogClass: "no-close",
                modal: true,
                closeOnEscape: false,
                draggable: false,
                resizable: false,
                buttons: []
              });
}
//-----------------------------------------------------------------------------------------------------
function closeWaitDialog()
{
  $("#data_requested_dialog" ).dialog('close');
}
//-----------------------------------------------------------------------------------------------------
function showMessage(message, close_callback)
{
  $('#message_dialog_message').html(message);

  $("#message_dialog").dialog({modal:true, buttons: [
    {text: "ОК", click: function(){$(this).dialog("close");} }
  ], close: function(){ if(close_callback) close_callback(); } });     
}
//-----------------------------------------------------------------------------------------------------
function promptMessage(message, yesFunc, cancelFunc)
{
  $('#prompt_dialog_message').html(message);

  $("#prompt_dialog").dialog({modal:true, buttons: [
    {text: "ОК", click: function(){$(this).dialog("close"); if(yesFunc) yesFunc(); } },
    {text: "Отмена", click: function(){$(this).dialog("close");  if(cancelFunc) cancelFunc(); } }
  ] });     
}
//-----------------------------------------------------------------------------------------------------
// устанавливаем дату/время для контроллера
function setControllerTime()
{
  promptMessage('Установить дату/время контроллера на текущее время компьютера?',
  function() {
    
    controllerInternalDate = new Date();
    
    var cmd = '0|DATETIME|' + formatDateTime(controllerInternalDate);
    controller.queryCommand(false,cmd);
    
  }

  );
}
//-----------------------------------------------------------------------------------------------------
// добавляем дельту
function newDelta()
{

  if(deltaList.List.length >= MAX_DELTAS)
  {
    showMessage("Достигнуто максимальное количество дельт: " + MAX_DELTAS);
    return;
  }
  
  $('#delta_type').trigger('change'); 
  
 $("#new_delta_dialog").dialog({modal:true, buttons: [{text: "Добавить", click: function(){
  
      var delta_type = $('#delta_type').val();
      var delta_module1 = $('#delta_module1').val();
      var delta_module2 = $('#delta_module2').val();
      var delta_index1 = parseInt($('#delta_index1').val());
      var delta_index2 = parseInt($('#delta_index2').val());
      
      if(isNaN(delta_index1)) delta_index1 = 0;
      if(isNaN(delta_index2)) delta_index2 = 0;
      
      var delta = new Delta(delta_type,delta_module1,delta_index1,delta_module2,delta_index2);
      
      if(!deltaList.Add(delta))
      {
        showMessage('Такая дельта уже существует!');
        return;
      }
      
      deltaView.fillList('#DELTA_LIST');            
      $(this).dialog("close");
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });     
  
}
//-----------------------------------------------------------------------------------------------------
function addReservationCheckbox(moduleName,sensorIndex)
{
  var list = $('#reservation_sensors_list');

  var row = $('<div/>').appendTo(list);

  var chb = $('<input/>',{type: 'checkbox', id: 'reservation_checkbox', value: sensorIndex, 'module_name' : moduleName});
  chb.appendTo(row);
  var mnemonicName = controller.SensorsNames.getMnemonicName(new Sensor(sensorIndex,moduleName));
  if(mnemonicName == sensorIndex)
  {
    var rr  = new Reservation(moduleName,sensorIndex);
    mnemonicName = rr.getDisplayString();
  }  
  $('<label/>').text(mnemonicName).appendTo(row);
}
//-----------------------------------------------------------------------------------------------------
function newReservation()
{
  if(reservations.Items.length > 9)
  {
    showMessage('Максимальное кол-во списков резервирования - 10!');
    return;
  }
  
  $('#reservation_type_select').trigger('change');
  

  $("#new_reservation_dialog").dialog({modal:true, width: 400, buttons: [{text: "Добавить", click: function(){

      
      var checked_cnt = 0;
      $('#reservation_sensors_list').find('div #reservation_checkbox').each(function(idx,elem){
      
        if(elem.checked)
          checked_cnt++;
      
      });
      
      if(checked_cnt < 2)
      {
        showMessage('Выберите по крайней мере два датчика для списка резервирования!');
        return;
      }
      
      if(checked_cnt > 9)
      {
        showMessage('Максимальное кол-во датчиков в одном списке резервирования - 10!');
        return;
      }
      
      var srcListLength = reservations.Items.length;
      
      var resList = reservations.Add($('#reservation_type_select').val());
      
      $('#reservation_sensors_list').find('div #reservation_checkbox').each(function(idx,elem){
      
          if(elem.checked)
          {
            var $elem = $(elem);
            var module_name = $elem.attr('module_name');
            var sensor_index = $elem.val();
            
            resList.Add(module_name,sensor_index);  
              
          }
      
      });
                
       addReservationRow('#RESERVATION_LIST',resList,srcListLength);
  

  
      $(this).dialog("close");
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });       
  
}
//-----------------------------------------------------------------------------------------------------
function saveTimers()
{
  showWaitDialog();
              
   var cmd = "TMR";
   
   for(var i=1;i<5;i++)
   {
      
      var tmrDayMaskAndEnable = 0;
      
      var en = $('#timerEnabled' + i).get(0).checked;
      if(en)
        tmrDayMaskAndEnable |= 128;
        
      $('#timerDayMask' + i).find('input[type=checkbox]').each(function(idx,elem){
      
          if(elem.checked)
            tmrDayMaskAndEnable |= (1 << idx);
      
      });
      
      cmd += "|" + tmrDayMaskAndEnable;
      
      var pin = parseInt($('#timerPin' + i).val());
      if(isNaN(pin))
          pin = 0;
      if(pin < 0)
        pin = 0;

      $('#timerPin' + i).val(pin);
        
        cmd += "|" + pin;
        
      var onMin = parseInt($('#timerOnMin' + i).val());
      if(isNaN(onMin))
        onMin = 0;
      if(onMin < 0)
        onMin = 0;
        
      var onSec = parseInt($('#timerOnSec' + i).val());
      if(isNaN(onSec))
        onSec = 0;
      if(onSec < 0)
        onSec = 0;
        
      var tmrHoldOnTime = onMin*60 + onSec;
      if(tmrHoldOnTime > 65535)
      {
        tmrHoldOnTime = 65535;
        
        $('#timerOnMin' + i).val(parseInt(tmrHoldOnTime/60));
        $('#timerOnSec' + i).val(parseInt(tmrHoldOnTime%60));
      }
        
      cmd += "|" + tmrHoldOnTime;
 
     var offMin = parseInt($('#timerOffMin' + i).val());
      if(isNaN(offMin))
        offMin = 0;
      if(offMin < 0)
        offMin = 0;

     
      var offSec = parseInt($('#timerOffSec' + i).val());
      if(isNaN(offSec))
        offSec = 0;
      if(offSec < 0)
        offSec = 0;
        
      var tmrHoldOffTime = offMin*60 + offSec;
      if(tmrHoldOffTime > 65535)
      {
        tmrHoldOffTime = 65535;
        $('#timerOffMin' + i).val(parseInt(tmrHoldOffTime/60));
        $('#timerOffSec' + i).val(parseInt(tmrHoldOffTime%60));
      }
        
      cmd += "|" + tmrHoldOffTime;      
      
   } // for
      
    controller.queryCommand(false,cmd,function(obj,answer){
    
      closeWaitDialog();
      
      if(answer.IsOK)
        showMessage("Данные успешно сохранены!");
      else
        showMessage("Ошибка сохранения данных :(");
    });
}
//-----------------------------------------------------------------------------------------------------
function saveReservationList()
{

 showWaitDialog();
              
   controller.queryCommand(false,'RSRV|DEL',function(obj,answer){
    
        if(!answer.IsOK)
        {
           closeWaitDialog();
           return;
        } 
        
          
          for(var i=0;i<reservations.Items.length;i++)
          {
            var resInfo = reservations.Items[i];
            var cmd = 'RSRV|ADD|' + resInfo.Type;

              for(var j=0;j<resInfo.Items.length;j++)
              {
                cmd += '|';
                var rr = resInfo.Items[j];
                cmd += rr.ModuleName + '|' + rr.SensorIndex;
                
              } // for
    
              controller.queryCommand(false,cmd,function(obj,addResult){
              
                                   
              
              });
            
              
          } // for
          
                    controller.queryCommand(false,'RSRV|SAVE',function(obj,saveResult){
                    
                      closeWaitDialog();
                          if(saveResult.IsOK)
                              showMessage("Данные успешно сохранены!");
                            else
                              showMessage("Ошибка сохранения данных :(");                      
                      
                    });
          
      
    
    });              
              
}
//-----------------------------------------------------------------------------------------------------
// сохраняем список дельт в контроллер
function saveDeltasList()
{

  showWaitDialog();
              
              
    controller.queryCommand(false,'DELTA|DEL',function(obj,answer){
    
        if(!answer.IsOK)
        {
           closeWaitDialog();
           return;
        } 
        
          
          for(var i=0;i<deltaList.List.length;i++)
          {
            var delta = deltaList.List[i];
            var cmd = 'DELTA|ADD|' + delta.Type + '|' + delta.ModuleName1 + '|' + delta.Index1 + '|' + delta.ModuleName2 + '|' + delta.Index2;

            
              controller.queryCommand(false,cmd,function(obj,addResult){
              
                                   
              
              });
            
              
          } // for
          
                    controller.queryCommand(false,'DELTA|SAVE',function(obj,saveResult){
                    
                            closeWaitDialog();
                            if(saveResult.IsOK)
                              showMessage("Данные успешно сохранены!");
                            else
                              showMessage("Ошибка сохранения данных :(");                      

                      
                    });
          
      
    
    });

}
//-----------------------------------------------------------------------------------------------------
// редактируем настройки Wi-Fi
function editWiFiSettings()
{
 $("#wifi_dialog").dialog({modal:true, buttons: [{text: "Изменить", click: function(){

      
      var shouldConnect = $('#connect_to_router').is(':checked') ? 1 : 0;
      
      var data = {
      
        connect_to_router: shouldConnect,
        router_id: $('#router_id').val(),
        router_pass: $('#router_pass').val(),
        station_id: $('#station_id').val(),
        station_pass: $('#station_pass').val()
        
        };
        
        if(data.router_id == '' || data.router_pass == '' || data.station_id == '' || data.station_pass == '')
          return;
      
        $(this).dialog("close");

      
      showWaitDialog();
                    
        controller.queryServerScript("/x_save_wifi_settings.php",data, function(obj,answer){
        
            var cmd = 'WIFI|T_SETT|' + data.connect_to_router + '|' + data.router_id + '|' 
            + data.router_pass + '|' + data.station_id + '|' + data.station_pass;
        
            controller.queryCommand(false,cmd,function(obj,answer){
           
                closeWaitDialog();
                /*
                if(answer.IsOK)
                  showMessage("Данные успешно сохранены!");
                else
                  showMessage("Ошибка сохранения данных :(");                      
                */
                showMessage("Настройки Wi-Fi применены, ждите, когда контроллер переконнектится.");

            });
        
           
            
        });      
      
  
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  

}
//-----------------------------------------------------------------------------------------------------
// показывает информацию о датчиках, прописанных в прошивке
function showSensorsInfo()
{
 $("#sensors_info_dialog").dialog({modal:true, buttons: [
  {text: "OK", click: function(){$(this).dialog("close");} }
  ] });
}
//-----------------------------------------------------------------------------------------------------
// редактируем номер телефона
function editPhoneNumber()
{
 $("#phone_number_dialog").dialog({modal:true, buttons: [{text: "Изменить", click: function(){

      $(this).dialog("close");

      var num = $('#edit_phone_number').val();
      
      var selectOp = $('#gsm_provider').get(0);
      var gsmProvider = selectOp.options[selectOp.selectedIndex].value;
      
      showWaitDialog(); 
      controller.queryCommand(false,'0|PHONE|' + num,function(obj,answer){
      
              controller.queryCommand(false,'SMS|PROV|' + gsmProvider,function(obj,answer2){
      
              closeWaitDialog();
              
              if(answer2.IsOK)
                showMessage("Данные успешно сохранены!");
              else
                showMessage("Ошибка сохранения данных :(");  
                
                });                    
      
      });
      
  
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  
}
//-----------------------------------------------------------------------------------------------------
// редактируем настройки PH
function editPHCalibration()
{
 $("#ph_calibration_dialog").dialog({modal:true, width: 500, buttons: [{text: "Изменить", click: function(){

      var cal1 = parseInt($('#ph_calibraton').val());
      
      
      if(isNaN(cal1))
        return;
        

        
       if(cal1 > 100)
       {
        cal1 = 100;
        $('#ph_calibraton').val(cal1);
       }

       if(cal1 < -100)
       {
        cal1 = -100;
        $('#ph_calibraton').val(cal1);
       }
       
       var ph4V = parseInt($('#ph4Voltage').val());
       if(isNaN(ph4V) || ph4V < 0)
        return;

       var ph7V = parseInt($('#ph7Voltage').val());
       if(isNaN(ph7V) || ph7V < 0)
        return;

       var ph10V = parseInt($('#ph10Voltage').val());
       if(isNaN(ph10V) || ph10V < 0)
        return;

       var phTempSensor = parseInt($('#phTemperatureSensor').val());
       if(isNaN(phTempSensor) || phTempSensor < 0)
        return;

       var phCalTemp = parseFloat($('#phCalibrationTemperature').val());
       if(isNaN(phCalTemp) || phCalTemp < 0)
        return;
        
       phCalTemp = parseInt(phCalTemp*100);
       
       var phTarget = parseFloat($('#phTarget').val());
       if(isNaN(phTarget) || phTarget < 0)
        return;
        
       phTarget = parseInt(phTarget*100);
       
       var phHisteresis = parseFloat($('#phHisteresis').val());
       if(isNaN(phHisteresis) || phHisteresis < 0)
        return;
        
       phHisteresis = parseInt(phHisteresis*100);
              
       var phMixPumpTime = parseInt($('#phMixPumpTime').val());
       if(isNaN(phMixPumpTime) || phMixPumpTime < 0)
        return;
        
       var phReagentPumpTime = parseInt($('#phReagentPumpTime').val());
       if(isNaN(phReagentPumpTime) || phReagentPumpTime < 0)
        return;

      $(this).dialog("close");

      
      showWaitDialog();
                    
      controller.queryCommand(false,'PH|T_SETT|' + cal1 + '|' + ph4V + '|' + ph7V + '|' + ph10V + '|' + phTempSensor  + '|' + phCalTemp
      + '|' + phTarget
      + '|' + phHisteresis
      + '|' + phMixPumpTime
      + '|' + phReagentPumpTime
      
      ,function(obj,answer){
      
      closeWaitDialog();
      if(answer.IsOK)
        showMessage("Данные успешно сохранены!");
      else
        showMessage("Ошибка сохранения данных :(");                      
      
      });
      
  
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  
}
//-----------------------------------------------------------------------------------------------------
// редактируем калибровки расходомеров
function editFlowCalibration()
{
 $("#flow_calibration_dialog").dialog({modal:true, buttons: [{text: "Изменить", click: function(){

      var cal1 = parseInt($('#flow_calibraton1').val());
      var cal2 = parseInt($('#flow_calibraton2').val());
      
      
      if(isNaN(cal1) || isNaN(cal2))
        return;
        
      if(cal1 < 1 || cal2 < 1)
        return;
        
       if(cal1 > 255)
       {
        cal1 = 255;
        $('#flow_calibraton1').val(cal1);
       }

       if(cal2 > 255)
       {
        cal2 = 255;
        $('#flow_calibraton2').val(cal2);
       }


      $(this).dialog("close");

      
      showWaitDialog();
                    
      controller.queryCommand(false,'FLOW|T_SETT|' + cal1 + '|' + cal2,function(obj,answer){
      
      closeWaitDialog();
      if(answer.IsOK)
        showMessage("Данные успешно сохранены!");
      else
        showMessage("Ошибка сохранения данных :(");                      
      
      });
      
  
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  
}
//-----------------------------------------------------------------------------------------------------
// получаем список дельт с контроллера
function queryDeltasList()
{
  closeWaitDialog();
  showWaitDialog();
              
              
     controller.queryCommand(true,'DELTA|CNT',function(obj,answer){
     
            if(!answer.IsOK)
            {
              closeWaitDialog();
              return;
            }
     
            var cnt = parseInt(answer.Params[2]);
            deltaList.clear();
            
            if(!cnt)
            {
              closeWaitDialog();
              return;
            }
            
            for(var i=0;i<cnt;i++)
            {
            
                  controller.queryCommand(true,'DELTA|VIEW|' + i.toString(), function(obj,deltaInfo) {
                  
                    if(!deltaInfo.IsOK)
                    {
                      closeWaitDialog();
                      return;
                    }                  
                          
                    var delta = new Delta(deltaInfo.Params[3],deltaInfo.Params[4],deltaInfo.Params[5],deltaInfo.Params[6],deltaInfo.Params[7]);
                    deltaList.Add(delta);
                    if(deltaList.List.length == cnt)
                    {
                        closeWaitDialog();
                        
                        // заполняем список дельт
                        deltaView.fillList('#DELTA_LIST');
                    }
                  
                  });
            
            } // for
     
     });

}
//-----------------------------------------------------------------------------------------------------
// обработчик онлайн-статуса контроллера
controller.OnStatus = function(obj)
{
  var is_online = controller.IsOnline();
    
  $('#reset_controller_link, #controller_time_button').toggle(is_online);  
  
  
  if(is_online)
  { 
    $('#wait_block').hide();
    $('#offline_block').hide();
    $('#online_block').show();
  }
  else
  {
    $('#wait_block').hide();
    $('#offline_block').show();
    $('#online_block').hide();
  }
};
//-----------------------------------------------------------------------------------------------------
// редактируем настройки канала
function editWateringChannel(channel, row)
{

      var lst = $('#water_channels_days').find('div #watering_channel_day');

      for(var i=0;i<lst.length;i++)
      {
         var elem = lst.get(i);
          var dayMask = parseInt(elem.value);
          elem.checked = (channel.WateringDays & dayMask) == dayMask;
      } // for
      
      $('#watering_start_hour').val(parseInt(channel.StartTime/60)); 
      $('#watering_start_minute').val(parseInt(channel.StartTime%60)); 
      $('#watering_time').val(channel.WateringTime); 
      $('#watering_sensor_index').val(channel.WateringSensorIndex); 
      $('#watering_stop_border').val(channel.WateringStopBorder); 
      $('#watering_start_border').val(channel.WateringStartBorder); 



 $("#water_channel_settings_dialog").dialog({modal:true, buttons: [{text: "Изменить", click: function(){

      $(this).dialog("close");
      
      var watering_start_hour = parseInt($('#watering_start_hour').val());
      if(isNaN(watering_start_hour))
        watering_start_hour = parseInt(channel.StartTime/60);

      var watering_start_minute = parseInt($('#watering_start_minute').val());
      if(isNaN(watering_start_minute))
        watering_start_minute = parseInt(channel.StartTime%60);
      
      watering_start_hour = Math.abs(watering_start_hour);
      if(watering_start_hour > 23)
        watering_start_hour = 23;
        
      channel.StartTime = watering_start_hour*60 + watering_start_minute;
      
      
      var watering_time = parseInt($('#watering_time').val());
      if(isNaN(watering_time))
        watering_time = channel.WateringTime;
        
       channel.WateringTime = Math.abs(watering_time);

      var watering_sensor_index = parseInt($('#watering_sensor_index').val());
      if(isNaN(watering_sensor_index))
        watering_sensor_index = channel.WateringSensorIndex;
        
       channel.WateringSensorIndex = (watering_sensor_index);

      var watering_stop_border = parseInt($('#watering_stop_border').val());
      if(isNaN(watering_stop_border))
        watering_stop_border = channel.WateringStopBorder;
        
      if(watering_stop_border > 100)
        watering_stop_border = 100;
       
       if(watering_stop_border < 0)
        watering_stop_border = 0;
        
       channel.WateringStopBorder = (watering_stop_border);
	   
      var watering_start_border = parseInt($('#watering_start_border').val());
      if(isNaN(watering_start_border))
        watering_start_border = channel.WateringStartBorder;
        
      if(watering_start_border > 100)
        watering_start_border = 100;
       
       if(watering_start_border < 0)
        watering_start_border = 0;
        
       channel.WateringStartBorder = (watering_start_border);	   

        
       channel.WateringDays = 0;
        
       var lst = $('#water_channels_days').find('div #watering_channel_day');

        for(var i=0;i<lst.length;i++)
        {
           var elem = lst.get(i);
           if(elem.checked)
           {
            var dayMask = parseInt(elem.value);
             channel.WateringDays |= dayMask;
           }
           
        } // for 

      row.find('#water_channel_days').html(channel.getWateringDaysString(true));
      row.find('#water_channel_start').html(formatWateringStartTime(channel.StartTime));
      row.find('#water_channel_time').html(channel.WateringTime + ' мин');
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });
}
//-----------------------------------------------------------------------------------------------------
function formatWateringStartTime(tm)
{
  var hour = parseInt(tm/60);
  var minute = parseInt(tm%60);
  
  var res = '';
  if(hour < 10)
    res += '0';
    
  res += hour;
  
  res += ':';
  
  if(minute < 10)
    res += '0';
    
   res += minute;
   
   return res;
}
//-----------------------------------------------------------------------------------------------------
// добавляет строку в таблицу настроек каналов
function addWaterChannelRow(parentElement, channel, num)
{
  if(!channel)
  {
    // запросили заголовок
    var row = $('<div/>',{'class': 'row', id: 'water_channels_header'});
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("#").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Дни недели").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Начало").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Время").appendTo(row);
    
    row.appendTo(parentElement);
    return;
  }
  
    var row = $('<div/>',{'class': 'row', id: 'water_channel_' + num});
    $('<div/>',{'class': 'row_item', id: 'water_channel_index'}).html(num + 1).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'water_channel_days'}).html(channel.getWateringDaysString(true)).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'water_channel_start'}).html(formatWateringStartTime(channel.StartTime)).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'water_channel_time'}).html(channel.WateringTime + ' мин').appendTo(row);
    
    var actions = $('<div/>',{'class': 'row_item actions', id: 'actions'}).appendTo(row);
    
    $('<div/>',{'class': 'action', title: 'Редактировать настройки канала'}).appendTo(actions).button({
      icons: {
        primary: "ui-icon-pencil"
      }, text: false
    }).click({row: row, channel : channel, channel_index: num}, function(ev){
              
              editWateringChannel(ev.data.channel,ev.data.row);
              
      
              });    
    
    row.appendTo(parentElement);
  
}
//-----------------------------------------------------------------------------------------------------
// заполняем таблицу настроек каналов полива
function fillWaterChannelsList()
{
  addWaterChannelRow('#WATER_CHANNELS_LIST', null);
  
  for(var i=0;i<wateringSettings.Channels.length;i++)
  {
    var channel = wateringSettings.Channels[i];
    addWaterChannelRow('#WATER_CHANNELS_LIST', channel, i);
  } // for
}
//-----------------------------------------------------------------------------------------------------
// возвращает строку с маской рабочих дней для правила
function getRuleDaymaskString(daymask)
{
var result = '';
  for(var i=0;i<7;i++)
  {
    if(BitIsSet(daymask,i))
    {
      if(result.length)
        result += ',';
        
      result += SHORT_WEEKDAYS[i];
    }
  } // for
  
  return result;
}
//-----------------------------------------------------------------------------------------------------
// добавляет строку в таблицу правил
function addRuleRow(parentElement, rule, num)
{
  if(!rule)
  {
    // запросили заголовок
    var row = $('<div/>',{'class': 'row', id: 'rules_list_header'});
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("#").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Имя").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Начало").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Активно").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Дни").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Следим за").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Действие").appendTo(row);
    
    row.appendTo(parentElement);
    return;
  }
  
    var row = $('<div/>',{'class': 'row', id: 'rule_' + num});
    $('<div/>',{'class': 'row_item', id: 'rule_index'}).html(num + 1).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'rule_name'}).html(rule.Name).appendTo(row);
    
    var rh = parseInt(rule.StartTime/60);
    if(rh < 10)
      rh = '0' + rh;
    var rm = parseInt(rule.StartTime%60);
    if(rm < 10)
      rm = '0' + rm;
      
    var ruleStartTime = rh + ':' + rm;
    
    $('<div/>',{'class': 'row_item', id: 'rule_start'}).html(ruleStartTime).appendTo(row);
    
    $('<div/>',{'class': 'row_item', id: 'rule_time'}).html(rule.WorkTime + ' мин').appendTo(row);
    
    // добавляем дни активности
    $('<div/>',{'class': 'row_item', id: 'rule_daymask'}).html(getRuleDaymaskString(rule.DayMask)).appendTo(row);
    
    $('<div/>',{'class': 'row_item', id: 'rule_target'}).html(rule.getTargetDescription()).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'rule_command'}).html(rule.getTargetCommandDescription()).appendTo(row);
    
    var actions = $('<div/>',{'class': 'row_item actions', id: 'actions'}).appendTo(row);
    
    $('<div/>',{'class': 'action', title: 'Редактировать правило'}).appendTo(actions).button({
      icons: {
        primary: "ui-icon-pencil"
      }, text: false
    }).click({row: row, rule : rule, rule_index: num}, function(ev){
              
             newRule(ev.data.rule,ev.data.row);
              
      
              });
              
    $('<div/>',{'class': 'action', title: 'Удалить правило'}).appendTo(actions).button({
      icons: {
        primary: "ui-icon-close"
      }, text: false
    }).click({row: row, rule : rule}, function(ev){
              
                ev.data.row.remove();
                rulesList.Rules.remove(ev.data.rule);
                
              });                  
    
    row.appendTo(parentElement);
    
  
}
//-----------------------------------------------------------------------------------------------------
function addReservationRow(parentElement, reservation, num)
{
  
    var row = $('<div/>',{'class': 'row reservation', id: 'reservation_' + num});
    $('<div/>',{'class': 'row_item', id: 'reservation_index'}).html(num + 1).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'reservation_type'}).html(reservation.getTypeString()).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'reservation_sensors'}).html(reservation.getSensorsString()).appendTo(row);
    
        
    var actions = $('<div/>',{'class': 'row_item actions', id: 'actions'}).appendTo(row);
    
              
    $('<div/>',{'class': 'action', title: 'Удалить список резервирования'}).appendTo(actions).button({
      icons: {
        primary: "ui-icon-close"
      }, text: false
    }).click({row: row, reservation : reservation}, function(ev){
              
                ev.data.row.remove();
                reservations.Items.remove(ev.data.reservation);
                
                $('#RESERVATION_LIST').find('div #reservation_index').each(function(idx,elem){
                
                  $(elem).html(idx + 1);
                
                });
                                
              });                  
    
    row.appendTo(parentElement);  
  
}
//-----------------------------------------------------------------------------------------------------
function fillReservationsList()
{
  
  for(var i=0;i<reservations.Items.length;i++)
  {
      addReservationRow('#RESERVATION_LIST',reservations.Items[i],i);
  } // for
  
}
//-----------------------------------------------------------------------------------------------------
// добавляет строку в таблицу SMS
function addSMSRow(parentElement, sms, num)
{
  if(!sms)
  {
    // запросили заголовок
    var row = $('<div/>',{'class': 'row', id: 'sms_list_header'});
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("#").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Текст СМС").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Ответ от контроллера").appendTo(row);
    $('<div/>',{'class': 'row_item ui-widget-header'}).html("Команда").appendTo(row);    
    row.appendTo(parentElement);
    return;
  }
  
    var row = $('<div/>',{'class': 'row sms', id: 'sms_' + num});
    $('<div/>',{'class': 'row_item', id: 'sms_index'}).html(num + 1).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'sms_text'}).html(sms.SMSText).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'sms_answer'}).html(sms.AnswerText).appendTo(row);
    $('<div/>',{'class': 'row_item', id: 'sms_command'}).html(sms.Command).appendTo(row);
    
        
    var actions = $('<div/>',{'class': 'row_item actions', id: 'actions'}).appendTo(row);
    
              
    $('<div/>',{'class': 'action', title: 'Удалить SMS'}).appendTo(actions).button({
      icons: {
        primary: "ui-icon-close"
      }, text: false
    }).click({row: row, sms : sms}, function(ev){
              
                ev.data.row.remove();
                smsList.List.remove(ev.data.sms);
                
                controller.queryServerScript("/x_delete_sms.php",{sms_text: ev.data.sms.SMSText}, function(obj,result){});
                
              });                  
    
    row.appendTo(parentElement);
    
  
}
//-----------------------------------------------------------------------------------------------------
var __globalRuleDaymask = 0xFF;
// показывает диалог редактирования правил
function adjustRuleDaymask()
{
    
      var lst = $('#rule_daymask_box').find('div #rule_work_day');

      for(var i=0;i<lst.length;i++)
      {
        var elem = lst.get(i);
        var d = parseInt(elem.value);
        elem.checked = (__globalRuleDaymask & d) == d;
      } // for  
      
      $("#rule_daymask_dialog").dialog({modal:true, buttons: [
      
        {
          text: "OK", click: function()
          {
              var lst = $('#rule_daymask_box').find('div #rule_work_day');
              __globalRuleDaymask = 0;
              for(var i=0;i<lst.length;i++)
              {
                 var elem = lst.get(i);
                 if(elem.checked)
                 {
                  var dayMask = parseInt(elem.value);
                   __globalRuleDaymask |= dayMask;
                 }
              } // for 
              
              $(this).dialog('close');             
          } 
        } 
        ,{
          text: "Отмена", click: function(){ $(this).dialog('close');} 
        } 
      ] 
      
      });    



}
//-----------------------------------------------------------------------------------------------------
// создаёт новое правило
function newRule(editedRule, editedRow)
{

  if(rulesList.Rules.length >= MAX_RULES)
  {
    showMessage('Достигнуто максимальное количество правил!');
    return;
  }

  $('#rule_target_input').val('_').trigger('change');
  $('#rule_name_input').val('');
  $('#rule_name_input').removeAttr('disabled');
  $('#rule_start_hour_input').val('0');
  $('#rule_start_minute_input').val('0');
  
  $('#rule_work_time_input').val('0');
  $('#rule_sensor_index_input').val('0');
  $('#rule_pin_number').val('0');
  $('#rule_sensor_value_box').val('0');
  $('#linked_rules_box').empty();
  $('#rule_sensor_value_input').val('');
  $('#rule_action_input').val('0').trigger('change');
  $('#rule_additional_param_input').val('');
  $('#rule_wnd_interval_input').val('');
  $('#is_alarm_rule').get(0).checked = false;
  
  __globalRuleDaymask = 0xFF;

  
  var newRuleRequested = !editedRule;
  
  
      
  for(var i=0;i<rulesList.Rules.length;i++)
  {
    var rule = rulesList.Rules[i];
    
    if(editedRule && editedRule.Name == rule.Name)
      continue;
       
    
    var row = $('<div/>');
    var chb = $('<input/>',{type: 'checkbox', id: 'linked_rule_index', value: i});
    chb.appendTo(row);
        
    if(editedRule && editedRule.LinkedRules.includes(rule.Name))
      chb.get(0).checked = true;
    
    $('<label/>').appendTo(row).html(rule.Name + ': ' + rule.getTargetCommandDescription());
    
    row.appendTo('#linked_rules_box');
  } // for
  
  if(editedRule)
  {
    // запросили редактирование правила, заполняем поля в форме
    $('#rule_name_input').attr('disabled','disabled');
    $('#rule_name_input').val(editedRule.Name);
    
    var targ = editedRule.Target.toString();
    if(targ == '0')
      targ = '_';
    
    $('#rule_target_input').val(targ).trigger('change');
    $('#rule_module_select').val(editedRule.ModuleName).trigger('change');
    $('#rule_start_hour_input').val(parseInt(editedRule.StartTime/60));
    $('#rule_start_minute_input').val(parseInt(editedRule.StartTime%60));
    $('#rule_work_time_input').val(editedRule.WorkTime);
    $('#rule_sensor_index_input').val(editedRule.SensorIndex);
    $('#rule_pin_number').val(editedRule.SensorIndex);    
    $('#rule_sensor_operand').val(editedRule.Operand);
    $('#rule_sensor_value_input').val(editedRule.AlertCondition);
    $('#rule_pin_state_input').val(editedRule.Operand);
    
    $('#is_alarm_rule').get(0).checked = editedRule.IsAlarm;
    
    var tci = editedRule.getTargetCommandIndex();
    $('#rule_action_input').val(tci).trigger('change');
    $('#rule_additional_param_input').val(editedRule.getAdditionalParam());
		
    if(tci == 0) // открыть окна, надо искать интервал
    {
      var tcSplitted = editedRule.TargetCommand.split("|");
      var passedInterval = parseInt(tcSplitted[tcSplitted.length-1]);
      if(isNaN(passedInterval))
        passedInterval = 0;
        
        passedInterval /= 1000;
        
      $('#rule_wnd_interval_input').val(passedInterval);
        
    }
	else
	if(tci == 9 || tci == 10) // вкл/выкл MCP
	{
			$('#rule_wnd_interval_input').val(editedRule.getAdditionalParam2());
	}
    
    
    __globalRuleDaymask = editedRule.DayMask;
    
  }  
      


 $("#rule_edit_dialog").dialog({modal:true, width:600, buttons: [{text: "OK", click: function(){
  
      var ruleName = $('#rule_name_input').val();
      if(ruleName == '')
      {
        showMessage('Укажите имя правила!');
        $('#rule_name_input').focus();
        return;
      }
      
      ruleName = ruleName.toUpperCase();
      
      var ruleTarget = $('#rule_target_input').val();
      var ruleStartHour = parseInt($('#rule_start_hour_input').val());
      if(isNaN(ruleStartHour))
        ruleStartHour = 0;

      var ruleStartMinute = parseInt($('#rule_start_minute_input').val());
      if(isNaN(ruleStartMinute))
        ruleStartMinute = 0;
        
      var ruleStartTime = ruleStartHour*60 + ruleStartMinute;
        
      var ruleWorkTime = parseInt($('#rule_work_time_input').val());
      if(isNaN(ruleWorkTime))
        ruleWorkTime = 0;
        
      var ruleDaymask = __globalRuleDaymask;
        
      var sensorIndex = parseInt($('#rule_sensor_index_input').val());
      if(ruleTarget == 'PIN')
      {
        sensorIndex = parseInt($('#rule_pin_number').val())   
      }
      
      if(isNaN(sensorIndex))
        sensorIndex = 0;
        
      var targetPinState = $('#rule_pin_state_input').val();
      
      var moduleName = $('#rule_module_select').val();
      
      var operand = $('#rule_sensor_operand').val();
      var alertCondition = $('#rule_sensor_value_input').val();
      
      var rule_action_input = parseInt($('#rule_action_input').val());
      var rule_additional_param_input = $('#rule_additional_param_input').val();
      
      var linked_rules = '';
      
      var lst = $('#linked_rules_box').find('div #linked_rule_index');

      for(var i=0;i<lst.length;i++)
      {
         var elem = lst.get(i);
         if(elem.checked)
         {
          if(linked_rules != '')
            linked_rules += ',';
            
            linked_rules += rulesList.Rules[elem.value].Name;
         }
     }
     
     if(linked_rules == '')
      linked_rules = '_';
      
      
      var isalarm = $('#is_alarm_rule').get(0).checked ? '1' : '0';
        
      
      if(ruleTarget != '_')
      {
        // если следим за чем-то, то проверяем параметры
        if(ruleTarget == 'PIN')
        {
            alertCondition = '1';
            moduleName = '0';
            operand = targetPinState;
        }
        else
        {
          if(alertCondition == '')
          {
            showMessage('Введите показания датчика!');
            return;
          }
        }
      }
      else
      {
        // когда ни за чем не следим, ссылаемся на модуль "0"
          moduleName = '0';
          alertCondition = '0';
          operand = '>';
          
      }
      
       if((rule_action_input == 0 || rule_action_input == 1 || rule_action_input == 4 || rule_action_input == 5 || rule_action_input == 6
	   || rule_action_input == 7 || rule_action_input == 8 || rule_action_input == 9 || rule_action_input == 10) 
       && rule_additional_param_input == '')
       {
        showMessage('Укажите дополнительные параметры!');
        $('#rule_additional_param_input').focus();
        return;
       }
       
       var targetCommand = rulesList.buildTargetCommand(rule_action_input,rule_additional_param_input, $('#rule_wnd_interval_input').val());
       
       if(rule_action_input == 0) // открыть окна
       {
        var wndInt = parseInt($('#rule_wnd_interval_input').val());
        if(isNaN(wndInt))
          wndInt = 0;
          
          if(wndInt > 0)
            targetCommand += '|' + (wndInt*1000);
       }
                     
       // вроде всё проверили, пытаемся посмотреть
       var fullRuleString = 'dummy|dummy|dummy|' + ruleName + '|' + moduleName + '|' + ruleTarget + '|' + sensorIndex + '|' + operand + '|' + alertCondition + '|' + 
       ruleStartTime + '|' + ruleWorkTime + '|' + ruleDaymask + '|' + linked_rules + '|' + isalarm + '|' + targetCommand;
              
       if(newRuleRequested)
       {
        // новое правило
        editedRule = rulesList.Add();
        editedRule.Construct(fullRuleString.split('|'));
        addRuleRow('#RULES_LIST', editedRule, rulesList.Rules.length - 1);
       }
       else
       {
        // редактируем правило
        
        editedRule.Construct(fullRuleString.split('|'));
        editedRow.find('#rule_name').html(editedRule.Name);
        
        var rh = parseInt(editedRule.StartTime/60);
        if(rh < 10)
          rh = '0' + rh;
        var rm = parseInt(editedRule.StartTime%60);
        if(rm < 10)
          rm = '0' + rm;
          
        var ruleStartTime = rh + ':' + rm;
            
        editedRow.find('#rule_start').html(ruleStartTime);
        editedRow.find('#rule_time').html(editedRule.WorkTime + ' мин');
        editedRow.find('#rule_target').html(editedRule.getTargetDescription());
        editedRow.find('#rule_command').html(editedRule.getTargetCommandDescription());
        editedRow.find('#rule_daymask').html(getRuleDaymaskString(editedRule.DayMask));
       }
      

      $(this).dialog("close");
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });     
  
}
//-----------------------------------------------------------------------------------------------------
function newSms()
{
  $('#sms_add_form').get(0).reset();
  
  $("#new_sms_dialog").dialog({modal:true, width:350, buttons: [{text: "OK", click: function(){
  
    var txt = $('#sms_text_input').val().trim();
    var ans = $('#sms_answer_input').val().trim();
    
    
    if(txt == '' || ans == '')
    {
      showMessage("Пожалуйста, укажите текст СМС и ответ контроллера!");
      return;
    }
    
    var list_index = parseInt($('#sms_commands_list').val());
    var cmd = SMS_BUILD_COMMANDS[list_index];
    
    var wantParam = cmd[1];
    var smsParam = $('#sms_param_text').val().trim();
    if(wantParam && smsParam == '')
    {
      showMessage('Пожалуйста, укажите дополнительный параметр!');
      return;
    }
    
    var targetCommand = cmd[0];
    
    if(wantParam)
      targetCommand = cmd[0].replace('{0}',smsParam);
      
    var addedSms = smsList.Add(txt,ans,targetCommand);
    
    // теперь ищем, есть ли такое же в списке
    var found = false;
    $('#SMS_LIST').find('DIV.row.sms').each(function(idx,elem){
    
      var e  = $(elem).find('DIV#sms_text');
      if(e.html() == txt)
      {
        found = true;
        $(elem).find('DIV#sms_answer').html(ans);
        $(elem).find('DIV#sms_command').html(targetCommand);
      }
    
    });
        
    if(!found) // новое СМС
    {
      addSMSRow('#SMS_LIST', addedSms, smsList.List.length-1);
    }
    
    controller.queryServerScript("/x_add_sms.php",{sms_text: txt, sms_answer: ans, sms_command: targetCommand}, function(obj,result){});
    
    $(this).dialog('close');
  
  } } ] 
  
  });

}
//-----------------------------------------------------------------------------------------------------
// сохраняем список правил
function saveRulesList()
{

    showWaitDialog();
              
     var rulesToProcess = rulesList.Rules.length;
     var processedRules = 0;
              
     controller.queryCommand(false,"ALERT|RULE_DELETE|ALL",function(obj,answer){
     
          for(var i=0;i<rulesList.Rules.length;i++)
          {
            var rule = rulesList.Rules[i];
            var cmd = 'ALERT|RULE_ADD|' + rule.getAlertRule();
            controller.queryCommand(false,cmd, function(obj, processResult){
            
              processedRules++;
              if(processedRules >= rulesToProcess)
              {
                  controller.queryCommand(false,"ALERT|SAVE", function(obj,answer) {
                
                            closeWaitDialog();
                            if(answer.IsOK)
                              showMessage("Данные успешно сохранены!");
                            else
                              showMessage("Ошибка сохранения данных :(");                      
                
                });
              }
            
            });
          } // for
          
          
          if(!rulesToProcess)
          {
             controller.queryCommand(false,"ALERT|SAVE", function(obj,answer) {
                
                            closeWaitDialog();
                            if(answer.IsOK)
                              showMessage("Данные успешно сохранены!");
                            else
                              showMessage("Ошибка сохранения данных :(");                      
                
                });
          }
     
     });

}
//-----------------------------------------------------------------------------------------------------
function saveStatSmsSettings()
{
  showWaitDialog();
  
  var module1 = $("#statsmsmodule1").val();
  var sensor1 = $("#statsmssensor1").spinner("value");
  var label1 = $("#statsmslabel1").val().trim();
  if(!label1.length)
    label1 = "_";
    
  if(label1 != "_")
  {
    // encode label 1
    var uint8Array = new TextEncoder("utf-8").encode(label1);
    
    label1 = "";
    for(var i=0;i<uint8Array.length;i++)
    {
      label1 += uint8Array[i].toString(16).toUpperCase();;
    }
    
  }
  
  var module2 = $("#statsmsmodule2").val();
  var sensor2 = $("#statsmssensor2").spinner("value");
  var label2 = $("#statsmslabel2").val().trim();
  if(!label2.length)
    label2 = "_";
    
  if(label2 != "_")
  {
    // encode label 2
    var uint8Array = new TextEncoder("utf-8").encode(label2);
    
    label2 = "";
    for(var i=0;i<uint8Array.length;i++)
    {
      label2 += uint8Array[i].toString(16).toUpperCase();;
    }
    
  }
  
  controller.queryCommand(false,"SMS|STATSENSORS|" + module1 + "|" + sensor1 + "|" + label1 + "|" + module2 + "|" + sensor2 + "|" + label2,function(obj,answer){
     
       closeWaitDialog();
        
       if(answer.IsOK)
          showMessage("Данные успешно сохранены!");
        else
          showMessage("Ошибка сохранения данных :(");           
     
     });  
    
}
//-----------------------------------------------------------------------------------------------------
function saveSmsList()
{

    showWaitDialog();
              
     var smsToProcess = smsList.List.length;
     var processedSms = 0;
          
     // на будущее оставим такую команду для удаления всех СМС    
     controller.queryCommand(false,"SMS|DELETE|ALL",function(obj,answer){
     
          for(var i=0;i<smsList.List.length;i++)
          {
            var sms = smsList.List[i];
            var cmd = 'SMS|ADD|' + sms.getSMSCommand();
            controller.queryCommand(false,cmd, function(obj, processResult){
            
              processedSms++;
              if(processedSms >= smsToProcess)
              {
                  
                
                    closeWaitDialog();
                    if(processResult.IsOK)
                      showMessage("Данные успешно сохранены!");
                    else
                      showMessage("Ошибка сохранения данных :(");                      
                
               
              }
            
            });
          } // for
          
          
          if(!smsToProcess)
          {
            closeWaitDialog();
            
          }
     
     });

}
//-----------------------------------------------------------------------------------------------------
// заполняем список правил
function fillRulesList()
{
  $('#RULES_LIST').html('');
  addRuleRow('#RULES_LIST', null);
  
  for(var i=0;i<rulesList.Rules.length;i++)
  {
    var rule = rulesList.Rules[i];
    addRuleRow('#RULES_LIST', rule, i);
  } // for  
}
//-----------------------------------------------------------------------------------------------------
// заполняем список СМС
function fillSMSList()
{
  $('#SMS_LIST').html('');
  addSMSRow('#SMS_LIST', null);
  
  for(var i=0;i<smsList.List.length;i++)
  {
    var sms = smsList.List[i];
    addSMSRow('#SMS_LIST', sms, i);
  } // for  
}
//-----------------------------------------------------------------------------------------------------
// сохраняем настройки для всех каналов полива одновременно
var wateringTotalCommands = 1;
var wateringProcessedCommands = 0;

function saveAllChannelsWateringOptions(watering_option)
{
  wateringSettings.WateringOption = watering_option;
  wateringTotalCommands = 1;
  wateringProcessedCommands = 0;
    
  if(watering_option == 1) // все каналы сразу
  {
      var wTime = parseInt($('#all_watering_time').val());
      if(!isNaN(wTime))
        wateringSettings.WateringTime = Math.abs(wTime);
      else
        $('#all_watering_time').val(wateringSettings.WateringTime);

      var wIndex = parseInt($('#all_watering_sensor_index').val());
      if(!isNaN(wIndex))
        wateringSettings.WateringSensorIndex = (wIndex);
      else
        $('#all_watering_sensor_index').val(wateringSettings.WateringSensorIndex);

      var wBorder = parseInt($('#all_watering_stop_border').val());
      if(!isNaN(wBorder))
      {
        if(wBorder < 0)
          wBorder = 0;
        
        if(wBorder > 100)
          wBorder = 100;
          
        wateringSettings.WateringStopBorder = (wBorder);
      }
      else
        $('#all_watering_stop_border').val(wateringSettings.WateringStopBorder);
		
		
      wBorder = parseInt($('#all_watering_start_border').val());
      if(!isNaN(wBorder))
      {
        if(wBorder < 0)
          wBorder = 0;
        
        if(wBorder > 100)
          wBorder = 100;
          
        wateringSettings.WateringStartBorder = (wBorder);
      }
      else
        $('#all_watering_start_border').val(wateringSettings.WateringStartBorder);		

        
      var wStart = parseInt($('#all_watering_start_hour').val());
      var wStartMinute = parseInt($('#all_watering_start_minute').val());
      
      if(isNaN(wStartMinute))
        wStartMinute = 0;
        
        wStartMinute = Math.abs(wStartMinute);
        if(wStartMinute > 59)
          wStartMinute = 59;
          
       $('#all_watering_start_minute').val(wStartMinute);
      
      if(!isNaN(wStart))
      {
        wStart = Math.abs(wStart);
        if(wStart > 23)
          wStart = 23;
          
          $('#all_watering_start_hour').val(wStart);
          wateringSettings.StartTime = wStart*60 + wStartMinute;
          
      } // if(!isNaN(wStart))
      else
      {
        $('#all_watering_start_hour').val(parseInt(wateringSettings.StartTime/60));
      }
      

      // теперь сохраняем дни недели
      wateringSettings.WateringDays = 0;
      
      var lst = $('#all_watering_channels_days').find('div #all_watering_channels_day');

      for(var i=0;i<lst.length;i++)
      {
         var elem = lst.get(i);
         if(elem.checked)
         {
          var dayMask = parseInt(elem.value);
           wateringSettings.WateringDays |= dayMask;
         }
      } // for 

   } // watering_option == 1 
   
    showWaitDialog(); 
   
   if(watering_option == 2)
   {
      wateringTotalCommands += wateringSettings.Channels.length;
    
     
     for(var i=0;i<wateringSettings.Channels.length;i++)
     {
        var channel = wateringSettings.Channels[i];
        var cmd = "WATER|CH_SETT|" + i + '|' + channel.WateringDays + '|' + channel.WateringTime + '|' + channel.StartTime
        + '|' + channel.WateringSensorIndex + '|' + channel.WateringStopBorder + '|' + channel.WateringStartBorder;
        
        controller.queryCommand(false,cmd,function(obj,answer){
                
                      wateringProcessedCommands++;
                      
                      if(wateringProcessedCommands >= wateringTotalCommands) {
                        closeWaitDialog();

                        if(answer.IsOK)
                          showMessage("Данные успешно сохранены!");
                        else
                          showMessage("Ошибка сохранения данных :(");                      

                        }
                                      
                });         
        
     } // for

    
   } // if(watering_option == 2)
   
   wateringSettings.TurnOnPump = $('#turn_on_pump').get(0).checked ? 1 : 0;
   wateringSettings.SwitchToAutoAfterMidnight = $('#switch_w_to_aam').get(0).checked ? 1 : 0;
              
    // теперь можем загружать всё это добро в контроллер
    var cmd = "WATER|T_SETT|" + wateringSettings.WateringOption + '|' +   wateringSettings.WateringDays + '|' +
               wateringSettings.WateringTime + '|' + wateringSettings.StartTime + '|' + wateringSettings.TurnOnPump
               + '|' + wateringSettings.WateringSensorIndex + '|' + wateringSettings.WateringStopBorder + '|' +
               wateringSettings.SwitchToAutoAfterMidnight + '|' + wateringSettings.WateringStartBorder;
               
//     console.log(cmd);
                                  
                controller.queryCommand(false,cmd,function(obj,answer){
                
                      wateringProcessedCommands++;
                      
                      if(wateringProcessedCommands >= wateringTotalCommands) {
                        closeWaitDialog();
                        if(answer.IsOK)
                          showMessage("Данные успешно сохранены!");
                        else
                          showMessage("Ошибка сохранения данных :(");                           
                        }
                                      
                }); 
    
    
    
}
//-----------------------------------------------------------------------------------------------------
// сохраняем настройки полива
function saveWateringSettings()
{
  var watering_option = parseInt($('#watering_option').val());  
   saveAllChannelsWateringOptions(watering_option);
}
//-----------------------------------------------------------------------------------------------------
function doRequestRulesList()
{
  showWaitDialog();
              
    requestRulesList(function(){
    
      closeWaitDialog();
    
    });
              
}
//-----------------------------------------------------------------------------------------------------
// запрашивает список правил
function requestRulesList(doneFunc)
{
    rulesList.Clear();
    $('#RULES_LIST').html('');
    
    var rulesCnt = 0;
    
     controller.queryCommand(true,'ALERT|RULES_CNT',function(obj,answer){
           
          
          if(answer.IsOK)
          {
            rulesCnt = parseInt(answer.Params[2]);
            for(var i=0;i<rulesCnt;i++)
            {
                var cmd = 'ALERT|RULE_VIEW|' + i;
                
                controller.queryCommand(true,cmd,function(obj,ruleSettings){
                
                  if(ruleSettings.IsOK)
                  {                    
                    var rule = rulesList.Add();
                    rule.Construct(ruleSettings.Params);
                    
                    if(rulesCnt == rulesList.Rules.length)
                    {
                      fillRulesList();
                      
                      if(doneFunc)
                        doneFunc();
                    }
                  } // is ok
                
                });
            } // for
            
            if(!rulesCnt)
            {
              if(doneFunc)
                doneFunc();
            } // !rulesCnt
            
          } // is ok
        
        });
    

}
//-----------------------------------------------------------------------------------------------------
var totalTempSensors = 0; // кол-во температурных датчиков в прошивке
var totalHumiditySensors = 0; // кол-во датчиков влажности в прошивке
var totalLuminositySensors = 0; // кол-во датчиков освещённости в прошивке
var totalSoilMoistureSensors = 0; // кол-во датчиков влажности в прошивке
var totalPHSensors = 0; // кол-во датчиков pH в прошивке
//-----------------------------------------------------------------------------------------------------
// событие "Получен список модулей в прошивке"
controller.OnGetModulesList = function(obj)
{  

  showWaitDialog();
  
  
    controller.queryCommand(true,'0|LIMITS',function(obj,answer){
         
    if(answer.IsOK)
    {
      MAX_RULES = parseInt(answer.Params[1]);
      MAX_DELTAS = parseInt(answer.Params[2]);
    }
  
  });

    var hasDeltaModule = controller.Modules.includes('DELTA');
    $('#DELTA_MENU').toggle(hasDeltaModule); // работаем с дельтами только если в прошивке есть модуль дельт
    if(hasDeltaModule)
    {
      // настраиваем диалог добавления дельт
      $('#delta_type').find('option').each(function(idx,elem)
      {
        var moduleName = elem.value;
        if(moduleName == 'TEMP')
          moduleName = 'STATE';
          
        if(!controller.Modules.includes(moduleName))
          $(elem).remove();
      });
      
      // показываем меню дельт, если есть хотя бы из чего-нибудь получать дельты
      $('#DELTA_MENU').toggle($('#delta_type').children().length > 0);
      
      
    }
    
    // настраиваем диалог добавления нового правила
      $('#rule_target_input').find('option').each(function(idx,elem)
      {
        var moduleName = elem.value;

        if(moduleName == 'PIN' || moduleName == '_')
          return;

        if(moduleName == 'TEMP')
          moduleName = 'STATE';
          
          
        if(!controller.Modules.includes(moduleName))
          $(elem).remove();
      });    
    
    
    var hasSMSModule= controller.Modules.includes('SMS');
    
    if(hasSMSModule) // если в прошивке есть модуль Neoway M590
    {
    
    
        controller.queryCommand(true,'0|PHONE',function(obj,answer){
           
          $('#phone_number').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
            $('#edit_phone_number').val(answer.Params[1]);
          }
        
        });
        
        controller.queryCommand(true,'SMS|STATSENSORS',function(obj,answer){
               
           $('#statsmssettings').toggle(answer.IsOK);
                     
          if(answer.IsOK)
          {
            var module1 = parseInt(answer.Params[2]);
            var sensor1 = parseInt(answer.Params[3]);
            var label1Encoded =  answer.Params[4];
            if(label1Encoded == "_")
              label1Encoded = "";

            var module2 = parseInt(answer.Params[5]);
            var sensor2 = parseInt(answer.Params[6]);
            var label2Encoded =  answer.Params[7].trim();
            if(label2Encoded == "_")
              label2Encoded = "";
                
            
            var label1 = "";
            var label2 = "";
            
            if(label1Encoded.length > 1)
            {
              var uintArray = new Array();
              
              for(var i=0;i<label1Encoded.length;i+=2)
              {
                uintArray.push(parseInt(label1Encoded.substr(i,2),16));
              }
              /*
              var enc = String.fromCharCode.apply(null,uintArray);              
              label1 = decodeURIComponent(escape(enc));
             */ 
              
              var arrBuff = new ArrayBuffer(uintArray.length);
              var bw = new Uint8Array(arrBuff);
              for(var i=0;i<uintArray.length;i++)
                bw[i] = uintArray[i];
                
              label1 = new TextDecoder().decode(arrBuff);
            }
            
            if(label2Encoded.length > 1)
            {
              var uintArray = new Array();
              
              for(var i=0;i<label2Encoded.length;i+=2)
              {
                uintArray.push(parseInt(label2Encoded.substr(i,2),16));
              }

              
              var arrBuff = new ArrayBuffer(uintArray.length);
              var bw = new Uint8Array(arrBuff);
              for(var i=0;i<uintArray.length;i++)
                bw[i] = uintArray[i];
                
              label2 = new TextDecoder().decode(arrBuff);
            }      
            
            $('#statsmsmodule1').val(module1);
            $('#statsmsmodule2').val(module2);
            
            $('#statsmssensor1').spinner('value',sensor1);
            $('#statsmssensor2').spinner('value',sensor2);
            
            $('#statsmslabel1').val(label1);
            $('#statsmslabel2').val(label2);
            
            
          }

        
        });        
        
        controller.queryCommand(true,'SMS|PROV',function(obj,answer){
                     
          if(answer.IsOK)
          {
            $('#gsm_provider').get(0).selectedIndex = answer.Params[2];
          }
        
        });        
        
        controller.queryServerScript("/x_get_sms.php",{}, function(obj,result){
           
          $('#SMS_MENU').toggle(true);
          var answer = result.sms_list;
          smsList.Clear();
          
          for(var i=0;i<answer.length;i++)
          {            
            var sms = answer[i];
           
            smsList.Add(sms.sms_text, sms.sms_answer, sms.sms_command);
            
          } // for
          fillSMSList();
          
          // тут заполняем список команд для СМС
          $('#sms_commands_list').html('');
          // 
          for(var i=0;i<SMS_BUILD_COMMANDS.length;i++)
          {
            var cmd = SMS_BUILD_COMMANDS[i];
            $('#sms_commands_list').append($('<option/>',{value: i }).text(cmd[2]));
          } // for
          
           $('#sms_commands_list').change(function(){
          
            var list_index = parseInt($(this).val());
            
            var cmd = SMS_BUILD_COMMANDS[list_index];
            $('#sms_param').toggle(cmd[1]);
            $('#sms_param_caption').html(cmd[3]);
            
          });
                     
        });       
        
    } // includes('SMS'
    
    if(controller.Modules.includes('SCN'))
    {
      $('#SCENE_MENU').toggle(true);
      controller.queryCommand(true,'SCN',function(obj,answer){
          if(answer.IsOK)
          {
            var scCount = parseInt(answer.Params[1]);
            for(var i=0;i<scCount;i++)
            {
                controller.queryCommand(true,'SCN|SETT|' + i,function(o,a){
                      if(a.IsOK)
                      {
                        var encodedName = a.Params[4];
                        
                        var decodedName = "";
                        
                        if(encodedName.length > 1)
                        {
                          var uintArray = new Array();
                          
                          for(var i=0;i<encodedName.length;i+=2)
                          {
                            uintArray.push(parseInt(encodedName.substr(i,2),16));
                          }
                          
                          var arrBuff = new ArrayBuffer(uintArray.length);
                          var bw = new Uint8Array(arrBuff);
                          for(var i=0;i<uintArray.length;i++)
                            bw[i] = uintArray[i];
                            
                          decodedName = new TextDecoder().decode(arrBuff);
                        }
                        
                        $('<option/>').html(decodedName).appendTo('#scene_list');                       
                        
                      }
                });
            } // for
          }
      });
    } // scenes
    
    if(controller.Modules.includes('FLOW')) // если в прошивке есть модуль расходомеров
    {
        controller.queryCommand(true,'FLOW|T_SETT',function(obj,answer){
           
          $('#flow_calibration_button').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
            $('#flow_calibraton1').val(answer.Params[2]);
            $('#flow_calibraton2').val(answer.Params[3]);
          }
        
        });
    }  
    
    if(controller.Modules.includes('PH')) // если в прошивке есть модуль PH
    {
        controller.queryCommand(true,'PH|T_SETT',function(obj,answer){
           
          $('#ph_calibration_button').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
            $('#ph_calibraton').val(answer.Params[2]);
            $('#ph4Voltage').val(answer.Params[3]);
            $('#ph7Voltage').val(answer.Params[4]);
            $('#ph10Voltage').val(answer.Params[5]);
            $('#phTemperatureSensor').val(answer.Params[6]);
            $('#phCalibrationTemperature').val(answer.Params[7].replace(/[,]+/g,'.'));
            
            var phTarget = parseFloat(answer.Params[8])/100;
            $('#phTarget').val(phTarget);
            
            var phHisteresis = parseFloat(answer.Params[9])/100;
            $('#phHisteresis').val(phHisteresis);
            
            $('#phMixPumpTime').val(answer.Params[10]);
            $('#phReagentPumpTime').val(answer.Params[11]);
            
          }
        
        });
    }      
    
    if(controller.Modules.includes('TMR')) // если в прошивке есть модуль таймеров
    {
        controller.queryCommand(true,'TMR',function(obj,answer){
           
          $('#TIMERS_MENU').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
           
           var tmrNumber = 1;
           for(var i=0;i<16;i+=4)
           {
              var num = i+1;
              var tmrDayMaskAndEnable = parseInt(answer.Params[num]);
              var tmrPin = parseInt(answer.Params[num+1]);
              var tmrHoldOnTime = parseInt(answer.Params[num+2]);
              var tmrHoldOffTime = parseInt(answer.Params[num+3]);
              
              $('#timerPin' + tmrNumber).val(tmrPin);
              $('#timerOnMin' + tmrNumber).val(parseInt(tmrHoldOnTime/60));
              $('#timerOnSec' + tmrNumber).val(parseInt(tmrHoldOnTime%60));
              $('#timerOffMin' + tmrNumber).val(parseInt(tmrHoldOffTime/60));
              $('#timerOffSec' + tmrNumber).val(parseInt(tmrHoldOffTime%60));
              
              $('#timerEnabled' + tmrNumber).get(0).checked = (tmrDayMaskAndEnable & 128);
              $('#timerDayMask' + tmrNumber).find('input[type=checkbox]').each(function(idx,elem){
              
                elem.checked = tmrDayMaskAndEnable & (1 << idx);
              
              });
              
              
              tmrNumber++;
           } // for
          } // isOK
        
        });
    }          
    
    // запрашиваем информацию о прошивке
    controller.queryCommand(true,'0|WIRED',function(obj,answer){
           
          $('#sensors_info_button').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
            totalTempSensors += parseInt(answer.Params[1]);
            totalHumiditySensors += parseInt(answer.Params[2]);
            totalLuminositySensors += parseInt(answer.Params[3]);
            totalSoilMoistureSensors += parseInt(answer.Params[4]);
            totalPHSensors += parseInt(answer.Params[5]);
          
          }
        
        });
        
    controller.queryCommand(true,'0|UNI',function(obj,answer2){
    
        if(answer2.IsOK)
        {
            totalTempSensors += parseInt(answer2.Params[1]);
            totalHumiditySensors += parseInt(answer2.Params[2]);
            totalLuminositySensors += parseInt(answer2.Params[3]);
            totalSoilMoistureSensors += parseInt(answer2.Params[4]);
            totalPHSensors += parseInt(answer2.Params[5]);

            $('#sensors_info_temp').html(totalTempSensors);
            $('#sensors_info_humidity').html(totalHumiditySensors);
            $('#sensors_info_luminosity').html(totalLuminositySensors);
            $('#sensors_info_soil').html(totalSoilMoistureSensors);
            $('#sensors_info_ph').html(totalPHSensors);

        }
    
    });        
    
    // запрашиваем список правил
    requestRulesList(function(){ 
      
      $('#RULES_MENU').toggle(true);
    
    });
    
    var hasWIFIModule = controller.Modules.includes('WIFI');
    
    if(hasWIFIModule) // если в прошивке есть модуль wi-fi
    {
    
          controller.queryCommand(true,'WIFI|T_SETT',function(obj,answer){
                     
          if(answer.IsOK)
          {
            $('#wifi_menu').toggle(true);
             
            var checked = false;
            if(answer.Params[2] == "1")
              checked = true;
              
            $('#router_id').val(answer.Params[3]);
            $('#router_pass').val(answer.Params[4]);
            $('#station_id').val(answer.Params[5]);
            $('#station_pass').val(answer.Params[6]);
            if(checked)
              $('#connect_to_router').attr('checked', 'checked');
            else
              $('#connect_to_router').removeAttr('checked');
          }
        
        });               
        
    }
    
    if(controller.Modules.includes('HTTP') && hasWIFIModule)
    {
        controller.queryCommand(true,'HTTP|KEY',function(obj,answer){
        
            $('#HTTP_MENU').toggle(answer.IsOK);
            if(answer.IsOK) {
            
                $('#http_api_key').val(answer.Params[2]);
                $('#http_api_enabled').get(0).checked = parseInt(answer.Params[3]) == 1;
                $('#http_timezone').val(answer.Params[4]);
                $('#http_send_status').get(0).checked = parseInt(answer.Params[5]) == 1;
                $('#http_send_state').get(0).checked = parseInt(answer.Params[6]) == 1;
                
                
                $('#http_timezone').spinner({
                    min: -1000,
                    max: 1000,
                    step: 5
                });
            }
        
        });
    
    }
    
    
    if(controller.Modules.includes('IOT') && (hasWIFIModule || hasSMSModule))
    {
      // можем получать настройки IoT
        controller.queryCommand(true,'IOT|T_SETT',function(obj,answer){
           
          $('#IOT_MENU').toggle(answer.IsOK);
          
          if(answer.IsOK)
          {
              // тут парсим настройки IoT
              var paramIdx = 1;

              var flags = parseInt(answer.Params[paramIdx++]);
              var thingspeak_enabled = (flags & 1) == 1;
              $('#thingspeak_enabled').get(0).checked = thingspeak_enabled;
              
              var iot_interval = parseInt(answer.Params[paramIdx++])/1000;
              $('#iot_interval').val(iot_interval);
              
              // теперь пробегаем по всем датчикам
              var sourceList = $('#iot_all_sensors');
              sourceList.empty();
              
              var selectedList = $('#iot_selected_sensors');
              selectedList.empty();
              
              var idCntr = 0;
              for(var i=0;i<totalTempSensors;i++)
              {
                var div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                var lbl = $('<span/>').text('Температура, датчик "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'STATE')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 1, sensorIndex : i, sensorType : 1});
                
                idCntr++;
              } // for

              for(var i=0;i<totalHumiditySensors;i++)
              {
                var div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                var lbl = $('<span/>').text('Температура, датчик влажности "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'HUMIDITY')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 2, sensorIndex : i, sensorType : 1});
                
                idCntr++;
                
                div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                lbl = $('<span/>').text('Влажность, датчик влажности "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'HUMIDITY')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 2, sensorIndex : i, sensorType : 8});
                
                idCntr++;                
              } // for
              
              for(var i=0;i<totalLuminositySensors;i++)
              {
                var div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                var lbl = $('<span/>').text('Освещённость, датчик "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'LIGHT')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 3, sensorIndex : i, sensorType : 4});
                
                idCntr++;
              } // for
              
              for(var i=0;i<totalSoilMoistureSensors;i++)
              {
                var div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                var lbl = $('<span/>').text('Влажность почвы, датчик "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'SOIL')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 4, sensorIndex : i, sensorType : 64});
                
                idCntr++;
              } // for                                 
              
               for(var i=0;i<totalPHSensors;i++)
              {
                var div = $('<div/>',{'class' : 'iot_sensor ui-state-default ui-corner-all'}).appendTo(sourceList);
                var lbl = $('<span/>').text('Показания pH, датчик "' + controller.SensorsNames.getMnemonicName(new Sensor(i,'PH')) + '"').appendTo(div);
                
                div.data('iot_sensor',{moduleID : 5, sensorIndex : i, sensorType : 128});
                
                idCntr++;
              } // for    
              
              var sensors = sourceList.find('div.iot_sensor');
              
              // и выделяем нужные из них
              for(var i=0;i<8;i++)
              {
                var moduleID = parseInt(answer.Params[paramIdx++]);
                var sensorType = parseInt(answer.Params[paramIdx++]);
                var sensorIndex = parseInt(answer.Params[paramIdx++]);
                
                sensors.each(function(){
                
                    var record = $(this);
                    var iot_sensor = record.data('iot_sensor');
                    if(iot_sensor.moduleID == moduleID && iot_sensor.sensorType == sensorType && iot_sensor.sensorIndex == sensorIndex)
                    {
                      record.detach();
                      record.appendTo(selectedList);
                    }
                
                });
                
              } // for
              
              // теперь делаем списки сортируемыми
              selectedList.sortable({revert:true, connectWith: '.iot_sortable'});
              sourceList.sortable({revert:true, connectWith: '.iot_sortable'});
              
              // теперь получаем ID канала ThingSpeak
              var thingSpeakChannelKey = answer.Params[paramIdx++];
              $('#thingspeak_channel').val(thingSpeakChannelKey);
              
          }
       });
    }
    
    if(controller.Modules.includes('CC')) // если в прошивке есть модуль составных команд
    {
        controller.queryServerScript("/x_get_composite_commands.php",{}, function(obj,result){
           
          
          var answer = result.composite_commands;
          compositeCommands.Clear();
          
          for(var i=0;i<answer.length;i++)
          {            
            var cc = answer[i];
           
            compositeCommands.Add(cc.list_index, cc.list_name, cc.command_action, cc.command_param);
            
          } // for
          
          // тут заполнение списка составных команд
            for(var i=0;i<compositeCommands.List.length;i++)
            {
              var ccList = compositeCommands.List[i];
              $('#cc_lists').append($('<option/>',{value: ccList.Index }).text('#' + i + ' - ' + ccList.Name));
            } // for
          
          $('#cc_lists').change(function(){
          
            var list_index = $(this).val();
            compositeCommands.fillList('#CC_LIST',list_index);
          
          });
          
          $('#CC_MENU').toggle(true);
          $('#cc_lists_box').toggle(true);
          
           $('#cc_lists').trigger('change');
          
          
        });
    } // composite commands end
    
    if(controller.Modules.includes('WATER')) // если есть модуль полива в прошивке
    {
       
       //$('#WATER_MENU').toggle(true);
       
       var channelsToRetrieve = 0;
       var retrievedChannels = 0;
       
        controller.queryCommand(true,'WATER|T_SETT',function(obj,answer){
        
              //$('#WATER_MENU').toggle(answer.IsOK);
              
              if(answer.IsOK)
              {           
                //  console.log(answer);
              
                  wateringSettings = new WateringSettings(answer.Params[2],answer.Params[3],answer.Params[4],answer.Params[5],answer.Params[6],answer.Params[7],answer.Params[8], answer.Params[9], answer.Params[10]);
                  
                  $('#all_watering_start_hour').val(parseInt(wateringSettings.StartTime/60));
                  $('#all_watering_start_minute').val(parseInt(wateringSettings.StartTime%60));
                  
                  $('#all_watering_time').val(wateringSettings.WateringTime);
                  $('#all_watering_sensor_index').val(wateringSettings.WateringSensorIndex);
                  $('#all_watering_stop_border').val(wateringSettings.WateringStopBorder);
                  $('#all_watering_start_border').val(wateringSettings.WateringStartBorder);
                  
                  var lst = $('#all_watering_channels_days').find('div #all_watering_channels_day');

                  for(var i=0;i<lst.length;i++)
                  {
                     var elem = lst.get(i);
                     var dayMask = parseInt(elem.value);
                     elem.checked = (wateringSettings.WateringDays & dayMask) == dayMask;
                      
                  } // for
                  
                  controller.queryCommand(true,'WATER|CHANNELS',function(obj,cntChannels){
                  
                      if(cntChannels.IsOK)
                      {
                        channelsToRetrieve = parseInt(cntChannels.Params[2]);
                        
                        if(!channelsToRetrieve)
                          return;
                          
                        for(var i=0;i<channelsToRetrieve;i++)
                        {
                            controller.queryCommand(true,'WATER|CH_SETT|' + i,function(obj,channelData){
                            retrievedChannels++;
                            
                                if(channelData.IsOK)
                                {
                                  var channel = new WaterChannelSettings(channelData.Params[3], channelData.Params[4], channelData.Params[5], channelData.Params[6], channelData.Params[7], channelData.Params[8]);
                                  wateringSettings.Add(channel);
                                } // if
                            
                                if(retrievedChannels >= channelsToRetrieve)
                                {
                                  $('#WATER_MENU').toggle(true);
                                  $('#watering_option').val(wateringSettings.WateringOption);
                                  $('#watering_option').trigger('change');
                                  
                                  $('#turn_pump_box').toggle(wateringSettings.WateringOption > 0);
                                  $('#switch_w_a_m_box').toggle(wateringSettings.WateringOption > 0);
                                  
                                  $('#turn_on_pump').get(0).checked = wateringSettings.TurnOnPump == 1;
                                  $('#switch_w_to_aam').get(0).checked = wateringSettings.SwitchToAutoAfterMidnight > 0;
                                  
                                  fillWaterChannelsList(); // заполняем таблицу настроек каналов полива 
                                } // if
                            
                            });
                        } // for
                      } // if(cntChannels.IsOK)
                  
                  });
              } // answer.IsOK
        
        });
    } // water
    
    if(controller.Modules.includes('RSRV')) // есть список резервирования
    {
      $('#RESERVATION_MENU').toggle(true);
      $('#RESERVATION_LIST').html("");
      reservations.Clear();
      
        var reservationsToRetrieve = 0;
        var retrievedReservations = 0;
        
        controller.queryCommand(true,'RSRV|CNT',function(obj,answer){
        
          if(answer.IsOK)
          {
            reservationsToRetrieve = parseInt(answer.Params[2]);
            
            for(var i=0;i<reservationsToRetrieve;i++)
            {
                  controller.queryCommand(true,'RSRV|VIEW|' + i,function(obj,reservationInfo){
                  
                      retrievedReservations++;
                      if(reservationInfo.IsOK)
                      {
                        var resType = reservationInfo.Params[3];
                        var resList = reservations.Add(resType);
                        
                        for(var j=4;j<reservationInfo.Params.length;j+=2)
                        {
                          resList.Add(reservationInfo.Params[j],parseInt(reservationInfo.Params[j+1]));
                        } // for
                        
                      } // IsOK
                      
                      if(retrievedReservations == reservationsToRetrieve)
                        fillReservationsList();
                  
                  });
            } // for
          } // if(answer.IsOK)
        
        });
      
    }
    
    
    if(controller.Modules.includes('DELTA'))
      queryDeltasList(); // получаем список дельт
      
    controller.queryCommand(true,'STAT|FREERAM',function(obj,answer){
    
          closeWaitDialog();
          
      });      
  
};
//-----------------------------------------------------------------------------------------------------
// создаём новый список составных команд
function newCCList()
{

$("#new_cc_list_dialog").dialog({modal:true, buttons: [{text: "Добавить", click: function(){


      var list_name = $('#cc_list_name').val();
      
      if(list_name == '')
        return;

      $(this).dialog("close");
      
      var newList = compositeCommands.addNewList(list_name);
      $('#cc_lists').append($('<option/>',{value: newList.Index }).text('#' + (newList.Index - 1) + ' - ' + newList.Name));
      
      $('#cc_lists').val(newList.Index);
      $('#cc_lists').trigger('change');
    
      $(this ).dialog('close');
      
 
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  
}
//-----------------------------------------------------------------------------------------------------
// создаём новую команду в списке составных команд
function newCCCommand()
{
  var idx = $('#cc_lists').val();
  if(idx === null)
  {
    showMessage('Создайте список составных команд!');
    return;
  }


$("#new_cc_command_dialog").dialog({modal:true, buttons: [{text: "Добавить", click: function(){


      var cc_action = $('#cc_type').val();
      var cc_param = parseInt($('#cc_param').val());
      if(isNaN(cc_param))
        cc_param = 0;
                
        compositeCommands.Add(idx,'',cc_action,cc_param);
        $('#cc_lists').trigger('change');
              
      $(this ).dialog('close');
 
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  


}
//-----------------------------------------------------------------------------------------------------
function saveHTTPSettings()
{
  var httpEnabled = $('#http_api_enabled').get(0).checked ? 1 : 0;
  var sendSensors = $('#http_send_status').get(0).checked ? 1 : 0;
  var sendState = $('#http_send_state').get(0).checked ? 1 : 0;
  
  var apiKey = $('#http_api_key').val().trim();
  var rawTimezoneVal = $('#http_timezone').val().trim();
  var timezone = parseInt(rawTimezoneVal);
  
  if(apiKey.length != 32)
  {
    showMessage("Ключ API должен быть длиной в 32 символа!", function(){
    
      $('#http_api_key').focus();
    
    });
    return;
  }
  
  if(rawTimezoneVal == '' || isNaN(timezone))
  {
    showMessage("Пожалуйста, укажите вашу часовую зону!", function(){
    
      $('#http_timezone').focus();
    
    });
    return;
  }
  
  var full_command = "HTTP|KEY|" + apiKey + '|' + httpEnabled + '|' + timezone + '|' + sendSensors + '|' + sendState;
  
  showWaitDialog();
  
  controller.queryCommand(false,full_command,function(obj,answer){
                                
                                  closeWaitDialog();
                                  if(answer.IsOK)
                                    showMessage("Данные успешно сохранены!");
                                  else
                                    showMessage("Ошибка сохранения данных :(");                           
                                  
                                  
                                });  
  
  
}
//-----------------------------------------------------------------------------------------------------
function saveIoTSettings()
{
  var flags = 0;
  if($('#thingspeak_enabled').get(0).checked)
    flags |= 1;
    
  var interval = parseInt($('#iot_interval').val());
  if(isNaN(interval) || interval < 1) {
    showMessage("Пожалуйста, укажите положительный интервал в секундах.", function(){
    
      $('#iot_interval').focus();
    
    });
    return;
  }
  
  interval *= 1000;
  
  var thingSpeakChannelKey = $('#thingspeak_channel').val().substring(0,19);
  
  // теперь выбираем все датчики
  var lst = $('#iot_selected_sensors');
  var sensors_command = '';
  var sensorsProcessed = 0;
  
  lst.find('div.iot_sensor').each(function(){
    if(sensorsProcessed > 8)
      return;
      
      var record = $(this);
    
      sensorsProcessed++;
      var iot_sensor = record.data('iot_sensor');
      
      sensors_command += "|" + iot_sensor.moduleID + "|" + iot_sensor.sensorType + "|" + iot_sensor.sensorIndex;
      
  });
  
  // дополняем до 8
  for(var i=sensorsProcessed;i<8;i++)
  {
    sensors_command += "|0|0|0";
  }
  
  // теперь составляем команду
  var full_command = "IOT|T_SETT|" + flags + "|" + interval + sensors_command + "|" + thingSpeakChannelKey;
  
  
  showWaitDialog();
  
  controller.queryCommand(false,full_command,function(obj,answer){
                                
                                  closeWaitDialog();
                                  if(answer.IsOK)
                                    showMessage("Данные успешно сохранены!");
                                  else
                                    showMessage("Ошибка сохранения данных :(");                           
                                  
                                  
                                });  


}
//-----------------------------------------------------------------------------------------------------
// сохраняем все списки составных команд в БД
function saveCCLists()
{
  showWaitDialog();
      
   var totalCommands = compositeCommands.List.length;
   var commandsProcessed = 0;
   var currentListIndex = 0;  
            
   for(var i=0;i<compositeCommands.List.length;i++)
   {
    var ccList = compositeCommands.List[i];
    totalCommands += ccList.List.length;
   } // for
   
   controller.queryServerScript("/x_clear_cc_lists.php",{}, function(obj,result){
 
 
      if(commandsProcessed == totalCommands)
      {
        closeWaitDialog();
      }
 
       for(var i=0;i<compositeCommands.List.length;i++)
       {
          var ccList = compositeCommands.List[i];
                    
          controller.queryServerScript("/x_add_cc_list.php",{list_name: ccList.Name, list_index: ccList.Index }, function(obj,result){
          
            commandsProcessed++;
            
                  if(commandsProcessed == totalCommands)
                  {
                    closeWaitDialog();
                    showMessage("Данные успешно сохранены!");
                  }
            
                 var thisCCList = compositeCommands.List[currentListIndex++];
            
                for(var j=0;j<thisCCList.List.length;j++)
                { 
                  var cc = thisCCList.List[j];
                  
                  controller.queryServerScript("/x_add_cc_command.php",{list_index: cc.ParentList.Index, command_action: cc.Action, command_param: cc.Param }, function(obj,result){
                  commandsProcessed++;
                  
                  if(commandsProcessed == totalCommands)
                  {
                    closeWaitDialog();
                    showMessage("Данные успешно сохранены!");
                  }
                  
                  });
                } // for
          });
       } // for   
   
   });
   

}
//-----------------------------------------------------------------------------------------------------
// загружаем составные команды в контроллер
function uploadCCCommands()
{

  // cc_list_selector
  $('#cc_list_selector').empty();
  var selectedLists = new Array();
  var listCounter = 0;
 
  for(var i=0;i<compositeCommands.List.length;i++)
  {
    var ccList = compositeCommands.List[i];
    var row = $('<div/>');
    $('<input/>',{type: 'checkbox', id: 'cc_list_checkbox', value: ccList.Index}).appendTo(row);
    $('<label/>').appendTo(row).html(ccList.Name);
    
    row.appendTo('#cc_list_selector');
  } // for
  

$("#select_cc_lists_dialog").dialog({modal:true, width:500, buttons: [{text: "Загрузить", click: function(){


      var lst = $('#cc_list_selector').find('div #cc_list_checkbox');
      for(var i=0;i<lst.length;i++)
      {
        var elem = lst.get(i);
        if(elem.checked)
          selectedLists.push(parseInt(elem.value));
      }

              
      $('#select_cc_lists_dialog').dialog('close');
 

             showWaitDialog();
                          
                          
                controller.queryCommand(false,'CC|DEL',function(obj,answer){
                
                    if(!answer.IsOK)
                    {
                       closeWaitDialog();
                       return;
                    } 
                    
                      
                      for(var i=0;i<compositeCommands.List.length;i++)
                      {
                        var ccList = compositeCommands.List[i];
                        
                        
                        if(!selectedLists.includes(ccList.Index))
                          continue;
                        
                          for(var j=0;j<ccList.List.length;j++)
                          {
                            var cc = ccList.List[j];
                              
                            var cmd = 'CC|ADD|' + listCounter + '|' + cc.Action + '|' + cc.Param;

                        
                            controller.queryCommand(false,cmd,function(obj,addResult){
                                                
                          
                              });                
                          } // for
                        
                          listCounter++;
                        
                        
                          
                      } // for
                      
                                controller.queryCommand(false,'CC|SAVE',function(obj,answer){
                                
                                  closeWaitDialog();
                                  if(answer.IsOK)
                                    showMessage("Данные успешно сохранены!");
                                  else
                                    showMessage("Ошибка сохранения данных :(");                           
                                  
                                  
                                });
                      
                  
                
                });
                
  
  } }
  
  , {text: "Отмена", click: function(){$(this).dialog("close");} }
  ] });  
                

}
//-----------------------------------------------------------------------------------------------------
// удаляем список составных команд
function deleteCCList()
{
  var idx = $('#cc_lists').val();

  
  compositeCommands.remove(idx);
  
  var index = $('#cc_lists').get(0).selectedIndex;
  $('#cc_lists option:eq(' + index + ')').remove();
  
  $('#cc_lists').get(0).selectedIndex = 0;  
  $('#cc_lists').trigger('change');
}
//-----------------------------------------------------------------------------------------------------
function addRuleModuleToList(moduleName)
{
  $('#rule_module_select').append($('<option/>',{value: moduleName}).text(ModuleNamesBindings[moduleName]));
}
//-----------------------------------------------------------------------------------------------------
function runScene()
{
  var selIdx = $('#scene_list').get(0).selectedIndex;
   if(selIdx > -1)
   {
      showWaitDialog();
      
      controller.queryCommand(false,'SCN|EXEC|' + selIdx,function(obj,answer){
                                
                                  closeWaitDialog();
                                  if(answer.IsOK)
                                    showMessage("Выбранный сценарий запущен!");
                                  else
                                    showMessage("Ошибка выполнения команды :(");                           
                                  
                                  
                                });      
   }
}
//-----------------------------------------------------------------------------------------------------
function stopScene()
{
  var selIdx = $('#scene_list').get(0).selectedIndex;
   if(selIdx > -1)
   {
      showWaitDialog();
      
      controller.queryCommand(false,'SCN|STOP|' + selIdx,function(obj,answer){
                                
                                  closeWaitDialog();
                                  if(answer.IsOK)
                                    showMessage("Выбранный сценарий остановлен!");
                                  else
                                    showMessage("Ошибка выполнения команды :(");                           
                                  
                                  
                                });      
   }
}
//-----------------------------------------------------------------------------------------------------
// выполняем команду, введённую пользователем
function execCommandPrompt()
{
  var cmd = $('#command_prompt_text').val();
  cmd = cmd.toUpperCase();
 
  if(cmd == "")
  {
    showMessage("Введите текст команды!");
    return;
  }
  
  var getIdx = cmd.indexOf("CTGET=");
  var setIdx = cmd.indexOf("CTSET=");
  
  if((getIdx == -1 && setIdx == -1))
  {
    showMessage("Команда должна начинаться с CTGET= или CTSET=<p>Пожалуйста, напишите команду правильно.");
    return;
  }
  
  var isGet = getIdx != -1;
  cmd = cmd.substring(6);
  
  if(cmd.length < 1)
  {
    showMessage("Команда не должна быть пустой!");
    return;
  }
  
  $("#exec_command_button").button('disable');
  
    controller.queryCommand(isGet,cmd,function(obj,answer){
                                
                                  $("#exec_command_button").button('enable');
                                  $('#controller_answer_text').val(answer.RawData);
                                  
                                });  
  
  
}
//-----------------------------------------------------------------------------------------------------
$(document).ready(function(){

  lastVisibleContent = $('#welcome');

  controller.querySensorNames(); // получаем имена датчиков
  
  $('#reservation_type_select').change(function(){
  
      var tp = $(this).val();
      
      // заполняем список датчиков для выбранного типа резервирования
      var list = $('#reservation_sensors_list');
      list.html("");
      
      var cnt = 0;
      var mName = tp;
      
      if(tp == 'TEMP')
      {
        cnt = totalTempSensors;
        mName = 'STATE';
      }
      else
      if(tp == 'HUMIDITY')
        cnt = totalHumiditySensors;
      else
      if(tp == 'LIGHT')
        cnt = totalLuminositySensors;
      else
      if(tp == 'SOIL')
        cnt = totalSoilMoistureSensors;
      else
      if(tp == 'PH')
        cnt = totalPHSensors;
        
      for(var i=0;i<cnt;i++)
      {
        addReservationCheckbox(mName,i);
      } // for
      
      if(tp == 'TEMP')
      {
        // для температуры добавляем ещё и датчики влажности
        for(var i=0;i<totalHumiditySensors;i++)
        {
          addReservationCheckbox('HUMIDITY',i);
        }
      } // if
  
  });
    
  $('#rule_action_input').change(function(){
  
      var val = parseInt($(this).val());
      var ed = $('#rule_additional_param_input');
      ed.attr('placeholder','');
      $('#rule_additional_param').toggle(false);
      $('#ruleWndInterval').toggle(false);
      
      var $addParamCaption = $('#rule_additional_param_caption');
      $addParamCaption.html("Дополнительный параметр:");
      
      switch(val)
      {
        case 0:
        case 1:
          $addParamCaption.html("Номера окон:");
          ed.attr('placeholder','ALL - все окна, 0-2 - диапазон');
          $('#rule_additional_param').toggle(true);
		  $('#wndPosHint').text('Положение окна, с:');
		  $('#rule_wnd_interval_input').attr('placeholder','0 - не указано');
          $('#ruleWndInterval').toggle(val == 0);
        break;
      
        case 4:
        case 5:
          $addParamCaption.html("Номера пинов:");
          ed.attr('placeholder','номера пинов, через запятую');
          $('#rule_additional_param').toggle(true);
        break;
        
        case 6:
          $addParamCaption.html("Индекс составной команды:");
          ed.attr('placeholder','индекс составной команды');
          $('#rule_additional_param').toggle(true);
        break;

        case 7:
        case 8:
          $addParamCaption.html("Номер сценария:");
          ed.attr('placeholder','номер сценария');
          $('#rule_additional_param').toggle(true);
        break;
		
        case 9:
        case 10:
          $addParamCaption.html("Номер микросхемы:");
          ed.attr('placeholder','номер микросхемы');
          $('#rule_additional_param').toggle(true);
		  $('#wndPosHint').text('Канал микросхемы:');
		  $('#rule_wnd_interval_input').attr('placeholder','0-15');
          $('#ruleWndInterval').toggle(true);
		  
        break;
		

      } // switch
  });
  
  $('#rule_module_select').change(function(){
  /*
    var moduleName = $(this).val();
    if(moduleName == null) // ничего не надо заполнять
      return;
      
   // заполняем список датчиков для выбранного модуля
      var cnt = 0;
      $('#rule_sensor_index_input').empty().text('');
      switch(moduleName)
      {
        case 'STATE': cnt = totalTempSensors; break;
        case 'HUMIDITY': cnt = totalHumiditySensors; break;
        case 'LIGHT': cnt = totalLuminositySensors; break;
        case 'SOIL': cnt = totalSoilMoistureSensors; break;
        case 'PH': cnt = totalPHSensors; break;
        
        case 'DELTA':
        {
          // список дельт обрабатываем отдельно
          var deltaType = $('#rule_target_input').val();
          for(var i=0;i<deltaList.List.length;i++)
          {
            var delta = deltaList.List[i];
            if(deltaType == delta.Type)
              $('<option/>',{value: i}).text(controller.SensorsNames.getMnemonicName(new Sensor(i,'DELTA'))).appendTo('#rule_sensor_index_input');
            
          }// for
        }
        break;
      }
      
      // получили кол-во датчиков выбранного модуля, добавляем их в выпадающий список
      for(var i=0;i<cnt;i++)
      {
        $('<option/>',{value: i}).text(controller.SensorsNames.getMnemonicName(new Sensor(i,moduleName))).appendTo('#rule_sensor_index_input');
      }
    */      
  
  });
  
  $('#rule_target_input').change(function(){
  
    var val = $(this).val();
    $('#rule_module_box').toggle(val != '_');
    $('#rule_index_box').toggle(val != '_');
    $('#rule_sensor_value_box').toggle(val != '_');
    $('#rule_pin_state_box').toggle(false);
    
    $('#rule_module_select').empty().val('');
    $('#rule_sensor_index_description').text('Индекс датчика:');
    $('#rule_pin_number').toggle(false);
    $('#rule_sensor_index_input').toggle(true);
    
    switch(val)
    {
      case '_': // ни за чем не следим
      {
      
      }
      break;
      
      case 'TEMP': // следим за температурой
      {
        addRuleModuleToList('STATE');
        addRuleModuleToList('HUMIDITY');
        addRuleModuleToList('DELTA');        
      }
      break;
      
      case 'HUMIDITY': // следим за влажностью
      {
        addRuleModuleToList('HUMIDITY');
        addRuleModuleToList('DELTA');        
      }
      break;
      
      case 'LIGHT': // следим за освещенностью
      {
        addRuleModuleToList('LIGHT');
        addRuleModuleToList('DELTA');        
      }
      break;
      
      case 'SOIL': // следим за влажностью почвы
      {
        addRuleModuleToList('SOIL');
        addRuleModuleToList('DELTA');        
      }
      break;
      
      case 'PH': // следим за состоянием pH
      {
        addRuleModuleToList('PH');
        addRuleModuleToList('DELTA');        
      }
      break;
      
      case 'PIN': // следим за уровнем пина
      {
        $('#rule_module_box').toggle(false);
        $('#rule_sensor_index_description').text('Номер пина:');
        $('#rule_sensor_value_box').toggle(false);
        $('#rule_pin_state_box').toggle(true);
        $('#rule_pin_number').toggle(true);
        $('#rule_sensor_index_input').toggle(false);
      }
      break;
    } // switch
    
    $('#rule_module_select').trigger('change');
  
  });
  
  $('#command_prompt_text').keypress(function(e){
  
    if(e.which == 13)
    {
      execCommandPrompt();
    }
  
  });
  
  
  $( "#get_delta_button, #get_rules_button" ).button({
      icons: {
        primary: "ui-icon-arrowthickstop-1-s"
      }
    });
    
  $( "#save_delta_button, #save_iot_button, #save_statsms_button, #save_http_button, #save_cc_button, #save_watering_button, #save_rules_button, #save_sms_button, #save_timers_button" ).button({
      icons: {
        primary: "ui-icon-arrowthickstop-1-n"
      }
    }); 
    
  $( "#new_delta_button, #new_cc_button, #new_rule_button, #new_sms_button" ).button({
      icons: {
        primary: "ui-icon-document"
      }
    });
    
      $( "#phone_number" ).button({
      icons: {
        primary: "ui-icon-note"
      }
    });

    
    $('#new_cc_list').button({
      icons: {
        primary: "ui-icon-note"
      }
    });
  
    
    $('#delete_cc_list, #stop_scene').button({
      icons: {
        primary: "ui-icon-close"
      }
    });  
    
    $('#save_cc_lists').button({
      icons: {
        primary: "ui-icon-disk"
      }
    });
    
    $('#exec_command_button, #run_scene').button({
      icons: {
        primary: "ui-icon-play"
      }
    });
        

    $("#statsmssensor1, #statsmssensor2").spinner({min: -1, max: 100});

    
    $('#iot_interval, #cc_param, #flow_calibraton1, #flow_calibraton2, #rule_pin_number, #timerPin1, #timerPin2, #timerPin3, #timerPin4, #timerOnMin1, #timerOnMin2, #timerOnMin3, #timerOnMin4, #timerOnSec1, #timerOnSec2, #timerOnSec3, #timerOnSec4, #timerOffMin1, #timerOffMin2, #timerOffMin3, #timerOffMin4, #timerOffSec1, #timerOffSec2, #timerOffSec3, #timerOffSec4, #rule_wnd_interval_input, #rule_sensor_index_input').forceNumericOnly();     

    $('#all_watering_start_hour, #all_watering_start_minute, #all_watering_time, #ph_calibraton, #ph4Voltage, #ph7Voltage, #ph10Voltage, #phTemperatureSensor, #phCalibrationTemperature, #phTarget, #phHisteresis, #phMixPumpTime, #phReagentPumpTime').forceNumericOnly();
    $('#watering_start_hour, #watering_start_minute, #watering_time, #watering_sensor_index, #watering_stop_border, #watering_start_border, #all_watering_sensor_index, #all_watering_stop_border, #all_watering_start_border').forceNumericOnly(); 

    $('#rule_work_time_input, #rule_start_time_input').forceNumericOnly();
//    $('#rule_work_time_input, #rule_start_time_input, #rule_sensor_value_input').forceNumericOnly();
    
    for(var i=0;i<CompositeActionsNames.length;i++)
    {
      $('#cc_type').append($('<option/>',{value: i}).text(CompositeActionsNames[i]));
    }
    
    $('#watering_option').change(function() {
    
      var watering_option = parseInt($(this).val());
      $('#watering_all_channels').toggle(false);
      $('#watering_separate_channels').toggle(false);
      $('#turn_pump_box').toggle(false);
      
      switch(watering_option)
      {
        case 1:
          $('#watering_all_channels').toggle(true);
          $('#turn_pump_box').toggle(true);
        break;
        
        case 2:
          $('#watering_separate_channels').toggle(true);
          $('#turn_pump_box').toggle(true);
        break;
      } // switch
    
    });
    
    
    $('#delta_type').change(function() {
    
      var delta_type = $(this).val();
      
       $('#delta_module1').empty().text('');
       $('#delta_module2').empty().text('');
       
        if(delta_type == 'TEMP')
        {
          $('#delta_module1').append($('<option/>',{value: 'STATE'}).text('Модуль температур'));
          $('#delta_module2').append($('<option/>',{value: 'STATE'}).text('Модуль температур'));
          
          $('#delta_module1').append($('<option/>',{value: 'HUMIDITY'}).text('Модуль влажности'));
          $('#delta_module2').append($('<option/>',{value: 'HUMIDITY'}).text('Модуль влажности'));
                   
        } // if TEMP
        else if(delta_type == 'HUMIDITY')
        {
          $('#delta_module1').append($('<option/>',{value: 'HUMIDITY'}).text('Модуль влажности'));
          $('#delta_module2').append($('<option/>',{value: 'HUMIDITY'}).text('Модуль влажности'));
        } // HUMIDITY
        else if(delta_type == 'LIGHT')
        {
          $('#delta_module1').append($('<option/>',{value: 'LIGHT'}).text('Модуль освещенности'));
          $('#delta_module2').append($('<option/>',{value: 'LIGHT'}).text('Модуль освещенности'));
        } // LIGHT
        else if(delta_type == 'SOIL')
        {
          $('#delta_module1').append($('<option/>',{value: 'SOIL'}).text('Модуль влажности почвы'));
          $('#delta_module2').append($('<option/>',{value: 'SOIL'}).text('Модуль влажности почвы'));
        } // SOIL
        else if(delta_type == 'PH')
        {
          $('#delta_module1').append($('<option/>',{value: 'PH'}).text('Модуль контроля pH'));
          $('#delta_module2').append($('<option/>',{value: 'PH'}).text('Модуль контроля pH'));
        } // PH
       
       $('#delta_module1').trigger('change');
       $('#delta_module2').trigger('change');
    
    });   
    
    $('#delta_type').trigger('change'); 
    
    $('#delta_module1').change(function(){
    
      // заполняем список датчиков для выбранного первого модуля
      var deltaType = $('#delta_module1').val();
      var cnt = 0;
      switch(deltaType)
      {
        case 'STATE': cnt = totalTempSensors; break;
        case 'HUMIDITY': cnt = totalHumiditySensors; break;
        case 'LIGHT': cnt = totalLuminositySensors; break;
        case 'SOIL': cnt = totalSoilMoistureSensors; break;
        case 'PH': cnt = totalPHSensors; break;
      }
      
      // получили кол-во датчиков выбранного модуля, добавляем их в выпадающий список
      $('#delta_index1').empty().text('');
      for(var i=0;i<cnt;i++)
      {
        $('<option/>',{value: i}).text(controller.SensorsNames.getMnemonicName(new Sensor(i,deltaType))).appendTo('#delta_index1');
      }
    
    
    }); 
    
    $('#delta_module2').change(function(){
    
      // заполняем список датчиков для выбранного первого модуля
      var deltaType = $('#delta_module2').val();
      var cnt = 0;
      switch(deltaType)
      {
        case 'STATE': cnt = totalTempSensors; break;
        case 'HUMIDITY': cnt = totalHumiditySensors; break;
        case 'LIGHT': cnt = totalLuminositySensors; break;
        case 'SOIL': cnt = totalSoilMoistureSensors; break;
        case 'PH': cnt = totalPHSensors; break;
      }
      
      // получили кол-во датчиков выбранного модуля, добавляем их в выпадающий список
      $('#delta_index2').empty().text('');
      for(var i=0;i<cnt;i++)
      {
        $('<option/>',{value: i}).text(controller.SensorsNames.getMnemonicName(new Sensor(i,deltaType))).appendTo('#delta_index2');
      }
    
    
    });     

});
//-----------------------------------------------------------------------------------------------------
</script>
{/literal}