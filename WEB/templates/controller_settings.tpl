{* Smarty *}


<div id="message_dialog" title="Сообщение" class='hdn'>
  <p id='message_dialog_message'></p>
</div>

<div id="prompt_dialog" title="Подтверждение" class='hdn'>
  <p id='prompt_dialog_message'></p>
</div>


<div id="data_requested_dialog" title="Обработка данных..." class='hdn'>
  <p>Пожалуйста, подождите, пока данные обрабатываются...</p>
</div>

<div id='new_sms_dialog' title='Новое СМС' class='hdn'>
  <form id='sms_add_form'>
  
    Текст СМС:<br/>
    <input type='text' id='sms_text_input' name='sms_text_input' maxlength='50' value='' style='width:100%;'/><br/>
    Ответ контроллера:<br/>
    <input type='text' id='sms_answer_input' name='sms_answer_input' maxlength='100' value='' style='width:100%;'/><br/>
    
    Выполнить команду:<br/>
    <select id='sms_commands_list' style='width:100%;' name='sms_commands_list'>
    </select>

    <div id='sms_param' class='hdn'>
      <span id='sms_param_caption'>Дополнительный параметр:</span><br/>
      <input type='text' id='sms_param_text' name='sms_param_text' maxlength='100' value='' style='width:100%;'/><br/>
    </div>
    
  </form>
</div>

<div id='phone_number_dialog' title='Настройки GSM' class='hdn'>
  <form>
    Ваш номер телефона:<br/>
    <input type='text' id='edit_phone_number' maxlength='20' value='' style='width:100%;'/><br/>
    
    Оператор SIM контроллера:<br/>
    <select id='gsm_provider' style='width:100%;'>
      <option value='0' selected>МТС (Россия)</option>
      <option value='1'>Билайн (Россия)</option>
      <option value='2'>Мегафон (Россия)</option>
      <option value='3'>Теле2 (Россия)</option>
      <option value='4'>Йота (Россия)</option>
      <option value='5'>МТС (Беларусь)</option>
      <option value='6'>Velcom (Беларусь)</option>
      <option value='7'>Privet (Беларусь)</option>
      <option value='8'>Life (Беларусь)</option>
    </select>
  </form>
</div>

<div id='new_cc_list_dialog' title='Новый список' class='hdn'>
  <form>
    Имя списка:<br/>
    <input type='text' id='cc_list_name' maxlength='100' value='' style='width:100%;'/><br/>
  </form>
</div>

<div id='new_cc_command_dialog' title='Новая команда' class='hdn'>
  <form>
    Тип команды:<br/>
    <select id='cc_type' style='width:100%;'/></select><br/>
    Параметр:<br/>
    <input type='text' id='cc_param' maxlength='100' value='' style='width:100%;'/><br/>
  </form>
</div>

<div id='select_cc_lists_dialog' title='Выбор списков' class='hdn'>
  Выберите списки команд, которые надо загрузить в контроллер:
  <div id='cc_list_selector' class='padding_around8px'>
  </div>
</div>

<div id='sensors_info_dialog' title='Датчиков в прошивке' class='hdn'>

  <table width='100%' border='0' cellspacing='0' cellpadding='2'>

    <tr>
      <td>
        Температуры:
      </td>
      <td align='right' class='bold'>
        <div id='sensors_info_temp'><span class='no_data'>&lt;нет данных&gt;</span></div>
      </td>
    </tr>

    <tr>
      <td>
        Влажности:
      </td>
      <td align='right' class='bold'>
        <div id='sensors_info_humidity'><span class='no_data'>&lt;нет данных&gt;</span></div>
      </td>
    </tr>

    <tr>
      <td>
        Освещённости:
      </td>
      <td align='right' class='bold'>
        <div id='sensors_info_luminosity'><span class='no_data'>&lt;нет данных&gt;</span></div>
      </td>
    </tr>

    <tr>
      <td>
        Влажности почвы:
      </td>
      <td align='right' class='bold'>
        <div id='sensors_info_soil'><span class='no_data'>&lt;нет данных&gt;</span></div>
      </td>
    </tr>

    <tr>
      <td>
        Значения pH:
      </td>
      <td align='right' class='bold'>
        <div id='sensors_info_ph'><span class='no_data'>&lt;нет данных&gt;</span></div>
      </td>
    </tr>
    
  </table>

</div>





<div id='wifi_dialog' title='Настройки Wi-Fi' class='hdn'>
  <form>
    SSID роутера:<br/>
    <input type='text' id='router_id' maxlength='50' value='' style='width:100%;'/><br/>
    Пароль роутера:<br/>
    <input type='password' id='router_pass' maxlength='50' value='' style='width:100%;'/><br/>
    <input type='checkbox' id='connect_to_router' value='1'/>
    <label for='connect_to_router'>Коннектиться к роутеру</label><br/><br/>
    SSID модуля ESP:<br/>
    <input type='text' id='station_id' maxlength='50' value='' style='width:100%;'/><br/>
    Пароль модуля ESP:<br/>
    <input type='password' id='station_pass' maxlength='50' value='' style='width:100%;'/><br/>
    <p>
      ВНИМАНИЕ! При записи настроек Wi-Fi контроллер перезапустит ESP, и веб-интерфейс на какое-то
      время станет недоступен!
    </p>
    
  </form>
</div>

<div id='flow_calibration_dialog' title='Калибровка расходомеров' class='hdn'>
  <form>
    <p>
    Калибровка расходомера №1:<br/>
    <input type='text' id='flow_calibraton1' maxlength='3' value='' style='width:100%;'/>
    </p>
    <p>
    Калибровка расходомера №2:<br/>
    <input type='text' id='flow_calibraton2' maxlength='3' value='' style='width:100%;'/>
    </p>
    
  </form>
</div>


<div id='ph_calibration_dialog' title='Настройки PH' class='hdn'>
  <form>
  
    <table border='0' width='100%' cellspacing='0' cellpadding='4'>
    
      <tr>
        <td width='50%' valign='bottom'>
          Поправочное число, сотые доли (-100 ~ 100):<br/>
          <input type='text' id='ph_calibraton' maxlength='4' value='' style='width:100%;'/>
        </td>
        
        <td width='50%' valign='bottom'>
          Калибровочный раствор 4 pH, милливольт:<br/>
          <input type='text' id='ph4Voltage' maxlength='4' value='' style='width:100%;'/>
        </td>
      </tr>
      
      <tr>
        <td valign='bottom'>
        Калибровочный раствор 7 pH, милливольт:<br/>
        <input type='text' id='ph7Voltage' maxlength='4' value='' style='width:100%;'/>
        </td>
        
        <td valign='bottom'>
          Калибровочный раствор 10 pH, милливольт:<br/>
          <input type='text' id='ph10Voltage' maxlength='4' value='' style='width:100%;'/>
        </td>
      </tr>
      
      <tr>
        <td valign='bottom'>
          Индекс датчика температуры:<br/>
          <input type='text' id='phTemperatureSensor' maxlength='3' value='' style='width:100%;'/>
        </td>
        
        <td valign='bottom'>
          Температура калибровочных растворов:<br/>
          <input type='text' id='phCalibrationTemperature' maxlength='6' value='' style='width:100%;'/>
        </td>
      </tr>
      
      <tr>
        <td valign='bottom'>
          Поддерживаемое значение pH:<br/>
          <input type='text' id='phTarget' maxlength='3' value='' style='width:100%;'/>
        </td>
        
        <td valign='bottom'>
          Гистерезис:<br/>
          <input type='text' id='phHisteresis' maxlength='6' value='' style='width:100%;'/>
        </td>
      </tr>
      
      
      <tr>
        <td valign='bottom'>
          Время работы насоса перемешивания, с:<br/>
          <input type='text' id='phMixPumpTime' maxlength='3' value='' style='width:100%;'/>
        </td>
        
        <td valign='bottom'>
          Время работы насоса подачи реагента, для изменения pH на 0.1, с:<br/>
          <input type='text' id='phReagentPumpTime' maxlength='6' value='' style='width:100%;'/>
        </td>
      </tr>            
      
    </table>

    
  </form>
</div>

<div id="new_reservation_dialog" title="Новый список резервирования" class='hdn'>

<form>
  <div class='button_menu_spacer'>
  Тип резервирования:<br/>
  <select id='reservation_type_select' style='width:100%;'>
  <option value='TEMP'>Температура</option>
  <option value='HUMIDITY'>Влажность</option>
  <option value='LIGHT'>Освещенность</option>
  <option value='SOIL'>Влажность почвы</option>
  <option value='PH'>Значение pH</option>
  </select><br/>
  </div> 
  
  <div class='button_menu_spacer'>
  <div class='ui-widget-header ui-corner-all button_menu_spacer' style='margin-top:6px;'>Датчики для резервирования</div>
    <div id='reservation_sensors_list' style='height:120px;overflow-y:scroll;'>
    </div>
  </div>
   
</form>

</div>

<div id="new_delta_dialog" title="Новая дельта" class='hdn'>

  <form>
  <div class='button_menu_spacer'>
  Тип дельты:<br/>
  <select id='delta_type' style='width:100%;'>
  <option value='TEMP'>Температура</option>
  <option value='HUMIDITY'>Влажность</option>
  <option value='LIGHT'>Освещенность</option>
  <option value='SOIL'>Влажность почвы</option>
  <option value='PH'>Значение pH</option>
  </select><br/>
  </div>
  <div class='button_menu_spacer'>
  Модуль 1:<br/>
  <select id='delta_module1' style='width:100%;'></select><br/>
  </div>
  <div class='button_menu_spacer'>
  Датчик 1:<br/>
  <select id='delta_index1' style='width:100%;'></select><br/>
  </div>
  <div class='button_menu_spacer'>
  Модуль 2:<br/>
  <select id='delta_module2' style='width:100%;'></select><br/>
  </div>
  <div class='button_menu_spacer'>
  Датчик 2:<br/>
  <select id='delta_index2' style='width:100%;'/></select><br/>
  </div>
  </form>

</div>

<div id="rule_daymask_dialog" title="Дни работы правила" class='hdn'>
   <div id='rule_daymask_box' class='padding_around8px'>
    <div><input type='checkbox' id='rule_work_day' value='1'/>Понедельник</div>
    <div><input type='checkbox' id='rule_work_day' value='2'/>Вторник</div>
    <div><input type='checkbox' id='rule_work_day' value='4'/>Среда</div>
    <div><input type='checkbox' id='rule_work_day' value='8'/>Четверг</div>
    <div><input type='checkbox' id='rule_work_day' value='16'/>Пятница</div>
    <div><input type='checkbox' id='rule_work_day' value='32'/>Суббота</div>
    <div><input type='checkbox' id='rule_work_day' value='64'/>Воскресенье</div>
  </div>
</div>

<div id="rule_edit_dialog" title="Настройки правила" class='hdn'>
  <form>
  
    <div class='half'>
      <div class='half_box half_left left_align'>
        <div class='padding_around8px'>
  
            <div class='button_menu_spacer'>
              Имя правила:<br/>
              <input type='text' id='rule_name_input' maxlength='5' value='' style='width:100%;text-transform:uppercase;'/>
            </div>
            
            <div class='button_menu_spacer'> 
              Следим за:<br/>
              <select id='rule_target_input' style='width:100%;'>
              <option value='_'>Ни за чем не следим</option>
              <option value='TEMP'>Температурой</option>
              <option value='HUMIDITY'>Влажностью</option>
              <option value='LIGHT'>Освещенностью</option>
              <option value='PIN'>Уровнем пина</option>
              <option value='SOIL'>Влажностью почвы</option>
              <option value='PH'>Значением pH</option>
              </select>
            </div>
            
            
          <div class='button_menu_spacer'>  
            <div class='ui-widget-header ui-corner-all'>Начало работы</div>
            <div>
              <div style='float:left;width:50%'>
                Час:
                <div style='margin-right:8px;'>
                  <input type='text' id='rule_start_hour_input' maxlength='2' value='' style='width:100%;'/>
                </div>
              </div>

              <div style='float:left;width:50%'>
                Минута:
                <div>
                  <input type='text' id='rule_start_minute_input' maxlength='2' value='' style='width:100%;'/>
                </div>
              </div>
              
            </div>
          </div>
          
          <div class='button_menu_spacer'>
            Продолжительность, мин:
            <div>
              <div style='float:left;width:80%'>
                <input type='text' id='rule_work_time_input' maxlength='5' value='' style='width:100%;'/>
              </div>
              <div style='float:right;text-align:right;'>
                <button onclick="adjustRuleDaymask();return false;">Дни</button>
              </div>
            </div>
          </div>
          
          <div class='button_menu_spacer'>&nbsp;</div>
      
      </div>
      
    </div> 
  </div>
  
  <div class='half'>
    <div class='half_box half_right left_align'>
    
      <div class='padding_around8px'>
    
            <div id='rule_module_box' class='button_menu_spacer hdn'>
                Модуль, который опрашиваем:<br/>
                <select id='rule_module_select' style='width:100%;'>
                </select>
            </div>
            
          <div id='rule_index_box' class='button_menu_spacer hdn'>
            <div id='rule_sensor_index_description'>Индекс датчика:</div>
            <!--select id='rule_sensor_index_input' style='width:100%;'></select-->
			 <input type='text' id='rule_sensor_index_input' class='hdn' style='width:100%' maxlength='3'/>
            <input type='text' id='rule_pin_number' class='hdn' style='width:100%' maxlength='2'/>
          </div>
          
          
          <div id='rule_pin_state_box' class='button_menu_spacer hdn'>
            <div>Уровень на пине:</div>
            <div>
              <select id='rule_pin_state_input' style='width:100%;'>
                <option value='>='>высокий</option>
                <option value='<'>низкий</option>
              </select>
            </div>
          </div>

          <div id='rule_sensor_value_box' class='button_menu_spacer hdn'>
            Показания датчика:<br clear='left'/>
            <div class='half'>
                <div class='half_box half_left left_align'>
                 
                  <select id='rule_sensor_operand' style='width:100%;'>
                    <option value='>'>больше чем</option>
                    <option value='>='>больше либо равны</option>
                    <option value='<'>меньше чем</option>
                    <option value='<='>меньше либо равны</option>
                  </select>
                </div>
            </div>
                
            <div class='half'>
                <div class='half_box half_right left_align'>
                  <input type='text' id='rule_sensor_value_input' maxlength='4' value='' style='width:100%;'/>
                </div>                
            </div>
            
            
          </div>

            <div style='clear:left;'>
              Выполнить действие:
            </div>
            <div class='button_menu_spacer'>
              <select id='rule_action_input' style='width:100%'>
                <option value='0'>Открыть окна</option>
                <option value='1'>Закрыть окна</option>
                <option value='2'>Включить досветку</option>
                <option value='3'>Выключить досветку</option>
                <option value='4'>Высокий уровень на пинах</option>
                <option value='5'>Низкий уровень на пинах</option>
                <option value='6'>Выполнить составную команду</option>
                <option value='7'>Запустить сценарий</option>
                <option value='8'>Остановить сценарий</option>
                <option value='9'>Включить канал MCP23S17</option>
                <option value='10'>Выключить канал MCP23S17</option>
              </select>
            </div>
            
            <div class='button_menu_spacer hdn' id='rule_additional_param'>
                <div id='rule_additional_param_caption'>
                  Дополнительный параметр:
                </div>
                
                <div>
                  <input type='text' id='rule_additional_param_input' style='width:100%' placeholder=''/>
                </div>
                
                <div id='ruleWndInterval' class='hdn'>
                <span id='wndPosHint'>Положение окна, с:</span><br/>
                <input type='text' id='rule_wnd_interval_input' style='width:100%' placeholder='0 - не указано'/>
                </div>
                
            </div>            
      
      </div>
    
    </div>
  </div>

  <br clear='left'/>
  
  <div>
       <div class='ui-widget-header ui-corner-all'>Не выполнять, если сработали правила</div>
        <div id='linked_rules_box' class='padding_around8px' style='max-height:80px;overflow:auto;'>
        </div>
  </div>

  <div>
       <div class='ui-widget-header ui-corner-all'>Тревожное правило</div>
       <div class='padding_around8px'>
        <input type='checkbox' id='is_alarm_rule' value='0'/><label for='is_alarm_rule'>Это правило тревожное (при его срабатывании получите СМС)</label>
       </div>
  </div>

  
  <div class='ui-widget-header ui-corner-all'>
        Специальные значения
  </div>
  <div style='font-size:70%;'>
  <ul>
    <li><b>"%TO%"</b> - температура открытия из настроек;</li>
    <li><b>"%TC%"</b> - температура закрытия из настроек;</li>
    <li><b>"-2"</b> - игнорировать показания с датчика освещенности;</li>
    <li><b>"ALL"</b> - все окна, <b>"0-2"</b> - диапазон окон, <b>"7"</b> - одно окно;</li>
    <li>Если время начала и продолжительность работы правила установлены в <b>"0"</b> - правило работает всегда.</li>
  </ul>
  </div>
    
  </form>
</div>

<div id="water_channel_settings_dialog" title="Настройки канала" class='hdn'>

   Дни работы канала:
   <div id='water_channels_days' class='padding_around8px'>
    <div><input type='checkbox' id='watering_channel_day' value='1'/>Понедельник</div>
    <div><input type='checkbox' id='watering_channel_day' value='2'/>Вторник</div>
    <div><input type='checkbox' id='watering_channel_day' value='4'/>Среда</div>
    <div><input type='checkbox' id='watering_channel_day' value='8'/>Четверг</div>
    <div><input type='checkbox' id='watering_channel_day' value='16'/>Пятница</div>
    <div><input type='checkbox' id='watering_channel_day' value='32'/>Суббота</div>
    <div><input type='checkbox' id='watering_channel_day' value='64'/>Воскресенье</div>
  </div>

    <div>
      Начало работы (чч:мм):<br/>
      <input type='text' maxlength='2' id='watering_start_hour' style='width:45%;'/> : 
      <input type='text' maxlength='2' id='watering_start_minute' style='width:45%;'/>
    </div>

    <div>
      Продолжительность полива, минут:<br/>
      <input type='text' maxlength='5' id='watering_time' style='width:100%;'/>
    </div>    

    <div>
      Индекс датчика влажности почвы:<br/>
      <input type='text' maxlength='5' id='watering_sensor_index' style='width:100%;'/>
    </div>    

    <div>
      Влажность мин, %:<br/>
      <input type='text' maxlength='5' id='watering_start_border' style='width:100%;'/>
    </div>    
	
    <div>
      Влажность макс, %:<br/>
      <input type='text' maxlength='5' id='watering_stop_border' style='width:100%;'/>
    </div>    

  
</div>



{include file='controller_head.tpl' additional_text=', настройки' room='settings'}

<div id='wait_block' style='padding-left:20px;'>
<img src='images/wait.gif'/ align='absmiddle'> Подождите, идёт обработка данных...
</div>

<div id='offline_block' class='hdn'>
{include file='controller_offline.tpl'}
</div>

<div id='online_block' class='hdn'>

    <div class='left_menu'>
    
      <div class='menuitem hdn' id='DELTA_MENU' onclick="content(this);">Список дельт</div>
      <div class='menuitem hdn' id='CC_MENU' onclick="content(this);">Составные команды</div>
      <div class='menuitem hdn' id='RESERVATION_MENU' onclick="content(this);">Резервирование</div>
      <div class='menuitem hdn' id='RULES_MENU' onclick="content(this);">Правила</div>
      <div class='menuitem hdn' id='WATER_MENU' onclick="content(this);">Настройки полива</div>
      <div class='menuitem hdn' id='TIMERS_MENU' onclick="content(this);">Таймеры</div>
      <div class='menuitem hdn' id='SMS_MENU' onclick="content(this);">Список SMS</div>
      <div class='menuitem hdn' id='IOT_MENU' onclick="content(this);">IoT</div>
      <div class='menuitem hdn' id='HTTP_MENU' onclick="content(this);">HTTP API</div>
      <div class='menuitem hdn' id='SCENE_MENU' onclick="content(this);">Сценарии</div>
      <div class='menuitem' id='COMMAND_PROMPT_MENU' onclick="content(this);">Командная строка</div>

      <div class='menuitem hdn' id='ph_calibration_button' onclick="editPHCalibration();">Настройки PH</div>
      <div class='menuitem hdn' id='flow_calibration_button' onclick="editFlowCalibration();">Расходомеры</div>
      <div class='menuitem hdn' id='controller_time_button' onclick="setControllerTime();">Дата/время</div>

      <div class='menuitem hdn' id='wifi_menu' onclick="editWiFiSettings();">Настройки Wi-Fi</div>
      <div class='menuitem hdn' id='sensors_info_button' onclick="showSensorsInfo();">О датчиках</div>

    </div>
    
    
    <div class="page_content">
      
            <div class='content hdn' id='TIMERS_MENU_CONTENT'>
             <h1><span class='innerh'>Настройки периодических таймеров</span></h1>
              
              <div class='button_menu_spacer'>
              <a href='javascript:saveTimers();' id='save_timers_button'>Сохранить в контроллер</a>
              </div>
   
                    
                      <div>                      
                        <div class='half'>
                            <div class='half_box half_left'>
                            
                                {include file='timer.tpl' num=1}
                                
                             </div>
                        </div>
                      
                        <div class='half'>
                          <div class='half_box half_right'>
                              
                              {include file='timer.tpl' num=2}
                              
                          </div>
                        </div>
                      
                     </div>
                     
                    <br clear='left'/><br/>  
                    
                    <div>                      
                        <div class='half'>
                            <div class='half_box half_left'>
                                {include file='timer.tpl' num=3}
                             </div>
                        </div>
                      
                        <div class='half'>
                          <div class='half_box half_right'>
                              {include file='timer.tpl' num=4}
                          </div>
                        </div>
                      
                     </div> 
                     
                    
            </div>
            
            <div class='content hdn' id='HTTP_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>HTTP API сервиса gardenboss.ru</span></h1>
                    
                        <div class='button_menu_spacer'>
                          Ключ доступа к сервису:<br/><input type='text' id='http_api_key' value='' maxlength='32'/>
                        </div>                    

                        <div class='button_menu_spacer'>
                          Ваш часовой пояс (в минутах от UTC): <input type='text' id='http_timezone' value='' maxlength='32'/>
                        </div>        
                               
                        <div class='button_menu_spacer'>
                          <input type='checkbox' id='http_api_enabled'><label for='http_api_enabled'>HTTP API активно?</label>
                        </div>                    
                        <div class='button_menu_spacer'>
                          <input type='checkbox' id='http_send_status'><label for='http_send_status'>Посылать данные датчиков на сервис?</label>
                        </div>                    
                        <div class='button_menu_spacer'>
                          <input type='checkbox' id='http_send_state'><label for='http_send_state'>Посылать статус контроллера на сервис?</label>
                        </div>                    
                    


                        
                    <div style='margin-top:10px;'>
                        <button id='save_http_button' onclick='saveHTTPSettings();'>Сохранить</button>                        
                    </div>                        
                                           
            </div>
            
            <div class='content hdn' id='SCENE_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список сценариев</span></h1>
                    
                        <div class='button_menu_spacer'>
                          <select id='scene_list' size=8>
                          </select>
                        </div>                   
                        
                    <div style='margin-top:10px;'>
                        <button id='run_scene' onclick='runScene();'>Запустить выбранный сценарий</button>                        
                        <button id='stop_scene' onclick='stopScene();'>Остановить выбранный сценарий</button>                        
                    </div>                        
                                           
            </div>            
            
            
                  <div class='content hdn' id='IOT_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Настройки IoT</span></h1>
                    
                        <div class='button_menu_spacer'>
                          <input type='checkbox' id='thingspeak_enabled'><label for='thingspeak_enabled'>Отправлять данные на ThingSpeak?</label>
                        </div>
                        <div class='button_menu_spacer'>
                          Интервал, с:<br/><input type='text' id='iot_interval' value=''/>
                        </div>
                        <div class='button_menu_spacer'>
                          Ключ канала ThingSpeak:<br/><input type='text' id='thingspeak_channel' value='' maxlength='19'/>
                        </div>                    
                    
                     <div class='info-info bottom-spacer'>
                      <span class='ui-icon ui-icon-info' style='float:left;margin-right:10px;'></span>Датчики можно перетаскивать между списками.
                     </div>
                     
                     <div>                    
                    
                        <table border='0' width='100%'>
                      
                        <tr>
                        <td valign='top' width='50%'>
                           <h2><span class='innerh'>Доступные датчики</span></h2>
                           
                           <div class='iot_sortable' style='height:300px;overflow:auto;' id='iot_all_sensors'>
                          </div>

                        </td>
                        <td valign='top'>
                          <h2><span class='innerh'>Выбранные датчики (максимум 8)</span></h2>
                          
                          <div class='iot_sortable' style='height:300px;overflow:auto;' id='iot_selected_sensors'>
                          </div>
                          
                        </td>
                        </tr>
                          
                        </table>
                      
                      </div>
                      
                    
                    <div style='margin-top:10px;'>
                        <button id='save_iot_button' onclick='saveIoTSettings();'>Сохранить</button>                        
                    </div>
                    
                  </div>            
                  
      
                  <div class='content hdn' id='DELTA_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список виртуальных датчиков дельт</span></h1>
                    
                    <div id='DELTA_LIST'></div>
                    <br clear='left'/>
                    <div><br/><br/>
                    
                        <button id='get_delta_button' onclick='queryDeltasList();'>Получить список дельт</button>
                        <button id='save_delta_button' onclick='saveDeltasList();'>Сохранить</button>
                        <button id='new_delta_button' onclick='newDelta();'>Новая дельта</button>
                        
                    </div>
                    
                  </div>
                  
                   <div class='content hdn' id='COMMAND_PROMPT_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Командная строка контроллера</span></h1>
                    
                    <div class='button_menu_spacer' style='margin-right:10px;'>
                    Текст команды:<br/>
                    <input type='text' id='command_prompt_text' value='' style='width:100%;text-transform:uppercase;'/>
                    </div>
            
                    <div class='button_menu_spacer'>
                    
                        <button id='exec_command_button' onclick='execCommandPrompt();'>Выполнить</button>
                        
                    </div>
                                        
                    <div class='button_menu_spacer' style='margin-right:10px;'>
                    Ответ контроллера:<br/>
                    <textarea id='controller_answer_text' style='width:100%;height:200px;'></textarea>
                    </div>
                    

                    
                  </div>
                                   
                  <div class='content hdn' id='SMS_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список SMS для управления контроллером</span></h1>
                    
                    <div id='SMS_LIST'></div>
                    <br clear='left'/>
                    <div><br/><br/>
                    
                        <button id='save_sms_button' onclick='saveSmsList();'>Сохранить в контроллер</button>
                        <button id='new_sms_button' onclick='newSms();'>Добавить SMS</button>
                        <button id='phone_number' class='hdn' onclick="editPhoneNumber();">Ваш номер и оператор</button>
                        
                    </div>
                    
                    <div id="statsmssettings">
                    <br/><br/>
                    <h1><span class='innerh'>Настройки SMS статистики</span></h1>
                    
                    <table border="0" width="100%">
                      <tr>
                        <td valign="top" width="50%">
                          <h2><span class='innerh'>Датчик 1</span></h2>
                          
                          Модуль:<br/>
                          <select id='statsmsmodule1'>
                            <option value="0">не указан</option>
                            <option value="1">модуль температур</option>
                            <option value="2">модуль влажности</option>
                            <option value="3">модуль освещённости</option>
                            <option value="4">модуль влажности почвы</option>
                         </select>
                         <br/>
                         
                         <div style="text-align:right;">
                           Индекс датчика (-1 - не указан):
                           <input type="text" id="statsmssensor1" value="-1" style="width:80px;"/>
                         </div>
                         
                         <br/><br/>
                         Подпись:<br/>
                         <input type="text" id="statsmslabel1" maxlength="20"/>
                         
                          
                        <td>
                        <td valign="top" width="50%">
                          <h2><span class='innerh'>Датчик 2</span></h2>
                          
                          Модуль:<br/>
                          <select id='statsmsmodule2'>
                            <option value="0">не указан</option>
                            <option value="1">модуль температур</option>
                            <option value="2">модуль влажности</option>
                            <option value="3">модуль освещённости</option>
                            <option value="4">модуль влажности почвы</option>
                         </select>
                         <br/>
                         
                         <div style="text-align:right;">
                           Индекс датчика (-1 - не указан):
                           <input type="text" id="statsmssensor2" value="-1" style="width:80px;"/>
                         </div>
                         
                         <br/><br/>
                         Подпись:<br/>
                         <input type="text" id="statsmslabel2" maxlength="20"/>
                                                   
                        <td>
                      <tr>
                      <tr>
                        <td colspan="2">
                          <button id='save_statsms_button' onclick='saveStatSmsSettings();'>Сохранить в контроллер</button>
                        </td>
                      </tr>
                    </table>
                    </div>
                    
                  </div>
                  
                  
                  <div class='content hdn' id='RESERVATION_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список резервирования датчиков</span></h1>
                    
                    <div class="row" id="RESERVATION_LIST_HEADER">
                      <div class="row_item ui-widget-header">#</div>
                      <div class="row_item ui-widget-header">Тип резервирования</div>
                      <div class="row_item ui-widget-header">Датчики</div>
                    </div>                    

                    <div id='RESERVATION_LIST'></div>
                    <br clear='left'/>
                    <div><br/><br/>
                    
                        <button id='save_sms_button' onclick='saveReservationList();'>Сохранить в контроллер</button>
                        <button id='new_sms_button' onclick='newReservation();'>Добавить резервирование</button>
                        
                    </div>
                    
                    <br/><br/>
                    <h2><span class='innerh'>Что такое список резервирования?</span></h2>
                    <div class='button_menu_spacer' style='font-size:80%;'>
                    Список резервирования - это горячая замена показаний одного датчика показаниями другого. Если два датчика 
                    находятся в одном списке резервирования и один из них вышел из строя, то для такого датчика будут браться показания 
                    с других датчиков, находящихся с ним в одном списке резервирования. Списки резервирования используются правилами при
                    проверке показаний с датчиков, и нужны для обеспечения бесперебойной работы правил. В остальном - показания с любого 
                    датчика выводятся "как есть".
                    </div>
                    
                  </div>                  
                  
                  
                  <div class='content hdn' id='RULES_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список правил</span></h1>
                    
                    <div id='RULES_LIST'></div>
                    <br clear='left'/>
                    <div><br/><br/>
                    
                        <button id='get_rules_button' onclick='doRequestRulesList();'>Получить список правил</button>
                        <button id='save_rules_button' onclick='saveRulesList();'>Сохранить в контроллер</button>
                        <button id='new_rule_button' onclick='newRule();'>Новое правило</button>
                        
                    </div>
                    
                  </div>

                  <div class='content hdn' id='CC_MENU_CONTENT'>
                  
                    <h1><span class='innerh'>Список составных команд</span></h1>
                    <div id='cc_lists_box' class='hdn'>
                        <select id='cc_lists' style='width:100%'></select>
                        <div class='padding_top8px'>
                          <button id='new_cc_list' onclick='newCCList();'>Новый список</button>
                          <button id='delete_cc_list' onclick='deleteCCList();'>Удалить текущий список</button>
                          <button id='save_cc_lists' onclick='saveCCLists();'>Сохранить данные в БД</button>
                        </div>

                      <br/>
                    </div>
                    <div id='CC_LIST'></div>
                    <br clear='left'/>
                    <div><br/><br/>
                    
                        <button id='save_cc_button' onclick="uploadCCCommands();">Загрузить команды в контроллер</button>
                        <button id='new_cc_button' onclick="newCCCommand();">Новая составная команда</button>
                        
                    </div>
                    
                  </div>

                  <div class='content hdn' id='WATER_MENU_CONTENT'>

                    <h1><span class='innerh'>Настройки полива</span></h1>
                    
                    
                    
                         <div>
                          Автоматическое управление поливом: <br/>
                          <select id='watering_option' style='width:100%'>
                            <option value='0'>Отключено</option>
                            <option value='1'>По дням недели (все каналы одновременно)</option>
                            <option value='2'>По дням недели (раздельное управление каналами)</option>
                          </select>
                         </div>
                         
                         <div id='watering_all_channels' class='hdn'>
                          <p>
                           
                            <div class='half'>
                                <div class='half_box half_left'>
                                  <div class='ui-widget-header ui-corner-top'>Дни недели</div>
                                  <div class='ui-widget-content padding_around8px left_align'id='all_watering_channels_days'>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='1'/>Понедельник</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='2'/>Вторник</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='4'/>Среда</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='8'/>Четверг</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='16'/>Пятница</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='32'/>Суббота</div>
                                      <div><input type='checkbox' id='all_watering_channels_day' value='64'/>Воскресенье</div>
                                  </div>
                                </div>
                            </div>    

                            <div class='half'>
                                <div class='half_box half_right'>

                                  <div class='ui-widget-header ui-corner-top'>Настройки</div>
                                  <div class='ui-widget-content padding_around8px left_align'>
                                    <div>
                                      Начало работы (чч:мм):<br/>
                                      <input type='text' maxlength='2' id='all_watering_start_hour' style='width:45%;'/> : 
                                      <input type='text' maxlength='2' id='all_watering_start_minute' style='width:45%;'/>
                                    </div>
                                    
                                    <div>
                                      Продолжительность полива, минут:<br/>
                                      <input type='text' maxlength='5' id='all_watering_time' style='width:100%;'/>
                                    </div>
                                    
                                    <div>
                                      Датчик влажности почвы:<br/>
                                      <input type='text' maxlength='5' id='all_watering_sensor_index' style='width:100%;'/>
                                    </div>

                                    <div>
                                      Влажность мин, %:<br/>
                                      <input type='text' maxlength='5' id='all_watering_start_border' style='width:100%;'/>
                                    </div>
									
									
                                    <div>
                                      Влажность макс, %:<br/>
                                      <input type='text' maxlength='5' id='all_watering_stop_border' style='width:100%;'/>
                                    </div>

                                  </div>

                                </div>
                            </div>    
                           
                          </p>
                         </div>
                         
                         <div id='watering_separate_channels' class='hdn'>
                          <p>
                            <div id='WATER_CHANNELS_LIST'></div>
                          </p>
                         </div>
                         
                         <div id='turn_pump_box' class='hdn'><br clear='left'/><br/>
                          <input type='checkbox' value='1' id='turn_on_pump'><label for='turn_on_pump'>Включать насос при поливе на любом из каналов</label>
                         </div>
            
                         <div id='switch_w_a_m_box' class='hdn'>
                          <input type='checkbox' value='1' id='switch_w_to_aam'><label for='switch_w_to_aam'>Переходить в автоматический режим после полуночи</label>
                         </div>
                                                  
                         <div><br clear='left'/><br/>
                         
                              <button id='save_watering_button' onclick="saveWateringSettings();">Сохранить в контроллер</button>

                         </div>
                     
                  </div>        
    
                  <div class='content' id='welcome'>

                    <h1><span class='innerh'>Настройки контроллера</span></h1>
                    
                      Для редактирования настроек контроллера выберите пункт меню слева.
 
                  </div>    
    
    </div>


</div>

{include file="controller_settings_helpers.tpl"}

{include file="commands_help.tpl"}