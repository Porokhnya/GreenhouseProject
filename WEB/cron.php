<?php
// скрипт работы по крону
// поскольку здесь нам Smarty не нужен, подключаем всё напрямую

// подключаем базовые определения
require_once($_SERVER['DOCUMENT_ROOT'] . "/base_define.php");
// подключаем работу с базой
require_once(INCLUDE_PATH . "utils/database.config.php");
// подключаем работу с сокетами
require_once(INCLUDE_PATH . "utils/socket_transport.php");


$SIMULATION = false; // ФЛАГ СИМУЛЯЦИИ, БЕЗ ЗАПИСИ В БД
$SAVE_CONTROLLER_STATE = false; // сохранять или нет состояние полива/окон/досветки в БД

// массив возможных состояний контроллера
$states = array();
// массив типов сенсоров
$sensor_types = array();
// массив модулей
$modules = array();
// список зарегистрированных контроллеров
$controllers = array();


if($SIMULATION)
  echo "<pre>";

function BitIsSet($val, $pos)
{
  return ($val & (1 << $pos)) != 0;
}

function requestControllerData($controller_id,$address)
{
  global $dbengine;
  global $states;
  global $sensor_types;
  global $modules;
  global $SIMULATION;
  global $SAVE_CONTROLLER_STATE;
  
  $NO_TEMPERATURE_DATA = "-128.00"; // нет показаний с датчика температуры
  $NO_LUMINOSITY_DATA = "-1.00"; // нет показаний с датчика освещенности
  $NO_CO2_DATA = "-1.00"; // нет показаний с датчика CO2
  $NO_EC_DATA = "-1.00"; // нет показаний с датчика EC
  $NO_PH_DATA = "-128.00"; // нет показаний с датчика pH
  
  // пытаемся законнектиться
  $tp = new SocketTransport();
  if($tp->open($address))
  {
    // законнектились
    if(!$SIMULATION)
    {
      $dbengine->beginTransaction();
      $dbengine->exec("UPDATE controllers SET is_online=1 WHERE controller_id=$controller_id;");
      $dbengine->commitTransaction();
    }
    
    // теперь собираем данные
    $line = trim($tp->ctget('0|STAT'));
    
    // проверяем, ок ли ответ?
    $pos = strpos($line, 'OK=');
    if(!($pos === false))
    {
      // ответ OK, продолжаем парсить
      $line = substr($line,3);
      
      if($SIMULATION)
        echo $line . "\n";
      
      // теперь разбираем, чего там пришло
      $firstByte = hexdec(substr($line,0, 2));
      $secondByte = hexdec(substr($line,2, 2));
      
      if($SIMULATION)
      {
        echo $firstByte . "\n";
        echo $secondByte . "\n";
      }
      
      $line = substr($line,4);
      
      if($SIMULATION)
        echo $line . "\n";
      
      // проверяем состояния
      $windows_open = BitIsSet($firstByte,0) ? 1 : 0;
      $windows_auto_mode = BitIsSet($firstByte,1) ? 1 : 0 ;

      $water_on = BitIsSet($firstByte,2) ? 1 : 0;
      $water_auto_mode = BitIsSet($firstByte,3) ? 1 : 0 ;

      $light_on = BitIsSet($firstByte,4) ? 1 : 0;
      $light_auto_mode = BitIsSet($firstByte,5) ? 1 : 0 ;

      if($SIMULATION)
      {
        echo "windows open: $windows_open\nwindows auto mode: $windows_auto_mode\nwater on: $water_on\nwater auto mode: $water_auto_mode\nlight on: $light_on\nlight auto mode: $light_auto_mode\n";
      }
      
      // теперь генерируем набор SQL для вставки
      if(!$SIMULATION)
        $dbengine->beginTransaction();
        
        if($SAVE_CONTROLLER_STATE)
        {
      
            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['WINDOWS']}, $windows_open);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);

            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['WINDOWS_MODE']}, $windows_auto_mode);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);

            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['WATER']}, $water_on);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);

            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['WATER_MODE']}, $water_auto_mode);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);

            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['LIGHT']}, $light_on);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);

            $sql = "INSERT INTO controller_state(controller_id,state_id,state) VALUES($controller_id, {$states['LIGHT_MODE']}, $light_auto_mode);";
            if($SIMULATION)
              echo $sql . "\n";
            else
              $dbengine->exec($sql);
          
        } // if($SAVE_CONTROLLER_STATE)


      // тут уже обновили состояние контроллера, пора переходить к данным
      while(strlen($line) > 0)
      {

        $f = substr($line,0, 4);
        $line = substr($line,4);
        
        $flags = hexdec($f);
        $tempPresent = ($flags & 1) == 1;
        $luminosityPresent = ($flags & 4) == 4;
        $humidityPresent = ($flags & 8) == 8;
        $waterFlowInstantPresent = ($flags & 16) == 16;
        $waterFlowIncrementalPresent = ($flags & 32) == 32;
        $soilMoisturePresent = ($flags & 64) == 64;        
        $phPresent = ($flags & 128) == 128;
        $co2Present = ($flags & 2) == 2;
        $ecPresent = ($flags & 256) == 256;
        
        // читаем байт имени модуля
         $namelen = hexdec(substr($line,0, 2));
         $moduleName = substr($line,2, $namelen);
         
         // получаем ID модуля
         $module_id = intval(@$modules[$moduleName]);
         
         if($SIMULATION)
          echo "Module name: $moduleName; module id: $module_id\n";
         
         $line = substr($line,2 + $namelen);
         
         $cnt = 0;

         // прочитали имя модуля, можем работать с датчиками
         // сперва идут температурные, первый байт - их количество
          if ($tempPresent)
          {
              $cnt =  hexdec(substr($line,0, 2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['TEMP']);

              // теперь читаем данные с датчиков
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0, 2));
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);

                  // теперь смотрим, есть ли показания с датчика
                  $haveSensorData = !($val == "FF" && $fract == "FF");
                  $temp = $NO_TEMPERATURE_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $hexTemp = hexdec($val);
                      
                      // конвертируем в знаковое представление
                      if(($hexTemp & 0x80) > 0)
                        $hexTemp = $hexTemp - 0x100;
                      
                      $temp = '' . $hexTemp . ".";
                      $fractVal = hexdec($fract);
                      
                      if ($fractVal < 10)
                          $temp .= "0";
                      $temp .= '' . $fractVal;
                  }
                  
                  if($moduleName != "0") // игнорируем температуру прибора
                  {
                    // получили показания с датчика, надо их сохранить в БД
                    $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$temp);";
                    if($SIMULATION)
                      echo "$sql\n";
                    else
                      $dbengine->exec($sql);
                      
                  } // if
                  
              } // for
          } // if($tempPresent)
          // опрос температурных датчиков окончен, переходим на датчики влажности
          
          if ($humidityPresent)
          {
              // переходим на чтение данных с датчиков влажности
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['HUMIDITY']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0,2));
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);

                  // теперь смотрим, есть ли показания с датчика
                  $haveSensorData = !($val == "FF" && $fract == "FF");
                  $humidity = $NO_TEMPERATURE_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $humidity = '' . hexdec($val) . ".";
                      $fractVal = hexdec($fract);
                      if ($fractVal < 10)
                          $humidity .= "0";
                      $humidity .= '' . $fractVal;
                  }

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$humidity);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);

              } // for
          } // if($humidityPresent)  
          
          if ($luminosityPresent)
          {
              // далее идут показания датчиков освещенности
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['LIGHT']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0,2));
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);

                  // теперь смотрим, есть ли показания с датчика
                  $haveSensorData = !($val == "FF" && $fract == "FF");
                  $luminosity = $NO_LUMINOSITY_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $luminosity = hexdec( ($val . $fract) );

                  }

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$luminosity);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);

              } // for
          } // $luminosityPresent

          if ($waterFlowInstantPresent)
          {
              // далее идут показания датчиков мгновенного расхода воды
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['FLOW_INSTANT']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                 $sensorIdx = hexdec(substr($line,0,2));

                  // затем 4 байта - его показания
                  $dt = substr($line,2, 8);
                  $line = substr($line,10);
                  $flow = hexdec($dt);

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$flow);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);

              } // for
          } // $waterFlowInstantPresent


          if ($waterFlowIncrementalPresent)
          {
              // далее идут показания датчиков накопительного расхода воды
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['FLOW_INCREMENTAL']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0,2));

                  // затем 4 байта - его показания
                  $dt = substr($line,2, 8);
                  $line = substr($line,10);
                  $flow = hexdec($dt);

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$flow);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);

              } // for
              
              
          } // $waterFlowIncrementalPresent

          // разбираем показания с датчиков влажности почвы
          if ($soilMoisturePresent)
          {
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
                            
              $sensor_type_id = intval(@$sensor_types['SOIL']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                 $sensorIdx = hexdec(substr($line,0,2));
                 
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);
                  
                  // теперь смотрим, есть ли показания с датчика
                 $haveSensorData = !($val == "FF" && $fract == "FF");
                 $temp = $NO_TEMPERATURE_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $temp = '' . hexdec($val) . ".";
                      $fractVal = hexdec($fract);
                      if ($fractVal < 10)
                          $temp .= "0";
                      $temp .= '' . $fractVal;
                  }

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$temp);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);
              } // for
          } // if($soilMoisturePresent)
          
          if ($phPresent)
          {
              // переходим на чтение данных с датчиков PH
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
              $sensor_type_id = intval(@$sensor_types['PH']);

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                 $sensorIdx = hexdec(substr($line,0,2));
                 
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  // затем два байта - вольтаж, мы его просто игнорируем
                  $phMV = substr($line,6,4);
                  
                  // переходим на следующий датчик
                  $line = substr($line,10);

                  // теперь смотрим, есть ли показания с датчика
                  // теперь смотрим, есть ли показания с датчика
                 $haveSensorData = !($val == "FF" && $fract == "FF");
                 $phValue = $NO_PH_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $phValue = '' . hexdec($val) . ".";
                      $fractVal = hexdec($fract);
                      if ($fractVal < 10)
                          $phValue .= "0";
                      $phValue .= '' . $fractVal;
                  }

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$phValue);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);
                  

               } // for
               
            } // if($phPresent)
          
	  
		if ($co2Present)
          {
              // далее идут показания датчиков CO2
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
			  /*
              $sensor_type_id = intval(@$sensor_types['CO2']);
			  */

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0,2));
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);

                  // теперь смотрим, есть ли показания с датчика
                  $haveSensorData = !($val == "FF" && $fract == "FF");
                  $ppm = $NO_CO2_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $ppm = hexdec( ($val . $fract) );

                  }
				  
				  /*

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$ppm);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);
				*/

              } // for
          } // $co2Present	

		if ($ecPresent)
          {
              // далее идут показания датчиков EC
              $cnt = hexdec(substr($line,0,2));
              $line = substr($line,2);
              
			  /*
              $sensor_type_id = intval(@$sensor_types['EC']);
			  */

              // обрабатываем их
              for ($i = 0; $i < $cnt; $i++)
              {
                  // первым байтом идёт индекс датчика
                  $sensorIdx = hexdec(substr($line,0,2));
                  // затем два байта - его показания
                  $val = substr($line,2, 2);
                  $fract = substr($line,4, 2);
                  $line = substr($line,6);

                  // теперь смотрим, есть ли показания с датчика
                  $haveSensorData = !($val == "FF" && $fract == "FF");
                  $ppm = $NO_EC_DATA;
                  if ($haveSensorData)
                  {
                      // имеем показания, надо сконвертировать
                      $ppm = hexdec( ($val . $fract) );

                  }
				  
				  /*

                  // получили показания с датчика, надо их сохранить в БД
                  $sql = "INSERT INTO controller_data(controller_id,sensor_type_id,module_id,sensor_index,sensor_data) VALUES($controller_id,$sensor_type_id,$module_id,$sensorIdx,$ppm);";
                  if($SIMULATION)
                    echo "$sql\n";
                  else
                    $dbengine->exec($sql);
				*/

              } // for
          } // $ecPresent	  
		  

          // все датчики обработали, переходим к следующему модулю          
          
        
      } // while

      
      if(!$SIMULATION)
        $dbengine->commitTransaction();
      
    } // if ok answer
    
    $tp->close(); // закрываем соединение
  } // if open
  else
  {
    // контроллер оффлайн
    if(!$SIMULATION)
    {
      $dbengine->beginTransaction();
      $dbengine->exec("UPDATE controllers SET is_online=0 WHERE controller_id=$controller_id;");
      $dbengine->commitTransaction();
    }
    
  }
  
  
} // function

if($SAVE_CONTROLLER_STATE)
{
    // заполняем массив состояний
    $res = $dbengine->query("SELECT state_id, state_name FROM states;");
    if($res !== FALSE)
    {
      while($arr = $res->fetchArray())
      {
        $states[ $arr['state_name'] ] = $arr['state_id'];
      }
    }
    
} // $SAVE_CONTROLLER_STATE

// заполняем массив типов сенсоров
$res = $dbengine->query("SELECT sensor_type_id, sensor_type FROM sensor_types;");
if($res !== FALSE)
{
  while($arr = $res->fetchArray())
  {
    $sensor_types[ $arr['sensor_type'] ] = $arr['sensor_type_id'];
  }
}
  
// заполняем массив типов модулей
$res = $dbengine->query("SELECT module_id, module_name FROM modules;");
if($res !== FALSE)
{
  while($arr = $res->fetchArray())
  {
    $modules[ $arr['module_name'] ] = $arr['module_id'];
  }
}
 

// заполняем список зарегистрированных контроллеров
$res = $dbengine->query("SELECT controller_id, controller_address FROM controllers;");
if($res !== FALSE)
{
  while($arr = $res->fetchArray())
  {
    $controllers[ $arr['controller_id'] ] = $arr['controller_address'];
  }
}


// начинаем работать
foreach($controllers as $id => $address)
{
  requestControllerData($id,$address);
}

?>
