#pragma once

#if (TARGET_BOARD != DUE_BOARD)
  #error "Target board MUST BE DUE_BOARD for this configuration!"
#endif

//--------------------------------------------------------------------------------------------------------------------------------
#include "REG_KEY.h" // подключаем данные регистрационного ключа
//--------------------------------------------------------------------------------------------------------------------------------
// настройки альтернативного модуля управления фрамугами
//--------------------------------------------------------------------------------------------------------------------------------
#define WM_BACK_CAPTION "$ НАЗАД"
#define WM_SAVE_CAPTION "СОХРАНИТЬ"
#define WM_WINDOWS_CAPTION "0 ФРАМУГИ"
#define WM_SCENE_BUTTON_CAPTION "9 СЦЕНАРИИ" 
#define WM_RAIN_CAPTION "ДАТЧИК ДОЖДЯ"
#define WM_WIND_CAPTION "ДАТЧИК ВЕТРА"
#define WM_OPTIONS_CAPTION "1 ОПЦИИ"
#define WM_T25_CAPTION "^ открытия 25%"
#define WM_T50_CAPTION "^ открытия 50%"
#define WM_T75_CAPTION "^ открытия 75%"
#define WM_T100_CAPTION "^ открытия 100%"
#define WM_HISTERESIS_CAPTION "Гистерезис"
#define WM_SENSOR_CAPTION "Датчик N"
#define WM_ON_CAPTION "МОДУЛЬ ВКЛ"
#define WM_OFF_CAPTION "МОДУЛЬ ВЫКЛ"
#define WM_TSECTION_1_CAPTION "^ секции N1"
#define WM_TSECTION_2_CAPTION "^ секции N2"
#define WM_TSECTION_3_CAPTION "^ секции N3"
#define WM_TSECTION_4_CAPTION "^ секции N4"
#define WM_WLIST_CAPTION "ОКНA ПО СЕКЦИЯМ"
#define WM_CLOCK_CAPTION "6 ЧАСЫ И ПР."
#define WM_ENTERKEY_CAPTION "ВВОД КЛЮЧА"
#define RESTRICTED_FEATURES F("ОГРАНИЧЕННАЯ ВЕРСИЯ")
#define UNAVAIL_FEATURE F("НЕДОСТУПНО")
#define WTRFLOW_CAPTION1 "РАСХОД ВОДЫ #1"
#define WTRFLOW_CAPTION2 "РАСХОД ВОДЫ #2"
#define WINDSPEED_CAPTION "ВЕТЕР"
#define WINDSPEED_DIRECTION_CAPTION "НАПРАВЛЕНИЕ"
#define RAIN_BOX_CAPTION "ДОЖДЬ"
#define RAIN_BOX_YES "ЕСТЬ"
#define RAIN_BOX_NO "НЕТ"
#define WM_DRIVE_CAPTION "8 УПРАВЛЕНИЕ"
#define WM_WATERING_CAPTION "4 ПОЛИВ"
#define WM_LIGHT_CAPTION "5 ДОСВЕТКА"

#define WS_25_CAPTION "ОТКРЫТИЕ НА 25%"
#define WS_50_CAPTION "ОТКРЫТИЕ НА 50%"
#define WS_75_CAPTION "ОТКРЫТИЕ НА 75%"
#define WS_100_CAPTION "ОТКРЫТИЕ НА 100%"

#define FLOW_CONTROL_BUTTON "2РАСХОДОМЕРЫ"
#define FLOW_RESET_BUTTON1 "СБРОСИТЬ РАСХОД #1"
#define FLOW_RESET_BUTTON2 "СБРОСИТЬ РАСХОД #2"
#define FLOW_CAL1_CAPTION "Калибровка #1"
#define FLOW_CAL2_CAPTION "Калибровка #2"

#define WM_WIND_SPEED_CAPTION "Порог закрытия, м/с"
#define WM_HURRICANE_SPEED_CAPTION "Ураган, м/с"

#define WM_ORIENTATION_CAPTION "ОРИЕНТАЦИЯ ОКОН"
#define WM_NORTH_CAPTION "Север"
#define WM_SOUTH_CAPTION "Юг"
#define WM_EAST_CAPTION "Восток"
#define WM_WEST_CAPTION "Запад"

#define WM_LIGHT_BEGIN_H_CAPTION "Начало, ч:"
#define WM_LIGHT_BEGIN_DH_CAPTION "Длительность, ч:"
#define WM_LIGHT_LUM_CAPTION "Порог, люкс:"
#define WM_LIGHT_HISTERESIS_CAPTION "Гистерезис:"

#define WM_RAIN_CLOSE_CAPTION "Закрыть"
#define WM_RAIN_BGCOLOR VGA_GREEN
#define WM_RAIN_FONT_COLOR VGA_WHITE

#define WM_SECTION_NOLINK_CAPTION "-"
#define WM_SECTION_LINK1_CAPTION "Секция 1"
#define WM_SECTION_LINK2_CAPTION "Секция 2"
#define WM_SECTION_LINK3_CAPTION "Секция 3"
#define WM_SECTION_LINK4_CAPTION "Секция 4"
#define WM_SECTION_LABEL_PREFIX "Окно #"

#define CYCLE_VENT_BUTTON_CAPTION "<РЕЦИРКУЛЯЦИЯ"  //БЫЛО: "ВОЗДУХООБМЕН" - КМВ
#define CYCLE_VENT_SCREEN_CAPTION "РЕЦИРКУЛЯЦИЯ"

#define CHANNEL_N_1_CAPTION "КАНАЛ #1"
#define CHANNEL_N_2_CAPTION "КАНАЛ #2"
#define CHANNEL_N_3_CAPTION "КАНАЛ #3"
#define CYCLE_VENT_STIME_CAPTION "НАЧАЛО:"
#define CYCLE_VENT_ETIME_CAPTION "КОНЕЦ:"
#define CYCLE_VENT_WTIME_CAPTION "РАБОТА, МИН:"
#define CYCLE_VENT_ITIME_CAPTION "ОТДЫХ, МИН:"


#define VENT_BUTTON_CAPTION "= ВЕНТИЛЯЦИЯ"
#define VENT_SCREEN_CAPTION "ВЕНТИЛЯЦИЯ"

#define VENT_MIN_WORKTIME_CAPTION "Мин. выбег, мин:сек:"
#define VENT_MAX_WORKTIME_CAPTION "Макс. выбег, мин:сек:"
#define VENT_REST_TIME_CAPTION "Отдых, мин:сек:"
#define VENT_SENSOR_CAPTION "Датчик:"
#define VENT_TEMP_CAPTION "Порог температуры:"
#define VENT_HISTERESIS_CAPTION "Гистерезис:"
#define VENT_MAX_WORK_TIME 5999l


#define THERMOSTAT_BUTTON_CAPTION "> ТЕРМОСТАТ"
#define THERMOSTAT_SCREEN_CAPTION "ТЕРМОСТАТ"

#define THERMOSTAT_SENSOR_CAPTION "Датчик:"
#define THERMOSTAT_TEMP_CAPTION "Температура:"
#define THERMOSTAT_HISTERESIS_CAPTION "Гистерезис:"

#define CONTROL_PH_CAPTION "?КОНТРОЛЬ pH"
#define CONTROL_PH_SCREEN_CAPTION "КОНТРОЛЬ pH"

#define CONTROL_CO2_CAPTION "BКОНТРОЛЬ CO2"
#define CONTROL_CO2_SCREEN_CAPTION "КОНТРОЛЬ CO2"

#define DOORS_BUTTON_CAPTION "C ДВЕРИ"
#define DOORS_SCREEN_CAPTION "ДВЕРИ"

#define SPRAY_BUTTON_CAPTION "D СПРИНКЛЕРЫ"
#define SPRAY_SCREEN_CAPTION "СПРИНКЛЕРЫ"

#define WM_BLINK_ON_TEXT_COLOR VGA_WHITE
#define WM_BLINK_OFF_TEXT_COLOR 0x3A8D
#define WM_ON_BLINK_BGCOLOR VGA_MAROON
#define WM_OFF_BLINK_BGCOLOR VGA_SILVER

#define WM_SECTION_1_BGCOLOR VGA_YELLOW
#define WM_SECTION_1_FONT_COLOR VGA_BLACK

#define WM_SECTION_2_BGCOLOR VGA_GREEN
#define WM_SECTION_2_FONT_COLOR VGA_WHITE

#define WM_SECTION_3_BGCOLOR VGA_BLUE
#define WM_SECTION_3_FONT_COLOR VGA_WHITE

#define WM_SECTION_4_BGCOLOR VGA_MAROON
#define WM_SECTION_4_FONT_COLOR VGA_WHITE

// скорости ветра по умолчанию, м/с
#define WM_DEFAULT_WIND_SPEED 15
#define WM_DEFAULT_HURRICANE_SPEED 20

// температуры открытия по умолчанию
#define WM_25PERCENTS_OPEN_TEMP 26
#define WM_50PERCENTS_OPEN_TEMP 28
#define WM_75PERCENTS_OPEN_TEMP 30
#define WM_100PERCENTS_OPEN_TEMP 32
#define WM_DEFAULT_HISTERESIS 5 // гистерезис по умолчанию, десятые доли, шаг - 5 десятых
#define WM_ACTIVE_FLAG true // признак активности модуля управления окнами, по умолчанию

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля управления отоплением
//--------------------------------------------------------------------------------------------------------------------------------
#define HEAT_BUTTON_CAPTION "7 ОТОПЛЕНИЕ"
#define HEAT_BUTTON_CAPTION1 "КОНТУР ВЕРХНИЙ"
#define HEAT_BUTTON_CAPTION2 "КОНТУР БОКОВОЙ"
#define HEAT_BUTTON_CAPTION3 "КОНТУР ПОЧВА"
#define HEAT_MIN_TEMP_CAPTION "Мин. ^ контура"
#define HEAT_MAX_TEMP_CAPTION "Макс. ^ контура"
#define HEAT_ETHALON_TEMP_CAPTION "^ климата"
#define HEAT_HISTERESIS_CAPTION "Гистерезис"
#define HEAT_SENSOR_CAPTION "Контур Д"
#define HEAT_AIR_CAPTION "Климат Д"
#define HEAT_WORKTIME_CAPTION "Работа привода, с"
#define HEAT_AIR "ВОЗДУХ"

#define HEAT_INFO_BOX_CONTOUR "КОНТУР:"
#define HEAT_INFO_BOX_MODULE "МОДУЛЬ:"

#define HEAT_INFO_BOX_CONTOUR_ON "ВКЛ"
#define HEAT_INFO_BOX_CONTOUR_OFF "ВЫКЛ"

#define HEAT_INFO_BOX_MODULE_ON "ВКЛ"
#define HEAT_INFO_BOX_MODULE_OFF "ВЫКЛ"

#define HEAT_DRIVE_CHANNEL_ON "ВКЛЮЧЕН"
#define HEAT_DRIVE_CHANNEL_OFF "ВЫКЛЮЧЕН"


#define HEAT_DEFAULT_MIN_TEMP 15
#define HEAT_DEFAULT_MAX_TEMP 90
#define HEAT_DEFAULT_ETHALON_TEMP 25
#define HEAT_DEFAULT_HISTERESIS 20

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля затенения
//--------------------------------------------------------------------------------------------------------------------------------
#define WM_SHADOW_CAPTION "3 ЗАТЕНЕНИЕ"
#define SHADOW_BUTTON_CAPTION1 "ШТОРА #1"
#define SHADOW_BUTTON_CAPTION2 "ШТОРА #2"
#define SHADOW_BUTTON_CAPTION3 "ШТОРА #3"
#define SHADOW_WORKTIME_CAPTION "Работа привода, с"
#define SHADOW_HISTERESIS_CAPTION "Гистерезис"
#define SHADOW_BORDER_CAPTION "Порог, люкс"
#define HEAT_SENSORINDEX_CAPTION "Датчик"
#define SHADOW_DRIVE_CHANNEL_ON "РАСКРЫТА"
#define SHADOW_DRIVE_CHANNEL_OFF "ЗАКРЫТА"

//--------------------------------------------------------------------------------------------------------------------------------
// закомментировать, если не нужно управление окнами по температуре
#define USE_TEMP_SENSORS
#define SUPPORTED_WINDOWS 16 // кол-во поддерживаемых окон, максимум 16 (по два реле на мотор, для 8-ми канального модуля реле - 4 окна)
#define WATER_RELAYS_COUNT 16 // сколько каналов управления поливом используется (максимум - 16)
// уровни для реле каналов полива
#define WATER_RELAY_ON LOW // уровень для включения реле
#define LIGHT_RELAY_ON LOW // уровень для включения реле (можно менять через конфигуратор потом)
#define RELAY_ON LOW // уровень для включения реле фрамуг (можно менять через конфигуратор потом)
//--------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля измерения давления
//--------------------------------------------------------------------------------------------------------------------------------
#define MIN_SEALEVEL_VAL -100
#define MAX_SEALEVEL_VAL 8848
#define PREDICT_MEASURES 10 // какое кол-во измерений давления использовать для предсказания
#define PREDICT_INTERVAL 600000 // интервал между сбором предсказаний по давлению, мс
#define TFT_SEALEVEL_CAPTION "Уровень моря, м:"

//--------------------------------------------------------------------------------------------------------------------------------
// настройки EEPROM для хранения данных
//--------------------------------------------------------------------------------------------------------------------------------
#define SETT_HEADER1 0x1F // байты, сигнализирующие о наличии сохранённых настроек, первый
#define SETT_HEADER2 0xBF // и второй

//--------------------------------------------------------------------------------------------------------------------------------
// адреса хранения данных в EEPROM (МИНИМАЛЬНЫЙ ОБЪЁМ EEPROM - 16 Кб !!!)
//--------------------------------------------------------------------------------------------------------------------------------

#define CONTROLLER_ID_EEPROM_ADDR 1 // по какому адресу располагается ID контроллера, 1 байт
#define UNI_SENSOR_INDICIES_EEPROM_ADDR 2 // с какого адреса идут выданные индексы для универсальных сенсоров, 10 байт

#define WIFI_STATE_EEPROM_ADDR 12// адрес хранения состояния Wi-Fi (коннектится к роутеру или нет), 1 байт
#define STATION_PASSWORD_EEPROM_ADDR 13// адрес хранения пароля к точке ESP, 20 байт
#define STATION_ID_EEPROM_ADDR 33// адрес хранения ID точки доступа ESP, 20 байт
#define ROUTER_PASSWORD_EEPROM_ADDR 53// адрес хранения пароля к роутеру, 20 байт
#define ROUTER_ID_EEPROM_ADDR 73// адрес хранение ID роутера, 20 байт

#define SMS_NUMBER_EEPROM_ADDR 93 // адрес хранения номера телефона хозяина, 15 байт

// НЕ ИСПОЛЬЗУЕТСЯ !!!
#define GSM_PROVIDER_EEPROM_ADDR 108 // адрес хранения оператора GSM, 1 байт

#define IOT_SETTINGS_EEPROM_ADDR 109 // адрес хранения настроек IOT, 51 байт
#define OPEN_INTERVAL_EEPROM_ADDR 160 // адрес хранения настроек интервала открытия окон, 4 байта
#define CLOSE_TEMP_EEPROM_ADDR 164 // адрес хранения температуры закрытия, 1 байт
#define OPEN_TEMP_EEPROM_ADDR 165 // адрес хранения температуры открытия, 1 байт

#define WATERING_OPTION_EEPROM_ADDR 166 // адрес хранения текущей опции полива, 1 байт
#define TURN_PUMP_EEPROM_ADDR 167 // адрес хранения флага - включить ли насос при поливе, 1 байт
#define START_WATERING_TIME_EEPROM_ADDR 168 // адрес хранения начала полива для всех каналов, 2 байта
#define WATERING_TIME_EEPROM_ADDR 170 // адрес хранения продолжительности полива для всех каналов, 2 байта
#define WATERING_WEEKDAYS_EEPROM_ADDR 172 // адрес хранения маски дней недели полива на всех каналах, 1 байт
#define WATERING_SENSOR_EEPROM_ADDR 173 // адрес хранения индекса датчика в модуле влажности почв, показания с которого учитываются при поливе, 1 байт
#define WATERING_STOP_BORDER_EEPROM_ADDR 174 // адрес хранения показаний с датчика, по которым полив на всех каналах выключается, 1 байт
#define WATERING_CHANNELS_SETTINGS_EEPROM_ADDR 175 // адрес начала настроек каналов полива, 16 каналов*7 байт на канал - 112 байт
#define WATERING_TURN_TO_AUTOMODE_AFTER_MIDNIGHT_ADDRESS 299 // адрес настройки "переходить в авторежим после полуночи", 1 байт
#define WATERING_STATUS_EEPROM_ADDR 300 // с какого адреса у нас идут статусы каналов полива, по 5 байт на канал, 100 байт

#define WATERFLOW_EEPROM_ADDR 400 // с какого адреса у нас будут записываться показания датчиков расхода воды, 12 байт

#define DELTA_SETTINGS_EEPROM_ADDR 412 // с какого адреса в EEPROM начинаются настройки дельт, 500 байт на 20 дельт
#define PH_SETTINGS_EEPROM_ADDR 912 // с какого адреса идут настройки PH-модуля, 30 байт
#define TIMERS_EEPROM_ADDR 942 // у нас 4 таймера, на каждый - 10 байт + заголовок (2 байта), итого - 42 байта 
#define RESERVATION_ADDR 984 // адрес, с которого пишутся настройки резервирования (10 списков по 12 байт + 3 байта = 123 байта)
#define GUID_ADDRESS 1110 // адрес, по которому хранится уникальный GUID контроллера (32 символа без пробелов + 2 байта - заголовок присутствия - 34 байта)
#define HTTP_API_KEY_ADDRESS 1144 // адрес, ко которому хранится ключ доступа к HTTP API (32 символа без пробелов + 2 байта - заголовок присутствия + 1 байт - флаг активности, вкл/выкл) - 35 байт
#define HTTP_SEND_SENSORS_DATA_ADDRESS 1179 // адрес хранения флага - отсылать ли слепок показаний датчиков контроллера при проверке задач по HTTP, 1 байт
#define TIMEZONE_ADDRESS 1180 // адрес хранения часового пояса контроллера, 4 байта (2 байта заголовок присутствия, 2 байта - таймзона). Хранится в минутах
#define HTTP_SEND_STATUS_ADDRESS 1184 // адрес хранения флага - отсылать ли слепок состояния контроллера при проверке задач по HTTP, 1 байт

#define MQTT_ENABLED_FLAG_ADDRESS 1185 // адрес хранения флага - активен ли MQTT-клиент, 1 байт
#define MQTT_INTERVAL_BETWEEN_TOPICS_ADDRESS 1186 // адрес хранения интервала (в секундах) между публикацией топиков в брокер MQTT, 1 байт

#define WM_T25_ADDRESS 1200 // адрес хранения температуры открытия на 25%, 4 байта*4канала
#define WM_T50_ADDRESS 1216 // адрес хранения температуры открытия на 50%, 4 байта*4канала
#define WM_T75_ADDRESS 1232 // адрес хранения температуры открытия на 75%, 4 байта*4канала
#define WM_T100_ADDRESS 1248 // адрес хранения температуры открытия на 100%, 4 байта*4канала
#define WM_HISTERESIS_ADDRESS 1264 // адрес хранения значения гистерезиса, 4 байта*4канала
#define WM_SENSOR_ADDRESS 1280 // адрес хранения индекса датчика, 4 байта*4канала
#define WM_ACTIVE_ADDRESS 1296 // адрес хранения флага активности, 3 байта*4канала
#define WM_BINDING_ADDRESS 1308 // адрес хранения привязок каналов фрамуг к секциям контроля, 18 байт
#define WM_RAIN_BINDING_ADDRESS 1326 // адрес хранения привязок каналов фрамуг к датчику дождя, 18 байт
#define WM_ORIENTATION_BINDING_ADDRESS 1344 // адрес хранения привязок ориентации окон, 18 байт
#define WM_WIND_SPEED_ADDRESS 1362 // адрес хранения уставки скорости ветра, 4 байта
#define WM_HURRICANE_SPEED_ADDRESS 1366 // адрес хранения уставки скорости урагана, 4 байта
#define WM_HEAT_SETTINGS_ADDRESS 1370 // адрес хранения настроек контуров отопления, 15 байт*3 контура
#define WM_HEAT_WORKTIME_ADDRESS  1415 // адрес хранения времени работы приводов отопления, 4 байта*3 контура
#define WM_SEALEVEL_ADDRESS 1450 // адрес хранения значения высоты над уровнем моря, 4 байта
#define WM_SHADOW_WORKTIME_ADDRESS  1454 // адрес хранения времени работы приводов штор, 4 байта*3 контура
#define WM_LIGHT_SETTINGS_ADDRESS 1470 // адрес хранения настроек досветки, 13 байт
#define WM_SHADOW_SETTINGS_ADDRESS 1490 // адрес хранения настроек каналов штор, 10 байт*3 канала
#define WM_KEY_ADDRESS	1550 // адрес хранения регистрационного ключа, 10 байт

#define WATERING_START_BORDER_ADDRESS 1563 // настройки хранения нижнего порога включения каналов полива, 17 байт

#define SYNC_SETTINGS_ADDRESS 1600 // адрес хранения настроек синхронизации времени, 60 байт
#define WM_CYCLE_SETTINGS_ADDRESS 1700 // адрес хранения настроек циркуляционной вентиляции, 16 байт*3 канала
#define WM_VENT_SETTINGS_ADDRESS  1800 // адрес хранения настроек вентиляции по температуре, 20 байт*3 канала
#define WM_THERMOSTAT_SETTINGS_ADDRESS  1900 // адрес хранения настроек термостатов, 10 байт*3 канала
#define WM_CO2_SETTINGS_ADDRESS 1930  // адрес хранения настроек контроля СО2, 16 байт*3 канала

#define SCHEDULE_ACTIVE_FLAG_ADDRESS  1990 // адрес хранения флага активности расписаний (1 байт)
#define SCHEDULE_LAST_RUN_DATE_ADDRESS 1991 // адрес хранения даты последнего пакетного выполнения расписаний (6 байт)

// настройки хранения информации по GSM
#define GSM_PROVIDER_NAME_ADDRESS 2000 // адрес хранения имени провайдера, 20 байт, включая завершающий 0
#define GSM_APN_ADDRESS  2020 // адрес APN, 50 байт, включая завершающий 0
#define GSM_APN_USER_ADDRESS 2070 // адрес имени пользователя APN, 20 байт, включая завершающий 0
#define GSM_APN_PASS_ADDRESS 2090 // адрес пароля пользователя APN, 20 байт, включая завершающий 0
#define GSM_BALANCE_CUSD_ADDRESS 2110 // адрес хранения команды запроса баланса, 50 байт, включая завершающий 0

// адрес хранения привязок DS18B20 к адресам, максимум 100 записей, одна запись - 10 байт, т.е. отводится 1000 байт на это дело
#define DS18B20_BINDING_ADDRESS 2200

#define COMPOSITE_COMMANDS_START_ADDR 3248 // с четвёртого килобайта в EEPROM идут составные команды

#define EEPROM_RULES_START_ADDR 5120 // с пятого килобайта в EEPROM идут правила

#define WATER_TANK_BINDING_ADDRESS  10000 // тут хранятся привезки модуля бака в настройкам, 50 байт (про запас)
#define CO2_BINDING_ADDRESS 10300 // тут хранятся привязки CO2 к железу, 100 байт (про запас)
#define PH_BINDING_ADDRESS 10400 // тут хранятся привязки pH к железу, 100 байт (про запас)
#define THERMOSTAT_BINDING_ADDRESS 10500 // тут хранятся привязки термостата к железу, 100 байт (про запас)
#define VENT_BINDING_ADDRESS 10600 // тут хранятся привязки вентиляции к железу, 100 байт (про запас)
#define CYCLE_VENT_BINDING_ADDRESS 10700 // тут хранятся привязки воздухообмена к железу, 100 байт (про запас)
#define SHADOW_BINDING_ADDRESS 10800 // тут хранятся привязки затенения к железу, 100 байт (про запас)
#define HEAT_BINDING_ADDRESS 10900 // тут хранятся привязки отопления к железу, 100 байт (про запас)
#define LIGHT_BINDING_ADDRESS 11000 // тут хранятся привязки досветки к железу, 100 байт (про запас)
#define RAIN_BINDING_ADDRESS 11100 // тут хранятся привязки датчика дождя к железу, 100 байт (про запас)
#define WIND_BINDING_ADDRESS 11200 // тут хранятся привязки датчика ветра к железу, 100 байт (про запас)
#define BUZZER_BINDING_ADDRESS 11300 // тут хранятся привязки пищалки к железу, 100 байт (про запас)
#define NRF_BINDING_ADDRESS 11400 // тут хранятся привязки nRF к железу, 20 байт (про запас)
#define LORA_BINDING_ADDRESS 11420 // тут хранятся привязки LoRa к железу, 20 байт (про запас)
#define TIMER_BINDING_ADDRESS 11440 // тут хранятся привязки таймеров к железу, 20 байт (про запас)
#define DALLAS_BINDING_ADDRESS 11500 // тут хранятся привязки DS18B20 к железу, 100 байт (про запас)
#define ONEWIRE_BINDING_ADDRESS 11600 // тут хранятся привязки DS18B20 к железу, 50 байт (про запас)
#define DIODES_BINDING_ADDRESS 11650 // тут хранятся привязки информационных диодов к железу, 20 байт (про запас)
#define WINDOWS_BINDING_ADDRESS 11700 // тут хранятся привязки фрамуг к железу, 100 байт (про запас)
#define ENDSTOPS_BINDING_ADDRESS 11800 // тут хранятся привязки концевиков к железу, 100 байт (про запас)
#define SOIL_BINDING_ADDRESS 11900 // тут хранятся привязки датчиков влажности почвы к железу, 100 байт (про запас)
#define HUMIDITY_BINDING_ADDRESS 12000 // тут хранятся привязки датчиков влажности к железу, 100 байт (про запас)
#define RS485_BINDING_ADDRESS 12100 // тут хранятся привязки RS-485 к железу, 50 байт (про запас)
#define WATERING_BINDING_ADDRESS 12150 // тут хранятся привязки полива к железу, 100 байт (про запас)
#define GSM_BINDING_ADDRESS 12250 // тут хранятся привязки SIM800 к железу, 100 байт (про запас)
#define WIFI_BINDING_ADDRESS 12350 // тут хранятся привязки ESP к железу, 100 байт (про запас)
#define TFT_BINDING_ADDRESS 12450 // тут хранятся привязки TFT к железу, 50 байт (про запас)
#define FLOW_BINDING_ADDRESS 12500 // тут хранятся привязки расходомеров к железу, 50 байт (про запас)
#define TFT_SENSORS_BINDING_ADDRESS 12550 // тут хранятся настройки датчиков TFT, 12 записей, по 50 байт каждая, итого - 600 байт
#define WPOWER_BINDING_ADDRESS 13200 // тут хранятся настройки для управления питанием фрамуг, 50 байт
#define DOOR_BINDING_ADDRESS 13300 // тут хранятся настройки привязок для управления дверьми, 100 байт
#define WM_DOOR_SETTINGS_ADDRESS 13400 // тут хранятся настройки управления дверями, 20 байт*2 канала = 40 байт
#define INTERVALS_BINDING_ADDRESS  14000 // тут хранятся настройки интервалов открытия окон, 16 окон*4 байта + 2 байта заголовка + CRC(1 байт) = 30 байт
#define WM_HUMIDITY_SPRAY_SETTINGS_ADDRESS 14100 // тут хранятся настройки привязки распыления для поддержания влажности: 3 канала*20 байт = 60 байт
#define SPRAY_BINDING_ADDRESS              14200 // настройки привязки распрыскивания к железу, 100 байт (про запас)
#define DS18B20_EMULATION_BINDING_ADDRESS 14300 // тут хранятся привязки эмуляторов DS18B20 к железу, 100 байт (про запас)
#define DS18B20_EMULATION_STORE_ADDRESS 14400 // адрес хранения привязок эмуляторов DS18B20 к адресам, максимум 20 записей, одна запись - 10 байт, т.е. отводится 200 байт на это дело
//--------------------------------------------------------------------------------------------------------------------------------
// настройки Serial
//--------------------------------------------------------------------------------------------------------------------------------
#define SERIAL_BAUD_RATE 57600 // скорость работы с портами, бод (конфигуратор работает с портом именно на скорости 57600)
#define VIRTUAL_PIN_START_NUMBER 80 // номер пина, с которого все пины будут считаться виртуальными, т.е. запись в них производиться не будет, однако в карте пинов (16 байт) этот статус будет отражён.

//--------------------------------------------------------------------------------------------------------------------------------
// настройки максимумов
//--------------------------------------------------------------------------------------------------------------------------------
#define MAX_ALERT_RULES 50 // максимальное кол-во поддерживаемых правил
#define MAX_DELTAS 20 // максимальное кол-во дельт. Внимание: на 20 дельт нужно примерно 500 байт в EEPROM, следите за непересечением адресов!!!


//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// настройки TFT (используется 7'' экран на контроллере SSD1963 с разрешением 800x480)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// TFT model
#define TFT_MODEL CTE70 // SSD1963 (16bit) 800x480 Alternative Init
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// цвета для TFT
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_BACK_COLOR 0xFF,0xFF,0xFF // цвет фона
#define TFT_BUTTON_COLORS VGA_WHITE, VGA_GRAY, VGA_WHITE, VGA_RED, VGA_BLUE // цвета кнопок
#define TFT_FONT_COLOR 0x4B, 0x4C, 0x4B // цвет шрифта по умолчанию
#define TFT_CHANNELS_BUTTON_COLORS 0x3A8D, VGA_SILVER, VGA_GRAY, VGA_RED, 0xEF7D // цвета кнопок для каналов
#define INFO_BOX_BACK_COLOR 97,44,8 // цвет фона для информационного бокса
#define INFO_BOX_BORDER_COLOR VGA_BLACK // цвет рамки информационного бокса
#define INFO_BOX_CAPTION_COLOR 0x30, 0x7B, 0xB5 // цвет заголовка информационного бокса
#define SENSOR_BOX_FONT_COLOR VGA_WHITE // цвет показаний датчика
#define SENSOR_BOX_UNIT_COLOR VGA_WHITE // цвет единиц изменений датчика
#define MODE_ON_COLOR VGA_GREEN  // цвет "вкл", "авто"
#define MODE_OFF_COLOR VGA_MAROON // цвет "выкл", "ручной"
#define CHANNELS_BUTTONS_TEXT_COLOR VGA_WHITE // цвет текста кнопок каналов
#define CHANNELS_BUTTONS_BG_COLOR 0xEF7D    // цвет фона кнопок каналов
#define CHANNEL_BUTTONS_TEXT_COLOR 0x3A8D   // цвет текста кнопки одного канала
#define TIME_PART_FONT_COLOR VGA_TEAL // цвет текста кнопки установки компонента времени
#define TIME_PART_SELECTED_FONT_COLOR VGA_WHITE // цвет текста активной кнопки компонента времени
#define TIME_PART_BG_COLOR 0xEF7D // цвет фона кнопки компонента времени
#define TIME_PART_SELECTED_BG_COLOR VGA_GREEN // цвет фона выбранной кнопки компонента времени
#define STATUS_ON_COLOR VGA_LIME // цвет статусов на экране ожидания

#define TFT_MANUAL_MODE_CAPTION_COLOR VGA_YELLOW     //цвет шрифта, которым будет написан ручной режим работы в инфобоксах -КМВ
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// настройки подписей и пр.
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_WINDOW_STATUS_CAPTION "ФРАМУГИ"
#define TFT_WATER_STATUS_CAPTION "ПОЛИВ"
#define TFT_LIGHT_STATUS_CAPTION "ДОСВЕТКА"
#define TFT_STATUS_CAPTION "СТАТУС:"
#define TFT_MODE_CAPTION "РЕЖИМ:"
#define TFT_WINDOWS_OPEN_CAPTION "ОТКРЫТЫ"
#define TFT_WINDOWS_CLOSED_CAPTION "ЗАКРЫТЫ"
#define TFT_WATER_ON_CAPTION "ВКЛ"
#define TFT_WATER_OFF_CAPTION "ВЫКЛ"
#define TFT_LIGHT_ON_CAPTION "ВКЛ"
#define TFT_LIGHT_OFF_CAPTION "ВЫКЛ"
#define TFT_AUTO_MODE_CAPTION "АВТО"
#define TFT_MANUAL_MODE_CAPTION "РУЧНОЙ"
#define AUTO_MODE_LABEL "РЕЖИМ: АВТО"
#define MANUAL_MODE_LABEL "РЕЖИМ: РУЧНОЙ"
#define OPEN_ALL_LABEL "ОТКРЫТЬ ВСЕ"
#define CLOSE_ALL_LABEL "ЗАКРЫТЬ ВСЕ"
#define TURN_ON_ALL_WATER_LABEL "ВКЛ ПОЛИВ"
#define TURN_OFF_ALL_WATER_LABEL "ВЫКЛ ПОЛИВ"
#define SKIP_WATERING_LABEL "ПРОПУСТИТЬ"
#define TURN_ON_ALL_LIGHT_LABEL "СВЕТ ВКЛ"
#define TURN_OFF_ALL_LIGHT_LABEL "СВЕТ ВЫКЛ"
#define TFT_TOPEN_CAPTION "^ откр (для правил):"
#define TFT_TCLOSE_CAPTION "^ закр (для правил):"
#define TFT_INTERVAL_CAPTION "Время открытия, с:"
#define TFT_CURRENTTIME_CAPTION "Дата/время (выделите компонент для изменения):"
#define TFT_SENSOR_TEMPERATURE_CAPTION F("Температура #")
#define TFT_SENSOR_HUMIDITY_CAPTION F("Влажность #")
#define TFT_SENSOR_LUMINOSITY_CAPTION F("Освещенность #")
#define TFT_SENSOR_SOIL_CAPTION F("Почва #")
#define TFT_LIST_SCROLL_UP_CAPTION "{ ВВЕРХ"
#define TFT_LIST_SCROLL_DOWN_CAPTION "} ВНИЗ"
#define TFT_TIME_SYNC_BUTTON_CAPTION "СИНХРОНИЗАЦИЯ ВРЕМЕНИ"
#define TFT_SYNC_ENABLED_CAPTION "ВКЛЮЧЕНА"
#define TFT_SYNC_DISABLED_CAPTION "ВЫКЛЮЧЕНА"
#define TFT_TIME_SYNC_GSM_INTERVAL_CAPTION "ИНТЕРВАЛ GSM, Ч:"
#define TFT_TIME_SYNC_WIFI_INTERVAL_CAPTION "ИНТЕРВАЛ WI-FI, Ч:"
#define TFT_TIMEZONE_BOX_CAPTION "ТАЙМЗОНА, МИНУТ:"
#define TFT_WATERING_SETT_CAPTION "НАСТРОЙКА"
#define TFT_WATERING_SETT_SCREEN_CAPTION "НАСТРОЙКА КАНАЛОВ ПОЛИВА"
#define TFT_WATERING_SETT_HUM_BORDER "ВЛАЖНОСТЬ MAX:"
#define TFT_WATERING_SETT_HUM_BORDER_MIN "ВЛАЖНОСТЬ MIN:"
#define TFT_WATERING_SETT_SENSOR "ДАТЧИК ПОЧВЫ:"
#define TFT_WATERING_SETT_DURATION "ДЛИТЕЛЬНОСТЬ, МИН:"
#define TFT_WATERING_SETT_START "НАЧАЛО, Ч:"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
