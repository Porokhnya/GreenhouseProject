#ifndef _CONFIGURATION_SHARED_H
#define _CONFIGURATION_SHARED_H

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ОБЩИЕ ДЛЯ ВСЕХ КОНФИГУРАЦИЙ НАСТРОЙКИ
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

//--------------------------------------------------------------------------------------------------------------------------------
// unit labels
// подписи к видам измерений
//--------------------------------------------------------------------------------------------------------------------------------
#define UNIT_CELSIUS F(" C") // Цельсии (unit for Celsius)
#define UNIT_FAHRENHEIT F(" F") // Фаренгейты (unit for Fahrenheit)
#define UNIT_PH F(" pH") // pH
#define UNIT_LITRES F(" л") // литры (litres)
#define UNIT_LUX F(" люкс") // люксы (lux)
#define UNIT_PPM F(" ppm") // ppm (для CO2)
#define UNIT_EC F(" ppm") // ppm (для EC)


//--------------------------------------------------------------------------------------------------------------------------------
// Stat SMS settings
// Настройки СМС статистики
//--------------------------------------------------------------------------------------------------------------------------------
// windows label
// состояние окон
#define W_STATE F("Окна: ") 

// closed
// закрыты
#define W_CLOSED F("закр") 

// open
// открыты
#define W_OPEN F("откр") 

// watering label
// состояние полива
#define WTR_STATE F("Полив: ")

// off
// полив выкл
#define WTR_OFF F("выкл")

// on
// полив вкл
#define WTR_ON F("вкл")

// no data from sensor
// нет данных с датчика
#define NO_DATA F("<нет данных>")

//--------------------------------------------------------------------------------------------------------------------------------
// определения вида управления пинами (настройки управления по тому или иному принципу специфичны для каждого модуля, их смотрите ниже)
//--------------------------------------------------------------------------------------------------------------------------------
#define DRIVE_DIRECT 1    // прямое управление пинами
#define DRIVE_MCP23S17 2  // через расширитель портов MCP23S17
#define DRIVE_MCP23017 3 // через расширитель портов MCP20S17
#define DRIVE_PCF8574 4 // через микросхему PCF8574

//--------------------------------------------------------------------------------------------------------------------------------
// определения типов памяти, поддерживаемых прошивкой
//--------------------------------------------------------------------------------------------------------------------------------

#define EEPROM_BUILTIN 1 // встроенный EEPROM
#define EEPROM_AT24C32 2 // I2C-память AT24C32 
#define EEPROM_AT24C64 3 // I2C-память AT24C64 
#define EEPROM_AT24C128 4 // I2C-память AT24C128 
#define EEPROM_AT24C256 5 // I2C-память AT24C256 
#define EEPROM_AT24C512 6 // I2C-память AT24C512 

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля датчиков влажности почвы (актуально при раскомментированной команде USE_SOIL_MOISTURE_MODULE)
//--------------------------------------------------------------------------------------------------------------------------------
#define FREQUENCY_SOIL_MOISTURE 2 // частотный датчик влажности почвы
#define ANALOG_SOIL_MOISTURE 1 // аналоговый датчик влажности почвы
#define H_SENSOR(pin,pin2,type) { (pin) , (pin2), (type) } // для удобства добавления сенсора в массив

//--------------------------------------------------------------------------------------------------------------------------------
// настройки IoT (актуально при раскомментированной команде USE_IOT_MODULE)
//--------------------------------------------------------------------------------------------------------------------------------
#define IOT_USER_AGENT F("greenhouse") // User-agent для запроса
#define THINGSPEAK_IP F("184.106.153.149") // IP сервиса ThingSpeak
#define THINGSPEAK_HOST F("api.thingspeak.com") // Имя хоста ThingSpeak

//--------------------------------------------------------------------------------------------------------------------------------
// Настройки модуля запроса команд к контроллеру по HTTP (актуально при раскомментированной команде USE_HTTP_MODULE)
//--------------------------------------------------------------------------------------------------------------------------------

#define HTTP_SERVER_IP "gardenboss.ru"   // адрес хоста, который мы опрашиваем на команды (IP или доменное имя)
#define HTTP_SERVER_HOST "gardenboss.ru" // имя хоста (для заголовка Host)
#define HTTP_POLL_INTERVAL 300 // через сколько секунд проверять на команды (минимум - 300 секунд, т.е. 5 минут)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// ||
// \/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// !!! СЛУЖЕБНЫЕ НАСТРОЙКИ - МЕНЯЕМ, ЕСЛИ ПОНИМАЕМ, ДЛЯ ЧЕГО И ЗАЧЕМ !!!
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//--------------------------------------------------------------------------------------------------------------------------------
// запрещаем использование более 16 каналов полива в прошивке (иначе не влезем в структуру состояния контроллера)
//--------------------------------------------------------------------------------------------------------------------------------
#if WATER_RELAYS_COUNT > 16
#error WATER CHANNELS IS LIMITED to 16 !!!
#endif
//--------------------------------------------------------------------------------------------------------------------------------
// запрещаем использование более 16 окон в прошивке (иначе не влезем в структуру состояния контроллера)
//--------------------------------------------------------------------------------------------------------------------------------
#if SUPPORTED_WINDOWS > 16
#error WINDOWS COUNT IS LIMITED to 16 !!!
#endif
//--------------------------------------------------------------------------------------------------------------------------------
// запрещаем использовать более одного дисплея в прошивке (ибо бессмысленно - два дисплея одновременно)
//--------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_LCD_MODULE)
  #if defined(USE_TFT_MODULE) || defined(USE_NEXTION_MODULE)
  #error PLEASE DONT USE TWO OR MORE DISPLAYS !!!
  #endif
#endif

#if defined(USE_NEXTION_MODULE)
  #if defined(USE_TFT_MODULE) || defined(USE_LCD_MODULE)
  #error PLEASE DONT USE TWO OR MORE DISPLAYS !!!
  #endif
#endif

#if defined(USE_TFT_MODULE)
  #if defined(USE_LCD_MODULE) || defined(USE_NEXTION_MODULE)
  #error PLEASE DONT USE TWO OR MORE DISPLAYS !!!
  #endif
#endif
//--------------------------------------------------------------------------------------------------------------------------------
// настройки максимумов
//--------------------------------------------------------------------------------------------------------------------------------
#define MAX_RECEIVE_BUFFER_LENGTH 2048 // максимальная длина (в байтах) пакета в сети, дла защиты от спама

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля алертов (событий по срабатыванию каких-либо условий)
//--------------------------------------------------------------------------------------------------------------------------------
#define ALERT F("ALERT") // произошло событие
#define VIEW_COMMAND F("VIEW") // команда просмотра события CTGET=ALERT|VIEW|0
#define CNT_COMMAND F("CNT") // сколько зарегистрировано событий CTGET=ALERT|CNT
// правило алерта CTSET=ALERT|RULE_ADD|RuleName|STATE|TEMP|1|>|23|Время начала работы|Продолжительность работы, мин|Маска дней недели|Список связанных правил|Команды для стороннего модуля
// пример №1: CTSET=ALERT|RULE_ADD|N1|STATE|TEMP|1|>|23|0|30|127|N3,N4|CTSET=STATE|WINDOW|ALL|OPEN
// пример №2: CTSET=ALERT|RULE_ADD|N1|STATE|TEMP|1|>|23|0|0|127|_|CTSET=STATE|WINDOW|ALL|OPEN
#define ADD_RULE F("RULE_ADD") // добавить правило
#define RULE_CNT F("RULES_CNT") // кол-во правил CTGET=ALERT|RULES_CNT
#define RULE_VIEW F("RULE_VIEW") // просмотр правила по индексу CTGET=ALERT|RULE_VIEW|0
#define RULE_STATE F("RULE_STATE") // включить/выключить правило по имени CTSET=ALERT|RULE_STATE|RuleName|ON, CTSET=ALERT|RULE_STATE|RuleName|OFF, CTSET=ALERT|RULE_STATE|ALL|OFF
#define RULE_ALERT F("RULE_ALERT")
// получить состояние правила по индексу -  CTGET=ALERT|RULE_STATE|0

#define RULE_DELETE F("RULE_DELETE") // удалить правило по имени CTSET=ALERT|RULE_DELETE|RuleName - ПРИ УДАЛЕНИИ ВСЕ ПРАВИЛА СДВИГАЮТСЯ К ГОЛОВЕ ОТ УДАЛЁННОГО !!! 
// Специальный параметр ALL (CTSET=ALERT|RULE_DELETE|ALL) удаляет все правила.

#define SAVE_RULES F("SAVE") // команда "сохранить правила", CTSET=ALERT|SAVE
#define GREATER_THAN F(">") // больше чем
#define GREATER_OR_EQUAL_THAN F(">=") // больше либо равно
#define LESS_THAN F("<") // меньше чем
#define LESS_OR_EQUAL_THAN F("<=") // меньше или равно
#define T_OPEN_MACRO F("%TO%") // макроподстановка температуры открытия из настроек
#define T_CLOSE_MACRO F("%TC%") // макроподстановка температуры закрытия из настроек


//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля логгирования информации
//--------------------------------------------------------------------------------------------------------------------------------
//#define ADD_LOG_HEADER // закомментировать, если не надо добавлять первые строки с информацией, с каких модулей есть данные).
// Первая строка информации имеет вид:
// MODULE_NAME1=MODULE_IDX1,MODULE_NAME2=MODULE_IDX2,MODULE_NAMEn=MODULE_IDXn
// где MODULE_NAMEn - имя модуля, MODULE_IDXn = индекс модуля в системе.
// Вторая строка информации сообщает о типах датчиков, и имеет вид:
// TYPE1=IDX1,TYPEn=IDXn
// где TYPEn - название типа (например, TEMP для температуры), IDXn - числовое представление типа.
//#define LOG_CNANGE_NAME_TO_IDX // раскомментировать, если нужен лог меньшего размера.
// в этом случае в каждой строке вместо имени модуля подставляется его индекс в системе. 
//#define LOG_CHANGE_TYPE_TO_IDX // раскомментировать, если нужен лог меньшего размера.
// в этом случае в каждой строке вместо названия типа датчика подставляется его индекс в системе.
//#define WRITE_ABSENT_SENSORS_DATA // раскомментировать, если надо писать показания датчика, даже если показаний с него нет
#define LOG_TEMP_TYPE F("RT") // тип для температуры, который запишется в файл
#define LOG_HUMIDITY_TYPE F("RH") // тип для влажности, который запишется в файл
#define LOG_LUMINOSITY_TYPE F("RL") // тип для освещенности, который запишется в файл
#define LOG_WATERFLOW_TYPE F("WF") // тип для датчика расхода воды, который запишется в файл
#define LOG_SOIL_TYPE F("SM") // тип для датчика влажности почвы, который запишется в файл
#define LOG_PH_TYPE F("PH") // тип для датчика pH, который запишется в файл
#define COMMA_DELIMITER F(",") // разделитель полей в CSV
#define LOGS_DIRECTORY F("logs") // название папки с логами на карточке
#define ACTIONS_DIRECTORY F("actions") // название папки с логами действий на карточке
#define END_OF_FILE F("END_OF_FILE") // какую строку посылаем, когда весь файл вычитали
#define FOLLOW F("FOLLOW") // ответ, что файл будет выслан следующими строками
#define FILE_COMMAND F("FILE") // получить данные с файла
#define ACTIONS_COMAND F("ACTION") // получить данные с файла действий


//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля освещенности
//--------------------------------------------------------------------------------------------------------------------------------
#define LIGHT_STATE_COMMAND F("STATE") // CTGET=LIGHT|STATE
#define NO_LUMINOSITY_DATA -1 // нет показаний с датчика освещенности (И ЭТУ КОНСТАНТУ НЕ ТРОГАЕМ, ДАЖЕ ЕСЛИ ОБКОЛОЛОСЬ УЖЕ ВСЁ !!!)
#define NO_CO2_DATA 0xFFFF  // нет данных о показаниях CO2

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля управления фрамугами
//--------------------------------------------------------------------------------------------------------------------------------
#define STATE_OPENING F("OPENING") // Открывается
#define STATE_CLOSING F("CLOSING") // Закрывается
#define STATE_CLOSED F("CLOSED") // Закрыто
#define WM_AUTOMATIC F("AUTO") // автоматический режим управления фрамугами
#define WM_MANUAL F("MANUAL") // ручной режим управления фрамугами
#define WORK_MODE F("MODE") // получить/установить режим работы CTGET=STATE|MODE, CTSET=STATE|MODE|AUTO, CTSET=STATE|MODE|MANUAL
#define WM_INTERVAL F("INTERVAL") // получить/установить интервал на открытие/закрытие окон CTGET=STATE|INTERVAL, CTSET=STATE|INTERVAL|3000
#define TOPEN_COMMAND F("TOPEN") // получить/установить температуру открытия фрамуг CTGET=STATE|TOPEN, CTSET=STATE|TOPEN|20
#define TCLOSE_COMMAND F("TCLOSE") // получить/установить температуру закрытия фрамуг CTGET=STATE|TCLOSE, CTSET=STATE|TCLOSE|15
#define STATE_OPEN F("OPEN") // Открыть CTSET=STATE|WINDOW|0|OPEN, CTSET=STATE|WINDOW|ALL|OPEN, CTSET=STATE|WINDOW|0-2|OPEN|2000
#define ALL F("ALL") // отработать все каналы
#define PROP_WINDOW F("WINDOW") // название канала, чтобы было понятно
#define PROP_WINDOW_CNT F("WINDOW_CNT") // кол-во фрамуг CTGET=STATE|WINDOW|WINDOW_CNT
#define PROP_WINDOW_STATEMASK F("STATEMASK") // CTGET=STATE|WINDOW|STATEMASK - получить состояние всех окон в виде маски. Ответ: OK=STATE|WINDOW|STATEMASK|Кол-во окон|Маска,
// где Маска - байты маски в виде шестнадцатеричной строки (например "F0"), для каждого окна в этих байтах - по два бита, их значение: 00 - закрыто, 01 - открывается, 10 - закрывается, 11 - открыто.
// например, для 4-х окон будет 1 байт (4*2 бита = 8 бит = 1 байт), для 5 окон - два байта, при этом во втором байте значащими будут только младшие 2 бита и т.д.
#define TEMP_SETTINGS F("T_SETT") // получить/установить температуры срабатывания, CTGET=STATE|T_SETT, CTSET=STATE|T_SETT|t open|t close
#define NO_TEMPERATURE_DATA -128 // нет данных с датчика температуры (ВООБЩЕ НЕ ТРОГАЕМ ЭТУ КОНСТАНТУ, ДАЖЕ ЕСЛИ ОЧЕНЬ КОЛЕТСЯ!!!)

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля управления поливом
//--------------------------------------------------------------------------------------------------------------------------------
#define WATER_SETTINGS_COMMAND F("T_SETT") // получить/установить настройки управления поливом: CTGET=WATER|T_SETT, CTSET=WATER|T_SETT|WateringOption|WateringDays|WateringTime|StartTime|TurnOnPump , где
// WateringOption = 0 (выключено автоматическое управление поливом), 1 - автоматическое управление поливом включено (все каналы), 2 - автоуправление отдельно по каналам
// WateringDays - битовая маска дней недели (младший бит - понедельник и т.д.)
// WateringTime - продолжительность полива в минутах, максимальное значение - 65535 (два байта)
// StartTime - время начала полива, в минутах от начала суток
// TurnOnPump - включать (1) или нет (0) насос при активном поливе на любом из каналов
#define WATER_CHANNEL_SETTINGS F("CH_SETT") // получить/установить настройки отдельного канала управления поливом: CTGET=WATER|CH_SETT|0, CTSET=WATER|CH_SETT|0|WateringDays|WateringTime|StartTime
#define WATER_CHANNELS_COUNT_COMMAND F("CHANNELS") // получить кол-во поддерживаемых каналов полива: CTGET=WATER|CHANNELS

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля контроля воды
//--------------------------------------------------------------------------------------------------------------------------------
#define FLOW_CALIBRATION_COMMAND F("T_SETT") // получить/установить факторы калибровки: CTGET=FLOW|T_SETT, CTSET=FLOW|T_SETT|factor1|factor2

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля pH
//--------------------------------------------------------------------------------------------------------------------------------
#define PH_SETTINGS_COMMAND F("T_SETT") // получить/установить настройки: CTGET=PH|T_SETT, CTSET=PH|T_SETT|calibration_factor

//--------------------------------------------------------------------------------------------------------------------------------
// настройки главного контроллера
//--------------------------------------------------------------------------------------------------------------------------------
#define MIN_COMMAND_LENGTH 6 // минимальная длина правильной текстовой команды
#define CMD_PREFIX  F("CT") // запрос к контроллеру
#define CHILD_PREFIX F("CD") // запрос к дочернему модулю
#define CMD_PREFIX_LEN  2  // длина префикса команды

#define CMD_SET F("SET") // установить значение
#define CMD_GET F("GET") // получить значение
#define CMD_TYPE_LEN 3 // длина типа команды

//--------------------------------------------------------------------------------------------------------------------------------
#define READY F("READY") // будет напечатано в Serial после загрузки
//--------------------------------------------------------------------------------------------------------------------------------

//--------------------------------------------------------------------------------------------------------------------------------
// состояния вкл/выкл, для команд
//--------------------------------------------------------------------------------------------------------------------------------
#define STATE_ON F("ON") // Включено
#define STATE_ON_ALT F("1") // Включено
#define STATE_OFF F("OFF") // Выключено
#define STATE_OFF_ALT F("0") // Выключено

//--------------------------------------------------------------------------------------------------------------------------------
// ОТВЕТЫ ЗА ЗАПРОСЫ
//--------------------------------------------------------------------------------------------------------------------------------
#define OK_ANSWER F("OK") // ответ - всё ок
#define ERR_ANSWER F("ER") // ответ - ошибка
#define UNKNOWN_MODULE F("UNKNOWN_MODULE") // запрос к неизвестному модулю
#define PARAMS_MISSED F("PARAMS_MISSED") // пропущены параметры команды
#define UNKNOWN_COMMAND F("UNKNOWN_COMMAND") // неизвестная команда
#define NOT_SUPPORTED F("NOT_SUPPORTED") // не поддерживается

//--------------------------------------------------------------------------------------------------------------------------------
// РАЗДЕЛИТЕЛЬ ПАРАМЕТРОВ
//--------------------------------------------------------------------------------------------------------------------------------
#define PARAM_DELIMITER F("|")
//--------------------------------------------------------------------------------------------------------------------------------
// разделитель команды и ответа
//--------------------------------------------------------------------------------------------------------------------------------
#define COMMAND_DELIMITER F("=")

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля PIN
//--------------------------------------------------------------------------------------------------------------------------------
#define PIN_TOGGLE F("T") // CTGET=PIN|13, CTSET=PIN|13|1, CTSET=PIN|13|ON, CTSET=PIN|13|OFF, CTSET=PIN|13|0, CTSET=PIN|13|T
#define PIN_DETACH F("DETACH") // не устанавливать состояние пина

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля статистикм
//--------------------------------------------------------------------------------------------------------------------------------
#define FREERAM_COMMAND F("FREERAM") // показать кол-во свободной памяти CTGET=STAT|FREERAM
#define UPTIME_COMMAND F("UPTIME") // показать время работы (в секундах) CTGET=STAT|UPTIME
#ifdef USE_DS3231_REALTIME_CLOCK
#define CURDATETIME_COMMAND F("DATETIME") // вывести текущую дату и время CTGET=STAT|DATETIME
#endif

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля управления по SMS
//--------------------------------------------------------------------------------------------------------------------------------
#define STAT_COMMAND F("STAT") // получить текущую статистику по SMS, CTGET=SMS|STAT
#define BALANCE_COMMAND F("BAL") // получить баланс по SMS, CTGET=SMS|BAL
#define SMS_OPEN_COMMAND F("#1") // открыть окна
#define SMS_CLOSE_COMMAND F("#0") // закрыть окна
#define SMS_STAT_COMMAND F("#9") // получить статистику
#define SMS_AUTOMODE_COMMAND F("#8") // установить автоматический режим работы
#define SMS_WATER_ON_COMMAND F("#4") // включить полив
#define SMS_WATER_OFF_COMMAND F("#6") // выключить полив
#define SMS_BALANCE_COMMAND F("#5") // получить баланс на счету
#define SMS_RESET_COMMAND F("####") // перезагрузить контроллер

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля WI-FI
//--------------------------------------------------------------------------------------------------------------------------------
#define WIFI_SETTINGS_COMMAND F("T_SETT") // установить настройки модуля: CTSET=WIFI|T_SETT|SHOULD_CONNECT_TO_ROUTER(0 or 1)|ROUTER_ID|ROUTER_PASS|STATION_ID|STATION_PASS
#define IP_COMMAND F("IP") // получить текущие IP-адреса, как самой точки доступа, так и назначенный роутером, CTGET=WIFI|IP
#define BUSY F("BUSY") // если мы не можем ответить на запрос - тогда возвращаем ER=WIFI|BUSY
//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля составных команд
//--------------------------------------------------------------------------------------------------------------------------------
#define CC_ADD_COMMAND F("ADD") // добавить составную команду, CTSET=CC|ADD|ListIndex|CommandAction|AdditionalParam
#define CC_SAVE_COMMAND F("SAVE") // сохранить все настройки составных команд в EEPROM, CTSET=CC|SAVE
#define CC_DELETE_COMMAND F("DEL") // удалить все составные команды, CTSET=CC|DEL
#define CC_PROCESS_COMMAND F("EXEC") // выполнить составную команду, CTSET=CC|EXEC|ListIndex


//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля дельт
//--------------------------------------------------------------------------------------------------------------------------------
#define DELTA_ADD_COMMAND F("ADD") // добавить дельту, CTSET=DELTA|ADD|SensorType|ModuleName1|SensorIndex1|ModuleName2|SensorIndex2
#define DELTA_SAVE_COMMAND F("SAVE") // сохранить все настройки дельт в EEPROM, CTSET=DELTA|SAVE
#define DELTA_DELETE_COMMAND F("DEL") // удалить все дельты, CTSET=DELTA|DEL
#define DELTA_VIEW_COMMAND F("VIEW") // просмотр дельты по индексу, CTGET=DELTA|VIEW|0
#define DELTA_COUNT_COMMAND F("CNT") // получить кол-во сохранённых дельт, CTGET=DELTA|CNT

//--------------------------------------------------------------------------------------------------------------------------------
 // свойства модулей
//--------------------------------------------------------------------------------------------------------------------------------
#define PROP_TEMP_CNT F("TEMP_CNT") // кол-во датчиков температуры CTGET=0|PROP|TEMP|TEMP_CNT, CTSET=0|PROP|TEMP|TEMP_CNT|2
#define PROP_RELAY_CNT F("RELAY_CNT") // кол-во каналов реле CTGET=0|PROP|MODULE_NAME|RELAY_CNT, CTSET=0|PROP|MODULE_NAME|RELAY_CNT|2
#define PROP_CNT F("CNT") // свойство - кол-во любых датчиков
#define PROP_TEMP F("TEMP") // нам передали/запросили температуру CTGET=0|PROP|MODULE_NAME|TEMP|0, CTSET=0|PROP|MODULE_NAME|TEMP|0|36,6
#define PROP_LIGHT F("LIGHT") // свойство "освещенность"
#define PROP_HUMIDITY F("HUMIDITY") // свойство "влажность"
#define PROP_CO2 F("CO2") // свойство "CO2"
#define PROP_EC F("EC") // свойство "EC"
#define PROP_PIN F("PIN") // свойство "пин" (для слежения за статусом пинов)
#define PROP_SOIL F("SOIL") // свойство "влажность почвы"
#define PROP_PH F("PH") // свойство pH
#define PROP_FLOW_INCREMENTAL F("AFLOW") // свойство "накопительный расход воды"
#define PROP_FLOW_INSTANT F("BFLOW") // свойство "мгновенный расход воды"
#define PROP_NONE F("_") // нет свойства

//--------------------------------------------------------------------------------------------------------------------------------
// команды модуля "0"
//--------------------------------------------------------------------------------------------------------------------------------
//#define USE_REMOTE_MODULES // раскомментировать, если нужна регистрация модулей на лету (при использовании сторонних железок, общающихся с контроллером)
#define NEWLINE F("\r\n")
#define SETTIME_COMMAND F("DATETIME") // установка даты/времени CTSET=0|DATETIME|DD.MM.YYYY hh:mm:ss
#define ADD_COMMAND F("ADD") // команда регистрации модуля CTSET=0|ADD|MODULE_NAME
#define PING_COMMAND F("PING") // команда пинга контроллера CTGET=0|PING
#define REGISTERED_MODULES_COMMAND F("LIST") // пролистать зарегистрированные модули CTGET=0|LIST
#define SMS_NUMBER_COMMAND F("PHONE") // сохранить/вернуть номер телефона для управления контроллером по СМС: CTSET=0|PHONE|+7918..., CTGET=0|PHONE
#define PONG F("PONG") // ответ на запрос пинга
#define REG_SUCC F("ADDED") // модуль зарегистрирован, или команда обработана
#define REG_DEL F("DELETED") // удалено
#define REG_ERR F("EXIST") // модуль уже зарегистрирован
#define UNKNOWN_PROPERTY F("UNKNOWN_PROPERTY") // неизвестное свойство
#define STATUS_COMMAND F("STAT") // получить статус внутренних состояний в виде закодированного пакета, CTGET=0|STAT
#define RESET_COMMAND F("RST") // перезагрузить контроллер
#define ID_COMMAND F("ID") // получить/установить ID контроллера
#define WIRED_COMMAND F("WIRED") // получить список кол-ва проводных датчиков, CTGET=0|WIRED (Температура|Влажность|Освещенность|Влажность почвы|PH)
#define UNI_COUNT_COMMAND F("UNI") // получить список кол-ва универсальных датчиков, CTGET=0|UNI (Температура|Влажность|Освещенность|Влажность почвы|PH)
#define UNI_NOT_FOUND F("U_NONE") // ответ на запрос CTGET=0|U_SEARCH, если универсального модуля не найдено
#define UNI_SEARCH F("U_SEARCH") // запрос CTGET=0|U_SEARCH, выдаёт информацию об универсальном модуле в формате OK=SCRATCHPAD_DATA и ERR=U_NONE, если датчика на линии нет
#define UNI_REGISTER F("U_REG") // запрос CTSET=0|U_REG|SCRATCHPAD_DATA, регистрирует подсоединённый к линии регистрации датчик, возвращает OK=ADDED, если датчик есть, и ERR=U_NONE, если датчика на линии нет
#define UNI_DIFFERENT_SCRATCHPAD F("SCRATCH_TYPE_ERROR") // ошибка при регистрации, разные типы скратчпада переданы
#define UNI_RF_CHANNEL_COMMAND F("RF") // команда на получение/установку канала для nRF
#define PINS_COMMAND F("PINS") // получить состояние пинов, CTGET=0|PINS, ответ OK=PINS|Кол-во_байт_в_пакете|HEX-пакет_занятых_пинов|HEX-пакет_режима_пинов
//--------------------------------------------------------------------------------------------------------------------------------
#define SD_BUFFER_LENGTH 256 // размер буфера для блочного чтения с SD
//--------------------------------------------------------------------------------------------------------------------------------
// общий буфер для команд
//--------------------------------------------------------------------------------------------------------------------------------
#define SHARED_BUFFER_LENGTH 512 // сколько байт резервировать для общего буфера обмена
#define WINDOWS_STATUS_BIT 0 // номер бита статуса окон (1 - открыты, 0 - закрыты)
#define WINDOWS_MODE_BIT 1 // номер бита режима работы окон (1 - авто, 0 - ручной)
#define WATER_STATUS_BIT 2 // номер бита статуса полива (1 - включен, 0 - выключен)
#define WATER_MODE_BIT 3 // номер бита режима работы полива (1 - авто, 0 - ручной)
#define LIGHT_STATUS_BIT 4 // номер бита состояния досветки (1 - включена, 0 - выключена)
#define LIGHT_MODE_BIT 5 // номер бита режима работы досветки (1 - авто, 0 - ручной)
#define WINDOWS_POS_CHANGED_BIT 6 // бит, выставленный в 1, если окна закончили смену позиции
#define PH_FLOW_ADD_BIT 7 // бит, выставляемый в 1, если насос заполнения бака pH включен
#define PH_MIX_PUMP_BIT 8 // бит, выставляемый в 1, если насос перемешивания pH работает
#define PH_PLUS_PUMP_BIT 9 // бит, выставляемый в 1, если насос повышения pH работает
#define PH_MINUS_PUMP_BIT 10 // бит, выставляемый в 1, если насос понижения pH работает
#define STATUSES_BYTES 2 // сколько байт под статусы у нас используется

#define SAVE_STATUS(bnum,sta) WORK_STATUS.SetStatus((bnum),(sta))

//--------------------------------------------------------------------------------------------------------------------------------
// настройки модуля логгирования информации (актуально при раскомментированной команде USE_LOG_MODULE)
//--------------------------------------------------------------------------------------------------------------------------------
//#define LOG_ACTIONS_ENABLED // закомментировать, если не нужна запись действий на карту (например, события "включён полив" и т.п.)

//--------------------------------------------------------------------------------------------------------------------------------
// проверка целостности
//--------------------------------------------------------------------------------------------------------------------------------
#define UNUSED(expr) do { (void)(expr); } while (0)

#endif
