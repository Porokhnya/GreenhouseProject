/*
 ПЕРЕД КОМПИЛЯЦИЕЙ!

 0. Перед ПЕРВОЙ заливкой прошивки, в зависимости от типа используемой EEPROM - почистить EEPROM, забив все ячейки значением 0xFF !!!

 1. УСТАНОВИТЬ ВСЕ БИБЛИОТЕКИ ИМЕННО ИЗ АРХИВА С ПРОЕКТОМ (НЕКОТОРЫЕ БИБЛИОТЕКИ ПРАВЛЕНЫ ИМЕННО ПОД ПРОЕКТ)!!!
 
 2. ПРИ СБОРКЕ ПОД Mega2560: пойти в папку установки Arduino, найти в подпапке hardware\arduino\avr\cores\arduino
    файл HardwareSerial.h, открыть его в текстовом редакторе, найти внутри строчку
      #define SERIAL_RX_BUFFER_SIZE 64
      
    и поменять 64 на 128 
    
    ПЕРЕЗАПУСТИТЬ Arduino IDE !!!


 3. ПРИ СБОРКЕ ПОД Due, после установки через менеджер плат пакета "Arduino SAM boards" (в пакете одна плата Arduino Due),
    пойти в C:\Documents and Settings\ТУТ_ИМЯ_ЮЗЕРА\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.11\cores\arduino
    (версия 1.6.11 - для примера, смотрите, какая у вас версия будет лежать в подпапке "sam"), найти внутри папки файл
    RingBuffer.h, открыть его в текстовом редакторе, найти внутри строчку

    #define SERIAL_BUFFER_SIZE 128

    и поменять 128 на 1024

    ПЕРЕЗАПУСТИТЬ Arduino IDE !!!!

 4. ПРИ СБОРКЕ ПОД STM32 - установить поддержку Due (см. выше), затем распаковать архив Libraries\Arduino_STM32.zip в папку <ПАПКА_СКЕТЧЕЙ>\hardware
    ( если папки hardware в папке скетчей нет - создать её). Внутри папки hardware должна появиться папка Arduino_STM32, внутри которой будет код
    поддержки STM32.

    ВНИМАНИЕ: ПОДДЕРЖКА STM32 ТЕСТИРУЕТСЯ НА КАМНЕ STM32F103ZE6 !!!

    ВНИМАНИЕ: ПОДДЕРЖКА STM32 ТОЛЬКО ВВОДИТСЯ, ПОЭТОМУ НЕ ВСЕ МОДУЛИ ЕЩЁ ПРОТЕСТИРОВАНЫ НА КОМПИЛИРУЕМОСТЬ !!!

 5. В зависимости от типа платы (Mega2560, Due, STM32) - пойти в настройки Configuration_MEGA.h или Configuration_DUE.h или Configuration_STM32.h,
    привести настройки в тот вид, который вам нужен, внимательно читая инструкции.

 6. Если что-то работает не так - есть отладочные режимы (конфигуратор с ними не работает!) - пойти в Configuration_DEBUG.h,
    раскомментировать нужный отладочный режим, в мониторе порта (или любой другой терминальной программе) смотреть, что происходит.
    Если ничего не понятно, то: создать документ, где описать проблему (я, такой-то такой-то, делал то-то и то-то, не получается что-то),
    приложить к документу лог из монитора порта и выложить на форум, с просьбой к разработчику посмотреть, в чём дело. Не забыть приложить
    ваш файл настроек, а также указать, под какую плату компилируете и что из железа используете. Чем больше информации - тем проще будет вам помочь!

 После всех указанных действий проект готов к загрузке. Если что-то не получается, то: 1) всегда есть форум 2) во всяком софте есть ошибки 3) наверняка
 вы что-то недопоняли или делаете не так. Для утрясания всего этого добра есть форум, и раз вы пользуетесь этим проектом - вы уже знаете, где идёт обсуждение ;)

 Но уж если не знаете - пишите на spywarrior@gmail.com, я вам кину ссылку на обсуждение. Только помните: я делаю этот проект в свободное время, и физически не 
 способен удовлетворить все ваши потребности, ответить на тысячу вопросов сразу и т.п. Короче, такт и понимание - приветствуются.

 (с) Порохня Дмитрий, 2015-2019.
 
*/
 
#include "Globals.h"

#include "CommandBuffer.h"
#include "CommandParser.h"
#include "ModuleController.h"
#include "AlertModule.h"
#include "ZeroStreamListener.h"
#include "Memory.h"
#include "InteropStream.h"

#ifdef USE_SCENE_MODULE
#include "SceneModule.h"
#endif

#ifdef USE_HTTP_MODULE
#include "HttpModule.h"
#endif

#ifdef USE_PIN_MODULE
#include "PinModule.h"
#endif

#ifdef USE_STAT_MODULE
#include "StatModule.h"
#endif

#ifdef USE_TEMP_SENSORS
#include "TempSensors.h"
#endif

#ifdef USE_SMS_MODULE
#include "SMSModule.h"
#endif

#ifdef USE_WATERING_MODULE
#include "WateringModule.h"
#endif

#ifdef USE_LUMINOSITY_MODULE
#include "LuminosityModule.h"
#endif

#ifdef USE_HUMIDITY_MODULE
#include "HumidityModule.h"
#endif

#ifdef USE_WIFI_MODULE
#include "WiFiModule.h"
#endif

#ifdef USE_LOG_MODULE
#include "LogModule.h"
#endif

#ifdef USE_DELTA_MODULE
#include "DeltaModule.h"
#endif

#ifdef USE_LCD_MODULE
#include "LCDModule.h"
#endif

#ifdef USE_NEXTION_MODULE
#include "NextionModule.h"
#endif

#ifdef USE_WATERFLOW_MODULE
#include "WaterflowModule.h"
#endif

#ifdef USE_COMPOSITE_COMMANDS_MODULE
#include "CompositeCommandsModule.h"
#endif

#ifdef USE_SOIL_MOISTURE_MODULE
#include "SoilMoistureModule.h"
#endif

#ifdef USE_W5100_MODULE
#include "EthernetModule.h"
#endif

#ifdef USE_RESERVATION_MODULE
#include "ReservationModule.h"
#endif

#ifdef USE_TIMER_MODULE
#include "TimerModule.h"
#endif

#ifdef USE_PH_MODULE
#include "PHModule.h"
#endif

#ifdef USE_IOT_MODULE
#include "IoTModule.h"
#endif

#ifdef USE_TFT_MODULE
#include "TFTModule.h"
#endif

#ifdef USE_BUZZER
#include "Buzzer.h"
#endif

#ifdef USE_MCP_MODULE
#include "MCPModule.h"
#endif

///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////


#include "DelayedEvents.h"

#include<Wire.h>

#ifdef DUE_START_DEBUG
#define START_LOG(x) {Serial.println((x)); Serial.flush(); }
#else
#define START_LOG(x) (void) 0
#endif

// NEW CODE //////////////////////////////////////////
#if defined(USE_TIMER_ONE_FOR_ESP_SIM800) && ( defined(USE_SMS_MODULE) || defined(USE_WIFI_MODULE))
#include "TimerOne.h"
void TimerOneHandler()
{
   #ifdef USE_WIFI_MODULE
   if(!ESPInUpdate) // только есть ESP не в функции update, т.е. работает с внутренним буфером
   {
      ESP.readFromStream();
   }
    #endif

   #ifdef USE_SMS_MODULE
   if(!SIM800InUpdate) // только есть SIM800 не в функции update, т.е. работает с внутренним буфером
   {
    SIM800.readFromStream();
   }
   #endif   
}
#endif
// NEW CODE //////////////////////////////////////////

// таймер
unsigned long lastMillis = 0;

// Ждем команды из сериала
CommandBuffer commandsFromSerial(&Serial);

// Парсер команд
CommandParser commandParser;

// Контроллер модулей
ModuleController controller;

#ifdef USE_PIN_MODULE
//  Модуль управления цифровыми пинами
PinModule pinModule;
#endif

#ifdef USE_STAT_MODULE
// Модуль вывода статистики
StatModule statModule;
#endif

#ifdef USE_TEMP_SENSORS
// модуль опроса температурных датчиков и управления фрамугами
TempSensors tempSensors;
#endif

#ifdef USE_IOT_MODULE
// модуль отсыла данных на внешние IoT-хранилища
IoTModule iotModule; 
#endif

#ifdef USE_HTTP_MODULE
// модуль проверки на наличие входящих команд
HttpModule httpModule;
#endif

#ifdef USE_SMS_MODULE
// модуль управления по SMS
 SMSModule smsModule;
#endif

#ifdef USE_WATERING_MODULE
// модуль управления поливом
WateringModule wateringModule;
#endif

#ifdef USE_LUMINOSITY_MODULE
// модуль управления досветкой и получения значений освещённости
LuminosityModule luminosityModule;
#endif

#ifdef USE_HUMIDITY_MODULE
// модуль работы с датчиками влажности DHT
HumidityModule humidityModule;
#endif

#ifdef USE_LOG_MODULE
// модуль логгирования информации
LogModule logModule;
#endif

#ifdef USE_DELTA_MODULE
// модуль сбора дельт с датчиков
DeltaModule deltaModule;
#endif

#ifdef USE_LCD_MODULE
// модуль LCD
LCDModule lcdModule;
#endif

#ifdef USE_NEXTION_MODULE
// модуль Nextion
NextionModule nextionModule;
#endif

#ifdef USE_WATERFLOW_MODULE
// модуль учёта расхода воды
WaterflowModule waterflowModule;
#endif

#ifdef USE_COMPOSITE_COMMANDS_MODULE
// модуль составных команд
CompositeCommandsModule compositeCommands;
#endif

#ifdef USE_SOIL_MOISTURE_MODULE
// модуль датчиков влажности почвы
SoilMoistureModule soilMoistureModule;
#endif

#ifdef USE_PH_MODULE
// модуль контроля pH
PhModule phModule;
#endif

#ifdef USE_W5100_MODULE
// модуль поддержки W5100
EthernetModule ethernetModule;
#endif

#ifdef USE_RESERVATION_MODULE
ReservationModule reservationModule;
#endif

#ifdef USE_MCP_MODULE
MCPModule mcpModule;
#endif

#ifdef USE_TIMER_MODULE
TimerModule timerModule;
#endif

#ifdef USE_TFT_MODULE
TFTModule tftModule;
#endif

#ifdef USE_READY_DIODE
  #ifdef BLINK_READY_DIODE
   BlinkModeInterop readyDiodeBlinker;
  #endif
#endif

#ifdef USE_WIFI_MODULE
// модуль работы по Wi-Fi
WiFiModule wifiModule;
#endif

#ifdef USE_SCENE_MODULE
SceneModule sceneModule;
#endif

///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
DynamicSensorsResetModule dynamicResetSensors;
#endif
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////


ZeroStreamListener zeroStreamModule;
AlertModule alertsModule;

#ifdef USE_EXTERNAL_WATCHDOG
  typedef enum
  {
    WAIT_FOR_TRIGGERED,
    WAIT_FOR_NORMAL 
  } ExternalWatchdogState;
  
  typedef struct
  {
    uint16_t timer;
    ExternalWatchdogState state;
  } ExternalWatchdogSettings;

  ExternalWatchdogSettings watchdogSettings;
#endif
//--------------------------------------------------------------------------------------------------------------------------------
bool canCallYield = false;
//--------------------------------------------------------------------------------------------------------------------------------
void initI2C()
{

   #if TARGET_BOARD == MEGA_BOARD
   
    Wire.begin();
 //   Wire.setClock(I2C_SPEED);

   #elif TARGET_BOARD == DUE_BOARD

    // инициализация I2C, специфичная для Arduino Due

    // инициализируем стандартными методами
      Wire.begin();
   //   Wire.setClock(I2C_SPEED);

      if(DS3231_WIRE_NUMBER == 1)
      {
        Wire1.begin();
  //      Wire1.setClock(I2C_SPEED);
      }    
   
   #elif TARGET_BOARD == STM32_BOARD

    // инициализация для STM32
      Wire.begin();
  //    Wire.setClock(I2C_SPEED);

      if(DS3231_WIRE_NUMBER == 1)
      {
        Wire1.begin();
//        Wire1.setClock(I2C_SPEED);
      }
   #else
      #error "UNKNOWN TARGET BOARD!!!"
   #endif
     
}
//--------------------------------------------------------------------------------------------------------------------------------
void doResetI2C(uint8_t sclPin, uint8_t sdaPin)
{
  pinMode(sdaPin, OUTPUT);
  digitalWrite(sdaPin,HIGH);
  pinMode(sclPin,OUTPUT);
  
  for(uint8_t i=0;i<10;i++) // Send NACK signal
  {
    digitalWrite(sclPin,HIGH);
    delayMicroseconds(5);
    digitalWrite(sclPin,LOW);
    delayMicroseconds(5);   
  }

  // Send STOP signal
  digitalWrite(sdaPin,LOW);
  delayMicroseconds(5);
  digitalWrite(sclPin,HIGH);
  delayMicroseconds(2);
  digitalWrite(sdaPin,HIGH);
  delayMicroseconds(2);
  
  
  pinMode(sclPin,INPUT);   
  pinMode(sdaPin,INPUT);   
}
//--------------------------------------------------------------------------------------------------------------------------------
void resetI2C()
{
  doResetI2C(SCL, SDA);
  
 #if TARGET_BOARD == DUE_BOARD
    if(DS3231_WIRE_NUMBER == 1)
      doResetI2C(SCL1, SDA1);     
 #endif 
}
//--------------------------------------------------------------------------------------------------------------------------------
void setup() 
{
  canCallYield = false;


  Serial.begin(SERIAL_BAUD_RATE); // запускаем Serial на нужной скорости

#if (TARGET_BOARD == DUE_BOARD)
  while(!Serial); // ждём инициализации Serial
#endif

  START_LOG(1);

 
  #if TARGET_BOARD == STM32_BOARD
    WORK_STATUS.PinMode(20,INPUT,false);
    WORK_STATUS.PinMode(21,OUTPUT,false);
  #else
    WORK_STATUS.PinMode(SDA,INPUT,false);
    WORK_STATUS.PinMode(SCL,OUTPUT,false);
  #endif

  START_LOG(2);

  resetI2C();

  START_LOG(3);

  initI2C();  

   START_LOG(4);

  // инициализируем память (EEPROM не надо, а вот I2C - надо)
  MemInit(); 

  START_LOG(5);

  WORK_STATUS.PinMode(0,INPUT,false);
  WORK_STATUS.PinMode(1,OUTPUT,false);

  START_LOG(6);

  #ifdef USE_EXTERNAL_WATCHDOG
    WORK_STATUS.PinMode(WATCHDOG_REBOOT_PIN,OUTPUT,true);
    digitalWrite(WATCHDOG_REBOOT_PIN,WATCHDOG_NORMAL_LEVEL);

    watchdogSettings.timer = 0;
    watchdogSettings.state = WAIT_FOR_TRIGGERED;
  #endif
 
  // настраиваем все железки
  controller.Setup();

  START_LOG(7);

  #ifdef USE_BUZZER
  Buzzer.begin();
  #endif

  START_LOG(8);
   
  // устанавливаем провайдера команд для контроллера
  controller.SetCommandParser(&commandParser);

  START_LOG(9);
  
  // регистрируем модули  
  #ifdef USE_PIN_MODULE  
  controller.RegisterModule(&pinModule);
  #endif

  START_LOG(10);
  
  #ifdef USE_STAT_MODULE
  controller.RegisterModule(&statModule);
  #endif

  START_LOG(11);

  #ifdef USE_TEMP_SENSORS
  controller.RegisterModule(&tempSensors);
  #endif

  START_LOG(12);

  #ifdef USE_WATERING_MODULE
  controller.RegisterModule(&wateringModule);
  #endif

  START_LOG(13);

  #ifdef USE_LUMINOSITY_MODULE
  controller.RegisterModule(&luminosityModule);
  #endif

  START_LOG(14);

  #ifdef USE_HUMIDITY_MODULE
  controller.RegisterModule(&humidityModule);
  #endif

  START_LOG(15);

  #ifdef USE_DELTA_MODULE
  controller.RegisterModule(&deltaModule);
  #endif

  START_LOG(16);

  #ifdef USE_MCP_MODULE
  controller.RegisterModule(&mcpModule);
  #endif
  
  START_LOG(17);

  START_LOG(18);

  #ifdef USE_WATERFLOW_MODULE
  controller.RegisterModule(&waterflowModule);
  #endif

  START_LOG(19);

  #ifdef USE_COMPOSITE_COMMANDS_MODULE
  controller.RegisterModule(&compositeCommands);
  #endif

  START_LOG(20);
 
  #ifdef USE_SOIL_MOISTURE_MODULE
  controller.RegisterModule(&soilMoistureModule);
  #endif

  START_LOG(21);

  #ifdef USE_PH_MODULE
  controller.RegisterModule(&phModule);
  #endif

  START_LOG(22);

  #ifdef USE_W5100_MODULE
  controller.RegisterModule(&ethernetModule);
  #endif

  START_LOG(23);

  #ifdef USE_RESERVATION_MODULE
  controller.RegisterModule(&reservationModule);
  #endif

  START_LOG(24);

  #ifdef USE_TIMER_MODULE
  controller.RegisterModule(&timerModule);
  #endif

  #ifdef USE_SCENE_MODULE
  controller.RegisterModule(&sceneModule);
  #endif

///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
  #ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
  controller.RegisterModule(&dynamicResetSensors);
  #endif
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////  

  START_LOG(25);

  #ifdef USE_LOG_MODULE
  controller.RegisterModule(&logModule);
  controller.SetLogWriter(&logModule); // задаём этот модуль как модуль, который записывает события в лог
  #endif

  START_LOG(26);

  // модуль Wi-Fi регистрируем до модуля SMS, поскольку Wi-Fi дешевле, чем GPRS, для отсыла данных в IoT-хранилища
  #ifdef USE_WIFI_MODULE
  controller.RegisterModule(&wifiModule);
  #endif 

  START_LOG(27);

  #ifdef USE_SMS_MODULE
  controller.RegisterModule(&smsModule);
  #endif

  START_LOG(28);

  #ifdef USE_IOT_MODULE
    controller.RegisterModule(&iotModule);
  #endif

  START_LOG(29);

  #ifdef USE_HTTP_MODULE
    controller.RegisterModule(&httpModule);
  #endif

  START_LOG(30);

  #ifdef USE_LCD_MODULE
  controller.RegisterModule(&lcdModule);
  #endif

  #ifdef USE_NEXTION_MODULE
  controller.RegisterModule(&nextionModule);
  #endif

  #ifdef USE_TFT_MODULE
    controller.RegisterModule(&tftModule);
  #endif

 START_LOG(31);

  controller.RegisterModule(&zeroStreamModule);

 START_LOG(32);

 // модуль алертов регистрируем последним, т.к. он должен вычитать зависимости с уже зарегистрированными модулями
  controller.RegisterModule(&alertsModule);

 START_LOG(33);

  controller.begin(); // начинаем работу

 START_LOG(34);

  // collect garbage
  while(Serial.available()) 
    Serial.read();

  // Печатаем в Serial готовность
  Serial.print(READY);

  #ifdef USE_DS3231_REALTIME_CLOCK
  
   RealtimeClock rtc = controller.GetClock();
   RTCTime tm = rtc.getTime();

   Serial.print(F(", "));
   Serial.print(rtc.getDayOfWeekStr(tm));
   Serial.print(F(" "));
   Serial.print(rtc.getDateStr(tm));
   Serial.print(F(" - "));
   Serial.print(rtc.getTimeStr(tm));
      
  #endif 

  Serial.println();

 START_LOG(35);


// NEW CODE //////////////////////////////////////////
#if defined(USE_TIMER_ONE_FOR_ESP_SIM800) && ( defined(USE_SMS_MODULE) || defined(USE_WIFI_MODULE))
Timer1.initialize(TIMER_ONE_PERIOD);
Timer1.attachInterrupt(TimerOneHandler);
Timer1.start();
#endif
// NEW CODE //////////////////////////////////////////
 

  #ifdef USE_LOG_MODULE
    controller.Log(&logModule,READY); // печатаем в файл действий строчку Ready, которая скажет нам, что мега стартовала
  #endif

  // пискнем при старте, если есть баззер
  #ifdef USE_BUZZER
  Buzzer.buzz();
  #endif

 START_LOG(36);

 canCallYield = true;
  
}
//--------------------------------------------------------------------------------------------------------------------------------
// эта функция вызывается после обновления состояния каждого модуля.
// передаваемый параметр - указатель на обновлённый модуль.
// если модулю предстоит долгая работа - помимо этого инструмента
// модуль должен дёргать функцию yield, если чем-то долго занят!
//--------------------------------------------------------------------------------------------------------------------------------
void ModuleUpdateProcessed(AbstractModule* module)
{
  UNUSED(module);

  // используем её, чтобы проверить состояние порта UART для WI-FI-модуля - вдруг надо внеочередно обновить
    #ifdef USE_WIFI_MODULE
    // модуль Wi-Fi обновляем каждый раз после обновления очередного модуля
     ESP.update();
    #endif

   #ifdef USE_SMS_MODULE
   // и модуль GSM тоже тут обновим
    SIM800.update();
   #endif     
}
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_EXTERNAL_WATCHDOG
void updateExternalWatchdog()
{
  static unsigned long watchdogLastMillis = millis();
  unsigned long watchdogCurMillis = millis();

  uint16_t dt = watchdogCurMillis - watchdogLastMillis;
  watchdogLastMillis = watchdogCurMillis;

      watchdogSettings.timer += dt;
      switch(watchdogSettings.state)
      {
        case WAIT_FOR_TRIGGERED:
        {
          if(watchdogSettings.timer >= WATCHDOG_WORK_INTERVAL)
          {
            watchdogSettings.timer = 0;
            watchdogSettings.state = WAIT_FOR_NORMAL;
            digitalWrite(WATCHDOG_REBOOT_PIN, WATCHDOG_TRIGGERED_LEVEL);
          }
        }
        break;

        case WAIT_FOR_NORMAL:
        {
          if(watchdogSettings.timer >= WATCHDOG_PULSE_DURATION)
          {
            watchdogSettings.timer = 0;
            watchdogSettings.state = WAIT_FOR_TRIGGERED;
            digitalWrite(WATCHDOG_REBOOT_PIN, WATCHDOG_NORMAL_LEVEL);
          }          
        }
        break;
      }  
  
}
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void processCommandsFromSerial()
{
 // смотрим, есть ли входящие команды
   if(commandsFromSerial.HasCommand())
   {
    // есть новая команда
    Command cmd;
    if(commandParser.ParseCommand(commandsFromSerial.GetCommand(), cmd))
    {
       Stream* answerStream = commandsFromSerial.GetStream();
      // разобрали, назначили поток, с которого пришла команда
        cmd.SetIncomingStream(answerStream);

      // запустили команду в обработку
       controller.ProcessModuleCommand(cmd);
 
    } // if
    else
    {
      // что-то пошло не так, игнорируем команду
    } // else
    
    commandsFromSerial.ClearCommand(); // очищаем полученную команду
   } // if  
}
//--------------------------------------------------------------------------------------------------------------------------------
void loop() 
{
// отсюда можно добавлять любой сторонний код

// до сюда можно добавлять любой сторонний код

  
    // вычисляем время, прошедшее с момента последнего вызова
    unsigned long curMillis = millis();
    uint16_t dt = curMillis - lastMillis;
    
    lastMillis = curMillis; // сохраняем последнее значение вызова millis()


   #ifdef USE_EXTERNAL_WATCHDOG
     updateExternalWatchdog();
   #endif // USE_EXTERNAL_WATCHDOG
    
#ifdef USE_READY_DIODE

  #ifdef BLINK_READY_DIODE

    // будем мигать информационным диодом
    static bool blink_ready_diode_inited = false;
    if(!blink_ready_diode_inited) {
      blink_ready_diode_inited = true;
      readyDiodeBlinker.begin(DIODE_READY_PIN);
      readyDiodeBlinker.blink(READY_DIODE_BLINK_INTERVAL);
    }

    readyDiodeBlinker.update(dt);
  #else
    static bool blink_ready_diode_inited = false;
    if(!blink_ready_diode_inited) {
      blink_ready_diode_inited = true;

      // просто зажигаем информационный светодиод при старте
      
      #if INFO_DIODES_DRIVE_MODE == DRIVE_DIRECT
        WORK_STATUS.PinMode(DIODE_READY_PIN,OUTPUT);
        WORK_STATUS.PinWrite(DIODE_READY_PIN,HIGH);
      #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23S17
        #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
          WORK_STATUS.MCP_SPI_PinMode(INFO_DIODES_MCP23S17_ADDRESS,DIODE_READY_PIN,OUTPUT);
          WORK_STATUS.MCP_SPI_PinWrite(INFO_DIODES_MCP23S17_ADDRESS,DIODE_READY_PIN,HIGH);
        #endif
      #elif INFO_DIODES_DRIVE_MODE == DRIVE_MCP23017
        #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
          WORK_STATUS.MCP_I2C_PinMode(INFO_DIODES_MCP23017_ADDRESS,DIODE_READY_PIN,OUTPUT);
          WORK_STATUS.MCP_I2C_PinWrite(INFO_DIODES_MCP23017_ADDRESS,DIODE_READY_PIN,HIGH);
        #endif
      #endif        
    }
  #endif

#endif

  // смотрим, есть ли входящие команды
   processCommandsFromSerial();
    
   // обновляем состояние всех зарегистрированных модулей
   controller.UpdateModules(dt,ModuleUpdateProcessed);

   CoreDelayedEvent.update();

   
// отсюда можно добавлять любой сторонний код

// до сюда можно добавлять любой сторонний код

}
//--------------------------------------------------------------------------------------------------------------------------------
void esp_sim800_call()
{
   #if defined(USE_TIMER_ONE_FOR_ESP_SIM800) && ( defined(USE_SMS_MODULE) || defined(USE_WIFI_MODULE))
   Timer1.stop();
   #endif
  
   #ifdef USE_WIFI_MODULE
    ESP.readFromStream();
    #endif

   #ifdef USE_SMS_MODULE
   SIM800.readFromStream();
   #endif   

   #if defined(USE_TIMER_ONE_FOR_ESP_SIM800) && ( defined(USE_SMS_MODULE) || defined(USE_WIFI_MODULE))
   Timer1.resume();
   #endif  
}
//--------------------------------------------------------------------------------------------------------------------------------
// обработчик простоя, используем и его. Если сторонняя библиотека устроена правильно - она будет
// вызывать yield периодически - этим грех не воспользоваться, чтобы избежать потери данных
// в портах UART. 
//--------------------------------------------------------------------------------------------------------------------------------
void yield()
{
  if(!canCallYield)
    return;
    
// отсюда можно добавлять любой сторонний код, который надо вызывать, когда МК чем-то долго занят (например, чтобы успокоить watchdog)


// до сюда можно добавлять любой сторонний код

   #ifdef USE_EXTERNAL_WATCHDOG
     updateExternalWatchdog();
   #endif // USE_EXTERNAL_WATCHDOG

    esp_sim800_call();

   #ifdef USE_LCD_MODULE
    rotaryEncoder.update(); // обновляем энкодер меню
   #endif

   CoreDelayedEvent.update();


// отсюда можно добавлять любой сторонний код, который надо вызывать, когда МК чем-то долго занят (например, чтобы успокоить watchdog)

// до сюда можно добавлять любой сторонний код

}
//--------------------------------------------------------------------------------------------------------------------------------
void serialEvent1()
{  
  esp_sim800_call(); 
}
//--------------------------------------------------------------------------------------------------------------------------------
void serialEvent2()
{
 esp_sim800_call();
}
//--------------------------------------------------------------------------------------------------------------------------------
void serialEvent3()
{
  esp_sim800_call();
}
//--------------------------------------------------------------------------------------------------------------------------------

