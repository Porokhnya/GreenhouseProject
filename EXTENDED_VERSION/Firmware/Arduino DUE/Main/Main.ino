 /*
 * 
 * ВНИМАНИЕ !!! ВНИМАНИЕ !!! ВНИМАНИЕ !!!
 * 
 * ДАННАЯ РАСШИРЕННАЯ ВЕРСИЯ ПРОШИВКИ СОБИРАЕТСЯ ТОЛЬКО ПОД ПЛАТУ ARDUINO DUE, Т.К. АДАПТИРОВАЛАСЬ И СОБИРАЛАСЬ СПЕЦИАЛЬНО ПОД СВЯЗКУ ЭТОГО МК
 * С СЕМИДЮЙМОВЫМ МОНИТОРОМ. ПОПЫТКА СОБРАТЬ ЭТУ ВЕРСИЮ ПОД ДРУГУЮ ПЛАТУ - ОБРЕЧЕНА НА ПРОВАЛ !!!
 * 
 * 
 * 
 ПЕРЕД КОМПИЛЯЦИЕЙ!

 1. Перед ПЕРВОЙ заливкой прошивки, в зависимости от типа используемой EEPROM - почистить EEPROM, забив все ячейки значением 0xFF !!!

 2. УСТАНОВИТЬ ВСЕ БИБЛИОТЕКИ ИМЕННО ИЗ АРХИВА С ПРОЕКТОМ (НЕКОТОРЫЕ БИБЛИОТЕКИ ПРАВЛЕНЫ ИМЕННО ПОД ПРОЕКТ)!!!
 
 3. ПРИ СБОРКЕ ПОД Due, после установки через менеджер плат пакета "Arduino SAM boards" (в пакете одна плата Arduino Due),
    пойти в C:\Documents and Settings\ТУТ_ИМЯ_ЮЗЕРА\AppData\Local\Arduino15\packages\arduino\hardware\sam\1.6.11\cores\arduino
    (версия 1.6.11 - для примера, смотрите, какая у вас версия будет лежать в подпапке "sam"), найти внутри папки файл
    RingBuffer.h, открыть его в текстовом редакторе, найти внутри строчку

    #define SERIAL_BUFFER_SIZE 128

    и поменять 128 на 1024

    ПЕРЕЗАПУСТИТЬ Arduino IDE !!!!

 4. Пойти в настройки Configuration_DUE.h, Configuration_DUE_HARDWARE.h,
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

 (с) Порохня Дмитрий, 2015-2020.
 
*/
 
#include "Globals.h"

#include "CommandBuffer.h"
#include "CommandParser.h"
#include "ModuleController.h"
#include "AlertModule.h"
#include "ZeroStreamListener.h"
#include "Memory.h"
#include "InteropStream.h"
#include "PowerManager.h"

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

/*
#ifdef USE_LCD_MODULE
#include "LCDModule.h"
#endif
*/

/*
#ifdef USE_NEXTION_MODULE
#include "NextionModule.h"
#endif
*/

#ifdef USE_WATERFLOW_MODULE
#include "WaterflowModule.h"
#endif

#ifdef USE_COMPOSITE_COMMANDS_MODULE
#include "CompositeCommandsModule.h"
#endif

#ifdef USE_SOIL_MOISTURE_MODULE
#include "SoilMoistureModule.h"
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

#ifdef USE_CO2_MODULE
#include "CO2Module.h"
#endif

#ifdef USE_EC_MODULE
#include "ECModule.h"
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

#ifdef USE_PRESSURE_MODULE
#include "PressureModule.h"
#endif

///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////
#ifdef USE_DYNAMIC_SENSORS_RESET_MODULE
#include "DynamicSensorsResetModule.h"
#endif
///////////////////////////// КОД СБРОСА ДИНАМИЧЕСКИХ ДАТЧИКОВ ////////////////////////////////////////

#ifdef USE_DOOR_MODULE
#include "DoorModule.h"
#endif

#ifdef USE_DS18B20_EMULATION_MODULE
#include "OneWireEmulationModule.h"
#endif

#ifdef USE_SCHEDULE_MODULE
#include "ScheduleModule.h"
#endif

#ifdef USE_WATER_TANK_MODULE
#include "WaterTankModule.h"
#endif


#include "LogicManageModule.h"
#include "EEPROMSettingsModule.h"

#include "DelayedEvents.h"

#include<Wire.h>

#ifdef DUE_START_DEBUG
#define START_LOG(x) {Serial.println((x)); Serial.flush(); }
#else
#define START_LOG(x) (void) 0
#endif


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

/*
#ifdef USE_LCD_MODULE
// модуль LCD
LCDModule lcdModule;
#endif
*/

/*
#ifdef USE_NEXTION_MODULE
// модуль Nextion
NextionModule nextionModule;
#endif
*/

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

#ifdef USE_CO2_MODULE
// модуль контроля CO2
CO2Module co2Module;
#endif

#ifdef USE_EC_MODULE
// модуль контроля EC
ECModule ecModule;
#endif

#ifdef USE_RESERVATION_MODULE
ReservationModule reservationModule;
#endif

#ifdef USE_MCP_MODULE
MCPModule mcpModule;
#endif

#ifdef USE_PRESSURE_MODULE
PressureModule pressureModule;
#endif

#ifdef USE_TIMER_MODULE
TimerModule timerModule;
#endif

#ifdef USE_TFT_MODULE
TFTModule tftModule;
#endif

#ifdef USE_DOOR_MODULE
DoorModule doorModule;
#endif

#ifdef USE_DS18B20_EMULATION_MODULE
OneWireEmulationModule oneWireEmulation;
#endif

#ifdef USE_SCHEDULE_MODULE
ScheduleModule scheduleModule;
#endif

#ifdef USE_WATER_TANK_MODULE
WaterTankModule waterTankModule;
#endif


BlinkModeInterop readyDiodeBlinker;
bool canWorkWithReadyDiode = false;

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

LogicManageModuleClass logicManageModule;
EEPROMSettingsModule eepromSettingsModule;
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
#if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//--------------------------------------------------------------------------------------------------------------------------------
#define EEFC_FKEY 0x5A
//#define EEFC_FMR_SCOD   (0x1ul << 16) 
//#define IFLASH0_ADDR  0x00080000                               // Flash memory begins at 0x00080000
//#define IFLASH1_ADDR ( IFLASH0_ADDR + IFLASH_SIZE / 2 )        // IFLASH1_ADDR  = 0x000C0000


__attribute__ ((section (".ramfunc")))
void ReadUniqueID( uint32_t * latch_buffer )
{
    char __FWS;
  
                                                                  // Set bit 16 of EEFC_FMR : See chap. 49.1.1.2 page 1442   
    EFC0->EEFC_FMR |= EEFC_FMR_SCOD;                              // Sequential code optimization disable
    
  __FWS = (EFC0->EEFC_FMR & EEFC_FMR_FWS_Msk)>>EEFC_FMR_FWS_Pos ; // Save FWS value
    EFC0->EEFC_FMR &=~ EEFC_FMR_FWS_Msk;
    EFC0->EEFC_FMR |= EEFC_FMR_FWS(6);                            // 6+1 wait states for read and write operations
    
   
    EFC0->EEFC_FMR &=~EEFC_FMR_FAM;                               // 128-bit access in read Mode only, to enhance access speed.
    
    while(!(EFC0->EEFC_FSR & EEFC_FSR_FRDY));                     // Send the  STUI command if FRDY bit is high to begin reading in flash
    EFC0->EEFC_FCR = EEFC_FCR_FKEY(EEFC_FKEY) | EFC_FCMD_STUI ;    
                                                                  //Wait till FRDY falls down
    while(EFC0->EEFC_FSR & EEFC_FSR_FRDY);   
                                                                  // The Unique Identifier is located in the first 128 bits of 
                                                                  // the Flash bank 0 in between 0x080000 and 0x08000C (and of the Flash bank 1 in between 0X0C0000 and 0x0C000C ??) 
    memcpy(latch_buffer, (void*)IFLASH0_ADDR, 16);                // Read first 128 bits ( 16 first bytes) in one shot beginning at address IFLASHIndex_ADDR
       
                                                                  // Send the SPUI command to stop reading in flash    
    EFC0->EEFC_FCR = EEFC_FCR_FKEY(EEFC_FKEY) | EFC_FCMD_SPUI ;
                                                                  // Wait till FRDY rises up
    while(!(EFC0->EEFC_FSR & EEFC_FSR_FRDY));
    
                                                                  // Clear bit 16 of EEFC_FMR : See chap. 49.1.1.2 page 1442
    EFC0->EEFC_FMR &= ~EEFC_FMR_SCOD;                             // Sequential code optimization enable
    
    EFC0->EEFC_FMR &=~ EEFC_FMR_FWS_Msk;                          // Restore FWS value
    EFC0->EEFC_FMR |= EEFC_FMR_FWS(__FWS);                        
   
}//--------------------------------------------------------------------------------------------------------------------------------
#endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
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

    #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
    uint32_t latch_buffer[4]; 
    ReadUniqueID( latch_buffer ) ;
    
    // сохраняем ключи для дальнейшей проверки регистрационного кода
    MainController->saveKeyParts(latch_buffer[0],latch_buffer[1],latch_buffer[2],latch_buffer[3]);

    #endif // (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)

  START_LOG(7);

  START_LOG(8);
   
  // устанавливаем провайдера команд для контроллера
  controller.SetCommandParser(&commandParser);

  START_LOG(9);

  controller.RegisterModule(&eepromSettingsModule);

    // настраиваем менеджер питания
  PowerManager.begin();

  #ifdef USE_BUZZER
  Buzzer.begin();
  #endif
  
  
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

  #ifdef USE_PRESSURE_MODULE
  controller.RegisterModule(&pressureModule);
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

  #ifdef USE_CO2_MODULE
  controller.RegisterModule(&co2Module);
  #endif  

  START_LOG(22);

  #ifdef USE_EC_MODULE
  controller.RegisterModule(&ecModule);
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

  #ifdef USE_DOOR_MODULE
  controller.RegisterModule(&doorModule);
  #endif

  #ifdef USE_DS18B20_EMULATION_MODULE
  controller.RegisterModule(&oneWireEmulation);
  #endif

  START_LOG(25);

    #ifdef USE_WATER_TANK_MODULE
      controller.RegisterModule(&waterTankModule);
    #endif


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

/*
  #ifdef USE_LCD_MODULE
  controller.RegisterModule(&lcdModule);
  #endif
*/
/*
  #ifdef USE_NEXTION_MODULE
  controller.RegisterModule(&nextionModule);
  #endif
*/
  #ifdef USE_TFT_MODULE
    controller.RegisterModule(&tftModule);
  #endif

 START_LOG(31);

  controller.RegisterModule(&zeroStreamModule);
  controller.RegisterModule(&logicManageModule);
  

 START_LOG(32);

 // модуль алертов регистрируем последним, т.к. он должен вычитать зависимости с уже зарегистрированными модулями
  controller.RegisterModule(&alertsModule);

 START_LOG(33);

#ifdef USE_SCHEDULE_MODULE // регистрируем модуль расписаний последним, чтобы он обновлялся последним, и сообщал уже обновившим своё состояние модулям о возможных сменах настроек
  controller.RegisterModule(&scheduleModule);
#endif
 

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

  #ifdef USE_LOG_MODULE
    controller.Log(&logModule,READY); // печатаем в файл действий строчку Ready, которая скажет нам, что контроллер начал работу
  #endif



 START_LOG(36);

  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
  
    String part1 = String((uint32_t)latch_buffer[0], HEX);
    String part2 = String((uint32_t)latch_buffer[1], HEX);
    String part3 = String((uint32_t)latch_buffer[2], HEX);
    String part4 = String((uint32_t)latch_buffer[3], HEX);

    bool b1 = RulesDispatcher->getSKpart() == part1;
    bool b2 = ZeroStream->getSKpart() == part2;
    bool b3 = MainController->getSKpart() == part3;
    bool b4 = ModuleInterop.getSKpart() == part4;

    if(!b1 || !b2 || !b3 || !b4)
    {
            
      Serial.println(F("UNREGISTERED COPY DETECTED!"));

      #ifdef USE_TFT_MODULE
        tftModule.halt();
      #else
        while(1);
      #endif

    } // if(!b1
     
  #endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)

  #ifdef SD_USED

    if(controller.SDSupported() && !controller.HasSDCard())
    {
      Serial.println(F("NO SD FOUND!"));
       
      #ifdef USE_TFT_MODULE
       Vector<const char*> lines;
       lines.push_back("НЕ НАЙДЕНА SD-КАРТОЧКА!");
       lines.push_back("");
       lines.push_back("ВСТАВЬТЕ КАРТОЧКУ И ПЕРЕЗАГРУЗИТЕ КОНТРОЛЛЕР.");
       MessageBox->halt("СООБЩЕНИЕ",lines,true,true);
       tftModule.Update();
      #else
        while(1);
      #endif
    }
  #endif // SD_USED
  

  // тут проверяем на инициализацию EEPROM
  uint8_t controlByte = MemRead(0);
  if(controlByte != MEM_CONTROL_BYTE)
  {
      #ifdef USE_TFT_MODULE
        TFTScreen->switchToScreen("MEMINIT");
        tftModule.Update();
      #else
        Serial.println(F("Start EEPROM clearance..."));
        MemClear();
        Serial.println(F("Please restart controller."));
        while(1);       
      #endif
  }

  // пискнем при старте, если есть баззер
  #ifdef USE_BUZZER
  Buzzer.buzz();
  #endif   

 // настраиваем информационные диоды
 DiodesBinding bnd = HardwareBinding->GetDiodesBinding();
 if(bnd.LinkType != linkUnbinded)
 {
    if(bnd.LinkType == linkDirect)
    {
        if(bnd.ReadyDiodePin != UNBINDED_PIN && bnd.ReadyDiodePin > 1)
        {
          #ifndef DISABLE_DIODES_CONFIGURE
          if(EEPROMSettingsModule::SafePin(bnd.ReadyDiodePin))
          #endif
          {
             canWorkWithReadyDiode = true;
          }
        }        
    }
    else
    if(bnd.LinkType == linkMCP23S17)
    {
      #if defined(USE_MCP23S17_EXTENDER) && COUNT_OF_MCP23S17_EXTENDERS > 0
      if(bnd.ReadyDiodePin != UNBINDED_PIN)
        {
             canWorkWithReadyDiode = true;
        }

        
      #endif
    }
    else
    if(bnd.LinkType == linkMCP23017)
    {
      #if defined(USE_MCP23017_EXTENDER) && COUNT_OF_MCP23017_EXTENDERS > 0
      if(bnd.ReadyDiodePin != UNBINDED_PIN)
        {
             canWorkWithReadyDiode = true;
        }

       
      #endif
    }

    if(canWorkWithReadyDiode)
    {
      readyDiodeBlinker.begin(bnd.ReadyDiodePin);

        if(bnd.BlinkReadyDiode == 1)
        {
          readyDiodeBlinker.blink(READY_DIODE_BLINK_INTERVAL);      
        }
        else
        {
          canWorkWithReadyDiode = false;
          readyDiodeBlinker.out(bnd.Level); // включаем светодиод при старте
        }
    }
    
 } // if(bnd.LinkType != linkUnbinded)
  
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

  CoreDelayedEvent.update();

  // используем её, чтобы проверить состояние порта UART для WI-FI-модуля - вдруг надо внеочередно обновить
    #ifdef USE_WIFI_MODULE
    // модуль Wi-Fi обновляем каждый раз после обновления очередного модуля
     if(module != &wifiModule) 
     {
        ESP.update();
     }
    #endif

   #ifdef USE_SMS_MODULE
   // и модуль GSM тоже тут обновим
    if(module != &smsModule)
    {
      SIM800.update();
    }
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
//uint32_t start = millis();
// до сюда можно добавлять любой сторонний код

  


   #ifdef USE_EXTERNAL_WATCHDOG
     updateExternalWatchdog();
   #endif // USE_EXTERNAL_WATCHDOG


    if(canWorkWithReadyDiode)
    {
      readyDiodeBlinker.update();   
    }

  // смотрим, есть ли входящие команды
   processCommandsFromSerial();
    
   // обновляем состояние всех зарегистрированных модулей
   controller.UpdateModules(ModuleUpdateProcessed);

   CoreDelayedEvent.update();

   
// отсюда можно добавлять любой сторонний код
//uint32_t end = millis() - start;
//Serial.print("One loop() pass time, ms: ");
//Serial.println(end);
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

