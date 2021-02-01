#pragma once
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "Globals.h"

#ifdef USE_TFT_MODULE

#include "TinyVector.h"
#include "RTCSupport.h"

#if TARGET_BOARD == STM32_BOARD
#include <UTFTSTM32.h>
#else
#include <UTFT.h>
#endif

#include <URTouchCD.h>
#include <URTouch.h>
#include "UTFT_Buttons_Rus.h"
#include "UTFTRus.h"
#include "Settings.h"
#include "LogicManageModule.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_IDLE_SCREEN_BUTTON_WIDTH 128
#define TFT_IDLE_SCREEN_BUTTON_HEIGHT 90
#define TFT_IDLE_SCREEN_BUTTON_SPACING 5
#define TFT_BACK_BUTTON_WIDTH 250

#define INFO_BOX_WIDTH 240
#define INFO_BOX_HEIGHT 80
#define INFO_BOX_V_SPACING 20
#define INFO_BOX_CONTENT_PADDING 8

#define SENSOR_BOX_WIDTH 240
#define SENSOR_BOX_HEIGHT 120
#define SENSOR_BOX_V_SPACING 30
#define SENSOR_BOX_H_SPACING 20
#define SENSOR_BOXES_PER_LINE 3

#define CHANNELS_BUTTONS_PER_LINE 4
#define CHANNELS_BUTTON_WIDTH 165
#define CHANNELS_BUTTON_HEIGHT 50
#define ALL_CHANNELS_BUTTON_WIDTH 226
#define ALL_CHANNELS_BUTTON_HEIGHT 70

#define TFT_TEXT_INPUT_WIDTH 170
#define TFT_TEXT_INPUT_HEIGHT 80
#define TFT_ARROW_BUTTON_WIDTH 70
#define TFT_ARROW_BUTTON_HEIGHT 80
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTMenu;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  int x;
  int y;
  int w;
  int h;
} TFTInfoBoxContentRect;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTInfoBox
{
  public:
    TFTInfoBox(const char* caption, int width, int height, int x, int y, int captionXOffset=0);
    ~TFTInfoBox();

    void draw(TFTMenu* menuManager);
    void drawCaption(TFTMenu* menuManager, const char* caption);
    int getWidth() {return boxWidth;}
    int getHeight() {return boxHeight;}
    int getX() {return posX;}
    int getY() {return posY;}
    const char* getCaption() {return boxCaption;}

    TFTInfoBoxContentRect getContentRect(TFTMenu* menuManager);


   private:

    int boxWidth, boxHeight, posX, posY, captionXOffset;
    const char* boxCaption;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void drawValueInBox(TFTInfoBox* box, const String& strVal, uint8_t* font = SevenSegNumFontMDS);
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct ITickHandler
{
  virtual void onTick() = 0;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TickerClass
{
  public:
    TickerClass();
    ~TickerClass();

   void setIntervals(uint16_t beforeStartTickInterval,uint16_t tickInterval);
   void start(ITickHandler* h);
   void stop();
   void tick();

private:

  uint16_t beforeStartTickInterval;
  uint16_t tickInterval;

  uint32_t timer;
  bool started, waitBefore;

  ITickHandler* handler;
  
};
extern TickerClass Ticker;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// абстрактный класс экрана для TFT
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class AbstractTFTScreen
{
  public:

    virtual void setup(TFTMenu* menuManager) = 0;
    virtual void update(TFTMenu* menuManager) = 0;
    virtual void draw(TFTMenu* menuManager) = 0;
    virtual void onActivate(TFTMenu* menuManager){}
    virtual void onButtonPressed(TFTMenu* menuManager,int buttonID) {}
    virtual void onButtonReleased(TFTMenu* menuManager,int buttonID) {}
  
    AbstractTFTScreen();
    virtual ~AbstractTFTScreen();
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct MessageBoxResultSubscriber
{
  virtual void onMessageBoxResult(bool okPressed) = 0;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
struct KeyboardInputTarget
{
  virtual void onKeyboardInputResult(const String& input, bool okPressed) = 0;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс экрана ожидания
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool isWindowsOpen : 1;
  bool windowsAutoMode : 1;

  bool isWaterOn : 1;
  bool waterAutoMode : 1;

  bool isLightOn : 1;
  bool lightAutoMode : 1;
  
} IdleScreenFlags;
#pragma pack(pop)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  TFTInfoBox* box;
  
} TFTInfoBoxWithIndex;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define TFT_IDLE_SCREENS 6 // сколько у нас экранов на экране ожидания
#define TFT_IDLE_STATUSES_SCREEN_NUMBER 2
#define TFT_IDLE_SENSORS_SCREEN_NUMBER 0
#define TFT_IDLE_SENSORS_SCREEN_NUMBER2 1
#define TFT_IDLE_HEAT_SCREEN_NUMBER 3
#define TFT_IDLE_FLOW_SCREEN_NUMBER 4
#define TFT_IDLE_PH_SCREEN_NUMBER 5
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTDriveScreen : public AbstractTFTScreen
{
public:
    TFTDriveScreen();
    ~TFTDriveScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

private:

  UTFT_Buttons_Rus* screenButtons;

  int backButton;
  int windowsButton;
  int waterButton;
  int lightButton;
  int heatButton;
  int shadowButton;
  int flowButton;
  int cycleVentButton;
  int ventButton;
  int thermostatButton;
  int phButton;
  int co2Button;
  int doorButton;
  int sprayButton;
  int ecButton;
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CO2_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTCO2SettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTCO2SettingsScreen();
    ~TFTCO2SettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      Vector<int> weekdaysButtons;

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* startTimeBox;
      int decStartTimeButton, incStartTimeButton;
      void incStartTime(int val);

      TFTInfoBox* endTimeBox;
      int decEndTimeButton, incEndTimeButton;
      void incEndTime(int val);

      TFTInfoBox* ppmBox;
      int decPPMButton, incPPMButton;
      void incPPM(int val);

      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      CO2Settings settings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CO2_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_DOOR_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "DoorModule.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTDoorScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTDoorScreen();
    ~TFTDoorScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();

    void show(uint8_t channel);

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка температуры
      TFTInfoBox* tempBox;
      int decTempButton, incTempButton;
      void incTemp(int val);   

      // датчик температуры
      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      // время начала работы, минут от начала суток
      TFTInfoBox* startWorkTimeBox;
      int decStartWorkTimeButton, incStartWorkTimeButton;
      void incStartWorkTime(int val);

      // время окончания работы, минут от начала суток
      TFTInfoBox* endWorkTimeBox;
      int decEndWorkTimeButton, incEndWorkTimeButton;
      void incEndWorkTime(int val);

      // время работы привода, секунд  
      TFTInfoBox* driveWorkTimeBox;
      int decDriveWorkTimeButton, incDriveWorkTimeButton;
      void incDriveWorkTime(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(String& result);

      uint16_t tempSensorsCount, humiditySensorsCount;

      DoorSettings doorSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTDoorsScreen : public AbstractTFTScreen
{
public:
    TFTDoorsScreen();
    ~TFTDoorsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    
private:

  UTFT_Buttons_Rus* screenButtons;

  int backButton;
  int door1Button;
  int door2Button;  

  int door1ModeButton, door2ModeButton;
  DoorWorkMode door1Mode, door2Mode;

  int door1OpenCloseButton, door2OpenCloseButton;
  bool door1OpenFlag, door2OpenFlag;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_DOOR_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERFLOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTFlowScreen : public AbstractTFTScreen, public MessageBoxResultSubscriber, public ITickHandler
{
public:
    TFTFlowScreen();
    ~TFTFlowScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onMessageBoxResult(bool okPressed);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
    
private:

  UTFT_Buttons_Rus* screenButtons;

  int backButton;
  int resetFlow1Button;
  int resetFlow2Button;
  int saveButton;
  int tickerButton;

  uint32_t blinkTimer;
  bool blinkOn, blinkActive;
  void blinkSaveSettingsButton(bool bOn);

  void saveSettings();

  TFTInfoBox* cal1Box;
  int decCal1Button, incCal1Button;
  void incCal1(int val);

  TFTInfoBox* cal2Box;
  int decCal2Button, incCal2Button;
  void incCal2(int val);

  uint8_t cal1, cal2;
  

  int flowToReset;
  bool confirmMode;
  bool messageBoxOkPressed;
  bool wantShowOkMessage;

  void confirm(uint8_t num);
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERFLOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTIdleScreen : public AbstractTFTScreen
{
  public:
  
    TFTIdleScreen();
    ~TFTIdleScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

private:

  #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
  uint32_t unrTimer;
  bool rCopy;
  #endif

  bool drawCalled;

  int currentScreen;
  void eraseScreenArea(TFTMenu* menuManager);
  void drawCurrentScreen(TFTMenu* menuManager);
  void drawStatuses(TFTMenu* menuManager);
  void drawSensors(TFTMenu* menuManager);
  void drawHeatSensors(TFTMenu* menuManager);
  void drawHeatSensorsData(TFTMenu* menuManager);
  void drawHeatSensorData(TFTMenu* menuManager, TFTInfoBox* box, Temperature& t);
  void drawFlowData(TFTMenu* menuManager);
  void drawFlowData(TFTMenu* menuManager, TFTInfoBox* box, unsigned long fl);
  unsigned long GetFlowData(uint8_t idx);
  
  void updateCurrentScreen(TFTMenu* menuManager);
  
  void updateSensors(TFTMenu* menuManager);

  void updateSensors2(TFTMenu* menuManager);
  void drawSensors2(TFTMenu* menuManager);
  TFTInfoBoxWithIndex sensors2[6];
  uint16_t sensorsTimer2;  
  
  void updateStatuses(TFTMenu* menuManager);
  void updateHeatSensors(TFTMenu* menuManager);
  void updateFlowData(TFTMenu* menuManager);

  uint32_t uptimeMinutes;
  void drawUptimeMinutes(TFTMenu* menuManager);


  #ifdef USE_WATERFLOW_MODULE
    unsigned long flow1Data, flow2Data;
  #endif

  #ifdef USE_WINDOW_MANAGE_MODULE
    uint32_t windSpeed;
    CompassPoints windDirection;
    bool hasRain;
  #endif

  TFTInfoBox* flow1Box;
  TFTInfoBox* flow2Box;
  TFTInfoBox* windSpeedBox;
  TFTInfoBox* windDirectionBox;
  TFTInfoBox* rainStatusBox;

  TFTInfoBox* heatBox1Sensor;
  TFTInfoBox* heatBox1Air;

  TFTInfoBox* heatBox2Sensor;
  TFTInfoBox* heatBox2Air;

  TFTInfoBox* heatBox3Sensor;
  TFTInfoBox* heatBox3Air;

  #ifdef USE_SMS_MODULE
    uint8_t gsmSignalQuality;
    bool gprsAvailable;
    bool providerNamePrinted;
    int providerNameLength;
    int drawGSMIcons(TFTMenu* menuManager, int curIconRightMargin, bool qualityChanged, bool gprsChanged);
  #endif

  #ifdef USE_WIFI_MODULE
    uint8_t wifiSignalQuality;
    bool connectedToRouter;
    int drawWiFiIcons(TFTMenu* menuManager, int curIconRightMargin, bool connectChanged, bool qualityChanged);
  #endif


  #ifdef USE_LORA_GATE
    int lastLoraRSSI;
    int drawLoRaIcons(TFTMenu* menuManager, int curIconRightMargin, bool rssiChanged, bool drawCaption);
  #endif
  

  #if defined(USE_PRESSURE_MODULE) && defined(DRAW_PRESSURE_ON_SCREEN)
    int drawPressure(TFTMenu* menuManager, int curIconRightMargin);
    int32_t lastPressure;
  #endif

  #ifdef USE_DS3231_REALTIME_CLOCK
  int lastMinute;
  void DrawDateTime(TFTMenu* menuManager);
  #endif

  UTFT_Buttons_Rus* screenButtons;
  IdleScreenFlags flags;

  TFTInfoBoxWithIndex sensors[6];
  uint16_t sensorsTimer;

  void drawStatusesInBox(TFTMenu* menuManager,TFTInfoBox* box, bool status, bool mode, const char* onStatusString, const char* offStatusString, const char* autoModeString, const char* manualModeString, const char* status1Caption=TFT_STATUS_CAPTION, const char* status2Caption = TFT_MODE_CAPTION);
  void drawSensorData(TFTMenu* menuManager,TFTInfoBox* box, int dataIndex, bool forceDraw=false);
  void drawSensorData(TFTMenu* menuManager,TFTInfoBox* box, TFTSpecialSimbol unitChar, bool hasData, bool negative, String& value, const String& fract);

  TFTInfoBox* windowStatusBox;
  void drawWindowStatus(TFTMenu* menuManager);

  int tftSensorsButton, driveButton;

  TFTInfoBox* waterStatusBox;
  void drawWaterStatus(TFTMenu* menuManager);


  TFTInfoBox* lightStatusBox;
  void drawLightStatus(TFTMenu* menuManager);

  TFTInfoBox* heatBox1, *heatBox2, *heatBox3;

  #ifdef USE_HEAT_MODULE
  
  bool heat1Active, heat1On;
  bool heat2Active, heat2On;
  bool heat3Active, heat3On;
  uint32_t heatTimer;

  Temperature heatBox1SensorData, heatBox1AirData;
  Temperature heatBox2SensorData, heatBox2AirData;
  Temperature heatBox3SensorData, heatBox3AirData;
  
  #endif

  
  void drawHeatStatus(TFTMenu* menuManager);
  void drawHeatStatus(TFTMenu* menuManager, uint8_t heatSection, TFTInfoBox* box, bool active, bool on);
  

  TFTInfoBox* phFlowMixBox, *phPlusMinusBox;
  #if defined(USE_PH_MODULE) || defined(USE_EC_MODULE)
    bool phFlowAddOn, phMixOn, phPlusOn, phMinusOn;
  #endif

  void drawPHStatus(TFTMenu* menuManager);
  void drawPHStatus(TFTMenu* menuManager, uint8_t whichSection, TFTInfoBox* box, bool on1, bool on2);


  // USE_WATER_TANK_MODULE
  TFTInfoBox* waterTankStatusBox, *waterTankCommandsBox;
  void drawWaterTankStatus(TFTMenu* menuManager);
  uint8_t wTankErrorType, wTankHasErrors, wTankFillStatus;
  bool wTankIsValveOn;
  uint8_t fillTankButton;
  // USE_WATER_TANK_MODULE


  int prevScreenButton, nextScreenButton;

    
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс экрана информации по окнам
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TEMP_SENSORS

typedef Vector<String*> WindowsChannelsCaptions;

class TFTWindowScreen : public AbstractTFTScreen
{
  public:
  
    TFTWindowScreen();
    ~TFTWindowScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:
      int backButton, percentsButton, openAllButton, closeAllButton, modeButton;
      uint8_t percentsCounter;
      UTFT_Buttons_Rus* screenButtons;

      unsigned long lastWindowsState; // последнее состояние окон
      bool inited;
      bool lastAnyChannelActive;
      bool lastWindowsAutoMode;

      WindowsChannelsCaptions labels;
      Vector<uint8_t> windowsPercents;

};
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_WATERING_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<String*> WaterChannelsCaptions;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWateringScreen : public AbstractTFTScreen
{
  public:
  
    TFTWateringScreen();
    ~TFTWateringScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    private:
      int backButton, skipButton, channelsButton;
      UTFT_Buttons_Rus* screenButtons;

      bool inited;
      uint16_t lastWaterChannelsState; // последнее состояние полива
      bool lastWateringAutoMode;

      WaterChannelsCaptions labels;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWateringChannelsScreen : public AbstractTFTScreen
{
  public:
  
    TFTWateringChannelsScreen();
    ~TFTWateringChannelsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    private:
      int backButton;
      UTFT_Buttons_Rus* screenButtons;

      WaterChannelsCaptions labels;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWateringChannelSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTWateringChannelSettingsScreen();
    ~TFTWateringChannelSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();

    void editChannel(int num);

    private:
      int backButton,saveButton;

      int channelNumber;
      
      UTFT_Buttons_Rus* screenButtons;

      int tickerButton;

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      void blinkSaveSettingsButton(bool bOn);
      void saveSettings();

      TFTInfoBox* hourBox;
      int decHourButton, incHourButton;
      void incHour(int val);

      TFTInfoBox* durationBox;
      int decDurationButton, incDurationButton;
      void incDuration(int val);

      TFTInfoBox* humidityBorderBox;
      int decHumidityBorderButton, incHumidityBorderButton;
      void incHumidityBorder(int val);

      TFTInfoBox* humidityBorderMinBox;
      int decHumidityBorderMinButton, incHumidityBorderMinButton;
      void incHumidityBorderMin(int val);
      

      TFTInfoBox* sensorIndexBox;
      int decSensorIndexButton, incSensorIndexButton;
      void incSensorIndex(int val);

      Vector<int> weekdaysButtons;


      uint8_t wateringDays;
      uint16_t wateringTime;
      uint16_t startWateringTime;
      int8_t wateringSensorIndex;
      uint8_t wateringStopBorder;
      uint8_t wateringStartBorder;

      uint8_t humiditySensorsCount;

      String sensorDataString;
      int sensorDataLeft, sensorDataTop;
      void drawSensorData(TFTMenu* menuManager);
      void getSensorData();

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern TFTWateringChannelSettingsScreen* WateringChannelSettingsScreen;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WATERING_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_LUMINOSITY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTLightScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTLightScreen();
    ~TFTLightScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();

    private:
      int backButton, lightOnButton, lightOffButton, lightModeButton;
      #ifdef USE_LIGHT_MANAGE_MODULE
      int saveButton, onOffButton;
      #endif
      
      UTFT_Buttons_Rus* screenButtons;

      int tickerButton;

      #ifdef USE_LIGHT_MANAGE_MODULE
      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      void blinkSaveSettingsButton(bool bOn);

      void saveSettings();

      TFTInfoBox* hourBox;
      int decHourButton, incHourButton;
      void incHour(int val);

      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);

      TFTInfoBox* durationHourBox;
      int decDurationHourButton, incDurationHourButton;
      void incDurationHour(int val);

      TFTInfoBox* luxBox;
      int decLuxButton, incLuxButton;
      void incLux(int val);

      LightSettings lightSettings;
      #endif // USE_LIGHT_MANAGE_MODULE

      bool inited;
      bool lastLightIsOn; // последнее состояние досветки
      bool lastLightAutoMode;


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_LUMINOSITY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SCENE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  uint16_t sceneNumber;
  char* sceneName;
  bool sceneRunning;
  int buttonId;
  
} SceneButton;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<SceneButton> SceneButtons;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTSceneScreen : public AbstractTFTScreen
{
  public:
  
    TFTSceneScreen();
    ~TFTSceneScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    private:
      int backButton;
      UTFT_Buttons_Rus* screenButtons;

      bool inited;
      void initScenes(TFTMenu* menuManager);
      SceneButtons sceneButtons;
      void updateSceneButtons();


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SCENE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  tftSensorTemperature,
  tftSensorHumidity,
  tftSensorLuminosity,
  tftSensorSoilMoisture,
  tftSensorPH,
  tftSensorCO2,
  tftSensorSystemTemperature,
  tftSensorEC,
  tftSensorEC_PH,
  
} TFTSensorType;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  TFTSensorType type;
  uint8_t sensorIndex;
  
} TFTSensorData;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<TFTSensorData> TFTSensorsList;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTSensorsScreen : public AbstractTFTScreen, public KeyboardInputTarget
{
  public:
  
    TFTSensorsScreen();
    ~TFTSensorsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onKeyboardInputResult(const String& input, bool okPressed);

    private:
      int backButton, backwardButton, forwardButton, optionsButton, enterKeyButton;
      UTFT_Buttons_Rus* screenButtons;

#if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
    int windowsSettingsButton;
#endif       

#ifdef USE_HEAT_MODULE
  int heatSettingsButton;
#endif

#ifdef USE_SCENE_MODULE
  int sceneButton;
#endif

#ifdef USE_SHADOW_MODULE
  int shadowButton;
#endif

      int offset;
      TFTSensorsList sensors;
      void initSensors();

      void drawListFrame(TFTMenu* menuManager);
      void drawSensors(TFTMenu* menuManager, bool isBGCleared, bool onlyData=false);
      void drawSensor(TFTMenu* menuManager,uint8_t row, TFTSensorData* data, bool isBGCleared, bool onlyData=false);
      


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_WINDOW_MANAGE_MODULE) && defined(USE_TEMP_SENSORS)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<String*> WMWindowsChannelsCaptions;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWChannelsScreen : public AbstractTFTScreen
{
  public:
  
    TFTWChannelsScreen();
    ~TFTWChannelsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      WMWindowsChannelsCaptions labels;
      void drawLabels(TFTMenu* menuManager);

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);
      void saveSettings();

      WindowsChannelsBinding binding;
      void markButton(int button, uint8_t bindTo, bool redraw);

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTOrientationScreen : public AbstractTFTScreen
{
  public:
  
    TFTOrientationScreen();
    ~TFTOrientationScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      WMWindowsChannelsCaptions labels;
      void drawLabels(TFTMenu* menuManager);

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);
      void saveSettings();

      WindowsChannelsBinding binding;
      void markButton(int button, uint8_t bindTo, bool redraw);

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTRainScreen : public AbstractTFTScreen
{
  public:
  
    TFTRainScreen();
    ~TFTRainScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      WMWindowsChannelsCaptions labels;
      void drawLabels(TFTMenu* menuManager);

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);
      void saveSettings();

      WindowsChannelsBinding binding;
      void markButton(int button, uint8_t bindTo, bool redraw);

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWMScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTWMScreen(uint8_t channel);
    ~TFTWMScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
  
    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* percents25Box;
      int16_t percents25Temp;
      int dec25PercentsButton, inc25PercentsButton;
      void inc25Temp(int val);
      
      TFTInfoBox* percents50Box;
      int16_t percents50Temp;
      int dec50PercentsButton, inc50PercentsButton;
      void inc50Temp(int val);

      TFTInfoBox* percents75Box;
      int16_t percents75Temp;
      int dec75PercentsButton, inc75PercentsButton;
      void inc75Temp(int val);

      TFTInfoBox* percents100Box;
      int16_t percents100Temp;
      int dec100PercentsButton, inc100PercentsButton;
      void inc100Temp(int val);

      TFTInfoBox* histeresisBox;
      int16_t histeresis;
      int decHisteresisButton, incHisteresisButton;
      String formatHisteresis();

      TFTInfoBox* sensorBox;
      int16_t sensor;
      int decSensorButton, incSensorButton;

      bool isActive;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;
      void drawSensorData(TFTMenu* menuManager);
      void getSensorData();

      uint16_t tempSensorsCount, humiditySensorsCount;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWindScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTWindScreen();
    ~TFTWindScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
    
    private:

      int tickerButton;
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* windSpeedBox;
      int16_t windSpeed;
      int decWindSpeedButton, incWindSpeedButton;
      void incWindSpeed(int val);
      
      TFTInfoBox* hurricaneSpeedBox;
      int16_t hurricaneSpeed;
      int decHurricaneSpeedButton, incHurricaneSpeedButton;
      void incHurricaneSpeed(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTWMenuScreen : public AbstractTFTScreen
{
  public:
  
    TFTWMenuScreen();
    ~TFTWMenuScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:

      int backButton;
      int windowsButton, rainButton, orientationButton, windButton;
      int temp1Button, temp2Button, temp3Button, temp4Button;
  
      UTFT_Buttons_Rus* screenButtons;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_WINDOW_MANAGE_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_SHADOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTShadowScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTShadowScreen();
    ~TFTShadowScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
    

    private:

      int tickerButton;

      int backButton, saveButton;
      int shadow1Button, shadow2Button, shadow3Button;
  
      UTFT_Buttons_Rus* screenButtons;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);   

      void incWorkTime1(int val);
      void incWorkTime2(int val);
      void incWorkTime3(int val);

      TFTInfoBox* workTimeBox1;
      int decWorkTimeButton1, incWorkTimeButton1;
      int16_t shadowDriveWorkTime1;

      TFTInfoBox* workTimeBox2;
      int decWorkTimeButton2, incWorkTimeButton2;
      int16_t shadowDriveWorkTime2;

      TFTInfoBox* workTimeBox3;
      int decWorkTimeButton3, incWorkTimeButton3;
      int16_t shadowDriveWorkTime3;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTShadowSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTShadowSettingsScreen(uint8_t channel);
    ~TFTShadowSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* luxBox;
      int decLuxButton, incLuxButton;
      void incLux(int val);

      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);

      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;
      
      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(uint8_t which, String& result);

      uint16_t lumSensorsCount;

      ShadowSettings shadowSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTShadowDriveScreen : public AbstractTFTScreen
{
  public:
  
    TFTShadowDriveScreen();
    ~TFTShadowDriveScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    void onActivate(TFTMenu* menuManager);

    private:
      int backButton;

      int shadow1OnButton, shadow1OffButton, shadow1ModeButton;
      int shadow2OnButton, shadow2OffButton, shadow2ModeButton;
      int shadow3OnButton, shadow3OffButton, shadow3ModeButton;

      bool shadow1On, shadow2On, shadow3On;
      bool shadow1Active, shadow2Active, shadow3Active;
      bool shadowMode1, shadowMode2, shadowMode3;
      
      bool drawCalled;
                  
      UTFT_Buttons_Rus* screenButtons;


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_SHADOW_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTVentScreen : public AbstractTFTScreen
{
  public:
  
    TFTVentScreen();
    ~TFTVentScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:

      int backButton;
      int channel1Button, channel2Button, channel3Button;

      void updateVentButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw=false);
      int channel1OnOffButton, channel2OnOffButton, channel3OnOffButton;
      int channel1AutoManualButton, channel2AutoManualButton, channel3AutoManualButton;
  
      UTFT_Buttons_Rus* screenButtons;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTVentSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTVentSettingsScreen(uint8_t channel);
    ~TFTVentSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка температуры
      TFTInfoBox* tempBox;
      int decTempButton, incTempButton;
      void incTemp(int val);

      // гистерезис
      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);
      String formatHisteresis();

      // датчик температуры
      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      // минимальный выбег, секунд  
      TFTInfoBox* minWorkTimeBox;
      int decMinWorkTimeButton, incMinWorkTimeButton;
      void incMinWorkTime(int val);

      // максимальный выбег, секунд  
      TFTInfoBox* maxWorkTimeBox;
      int decMaxWorkTimeButton, incMaxWorkTimeButton;
      void incMaxWorkTime(int val);

      // время отдыха, секунд  
      TFTInfoBox* restTimeBox;
      int decRestTimeButton, incRestTimeButton;
      void incRestTime(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(String& result);

      uint16_t tempSensorsCount, humiditySensorsCount;

      VentSettings ventSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HUMIDITY_SPRAY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTSprayScreen : public AbstractTFTScreen
{
  public:
  
    TFTSprayScreen();
    ~TFTSprayScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:

      int backButton;
      int channel1Button, channel2Button, channel3Button;

      void updateSprayButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw=false);
      int channel1OnOffButton, channel2OnOffButton, channel3OnOffButton;
      int channel1AutoManualButton, channel2AutoManualButton, channel3AutoManualButton;
  
      UTFT_Buttons_Rus* screenButtons;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTSpraySettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTSpraySettingsScreen(uint8_t channel);
    ~TFTSpraySettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка влажности
      TFTInfoBox* sprayOnBox;
      int decSprayOnButton, incSprayOnButton;
      void incSprayOn(int val);

      TFTInfoBox* sprayOffBox;
      int decSprayOffButton, incSprayOffButton;
      void incSprayOff(int val);

      // гистерезис
      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);
      String formatHisteresis();

      // датчик влажности
      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      TFTInfoBox* startTimeBox;
      int decStartTimeButton, incStartTimeButton;
      void incStartTime(int val);

      TFTInfoBox* endTimeBox;
      int decEndTimeButton, incEndTimeButton;
      void incEndTime(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(String& result);

      uint16_t humiditySensorsCount;

      HumiditySpraySettings spraySettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HUMIDITY_SPRAY_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if defined(USE_PH_MODULE) || defined(USE_EC_MODULE)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTPHControlScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
    TFTPHControlScreen();
    ~TFTPHControlScreen();

    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    
    

  private:

    uint16_t phValue; // значение pH, за которым следим
    uint16_t histeresis; // гистерезис

    uint16_t mixWorkTime; // время работы насоса перемешивания, с
    uint16_t reagentWorkTime; // время работы подачи реагента, с

    int tickerButton;
    int backButton, saveButton, phSettingsButton;
    UTFT_Buttons_Rus* screenButtons;

    // гистерезис
    TFTInfoBox* histeresisBox;
    int decHisteresisButton, incHisteresisButton;
    void incHisteresis(int val);
    String formatHisteresis();

    // уставка поддерживаемого значения pH
    TFTInfoBox* phBox;
    int decPHButton, incPHButton;
    void incPH(int val);
    String formatPH();

    
    // время работы насоса перемешивания, секунд
    TFTInfoBox* mixWorkTimeBox;
    int incMixWorkTimeButton, decMixWorkTimeButton;
    void incMixWorkTime(int val);

    // время работы насоса подачи реагента, секунд
    TFTInfoBox* reagentWorkTimeBox;
    int incReagentWorkTimeButton, decReagentWorkTimeButton;
    void incReagentWorkTime(int val);


    void saveSettings();

    uint32_t blinkTimer;
    bool blinkOn, blinkActive;
    
    void blinkSaveSettingsButton(bool bOn);
    
       
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTPHSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTPHSettingsScreen();
    ~TFTPHSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      int tickerButton;
    
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка температуры калибровки
      TFTInfoBox* tempBox;
      int decTempButton, incTempButton;
      void incTemp(int val);

      // поправочное число
      TFTInfoBox* calibrationBox;
      int decCalibrationButton, incCalibrationButton;
      void incCalibration(int val);

      // датчик температуры
      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      // показания в милливольтах для тестового раствора 4 pH
      TFTInfoBox* ph4VoltageBox;
      int decPh4VoltageBoxButton, incPh4VoltageBoxButton;
      void incPh4VoltageBox(int val);

      // показания в милливольтах для тестового раствора 7 pH
      TFTInfoBox* ph7VoltageBox;
      int decPh7VoltageBoxButton, incPh7VoltageBoxButton;
      void incPh7VoltageBox(int val);

      // показания в милливольтах для тестового раствора 10 pH
      TFTInfoBox* ph10VoltageBox;
      int decPh10VoltageBoxButton, incPh10VoltageBoxButton;
      void incPh10VoltageBox(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(String& result);

      uint16_t tempSensorsCount, humiditySensorsCount;

    int calibration; // калибровка, в сотых долях
    int16_t ph4Voltage; // показания в милливольтах для тестового раствора 4 pH
    int16_t ph7Voltage; // показания в милливольтах для тестового раствора 7 pH
    int16_t ph10Voltage; // показания в милливольтах для тестового раствора 10 pH
    int8_t phTemperatureSensorIndex; // индекс датчика температуры, который завязан на измерения pH
    int phSamplesTemperature; // температура, при которой производилась калибровка


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_PH_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_THERMOSTAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTThermostatScreen : public AbstractTFTScreen
{
  public:
  
    TFTThermostatScreen();
    ~TFTThermostatScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:

      int backButton;
      int channel1Button, channel2Button, channel3Button;

      void updateThermostatButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw=false);
      int channel1OnOffButton, channel2OnOffButton, channel3OnOffButton;
      int channel1AutoManualButton, channel2AutoManualButton, channel3AutoManualButton;
  
      UTFT_Buttons_Rus* screenButtons;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTThermostatSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTThermostatSettingsScreen(uint8_t channel);
    ~TFTThermostatSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка температуры
      TFTInfoBox* tempBox;
      int decTempButton, incTempButton;
      void incTemp(int val);

      // гистерезис
      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      void incHisteresis(int val);
      String formatHisteresis();

      // датчик температуры
      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;    

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(String& result);

      uint16_t tempSensorsCount, humiditySensorsCount;

      ThermostatSettings thermostatSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_THERMOSTAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_CYCLE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTCycleVentScreen : public AbstractTFTScreen
{
  public:
  
    TFTCycleVentScreen();
    ~TFTCycleVentScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    private:

      int backButton;
      int channel1Button, channel2Button, channel3Button;
      int channel1OnOffButton, channel2OnOffButton, channel3OnOffButton;
      int channel1AutoManualButton, channel2AutoManualButton, channel3AutoManualButton;

      void updateVentButtons(uint8_t channel, int btnOnOff, int btnMode, bool forceRedraw=false);
  
      UTFT_Buttons_Rus* screenButtons;
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTCycleVentSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTCycleVentSettingsScreen(uint8_t channel);
    ~TFTCycleVentSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      Vector<int> weekdaysButtons;

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* startTimeBox;
      int decStartTimeButton, incStartTimeButton;
      void incStartTime(int val);

      TFTInfoBox* endTimeBox;
      int decEndTimeButton, incEndTimeButton;
      void incEndTime(int val);

      TFTInfoBox* workTimeBox;
      int decWorkTimeButton, incWorkTimeButton;
      void incWorkTime(int val);

      TFTInfoBox* idleTimeBox;
      int decIdleTimeButton, incIdleTimeButton;
      void incIdleTime(int val);

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      CycleVentSettings ventSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_CYCLE_VENT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_HEAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTHeatScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTHeatScreen();
    ~TFTHeatScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);

    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
    

    private:

      int tickerButton;

      int backButton, saveButton;
      int heat1Button, heat2Button, heat3Button;
  
      UTFT_Buttons_Rus* screenButtons;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);   

      void incWorkTime1(int val);
      void incWorkTime2(int val);
      void incWorkTime3(int val);

      TFTInfoBox* workTimeBox1;
      int decWorkTimeButton1, incWorkTimeButton1;
      int16_t heatDriveWorkTime1;

      TFTInfoBox* workTimeBox2;
      int decWorkTimeButton2, incWorkTimeButton2;
      int16_t heatDriveWorkTime2;

      TFTInfoBox* workTimeBox3;
      int decWorkTimeButton3, incWorkTimeButton3;
      int16_t heatDriveWorkTime3;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTHeatDriveScreen : public AbstractTFTScreen
{
  public:
  
    TFTHeatDriveScreen();
    ~TFTHeatDriveScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    void onActivate(TFTMenu* menuManager);

    private:
      int backButton;

      int heat1OnButton, heat1OffButton, heat1ModeButton;
      int heat2OnButton, heat2OffButton, heat2ModeButton;
      int heat3OnButton, heat3OffButton, heat3ModeButton;

      bool heat1On, heat2On, heat3On;
      bool heat1Active, heat2Active, heat3Active;
      bool heatMode1, heatMode2, heatMode3;
      
      bool drawCalled;
                  
      UTFT_Buttons_Rus* screenButtons;


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTHeatSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTHeatSettingsScreen(uint8_t channel);
    ~TFTHeatSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      uint8_t channel;
      int tickerButton;
    
      int backButton, saveButton, onOffButton;
      UTFT_Buttons_Rus* screenButtons;

      TFTInfoBox* minTempBox;
      int decMinTempButton, incMinTempButton;
      void incMinTemp(int val);
      
      TFTInfoBox* maxTempBox;
      int decMaxTempButton, incMaxTempButton;
      void incMaxTemp(int val);

      TFTInfoBox* ethalonTempBox;
      int decEthalonTempButton, incEthalonTempButton;
      void incEthalonTemp(int val);

      TFTInfoBox* airSensorBox;
      int decAirSensorButton, incAirSensorButton;

      TFTInfoBox* histeresisBox;
      int decHisteresisButton, incHisteresisButton;
      String formatHisteresis();

      TFTInfoBox* sensorBox;
      int decSensorButton, incSensorButton;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

      int sensorDataLeft, sensorDataTop;
      String sensorDataString;

      int sensorDataLeft2, sensorDataTop2;
      String sensorDataString2;
      
      void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
      void getSensorData(uint8_t which, String& result);

      uint16_t tempSensorsCount, humiditySensorsCount;

      HeatSettings heatSettings;

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_HEAT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_EC_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTECControlScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
    TFTECControlScreen();
    ~TFTECControlScreen();

    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    
    

  private:

  ECSettings ecSettings;

    int tickerButton;
    int backButton, saveButton, ecSettingsButton;
    UTFT_Buttons_Rus* screenButtons;

    // значение ppm
    TFTInfoBox* ppmBox;
    int decPPMButton, incPPMButton;
    void incPPM(int val);
    
    // гистерезис
    TFTInfoBox* histeresisBox;
    int decHisteresisButton, incHisteresisButton;
    void incHisteresis(int val);

    // ннтервал
    TFTInfoBox* intervalBox;
    int decIntervalButton, incIntervalButton;
    void incInterval(int val);
    
    // датчик EC
    TFTInfoBox* sensorBox;
    int decSensorButton, incSensorButton;

    int sensorDataLeft, sensorDataTop;
    String sensorDataString;
    int ecSensorsCount;

    void drawSensorData(TFTMenu* menuManager, String& which, int left, int top);
    void getSensorData(String& result);
  
    // интервал подачи
    TFTInfoBox* workIntervalBox;
    int decWorkIntervalButton, incWorkIntervalButton;
    void incWorkInterval(int val);

    // время подачи
    TFTInfoBox* workTimeBox;
    int decWorkTimeButton, incWorkTimeButton;
    void incWorkTime(int val);

    void saveSettings();

    uint32_t blinkTimer;
    bool blinkOn, blinkActive;
    
    void blinkSaveSettingsButton(bool bOn);
    
       
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTECSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTECSettingsScreen();
    ~TFTECSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();    

    private:

      int tickerButton;
    
      int backButton, saveButton;
      UTFT_Buttons_Rus* screenButtons;

      // уставка температуры калибровки
      TFTInfoBox* calibrationBox;
      int decCalibrationButton, incCalibrationButton;
      void incCalibration(int val);

      // уставка подачи реагента A
      TFTInfoBox* agentABox;
      int decAgentAButton, incAgentAButton;
      void incAgentA(int val);

      // уставка подачи реагента B
      TFTInfoBox* agentBBox;
      int decAgentBButton, incAgentBButton;
      void incAgentB(int val);

      // уставка подачи реагента C
      TFTInfoBox* agentCBox;
      int decAgentCButton, incAgentCButton;
      void incAgentC(int val);

      // уставка времени подачи воды
      TFTInfoBox* waterBox;
      int decWaterButton, incWaterButton;
      void incWater(int val);

      // уставка времени перемешивания
      TFTInfoBox* mixBox;
      int decMixButton, incMixButton;
      void incMix(int val);
    
    ECSettings ecSettings;

      void saveSettings();

      uint32_t blinkTimer;
      bool blinkOn, blinkActive;
      
      void blinkSaveSettingsButton(bool bOn);

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_EC_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTUnrKScreen : public AbstractTFTScreen
{
  public:
  
    TFTUnrKScreen();
    ~TFTUnrKScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    private:
};

//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // #if (TARGET_BOARD == DUE_BOARD) && defined(PROTECT_ENABLED)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTTimeSyncScreen : public AbstractTFTScreen, public ITickHandler
{
public:
    TFTTimeSyncScreen();
    ~TFTTimeSyncScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();
    
private:

  UTFT_Buttons_Rus* screenButtons;

  int backButton;
  int gsmActiveButton;
  int wifiActiveButton;
  int saveButton;
  int tickerButton;

  uint32_t blinkTimer;
  bool blinkOn, blinkActive;
  void blinkSaveSettingsButton(bool bOn);

  void saveSettings();

  TFTInfoBox* gsmIntervalBox;
  int decGsmIntervalButton, incGsmIntervalButton;
  void incGsmInterval(int val);

  TFTInfoBox* wifiIntervalBox;
  int decWifiIntervalButton, incWifiIntervalButton;
  void incWifiInterval(int val);

  TFTInfoBox* timezoneBox;
  int decTimezoneButton, incTimezoneButton;
  void incTimezone(int val);

  bool gsmActive, wifiActive;
  uint8_t wifiInterval, gsmInterval;
  int16_t ntpTimezone;

  int gsmBtnLeft, wifiBtnLeft, controlButtonsTop;
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTSettingsScreen : public AbstractTFTScreen, public ITickHandler
{
  public:
  
    TFTSettingsScreen();
    ~TFTSettingsScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);
    void onActivate(TFTMenu* menuManager);
    
    void onButtonPressed(TFTMenu* menuManager,int buttonID);
    void onButtonReleased(TFTMenu* menuManager,int buttonID);
    void onTick();

    private:
      int backButton, timeSyncButton, decOpenTempButton, incOpenTempButton, decCloseTempButton, incCloseTempButton, incIntervalButton, decIntervalButton, incSealevelButton, decSealevelButton;
      UTFT_Buttons_Rus* screenButtons;


      int tickerButton;

      void incInterval(int val);
      void incOpenTemp(int val);
      void incCloseTemp(int val);
      void incSealevel(int val);

      bool inited;

      uint8_t openTemp, closeTemp;
      uint16_t interval;
      int16_t sealevel;

      TFTInfoBox* closeTempBox;
      TFTInfoBox* openTempBox;
      TFTInfoBox* intervalBox;
      TFTInfoBox* sealevelBox;

      #ifdef USE_DS3231_REALTIME_CLOCK
      uint32_t controllerTimeChangedTimer;

      int decTimePartButton, incTimePartButton, dayButton, monthButton, yearButton, hourButton, minuteButton;
      int selectedTimePartButton;

      void incDateTimePart(int btn);
      
      String strDay, strMonth, strYear, strHour, strMinute, strSecond;

      RTCTime controllerTime;
      bool controllerTimeChanged;

      void updateTimeButtons(RTCTime& tm, bool redraw);      
      
      uint16_t stepVal(int8_t dir, uint16_t minVal,uint16_t maxVal, int16_t val);
      String addLeadingZero(int val);
      #endif


};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTMemInitScreen : public AbstractTFTScreen
{
  public:
  
    TFTMemInitScreen();
    ~TFTMemInitScreen();
    
    void setup(TFTMenu* menuManager);
    void update(TFTMenu* menuManager);
    void draw(TFTMenu* menuManager);

    private:

};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  const char* screenName;
  AbstractTFTScreen* screen;
  
} TFTScreenInfo;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<TFTScreenInfo> TFTScreensList; // список экранов
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс-менеджер работы с TFT
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
  bool isLCDOn : 1;
  byte pad : 7;
  
} TFTMenuFlags;
#pragma pack(pop)
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  mbShow,
  mbConfirm,
  mbHalt
  
} MessageBoxType;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class MessageBoxScreen : public AbstractTFTScreen
{
  public:

  static AbstractTFTScreen* create();

  void confirm(const char* caption, Vector<const char*>& lines, AbstractTFTScreen* okTarget, AbstractTFTScreen* cancelTarget, MessageBoxResultSubscriber* sub=NULL);
  void show(const char* caption, Vector<const char*>& lines, AbstractTFTScreen* okTarget, MessageBoxResultSubscriber* sub=NULL);
  
  void halt(const char* caption, Vector<const char*>& lines, bool errorColors=true, bool haltInWhile=false);
    
protected:

    virtual void setup(TFTMenu* dc);
    virtual void update(TFTMenu* dc);
    virtual void draw(TFTMenu* dc);

private:
      MessageBoxScreen();

      AbstractTFTScreen* targetOkScreen;
      AbstractTFTScreen* targetCancelScreen;
      Vector<const char*> lines;

      const char* caption;

      MessageBoxResultSubscriber* resultSubscriber;
      bool errorColors, haltInWhile;

      UTFT_Buttons_Rus* buttons;
      int yesButton, noButton;

      MessageBoxType boxType;

      void recreateButtons();
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern MessageBoxScreen* MessageBox;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  ktFull, // полная клавиатура
  ktNumbers, // ввод только чисел и точки
  
} KeyboardType;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#define KBD_BUTTONS_IN_ROW  10
#define KBD_BUTTON_WIDTH  70
#define KBD_BUTTON_HEIGHT  50
#define KBD_SPACING  9
 #define KDB_BIG_BUTTON_HEIGHT 100
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class KeyboardScreen : public AbstractTFTScreen
{
  public:

  static AbstractTFTScreen* create();

  void show(const String& val, int maxLen, KeyboardInputTarget* inputTarget, KeyboardType keyboardType=ktFull, bool eng=false);

  ~KeyboardScreen();  
protected:

    virtual void setup(TFTMenu* dc);
    virtual void update(TFTMenu* dc);
    virtual void draw(TFTMenu* dc);

private:
      KeyboardScreen();

      KeyboardInputTarget* inputTarget;
      String inputVal;
      int maxLen;
      bool isRusInput;

      UTFT_Buttons_Rus* buttons;
      Vector<String*> captions;
      Vector<String*> altCaptions;

      int backspaceButton, okButton, cancelButton, switchButton, spaceButton;

      void createKeyboard(TFTMenu* dc);
      void drawValue(TFTMenu* dc, bool deleteCharAtCursor=false);
      void switchInput(bool redraw);

      void applyType(KeyboardType keyboardType);

      int cursorPos;
      void redrawCursor(TFTMenu* dc, bool erase);

      char utf8Bytes[7];
  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern KeyboardScreen* Keyboard;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class TFTMenu
{

public:
  TFTMenu();

  void setup();
  void update();

  void switchToScreen(const char* screenName);
  void switchToScreen(AbstractTFTScreen* to);
  
  UTFT* getDC() { return tftDC; };
  URTouch* getTouch() { return tftTouch; };
  UTFTRus* getRusPrinter() { return &rusPrint; };
 
  void resetIdleTimer();

  void onButtonPressed(int button);
  void onButtonReleased(int button);

private:

  TFTScreensList screens;
  UTFT* tftDC;
  URTouch* tftTouch;
  UTFTRus rusPrint;

  int currentScreenIndex;
  
  AbstractTFTScreen* switchTo;
  int switchToIndex;

  unsigned long idleTimer;
  
  TFTMenuFlags flags;

  void lcdOn();
  void lcdOff();
  void switchBacklight(uint8_t level);

  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern TFTMenu* TFTScreen;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TFT_MODULE

