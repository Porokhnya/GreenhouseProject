#pragma once
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
#include "Globals.h"

#ifdef USE_TFT_MODULE
#if TARGET_BOARD == STM32_BOARD
#include <UTFTSTM32.h>
#else
#include <UTFT.h>
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// класс поддержки русских шрифтов для UTFT
// поддерживает кастомный шрифт с единицами измерения SensorFont.c, для вывода значков используется printSpecialChar.
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  charUnknown,

  // SensorFont
  charMinus,
  charPlus,
  charPercent,
  charDegree,
  charLux,
  charDot,
  charComma,
  charRightArrow,
  charLeftArrow,
  charWindSpeed,

  // SensorFont2
  charLitres,
  charM3,
  charEast,
  charWest,
  charNorth,
  charSouth,
  charCO2,
  
} TFTSpecialSimbol;
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
class UTFTRus
{
  public:
    UTFTRus();
    void init(UTFT* uTft);

    int utf8_strlen(const char* str) { return print(str,0,0,0,true); }
    int print(const char* str,int x, int y, int deg=0, bool computeStringLengthOnly=false);
    void printSpecialChar(TFTSpecialSimbol ch, int x, int y, int deg=0);
    char mapChar(TFTSpecialSimbol ch);
    
  private:
    UTFT* pDisplay;


  
};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
extern uint8_t BigRusFont[];
extern uint8_t SmallRusFont[];
extern uint8_t SevenSegNumFontMDS[];
extern uint8_t SevenSegNumFontPlus[];
extern uint8_t SensorFont[];
extern uint8_t SensorFont2[];
extern uint8_t Various_Symbols_32x32[];
extern uint8_t IconsFont[];
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
