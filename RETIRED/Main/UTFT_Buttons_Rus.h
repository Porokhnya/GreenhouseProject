/*
  UTFT_Buttons.h - Add-on Library for UTFT: Buttons
  Copyright (C)2016 Rinky-Dink Electronics, Henning Karlsen. All right reserved
  
  This library adds simple but easy to use buttons to extend the use
  of the UTFT and URTouch libraries.

  You can find the latest version of the library at 
  http://www.RinkyDinkElectronics.com/

  This library is free software; you can redistribute it and/or
  modify it under the terms of the CC BY-NC-SA 3.0 license.
  Please see the included documents for further information.

  Commercial use of this library requires you to buy a license that
  will allow commercial use. This includes using the library,
  modified or not, as a tool to sell products.

  The license applies to all part of the library including the 
  examples and tools supplied with the library.
*/

#pragma once
#include "Globals.h"

#ifdef USE_TFT_MODULE

#define UTFT_BUTTONS_VERSION	103

#if defined(__AVR__)
	#include "Arduino.h"
#elif defined(__PIC32MX__)
	#include "WProgram.h"
#elif defined(__arm__)
	#include "Arduino.h"
#endif

#if TARGET_BOARD == STM32_BOARD
#include <UTFTSTM32.h>
#else
#include <UTFT.h>
#endif

#include <URTouch.h>
#if ((!defined(UTFT_VERSION)) || (UTFT_VERSION<241))
	#error : You will need UTFT v2.41 or higher to use this add-on library...
#endif

#include "UTFTRus.h"

#define MAX_BUTTONS	20	// Maximum number of buttons available at one time

// Define presets for button status
#define BUTTON_DISABLED			0x0001
#define BUTTON_SYMBOL			0x0002
#define BUTTON_SYMBOL_REP_3X	0x0004
#define BUTTON_BITMAP			0x0008	
#define BUTTON_NO_BORDER		0x0010
#define BUTTON_UNUSED			0x8000
#define BUTTON_HAS_BACK_COLOR 0x0020
#define BUTTON_HAS_FONT_COLOR 0x0040


typedef struct
{
  int16_t			pos_x, pos_y, width, height;
  word backColor, fontColor;
  uint16_t			flags;
  const char				*label;
  bitmapdatatype	data;
} button_type;

typedef void (*DrawButtonsUpdateFunc)(void);
typedef void (*OnCheckButtonsFunc)(int button);

class UTFT_Buttons_Rus
{
	public:
		UTFT_Buttons_Rus(UTFT *ptrUTFT, URTouch *ptrURTouch, UTFTRus* pTFTRus);

		int		addButton(uint16_t x, uint16_t y, uint16_t width, uint16_t height, const char *label, uint16_t flags=0);
		int		addButton(uint16_t x, uint16_t y, uint16_t width, uint16_t height, bitmapdatatype data, uint16_t flags=0);
		void	drawButtons(DrawButtonsUpdateFunc func=NULL);
		void	drawButton(int buttonID);
		void	enableButton(int buttonID, boolean redraw=false);
		void	disableButton(int buttonID, boolean redraw=false);
		void	relabelButton(int buttonID, const char *label, boolean redraw=false);
		boolean	buttonEnabled(int buttonID);
		void	deleteButton(int buttonID);
		void	deleteAllButtons();
		int		checkButtons(OnCheckButtonsFunc func = NULL);
		void	setTextFont(uint8_t* font);
		void	setSymbolFont(uint8_t* font);
		void	setButtonColors(word atxt, word iatxt, word brd, word brdhi, word back);
    void setButtonBackColor(int buttonID, word color);
    void setButtonFontColor(int buttonID, word color);

	protected:
		UTFT		*_UTFT;
		URTouch		*_URTouch;

    UTFTRus* pRusPrinter;
    
		button_type	buttons[MAX_BUTTONS];
		word		_color_text, _color_text_inactive, _color_background, _color_border, _color_hilite;
		uint8_t		*_font_text, *_font_symbol;
};

#endif
