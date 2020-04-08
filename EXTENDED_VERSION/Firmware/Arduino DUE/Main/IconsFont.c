// UTFT font IconsFont size: 32x32
//символы для иконок:
// 0 - фрамуги
// 1 - опции
// 2 - расходомер
// 3 - шторы
// 4 - капля (полив)
// 5 - солнце (досветка)
// 6 - часы
// 7 - батарея (обогрев)
// 8 - управление
// 9 - сценарии 
// : - дождь
// ; - ясная погода (солнечно)
// < - рециркуляция
// = - внешняя вентиляция
// > - термостат
// ? - модуль рН
// @ - бак для воды для полива
// eng 'A' - розетка (управление внешним устройством)   
#if defined(__AVR__)
	#include <avr/pgmspace.h>
	#define fontdatatype const uint8_t
#elif defined(__PIC32MX__)
	#define PROGMEM
	#define fontdatatype const unsigned char
#elif defined(__arm__)
	#define PROGMEM
	#define fontdatatype const unsigned char
#endif


fontdatatype IconsFont[2308] PROGMEM = {0x20,0x20,0x30,0x12,	// size 32x32, first 48, count 18
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
0x00,0x7F,0xFF,0xF8,0x00,0x7F,0xFF,0xF8,0x00,0x7F,0xFF,0xF8,0x00,0x7F,0xFF,0xF8,
0x00,0x60,0x00,0x18,0x00,0x60,0x00,0x18,0x00,0x60,0x00,0x18,0x00,0x60,0x00,0x18,
0x1F,0xFF,0xFE,0x18,0x1F,0xFF,0xFE,0x18,0x1F,0xFF,0xFE,0x18,0x1F,0xFF,0xFE,0x18,
0x18,0x00,0x06,0x18,0x18,0x00,0x07,0xF8,0x18,0x00,0x07,0xF8,0x18,0x00,0x06,0x00,
0x18,0x00,0x06,0x00,0x18,0x00,0x06,0x00,0x18,0x00,0x06,0x00,0x18,0x00,0x06,0x00,
0x18,0x00,0x06,0x00,0x1F,0xFF,0xFE,0x00,0x1F,0xFF,0xFE,0x00,0x00,0x00,0x00,0x00,
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// digit '0' windows
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x1E,0x00,0x00,0x00,0x3F,0x00,0x00,
0x00,0x7F,0x83,0xC0,0x01,0xFC,0x07,0x80,0x03,0xF8,0x0F,0x00,0x07,0xE0,0x0E,0x00,
0x07,0xF0,0x0F,0x08,0x1F,0xF8,0x0F,0x98,0x3F,0xFC,0x0F,0xF8,0x7E,0x7E,0x1F,0xF0,
0x3E,0x3F,0x3F,0xE0,0x1E,0x1F,0x7E,0x00,0x04,0x0E,0xFC,0x00,0x00,0x05,0xF8,0x00,
0x00,0x03,0xF0,0x00,0x00,0x07,0xE8,0x00,0x00,0x0F,0xDC,0x00,0x00,0x1F,0xBE,0x00,
0x01,0xFF,0x7F,0x00,0x03,0xFE,0x3F,0x80,0x07,0xFE,0x1F,0xC0,0x06,0x7C,0x0F,0xE0,
0x04,0x1C,0x07,0xF0,0x00,0x1C,0x03,0xF8,0x00,0x1C,0x01,0xF0,0x00,0x38,0x00,0xE0,
0x00,0x70,0x00,0x40,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// digit '1' options
0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x00,0x00,0x7F,0xFC,0x00,0x00,0xFF,0xFF,0x00,
0x03,0xF0,0x0F,0xC0,0x07,0xC0,0x03,0xE0,0x0F,0x00,0x00,0xF0,0x1E,0x00,0x00,0x78,
0x1C,0x00,0x00,0x38,0x38,0x00,0x00,0x1C,0x38,0x00,0x00,0x1C,0x71,0x80,0x00,0x0E,
0x70,0xC0,0x00,0x0E,0x60,0x70,0x00,0x06,0xE0,0x3C,0x00,0x07,0xE0,0x1E,0x00,0x07,
0xE0,0x0F,0x80,0x07,0xE0,0x07,0xC0,0x07,0xE0,0x03,0xC0,0x07,0xE0,0x01,0x80,0x07,
0x60,0x00,0x00,0x06,0x70,0x18,0xC2,0x0E,0x70,0x25,0x26,0x0E,0x38,0x25,0x2A,0x1C,
0x38,0x25,0x22,0x1C,0x1C,0x25,0x22,0x38,0x1E,0x18,0xC2,0x78,0x0F,0x00,0x00,0xF0,
0x07,0xC0,0x03,0xE0,0x03,0xF0,0x0F,0xC0,0x00,0xFF,0xFF,0x00,0x00,0x3F,0xFC,0x00,	// digit '2' water gauge
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x00,0x01,0x30,0x38,0x00,0x03,0x70,0x70,
0x00,0x03,0x70,0xE0,0x01,0xC7,0x71,0xC0,0x01,0xF7,0x73,0x81,0x01,0xF8,0x77,0x03,
0x01,0xE0,0x7E,0x07,0x01,0xC0,0x7C,0x0E,0x01,0x80,0x78,0x1C,0x03,0x00,0x78,0x38,
0x1F,0x00,0x7C,0x70,0x7E,0x00,0x7E,0x20,0x3E,0x00,0x77,0x00,0x0E,0x00,0x73,0x80,
0x06,0x00,0x71,0xC0,0x02,0x00,0x76,0xE0,0x02,0x00,0x7E,0x72,0x07,0x00,0x7C,0x3A,
0x07,0x00,0x78,0x1E,0x0F,0x80,0x78,0x0E,0x1F,0xC0,0x7C,0x1E,0x1F,0xE0,0x7E,0x00,
0x00,0x38,0x77,0x00,0x00,0x3E,0x73,0x90,0x00,0x3E,0x71,0xD0,0x00,0x3C,0x70,0xF0,
0x00,0x38,0x70,0x70,0x00,0x10,0x70,0xF0,0x00,0x00,0x30,0x00,0x00,0x00,0x00,0x00,	// digit '3' blinded sun
0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x03,0x00,0x00,0x00,0x03,0x00,0x00,
0x00,0x03,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x07,0x80,0x00,0x00,0x0F,0xC0,0x00,
0x00,0x0F,0xC0,0x00,0x00,0x1F,0xE0,0x00,0x00,0x3F,0xF0,0x00,0x00,0x7F,0xF8,0x00,
0x00,0xFF,0xBC,0x00,0x00,0xFF,0x9C,0x00,0x01,0xFF,0x8E,0x00,0x03,0xFF,0xC7,0x00,
0x03,0xFF,0xC3,0x00,0x07,0xFF,0xC3,0x80,0x07,0xFF,0xE1,0x80,0x07,0xFF,0xE1,0x80,
0x07,0xFF,0xE1,0x80,0x07,0xFF,0xE1,0x80,0x07,0xFF,0xC3,0x80,0x03,0xFF,0xC3,0x00,
0x03,0xFF,0xC7,0x00,0x01,0xFF,0x8E,0x00,0x01,0xFF,0x9E,0x00,0x00,0xFF,0xBC,0x00,
0x00,0x3F,0xF0,0x00,0x00,0x0F,0xC0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// digit '4' waterdrop
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x80,0x00,0x00,0x03,0xC0,0x00,
0x00,0x03,0xC0,0x00,0x01,0xC7,0xE3,0x80,0x01,0xF7,0xEF,0x80,0x01,0xF8,0x1F,0x80,
0x01,0xE0,0x07,0x80,0x01,0xC0,0x03,0x80,0x01,0x80,0x01,0x80,0x03,0x00,0x00,0xC0,
0x1F,0x00,0x00,0xF8,0x7E,0x00,0x00,0x7C,0x3E,0x00,0x00,0x7C,0x0E,0x00,0x00,0x70,
0x06,0x00,0x00,0x60,0x02,0x00,0x00,0x40,0x02,0x00,0x00,0x40,0x07,0x00,0x00,0xE0,
0x07,0x00,0x00,0xE0,0x0F,0x80,0x01,0xF0,0x1F,0xC0,0x03,0xF0,0x1F,0xE0,0x07,0xF8,
0x00,0x38,0x1C,0x00,0x00,0x3F,0xFC,0x00,0x00,0x3E,0x7C,0x00,0x00,0x3C,0x3C,0x00,
0x00,0x38,0x1C,0x00,0x00,0x10,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// digit '5' sun
0x00,0x00,0x00,0x00,0x00,0x0F,0xE0,0x00,0x00,0x7F,0xFC,0x00,0x00,0xFF,0xFF,0x00,
0x03,0xF0,0x0F,0xC0,0x07,0xC0,0x03,0xE0,0x0F,0x00,0x00,0xF0,0x1E,0x00,0x00,0x78,
0x1C,0x00,0x00,0x38,0x38,0x00,0x01,0x1C,0x38,0x00,0x03,0x9C,0x71,0x80,0x07,0x0E,
0x70,0xC0,0x0E,0x0E,0x60,0x70,0x1C,0x06,0xE0,0x3C,0x38,0x07,0xE0,0x1E,0x70,0x07,
0xE0,0x0F,0xE0,0x07,0xE0,0x07,0xC0,0x07,0xE0,0x03,0xC0,0x07,0xE0,0x01,0x80,0x07,
0x60,0x00,0x00,0x06,0x70,0x00,0x00,0x0E,0x70,0x00,0x00,0x0E,0x38,0x00,0x00,0x1C,
0x38,0x00,0x00,0x1C,0x1C,0x00,0x00,0x38,0x1E,0x00,0x00,0x78,0x0F,0x00,0x00,0xF0,
0x07,0xC0,0x03,0xE0,0x03,0xF0,0x0F,0xC0,0x00,0xFF,0xFF,0x00,0x00,0x3F,0xFC,0x00,	// digit '6' clock
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0C,0x30,0x61,0x80,0x0C,0x30,0x61,0x80,
0x06,0x18,0x30,0xC0,0x06,0x18,0x30,0xC0,0x06,0x18,0x30,0xC0,0x06,0x18,0x30,0xC0,
0x0C,0x30,0x61,0x80,0x0C,0x30,0x61,0x80,0x00,0x00,0x00,0x00,0x0E,0x1C,0x38,0x70,
0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0xFF,0x3E,0x7C,0xF8,0xFF,0x3E,0x7C,0xF8,
0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,
0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,
0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xFF,
0x1F,0x3E,0x7C,0xFF,0x1F,0x3E,0x7C,0xF8,0x1F,0x3E,0x7C,0xF8,0x0E,0x1C,0x38,0x70,	// digit '7' heat
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x0E,0x03,0x80,0x70,0x0E,0x03,0x80,0x70,
0x0E,0x03,0x80,0x70,0x00,0x03,0x80,0x70,0x00,0x03,0x80,0x70,0x1F,0x03,0x80,0x70,
0x3F,0x83,0x80,0x70,0x31,0x83,0x80,0x00,0x31,0x83,0x80,0x00,0x3F,0x83,0x80,0xF8,
0x1F,0x03,0x81,0xFC,0x00,0x03,0x81,0x8C,0x0E,0x03,0x81,0x8C,0x0E,0x03,0x81,0xFC,
0x0E,0x00,0x00,0xF8,0x0E,0x00,0x00,0x00,0x0E,0x00,0x00,0x00,0x0E,0x07,0xC0,0x70,
0x0E,0x0F,0xE0,0x70,0x0E,0x0C,0x60,0x70,0x0E,0x0C,0x60,0x70,0x0E,0x0F,0xE0,0x70,
0x0E,0x07,0xC0,0x70,0x0E,0x00,0x00,0x70,0x0E,0x00,0x00,0x70,0x0E,0x03,0x80,0x70,
0x0E,0x03,0x80,0x70,0x0E,0x03,0x80,0x70,0x0E,0x03,0x80,0x70,0x00,0x00,0x00,0x00,	// digit '8' controls
0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x10,0x00,
0x00,0x00,0x30,0x00,0x07,0xFE,0x7F,0xF0,0x0F,0xFE,0xFF,0xF8,0x1F,0xFE,0x7F,0xFC,
0x3C,0x00,0x30,0x3C,0x38,0x00,0x10,0x1C,0x38,0x00,0x00,0x1C,0x38,0x00,0x00,0x1C,
0x38,0x00,0x00,0x1C,0xFE,0x00,0x00,0x00,0x7C,0x00,0x00,0x08,0x38,0x00,0x00,0x1C,
0x10,0x00,0x00,0x3E,0x00,0x00,0x00,0x7F,0x38,0x00,0x00,0x1C,0x38,0x00,0x00,0x1C,
0x38,0x00,0x00,0x1C,0x38,0x00,0x00,0x1C,0x38,0x00,0x00,0x1C,0x38,0x04,0x00,0x1C,
0x3C,0x06,0x00,0x3C,0x3F,0xFF,0x3F,0xFC,0x1F,0xFF,0xBF,0xF8,0x0F,0xFF,0x3F,0xF0,
0x00,0x06,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,	// digit '9' scenarios-arrows
0x00,0x00,0x00,0x00,0x00,0x07,0x80,0x00,0x00,0x1F,0xE0,0x00,0x00,0x7F,0xF8,0x00,
0x00,0xF8,0x7C,0x00,0x01,0xE0,0x1C,0x00,0x01,0xC0,0x0E,0x00,0x03,0x80,0x0E,0x00,
0x03,0x80,0x07,0x00,0x0F,0x80,0x07,0xF0,0x1F,0x80,0x07,0xF8,0x3C,0x00,0x00,0x3C,
0x70,0x00,0x00,0x0E,0xE0,0x00,0x00,0x06,0xE0,0x00,0x00,0x07,0xE0,0x00,0x00,0x07,
0xE0,0x00,0x00,0x07,0xE0,0x1C,0x63,0x87,0xE0,0x1C,0xE3,0x07,0x70,0x18,0xE7,0x0F,
0x78,0x38,0xE7,0x1E,0x3C,0x39,0xC7,0x3C,0x1F,0x39,0xC6,0x78,0x0F,0x71,0xCE,0x70,
0x00,0x71,0x8E,0x00,0x00,0x73,0x8C,0x00,0x00,0x63,0x9C,0x00,0x00,0x63,0x9C,0x00,
0x00,0x03,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x07,0x00,0x00,0x00,0x00,0x00,0x00,	// symbol ':' rain	
0x00,0x01,0x00,0x00,0x00,0x03,0x80,0x00,0x00,0x03,0x80,0x00,0x18,0x03,0x80,0x30,
0x1C,0x01,0x00,0x70,0x0E,0x00,0x00,0xE0,0x07,0x00,0x01,0xC0,0x03,0x07,0xC1,0x80,
0x00,0x1F,0xF0,0x00,0x00,0x7F,0xFC,0x00,0x00,0x78,0x3C,0x00,0x00,0xF0,0x1E,0x00,
0x00,0xE0,0x0E,0x00,0x01,0xC0,0x07,0x00,0x71,0xC0,0x07,0x1C,0xF9,0xC0,0x07,0x3E,
0x71,0xC0,0x07,0x1C,0x01,0xC0,0x07,0x00,0x00,0xE0,0x0E,0x00,0x00,0xF0,0x1E,0x00,
0x00,0x78,0x3C,0x00,0x00,0x7F,0xFC,0x00,0x00,0x1F,0xF0,0x00,0x03,0x07,0xC1,0x80,
0x07,0x00,0x01,0xC0,0x0E,0x00,0x00,0xE0,0x1C,0x01,0x00,0x70,0x18,0x03,0x80,0x30,
0x00,0x03,0x80,0x00,0x00,0x03,0x80,0x00,0x00,0x01,0x00,0x00,0x00,0x00,0x00,0x00,		// symbol ';'  sun
0x00,0x03,0xC0,0x00,0x00,0x07,0xE0,0x00,0x00,0x0F,0xF0,0x00,0x00,0x1E,0x78,0x00,
0x00,0x3C,0x3C,0x00,0x00,0x78,0x1E,0x00,0x00,0xF0,0x0F,0x00,0x01,0xE0,0x07,0x80,
0x03,0xC0,0x03,0xC0,0x07,0x80,0x01,0xE0,0x0F,0x00,0x00,0xF0,0x1E,0x00,0x00,0x78,
0x3C,0x3F,0xFC,0x3C,0x78,0x3F,0xFF,0x1E,0xF0,0x3F,0xFF,0x8F,0xF0,0x3F,0xFF,0xCF,
0x30,0x00,0x0F,0xCC,0x30,0x00,0x03,0xCC,0x30,0x00,0x03,0xCC,0x30,0x20,0x03,0xCC,
0x30,0x60,0x03,0xCC,0x30,0xE0,0x0F,0xCC,0x31,0xFF,0xFF,0xCC,0x33,0xFF,0xFF,0x8C,
0x33,0xFF,0xFF,0x0C,0x31,0xFF,0xFC,0x0C,0x30,0xE0,0x00,0x0C,0x30,0x60,0x00,0x0C,
0x30,0x20,0x00,0x0C,0x30,0x00,0x00,0x0C,0x3F,0xFF,0xFF,0xFC,0x3F,0xFF,0xFF,0xFC,	// symbol '<' cycle vent
0x00,0x03,0xC0,0x00,0x00,0x07,0xE0,0x00,0x00,0x0F,0xF0,0x00,0x00,0x1E,0x78,0x00,
0x00,0x3C,0x3C,0x00,0x00,0x78,0x1E,0x00,0x00,0xF0,0x0F,0x00,0x01,0xE0,0x07,0x80,
0x03,0xC0,0x03,0xC0,0x07,0x80,0x01,0xE0,0x0F,0x00,0x00,0xF0,0x1E,0x00,0x00,0x78,
0x3C,0x00,0x00,0x3C,0x78,0x00,0x00,0x1E,0xF0,0x00,0x00,0x0F,0xF0,0x00,0x00,0x0F,
0x30,0x00,0x00,0x0C,0x30,0x00,0x01,0x0C,0x00,0x00,0x01,0x8C,0x00,0x00,0x01,0xC4,
0x1F,0xFF,0xFF,0xE4,0x3F,0xFF,0xFF,0xF0,0x3F,0xFF,0xFF,0xF8,0x3F,0xFF,0xFF,0xF0,
0x1F,0xFF,0xFF,0xE4,0x00,0x00,0x01,0xC4,0x00,0x00,0x01,0x8C,0x30,0x00,0x01,0x0C,
0x30,0x00,0x00,0x0C,0x30,0x00,0x00,0x0C,0x3F,0xFF,0xFF,0xFC,0x3F,0xFF,0xFF,0xFC,		// symbol '=' external vent
0x00,0x00,0x00,0x00,0x3F,0xFF,0xFF,0xFC,0x7F,0xFF,0xFF,0xFE,0x60,0x00,0x00,0x06,
0x60,0x00,0x00,0x06,0x67,0xFF,0xFF,0xE6,0x64,0x00,0x00,0x26,0x64,0xF9,0xF3,0x26,
0x64,0xF9,0xF4,0xA6,0x64,0x18,0x34,0xA6,0x64,0x18,0x33,0x26,0x64,0xF9,0xF0,0x26,
0x64,0xF9,0xF0,0x26,0x64,0xC0,0x30,0x26,0x64,0xC0,0x30,0x26,0x64,0xF9,0xF0,0x26,
0x64,0xF9,0xF0,0x26,0x64,0x00,0x00,0x26,0x67,0xFF,0xFF,0xE6,0x60,0x00,0x00,0x06,
0x60,0x00,0x00,0x06,0x60,0x00,0x00,0x06,0x6F,0xE3,0xC0,0x86,0x6C,0x66,0x61,0xC6,
0x66,0xC4,0x23,0x66,0x63,0x86,0x66,0x36,0x61,0x03,0xC7,0xF6,0x60,0x00,0x00,0x06,
0x60,0x00,0x00,0x06,0x7F,0xFF,0xFF,0xFE,0x1F,0xFF,0xFF,0xF8,0x00,0x00,0x00,0x00,	// symbol '>' thermostat
0x00,0x00,0x00,0x00,0x30,0x00,0x00,0x0C,0x33,0xF0,0x0F,0xCC,0x37,0xF8,0x1F,0xEC,
0x3E,0x1C,0x38,0x7C,0x3C,0x0C,0x30,0x3C,0x30,0x07,0xE0,0x0C,0x30,0x03,0xC0,0x0C,
0x30,0x00,0x00,0x0C,0x30,0x00,0x00,0x0C,0x30,0x00,0x60,0x0C,0x30,0x00,0x60,0x0C,
0x30,0x00,0x60,0x0C,0x30,0x00,0x60,0x0C,0x31,0xB8,0x6F,0x0C,0x31,0xFC,0x7F,0x8C,
0x31,0xCE,0x71,0x8C,0x31,0x86,0x61,0x8C,0x31,0x86,0x61,0x8C,0x31,0x86,0x61,0x8C,
0x31,0x86,0x61,0x8C,0x31,0xCE,0x61,0x8C,0x31,0xFC,0x61,0x8C,0x31,0xB8,0x61,0x8C,
0x31,0x80,0x00,0x0C,0x31,0x80,0x00,0x0C,0x31,0x80,0x00,0x0C,0x31,0x80,0x00,0x0C,
0x38,0x00,0x00,0x1C,0x1F,0xFF,0xFF,0xF8,0x0F,0xFF,0xFF,0xF0,0x00,0x00,0x00,0x00,	        // symbol '?' PH icon
0x00,0x7F,0xFE,0x00,0x00,0xFF,0xFF,0x00,0x00,0x80,0x01,0x00,0x00,0x30,0x0C,0x00,
0x0F,0xF0,0x0F,0xF0,0x1F,0xF0,0x0F,0xF8,0x18,0x00,0x00,0x18,0x78,0x00,0x00,0x18,
0x78,0x00,0x00,0x18,0x00,0x00,0x00,0x18,0x00,0x00,0x00,0x18,0x78,0x00,0x00,0x18,
0x78,0x00,0x00,0x18,0x19,0xC3,0xC3,0x98,0x1B,0x67,0x67,0xD8,0x1E,0xBE,0xBE,0xF8,
0x1D,0x55,0x55,0x58,0x1A,0xAA,0xAA,0xB8,0x1D,0x55,0x55,0x58,0x1A,0xAA,0xAA,0xB8,
0x1D,0x55,0x55,0x58,0x1A,0xAA,0xAA,0xBE,0x1D,0x55,0x55,0x5E,0x1A,0xAA,0xAA,0xA8,
0x1D,0x55,0x55,0x54,0x1A,0xAA,0xAA,0xBE,0x1D,0x55,0x55,0x5E,0x1A,0xAA,0xAA,0xB8,
0x1D,0x55,0x55,0x78,0x0F,0xFF,0xFF,0xF0,0x07,0xFF,0xFF,0xE0,0x00,0x00,0x00,0x00,	// symbol '@' Water Tank
0x3F,0xFF,0xFF,0xFC,0x7F,0xFF,0xFF,0xFE,0xF0,0x00,0x00,0x0F,0xE0,0x0E,0x70,0x07,
0xC0,0x3E,0x7C,0x03,0xC0,0xF6,0x6F,0x03,0xC1,0xC7,0xE3,0x83,0xC3,0x87,0xE1,0xC3,
0xC3,0x00,0x00,0xC3,0xC6,0x00,0x00,0x63,0xC6,0x00,0x00,0x63,0xCC,0x00,0x00,0x33,
0xCC,0x00,0x00,0x33,0xCC,0x00,0x00,0x33,0xCC,0x70,0x0E,0x33,0xCC,0xF8,0x1F,0x33,
0xCC,0xD8,0x1B,0x33,0xCC,0xF8,0x1F,0x33,0xCC,0x70,0x0E,0x33,0xCC,0x00,0x00,0x33,
0xC6,0x00,0x00,0x63,0xC6,0x00,0x00,0x63,0xC3,0x00,0x00,0xC3,0xC3,0x87,0xE1,0xC3,
0xC1,0xC7,0xE3,0x83,0xC0,0xF6,0x6F,0x03,0xC0,0x3E,0x7C,0x03,0xC0,0x1E,0x78,0x03,
0xE0,0x00,0x00,0x07,0xF0,0x00,0x00,0x0F,0x7F,0xFF,0xFF,0xFE,0x3F,0xFF,0xFF,0xFC	        // eng 'A'  wall socket
};
