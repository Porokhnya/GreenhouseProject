//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#include "UTFTRus.h"
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_TFT_MODULE
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
const uint8_t utf8_rus_charmap[] PROGMEM = {'A',128,'B',129,130,'E',131,132,133,134,135,'K',136,'M','H','O',137,'P','C','T',138,139,'X',140,141,
142,143,144,145,146,147,148,149,'a',150,151,152,153,'e',154,155,156,157,158,159,160,161,162,'o',163,'p','c',164,'y',165,'x',166,167,168,169,170,
171,172,173,174,175};
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
// UTFTRus
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
UTFTRus::UTFTRus()
{
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void UTFTRus::init(UTFT* uTft)
{
   pDisplay = uTft;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void UTFTRus::printSpecialChar(TFTSpecialSimbol specialSymbol, int x, int y, int deg)
{
  int iCh = mapChar(specialSymbol);
  if(iCh == -1)
    return;

 char ch = (char) iCh;

    if (deg==0) 
    {
        pDisplay->printChar(ch, x, y);
    } 
    else 
    {
        pDisplay->rotateChar(ch, x, y, 0, deg);
    }

   yield();
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
char UTFTRus::mapChar(TFTSpecialSimbol ch)
{
  switch(ch)
  {
    case charComma: // ,
      return '5';
      
    case charDot: // .
      return '3';

    case charMinus: // -
      return '4';

    case charPlus: // +
      return '2';

    case charPercent: // PERCENT
      return '0';

    case charLux: // LUX
      return '1';

   case charDegree: // °
    return '6';

   case charRightArrow: // >
    return '7';

   case charLeftArrow: // <
    return '8';

   case charUnknown:
    return -1;

  }

  return -1;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
int UTFTRus::print(const char* st,int x, int y, int deg, bool computeStringLengthOnly)
{
 int stl, i;
  stl = strlen(st);

  if (pDisplay->orient==PORTRAIT)
  {
    if (x==RIGHT) 
      x=(pDisplay->disp_x_size+1)-(stl*pDisplay->cfont.x_size);
  
    if (x==CENTER) 
      x=((pDisplay->disp_x_size+1)-(stl*pDisplay->cfont.x_size))/2;
  } 
  else 
  {
    if (x==RIGHT) 
      x=(pDisplay->disp_y_size+1)-(stl*pDisplay->cfont.x_size);
    
    if (x==CENTER) 
      x=((pDisplay->disp_y_size+1)-(stl*pDisplay->cfont.x_size))/2;
  }
  
  uint8_t utf_high_byte = 0;
  uint8_t ch, ch_pos = 0;
  
  for (i = 0; i < stl; i++) 
  {
    ch = st[i];
    
    if ( ch >= 128) 
    {
      if ( utf_high_byte == 0 && (ch ==0xD0 || ch == 0xD1)) 
      {
        utf_high_byte = ch;
        continue;
      } 
      else 
      {
        if ( utf_high_byte == 0xD0) 
        {
          if (ch == 0x81) 
          { //Ё
            ch = 6;
          } 
          else 
          {
            if(ch <= 0x95) 
            {
              ch -= 0x90;
            } 
            else if( ch < 0xB6)
            {
              ch -= (0x90 - 1);
            } 
            else 
            {
              ch -= (0x90 - 2);
            }
          }
          
          ch = pgm_read_byte((utf8_rus_charmap + ch));
        
        } 
        else if (utf_high_byte == 0xD1) 
        {
          if (ch == 0x91) 
          {//ё
            ch = 39;
          } 
          else 
          {
            ch -= 0x80;
            ch += 50;
          }
          
          ch = pgm_read_byte((utf8_rus_charmap + ch));
        }
        
        utf_high_byte = 0;
      }
    } 
    else 
    {
      utf_high_byte = 0;
    }
    

    if (deg==0) 
    {
      if(!computeStringLengthOnly)
      {
        pDisplay->printChar(ch, x + (ch_pos * (pDisplay->cfont.x_size)), y);
      }
    } 
    else 
    {
      if(!computeStringLengthOnly)
      {
        pDisplay->rotateChar(ch, x, y, ch_pos, deg);
      }
    }
    ++ch_pos;
  } // for  

  yield();
  
  return ch_pos;
}
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
#endif // USE_TFT_MODULE
