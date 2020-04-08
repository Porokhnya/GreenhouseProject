#include "PDUClasses.h"
#include <avr/pgmspace.h>
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
PDUHelper PDU;
//--------------------------------------------------------------------------------------------------------------------------------------
const char HEX_CHARS[]  PROGMEM = {"0123456789ABCDEF"};
//--------------------------------------------------------------------------------------------------------------------------------------
unsigned int PDUMessageEncoder::utf8GetCharSize(unsigned char bt) 
{ 
  if (bt < 128) 
  return 1; 
  else if ((bt & 0xE0) == 0xC0) 
  return 2; 
  else if ((bt & 0xF0) == 0xE0) 
  return 3; 
  else if ((bt & 0xF8) == 0xF0) 
  return 4; 
  else if ((bt & 0xFC) == 0xF8) 
  return 5; 
  else if ((bt & 0xFE) == 0xFC) 
  return 6; 

 
  return 1; 
} 
//--------------------------------------------------------------------------------------------------------------------------------------
bool PDUMessageEncoder::utf8ToUInt(const String& bytes, unsigned int& target) 
{ 
  unsigned int bsize = bytes.length(); 
  target = 0; 
  bool result = true; 
  if (bsize == 1) 
  { 
    if ((bytes[0] >> 7) != 0) 
    { 
      result = false; 
    } 
      else 
      target = bytes[0]; 
   } 
    else 
    { 
      unsigned char cur_byte = bytes[0]; 
      target = ((cur_byte & (0xFF >> (bsize + 1))) << (6 * (bsize - 1))); 
      unsigned int i = 1; 
      
      while (result && (i < bsize)) 
      { 
        cur_byte = bytes[i]; 
        if ((cur_byte >> 6) != 2) 
        { 
          result = false; 
         } 
         else 
          target |= ((cur_byte & 0x3F) << (6 * (bsize - 1 - i))); i++; 
       } 
    } 
    return result; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
PDUMessageEncoder::PDUMessageEncoder()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
void PDUMessageEncoder::UTF8ToUCS2(const String& s, unsigned int& bytesProcessed, String* output)
{
  // Попытка оптимизировать работу с памятью
  const char* ptr = s.c_str();
 

  // в исходной строке - N байт. в кодировке UCS2 каждый символ - два байта, в строковом представлении - 4 символа,
  // т.е. нам надо зарезервировать буфер в 4 раза больше, чем длина строки. В дополнение к этому зарезервируем
  // в хвосте ещё 34 байта, которые будут нужны для заголовков, итого - плюс 35 байт, учитывая место под завершающий ноль.
  // но мы добавим ещё пару байт, на всякий, итого - 37.
  output->reserve(s.length()*4+PDU_EXTRA_LENGTH);

  while(*ptr) 
  {
    byte curChar = (byte) *ptr;
    unsigned int charSz = utf8GetCharSize(curChar);

    // у нас максимальная длина символа в UTF-8 - 6 байт, поэтому используем промежуточный буфер в 7 байт
    static char curBytes[7] = {0};

    for(byte i=0;i<charSz;i++) 
    {
      curBytes[i] = *ptr++;
    }

    curBytes[charSz] = '\0'; // добавляем завершающий 0

    unsigned int myChar;
    if(utf8ToUInt(curBytes,myChar))
    {
      bytesProcessed++;
      *output += ( ToHex( (myChar & 0xFF00)>>8 )) + ( ToHex( myChar & 0xFF ) );
    }
    
  } // while

  //TODO: ПОД ВОПРОСОМ - НАДО ЛИ ЭТО ДЕЛАТЬ! Т.К. ЕСЛИ ЭТО ДЕЛАТЬ - В СТРОКЕ ПОЯВЛЯЕТСЯ ЛИШНИЙ 0!!!
  //*output += '\0';

}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageEncoder::EncodePhoneNumber(const char* nm)
{
  
    String result;
    
    result.reserve(strlen(nm)+2);
    result = nm;
    
    if(result.length() % 2 > 0)
      result += F("F");
       
    uint8_t i=0;
    while(i < result.length())
    {
      char ch = result[i+1];
      result[i+1] = result[i];
      result[i] = ch;
      
      i+=2;
    }

  return result;

}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageEncoder::ToHex(int i)
{  
  
  char out[3] = {0};
  int idx = i & 0xF;
  //char char1 
  out[1] = (char) pgm_read_byte_near( HEX_CHARS + idx );
  i>>=4;
  idx = i & 0xF;
  //char char2
  out[0] = (char) pgm_read_byte_near( HEX_CHARS + idx );


  return out;

}
//--------------------------------------------------------------------------------------------------------------------------------------
PDUOutgoingMessage PDUMessageEncoder::Encode(const String& recipientPhoneNum, const String& utf8Message, bool isFlash, String* outBuffer, bool incomingMessageInUCS2Format)
{
  PDUOutgoingMessage result;

  if(!outBuffer)
    return result;

   *outBuffer = ""; // очищаем строку

   // сохраняем указатель на неё
   result.Message = outBuffer;

  // Попытка оптимизации работы с памятью
  
  const char* phonePtr = recipientPhoneNum.c_str();
  if(*phonePtr == '+')
    phonePtr++; 

  int phoneNumLen = strlen(phonePtr);

  String encodedPhoneNum = EncodePhoneNumber(phonePtr);

  int reserveLen = encodedPhoneNum.length() + 7; // 1 байт - символ завершения строки, два байта - "91", 4 байта - длиза номера, закодированная в UCS2
  
  String recipient; 
  recipient.reserve(reserveLen);
  recipient = ToHex(phoneNumLen) + F("91") + encodedPhoneNum;
  
  unsigned int bytesProcessed = 0;
  
  if(!incomingMessageInUCS2Format) // если входящее сообщение не в UCS2 - кодируем его
    UTF8ToUCS2(utf8Message,bytesProcessed,result.Message);
  else {
    // иначе - подсчитываем необходимые нам данные
    bytesProcessed = utf8Message.length()/4; // поскольку каждый символ в UCS2 кодируется 4 байтами - кол-во символов вычисляется легко 

    // проверяем - возможно, СМС в одно не вместится, тогда - режем нещадно
    #define ONE_SMS_MAXLENGTH 50
    if(utf8Message.length() > ONE_SMS_MAXLENGTH*4)
    {
      bytesProcessed = ONE_SMS_MAXLENGTH;
    }

    // выделяем буфер нужной длины
    outBuffer->reserve(utf8Message.length()+PDU_EXTRA_LENGTH);
    *outBuffer = utf8Message.c_str();
    //outBuffer->setCharAt(bytesProcessed*4,'\0');
    outBuffer->remove(bytesProcessed*4);
  }
    
  String strBytesProcessed = ToHex(bytesProcessed*2);
  
  // длина headers - 6 байт, фиксирована
  // длина recipient - максимум 20 байт
  // длина headers2 - 8 байт
  // итого имеем, что в message надо иметь запас буфера в 6+20+8 = 34 байта,
  // таким образом мы сэкономим прорву памяти, не копируя туда-сюда буфера.

  // в result.Message у нас уже лежит строка, у которой в конце есть свободный хвост.
  // если мы сдвинем строку так, чтобы в начале оставался пробел нужной нам величины - 
  // мы просто сможем записать туда нужную нам информацию.
  unsigned int neededShiftLen = 10 + recipient.length() + strBytesProcessed.length();

  // теперь мы должны сдвинуть каждый символ строки на neededShiftLen позиций вправо.
  // сперва ищем окончание строки
  const char* strBegin = result.Message->c_str();
  const char* strEnd = strBegin;
  while(*strEnd)
    strEnd++;

   // потом добавляем в строку N символов, чтобы она пересчитала свою длину.
   // память при этом выделяться не будет, т.к. буфер уже выделен.
   for(unsigned int i=0;i<neededShiftLen;i++)
    *(result.Message) += '\0';

  // strEnd у нас стоит на нулевом символе, после него - свободный буфер с памятью, которую мы можем использовать.
  // начальная позиция копирования у нас - strEnd, а сдвиг - neededShiftLen, т.е. каждый символ из strEnd мы копируем
  // вправо на neededShiftLen до тех пор, пока не дойдём до начала строки
  while(strEnd >= strBegin) 
  {
    unsigned int startPos = (strEnd - strBegin);
    unsigned int endPos = (startPos + neededShiftLen);
    char ch = result.Message->charAt(startPos);
    result.Message->setCharAt(endPos,ch);
    strEnd--;  
  }

  // теперь у нас всё сдвинуто вправо на нужную величину, пишем в message всё, что нужно.
  unsigned int curWritePos = 0;
  
  unsigned int lengthHelper = 0;
  
  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++,'1');
  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++,'0');

  lengthHelper = recipient.length();
  for(unsigned int i=0;i<lengthHelper;i++) 
  {
    result.Message->setCharAt(curWritePos,recipient.charAt(i));
    curWritePos++;
  }

  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++,'0');
  result.Message->setCharAt(curWritePos++, isFlash ? '1' : '0');
  result.Message->setCharAt(curWritePos++,'8');

  for(unsigned int i=0;i<strBytesProcessed.length();i++)
    result.Message->setCharAt(curWritePos++,strBytesProcessed[i]);

  result.MessageLength = result.Message->length()/2 - 1; // без учёта длины смс-центра, мы его не указываем (пишем "00"),значит - минус 1 байт.
  //result.MessageLength = strlen(result.Message->c_str())/2 - 1; // без учёта длины смс-центра, мы его не указываем (пишем "00"),значит - минус 1 байт. 

  return result; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
PDUMessageDecoder::PDUMessageDecoder()
{
  
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t PDUMessageDecoder::HexToNum(const String& numberS) 
{
  uint8_t tens = MakeNum(numberS[0]);
  
  uint8_t ones = 0;
  
  if(numberS.length() > 1) // means two characters entered
    ones = MakeNum(numberS[1]);
    
  if(ones == 'X') 
    return  0;
    
  return  (tens * 16) + ones;
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t PDUMessageDecoder::MakeNum(char ch) 
{
  if((ch >= '0') && (ch <= '9'))
    return ((uint8_t) ch) - '0';
  
  switch(ch) 
  {
    case 'A':
    case 'a': return 10;
    
    case 'B':
    case 'b': return 11;
    
    case 'C':
    case 'c': return 12;
    
    case 'D':
    case 'd': return 13;

    case 'E':
    case 'e': return 14;
    
    case 'F':
    case 'f': return 15;
    
    default: return 16;
    }

}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageDecoder::exchangeOctets(const String& src)
{
  String out;
  for(uint16_t i =0;i<src.length();i+=2)
  {
    out += mapChar(src[i+1]);
    out += mapChar(src[i]);
  }

  return out;
}
//--------------------------------------------------------------------------------------------------------------------------------------
char PDUMessageDecoder::mapChar(char ch)
{
  if((ch >= '0') && (ch <= '9'))
    return ch;

  switch(ch)
  {
    case '*':
      return 'A';
      
    case '#':
      return 'B';
      
    case 'A':
    case 'a':
      return 'C';
      
    case 'B':
    case 'b':
      return 'D';
      
    case 'C':
    case 'c':
      return 'E';
      
    default:
      return 'F';
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
uint8_t PDUMessageDecoder::DCS_Bits(const String& tp_DCS)
{
  uint8_t AlphabetSize=7; // Set Default
  uint8_t pomDCS = HexToNum(tp_DCS); 
    
  switch(pomDCS & 192)
  {
    case 0: 
      switch(pomDCS & 12)
      {
        case 4:
          AlphabetSize=8;
          break;
        case 8:
          AlphabetSize=16;
          break;
      }
      break;
    case 192:
      switch(pomDCS & 0x30)
      {
        case 0x20:
          AlphabetSize=16;
          break;
        case 0x30:
          if (pomDCS & 0x4)
          {
            ;
          }
          else
          {
            AlphabetSize=8;
          }
          break;
      }
      break;
            
  }
  return AlphabetSize; 
}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageDecoder::getUTF8From16BitEncoding(const String& ucs2Message)
{
  String result;

  unsigned char buff[6] = {0};
  for(uint16_t i=0;i<ucs2Message.length();i+=4)
  {
     String hex1 = ucs2Message.substring(i,i+2);
     String hex2 = ucs2Message.substring(i+2,i+4);
     unsigned long ucs2Code = HexToNum(hex1)*256 + HexToNum(hex2);

     if(UCS2ToUTF8(ucs2Code,buff) > 0)
      result += (char*) buff;
  } // for

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageDecoder::getUTF8From8BitEncoding(const String& ucs2Message)
{
 
  String result;

  unsigned char buff[6] = {0};
  for(uint16_t i=0;i<ucs2Message.length();i+=2)
  {
     String hex1 = ucs2Message.substring(i,i+2);
     unsigned long ucs2Code = HexToNum(hex1);

    if(UCS2ToUTF8(ucs2Code,buff) > 0)
      result += (char*) buff;

  } // for

  return result;  
}
//--------------------------------------------------------------------------------------------------------------------------------------
String PDUMessageDecoder::getUTF8From7BitEncoding(const String& ucs2Message, uint16_t trueLength)
{
  String result;

  String bytes;

  // собираем все байты из их текстового представления
  for(uint16_t i = 0; i < ucs2Message.length();i+=2)
  {
    String hex1 = ucs2Message.substring(i,i+2);
    char ucs2Code = (char) HexToNum(hex1);
    bytes += ucs2Code; 
  } // for
    
   // начинаем декодировать семибитную кодировку
   int bits=0, i = 0, last = 0;
   uint16_t j = 0;

    while(j < trueLength)
      {
        if(bits < 7)
        {
          last |= ((byte)bytes[i++]) << bits;
          bits+=8;
        }
        char c = last & 0x7F;
        last >>= 7;
        bits -= 7;
        result += (c == 0x02) ? String(F("\n")) : String(c);
        j++;
      }

 return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
int PDUMessageDecoder::UCS2ToUTF8 (unsigned long ucs2, unsigned char * utf8)
{
    if (ucs2 < 0x80) 
    {
        utf8[0] = ucs2;
        utf8[1] = '\0';
        return 1;
    }
    if (ucs2 >= 0x80  && ucs2 < 0x800) 
    {
        utf8[0] = (ucs2 >> 6)   | 0xC0;
        utf8[1] = (ucs2 & 0x3F) | 0x80;
        utf8[2] = '\0';
        return 2;
    }
    if (ucs2 >= 0x800 && ucs2 < 0xFFFF) 
    {
      if (ucs2 >= 0xD800 && ucs2 <= 0xDFFF) 
      {
          /* Ill-formed. */
          return 0;
      }
        utf8[0] = ((ucs2 >> 12)       ) | 0xE0;
        utf8[1] = ((ucs2 >> 6 ) & 0x3F) | 0x80;
        utf8[2] = ((ucs2      ) & 0x3F) | 0x80;
        utf8[3] = '\0';
        return 3;
    }
    if (ucs2 >= 0x10000 && ucs2 < 0x10FFFF) 
    {
      utf8[0] = 0xF0 | (ucs2 >> 18);
      utf8[1] = 0x80 | ((ucs2 >> 12) & 0x3F);
      utf8[2] = 0x80 | ((ucs2 >> 6) & 0x3F);
      utf8[3] = 0x80 | ((ucs2 & 0x3F));
      utf8[4] = '\0';
        return 4;
    }
    return 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
PDUIncomingMessage PDUMessageDecoder::Decode(const String& ucs2Message)
{
  PDUIncomingMessage result;
  result.IsDecodingSucceed = true;

  String workStr = ucs2Message;


  uint8_t smscNumberLength = HexToNum(workStr.substring(0,2));
    
  String smsCenterInfo = workStr.substring(2,2+(smscNumberLength*2));
  uint8_t smscTypeOfAddress = smsCenterInfo.substring(0,2).toInt();

  String smsCenterNumber = smsCenterInfo.substring(2,2+(smscNumberLength*2));

  if(smscNumberLength > 0) // есть информация об СМС-центре
  {
     smsCenterNumber = exchangeOctets(smsCenterNumber);
     
     if(smscTypeOfAddress == 91)
      smsCenterNumber = String(F("+")) + smsCenterNumber;

      smsCenterNumber.replace(F("F"),F("")); // убираем последнюю F, если она есть
     
  } // if(smscNumberLength)

  // сохраняем данные об СМС-центре
  result.SMSCenterNumber = smsCenterNumber;
  

  uint16_t start = 0;
  uint16_t startDeliveryInfo = (smscNumberLength*2)+2;

  start = startDeliveryInfo;
  String smsDeliver = workStr.substring(start,start+2);
  start = start + 2;

  uint8_t smsDeliverBits = HexToNum(smsDeliver);

  if((smsDeliverBits & 0x03) == 0) // входящее сообщение
  {
      uint8_t senderAddrLen = HexToNum(workStr.substring(start,start+2));
      start += 2;
      String typeOfAddress = workStr.substring(start,start+2);
      start += 2;

     String sender_number;

      if (typeOfAddress == F("D0"))
      {
          if(senderAddrLen%2 != 0)
            senderAddrLen++;

          String encodedNum = workStr.substring(start,start+senderAddrLen);
 
          sender_number = getUTF8From7BitEncoding(encodedNum,uint16_t(senderAddrLen/2*8/7));


      } // if (sender_typeOfAddress == "D0")
      else
      {
  
        if(senderAddrLen%2 != 0)
          senderAddrLen++;
  
          sender_number = exchangeOctets(workStr.substring(start,start+senderAddrLen));

          sender_number.replace(F("F"),"");
          
          if (typeOfAddress.toInt() == 91)
            sender_number = "+" + sender_number;


      } // else

     // сохраняем номер телефона отправителя
      result.SenderNumber = sender_number;

      // тут декодируем сообщение...
      start += senderAddrLen;
      
      //String tp_PID = workStr.substring(start,start+2);
      start +=2;

      String tp_DCS = workStr.substring(start,start+2);
      start +=2;

     // skip timestamp
     start +=14;

     uint16_t messageLength = HexToNum(workStr.substring(start,start+2));
     start += 2;
 
     uint8_t bitSize = DCS_Bits(tp_DCS);
  
    if (bitSize==7)
    {

     String mess = workStr.substring(start);
      result.Message = getUTF8From7BitEncoding(mess,messageLength);

     }
    else 
    if (bitSize==8)
    {
      String mess = workStr.substring(start);
      result.Message = getUTF8From8BitEncoding(mess);
     }
    else 
    if (bitSize==16)
    {
      String mess = workStr.substring(start);
      result.Message = getUTF8From16BitEncoding(mess);
    }


   // сохраняем декодированное сообщение
    result.Message = result.Message.substring(0,messageLength);

  } // if((smsDeliverBits & 0x03) == 0)
  else
  if ((smsDeliverBits & 0x03) == 1 || (smsDeliverBits & 0x03) == 3) // сообщение для пересылки
  {
    // uint8_t MessageReference = HexToNum(workStr.substring(start,start+2));
    start += 2;

    // length in decimals
    uint8_t sender_addressLength = HexToNum(workStr.substring(start,start+2));
    if(sender_addressLength%2 != 0)
    {
      sender_addressLength++;
    }
    start += 2;

    String sender_typeOfAddress = workStr.substring(start,start+2);
    start += 2;

    String sender_number = exchangeOctets(workStr.substring(start,start+sender_addressLength));
    
    sender_number.replace(F("F"),F(""));
    
    if (sender_typeOfAddress.toInt() == 91)
    {
      sender_number = String(F("+")) + sender_number;
    }
    start += sender_addressLength;

    result.SenderNumber = sender_number;

    //String tp_PID = workStr.substring(start,start+2);
    start +=2;

     String tp_DCS = workStr.substring(start,start+2);
     start +=2;

    switch( smsDeliverBits & 0x18 )
    {
      case 0: // Not Present
        break;
      case 0x10: // Relative
              start +=2;
        break;
      case 0x08: // Enhanced
              start +=14;
        break;
      case 0x18: // Absolute
              start +=14;
        break;
    }

    uint8_t messageLength = HexToNum(workStr.substring(start,start+2));
    start += 2;
    uint8_t bitSize = DCS_Bits(tp_DCS);

    if (bitSize==7)
    {
      String mess = workStr.substring(start);
      result.Message = getUTF8From7BitEncoding(mess,messageLength);
    }
    else 
    
    if (bitSize==8)
    {
      String mess = workStr.substring(start);
      result.Message = getUTF8From8BitEncoding(mess);
    }
    else if (bitSize==16)
    {
      String mess = workStr.substring(start);
      result.Message = getUTF8From16BitEncoding(mess);
    }

    result.Message = result.Message.substring(0,messageLength);
    
  }
  else
  {
    result.IsDecodingSucceed = false; // другие сообщения не парсим
  }

  return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------

