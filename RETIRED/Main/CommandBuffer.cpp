#include "CommandBuffer.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------------
CommandBuffer::CommandBuffer(Stream* s) : pStream(s)
{
    strBuff = new String();
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool CommandBuffer::HasCommand()
{
  if(!(pStream && pStream->available()))
    return false;

    char ch;
    while(pStream->available())
    {
      ch = pStream->read();

///////      pStream->print(ch);
      
      if(ch == '\r' || ch == '\n')
      {
        return strBuff->length() > 0; // вдруг лишние управляющие символы придут в начале строки?
      } // if

      *strBuff += ch;
      // не даём вычитать больше символов, чем надо - иначе нас можно заспамить
      if(strBuff->length() >= MAX_RECEIVE_BUFFER_LENGTH)
      {
         ClearCommand();
         return false;
      } // if
    } // while

    return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------


