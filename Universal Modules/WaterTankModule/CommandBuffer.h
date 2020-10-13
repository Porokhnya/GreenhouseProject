#ifndef _COMMAND_BUFFER_H
#define _COMMAND_BUFFER_H

#include <Stream.h>
//--------------------------------------------------------------------------------------------------------------------------------------
// класс для накопления команды из потока
//--------------------------------------------------------------------------------------------------------------------------------------
class CommandBuffer
{
private:
  Stream* pStream;
  String* strBuff;
public:
  CommandBuffer(Stream* s);

  bool HasCommand();
  const String& GetCommand() {return *strBuff;}
  void ClearCommand() { delete strBuff; strBuff = new String(); }
  Stream* GetStream() {return pStream;}

};
//--------------------------------------------------------------------------------------------------------------------------------------
#endif
