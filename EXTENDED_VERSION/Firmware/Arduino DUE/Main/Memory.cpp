#include "Memory.h"
#include "Globals.h"
//--------------------------------------------------------------------------------------------------------------------------------
#ifdef USE_EXTERNAL_WATCHDOG
extern void updateExternalWatchdog();
#endif
//------------------------------------------------------------------------------------------------------------------------------------------------------------------------
void* MemFind(const void *haystack, size_t n, const void *needle, size_t m)
{
        const unsigned char *y = (const unsigned char *)haystack;
        const unsigned char *x = (const unsigned char *)needle;

        size_t j, k, l;

        if (m > n || !m || !n)
                return NULL;

        if (1 != m) {
                if (x[0] == x[1]) {
                        k = 2;
                        l = 1;
                } else {
                        k = 1;
                        l = 2;
                }

                j = 0;
                while (j <= n - m) {
                        if (x[1] != y[j + 1]) {
                                j += k;
                        } else {
                                if (!memcmp(x + 2, y + j + 2, m - 2)
                                    && x[0] == y[j])
                                        return (void *)&y[j];
                                j += l;
                        }
                }
        } else
                do {
                        if (*y == *x)
                                return (void *)y;
                        y++;
                } while (--n);

        return NULL;
}
//--------------------------------------------------------------------------------------------------------------------------------------
#if EEPROM_USED_MEMORY == EEPROM_BUILTIN
  #include <EEPROM.h>
#else
  #include "AT24CX.h"
  AT24CX* memoryBank;
#endif
//--------------------------------------------------------------------------------------------------------------------------------
void MemInit()
{
#if EEPROM_USED_MEMORY == EEPROM_BUILTIN
  // не надо инициализировать дополнительно
#elif EEPROM_USED_MEMORY == EEPROM_AT24C32
   memoryBank = new AT24C32(EEPROM_MEMORY_INDEX);
#elif EEPROM_USED_MEMORY == EEPROM_AT24C64
  memoryBank = new AT24C64(EEPROM_MEMORY_INDEX);
#elif EEPROM_USED_MEMORY == EEPROM_AT24C128
 memoryBank = new AT24C128(EEPROM_MEMORY_INDEX);
#elif EEPROM_USED_MEMORY == EEPROM_AT24C256
 memoryBank = new AT24C256(EEPROM_MEMORY_INDEX);
#elif EEPROM_USED_MEMORY == EEPROM_AT24C512
  memoryBank = new AT24C512(EEPROM_MEMORY_INDEX);
#endif  
}
//--------------------------------------------------------------------------------------------------------------------------------
uint8_t MemRead(unsigned int address)
{
  #if EEPROM_USED_MEMORY == EEPROM_BUILTIN
    return EEPROM.read(address);
  #else
     return memoryBank->read(address);
  #endif      

}
//--------------------------------------------------------------------------------------------------------------------------------
void MemWrite(unsigned int address, uint8_t val)
{
  #if EEPROM_USED_MEMORY == EEPROM_BUILTIN
    EEPROM.write(address, val);
  #else
    memoryBank->write(address,val);
  #endif
}
//--------------------------------------------------------------------------------------------------------------------------------
void MemClear()
{
  uint32_t len = 0;
  
  #if EEPROM_USED_MEMORY == EEPROM_BUILTIN
    len = EEPROM.length();
  #elif EEPROM_USED_MEMORY == EEPROM_AT24C32
     len = 4*1024;
  #elif EEPROM_USED_MEMORY == EEPROM_AT24C64
    len = 8*1024;
  #elif EEPROM_USED_MEMORY == EEPROM_AT24C128
   len = 16*1024;
  #elif EEPROM_USED_MEMORY == EEPROM_AT24C256
   len = 32*1024;
  #elif EEPROM_USED_MEMORY == EEPROM_AT24C512
    len = 64*1024;
  #endif  

  for(uint32_t address=0;address<len;address++)
  {
    #if EEPROM_USED_MEMORY == EEPROM_BUILTIN
      EEPROM.write(address, 0xFF);
    #else
       memoryBank->write(address,0xFF);
    #endif

    #ifdef USE_EXTERNAL_WATCHDOG
    updateExternalWatchdog();
    #endif
       
  } // for

    #if EEPROM_USED_MEMORY == EEPROM_BUILTIN
      EEPROM.write(0, MEM_CONTROL_BYTE);
    #else
       memoryBank->write(0,MEM_CONTROL_BYTE);
    #endif    
  
}
//--------------------------------------------------------------------------------------------------------------------------------

