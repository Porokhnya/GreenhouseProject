#include "DS18B20Query.h"
#include "Globals.h"
#include <OneWire.h>
#include "AbstractModule.h"
#include "Memory.h"
//--------------------------------------------------------------------------------------------------------------------------------------
DS18B20Dispatcher DS18B20LineManager;
//--------------------------------------------------------------------------------------------------------------------------------------
DS18B20Dispatcher::DS18B20Dispatcher()
{
	baseEEPROMAddress = 0;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::begin(int eepromAddr)
{
  baseEEPROMAddress = eepromAddr;
	savedBindings.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::addBinding(uint8_t pin, uint8_t sensorIndex)
{
	DS18B20Binding bnd;
	memset(&bnd,0xFF,sizeof(DS18B20Binding));
	
	bnd.pin = pin;
	bnd.index = sensorIndex;
	
	// теперь вычитываем с EEPROM сохранённый адрес для этой записи
	uint32_t readAddr = baseEEPROMAddress + savedBindings.size()*sizeof(DS18B20Binding);
	DS18B20Binding saved;
	uint8_t* writePtr = (uint8_t*)&saved;
	for(size_t i=0;i<sizeof(DS18B20Binding);i++)
	{
		*writePtr++ = MemRead(readAddr++);
	}
	
	// копируем сохранённый адрес
	memcpy(bnd.address,saved.address,sizeof(saved.address));
	
	savedBindings.push_back(bnd);
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::alreadyConverted(uint8_t pin)
{
	for(size_t i=0;i<conversionList.size();i++)
	{
		if(conversionList[i] == pin)
    {
			return true;
    }
	}
	return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::beginConversion()
{
 #ifdef DS18B20_DEBUG
	DEBUG_LOGLN(F("DS18B20: begin conversion..."));
 #endif
	
	conversionList.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::startConversion(uint8_t pin)
{
	if(!pin)
  {
		return;
  }
	
	if(alreadyConverted(pin))
	{	
		return;
	}
#ifdef DS18B20_DEBUG	
	DEBUG_LOG(F("DS18B20: start conversion on pin "));
	DEBUG_LOGLN(String(pin));
#endif
	
	conversionList.push_back(pin);

	OneWire ow(pin);

	if(!ow.reset()) // нет датчика
	{
#ifdef DS18B20_DEBUG  
		DEBUG_LOG(F("DS18B20: no sensors found on pin "));
		DEBUG_LOGLN(String(pin));
#endif
		return;
	}

#ifdef DS18B20_DEBUG
	DEBUG_LOG(F("DS18B20: start conversion on pin "));
	DEBUG_LOGLN(String(pin));
#endif

	ow.write(0xCC); // пофиг на адреса (SKIP ROM)
	ow.write(0x44); // запускаем преобразование

	ow.reset();		
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::alreadySetResolution(uint8_t pin)
{
	for(size_t i=0;i<resolutionList.size();i++)
	{
		if(resolutionList[i] == pin)
    {
			return true;
    }
	}
	return false;	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::beginSetResolution()
{
#ifdef DS18B20_DEBUG  
	DEBUG_LOGLN(F("DS18B20: begin set resolution..."));
#endif
	
	resolutionList.empty();	
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::setResolution(uint8_t pin,DS18B20Resolution res)
{
	if(!pin)
  {
		return;
  }

if(alreadySetResolution(pin))
 {
		return;
 }
#ifdef DS18B20_DEBUG	
	DEBUG_LOG(F("DS18B20: set resolution on pin "));
	DEBUG_LOGLN(String(pin));
#endif
  
	resolutionList.push_back(pin);	

	OneWire ow(pin);


	if(!ow.reset()) // нет датчика
	{
#ifdef DS18B20_DEBUG  
		DEBUG_LOG(F("DS18B20: no sensors found on pin "));
  	DEBUG_LOGLN(String(pin));		
#endif		
		return;  
	}

	ow.write(0xCC); // пофиг на адреса (SKIP ROM)
	ow.write(0x4E); // запускаем запись в scratchpad

	ow.write(0); // верхний температурный порог 
	ow.write(0); // нижний температурный порог
	ow.write(res); // разрешение датчика

	ow.reset();
	ow.write(0xCC); // пофиг на адреса (SKIP ROM)
	ow.write(0x48); // COPY SCRATCHPAD
	delay(10);
	ow.reset();	
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::alreadyScanned(uint8_t pin)
{
	for(size_t i=0;i<scanList.size();i++)
	{
		if(scanList[i] == pin)
    {
			return true;
    }
	}
	return false;		
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::beginScan()
{
#ifdef DS18B20_DEBUG  
	DEBUG_LOGLN(F("DS18B20: begin scan lines..."));
#endif
	
	scanList.empty();
	scanResults.empty();
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::scan(uint8_t pin)
{
	if(!pin)
  {
		return;
  }

	if(alreadyScanned(pin))
  {
		return;
  }
#ifdef DS18B20_DEBUG	
	DEBUG_LOG(F("DS18B20: scan on pin "));
	DEBUG_LOGLN(String(pin));
#endif
	
	scanList.push_back(pin);	

	OneWire ow(pin);

	if(!ow.reset()) // нет датчика
	{
#ifdef DS18B20_DEBUG  
		DEBUG_LOG(F("DS18B20: no sensors found on pin "));
		DEBUG_LOGLN(String(pin));		
#endif		
		return;  
	}

	// тут сканируем линию
  byte addr[8];
  ow.reset_search();
  
  while(ow.search(addr))
  {
	  // получили следующий адрес на линии
	  if(OneWire::crc8(addr, 7) == addr[7])
	  {
		  // хороший адрес, надо сохранить этот адрес у себя, чтобы потом его использовать
		  DS18B20Binding bnd;
		  memset(&bnd,0xFF,sizeof(DS18B20Binding));
		  bnd.pin = pin;
		  memcpy(bnd.address,addr,sizeof(addr));
      
#ifdef DS18B20_DEBUG		  
		  DEBUG_LOG(F("DS18B20: found sensor on pin "));
		  DEBUG_LOG(String(pin));
		  DEBUG_LOG(F(" with address "));
		  for(size_t i=0;i<sizeof(addr);i++)
		  {
			  DEBUG_LOG(String(addr[i],HEX));
			  DEBUG_LOG(" ");
		  }
		  
		  DEBUG_LOGLN("");
#endif // DS18B20_DEBUG
		  
		  scanResults.push_back(bnd);
	  }
	  
  } // while

#ifdef DS18B20_DEBUG  
  DEBUG_LOG(F("DS18B20: scan done, total sensors found: "));
  DEBUG_LOGLN(String(scanResults.size()));
#endif
  
  // теперь у нас есть набор адресов датчиков, которые отзываются, на каждом пине из настроек

 	
}
//--------------------------------------------------------------------------------------------------------------------------------------
int DS18B20Dispatcher::findBinding(uint8_t sensorIndex, uint8_t pin)
{
	uint8_t address[8];
	memset(address,0xFF,sizeof(address));
	
	for(size_t i=0;i<savedBindings.size();i++)
	{
		if(savedBindings[i].pin == pin && savedBindings[i].index == sensorIndex)
		{
			if(memcmp(address,savedBindings[i].address,sizeof(address))) // есть сохранённый адрес
      {
				return i;
      }
			
			break;
		}
	}
	
	return -1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::inScanResults(uint8_t pin, const DS18B20Binding& binding)
{
	for(size_t i=0;i<scanResults.size();i++)
	{
		if(scanResults[i].pin == pin && !memcmp(binding.address,scanResults[i].address,sizeof(binding.address))) // датчик - в откликнувшихся на линии
    {
			return true;
    }
	}
	
	return false;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::resetBinding(int recordIndex)
{
	if(recordIndex < 0 || recordIndex >= savedBindings.size())
  {
		return;
  }
	
	// сбрасываем адрес в ноль
	memset(savedBindings[recordIndex].address,0xFF,sizeof(savedBindings[recordIndex].address));	
	saveBinding(recordIndex);
	
}
//--------------------------------------------------------------------------------------------------------------------------------------
int DS18B20Dispatcher::findFree(uint8_t pin, int& scanResultsIndex) // ищем первый датчик на линии, для которого отсутствуют привязки к индексу
{
	scanResultsIndex = -1;
	
	uint8_t address[8];
	memset(address,0xFF,sizeof(address));
	
	for(size_t i=0;i<scanResults.size();i++)
	{
		if(scanResults[i].pin == pin)
		{
			bool found = false;
			for(size_t k=0;k<savedBindings.size();k++)
			{
				if(savedBindings[k].pin == pin && !memcmp(scanResults[i].address,savedBindings[k].address,sizeof(scanResults[i].address)))
				{
					found = true;
					break;
				}
			} // for
			
			if(!found)
			{
				// нет датчика на линии - с таким адресом в привязках. Ищем первый свободный слот в привязках.
				for(size_t k=0;k<savedBindings.size();k++)
				{
					if(savedBindings[k].pin == pin && !memcmp(address,savedBindings[k].address,sizeof(address)))
					{
						scanResultsIndex = i;
						return k;
					}
				}
			}
		}
	} // for
	
	return -1;
}
//--------------------------------------------------------------------------------------------------------------------------------------
void DS18B20Dispatcher::saveBinding(int recordIndex)
{
	//СОХРАНЕНИЕ В EEPROM
	uint32_t writeAddr = baseEEPROMAddress + recordIndex*sizeof(DS18B20Binding);
	uint8_t* readAddr = (uint8_t*)&(savedBindings[recordIndex]);
	
	for(size_t i=0;i<sizeof(DS18B20Binding);i++)
	{
		MemWrite(writeAddr++,*readAddr++);
	}	
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::getTemperature(uint8_t sensorIndex, uint8_t pin, DS18B20Temperature& result)
{
	
	result.Whole = NO_TEMPERATURE_DATA; // нет данных с датчика
	result.Fract = 0;
	result.Negative = false;

#ifdef DS18B20_DEBUG  
	DEBUG_LOG(F("DS18B20: request temperature on pin "));
	DEBUG_LOG(String(pin));
	DEBUG_LOG(F(" and sensor index "));	
	DEBUG_LOGLN(String(sensorIndex));
#endif
	
	bool address_found = false;
	byte addr[8]; // адрес, который ищем	

	// у нас запросили температуру с датчика по индексу, и всё, что мы знаем - это пин, на котором он висит.
	// нам надо для начала посмотреть - есть ли у нас в сохранённых привязках адрес для датчика с таким пином
	// и таким индексом.
	int recordIndex = findBinding(sensorIndex,pin);
	// Если есть - то проверяем - есть ли такой датчик в откликнувшихся. Если датчик не откликнулся - то сбрасываем ему привязку к адресу.
	
	if(recordIndex != -1) // нашли в сохранённых ранее привязках адреса к индексу
	{
#ifdef DS18B20_DEBUG  
		DEBUG_LOG(F("DS18B20: found saved binding at index "));
		DEBUG_LOG(String(recordIndex));
		DEBUG_LOG(F(" address "));
		  for(size_t i=0;i<sizeof(savedBindings[recordIndex].address);i++)
		  {
			  DEBUG_LOG(String(savedBindings[recordIndex].address[i],HEX));
			  DEBUG_LOG(" ");
		  }
        DEBUG_LOGLN("");
#endif // DS18B20_DEBUG

		
		if(inScanResults(pin,savedBindings[recordIndex]))
		{
			// датчик есть в откликнувшихся на линии
#ifdef DS18B20_DEBUG      
			DEBUG_LOG(F("DS18B20: sensor exists on line "));
			DEBUG_LOGLN(String(pin));
#endif
			
			// говорим, что нашли датчик, копируем его адрес для дальнейшего опроса
			address_found = true;
			memcpy(addr,savedBindings[recordIndex].address,sizeof(addr));
		}
		else
		{
			// датчика нет в откликнувшихся на линии
#ifdef DS18B20_DEBUG			
			DEBUG_LOG(F("DS18B20: sensor NOT exists on line "));
			DEBUG_LOGLN(String(pin));
#endif
			
			// сбрасываем его привязку к адресу
			resetBinding(recordIndex);
		}
	}
	else // не нашли привязку датчика к адресу
	{
#ifdef DS18B20_DEBUG  
		DEBUG_LOG(F("DS18B20: NO BINDING FOUND FOR SENSOR INDEX "));
		DEBUG_LOGLN(String(sensorIndex));	
#endif
		// Если датчика нет в сохранённых привязках - то мы туда помещаем первый откликнувшийся на линии, но НЕ ИМЕЮЩИЙ привязок до этого.
		// по итогу - мы получаем адрес для сканирования.
		int scanResultsIndex = -1;
		int freeBindingIndex = findFree(pin, scanResultsIndex);
		
		if(scanResultsIndex > -1 && freeBindingIndex > -1)
		{
			// нашли свободный слот, сохраняем туда адрес датчика на линии
#ifdef DS18B20_DEBUG      
			DEBUG_LOG(F("DS18B20: found free slot in binding: "));
			DEBUG_LOG(String(freeBindingIndex));
			DEBUG_LOG(F("; scan index = "));
			DEBUG_LOGLN(String(scanResultsIndex));
#endif
			
			memcpy(savedBindings[freeBindingIndex].address,scanResults[scanResultsIndex].address,sizeof(savedBindings[freeBindingIndex].address));
			saveBinding(freeBindingIndex);
			
			// говорим, что нашли адрес
			address_found = true;
			memcpy(addr,scanResults[scanResultsIndex].address,sizeof(addr));
		}
	}
	
	
	if(address_found) // нашли адрес
	{
		  OneWire ow(pin);

		  if(!ow.reset()) // нет датчика
		  {
#ifdef DS18B20_DEBUG      
			DEBUG_LOG(F("DS18B20: no sensors found on pin "));
			DEBUG_LOGLN(String(pin));
#endif 
			return false;
		  }
		  
		  uint8_t type_s;

		  switch (addr[0]) 
		  {
			case 0x10:
#ifdef DS18B20_DEBUG      
			  DEBUG_LOGLN(F("DS18B20: Chip = DS18S20"));  // or old DS1820
#endif  
			  type_s = 1;
			  break;
			case 0x28:
#ifdef DS18B20_DEBUG      
				DEBUG_LOGLN(F("DS18B20: Chip = DS18B20"));
#endif  
			  type_s = 0;
			  break;
			case 0x22:
#ifdef DS18B20_DEBUG      
				DEBUG_LOGLN(F("DS18B20: Chip = DS1822"));
#endif
			  type_s = 0;
			  break;
			default:
#ifdef DS18B20_DEBUG      
				DEBUG_LOGLN(F("DS18B20: Device is not a DS18x20 family device."));
#endif
			  return false;
		  }   

		  ow.select(addr);
		  
		  ow.write(0xBE); // читаем scratchpad датчика на пине
		  byte data[9];

		  for(uint8_t i=0;i<9;i++)
			data[i] = ow.read();


		 if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
		 {
#ifdef DS18B20_DEBUG      
			 DEBUG_LOGLN(F("DS18B20: CRC NOT VALID!"));
#endif 
			  return false;
		 }

		 // Convert the data to actual temperature
		  // because the result is a 16 bit signed integer, it should
		  // be stored to an "int16_t" type, which is always 16 bits
		  // even when compiled on a 32 bit processor.
		  int16_t raw = (data[1] << 8) | data[0];
		  if (type_s) 
		  {
  			raw = raw << 3; // 9 bit resolution default
  			if (data[7] == 0x10) 
  			{
  			  // "count remain" gives full 12 bit resolution
  			  raw = (raw & 0xFFF0) + 12 - data[6];
  			}
		  } 
		  else 
		  {
  			byte cfg = (data[4] & 0x60);
  			// at lower res, the low bits are undefined, so let's zero them
  			if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  			else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  			else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  			//// default is 12 bit resolution, 750 ms conversion time
		  }
		  
		  float celsius = ((float) raw / 16.0)*100.0;
		  int32_t tc_100 = celsius;
		  
		  result.Negative = tc_100 < 0;
		  result.Whole = abs(tc_100/100);
		  result.Fract = abs(tc_100 % 100);


		  if(result.Whole < -55 || result.Whole > 125)
		  {
			result.Negative = false;
			result.Whole = NO_TEMPERATURE_DATA;
			result.Fract = 0;
		  }		  
				
	} // if(address_found) // нашли адрес
	
	return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::getType(uint8_t sensorIndex, uint8_t pin, ModuleStates& resultType, OneWireAddress& resultAddress)
{

  resultType = StateUnknown; // неизвестный науке тип датчика

  
  bool address_found = false;
  byte addr[8]; // адрес, который ищем  

  // у нас запросили температуру с датчика по индексу, и всё, что мы знаем - это пин, на котором он висит.
  // нам надо для начала посмотреть - есть ли у нас в сохранённых привязках адрес для датчика с таким пином
  // и таким индексом.
  int recordIndex = findBinding(sensorIndex,pin);
  // Если есть - то проверяем - есть ли такой датчик в откликнувшихся. Если датчик не откликнулся - то сбрасываем ему привязку к адресу.
  
  if(recordIndex != -1) // нашли в сохранённых ранее привязках адреса к индексу
  {
#ifdef DS18B20_DEBUG  
    DEBUG_LOG(F("1-Wire: found saved binding at index "));
    DEBUG_LOG(String(recordIndex));
    DEBUG_LOG(F(" address "));
      for(size_t i=0;i<sizeof(savedBindings[recordIndex].address);i++)
      {
        DEBUG_LOG(String(savedBindings[recordIndex].address[i],HEX));
        DEBUG_LOG(" ");
      }
        DEBUG_LOGLN("");
#endif // DS18B20_DEBUG

    
    if(inScanResults(pin,savedBindings[recordIndex]))
    {
      // датчик есть в откликнувшихся на линии
#ifdef DS18B20_DEBUG      
      DEBUG_LOG(F("1-Wire: sensor exists on line "));
      DEBUG_LOGLN(String(pin));
#endif
      
      // говорим, что нашли датчик, копируем его адрес для дальнейшего опроса
      address_found = true;
      memcpy(addr,savedBindings[recordIndex].address,sizeof(addr));
    }
    else
    {
      // датчика нет в откликнувшихся на линии
#ifdef DS18B20_DEBUG      
      DEBUG_LOG(F("1-Wire: sensor NOT exists on line "));
      DEBUG_LOGLN(String(pin));
#endif
      
      // сбрасываем его привязку к адресу
      resetBinding(recordIndex);
    }
  }
  else // не нашли привязку датчика к адресу
  {
#ifdef DS18B20_DEBUG  
    DEBUG_LOG(F("1-Wire: NO BINDING FOUND FOR SENSOR INDEX "));
    DEBUG_LOGLN(String(sensorIndex)); 
#endif
    // Если датчика нет в сохранённых привязках - то мы туда помещаем первый откликнувшийся на линии, но НЕ ИМЕЮЩИЙ привязок до этого.
    // по итогу - мы получаем адрес для сканирования.
    int scanResultsIndex = -1;
    int freeBindingIndex = findFree(pin, scanResultsIndex);
    
    if(scanResultsIndex > -1 && freeBindingIndex > -1)
    {
      // нашли свободный слот, сохраняем туда адрес датчика на линии
#ifdef DS18B20_DEBUG      
      DEBUG_LOG(F("1-Wire: found free slot in binding: "));
      DEBUG_LOG(String(freeBindingIndex));
      DEBUG_LOG(F("; scan index = "));
      DEBUG_LOGLN(String(scanResultsIndex));
#endif
      
      memcpy(savedBindings[freeBindingIndex].address,scanResults[scanResultsIndex].address,sizeof(savedBindings[freeBindingIndex].address));
      saveBinding(freeBindingIndex);
      
      // говорим, что нашли адрес
      address_found = true;
      memcpy(addr,scanResults[scanResultsIndex].address,sizeof(addr));
    }
  }

  // копируем результирующий адрес в результаты функции
  memcpy(resultAddress,addr,sizeof(addr));
  
  if(address_found) // нашли адрес
  {
      OneWire ow(pin);

      if(!ow.reset()) // нет датчика
      {
#ifdef DS18B20_DEBUG      
      DEBUG_LOG(F("1-Wire: no sensors found on pin "));
      DEBUG_LOGLN(String(pin));
#endif 
      return false;
      }
      
      uint8_t type_s;

      switch (addr[0]) 
      {
      case 0x10:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS18S20"));  // or old DS1820
#endif  
        resultType = StateTemperature;
        break;
        
      case 0x28:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS18B20"));
#endif  
        resultType = StateTemperature;
        break;
        
      case 0x22:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS1822"));
#endif
        resultType = StateTemperature;
        break;

     case 0xA0: // датчик влажности почвы, скратчпад идентичен DS18B20
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Soil Moisture sensor"));
#endif
        resultType = StateTemperature;
     break;

     case 0xA1: // датчик влажности и температуры, скратчпад - 9 байт
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Humidity sensor"));
#endif
        resultType = StateHumidity;
     break;

     case 0xA2: // датчик освещённости, скратчпад - 9 байт
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Luminosity sensor"));
#endif
        resultType = StateLuminosity;
     break;
     
      default:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Unknown device."));
#endif
        return false;
      } 

  } // if(address_found) // нашли адрес

  return (resultType != StateUnknown);
}
//--------------------------------------------------------------------------------------------------------------------------------------
long DS18B20Dispatcher::asLuminosity(uint8_t* data, size_t dataLength, OneWireAddress& address)
{
  long result = NO_LUMINOSITY_DATA;

  if(!data || dataLength < 9)
  {
    return result;
  }

    if(address[0] != 0xA2) // это не датчик освещённости, выдающий себя за DS18B20
    {
      return result;
    }

     if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
     {
#ifdef DS18B20_DEBUG      
       DEBUG_LOGLN(F("1-Wire: Luminosity sensor, CRC NOT VALID!"));
#endif 
        return result;
     }    

    uint8_t* p = (uint8_t*)&result;

    // просто копируем сырые данные в переменную
    *p = data[0]; p++;
    *p = data[1]; p++;
    *p = data[2]; p++;
    *p = data[3]; p++;

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
DS18B20Temperature DS18B20Dispatcher::asTemperature(uint8_t* data, size_t dataLength, OneWireAddress& address)
{
  DS18B20Temperature result;
  
  result.Whole = NO_TEMPERATURE_DATA; // нет данных с датчика
  result.Fract = 0;
  result.Negative = false;
  
  if(!data || dataLength < 9)
  {
    return result;
  }

    uint8_t type_s;

      switch (address[0]) 
      {
      case 0x10:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS18S20"));  // or old DS1820
#endif  
        type_s = 1;
        break;
      case 0x28:
      case 0xA0: // датчик влажности почвы, который идентичен DS18B20
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS18B20"));
#endif  
        type_s = 0;
        break;
      case 0x22:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Chip = DS1822"));
#endif
        type_s = 0;
        break;

      default:
#ifdef DS18B20_DEBUG      
        DEBUG_LOGLN(F("1-Wire: Device is not a DS18x20 family device."));
#endif
        return result;
      }  

if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
     {
#ifdef DS18B20_DEBUG      
       DEBUG_LOGLN(F("1-Wire: Temperature sensor, CRC NOT VALID!"));
#endif 
        return result;
     }

     // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
      if (type_s) 
      {
        raw = raw << 3; // 9 bit resolution default
        if (data[7] == 0x10) 
        {
          // "count remain" gives full 12 bit resolution
          raw = (raw & 0xFFF0) + 12 - data[6];
        }
      } 
      else 
      {
        byte cfg = (data[4] & 0x60);
        // at lower res, the low bits are undefined, so let's zero them
        if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
        else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
        else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
        //// default is 12 bit resolution, 750 ms conversion time
      }
      
      float celsius = ((float) raw / 16.0)*100.0;
      int32_t tc_100 = celsius;
      
      result.Negative = tc_100 < 0;
      result.Whole = abs(tc_100/100);
      result.Fract = abs(tc_100 % 100);


      if(result.Whole < -55 || result.Whole > 125)
      {
      result.Negative = false;
      result.Whole = NO_TEMPERATURE_DATA;
      result.Fract = 0;
      }     

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
HumidityCompositeData DS18B20Dispatcher::asHumidity(uint8_t* data, size_t dataLength, OneWireAddress& address)
{
  HumidityCompositeData result;
  
  result.TemperatureValue.Whole = NO_TEMPERATURE_DATA; // нет данных с датчика
  result.TemperatureValue.Fract = 0;
  result.TemperatureValue.Negative = false;
  
  if(!data || dataLength < 9)
  {
    return result;
  }

    if(address[0] != 0xA1) // это не датчик влажности, выдающий себя за DS18B20
    {
      return result;
    }  


     if (OneWire::crc8( data, 8) != data[8]) // проверяем контрольную сумму
     {
#ifdef DS18B20_DEBUG      
       DEBUG_LOGLN(F("1-Wire: Humidity sensor, CRC NOT VALID!"));
#endif 
        return result;
     }

     // Convert the data to actual temperature
      // because the result is a 16 bit signed integer, it should
      // be stored to an "int16_t" type, which is always 16 bits
      // even when compiled on a 32 bit processor.
      int16_t raw = (data[1] << 8) | data[0];
            
      float celsius = ((float) raw / 16.0)*100.0;
      int32_t tc_100 = celsius;
      
      result.TemperatureValue.Negative = tc_100 < 0;
      result.TemperatureValue.Whole = abs(tc_100/100);
      result.TemperatureValue.Fract = abs(tc_100 % 100);


      if(result.TemperatureValue.Whole < -55 || result.TemperatureValue.Whole > 125)
      {
        result.TemperatureValue.Negative = false;
        result.TemperatureValue.Whole = NO_TEMPERATURE_DATA;
        result.TemperatureValue.Fract = 0;
      }  

      result.HumidityValue.Value = data[2];
      result.HumidityValue.Fract = data[3];

    return result;
}
//--------------------------------------------------------------------------------------------------------------------------------------
size_t DS18B20Dispatcher::getRawDataLength(ModuleStates sensorType)
{
  switch(sensorType)
  {
    case StateTemperature:
    case StateSoilMoisture:
    case StatePH:
      return 9;
      
    case StateWaterFlowIncremental:
    case StateWaterFlowInstant:
    case StateHumidity:
    case StateLuminosity:
    case StateCO2:
    case StateEC:
      return 9;

    default:
      return 0;
  }
}
//--------------------------------------------------------------------------------------------------------------------------------------
bool DS18B20Dispatcher::getRawData(uint8_t pin, OneWireAddress& address, ModuleStates sensorType, uint8_t* resultData)
{
  if(!resultData || sensorType == StateUnknown)
  {
    return false;
  }

  size_t dataLen = getRawDataLength(sensorType);
  if(dataLen < 1)
  {
    return false;
  }

      OneWire ow(pin);

      if(!ow.reset()) // нет датчика
      {
#ifdef DS18B20_DEBUG      
      DEBUG_LOG(F("1-Wire: no sensors found on pin "));
      DEBUG_LOGLN(String(pin));
#endif 
      return false;
      }
   
      ow.select(address);
      
      ow.write(0xBE); // читаем scratchpad датчика на пине

      for(size_t i=0;i<dataLen;i++)
      {
          resultData[i] = ow.read();
      }

   return true;
}
//--------------------------------------------------------------------------------------------------------------------------------------

