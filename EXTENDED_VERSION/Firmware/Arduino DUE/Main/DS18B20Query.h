#pragma once

#include <Arduino.h>
#include "TinyVector.h"
#include "AbstractModule.h"
//--------------------------------------------------------------------------------------------------------------------------------------
/*
 Используемые типы датчиков, известные на линии 1-Wire. Если к типу нет комментария - это значение лучше не использовать в эмуляторе!!!
    0x01
    0x05
    0x09
    0x0F
    0x10  - DS18S20, температурный датчик, 9 байт скратчпад с данными
    0x1D
    0x20
    0x22 - DS1822, температурный датчик, 9 байт скратчпад с данными
    0x23
    0x26
    0x28 - DS18B20, температурный датчик, 9 байт скратчпад с данными
    0x29
    0x2C
    0x2D
    0x3A
    0xA0 - датчик влажности почвы, 9 байт скратчпад с данными точно такой же, как у DS18B20. Используется эмуляция DS18B20
    0xA1 - датчик влажности и температуры, 9 байт скратчпад с данными точно такой же, как у DS18B20. Используется эмуляция DS18B20
    0xA2 - датчик освещённости, скратчпад, 9 байт скратчпад с данными точно такой же, как у DS18B20. Используется эмуляция DS18B20
    0xFC
 */
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  bool Negative;
  int Whole;
  int Fract;
  
} DS18B20Temperature;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef struct
{
  DS18B20Temperature TemperatureValue;
  Humidity HumidityValue;
  
} HumidityCompositeData;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef enum
{
  temp9bit = 0x1F,
  temp10bit = 0x3F,
  temp11bit = 0x5F,
  temp12bit = 0x7F
  
} DS18B20Resolution;
//--------------------------------------------------------------------------------------------------------------------------------------
// класс диспетчера линии DS18B20
//--------------------------------------------------------------------------------------------------------------------------------------
#pragma pack(push,1)
typedef struct
{
	uint8_t pin;
	uint8_t index;
	uint8_t address[8];
	
} DS18B20Binding;
#pragma pack(pop)
//--------------------------------------------------------------------------------------------------------------------------------------
typedef Vector<DS18B20Binding> DS18B20BindingList;
//--------------------------------------------------------------------------------------------------------------------------------------
typedef uint8_t OneWireAddress[8];
//--------------------------------------------------------------------------------------------------------------------------------------
class DS18B20Dispatcher
{
	private:
		
		Vector<uint8_t> conversionList;
		bool alreadyConverted(uint8_t pin);

		Vector<uint8_t> resolutionList;
		bool alreadySetResolution(uint8_t pin);

		Vector<uint8_t> scanList;
		bool alreadyScanned(uint8_t pin);
		
		DS18B20BindingList scanResults;
		DS18B20BindingList savedBindings;
		
		int findBinding(uint8_t sensorIndex, uint8_t pin);
		bool inScanResults(uint8_t pin, const DS18B20Binding& binding);
		
		void resetBinding(int recordIndex);
		void saveBinding(int recordIndex);
		
		int findFree(uint8_t pin, int& scanResultsIndex);

    int baseEEPROMAddress;
		
	public:
		DS18B20Dispatcher();
		
		void begin(int eepromEddress);
		void addBinding(uint8_t pin, uint8_t sensorIndex);
		
		void beginConversion();
		void startConversion(uint8_t pin);
		
		void beginSetResolution();
		void setResolution(uint8_t pin,DS18B20Resolution res);
		
		void beginScan();
		void scan(uint8_t pin);
		
		bool getTemperature(uint8_t sensorIndex, uint8_t pin, DS18B20Temperature& result);

    // возвращает тип датчика на линии, его адрес, в переменных resultType и resultAddress. Если датчик с такими параметрами не найден, то возвращает false
    bool getType(uint8_t sensorIndex, uint8_t pin, ModuleStates& resultType, OneWireAddress& resultAddress);

    // возвращает длину данных, необходимых для чтения с датчика
    size_t getRawDataLength(ModuleStates sensorType);

    // возвращает сырые данные с датчика
    bool getRawData(uint8_t pin, OneWireAddress& address, ModuleStates sensorType, uint8_t* resultData);

    // возвращает сырые данные, как температуру
    DS18B20Temperature asTemperature(uint8_t* data, size_t dataLength, OneWireAddress& address);
    
    // возвращает сырые данные, как освещённость
    long asLuminosity(uint8_t* data, size_t dataLength, OneWireAddress& address);

    HumidityCompositeData asHumidity(uint8_t* data, size_t dataLength, OneWireAddress& address);
	
};
//--------------------------------------------------------------------------------------------------------------------------------------
extern DS18B20Dispatcher DS18B20LineManager;
//--------------------------------------------------------------------------------------------------------------------------------------

