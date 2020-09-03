#include "HMC5883.h"

#include <Wire.h>

HMC5883::HMC5883()
{
 address = QMC5883_ADDR; 
}

void HMC5883::setAddress(uint8_t addr){
  address = addr;
}

void HMC5883::WriteReg(byte Reg,byte val){
  Wire.beginTransmission(address); //start talking
  Wire.write(Reg); // Tell the HMC5883 to Continuously Measure
  Wire.write(val); // Set the Register
  Wire.endTransmission();
}

void HMC5883::init(){
  WriteReg(0x0B,0x01);
  //Define Set/Reset period
  setMode(Mode_Continuous,ODR_200Hz,RNG_8G,OSR_512);
  /*
  Define
  OSR = 512
  Full Scale Range = 8G(Gauss)
  ODR = 200HZ
  set continuous measurement mode
  */
}

void HMC5883::setMode(uint8_t mode,uint8_t odr,uint8_t rng,uint8_t osr)
{
  WriteReg(0x09,mode|odr|rng|osr);
}


void HMC5883::softReset()
{
  WriteReg(0x0A,0x80);
}

void HMC5883::read(int* x,int* y,int* z)
{
  Wire.beginTransmission(address);
  Wire.write(0x00);
  Wire.endTransmission();
  if(Wire.requestFrom(address, (uint8_t)6) == 6)
  {
    *x = Wire.read(); //LSB  x
    *x |= Wire.read() << 8; //MSB  x
    *y = Wire.read(); //LSB  z
    *y |= Wire.read() << 8; //MSB z
    *z = Wire.read(); //LSB y
    *z |= Wire.read() << 8; //MSB y
  }
}

void HMC5883::read(int* x,int* y,int* z,int* a)
{
  read(x,y,z);
  *a = azimuth(y,x);
}

void HMC5883::read(int* x,int* y,int* z,float* a)
{
  read(x,y,z);
  *a = azimuth(y,x);
}


float HMC5883::azimuth(int *a, int *b)
{
  float azimuth = atan2((int)*a,(int)*b) * 180.0/PI;
  return azimuth < 0 ? 360 + azimuth : azimuth;
}
