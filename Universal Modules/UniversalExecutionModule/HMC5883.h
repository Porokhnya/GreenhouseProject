#ifndef HMC5883_CLASS
#define HMC5883_CLASS

#include "Arduino.h"
#include "Wire.h"

#define QMC5883_ADDR 0x0D


//REG CONTROL

//0x09

#define Mode_Standby    0b00000000
#define Mode_Continuous 0b00000001

#define ODR_10Hz        0b00000000
#define ODR_50Hz        0b00000100
#define ODR_100Hz       0b00001000
#define ODR_200Hz       0b00001100

#define RNG_2G          0b00000000
#define RNG_8G          0b00010000

#define OSR_512         0b00000000
#define OSR_256         0b01000000
#define OSR_128         0b10000000
#define OSR_64          0b11000000


class HMC5883{
public:


  HMC5883();
void setAddress(uint8_t addr);

void init(); //init qmc5883

void setMode(uint8_t mode,uint8_t odr,uint8_t rng,uint8_t osr); // setting

void softReset(); //soft RESET

void read(int* x,int* y,int* z); //reading
void read(int* x,int* y,int* z,int* a);
void read(int* x,int* y,int* z,float* a);

float azimuth(int* a,int* b);

private:

void WriteReg(uint8_t Reg,uint8_t val);
uint8_t address;

};



#endif
