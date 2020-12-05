#pragma once
//--------------------------------------------------------------------------------------------------------------------------------
#include <Arduino.h>
//--------------------------------------------------------------------------------------------------------------------------------
#define NUM_CHANNELS 1
#define ADC_CHANNELS ADC_CHER_CH15
#define BUFFER_SIZE 50*NUM_CHANNELS
#define NUMBER_OF_BUFFERS 2
//--------------------------------------------------------------------------------------------------------------------------------
class ADCSampler 
{
  public:
    ADCSampler();
    void begin(unsigned int samplingRate);
    void end();
    void handleInterrupt();
    bool available();
    unsigned int getSamplingRate();
    uint16_t* getFilledBuffer(int *bufferLength);
    void readBufferDone();
  private:
    unsigned int sampleingRate;
    volatile bool dataReady;
    uint16_t adcBuffer[NUMBER_OF_BUFFERS][BUFFER_SIZE];
    unsigned int adcDMAIndex;        //!< This hold the index of the next DMA buffer
    unsigned int adcTransferIndex;   //!< This hold the last filled buffer

};
//--------------------------------------------------------------------------------------------------------------------------------


