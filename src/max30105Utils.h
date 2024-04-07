#include <stdio.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/i2c.h"

class max30105Utils {
public:
    void setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange);
    bool begin(uint8_t address);
    void softReset(void);
    uint16_t check(void);
    float readTemperature();
    void wakeUp(void);
    void setLEDMode(uint8_t mode);
    void setADCRange(uint8_t adcRange);
    void setSampleRate(uint8_t sampleRate);
    void setPulseWidth(uint8_t pulseWidth);
    void setPulseAmplitudeRed(uint8_t amplitude);
    void setPulseAmplitudeIR(uint8_t amplitude);
    void setPulseAmplitudeGreen(uint8_t amplitude);
    void setPulseAmplitudeProximity(uint8_t amplitude);
    void setProximityThreshold(uint8_t threshMSB);
    void enableSlot(uint8_t slotNumber, uint8_t device);
    void disableSlots(void);
    void setFIFOAverage(uint8_t numberOfSamples);
    void clearFIFO(void);
    void enableFIFORollover(void);
    void disableFIFORollover(void);
    void setFIFOAlmostFull(uint8_t numberOfSamples);
    uint8_t getWritePointer(void);
    uint8_t getReadPointer(void);
    uint32_t getRed(void);
    void bitMask(uint8_t reg, uint8_t mask, uint8_t thing);
    uint8_t readRegister8(uint8_t address, uint8_t reg);
    void writeRegister8(uint8_t address, uint8_t reg, uint8_t value);
    void shutDown(void);
    uint32_t getIR(void);
    uint32_t getGreen(void);
    uint32_t getFIFORed(void);
    uint32_t getFIFOIR(void);
    uint32_t getFIFOGreen(void);
    uint8_t available(void);
    void nextSample(void);
    bool safeCheck(uint8_t maxTimeToCheck);

private:
    uint8_t activeLEDs;
    uint8_t revisionId;
    uint8_t m_address;
public:
    #define STORAGE_SIZE 4 //Each long is 4 bytes so limit this to fit on your micro
    typedef struct Record
    {
        uint32_t red[STORAGE_SIZE];
        uint32_t IR[STORAGE_SIZE];
        uint32_t green[STORAGE_SIZE];
        uint8_t head;
        uint8_t tail;
    } sense_struct; //This is our circular buffer of readings from the sensor

    sense_struct sense;

};