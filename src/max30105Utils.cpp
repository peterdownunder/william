#include "max30105Utils.h"
#include "max30105const.h"
#include <cstddef>
#include <memory.h>

#define RetainBusControl true
#define ReleaseBusControl false
#define I2C_BUFFER_LENGTH 32

unsigned int reverse_bit32(unsigned int x)
{
	x = ((x & 0x55555555) << 1) | ((x & 0xAAAAAAAA) >> 1);
	x = ((x & 0x33333333) << 2) | ((x & 0xCCCCCCCC) >> 2);
	x = ((x & 0x0F0F0F0F) << 4) | ((x & 0xF0F0F0F0) >> 4);
	x = ((x & 0x00FF00FF) << 8) | ((x & 0xFF00FF00) >> 8);
	return (x << 16) | (x >> 16);
}
bool max30105Utils::begin(uint8_t address)
{
  uint8_t chipid[1];

  i2c_write_blocking(i2c_default, address, &MAX30105_PARTID, 1, RetainBusControl);
  i2c_read_blocking(i2c_default, address, chipid, 1, ReleaseBusControl);
  printf("device id:0x%02x\n", chipid[0]);

  if (chipid[0] != MAX30105_CHIPID)
    return false;

  // reset i2s
  uint8_t reset[2];
  uint8_t resetstate[1];
  reset[0] = MAX30105_MODECONFIG;
  reset[1] = MAX30105_RESET;
  i2c_write_blocking(i2c_default, address, reset, 2, RetainBusControl);
  while (true)
  {
    i2c_read_blocking(i2c_default, address, resetstate, 1, ReleaseBusControl);
    if (resetstate[0] == 0xf)
    {
      revisionId = readRegister8(address, MAX30105_REVISIONID);
      m_address = address;
      return true;
    }
  }
}
void max30105Utils::setup(uint8_t powerLevel, uint8_t sampleAverage, uint8_t ledMode, int sampleRate, int pulseWidth, int adcRange)
{
  softReset(); // Reset all configuration, threshold, and data registers to POR values

  // FIFO Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // The chip will average multiple samples of same type together if you wish
  if (sampleAverage == 1)
    setFIFOAverage(MAX30105_SAMPLEAVG_1); // No averaging per FIFO record
  else if (sampleAverage == 2)
    setFIFOAverage(MAX30105_SAMPLEAVG_2);
  else if (sampleAverage == 4)
    setFIFOAverage(MAX30105_SAMPLEAVG_4);
  else if (sampleAverage == 8)
    setFIFOAverage(MAX30105_SAMPLEAVG_8);
  else if (sampleAverage == 16)
    setFIFOAverage(MAX30105_SAMPLEAVG_16);
  else if (sampleAverage == 32)
    setFIFOAverage(MAX30105_SAMPLEAVG_32);
  else
    setFIFOAverage(MAX30105_SAMPLEAVG_4);

  // setFIFOAlmostFull(2); //Set to 30 samples to trigger an 'Almost Full' interrupt
  enableFIFORollover(); // Allow FIFO to wrap/roll over

  // Mode Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (ledMode == 3)
    setLEDMode(MAX30105_MODE_MULTILED); // Watch all three LED channels
  else if (ledMode == 2)
    setLEDMode(MAX30105_MODE_REDIRONLY); // Red and IR
  else
    setLEDMode(MAX30105_MODE_REDONLY); // Red only
  activeLEDs = ledMode;                // Used to control how many uint8_ts to read from FIFO buffer

  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Particle Sensing Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  if (adcRange < 4096)
    setADCRange(MAX30105_ADCRANGE_2048); // 7.81pA per LSB
  else if (adcRange < 8192)
    setADCRange(MAX30105_ADCRANGE_4096); // 15.63pA per LSB
  else if (adcRange < 16384)
    setADCRange(MAX30105_ADCRANGE_8192); // 31.25pA per LSB
  else if (adcRange == 16384)
    setADCRange(MAX30105_ADCRANGE_16384); // 62.5pA per LSB
  else
    setADCRange(MAX30105_ADCRANGE_2048);

  if (sampleRate < 100)
    setSampleRate(MAX30105_SAMPLERATE_50); // Take 50 samples per second
  else if (sampleRate < 200)
    setSampleRate(MAX30105_SAMPLERATE_100);
  else if (sampleRate < 400)
    setSampleRate(MAX30105_SAMPLERATE_200);
  else if (sampleRate < 800)
    setSampleRate(MAX30105_SAMPLERATE_400);
  else if (sampleRate < 1000)
    setSampleRate(MAX30105_SAMPLERATE_800);
  else if (sampleRate < 1600)
    setSampleRate(MAX30105_SAMPLERATE_1000);
  else if (sampleRate < 3200)
    setSampleRate(MAX30105_SAMPLERATE_1600);
  else if (sampleRate == 3200)
    setSampleRate(MAX30105_SAMPLERATE_3200);
  else
    setSampleRate(MAX30105_SAMPLERATE_50);

  // The longer the pulse width the longer range of detection you'll have
  // At 69us and 0.4mA it's about 2 inches
  // At 411us and 0.4mA it's about 6 inches
  if (pulseWidth < 118)
    setPulseWidth(MAX30105_PULSEWIDTH_69); // Page 26, Gets us 15 bit resolution
  else if (pulseWidth < 215)
    setPulseWidth(MAX30105_PULSEWIDTH_118); // 16 bit resolution
  else if (pulseWidth < 411)
    setPulseWidth(MAX30105_PULSEWIDTH_215); // 17 bit resolution
  else if (pulseWidth == 411)
    setPulseWidth(MAX30105_PULSEWIDTH_411); // 18 bit resolution
  else
    setPulseWidth(MAX30105_PULSEWIDTH_69);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // LED Pulse Amplitude Configuration
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  // Default is 0x1F which gets us 6.4mA
  // powerLevel = 0x02, 0.4mA - Presence detection of ~4 inch
  // powerLevel = 0x1F, 6.4mA - Presence detection of ~8 inch
  // powerLevel = 0x7F, 25.4mA - Presence detection of ~8 inch
  // powerLevel = 0xFF, 50.0mA - Presence detection of ~12 inch

  setPulseAmplitudeRed(powerLevel);
  setPulseAmplitudeIR(powerLevel);
  setPulseAmplitudeGreen(powerLevel);
  setPulseAmplitudeProximity(powerLevel);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  // Multi-LED Mode Configuration, Enable the reading of the three LEDs
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-
  enableSlot(1, SLOT_RED_LED);
  if (ledMode > 1)
    enableSlot(2, SLOT_IR_LED);
  if (ledMode > 2)
    enableSlot(3, SLOT_GREEN_LED);
  // enableSlot(1, SLOT_RED_PILOT);
  // enableSlot(2, SLOT_IR_PILOT);
  // enableSlot(3, SLOT_GREEN_PILOT);
  //-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-

  clearFIFO(); // Reset the FIFO before we begin checking the sensor
}
uint16_t max30105Utils::check(void)
{
#pragma pack(1)
  union dataBuffer {
    int8_t readBuffer[128];
    struct {
      uint Red : 18;
      uint DeadZone2: 6;
      uint InfraRed : 18;
      uint DeadZone1: 6;
      uint Green : 18;
      uint DeadZone3: 6;
    } Levels;
  }db;

  #pragma pack()
  int yy = sizeof(db.Levels);
  printf("%d %d %d\n", yy, sizeof(db.readBuffer),  sizeof(db));  
  uint8_t readPointer = getReadPointer();
  uint8_t writePointer = getWritePointer();
  int numberOfSamples = 0;
  // Do we have new data?
  if (readPointer != writePointer)
  {
    // Calculate the number of readings we need to get from sensor
    numberOfSamples = writePointer - readPointer;
    if (numberOfSamples < 0)
      numberOfSamples += 32; // Wrap condition

    // We now have the number of readings, now calc bytes to read
    // For this example we are just doing Red and IR (3 bytes each)
    int bytesLeftToRead = numberOfSamples * activeLEDs * 3;

    i2c_write_blocking(i2c_default, m_address, &MAX30105_FIFODATA, 1, RetainBusControl);

    while (bytesLeftToRead > 0)
    {
      int toGet = bytesLeftToRead;
      if (toGet > I2C_BUFFER_LENGTH)
      {
        toGet = I2C_BUFFER_LENGTH - (I2C_BUFFER_LENGTH % (activeLEDs * 3)); // Trim toGet to be a multiple of the samples we need to read
      }
      bytesLeftToRead -= toGet;

      while (toGet > 0)
      {
        uint8_t readBuffer[128]; // Array of 4 bytes that we will convert into long
        uint8_t temp[4]; // Array of 4 bytes that we will convert into long
        uint32_t tempIR;
        uint32_t tempRed;
        uint32_t tempGreen;
        memset(temp, 0, sizeof(temp));
        sense.head++;               // Advance the head of the storage struct
        sense.head %= STORAGE_SIZE; // Wrap condition
        i2c_read_blocking(i2c_default, m_address, readBuffer, toGet, ReleaseBusControl);
        int r = 0;
        temp[3] = 0;
        temp[2] = readBuffer[r++];
        temp[1] = readBuffer[r++];
        temp[0] = readBuffer[r++];
        memcpy(&tempRed, temp, 3);
        tempRed &= 0x3FFFF;              // Zero out all but 18 bits
        sense.red[sense.head] = tempRed; // Store this reading into the sense array
        if (activeLEDs > 1)
        {
          temp[3] = 0;
          temp[2] = readBuffer[r++];
          temp[1] = readBuffer[r++];
          temp[0] = readBuffer[r++];
          memcpy(&tempIR, temp, sizeof(tempIR));
          tempIR &= 0x3FFFF; // Zero out all but 18 bits
          sense.IR[sense.head] = tempIR;
        }
        if (activeLEDs > 2)
        {
          temp[3] = 0;
          temp[2] = readBuffer[r++];
          temp[1] = readBuffer[r++];
          temp[0] = readBuffer[r++];
          memcpy(&tempGreen, temp, sizeof(tempGreen));
          tempGreen &= 0x3FFFF; // Zero out all but 18 bits
          sense.green[sense.head] = tempGreen;
        }
        memccpy(db.readBuffer, readBuffer, 128, sizeof(db.readBuffer));
        int32_t ir = db.Levels.InfraRed;
        int32_t rr = reverse_bit32(db.Levels.InfraRed);
        printf("Red: %d Green: %d InfraRed: %d\n", ir, rr, tempIR);
        toGet -= activeLEDs * 3;
      }
    }
  }
  return numberOfSamples;
}

void max30105Utils::softReset(void)
{

  bitMask(MAX30105_MODECONFIG, MAX30105_RESET_MASK, MAX30105_RESET);

  // Poll for bit to clear, reset is then complete
  // Timeout after 100ms
  uint32_t msSinceBoot = to_ms_since_boot(get_absolute_time());
  while (to_ms_since_boot(get_absolute_time()) - msSinceBoot < 100)
  {
    uint8_t response = readRegister8(m_address, MAX30105_MODECONFIG);
    if ((response & MAX30105_RESET) == 0)
      break;     // We're done!
    sleep_ms(1); // Let's not over burden the I2C bus
  }
}

void max30105Utils::setFIFOAverage(uint8_t numberOfSamples)
{
  bitMask(MAX30105_FIFOCONFIG, MAX30105_SAMPLEAVG_MASK, numberOfSamples);
}

void max30105Utils::bitMask(uint8_t reg, uint8_t mask, uint8_t thing)
{
  uint8_t readByte[1];
  i2c_write_blocking(i2c_default, m_address, &reg, 1, RetainBusControl);
  i2c_read_blocking(i2c_default, m_address, readByte, 1, ReleaseBusControl);

  // Zero-out the portions of the register we're interested in
  uint8_t originalContents = readByte[0] & mask;

  // Change contents
  writeRegister8(m_address, reg, originalContents | thing);
}

uint8_t max30105Utils::readRegister8(uint8_t address, uint8_t reg)
{
  uint8_t readByte[1];
  readByte[0] = 0;
  i2c_write_blocking(i2c_default, address, &reg, 1, ReleaseBusControl);

  // uint8_t avail = i2c_get_read_available(i2c_default);
  // printf ("%d", avail);
  // int tries = 0;
  // while (avail == 0)
  // {
  //   sleep_ms(5);
  //   if (tries++ > 200) break;
  //   avail = i2c_get_read_available(i2c_default);
  // }
  //  if (tries >= 200) return (0); //Fail

  i2c_read_blocking(i2c_default, address, readByte, 1, ReleaseBusControl);
  //  printf ("%d", avail);
  return (readByte[0]);
}

void max30105Utils::writeRegister8(uint8_t address, uint8_t reg, uint8_t value)
{
  uint8_t data[2] = {reg, value};
  i2c_write_blocking(i2c_default, address, data, 2, ReleaseBusControl);
}

void max30105Utils::shutDown(void)
{
  // Put IC into low power mode (datasheet pg. 19)
  // During shutdown the IC will continue to respond to I2C commands but will
  // not update with or take new readings (such as temperature)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_SHUTDOWN);
}

void max30105Utils::wakeUp(void)
{
  // Pull IC out of low power mode (datasheet pg. 19)
  bitMask(MAX30105_MODECONFIG, MAX30105_SHUTDOWN_MASK, MAX30105_WAKEUP);
}

void max30105Utils::setLEDMode(uint8_t mode)
{
  // Set which LEDs are used for sampling -- Red only, RED+IR only, or custom.
  // See datasheet, page 19
  bitMask(MAX30105_MODECONFIG, MAX30105_MODE_MASK, mode);
}

void max30105Utils::setADCRange(uint8_t adcRange)
{
  // adcRange: one of MAX30105_ADCRANGE_2048, _4096, _8192, _16384
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_ADCRANGE_MASK, adcRange);
}

void max30105Utils::setSampleRate(uint8_t sampleRate)
{
  // sampleRate: one of MAX30105_SAMPLERATE_50, _100, _200, _400, _800, _1000, _1600, _3200
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_SAMPLERATE_MASK, sampleRate);
}

void max30105Utils::setPulseWidth(uint8_t pulseWidth)
{
  // pulseWidth: one of MAX30105_PULSEWIDTH_69, _188, _215, _411
  bitMask(MAX30105_PARTICLECONFIG, MAX30105_PULSEWIDTH_MASK, pulseWidth);
}

// NOTE: Amplitude values: 0x00 = 0mA, 0x7F = 25.4mA, 0xFF = 50mA (typical)
// See datasheet, page 21
void max30105Utils::setPulseAmplitudeRed(uint8_t amplitude)
{
  writeRegister8(m_address, MAX30105_LED1_PULSEAMP, amplitude);
}

void max30105Utils::setPulseAmplitudeIR(uint8_t amplitude)
{
  writeRegister8(m_address, MAX30105_LED2_PULSEAMP, amplitude);
}

void max30105Utils::setPulseAmplitudeGreen(uint8_t amplitude)
{
  writeRegister8(m_address, MAX30105_LED3_PULSEAMP, amplitude);
}

void max30105Utils::setPulseAmplitudeProximity(uint8_t amplitude)
{
  writeRegister8(m_address, MAX30105_LED_PROX_AMP, amplitude);
}

void max30105Utils::setProximityThreshold(uint8_t threshMSB)
{
  // Set the IR ADC count that will trigger the beginning of particle-sensing mode.
  // The threshMSB signifies only the 8 most significant-bits of the ADC count.
  // See datasheet, page 24.
  writeRegister8(m_address, MAX30105_PROXINTTHRESH, threshMSB);
}

// Given a slot number assign a thing to it
// Devices are SLOT_RED_LED or SLOT_RED_PILOT (proximity)
// Assigning a SLOT_RED_LED will pulse LED
// Assigning a SLOT_RED_PILOT will ??
void max30105Utils::enableSlot(uint8_t slotNumber, uint8_t device)
{

  uint8_t originalContents;

  switch (slotNumber)
  {
  case (1):
    bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT1_MASK, device);
    break;
  case (2):
    bitMask(MAX30105_MULTILEDCONFIG1, MAX30105_SLOT2_MASK, device << 4);
    break;
  case (3):
    bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT3_MASK, device);
    break;
  case (4):
    bitMask(MAX30105_MULTILEDCONFIG2, MAX30105_SLOT4_MASK, device << 4);
    break;
  default:
    // Shouldn't be here!
    break;
  }
}

// Clears all slot assignments
void max30105Utils::disableSlots(void)
{
  writeRegister8(m_address, MAX30105_MULTILEDCONFIG1, 0);
  writeRegister8(m_address, MAX30105_MULTILEDCONFIG2, 0);
}

// Resets all points to start in a known state
// Page 15 recommends clearing FIFO before beginning a read
void max30105Utils::clearFIFO(void)
{
  writeRegister8(m_address, MAX30105_FIFOWRITEPTR, 0);
  writeRegister8(m_address, MAX30105_FIFOOVERFLOW, 0);
  writeRegister8(m_address, MAX30105_FIFOREADPTR, 0);
}

// Enable roll over if FIFO over flows
void max30105Utils::enableFIFORollover(void)
{
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_ENABLE);
}

// Disable roll over if FIFO over flows
void max30105Utils::disableFIFORollover(void)
{
  bitMask(MAX30105_FIFOCONFIG, MAX30105_ROLLOVER_MASK, MAX30105_ROLLOVER_DISABLE);
}

// Set number of samples to trigger the almost full interrupt (Page 18)
// Power on default is 32 samples
// Note it is reverse: 0x00 is 32 samples, 0x0F is 17 samples
void max30105Utils::setFIFOAlmostFull(uint8_t numberOfSamples)
{
  bitMask(MAX30105_FIFOCONFIG, MAX30105_A_FULL_MASK, numberOfSamples);
}

// Read the FIFO Write Pointer
uint8_t max30105Utils::getWritePointer(void)
{
  return (readRegister8(m_address, MAX30105_FIFOWRITEPTR));
}

// Read the FIFO Read Pointer
uint8_t max30105Utils::getReadPointer(void)
{
  return (readRegister8(m_address, MAX30105_FIFOREADPTR));
}

// Report the most recent red value
uint32_t max30105Utils::getRed(void)
{
  // Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.red[sense.head]);
  else
    return (0); // Sensor failed to find new data
}

// Report the most recent IR value
uint32_t max30105Utils::getIR(void)
{
  // Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.IR[sense.head]);
  else
    return (0); // Sensor failed to find new data
}

// Report the most recent Green value
uint32_t max30105Utils::getGreen(void)
{
  // Check the sensor for new data for 250ms
  if (safeCheck(250))
    return (sense.green[sense.head]);
  else
    return (0); // Sensor failed to find new data
}

// Report the next Red value in the FIFO
uint32_t max30105Utils::getFIFORed(void)
{
  return (sense.red[sense.tail]);
}

// Report the next IR value in the FIFO
uint32_t max30105Utils::getFIFOIR(void)
{
  return (sense.IR[sense.tail]);
}

// Report the next Green value in the FIFO
uint32_t max30105Utils::getFIFOGreen(void)
{
  return (sense.green[sense.tail]);
}

uint8_t max30105Utils::available(void)
{
  int8_t numberOfSamples = sense.head - sense.tail;
  if (numberOfSamples < 0)
    numberOfSamples += STORAGE_SIZE;

  return (numberOfSamples);
}
// Advance the tail
void max30105Utils::nextSample(void)
{
  if (available()) // Only advance the tail if new data is available
  {
    sense.tail++;
    sense.tail %= STORAGE_SIZE; // Wrap condition
  }
}

bool max30105Utils::safeCheck(uint8_t maxTimeToCheck)
{
  uint32_t msSinceBoot = to_ms_since_boot(get_absolute_time());
  while (to_ms_since_boot(get_absolute_time()) - msSinceBoot < maxTimeToCheck)
  {
    if (check() > 0) // We found new data!
      return (true);

    sleep_ms(1); // Let's not over burden the I2C bus
  }
  return false;
}

float max30105Utils::readTemperature()
{

  // DIE_TEMP_RDY interrupt must be enabled
  // See issue 19: https://github.com/sparkfun/SparkFun_MAX3010x_Sensor_Library/issues/19

  // Step 1: Config die temperature register to take 1 temperature sample
  writeRegister8(m_address, MAX30105_DIETEMPCONFIG, 0x01);

  // Poll for bit to clear, reading is then complete
  // Timeout after 100ms
  uint32_t msSinceBoot = to_ms_since_boot(get_absolute_time());
  while (to_ms_since_boot(get_absolute_time()) - msSinceBoot < 100)
  {
    // uint8_t response = readRegister8(_i2caddr, MAX30105_DIETEMPCONFIG); //Original way
    // if ((response & 0x01) == 0) break; //We're done!

    // Check to see if DIE_TEMP_RDY interrupt is set
    uint8_t response = readRegister8(m_address, MAX30105_INTSTAT2);
    if ((response & MAX30105_INT_DIE_TEMP_RDY_ENABLE) > 0)
      break;     // We're done!
    sleep_ms(1); // Let's not over burden the I2C bus
  }
  // TODO How do we want to fail? With what type of error?
  //? if(millis() - startTime >= 100) return(-999.0);

  // Step 2: Read die temperature register (integer)
  int8_t tempInt = readRegister8(m_address, MAX30105_DIETEMPINT);
  uint8_t tempFrac = readRegister8(m_address, MAX30105_DIETEMPFRAC); // Causes the clearing of the DIE_TEMP_RDY interrupt

  // Step 3: Calculate temperature (datasheet pg. 23)
  return (float)tempInt + ((float)tempFrac * 0.0625);
}
