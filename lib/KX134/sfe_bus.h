// The following classes specify the behavior for communicating
// over the respective data buses: Inter-Integrated Circuit (I2C)
// and Serial Peripheral Interface (SPI). For ease of implementation
// an abstract interface (QwIDeviceBus) is used.

#pragma once

#include <Wire.h>
#include <SPI.h>

namespace sfe_KX13X
{

  // The following abstract class is used an interface for upstream implementation.
  class QwIDeviceBus
  {
  public:
    virtual bool ping(uint8_t address) = 0;

    virtual bool writeRegisterByte(uint8_t address, uint8_t offset, uint8_t data) = 0;

    virtual int writeRegisterRegion(uint8_t address, uint8_t offset, const uint8_t *data, uint16_t length) = 0;

    virtual int readRegisterRegion(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t numBytes) = 0;
  };

  // The QwI2C device defines behavior for I2C implementation based around the TwoWire class (Wire).
  // This is Arduino specific.
  class QwI2C : public QwIDeviceBus
  {
  public:
    QwI2C(void);

    bool init();

    bool init(TwoWire &wirePort, bool bInit = false);

    bool ping(uint8_t address);

    bool writeRegisterByte(uint8_t address, uint8_t offset, uint8_t data);

    int writeRegisterRegion(uint8_t address, uint8_t offset, const uint8_t *data, uint16_t length);

    int readRegisterRegion(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t numBytes);

  private:
    TwoWire *_i2cPort;
  };

  // The SfeSPI class defines behavior for SPI implementation based around the SPIClass class (SPI).
  // This is Arduino specific.
  // Paramaters like "address" are kept although irrelevant to SPI due to the use of the abstract class
  // as interface, QwIDeviceBus.
  class SfeSPI : public QwIDeviceBus
  {
  public:
    SfeSPI(void);

    bool init(uint8_t cs, bool bInit = false);

    bool init(SPIClass &spiPort, SPISettings &kxSettings, uint8_t cs, bool bInit = false);

    bool ping(uint8_t address);

    bool writeRegisterByte(uint8_t address, uint8_t offset, uint8_t data);

    int writeRegisterRegion(uint8_t address, uint8_t offset, const uint8_t *data, uint16_t length);

    int readRegisterRegion(uint8_t addr, uint8_t reg, uint8_t *data, uint16_t numBytes);

  private:
    SPIClass *_spiPort;
    // Settings are used for every transaction.
    SPISettings _sfeSPISettings;
    uint8_t _cs;
  };

};