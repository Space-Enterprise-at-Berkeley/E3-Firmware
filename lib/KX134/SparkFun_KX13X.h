// The following class implements the	Arduino Interface for the SparkFun Triple
// Axis Acceleromters - KX132/KX134.

#pragma once
#include "SparkFun_Qwiic_KX13X.h"
#include <Wire.h>
#include <SPI.h>

class SparkFun_KX132 : public QwDevKX132
{

public:
  SparkFun_KX132(){};

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the ISM330DHCX library and connect to
  // the ISM330DHCX device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  wirePort    optional. The Wire port. If not provided, the default port is used
  //  address     optional. I2C Address. If not provided, the default address is used.
  //  retval      true on success, false on startup failure
  //
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an I2C object which then defaults to Wire.
  bool begin(uint8_t deviceAddress = KX13X_ADDRESS_HIGH)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus, deviceAddress);

    // Initialize the I2C buss class i.e. setup default Wire port
    _i2cBus.init();

    // Initialize the system - return results
    return this->QwDevKX132::init();
  }

  // Version 2:
  //  User passes in an I2C object and an address (optional).
  bool begin(TwoWire &wirePort, uint8_t deviceAddress = KX13X_ADDRESS_HIGH)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus, deviceAddress);

    // Give the I2C port provided by the user to the I2C bus class.
    _i2cBus.init(wirePort, true);

    // Initialize the system - return results
    return this->QwDevKX132::init();
  }

private:
  // I2C bus class
  sfe_KX13X::QwI2C _i2cBus;
};

class SparkFun_KX132_SPI : public QwDevKX132
{
public:
  SparkFun_KX132_SPI(){};

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the ISM330DHCX library and connect to
  // the ISM330DHCX device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  spiPort     optional. The SPI port. If not provided, the default port is used
  //  SPISettings optional. SPI "transaction" settings are need for every data transfer.
  //												Default used if not provided.
  //  Chip Select mandatory. The chip select pin ("CS") can't be guessed, so must be provided.
  //  retval      true on success, false on startup failure
  //
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an SPI object which then defaults to SPI.

  bool begin(uint8_t cs)
  {
    // Setup a SPI object and pass into the superclass
    setCommunicationBus(_spiBus);

    // Initialize the SPI bus class with the chip select pin, SPI port defaults to SPI,
    // and SPI settings are set to class defaults.
    _spiBus.init(cs, true);

    // Initialize the system - return results
    return this->QwDevKX132::init();
  }

  bool begin(SPIClass &spiPort, SPISettings kxSettings, uint8_t cs)
  {
    // Setup a SPI object and pass into the superclass
    setCommunicationBus(_spiBus);

    // Initialize the SPI bus class with provided SPI port, SPI setttings, and chip select pin.
    _spiBus.init(spiPort, kxSettings, cs, true);

    // Initialize the system - return results
    return this->QwDevKX132::init();
  }

private:
  // SPI bus class
  sfe_KX13X::SfeSPI _spiBus;
};

class SparkFun_KX134 : public QwDevKX134
{

public:
  SparkFun_KX134(){};

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the ISM330DHCX library and connect to
  // the ISM330DHCX device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  wirePort    optional. The Wire port. If not provided, the default port is used
  //  address     optional. I2C Address. If not provided, the default address is used.
  //  retval      true on success, false on startup failure
  //
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an I2C object which then defaults to Wire.
  bool begin(uint8_t deviceAddress = KX13X_ADDRESS_HIGH)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus, deviceAddress);

    // Initialize the I2C buss class i.e. setup default Wire port
    _i2cBus.init();

    // Initialize the system - return results
    return this->QwDevKX134::init();
  }

  // Version 2:
  //  User passes in an I2C object and an address (optional).
  bool begin(TwoWire &wirePort, uint8_t deviceAddress = KX13X_ADDRESS_HIGH)
  {
    // Setup  I2C object and pass into the superclass
    setCommunicationBus(_i2cBus, deviceAddress);

    // Give the I2C port provided by the user to the I2C bus class.
    _i2cBus.init(wirePort, true);

    // Initialize the system - return results
    return this->QwDevKX134::init();
  }

private:
  // I2C bus class
  sfe_KX13X::QwI2C _i2cBus;
};

class SparkFun_KX134_SPI : public QwDevKX134
{
public:
  SparkFun_KX134_SPI(){};

  ///////////////////////////////////////////////////////////////////////
  // begin()
  //
  // This method is called to initialize the ISM330DHCX library and connect to
  // the ISM330DHCX device. This method must be called before calling any other method
  // that interacts with the device.
  //
  // This method follows the standard startup pattern in SparkFun Arduino
  // libraries.
  //
  //  Parameter   Description
  //  ---------   ----------------------------
  //  spiPort     optional. The SPI port. If not provided, the default port is used
  //  SPISettings optional. SPI "transaction" settings are need for every data transfer.
  //												Default used if not provided.
  //  Chip Select mandatory. The chip select pin ("CS") can't be guessed, so must be provided.
  //  retval      true on success, false on startup failure
  //
  // This methond is overridden, implementing two versions.
  //
  // Version 1:
  // User skips passing in an SPI object which then defaults to SPI.

  bool begin(uint8_t cs)
  {
    // Setup a SPI object and pass into the superclass
    setCommunicationBus(_spiBus);

    // Initialize the SPI bus class with the chip select pin, SPI port defaults to SPI,
    // and SPI settings are set to class defaults.
    _spiBus.init(cs, true);

    // Initialize the system - return results
    return this->QwDevKX134::init();
  }

  bool begin(SPIClass &spiPort, SPISettings kxSettings, uint8_t cs)
  {
    // Setup a SPI object and pass into the superclass
    setCommunicationBus(_spiBus);

    // Initialize the SPI bus class with provided SPI port, SPI setttings, and chip select pin.
    _spiBus.init(spiPort, kxSettings, cs, true);

    // Initialize the system - return results
    return this->QwDevKX134::init();
  }

private:
  // SPI bus class
  sfe_KX13X::SfeSPI _spiBus;
};