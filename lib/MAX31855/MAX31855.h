#pragma once

#include <Arduino.h>
#include <SPI.h>

#define MAX31855_FAULT_OPEN (0x01)      ///< Enable open circuit fault check
#define MAX31855_FAULT_SHORT_GND (0x02) ///< Enable short to GND fault check
#define MAX31855_FAULT_SHORT_VCC (0x04) ///< Enable short to VCC fault check
#define MAX31855_FAULT_ALL (0x07)       ///< Enable all fault checks

class MAX31855 {
    public:
        int init(SPIClass *spi, uint8_t chipSelect); // assume that numSensors is < max Size of packet. Add some error checking here
        int readCelsius(float *temperature);
    private:
        uint32_t spiread32(void);
        uint8_t _chipSelect;
        SPIClass *_spi;
};