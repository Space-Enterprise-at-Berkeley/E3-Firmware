/*
 * Library for the TI ADS1231 24-Bit Analog-to-Digital Converter
 */

#ifndef ADS1231_H
#define ADS1231_H

#define ADS1231_ERROR_HIGH_TIMEOUT -1
#define ADS1231_ERROR_LOW_TIMEOUT -2

class ADS1231 {
    public:
        ADS1231(uint8_t clkPin,uint8_t dataPin);
        ADS1231();
        void init(uint8_t clkPin,uint8_t dataPin);
        int getValue(long &val);
        bool begin();
    private:
        uint8_t pData;
        uint8_t pClk;
};


#endif
