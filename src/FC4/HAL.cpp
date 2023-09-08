#include "HAL.h"

namespace HAL {

    void initHAL() {

        pinMode(adcCS, OUTPUT);
        pinMode(radioCS, OUTPUT);
        pinMode(bbCS, OUTPUT);

        // initialize SPI
        spi0 = new SPIClass(HSPI);
        spi0->begin(sckPin, misoPin, mosiPin, adcCS);

        // initialize I2C
        Wire.begin(sdaPin, sclPin); 
        Wire.setClock(100000);

        // initialize IO expanders and channels
        MCP0.begin();
        MCP1.begin();
        MCP0.pinMode8(0x00);  // 0 = output , 1 = input
        MCP1.pinMode8(0x00);  // 0 = output , 1 = input

        // initialize ADC
        adc.init(spi0, adcCS, adcRDY);
        adc.setAllInputsSeparate();
        adc.enableOTFMode();

        // barometer
        if (!bmp.begin_I2C(0x76)) {
            Serial.println("Could not find a valid BMP3 sensor, check wiring!");
            while (1);
        }

        // imu
        if (!dso32.begin_I2C()) {
            while (1) {
            delay(10);
            }
        }

        dso32.setAccelRange(LSM6DSO32_ACCEL_RANGE_16_G);
        dso32.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS );
        dso32.setAccelDataRate(LSM6DS_RATE_3_33K_HZ);
        dso32.setGyroDataRate(LSM6DS_RATE_3_33K_HZ);


        bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
        bmp.setOutputDataRate(BMP3_ODR_200_HZ);
        bmp.performReading();
        set_barometer_ground_altitude();  

    }
};
