#include <FDC2214.h>

FDC2214::FDC2214() { _device_id = 0x3055; }

void FDC2214::init(TwoWire *theWire, uint8_t i2c_addr) {
    _wire = theWire;
    _dev_addr = i2c_addr;
    // Set sensor configuration registers

    writeRegister16(FDC2214_CONFIG, 0x1601);
    writeRegister16(FDC2214_MUX_CONFIG, 0xC20D);
    writeRegister16(FDC2214_SETTLECOUNT_CH0, 0x0010);
    writeRegister16(FDC2214_SETTLECOUNT_CH1, 0x0010);
    writeRegister16(FDC2214_RCOUNT_CH0, 0xA800);
    writeRegister16(FDC2214_RCOUNT_CH1, 0xA800);
    writeRegister16(FDC2214_CLOCK_DIVIDERS_CH0, 0x2001);
    writeRegister16(FDC2214_CLOCK_DIVIDERS_CH1, 0x2001);
    writeRegister16(FDC2214_DRIVE_CH0, 0xF800);
    writeRegister16(FDC2214_DRIVE_CH1, 0xF800);
    writeRegister16(FDC2214_OFFSET_CH0, 0x0000);
    writeRegister16(FDC2214_OFFSET_CH1, 0x0000);

    low_pass0 = 0;
    low_pass1 = 0;
}

unsigned long FDC2214::readSensor(int channel){
    uint32_t msb_reg;
    uint32_t lsb_reg;
    
    switch(channel){
        case 0:
            msb_reg = readRegister16(FDC2214_DATA_CH0_MSB);
            lsb_reg = readRegister16(FDC2214_DATA_CH0_LSB);
            break;
        case 1:
            msb_reg = readRegister16(FDC2214_DATA_CH1_MSB);
            lsb_reg = readRegister16(FDC2214_DATA_CH1_LSB);
            break;
    }
    unsigned long reading = (uint32_t)(msb_reg & FDC2214_DATA_CHx_MASK_DATA) << 16;
    reading |= lsb_reg;
    return reading;
}

float FDC2214::correctedCapacitance(float avgRef, float baseline){
    return (readCapacitance(00) - (avgRef - baseline));
}

// float FDC2214::readDiffCapacitance(){
//     const double fixedL = 0.000010; // 10 μH
//     const double fRef = 40000000; //40 MHz

//     Adafruit_I2CRegister clockdiv_regch0 = Adafruit_I2CRegister(i2c_dev, FDC2214_CLOCK_DIVIDERS_CH0, 2, MSBFIRST);
//     Adafruit_I2CRegister drive_regch0 = Adafruit_I2CRegister(i2c_dev, FDC2214_DRIVE_CH0, 2, MSBFIRST);
//     // Set 1x clock divider for differential measurement
//     float fSens = readSensor(0) * fRef / pow(2, 28);
//     float capVal = (1.0 / (fixedL * pow(2.0 * PI * fSens, 2.0)));

//     return capVal * pow(10, 12);
// }

float FDC2214::readCapacitance(int channel){
    const double fixedL = 0.000010; // 10 μH
    // const double paraC0 = .00000000001359; // 13.59 pF
    // const double paraC1 = .00000000001469; // 14.69 pF
    const double diffC = .000000000038; 
    // const double fRef = 43355000; //43.355 MHz
    const double fRef = 40000000; //40 MHz

    float fSens = readSensor(channel) * fRef / pow(2, 28);
    // double capVal0 = (1.0 / (fixedL * pow(2.0* PI * fSens0, 2.0)));
    float capVal = (pow(1/((fSens * 2) * PI * sqrt(fixedL * diffC)) - 1, 2) - 1) * diffC;

    return capVal * pow(10, 12);
}



uint16_t FDC2214::readRegister16(uint8_t reg) {
    _wire->beginTransmission(_dev_addr);
    _wire->write(reg);
    _wire->endTransmission();

    _wire->requestFrom(_dev_addr, 2);
    return (_wire->read() << 8 | _wire->read());
}

void FDC2214::writeRegister16(uint8_t reg, uint16_t val)
{
    uint8_t vla;
    vla = (uint8_t)val;
    val >>= 8;

    _wire->beginTransmission(_dev_addr);
    _wire->write(reg);
    _wire->write((uint8_t)val);
    _wire->write(vla);
    _wire->endTransmission();
}