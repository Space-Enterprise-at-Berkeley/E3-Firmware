// ADS8688 library for Arduino
// initial release by Sylvain GARNAVAULT - 2016/03/13
 
#include "ADS8688.h"
 
////////////////////////
//  INSTANCIATION     //
////////////////////////

ADS8688::ADS8688() {}

bool ADS8688::init(SPIClass *theSPI, uint8_t cs) {
_cs = cs;
_theSPI = theSPI;
_mode = MODE_IDLE;            // start in Idle mode
_vref = 4.096;                // vref at 4.096
_feature = 0;   

_theSPI->begin();

pinMode(_cs, OUTPUT);
digitalWrite(_cs, HIGH);

return true;
}

ADS8688::ADS8688(SPIClass *SPI) {
_cs = 10;                     // default chip select pin
_mode = MODE_IDLE;            // start in Idle mode
_vref = 4.096;                // vref at 4.096
_feature = 0;                 // start with no feature
pinMode(_cs,OUTPUT);          // set the pin as output
digitalWrite(_cs,HIGH);       // set the pin to default HIGH state
SPI->begin();                  // initiate SPI
}

/*
ADS8688::ADS8688(SPIClass *SPI, uint8_t cs) {
_cs = cs;                     // choose the chip select pin
_mode = MODE_IDLE;            // start in Idle mode
_vref = 4.096;                // vref at 4.096
_feature = 0;                 // start with no feature
pinMode(_cs,OUTPUT);          // set the pin as output
digitalWrite(_cs,HIGH);       // set the pin to default HIGH state
SPI->begin();                  // initiate SPI
}
*/

/////////////////////////
//  PUBLIC METHODS    //
/////////////////////////

uint16_t ADS8688::noOp() {
return cmdRegister(NO_OP);
}

uint16_t ADS8688::standBy() {
return cmdRegister(STDBY);
}

uint16_t ADS8688::powerDown() {
return cmdRegister(PWR_DN);
}

uint16_t ADS8688::reset() {
return cmdRegister(RST);
}

uint16_t ADS8688::autoRst() {
return cmdRegister(AUTO_RST);
}

uint16_t ADS8688::manualChannel(uint8_t ch) {
uint8_t reg;
switch (ch) {
    case 0:  reg = MAN_Ch_0;break;
    case 1:  reg = MAN_Ch_1;break;
    case 2:  reg = MAN_Ch_2;break;
    case 3:  reg = MAN_Ch_3;break;
    case 4:  reg = MAN_Ch_4;break;
    case 5:  reg = MAN_Ch_5;break;
    case 6:  reg = MAN_Ch_6;break;
    case 7:  reg = MAN_Ch_7;break;
    case 8:  reg = MAN_AUX; break;
    default: reg = MAN_Ch_0;break;
}
return cmdRegister(reg);
}

//-----------------------------------------------------------------------------

void ADS8688::setChannelSPD(uint8_t flag) {
setChannelSequence(flag);
setChannelPowerDown((uint8_t)~flag);
}

void ADS8688::setChannelSequence(uint8_t flag) {
writeRegister(AUTO_SEQ_EN,flag);
}

void ADS8688::setChannelPowerDown(uint8_t flag) {
writeRegister(CH_PWR_DN,flag);
}

uint8_t ADS8688::getChannelSequence() {
return readRegister(AUTO_SEQ_EN);
}

uint8_t ADS8688::getChannelPowerDown() {
return readRegister(CH_PWR_DN);
}

//-----------------------------------------------------------------------------

uint8_t ADS8688::getChannelRange(uint8_t ch) {
uint8_t reg;
switch (ch) {
    case 0:  reg = RG_Ch_0;break;
    case 1:  reg = RG_Ch_1;break;
    case 2:  reg = RG_Ch_2;break;
    case 3:  reg = RG_Ch_3;break;
    case 4:  reg = RG_Ch_4;break;
    case 5:  reg = RG_Ch_5;break;
    case 6:  reg = RG_Ch_6;break;
    case 7:  reg = RG_Ch_7;break;
    default: reg = RG_Ch_0;break;
    }
return readRegister(reg);
}

void ADS8688::setChannelRange(uint8_t ch, uint8_t range) {
uint8_t reg;
switch (ch) {
    case 0:  reg = RG_Ch_0;break;
    case 1:  reg = RG_Ch_1;break;
    case 2:  reg = RG_Ch_2;break;
    case 3:  reg = RG_Ch_3;break;
    case 4:  reg = RG_Ch_4;break;
    case 5:  reg = RG_Ch_5;break;
    case 6:  reg = RG_Ch_6;break;
    case 7:  reg = RG_Ch_7;break;
    default: reg = RG_Ch_0;break;
    }
writeRegister(reg,range);
}

void ADS8688::setGlobalRange(uint8_t range) {
for (uint8_t i=0;i<8;i++) setChannelRange(i,range);
}

//-----------------------------------------------------------------------------

uint8_t ADS8688::getId() {
return (getFeatureSelect() >> 6);
}

void ADS8688::setId(uint8_t id) {
_feature = (_feature & 0b00010111) | ((id & 0b11)<<6);
writeRegister(FT_SEL,_feature);
}

bool ADS8688::getAlarm() {
return (getFeatureSelect() >> 4) & 1;
}

void ADS8688::setAlarm(bool alarm) {
_feature = (_feature & 0b11000111) | ((alarm == true)<<4);
writeRegister(FT_SEL,_feature);
}

uint8_t ADS8688::getSdo() {
return (getFeatureSelect() & 0b111);
}

void ADS8688::setSdo(uint8_t sdo) {
_feature = (_feature & 0b11010000) | (sdo & 0b111);
writeRegister(FT_SEL,_feature);
}

uint8_t ADS8688::getFeatureSelect() {
return readRegister(FT_SEL);
}

void ADS8688::setFeatureSelect(uint8_t id, bool alarm, uint8_t sdo) {
_feature = ((id & 0b11)<<6) | ((alarm == true)<<4) | (sdo & 0b111);
writeRegister(FT_SEL,_feature);
}

//-----------------------------------------------------------------------------

uint8_t ADS8688::getAlarmOverview() {
return readRegister(ALARM_OVERVIEW);
}

uint8_t ADS8688::getFirstTrippedFlag() {
return readRegister(ALARM_CH0_TRIPPED_FLAG);
}

uint8_t ADS8688::getSecondTrippedFlag() {
return readRegister(ALARM_CH4_TRIPPED_FLAG);
}

uint16_t ADS8688::getTrippedFlags() {
uint8_t MSB = readRegister(ALARM_CH0_TRIPPED_FLAG);
uint8_t LSB = readRegister(ALARM_CH4_TRIPPED_FLAG);
return (MSB << 8) | LSB;
}

uint8_t ADS8688::getFirstActiveFlag() {
return readRegister(ALARM_CH0_ACTIVE_FLAG);
}

uint8_t ADS8688::getSecondActiveFlag() {
return readRegister(ALARM_CH4_ACTIVE_FLAG);
}

uint16_t ADS8688::getActiveFlags() {
uint8_t MSB = readRegister(ALARM_CH0_ACTIVE_FLAG);
uint8_t LSB = readRegister(ALARM_CH4_ACTIVE_FLAG);
return (MSB << 8) | LSB;
}

//-----------------------------------------------------------------------------

uint8_t ADS8688::getChannelHysteresis(uint8_t ch) {
uint8_t reg = 5*(ch>7?7:ch) + CH0_HYST;
return readRegister(reg);
}

uint16_t ADS8688::getChannelLowThreshold(uint8_t ch) {
uint8_t reg = 5*(ch>7?7:ch) + CH0_LT_MSB;
uint8_t MSB = readRegister(reg);
uint8_t LSB = readRegister(reg+1);
return (MSB << 8) | LSB;
}

uint16_t ADS8688::getChannelHighThreshold(uint8_t ch) {
uint8_t reg = 5*(ch>7?7:ch) + CH0_HT_MSB;
uint8_t MSB = readRegister(reg);
uint8_t LSB = readRegister(reg+1);
return (MSB << 8) | LSB;
}

void ADS8688::setChannelHysteresis(uint8_t ch, uint8_t val) {
uint8_t reg = 5*(ch>7?7:ch) + CH0_HYST;
writeRegister(reg,val);
}

void ADS8688::setChannelLowThreshold(uint8_t ch, uint16_t val) {
uint8_t reg = 5*(ch>7?7:ch) + CH0_LT_MSB;
writeRegister(reg,val>>8);
writeRegister(reg+1,val&255);
}

void ADS8688::setChannelHighThreshold(uint8_t ch, uint16_t val)  {
uint8_t reg = 5*(ch>7?7:ch) + CH0_HT_MSB;
writeRegister(reg,val>>8);
writeRegister(reg+1,val&255);
}

//-----------------------------------------------------------------------------

uint8_t ADS8688::getCommandReadBack() {
return readRegister(CMD_READBACK);
}

//-----------------------------------------------------------------------------

void ADS8688::setVREF(float vref) {
_vref = vref;
}

float ADS8688::I2V(uint16_t x, uint8_t range) {
float out_min, out_max;
switch (range) {
    case R1:
        out_min = -1.25 * _vref;
        out_max = 1.25 * _vref;
        break;
    case R2:
        out_min = -0.625 * _vref;
        out_max = 0.625 * _vref;
        break;
    case R3:
        out_min = -0.3125 * _vref;
        out_max = 0.3125 * _vref;
        break;
    case R4:
        out_min = -0.15625 * _vref;
        out_max = 0.15625 * _vref;
        break;
    case R5:
        out_min = 0 * _vref;
        out_max = 2.5 * _vref;
        break;
    case R6:
        out_min = 0 * _vref;
        out_max = 1.25 * _vref;
        break;
    case R7:
        out_min = 0 * _vref;
        out_max = 0.625 * _vref;
        break;
    case R8:
        out_min = 0 * _vref;
        out_max = 0.3125 * _vref;
        break;
    default:
        out_min = -2.5 * _vref;
        out_max = 2.5 * _vref;
        break;
        
}
return (float)x * (out_max - out_min) / 65535. + out_min; //used to be 65535
}

uint16_t ADS8688::V2I(float x, uint8_t range) {
float in_min, in_max;
switch (range) {
case R1:
        in_min = -1.25 * _vref;
        in_max = 1.25 * _vref;
        break;
    case R2:
        in_min = -0.625 * _vref;
        in_max = 0.625 * _vref;
        break;
    case R3:
        in_min = -0.3125 * _vref;
        in_max = 0.3125 * _vref;
        break;
    case R4:
        in_min = -0.15625 * _vref;
        in_max = 0.15625 * _vref;
        break;
    case R5:
        in_min = 0 * _vref;
        in_max = 2.5 * _vref;
        break;
    case R6:
        in_min = 0 * _vref;
        in_max = 1.25 * _vref;
        break;
    case R7:
        in_min = 0 * _vref;
        in_max = 0.625 * _vref;
        break;
    case R8:
        in_min = 0 * _vref;
        in_max = 0.3125 * _vref;
        break;
    default:
        in_min = -2.5 * _vref;
        in_max = 2.5 * _vref;
        break;
        
}
return (x-in_min) * 65535. / (in_max-in_min); //used to be 65535
}

/////////////////////////
//  PRIVATE METHODS    //
/////////////////////////

void ADS8688::writeRegister(uint8_t reg, uint8_t val) {
_theSPI->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
digitalWrite(_cs, LOW);
_theSPI->transfer((reg << 1) | 0x01);
_theSPI->transfer(val);;
_theSPI->transfer(0x00);
digitalWrite(_cs, HIGH);
_theSPI->endTransaction();
_mode = MODE_PROG;
}

uint8_t ADS8688::readRegister(uint8_t reg) {
_theSPI->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
digitalWrite(_cs, LOW);
_theSPI->transfer((reg << 1) | 0x00);
_theSPI->transfer(0x00);
byte result = _theSPI->transfer(0x00);
digitalWrite(_cs, HIGH);
_theSPI->endTransaction();
_mode = MODE_PROG;
return result;
}

uint16_t ADS8688::cmdRegister(uint8_t reg) {
_theSPI->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
digitalWrite(_cs, LOW);
_theSPI->transfer(reg);
_theSPI->transfer(0x00);
int16_t result = 0;
if (_mode > 4) {
    // only 16 bit if POWERDOWN or STDBY or RST or IDLE
    byte MSB = _theSPI->transfer(0x00);
    byte LSB = _theSPI->transfer(0x00);
    result = ( MSB << 8) | LSB; //used to be ( MSB << 8) | LSB [changed to try and adapt to 12 bit]
    }
digitalWrite(_cs, HIGH);
_theSPI->endTransaction();

// when exit power down it takes 15 ms to be operationnal
if (_mode == MODE_POWER_DN) delay(15);

switch (reg) {
    case NO_OP:
        switch (_mode) {
            case MODE_RESET:    _mode = MODE_IDLE;
                break;
            case MODE_PROG :    _mode = MODE_IDLE;
                break;
            case MODE_AUTO_RST: _mode = MODE_AUTO;
                break;
        }
        break;
    case STDBY:
        _mode = MODE_STANDBY;
        break;
    case PWR_DN:
        _mode = MODE_POWER_DN;
        break;
    case RST:
        _mode = MODE_RESET;
        break;
    case AUTO_RST:
        _mode = MODE_AUTO_RST;
        break;
    default:
        _mode = MODE_MANUAL;
        break;
}
return result;
}

void ADS8688::readDaisyChain(uint16_t *rawValues, int numADCs) {
    _theSPI->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_cs, LOW);

    _theSPI->transfer(0x00); //equivalent to sending 0x00 (NO_OP)
    _theSPI->transfer(0x00);

    // Read N 16-bit results (MSB first from ADC_N to ADC_1)
    for (int i = 0; i < numADCs; i++) {
        byte MSB = _theSPI->transfer(0x00);
        byte LSB = _theSPI->transfer(0x00);
        rawValues[i] = (MSB << 8) | LSB;
    }

    digitalWrite(_cs, HIGH);
    _theSPI->endTransaction();
    //return;
}