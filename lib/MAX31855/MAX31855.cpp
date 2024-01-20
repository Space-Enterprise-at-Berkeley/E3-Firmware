#include <MAX31855.h>

int MAX31855::init(SPIClass *spi, uint8_t chipSelect)
{ // assume that numSensors is < max Size of packet. Add some error checking here
    _chipSelect = chipSelect;
    _spi = spi;
    //Serial.print("MAX init with chipSelect ");
    //Serial.println(_chipSelect);

    _spi->begin();
    //Serial.println("mid spi done");
    //_spi->beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));

    //Serial.println("MAX spi done");

    pinMode(_chipSelect, OUTPUT);
    digitalWrite(_chipSelect, HIGH);

    //Serial.println("MAX done done");

    return 0;
}

void MAX31855::readCelsius(float *temperature, float *cjt, uint8_t *fault)
{
    int32_t v;
    int32_t c; //cold junction temp

    digitalWrite(_chipSelect, LOW);
    v = spiread32();
    digitalWrite(_chipSelect, HIGH);

    uint8_t f = v & 0x7;

    c = (v & 0xFFF0) >> 4;

    if (c & 0x800) {
      c = 0xFFF000 | (c & 0x000FFF);
    }

    if (v & 0x80000000)
    {
      // Negative value, drop the lower 18 bits and explicitly extend sign bits.
      v = 0xFFFFC000 | ((v >> 18) & 0x00003FFF);
    }
    else
    {
      // Positive value, just drop the lower 18 bits.
      v >>= 18;
    }

    double centigrade = v;
    double coldjunct = c;

    // LSB = 0.25 degrees C
    centigrade *= 0.25;
    coldjunct *= 0.0625; //LSB = 0.0625 for cjt

    *temperature = centigrade;
    *cjt = coldjunct;
    *fault = f;
}

uint32_t MAX31855::spiread32(void)
{
    uint32_t d = 0;
    uint8_t buf[4];

    _spi->transfer(buf, 4);

    d = buf[0];
    d <<= 8;
    d |= buf[1];
    d <<= 8;
    d |= buf[2];
    d <<= 8;
    d |= buf[3];

    return d;
}
