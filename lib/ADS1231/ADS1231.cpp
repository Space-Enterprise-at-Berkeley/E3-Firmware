#include <Arduino.h>
#include <limits.h>
#include <ADS1231.h>

unsigned long ads1231_last_millis = 0;
int ads1231_offset = 0;

/*
 * Initialize the interface pins
 */
ADS1231::ADS1231(uint8_t clkPin, uint8_t dataPin)
{
    pData = dataPin;
    pClk = clkPin;

}

ADS1231::ADS1231(){
}

void ADS1231::init(uint8_t clkPin,uint8_t dataPin){
    pData = dataPin;
    pClk = clkPin;
    begin();
}

bool ADS1231::begin() {
    // Init clock pin
    pinMode(pClk, OUTPUT);

    // Init data pin
    pinMode(pData, INPUT);

    // Enable pullup to get a consistent state in case of disconnect
    digitalWrite(pData, HIGH);

    // Set CLK low to get the ADS1231 out of suspend
    digitalWrite(pClk, 0);

    if ((pClk > 1) && (pClk < 14) &&
        (pData > 1) && (pData < 14) &&
        (pData != pClk)) {
            return 1;
    } else {
        return 0;
    }
    

}

/*
* GET raw ADC value. This call is blocking. up to 100ms in 10SPS, 12.5 ms in 80 SPS
 */
int ADS1231::getValue(long& val)
{
    int i=0;
    unsigned long start;

    /* A high to low transition on the data pin for data ready*/
    start=millis();
    while(digitalRead(pData) != HIGH)
    {
        if(millis() > start+150)
            return ADS1231_ERROR_HIGH_TIMEOUT; // Timeout waiting for HIGH
    }
    start=millis();
    while(digitalRead(pData) != LOW)
    {
        if(millis() > start+150)
            return ADS1231_ERROR_LOW_TIMEOUT; // Timeout waiting for LOW
    }
    ads1231_last_millis = millis();

    // Read 24 bits
    for(i=23 ; i >= 0; i--) {
        digitalWrite(pClk, HIGH);
        val = (val << 1) + digitalRead(pData);
        digitalWrite(pClk, LOW);
        
    }

    /* Accomodate for sign bit */
    val = (val << 8) / 256;

    /* Force data pin low to catch rising edge on next read*/
    digitalWrite(pClk, HIGH);
    digitalWrite(pClk, LOW);

    return 1; // Success
}


